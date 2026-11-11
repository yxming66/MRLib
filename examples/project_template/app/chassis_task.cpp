#include "app/chassis_task.hpp"
#include "app/app_channels.hpp"
#include "app/app_init.h"
#include "app/chassis_assembly.hpp"
#include "app/robot_config.hpp"
#include "board/bindings.hpp"
#include "bsp/adapters/can_port.hpp"
#include "bsp/time.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"

namespace {
// 任务自己持有设备，不把电机/控制器引用暴露给其他任务。
void Task_Chassis(void* argument) {
  (void)argument;
  // 此时板级端点已登记，Task_Init 已完成 CAN 初始化。
  static mr::board::BspCanPort port{
      BSP_CAN_FindEndpoint(board::bindings::kChassisCan), BSP_TIME_Get_ms};
  static app::ChassisAssembly drive{port};
  static app::ChassisStatus status;

  // 初始化只执行一次，成功后才进入 while 循环。
  g_app.chassis_registration_result = drive.RegisterFeedback();
  if (g_app.chassis_registration_result != DEVICE_OK) {
    g_app.state = APP_CHASSIS_FEEDBACK_REGISTER_FAILED;
    osThreadExit();
    return;
  }
  status.feedback_registered = true;
  app::ChassisCommand latest_command{};
  bool have_command = false;
  uint64_t previous_us = BSP_TIME_Get_us();
  TickType_t wake_tick = xTaskGetTickCount();
  constexpr TickType_t period_ticks = pdMS_TO_TICKS(app::params::kChassisPeriodMs);
  static_assert(period_ticks > 0, "Chassis period must be at least one RTOS tick");
  static_assert(app::params::kChassisPeriodMs * configTICK_RATE_HZ % 1000 == 0,
                "Chassis period must be an exact number of RTOS ticks");

  while (1) {
    // 1. 收本板其他任务发布的最新指令，不阻塞。
    app::ChassisCommand incoming;
    if (app::TryReceiveChassisCommand(incoming)) {
      latest_command = incoming;
      have_command = true;
    }
    const uint32_t now_ms = BSP_TIME_Get_ms();
    const uint64_t now_us = BSP_TIME_Get_us();
    status.command_received = have_command;
    status.last_command_ms = latest_command.received_ms;
    status.command_fresh = have_command &&
        uint32_t(now_ms - latest_command.received_ms) <= app::params::kCommandTimeoutMs;
    status.command_enabled = status.command_fresh && latest_command.enabled;

    // 2. 读四轮反馈；无反馈/超时只标记离线，继续观察后续恢复。
    status.online_mask = drive.PollFeedback();
    const auto motors = drive.Motors();
    for (std::size_t i = 0; i < motors.size(); ++i) {
      const auto state = motors[i]->GetState();
      const auto& feedback = drive.feedback[i];
      auto& out = status.wheels[i];
      out.feedback_id = app::params::kWheelFeedbackIds[i];
      out.result = drive.feedback_results[i];
      out.online = feedback.online;
      out.timestamp_ms = feedback.timestamp_ms;
      out.sequence = feedback.sequence;
      out.position_rad = feedback.position_rad;
      out.velocity_rad_s = state.velocity_rad_s;
      out.linear_velocity_mps = feedback.linear_velocity_mps;
      out.torque_nm = feedback.output_torque_nm;
      out.temperature_c = state.temperature_c;
    }

    // 3. 当前只完成任务/通道/反馈接入。
    // 待方向、PID、速度/力矩限幅确认后，在这里接底盘控制周期。
    // 不调用 Enable/SetCommand/Step/CommitOutput/Flush；收到 enabled 也不会运动。
    status.control_enabled = false;

    // 4. 发布完整状态快照，其他任务通过 ReadChassisStatus 读取。
    status.updated_ms = now_ms;
    status.clock_advanced = now_us > previous_us;
    previous_us = now_us;
    ++status.heartbeat;
    if (!app::PublishChassisStatus(status)) {
      g_app.state = APP_CHASSIS_STATUS_PUBLISH_FAILED;
      osThreadExit();
      return;
    }

    // 5. 等下一个周期。超时记录并重新同步，不连续补跑旧周期。
    const TickType_t elapsed = xTaskGetTickCount() - wake_tick;
    if (elapsed >= period_ticks) {
      ++status.deadline_misses;
      wake_tick = xTaskGetTickCount();
    }
    vTaskDelayUntil(&wake_tick, period_ticks);
  }
}
}  // namespace

namespace app {
bool CreateChassisTask() {
  static StaticTask_t control;
  alignas(8) static StackType_t stack[params::kChassisStackBytes / sizeof(StackType_t)];
  static osThreadId_t thread = nullptr;
  if (thread || osKernelGetState() != osKernelRunning) return false;
  osThreadAttr_t attributes{};
  attributes.name = "Task_Chassis";
  attributes.priority = osPriorityAboveNormal;
  attributes.cb_mem = &control;
  attributes.cb_size = sizeof(control);
  attributes.stack_mem = stack;
  attributes.stack_size = sizeof(stack);
  thread = osThreadNew(Task_Chassis, nullptr, &attributes);
  return thread != nullptr;
}
}  // namespace app
