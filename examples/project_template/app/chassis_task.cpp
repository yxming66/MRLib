#include "app/chassis_task.hpp"
#include "app/cmd_center.hpp"
#if APP_EXAMPLE_ENABLE_DR16
#include "app/remote_control_task.hpp"
#endif
#include "app/app_init.h"
#include "app/chassis_assembly.hpp"
#include "app/robot_config.hpp"
#include "board/bindings.hpp"
#include "bsp/adapters/can_port.hpp"
#include "bsp/time.h"
#include "task/periodic_task.hpp"
#include "FreeRTOS.h"
#include "task.h"

namespace {
// ==================== 1. 对外状态快照 ====================
app::ChassisStatus latest_status{};
bool have_status = false;

void StoreStatus(const app::ChassisStatus& status) {
  taskENTER_CRITICAL();
  latest_status = status;
  have_status = true;
  taskEXIT_CRITICAL();
}

bool ReadRemote(void*, app::RemoteInput& input) {
#if APP_EXAMPLE_ENABLE_DR16
  return app::RemoteControlTask::Instance().ReadLatest(input);
#else
  input = {};
  return false;  // 默认未接收遥控器，命令中心保持安全输出。
#endif
}

// ==================== 2. 命令中心与底盘反馈的所有者任务 ====================
// 业务只负责初始化、单周期处理和故障处理；PeriodicTask 负责循环与等待。
class ChassisApplication final {
 public:
  bool Init() {
    const app::CommandCenterBindings bindings{
        nullptr, ReadRemote, nullptr, app::SelectChassisMode};
    // read_nuc 暂不绑定；接入 NUC 后改为其线程安全快照读取回调。
    if (!app::InitCommandCenter(bindings)) {
      g_app.state = APP_CMD_CENTER_INIT_FAILED;
      return false;
    }
    g_app.chassis_registration_result = drive.RegisterFeedback();
    if (g_app.chassis_registration_result != DEVICE_OK) {
      g_app.state = APP_CHASSIS_FEEDBACK_REGISTER_FAILED;
      return false;
    }
    status.feedback_registered = true;
    previous_us = BSP_TIME_Get_us();
    return true;
  }

  bool Step(float dt_s) {
    // 当前仅采集反馈；接入控制器后使用此固定配置周期（秒）。
    (void)dt_s;
    // 1. 读取输入快照、执行模式回调和仲裁，按值取得唯一最终命令。
    const uint32_t now_ms = BSP_TIME_Get_ms();
    const uint64_t now_us = BSP_TIME_Get_us();
    status.command = app::UpdateCommandCenter(now_ms);
    status.command_center = app::GetCommandCenterStatus();
    status.command_received = status.command_center.publish_ok;
    status.last_command_ms = status.command.received_ms;
    status.command_fresh = status.command.input_fresh;
    status.command_enabled = status.command.enabled;

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

    // 3. 当前只完成任务/命令映射/反馈接入。
    // 待方向、PID、速度/力矩限幅确认后，在这里接底盘控制周期。
    // 不调用 Enable/SetCommand/Step/CommitOutput/Flush；收到 enabled 也不会运动。
    status.control_enabled = false;

    // 4. 发布完整状态快照，其他任务通过 ReadChassisStatus 读取。
    status.updated_ms = now_ms;
    status.clock_advanced = now_us > previous_us;
    previous_us = now_us;
    ++status.heartbeat;
    StoreStatus(status);

    return true;
  }

  void Fault(mr::task::TaskFault why) {
    // PeriodicTask 在 Fault 返回后退出线程，不自动重启或补跑周期。
    // 当前没有电机输出；将来启用控制时，这里必须补充停机输出策略。
    status.control_enabled = false;
    if (why == mr::task::TaskFault::kDeadlineMissed) {
      ++status.deadline_misses;
      g_app.state = APP_CHASSIS_DEADLINE_MISSED;
    } else if (why == mr::task::TaskFault::kDelayFailed) {
      g_app.state = APP_CHASSIS_DELAY_FAILED;
    }
    // Init/Step 的具体错误状态已由业务填写。
    status.updated_ms = BSP_TIME_Get_ms();
    status.command = {};
    status.command_enabled = false;
    status.command_fresh = false;
    StoreStatus(status);
  }

 private:
  mr::board::BspCanPort port{
      BSP_CAN_FindEndpoint(board::bindings::kChassisCan), BSP_TIME_Get_ms};
  app::ChassisAssembly drive{port};
  app::ChassisStatus status{};
  uint64_t previous_us = 0;
};

}  // namespace

namespace app {
// ==================== 3. 状态读取与静态线程创建 ====================
bool ReadChassisStatus(ChassisStatus& status) {
  taskENTER_CRITICAL();
  const bool available = have_status;
  status = latest_status;
  taskEXIT_CRITICAL();
  return available;
}

bool CreateChassisTask() {
  // 由运行中的 Task_Init 调用，CAN 已准备好。
  if (osKernelGetState() != osKernelRunning) return false;
  static ChassisApplication application;
  static mr::task::PeriodicTask task;
  static StaticTask_t control;
  alignas(8) static StackType_t stack[params::kChassisStackBytes / sizeof(StackType_t)];
  static_assert(params::kChassisPeriodMs > 0 && params::kChassisPeriodMs <= UINT32_MAX / 1000u);
  static_assert(params::kChassisStackBytes % sizeof(StackType_t) == 0);
  const mr::task::PeriodicTaskConfig config{
      "Task_Chassis", params::kChassisPeriodMs * 1000u,
      params::kChassisStackBytes, osPriorityAboveNormal};
  const mr::task::TaskHooks hooks{
      &application,
      [](void* context) { return static_cast<ChassisApplication*>(context)->Init(); },
      [](void* context, float dt_s) {
        return static_cast<ChassisApplication*>(context)->Step(dt_s);
      },
      [](void* context, mr::task::TaskFault why) {
        static_cast<ChassisApplication*>(context)->Fault(why);
      }};
  osThreadAttr_t storage{};
  storage.cb_mem = &control;
  storage.cb_size = sizeof(control);
  storage.stack_mem = stack;
  storage.stack_size = sizeof(stack);
  return task.Create(config, hooks, storage);
}
}  // namespace app
