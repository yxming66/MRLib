#include "app/remote_control_task.hpp"

#include "bsp/time.h"
#include "device/device.h"
#include "task.h"

namespace app {

// ==================== 1. 生命周期与线程启动 ====================
RemoteControlTask& RemoteControlTask::Instance() {
  static RemoteControlTask task;
  return task;
}

bool RemoteControlTask::Start(BSP_UART_t uart) {
  if (started_ || uart == BSP_UART_ERR || !BSP_UART_GetHandle(uart) ||
      osKernelGetState() != osKernelRunning)
    return false;
  uart_ = uart;
  osThreadAttr_t attributes{};
  attributes.name = "Task_RemoteControl";
  attributes.priority = osPriorityAboveNormal;
  attributes.cb_mem = &control_;
  attributes.cb_size = sizeof(control_);
  attributes.stack_mem = stack_;
  attributes.stack_size = sizeof(stack_);
  started_ = osThreadNew(Entry, this, &attributes) != nullptr;
  return started_;
}

// ==================== 2. 最新快照：任务间短临界区复制 ====================
// 只能在任务上下文调用；接收任务独占写入，多个读者不会消费掉快照。
bool RemoteControlTask::ReadLatest(RemoteInput& input) const {
  taskENTER_CRITICAL();
  const bool available = have_latest_input_;
  input = latest_input_;
  taskEXIT_CRITICAL();
  return available;
}

void RemoteControlTask::StoreLatest(const RemoteInput& input) {
  taskENTER_CRITICAL();
  latest_input_ = input;
  have_latest_input_ = true;
  taskEXIT_CRITICAL();
}

// ==================== 3. 阻塞接收与解析 ====================
void RemoteControlTask::Entry(void* context) {
  static_cast<RemoteControlTask*>(context)->Run();
}

[[noreturn]] void RemoteControlTask::Run() {
  // DR16_Init stores this task ID for its UART ISR notification callback.
  if (DR16_Init(&receiver_, uart_) != DEVICE_OK) {
    PublishOffline();
    osThreadExit();
    for (;;) (void)osDelay(1u);
  }

  for (;;) {
    if (DR16_StartDmaRecv(&receiver_) != DEVICE_OK)
      DR16_Service(&receiver_);  // May rearm reception; wait for that transfer below.
    if (DR16_WaitDmaCplt(20u) && DR16_ParseData(&receiver_) == DEVICE_OK) {
      const DR16_Data_t& data = receiver_.data;
      const RemoteInput input{
          data.ch_l_x, data.ch_l_y, data.ch_r_x, data.ch_r_y, data.ch_res,
          ConvertSwitch(data.sw_l), ConvertSwitch(data.sw_r),
          BSP_TIME_Get_ms(), ++sequence_, true};
      StoreLatest(input);
      continue;
    }

    (void)DR16_Offline(&receiver_);
    PublishOffline();
    (void)osDelay(1u);
  }
}

// ==================== 4. 离线快照与驱动枚举适配 ====================
void RemoteControlTask::PublishOffline() {
  RemoteInput input{};
  input.updated_ms = BSP_TIME_Get_ms();
  input.sequence = ++sequence_;
  input.online = false;
  StoreLatest(input);
}

RemoteSwitch RemoteControlTask::ConvertSwitch(DR16_SwitchPos_t position) const {
  switch (position) {
    case DR16_SW_UP: return RemoteSwitch::kUp;
    case DR16_SW_MID: return RemoteSwitch::kMiddle;
    case DR16_SW_DOWN: return RemoteSwitch::kDown;
    default: return RemoteSwitch::kUnknown;
  }
}

}  // namespace app
