#include "app/app_init.h"
#include "app/chassis_task.hpp"
#if APP_EXAMPLE_ENABLE_DR16
#include "app/remote_control_task.hpp"
#include "board/bindings.hpp"
#endif
#include "bsp/fdcan.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"

volatile App_Diagnostics g_app{APP_NOT_STARTED, BSP_ERR, BSP_ERR};

namespace {
// 熟悉的初始化线程：先公共资源，后业务任务，完成后退出。
void Task_Init(void* argument) {
  (void)argument;
  g_app.state = APP_INITIALIZING;
  g_app.can_init_result = BSP_FDCAN_Init();  // 调度器已运行；全板只调用这里一次。
  if (g_app.can_init_result != BSP_OK) {
    g_app.state = APP_CAN_INIT_FAILED;
    osThreadExit();
    return;
  }
#if APP_EXAMPLE_ENABLE_DR16
  if (!app::RemoteControlTask::Instance().Start(
          BSP_UART_FindEndpoint(board::bindings::kRemoteUart))) {
    g_app.state = APP_REMOTE_TASK_CREATE_FAILED;
    osThreadExit();
    return;
  }
#endif
  if (!app::CreateChassisTask()) {
    g_app.state = APP_CHASSIS_TASK_CREATE_FAILED;
    osThreadExit();
    return;
  }
  // 接入 NUC 时在这里创建接收任务，并向 CmdCenter 绑定快照读取回调。
  // Task_Init 优先级高于底盘任务，下面的状态先写好，再退出。
  g_app.state = APP_TASKS_CREATED;
  osThreadExit();
}
}  // namespace

extern "C" int App_CreateInitTask(void) {
  static StaticTask_t control;
  alignas(8) static StackType_t stack[2048 / sizeof(StackType_t)];
  static osThreadId_t thread = nullptr;
  if (thread || osKernelGetState() != osKernelReady) return 0;
  osThreadAttr_t attributes{};
  attributes.name = "Task_Init";
  attributes.priority = osPriorityHigh;
  attributes.cb_mem = &control;
  attributes.cb_size = sizeof(control);
  attributes.stack_mem = stack;
  attributes.stack_size = sizeof(stack);
  thread = osThreadNew(Task_Init, nullptr, &attributes);
  return thread != nullptr;
}
