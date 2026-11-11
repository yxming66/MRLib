#pragma once
namespace app {
// 由 Task_Init 调用。其他任务通过 app_channels 发布指令/读取状态。
bool CreateChassisTask();
}  // namespace app
