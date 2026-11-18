#pragma once
#include "app/app_messages.hpp"
namespace app {
// 由 Task_Init 调用。此任务独占 Init/Update/GetCommandCenterStatus。
bool CreateChassisTask();
// 任务上下文读取同一周期的完整快照，不直接跨任务访问 CmdCenter。
bool ReadChassisStatus(ChassisStatus& status);
}  // namespace app
