#pragma once
#include "app/app_messages.hpp"

namespace app {
// Task_Init 创建一次；其他接口只在已启动的任务中调用，不供 ISR 使用。
bool InitChannels();

// 单接收者（底盘任务），长度 1，最新有效指令覆盖旧值。
// 发布者应在收到/产生真正的新指令时调用，不能反复刷新旧指令时间。
bool PublishChassisCommand(const ChassisCommand& command);
bool TryReceiveChassisCommand(ChassisCommand& command);

// 底盘任务发布；通信、监控等多个任务可读取同一个完整快照。
// 读取不移除数据；尚未发布时返回 false。调用者自行检查 updated_ms。
bool PublishChassisStatus(const ChassisStatus& status);
bool ReadChassisStatus(ChassisStatus& status);
}  // namespace app
