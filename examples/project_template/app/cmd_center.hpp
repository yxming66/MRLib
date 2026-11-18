#pragma once

#include <cstdint>
#include "app/app_messages.hpp"

namespace app {

// ==================== 输入绑定 ====================
// 接收任务提供受同步保护的最新快照；所有回调由命令中心的所有者任务调用。
// context 须长期有效。read_nuc 可空，此时自动模式输出安全命令。
// select_chassis_mode 未绑定时保持 Disabled；本项目显式绑定 SelectChassisMode。
struct CommandCenterBindings {
  void* context = nullptr;
  bool (*read_remote)(void*, RemoteInput&) = nullptr;
  bool (*read_nuc)(void*, NucInputSnapshot&) = nullptr;
  ChassisMode (*select_chassis_mode)(void*, const RemoteInput&) = nullptr;
};

// ==================== 项目模式选择回调 ====================
// 右拨杆中档手动、下档自动，上档/未知停止；
// 左拨杆负责布防和使能，手动时左拨杆下档选择上台阶模式。
ChassisMode SelectChassisMode(void* context, const RemoteInput& remote);

// ==================== 服务接口 ====================
// 只装配一次；同一 bindings 重复调用幂等，禁止运行期间更换输入对象。
bool InitCommandCenter(const CommandCenterBindings& bindings);

// 同一个任务调用 Init/Update/GetStatus。无有效路由或未初始化时返回安全命令。
ChassisCommand UpdateCommandCenter(uint32_t now_ms);
CommandCenterStatus GetCommandCenterStatus();

}  // namespace app
