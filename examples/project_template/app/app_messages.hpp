#pragma once
#include <array>
#include <cstdint>

namespace app {
// 本板内存消息，不能直接 memcpy 成 CAN/UART 报文；板间协议需要独立编解码。
enum class RemoteSwitch : uint8_t { kUnknown, kUp, kMiddle, kDown };
struct RemoteInput {
  float left_x = 0.0f;
  float left_y = 0.0f;
  float right_x = 0.0f;
  float right_y = 0.0f;
  float channel5 = 0.0f;
  RemoteSwitch left_switch = RemoteSwitch::kUnknown;
  RemoteSwitch right_switch = RemoteSwitch::kUnknown;
  uint32_t updated_ms = 0;
  uint32_t sequence = 0;
  bool online = false;
};

// 输入来源与期望模式分开：自动模式失去 NUC 时，实际来源为 kNone。
enum class CommandSource : uint8_t { kNone, kDr16, kNuc };
enum class ChassisMode : uint8_t { kDisabled, kManual, kStairAssist, kAuto };
enum class CommandSafetyReason : uint8_t {
  kNone, kNotInitialized, kRemoteUnavailable, kNotEnabled,
  kNoValidRoute, kConfigurationError, kPublishFailed,
};

// NUC 接收任务解码后的板内快照。received_ms 必须是本板接收时刻，
// 不使用未同步的 NUC 时间；接收任务每个新报文更新 sequence。
struct NucInputSnapshot {
  float vx_mps = 0.0f;
  float vy_mps = 0.0f;
  float wz_rad_s = 0.0f;
  uint32_t received_ms = 0;
  uint32_t sequence = 0;
  bool valid = false;
};

struct CommandCenterStatus {
  RemoteInput remote{};
  ChassisMode chassis_mode = ChassisMode::kDisabled;
  CommandSource chassis_source = CommandSource::kNone;
  CommandSafetyReason safety_reason = CommandSafetyReason::kNotInitialized;
  uint32_t updated_ms = 0;
  bool remote_fresh = false;
  bool nuc_fresh = false;
  bool armed = false;
  bool initialized = false;
  bool publish_ok = false;
  bool overflowed = false;
};

struct ChassisCommand {
  float vx_mps = 0;
  float vy_mps = 0;
  float wz_rad_s = 0;
  // 台阶模式的角度目标变化率及履带速度；未装配对应机构时不消费这些字段。
  float front_target_rate_rad_s = 0;
  float support_target_rate_rad_s = 0;
  float left_track_speed_rad_s = 0;
  float right_track_speed_rad_s = 0;
  bool enabled = false;
  bool stair_enabled = false;
  bool input_fresh = false;
  uint32_t received_ms = 0;  // 命令在本板的生成时刻。
  uint32_t source_received_ms = 0;  // 原始输入的本板接收时刻，不随 tick 刷新。
  CommandSource source = CommandSource::kNone;
};
struct WheelStatus {
  uint16_t feedback_id = 0;
  int8_t result = -1;
  bool online = false;
  uint32_t timestamp_ms = 0;
  uint32_t sequence = 0;
  float position_rad = 0;
  float velocity_rad_s = 0;
  float linear_velocity_mps = 0;
  float torque_nm = 0;
  float temperature_c = 0;
};
struct ChassisStatus {
  CommandCenterStatus command_center{};
  ChassisCommand command{};  // 同一周期实际胜出的命令。
  uint32_t updated_ms = 0;
  uint32_t heartbeat = 0;
  uint32_t deadline_misses = 0;
  uint32_t last_command_ms = 0;
  uint8_t online_mask = 0;
  bool feedback_registered = false;
  bool command_received = false;
  bool command_fresh = false;
  bool command_enabled = false;
  bool control_enabled = false;  // 本阶段固定 false，只读电机反馈。
  bool clock_advanced = false;
  std::array<WheelStatus, 4> wheels{};
};
}  // namespace app
