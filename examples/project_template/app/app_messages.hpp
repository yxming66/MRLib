#pragma once
#include <array>
#include <cstdint>

namespace app {
// 本板内存消息，不能直接 memcpy 成 CAN/UART 报文；板间协议需要独立编解码。
struct ChassisCommand {
  float vx_mps = 0;
  float vy_mps = 0;
  float wz_rad_s = 0;
  bool enabled = false;
  uint32_t received_ms = 0;  // PublishChassisCommand 使用本板时钟填写。
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
