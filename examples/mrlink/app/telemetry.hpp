#pragma once
#include "component/communication/mrlink/mrlink.hpp"
namespace app {
struct Telemetry {
  static constexpr uint8_t topic = 0x20;
  uint16_t sequence;
  float yaw_rad;
};
}  // namespace app
namespace mr::link {
template <>
struct MessageCodec<app::Telemetry> {
  static constexpr uint16_t size = 6;
  static bool Encode(const app::Telemetry& v, uint8_t* p, uint16_t n) {
    if (!p || n != size) return false;
    wire::PutU16(p, v.sequence);
    wire::PutF32(p + 2, v.yaw_rad);
    return true;
  }
  static bool Decode(const uint8_t* p, uint16_t n, app::Telemetry* v) {
    if (!p || !v || n != size) return false;
    v->sequence = wire::GetU16(p);
    v->yaw_rad = wire::GetF32(p + 2);
    return true;
  }
};
}  // namespace mr::link
