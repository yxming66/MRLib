#pragma once

#include <cstdint>

namespace mr::wheel {

/** Normalized feedback exposed to robot modules by a wheel device. */
struct WheelFeedback {
  float position_rad = 0.0f;
  float linear_velocity_mps = 0.0f;
  float output_torque_nm = 0.0f;
  bool online = false;
};

/**
 * Hardware-independent wheel drive contract.
 *
 * Implementations own motor direction, gear ratio, wheel radius and the
 * conversion from output torque in N·m to the physical motor command. This
 * interface uses fixed ownership: modules keep non-owning pointers and never
 * allocate or delete wheel devices.
 */
class WheelDevice {
 public:
  virtual ~WheelDevice() = default;

  virtual int8_t Register() = 0;
  virtual int8_t Enable() = 0;
  virtual int8_t Relax() = 0;
  virtual int8_t UpdateFeedback(WheelFeedback& feedback) = 0;
  virtual int8_t SetTorque(float torque_nm) = 0;
  virtual int8_t UpdateCommand() = 0;
  virtual int8_t CommitCommand() = 0;
};

}  // namespace mr::wheel
