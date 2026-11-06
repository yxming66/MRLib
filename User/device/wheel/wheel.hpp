#pragma once
#include "component/math/scalar.hpp"
#include "device/device.h"
#include "device/wheel/wheel_interface.hpp"
namespace mr::wheel {
// Motor is app-owned and exposes output-shaft SI units. Bind a raw MotorT or
// a compatible drive whose Update only refreshes feedback, not a controller.
// Chassis owns wheel velocity PID; this adapter adds no second velocity loop.
template <typename Drive>
class MotorWheel final : public WheelDevice {
 public:
  MotorWheel(Drive& drive, float radius_m) : drive_(drive), radius_m_(radius_m) {}
  MotorWheel(const MotorWheel&) = delete;
  MotorWheel& operator=(const MotorWheel&) = delete;
  bool IsValid() const { return component::math::is_positive_scalar(radius_m_); }
  float RadiusM() const { return radius_m_; }
  int8_t Register() override { return IsValid() ? drive_.Register() : DEVICE_ERR; }
  int8_t Enable() override { return IsValid() ? drive_.Enable() : DEVICE_ERR; }
  int8_t Relax() override {
    drive_.ClearPendingCommand();
    return drive_.Relax();
  }
  int8_t UpdateFeedback(WheelFeedback& feedback) override {
    feedback = {};
    if (!IsValid()) return DEVICE_ERR;
    const auto result = drive_.Update();
    const auto state = drive_.GetState();
    feedback.position_rad = state.position_rad;
    feedback.linear_velocity_mps = state.velocity_rad_s * radius_m_;
    feedback.output_torque_nm = state.torque_nm;
    feedback.timestamp_ms = state.last_feedback_tick;
    feedback.sequence = state.feedback_sequence;
    feedback.online = result == DEVICE_OK && state.online && state.feedback_valid &&
                      state.position_valid && !state.protocol_fault &&
                      !state.temperature_limit_latched &&
                      component::math::is_finite_scalar(feedback.position_rad) &&
                      component::math::is_finite_scalar(feedback.linear_velocity_mps) &&
                      component::math::is_finite_scalar(feedback.output_torque_nm);
    if (!feedback.online) {
      drive_.ClearPendingCommand();
      return result == DEVICE_OK ? DEVICE_ERR_NO_DEV : result;
    }
    return DEVICE_OK;
  }
  int8_t SetTorque(float torque_nm) override {
    if (!IsValid() || !component::math::is_finite_scalar(torque_nm)) return DEVICE_ERR;
    return drive_.SetTorque(torque_nm);
  }
  // Drive::SetTorque prepares the command. No hidden update or second PID.
  int8_t UpdateCommand() override { return IsValid() ? DEVICE_OK : DEVICE_ERR; }
  int8_t CommitCommand() override { return IsValid() ? drive_.CommitCommand() : DEVICE_ERR; }
  bool HasPendingCommand() const { return drive_.HasPendingCommand(); }
  void ClearPendingCommand() { drive_.ClearPendingCommand(); }

 private:
  Drive& drive_;
  float radius_m_;
};
}  // namespace mr::wheel
