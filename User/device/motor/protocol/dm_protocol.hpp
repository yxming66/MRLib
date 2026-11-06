#pragma once

// DM 电机协议实现，供 MotorT 内部使用。
// 对外推荐通过 MotorFactory::Create 创建 MotorT；典型顺序是 Register、
// 周期 Update、SetTorque/SetPosition/SetMIT，最后 CommitCommand。
#include "device/motor/bus/can_transport.hpp"
#include "device/motor/core/motor_install_spec.hpp"
#include "device/motor/core/motor_instance_config.hpp"
#include "device/motor/core/motor_state.hpp"
#include "device/motor/core/motor_traits.hpp"
#include "device/motor/protocol/motor_protocol_fwd.hpp"
#include "device/motor/protocol/pending_command_type.hpp"
#include "device/motor/protocol/protocol_common.hpp"
#include "device/motor/protocol/protocol_types.hpp"
namespace mr::motor {
template <MotorModel Model> class MotorProtocol<MotorKind::DM, Model> {
 public:
  MotorProtocol(const MotorInstanceConfig<MotorKind::DM>&, const MotorInstallSpec&, MotorState&, DmModule);
  int8_t Register(); int8_t Enable(); int8_t Disable(); int8_t Relax(); int8_t SetZero(); int8_t Update(); int8_t CommitCommand();
  bool HasPendingCommand() const { return pending_ != PendingCommandType::None; } void ClearPendingCommand(); int8_t SetTorque(float); int8_t SetVelocity(float); int8_t SetPosition(float, float); int8_t SetMIT(float, float, float, float, float);
 private:
  FeedbackRead Decode(DmFeedback&); int8_t Send(const CanFrame&) const; void RefreshState(const DmFeedback&);
  MotorInstanceConfig<MotorKind::DM> config_{}; MotorInstallSpec install_{}; MotorState& state_; TransmissionMapper mapper_{}; RotorPositionTracker tracker_{}; CanTransport transport_{}; DmModule module_{}; DmFeedback feedback_{}; uint32_t last_sequence_ = 0U; uint32_t last_feedback_tick_ = 0U; bool have_feedback_ = false; bool registered_ = false; bool last_online_ = false;
  PendingCommandType pending_ = PendingCommandType::None; float pending_velocity_ = 0.0f; float pending_position_ = 0.0f; float pending_velocity_limit_ = 0.0f; float pending_mit_position_ = 0.0f; float pending_mit_velocity_ = 0.0f; float pending_kp_ = 0.0f; float pending_kd_ = 0.0f; float pending_torque_ = 0.0f;
};
}
