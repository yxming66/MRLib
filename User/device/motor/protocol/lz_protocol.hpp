#pragma once

// LZ 扩展 CAN 电机协议实现，供 MotorT 内部使用。
// 对外推荐通过 MotorFactory::Create 创建 MotorT；典型顺序是 Register、
// 周期 Update、设置目标命令，再调用 CommitCommand 下发扩展帧。
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
template <MotorModel Model> class MotorProtocol<MotorKind::LZ, Model> {
 public:
  MotorProtocol(const MotorInstanceConfig<MotorKind::LZ>&, const MotorInstallSpec&, MotorState&, LzModule);
  int8_t Register(); int8_t Enable(); int8_t Disable(); int8_t Relax(); int8_t SetZero(); int8_t Update(); int8_t CommitCommand();
  bool HasPendingCommand() const { return pending_ != PendingCommandType::None; } void ClearPendingCommand(); int8_t SetTorque(float); int8_t SetVelocity(float); int8_t SetPosition(float, float); int8_t SetMIT(float, float, float, float, float);
 private:
  uint32_t FeedbackId() const; uint32_t BuildId(uint8_t, uint16_t, uint8_t) const; FeedbackRead DecodeFeedback(LzFeedback&); int8_t Send(uint32_t, const uint8_t*) const; void RefreshState(const LzFeedback&);
  MotorInstanceConfig<MotorKind::LZ> config_{}; MotorInstallSpec install_{}; MotorState& state_; TransmissionMapper mapper_{}; RotorPositionTracker tracker_{}; CanTransport transport_{}; LzModule module_{}; LzFeedback feedback_{}; uint32_t last_sequence_ = 0U; uint32_t last_feedback_tick_ = 0U; bool have_feedback_ = false; bool registered_ = false; bool last_online_ = false;
  PendingCommandType pending_ = PendingCommandType::None; float pending_velocity_ = 0.0f; float pending_position_ = 0.0f; float pending_velocity_limit_ = 0.0f; float pending_mit_position_ = 0.0f; float pending_mit_velocity_ = 0.0f; float pending_kp_ = 0.0f; float pending_kd_ = 0.0f; float pending_torque_ = 0.0f;
};
}
