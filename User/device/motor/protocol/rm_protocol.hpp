#pragma once

// RM 电机协议实现，供 MotorT 内部使用。
// 对外推荐通过 MotorFactory::Create 创建 MotorT；周期任务中调用 Update，
// 控制任务调用 SetTorque 后再 CommitCommand。多个 RM 电机可共享同一 CAN 总线。
#include "device/motor/bus/rm_command_bus.hpp"
#include "device/motor/core/motor_install_spec.hpp"
#include "device/motor/core/motor_instance_config.hpp"
#include "device/motor/core/motor_state.hpp"
#include "device/motor/core/motor_traits.hpp"
#include "device/motor/protocol/motor_protocol_fwd.hpp"
#include "device/motor/protocol/protocol_common.hpp"
#include "device/motor/protocol/protocol_types.hpp"
namespace mr::motor {
template <MotorModel Model>
class MotorProtocol<MotorKind::RM, Model> {
 public:
  MotorProtocol(const MotorInstanceConfig<MotorKind::RM>&, const MotorInstallSpec&, MotorState&,
                RmModule);
  ~MotorProtocol() {
    if (config_.commands) config_.commands->Release(this);
    if (config_.port) config_.port->ReleaseEndpoints(this);
  }
  MotorProtocol(const MotorProtocol&) = delete;
  MotorProtocol& operator=(const MotorProtocol&) = delete;
  int8_t Register();
  int8_t Enable();
  int8_t Disable();
  int8_t Relax();
  int8_t Update();
  int8_t CommitCommand();
  bool HasPendingCommand() const {
    return pending_valid_ || (config_.commands && config_.commands->Pending(this));
  }
  void ClearPendingCommand();
  int8_t SetTorque(float);
  int8_t SetZero();

 private:
  FeedbackRead Decode(RmFeedback&);
  void RefreshState(const RmFeedback&);
  MotorInstanceConfig<MotorKind::RM> config_{};
  MotorInstallSpec install_{};
  MotorState& state_;
  TransmissionMapper mapper_{};
  CanTransport transport_{};
  RmModule module_{};
  RmFeedback feedback_{};
  uint32_t last_sequence_ = 0U;
  uint32_t last_tick_ = 0U;
  bool have_feedback_ = false;
  bool registered_ = false;
  bool position_initialized_ = false;
  float last_position_ = 0.0f;
  float accumulated_position_ = 0.0f;
  float zero_offset_ = 0.0f;
  float pending_current_ = 0.0f;
  bool pending_valid_ = false;
  bool pending_relax_ = false;
};
}  // namespace mr::motor
