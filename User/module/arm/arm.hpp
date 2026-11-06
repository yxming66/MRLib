#pragma once

#include <optional>
#include "module/arm/arm_output_adapter.hpp"
#include "robotics/arm/control/arm_controller.hpp"

namespace mr::module::arm {
enum class Lifecycle { kUninitialized, kReady, kEnabled, kFault };

// Fixed storage; no motor factory, HAL, RTOS, remote mapping or robot preset.
template <int N>
class Arm final {
 public:
  using Model = robotics::arm::ArmModel<N>;
  using Command = robotics::arm::ArmCommand<N>;
  using Output = robotics::arm::ArmOutput<N>;
  using Controller = robotics::arm::control::ArmController<N>;
  struct Config {
    Model model{};
    robotics::arm::control::ArmControllerConfig<N> controller{};
    typename ArmOutputAdapter<N>::Config output{};
  };
  Arm() = default;
  Arm(const Arm&) = delete;
  Arm& operator=(const Arm&) = delete;
  Arm(Arm&&) = delete;
  Arm& operator=(Arm&&) = delete;

  // Configuration/model are copied. Devices must outlive this instance.
  // Init registers devices but never enables them. No implicit reconfiguration.
  int8_t Init(const Config& config) {
    if (lifecycle_ != Lifecycle::kUninitialized) return DEVICE_ERR_INITED;
    if (!config.model.Validate() || !ArmOutputAdapter<N>::IsConfigValid(config.output))
      return DEVICE_ERR;
    model_ = config.model;
    controller_.emplace(model_, config.controller);
    if (!controller_->IsConfigured()) { controller_.reset(); return DEVICE_ERR; }
    (void)output_.Bind(config.output);
    const auto result = output_.Register();
    if (result != DEVICE_OK) return Fault(result);
    registered_ = true;
    lifecycle_ = Lifecycle::kReady;
    return DEVICE_OK;
  }
  int8_t UpdateFeedback() {
    if (lifecycle_ == Lifecycle::kUninitialized) return DEVICE_ERR_NULL;
    pending_ = false;
    output_.ClearPending();
    const auto result = output_.UpdateFeedback(feedback_);
    if (result != DEVICE_OK) return Fault(result);
    feedback_ready_ = true;
    return DEVICE_OK;
  }
  // Recovery: fresh valid feedback -> explicit Enable. A registration failure
  // is permanent for this instance because IActuator has no unregister.
  int8_t Enable() {
    if (lifecycle_ == Lifecycle::kUninitialized) return DEVICE_ERR_NULL;
    if (!registered_) return last_device_error_;
    if (lifecycle_ == Lifecycle::kEnabled) return DEVICE_ERR_INITED;
    if (!robotics::arm::IsJointStateUsable(feedback_) ||
        !robotics::arm::safety::IsJointPositionSafe(model_, feedback_.position)) return DEVICE_ERR;
    const auto result = output_.Enable();
    if (result != DEVICE_OK) return Fault(result);
    controller_->Reset(feedback_);
    command_ = {};
    command_.mode = robotics::arm::ControlMode::kHold;
    lifecycle_ = Lifecycle::kEnabled;
    last_device_error_ = DEVICE_OK;
    return DEVICE_OK;
  }
  int8_t Disable() {
    if (lifecycle_ == Lifecycle::kUninitialized) return DEVICE_ERR_NULL;
    pending_ = false;
    feedback_ready_ = false;
    command_ = {};
    const auto result = output_.Relax();
    if (result != DEVICE_OK) return Fault(result);
    if (lifecycle_ != Lifecycle::kFault) lifecycle_ = Lifecycle::kReady;
    return DEVICE_OK;
  }
  int8_t SetCommand(const Command& command) {
    using robotics::arm::ControlMode;
    if (lifecycle_ != Lifecycle::kEnabled) return DEVICE_ERR;
    if (!robotics::arm::IsArmCommandFinite(command)) return DEVICE_ERR;
    switch (command.mode) {
      case ControlMode::kDisabled: return Disable();
      case ControlMode::kHold:
      case ControlMode::kMoveJoint:
      case ControlMode::kMoveCartesian: break;
      default: return DEVICE_ERR_UNSUPPORTED;
    }
    command_ = command;
    return DEVICE_OK;
  }
  int8_t Step(float dt_s) {
    if (lifecycle_ != Lifecycle::kEnabled) return DEVICE_ERR;
    if (!feedback_ready_) return Fault(DEVICE_ERR);
    feedback_ready_ = false;
    pending_ = false;
    output_.ClearPending();
    last_output_ = controller_->Update(feedback_, command_, dt_s);
    if (!robotics::arm::IsOk(last_output_.status)) return Fault(DEVICE_ERR);
    const auto result = output_.Apply(last_output_.joint_command);
    if (result != DEVICE_OK) return Fault(result);
    pending_ = true;
    return DEVICE_OK;
  }
  int8_t CommitOutput() {
    if (lifecycle_ != Lifecycle::kEnabled || !pending_) return DEVICE_ERR;
    pending_ = false;
    const auto result = output_.Commit();
    return result == DEVICE_OK ? DEVICE_OK : Fault(result);
  }
  int8_t SetZero(size_t index) {
    if (lifecycle_ != Lifecycle::kReady || index >= N) return DEVICE_ERR;
    const auto result = output_.SetZero(index);
    feedback_.valid = false;
    feedback_ready_ = false;
    return result == DEVICE_OK ? DEVICE_OK : Fault(result);
  }
  Lifecycle lifecycle() const { return lifecycle_; }
  int8_t last_device_error() const { return last_device_error_; }
  int8_t last_relax_error() const { return last_relax_error_; }
  const auto& feedback() const { return feedback_; }
  const Output& output() const { return last_output_; }
  const Model& model() const { return model_; }
 private:
  int8_t Fault(int8_t error) {
    lifecycle_ = Lifecycle::kFault;
    last_device_error_ = error;
    pending_ = false;
    feedback_.valid = false;
    feedback_ready_ = false;
    command_ = {};
    last_relax_error_ = output_.Relax();
    return error;
  }
  Model model_{};
  std::optional<Controller> controller_{};
  ArmOutputAdapter<N> output_{};
  robotics::arm::JointState<N> feedback_{};
  Command command_{};
  Output last_output_{};
  Lifecycle lifecycle_ = Lifecycle::kUninitialized;
  int8_t last_device_error_ = DEVICE_OK;
  int8_t last_relax_error_ = DEVICE_OK;
  bool registered_ = false;
  bool pending_ = false;
  bool feedback_ready_ = false;
};
}  // namespace mr::module::arm
