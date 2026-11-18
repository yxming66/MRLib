#pragma once

#include "module/gimbal/detail/joint_group.hpp"

namespace mr::module::gimbal {

class YawPitchGimbal final {
 public:
  using Config = JointConfig<2>;
  using Feedback = JointFeedback<2>;
  enum class Mode : uint8_t { kNormal };
  struct Target { float yaw_rad = 0.0f; float pitch_rad = 0.0f; };

  Status Register(const Config& config);
  Status Enable();
  Status Disable();
  Status SetMode(Mode mode);
  Status SetTarget(const Target& target);
  // Samples feedback, executes the selected mode, and commits actuator output.
  Status Update(float dt_s);

  Mode mode() const { return mode_; }
  const Target& target() const { return target_; }
  const Feedback& feedback() const { return joints_.feedback(); }
  const std::array<float, 2>& command() const { return joints_.command(); }
  Lifecycle lifecycle() const { return joints_.lifecycle(); }
  int8_t last_device_error() const { return joints_.last_device_error(); }
  int8_t last_relax_error() const { return joints_.last_relax_error(); }

 private:
  Status UpdateNormal(float dt_s);

  detail::JointGroup<2> joints_{};
  Target target_{};
  Mode mode_ = Mode::kNormal;
};

}  // namespace mr::module::gimbal
