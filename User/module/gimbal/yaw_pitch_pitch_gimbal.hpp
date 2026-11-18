#pragma once

#include "module/gimbal/detail/joint_group.hpp"

namespace mr::module::gimbal {

class YawPitchPitchGimbal final {
 public:
  struct Config : JointConfig<3> {
    float low_fire_pitch1_rad = 0.35f;
    float high_fire_pitch1_rad = 1.05f;
  };
  using Feedback = JointFeedback<3>;
  enum class Mode : uint8_t { kNormal, kLowFire, kHighFire };
  struct Target {
    float yaw_rad = 0.0f;
    float pitch1_rad = 0.0f;
    float pitch2_rad = 0.0f;
  };

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
  const std::array<float, 3>& command() const { return joints_.command(); }
  Lifecycle lifecycle() const { return joints_.lifecycle(); }
  int8_t last_device_error() const { return joints_.last_device_error(); }
  int8_t last_relax_error() const { return joints_.last_relax_error(); }

 private:
  Status UpdateNormal(float dt_s);
  Status UpdateLowFire(float dt_s);
  Status UpdateHighFire(float dt_s);

  detail::JointGroup<3> joints_{};
  Target target_{};
  Mode mode_ = Mode::kNormal;
  float low_fire_pitch1_rad_ = 0.35f;
  float high_fire_pitch1_rad_ = 1.05f;
};

}  // namespace mr::module::gimbal
