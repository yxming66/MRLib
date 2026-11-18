#pragma once

#include "module/gimbal/detail/joint_group.hpp"

namespace mr::module::gimbal {

class YawPitchPitchGimbal final {
 public:
  struct Config : JointConfig<3> {
    float low_fire_pitch1_rad = 0.35f;
    float high_fire_pitch1_rad = 1.05f;
    float posture_speed_rad_s = 0.3f;
    float leveling_speed_rad_s = 0.6f;
    float fold_yaw_speed_rad_s = 0.5f;
    float position_tolerance_rad = 0.05f;
    float settle_time_s = 0.1f;
  };
  using Feedback = JointFeedback<3>;
  enum class Mode : uint8_t { kFolded, kLowFire, kHighFire };
  enum class Stage : uint8_t {
    kInactive, kLeveling, kMoving, kSettling, kWorking, kFolded,
    kFoldYaw, kFoldPitch,
  };
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
  Stage stage() const { return stage_; }
  bool pitch2_control_ready() const { return stage_ == Stage::kWorking; }
  // Folded zero is horizontal. Opposing joint directions: head = pitch1 - pitch2.
  float head_pitch_rad() const { return feedback().angles[1] - feedback().angles[2]; }
  const Target& target() const { return target_; }
  const Feedback& feedback() const { return joints_.feedback(); }
  const std::array<float, 3>& command() const { return joints_.command(); }
  Lifecycle lifecycle() const { return joints_.lifecycle(); }
  int8_t last_device_error() const { return joints_.last_device_error(); }
  int8_t last_relax_error() const { return joints_.last_relax_error(); }

 private:
  Status UpdateTransition(float dt_s);
  Status UpdateFold(float dt_s);
  void BeginTransition();
  bool IsLevel() const;
  float PostureAngle() const;

  detail::JointGroup<3> joints_{};
  Target target_{};
  Config config_{};
  Mode mode_ = Mode::kFolded;
  Stage stage_ = Stage::kInactive;
  bool transition_requested_ = false;
  std::array<float, 3> transition_command_{};
  float settled_s_ = 0.0f;
};

}  // namespace mr::module::gimbal
