#include "app/gimbal_task.hpp"

#include <cmath>

#include "component/math/scalar.hpp"

namespace app {
namespace {
bool IsConfigValid(const GimbalTaskIo& io, const GimbalTaskConfig& config) {
  return io.now_ms && io.read_command && io.read_imu_yaw &&
      config.module.kinematics.model == mr::robotics::gimbal::Model::kYawPitchPitch &&
      config.period_us > 0 && config.command_timeout_ms > 0 && config.imu_timeout_ms > 0 &&
      mr::component::math::is_finite_scalar(config.folded_angle_rad) &&
      mr::component::math::is_finite_scalar(config.normal_pit1_angle_rad) &&
      mr::component::math::is_finite_scalar(config.yaw_zero_offset_rad) &&
      mr::component::math::is_finite_scalar(config.fold_rate_rad_s) &&
      config.fold_rate_rad_s > 0.0f &&
      mr::component::math::is_finite_scalar(config.normal_pit2_rate_rad_s) &&
      config.normal_pit2_rate_rad_s > 0.0f &&
      mr::component::math::is_finite_scalar(config.endpoint_tolerance_rad) &&
      config.endpoint_tolerance_rad > 0.0f &&
      config.module.kinematics.yaw_pitch_pitch.min_pitch1_rad <= config.folded_angle_rad &&
      config.module.kinematics.yaw_pitch_pitch.max_pitch1_rad >= config.normal_pit1_angle_rad &&
      config.module.kinematics.yaw_pitch_pitch.min_pitch2_rad <= config.folded_angle_rad &&
      config.module.kinematics.yaw_pitch_pitch.max_pitch2_rad >= config.normal_pit1_angle_rad;
}

bool Fresh(uint32_t now_ms, uint32_t timestamp_ms, uint32_t timeout_ms) {
  return uint32_t(now_ms - timestamp_ms) <= timeout_ms;
}
}  // namespace

bool GimbalTask::Start(mr::module::gimbal::Gimbal& gimbal,
                       const GimbalTaskIo& io,
                       const GimbalTaskConfig& config) {
  if (started_ || osKernelGetState() != osKernelRunning || !IsConfigValid(io, config))
    return false;
  gimbal_ = &gimbal;
  io_ = io;
  config_ = config;
  const mr::task::PeriodicTaskConfig task_config{
      "Task_Gimbal", config_.period_us, kStackBytes, osPriorityAboveNormal};
  const mr::task::TaskHooks hooks{
      this,
      [](void* p) { return static_cast<GimbalTask*>(p)->Init(); },
      [](void* p, float dt_s) { return static_cast<GimbalTask*>(p)->Step(dt_s); },
      [](void* p, mr::task::TaskFault why) { static_cast<GimbalTask*>(p)->Fault(why); }};
  osThreadAttr_t storage{};
  storage.cb_mem = &control_;
  storage.cb_size = sizeof(control_);
  storage.stack_mem = stack_;
  storage.stack_size = sizeof(stack_);
  started_ = periodic_.Create(task_config, hooks, storage);
  return started_;
}

bool GimbalTask::Init() {
  if (gimbal_->Register(config_.module) != mr::module::gimbal::Status::kOk ||
      gimbal_->UpdateFeedback() != mr::module::gimbal::Status::kOk)
    return false;
  const auto& encoder = gimbal_->feedback().angles;
  if (Within(encoder[1], config_.folded_angle_rad) &&
      Within(encoder[2], config_.folded_angle_rad)) {
    mode_ = GimbalTaskMode::kFolded;
  } else if (Within(encoder[1], config_.normal_pit1_angle_rad)) {
    mode_ = GimbalTaskMode::kNormal;
    normal_pit2_rad_ = encoder[2];
  } else {
    // Do not guess the fold phase after a reset mid-motion. Require homing or
    // explicit operator recovery before enabling this mechanism.
    return false;
  }
  if (gimbal_->Enable() != mr::module::gimbal::Status::kOk) return false;
  if (!ReadInputs(io_.now_ms(io_.context))) return false;
  status_.mode = mode_;
  status_.encoder_angles = encoder;
  status_.valid = true;
  return Publish();
}

bool GimbalTask::Step(float dt_s) {
  const uint32_t now_ms = io_.now_ms(io_.context);
  if (!ReadInputs(now_ms) || !Fresh(now_ms, command_.timestamp_ms, config_.command_timeout_ms) ||
      gimbal_->UpdateFeedback() != mr::module::gimbal::Status::kOk)
    return false;

  const auto& encoder = gimbal_->feedback().angles;
  if (mode_ == GimbalTaskMode::kNormal && command_.folded) BeginFold();
  if (mode_ == GimbalTaskMode::kFolded && !command_.folded) BeginUnfold();

  mr::robotics::gimbal::JointAngles target = encoder;
  switch (mode_) {
    case GimbalTaskMode::kNormal: {
      if (!imu_.valid || !Fresh(now_ms, imu_.timestamp_ms, config_.imu_timeout_ms)) return false;
      const float desired_joint_yaw = mr::component::math::wrap_to_pi(
          command_.target_world_yaw_rad - imu_.base_world_yaw_rad - config_.yaw_zero_offset_rad);
      target[0] = desired_joint_yaw;
      target[1] = config_.normal_pit1_angle_rad;
      normal_pit2_rad_ = mr::component::math::move_towards(
          normal_pit2_rad_, command_.target_pitch2_rad,
          config_.normal_pit2_rate_rad_s * dt_s);
      target[2] = normal_pit2_rad_;
      status_.imu_used_for_heading = true;
      status_.imu_base_yaw_rad = imu_.base_world_yaw_rad;
      break;
    }
    case GimbalTaskMode::kAligningForFold:
      target[1] = transition_pit1_rad_;
      align_pit2_rad_ = mr::component::math::move_towards(
          align_pit2_rad_, transition_pit1_rad_, config_.fold_rate_rad_s * dt_s);
      target[2] = align_pit2_rad_;
      status_.imu_used_for_heading = false;
      if (Within(encoder[1], transition_pit1_rad_) &&
          Within(encoder[2], transition_pit1_rad_) &&
          Within(align_pit2_rad_, transition_pit1_rad_)) {
        mode_ = GimbalTaskMode::kFolding;
        transition_pit1_rad_ = encoder[1];
      }
      break;
    case GimbalTaskMode::kFolding:
      transition_pit1_rad_ = mr::component::math::move_towards(
          transition_pit1_rad_, config_.folded_angle_rad, config_.fold_rate_rad_s * dt_s);
      target[1] = transition_pit1_rad_;
      target[2] = transition_pit1_rad_;
      status_.imu_used_for_heading = false;
      if (Within(transition_pit1_rad_, config_.folded_angle_rad) &&
          Within(encoder[1], config_.folded_angle_rad) &&
          Within(encoder[2], config_.folded_angle_rad))
        mode_ = GimbalTaskMode::kFolded;
      break;
    case GimbalTaskMode::kUnfolding:
      transition_pit1_rad_ = mr::component::math::move_towards(
          transition_pit1_rad_, config_.normal_pit1_angle_rad, config_.fold_rate_rad_s * dt_s);
      target[1] = transition_pit1_rad_;
      target[2] = transition_pit1_rad_;
      status_.imu_used_for_heading = false;
      if (Within(transition_pit1_rad_, config_.normal_pit1_angle_rad) &&
          Within(encoder[1], config_.normal_pit1_angle_rad) &&
          Within(encoder[2], config_.normal_pit1_angle_rad)) {
        mode_ = GimbalTaskMode::kNormal;
        normal_pit2_rad_ = config_.normal_pit1_angle_rad;
      }
      break;
    case GimbalTaskMode::kFolded:
      target[1] = config_.folded_angle_rad;
      target[2] = config_.folded_angle_rad;
      status_.imu_used_for_heading = false;
      break;
    default: return false;
  }

  if (gimbal_->SetTargetAngles(target) != mr::module::gimbal::Status::kOk ||
      gimbal_->Step(dt_s) != mr::module::gimbal::Status::kOk ||
      gimbal_->CommitOutput() != mr::module::gimbal::Status::kOk)
    return false;
  status_.mode = mode_;
  status_.encoder_angles = encoder;
  ++status_.heartbeat;
  status_.valid = true;
  return Publish();
}

void GimbalTask::Fault(mr::task::TaskFault why) {
  (void)why;
  if (gimbal_ && gimbal_->lifecycle() != mr::module::gimbal::Lifecycle::kUninitialized)
    (void)gimbal_->Disable();
  mode_ = GimbalTaskMode::kFault;
  status_.mode = mode_;
  status_.valid = false;
  (void)Publish();
}

bool GimbalTask::ReadInputs(uint32_t now_ms) {
  GimbalCommand command{};
  if (!io_.read_command(io_.context, &command) || !command.valid ||
      !mr::component::math::is_finite_scalar(command.target_world_yaw_rad) ||
      !mr::component::math::is_finite_scalar(command.target_pitch2_rad) ||
      !Fresh(now_ms, command.timestamp_ms, config_.command_timeout_ms))
    return false;
  command_ = command;
  GimbalImuYaw imu{};
  if (io_.read_imu_yaw(io_.context, &imu) && imu.valid &&
      mr::component::math::is_finite_scalar(imu.base_world_yaw_rad))
    imu_ = imu;
  return true;
}

bool GimbalTask::Publish() {
  return !io_.publish_status || io_.publish_status(io_.context, &status_);
}

void GimbalTask::BeginFold() {
  transition_pit1_rad_ = gimbal_->feedback().angles[1];
  align_pit2_rad_ = gimbal_->feedback().angles[2];
  mode_ = GimbalTaskMode::kAligningForFold;
}

void GimbalTask::BeginUnfold() {
  transition_pit1_rad_ = gimbal_->feedback().angles[1];
  mode_ = GimbalTaskMode::kUnfolding;
}

bool GimbalTask::Within(float lhs, float rhs) const {
  return std::fabs(lhs - rhs) <= config_.endpoint_tolerance_rad;
}

}  // namespace app
