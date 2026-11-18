#include "app/gimbal_task.hpp"

#include "component/math/scalar.hpp"

namespace app {
namespace {
bool IsConfigValid(const GimbalTaskIo& io, const GimbalTaskConfig& config) {
  return io.now_ms && io.read_command && io.read_imu_yaw &&
      config.period_us > 0 && config.command_timeout_ms > 0 && config.imu_timeout_ms > 0 &&
      mr::component::math::is_finite_scalar(config.yaw_zero_offset_rad) &&
      mr::component::math::is_finite_scalar(config.pitch_rate_rad_s) &&
      config.pitch_rate_rad_s > 0.0f;
}

bool Fresh(uint32_t now_ms, uint32_t timestamp_ms, uint32_t timeout_ms) {
  return uint32_t(now_ms - timestamp_ms) <= timeout_ms;
}
}  // namespace

bool GimbalTask::Start(mr::module::gimbal::YawPitchGimbal& gimbal,
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
  const uint32_t now_ms = io_.now_ms(io_.context);
  if (gimbal_->Register(config_.module) != mr::module::gimbal::Status::kOk ||
      !ReadInputs(now_ms) ||
      !Fresh(now_ms, imu_.timestamp_ms, config_.imu_timeout_ms) ||
      gimbal_->Enable() != mr::module::gimbal::Status::kOk)
    return false;

  pitch_target_rad_ = gimbal_->feedback().angles[1];
  status_.mode = GimbalTaskMode::kTracking;
  status_.encoder_angles = gimbal_->feedback().angles;
  status_.imu_base_yaw_rad = imu_.base_world_yaw_rad;
  status_.valid = true;
  return Publish();
}

bool GimbalTask::Step(float dt_s) {
  const uint32_t now_ms = io_.now_ms(io_.context);
  if (!ReadInputs(now_ms) || !Fresh(now_ms, command_.timestamp_ms, config_.command_timeout_ms) ||
      !Fresh(now_ms, imu_.timestamp_ms, config_.imu_timeout_ms))
    return false;

  const float yaw_target = mr::component::math::wrap_to_pi(
      command_.target_world_yaw_rad - imu_.base_world_yaw_rad - config_.yaw_zero_offset_rad);
  pitch_target_rad_ = mr::component::math::move_towards(
      pitch_target_rad_, command_.target_pitch_rad, config_.pitch_rate_rad_s * dt_s);
  const mr::module::gimbal::YawPitchGimbal::Target target{yaw_target, pitch_target_rad_};

  if (gimbal_->SetTarget(target) != mr::module::gimbal::Status::kOk ||
      gimbal_->Update(dt_s) != mr::module::gimbal::Status::kOk)
    return false;

  status_.mode = GimbalTaskMode::kTracking;
  status_.encoder_angles = gimbal_->feedback().angles;
  status_.imu_base_yaw_rad = imu_.base_world_yaw_rad;
  ++status_.heartbeat;
  status_.valid = true;
  return Publish();
}

void GimbalTask::Fault(mr::task::TaskFault why) {
  (void)why;
  if (gimbal_ && gimbal_->lifecycle() != mr::module::gimbal::Lifecycle::kUninitialized)
    (void)gimbal_->Disable();
  status_.mode = GimbalTaskMode::kFault;
  status_.valid = false;
  (void)Publish();
}

bool GimbalTask::ReadInputs(uint32_t now_ms) {
  GimbalCommand command{};
  if (!io_.read_command(io_.context, &command) || !command.valid ||
      !mr::component::math::is_finite_scalar(command.target_world_yaw_rad) ||
      !mr::component::math::is_finite_scalar(command.target_pitch_rad) ||
      !Fresh(now_ms, command.timestamp_ms, config_.command_timeout_ms))
    return false;
  command_ = command;

  GimbalImuYaw imu{};
  if (!io_.read_imu_yaw(io_.context, &imu) || !imu.valid ||
      !mr::component::math::is_finite_scalar(imu.base_world_yaw_rad))
    return false;
  imu_ = imu;
  return true;
}

bool GimbalTask::Publish() {
  return !io_.publish_status || io_.publish_status(io_.context, &status_);
}

}  // namespace app
