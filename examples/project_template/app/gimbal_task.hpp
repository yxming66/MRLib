#pragma once

#include <cstdint>

#include "module/gimbal/gimbal.hpp"
#include "task/periodic_task.hpp"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

namespace app {

struct GimbalCommand {
  float target_world_yaw_rad = 0.0f;
  float target_pitch_rad = 0.0f;
  uint32_t timestamp_ms = 0;
  bool valid = false;
};

struct GimbalImuYaw {
  float base_world_yaw_rad = 0.0f;
  uint32_t timestamp_ms = 0;
  bool valid = false;
};

enum class GimbalTaskMode : uint8_t { kUninitialized, kTracking, kFault };

struct GimbalTaskStatus {
  GimbalTaskMode mode = GimbalTaskMode::kUninitialized;
  std::array<float, 2> encoder_angles{};
  float imu_base_yaw_rad = 0.0f;
  uint32_t heartbeat = 0;
  bool valid = false;
};

// Project-owned callbacks adapt command/IMU channels and the board clock.
struct GimbalTaskIo {
  void* context = nullptr;
  uint32_t (*now_ms)(void*) = nullptr;
  bool (*read_command)(void*, GimbalCommand*) = nullptr;
  bool (*read_imu_yaw)(void*, GimbalImuYaw*) = nullptr;
  bool (*publish_status)(void*, const GimbalTaskStatus*) = nullptr;
};

struct GimbalTaskConfig {
  mr::module::gimbal::YawPitchGimbal::Config module{};
  uint32_t period_us = 2000;
  uint32_t command_timeout_ms = 100;
  uint32_t imu_timeout_ms = 20;
  float yaw_zero_offset_rad = 0.0f;
  float pitch_rate_rad_s = 0.8f;
};

// Static-storage yaw-pit task. Start it from Task_Init after board IO is ready.
class GimbalTask final {
 public:
  static constexpr uint32_t kStackBytes = 4096;
  GimbalTask() = default;
  GimbalTask(const GimbalTask&) = delete;
  GimbalTask& operator=(const GimbalTask&) = delete;

  bool Start(mr::module::gimbal::YawPitchGimbal& gimbal,
             const GimbalTaskIo& io,
             const GimbalTaskConfig& config);
  bool Init();
  bool Step(float dt_s);
  void Fault(mr::task::TaskFault why);
  const GimbalTaskStatus& status() const { return status_; }

 private:
  bool ReadInputs(uint32_t now_ms);
  bool Publish();

  mr::module::gimbal::YawPitchGimbal* gimbal_ = nullptr;
  GimbalTaskIo io_{};
  GimbalTaskConfig config_{};
  mr::task::PeriodicTask periodic_{};
  StaticTask_t control_{};
  alignas(8) StackType_t stack_[kStackBytes / sizeof(StackType_t)]{};
  GimbalCommand command_{};
  GimbalImuYaw imu_{};
  GimbalTaskStatus status_{};
  float pitch_target_rad_ = 0.0f;
  bool started_ = false;
};

}  // namespace app
