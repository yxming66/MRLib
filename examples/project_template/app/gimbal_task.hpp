#pragma once

#include <array>
#include <cstdint>

#include "module/gimbal/gimbal.hpp"
#include "task/periodic_task.hpp"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

namespace app {

struct GimbalCommand {
  bool folded = false;
  float target_world_yaw_rad = 0.0f;
  float target_pitch2_rad = 0.0f;
  uint32_t timestamp_ms = 0;
  bool valid = false;
};

struct GimbalImuYaw {
  float base_world_yaw_rad = 0.0f;
  uint32_t timestamp_ms = 0;
  bool valid = false;
};

enum class GimbalTaskMode : uint8_t {
  kUnknown,
  kNormal,
  kAligningForFold,
  kFolding,
  kFolded,
  kUnfolding,
  kFault,
};

struct GimbalTaskStatus {
  GimbalTaskMode mode = GimbalTaskMode::kUnknown;
  mr::robotics::gimbal::JointAngles encoder_angles{};
  float imu_base_yaw_rad = 0.0f;
  uint32_t heartbeat = 0;
  bool imu_used_for_heading = false;
  bool valid = false;
};

// Project-owned adapters connect these callbacks to message queues, the IMU
// task, and board time. read_command returns the latest snapshot each cycle.
struct GimbalTaskIo {
  void* context = nullptr;
  uint32_t (*now_ms)(void*) = nullptr;
  bool (*read_command)(void*, GimbalCommand*) = nullptr;
  bool (*read_imu_yaw)(void*, GimbalImuYaw*) = nullptr;
  bool (*publish_status)(void*, const GimbalTaskStatus*) = nullptr;
};

struct GimbalTaskConfig {
  mr::module::gimbal::Gimbal::Config module{};
  uint32_t period_us = 2000;
  uint32_t command_timeout_ms = 100;
  uint32_t imu_timeout_ms = 20;
  float folded_angle_rad = 0.0f;
  float normal_pit1_angle_rad = 1.57079632679f;
  float yaw_zero_offset_rad = 0.0f;
  float fold_rate_rad_s = 0.5f;
  float normal_pit2_rate_rad_s = 0.8f;
  float endpoint_tolerance_rad = 0.02f;
};

// Static-storage periodic app example. Build one instance in the board app and
// call Start from Task_Init after the board endpoints and queues are ready.
class GimbalTask final {
 public:
  static constexpr uint32_t kStackBytes = 4096;
  GimbalTask() = default;
  GimbalTask(const GimbalTask&) = delete;
  GimbalTask& operator=(const GimbalTask&) = delete;

  bool Start(mr::module::gimbal::Gimbal& gimbal,
             const GimbalTaskIo& io,
             const GimbalTaskConfig& config);
  bool Init();
  bool Step(float dt_s);
  void Fault(mr::task::TaskFault why);
  const GimbalTaskStatus& status() const { return status_; }

 private:
  bool ReadInputs(uint32_t now_ms);
  bool Publish();
  void BeginFold();
  void BeginUnfold();
  bool Within(float lhs, float rhs) const;

  mr::module::gimbal::Gimbal* gimbal_ = nullptr;
  GimbalTaskIo io_{};
  GimbalTaskConfig config_{};
  mr::task::PeriodicTask periodic_{};
  StaticTask_t control_{};
  alignas(8) StackType_t stack_[kStackBytes / sizeof(StackType_t)]{};
  GimbalCommand command_{};
  GimbalImuYaw imu_{};
  GimbalTaskStatus status_{};
  GimbalTaskMode mode_ = GimbalTaskMode::kUnknown;
  float transition_pit1_rad_ = 0.0f;
  float align_pit2_rad_ = 0.0f;
  float normal_pit2_rad_ = 0.0f;
  bool started_ = false;
};

}  // namespace app
