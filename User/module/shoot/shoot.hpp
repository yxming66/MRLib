#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "component/controller/pid_core.h"
#include "component/filter/biquad.h"
#include "device/device.h"
#include "device/motor/core/motor_state.hpp"

namespace mr::module::shoot {

constexpr std::size_t kMaxFrictionMotors = 6;
constexpr std::size_t kMaxLevels = 2;

enum class Status : uint8_t {
  kOk,
  kInvalidArgument,
  kInvalidConfiguration,
  kInvalidState,
  kStaleFeedback,
  kDeviceError,
};

enum class Mode : uint8_t { kSafe, kSingle, kBurst, kContinuous };
enum class RunningState : uint8_t { kIdle, kReady, kFiring };
enum class JamState : uint8_t { kNormal, kSuspected, kConfirmed, kRecovering };

struct Command {
  Mode mode = Mode::kSafe;
  bool ready = false;
  bool fire = false;
};

struct HeatData {
  bool online = false;
  float maximum = 0.0f;
  float cooling_per_second = 0.0f;
  float per_shot = 0.0f;
  float current = 0.0f;
};

struct HeatConfig {
  bool enabled = false;
  // Thresholds are fractions of maximum when <= 1, otherwise absolute heat.
  float maximum_frequency_hz = 0.0f;
  float warn_threshold = 0.7f;
  float saturation_threshold = 0.4f;
  float stop_threshold = 0.08f;
  // <= 1 means a fraction of the maximum shot count; > 1 means shot count.
  float reserve_shots = 0.0f;
};

struct Config {
  std::size_t friction_count = 0;
  std::array<uint8_t, kMaxFrictionMotors> friction_level{}; // 1-based.
  std::array<float, kMaxLevels> level_velocity_ratio{1.0f, 1.0f};
  float friction_target_velocity_rad_s = 0.0f;
  float friction_input_cutoff_hz = 0.0f;
  float friction_output_cutoff_hz = 0.0f;
  MrPidConfig friction_pid{}; // rad/s -> N*m.

  std::size_t trigger_teeth_per_revolution = 0;
  float trigger_max_velocity_rad_s = 0.0f; // Feeder disk output.
  float trigger_position_cutoff_hz = 0.0f;
  float trigger_velocity_cutoff_hz = 0.0f;
  float trigger_output_cutoff_hz = 0.0f;
  MrPidConfig trigger_position_pid{}; // rad -> rad/s.
  MrPidConfig trigger_velocity_pid{}; // rad/s -> N*m.

  float shot_frequency_hz = 0.0f;
  std::size_t burst_shots = 1;
  bool jam_detection_enabled = false;
  float jam_current_threshold_a = 0.0f;
  float jam_suspected_time_s = 0.0f;
  float jam_recovery_time_s = 0.3f;
  float jam_recovery_current_a = 0.0f;
  float shot_detect_velocity_drop_rad_s = 0.0f;
  float shot_detect_suspected_time_s = 0.0005f;
  float friction_ready_ratio = 0.95f;
  HeatConfig heat{};
  uint32_t feedback_timeout_ms = 20;
};

struct MotorFeedback {
  float position_rad = 0.0f;
  float velocity_rad_s = 0.0f;
  float torque_nm = 0.0f;
  float current_a = 0.0f;
  float temperature_c = 0.0f;
  uint32_t timestamp_ms = 0;
  bool online = false;
};

// Application-owned adapter around a MotorT. RM CommitCommand stages a command;
// the application still owns RmCommandBus BeginCycle/Flush for the whole robot.
class MotorDevice {
 public:
  virtual ~MotorDevice() = default;
  virtual int8_t Register() = 0;
  virtual int8_t Enable() = 0;
  virtual int8_t Update() = 0;
  virtual mr::motor::MotorState State() const = 0;
  virtual int8_t SetTorque(float torque_nm) = 0;
  virtual int8_t CommitCommand() = 0;
  virtual void ClearPendingCommand() = 0;
  virtual int8_t Relax() = 0;
  virtual float PeakTorqueNm() const = 0;
  virtual float MaxVelocityRadS() const = 0;
  virtual float CurrentA() const = 0;
};

template <typename Motor>
class MotorDeviceAdapter final : public MotorDevice {
 public:
  explicit MotorDeviceAdapter(Motor& motor) : motor_(&motor) {}
  int8_t Register() override { return motor_->Register(); }
  int8_t Enable() override { return motor_->Enable(); }
  int8_t Update() override { return motor_->Update(); }
  mr::motor::MotorState State() const override { return motor_->GetState(); }
  int8_t SetTorque(float value) override { return motor_->SetTorque(value); }
  int8_t CommitCommand() override { return motor_->CommitCommand(); }
  void ClearPendingCommand() override { motor_->ClearPendingCommand(); }
  int8_t Relax() override { return motor_->Relax(); }
  float PeakTorqueNm() const override {
    return Motor::peak_torque() * motor_->GetInstallConfig().external_ratio;
  }
  float MaxVelocityRadS() const override {
    return Motor::max_velocity() / motor_->GetInstallConfig().external_ratio;
  }
  float CurrentA() const override {
    const auto state = motor_->GetState();
    const float torque_per_amp = Motor::torque_constant() * Motor::gear_ratio() *
        motor_->GetInstallConfig().external_ratio;
    return torque_per_amp > 0 ? state.torque_nm / torque_per_amp : 0.0f;
  }

 private:
  Motor* motor_;
};

struct Devices {
  std::array<MotorDevice*, kMaxFrictionMotors> friction{};
  MotorDevice* trigger = nullptr;
};

struct Feedback {
  std::array<MotorFeedback, kMaxFrictionMotors> friction{};
  MotorFeedback trigger{};
  bool valid = false;
};

struct Debug {
  Mode mode = Mode::kSafe;
  RunningState running_state = RunningState::kIdle;
  JamState jam_state = JamState::kNormal;
  float trigger_target_position_rad = 0.0f;
  float trigger_position_output_rad_s = 0.0f;
  float trigger_torque_output_nm = 0.0f;
  float friction_average_velocity_rad_s = 0.0f;
  float friction_velocity_drop_rad_s = 0.0f;
  float estimated_heat = 0.0f;
  float fused_heat = 0.0f;
  uint32_t shots_fired = 0;
  uint16_t shots_remaining = 0;
  uint16_t shots_available = UINT16_MAX;
  float actual_frequency_hz = 0.0f;
};

// Mechanism controller only. Input mapping, referee parsing, task scheduling and
// shared CAN bus transaction boundaries remain owned by the application.
class Shoot final {
 public:
  Shoot() = default;
  Shoot(const Shoot&) = delete;
  Shoot& operator=(const Shoot&) = delete;

  Status Register(const Config& config, const Devices& devices) {
    if (lifecycle_ != Lifecycle::kUnregistered) return Status::kInvalidState;
    if (!ValidConfig(config)) return Status::kInvalidConfiguration;
    if (!ValidDevices(config, devices)) return Status::kInvalidArgument;
    config_ = config;
    devices_ = devices;
    const std::size_t total = config_.friction_count + 1;
    for (std::size_t i = 0; i < total; ++i) {
      MotorDevice* device = At(i);
      if (device->Register() != DEVICE_OK) {
        lifecycle_ = Lifecycle::kFault;
        return Status::kDeviceError;
      }
    }
    if (!ConfigureControllers()) {
      lifecycle_ = Lifecycle::kFault;
      return Status::kInvalidConfiguration;
    }
    lifecycle_ = Lifecycle::kRegistered;
    return Status::kOk;
  }

  Status Enable(uint32_t now_ms) {
    if (lifecycle_ != Lifecycle::kRegistered) return Status::kInvalidState;
    const Status feedback_result = ReadFeedback(now_ms);
    if (feedback_result != Status::kOk) return feedback_result;
    for (std::size_t i = 0; i < config_.friction_count + 1; ++i) {
      if (At(i)->Enable() != DEVICE_OK) {
        lifecycle_ = Lifecycle::kFault;
        return Status::kDeviceError;
      }
    }
    lifecycle_ = Lifecycle::kEnabled;
    debug_.trigger_target_position_rad = TriggerDiskPosition();
    return Status::kOk;
  }

  Status Disable() {
    if (lifecycle_ != Lifecycle::kEnabled && lifecycle_ != Lifecycle::kFault)
      return Status::kInvalidState;
    ClearPending();
    int8_t result = DEVICE_OK;
    for (std::size_t i = 0; i < config_.friction_count + 1; ++i)
      if (At(i)->Relax() != DEVICE_OK) result = DEVICE_ERR;
    feedback_.valid = false;
    lifecycle_ = result == DEVICE_OK ? Lifecycle::kRegistered : Lifecycle::kFault;
    return result == DEVICE_OK ? Status::kOk : Status::kDeviceError;
  }

  Status UpdateFeedback(uint32_t now_ms) {
    if (lifecycle_ != Lifecycle::kEnabled) return Status::kInvalidState;
    return ReadFeedback(now_ms);
  }

  Status SetHeatData(const HeatData& data) {
    if (!Finite(data.maximum) || !Finite(data.cooling_per_second) ||
        !Finite(data.per_shot) || !Finite(data.current) || data.maximum < 0 ||
        data.cooling_per_second < 0 || data.per_shot < 0 || data.current < 0) {
      heat_.online = false;
      debug_.shots_available = UINT16_MAX;
      return Status::kInvalidArgument;
    }
    heat_ = data;
    if (!HeatActive()) debug_.shots_available = UINT16_MAX;
    return Status::kOk;
  }

  Status Update(float dt_s, uint32_t now_ms, const Command& command) {
    if (lifecycle_ != Lifecycle::kEnabled || !feedback_.valid)
      return Status::kInvalidState;
    for (std::size_t i = 0; i < config_.friction_count + 1; ++i) {
      const MotorFeedback& value = i == config_.friction_count
          ? feedback_.trigger : feedback_.friction[i];
      if (uint32_t(now_ms - value.timestamp_ms) > config_.feedback_timeout_ms) {
        feedback_.valid = false;
        ClearPending();
        return Status::kStaleFeedback;
      }
    }
    if (!Finite(dt_s) || dt_s < 1.0e-6f || dt_s > 1.0f ||
        static_cast<uint8_t>(command.mode) > static_cast<uint8_t>(Mode::kContinuous))
      return Status::kInvalidArgument;
    dt_s_ = dt_s;
    now_s_ += dt_s;
    UpdateHeat(dt_s);
    const bool ready = command.ready && command.mode != Mode::kSafe;
    if (command.mode != Mode::kSafe && jam_state_ == JamState::kRecovering) {
      last_fire_ = command.fire;
      debug_.mode = command.mode;
      if (!StageFriction(command.ready) ||
          !StageTrigger(debug_.trigger_target_position_rad)) return DeviceFailure();
      if (now_s_ - jam_since_s_ >= config_.jam_recovery_time_s &&
          std::fabs(feedback_.trigger.current_a) <= config_.jam_recovery_current_a) {
        debug_.jam_state = JamState::kNormal;
        jam_state_ = JamState::kNormal;
      }
      return Status::kOk;
    }
    if (!ready) {
      debug_.running_state = RunningState::kIdle;
      debug_.shots_remaining = 0;
      debug_.trigger_target_position_rad = TriggerDiskPosition();
      ResetControllers();
    }
    if (command.mode == Mode::kSafe) {
      for (std::size_t i = 0; i < config_.friction_count; ++i)
        if (!StageTorque(*devices_.friction[i], 0.0f)) return DeviceFailure();
      if (!StageTorque(*devices_.trigger, 0.0f)) return DeviceFailure();
      debug_.mode = Mode::kSafe;
      jam_state_ = JamState::kNormal;
      debug_.jam_state = JamState::kNormal;
      last_fire_ = command.fire;
      return Status::kOk;
    }

    const bool rising = ready && !last_fire_ && command.fire;
    if (ready && debug_.running_state == RunningState::kIdle)
      debug_.running_state = RunningState::kReady;
    if (rising && debug_.running_state == RunningState::kReady) {
      debug_.running_state = RunningState::kFiring;
      debug_.trigger_target_position_rad = TriggerDiskPosition();
      switch (command.mode) {
        case Mode::kSingle: debug_.shots_remaining = 1; break;
        case Mode::kBurst: debug_.shots_remaining = ClampShots(config_.burst_shots); break;
        case Mode::kContinuous: debug_.shots_remaining = UINT16_MAX; break;
        default: debug_.shots_remaining = 0; break;
      }
    }

    if (debug_.running_state == RunningState::kFiring) {
      const Status shot_result = AdvanceFeed(command);
      if (shot_result != Status::kOk) return shot_result;
    }
    if (!command.fire && debug_.running_state == RunningState::kFiring) {
      debug_.running_state = RunningState::kReady;
      debug_.shots_remaining = 0;
      debug_.trigger_target_position_rad = TriggerDiskPosition();
    }
    last_fire_ = command.fire;
    debug_.mode = command.mode;

    if (!UpdateJam()) return DeviceFailure();
    if (!StageFriction(ready) || !StageTrigger(debug_.trigger_target_position_rad))
      return DeviceFailure();
    (void)now_ms;
    return Status::kOk;
  }

  const Feedback& feedback() const { return feedback_; }
  const Debug& debug() const { return debug_; }
  const HeatData& heat_data() const { return heat_; }

 private:
  enum class Lifecycle : uint8_t { kUnregistered, kRegistered, kEnabled, kFault };
  struct Filter {
    MrBiquadCoefficients coefficients = MrBiquad_Bypass();
    MrBiquadState state{};
  };

  Status ReadFeedback(uint32_t now_ms) {
    feedback_ = {};
    for (std::size_t i = 0; i < config_.friction_count + 1; ++i) {
      MotorDevice* device = At(i);
      if (device->Update() != DEVICE_OK) return Status::kStaleFeedback;
      const auto state = device->State();
      const float current = device->CurrentA();
      if (!state.online || !state.feedback_valid || !state.position_valid ||
          uint32_t(now_ms - state.last_feedback_tick) > config_.feedback_timeout_ms ||
          !Finite(state.position_rad) || !Finite(state.velocity_rad_s) ||
          !Finite(state.torque_nm) || !Finite(state.temperature_c) || !Finite(current)) {
        feedback_.valid = false;
        return Status::kStaleFeedback;
      }
      MotorFeedback value{state.position_rad, state.velocity_rad_s, state.torque_nm,
                          current, state.temperature_c, state.last_feedback_tick, true};
      if (i == config_.friction_count) feedback_.trigger = value;
      else feedback_.friction[i] = value;
    }
    feedback_.valid = true;
    return Status::kOk;
  }

  static bool InitFilter(Filter& filter, float sample_hz, float cutoff_hz) {
    if (MrBiquad_LowPass(&filter.coefficients, sample_hz, cutoff_hz) == MR_FILTER_INVALID)
      return false;
    MrBiquad_Clear(&filter.state);
    MrBiquad_Reset(&filter.state, &filter.coefficients, 0.0f);
    return true;
  }
  static float ApplyFilter(Filter& filter, float input) {
    return MrBiquad_Update(&filter.state, &filter.coefficients, input);
  }
  static void ResetFilter(Filter& filter, float value) {
    MrBiquad_Reset(&filter.state, &filter.coefficients, value);
  }

  static bool Finite(float value) { return std::isfinite(value); }
  static uint16_t ClampShots(std::size_t value) {
    return value > UINT16_MAX ? UINT16_MAX : static_cast<uint16_t>(value);
  }

  static bool ValidPid(const MrPidConfig& c) {
    return MrPid_ConfigValid(&c);
  }

  static bool ValidConfig(const Config& c) {
    if (c.friction_count == 0 || c.friction_count > kMaxFrictionMotors ||
        c.trigger_teeth_per_revolution == 0 || c.feedback_timeout_ms == 0 ||
        !Finite(c.friction_target_velocity_rad_s) || c.friction_target_velocity_rad_s <= 0 ||
        !Finite(c.friction_input_cutoff_hz) || !Finite(c.friction_output_cutoff_hz) ||
        !Finite(c.trigger_max_velocity_rad_s) || c.trigger_max_velocity_rad_s <= 0 ||
        !Finite(c.trigger_position_cutoff_hz) || !Finite(c.trigger_velocity_cutoff_hz) ||
        !Finite(c.trigger_output_cutoff_hz) || !Finite(c.shot_frequency_hz) ||
        c.shot_frequency_hz <= 0 || c.burst_shots == 0 ||
        !Finite(c.jam_current_threshold_a) || c.jam_current_threshold_a < 0 ||
        !Finite(c.jam_suspected_time_s) || c.jam_suspected_time_s < 0 ||
        !Finite(c.jam_recovery_time_s) || c.jam_recovery_time_s < 0 ||
        !Finite(c.jam_recovery_current_a) || c.jam_recovery_current_a < 0 ||
        !Finite(c.shot_detect_velocity_drop_rad_s) || c.shot_detect_velocity_drop_rad_s < 0 ||
        !Finite(c.shot_detect_suspected_time_s) || c.shot_detect_suspected_time_s < 0 ||
        !Finite(c.friction_ready_ratio) || c.friction_ready_ratio <= 0 ||
        c.friction_ready_ratio > 1 || !ValidPid(c.friction_pid) ||
        !ValidPid(c.trigger_position_pid) || !ValidPid(c.trigger_velocity_pid)) return false;
    if (c.heat.enabled && (!Finite(c.heat.maximum_frequency_hz) ||
        c.heat.maximum_frequency_hz < 0 || !Finite(c.heat.warn_threshold) ||
        !Finite(c.heat.saturation_threshold) || !Finite(c.heat.stop_threshold) ||
        !Finite(c.heat.reserve_shots) || c.heat.reserve_shots < 0)) return false;
    for (std::size_t i = 0; i < c.friction_count; ++i)
      if (c.friction_level[i] == 0 || c.friction_level[i] > kMaxLevels) return false;
    for (float ratio : c.level_velocity_ratio)
      if (!Finite(ratio) || ratio <= 0) return false;
    return true;
  }

  static bool ValidDevices(const Config& c, const Devices& d) {
    if (!d.trigger) return false;
    for (std::size_t i = 0; i < c.friction_count; ++i) {
      if (!d.friction[i] || d.friction[i] == d.trigger) return false;
      for (std::size_t j = 0; j < i; ++j)
        if (d.friction[i] == d.friction[j]) return false;
      if (!Finite(d.friction[i]->PeakTorqueNm()) || d.friction[i]->PeakTorqueNm() <= 0 ||
          !Finite(d.friction[i]->MaxVelocityRadS()) ||
          d.friction[i]->MaxVelocityRadS() <= 0) return false;
    }
    return Finite(d.trigger->PeakTorqueNm()) && d.trigger->PeakTorqueNm() > 0 &&
        Finite(d.trigger->MaxVelocityRadS()) && d.trigger->MaxVelocityRadS() > 0;
  }

  bool ConfigureControllers() {
    for (std::size_t i = 0; i < config_.friction_count; ++i) {
      MrPidConfig pid = config_.friction_pid;
      pid.output_limit = pid.output_limit > 0
          ? fminf(pid.output_limit, devices_.friction[i]->PeakTorqueNm())
          : devices_.friction[i]->PeakTorqueNm();
      if (!MrPid_Configure(&friction_pid_[i], &pid) ||
          !InitFilter(friction_input_[i], pid.control_freq_hz,
                      config_.friction_input_cutoff_hz) ||
          !InitFilter(friction_output_[i], pid.control_freq_hz,
                      config_.friction_output_cutoff_hz))
        return false;
    }
    MrPidConfig position = config_.trigger_position_pid;
    position.error_period = 0.0f; // Feeder position is cumulative, not a one-turn angle.
    const float max_velocity = fminf(config_.trigger_max_velocity_rad_s,
                                     devices_.trigger->MaxVelocityRadS());
    position.output_limit = position.output_limit > 0
        ? fminf(position.output_limit, max_velocity) : max_velocity;
    MrPidConfig velocity = config_.trigger_velocity_pid;
    velocity.output_limit = velocity.output_limit > 0
        ? fminf(velocity.output_limit, devices_.trigger->PeakTorqueNm())
        : devices_.trigger->PeakTorqueNm();
    return MrPid_Configure(&trigger_position_pid_, &position) &&
        MrPid_Configure(&trigger_velocity_pid_, &velocity) &&
        InitFilter(trigger_position_input_, position.control_freq_hz,
                   config_.trigger_position_cutoff_hz) &&
        InitFilter(trigger_velocity_input_, velocity.control_freq_hz,
                   config_.trigger_velocity_cutoff_hz) &&
        InitFilter(trigger_output_, velocity.control_freq_hz,
                   config_.trigger_output_cutoff_hz);
  }

  MotorDevice* At(std::size_t index) const {
    return index < config_.friction_count ? devices_.friction[index] : devices_.trigger;
  }

  float TriggerDiskPosition() const {
    return feedback_.trigger.position_rad;
  }

  bool HeatActive() const {
    return config_.heat.enabled && heat_.online && Finite(heat_.maximum) && heat_.maximum > 0 &&
        Finite(heat_.cooling_per_second) && heat_.cooling_per_second >= 0 &&
        Finite(heat_.per_shot) && heat_.per_shot > 0 && Finite(heat_.current) &&
        heat_.current >= 0;
  }

  void UpdateHeat(float dt) {
    debug_.actual_frequency_hz = config_.shot_frequency_hz;
    if (!HeatActive()) {
      debug_.shots_available = UINT16_MAX;
      return;
    }
    if (heat_.current != last_referee_heat_) {
      estimated_heat_ = heat_.current;
      last_referee_heat_ = heat_.current;
    }
    if (estimated_heat_ > 0)
      estimated_heat_ = fmaxf(0.0f, estimated_heat_ - heat_.cooling_per_second * dt);
    debug_.fused_heat = fmaxf(heat_.current, estimated_heat_);
    debug_.estimated_heat = estimated_heat_;
    const float available = fmaxf(0.0f, heat_.maximum - debug_.fused_heat);
    debug_.shots_available = ClampShots(static_cast<std::size_t>(available / heat_.per_shot));
    const float warn = Threshold(config_.heat.warn_threshold, heat_.maximum);
    const float saturation = Threshold(config_.heat.saturation_threshold, heat_.maximum);
    const float stop = Threshold(config_.heat.stop_threshold, heat_.maximum);
    const float cooling_frequency = heat_.cooling_per_second / heat_.per_shot;
    const float max_frequency = config_.heat.maximum_frequency_hz > 0
        ? config_.heat.maximum_frequency_hz : config_.shot_frequency_hz;
    if (available > warn) debug_.actual_frequency_hz = max_frequency;
    else if (available > saturation && warn > saturation)
      debug_.actual_frequency_hz = cooling_frequency +
          (max_frequency - cooling_frequency) * (available - saturation) / (warn - saturation);
    else if (available > stop) debug_.actual_frequency_hz = cooling_frequency;
    else debug_.actual_frequency_hz = 0;
    if (config_.heat.reserve_shots > 0 && debug_.shots_available <= ReserveShots())
      debug_.actual_frequency_hz = 0;
  }

  static float Threshold(float value, float maximum) {
    if (value <= 0) return 0;
    return value <= 1 ? value * maximum : value;
  }

  uint16_t ReserveShots() const {
    if (!HeatActive() || config_.heat.reserve_shots <= 0) return 0;
    if (config_.heat.reserve_shots > 1) return ClampShots(
        static_cast<std::size_t>(config_.heat.reserve_shots));
    return ClampShots(static_cast<std::size_t>(ceilf(config_.heat.reserve_shots *
        heat_.maximum / heat_.per_shot)));
  }

  void DetectShots() {
    float sum = 0;
    for (std::size_t i = 0; i < config_.friction_count; ++i)
      sum += fabsf(friction_velocity_[i]);
    debug_.friction_average_velocity_rad_s = sum / config_.friction_count;
    const float target = config_.friction_target_velocity_rad_s;
    debug_.friction_velocity_drop_rad_s = target - debug_.friction_average_velocity_rad_s;
    if (debug_.running_state == RunningState::kIdle) {
      shot_detect_ready_ = false;
      return;
    }
    if (!shot_detect_ready_) {
      if (debug_.friction_average_velocity_rad_s >= target * config_.friction_ready_ratio)
        shot_detect_ready_ = true;
      return;
    }
    if (debug_.friction_velocity_drop_rad_s > config_.shot_detect_velocity_drop_rad_s) {
      if (!shot_drop_pending_) {
        shot_drop_pending_ = true;
        shot_drop_since_s_ = now_s_;
      } else if (now_s_ - shot_drop_since_s_ >= config_.shot_detect_suspected_time_s) {
        ++debug_.shots_fired;
        estimated_heat_ += HeatActive() ? heat_.per_shot : 0.0f;
        shot_drop_pending_ = false;
        shot_detect_ready_ = false;
      }
    } else {
      shot_drop_pending_ = false;
    }
  }

  Status AdvanceFeed(const Command& command) {
    DetectShots();
    const float period = debug_.actual_frequency_hz > 0
        ? 1.0f / debug_.actual_frequency_hz : INFINITY;
    const float error = debug_.trigger_target_position_rad - TriggerDiskPosition();
    if (debug_.actual_frequency_hz > 0 && now_s_ - last_shot_s_ >= period && command.fire &&
        fabsf(error) <= 1.0f && debug_.shots_remaining > 0) {
      debug_.trigger_target_position_rad += 6.28318530718f /
          static_cast<float>(config_.trigger_teeth_per_revolution);
      last_shot_s_ = now_s_;
      if (debug_.shots_remaining != UINT16_MAX) --debug_.shots_remaining;
    }
    if (debug_.shots_remaining == 0) debug_.running_state = RunningState::kReady;
    return Status::kOk;
  }

  bool UpdateJam() {
    if (!config_.jam_detection_enabled || config_.jam_current_threshold_a <= 0 ||
        debug_.running_state == RunningState::kIdle) {
      jam_state_ = JamState::kNormal;
      debug_.jam_state = jam_state_;
      return true;
    }
    const float current = fabsf(feedback_.trigger.current_a);
    if (jam_state_ == JamState::kNormal && current > config_.jam_current_threshold_a) {
      jam_state_ = JamState::kSuspected;
      jam_since_s_ = now_s_;
    } else if (jam_state_ == JamState::kSuspected) {
      if (current < config_.jam_current_threshold_a) jam_state_ = JamState::kNormal;
      else if (now_s_ - jam_since_s_ >= config_.jam_suspected_time_s) {
        jam_state_ = JamState::kRecovering;
        debug_.trigger_target_position_rad -= 6.28318530718f /
            static_cast<float>(config_.trigger_teeth_per_revolution);
        jam_since_s_ = now_s_;
      }
    }
    debug_.jam_state = jam_state_;
    return true;
  }

  bool StageFriction(bool enabled) {
    for (std::size_t i = 0; i < config_.friction_count; ++i) {
      const auto raw = ApplyFilter(friction_input_[i], feedback_.friction[i].velocity_rad_s);
      friction_velocity_[i] = raw;
      const uint8_t level = config_.friction_level[i];
      const float target = enabled ? config_.friction_target_velocity_rad_s *
          config_.level_velocity_ratio[level - 1] : 0.0f;
      MrPidStep step = MrPid_Update(&friction_pid_[i], target, raw, 0, dt_s_);
      if (!step.valid) return false;
      const float torque = ApplyFilter(friction_output_[i], step.output);
      if (!StageTorque(*devices_.friction[i], torque)) return false;
    }
    return true;
  }

  bool StageTrigger(float target_position) {
    debug_.trigger_target_position_rad = target_position;
    const float position = ApplyFilter(trigger_position_input_, TriggerDiskPosition());
    MrPidStep position_step = MrPid_Update(&trigger_position_pid_, target_position, position, 0, dt_s_);
    if (!position_step.valid) return false;
    debug_.trigger_position_output_rad_s = fmaxf(-config_.trigger_max_velocity_rad_s,
        fminf(config_.trigger_max_velocity_rad_s, position_step.output));
    const float velocity = ApplyFilter(trigger_velocity_input_, feedback_.trigger.velocity_rad_s);
    MrPidStep velocity_step = MrPid_Update(&trigger_velocity_pid_,
        debug_.trigger_position_output_rad_s, velocity, 0, dt_s_);
    if (!velocity_step.valid) return false;
    debug_.trigger_torque_output_nm = ApplyFilter(trigger_output_, velocity_step.output);
    return StageTorque(*devices_.trigger, debug_.trigger_torque_output_nm);
  }

  static bool StageTorque(MotorDevice& device, float torque) {
    if (!Finite(torque) || device.SetTorque(torque) != DEVICE_OK ||
        device.CommitCommand() != DEVICE_OK) return false;
    return true;
  }

  Status DeviceFailure() {
    ClearPending();
    lifecycle_ = Lifecycle::kFault;
    debug_.running_state = RunningState::kIdle;
    debug_.shots_remaining = 0;
    return Status::kDeviceError;
  }
  void ClearPending() {
    for (std::size_t i = 0; i < config_.friction_count + 1; ++i)
      if (At(i)) At(i)->ClearPendingCommand();
  }
  void ResetControllers() {
    for (std::size_t i = 0; i < config_.friction_count; ++i) {
      MrPid_Reset(&friction_pid_[i], 0);
      ResetFilter(friction_input_[i], 0);
      ResetFilter(friction_output_[i], 0);
    }
    MrPid_Reset(&trigger_position_pid_, 0);
    MrPid_Reset(&trigger_velocity_pid_, 0);
    ResetFilter(trigger_position_input_, 0);
    ResetFilter(trigger_velocity_input_, 0);
    ResetFilter(trigger_output_, 0);
  }

  Config config_{};
  Devices devices_{};
  Feedback feedback_{};
  HeatData heat_{};
  Debug debug_{};
  std::array<MrPid, kMaxFrictionMotors> friction_pid_{};
  MrPid trigger_position_pid_{}, trigger_velocity_pid_{};
  std::array<Filter, kMaxFrictionMotors> friction_input_{}, friction_output_{};
  Filter trigger_position_input_{}, trigger_velocity_input_{}, trigger_output_{};
  std::array<float, kMaxFrictionMotors> friction_velocity_{};
  Lifecycle lifecycle_ = Lifecycle::kUnregistered;
  JamState jam_state_ = JamState::kNormal;
  float now_s_ = 0, dt_s_ = 0, last_shot_s_ = 0, jam_since_s_ = 0;
  float shot_drop_since_s_ = 0, estimated_heat_ = 0, last_referee_heat_ = -1;
  bool last_fire_ = false, shot_detect_ready_ = false, shot_drop_pending_ = false;
};

}  // namespace mr::module::shoot
