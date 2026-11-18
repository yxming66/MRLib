#pragma once

#include <array>
#include <cstddef>
#include <cmath>
#include <cstdint>

namespace mr::module::stair_assist {

enum class Status : uint8_t {
  kOk,
  kInvalidArgument,
  kInvalidConfiguration,
  kInvalidState,
  kStaleFeedback,
  kDeviceError,
};

enum class State : uint8_t { kUnregistered, kRegistered, kEnabled, kFault };

struct AxisFeedback {
  float position = 0.0f;
  float velocity = 0.0f;
  uint32_t timestamp_ms = 0;
  bool online = false;
};

// Project adapters convert physical units to each motor's native output units.
// CommitCommand may transmit immediately (DM) or stage a shared bus (RM).
class AxisDevice {
 public:
  virtual ~AxisDevice() = default;
  virtual int8_t Register() = 0;
  virtual int8_t Enable() = 0;
  virtual int8_t UpdateFeedback(AxisFeedback& feedback) = 0;
  virtual int8_t SetPosition(float position, float max_velocity) = 0;
  virtual int8_t SetVelocity(float velocity) = 0;
  virtual int8_t UpdateCommand() = 0;
  virtual int8_t CommitCommand() = 0;
  virtual void ClearPendingCommand() = 0;
  virtual int8_t Relax() = 0;
};

struct Config {
  float min_front_angle_rad = 0.0f;
  float max_front_angle_rad = 0.0f;
  float max_front_speed_rad_s = 0.0f;
  float min_support_extension_mm = 0.0f;
  float max_support_extension_mm = 0.0f;
  float max_support_speed_mm_s = 0.0f;
  float max_track_speed_rad_s = 0.0f;
  uint32_t feedback_timeout_ms = 0;
};

struct Command {
  float front_angle_rad = 0.0f;
  float support_extension_mm = 0.0f;
  float left_track_speed_rad_s = 0.0f;
  float right_track_speed_rad_s = 0.0f;
};

struct Devices {
  AxisDevice* front_pivot = nullptr;
  AxisDevice* rear_support = nullptr;
  AxisDevice* left_track = nullptr;
  AxisDevice* right_track = nullptr;
};

struct Feedback {
  AxisFeedback front_pivot{};
  AxisFeedback rear_support{};
  AxisFeedback left_track{};
  AxisFeedback right_track{};
  bool valid = false;
};

// Coordinates the four actuators only. Climb sequencing and input selection
// stay in the application. All device storage remains owned by the caller.
class StairAssist final {
 public:
  Status Register(const Config& config, const Devices& devices) {
    if (state_ != State::kUnregistered) return Status::kInvalidState;
    if (!ValidConfig(config)) return Status::kInvalidConfiguration;
    if (!devices.front_pivot || !devices.rear_support || !devices.left_track ||
        !devices.right_track)
      return Status::kInvalidArgument;

    config_ = config;
    devices_ = devices;
    for (AxisDevice* device : Axes()) {
      if (device->Register() != 0) {
        state_ = State::kFault;
        return Status::kDeviceError;
      }
    }
    state_ = State::kRegistered;
    return Status::kOk;
  }

  // DM Enable sends a CAN frame immediately. Call only after the mechanism is secured.
  Status Enable() {
    if (state_ != State::kRegistered) return Status::kInvalidState;
    for (AxisDevice* device : Axes()) {
      if (device->Enable() != 0) {
        state_ = State::kFault;
        return Status::kDeviceError;
      }
    }
    state_ = State::kEnabled;
    return Status::kOk;
  }

  Status UpdateFeedback(uint32_t now_ms) {
    if (state_ != State::kEnabled || output_pending_) return Status::kInvalidState;
    const auto axes = Axes();
    AxisFeedback* values[] = {&feedback_.front_pivot, &feedback_.rear_support,
                              &feedback_. left_track, &feedback_.right_track};
    for (size_t i = 0; i < axes.size(); ++i) {
      if (axes[i]->UpdateFeedback(*values[i]) != 0 || !values[i]->online ||
          uint32_t(now_ms - values[i]->timestamp_ms) > config_.feedback_timeout_ms ||
          !std::isfinite(values[i]->position) || !std::isfinite(values[i]->velocity)) {
        feedback_ = {};
        return Status::kStaleFeedback;
      }
    }
    feedback_.valid = true;
    return Status::kOk;
  }

  Status SetCommand(const Command& command) {
    if (state_ != State::kEnabled) return Status::kInvalidState;
    if (!std::isfinite(command.front_angle_rad) ||
        !std::isfinite(command.support_extension_mm) ||
        !std::isfinite(command.left_track_speed_rad_s) ||
        !std::isfinite(command.right_track_speed_rad_s))
      return Status::kInvalidArgument;
    if (command.front_angle_rad < config_.min_front_angle_rad ||
        command.front_angle_rad > config_.max_front_angle_rad ||
        command.support_extension_mm < config_.min_support_extension_mm ||
        command.support_extension_mm > config_.max_support_extension_mm ||
        std::fabs(command.left_track_speed_rad_s) > config_.max_track_speed_rad_s ||
        std::fabs(command.right_track_speed_rad_s) > config_.max_track_speed_rad_s)
      return Status::kInvalidArgument;
    command_ = command;
    return Status::kOk;
  }

  // Prepares device-side commands but does not transmit or commit them.
  Status PrepareOutput() {
    if (state_ != State::kEnabled || !feedback_.valid || output_pending_)
      return Status::kInvalidState;
    if (devices_.front_pivot->SetPosition(command_.front_angle_rad,
                                          config_.max_front_speed_rad_s) != 0 ||
        devices_.rear_support->SetPosition(command_.support_extension_mm,
                                           config_.max_support_speed_mm_s) != 0 ||
        devices_.left_track->SetVelocity(command_.left_track_speed_rad_s) != 0 ||
        devices_.right_track->SetVelocity(command_.right_track_speed_rad_s) != 0)
      return Status::kDeviceError;
    for (AxisDevice* device : Axes()) {
      if (device->UpdateCommand() != 0) {
        for (AxisDevice* rollback : Axes()) rollback->ClearPendingCommand();
        return Status::kDeviceError;
      }
    }
    output_pending_ = true;
    feedback_.valid = false;
    return Status::kOk;
  }

  // DM commits transmit immediately; RM adapters may only stage to a shared bus.
  Status CommitOutput() {
    if (state_ != State::kEnabled || !output_pending_) return Status::kInvalidState;
    for (AxisDevice* device : Axes()) {
      if (device->CommitCommand() != 0) {
        state_ = State::kFault;
        output_pending_ = false;
        return Status::kDeviceError;
      }
    }
    output_pending_ = false;
    return Status::kOk;
  }

  State state() const { return state_; }
  const Command& command() const { return command_; }
  const Feedback& feedback() const { return feedback_; }
  bool output_pending() const { return output_pending_; }

 private:
  static bool ValidConfig(const Config& c) {
    return std::isfinite(c.min_front_angle_rad) && std::isfinite(c.max_front_angle_rad) &&
           c.min_front_angle_rad < c.max_front_angle_rad &&
           std::isfinite(c.max_front_speed_rad_s) && c.max_front_speed_rad_s > 0.0f &&
           std::isfinite(c.min_support_extension_mm) &&
           std::isfinite(c.max_support_extension_mm) &&
           c.min_support_extension_mm < c.max_support_extension_mm &&
           std::isfinite(c.max_support_speed_mm_s) && c.max_support_speed_mm_s > 0.0f &&
           std::isfinite(c.max_track_speed_rad_s) && c.max_track_speed_rad_s > 0.0f &&
           c.feedback_timeout_ms > 0;
  }

  std::array<AxisDevice*, 4> Axes() const {
    return {devices_.front_pivot, devices_.rear_support,
            devices_.left_track, devices_.right_track};
  }

  Config config_{};
  Devices devices_{};
  Command command_{};
  Feedback feedback_{};
  State state_ = State::kUnregistered;
  bool output_pending_ = false;
};

}  // namespace mr::module::stair_assist
