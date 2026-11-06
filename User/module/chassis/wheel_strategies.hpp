#pragma once

#include <array>
#include <cstddef>
#include <cmath>

#include "component/controller/pid.hpp"
#include "component/math/scalar.hpp"
#include "device/wheel/wheel_interface.hpp"

namespace mr::module::chassis {

/**
 * Optional slew limiter for wheel speed references.
 *
 * The limiter is deliberately independent of a chassis model and of the
 * wheel device. It only owns the previous reference, so it can be reused by
 * any fixed-size wheel arrangement. A non-positive limit disables limiting.
 */
template <std::size_t WheelCount>
class WheelSpeedPlanner final {
 public:
  static_assert(WheelCount > 0U, "a wheel planner needs at least one wheel");

  void Configure(float max_acceleration_mps2) {
    max_acceleration_mps2_ =
        mr::component::math::is_finite_scalar(max_acceleration_mps2) &&
                max_acceleration_mps2 > 0.0f
            ? max_acceleration_mps2
            : 0.0f;
  }

  void Reset() { previous_ = {}; }

  bool Apply(const std::array<float, WheelCount>& target,
             float dt_s,
             std::array<float, WheelCount>& planned) {
    if (!mr::component::math::values_are_finite(target.data(), WheelCount) ||
        !mr::component::math::is_finite_scalar(dt_s) || dt_s <= 0.0f) {
      return false;
    }

    if (max_acceleration_mps2_ <= 0.0f) {
      planned = target;
      previous_ = target;
      return true;
    }

    const float max_delta = max_acceleration_mps2_ * dt_s;
    if (!mr::component::math::is_finite_scalar(max_delta) || max_delta < 0.0f) {
      return false;
    }

    std::array<float, WheelCount> next{};
    for (std::size_t index = 0U; index < WheelCount; ++index) {
      const float previous = mr::component::math::is_finite_scalar(previous_[index])
                                 ? previous_[index]
                                 : 0.0f;
      next[index] = MoveTowards(previous, target[index], max_delta);
    }
    planned = next;
    previous_ = next;
    return true;
  }

  float max_acceleration_mps2() const { return max_acceleration_mps2_; }

 private:
  static float MoveTowards(float current, float target, float max_delta) {
    const float delta = target - current;
    if (!mr::component::math::is_finite_scalar(delta) ||
        std::fabs(delta) <= max_delta) {
      return target;
    }
    return current + (delta > 0.0f ? max_delta : -max_delta);
  }

  float max_acceleration_mps2_ = 0.0f;
  std::array<float, WheelCount> previous_{};
};

struct WheelPositionHoldConfig {
  bool enabled = false;
  float enter_speed_mps = 0.8f;
  float enter_still_time_s = 0.03f;
  float max_position_step_rad = 0.35f;
  mr::comp::cntlr::pid_config position_pid{};
};

/**
 * Optional zero-command wheel position hold.
 *
 * This strategy only decides whether a zero command should be held and
 * computes the corresponding torque. Device registration and command
 * submission remain the responsibility of the owning chassis module.
 */
template <std::size_t WheelCount>
class WheelPositionHold final {
 public:
  static_assert(WheelCount > 0U, "a position hold needs at least one wheel");

  bool Configure(const WheelPositionHoldConfig& config) {
    config_ = config;
    if (!config_.enabled) {
      Reset();
      return true;
    }
    if (!mr::component::math::is_finite_scalar(config_.enter_speed_mps) ||
        config_.enter_speed_mps < 0.0f ||
        !mr::component::math::is_finite_scalar(config_.enter_still_time_s) ||
        config_.enter_still_time_s < 0.0f ||
        !mr::component::math::is_finite_scalar(config_.max_position_step_rad) ||
        config_.max_position_step_rad <= 0.0f ||
        !mr::comp::cntlr::pid::IsConfigValid(config_.position_pid)) {
      configured_ = false;
      Reset();
      return false;
    }
    for (auto& controller : controllers_) {
      if (!controller.Configure(config_.position_pid)) {
        configured_ = false;
        Reset();
        return false;
      }
    }
    configured_ = true;
    Reset();
    return true;
  }

  void Reset() {
    active_ = false;
    still_time_s_ = 0.0f;
    hold_position_rad_ = {};
    last_position_rad_ = {};
    for (auto& controller : controllers_) {
      controller.Reset();
    }
  }

  bool Update(const std::array<mr::wheel::WheelFeedback, WheelCount>& feedback,
              bool zero_command,
              float dt_s,
              std::array<float, WheelCount>& torque) {
    torque = {};
    if (!configured_ || !config_.enabled ||
        !mr::component::math::is_finite_scalar(dt_s) || dt_s <= 0.0f) {
      return false;
    }

    if (!zero_command) {
      Reset();
      return false;
    }

    if (!active_) {
      for (const auto& wheel : feedback) {
        if (!wheel.online ||
            !mr::component::math::is_finite_scalar(wheel.linear_velocity_mps) ||
            std::fabs(wheel.linear_velocity_mps) > config_.enter_speed_mps) {
          still_time_s_ = 0.0f;
          return false;
        }
      }
      still_time_s_ += dt_s;
      if (still_time_s_ < config_.enter_still_time_s) {
        return false;
      }
      for (std::size_t index = 0U; index < WheelCount; ++index) {
        if (!mr::component::math::is_finite_scalar(feedback[index].position_rad)) {
          still_time_s_ = 0.0f;
          return false;
        }
        hold_position_rad_[index] = feedback[index].position_rad;
        last_position_rad_[index] = feedback[index].position_rad;
        controllers_[index].Reset();
      }
      active_ = true;
    }

    std::array<float, WheelCount> next_torque{};
    for (std::size_t index = 0U; index < WheelCount; ++index) {
      const float position = feedback[index].position_rad;
      if (!feedback[index].online ||
          !mr::component::math::is_finite_scalar(position)) {
        return false;
      }
      if (std::fabs(position - last_position_rad_[index]) >
          config_.max_position_step_rad) {
        hold_position_rad_[index] = position;
        controllers_[index].Reset();
      }
      const auto result = controllers_[index].Step(
          hold_position_rad_[index], position, dt_s);
      if (!result.valid) {
        return false;
      }
      next_torque[index] = result.output;
      last_position_rad_[index] = position;
    }
    torque = next_torque;
    return true;
  }

  bool active() const { return active_; }

 private:
  WheelPositionHoldConfig config_{};
  std::array<mr::comp::cntlr::pid, WheelCount> controllers_{};
  std::array<float, WheelCount> hold_position_rad_{};
  std::array<float, WheelCount> last_position_rad_{};
  float still_time_s_ = 0.0f;
  bool configured_ = false;
  bool active_ = false;
};

}  // namespace mr::module::chassis
