#pragma once

// 电机协议层通用组件。
// 仅承载协议无关的单位换算、位置累计和命令准备逻辑；厂商驱动调用与反馈解码仍保留在各协议实现中。

#include "component/math/cyclic_tracker.hpp"
#include "component/math/scalar.hpp"
#include "component/math/transmission_mapper.hpp"
#include "device/motor/core/motor_state.hpp"
#include "device/motor/core/motor_install_spec.hpp"
#include "device/device.h"

namespace mr::motor {

inline constexpr float kDefaultRotorResyncDeltaRad =
    1.75f * mr::component::math::kPi;

struct TransmissionMapper : mr::component::math::TransmissionRatioMapper {
    float torque_constant = 0.0f;

    static TransmissionMapper FromExternalRatio(float external_ratio, float torque_constant) {
        return {mr::component::math::TransmissionRatioMapper::FromRatio(external_ratio),
                torque_constant};
    }

    static TransmissionMapper FromTotalRatio(float total_ratio, float torque_constant) {
        return {mr::component::math::TransmissionRatioMapper::FromRatio(total_ratio),
                torque_constant};
    }

    float ToRotorPosition(float output_position_rad) const {
        return ToInput(output_position_rad);
    }

    float ToRotorVelocity(float output_velocity_rad_s) const {
        return ToInput(output_velocity_rad_s);
    }

    float ToRotorLimit(float output_limit) const {
        return ToInputLimit(output_limit);
    }

    float ToOutputPosition(float rotor_position_rad) const {
        return ToOutput(rotor_position_rad);
    }

    float ToOutputVelocity(float rotor_velocity_rad_s) const {
        return ToOutput(rotor_velocity_rad_s);
    }

    float ToOutputTorque(float torque_current) const {
        return torque_current * torque_constant * ratio;
    }

    float ToTorqueCurrent(float output_torque_nm, bool reverse_output = false) const {
        const float signed_torque = reverse_output ? -output_torque_nm : output_torque_nm;
        if (torque_constant <= 0.0f || ratio <= 0.0f) {
            return 0.0f;
        }
        return signed_torque / (torque_constant * ratio);
    }
};

using RotorPositionTracker = mr::component::math::CyclicPositionTracker;

enum class FeedbackRead { kNew, kNoNew, kInvalid };
inline bool ValidInstall(const MotorInstallSpec& install) {
    return mr::component::math::is_positive_scalar(install.external_ratio);
}
inline bool FeedbackTimeoutValid(uint32_t timeout) { return timeout > 0 && timeout <= INT32_MAX; }
// A repeated/absent sample remains usable until its original receive timestamp
// expires. Polling never refreshes sample age. Future timestamps are rejected.
inline int8_t KeepFeedbackUntilTimeout(MotorState& state, uint32_t now, uint32_t timeout) {
    if (state.feedback_valid && FeedbackTimeoutValid(timeout) &&
        now - state.last_feedback_tick <= timeout) return DEVICE_OK;
    if (state.online) ++state.feedback_lost_count;
    state.online = state.feedback_valid = state.position_valid = false;
    return DEVICE_ERR_NO_DEV;
}
inline bool IsFreshFeedback(uint32_t now, uint32_t timestamp, uint32_t timeout) {
    return FeedbackTimeoutValid(timeout) && now - timestamp <= timeout;
}

inline void ResetZeroedState(MotorState& state) {
    state.position_rad = 0.0f;
    state.position_single_turn_rad = 0.0f;
    state.position_valid = false;
    state.online = false;
    state.feedback_valid = false;
    state.last_commit_ok = true;
}

inline void MarkFeedbackUnavailable(MotorState& state) {
    state.online = false;
    state.feedback_valid = false;
    state.position_valid = false;
}

inline void MarkCommandFailed(MotorState& state) {
    state.last_commit_ok = false;
}

inline void MarkNoPendingCommand(MotorState& state) {
    state.command_pending = false;
}

} // namespace mr::motor
