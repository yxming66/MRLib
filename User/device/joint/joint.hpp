#pragma once

/*
 * 关节封装：把电机输出映射为关节侧位置、速度和力矩。
 *
 * 快速用法：
 *   // 填入电机、传动比、方向和限位后构造 Joint
 *   joint.Register();
 *   joint.Update();
 *   joint.SetTargetAngle(target_rad);
 *   joint.PositionControl(target_rad, dt_s);
 *
 * 角度使用 rad，角速度使用 rad/s，力矩使用 N*m；多关节耦合应由上层
 * 机械臂控制器完成，不在单个 Joint 内访问其他关节。
 */

// 关节级封装，将电机输出映射为关节位置、速度和力矩。
// 使用方式：配置 Joint 的电机、传动比和限位后构造对象，周期调用 Update，
// 通过 SetPosition/SetVelocity/SetTorque 设置关节目标并提交命令。

#include <stdint.h>
#include <cstring>

#include "component/math/scalar.hpp"
#include "device/motor/motor.hpp"
#include "device/joint/joint_mapping.hpp"

namespace mr {

struct ActuatorState {
    float position;
    float velocity;
    float torque;
    bool online;

    ActuatorState() : position(0.0f), velocity(0.0f), torque(0.0f), online(false) {}
};

class IActuator {
public:
    virtual ~IActuator() = default;

    virtual int8_t Register() = 0;
    virtual int8_t Enable() = 0;
    virtual int8_t Update() = 0;
    virtual int8_t Relax() = 0;
    virtual int8_t SetZero() = 0;
    virtual int8_t CommitControl() = 0;
    virtual bool HasPendingControl() const = 0;
    virtual void ClearPendingControl() = 0;
    virtual int8_t SetMIT(float position, float velocity, float kp, float kd, float torque_ff) = 0;
    virtual int8_t SetTorque(float torque) = 0;
    virtual ActuatorState GetState() const = 0;
};

template <typename MotorType>
class MotorActuatorAdapter final : public IActuator {
public:
    MotorActuatorAdapter() : motor_(nullptr) {}
    explicit MotorActuatorAdapter(MotorType* motor) : motor_(motor) {}

    void SetMotor(MotorType* motor) { motor_ = motor; }
    MotorType* GetMotor() const { return motor_; }

    int8_t Register() override {
        return motor_ ? motor_->Register() : DEVICE_ERR_NULL;
    }
    int8_t Enable() override {
        return motor_ ? motor_->Enable() : DEVICE_ERR_NULL;
    }
    int8_t Update() override {
        return motor_ ? motor_->Update() : DEVICE_ERR_NULL;
    }
    int8_t Relax() override {
        return motor_ ? motor_->Relax() : DEVICE_ERR_NULL;
    }
    int8_t SetZero() override {
        return motor_ ? motor_->SetZero() : DEVICE_ERR_NULL;
    }
    int8_t CommitControl() override {
        return motor_ ? motor_->CommitCommand() : DEVICE_ERR_NULL;
    }
    bool HasPendingControl() const override { return motor_ ? motor_->HasPendingCommand() : false; }
    void ClearPendingControl() override {
        if (motor_) {
            motor_->ClearPendingCommand();
        }
    }
    int8_t SetMIT(float position, float velocity, float kp, float kd, float torque_ff) override {
        return motor_ ? motor_->SetMIT(position, velocity, kp, kd, torque_ff)
                      : DEVICE_ERR_NULL;
    }
    int8_t SetTorque(float torque) override {
        return motor_ ? motor_->SetTorque(torque) : DEVICE_ERR_NULL;
    }
    ActuatorState GetState() const override {
        ActuatorState state;
        if (!motor_) {
            return state;
        }
        const mr::motor::MotorState motor_state = motor_->GetState();
        state.position = motor_state.position_rad;
        state.velocity = motor_state.velocity_rad_s;
        state.torque = motor_state.torque_nm;
        state.online = motor_state.online;
        return state;
    }

private:
    MotorType* motor_;
};

struct JointControlParams {
    float qmin;
    float qmax;
    float kp;
    float kd;
};

class IJoint {
public:
    virtual ~IJoint() = default;

    virtual uint8_t GetId() const = 0;
    virtual const char* GetName() const = 0;
    virtual JointType GetType() const = 0;
    virtual float GetCurrentAngle() const = 0;
    virtual float GetCurrentVelocity() const = 0;
    virtual float GetTargetAngle() const = 0;
    virtual bool IsOnline() const = 0;
    virtual const JointControlParams& GetParams() const = 0;
    virtual float GetOffset() const = 0;
    virtual float GetFeedforwardTorque() const = 0;
    virtual float GetMotorCommandPosition() const = 0;
    virtual float GetMotorCommandVelocity() const = 0;
    virtual float GetMotorCommandTorque() const = 0;
    virtual float GetMotorCommandKp() const = 0;
    virtual float GetMotorCommandKd() const = 0;

    virtual void SetOffset(float offset) = 0;
    virtual void SetTargetAngle(float target) = 0;
    virtual void SetFeedforwardTorque(float torque) = 0;

    virtual int8_t Register() = 0;
    virtual int8_t Enable() = 0;
    virtual int8_t Update() = 0;
    virtual int8_t Relax() = 0;
    virtual int8_t SetZero() = 0;
    virtual int8_t CommitControl() = 0;
    virtual bool HasPendingControl() const = 0;
    virtual void ClearPendingControl() = 0;
    virtual int8_t PositionControl(float target_angle, float dt) = 0;
    virtual int8_t TorqueControl(float torque) = 0;
    virtual bool IsReached(float tolerance) const = 0;
};

class JointT final : public IJoint {
public:
        static constexpr uint8_t kJointNameMaxLen = 24U;

    JointT(uint8_t id, IActuator* actuator, const JointControlParams& params,
                     float q_offset, float freq,
                     const char* name = nullptr,
             JointType type = JointType::kRevolute)
        : id_(id),
      actuator_(actuator),
          params_(params),
          q_offset_(q_offset),
                    type_(type),
          transmission_() {
        (void)freq;
                SetName(name);
        for (uint8_t i = 0; i < kJointMaxCouplings; ++i) {
            coupling_sources_[i] = nullptr;
        }
        state_.current_angle = 0.0f;
        state_.current_velocity = 0.0f;
        state_.current_torque = 0.0f;
        state_.target_angle = 0.0f;
        state_.feedforward_torque = 0.0f;
        state_.motor_command_position = 0.0f;
        state_.motor_command_velocity = 0.0f;
        state_.motor_command_torque = 0.0f;
        state_.motor_command_kp = 0.0f;
        state_.motor_command_kd = 0.0f;
        state_.online = false;
    }

    JointT(const JointT&) = delete;
    JointT& operator=(const JointT&) = delete;

    uint8_t GetId() const override { return id_; }
    const char* GetName() const override { return name_; }
    JointType GetType() const override { return type_; }
    float GetCurrentAngle() const override { return state_.current_angle; }
    float GetCurrentVelocity() const override { return state_.current_velocity; }
    float GetTargetAngle() const override { return state_.target_angle; }
    bool IsOnline() const override { return state_.online; }
    const JointControlParams& GetParams() const override { return params_; }
    float GetOffset() const override { return q_offset_; }
    float GetFeedforwardTorque() const override { return state_.feedforward_torque; }
    float GetMotorCommandPosition() const override { return state_.motor_command_position; }
    float GetMotorCommandVelocity() const override { return state_.motor_command_velocity; }
    float GetMotorCommandTorque() const override { return state_.motor_command_torque; }
    float GetMotorCommandKp() const override { return state_.motor_command_kp; }
    float GetMotorCommandKd() const override { return state_.motor_command_kd; }

    void SetName(const char* name) {
        if (name == nullptr) {
            name_[0] = '\0';
            return;
        }

        std::strncpy(name_, name, kJointNameMaxLen - 1U);
        name_[kJointNameMaxLen - 1U] = '\0';
    }

    void SetOffset(float offset) override {
        if (mr::component::math::is_finite_scalar(offset)) {
            q_offset_ = offset;
        }
    }

    void SetTargetAngle(float target) override {
        if (mr::component::math::is_finite_scalar(target)) {
            state_.target_angle = target;
        }
    }

    void SetFeedforwardTorque(float torque) override {
        if (mr::component::math::is_finite_scalar(torque)) {
            state_.feedforward_torque = torque;
        }
    }

    void SetTransmission(const JointTransmission& transmission) {
        transmission_ = transmission;
    }

    const JointTransmission& GetTransmission() const {
        return transmission_;
    }

    bool SetCouplingSource(uint8_t term_index, const IJoint* source_joint) {
        if (term_index >= kJointMaxCouplings) {
            return false;
        }
        coupling_sources_[term_index] = source_joint;
        if (term_index >= transmission_.coupling_count) {
            transmission_.coupling_count = static_cast<uint8_t>(term_index + 1U);
        }
        transmission_.couplings[term_index].source_joint =
            (source_joint != nullptr) ? source_joint->GetId() : 0U;
        transmission_.couplings[term_index].enabled = (source_joint != nullptr);
        return true;
    }

    bool AddCoupling(const IJoint* source_joint, float coefficient) {
        const uint8_t source_joint_id =
            (source_joint != nullptr) ? source_joint->GetId() : 0U;
        if (!transmission_.AddCoupling(source_joint_id, coefficient)) {
            return false;
        }
        const uint8_t index = static_cast<uint8_t>(transmission_.coupling_count - 1U);
        coupling_sources_[index] = source_joint;
        transmission_.couplings[index].enabled = (source_joint != nullptr);
        return true;
    }

    void SetCoupledJoint(const IJoint* joint) {
        transmission_.ClearCouplings();
        for (uint8_t i = 0; i < kJointMaxCouplings; ++i) {
            coupling_sources_[i] = nullptr;
        }
        if (joint != nullptr) {
            AddCoupling(joint, 1.0f);
        }
    }

    int8_t Register() override {
        return actuator_ ? actuator_->Register() : DEVICE_ERR_NULL;
    }

    int8_t Enable() override {
        return actuator_ ? actuator_->Enable() : DEVICE_ERR_NULL;
    }

    int8_t Update() override {
        if (actuator_ == nullptr) {
            state_.online = false;
            return DEVICE_ERR_NULL;
        }
        const int8_t ret = actuator_->Update();
        if (ret != DEVICE_OK) {
            state_.online = false;
            return ret;
        }
        const ActuatorState actuator_state = actuator_->GetState();
        if (!actuator_state.online) {
            state_.online = false;
            return DEVICE_ERR_NO_DEV;
        }
        if (!mr::component::math::is_finite_scalar(actuator_state.position) ||
            !mr::component::math::is_finite_scalar(actuator_state.velocity) ||
            !mr::component::math::is_finite_scalar(actuator_state.torque)) {
            state_.online = false;
            return DEVICE_ERR;
        }

        const float joint_position =
            DecodeJointPosition(actuator_state.position);
        const float joint_velocity =
            DecodeJointVelocity(actuator_state.velocity);
        if (!mr::component::math::is_finite_scalar(joint_position) ||
            !mr::component::math::is_finite_scalar(joint_velocity)) {
            state_.online = false;
            return DEVICE_ERR;
        }

        state_.current_angle = joint_position;
        state_.current_velocity = joint_velocity;
        state_.current_torque = actuator_state.torque;
        state_.online = true;
        return DEVICE_OK;
    }

    int8_t Relax() override {
        return actuator_ ? actuator_->Relax() : DEVICE_ERR_NULL;
    }

    int8_t SetZero() override {
        if (actuator_ == nullptr) {
            return DEVICE_ERR_NULL;
        }
        const int8_t ret = actuator_->SetZero();
        if (ret == 0) {
            q_offset_ = 0.0f;
            state_.current_angle = 0.0f;
            state_.current_velocity = 0.0f;
            state_.target_angle = 0.0f;
            state_.motor_command_position = 0.0f;
            state_.motor_command_velocity = 0.0f;
            state_.motor_command_torque = 0.0f;
            state_.motor_command_kp = 0.0f;
            state_.motor_command_kd = 0.0f;
        }
        return ret;
    }

    int8_t CommitControl() override {
        return actuator_ ? actuator_->CommitControl() : DEVICE_ERR_NULL;
    }

    bool HasPendingControl() const override {
        return actuator_ ? actuator_->HasPendingControl() : false;
    }

    void ClearPendingControl() override {
        if (actuator_) {
            actuator_->ClearPendingControl();
        }
        state_.motor_command_position = 0.0f;
        state_.motor_command_velocity = 0.0f;
        state_.motor_command_torque = 0.0f;
        state_.motor_command_kp = 0.0f;
        state_.motor_command_kd = 0.0f;
    }

    int8_t PositionControl(float target_angle, float dt) override {
        (void)dt;
        if (actuator_ == nullptr) {
            return DEVICE_ERR_NULL;
        }
        if (!state_.online) {
            return DEVICE_ERR_NO_DEV;
        }
        if (!mr::component::math::is_finite_scalar(target_angle) ||
            !mr::component::math::is_finite_scalar(state_.current_angle) ||
            !mr::component::math::is_finite_scalar(params_.qmin) ||
            !mr::component::math::is_finite_scalar(params_.qmax) ||
            !mr::component::math::is_finite_scalar(params_.kp) ||
            !mr::component::math::is_finite_scalar(params_.kd) ||
            !mr::component::math::is_finite_scalar(state_.feedforward_torque) ||
            params_.qmin >= params_.qmax || params_.kp < 0.0f ||
            params_.kd < 0.0f) {
            return DEVICE_ERR;
        }

        float limited_target = target_angle;
        if (limited_target < params_.qmin) {
            limited_target = params_.qmin;
        } else if (limited_target > params_.qmax) {
            limited_target = params_.qmax;
        }

        constexpr float kLimitBoundaryEpsilon = 1.0e-4f;
        const bool at_lower_limit =
            state_.current_angle <= params_.qmin + kLimitBoundaryEpsilon;
        const bool at_upper_limit =
            state_.current_angle >= params_.qmax - kLimitBoundaryEpsilon;
        const bool pushing_lower = target_angle < state_.current_angle;
        const bool pushing_upper = target_angle > state_.current_angle;
        const bool blocked_by_limit =
            (at_lower_limit && pushing_lower) ||
            (at_upper_limit && pushing_upper);
        if (blocked_by_limit) {
            limited_target = state_.current_angle;
        }

        const float torque_ff =
            blocked_by_limit ? 0.0f : state_.feedforward_torque;
        const float actuator_position = EncodeActuatorPosition(limited_target);
        if (!mr::component::math::is_finite_scalar(actuator_position)) {
            return DEVICE_ERR;
        }

        const int8_t ret = actuator_->SetMIT(
            actuator_position, 0.0f, params_.kp, params_.kd, torque_ff);
        if (ret != DEVICE_OK) {
            return ret;
        }

        state_.target_angle = limited_target;
        state_.motor_command_position = actuator_position;
        state_.motor_command_velocity = 0.0f;
        state_.motor_command_torque = torque_ff;
        state_.motor_command_kp = params_.kp;
        state_.motor_command_kd = params_.kd;
        return DEVICE_OK;
    }

    int8_t TorqueControl(float torque) override {
        if (!mr::component::math::is_finite_scalar(torque)) {
            return DEVICE_ERR;
        }
        if (actuator_ == nullptr) {
            return DEVICE_ERR_NULL;
        }
        const int8_t ret = actuator_->SetTorque(torque);
        if (ret != DEVICE_OK) {
            return ret;
        }
        state_.feedforward_torque = torque;
        state_.motor_command_position = 0.0f;
        state_.motor_command_velocity = 0.0f;
        state_.motor_command_torque = torque;
        state_.motor_command_kp = 0.0f;
        state_.motor_command_kd = 0.0f;
        return DEVICE_OK;
    }

    bool IsReached(float tolerance) const override {
        return mr::component::math::abs_scalar(
                   state_.target_angle - state_.current_angle) < tolerance;
    }

private:
    struct JointStateCache {
        float current_angle;
        float current_velocity;
        float current_torque;
        float target_angle;
        float feedforward_torque;
        float motor_command_position;
        float motor_command_velocity;
        float motor_command_torque;
        float motor_command_kp;
        float motor_command_kd;
        bool online;
    };

    float SumCoupledJointAngles() const {
        float sum = 0.0f;
        for (uint8_t i = 0; i < transmission_.coupling_count; ++i) {
            if (!transmission_.couplings[i].enabled || coupling_sources_[i] == nullptr) {
                continue;
            }
            sum += transmission_.couplings[i].coefficient *
                   coupling_sources_[i]->GetCurrentAngle();
        }
        return sum;
    }

    float SumCoupledJointVelocities() const {
        float sum = 0.0f;
        for (uint8_t i = 0; i < transmission_.coupling_count; ++i) {
            if (!transmission_.couplings[i].enabled || coupling_sources_[i] == nullptr) {
                continue;
            }
            sum += transmission_.couplings[i].coefficient *
                   coupling_sources_[i]->GetCurrentVelocity();
        }
        return sum;
    }

    float DecodeJointPosition(float actuator_position) const {
        return ActuatorToJointPosition(
            transmission_, actuator_position, SumCoupledJointAngles(), q_offset_);
    }

    float DecodeJointVelocity(float actuator_velocity) const {
        return ActuatorToJointVelocity(
            transmission_, actuator_velocity, SumCoupledJointVelocities());
    }

    float EncodeActuatorPosition(float joint_position) const {
        return JointToActuatorPosition(
            transmission_, joint_position, SumCoupledJointAngles(), q_offset_);
    }

    uint8_t id_;
    IActuator* actuator_;
    JointControlParams params_;
    float q_offset_;
    JointType type_;
    char name_[kJointNameMaxLen];
    JointTransmission transmission_;
    const IJoint* coupling_sources_[kJointMaxCouplings];
    JointStateCache state_;
};

using DmJ4310Actuator = MotorActuatorAdapter<mr::motor::DmJ4310Motor>;
using DmJ4310PActuator = MotorActuatorAdapter<mr::motor::DmJ4310PMotor>;
using DmJ4340Actuator = MotorActuatorAdapter<mr::motor::DmJ4340Motor>;
using DmActuator = DmJ4310Actuator;
using LzActuator = MotorActuatorAdapter<mr::motor::LzRso3Motor>;
using DmJoint = JointT;
using LzJoint = JointT;

} // namespace mr
