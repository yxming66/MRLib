#pragma once

// Hardware- and motor-model-independent contracts for joint execution.

#include <stdint.h>

#include "device/joint/joint_mapping.hpp"

namespace mr {

struct ActuatorState {
  float position = 0.0f;
  float velocity = 0.0f;
  float torque = 0.0f;
  bool online = false;
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
  virtual int8_t SetMIT(float position, float velocity, float kp, float kd,
                        float torque_ff) = 0;
  virtual int8_t SetTorque(float torque) = 0;
  virtual ActuatorState GetState() const = 0;
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

}  // namespace mr
