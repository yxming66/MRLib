#pragma once

/*
 * 轮子设备封装：组合 RM M3508 电机、轮半径和轮子控制器。
 *
 * 快速用法：
 *   RmM3508WheelConfig config{};
 *   config.motor_param.can = BSP_CAN_1; config.motor_param.id = 1;
 *   config.radius_m = 0.076f;
 *   RmM3508Wheel wheel(config); wheel.Register();
 *
 * 控制周期：Update() -> SetTorque/SetPosition() -> CommitCommand()。
 */

// 轮子设备封装，将固定型号电机与轮半径、轮速控制器组合起来。
// 使用方式：填写 RmM3508WheelConfig，构造 RmM3508Wheel，依次调用
// Register、Enable、Update；控制时调用 SetTorque/SetPosition 和 CommitCommand。

#include <stdint.h>

#include "device/device.h"
#include "device/motor/core/motor_install_spec.hpp"
#include "device/motor/packages/controller/motor_controller.hpp"
#include "device/motor_rm.h"
#include "device/wheel/wheel_interface.hpp"

namespace mr::wheel {

struct WheelState {
  float position_rad = 0.0f;
  float angular_velocity_rad_s = 0.0f;
  float linear_velocity_mps = 0.0f;
  float torque_nm = 0.0f;
  float temperature_c = 0.0f;
  bool temperature_warning = false;
  bool temperature_over_limit = false;
  bool temperature_limit_latched = false;
  bool online = false;
};

struct RmM3508WheelConfig {
  MOTOR_RM_Param_t motor_param{};
  mr::motor::MotorInstallSpec install{};
  mr::motor::MotorTemperatureProtectionConfig temperature_protection{};
  mr::motor::MotorControllerConfig controller{};
  float radius_m = 0.0f;
};

class RmM3508Wheel final : public WheelDevice {
 public:
  explicit RmM3508Wheel(const RmM3508WheelConfig& config);

  int8_t Register() override;
  int8_t Enable() override;
  int8_t Relax() override;
  int8_t Update();
  int8_t UpdateFeedback();
  int8_t UpdateFeedback(WheelFeedback& feedback) override;
  int8_t UpdateCommand() override;
  int8_t SetTorque(float torque_nm) override;
  int8_t SetPosition(float position_rad, float max_velocity_rad_s = 0.0f);
  int8_t CommitCommand() override;
  bool HasPendingCommand() const;
  void ClearPendingCommand();

  const WheelState& State() const { return state_; }
  float RadiusM() const { return radius_m_; }
  bool IsValid() const { return controller_ != nullptr; }

 private:
  using Motor = mr::motor::RmM3508Motor;
  using Controller = mr::motor::MotorControllerT<Motor>;

  static float ResolveRadius(float radius_m);

  void RefreshState();

  float radius_m_;
  Motor* motor_;
  Controller* controller_;
  WheelState state_;
  alignas(Controller) unsigned char controller_storage_[sizeof(Controller)];
};

}  // namespace mr::wheel
