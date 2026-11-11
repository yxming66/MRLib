#pragma once
#include <array>
#include <cstdint>
#include "robotics/chassis/chassis_kinematics.hpp"

// 本工程手写参数，不由生成器管理。长度统一使用米。
namespace app::params {
inline constexpr uint32_t kChassisPeriodMs = 1;
inline constexpr uint32_t kMotorFeedbackTimeoutMs = 20;
inline constexpr uint32_t kCommandTimeoutMs = 100;
inline constexpr uint32_t kChassisStackBytes = 4096;

// 固定顺序：右前、左前、左后、右后。
inline constexpr std::array<uint16_t, 4> kWheelFeedbackIds{0x201, 0x202, 0x203, 0x204};
inline constexpr float kWheelDiameterM = 0.18f;
inline constexpr float kWheelRadiusM = kWheelDiameterM / 2.0f;
inline constexpr float kFrontRearDistanceM = 0.60f;
inline constexpr float kLeftRightDistanceM = 0.50f;
// 库接口使用半轮距，勿直接传入上面的完整中心距。
inline constexpr mr::robotics::chassis::ChassisKinematicsConfig kKinematics{
    mr::robotics::chassis::ChassisModel::kMecanum,
    {kFrontRearDistanceM / 2.0f, kLeftRightDistanceM / 2.0f}};

// M3508 自带的 3591/187 减速比在库的 traits 中，不在这里重复乘入。
inline constexpr float kExternalRatio = 1.0f;
// 安装方向尚未校验；当前保留各电机原始正方向，仅观察单轮反馈。
// 麦轮模型本身已有左右轮符号约定，不能简单把右侧全部设为反转。
inline constexpr std::array<bool, 4> kWheelReverse{false, false, false, false};
inline constexpr bool kWheelDirectionsConfirmed = false;

static_assert(kWheelRadiusM > 0 && kFrontRearDistanceM > 0 && kLeftRightDistanceM > 0);
static_assert(kChassisPeriodMs > 0 && kMotorFeedbackTimeoutMs > 0);
}  // namespace app::params
