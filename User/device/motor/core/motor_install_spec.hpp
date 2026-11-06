#pragma once

// 电机安装方向和外部传动配置。使用方式：直接使用 kDirectDriveInstall，或填写

// 定义电机安装信息。
// 用于描述机构侧附加传动比与输出方向翻转，供协议层完成输出端物理量换算。

namespace mr::motor {

struct MotorInstallSpec {
  // 机构额外传动比，定义为：电机输出轴角 / 机构最终输出角。无额外传动时填 1.0f。
  float external_ratio;
  // 方向反转统一使用 MotorInstanceConfig::reverse。
};

inline constexpr MotorInstallSpec kDirectDriveInstall{
    1.0f,

};

}  // namespace mr::motor
