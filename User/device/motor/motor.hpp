#pragma once

/*
 * 电机模块公共入口。
 *
 * 快速用法：包含本文件，选择 MotorFactory::Create 或具体 MotorT 别名，
 * 依次调用 Register、Enable、Update 和 SetXXX/CommitCommand。
 * 业务层不应直接依赖 protocol 目录下的内部实现头文件。
 */

// 电机模块公共入口。使用方式：包含本文件后选择 MotorFactory::Create 或具体
// MotorT 别名，依次调用 Register、Enable、Update 和 SetXXX/CommitCommand。

// motor 模块总入口。
// 默认导出强类型体系：MotorKind / MotorModel / Traits / Config / MotorT / aliases。
// 主路径不再依赖旧 runtime 抽象，仅保留当前仍有用途的静态工厂与类型别名入口。

#include "device/motor/core/motor_capability.hpp"
#include "device/motor/core/motor_instance_config.hpp"
#include "device/motor/core/motor_kind.hpp"
#include "device/motor/core/motor_model.hpp"
#include "device/motor/core/motor_aliases.hpp"
#include "device/motor/core/motor_state.hpp"
#include "device/motor/core/motor_t.hpp"
#include "device/motor/core/motor_traits.hpp"
