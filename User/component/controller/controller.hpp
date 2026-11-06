#pragma once

/*
 * 控制器组件聚合入口。
 *
 * 快速用法：包含本文件，选择 PID/LQR/MPC 控制器，完成 Config 配置后，
 * 在固定采样周期调用对应 Update/Step 接口并读取控制输出。
 */

// 控制器组件聚合接口。使用方式：包含本文件获得通用控制器类型，先构造并配置
// 控制器，再在固定采样周期调用对应 Update/Step 接口。

/*
 * C++ 控制律组件聚合头文件。
 *
 * 模块需要同时使用多种控制器时，直接包含这个头文件：
 *
 *   #include "component/controller/controller.hpp"
 *
 * 本文件包含所有控制器类型：
 *   - pid: 通用 PID 控制器
 *   - lqr: LQR 状态反馈控制器
 *   - cascade: 位置—速度串级控制器
 */

#include "component/controller/cascade.hpp"
#include "component/controller/controller_types.hpp"
#include "component/controller/lqr.hpp"
#include "component/controller/pid.hpp"
