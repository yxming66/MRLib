# 库架构与构建边界

## 层次

```text
app/task
    ↓
module
    ↓
robotics     device
    ↓           ↓
component   board adapter
```

- `component` 提供无状态数学、滤波、PID、轨迹和容器。
- `robotics` 只描述硬件无关的模型和算法。它不 include HAL、RTOS、CAN 或具体电机头文件。
- `device` 把电机、轮子、关节和传感器封装成稳定的设备接口。板级工程提供具体总线和外设适配。
- `module` 组合算法和设备，负责生命周期、模式、命令、故障处理及输出提交。
- `task/app` 负责调度和机器人业务，不直接实现运动学，也不直接操作底层电机协议。

## 底盘示例

`User/robotics/chassis` 包含麦轮和前全向后麦轮两种运动学实现。`ChassisKinematics` 用固定存储按注册配置选择模型：

```cpp
mr::robotics::chassis::ChassisKinematics model;
mr::robotics::chassis::ChassisKinematicsConfig config{};
config.model = mr::robotics::chassis::ChassisModel::kMecanum;
config.geometry = {0.25f, 0.20f};
model.Configure(config);
```

`User/module/chassis/chassis.hpp` 提供统一控制入口。它只依赖 `WheelDevice` 接口，因此模块不绑定 M3508 或其他具体电机：

```cpp
mr::module::chassis::Chassis chassis;
chassis.Register(chassis_config, wheel_devices);
chassis.Enable();
chassis.UpdateFeedback();
chassis.SetCommand({1.0f, 0.0f, 0.0f});
chassis.Step(0.002f);
chassis.CommitOutput();
```

轮子设备由应用工程创建并以非 owning 指针注册。设备负责电机型号、齿轮比、方向、轮半径和总线细节；模块只使用 m/s、rad/s 和 N·m 的标准单位。

模块在以下情况进入 `kFault` 并尝试让已注册设备进入安全状态：设备注册或使能失败、反馈无效、命令为非有限值、运动学失败、控制器输出无效或提交失败。

## CMake 边界

每个可复用层通过 target 暴露依赖。应用链接 `mr_module_chassis` 时会自动获得底盘运动学、控制器、滤波器和数学头文件；应用不需要复制源文件或手动追加 include 目录。

当前 target 只覆盖已经能在无 HAL 环境中验证的部分。电机协议、UART、FDCAN、USB 和具体板级实现仍由目标工程提供，等设备接口稳定后再加入对应 target。

## 当前范围

这一阶段建立的是架构基础，不替换 `User/module/chassis` 中现有重装机器人专用 Controller。旧 Controller 可以继续用于现有工程；新项目可先使用统一接口和模拟设备测试，再把真实轮子设备接入。这样可以逐步迁移控制逻辑，并保留行为回退路径。
