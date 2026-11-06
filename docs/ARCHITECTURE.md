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

- `component` 提供硬件无关的数学、滤波、PID、轨迹和容器；滤波器、控制器等可以有状态。
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
chassis.UpdateFeedback(board_time_ms);
chassis.SetCommand({1.0f, 0.0f, 0.0f});
chassis.Step(0.002f);
chassis.CommitOutput();
```

轮子设备由应用工程创建并以非 owning 指针注册。设备负责电机型号、齿轮比、方向、轮半径和总线细节；模块只使用 m/s、rad/s 和 N·m 的标准单位。

模块在以下情况进入 `kFault` 并尝试让已注册设备进入安全状态：设备注册或使能失败、反馈无效、命令为非有限值、运动学失败、控制器输出无效或提交失败。

`board_time_ms` 必须使用与轮子反馈时间戳相同的单调毫秒时钟。`ChassisConfig.feedback_timeout_ms` 限制接受的最大反馈年龄，默认 20 ms；超过时限会清除反馈、尝试放松所有已注册轮子并进入 `kFault`。无参 `UpdateFeedback()` 仅为迁移保留，缺少年龄检查，新项目应传入当前时间。

## CMake 边界

每个可复用层通过 target 暴露依赖。应用链接 `mr_module_chassis` 时会自动获得底盘运动学、控制器、滤波器和数学头文件；应用不需要复制源文件或手动追加 include 目录。

`cmake/targets.json` 登记当前公共 target 的名称、别名、入口头文件、编译源和直接依赖。顶层 CMake 用它创建 target，`mrctl.py` 用它校验 YAML 选择并展开编译检查头文件。项目配置只需列出直接使用的模块；新增 target 时同步更新 JSON Schema 的枚举，CLI 测试会检查两者一致。

当前 target 只覆盖已经能在无 HAL 环境中验证的部分。电机协议、UART、FDCAN、USB 和具体板级实现仍由目标工程提供，等设备接口稳定后再加入对应 target。

## 快速组装项目的目标

项目配置应描述“这个机器人由什么组成”，而不是列出要复制或勾选的源文件。长期目标是用 YAML 声明板级外设、设备类型与参数、机器人模型、模块及任务配置；工具解析配置和依赖图，报告缺项或冲突，并生成独立的装配代码与构建入口。生成区域和手写应用、板级适配区域必须分开，重复生成应当可重复且不覆盖用户文件。

模块层统一表示机器人能力。底盘、云台等模块对上提供稳定一致的配置和控制接口；注册时选择具体类型，并组合 `robotics` 层对应的模型与算法。模块不应随着底盘布局、云台种类或电机型号不断复制专用入口。定制外设通过板级适配实现设备接口，再由项目配置完成注册。

期望的项目体验是：从现有 STM32 板级模板新建目录，填写配置，运行 CLI，得到经过依赖检查的项目装配，再编译和实机验证。依赖检查应定位缺少的模块或头文件；项目维护者无需人工反复挑选传递依赖文件。VS Code 插件是 CLI 流程稳定后的可选界面，不能成为构建流程的唯一入口。

当前完成度有限：已登记的 CMake target 和 CLI 主要用于主机脚手架、target 依赖传递及公共头文件编译检查；STM32 外设、设备、模块和任务的 YAML 装配生成仍未实现。设计目标不可作为现成功能宣传。

## 当前范围

这一阶段建立的是架构基础，不替换 `User/module/chassis` 中现有重装机器人专用 Controller。旧 Controller 可以继续用于现有工程；新项目可先使用统一接口和模拟设备测试，再把真实轮子设备接入。这样可以逐步迁移控制逻辑，并保留行为回退路径。
