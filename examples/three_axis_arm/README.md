# 三轴机械臂项目装配示例

这里保存从原 module 迁出的三轴几何/质量预设与遥控工作空间策略。它们属于具体机器人，不是通用库默认值。

- `app/arm_model_config.*`：历史几何、质量、工具变换和规划参数转换；实际使用前按机器人测量值审核。
- `app/arm_command_adapter.*`：将项目 C 命令/摇杆输入转换为通用 `ArmCommand<3>`。
- `app/arm_control_types.h`：项目命令与参数，不再包含电机型号或总线地址。
- `app/assembly.hpp`：构造 `Arm<3>::Config` 并展示命令接入；项目提供三个执行器。

board 初始化 CAN，app 静态持有具体电机/执行器，再调用 `MakeConfig(parameters, actuators)` 和 `arm.Init(config)`。
温度保护和设备型号直接配置到 app 持有的设备。不能再依靠库按关节下标猜测型号。
`gravity_comp_scale` 应显式赋值（通常为 1），零就是关闭该关节的前馈。

遥控接入：Configure 遥控适配器 → UpdateFeedback → 需要时 Enable → 用当前反馈和正运动学结果 Reset 适配器 → 每周期检查 Control 的返回值。
Disable 或故障恢复后再次 Reset 适配器，避免沿用之前的遥控目标。调度、控制权切换和输入超时由 app 决定。

该示例源文件由主机回归编译，但不是可刷写的 STM32 工程。
