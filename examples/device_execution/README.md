# 手工装配设备执行链

本示例的周期函数 `app/rm_chassis_cycle.hpp` 已用于主机集成回归，连接真实 Chassis、MotorWheel、MotorT、RM 组帧代码；测试仅替换最底层 CAN 端口。

项目先初始化板级 CAN/IRQ/时钟，再持有一份 `CanPort`、一份 `RmCommandBus`、四个电机、四个轮子和一个 `Chassis`。按此顺序构造，反向析构。四个电机的 config 均填入同一 `port` 和 `commands`，选择实际型号、反馈 ID、方向与外部传动比；轮子传入电机引用和各自半径。

```cpp
mr::motor::RmCommandBus bus(port);
mr::motor::MotorInstanceConfig<mr::motor::MotorKind::RM> cfg;
cfg.port = &port;
cfg.commands = &bus;
cfg.id = 0x201;
mr::motor::RmM3508Motor motor0(cfg);
mr::wheel::MotorWheel<mr::motor::RmM3508Motor> wheel0(motor0, radius_m);
// 同样装配 wheel1..3，各电机反馈 ID 不重复。
// chassis.Register(config, {&wheel0,&wheel1,&wheel2,&wheel3}); 检查结果。
// chassis.Enable(); 检查结果。
// 每次注册、使能、放松都应在 BeginCycle 与 Flush 之间执行并处理错误。
const auto result = example::RunChassisCycle(chassis, bus, port.NowMs(), command, dt_s);
```

`config` 由项目提供：选择 robotics 运动学类型、几何参数、车体/轮速限制和轮速 PID。该调用不创建任务；应用决定运行周期、遥控映射及故障恢复。禁止同时为每个轮子再套一层速度控制器。

示例选择“任一底盘错误使整条 RM 总线置零”的策略，包括未到达的其他槽位。`result.ok()` 为 false 后应用应锁存故障；记录 `control/relax/flush/stop_flush`，停止正常命令，显式确认后再使能。总线若仍失败，继续显式重试停机发送；软件发送成功不等于物理电机已停机。共享其他机构的工程可在应用中制定更细的停机策略。

此示例只需选择 `mr_module_chassis`、`mr_device_motor`、`mr_device_wheel`。CMake 自动解析依赖，BSP 端口及 HAL/RTOS 源仍由 board 工程选择。独立机械臂关节则选择 `mr_device_joint` 与所用电机/控制器目标，并通过 `MotorActuatorAdapter` 绑定，不复制库文件。
