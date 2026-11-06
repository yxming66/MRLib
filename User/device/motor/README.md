# 电机执行链

电机配置必须显式传入 `CanPort* port`，不再保存 BSP 总线枚举。端口由 board/app 持有，负责 IRQ、接收存储、发送拷贝和毫秒时钟。可选适配器 `bsp/adapters/can_port.hpp` 连接现有 FDCAN BSP；未包含该适配器时公共电机头不依赖 HAL/RTOS。

`ReadLatest/ReadNext` 返回 0 表示帧，`kCanNoFrame` 表示暂时没有新帧，负数表示错误。传输边界只接受经典 CAN 的 0～8 字节数据帧；拒绝远程帧、非法 ID/DLC，读取失败不修改目标帧。端口必须在返回 Send 前复制数据；成功只表示底层已接受，不代表设备确认。

RM/DM/LZ 配置有 `feedback_timeout_ms`（默认 20），端口 NowMs 与接收时间戳必须来自同一个单调毫秒时钟。重复 latest 帧或暂空队列不会刷新接收时间，也不会提前判离线；超过原帧龄或读帧错误才失效。时间回绕按无符号差处理；使用间隔和超时须小于半个 uint32 范围。软件调用 Update 成功可能是仍有效的缓存，不代表收到新帧，检查 sequence/timestamp 区分。

## RM 周期发送

每个物理端口只装配一个 `RmCommandBus`，先构造端口与 bus，再构造电机，全部位于同一控制上下文。电机不可复制/移动。电机生命周期必须短于 bus，bus 短于端口。

```cpp
mr::motor::RmCommandBus rm_bus(can_port);
mr::motor::MotorInstanceConfig<mr::motor::MotorKind::RM> config;
config.port = &can_port;
config.commands = &rm_bus;
config.id = 0x201;
mr::motor::RmM3508Motor motor(config);
// 检查 Register、Update、控制计算和 CommitCommand 的所有返回值。
rm_bus.BeginCycle();
motor.Update();
motor.SetTorque(target_nm);
motor.CommitCommand();    // 仅放入本周期组帧，不发送
const auto result = rm_bus.Flush(); // 即使上层计算失败也必须走此出口
```

每个使用的组每周期只发送一次，未提交/已取消槽位为零，不保持上周期的非零输出。Flush 失败只保留失败组，重复 Flush 重试；成功组不会重发。新 BeginCycle 放弃旧失败组的命令，重新以全零准备下一周期。Flush 后不允许追加命令，必须开始新周期。

`Relax/Disable` 对 RM 同样是准备零电流，项目必须 Flush 才真正提交。`HasPendingCommand/last_commit_ok` 直到 Flush 返回后才反映发送结果；模块 CommitOutput 成功只表示组帧接受。紧急时调用 `rm_bus.EmergencyStop()` 对已注册的所有组提交全零，并检查失败重试。析构仅移除软件槽位，不访问硬件；停止调度/销毁设备前必须显式停止并发送。

RM 多圈软件设零使用累计角，方向统一使用 `config.reverse`，旧 `install.reverse_output` 已删除；外部传动比必须有限且大于零。

## 协议单位与能力

MotorT 对外使用最终机构输出轴的 rad、rad/s、Nm。RM 由转子角/电流结合内部及外部传动换算；DM/LZ 的线协议本来就是电机输出轴的物理量，不重复应用内部减速比或转矩常数。DM/LZ 只乘/除 external_ratio，转矩按虚功守恒换算，MIT Kp/Kd 除以 external_ratio²。方向只使用 config.reverse。

DM 的 mit_position_range_rad、mit_velocity_range_rad_s、mit_torque_range_nm 是 MIT **线协议量程**，不是机构软限位；必须匹配电机固件设定。默认沿用旧 ±12.56637rad、±30rad/s、型号峰值转矩（torque_range=0 表示选择此默认），不代表所有型号固件都使用此量程。浮点位置/速度模式按型号能力裁剪，位置模式中的速度上限始终是非负幅值。两种电机的 MIT 增益都要求非负。

DM 提供 MIT、浮点位置/速度、使能、禁能、设零及 ClearFault。ClearFault 清除待发命令并发送 FF..FB；状态故障位只在真实反馈确认后改变。旧 H3510 Hybrid 的 int16(position*10000) 与旧声称角度范围不相容，暂不提供该编码路径，待设备固件规格确认后再实现。

LZ ClearFault 发送停止命令并置 Byte0=1，故障状态仍等待反馈确认。LZ 当前仅支持 Motion 模式；固件运行模式须由 board/app 确认。SetVelocity 使用 kp=0 的 MIT 阻尼闭环；SetPosition 只支持 max_velocity=0，非零上限返回 DEVICE_ERR_UNSUPPORTED。需要目标速度用 SetMIT 明确传入，需要轨迹限速由上层规划。LZ 量程表仍需按具体型号固件实机核验。

SetXXX 失败不覆盖此前有效 pending。DM/LZ CommitCommand 成功后清除 pending，失败时保留供显式重试；调用方也可清除并停机。设零成功清除待发控制、使反馈无效，收到新帧后才恢复。MotorT Update 失联、故障或数值无效会清除 pending。默认温度预警/限制为 70/80°C，非正阈值关闭对应检查；DisableTemperatureProtection 可显式关闭。温度拒绝操作返回失败，即便其内部 Relax 已被接受。

## 端点所有权与构建

同一物理总线只能对应一个 CanPort 实例，不能用多个包装实例绕过地址登记。端口内有 64 个软件端点槽位；RM 总线按使用组登记 TX ID，电机登记反馈 ID；DM 同时保留反馈 ID 和 MIT/位置/速度三个命令 ID；LZ 登记目标电机身份及反馈键。重复地址或容量不足在 Register 返回错误，不能静默覆盖。软件端点与发送槽位随实例析构释放，不会自动停硬件。

现有 BSP 不支持对订阅/解析器做完整卸载，按启动时静态装配使用，不将软件槽位释放理解为运行时设备热替换。BSP 的解析回调表固定 8 项，且把原通用空队列错误映射为 kCanNoFrame；需要精确区分硬件错误的项目可实现自己的 CanPort。

CMake 选择 mr_device_motor；控制器和学习器分别选择 mr_device_motor_controller、mr_device_soft_limit_learning，自动带入 timer/math/controller 依赖。轮子/关节接口是独立目标，具体电机/控制器由项目另行选择。手工装配与可编译周期示例见 examples/device_execution。

删除了与 can.h FDCAN 映射不匹配的旧 bxCAN can.c。现有 H7 工程使用 fdcan.c；其他 MCU 可实现独立的 CanPort 后端，不要将旧源混入当前兼容头。

主机测试验证协议换算与控制时序；Cortex-M7 编译检查不等于完整 HAL 固件链接、烧录或设备固件量程验收。
