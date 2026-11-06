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

DM/LZ 的型号量程和线协议单位暂沿用当前实现，需要实际型号手册与固件验收；本次反馈时序和传输分层验证不构成量程核定。
