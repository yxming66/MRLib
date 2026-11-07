# 基础服务接口与迁移说明

这些接口用于项目手工装配和主机回归，尚未接入 YAML 自动装配。实例由项目持有，单个实例由一个执行上下文更新；跨任务传递快照的同步由应用处理。它们不创建线程、分配动态内存或初始化硬件。

## 二阶低通与陷波

`component/filter/biquad.h` 是 C11/C++17 共用的系数和状态实现。C 使用 `filter/filter.h`，C++ 使用 `low_pass_filter.hpp` / `notch_filter.hpp`。不使用 KF/EKF 时，无需包含矩阵；聚合头 `filter.hpp` 仍会包含全部滤波器。

```cpp
auto low = mr::comp::filter::low_pass::Build(40.0f, 1000.0f);
if (!low.valid()) { /* 配置错误，不能启用控制 */ }
low.Reset(initial_value);
float filtered = low.Update(raw_value); // 固定 1 kHz
// 或 low.Update(raw_value, actual_dt_s);
```

频率为 Hz，时间间隔为秒。截止频率不大于 0 表示禁用；陷波中心或带宽不大于 0 也表示禁用。非有限参数、非正采样频率、频率达到或超过奈奎斯特频率属于错误。`MR_FILTER_ENABLED`、`DISABLED` 和 `INVALID` 区分正常滤波、主动禁用和错误旁路；错误旁路返回输入，调用方必须检查状态，不能把它当成安全停机措施。

固定周期复用缓存系数；显式 dt 改变有效采样频率时重算，随后固定周期调用会恢复配置频率。显式 dt 覆盖配置采样频率，`status()` 反映最近有效周期路径；非法 dt 保持上次输出和状态，不消耗样本。非有限或超出共享标量幅值界限的输入也保持上次输出。

`Configure` 清空状态，setter 保留状态；切换参数或重新启用后是否 `Reset` 由应用按信号连续性决定。C 的 Init 从零开始，C++ 未 Reset 时首个有效样本用于初始化；需要相同起点时两端都显式 Reset。极低相对截止频率的单精度数值条件仍需按实际参数验证。

C 的 Init 现在返回 `MrFilterStatus`，原有忽略返回值的调用可继续编译，但应补上检查。C 滤波结构体布局已经改变，不承诺二进制兼容，不再直接访问旧系数字段；重新编译全部调用者。C++ 的 Configure/setter 现在返回 bool。`MrBiquad_TryUpdate` 返回更新是否成功，PID 用它将内部滤波算术失败传播为无效控制步；原 float 更新入口仍保持上次输出。

## 共享 PID

`component/controller/pid_core.h` 是唯一控制律，C11 可直接使用 `MrPidConfig/MrPid` 和 `MrPid_Configure/Update`。C++ `controller/pid.hpp` 保留 `pid_config`、Build、Update、Step、Reset 等入口，只进行表示转换；不再包含第二套积分/微分/限幅实现。两者都是头文件实现。

```cpp
mr::comp::cntlr::pid_config config{};
config.gains = {1.0f, 2.0f, 0.1f, 0.01f};
config.output_limit = mr::comp::cntlr::SymmetricLimit::Abs(5.0f);
config.integral_output_limit = mr::comp::cntlr::SymmetricLimit::Abs(0.5f);
config.control_freq_hz = 1000.0f;
auto controller = mr::comp::cntlr::pid::Build(config);
auto step = controller.Step(target, feedback, actual_dt_s);
if (step.valid) { /* 应用按自己的安全策略处理 step.output */ }
```

P 为 `k × error × p`，积分状态累积 `k × error × dt`，I 为状态乘以 i；I 限幅明确使用输出单位。增益允许正、负和零，小于 1e-6 的有效积分增益不会被静默禁用。输出限幅、积分输出限幅、输出变化率限幅与误差周期须非负，0 表示关闭；负值或非有限值属于配置错误。频率须在 1 Hz 到 1 MHz，滤波截止频率须满足滤波核心的约束，即使当前不使用 D 也检查配置。

默认微分针对 `k × feedback`，输出 D 为其时间导数乘以 `-d`，目标突变不直接产生微分冲击。Configure/Reset 后第一帧建立测量和滤波基准，D 为 0。None 模式的 D 为 0；External 模式的输入就是 `d(k × feedback)/dt`，不再乘一次 k，首帧即生效。接口不自动推断速度/角速度的尺度。

默认 anti-windup 同时考虑输出绝对值和变化率边界。积分可以恰好到达边界，禁止继续朝饱和方向积累，允许向退出饱和的方向变化。输出使用实际提交的积分，不使用被拒绝的候选值。`hold_integral_on_saturation=false` 关闭这项限制，但积分输出自身仍受配置上限约束。`saturated` 表示本步候选输出受到绝对值或变化率限制；仅到达积分自身上限不设置该标记。

无 dt 的 C++ 重载按配置频率计算；显式 dt 使用实际秒数，必须在 1 微秒至 1 秒。非法 dt 不再替换为默认周期或静默裁剪。非法输入、越界算术或微分滤波失败时，Step.valid 为 false，输出保留，积分/滤波/微分历史不推进。遇到丢弃的周期，应用应按恢复策略决定复位或提供距上次成功更新的间隔。Configure 失败则清除旧历史和输出并令 ready 为 false，不能继续用旧参数运行；这与单个错误 Step 的保持行为不同。

Reset(output) 清除积分和测量历史，output 只作为输出/斜率基准并受绝对值限幅；ResetIntegral(value) 的 value 是积分输出单位，下一次更新时生效，不回写上一次 Step 的诊断数据。`integral()` 返回原始积分状态，`integral_output()` 返回其输出贡献。直接 C 核心对象需先 Configure，再 Reset/Update；配置和状态由对象持有，不得直接修改内部字段。

### 旧参数接入

旧 `component/pid.h` 的 `KPID_Params_t` 字段顺序保持，`PID_Init/Calc/Reset` 由 `pid.c` 转发到共享核心。旧 `i_limit` 仍是积分状态限幅，唯一换算函数 `PID_ConfigFromLegacy` 将其转为 `abs(i) × i_limit`；0 表示不限幅。C++ 设备通过可选 `controller/pid_legacy.hpp` 的 TryLegacyPidConfig 使用同一换算，不再自行复制校验和换算公式。

`KPID_t` 布局已变化，旧 param、dt_min、dfilter 字段已移除；i 和 last 仅为只读诊断镜像。参数现在按值保存，修改原参数表后需要重新 Configure/Init。所有调用者必须重新编译；直接访问旧字段或依赖隐式实时调参的外部工程需迁移。

新控制律修正了旧实现的积分候选泄漏、积分限幅无法到达/释放、0 限幅阻止积分积累等问题，也改变了首帧 D、显式 dt 与半周期误差行为。环绕误差使用共享标量规则，正负半周期保留原符号。旧参数可以转换，但不保证旧闭环逐位一致，必须按实际执行器和机械结构重新验证控制响应。电机控制包已使用统一适配和 ready 判断；包本身的硬件依赖、输出链和故障安全尚未完成审查修复。

## 时钟与周期调用

`component/timer.hpp` 不再提供隐式 BSP 时钟。项目在配置中填写 `uint64_t (*)()` 回调，返回单调微秒；无时钟时 `ready()==false`、更新返回 0，周期判断为 false。构造对象不会读取时钟。

`MotorControllerConfig.clock_us` 将同类回调传给电机控制器，用于估算相邻控制步的实际 dt。留空时控制器按 `sample_freq` 使用固定周期；需要变周期 PID/滤波时，board adapter 应显式填入 BSP 的单调微秒函数。控制器源文件不再直接绑定 BSP 时钟。配置应使用带有反馈滤波、输出滤波和可选时钟的明确构造方式或成员初始化；旧的含义不清、会静默忽略参数的兼容构造已移除。软限位学习包目前仍直接依赖 BSP 时间。

```cpp
mr::comp::timer_config config{};
config.clock_us = BoardMonotonicUs;
config.first_dt_us = 1000;
auto timer = mr::comp::timer::Build(config);
float dt_s = timer.Update();
```

编译 `component/timer.cpp`。电机控制器时钟由 `MotorControllerConfig.clock_us` 注入；软限位学习包仍直接使用 `BSP_TIME_Get_us`，尚未完成平台隔离。

`component/time/periodic.hpp` 的 `Periodic` 接收调用方提供的 uint32 时间。单位自选且一致，周期在 1 到 INT32_MAX 之间；`Configure(period, now, due_now)` 决定是否立即到期。延迟调用只触发一次并跳过错过的周期，保持原相位；应用至少在半个 uint32 时间范围内调用一次，支持正常计数回绕。

## IMU、姿态估计与校准

`device/imu/imu_sample.h` 定义一帧完整的 `MrImuSample`：加速度 m/s²、角速度 rad/s、单调微秒时间戳、每帧变化的 sequence 和有效标记。板级适配先统一机体坐标，遵循 AHRS 静止时加速度 +Z 的输入约定。当前 BMI088 驱动尚未直接输出这一快照，需要项目适配。

`module/attitude/attitude.hpp` 的 `Estimator` 使用 C AHRS 算法，需编译 `component/ahrs.c`。先 Configure，再周期调用 `Step(sample, now_us, stationary)`；时间戳与 now 必须使用同一单调时钟。输出欧拉角为 rad，四元数以 q0 为标量部。

首次采样使用配置频率，随后按相邻被消费帧的实际时间间隔更新。重复 sequence、非递增时间戳返回 NoNewSample；未来/过期帧或超大间隔返回 StaleSample；非法数据返回 InvalidSample。超大间隔帧用于重新建立时间基准，下一帧可以恢复。sequence 允许自然回绕，但时间戳不应倒退。

`feedback().valid` 只表示最近一次 Step 产生了有效姿态。无新帧、校准和错误调用都会将它置为 false；已有数值可供诊断，应用不能据此继续当作新反馈使用。当前为 IMU-only 估计，不提供磁力计航向约束，yaw 会漂移。

调用 `StartCalibration()` 后，必须收到配置数量的连续静止、有效、未过期新帧；运动或错误帧打断连续窗口。`stationary` 由项目判断，不绑定某个轮速阈值。独立 `GyroBiasCalibration` 接收的是未经旧偏置扣除的原始角速度，因此重新校准不会重复扣除。完成的这一帧仍返回 Calibrating，更新偏置并将姿态重置为单位四元数，下一新帧才产生姿态输出；项目应决定何时允许该重置。可显式 CancelCalibration。

## 告警输出

`module/audio/alarm.hpp` 的 `Alarm` 仅计算告警等级、频率和占空比，不播放曲库或写 PWM。先 Configure，再调用 `Step(request, now_ms)`；时间单位为毫秒，所有时间差遵守小于半个 uint32 范围的约定。

请求过期后按启动时确定的最短时长保持，再关闭。低优先级请求不能提前打断保持期，高优先级可以升级；等级切换重新建立音调相位和保持期。同等级请求只刷新有效性，不重复延长最短保持。音调按实际经过时间交替，延迟调用不会逐个补播。项目负责音乐、校准提示与告警间的仲裁，并把输出送给 buzzer。

旧任务中的按钮、遥控拨杆、固定标定值、加热 PID、整机线程表与全局变量已移除。`User/task/README.md` 说明项目边界；音乐播放独立为 `module/audio/player.h` 与 `module/cloudmusic`，由 app 决定其与告警的输出优先级。

## 轮子反馈与底盘时效

`MotorWheel<Drive>` 绑定应用持有的电机和半径，不创建工厂对象或轮速 PID。电机输出轴角速度乘轮半径成为轮缘线速度，保留原 CAN 接收时间戳和序号；底盘独占轮速闭环。Drive 的 Update 必须只刷新反馈，不可用会计算旧目标的 controller.Update 冒充。

`ChassisConfig.feedback_timeout_ms` 默认 20 ms。项目使用与总线接收时间戳同源的单调毫秒时钟，执行 `UpdateFeedback(now_ms) → SetCommand → Step → CommitOutput`；反馈失效或计算失败后模块尝试 Relax。无参 UpdateFeedback 只保留兼容入口，不提供模块层帧龄检查。

## CAN 端口与电机协议

所有电机显式配置 `CanPort*`，可选 `bsp/adapters/can_port.hpp` 接入现有 FDCAN；公共电机头及协议不含 HAL/RTOS。每条物理总线只能持有一个端口实例。实例通过固定容量端点登记拒绝重复反馈地址、冲突命令地址或重复 LZ 目标，容量不足明确返回错误。登记和周期调度须在单个上下文串行执行。

RM/DM/LZ 将原接收时间戳和序号写入 `MotorState`。重复 latest 或队列暂空时，缓存持续有效到配置帧龄；轮询不刷新时间。错误帧、驱动错误、过期帧立即失效；`MotorT::Update` 失败会清除待发控制，设备故障也作为失败返回。温度限制触发后拒绝本次操作并按配置 Relax，不会将“放松成功”误报成“控制成功”；默认阈值 70/80°C，显式非正阈值关闭对应检查。

DM/LZ 协议按电机输出轴的 rad、rad/s、Nm 通信，只换算机构附加传动比，MIT 增益除以外部传动比平方。DM 的 MIT 量程由实例配置，必须匹配固件；LZ 当前只支持 Motion，SetPosition 的非零速度上限返回不支持，不能当作目标速度。完整配置、停机与迁移约定见 `User/device/motor/README.md`。

## RM 电机周期输出

项目为每条物理端口持有一个 `RmCommandBus`：`BeginCycle → 各电机 CommitCommand → Flush`。Commit 仅暂存，Flush 每个使用的组发送一次，未提交槽位置零。失败组可重试，已成功组不重发；开始下一周期会丢弃旧失败组命令。Relax/Disable 同样需要最终 Flush。发送成功只代表底层接受请求，不代表硬件已执行。

错误出口也必须 Flush；`examples/device_execution/app/rm_chassis_cycle.hpp` 展示完整调度及发送失败后的停机策略，已用真实模块/轮子/电机协议主机集成测试验证。板级 CAN 初始化、线程安排、实际时序与实机停机仍需项目验收。

## 关节命令缓存

`JointControlParams.kp/kd` 是显式 MIT 增益：零表示零增益，负数或非有限值会拒绝本次位置控制，不再根据 joint id 猜测默认值。`SetOffset`、`SetTargetAngle` 和 `SetFeedforwardTorque` 遇到非有限值时保留原值。`Update()` 只有在 actuator 返回成功、反馈在线且数据及映射结果有限时才更新快照；失败会标记离线并保留最后一组数值。位置 MIT 控制要求最近一次反馈有效，避免离线时重复消费旧状态；力矩控制保持显式开环入口。位置/力矩命令只有在 actuator 接受后才更新 `JointT` 的命令状态缓存；随后由同一控制上下文调用 `CommitControl()` 提交。当前 `PositionControl(target, dt)` 为原生 MIT 准备目标，`dt` 参数保留兼容但不参与此执行路径。

关节接口和状态定义位于 `device/joint/joint_interface.hpp`，通用映射/控制实现位于 `joint.hpp`，不依赖 motor 或 BSP。具体电机适配器及 `DmActuator`/`LzActuator` 等别名放在可选 `motor_actuator_adapter.hpp`；使用这些具体适配器的项目需显式包含此头文件。

耦合传动另用 `JointGroup<N>` 统一采样和向量力矩分配；独立关节支持位置、速度、力矩及 MIT 增益的完整传动换算。接口及能力边界见 `User/device/joint/README.md`。

## 通用机械臂模块装配边界

`module/arm/arm.hpp` 提供 `mr::module::arm::Arm<N>`，通过 `Config` 复制 robotics 模型、规划参数和关节增益，绑定 app 持有的完整 `IActuator` 数组。初始化只注册；有效反馈后显式 Enable。每周期依次 UpdateFeedback、SetCommand、Step、CommitOutput；错误锁存为 Fault 并清除待发控制，修复后重新采样并显式 Enable。详细生命周期、坐标单位和迁移说明见 `User/module/arm/README.md`。

旧三轴参数和遥控映射已迁到 `examples/three_axis_arm/app`；公共模块不包含具体电机、工厂后备路径、机器人几何或遥控策略。CMake 目标为 `mr_module_arm`，应用需提供自己的 board、设备、调度和输入超时策略。
## 音乐播放与资源边界

`Audio_Player` 接收音符输出回调和毫秒时间；`CloudMusic` 接收显式播放列表与控制事件。二者不包含 PWM、RTOS、DR16 或固定曲库。蜂鸣器设备仅保留硬件输出；原设备内播放状态机已删除。曲谱位于 `resources/music` 并按单曲编译，遥控手势和输出接入位于 `examples/music/app`。CMake 可分别链接 `mr_module_audio`、`mr_module_cloudmusic`；不会传递曲库。生命周期、错误、暂停和时间约定见 `User/module/cloudmusic/README.md`。

## 矩阵、估计器与轨迹

`matrix::solve(A,b,&x)` 直接执行带部分主元的消元，不先求逆；失败输出清零。
`inverse` 与 `inv(A,&out)` 返回必须检查的 bool，旧 `inv(A)` 已移除。
主元阈值为固定绝对正阈值，输入量级差异很大时应先无量纲化/缩放。
KF/EKF 要求 Q/R/P0 为有限、对称半正定矩阵，创新协方差仍须可解。
Predict/Update/Step 失败保留最后完整数值快照并令 valid=false，Reset 后恢复。
EKF 外部 callback context 的副作用不能回滚；状态维度较大时评估栈副本预算。

TrapezoidProfile 与 SCurveProfile 提供 Config、Configure、MinDuration、
ConfigureForDuration、Sample；时间单位秒，速度/加速度/jerk 使用与 distance 对应的单位。
二者为静止到静止轨迹，延长时长通过时间缩放保证上限。SynchronizedProfile
以最慢轴时长配置全部轴，Axis 返回可空指针，失败配置不可采样旧数据。
三次 Hermite 仅保证端点零速度，区间内端点加速度一般不为零。
在线梯形允许目标突变后的必要过冲，每步速度变化受 a*dt 约束；仅在位置容差
内且当前速度可合法单步制停时吸附目标。配置或 dt 非法不推进速度。旧 velocity_epsilon_floor 已移除，制停阈值由 a*dt 决定。

## 串级控制、命令中心与存储

旧 mpc.hpp/mpc 已更名 cascade.hpp/cascade：位置 P → 限速 → 速度参考斜率 →
速度 P → 输出限幅，没有预测优化器。限幅 0 表示关闭，参数非负且有限，dt 为正秒数；
StepOnce.valid=false 时输出为零、内部速度参考不推进。LQR 接收外部离线增益，
使用前检查 Step.valid；不执行在线 Riccati 求解。

Center 由单线程持有，禁止复制/移动，析构所有内嵌对象；外部 sink 指针、名称与
资源须活过 Center。input 类型唯一；output 返回可检查 OutputRef，容量错误后锁存
安全输出。普通更新失败/offline 的 input 不参与路由，publish 后检查 last_publish_ok。
Freshest 要求所有路由输入提供同一 uint32 毫秒时钟的 timestamp_ms()，按最旧输入帧龄
比较，时间间隔小于半个回绕周期；它不提供自动超时，过期规则放 input.online/condition。
CMSIS sink 默认不 reset；显式 reset 只用于独占生产者任务，清空整个队列。

C 环形队列保留 Cortex-M 单核 SPSC 同步：Put 只有一个生产者，Get/Peek/Drop 只有
一个消费者；Init/Reset 需要停用两端。LatestSlot 在 PRIMASK 临界区复制，载荷越大
中断延迟越长。两者不承诺桌面线程、多核、NMI 或 DMA 缓存一致性。C 句柄和 storage
不能按值复制，C++ 包装禁止复制/移动；外部 storage 须覆盖全部访问生命周期。

## 几何与机械臂

旋转 Try* API 返回 bool，失败清零输出；值返回便利函数遇到非法输入返回 NaN。
IsRotationValid/IsTransformValid 检查正交、det≈+1、齐次末行与有限性，不自动修复。
零轴只在角度为零时有效；四元数零范数无效，非零四元数会归一化。
PoseCoordinatesFromTransform/TransformFromPoseCoordinates 表示平移+旋转向量；
旧 Twist 名称保留 deprecated 包装，不是 SE(3) exp/log。
机械臂的坐标、单位、流式加速度、显式 IK 降级及正常/故障过渡边界见
[arm 接口说明](../User/robotics/arm/README.md)。


## MRLink 与 DMA 平台边界

协议与固定容量 C++ 类型接口统一在 component/communication/mrlink；HAL 平台通道在 bsp/mrlink，
bsp/adapters/mrlink.hpp 负责显式连接。mr_component_mrlink 不引入 HAL/RTOS；旧 device 头是兼容入口，
旧实现源已删除。所有 payload 使用显式 MessageCodec，旧 ABI 数据须 opt-in LegacyNativeCodec。

Init 后安装同步 hooks 再启动 ISR 输入；单任务消费，回调中的配置/重置返回 BUSY。
通道成功发送先复制数据，释放必须显式 Deinit 并检查结果。UART 回调占用失败不覆盖其他设备。
DMA 必须先配置 BSP_DMA_Configure 的 board hook；没有 hook 会拒绝启动。BMI088 和 DR16
采用对齐独占缓冲，board 仍须确认实际 RAM 与 Cache/MPU 策略。完整契约、迁移和源码列表见
[MRLink 接入说明](../User/device/mrlink/docs/README.md)，显式装配见 [例子](../examples/mrlink/README.md)。
