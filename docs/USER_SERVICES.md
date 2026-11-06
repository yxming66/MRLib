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

`MotorControllerConfig.clock_us` 将同类回调传给电机控制器，用于估算相邻控制步的实际 dt。留空时控制器按 `sample_freq` 使用固定周期；需要变周期 PID/滤波时，board adapter 应显式填入 BSP 的单调微秒函数。控制器源文件不再直接绑定 BSP 时钟。旧的五参数和七参数配置构造仍可编译，但不会隐式读取硬件时钟；使用者应检查原工程是否依赖实际 dt，并按需设置 `clock_us`。软限位学习包目前仍直接依赖 BSP 时间。

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

旧任务中的按钮、遥控拨杆、固定标定值、加热 PID、整机线程表与全局变量已移除。`User/task/README.md` 说明项目边界；尚未解耦的 `module/cloudmusic` 不属于上述新告警服务。

## 轮子反馈与底盘时效

`device/wheel/wheel_interface.hpp` 的 `WheelFeedback` 同时提供规范化物理量、`timestamp_ms` 和 `sequence`。时间戳由具体 BSP 总线接收路径产生，调用底盘时传入的 `now_ms` 必须和它使用同一个单调毫秒时钟；当前 BSP 的 CAN 兼容接口通过 `BSP_TIME_Get_ms()` 时间基准写入反馈时间戳。序号是总线帧标识，可用于诊断是否重复消费；模块允许一个新总线帧被多个更快的控制调用读取，但不会在时间戳超时后继续接受它。

`module/chassis::ChassisConfig.feedback_timeout_ms` 默认 20 ms，须在 1 到 `INT32_MAX` 之间。控制周期应调用 `UpdateFeedback(now_ms)`，再设置命令、`Step(dt_s)` 和 `CommitOutput()`。反馈超时、离线或含非有限值时模块清除反馈并进入故障态，同时尽力放松已注册设备。保留的无参 `UpdateFeedback()` 不能检查时间戳年龄，仅用于旧调用迁移；它不满足该时效契约。

RM、DM、LZ 电机协议将 CAN 帧时间戳和序号保存到 `MotorState`，读取失败时将 `online` 与 `feedback_valid` 置为 false 并递增 `feedback_lost_count`。本接口链目前覆盖协议数据到轮子快照和底盘 host 测试；STM32 协议源文件尚未在本批主机目标中编译，需在真实 HAL 工程完成固件及硬件回归。

## DM/LZ 电机命令

DM/LZ 的 `SetTorque`、`SetVelocity`、`SetPosition` 和 `SetMIT` 只更新该电机实例的待发命令；由同一控制上下文随后调用 `CommitCommand()`。输入中的 NaN/无穷值会被拒绝，且不会覆盖此前有效的 pending 命令。LZ 的 MIT 增益还要求 `Kp >= 0`、`Kd >= 0`；DM 的协议编码会把增益和物理命令映射到协议量程。

提交仅在 BSP 的 CAN 发送接口返回成功后清除 pending 命令并置 `last_commit_ok`；失败时保留命令供显式重试。这里的“成功”表示 BSP 接受了发送请求，不表示帧已在总线上完成发送或电机已经执行。DM 使用 latest-value 反馈槽，LZ 使用队列；调用 `Update()` 时没有新帧会将 `online` 和 `feedback_valid` 清零并增加 `feedback_lost_count`。协议层不会依据帧年龄自动判定过期，因此需要严格时效保护的项目应在应用/模块层使用同源单调时钟检查 `last_feedback_tick`。

Enable、Disable 和 Relax 的协议状态只在对应 CAN 发送接口成功后更新。Disable 在发送前会清除 pending 控制命令；如果发送失败，旧命令不会因此重新提交，调用方应显式重试 Disable。Fake-CAN 主机回归验证了输入拒绝、pending 保持和发送失败状态，但尚未验证设备固件响应、总线实际时序或硬件停机行为。

## RM 电机输出

`device/motor/protocol/rm_command_group.hpp` 集中定义 RM/GM6020 命令帧与 4 槽映射，并保存每条帧组最近一次成功提交的原始电流值。发送尝试使用局部候选快照；CAN 发送失败时不更新已提交快照，且设备保留当前 pending 命令供显式重试。`MotorProtocol::Relax/Disable` 将当前电流目标设为零并立即提交；只有 CAN 层接受该帧后协议状态才变为 Relaxed。主机 fake-CAN 回归会编译真实 RM protocol 与 CAN transport，并验证 Relax 失败后的重试状态。

RM 命令 bank 是静态的、按 CAN bus 共享，当前没有锁或跨线程仲裁；同一 bus 的所有 RM 电机命令必须由一个控制上下文串行更新/提交。各槽的电流会一起重发，因此这个接口仍不是多电机原子 flush，也没有监控实际 CAN 总线是否成功发出或电机是否执行。总线发送错误、具体电机对零电流的响应、ID 分组和混合型号部署都需在 STM32 目标板验证。

## 关节命令缓存

`JointControlParams.kp/kd` 是显式 MIT 增益：零表示零增益，负数或非有限值会拒绝本次位置控制，不再根据 joint id 猜测默认值。`SetOffset`、`SetTargetAngle` 和 `SetFeedforwardTorque` 遇到非有限值时保留原值。`Update()` 只有在 actuator 返回成功、反馈在线且数据及映射结果有限时才更新快照；失败会标记离线并保留最后一组数值。位置 MIT 控制要求最近一次反馈有效，避免离线时重复消费旧状态；力矩控制保持显式开环入口。位置/力矩命令只有在 actuator 接受后才更新 `JointT` 的命令状态缓存；随后由同一控制上下文调用 `CommitControl()` 提交。当前 `PositionControl(target, dt)` 为原生 MIT 准备目标，`dt` 参数保留兼容但不参与此执行路径。

关节接口和状态定义位于 `device/joint/joint_interface.hpp`，通用映射/控制实现位于 `joint.hpp`，不依赖 motor 或 BSP。具体电机适配器及 `DmActuator`/`LzActuator` 等别名放在可选 `motor_actuator_adapter.hpp`；使用这些具体适配器的项目需显式包含此头文件。

## 三轴机械臂模块装配边界

`module/arm/Runtime` 仍是三轴项目模块，但不再从库内全局配置读取参数，也不在初始化时隐式调用 `BSP_CAN_Init()`。应用应构造 `Runtime::Config`，填入 `Arm_Params_t` 指针；这份参数对象必须比 Runtime 活得更久。然后调用 `runtime.Init(config, control_freq_hz)`；CAN 初始化由 board/app 负责。Runtime 是普通的显式实例，不再提供隐藏默认实例或 CMSIS 队列入口。项目任务自行读取队列并调用 `runtime.SetCommand(command)`，随后执行 `Update()`、`Control(dt_s)` 和 `Commit()`。
