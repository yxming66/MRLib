# 云台模块

直接使用具体类：

- `YawPitchGimbal`：目标为 `yaw_rad / pitch_rad`，当前提供 `kNormal`。
- `YawPitchPitchGimbal`：目标为 `yaw_rad / pitch1_rad / pitch2_rad`，提供
  `kNormal / kLowFire / kHighFire`。

每个类在自己的 `.cpp` 中通过 `switch (mode_)` 分发控制逻辑。新增模式时，
在该机构类中添加枚举值和对应的 Update 方法即可。无需公共虚接口或策略对象。
`detail::JointGroup<N>` 仅复用执行器登记、反馈、限位、输出及故障处理。

```cpp
#include "module/gimbal/yaw_pitch_pitch_gimbal.hpp"
using Gimbal = mr::module::gimbal::YawPitchPitchGimbal;
Gimbal gimbal;
Gimbal::Config config{};
config.axes = {&yaw, &pitch1, &pitch2};
config.gains = {{{20.0f, 1.0f}, {16.0f, 0.8f}, {12.0f, 0.6f}}};
config.limits = {{{-3.14159265f, 3.14159265f},
                  {0.0f, 1.5707963f}, {-0.8f, 0.8f}}};
config.low_fire_pitch1_rad = 0.35f;
config.high_fire_pitch1_rad = 1.05f;

// 下面展示调用顺序，实际调用需要检查每次返回值。
gimbal.Register(config);
gimbal.SetMode(Gimbal::Mode::kNormal);
gimbal.Enable();
gimbal.SetTarget({0.2f, 0.5f, -0.1f});
gimbal.Update(0.002f);
```

所有角度均为弧度。两轴配置只需两个执行器和两组参数，三轴需要三组。
Enable 会读取反馈并把当前关节位置设为初始目标。Update 完成读取反馈、
模式计算、暂存控制和逐轴提交，不需要额外调用 Step 或 CommitOutput。
失败时清除待发命令并尝试释放各轴；总线提交不是原子操作。

三轴普通模式直接使用三轴目标。低位/高位模式分别将 pitch1 固定为配置角度，
yaw 和 pitch2 使用目标值；不会补偿 pitch2 或保持某个空间朝向。
切回普通模式后使用最近一次 SetTarget 的完整目标。
模式切换在下一次 Update 生效，目前没有过渡轨迹或射击就绪判定。
SetTarget 在所有模式下都会检查提交的全部关节角是否有限且在限位内。

Register 拒绝不合法的执行器、增益、限位或射击姿态角。
SetMode 可在登记成功后调用；SetTarget 需在使能后调用。
完整登记后的设备故障可以在成功 Disable 后重试 Enable；登记中途失败的对象
无法通过 Disable 恢复为可用对象。

电机对象、任务调度和输入选择由项目负责。这里不创建任务，不选择遥控器或 NUC。
