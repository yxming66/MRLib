# 云台模块

直接使用具体类，项目负责电机装配、任务和输入仲裁：

- YawPitchGimbal：两轴，目标 yaw_rad/pitch_rad，提供 kNormal。
- YawPitchPitchGimbal：三轴，提供 kFolded/kLowFire/kHighFire；三轴 Normal 已移除。

三轴模式行为直接写在 yaw_pitch_pitch_gimbal.cpp 内，内部 JointGroup 复用执行器
登记、反馈检查、限位、MIT 输出提交和故障停机，无公共 Controller/Behavior 框架。

## 三轴模式与补偿

折叠时三轴目标均为保存零点 0，头部水平。归一化后的两个 pitch 方向对头部仰角
作用相反：头部仰角 = pitch1 - pitch2。因此保持水平时 pitch2 = pitch1。
这个关系是相对底座的关节运动学，不使用空间方向目标或 IMU。

- Folded：先同时 yaw 限速回零与头部调平，保持 pitch1；两者稳定达标后，
  pitch1、pitch2 均以 posture_speed_rad_s 各自直接回零，不再约束头部水平；
  最终保持三轴零位，仍然使能。
  对应 FoldYaw → FoldPitch → Folded，统一使用 position_tolerance_rad。
- LowFire：pitch1 过渡到 low_fire_pitch1_rad，到位后固定保持，开放 yaw/pitch2 目标。
- HighFire：pitch1 过渡到 high_fire_pitch1_rad，其余流程相同。

每次使能或改变模式都重新从实测角执行。工作模式执行 Leveling → Moving → Settling →
Working。Leveling 保持 pitch1，pitch2 限速跟随实测 pitch1 以先恢复水平；
Moving 中 pitch1、pitch2 以相同的 posture_speed_rad_s 各自移向姿态角，
不跟随实测 pitch1、不约束头部水平。Settling 保持两轴目标为姿态角，
只要求各轴角度误差连续达标，不另检查速度或头部水平。姿态过渡不设超时停机。
实测角超出软件限位不触发失能；有限的目标角超限时截到边界继续输出。
因此补偿角被截断时可能无法调平，此时保持使能并等待到位条件满足。

pitch2_control_ready() 仅在 Working 为 true。过渡时忽略 pitch2 输入；到位时目标
重置为水平补偿角，项目的摇杆积分也应在 ready=false 时暂停。高低位切换与中途
反向收折都重新调平。Folded 时忽略全部运动输入。Target.pitch1_rad 字段保留用于
完整关节目标结构及限位检查，实际 pitch1 输出始终由所选姿态决定。

## 使用示例

```cpp
#include "module/gimbal/yaw_pitch_pitch_gimbal.hpp"
using Gimbal = mr::module::gimbal::YawPitchPitchGimbal;
Gimbal gimbal;
Gimbal::Config config{};
config.axes = {&yaw, &pitch1, &pitch2};
config.gains = {{{20.0f, 1.0f}, {16.0f, 0.8f}, {12.0f, 0.6f}}};
// 示例行程，必须按实际机械结构标定。高位需要 pitch2 >= 1.05 rad。
config.limits = {{{-3.14159265f, 3.14159265f},
                  {-0.02f, 1.5707963f}, {-0.8f, 1.5707963f}}};
config.low_fire_pitch1_rad = 0.35f;
config.high_fire_pitch1_rad = 1.05f;
// 实际代码检查所有返回值。
gimbal.Register(config);
gimbal.SetMode(Gimbal::Mode::kLowFire);
gimbal.Enable();
// 周期调用；展开完成后再从当前 command()[2] 接入 pitch2 目标。
gimbal.Update(0.002f);
```

Config 默认姿态速度 0.3 rad/s、调平速度 0.6 rad/s、yaw收折速度 0.5 rad/s，
库默认统一角度误差 0.05 rad、连续达标时间 0.1 s，项目可覆盖；无速度阈值。
Register 检查三轴零位、低/高位的 pitch1 和 pitch2 补偿均在各自行程内，
也检查有限的正参数及调平速度不低于姿态速度，拒绝不可达配置。

Enable 读取反馈、使能执行器并捕获当前位置，随后 Update 执行过渡与输出。
SetMode 可在 Ready/Enabled 调用，重复设置同一模式不重启过渡；SetTarget 仅在
Enabled 使用，检查全部关节角是否有限且在行程内。Update 自行读取、暂存并提交
MIT 输出；总线多帧提交不是原子操作。Disable 清理过渡状态并释放执行器。

标定时保存折叠零点，模块不发送 SetZero。没有 IMU 时保持的是相对底座水平，
真实跟踪精度由零点、机构关系和电机控制决定。
