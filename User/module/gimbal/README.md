# Gimbal module

`mr::module::gimbal::Gimbal` has exactly two compile-time axes in `[yaw, pitch]` order. The project owns and constructs both `IActuator` objects; the module does not choose motor models, initialize a bus, allocate memory, or create tasks.

```cpp
using Gimbal = mr::module::gimbal::Gimbal;
Gimbal gimbal;
Gimbal::Config config;
config.kinematics.model = mr::robotics::gimbal::Model::kYawPitch;
config.axes = {&yaw_actuator, &pitch_actuator};
config.gains = {{{20.0f, 1.0f}, {16.0f, 0.8f}}}; // kp, kd
config.kinematics.yaw_pitch.min_yaw_rad = -3.0f;
config.kinematics.yaw_pitch.max_yaw_rad =  3.0f;
config.kinematics.yaw_pitch.min_pitch_rad = -0.4f;
config.kinematics.yaw_pitch.max_pitch_rad =  0.8f;

gimbal.Register(config);       // validates and registers; outputs remain relaxed
gimbal.UpdateFeedback();       // required before enabling
gimbal.Enable();               // explicitly enables both axes and holds feedback
gimbal.SetTargetDirection({1.0f, 0.2f, 0.3f});
gimbal.UpdateFeedback();       // each control cycle consumes a fresh sample
gimbal.Step(0.002f);
gimbal.CommitOutput();         // commits both staged actuator commands
```

The application must check every result. `UpdateFeedback -> SetTarget... -> Step -> CommitOutput` is one cycle. After commit, another feedback update is required. Device code remains responsible for actual frame freshness and bus delivery. Input limits, actuator-side position limits and mechanical hard stops must agree; the module does not calibrate zero or infer motor direction. Any device failure clears pending commands and attempts to relax registered axes. Partial bus transmission cannot be rolled back. A fault latches the module; call `Disable()`, then sample valid feedback and explicitly `Enable()` to recover. If the control task faults, the application should explicitly call `Disable()` or provide its own shutdown policy.

`Step` sends position-mode MIT commands with configured gains and zero feedforward torque; it is not a PID controller and does not perform visual tracking. The caller supplies already filtered targets and the fixed sample interval.
