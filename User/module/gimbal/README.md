# Unified gimbal module

`mr::module::gimbal::Gimbal` is one board-neutral module for the supported gimbal models. It has fixed storage for at most three axes; registering `kYawPitch` activates two and `kYawPitchPitch` activates three. The project owns the `IActuator` objects and the module does not allocate memory, create a task, choose motor models, or initialize a bus.

```cpp
using Gimbal = mr::module::gimbal::Gimbal;
Gimbal gimbal;
Gimbal::Config config;
config.kinematics.model = mr::robotics::gimbal::Model::kYawPitchPitch;
config.axes = {&yaw_actuator, &pit1_actuator, &pit2_actuator};
config.gains = {{{20.0f, 1.0f}, {16.0f, 0.8f}, {12.0f, 0.6f}}};
config.kinematics.yaw_pitch_pitch.min_pitch1_rad = 0.0f;
config.kinematics.yaw_pitch_pitch.max_pitch1_rad = 1.5707963f;
config.kinematics.yaw_pitch_pitch.min_pitch2_rad = 0.0f;
config.kinematics.yaw_pitch_pitch.max_pitch2_rad = 1.5707963f;

gimbal.Register(config);     // validates, registers, and leaves outputs relaxed
gimbal.UpdateFeedback();     // encoder/device feedback
gimbal.Enable();
gimbal.SetTargetAngles({yaw_rad, pit1_rad, pit2_rad});
gimbal.Step(0.002f);
gimbal.CommitOutput();
```

The axis arrays always use `[yaw, pitch1, pitch2]` order. For ordinary yaw-pitch, set axis 2 to `nullptr`. `SetTargetDirection(direction, pitch1)` resolves the redundant pitch angle using the supplied pitch1 target. Check every return value. A successful output commit consumes that feedback sample; update again before the next step.

The module reads actuator feedback for safe enable/hold and sends position-mode MIT targets. It does not decide whether an application should use an IMU for world heading or encoders for fold state. Keep IMU selection, frame conversion, command freshness, and fold state transitions in the project app; the actuator device still needs its encoder for its inner control and diagnostics. Device failures clear staged commands and attempt to relax registered axes. Bus transmission is not atomic.
