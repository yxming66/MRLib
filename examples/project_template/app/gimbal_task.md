# Yaw-pit gimbal task example

`gimbal_task.cpp` is a basic two-axis yaw-pit application example. It uses
`YawPitchGimbal` directly and `PeriodicTask`; it
contains no mechanism-specific folding state machine. A project with an extra
mechanism such as folding can implement that sequence in its own application
code.

The board project owns the yaw and pitch actuators, their CAN IDs, zero offsets, gains, and safety limits. A typical setup in its running `Task_Init` looks like this:

```cpp
static mr::module::gimbal::YawPitchGimbal gimbal;
static app::GimbalTask gimbal_task;

app::GimbalTaskConfig config{};
config.module.axes = {&yaw_actuator, &pitch_actuator};
config.module.gains = {{{yaw_kp, yaw_kd}, {pitch_kp, pitch_kd}}};
config.module.limits[0] = {-mr::component::math::kPi, mr::component::math::kPi};
config.module.limits[1] = {-mr::component::math::kHalfPi, mr::component::math::kHalfPi};
config.period_us = 2000;
config.yaw_zero_offset_rad = 0.0f;
config.pitch_rate_rad_s = 0.8f;

const app::GimbalTaskIo io{
    &board_context, BoardNowMs, ReadLatestGimbalCommand,
    ReadLatestChassisImuYaw, PublishGimbalStatus};
if (!gimbal_task.Start(gimbal, io, config)) {
  // Record task creation failure; the actuators remain disabled.
}
```

`GimbalTask::Start` creates the static `PeriodicTask`. The task registers the actuators, reads encoder feedback, validates fresh command and IMU snapshots, then enables the module. Each cycle computes the joint yaw target as:

```text
joint_yaw = target_world_yaw - base_world_yaw - yaw_zero_offset
```

The pitch target follows the command with a rate limit. The `YawPitchGimbal::Update` method reads encoder feedback, computes the mode output, commits actuator commands, and uses encoder feedback for actuator control and diagnostics. The sample assumes the IMU is fixed to the chassis and reports chassis world yaw; adapt the conversion if the IMU is mounted elsewhere.

All angles are radians. The example does not select actuator models, configure board communication, perform homing, or define mechanism-specific sequences. Check zero directions, hard stops, gains, and command freshness for the actual mechanism before enabling motors.
