# GimbalTask app integration example

`gimbal_task.cpp` is project application code, not part of MRLib's module target. Copy it into the gimbal board app and implement `GimbalTaskIo` with that board's command channel, IMU task snapshot, board clock and status channel. Construct all three actuators and the `Gimbal` object with static lifetime, then call `GimbalTask::Start` from the board's `Task_Init` after the CAN endpoints and app channels are ready. The FreeRTOS scheduler must already be running.

The board owns actuator construction and hardware parameters. A typical setup in its task initialization code looks like this:

```cpp
static mr::module::gimbal::Gimbal gimbal;
static app::GimbalTask gimbal_task;

app::GimbalTaskConfig config{};
config.module.kinematics.model = mr::robotics::gimbal::Model::kYawPitchPitch;
config.module.axes = {&yaw_actuator, &pit1_actuator, &pit2_actuator};
config.module.gains = {{{yaw_kp, yaw_kd}, {pit1_kp, pit1_kd}, {pit2_kp, pit2_kd}}};
config.module.kinematics.yaw_pitch_pitch.min_pitch1_rad = 0.0f;
config.module.kinematics.yaw_pitch_pitch.max_pitch1_rad = mr::component::math::kHalfPi;
config.module.kinematics.yaw_pitch_pitch.min_pitch2_rad = 0.0f;
config.module.kinematics.yaw_pitch_pitch.max_pitch2_rad = mr::component::math::kHalfPi;
config.period_us = 2000;
config.normal_pit1_angle_rad = mr::component::math::kHalfPi;

const app::GimbalTaskIo io{
    &board_context, BoardNowMs, ReadLatestGimbalCommand,
    ReadLatestChassisImuYaw, PublishGimbalStatus};
if (!gimbal_task.Start(gimbal, io, config)) {
  // Record task creation failure and leave the actuators disabled.
}
```

The callback functions adapt project-owned queues and clocks to the task interface. `GimbalTask::Start` creates the static `PeriodicTask`; its `Init` registers actuators, samples encoder feedback and enables the gimbal only after the task starts. Call `Start` once from the running `Task_Init`, after board communication and project channels are initialized.

The sample assumes the IMU is fixed to the chassis and reports chassis world yaw. Normal-mode joint yaw is `target_world_yaw - imu_base_world_yaw - yaw_zero_offset`; the gimbal module still reads each motor encoder for actuator state and safe enable. Fold mode uses encoder feedback and does not require IMU yaw. If the IMU is mounted on the moving gimbal head, change this frame conversion to use head orientation directly.

On startup the task accepts encoder feedback only at the folded or normal pit1 endpoint; an intermediate angle after reset is treated as unknown and the task faults instead of guessing a motion phase. Folding first aligns pit2 with pit1 while pit1 holds, then moves both pitch setpoints together toward zero. Unfolding moves both together toward the normal angle. In normal mode pit1 is held at its normal angle, while pit2 follows a rate-limited target. All angles are radians.

The task example assumes its actuator handles were already configured by board code. It does not choose motor models, CAN IDs, zero offsets, hard stops, IMU mounting calibration, or a safe recovery procedure for an interrupted fold. Verify those against the mechanism before enabling outputs.
