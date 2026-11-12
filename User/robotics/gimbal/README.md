# Gimbal kinematics

`GimbalKinematics` supports yaw-pitch and yaw-pit-pit models. Both use a base frame of +X forward, +Y left, +Z up; positive yaw turns toward +Y and positive pitch raises the viewing direction. The yaw-pit-pit model assumes parallel, intersecting pitch axes whose positive angles add to world elevation. Axis offsets, camera extrinsics, backlash and gravity compensation belong to a future model or the application.

```cpp
mr::robotics::gimbal::GimbalKinematics model;
mr::robotics::gimbal::KinematicsConfig config;
if (!model.Configure(config)) { /* invalid model */ }

mr::robotics::gimbal::JointAngles target{};
auto result = model.DirectionToAngles({1.0f, 1.0f, 0.5f}, 0.0f, target);
```

For yaw-pit-pit, direction has infinitely many pitch1/pitch2 solutions. Supply the chosen pitch1 angle; the model solves pitch2:

```cpp
config.model = mr::robotics::gimbal::Model::kYawPitchPitch;
model.Configure(config);
mr::robotics::gimbal::JointAngles joints{};
model.DirectionToAngles(direction, pit1_target_rad, joints);
```

Direction input need not be normalized. A zero vector is invalid. A direction at either vertical pole has undefined yaw and returns `kSingularity`. Joint limits are specified in radians in the selected model config; conversion reports `kLimitViolation` instead of silently clamping.
