# Gimbal kinematics

`GimbalKinematics` currently supports a fixed yaw/pitch geometry. The base frame is +X forward, +Y left, +Z up. Positive yaw turns toward +Y and positive pitch raises the viewing direction. The axes are assumed to intersect; offsets, camera extrinsics, backlash and gravity compensation belong to a future model or the application.

```cpp
mr::robotics::gimbal::GimbalKinematics model;
mr::robotics::gimbal::KinematicsConfig config;
if (!model.Configure(config)) { /* invalid model */ }

mr::robotics::gimbal::Angles target{};
auto result = model.DirectionToAngles({1.0f, 1.0f, 0.5f}, target);
```

Direction input need not be normalized. A zero vector is invalid. A direction at either vertical pole has undefined yaw and returns `kSingularity`. Joint limits are specified in radians in `YawPitchGimbal::Config`; conversion reports `kLimitViolation` instead of silently clamping.
