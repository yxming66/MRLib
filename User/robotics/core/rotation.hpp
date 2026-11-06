#ifndef MR_ROBOTICS_CORE_ROTATION_HPP
#define MR_ROBOTICS_CORE_ROTATION_HPP

// 旋转、三维向量和姿态基础类型。
// 使用方式：用 matrix::identity<3>() 或本文件提供的旋转函数构造矩阵，
// 再将结果传给 Transform、正运动学或雅可比计算接口。

#include "component/math/matrix.hpp"

namespace mr::robotics::core {

using mr::component::math::Matrix;
namespace matrix = ::mr::component::math::matrix;

using Vec3 = Matrix<3, 1>;
using Vec4 = Matrix<4, 1>;
using Rotation = Matrix<3, 3>;

// RPY vector order is [roll, pitch, yaw]. The composed rotation is the common
// fixed-axis ZYX convention: R = Rz(yaw) * Ry(pitch) * Rx(roll).
// Quaternion vector order is [w, x, y, z]. All angles are in radians.
// Checked boundary APIs: failure returns false and clears output; nullptr is rejected.
// IsRotationValid checks R^T R = I and det(R) = +1. No implicit projection/repair.
bool IsRotationValid(const Rotation& rotation, float tolerance = 1.0e-4f);
[[nodiscard]] bool TryRotationFromRpy(const Vec3& rpy, Rotation* output);
[[nodiscard]] bool TryRpyFromRotation(const Rotation& rotation, Vec3* output);
[[nodiscard]] bool TryRotationFromAxisAngle(const Vec3& axis, float angle, Rotation* output);
[[nodiscard]] bool TryAxisAngleFromRotation(const Rotation& rotation, Vec4* output);
[[nodiscard]] bool TryRotationFromQuaternion(const Vec4& quaternion, Rotation* output);
[[nodiscard]] bool TryQuaternionFromRotation(const Rotation& rotation, Vec4* output);

// Value conveniences propagate invalid inputs as NaN. Use Try APIs at input boundaries.
Rotation RotationFromRpy(const Vec3& rpy);
Vec3 RpyFromRotation(const Rotation& rotation);
Rotation RotationFromAxisAngle(const Vec3& axis, float angle);
Vec4 AxisAngleFromRotation(const Rotation& rotation);
Rotation RotationFromQuaternion(const Vec4& quaternion);
Vec4 QuaternionFromRotation(const Rotation& rotation);

}  // namespace mr::robotics::core
#endif
