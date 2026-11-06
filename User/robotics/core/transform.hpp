#ifndef MR_ROBOTICS_CORE_TRANSFORM_HPP
#define MR_ROBOTICS_CORE_TRANSFORM_HPP

// 齐次变换与 Twist 类型。
// 使用方式：Transform T = matrix::identity<4>();
// 先用 rotation/translation 组件构造 T，再通过 T * point 或 T * pose 使用。

#include "rotation.hpp"

namespace mr::robotics::core {

using mr::component::math::Matrix;

using Transform = Matrix<4, 4>;
// Geometric velocity [linear velocity of the chosen origin; angular velocity].
// Both vectors must be expressed in the same explicitly documented frame.
using Twist6 = Matrix<6, 1>;
// Position [x,y,z] + rotation vector [rx,ry,rz]; not an SE(3) logarithm or velocity.
using PoseCoordinates6 = Matrix<6, 1>;
bool IsTransformValid(const Transform& transform, float tolerance = 1.0e-4f);
[[nodiscard]] bool TryMakeTransform(const Rotation&, const Vec3&, Transform*);
[[nodiscard]] bool TryInverseTransform(const Transform&, Transform*);
[[nodiscard]] bool TryPoseCoordinatesFromTransform(const Transform&, PoseCoordinates6*);
[[nodiscard]] bool TryTransformFromPoseCoordinates(const PoseCoordinates6&, Transform*);

Transform MakeTransform(const Rotation& rotation, const Vec3& translation);
Transform TransformFromTranslation(const Vec3& translation);
Transform TransformFromAxisAngle(const Vec3& axis, float angle);

Rotation RotationOf(const Transform& transform);
Vec3 TranslationOf(const Transform& transform);
Transform InverseTransform(const Transform& transform);

PoseCoordinates6 PoseCoordinatesFromTransform(const Transform& transform);
Transform TransformFromPoseCoordinates(const PoseCoordinates6& coordinates);

[[deprecated("Use PoseCoordinatesFromTransform; this is not SE(3) log")]]
inline PoseCoordinates6 TwistFromTransform(const Transform& transform) {
  return PoseCoordinatesFromTransform(transform);
}
[[deprecated("Use TransformFromPoseCoordinates; this is not SE(3) exp")]]
inline Transform TransformFromTwist(const PoseCoordinates6& coordinates) {
  return TransformFromPoseCoordinates(coordinates);
}

}  // namespace mr::robotics::core

#endif
