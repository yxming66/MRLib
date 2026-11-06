#include "transform.hpp"

#include <limits>

namespace mr::robotics::core {

Transform MakeTransform(const Rotation& rotation, const Vec3& translation) {
  const float data[16] = {
      rotation[0][0], rotation[0][1], rotation[0][2], translation[0][0],
      rotation[1][0], rotation[1][1], rotation[1][2], translation[1][0],
      rotation[2][0], rotation[2][1], rotation[2][2], translation[2][0],
      0.0f,           0.0f,           0.0f,           1.0f,
  };
  return Transform(data);
}

Transform TransformFromTranslation(const Vec3& translation) {
  return MakeTransform(::mr::component::math::matrix::identity_rect<3, 3>(), translation);
}

Transform TransformFromAxisAngle(const Vec3& axis, float angle) {
  return MakeTransform(RotationFromAxisAngle(axis, angle), matrix::zeros<3, 1>());
}

Rotation RotationOf(const Transform& transform) { return transform.block<3, 3>(0, 0); }

Vec3 TranslationOf(const Transform& transform) { return transform.block<3, 1>(0, 3); }

Transform InverseTransform(const Transform& transform) {
  if (!IsTransformValid(transform)) {
    Transform out;
    out.fill(std::numeric_limits<float>::quiet_NaN());
    return out;
  }
  const Rotation inverse_rotation = RotationOf(transform).transpose();
  return MakeTransform(inverse_rotation, -1.0f * inverse_rotation * TranslationOf(transform));
}

PoseCoordinates6 PoseCoordinatesFromTransform(const Transform& transform) {
  if (!IsTransformValid(transform)) {
    PoseCoordinates6 out;
    out.fill(std::numeric_limits<float>::quiet_NaN());
    return out;
  }
  const Vec3 translation = TranslationOf(transform);
  const Vec4 axis_angle = AxisAngleFromRotation(RotationOf(transform));
  const Vec3 angular = axis_angle.block<3, 1>(0, 0) * axis_angle[3][0];
  const float data[6] = {
      translation[0][0], translation[1][0], translation[2][0],
      angular[0][0],     angular[1][0],     angular[2][0],
  };
  return Twist6(data);
}

Transform TransformFromPoseCoordinates(const Twist6& twist) {
  const Vec3 translation = twist.block<3, 1>(0, 0);
  const Vec3 angular = twist.block<3, 1>(3, 0);
  const float angle = static_cast<float>(
      std::hypot(std::hypot(static_cast<double>(angular(0, 0)), angular(1, 0)), angular(2, 0)));
  return MakeTransform(RotationFromAxisAngle(angular, angle), translation);
}

bool IsTransformValid(const Transform& t, float tolerance) {
  if (!matrix::is_finite(t) || !IsRotationValid(RotationOf(t), tolerance)) return false;
  return std::fabs(t(3, 0)) <= tolerance && std::fabs(t(3, 1)) <= tolerance &&
         std::fabs(t(3, 2)) <= tolerance && std::fabs(t(3, 3) - 1) <= tolerance;
}
bool TryMakeTransform(const Rotation& r, const Vec3& v, Transform* out) {
  if (!out) return false;
  *out = {};
  if (!IsRotationValid(r) || !matrix::is_finite(v)) return false;
  *out = MakeTransform(r, v);
  return true;
}
bool TryInverseTransform(const Transform& input, Transform* out) {
  const Transform t = input;
  if (!out) return false;
  *out = {};
  if (!IsTransformValid(t)) return false;
  const Transform candidate = InverseTransform(t);
  if (!matrix::is_finite(candidate)) return false;
  *out = candidate;
  return true;
}
bool TryPoseCoordinatesFromTransform(const Transform& t, PoseCoordinates6* out) {
  if (!out) return false;
  *out = {};
  if (!IsTransformValid(t)) return false;
  *out = PoseCoordinatesFromTransform(t);
  return true;
}
bool TryTransformFromPoseCoordinates(const PoseCoordinates6& coordinates, Transform* out) {
  if (!out) return false;
  *out = {};
  if (!matrix::is_finite(coordinates)) return false;
  const Transform candidate = TransformFromPoseCoordinates(coordinates);
  if (!IsTransformValid(candidate)) return false;
  *out = candidate;
  return true;
}

}  // namespace mr::robotics::core
