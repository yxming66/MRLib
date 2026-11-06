#include "rotation.hpp"

#include <cmath>
#include <limits>

namespace mr::robotics::core {
namespace {
constexpr float kAxisEpsilon = 1.0e-6f;
template <int N>
bool Normalize(const Matrix<N, 1>& input, Matrix<N, 1>* out) {
  if (!matrix::is_finite(input)) return false;
  double norm = 0;
  for (int i = 0; i < N; ++i) norm = std::hypot(norm, static_cast<double>(input(i, 0)));
  if (norm == 0 || !std::isfinite(norm)) return false;
  for (int i = 0; i < N; ++i) (*out)(i, 0) = static_cast<float>(input(i, 0) / norm);
  return true;
}
template <int R, int C>
Matrix<R, C> Invalid() {
  Matrix<R, C> out;
  out.fill(std::numeric_limits<float>::quiet_NaN());
  return out;
}
}  // namespace

bool IsRotationValid(const Rotation& r, float tolerance) {
  if (!matrix::is_finite(r) || !std::isfinite(tolerance) || tolerance <= 0) return false;
  const auto gram = r.transpose() * r;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      if (!std::isfinite(gram(i, j)) || std::fabs(gram(i, j) - (i == j ? 1.0f : 0.0f)) > tolerance)
        return false;
  const float det = r(0, 0) * (r(1, 1) * r(2, 2) - r(1, 2) * r(2, 1)) -
                    r(0, 1) * (r(1, 0) * r(2, 2) - r(1, 2) * r(2, 0)) +
                    r(0, 2) * (r(1, 0) * r(2, 1) - r(1, 1) * r(2, 0));
  return std::isfinite(det) && std::fabs(det - 1) <= tolerance;
}

bool TryRotationFromRpy(const Vec3& input, Rotation* output) {
  const Vec3 rpy = input;
  if (!output) return false;
  *output = {};
  if (!matrix::is_finite(rpy)) return false;
  const float cy = std::cos(rpy(2, 0)), sy = std::sin(rpy(2, 0));
  const float cp = std::cos(rpy(1, 0)), sp = std::sin(rpy(1, 0));
  const float cr = std::cos(rpy(0, 0)), sr = std::sin(rpy(0, 0));
  const float data[] = {cy * cp,
                        cy * sp * sr - sy * cr,
                        cy * sp * cr + sy * sr,
                        sy * cp,
                        sy * sp * sr + cy * cr,
                        sy * sp * cr - cy * sr,
                        -sp,
                        cp * sr,
                        cp * cr};
  *output = Rotation(data);
  return true;
}
bool TryRpyFromRotation(const Rotation& rotation, Vec3* output) {
  if (!output) return false;
  *output = {};
  if (!IsRotationValid(rotation)) return false;
  const float cp = std::hypot(rotation(2, 1), rotation(2, 2));
  (*output)(1, 0) = std::atan2(-rotation(2, 0), cp);
  if (cp <= kAxisEpsilon) {
    (*output)(2, 0) = std::atan2(-rotation(0, 1), rotation(1, 1));
  } else {
    (*output)(0, 0) = std::atan2(rotation(2, 1), rotation(2, 2));
    (*output)(2, 0) = std::atan2(rotation(1, 0), rotation(0, 0));
  }
  return true;
}
bool TryRotationFromAxisAngle(const Vec3& input, float angle, Rotation* output) {
  const Vec3 axis = input;
  if (!output) return false;
  *output = {};
  if (!matrix::is_finite(axis) || !std::isfinite(angle)) return false;
  if (angle == 0) {
    *output = matrix::identity<3>();
    return true;
  }
  Vec3 unit;
  if (!Normalize(axis, &unit)) return false;
  const auto hat = mr::component::math::vector3f::hat(unit);
  *output = matrix::identity<3>() + hat * std::sin(angle) + hat * hat * (1 - std::cos(angle));
  return true;
}
bool TryRotationFromQuaternion(const Vec4& input, Rotation* output) {
  const Vec4 quaternion = input;
  if (!output) return false;
  *output = {};
  Vec4 q;
  if (!Normalize(quaternion, &q)) return false;
  const float w = q(0, 0), x = q(1, 0), y = q(2, 0), z = q(3, 0);
  const float data[] = {1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y),
                        2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
                        2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)};
  *output = Rotation(data);
  return true;
}

static Vec4 QuaternionFromValidRotation(const Rotation& rotation) {
  const float trace = rotation.trace();
  float data[4] = {};
  if (trace > 0.0f) {
    const float scale = 2.0f * sqrtf(trace + 1.0f);
    if (scale <= kAxisEpsilon) {
      data[0] = 1.0f;
      return Vec4(data);
    }
    data[0] = 0.25f * scale;
    data[1] = (rotation[2][1] - rotation[1][2]) / scale;
    data[2] = (rotation[0][2] - rotation[2][0]) / scale;
    data[3] = (rotation[1][0] - rotation[0][1]) / scale;
  } else if (rotation[0][0] > rotation[1][1] && rotation[0][0] > rotation[2][2]) {
    const float scale =
        2.0f * sqrtf(fmaxf(0.0f, 1.0f + rotation[0][0] - rotation[1][1] - rotation[2][2]));
    if (scale <= kAxisEpsilon) {
      data[0] = 1.0f;
      return Vec4(data);
    }
    data[0] = (rotation[2][1] - rotation[1][2]) / scale;
    data[1] = 0.25f * scale;
    data[2] = (rotation[0][1] + rotation[1][0]) / scale;
    data[3] = (rotation[0][2] + rotation[2][0]) / scale;
  } else if (rotation[1][1] > rotation[2][2]) {
    const float scale =
        2.0f * sqrtf(fmaxf(0.0f, 1.0f + rotation[1][1] - rotation[0][0] - rotation[2][2]));
    if (scale <= kAxisEpsilon) {
      data[0] = 1.0f;
      return Vec4(data);
    }
    data[0] = (rotation[0][2] - rotation[2][0]) / scale;
    data[1] = (rotation[0][1] + rotation[1][0]) / scale;
    data[2] = 0.25f * scale;
    data[3] = (rotation[1][2] + rotation[2][1]) / scale;
  } else {
    const float scale =
        2.0f * sqrtf(fmaxf(0.0f, 1.0f + rotation[2][2] - rotation[0][0] - rotation[1][1]));
    if (scale <= kAxisEpsilon) {
      data[0] = 1.0f;
      return Vec4(data);
    }
    data[0] = (rotation[1][0] - rotation[0][1]) / scale;
    data[1] = (rotation[0][2] + rotation[2][0]) / scale;
    data[2] = (rotation[1][2] + rotation[2][1]) / scale;
    data[3] = 0.25f * scale;
  }
  return Vec4(data);
}

bool TryQuaternionFromRotation(const Rotation& input, Vec4* output) {
  if (!output) return false;
  *output = {};
  if (!IsRotationValid(input)) return false;
  const auto q = QuaternionFromValidRotation(input);
  return Normalize(q, output);
}
bool TryAxisAngleFromRotation(const Rotation& rotation, Vec4* output) {
  if (!output) return false;
  *output = {};
  Vec4 q;
  if (!TryQuaternionFromRotation(rotation, &q)) return false;
  if (q(0, 0) < 0) q *= -1;
  const double norm = std::hypot(std::hypot(static_cast<double>(q(1, 0)), q(2, 0)), q(3, 0));
  (*output)(0, 0) = 1;
  if (norm > 0) {
    for (int i = 0; i < 3; ++i) (*output)(i, 0) = static_cast<float>(q(i + 1, 0) / norm);
    (*output)(3, 0) = static_cast<float>(2 * std::atan2(norm, static_cast<double>(q(0, 0))));
  }
  return true;
}
Rotation RotationFromRpy(const Vec3& v) {
  Rotation out;
  return TryRotationFromRpy(v, &out) ? out : Invalid<3, 3>();
}
Vec3 RpyFromRotation(const Rotation& r) {
  Vec3 out;
  return TryRpyFromRotation(r, &out) ? out : Invalid<3, 1>();
}
Rotation RotationFromAxisAngle(const Vec3& v, float a) {
  Rotation out;
  return TryRotationFromAxisAngle(v, a, &out) ? out : Invalid<3, 3>();
}
Vec4 AxisAngleFromRotation(const Rotation& r) {
  Vec4 out;
  return TryAxisAngleFromRotation(r, &out) ? out : Invalid<4, 1>();
}
Rotation RotationFromQuaternion(const Vec4& q) {
  Rotation out;
  return TryRotationFromQuaternion(q, &out) ? out : Invalid<3, 3>();
}
Vec4 QuaternionFromRotation(const Rotation& r) {
  Vec4 out;
  return TryQuaternionFromRotation(r, &out) ? out : Invalid<4, 1>();
}
}  // namespace mr::robotics::core
