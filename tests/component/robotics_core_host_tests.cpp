#include <cassert>
#include <cmath>

#include "robotics/arm/arm.hpp"
#include "robotics/core/transform.hpp"

namespace c = mr::robotics::core;
namespace a = mr::robotics::arm;
namespace mm = mr::component::math::matrix;

int main() {
  c::Rotation out;
  c::Vec3 zero;
  assert(!c::TryRotationFromAxisAngle(zero, 1, &out) && out.norm() == 0);
  assert(c::TryRotationFromAxisAngle(zero, 0, &out) && c::IsRotationValid(out));
  c::Vec4 q;
  assert(!c::TryRotationFromQuaternion(q, &out));
  assert(!mm::is_finite(c::RotationFromQuaternion(q)));
  q(0, 0) = 1e30f;
  q(1, 0) = 1e30f;
  assert(c::TryRotationFromQuaternion(q, &out) && c::IsRotationValid(out));
  auto bad = mm::identity<3>();
  bad(0, 0) = -1;
  assert(!c::IsRotationValid(bad));  // reflection
  bad(0, 0) = 2;
  assert(!c::TryQuaternionFromRotation(bad, &q) && q.norm() == 0);
  for (float angle : {1e-7f, 0.5f, 3.14159265f, 3.14158f}) {
    const float axes[][3] = {{0, 1, -1}, {1, -2, 3}, {0, 0, 1}};
    for (const auto& values : axes) {
      c::Vec3 axis(values);
      const auto r = c::RotationFromAxisAngle(axis, angle);
      const auto aa = c::AxisAngleFromRotation(r);
      const auto roundtrip = c::RotationFromAxisAngle(aa.block<3, 1>(0, 0), aa(3, 0));
      assert((r - roundtrip).norm() < 1e-4f);
    }
  }
  c::Vec3 rpy;
  rpy(0, 0) = 0.4f;
  rpy(1, 0) = 1.5707963268f;
  rpy(2, 0) = 0.7f;
  const auto locked = c::RotationFromRpy(rpy);
  assert((locked - c::RotationFromRpy(c::RpyFromRotation(locked))).norm() < 1e-5f);
  c::Transform t = mm::identity<4>();
  t(0, 0) = 2;
  assert(!a::IsTransformValid(t));
  assert(!c::TryInverseTransform(t, &t) && t.norm() == 0);
  c::PoseCoordinates6 coordinates;
  coordinates(0, 0) = 1;
  coordinates(4, 0) = 1;
  assert(c::TryTransformFromPoseCoordinates(coordinates, &t));
  const auto recovered = c::PoseCoordinatesFromTransform(t);
  assert((recovered - coordinates).norm() < 1e-5f);
  // Velocity is world-frame tool-origin velocity, even for a rotated start pose.
  a::planning::CartesianPlanRequest request;
  c::Vec3 z;
  z(2, 0) = 1;
  request.start = c::TransformFromAxisAngle(z, 1.5707963268f);
  request.goal = request.start;
  request.goal(0, 3) = 1;
  a::planning::CartesianPlanner planner;
  assert(planner.Configure(request));
  const float mid = planner.Duration() / 2, dt = 0.001f;
  const auto sample = planner.Sample(mid);
  const auto left = planner.Sample(mid - dt), right = planner.Sample(mid + dt);
  assert(sample.valid && std::fabs(sample.twist(1, 0)) < 1e-5f);
  assert(std::fabs(sample.twist(0, 0) - (right.pose(0, 3) - left.pose(0, 3)) / (2 * dt)) < 1e-4f);

  // One prismatic joint cannot satisfy a rotation request. Degradation is opt-in.
  a::JointModel joints[1];
  joints[0].type = a::JointType::kPrismatic;
  joints[0].limits.max_velocity = 1;
  joints[0].limits.max_acceleration = 2;
  a::ArmModel<1> model(joints, mm::identity<4>(), mm::identity<4>(), zero);
  a::control::ArmControllerConfig<1> cfg;
  assert(!cfg.enable_position_only_fallback);
  a::JointState<1> feedback;
  feedback.valid = true;
  a::ArmCommand<1> command;
  command.mode = a::ControlMode::kMoveCartesian;
  command.streaming_cartesian = true;
  command.position_only = false;
  command.pose_target = c::TransformFromAxisAngle(z, 0.5f);
  a::control::ArmController<1> strict(model, cfg);
  auto result = strict.Update(feedback, command, 0.01f);
  assert(!a::IsOk(result.status) && !result.position_only_fallback);
  assert(result.primary_ik_status == result.status);
  cfg.enable_position_only_fallback = true;
  a::control::ArmController<1> degraded(model, cfg);
  result = degraded.Update(feedback, command, 0.01f);
  assert(a::IsOk(result.status) && result.position_only_fallback);
  assert(!a::IsOk(result.primary_ik_status));
  command.position_only = true;
  command.pose_target = mm::identity<4>();
  command.pose_target(2, 3) = 0.8f;
  degraded.Reset(feedback);
  float velocity = 0;
  for (int i = 0; i < 300; ++i) {
    if (i == 100) command.pose_target(2, 3) = -0.5f;
    result = degraded.Update(feedback, command, 0.01f);
    assert(a::IsOk(result.status));
    const float dv = result.joint_command.velocity(0, 0) - velocity;
    assert(std::fabs(dv) <= 0.020001f);
    assert(std::fabs(result.joint_command.acceleration(0, 0) - dv / 0.01f) < 1e-5f);
    velocity = result.joint_command.velocity(0, 0);
    feedback.position = result.joint_command.position;
    feedback.velocity = result.joint_command.velocity;
  }
}
