#pragma once

// Smooth Cartesian interpolation. Inverse kinematics is deliberately outside
// this class; callers can feed each sampled pose to their chosen IK solver.

#include "../core/types.hpp"
#include "component/math/matrix.hpp"
#include "component/math/scalar.hpp"
#include "component/trajectory/cubic_blend.hpp"
#include "robotics/core/transform.hpp"

namespace mr::robotics::arm::planning {

struct CartesianPlanRequest {
  Transform start;
  Transform goal;
  Scalar max_linear_velocity_m_s;
  Scalar max_angular_velocity_rad_s;
  Scalar max_linear_acceleration_m_s2;
  Scalar max_angular_acceleration_rad_s2;

  CartesianPlanRequest()
      : start(mr::component::math::matrix::identity<4>()),
        goal(mr::component::math::matrix::identity<4>()),
        max_linear_velocity_m_s(0.2f),
        max_angular_velocity_rad_s(1.0f),
        max_linear_acceleration_m_s2(0.5f),
        max_angular_acceleration_rad_s2(2.0f) {}
};

struct CartesianPlanSample {
  Transform pose;
  // Geometric velocity at the tool origin, expressed in the model world frame.
  Twist6 twist;
  Scalar time_from_start_s;
  bool valid;
  bool finished;

  CartesianPlanSample()
      : pose(mr::component::math::matrix::identity<4>()),
        twist(mr::component::math::matrix::zeros<6, 1>()),
        time_from_start_s(0.0f),
        valid(false),
        finished(false) {}
};

class CartesianPlanner {
 public:
  CartesianPlanner()
      : request_(),
        relative_coordinates_(mr::component::math::matrix::zeros<6, 1>()),
        duration_s_(0.0f),
        valid_(false) {}

  bool Configure(const CartesianPlanRequest& request) {
    valid_ = false;
    duration_s_ = 0.0f;
    request_ = request;

    if (!IsTransformValid(request.start) || !IsTransformValid(request.goal) ||
        !mr::component::math::is_positive_scalar(request.max_linear_velocity_m_s) ||
        !mr::component::math::is_positive_scalar(request.max_angular_velocity_rad_s) ||
        !mr::component::math::is_positive_scalar(request.max_linear_acceleration_m_s2) ||
        !mr::component::math::is_positive_scalar(request.max_angular_acceleration_rad_s2)) {
      return false;
    }

    const Transform relative = mr::robotics::core::InverseTransform(request.start) * request.goal;
    relative_coordinates_ = mr::robotics::core::PoseCoordinatesFromTransform(relative);
    if (!mr::component::math::matrix::is_finite(relative_coordinates_)) {
      return false;
    }

    const Scalar linear_distance = relative_coordinates_.block<3, 1>(0, 0).norm();
    const Scalar angular_distance = relative_coordinates_.block<3, 1>(3, 0).norm();
    const Scalar linear_duration = mr::comp::traj::cubic_blend_duration_from_limits(
        linear_distance, request.max_linear_velocity_m_s, request.max_linear_acceleration_m_s2);
    const Scalar angular_duration = mr::comp::traj::cubic_blend_duration_from_limits(
        angular_distance, request.max_angular_velocity_rad_s,
        request.max_angular_acceleration_rad_s2);
    if (linear_duration < 0.0f || angular_duration < 0.0f) {
      return false;
    }

    duration_s_ = (linear_duration > angular_duration) ? linear_duration : angular_duration;
    valid_ = mr::component::math::is_finite_scalar(duration_s_);
    if (!valid_) {
      duration_s_ = 0.0f;
    }
    return valid_;
  }

  CartesianPlanSample Sample(Scalar elapsed_s) const {
    CartesianPlanSample sample;
    sample.time_from_start_s = elapsed_s;
    sample.pose = request_.start;
    if (!valid_ || !mr::component::math::is_finite_scalar(elapsed_s)) {
      return sample;
    }

    sample.valid = true;
    if (duration_s_ <= mr::component::math::kDefaultEpsilon) {
      sample.pose = request_.goal;
      sample.finished = true;
      return sample;
    }

    const mr::comp::traj::CubicBlendSample blend =
        mr::comp::traj::sample_cubic_blend(elapsed_s, duration_s_);
    sample.pose = request_.start *
                  mr::robotics::core::TransformFromPoseCoordinates(relative_coordinates_ * blend.s);
    const Rotation start_rotation = mr::robotics::core::RotationOf(request_.start);
    const Vec3 linear = start_rotation * relative_coordinates_.block<3, 1>(0, 0) * blend.ds;
    const Vec3 angular = start_rotation * relative_coordinates_.block<3, 1>(3, 0) * blend.ds;
    for (int i = 0; i < 3; ++i) {
      sample.twist(i, 0) = linear(i, 0);
      sample.twist(i + 3, 0) = angular(i, 0);
    }
    sample.valid = blend.valid && IsTransformValid(sample.pose) &&
                   mr::component::math::matrix::is_finite(sample.twist);
    sample.finished = blend.finished;
    if (sample.finished) {
      sample.pose = request_.goal;
      sample.twist = mr::component::math::matrix::zeros<6, 1>();
    }
    return sample;
  }

  Scalar Duration() const { return duration_s_; }
  bool IsValid() const { return valid_; }
  const CartesianPlanRequest& Request() const { return request_; }

 private:
  CartesianPlanRequest request_;
  mr::robotics::core::PoseCoordinates6 relative_coordinates_;
  Scalar duration_s_;
  bool valid_;
};

}  // namespace mr::robotics::arm::planning
