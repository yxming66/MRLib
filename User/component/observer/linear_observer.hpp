#pragma once

/*
 * 离散时间线性状态观测器（Luenberger observer）。
 *
 *     x(k + 1) = A x(k) + B u(k) + w(k)
 *     y(k)     = C x(k) + v(k)
 *
 * 本组件实现确定性的状态观测，不估计噪声协方差。观测器增益 L 由调用方
 * 根据系统模型和期望极点预先计算：
 *
 *     x_hat(k + 1) = A x_hat(k) + B u(k)
 *                    + L (y(k) - C x_hat(k))
 *
 * 模板参数 StateDim、MeasureDim 和 ControlDim 均为编译期常量，运行过程中不
 * 分配内存，适合在 MCU 控制周期内使用。
 */

#include "component/math/matrix.hpp"

namespace mr::comp::observer {

using mr::component::math::Matrix;
namespace matrix = ::mr::component::math::matrix;

template <int StateDim, int MeasureDim, int ControlDim = 1>
class linear_observer {
  static_assert(StateDim > 0, "linear_observer needs at least one state");
  static_assert(MeasureDim > 0, "linear_observer needs at least one measurement");
  static_assert(ControlDim > 0, "linear_observer ControlDim must be positive");

 public:
  using State = Matrix<StateDim, 1>;
  using Measurement = Matrix<MeasureDim, 1>;
  using Control = Matrix<ControlDim, 1>;
  using StateTransition = Matrix<StateDim, StateDim>;
  using ControlMatrix = Matrix<StateDim, ControlDim>;
  using Observation = Matrix<MeasureDim, StateDim>;
  using Gain = Matrix<StateDim, MeasureDim>;

  struct Config {
    StateTransition A;
    ControlMatrix B;
    Observation C;
    Gain L;
    State x0;

    Config()
        : A(matrix::identity_rect<StateDim, StateDim>()),
          B(matrix::zeros<StateDim, ControlDim>()),
          C(matrix::zeros<MeasureDim, StateDim>()),
          L(matrix::zeros<StateDim, MeasureDim>()),
          x0(matrix::zeros<StateDim, 1>()) {
      const int diagonal_count = (StateDim < MeasureDim) ? StateDim : MeasureDim;
      for (int i = 0; i < diagonal_count; ++i) {
        C[i][i] = 1.0f;
      }
    }
  };

  static linear_observer Build(const Config& config) { return linear_observer(config); }

  linear_observer() { Configure(Config{}); }
  explicit linear_observer(const Config& config) { Configure(config); }

  void Configure(const Config& config) {
    config_ = config;
    Reset(config_.x0);
  }

  void Reset() { Reset(config_.x0); }

  void Reset(const State& x0) {
    x_ = x0;
    innovation_ = matrix::zeros<MeasureDim, 1>();
    valid_ = IsUsable(x_) && IsModelUsable();
  }

  // 无控制输入时，控制量默认为零。
  bool Step(const Measurement& measurement) {
    const Control zero_control = matrix::zeros<ControlDim, 1>();
    return Step(zero_control, measurement);
  }

  // 执行一次观测器更新，并返回本次更新后的状态是否有效。
  bool Step(const Control& control, const Measurement& measurement) {
    if (!valid_ || !IsModelUsable() || !mr::component::math::matrix::is_finite(control) ||
        !mr::component::math::matrix::is_finite(measurement)) {
      valid_ = false;
      return false;
    }

    const Measurement innovation = measurement - config_.C * x_;
    const State next_state = config_.A * x_ + config_.B * control + config_.L * innovation;
    if (!IsUsable(innovation) || !IsUsable(next_state)) {
      valid_ = false;
      return false;
    }

    innovation_ = innovation;
    x_ = next_state;
    return valid_ = true;
  }

  const Config& config() const { return config_; }
  const State& state() const { return x_; }
  const Measurement& innovation() const { return innovation_; }
  bool valid() const { return valid_; }

 private:
  template <int Rows>
  static bool IsUsable(const Matrix<Rows, 1>& vector) {
    return matrix::is_finite(vector);
  }

  bool IsModelUsable() const {
    return mr::component::math::matrix::is_finite(config_.A) &&
           mr::component::math::matrix::is_finite(config_.B) &&
           mr::component::math::matrix::is_finite(config_.C) &&
           mr::component::math::matrix::is_finite(config_.L);
  }

  Config config_{};
  State x_ = matrix::zeros<StateDim, 1>();
  Measurement innovation_ = matrix::zeros<MeasureDim, 1>();
  bool valid_ = false;
};

}  // namespace mr::comp::observer
