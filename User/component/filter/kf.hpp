#pragma once

/*
 * 通用线性 Kalman 滤波器。
 *
 * 适用于线性离散系统：
 *
 *   x(k) = A * x(k-1) + B * u(k) + w
 *   z(k) = H * x(k) + v
 *
 * 模板参数：
 *   StateDim   : 状态向量维度
 *   MeasureDim : 测量向量维度
 *   ControlDim : 控制输入维度，默认 1；无控制输入时直接调用 Predict()
 *
 * 一维直接测量示例：
 *
 *   using Filter = mr::comp::filter::kf<1, 1>;
 *
 *   Filter::Config cfg;
 *   cfg.Q[0][0] = 0.01f;  // 过程噪声协方差
 *   cfg.R[0][0] = 0.10f;  // 测量噪声协方差
 *   cfg.P0[0][0] = 1.0f;  // 初始估计协方差
 *
 *   auto filter = Filter::Build(cfg);
 *   Filter::Measurement z = matrix::zeros<1, 1>();
 *   z[0][0] = measurement;
 *   filter.Step(z);
 *   float y = filter.state()[0][0];
 *
 * 有控制输入时：
 *
 *   Filter::Control u = matrix::zeros<1, 1>();
 *   filter.Step(u, z);
 *
 * 更新步内部使用 component/math/matrix.hpp 的线性方程求解器。
 * 协方差默认使用 Joseph 形式更新，数值上更稳。
 */

#include <stdint.h>

#include "component/filter/filter_types.hpp"
#include "component/math/matrix.hpp"

namespace mr::comp::filter {

using mr::component::math::Matrix;
namespace matrix = ::mr::component::math::matrix;

template <int StateDim, int MeasureDim, int ControlDim = 1>
class kf {
  static_assert(StateDim > 0, "kf needs at least one state");
  static_assert(MeasureDim > 0, "kf needs at least one measurement");
  static_assert(ControlDim > 0, "kf ControlDim must be positive");

 public:
  using State = Matrix<StateDim, 1>;
  using Measurement = Matrix<MeasureDim, 1>;
  using Control = Matrix<ControlDim, 1>;
  using StateTransition = Matrix<StateDim, StateDim>;
  using ControlMatrix = Matrix<StateDim, ControlDim>;
  using Observation = Matrix<MeasureDim, StateDim>;
  using StateCovariance = Matrix<StateDim, StateDim>;
  using MeasurementCovariance = Matrix<MeasureDim, MeasureDim>;
  using InnovationCovariance = Matrix<MeasureDim, MeasureDim>;
  using Gain = Matrix<StateDim, MeasureDim>;

  struct Config {
    StateTransition A;
    ControlMatrix B;
    Observation H;
    StateCovariance Q;
    MeasurementCovariance R;
    StateCovariance P0;
    State x0;
    Scalar min_pivot;
    bool joseph_covariance_update;

    Config()
        : A(matrix::identity_rect<StateDim, StateDim>()),
          B(matrix::zeros<StateDim, ControlDim>()),
          H(matrix::zeros<MeasureDim, StateDim>()),
          Q(matrix::zeros<StateDim, StateDim>()),
          R(matrix::identity_rect<MeasureDim, MeasureDim>()),
          P0(matrix::identity_rect<StateDim, StateDim>()),
          x0(matrix::zeros<StateDim, 1>()),
          min_pivot(mr::component::math::kDefaultMinPivot),
          joseph_covariance_update(true) {
      const uint16_t diag_count =
          (StateDim < MeasureDim) ? StateDim : MeasureDim;
      for (uint16_t i = 0; i < diag_count; ++i) {
        H[i][i] = 1.0f;
      }
    }
  };

  static kf Build(const Config& config) { return kf(config); }

  kf() { Configure(Config{}); }
  explicit kf(const Config& config) { Configure(config); }

  void Configure(const Config& config) {
    config_ = config;
    Reset(config_.x0, config_.P0);
  }

  void Reset() { Reset(config_.x0, config_.P0); }

  void Reset(const State& x0) { Reset(x0, config_.P0); }

  void Reset(const State& x0, const StateCovariance& P0) {
    x_ = x0;
    P_ = Symmetrized(P0);
    K_ = matrix::zeros<StateDim, MeasureDim>();
    innovation_ = matrix::zeros<MeasureDim, 1>();
    innovation_covariance_ = matrix::zeros<MeasureDim, MeasureDim>();
    valid_ = IsStateUsable(x_) && matrix::is_covariance(P0) &&
             mr::component::math::is_positive_scalar(config_.min_pivot, 0.0f) &&
             IsPredictModelUsable() && IsUpdateModelUsable();
  }

  // Fixed-storage transaction: failure latches invalid but retains the last complete snapshot.
  // EKF callbacks must not rely on rollback of mutations in their external context.
  bool Predict() { return Predict(Control{}); }
  bool Predict(const Control& control) {
    kf candidate = *this;
    if (!candidate.PredictImpl(control)) { valid_ = false; return false; }
    *this = candidate; return true;
  }
  bool Update(const Measurement& measurement) {
    kf candidate = *this;
    if (!candidate.UpdateImpl(measurement)) { valid_ = false; return false; }
    *this = candidate; return true;
  }
  bool Step(const Measurement& measurement) { return Step(Control{}, measurement); }
  bool Step(const Control& control, const Measurement& measurement) {
    kf candidate = *this;
    if (!candidate.PredictImpl(control) || !candidate.UpdateImpl(measurement)) {
      valid_ = false; return false;
    }
    *this = candidate; return true;
  }

  const Config& config() const { return config_; }
  const State& state() const { return x_; }
  const StateCovariance& covariance() const { return P_; }
  const Gain& gain() const { return K_; }
  const Measurement& innovation() const { return innovation_; }
  const InnovationCovariance& innovation_covariance() const {
    return innovation_covariance_;
  }
  bool valid() const { return valid_; }

 private:
  bool PredictImpl() {
    if (!valid_ || !IsPredictModelUsable()) {
      valid_ = false;
      return false;
    }

    x_ = config_.A * x_;
    UpdatePredictedCovariance();
    valid_ = IsStateUsable(x_) && IsStateCovarianceUsable(P_);
    return valid_;
  }

  bool PredictImpl(const Control& control) {
    if (!valid_ || !IsPredictModelUsable() ||
        !mr::component::math::matrix::is_finite(control)) {
      valid_ = false;
      return false;
    }

    x_ = config_.A * x_ + config_.B * control;
    UpdatePredictedCovariance();
    valid_ = IsStateUsable(x_) && IsStateCovarianceUsable(P_);
    return valid_;
  }

  bool UpdateImpl(const Measurement& measurement) {
    if (!valid_ || !IsUpdateModelUsable() ||
        !mr::component::math::matrix::is_finite(measurement)) {
      valid_ = false;
      return false;
    }

    const Matrix<StateDim, MeasureDim> pht = P_ * config_.H.transpose();
    innovation_ = measurement - config_.H * x_;
    innovation_covariance_ =
        Symmetrized(config_.H * pht + config_.R);

    Matrix<MeasureDim, StateDim> kt = matrix::zeros<MeasureDim, StateDim>();
    if (!mr::component::math::solve<MeasureDim, StateDim>(
            innovation_covariance_, pht.transpose(), &kt, config_.min_pivot)) {
      valid_ = false;
      return false;
    }

    K_ = kt.transpose();
    x_ = x_ + K_ * innovation_;
    UpdateCorrectedCovariance();

    valid_ = IsStateUsable(x_) && IsStateCovarianceUsable(P_) &&
             mr::component::math::matrix::is_finite(K_);
    return valid_;
  }

  template <int N>
  static Matrix<N, N> Symmetrized(const Matrix<N, N>& matrix) {
    return 0.5f * (matrix + matrix.transpose());
  }

  static bool IsStateUsable(const State& state) {
    return mr::component::math::matrix::is_finite(state);
  }

  static bool IsStateCovarianceUsable(const StateCovariance& covariance) {
    return matrix::is_covariance(covariance);
  }

  bool IsPredictModelUsable() const {
    return mr::component::math::matrix::is_finite(config_.A) &&
           mr::component::math::matrix::is_finite(config_.B) &&
           matrix::is_covariance(config_.Q);
  }

  bool IsUpdateModelUsable() const {
    return mr::component::math::matrix::is_finite(config_.H) &&
           matrix::is_covariance(config_.R);
  }

  void UpdatePredictedCovariance() {
    P_ = Symmetrized(config_.A * P_ * config_.A.transpose() + config_.Q);
  }

  void UpdateCorrectedCovariance() {
    const StateCovariance identity = matrix::identity_rect<StateDim, StateDim>();
    const StateCovariance ikh = identity - K_ * config_.H;

    if (config_.joseph_covariance_update) {
      P_ = Symmetrized(ikh * P_ * ikh.transpose() +
                       K_ * config_.R * K_.transpose());
    } else {
      P_ = Symmetrized(ikh * P_);
    }
  }

  Config config_{};
  State x_ = matrix::zeros<StateDim, 1>();
  StateCovariance P_ = matrix::identity_rect<StateDim, StateDim>();
  Gain K_ = matrix::zeros<StateDim, MeasureDim>();
  Measurement innovation_ = matrix::zeros<MeasureDim, 1>();
  InnovationCovariance innovation_covariance_ =
      matrix::zeros<MeasureDim, MeasureDim>();
  bool valid_ = false;
};

}  // namespace mr::comp::filter
