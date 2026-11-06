#pragma once

/*
 * 通用扩展 Kalman 滤波器。
 *
 * 适用于非线性离散系统：
 *
 *   x(k) = f(x(k-1), u(k)) + w
 *   z(k) = h(x(k)) + v
 *
 * 用户需要提供 f、df/dx、h、dh/dx 四个函数。这样 EKF 本身只负责预测、
 * 线性化后的协方差传播，以及测量修正。
 *
 * 模板参数：
 *   StateDim   : 状态向量维度
 *   MeasureDim : 测量向量维度
 *   ControlDim : 控制输入维度，默认 1；无控制输入时直接调用 Predict()
 *
 * 最小用法：
 *
 *   using Filter = mr::comp::filter::ekf<2, 1, 1>;
 *
 *   bool Transition(const Filter::State& x,
 *                   const Filter::Control& u,
 *                   Filter::State* out,
 *                   void* context);
 *   bool TransitionJacobian(const Filter::State& x,
 *                           const Filter::Control& u,
 *                           Filter::StateTransition* out,
 *                           void* context);
 *   bool Observation(const Filter::State& x,
 *                    Filter::Measurement* out,
 *                    void* context);
 *   bool ObservationJacobian(const Filter::State& x,
 *                            Filter::Observation* out,
 *                            void* context);
 *
 *   Filter::Config cfg;
 *   cfg.transition = Transition;
 *   cfg.transition_jacobian = TransitionJacobian;
 *   cfg.observation = Observation;
 *   cfg.observation_jacobian = ObservationJacobian;
 *   cfg.Q[0][0] = 0.01f;
 *   cfg.R[0][0] = 0.10f;
 *
 *   auto filter = Filter::Build(cfg);
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
class ekf {
  static_assert(StateDim > 0, "ekf needs at least one state");
  static_assert(MeasureDim > 0, "ekf needs at least one measurement");
  static_assert(ControlDim > 0, "ekf ControlDim must be positive");

 public:
  using State = Matrix<StateDim, 1>;
  using Measurement = Matrix<MeasureDim, 1>;
  using Control = Matrix<ControlDim, 1>;
  using StateTransition = Matrix<StateDim, StateDim>;
  using Observation = Matrix<MeasureDim, StateDim>;
  using StateCovariance = Matrix<StateDim, StateDim>;
  using MeasurementCovariance = Matrix<MeasureDim, MeasureDim>;
  using InnovationCovariance = Matrix<MeasureDim, MeasureDim>;
  using Gain = Matrix<StateDim, MeasureDim>;

  using TransitionFunction =
      bool (*)(const State& state,
               const Control& control,
               State* out,
               void* context);
  using TransitionJacobianFunction =
      bool (*)(const State& state,
               const Control& control,
               StateTransition* out,
               void* context);
  using ObservationFunction =
      bool (*)(const State& state, Measurement* out, void* context);
  using ObservationJacobianFunction =
      bool (*)(const State& state, Observation* out, void* context);

  struct Config {
    TransitionFunction transition;
    TransitionJacobianFunction transition_jacobian;
    ObservationFunction observation;
    ObservationJacobianFunction observation_jacobian;
    void* context;
    StateCovariance Q;
    MeasurementCovariance R;
    StateCovariance P0;
    State x0;
    Scalar min_pivot;
    bool joseph_covariance_update;

    Config()
        : transition(nullptr),
          transition_jacobian(nullptr),
          observation(nullptr),
          observation_jacobian(nullptr),
          context(nullptr),
          Q(matrix::zeros<StateDim, StateDim>()),
          R(matrix::identity_rect<MeasureDim, MeasureDim>()),
          P0(matrix::identity_rect<StateDim, StateDim>()),
          x0(matrix::zeros<StateDim, 1>()),
          min_pivot(mr::component::math::kDefaultMinPivot),
          joseph_covariance_update(true) {}
  };

  static ekf Build(const Config& config) { return ekf(config); }

  ekf() { Configure(Config{}); }
  explicit ekf(const Config& config) { Configure(config); }

  void Configure(const Config& config) {
    config_ = config;
    Reset(config_.x0, config_.P0);
  }

  void Reset() { Reset(config_.x0, config_.P0); }

  void Reset(const State& x0) { Reset(x0, config_.P0); }

  void Reset(const State& x0, const StateCovariance& P0) {
    x_ = x0;
    P_ = Symmetrized(P0);
    F_ = matrix::identity_rect<StateDim, StateDim>();
    H_ = matrix::zeros<MeasureDim, StateDim>();
    K_ = matrix::zeros<StateDim, MeasureDim>();
    predicted_measurement_ = matrix::zeros<MeasureDim, 1>();
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
    ekf candidate = *this;
    if (!candidate.PredictImpl(control)) { valid_ = false; return false; }
    *this = candidate; return true;
  }
  bool Update(const Measurement& measurement) {
    ekf candidate = *this;
    if (!candidate.UpdateImpl(measurement)) { valid_ = false; return false; }
    *this = candidate; return true;
  }
  bool Step(const Measurement& measurement) { return Step(Control{}, measurement); }
  bool Step(const Control& control, const Measurement& measurement) {
    ekf candidate = *this;
    if (!candidate.PredictImpl(control) || !candidate.UpdateImpl(measurement)) {
      valid_ = false; return false;
    }
    *this = candidate; return true;
  }

  const Config& config() const { return config_; }
  const State& state() const { return x_; }
  const StateCovariance& covariance() const { return P_; }
  const StateTransition& transition_jacobian() const { return F_; }
  const Observation& observation_jacobian() const { return H_; }
  const Gain& gain() const { return K_; }
  const Measurement& predicted_measurement() const {
    return predicted_measurement_;
  }
  const Measurement& innovation() const { return innovation_; }
  const InnovationCovariance& innovation_covariance() const {
    return innovation_covariance_;
  }
  bool valid() const { return valid_; }

 private:
  bool PredictImpl() {
    const Control zero_control = matrix::zeros<ControlDim, 1>();
    return PredictImpl(zero_control);
  }

  bool PredictImpl(const Control& control) {
    if (!valid_ || !IsPredictModelUsable() ||
        !mr::component::math::matrix::is_finite(control)) {
      valid_ = false;
      return false;
    }

    State predicted = matrix::zeros<StateDim, 1>();
    StateTransition transition_jacobian =
        matrix::zeros<StateDim, StateDim>();
    if (!config_.transition(x_, control, &predicted, config_.context) ||
        !config_.transition_jacobian(x_, control, &transition_jacobian,
                                     config_.context)) {
      valid_ = false;
      return false;
    }
    if (!IsStateUsable(predicted) ||
        !mr::component::math::matrix::is_finite(transition_jacobian)) {
      valid_ = false;
      return false;
    }

    F_ = transition_jacobian;
    x_ = predicted;
    P_ = Symmetrized(F_ * P_ * F_.transpose() + config_.Q);
    valid_ = IsStateUsable(x_) && IsStateCovarianceUsable(P_);
    return valid_;
  }

  bool UpdateImpl(const Measurement& measurement) {
    if (!valid_ || !IsUpdateModelUsable() ||
        !mr::component::math::matrix::is_finite(measurement)) {
      valid_ = false;
      return false;
    }

    Measurement predicted_measurement = matrix::zeros<MeasureDim, 1>();
    Observation observation_jacobian =
        matrix::zeros<MeasureDim, StateDim>();
    if (!config_.observation(x_, &predicted_measurement, config_.context) ||
        !config_.observation_jacobian(x_, &observation_jacobian,
                                      config_.context)) {
      valid_ = false;
      return false;
    }
    if (!mr::component::math::matrix::is_finite(predicted_measurement) ||
        !mr::component::math::matrix::is_finite(observation_jacobian)) {
      valid_ = false;
      return false;
    }

    predicted_measurement_ = predicted_measurement;
    H_ = observation_jacobian;

    const Matrix<StateDim, MeasureDim> pht = P_ * H_.transpose();
    innovation_ = measurement - predicted_measurement_;
    innovation_covariance_ = Symmetrized(H_ * pht + config_.R);

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
    return config_.transition != nullptr &&
           config_.transition_jacobian != nullptr &&
           matrix::is_covariance(config_.Q);
  }

  bool IsUpdateModelUsable() const {
    return config_.observation != nullptr &&
           config_.observation_jacobian != nullptr &&
           matrix::is_covariance(config_.R);
  }

  void UpdateCorrectedCovariance() {
    const StateCovariance identity = matrix::identity_rect<StateDim, StateDim>();
    const StateCovariance ikh = identity - K_ * H_;

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
  StateTransition F_ = matrix::identity_rect<StateDim, StateDim>();
  Observation H_ = matrix::zeros<MeasureDim, StateDim>();
  Gain K_ = matrix::zeros<StateDim, MeasureDim>();
  Measurement predicted_measurement_ = matrix::zeros<MeasureDim, 1>();
  Measurement innovation_ = matrix::zeros<MeasureDim, 1>();
  InnovationCovariance innovation_covariance_ =
      matrix::zeros<MeasureDim, MeasureDim>();
  bool valid_ = false;
};

}  // namespace mr::comp::filter
