#pragma once

#include <algorithm>
#include <cmath>

#include "component/math/matrix.hpp"

namespace mr::component::attitude {

/* Fixed-storage port of the repository QEKF: q + observable X/Y gyro biases.
 * The accelerometer corrects roll/pitch only; yaw correction remains disabled.
 */
class QuaternionEkf {
 public:
  using M6 = math::Matrix<6, 6>;
  using V6 = math::Matrix<6, 1>;
  using M3 = math::Matrix<3, 3>;
  using V3 = math::Matrix<3, 1>;

  struct Config {
    float process_noise_quaternion = 10.0f;
    float process_noise_bias = 0.001f;
    float measurement_noise = 10000000.0f;
    float fading = 1.0f;
    float accelerometer_lpf_seconds = 0.0f;
    float chi_square_threshold = 1.0e-8f;
  };

  bool Configure(const Config& cfg) {
    if (!math::is_positive_scalar(cfg.process_noise_quaternion) ||
        !math::is_positive_scalar(cfg.process_noise_bias) ||
        !math::is_positive_scalar(cfg.measurement_noise) ||
        !math::is_positive_scalar(cfg.fading) || cfg.fading > 1.0f ||
        !math::is_finite_scalar(cfg.accelerometer_lpf_seconds) || cfg.accelerometer_lpf_seconds < 0 ||
        !math::is_finite_scalar(cfg.chi_square_threshold) || cfg.chi_square_threshold <= 0.0f) return false;
    cfg_ = cfg;
    Reset();
    configured_ = true;
    return true;
  }

  void Reset() {
    x_ = {}; x_(0, 0) = 1.0f;
    p_ = {};
    for (int r = 0; r < 6; ++r) for (int c = 0; c < 6; ++c) p_(r,c) = r == c ? 1.0f : 0.1f;
    for (float& value : accel_) value = 0.0f;
    accel_initialized_ = false; converged_ = false; stable_error_count_ = 0;
    update_count_ = 0;
  }

  bool Step(const float gyro[3], const float accel[3], float dt) {
    if (!configured_ || !gyro || !accel || !math::is_positive_scalar(dt)) return false;
    for (int i=0;i<3;++i) if (!math::is_finite_scalar(gyro[i]) || !math::is_finite_scalar(accel[i])) return false;

    const float gx=gyro[0]-x_(4,0), gy=gyro[1]-x_(5,0), gz=gyro[2];
    M6 f = math::matrix::identity<6>();
    f(0,1)=-.5f*gx*dt; f(0,2)=-.5f*gy*dt; f(0,3)=-.5f*gz*dt;
    f(1,0)= .5f*gx*dt; f(1,2)= .5f*gz*dt; f(1,3)=-.5f*gy*dt;
    f(2,0)= .5f*gy*dt; f(2,1)=-.5f*gz*dt; f(2,3)= .5f*gx*dt;
    f(3,0)= .5f*gz*dt; f(3,1)= .5f*gy*dt; f(3,2)=-.5f*gx*dt;
    V6 xp=f*x_;
    float qn=std::sqrt(xp(0,0)*xp(0,0)+xp(1,0)*xp(1,0)+xp(2,0)*xp(2,0)+xp(3,0)*xp(3,0));
    if (!math::is_positive_scalar(qn)) return false;
    for(int i=0;i<4;++i) xp(i,0)/=qn;
    const float p0=xp(0,0),p1=xp(1,0),p2=xp(2,0),p3=xp(3,0);
    f(0,4)=p1*dt*.5f; f(0,5)=p2*dt*.5f;
    f(1,4)=-p0*dt*.5f; f(1,5)=p3*dt*.5f;
    f(2,4)=-p3*dt*.5f; f(2,5)=-p0*dt*.5f;
    f(3,4)=p2*dt*.5f; f(3,5)=-p1*dt*.5f;
    M6 q{}; for(int i=0;i<4;++i) q(i,i)=cfg_.process_noise_quaternion*dt;
    q(4,4)=q(5,5)=cfg_.process_noise_bias*dt;
    p_(4,4)=std::min(p_(4,4)/cfg_.fading,10000.0f);
    p_(5,5)=std::min(p_(5,5)/cfg_.fading,10000.0f);
    M6 pp=f*p_*f.transpose()+q;

    if (!accel_initialized_) { for(int i=0;i<3;++i) accel_[i]=accel[i]; accel_initialized_=true; }
    const float a=cfg_.accelerometer_lpf_seconds;
    for(int i=0;i<3;++i) accel_[i]=(accel_[i]*a+accel[i]*dt)/(dt+a);
    const float an=std::sqrt(accel_[0]*accel_[0]+accel_[1]*accel_[1]+accel_[2]*accel_[2]);
    if (!math::is_positive_scalar(an)) return false;
    V3 z; for(int i=0;i<3;++i) z(i,0)=accel_[i]/an;
    const float wn=std::sqrt(gx*gx+gy*gy+gz*gz);
    const bool stable=wn<0.3f && an>9.3f && an<10.3f;
    math::Matrix<3,6> h{};
    const float a0=2*xp(0,0),a1=2*xp(1,0),a2=2*xp(2,0),a3=2*xp(3,0);
    h(0,0)=-a2; h(0,1)=a3; h(0,2)=-a0; h(0,3)=a1;
    h(1,0)=a1; h(1,1)=a0; h(1,2)=a3; h(1,3)=a2;
    h(2,0)=a0; h(2,1)=-a1; h(2,2)=-a2; h(2,3)=a3;
    V3 pred; pred(0,0)=2*(xp(1,0)*xp(3,0)-xp(0,0)*xp(2,0));
    pred(1,0)=2*(xp(0,0)*xp(1,0)+xp(2,0)*xp(3,0));
    pred(2,0)=xp(0,0)*xp(0,0)-xp(1,0)*xp(1,0)-xp(2,0)*xp(2,0)+xp(3,0)*xp(3,0);
    const V3 residual=z-pred;
    M3 r=math::matrix::identity<3>()*cfg_.measurement_noise;
    M3 s{};
    // Full 6-state innovation covariance, including bias cross-covariance.
    M3 hpht{};
    for(int i=0;i<3;++i) for(int j=0;j<3;++j)
      for(int k=0;k<6;++k) for(int l=0;l<6;++l) hpht(i,j)+=h(i,k)*pp(k,l)*h(j,l);
    s=hpht+r;
    M3 sinv; if(!math::matrix::inverse(s,&sinv)) return false;
    const float chi=(residual.transpose()*sinv*residual)(0,0);
    if(chi<.5f*cfg_.chi_square_threshold) converged_=true;
    float gain_scale=1.0f;
    if(chi>cfg_.chi_square_threshold && converged_) {
      stable_error_count_=stable?stable_error_count_+1:0;
      if(stable_error_count_<=50) { x_=xp; p_=pp; ++update_count_; return true; }
      converged_=false;
    } else {
      if(chi>.1f*cfg_.chi_square_threshold && converged_)
        gain_scale=(cfg_.chi_square_threshold-chi)/(.9f*cfg_.chi_square_threshold);
      stable_error_count_=0;
    }
    math::Matrix<6,3> k{};
    for(int i=0;i<6;++i) for(int j=0;j<3;++j)
      for(int m=0;m<6;++m) for(int n=0;n<3;++n) k(i,j)+=pp(i,m)*h(n,m)*sinv(n,j);
    for(int i=0;i<6;++i) for(int j=0;j<3;++j) {
      k(i,j)*=gain_scale;
      if(i>=4) k(i,j)*=std::acos(std::min(1.0f,std::fabs(pred(j,0))))/1.5707963f;
    }
    V6 dx{};
    for(int i=0;i<6;++i) for(int j=0;j<3;++j) dx(i,0)+=k(i,j)*residual(j,0);
    if(converged_) for(int i=4;i<6;++i) dx(i,0)=std::clamp(dx(i,0),-.01f*dt,.01f*dt);
    dx(3,0)=0.0f; // Original QEKF deliberately does not correct yaw.
    x_=xp+dx;
    M6 kh{}; for(int i=0;i<6;++i) for(int j=0;j<6;++j)
      for(int n=0;n<3;++n) kh(i,j)+=k(i,n)*h(n,j);
    p_=pp-kh*pp;
    ++update_count_;
    return math::matrix::is_finite(x_) && math::matrix::is_finite(p_);
  }

  const V6& state() const { return x_; }
  float gyro_bias_x_rad_s() const { return x_(4,0); }
  float gyro_bias_y_rad_s() const { return x_(5,0); }
  uint64_t update_count() const { return update_count_; }
  bool configured() const { return configured_; }

 private:
  Config cfg_{};
  V6 x_{}; M6 p_{};
  float accel_[3]{};
  uint64_t update_count_=0;
  uint32_t stable_error_count_=0;
  bool accel_initialized_=false,converged_=false,configured_=false;
};
} // namespace mr::component::attitude
