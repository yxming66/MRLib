#include <cassert>
#include <cmath>
#include <limits>
#include "component/math/matrix.hpp"
#include "component/filter/kf.hpp"
#include "component/filter/ekf.hpp"
#include "component/trajectory/s_curve.hpp"
#include "component/trajectory/trapezoid.hpp"
#include "component/trajectory/online_trapezoid.hpp"
#include "component/trajectory/cubic_blend.hpp"

namespace m = mr::component::math;
namespace mm = m::matrix;
using M = m::Matrix<2, 2>;

static void Solver() {
  const float values[] = {0, 2, 1, 3};
  M a(values), b = mm::identity<2>(), x;
  assert(mm::solve(a, b, &x));
  assert((a*x-b).norm() < 1e-5f);
  assert(mm::inverse(a, &a)); // coefficient/output alias
  assert((a-x).norm() < 1e-5f);
  a = M(values);
  assert(mm::solve(a, b, &b)); // RHS/output alias
  assert((b-x).norm() < 1e-5f);
  assert(!mm::solve(a, b, &x, -1));
  assert(x.norm() == 0);
  b[0][0] = std::numeric_limits<float>::infinity();
  assert(!mm::solve(a, b, &x)); assert(x.norm() == 0);
  const float singular[] = {1,2,2,4};
  assert(!mm::inverse(M(singular), &x)); assert(x.norm() == 0);
  assert(!mm::inverse(a, &x, std::numeric_limits<float>::quiet_NaN()));
  const float tiny[] = {1e-20f,0,0,1e-20f};
  b = mm::ones<2,2>() * 1e30f;
  assert(!mm::solve(M(tiny), b, &x, 1e-30f)); assert(x.norm() == 0);
}

using E = mr::comp::filter::ekf<1, 1>;
static bool Transition(const E::State& x, const E::Control&, E::State* out, void*) {
  (*out)[0][0] = x[0][0] * 2; return true;
}
static bool Jacobian(const E::State&, const E::Control&, E::StateTransition* out, void*) {
  (*out)[0][0] = 2; return true;
}
static bool Observation(const E::State& x, E::Measurement* out, void*) {
  *out = x; return true;
}
static bool ObservationJacobian(const E::State&, E::Observation* out, void*) {
  (*out)[0][0] = 1; return true;
}

template<class Filter> static void Rollback(Filter filter) {
  const auto x = filter.state();
  const auto p = filter.covariance();
  typename Filter::Measurement bad;
  bad[0][0] = std::numeric_limits<float>::quiet_NaN();
  assert(!filter.Step(bad));
  assert(!filter.valid());
  assert((filter.state()-x).norm() == 0);
  assert((filter.covariance()-p).norm() == 0);
  assert(filter.innovation().norm() == 0);
  filter.Reset();
  assert(filter.valid());
  typename Filter::Measurement good;
  good[0][0] = 3;
  assert(filter.Step(good));
  assert(filter.state()[0][0] > x[0][0]);
}

namespace tr = mr::comp::traj;
static void Trajectories() {
  for (float distance : {0.001f, 0.2f, 3.0f, 20.0f, -3.0f}) {
    tr::SCurveProfile p;
    assert(p.Configure({distance, 2, 3, 10}));
    const float dt = p.Duration()/5000;
    auto prev = p.Sample(0);
    for (int k=1; k<=5000; ++k) {
      const auto now = p.Sample(k*dt);
      assert(now.valid);
      assert(std::fabs(now.velocity) <= 2.00001f);
      assert(std::fabs(now.acceleration) <= 3.00001f);
      assert(std::fabs(now.acceleration-prev.acceleration) <= 10.01f*dt+1e-6f);
      const float avg_v = (now.velocity+prev.velocity)/2;
      assert(std::fabs((now.position-prev.position)/dt-avg_v) < 0.01f);
      prev = now;
    }
    auto end = p.Sample(p.Duration());
    assert(end.finished && end.position == distance && end.velocity == 0 && end.acceleration == 0);
    assert(!p.Sample(NAN).valid);
  }
  tr::SynchronizedProfile<2,tr::SCurveProfile> sync;
  assert(sync.Configure({{{1,2,3,10}, {8,2,3,10}}}));
  const float t = sync.Duration();
  assert(!sync.SampleAxis(0,t*0.99f).finished);
  assert(!sync.SampleAxis(1,t*0.99f).finished);
  assert(sync.SampleAxis(0,t).position == 1 && sync.SampleAxis(1,t).position == 8);
  assert(!sync.Configure({{{1,2,3,10}, {NAN,2,3,10}}}));
  assert(!sync.SampleAxis(0,0).valid && sync.Axis(0) == nullptr);
  tr::TrapezoidProfile p;
  assert(p.ConfigureForDuration({1,2,3},10000)); // cancellation used to drop the acceleration phase
  auto mid = p.Sample(5000);
  assert(mid.valid && std::fabs(mid.position-0.5f) < 1e-5f && mid.velocity > 0);
  assert(p.Sample(10000).position == 1);
  assert(!p.Sample(NAN).valid);
  assert(!p.Configure({1,-1,2}));
  float velocity = 1, position = 0;
  for (int k=0; k<20000; ++k) {
    const float target = k < 2 ? 0.0001f : -1.0f;
    const float old = velocity;
    auto out = tr::update_trapezoid_axis(position,target,&velocity,2,3,0.001f);
    assert(out.valid && std::fabs(velocity-old) <= 0.003001f);
    position = out.position;
  }
  assert(std::fabs(position+1) <= 0.002f && velocity == 0);
  const float old = velocity;
  assert(!tr::update_trapezoid_axis(position,2,&velocity,2,3,NAN).valid);
  assert(velocity == old);
  assert(!tr::sample_cubic_blend(NAN,1).valid);
  assert(tr::cubic_blend_duration_from_limits(1,1,1,NAN) < 0);
}

int main() {
  Solver();
  Trajectories();
  mr::comp::filter::kf<1,1>::Config c;
  c.A[0][0] = 2; c.x0[0][0] = 1;
  Rollback(mr::comp::filter::kf<1,1>(c));
  E::Config e;
  e.transition = Transition; e.transition_jacobian = Jacobian;
  e.observation = Observation; e.observation_jacobian = ObservationJacobian;
  e.x0[0][0] = 1;
  Rollback(E(e));
  c.P0[0][0] = -1;
  assert((!mr::comp::filter::kf<1,1>(c).valid()));
  const float indefinite[] = {1,2,2,1};
  assert(!mm::is_covariance(M(indefinite)));
  const float singular[] = {1,1,1,1};
  assert(mm::is_covariance(M(singular)));
}
