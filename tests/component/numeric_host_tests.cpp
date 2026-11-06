#include <cassert>
#include <cmath>
#include <limits>
#include "component/math/matrix.hpp"
#include "component/filter/kf.hpp"
#include "component/filter/ekf.hpp"

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

int main() {
  Solver();
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
