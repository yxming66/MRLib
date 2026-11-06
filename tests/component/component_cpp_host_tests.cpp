#include <cstdio>
#include <limits>

#include "component/cmd_center/cmd_center.hpp"
#include "component/controller/cascade.hpp"
#include "component/controller/lqr.hpp"
#include "component/math/scalar.hpp"
#include "component/observer/linear_observer.hpp"

#define CHECK(condition)                                                   \
  do {                                                                     \
    if (!(condition)) {                                                    \
      std::fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #condition); \
      return false;                                                        \
    }                                                                      \
  } while (0)

namespace {
bool Near(float a, float b) { return std::isfinite(a) && std::fabs(a - b) < 1e-5f; }

bool TestScalarInterface() {
  namespace math = mr::component::math;
  CHECK(math::clamp_scalar(4.0f, -1.0f, 1.0f) == 1.0f);
  CHECK(math::wrap_error(-31.0f, 10.0f) == -1.0f);
  CHECK(math::wrap_to_pi(math::kPi) == -math::kPi);
  CHECK(Near(math::angle_error(-math::kPi + 0.1f, math::kPi - 0.1f), 0.2f));
  CHECK(math::sanitize_positive(-1.0f, 2.0f) == 2.0f);
  CHECK(math::sanitize_dt(0.0f, 0.002f, 0.001f, 0.01f) == 0.002f);
  float values[] = {-4.0f, 2.0f};
  CHECK(math::scale_to_abs_limit(values, 2, 2.0f));
  CHECK(values[0] == -2.0f && values[1] == 1.0f);
  return true;
}

bool TestObserverDimensions() {
  using Observer = mr::comp::observer::linear_observer<1, 1>;
  Observer::Config config;
  config.A[0][0] = 2.0f;
  config.B[0][0] = 3.0f;
  config.C[0][0] = 1.0f;
  config.L[0][0] = 0.5f;
  config.x0[0][0] = 1.0f;
  Observer observer(config);
  Observer::Control u{};
  Observer::Measurement y{};
  u[0][0] = 2.0f;
  y[0][0] = 5.0f;
  CHECK(observer.Step(u, y));
  CHECK(Near(observer.innovation()[0][0], 4.0f));
  CHECK(Near(observer.state()[0][0], 10.0f));  // 2*1 + 3*2 + .5*(5-1)
  y[0][0] = std::numeric_limits<float>::quiet_NaN();
  CHECK(!observer.Step(y));
  CHECK(!observer.valid());
  observer.Reset();
  CHECK(observer.valid());
  CHECK(Near(observer.state()[0][0], 1.0f));
  mr::comp::observer::linear_observer<2, 1> rectangular;
  CHECK(rectangular.valid());
  return true;
}

struct Command {
  int mode = 0;
  float value = 0.0f;
};
struct Remote {
  bool enabled = true;
  float stick = 0.5f;
};
struct Pc {
  bool enabled = false;
};
struct FromRemote {
  bool operator()(const Remote& remote, Command& out) const {
    if (!remote.enabled) return false;
    out = {1, remote.stick};
    return true;
  }
};
struct FromBoth {
  bool operator()(const Pc& pc, const Remote& remote, Command& out) const {
    if (!pc.enabled || !remote.enabled) return false;
    out = {2, remote.stick * 2.0f};
    return true;
  }
};
struct Safe {
  bool operator()(Command& out) const {
    out = {};
    return true;
  }
};
struct Sink {
  Command* destination;
  bool operator()(const Command& command) const {
    *destination = command;
    return true;
  }
};

bool TestCommandRouting() {
  Command published;
  cmd::Center<2, 1, 4> center;
  center.input<Remote>("remote").input<Pc>("pc");
  center.output<Command>("command", Sink{&published})
      .safe<Safe>()
      .routes(cmd::from<Remote, FromRemote>().priority(cmd::Priority::Manual),
              cmd::from<Pc, Remote, FromBoth>().priority(cmd::Priority::Remote));
  center.tick(1).publish();
  CHECK(!center.overflowed());
  CHECK(published.mode == 1 && published.value == 0.5f);
  center.context().input<Pc>()->enabled = true;
  center.tick(2).publish();
  CHECK(published.mode == 2 && published.value == 1.0f);
  center.context().input<Remote>()->enabled = false;
  center.tick(3).publish();
  CHECK(published.mode == 0 && published.value == 0.0f);
  return true;
}
struct TimedA {
  uint32_t stamp = 0;
  bool enabled = true;
  bool online() const { return enabled; }
  uint32_t timestamp_ms() const { return stamp; }
};
struct TimedB : TimedA {};
struct RouteA {
  bool operator()(const TimedA&, Command& c) const {
    c.mode = 1;
    return true;
  }
};
struct RouteB {
  bool operator()(const TimedB&, Command& c) const {
    c.mode = 2;
    return true;
  }
};
struct OwnedInput {
  int* live;
  explicit OwnedInput(int* count) : live(count) { ++*live; }
  ~OwnedInput() { --*live; }
};
bool TestCommandBoundaries() {
  static_assert(!std::is_copy_constructible_v<cmd::Center<>>);
  static_assert(!std::is_move_constructible_v<cmd::Center<>>);
  int live = 0;
  {
    cmd::Center<> owner;
    owner.input<OwnedInput>("owned", &live);
    CHECK(live == 1);
  }
  CHECK(live == 0);
  Command published;
  cmd::Center<2, 1, 4> center;
  center.input<TimedA>("a").input<TimedB>("b");
  auto* a = center.context().input<TimedA>();
  auto* b = center.context().input<TimedB>();
  a->stamp = 0xfffffff0u;
  b->stamp = 4;
  center.output<Command>("out", Sink{&published})
      .safe<Safe>()
      .arbitrate(cmd::HighestPriority{cmd::TieBreak::Freshest})
      .routes(cmd::from<TimedA, RouteA>(), cmd::from<TimedB, RouteB>());
  center.tick(5).publish();
  CHECK(published.mode == 2 && center.last_publish_ok());
  a->stamp = 6;
  center.tick(7).publish();
  CHECK(published.mode == 1);
  a->enabled = false;
  center.tick(8).publish();
  CHECK(published.mode == 2);
  auto invalid = center.output<int>("overflow", [](int) { return true; });
  CHECK(!invalid);
  // A failed registration may be chained without touching a previously constructed slot.
  invalid.safe([](int& x) {
    x = 0;
    return true;
  });
  center.tick(9).publish();
  CHECK(center.overflowed() && published.mode == 0);
  cmd::Center<1, 1, 1, 1> small;
  small.input<Remote>("r");
  small.output<Command>("o", Sink{&published})
      .route(cmd::from<Remote, FromRemote>())
      .route(cmd::from<Remote, FromRemote>());
  small.tick(1).publish();
  CHECK(small.overflowed() && published.mode == 0);
  cmd::Center<1, 1, 1> failure;
  failure.input<Remote>("r");
  failure.output<Command>("out", [](const Command&) { return false; });
  failure.tick(1).publish();
  CHECK(!failure.last_publish_ok());
  return true;
}

bool TestControllerSemantics() {
  using Cascade = mr::comp::cntlr::cascade;
  auto c = Cascade::Build(2, 3, 4, 1, 10);
  auto first = c.StepOnce(2, 0, 0, 0.1f);
  CHECK(first.valid && Near(first.target_velocity, 0.1f));
  CHECK(!c.StepOnce(2, 0, 0, NAN).valid);
  auto second = c.StepOnce(2, 0, 0, 0.1f);
  CHECK(second.valid && Near(second.target_velocity, 0.2f));
  CHECK(!Cascade::Build(NAN, 1).StepOnce(1, 0, 0, 0.1f).valid);
  using Lqr = mr::comp::cntlr::lqr<1, 1, 2>;
  Lqr::Config cfg;
  cfg.clamp_schedule = false;
  cfg.coeff[0][0] = {2, 3};
  Lqr l(cfg);
  CHECK(l.UpdateSchedule(2));
  CHECK(Near(l.StepError({1}).output[0], -7));
  CHECK(!l.UpdateSchedule(std::numeric_limits<float>::max()));
  CHECK(!l.StepError({1}).valid);
  return true;
}
}  // namespace

int main() {
  if (!TestScalarInterface() || !TestObserverDimensions() || !TestCommandRouting() ||
      !TestCommandBoundaries() || !TestControllerSemantics())
    return 1;
  std::puts("C++ component tests passed (5 groups)");
  return 0;
}
