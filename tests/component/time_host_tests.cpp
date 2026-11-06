#include "component/timer.hpp"
#include "component/time/periodic.hpp"
#include <cstdio>
#include <limits>

#define CHECK(x) do { if (!(x)) { std::fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #x); return false; } } while (0)
static uint64_t now_us;
static unsigned clock_calls;
static uint64_t Clock() { ++clock_calls; return now_us; }
static bool Timer() {
  mr::comp::timer unbound;
  CHECK(!unbound.ready() && !unbound.started());
  CHECK(unbound.UpdateUs() == 0 && !unbound.EveryUs(100));
  unbound.Reset();
  CHECK(!unbound.started() && !unbound.ExpiredUs(1));
  mr::comp::timer_config config{};
  config.clock_us = Clock; config.first_dt_us = 1000;
  config.min_dt_us = 100; config.max_dt_us = 2000;
  auto timer = mr::comp::timer::Build(config);
  CHECK(clock_calls == 0 && timer.ready());
  now_us = 5000;
  CHECK(timer.UpdateUs() == 1000);
  now_us += 20;
  CHECK(timer.UpdateUs() == 100);
  now_us += 4000;
  CHECK(timer.UpdateUs() == 2000);
  CHECK(timer.ElapsedUs() == 4020 && timer.SinceUpdateUs() == 0);
  now_us += 500;
  CHECK(timer.SinceUpdateUs() == 500);
  timer.Configure(config);
  CHECK(!timer.started());
  now_us = 10000;
  CHECK(!timer.EveryUs(1000));
  now_us = 13500;
  CHECK(timer.EveryUs(1000));
  CHECK(!timer.EveryUs(1000));
  now_us = 14000;
  CHECK(timer.EveryUs(1000)); // Keeps phase after skipped periods.
  CHECK(!timer.EveryUs(0));
  timer.ResetPeriod();
  now_us += 1000;
  CHECK(timer.ExpiredUs(1000));
  CHECK(mr::comp::timer::SecondsToUs(-1) == 0);
  CHECK(mr::comp::timer::SecondsToUs(std::numeric_limits<float>::quiet_NaN()) == 0);
  CHECK(mr::comp::timer::UsToMs(UINT64_MAX) == UINT32_MAX);
  return true;
}
static bool Periodic() {
  mr::component::time::Periodic periodic;
  CHECK(!periodic.Poll(0));
  CHECK(!periodic.Configure(0, 0));
  CHECK(!periodic.Configure(UINT32_MAX, 0));
  CHECK(periodic.Configure(10, 100));
  CHECK(periodic.Poll(100) && !periodic.Poll(100));
  CHECK(!periodic.Poll(109));
  CHECK(periodic.Poll(145) && periodic.next() == 150);
  CHECK(!periodic.Poll(149) && periodic.Poll(150));
  CHECK(periodic.Configure(10, UINT32_MAX - 5, false));
  CHECK(!periodic.Poll(UINT32_MAX) && !periodic.Poll(3));
  CHECK(periodic.Poll(4) && periodic.next() == 14);
  return true;
}
int main() {
  if (!Timer() || !Periodic()) return 1;
  std::puts("Time tests passed (injected clock, dt limits, skipped periods, uint32 wrap)");
}
