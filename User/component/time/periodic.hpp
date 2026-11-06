#pragma once
#include <cstdint>

namespace mr::component::time {

/* Poll at least once per half uint32 range. Units are chosen by the caller
 * (ticks, us, ms) and must be consistent. Missed periods are skipped in O(1).
 * Configure with due_now=true preserves the old task helper's first-call rule.
 */
class Periodic {
 public:
  bool Configure(uint32_t period, uint32_t now, bool due_now = true) {
    valid_ = period > 0 && period <= INT32_MAX;
    period_ = period;
    next_ = due_now ? now : now + period;
    return valid_;
  }
  bool Poll(uint32_t now) {
    if (!valid_ || now - next_ > INT32_MAX) return false;
    next_ += ((now - next_) / period_ + 1U) * period_;
    return true;
  }
  uint32_t next() const { return next_; }
 private:
  uint32_t period_ = 0, next_ = 0;
  bool valid_ = false;
};
}  // namespace mr::component::time
