#pragma once
#include "task/periodic_task.hpp"

namespace chassis_example {
// Project parameters, edited directly. Units are microseconds and bytes.
inline constexpr mr::task::PeriodicTaskConfig kTaskConfig{
    "chassis", 2000, 4096, osPriorityNormal};
}  // namespace chassis_example
