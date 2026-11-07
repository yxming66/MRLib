// Deterministic CMSIS call-contract test; does not simulate FreeRTOS scheduling.
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include "robot_tasks.h"
#include "robot_tasks.hpp"

namespace {
struct Exit {};
osKernelState_t kernel = osKernelReady;
uint32_t tick = 0, hz = 1000, period = 0, stack_bytes = 0;
osThreadFunc_t entry = nullptr;
void* argument = nullptr;
bool fail_create = false, fail_init = false, overrun = false, fail_delay = false;
bool bad_binding = false;
int creates = 0, steps = 0, inits = 0, faults = 0, waits = 0;
mr::task::TaskFault last_fault{};
bool Init(void*) { ++inits; return !fail_init; }
bool Step(void*, float dt) {
  assert(std::fabs(dt - period / 1000000.0f) < 1e-8f);
  ++steps;
  if (overrun) tick += period / 1000u;
  return steps < 3;
}
void Fault(void*, mr::task::TaskFault reason) { ++faults; last_fault = reason; }
void Run(mr::task::TaskFault expected, int expected_steps) {
  steps = inits = faults = waits = 0;
  tick = 0xfffffffeu;  // Includes tick counter rollover.
  try { entry(argument); assert(false); } catch (const Exit&) {}
  assert(inits == 1 && steps == expected_steps && faults == 1);
  assert(last_fault == expected);
}
}

osKernelState_t osKernelGetState() { return kernel; }
uint32_t osKernelGetTickFreq() { return hz; }
uint32_t osKernelGetTickCount() { return tick; }
osThreadId_t osThreadNew(osThreadFunc_t fn, void* context, const osThreadAttr_t* attr) {
  ++creates;
  assert(std::strcmp(attr->name, "chassis") == 0);
  assert(attr->stack_size == stack_bytes && attr->priority == osPriorityNormal);
  assert(attr->stack_mem == nullptr && attr->cb_mem == nullptr);
  if (fail_create) return nullptr;
  entry = fn; argument = context;
  return reinterpret_cast<osThreadId_t>(1);
}
osStatus_t osDelay(uint32_t) { assert(false); return osError; }
osStatus_t osDelayUntil(uint32_t target) {
  ++waits;
  assert(target - tick == period / 1000u);
  if (fail_delay) return osError;
  tick = target;
  return osOK;
}
void osThreadExit() { throw Exit{}; }

namespace mr::app {
mr::task::TaskHooks BindPeriodicTask(const char* name) {
  assert(std::strcmp(name, "app.chassis_cycle") == 0);
  return bad_binding ? mr::task::TaskHooks{} : mr::task::TaskHooks{nullptr, Init, Step, Fault};
}
}

int main(int argc, char** argv) {
  assert(argc == 3);
  period = static_cast<uint32_t>(std::strtoul(argv[1], nullptr, 10));
  stack_bytes = static_cast<uint32_t>(std::strtoul(argv[2], nullptr, 10));
  assert(mr::generated::kTaskConfig.period_us == period);
  kernel = osKernelRunning;
  assert(!MR_CreateConfiguredTask() && creates == 0);
  kernel = osKernelReady;
  bad_binding = true;
  assert(!MR_CreateConfiguredTask() && creates == 0);
  bad_binding = false;
  hz = 333;  // 2/4 ms cannot be represented at this tick frequency.
  assert(!MR_CreateConfiguredTask() && creates == 0);
  hz = 1000;
  fail_create = true;
  assert(!MR_CreateConfiguredTask() && creates == 1);
  fail_create = false;
  assert(MR_CreateConfiguredTask() && creates == 2);
  assert(!MR_CreateConfiguredTask() && creates == 2);
  assert(inits == 0 && steps == 0);  // Creation never runs application hooks.

  Run(mr::task::TaskFault::kStepFailed, 3);
  assert(waits == 2);
  fail_init = true;
  Run(mr::task::TaskFault::kInitFailed, 0);
  assert(waits == 0);
  fail_init = false;
  overrun = true;
  Run(mr::task::TaskFault::kDeadlineMissed, 1);
  assert(waits == 0);  // No catch-up loops or additional nonzero control output.
  overrun = false;
  fail_delay = true;
  Run(mr::task::TaskFault::kDelayFailed, 1);
}
