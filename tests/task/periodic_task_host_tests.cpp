// Deterministic CMSIS call-contract test; does not simulate FreeRTOS scheduling.
#include <cassert>
#include <cmath>
#include <cstring>
#include "task/periodic_task.hpp"

namespace {
struct Exit {};
osKernelState_t kernel = osKernelReady;
uint32_t tick = 0, hz = 1000, period = 0, stack_bytes = 0;
osThreadFunc_t entry = nullptr;
void* argument = nullptr;
bool fail_create = false, fail_init = false, overrun = false, fail_delay = false;
alignas(8) unsigned char task_stack[4096];
alignas(8) unsigned char task_control[128];
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
  assert(attr->stack_mem == task_stack && attr->cb_mem == task_control);
  assert(attr->cb_size == sizeof(task_control));
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

int main() {
  period = 2000;
  stack_bytes = sizeof(task_stack);
  const mr::task::PeriodicTaskConfig config{"chassis", period, stack_bytes, osPriorityNormal};
  const mr::task::TaskHooks hooks{nullptr, Init, Step, Fault};
  osThreadAttr_t storage{};
  storage.cb_mem = task_control;
  storage.cb_size = sizeof(task_control);
  storage.stack_mem = task_stack;
  storage.stack_size = sizeof(task_stack);
  mr::task::PeriodicTask task;
  kernel = osKernelRunning;
  assert(!task.Create(config, hooks, storage) && creates == 0);
  kernel = osKernelReady;
  assert(!task.Create(config, {}, storage) && creates == 0);
  assert(!task.Create(config, hooks, {}) && creates == 0);  // Never falls back to heap.
  auto small_stack = storage;
  small_stack.stack_size -= 8;
  assert(!task.Create(config, hooks, small_stack) && creates == 0);
  auto unaligned = storage;
  unaligned.stack_mem = task_stack + 1;
  assert(!task.Create(config, hooks, unaligned) && creates == 0);
  hz = 333;
  assert(!task.Create(config, hooks, storage) && creates == 0);
  hz = 1000;
  fail_create = true;
  assert(!task.Create(config, hooks, storage) && creates == 1);
  fail_create = false;
  assert(task.Create(config, hooks, storage) && creates == 2);
  assert(!task.Create(config, hooks, storage) && creates == 2);
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
