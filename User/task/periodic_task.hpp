#pragma once

#include <cstdint>
#include "cmsis_os2.h"

namespace mr::task {
// CMSIS-RTOS2 task shell, used with the project's FreeRTOS kernel.
// Application callbacks run only in this task. They must not throw or block
// beyond the control period. All borrowed strings/context must outlive it.
enum class TaskFault { kInitFailed, kStepFailed, kDeadlineMissed, kDelayFailed };
struct TaskHooks {
  void* context = nullptr;
  bool (*init)(void*) = nullptr;
  bool (*step)(void*, float dt_s) = nullptr;
  void (*fault)(void*, TaskFault) = nullptr;
};
struct PeriodicTaskConfig {
  const char* name;
  uint32_t period_us;
  uint32_t stack_bytes;  // CMSIS uses bytes, not FreeRTOS StackType_t words.
  osPriority_t priority;
};

class PeriodicTask final {
 public:
  PeriodicTask() = default;
  PeriodicTask(const PeriodicTask&) = delete;
  PeriodicTask& operator=(const PeriodicTask&) = delete;

  // 在内核 Ready 或 Running 状态，由单一创建者在任务上下文调用一次。
  // Running 时新线程可能在 Create 返回前运行，所有依赖必须先准备好；
  // 不依赖调度器启动前的隐式屏障。对象、回调上下文和静态存储须持续有效。
  // 项目提供静态控制块与栈，不回退到堆分配，不支持并发 Create。
  bool Create(const PeriodicTaskConfig& config, TaskHooks hooks,
              const osThreadAttr_t& storage) {
    const auto state = osKernelGetState();
    if (created_ || (state != osKernelReady && state != osKernelRunning) || !config.name ||
        !config.name[0] || !config.period_us || config.stack_bytes < 512u ||
        config.stack_bytes % 8u || !hooks.init || !hooks.step || !hooks.fault ||
        !storage.cb_mem || !storage.cb_size || !storage.stack_mem ||
        storage.stack_size < config.stack_bytes ||
        reinterpret_cast<uintptr_t>(storage.stack_mem) % 8u)
      return false;
    const uint64_t scaled = uint64_t{config.period_us} * osKernelGetTickFreq();
    if (!scaled || scaled % 1000000u || scaled / 1000000u >= 0x80000000u)
      return false;  // Reject sub-tick and rounded periods.
    period_ticks_ = static_cast<uint32_t>(scaled / 1000000u);
    dt_s_ = static_cast<float>(config.period_us) / 1000000.0f;
    hooks_ = hooks;
    osThreadAttr_t attr{};
    attr.name = config.name;
    attr.stack_size = config.stack_bytes;
    attr.priority = config.priority;
    attr.cb_mem = storage.cb_mem;
    attr.cb_size = storage.cb_size;
    attr.stack_mem = storage.stack_mem;
    // 在发布线程入口前完成配置；新线程可立即抢占创建者。
    created_ = true;
    handle_ = osThreadNew(Entry, this, &attr);
    created_ = handle_ != nullptr;
    return created_;
  }

 private:
  static void Entry(void* context) { static_cast<PeriodicTask*>(context)->Run(); }
  [[noreturn]] void Fail(TaskFault reason) {
    hooks_.fault(hooks_.context, reason);  // App relaxes outputs and latches fault.
    osThreadExit();  // No automatic restart or replay of missed control cycles.
    for (;;) (void)osDelay(1u);  // Defensive fallback if a nonconforming port returns.
  }
  void Run() {
    if (!hooks_.init(hooks_.context)) Fail(TaskFault::kInitFailed);
    uint32_t next = osKernelGetTickCount();
    for (;;) {
      next += period_ticks_;
      if (!hooks_.step(hooks_.context, dt_s_)) Fail(TaskFault::kStepFailed);
      const uint32_t remaining = next - osKernelGetTickCount();
      if (!remaining || remaining >= 0x80000000u) Fail(TaskFault::kDeadlineMissed);
      if (osDelayUntil(next) != osOK) Fail(TaskFault::kDelayFailed);
    }
  }
  TaskHooks hooks_{};
  osThreadId_t handle_ = nullptr;
  uint32_t period_ticks_ = 0;
  float dt_s_ = 0;
  bool created_ = false;
};
}  // namespace mr::task
