// 手写业务文件：修改这里，重新生成不会覆盖。
#include <array>
#include <cstring>
#include "bsp/adapters/can_port.hpp"
#include "bsp/time.h"
#include "robot_config.hpp"
#include "robot_tasks.hpp"
#include "../../device_execution/app/rm_chassis_cycle.hpp"

namespace {
using mr::module::chassis::ChassisStatus;

// 演示用调试器请求，初始关闭。实际项目在 ReadCommand 中接入带超时的命令快照。
// volatile 只供调试器使用，不是跨任务通信机制。
volatile bool request_run = false;
volatile uint32_t fault_count = 0;
volatile bool stop_failed = false;

class MotorPort final : public mr::motor::CanPort {
 public:
  mr::board::BspCanPort actual{MR_ROBOT_CAN_BINDING, BSP_TIME_Get_ms};
  uint32_t NowMs() const override { return actual.NowMs(); }
  int8_t RegisterLatest(uint32_t id) override { return actual.RegisterLatest(id); }
  int8_t RegisterQueue(uint32_t id, uint8_t n) override { return actual.RegisterQueue(id, n); }
  int8_t RegisterIdParser(mr::motor::CanIdParser p) override { return actual.RegisterIdParser(p); }
  int8_t ReadLatest(uint32_t id, mr::motor::CanFrame& frame) override { return actual.ReadLatest(id, frame); }
  int8_t ReadNext(uint32_t id, mr::motor::CanFrame& frame) override { return actual.ReadNext(id, frame); }
  int8_t Send(const mr::motor::CanFrame& frame) override {
    if constexpr (mr::generated::ChassisAssembly::kAllowMotorTx) return actual.Send(frame);
    (void)frame;
    return DEVICE_ERR;
  }
};

struct ChassisCommand {
  bool enabled = false;
  mr::robotics::chassis::PlanarVelocity velocity{};
};

class ChassisApplication {
 public:
  // 在任务内执行一次；这里不使能电机。
  bool Init() {
    if (BSP_FDCAN_Init() != BSP_OK) return false;
    return drive_.Register() == ChassisStatus::kOk;
  }

  // 每个周期执行一次：读命令 → 检查使能 → 反馈/控制/提交。
  // while、osDelayUntil、线程创建都不需要在这里重复编写。
  bool Step(float dt_s) {
    const auto command = ReadCommand();
    if (!command.enabled) return !running_ || Stop();
    if (!running_) {
      if (!HasFreshFeedback() || drive_.chassis.Enable() != ChassisStatus::kOk)
        return false;
      running_ = true;
    }
    return example::RunChassisCycle(drive_.chassis, drive_.commands, port_.NowMs(),
                                     command.velocity, dt_s).ok();
  }

  // 初始化/控制失败、周期超时、等待失败统一到这里；执行器随后退出任务。
  void Fault(mr::task::TaskFault) {
    ++fault_count;
    // Registration can have partially succeeded; always discard staged efforts
    // and attempt zero output, even when Init did not finish.
    if (!Stop()) stop_failed = true;
  }

 private:
  ChassisCommand ReadCommand() { return {request_run, {0.0f, 0.0f, 0.0f}}; }
  bool HasFreshFeedback() {
    mr::wheel::WheelFeedback feedback;
    const uint32_t now = port_.NowMs();
    for (auto* wheel : std::array<mr::wheel::WheelDevice*, 4>{
             &drive_.wheel_0, &drive_.wheel_1, &drive_.wheel_2, &drive_.wheel_3}) {
      if (wheel->UpdateFeedback(feedback) != DEVICE_OK || !feedback.online ||
          now - feedback.timestamp_ms > drive_.Config().feedback_timeout_ms)
        return false;
    }
    return true;
  }
  bool Stop() {
    running_ = false;
    drive_.commands.BeginCycle();
    const bool relaxed = drive_.chassis.Relax() == ChassisStatus::kOk;
    drive_.commands.BeginCycle();  // Flush zeros even if Relax failed.
    const bool sent = drive_.commands.Flush() == DEVICE_OK;
    return relaxed && sent;
  }
  MotorPort port_;
  mr::generated::ChassisAssembly drive_{port_};
  bool running_ = false;
};

ChassisApplication chassis;
}  // namespace

// YAML 中 app.chassis_cycle 对应的明确绑定。拼错入口会使创建失败。
namespace mr::app {
mr::task::TaskHooks BindPeriodicTask(const char* entry) {
  if (std::strcmp(entry, "app.chassis_cycle") != 0) return {};
  return {&chassis,
          [](void* p) { return static_cast<ChassisApplication*>(p)->Init(); },
          [](void* p, float dt_s) { return static_cast<ChassisApplication*>(p)->Step(dt_s); },
          [](void* p, mr::task::TaskFault reason) { static_cast<ChassisApplication*>(p)->Fault(reason); }};
}
}  // namespace mr::app
