#include "rm_chassis_cycle.hpp"
#include "h7_dma_policy.hpp"
#include "uart_link.hpp"
#include "bsp/adapters/can_port.hpp"
#include "bsp/time.h"
#include "cmsis_os2.h"
#include "component/time/periodic.hpp"
#include "device/motor/bus/rm_command_bus.hpp"
#include "device/wheel/wheel.hpp"
#include "module/arm/arm.hpp"
#include "module/chassis/chassis.hpp"
#include "arm_assembly.hpp"
#include "chassis_assembly.hpp"
#include "../board/bindings.hpp"

namespace {
using namespace mr::motor;
using mr::module::chassis::ChassisStatus;
using Arm = app::ArmAssembly::ArmType;
constexpr float kDtSeconds = app::params::kControlPeriodUs / 1000000.0f;

// Development request only: no remote message can enable this example.
// The default build rejects motor TX at the port as well as keeping Enable off.
volatile bool request_run = false;
volatile uint32_t cycles = 0, faults = 0, tx_rejected = 0, link_tx_failed = 0;
volatile int8_t last_error = 0;
class MotorPort final : public CanPort {
 public:
  mr::board::BspCanPort actual{BSP_CAN_FindEndpoint(board::bindings::kMotorCan), BSP_TIME_Get_ms};
  uint32_t NowMs() const override { return actual.NowMs(); }
  int8_t RegisterLatest(uint32_t id) override { return actual.RegisterLatest(id); }
  int8_t RegisterQueue(uint32_t id, uint8_t n) override { return actual.RegisterQueue(id, n); }
  int8_t RegisterIdParser(CanIdParser p) override { return actual.RegisterIdParser(p); }
  int8_t ReadLatest(uint32_t id, CanFrame& f) override { return actual.ReadLatest(id, f); }
  int8_t ReadNext(uint32_t id, CanFrame& f) override { return actual.ReadNext(id, f); }
  int8_t Send(const CanFrame& f) override {
    if constexpr (app::params::kAllowMotorTx) return actual.Send(f);
    (void)f;
    ++tx_rejected;
    return DEVICE_ERR;
  }
};

struct Robot {
  MotorPort port;
  app::ChassisAssembly drive{port};
  app::ArmAssembly manipulator{port};
  RmCommandBus& rm = drive.commands;
  mr::module::chassis::Chassis& chassis = drive.chassis;
  Arm& arm = manipulator.arm;
  board::ChannelAndLink telemetry;
  bool running = false, previous_request = false, fault = false;

  bool Init() {
    if (drive.Register(app::params::kControlPeriodUs) != ChassisStatus::kOk) return false;
    if (manipulator.Init() != DEVICE_OK) return false;
    return telemetry.Begin(BSP_UART_FindEndpoint(board::bindings::kTelemetryUart)) == MRLINK_OK;
  }
  void Stop() {
    running = false;
    // Board TX errors are recorded; retries use zero/disable commands only.
    rm.BeginCycle();
    const bool chassis_ok = chassis.Relax() == ChassisStatus::kOk;
    const bool arm_ok = arm.Disable() == DEVICE_OK;
    const bool flush_ok = rm.Flush() == DEVICE_OK;
    if (!chassis_ok || !arm_ok || !flush_ok) last_error = DEVICE_ERR;
  }
  bool Start() {
    // Reuse the same fresh-feedback clock as BSP reception. Do not fabricate it.
    mr::wheel::WheelFeedback feedback;
    for (auto* wheel : drive.Wheels())
      if (wheel->UpdateFeedback(feedback) != DEVICE_OK || !feedback.online) return false;
    if (arm.UpdateFeedback() != DEVICE_OK || arm.Enable() != DEVICE_OK) return false;
    return chassis.Enable() == ChassisStatus::kOk;
  }
  void Tick(uint32_t now) {
    (void)telemetry.link.Poll();
    const bool requested = request_run;
    if (!requested && running) Stop();
    if (requested && !previous_request) {
      fault = false;
      running = Start();
      if (!running) {
        fault = true;
        ++faults;
        Stop();
      }
    }
    previous_request = requested;
    if (running && !fault) {
      const auto result = example::RunChassisCycle(chassis, rm, now, {}, kDtSeconds);
      if (!result.ok()) {
        last_error = DEVICE_ERR;
        fault = true;
        ++faults;
        Stop();
        return;
      }
      int8_t status = arm.UpdateFeedback();
      if (status == DEVICE_OK) status = arm.Step(kDtSeconds);  // Hold captured at Enable.
      if (status == DEVICE_OK) status = arm.CommitOutput();
      if (status != DEVICE_OK) {
        last_error = status;
        fault = true;
        ++faults;
        Stop();
      }
    }
    ++cycles;
  }
};

// Static lifetime and AXI SRAM placement (including UART RX/TX owned buffers).
const board::DmaRegion regions[] = {{0x24000000, 0x24050000, false}};
board::DmaPolicy dma_policy{regions, 1};
}  // namespace

extern "C" void RobotTask(void*) {
  static Robot robot; // Construct after board endpoint registration, fixed storage.
  static_assert(app::params::kControlPeriodUs % 1000u == 0u,
                "Example scheduler requires whole milliseconds at 1000 Hz");
  constexpr uint32_t kPeriodTicks = app::params::kControlPeriodUs / 1000u;
  BSP_DMA_Configure(board::SynchronizeDma, &dma_policy);
  if (osKernelGetTickFreq() != 1000u || BSP_FDCAN_Init() != BSP_OK || !robot.Init()) {
    last_error = DEVICE_ERR;
    for (;;) (void)osDelay(1000);
  }
  mr::component::time::Periodic telemetry_period;
  telemetry_period.Configure(app::params::kTelemetryPeriodMs, BSP_TIME_Get_ms(), false);
  uint16_t sequence = 0;
  uint32_t next = osKernelGetTickCount();
  for (;;) {
    const uint32_t now = BSP_TIME_Get_ms();
    robot.Tick(now);
    if (telemetry_period.Poll(now)) {
      const app::Telemetry sample{sequence++, robot.arm.feedback().position[0][0]};
      if (!robot.telemetry.link.Publish(sample)) ++link_tx_failed;
    }
    // This Cube config uses 1000 Hz; delay-until keeps the control period stable.
    next += kPeriodTicks;
    if (osDelayUntil(next) != osOK) {
      ++faults;
      robot.Stop();
      next = osKernelGetTickCount();
    }
  }
}
