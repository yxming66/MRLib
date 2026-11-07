#include "../../device_execution/app/rm_chassis_cycle.hpp"
#include "../../mrlink/board/h7_dma_policy.hpp"
#include "../../mrlink/board/uart_link.hpp"
#include "bsp/adapters/can_port.hpp"
#include "bsp/time.h"
#include "cmsis_os2.h"
#include "component/time/periodic.hpp"
#include "device/joint/joint.hpp"
#include "device/joint/motor_actuator_adapter.hpp"
#include "device/motor/bus/rm_command_bus.hpp"
#include "device/wheel/wheel.hpp"
#include "module/arm/arm.hpp"
#include "module/chassis/chassis.hpp"

namespace {
using namespace mr::motor;
using mr::module::chassis::ChassisStatus;
using Arm = mr::module::arm::Arm<3>;

// Development request only: no remote message can enable this example.
// The default build rejects motor TX at the port as well as keeping Enable off.
volatile bool request_run = false;
volatile uint32_t cycles = 0, faults = 0, tx_rejected = 0, link_tx_failed = 0;
volatile int8_t last_error = 0;
class MotorPort final : public CanPort {
 public:
  mr::board::BspCanPort actual{BSP_CAN_1, BSP_TIME_Get_ms};
  uint32_t NowMs() const override { return actual.NowMs(); }
  int8_t RegisterLatest(uint32_t id) override { return actual.RegisterLatest(id); }
  int8_t RegisterQueue(uint32_t id, uint8_t n) override { return actual.RegisterQueue(id, n); }
  int8_t RegisterIdParser(CanIdParser p) override { return actual.RegisterIdParser(p); }
  int8_t ReadLatest(uint32_t id, CanFrame& f) override { return actual.ReadLatest(id, f); }
  int8_t ReadNext(uint32_t id, CanFrame& f) override { return actual.ReadNext(id, f); }
  int8_t Send(const CanFrame& f) override {
#if MR_H7_ALLOW_MOTOR_TX
    return actual.Send(f);
#else
    (void)f;
    ++tx_rejected;
    return DEVICE_ERR;
#endif
  }
};

struct Robot {
  MotorPort port;
  RmCommandBus rm{port};
  static MotorInstanceConfig<MotorKind::RM> WheelConfig(Robot& r, uint16_t id) {
    MotorInstanceConfig<MotorKind::RM> c;
    c.port = &r.port;
    c.commands = &r.rm;
    c.id = id;
    return c;
  }
  static MotorInstanceConfig<MotorKind::DM> JointConfig(Robot& r, uint16_t id) {
    MotorInstanceConfig<MotorKind::DM> c;
    c.port = &r.port;
    c.can_id = id;
    c.master_id = 0x310 + id;
    c.mit_position_range_rad = 12.5f;
    c.mit_velocity_range_rad_s = 30;
    c.mit_torque_range_nm = 10;
    return c;
  }
  RmM3508Motor m0{WheelConfig(*this, 0x201)}, m1{WheelConfig(*this, 0x202)},
      m2{WheelConfig(*this, 0x203)}, m3{WheelConfig(*this, 0x204)};
  mr::wheel::MotorWheel<RmM3508Motor> w0{m0, .1f}, w1{m1, .1f}, w2{m2, .1f}, w3{m3, .1f};
  DmJ4310Motor d0{JointConfig(*this, 1)}, d1{JointConfig(*this, 2)}, d2{JointConfig(*this, 3)};
  mr::DmJ4310Actuator a0{&d0}, a1{&d1}, a2{&d2};
  mr::JointT j0{0, &a0, {-1, 1, 0, 0}, 0, 1000}, j1{1, &a1, {-1, 1, 0, 0}, 0, 1000},
      j2{2, &a2, {-1, 1, 0, 0}, 0, 1000};
  mr::module::chassis::Chassis chassis;
  Arm arm;
  board::ChannelAndLink telemetry;
  bool running = false, previous_request = false, fault = false;

  bool Init() {
    mr::module::chassis::ChassisConfig c;
    c.kinematics.model = mr::robotics::chassis::ChassisModel::kMecanum;
    c.kinematics.geometry = {.4f, .4f};
    c.max_body_velocity = {2, 2, 2};
    c.max_wheel_speed_mps = 3;
    c.wheel_velocity_pid.gains = {1, 1, 0, 0};
    c.wheel_velocity_pid.output_limit = mr::comp::cntlr::SymmetricLimit::Abs(2);
    c.wheel_velocity_pid.control_freq_hz = 1000;
    if (chassis.Register(c, {&w0, &w1, &w2, &w3}) != ChassisStatus::kOk) return false;
    // Representative chain only. No zzrobot calibration or mass model is implied.
    mr::robotics::arm::JointModel joints[3];
    for (int i = 0; i < 3; ++i) {
      joints[i].limits.max_velocity = 1;
      joints[i].limits.max_acceleration = 2;
      joints[i].limits.position_enabled = true;
      joints[i].limits.position_min = -1;
      joints[i].limits.position_max = 1;
      joints[i].parent_to_joint[0][3] = .1f;
    }
    using mr::component::math::matrix::identity;
    Arm::Config cfg;
    cfg.model = {joints, identity<4>(), identity<4>(), mr::component::math::matrix::zeros<3, 1>()};
    cfg.output.actuators = {&j0, &j1, &j2};
    for (auto& gain : cfg.output.gains) gain = {5, .2f, 0};
    if (arm.Init(cfg) != DEVICE_OK) return false;
    return telemetry.Begin(BSP_UART_PC) == MRLINK_OK;
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
    for (auto* wheel : std::array<mr::wheel::WheelDevice*, 4>{&w0, &w1, &w2, &w3})
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
      const auto result = example::RunChassisCycle(chassis, rm, now, {}, .001f);
      if (!result.ok()) {
        last_error = DEVICE_ERR;
        fault = true;
        ++faults;
        Stop();
        return;
      }
      int8_t status = arm.UpdateFeedback();
      if (status == DEVICE_OK) status = arm.Step(.001f);  // Hold captured at Enable.
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
Robot robot;
const board::DmaRegion regions[] = {{0x24000000, 0x24050000, false}};
board::DmaPolicy dma_policy{regions, 1};
}  // namespace

extern "C" void RobotTask(void*) {
  BSP_DMA_Configure(board::SynchronizeDma, &dma_policy);
  if (osKernelGetTickFreq() != 1000u || BSP_FDCAN_Init() != BSP_OK || !robot.Init()) {
    last_error = DEVICE_ERR;
    for (;;) (void)osDelay(1000);
  }
  mr::component::time::Periodic telemetry_period;
  telemetry_period.Configure(100, BSP_TIME_Get_ms(), false);
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
    if (osDelayUntil(++next) != osOK) {
      ++faults;
      robot.Stop();
      next = osKernelGetTickCount();
    }
  }
}
