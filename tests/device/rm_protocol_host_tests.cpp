#include <array>
#include <cmath>
#include <cstdio>
#include <limits>

#include "device/device.h"
#include "device/motor/protocol/rm_protocol.hpp"

namespace {

struct FakeCanBus {
  BSP_CAN_StdDataFrame_t last_standard_frame{};
  unsigned register_latest_count = 0U;
  unsigned standard_transmit_count = 0U;
  bool fail_next_transmit = false;
};

std::array<FakeCanBus, BSP_CAN_NUM> g_fake_buses{};

int failures = 0;

void Expect(bool condition, const char* message) {
  if (!condition) {
    std::printf("FAIL: %s\n", message);
    ++failures;
  }
}

using RmM3508Protocol =
    mr::motor::MotorProtocol<mr::motor::MotorKind::RM,
                             mr::motor::MotorModel::M3508>;

RmM3508Protocol MakeProtocol(BSP_CAN_t bus, uint16_t id,
                             mr::motor::MotorState* state) {
  mr::motor::MotorInstanceConfig<mr::motor::MotorKind::RM> config{};
  config.can = bus;
  config.id = id;
  return RmM3508Protocol(config, mr::motor::kDirectDriveInstall, *state,
                         mr::motor::RmModule::M3508);
}

void TestRelaxSendsZeroAndRetriesFailedSend() {
  mr::motor::MotorState state{};
  auto protocol = MakeProtocol(BSP_CAN_1, 0x202U, &state);
  Expect(protocol.Register() == DEVICE_OK,
         "valid RM motor should register with the fake CAN bus");
  Expect(protocol.Enable() == DEVICE_OK,
         "RM protocol should enter enabled state before relax test");
  Expect(protocol.SetTorque(1.0f) == DEVICE_OK &&
             protocol.CommitCommand() == DEVICE_OK,
         "test should first establish a nonzero motor current command");
  const auto& active_frame = g_fake_buses[BSP_CAN_1].last_standard_frame;
  Expect(active_frame.data[2] != 0U || active_frame.data[3] != 0U,
         "the selected motor slot should contain nonzero current before relax");

  g_fake_buses[BSP_CAN_1].fail_next_transmit = true;
  Expect(protocol.Relax() == BSP_ERR,
         "failed zero-current frame should be reported");
  Expect(protocol.HasPendingCommand(),
         "failed zero-current frame should remain pending for retry");
  Expect(state.protocol_state == mr::motor::MotorProtocolState::Enabled,
         "failed relax must not claim the motor is relaxed");

  const auto& frame = g_fake_buses[BSP_CAN_1].last_standard_frame;
  Expect(frame.id == 0x200U && frame.dlc == 8U,
         "active and relax attempts should use the matching 8-byte RM group");
  Expect(frame.data[2] != 0U || frame.data[3] != 0U,
         "failed relax transmission should leave last successful output intact");
  Expect(g_fake_buses[BSP_CAN_1].standard_transmit_count == 2U,
         "test should attempt active output and failed relax transmission");
  Expect(protocol.CommitCommand() == DEVICE_OK,
         "pending zero-current frame should retry successfully");
  const auto& relaxed_frame = g_fake_buses[BSP_CAN_1].last_standard_frame;
  Expect(relaxed_frame.data[2] == 0U && relaxed_frame.data[3] == 0U,
         "successful retry should zero the selected motor slot");
  Expect(!protocol.HasPendingCommand(),
         "successful retry should clear the pending zero command");
  Expect(state.protocol_state == mr::motor::MotorProtocolState::Relaxed,
         "successful retry should commit the relaxed state");
}

void TestFailedCommandDoesNotLeakIntoAnotherSlotCommit() {
  mr::motor::MotorState failed_state{};
  auto failed_protocol = MakeProtocol(BSP_CAN_2, 0x204U, &failed_state);
  Expect(failed_protocol.Register() == DEVICE_OK,
         "first motor on the isolated fake bus should register");
  Expect(failed_protocol.SetTorque(1.0f) == DEVICE_OK,
         "finite torque command should be staged");
  g_fake_buses[BSP_CAN_2].fail_next_transmit = true;
  Expect(failed_protocol.CommitCommand() == BSP_ERR,
         "failed torque transmission should be reported");
  Expect(failed_protocol.HasPendingCommand(),
         "failed torque transmission should remain pending");

  mr::motor::MotorState other_state{};
  auto other_protocol = MakeProtocol(BSP_CAN_2, 0x201U, &other_state);
  Expect(other_protocol.Register() == DEVICE_OK,
         "second motor on the isolated fake bus should register");
  Expect(other_protocol.Relax() == DEVICE_OK,
         "second motor zero command should transmit successfully");
  const auto& other_frame = g_fake_buses[BSP_CAN_2].last_standard_frame;
  Expect(other_frame.id == 0x200U,
         "second motor should transmit through the shared command group");
  Expect(other_frame.data[6] == 0U && other_frame.data[7] == 0U,
         "failed slot update must not leak into another motor's frame");

  Expect(failed_protocol.CommitCommand() == DEVICE_OK,
         "failed motor command should still be explicitly retryable");
  const auto& retried_frame = g_fake_buses[BSP_CAN_2].last_standard_frame;
  Expect(retried_frame.data[6] != 0U || retried_frame.data[7] != 0U,
         "explicit retry should transmit the staged nonzero torque");
}

void TestNonFiniteTorqueDoesNotReplacePendingCommand() {
  mr::motor::MotorState state{};
  auto protocol = MakeProtocol(BSP_CAN_1, 0x203U, &state);
  Expect(protocol.SetTorque(std::numeric_limits<float>::quiet_NaN()) ==
             DEVICE_ERR,
         "NaN torque should be rejected");
  Expect(!protocol.HasPendingCommand(),
         "rejected NaN torque should not create a pending command");
  Expect(protocol.SetTorque(std::numeric_limits<float>::infinity()) ==
             DEVICE_ERR,
         "infinite torque should be rejected");
  Expect(!protocol.HasPendingCommand(),
         "rejected infinite torque should not create a pending command");
}

void TestInvalidIdIsRejectedAtRegistration() {
  mr::motor::MotorState state{};
  auto protocol = MakeProtocol(BSP_CAN_1, 0x209U, &state);
  const unsigned register_count =
      g_fake_buses[BSP_CAN_1].register_latest_count;
  Expect(protocol.Register() == DEVICE_ERR,
         "unsupported M3508 feedback ID should fail before BSP registration");
  Expect(g_fake_buses[BSP_CAN_1].register_latest_count == register_count,
         "invalid ID must fail before registering a BSP feedback slot");
}

}  // namespace

extern "C" {

int8_t BSP_CAN_RegisterLatestId(BSP_CAN_t bus, uint32_t) {
  if (bus >= BSP_CAN_NUM) {
    return BSP_ERR;
  }
  ++g_fake_buses[bus].register_latest_count;
  return BSP_OK;
}
int8_t BSP_CAN_RegisterId(BSP_CAN_t, uint32_t, uint8_t) { return BSP_OK; }
int8_t BSP_CAN_RegisterIdParser(BSP_CAN_IdParser_t) { return BSP_OK; }
int8_t BSP_CAN_GetLatestMessage(BSP_CAN_t, uint32_t,
                                BSP_CAN_Message_t*) {
  return BSP_ERR_NO_DEV;
}
int8_t BSP_CAN_GetMessage(BSP_CAN_t, uint32_t, BSP_CAN_Message_t*, uint32_t) {
  return BSP_ERR_NO_DEV;
}
int8_t BSP_CAN_TransmitStdDataFrame(BSP_CAN_t bus,
                                    const BSP_CAN_StdDataFrame_t* frame) {
  if (bus >= BSP_CAN_NUM || frame == nullptr) {
    return BSP_ERR;
  }
  FakeCanBus& fake = g_fake_buses[bus];
  ++fake.standard_transmit_count;
  if (fake.fail_next_transmit) {
    fake.fail_next_transmit = false;
    return BSP_ERR;
  }
  fake.last_standard_frame = *frame;
  return BSP_OK;
}
int8_t BSP_CAN_TransmitExtDataFrame(BSP_CAN_t,
                                    const BSP_CAN_ExtDataFrame_t*) {
  return BSP_ERR;
}

}  // extern "C"

int main() {
  TestRelaxSendsZeroAndRetriesFailedSend();
  TestFailedCommandDoesNotLeakIntoAnotherSlotCommit();
  TestNonFiniteTorqueDoesNotReplacePendingCommand();
  TestInvalidIdIsRejectedAtRegistration();
  if (failures != 0) {
    std::printf("%d RM protocol host test(s) failed\n", failures);
    return 1;
  }
  std::printf("RM protocol host tests passed\n");
  return 0;
}
