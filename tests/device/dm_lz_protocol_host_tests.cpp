#include <array>
#include <cmath>
#include <cstdio>
#include <limits>

#include "device/device.h"
#include "device/motor/protocol/dm_protocol.hpp"
#include "device/motor/protocol/lz_protocol.hpp"

namespace {

struct FakeCanBus {
  BSP_CAN_StdDataFrame_t last_standard_frame{};
  BSP_CAN_ExtDataFrame_t last_extended_frame{};
  uint32_t latest_id = 0U;
  uint32_t queue_id = 0U;
  unsigned register_latest_count = 0U;
  unsigned register_queue_count = 0U;
  unsigned parser_register_count = 0U;
  unsigned standard_transmit_count = 0U;
  unsigned extended_transmit_count = 0U;
  bool fail_next_standard = false;
  bool fail_next_extended = false;
};

std::array<FakeCanBus, BSP_CAN_NUM> g_fake_buses{};
int failures = 0;

void Expect(bool condition, const char* message) {
  if (!condition) {
    std::printf("FAIL: %s\n", message);
    ++failures;
  }
}

using DmProtocol = mr::motor::MotorProtocol<mr::motor::MotorKind::DM,
                                             mr::motor::MotorModel::J4310>;
using LzProtocol = mr::motor::MotorProtocol<mr::motor::MotorKind::LZ,
                                             mr::motor::MotorModel::RSO3>;

DmProtocol MakeDmProtocol(uint16_t master_id, uint16_t can_id,
                         mr::motor::MotorState* state) {
  mr::motor::MotorInstanceConfig<mr::motor::MotorKind::DM> config{};
  config.can = BSP_CAN_1;
  config.master_id = master_id;
  config.can_id = can_id;
  return DmProtocol(config, mr::motor::kDirectDriveInstall, *state,
                    mr::motor::DmModule::J4310);
}

LzProtocol MakeLzProtocol(mr::motor::MotorState* state) {
  mr::motor::MotorInstanceConfig<mr::motor::MotorKind::LZ> config{};
  config.can = BSP_CAN_2;
  config.motor_id = 3U;
  config.host_id = 0xfdu;
  config.mode = mr::motor::LzControlMode::Motion;
  return LzProtocol(config, mr::motor::kDirectDriveInstall, *state,
                    mr::motor::LzModule::Rso3);
}

void TestDmCommandValidationAndDisableStatus() {
  mr::motor::MotorState state{};
  auto protocol = MakeDmProtocol(0x201U, 0x001U, &state);
  Expect(protocol.Register() == DEVICE_OK,
         "DM with standard feedback and command IDs should register");
  Expect(g_fake_buses[BSP_CAN_1].latest_id == 0x201U,
         "DM should register the configured feedback identifier");
  Expect(protocol.Enable() == BSP_OK &&
             state.protocol_state == mr::motor::MotorProtocolState::Enabled,
         "DM should report enabled only after a successful enable frame");

  Expect(protocol.SetMIT(0.2f, 0.3f, 20.0f, 1.0f, 2.0f) == DEVICE_OK,
         "valid DM MIT command should be staged");
  g_fake_buses[BSP_CAN_1].fail_next_standard = true;
  Expect(protocol.CommitCommand() == BSP_ERR,
         "failed DM command transmission should be reported");
  Expect(protocol.HasPendingCommand(),
         "failed DM command should remain pending for explicit retry");
  Expect(protocol.SetMIT(std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f,
                         0.0f, 0.0f) == DEVICE_ERR,
         "DM should reject non-finite MIT inputs");
  Expect(protocol.HasPendingCommand(),
         "invalid DM input should not overwrite a pending valid command");
  Expect(protocol.CommitCommand() == BSP_OK &&
             !protocol.HasPendingCommand(),
         "valid DM command should clear after a successful retry");

  g_fake_buses[BSP_CAN_1].fail_next_standard = true;
  Expect(protocol.Disable() == BSP_ERR,
         "DM disable send failure should be returned to the caller");
  Expect(state.protocol_state == mr::motor::MotorProtocolState::Enabled,
         "DM should retain its prior state when disable transmission fails");
  Expect(protocol.Disable() == BSP_OK &&
             state.protocol_state == mr::motor::MotorProtocolState::Disabled,
         "DM should report disabled after the retry succeeds");
}

void TestDmInvalidAddressRejectedBeforeRegistration() {
  mr::motor::MotorState state{};
  const unsigned registrations =
      g_fake_buses[BSP_CAN_1].register_latest_count;
  auto protocol = MakeDmProtocol(0x800U, 0x001U, &state);
  Expect(protocol.Register() == DEVICE_ERR,
         "DM feedback ID outside standard CAN range should be rejected");
  Expect(g_fake_buses[BSP_CAN_1].register_latest_count == registrations,
         "invalid DM configuration must fail before BSP registration");

  auto invalid_command_id = MakeDmProtocol(0x201U, 0x500U, &state);
  Expect(invalid_command_id.Register() == DEVICE_ERR,
         "DM command base ID that overflows a supported mode should be rejected");
}

void TestLzValidationCommitAndRelaxStatus() {
  mr::motor::MotorState state{};
  auto protocol = MakeLzProtocol(&state);
  Expect(protocol.Register() == DEVICE_OK,
         "LZ protocol should register on a valid CAN bus");
  Expect(g_fake_buses[BSP_CAN_2].register_queue_count == 1U,
         "LZ feedback should use its registered queue");
  Expect(g_fake_buses[BSP_CAN_2].parser_register_count == 1U,
         "LZ registration should install its extended-ID parser");
  Expect(protocol.Enable() == BSP_OK &&
             state.protocol_state == mr::motor::MotorProtocolState::Enabled,
         "LZ should report enabled after the command is accepted");

  Expect(protocol.SetMIT(0.1f, 0.2f, -1.0f, 1.0f, 0.5f) == DEVICE_ERR,
         "LZ should reject negative proportional gain");
  Expect(!protocol.HasPendingCommand(),
         "rejected LZ input must not create a pending command");
  Expect(protocol.SetMIT(0.1f, 0.2f, 10.0f, 1.0f,
                         std::numeric_limits<float>::infinity()) == DEVICE_ERR,
         "LZ should reject infinite torque input");
  Expect(protocol.SetMIT(0.1f, 0.2f, 10.0f, 1.0f, 0.5f) == DEVICE_OK,
         "valid LZ MIT command should be staged");

  g_fake_buses[BSP_CAN_2].fail_next_extended = true;
  Expect(protocol.CommitCommand() == BSP_ERR,
         "failed LZ command transmission should be reported");
  Expect(protocol.HasPendingCommand(),
         "failed LZ command should remain pending for retry");
  Expect(protocol.SetPosition(std::numeric_limits<float>::quiet_NaN(), 0.0f) ==
             DEVICE_ERR,
         "LZ should reject non-finite position commands");
  Expect(protocol.HasPendingCommand(),
         "invalid LZ input should not overwrite a pending valid command");
  Expect(protocol.CommitCommand() == BSP_OK &&
             !protocol.HasPendingCommand(),
         "valid LZ command should clear after a successful retry");

  g_fake_buses[BSP_CAN_2].fail_next_extended = true;
  Expect(protocol.Disable() == BSP_ERR,
         "LZ disable send failure should be returned to the caller");
  Expect(state.protocol_state == mr::motor::MotorProtocolState::Enabled,
         "LZ should retain its prior state when disable transmission fails");
  Expect(protocol.Relax() == BSP_OK &&
             state.protocol_state == mr::motor::MotorProtocolState::Relaxed,
         "LZ should report relaxed only after its disable frame succeeds");
}

}  // namespace

extern "C" {

int8_t BSP_CAN_RegisterLatestId(BSP_CAN_t bus, uint32_t id) {
  if (bus >= BSP_CAN_NUM) {
    return BSP_ERR;
  }
  FakeCanBus& fake = g_fake_buses[bus];
  ++fake.register_latest_count;
  fake.latest_id = id;
  return BSP_OK;
}

int8_t BSP_CAN_RegisterId(BSP_CAN_t bus, uint32_t id, uint8_t) {
  if (bus >= BSP_CAN_NUM) {
    return BSP_ERR;
  }
  FakeCanBus& fake = g_fake_buses[bus];
  ++fake.register_queue_count;
  fake.queue_id = id;
  return BSP_OK;
}

int8_t BSP_CAN_RegisterIdParser(BSP_CAN_IdParser_t parser) {
  if (parser == nullptr) {
    return BSP_ERR;
  }
  ++g_fake_buses[BSP_CAN_2].parser_register_count;
  return BSP_OK;
}

int8_t BSP_CAN_GetLatestMessage(BSP_CAN_t, uint32_t, BSP_CAN_Message_t*) {
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
  if (fake.fail_next_standard) {
    fake.fail_next_standard = false;
    return BSP_ERR;
  }
  fake.last_standard_frame = *frame;
  return BSP_OK;
}

int8_t BSP_CAN_TransmitExtDataFrame(BSP_CAN_t bus,
                                    const BSP_CAN_ExtDataFrame_t* frame) {
  if (bus >= BSP_CAN_NUM || frame == nullptr) {
    return BSP_ERR;
  }
  FakeCanBus& fake = g_fake_buses[bus];
  ++fake.extended_transmit_count;
  if (fake.fail_next_extended) {
    fake.fail_next_extended = false;
    return BSP_ERR;
  }
  fake.last_extended_frame = *frame;
  return BSP_OK;
}

}  // extern "C"

int main() {
  TestDmCommandValidationAndDisableStatus();
  TestDmInvalidAddressRejectedBeforeRegistration();
  TestLzValidationCommitAndRelaxStatus();
  if (failures != 0) {
    std::printf("%d DM/LZ protocol host test(s) failed\n", failures);
    return 1;
  }
  std::printf("DM/LZ protocol host tests passed\n");
  return 0;
}
