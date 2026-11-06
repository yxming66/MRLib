#include <array>
#include <cstdio>

#include "device/motor/protocol/rm_command_group.hpp"

namespace {

using mr::motor::ResolveRmCommandAddress;
using mr::motor::RmCommandAddress;
using mr::motor::RmCommandBank;
using mr::motor::RmCommandGroup;
using mr::motor::RmModule;

int failures = 0;

void Expect(bool condition, const char* message) {
  if (!condition) {
    std::printf("FAIL: %s\n", message);
    ++failures;
  }
}

void TestGroupAddressMapping() {
  RmCommandAddress address{};
  Expect(ResolveRmCommandAddress(RmModule::M3508, 0x201U, &address) &&
             address.frame_id == 0x200U &&
             address.group == RmCommandGroup::k200 && address.slot == 0U,
         "RM feedback IDs 0x201-0x204 should use the 0x200 group");
  Expect(ResolveRmCommandAddress(RmModule::M2006, 0x208U, &address) &&
             address.frame_id == 0x1ffU &&
             address.group == RmCommandGroup::k1ff && address.slot == 3U,
         "RM feedback IDs 0x205-0x208 should use slots 0-3 in the 0x1ff group");
  Expect(ResolveRmCommandAddress(RmModule::Gm6020, 0x205U, &address) &&
             address.frame_id == 0x1ffU &&
             address.group == RmCommandGroup::k1ff && address.slot == 0U,
         "GM6020 feedback IDs 0x205-0x208 should use the 0x1ff group");
  Expect(ResolveRmCommandAddress(RmModule::Gm6020, 0x209U, &address) &&
             address.frame_id == 0x2ffU &&
             address.group == RmCommandGroup::k2ff && address.slot == 0U,
         "GM6020 feedback IDs 0x209-0x20b should use the 0x2ff group");
}

void TestGroupPayloadsAreIndependent() {
  RmCommandAddress group_200{};
  RmCommandAddress group_1ff{};
  Expect(ResolveRmCommandAddress(RmModule::M3508, 0x201U, &group_200),
         "0x200 address should resolve");
  Expect(ResolveRmCommandAddress(RmModule::Gm6020, 0x205U, &group_1ff),
         "0x1ff address should resolve");

  RmCommandBank bank{};
  Expect(bank.Set(group_200, 0x1234U), "0x200 command should stage");
  Expect(bank.Set(group_1ff, 0xabcdU), "0x1ff command should stage");

  std::array<uint8_t, 8U> data_200{};
  std::array<uint8_t, 8U> data_1ff{};
  Expect(bank.Encode(group_200, data_200.data(), data_200.size()),
         "0x200 command payload should encode");
  Expect(bank.Encode(group_1ff, data_1ff.data(), data_1ff.size()),
         "0x1ff command payload should encode");
  Expect(data_200[0] == 0x12U && data_200[1] == 0x34U,
         "payload should encode 16-bit current values in big-endian order");
  Expect(data_1ff[0] == 0xabU && data_1ff[1] == 0xcdU,
         "one command group should retain its own slot value");
  for (std::size_t index = 2U; index < data_200.size(); ++index) {
    Expect(data_200[index] == 0U,
           "unused 0x200 slots should start in the relaxed state");
  }

  RmCommandBank candidate = bank;
  Expect(candidate.Set(group_200, 0x5678U),
         "a candidate command bank should accept a staged update");
  std::array<uint8_t, 8U> committed_data{};
  Expect(bank.Encode(group_200, committed_data.data(), committed_data.size()),
         "the committed command bank should remain encodable");
  Expect(committed_data[0] == 0x12U && committed_data[1] == 0x34U,
         "staged candidate changes must not mutate the committed bank");
}

void TestInvalidAddressesAreRejected() {
  RmCommandAddress address{};
  Expect(!ResolveRmCommandAddress(RmModule::M3508, 0x200U, &address),
         "RM command frame IDs are not feedback IDs and should be rejected");
  Expect(!ResolveRmCommandAddress(RmModule::M3508, 0x209U, &address),
         "RM feedback IDs above 0x208 should be rejected");
  Expect(!ResolveRmCommandAddress(static_cast<RmModule>(99U), 0x201U, &address),
         "unknown RM module should be rejected");
  Expect(!ResolveRmCommandAddress(RmModule::M3508, 0x201U, nullptr),
         "null address output should be rejected");

  RmCommandBank bank{};
  const RmCommandAddress invalid{0x200U, RmCommandGroup::k200, 4U};
  Expect(!bank.Set(invalid, 0x1234U),
         "slot outside the 4-motor group should be rejected");
  std::array<uint8_t, 8U> data{};
  Expect(!bank.Encode(invalid, data.data(), data.size()),
         "invalid group slots must not be encoded");
}

}  // namespace

int main() {
  TestGroupAddressMapping();
  TestGroupPayloadsAreIndependent();
  TestInvalidAddressesAreRejected();
  if (failures != 0) {
    std::printf("%d RM command group host test(s) failed\n", failures);
    return 1;
  }
  std::printf("RM command group host tests passed\n");
  return 0;
}
