#include <cassert>
#include <cmath>
#include <limits>
#include "support/fake_can_port.hpp"
#include "device/motor/protocol/rm_protocol.hpp"
using namespace mr::motor;
using Protocol = MotorProtocol<MotorKind::RM, MotorModel::M3508>;
MotorInstanceConfig<MotorKind::RM> Config(FakeCanPort& port, RmCommandBus& bus, uint16_t id) {
  MotorInstanceConfig<MotorKind::RM> c;
  c.port=&port; c.commands=&bus; c.id=id; return c;
}
CanFrame Feedback(uint16_t angle, uint32_t timestamp, uint32_t sequence) {
  CanFrame f{}; f.id=0x201; f.dlc=8;
  f.data[0]=angle>>8; f.data[1]=angle; f.data[3]=10;
  f.timestamp_ms=timestamp; f.sequence=sequence; return f;
}
int main() {
  FakeCanPort port;
  RmCommandBus bus(port);
  MotorState a{}, b{}, c{};
  Protocol first(Config(port,bus,0x201),kDirectDriveInstall,a,RmModule::M3508);
  Protocol second(Config(port,bus,0x202),kDirectDriveInstall,b,RmModule::M3508);
  Protocol other(Config(port,bus,0x205),kDirectDriveInstall,c,RmModule::M3508);
  assert(first.Register()==0 && second.Register()==0 && other.Register()==0);
  MotorState duplicate_state{};
  Protocol duplicate(Config(port,bus,0x201),kDirectDriveInstall,duplicate_state,RmModule::M3508);
  assert(duplicate.Register()!=0);
  assert(first.SetTorque(1)==0 && first.CommitCommand()!=0); // No implicit cycle.
  bus.BeginCycle();
  assert(first.SetTorque(1)==0 && first.CommitCommand()==0);
  assert(second.SetTorque(2)==0 && second.CommitCommand()==0 && port.sent.empty());
  assert(first.HasPendingCommand() && !a.last_commit_ok);
  assert(bus.Flush()==0 && port.sent.size()==2); // Two registered groups.
  assert(port.sent[0].data[0]!=0 || port.sent[0].data[1]!=0);
  assert(port.sent[0].data[2]!=0 || port.sent[0].data[3]!=0);
  assert(!first.HasPendingCommand() && a.last_commit_ok);
  assert(bus.Flush()==0 && port.sent.size()==2); // Exactly once per group.
  bus.BeginCycle();
  assert(first.Relax()==0 && first.HasPendingCommand());
  port.fail_send=true;
  assert(bus.Flush()!=0 && first.HasPendingCommand() && !a.last_commit_ok);
  const auto count=port.sent.size();
  assert(bus.Flush()==0 && port.sent.size()==count+1); // Only retry failed group.
  assert(a.protocol_state==MotorProtocolState::Relaxed && !first.HasPendingCommand());
  for(auto byte:port.sent.back().data) assert(byte==0); // Missing slot never repeats old torque.
  bus.BeginCycle();
  assert(second.SetTorque(2)==0 && second.CommitCommand()==0);
  second.ClearPendingCommand();
  assert(bus.Flush()==0);
  for(auto byte:port.sent[port.sent.size()-2].data) assert(byte==0);
  assert(first.SetTorque(std::numeric_limits<float>::quiet_NaN())!=0);
  assert(bus.EmergencyStop()==0);
  // Feedback age, duplicate sample and uint32 wrap.
  port.now=UINT32_MAX-5; port.Push(Feedback(7000,port.now,1));
  assert(first.Update()==0 && a.online);
  const auto position=a.position_rad;
  port.now=3;
  assert(first.Update()==0 && a.position_rad==position && a.feedback_sequence==1);
  port.now=15;
  assert(first.Update()!=0 && !a.online && !a.position_valid);
  // Multiple encoder wraps before software zero: zero accumulated position.
  const uint16_t angles[]={7000,8100,100,2000,4000,6000,8100,100};
  uint32_t sequence=2;
  for(auto angle:angles) { ++port.now; port.Push(Feedback(angle,port.now,sequence++)); assert(first.Update()==0); }
  assert(first.SetZero()==0);
  ++port.now; port.Push(Feedback(100,port.now,sequence++));
  assert(first.Update()==0 && std::fabs(a.position_rad)<1e-6f);
  // Installation reversal applies equally to commands and feedback.
  FakeCanPort reversed_port; RmCommandBus reversed_bus(reversed_port);
  auto cfg=Config(reversed_port,reversed_bus,0x201); cfg.reverse=true;
  MotorState reversed{};
  Protocol motor(cfg,kDirectDriveInstall,reversed,RmModule::M3508);
  assert(motor.Register()==0);
  reversed_port.Push(Feedback(100,0,1));
  assert(motor.Update()==0 && reversed.velocity_rad_s<0);
  reversed_bus.BeginCycle(); assert(motor.SetTorque(1)==0 && motor.CommitCommand()==0);
  assert(reversed_bus.Flush()==0 && (reversed_port.sent[0].data[0]&0x80));
}
