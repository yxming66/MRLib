#include <cassert>
#include "support/fake_can_port.hpp"
#include "device/motor/protocol/dm_protocol.hpp"
#include "device/motor/protocol/lz_protocol.hpp"
using namespace mr::motor;
int main() {
  FakeCanPort port;
  MotorState dm_state, lz_state;
  MotorInstanceConfig<MotorKind::DM> dm_config;
  dm_config.port=&port; dm_config.master_id=0x101; dm_config.can_id=1;
  MotorProtocol<MotorKind::DM,MotorModel::J4310> dm(dm_config,kDirectDriveInstall,dm_state,DmModule::J4310);
  MotorInstanceConfig<MotorKind::LZ> lz_config;
  lz_config.port=&port; lz_config.motor_id=3; lz_config.host_id=0xfd;
  MotorProtocol<MotorKind::LZ,MotorModel::RSO3> lz(lz_config,kDirectDriveInstall,lz_state,LzModule::Rso3);
  assert(dm.Register()==0 && lz.Register()==0);
  CanFrame d{}; d.id=0x101; d.dlc=8; d.data[0]=0x11; d.data[1]=0x80; d.data[3]=0x80; d.data[4]=8;
  d.sequence=1; d.timestamp_ms=100; port.now=100; port.Push(d);
  CanFrame l{}; l.id=(2UL<<24)|(3UL<<8)|0xfd; l.format=CanFrame::Format::Extended;
  l.dlc=8; l.data[0]=0x80; l.data[2]=0x80; l.data[4]=0x80; l.sequence=1; l.timestamp_ms=100;
  port.Push(l);
  assert(dm.Update()==0 && lz.Update()==0 && dm_state.position_valid && lz_state.position_valid);
  port.now=120;
  assert(dm.Update()==0 && lz.Update()==0); // Same latest / empty queue within timeout.
  assert(dm_state.last_feedback_tick==100 && lz_state.last_feedback_tick==100);
  port.now=121;
  assert(dm.Update()!=0 && lz.Update()!=0 && !dm_state.online && !lz_state.online);
  d.sequence=2; d.timestamp_ms=121; port.Push(d);
  l.sequence=2; l.timestamp_ms=121; port.Push(l);
  assert(dm.Update()==0 && lz.Update()==0);
  port.fail_read=true;
  assert(dm.Update()!=0 && lz.Update()!=0); // Driver error invalidates immediately.
  port.fail_read=false;
  d.sequence=3; d.timestamp_ms=122; port.now=122; port.Push(d);
  assert(dm.Update()==0 && dm.SetZero()==0);
  assert(!dm_state.online && !dm_state.position_valid && dm.Update()!=0);
  l.sequence=3; l.timestamp_ms=122; port.Push(l);
  assert(lz.Update()==0 && lz.SetZero()==0);
  assert(!lz_state.online && !lz_state.position_valid && lz.Update()!=0);
  // Cross-format frames and stale/future samples never become valid feedback.
  d.sequence=4; d.timestamp_ms=123; d.format=CanFrame::Format::Extended; port.Push(d);
  assert(dm.Update()!=0);
  l.sequence=4; l.timestamp_ms=123; port.Push(l);
  assert(lz.Update()!=0); // Future timestamp relative to now=122.
  CanTransport transport(&port);
  CanFrame bad{}; bad.dlc=9;
  assert(transport.SendStandard(bad)!=0);
  const auto old=bad;
  port.latest[0x777]=bad;
  assert(transport.ReadLatest(0x777,&bad)!=0 && bad.dlc==old.dlc);
}
