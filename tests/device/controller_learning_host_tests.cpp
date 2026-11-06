#include <cassert>
#include <cmath>
#include "support/fake_can_port.hpp"
#include "device/motor/bus/rm_command_bus.hpp"
#include "device/motor/packages/controller/motor_controller.hpp"
#include "device/motor/packages/soft_limit_learning/soft_limit_learning.hpp"

namespace {
uint64_t now_us=0;
uint64_t Clock(){return now_us;}
void TestController() {
  using namespace mr::motor;
  FakeCanPort port; RmCommandBus bus(port);
  MotorInstanceConfig<MotorKind::RM> cfg; cfg.port=&port; cfg.commands=&bus; cfg.id=0x201;
  RmM3508Motor motor(cfg);
  MotorControllerConfig ctrl_cfg; ctrl_cfg.sample_freq=1000; // No PID fallback.
  MotorControllerT<RmM3508Motor> ctrl(motor,ctrl_cfg);
  assert(ctrl.Initialize()==0);
  CanFrame f; f.id=0x201; f.dlc=8; f.sequence=1; port.Push(f);
  bus.BeginCycle();
  assert(ctrl.Step([](auto& c){return c.SetMIT(0,1,0,2,0);})==0);
  // RM has no native MIT: damping must use desired minus actual velocity.
  assert(std::fabs(ctrl.GetLastOutputTorque()-2)<1e-5f);
  assert(ctrl.HasPendingCommand() && bus.Flush()==0 && !ctrl.HasPendingCommand());
  assert(port.sent.back().data[0]!=0 || port.sent.back().data[1]!=0);
  bus.BeginCycle();
  assert(ctrl.SetTorque(3)==0 && ctrl.UpdateCommand()==0);
  assert(ctrl.SetPosition(1)==DEVICE_ERR_UNSUPPORTED);
  assert(ctrl.UpdateCommand()==0 && std::fabs(ctrl.GetLastOutputTorque()-3)<1e-5f);
  assert(ctrl.Step([](auto&){return DEVICE_ERR;})==DEVICE_ERR && !ctrl.HasPendingCommand());
  assert(bus.Flush()==0 && port.sent.back().data[0]==0 && port.sent.back().data[1]==0);
  bus.BeginCycle(); port.now=21;
  assert(ctrl.Step([](auto& c){return c.SetTorque(2);})!=0);
  assert(ctrl.UpdateCommand()!=0 && !ctrl.HasPendingCommand());
  assert(bus.Flush()==0);
}
void TestLearning() {
  using namespace mr::motor;
  SoftLimitLearningConfig c{1,2,0,0.1f};
  c.stall_cycles_required=2; c.seek_startup_ignore_s=0; c.seek_timeout_s=1;
  SoftLimitLearning learner(c);
  MotorState s; s.online=s.feedback_valid=s.position_valid=true;
  s.position_rad=1; s.feedback_sequence=1;
  assert(learner.StartSeekUpper()); learner.Observe(s,0.01f);
  for(int i=0;i<10;++i) learner.Observe(s,0.01f);
  assert(learner.IsSeeking() && learner.GetStallCycles()==0);
  ++s.feedback_sequence; learner.Observe(s,0.01f);
  assert(learner.GetStallCycles()==1);
  ++s.feedback_sequence; learner.Observe(s,0.01f);
  assert(learner.HasUpper() && !learner.IsSeeking());
  learner.Reset(); assert(learner.StartSeekLower());
  s.feedback_valid=false;
  learner.Observe(s,0.5f); assert(!learner.CaptureLowerFromObserved());
  learner.Observe(s,0.5f); assert(learner.IsFailed());
  c.clock_us=Clock; c.seek_timeout_s=0.1f;
  SoftLimitLearning timed(c); assert(timed.StartSeekUpper());
  now_us=0; timed.Observe(s);
  now_us=110000; timed.Observe(s); assert(timed.IsFailed());
}
}
int main(){TestController(); TestLearning();}
