#include <cassert>
#include <cmath>
#include <limits>
#include "device/joint/joint_group.hpp"
#include "device/wheel/wheel.hpp"
#include "device/motor/core/motor_state.hpp"

namespace {
bool Near(float a, float b) { return std::fabs(a-b)<1e-4f; }
struct Actuator final : mr::IActuator {
  mr::ActuatorState state{};
  float p=0,v=0,kp=0,kd=0,t=0;
  bool pending=false, fail=false;
  int8_t Register() override { return 0; }
  int8_t Enable() override { return 0; }
  int8_t Update() override { return state.online?0:DEVICE_ERR_NO_DEV; }
  int8_t Relax() override { pending=false; return 0; }
  int8_t SetZero() override { return 0; }
  int8_t CommitControl() override { pending=false; return 0; }
  bool HasPendingControl() const override { return pending; }
  void ClearPendingControl() override { pending=false; }
  int8_t SetMIT(float a,float b,float c,float d,float e) override {
    if(fail) return DEVICE_ERR;
    p=a; v=b; kp=c; kd=d; t=e; pending=true; return 0;
  }
  int8_t SetTorque(float value) override {
    if(fail) return DEVICE_ERR;
    t=value; pending=true; return 0;
  }
  mr::ActuatorState GetState() const override { return state; }
};
struct Drive {
  mr::motor::MotorState state{};
  float torque=0; bool pending=false; int updates=0;
  int8_t Register(){return 0;} int8_t Enable(){return 0;} int8_t Relax(){return 0;}
  int8_t Update(){++updates; return 0;}
  auto GetState() const { return state; }
  int8_t SetTorque(float value){torque=value; pending=true; return 0;}
  int8_t CommitCommand(){pending=false; return 0;}
  bool HasPendingCommand() const{return pending;}
  void ClearPendingCommand(){pending=false;}
};
void TestIndependentTransmission() {
  Actuator a; a.state={-1.5f,-4,3,true,10,7};
  mr::JointT q(1,&a,{-10,10,8,4},0.5f,1000);
  mr::JointTransmission t; t.actuator_scale=-2;
  assert(q.SetTransmission(t));
  assert(q.Update()==0);
  assert(Near(q.GetCurrentAngle(),1) && Near(q.GetCurrentVelocity(),2));
  assert(Near(q.GetCurrentTorque(),-6));
  assert(q.SetMIT(2,3,8,4,10)==0);
  assert(Near(a.p,-3.5f) && Near(a.v,-6));
  assert(Near(a.kp,2) && Near(a.kd,1) && Near(a.t,-5));
  assert(q.GetState().timestamp_ms==10 && q.GetState().sequence==7);
  a.state.online=false;
  assert(q.Update()!=0 && !q.IsOnline() && !a.pending);
  t.actuator_scale=0; assert(!q.SetTransmission(t));
}
void TestCoupledTransmission() {
  Actuator a,b;
  a.state={2,4,5,true}; b.state={7,10,7,true};
  mr::JointT q0(0,&a,{-100,100,0,0},0,1000);
  mr::JointT q1(1,&b,{-100,100,0,0},0,1000);
  mr::JointTransmission t; t.actuator_scale=2; assert(q0.SetTransmission(t));
  t.actuator_scale=4; assert(q1.SetTransmission(t));
  assert(q1.AddCoupling(&q0,3)); // J = [2,0;3,4]
  mr::JointGroup<2> group({&q1,&q0}); // Deliberately reverse dependency order.
  assert(group.UpdateFeedback()==0);
  assert(Near(q0.GetCurrentAngle(),1) && Near(q1.GetCurrentAngle(),1));
  assert(Near(q0.GetCurrentVelocity(),2) && Near(q1.GetCurrentVelocity(),1));
  assert(Near(q0.GetCurrentTorque(),31) && Near(q1.GetCurrentTorque(),28));
  assert(q1.SetMIT(0,0,1,1,0)==DEVICE_ERR_UNSUPPORTED);
  assert(group.SetTorques({8,10})==0); // actuator efforts: b=2, a=2
  assert(Near(a.t,2) && Near(b.t,2));
  assert(Near(a.t*a.state.velocity+b.t*b.state.velocity,
              10*q0.GetCurrentVelocity()+8*q1.GetCurrentVelocity()));
  assert(group.SetTorques({std::numeric_limits<float>::infinity(),0})!=0);
  assert(!a.pending && !b.pending);
  b.fail=true;
  assert(group.SetTorques({8,10})!=0 && !a.pending && !b.pending);
  b.fail=false; assert(group.SetTorques({8,10})==0);
  q0.SetOffset(1); // Changing mapping invalidates an already resolved batch.
  assert(group.SetTorques({8,10})!=0 && !a.pending && !b.pending);
  assert(q0.AddCoupling(&q1,1)); assert(!group.Validate());
  mr::JointGroup<2> duplicate({&q0,&q0}); assert(!duplicate.Validate());
  mr::JointGroup<1> external({&q1}); assert(!external.Validate());
}
void TestWheel() {
  Drive drive;
  drive.state.online=drive.state.feedback_valid=drive.state.position_valid=true;
  drive.state.position_rad=2; drive.state.velocity_rad_s=10; drive.state.torque_nm=3;
  drive.state.feedback_sequence=5; drive.state.last_feedback_tick=20;
  mr::wheel::MotorWheel<Drive> wheel(drive,0.1f);
  mr::wheel::WheelFeedback state;
  assert(wheel.Register()==0 && wheel.UpdateFeedback(state)==0);
  assert(Near(state.linear_velocity_mps,1) && state.sequence==5 && state.timestamp_ms==20);
  assert(wheel.SetTorque(4)==0 && Near(drive.torque,4));
  assert(wheel.UpdateCommand()==0 && drive.updates==1);
  drive.state.feedback_valid=false;
  assert(wheel.UpdateFeedback(state)!=0 && !state.online && !drive.pending);
  mr::wheel::MotorWheel<Drive> invalid(drive,0);
  assert(invalid.Register()!=0);
}
}
int main(){TestIndependentTransmission(); TestCoupledTransmission(); TestWheel();}
