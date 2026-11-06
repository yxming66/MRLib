#pragma once
#include <array>
#include <cstddef>
#include "component/math/scalar.hpp"
#include "device/device.h"
#include "device/joint/joint_interface.hpp"
namespace mr {
class JointT final : public IJoint, public IActuator {
 public:
  static constexpr uint8_t kJointNameMaxLen=24;
  JointT(uint8_t id, IActuator* actuator, const JointControlParams& params,
         float offset, float frequency, const char* name=nullptr,
         JointType type=JointType::kRevolute)
      : id_(id), actuator_(actuator), params_(params), offset_(offset), type_(type) {
    (void)frequency; SetName(name);
  }
  JointT(const JointT&)=delete;
  JointT& operator=(const JointT&)=delete;
  uint8_t GetId() const override { return id_; }
  const char* GetName() const override { return name_.data(); }
  JointType GetType() const override { return type_; }
  float GetCurrentAngle() const override { return state_.position; }
  float GetCurrentVelocity() const override { return state_.velocity; }
  float GetCurrentTorque() const override { return state_.torque; }
  uint32_t FeedbackEpoch() const override { return resolved_epoch_; }
  float GetTargetAngle() const override { return target_; }
  bool IsOnline() const override { return state_.online; }
  const JointControlParams& GetParams() const override { return params_; }
  float GetOffset() const override { return offset_; }
  float GetFeedforwardTorque() const override { return feedforward_; }
  float GetMotorCommandPosition() const override { return command_position_; }
  float GetMotorCommandVelocity() const override { return command_velocity_; }
  float GetMotorCommandTorque() const override { return command_torque_; }
  float GetMotorCommandKp() const override { return command_kp_; }
  float GetMotorCommandKd() const override { return command_kd_; }
  ActuatorState GetState() const override { return state_; }
  IActuator* actuator() const { return actuator_; }
  void SetName(const char* name) {
    name_={}; if (!name) return;
    for (size_t i=0; i+1<name_.size() && name[i]; ++i) name_[i]=name[i];
  }
  void SetOffset(float value) override {
    if (Finite(value)) { offset_=value; Invalidate(); }
  }
  void SetTargetAngle(float value) override { if(Finite(value)) target_=value; }
  void SetFeedforwardTorque(float value) override { if(Finite(value)) feedforward_=value; }
  bool SetTransmission(const JointTransmission& value) {
    if (!ValidTransmission(value)) return false;
    transmission_=value; Invalidate(); return true;
  }
  const JointTransmission& GetTransmission() const { return transmission_; }
  const IJoint* CouplingSource(size_t i) const { return i<kJointMaxCouplings ? sources_[i] : nullptr; }
  bool SetCouplingSource(uint8_t index, const IJoint* source) {
    if (index>=kJointMaxCouplings || source==this) return false;
    sources_[index]=source;
    if(index>=transmission_.coupling_count) transmission_.coupling_count=index+1;
    transmission_.couplings[index].source_joint=source?source->GetId():0;
    transmission_.couplings[index].enabled=source!=nullptr;
    Invalidate(); return true;
  }
  bool AddCoupling(const IJoint* source, float coefficient) {
    if (!source || source==this || !Finite(coefficient) || transmission_.coupling_count>=kJointMaxCouplings) return false;
    const auto index=transmission_.coupling_count;
    if (!transmission_.AddCoupling(source->GetId(),coefficient)) return false;
    sources_[index]=source; Invalidate(); return true;
  }
  void SetCoupledJoint(const IJoint* source) {
    transmission_.ClearCouplings(); sources_={}; Invalidate();
    if (source) (void)AddCoupling(source,1);
  }
  int8_t Register() override { return IsValid() ? actuator_->Register() : DEVICE_ERR; }
  int8_t Enable() override { return IsValid() ? actuator_->Enable() : DEVICE_ERR; }
  int8_t Relax() override { ClearPendingControl(); return actuator_?actuator_->Relax():DEVICE_ERR_NULL; }
  int8_t SetZero() override {
    if (!actuator_) return DEVICE_ERR_NULL;
    if (HasCouplings()) return DEVICE_ERR_UNSUPPORTED;
    const auto result=actuator_->SetZero();
    if(result==0) { offset_=0; state_={}; target_=0; ClearPendingControl(); }
    return result;
  }
  int8_t CommitControl() override { return actuator_?actuator_->CommitControl():DEVICE_ERR_NULL; }
  bool HasPendingControl() const override { return actuator_ && actuator_->HasPendingControl(); }
  void ClearPendingControl() override {
    if(actuator_) actuator_->ClearPendingControl();
    command_position_=command_velocity_=command_torque_=command_kp_=command_kd_=0;
  }
  bool HasCouplings() const {
    for(size_t i=0;i<transmission_.coupling_count;++i)
      if(transmission_.couplings[i].enabled) return true;
    return false;
  }
  // Independent joints can update directly. Coupled joints require JointGroup.
  int8_t Update() override {
    if(HasCouplings()) return DEVICE_ERR_UNSUPPORTED;
    const uint32_t epoch=++standalone_epoch_;
    auto result=SampleFeedback(epoch);
    return result==0?ResolveFeedback(epoch):result;
  }
  int8_t SampleFeedback(uint32_t epoch) {
    sampled_epoch_=epoch; sampled_=false; state_.online=false;
    if(!IsValid()) { ClearPendingControl(); return DEVICE_ERR; }
    const auto result=actuator_->Update(); raw_=actuator_->GetState();
    if(result!=0 || !raw_.online || !Finite(raw_.position) || !Finite(raw_.velocity) || !Finite(raw_.torque))
      { ClearPendingControl(); return result!=0?result:DEVICE_ERR_NO_DEV; }
    sampled_=true; return DEVICE_OK;
  }
  int8_t ResolveFeedback(uint32_t epoch) {
    state_.online=false;
    if(!sampled_ || sampled_epoch_!=epoch) return DEVICE_ERR_NO_DEV;
    float position_sum=0,velocity_sum=0;
    for(size_t i=0;i<transmission_.coupling_count;++i) {
      const auto& term=transmission_.couplings[i]; if(!term.enabled) continue;
      const auto* source=sources_[i];
      if(!source || !source->IsOnline() || source->FeedbackEpoch()!=epoch) return DEVICE_ERR_NO_DEV;
      position_sum+=term.coefficient*source->GetCurrentAngle();
      velocity_sum+=term.coefficient*source->GetCurrentVelocity();
    }
    const float s=transmission_.actuator_scale;
    ActuatorState candidate=raw_;
    candidate.position=(raw_.position-offset_-transmission_.actuator_offset-position_sum)/s;
    candidate.velocity=(raw_.velocity-velocity_sum)/s;
    candidate.torque=raw_.torque*s;
    if(!Finite(candidate.position)||!Finite(candidate.velocity)||!Finite(candidate.torque)) return DEVICE_ERR;
    state_=candidate; resolved_epoch_=epoch; return DEVICE_OK;
  }
  void Invalidate() { state_.online=false; sampled_=false; ClearPendingControl(); }
  bool IsValid() const {
    return actuator_ && Finite(offset_) && ValidTransmission(transmission_) &&
      (type_==JointType::kRevolute || type_==JointType::kPrismatic) &&
      Finite(params_.qmin)&&Finite(params_.qmax)&&params_.qmin<params_.qmax;
  }
  int8_t PositionControl(float position,float dt) override {
    (void)dt;
    return SetMIT(position,0,params_.kp,params_.kd,feedforward_);
  }
  int8_t SetMIT(float position,float velocity,float kp,float kd,float torque) override {
    // A coupled transmission requires vector control; scalar MIT would apply
    // unintended torque to other joints. Never silently approximate it here.
    if(HasCouplings()) return DEVICE_ERR_UNSUPPORTED;
    if(!IsValid()||!state_.online) return DEVICE_ERR_NO_DEV;
    if(!Finite(position)||!Finite(velocity)||!Finite(kp)||!Finite(kd)||!Finite(torque)||kp<0||kd<0) return DEVICE_ERR;
    float limited=position<params_.qmin?params_.qmin:(position>params_.qmax?params_.qmax:position);
    const bool blocked=(state_.position<=params_.qmin+1e-4f && position<state_.position) ||
                       (state_.position>=params_.qmax-1e-4f && position>state_.position);
    if(blocked) { limited=state_.position; velocity=0; torque=0; }
    const float s=transmission_.actuator_scale;
    const float p=s*limited+transmission_.actuator_offset+offset_,v=s*velocity,t=torque/s;
    const float gain_scale=s*s;
    if(!Finite(p)||!Finite(v)||!Finite(t)||!Finite(kp/gain_scale)||!Finite(kd/gain_scale)) return DEVICE_ERR;
    const auto result=actuator_->SetMIT(p,v,kp/gain_scale,kd/gain_scale,t);
    if(result!=0) return result;
    target_=limited; command_position_=p; command_velocity_=v; command_torque_=t;
    command_kp_=kp/gain_scale; command_kd_=kd/gain_scale; return DEVICE_OK;
  }
  int8_t SetTorque(float torque) override { return TorqueControl(torque); }
  int8_t TorqueControl(float torque) override {
    if(HasCouplings()) return DEVICE_ERR_UNSUPPORTED;
    if(!IsValid()||!Finite(torque)||!Finite(torque/transmission_.actuator_scale)) return DEVICE_ERR;
    const auto result=actuator_->SetTorque(torque/transmission_.actuator_scale);
    if(result==0) { feedforward_=torque; command_torque_=torque/transmission_.actuator_scale; }
    return result;
  }
  bool IsReached(float tolerance) const override {
    return state_.online && Finite(tolerance) && tolerance>0 && component::math::abs_scalar(target_-state_.position)<tolerance;
  }
  // Group-only vector torque path and J^T feedback accumulation.
  float RawTorque() const { return raw_.torque; }
  void SetResolvedTorque(float torque) { state_.torque=torque; if(!Finite(torque)) Invalidate(); }
  int8_t StageActuatorTorque(float torque) {
    if(!actuator_||!Finite(torque)) return DEVICE_ERR;
    const auto result=actuator_->SetTorque(torque);
    if(result==0) command_torque_=torque;
    return result;
  }
 private:
  static bool Finite(float x) { return component::math::is_finite_scalar(x); }
  static bool ValidTransmission(const JointTransmission& t) {
    if(!Finite(t.actuator_scale)||component::math::abs_scalar(t.actuator_scale)<=1e-6f||
       !Finite(t.actuator_offset)||t.coupling_count>kJointMaxCouplings) return false;
    for(size_t i=0;i<t.coupling_count;++i) if(!Finite(t.couplings[i].coefficient)) return false;
    return true;
  }
  uint8_t id_;
  IActuator* actuator_;
  JointControlParams params_;
  float offset_;
  JointType type_;
  std::array<char,kJointNameMaxLen> name_{};
  JointTransmission transmission_{};
  std::array<const IJoint*,kJointMaxCouplings> sources_{};
  ActuatorState state_{},raw_{};
  float target_=0,feedforward_=0,command_position_=0,command_velocity_=0,command_torque_=0,command_kp_=0,command_kd_=0;
  uint32_t standalone_epoch_=0,sampled_epoch_=0,resolved_epoch_=0;
  bool sampled_=false;
};
using DmJoint=JointT;
using LzJoint=JointT;
}  // namespace mr
