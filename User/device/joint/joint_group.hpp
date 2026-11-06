#pragma once
#include "device/joint/joint.hpp"
namespace mr {
// Fixed DAG of linear transmissions a=J*q+b. Raw actuator feedback is sampled
// first, then joint positions are resolved in dependency order. Efforts use J^T.
template<size_t N>
class JointGroup final {
 public:
  static_assert(N > 0, "A joint group must contain at least one joint");
  explicit JointGroup(const std::array<JointT*,N>& joints):joints_(joints) {}
  bool Validate() {
    std::array<bool,N> done{};
    for(size_t i=0;i<N;++i) {
      if(!joints_[i]||!joints_[i]->IsValid()) return false;
      for(size_t j=0;j<i;++j)
        if(joints_[i]==joints_[j]||joints_[i]->actuator()==joints_[j]->actuator()) return false;
    }
    for(size_t step=0;step<N;++step) {
      bool found=false;
      for(size_t i=0;i<N;++i) if(!done[i]) {
        bool ready=true;
        const auto& t=joints_[i]->GetTransmission();
        for(size_t k=0;k<t.coupling_count;++k) if(t.couplings[k].enabled) {
          const auto source=Index(joints_[i]->CouplingSource(k));
          if(source==N) return false;
          ready=ready&&done[source];
        }
        if(ready) { order_[step]=i; done[i]=true; found=true; break; }
      }
      if(!found) return false;
    }
    return true;
  }
  int8_t UpdateFeedback() {
    valid_=false;
    if(!Validate()) { Invalidate(); return DEVICE_ERR; }
    ++epoch_;
    int8_t result=0;
    for(auto* joint:joints_) {
      const auto next=joint->SampleFeedback(epoch_);
      if(result==0) result=next;
    }
    if(result!=0) { Invalidate(); return result; }
    for(auto index:order_) {
      result=joints_[index]->ResolveFeedback(epoch_);
      if(result!=0) { Invalidate(); return result; }
    }
    for(auto* joint:joints_) joint->SetResolvedTorque(joint->RawTorque()*joint->GetTransmission().actuator_scale);
    for(auto* joint:joints_) {
      const auto& t=joint->GetTransmission();
      for(size_t k=0;k<t.coupling_count;++k) if(t.couplings[k].enabled) {
        auto* source=joints_[Index(joint->CouplingSource(k))];
        source->SetResolvedTorque(source->GetCurrentTorque()+t.couplings[k].coefficient*joint->RawTorque());
      }
    }
    for(auto* joint:joints_) if(!joint->IsOnline()) { Invalidate(); return DEVICE_ERR; }
    valid_=true; return DEVICE_OK;
  }
  int8_t SetTorques(const std::array<float,N>& torques) {
    if(!valid_||!Validate()) { ClearPending(); return DEVICE_ERR; }
    for(auto* joint:joints_)
      if(!joint->IsOnline() || joint->FeedbackEpoch()!=epoch_) {
        Invalidate(); return DEVICE_ERR_NO_DEV;
      }
    auto raw=torques;
    for(size_t step=N;step>0;--step) {
      const auto index=order_[step-1];
      const auto& t=joints_[index]->GetTransmission();
      raw[index]/=t.actuator_scale;
      if(!component::math::is_finite_scalar(raw[index])) { ClearPending(); return DEVICE_ERR; }
      for(size_t k=0;k<t.coupling_count;++k) if(t.couplings[k].enabled)
        raw[Index(joints_[index]->CouplingSource(k))]-=t.couplings[k].coefficient*raw[index];
    }
    for(size_t i=0;i<N;++i) {
      const auto result=joints_[i]->StageActuatorTorque(raw[i]);
      if(result!=0) { for(auto* joint:joints_) joint->ClearPendingControl(); return result; }
    }
    return DEVICE_OK;
  }
 private:
  void ClearPending() { for(auto* joint:joints_) if(joint) joint->ClearPendingControl(); }
  size_t Index(const IJoint* joint) const {
    for(size_t i=0;i<N;++i) if(joints_[i]==joint) return i;
    return N;
  }
  void Invalidate() { for(auto* joint:joints_) if(joint) joint->Invalidate(); }
  std::array<JointT*,N> joints_;
  std::array<size_t,N> order_{};
  uint32_t epoch_=0;
  bool valid_=false;
};
}  // namespace mr
