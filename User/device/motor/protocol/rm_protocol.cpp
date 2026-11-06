#include "device/motor/protocol/rm_protocol.hpp"
#include "component/math/scalar.hpp"
#include "device/device.h"
#include "device/motor/protocol/rm_command_group.hpp"
namespace mr::motor {
namespace { RmCommandBank g_outputs[BSP_CAN_NUM]{}; uint16_t Read16(const uint8_t* p){return static_cast<uint16_t>((p[0]<<8)|p[1]);} int16_t ReadS16(const uint8_t* p){return static_cast<int16_t>(Read16(p));} float Clip(float x,float l){return x>l?l:(x<-l?-l:x);} }
template <MotorModel M> MotorProtocol<MotorKind::RM,M>::MotorProtocol(const MotorInstanceConfig<MotorKind::RM>& c,const MotorInstallSpec& i,MotorState& s,RmModule m):config_(c),install_(i),state_(s),mapper_(TransmissionMapper::FromTotalRatio(MotorTraits<MotorKind::RM,M>::kGearRatio*i.external_ratio,MotorTraits<MotorKind::RM,M>::kTorqueConstant)),transport_(c.can),module_(m){}
template <MotorModel M>
int8_t MotorProtocol<MotorKind::RM, M>::Register() {
  if (registered_) {
    return DEVICE_OK;
  }
  RmCommandAddress address{};
  if (!transport_.is_valid() ||
      !ResolveRmCommandAddress(module_, config_.id, &address)) {
    return DEVICE_ERR;
  }
  const int8_t result = transport_.RegisterLatest(config_.id);
  if (result == BSP_OK) {
    registered_ = true;
    state_.protocol_state = MotorProtocolState::Registered;
  }
  return result;
}
template <MotorModel M> bool MotorProtocol<MotorKind::RM,M>::Decode(RmFeedback& f){CanFrame x{};if(transport_.ReadLatest(config_.id,&x)!=BSP_OK||x.dlc<8)return false;state_.last_feedback_tick=x.timestamp_ms;state_.feedback_sequence=x.sequence;state_.feedback_valid=true;f.angle=Read16(x.data);f.speed=ReadS16(x.data+2);f.current=ReadS16(x.data+4);f.temperature=x.data[6];f.error=x.data[7];return true;}
template <MotorModel M> void MotorProtocol<MotorKind::RM,M>::RefreshState(const RmFeedback& f){const float raw_pos=static_cast<float>(f.angle)*mr::component::math::kTwoPi/8192.0f;float p=raw_pos,v=static_cast<float>(f.speed)*0.104719755f,t=static_cast<float>(f.current)*MotorTraits<MotorKind::RM,M>::kCurrentRangeAmp/MotorTraits<MotorKind::RM,M>::kRawCurrentRange;if(config_.reverse){p=-p;v=-v;t=-t;}if(!position_initialized_){position_initialized_=true;last_position_=p;accumulated_position_=p;}const float d=mr::component::math::wrap_error(p-last_position_,mr::component::math::kTwoPi);accumulated_position_+=d;last_position_=p;state_.position_rad=mapper_.ToOutputPosition(accumulated_position_-zero_offset_);state_.position_single_turn_rad=mr::component::math::wrap_to_pi(state_.position_rad);state_.position_valid=true;state_.velocity_rad_s=mapper_.ToOutputVelocity(v);state_.torque_nm=mapper_.ToOutputTorque(t);state_.temperature_c=f.temperature;state_.online=true;state_.protocol_status_code=f.error;state_.protocol_fault=f.error?1UL<<((f.error<31)?f.error:31):0U;state_.protocol_state=f.error?MotorProtocolState::Fault:MotorProtocolState::Enabled;}
template <MotorModel M> int8_t MotorProtocol<MotorKind::RM,M>::Update(){RmFeedback f{};if(!Decode(f)){state_.online=false;state_.feedback_valid=false;++state_.feedback_lost_count;return DEVICE_ERR_NO_DEV;}feedback_=f;RefreshState(f);return DEVICE_OK;}
template <MotorModel M> int8_t MotorProtocol<MotorKind::RM,M>::Enable(){state_.protocol_state=MotorProtocolState::Enabled;return DEVICE_OK;}
template <MotorModel M>
int8_t MotorProtocol<MotorKind::RM, M>::Disable() {
  return Relax();
}

template <MotorModel M>
int8_t MotorProtocol<MotorKind::RM, M>::Relax() {
  pending_current_ = 0.0f;
  pending_valid_ = true;
  pending_relax_ = true;
  state_.command_pending = true;
  const int8_t result = CommitCommand();
  return result;
}

template <MotorModel M>
int8_t MotorProtocol<MotorKind::RM, M>::CommitCommand() {
  if (!pending_valid_) {
    state_.last_commit_ok = true;
    return DEVICE_OK;
  }

  RmCommandAddress address{};
  if (!transport_.is_valid() ||
      !ResolveRmCommandAddress(module_, config_.id, &address)) {
    state_.last_commit_ok = false;
    return DEVICE_ERR;
  }

  const float current_limit = MotorTraits<MotorKind::RM, M>::kCurrentRangeAmp;
  const float raw_range = MotorTraits<MotorKind::RM, M>::kRawCurrentRange;
  if (!mr::component::math::is_finite_scalar(pending_current_) ||
      !mr::component::math::is_finite_scalar(current_limit) ||
      current_limit <= 0.0f ||
      !mr::component::math::is_finite_scalar(raw_range) || raw_range <= 0.0f) {
    state_.last_commit_ok = false;
    return DEVICE_ERR;
  }

  const int16_t raw = static_cast<int16_t>(
      Clip(pending_current_, current_limit) * raw_range / current_limit);
  const uint8_t bus = static_cast<uint8_t>(config_.can);
  RmCommandBank candidate = g_outputs[bus];
  if (!candidate.Set(address, static_cast<uint16_t>(raw))) {
    state_.last_commit_ok = false;
    return DEVICE_ERR;
  }

  CanFrame frame{};
  frame.id = address.frame_id;
  frame.dlc = sizeof(frame.data);
  if (!candidate.Encode(address, frame.data, sizeof(frame.data))) {
    state_.last_commit_ok = false;
    return DEVICE_ERR;
  }

  const int8_t result = transport_.SendStandard(frame);
  state_.last_commit_ok = result == BSP_OK;
  if (result == BSP_OK) {
    g_outputs[bus] = candidate;
    if (pending_relax_) {
      state_.protocol_state = MotorProtocolState::Relaxed;
    }
    ClearPendingCommand();
  }
  return result;
}
template <MotorModel M>
void MotorProtocol<MotorKind::RM, M>::ClearPendingCommand() {
  pending_valid_ = false;
  pending_current_ = 0.0f;
  pending_relax_ = false;
  state_.command_pending = false;
}
template <MotorModel M>
int8_t MotorProtocol<MotorKind::RM, M>::SetTorque(float torque_nm) {
  if (!mr::component::math::is_finite_scalar(torque_nm)) {
    return DEVICE_ERR;
  }
  const float max_torque = MotorTraits<MotorKind::RM, M>::kPeakTorque;
  pending_current_ = mapper_.ToTorqueCurrent(
      Clip(torque_nm, max_torque), install_.reverse_output);
  pending_valid_ = true;
  pending_relax_ = false;
  state_.command_pending = true;
  return DEVICE_OK;
}
template <MotorModel M> int8_t MotorProtocol<MotorKind::RM,M>::SetZero(){if constexpr(M==MotorModel::M2006||M==MotorModel::M3508){zero_offset_=position_initialized_?last_position_:0.0f;state_.position_rad=0;return DEVICE_OK;}return DEVICE_ERR_UNSUPPORTED;}
#define INSTANTIATE(M) template class MotorProtocol<MotorKind::RM,M>;
INSTANTIATE(MotorModel::M2006) INSTANTIATE(MotorModel::M3508) INSTANTIATE(MotorModel::M6020)
}
