#include "device/motor/protocol/dm_protocol.hpp"
#include <string.h>
#include "component/math/scalar.hpp"
#include "device/device.h"

namespace mr::motor {
namespace {
constexpr float kPMin=-12.56637f, kPMax=12.56637f, kVMin=-30.0f, kVMax=30.0f;
bool IsFiniteCommand(float value){return mr::component::math::is_finite_scalar(value);}
bool IsValidAddress(const CanTransport& transport,
                    const MotorInstanceConfig<MotorKind::DM>& config) {
  return transport.is_valid() && config.master_id <= 0x7ffU &&
         config.can_id <= (0x7ffU - 0x300U);
}
uint16_t Encode(float x,float mn,float mx,uint8_t bits){ const float y=x<mn?mn:(x>mx?mx:x); const uint32_t n=(1UL<<bits)-1UL; return static_cast<uint16_t>(((y-mn)/(mx-mn))*n); }
float FloatFromRaw(uint16_t x,float mn,float mx,uint8_t bits){ return static_cast<float>(x)*(mx-mn)/static_cast<float>((1UL<<bits)-1UL)+mn; }
uint16_t Read16(const uint8_t* p){return static_cast<uint16_t>((p[0]<<8)|p[1]);}
}
template <MotorModel M> MotorProtocol<MotorKind::DM,M>::MotorProtocol(const MotorInstanceConfig<MotorKind::DM>& c,const MotorInstallSpec& i,MotorState& s,DmModule m):config_(c),install_(i),state_(s),mapper_(TransmissionMapper::FromTotalRatio(MotorTraits<MotorKind::DM,M>::kReducerRatio*i.external_ratio,MotorTraits<MotorKind::DM,M>::kTorqueConstant)),transport_(c.can),module_(m){}
template <MotorModel M> int8_t MotorProtocol<MotorKind::DM,M>::Register(){if(registered_)return DEVICE_OK;if(!IsValidAddress(transport_,config_))return DEVICE_ERR;const int8_t r=transport_.RegisterLatest(config_.master_id);if(r==BSP_OK){registered_=true;state_.protocol_state=MotorProtocolState::Registered;}return r;}
template <MotorModel M> bool MotorProtocol<MotorKind::DM,M>::Decode(DmFeedback& f){CanFrame x{};if(transport_.ReadLatest(config_.master_id,&x)!=BSP_OK||x.dlc<8)return false;state_.last_feedback_tick=x.timestamp_ms;state_.feedback_sequence=x.sequence;state_.feedback_valid=true;f.position=Read16(x.data+1);f.velocity=static_cast<uint16_t>((x.data[3]<<4)|(x.data[4]>>4));f.torque=static_cast<uint16_t>(((x.data[4]&15U)<<8)|x.data[5]);f.status=x.data[0]>>4;f.mos_temperature=x.data[6];f.rotor_temperature=x.data[7];return true;}
template <MotorModel M> void MotorProtocol<MotorKind::DM,M>::RefreshState(const DmFeedback& f){float p=FloatFromRaw(f.position,kPMin,kPMax,16),v=FloatFromRaw(f.velocity,kVMin,kVMax,12),t=FloatFromRaw(f.torque,-MotorTraits<MotorKind::DM,M>::kPeakTorque,MotorTraits<MotorKind::DM,M>::kPeakTorque,12);const float sign=config_.reverse?-1.0f:1.0f;p*=sign;v*=sign;t*=sign;if(!tracker_.initialized())tracker_.Sync(p);const float rp=tracker_.Accumulate(p,v,kPMax-kPMin,kDefaultRotorResyncDeltaRad);state_.position_rad=mapper_.ToOutputPosition(rp);state_.position_single_turn_rad=mr::component::math::wrap_to_pi(state_.position_rad);state_.velocity_rad_s=mapper_.ToOutputVelocity(v);state_.torque_nm=mapper_.ToOutputTorque(t);state_.temperature_c=f.rotor_temperature;state_.device_temperature_c=f.mos_temperature;state_.online=true;state_.protocol_status_code=f.status;state_.protocol_fault=(f.status>=8U&&f.status<=14U)?(1UL<<(f.status-8U)):0U;state_.protocol_state=state_.protocol_fault?MotorProtocolState::Fault:(f.status==1U?MotorProtocolState::Enabled:MotorProtocolState::Disabled);}
template <MotorModel M> int8_t MotorProtocol<MotorKind::DM,M>::Update(){DmFeedback f{};if(!Decode(f)){state_.online=false;state_.feedback_valid=false;++state_.feedback_lost_count;return DEVICE_ERR_NO_DEV;}feedback_=f;RefreshState(f);return DEVICE_OK;}
template <MotorModel M> int8_t MotorProtocol<MotorKind::DM,M>::Send(const CanFrame& f)const{return transport_.SendStandard(f);}
template <MotorModel M>
int8_t MotorProtocol<MotorKind::DM, M>::Enable() {
  CanFrame frame{};
  frame.id = config_.can_id;
  frame.dlc = 8U;
  for (uint8_t& byte : frame.data) {
    byte = 0xffU;
  }
  frame.data[7] = 0xfcU;
  const int8_t result = Send(frame);
  if (result == BSP_OK) {
    state_.protocol_state = MotorProtocolState::Enabled;
  }
  return result;
}

template <MotorModel M>
int8_t MotorProtocol<MotorKind::DM, M>::Disable() {
  ClearPendingCommand();
  CanFrame frame{};
  frame.id = config_.can_id;
  frame.dlc = 8U;
  for (uint8_t& byte : frame.data) {
    byte = 0xffU;
  }
  frame.data[7] = 0xfdU;
  const int8_t result = Send(frame);
  if (result == BSP_OK) {
    state_.protocol_state = MotorProtocolState::Disabled;
  }
  return result;
}

template <MotorModel M>
int8_t MotorProtocol<MotorKind::DM, M>::Relax() {
  const int8_t result = Disable();
  if (result == BSP_OK) {
    state_.protocol_state = MotorProtocolState::Relaxed;
  }
  return result;
}
template <MotorModel M> int8_t MotorProtocol<MotorKind::DM,M>::SetZero(){CanFrame f{};f.id=config_.can_id;f.dlc=8;for(uint8_t& b:f.data)b=0xff;f.data[7]=0xfe;const int8_t r=Send(f);if(r==BSP_OK)ResetZeroedState(state_);return r;}
template <MotorModel M> int8_t MotorProtocol<MotorKind::DM,M>::CommitCommand(){if(pending_==PendingCommandType::None){state_.last_commit_ok=true;return DEVICE_OK;}CanFrame f{};f.id=config_.can_id;f.dlc=8;if(pending_==PendingCommandType::Mit){const uint16_t p=Encode(config_.reverse?-pending_mit_position_:pending_mit_position_,kPMin,kPMax,16),v=Encode(config_.reverse?-pending_mit_velocity_:pending_mit_velocity_,kVMin,kVMax,12),kp=Encode(pending_kp_,0,500,12),kd=Encode(pending_kd_,0,5,12),t=Encode(config_.reverse?-pending_torque_:pending_torque_,-MotorTraits<MotorKind::DM,M>::kPeakTorque,MotorTraits<MotorKind::DM,M>::kPeakTorque,12);f.data[0]=p>>8;f.data[1]=p;f.data[2]=v>>4;f.data[3]=(v<<4)|(kp>>8);f.data[4]=kp;f.data[5]=kd>>4;f.data[6]=(kd<<4)|(t>>8);f.data[7]=t;}else{float a=pending_==PendingCommandType::Position?pending_position_:pending_velocity_,b=pending_==PendingCommandType::Position?pending_velocity_limit_:0;if(config_.reverse){a=-a;b=-b;}memcpy(f.data,&a,4);if(pending_==PendingCommandType::Position){memcpy(f.data+4,&b,4);f.id+=0x100U;}else{f.dlc=4;f.id+=0x200U;}}const int8_t r=Send(f);state_.last_commit_ok=(r==BSP_OK);if(r==BSP_OK)ClearPendingCommand();return r;}
template <MotorModel M> void MotorProtocol<MotorKind::DM,M>::ClearPendingCommand(){pending_=PendingCommandType::None;state_.command_pending=false;}
template <MotorModel M> int8_t MotorProtocol<MotorKind::DM,M>::SetTorque(float t){return SetMIT(state_.position_rad,0,0,0,t);}
template <MotorModel M> int8_t MotorProtocol<MotorKind::DM,M>::SetVelocity(float v){if(!IsFiniteCommand(v))return DEVICE_ERR;pending_velocity_=PrepareVelocityCommand(mapper_,v,MotorTraits<MotorKind::DM,M>::kMaxVelocity);pending_=PendingCommandType::Velocity;state_.command_pending=true;return DEVICE_OK;}
template <MotorModel M> int8_t MotorProtocol<MotorKind::DM,M>::SetPosition(float p,float v){if(!IsFiniteCommand(p)||!IsFiniteCommand(v))return DEVICE_ERR;const PositionCommandValues c=PreparePositionCommand(mapper_,p,v,MotorTraits<MotorKind::DM,M>::kMaxPosition,MotorTraits<MotorKind::DM,M>::kMaxVelocity);pending_position_=c.rotor_position;pending_velocity_limit_=c.rotor_velocity_limit;pending_=PendingCommandType::Position;state_.command_pending=true;return DEVICE_OK;}
template <MotorModel M> int8_t MotorProtocol<MotorKind::DM,M>::SetMIT(float p,float v,float kp,float kd,float t){if(!IsFiniteCommand(p)||!IsFiniteCommand(v)||!IsFiniteCommand(kp)||!IsFiniteCommand(kd)||!IsFiniteCommand(t))return DEVICE_ERR;const MitCommandValues c=PrepareMitCommand(mapper_,p,v,t,MotorTraits<MotorKind::DM,M>::kMaxPosition,MotorTraits<MotorKind::DM,M>::kMaxVelocity,MotorTraits<MotorKind::DM,M>::kPeakTorque);pending_mit_position_=c.rotor_position;pending_mit_velocity_=c.rotor_velocity;pending_kp_=kp;pending_kd_=kd;pending_torque_=c.torque_ff;pending_=PendingCommandType::Mit;state_.command_pending=true;return DEVICE_OK;}
#define INSTANTIATE(M) template class MotorProtocol<MotorKind::DM,M>;
INSTANTIATE(MotorModel::J10010) INSTANTIATE(MotorModel::J10010L) INSTANTIATE(MotorModel::J10422P) INSTANTIATE(MotorModel::J3507) INSTANTIATE(MotorModel::J4310) INSTANTIATE(MotorModel::J4310P) INSTANTIATE(MotorModel::J4340) INSTANTIATE(MotorModel::J4340P) INSTANTIATE(MotorModel::J6006) INSTANTIATE(MotorModel::J6248P) INSTANTIATE(MotorModel::J8006) INSTANTIATE(MotorModel::J8009) INSTANTIATE(MotorModel::J8009P) INSTANTIATE(MotorModel::H3510)
}
