#include "device/motor/protocol/dm_protocol.hpp"

#include <string.h>

#include "component/math/scalar.hpp"
#include "device/device.h"

namespace mr::motor {
namespace {

bool IsFiniteCommand(float value) { return mr::component::math::is_finite_scalar(value); }
bool IsValidAddress(const CanTransport& transport,
                    const MotorInstanceConfig<MotorKind::DM>& config) {
  return transport.is_valid() && FeedbackTimeoutValid(config.feedback_timeout_ms) &&
         config.master_id <= 0x7ffU && config.can_id > 0 && config.can_id <= (0x7ffU - 0x300U) &&
         mr::component::math::is_positive_scalar(config.mit_position_range_rad) &&
         mr::component::math::is_finite_scalar(2 * config.mit_position_range_rad) &&
         mr::component::math::is_positive_scalar(config.mit_velocity_range_rad_s) &&
         mr::component::math::is_finite_scalar(2 * config.mit_velocity_range_rad_s) &&
         mr::component::math::is_finite_scalar(config.mit_torque_range_nm) &&
         config.mit_torque_range_nm >= 0 &&
         mr::component::math::is_finite_scalar(2 * config.mit_torque_range_nm);
}
uint16_t Encode(float x, float mn, float mx, uint8_t bits) {
  const float y = x < mn ? mn : (x > mx ? mx : x);
  const uint32_t n = (1UL << bits) - 1UL;
  return static_cast<uint16_t>(((y - mn) / (mx - mn)) * n);
}
float FloatFromRaw(uint16_t x, float mn, float mx, uint8_t bits) {
  return static_cast<float>(x) * (mx - mn) / static_cast<float>((1UL << bits) - 1UL) + mn;
}
uint16_t Read16(const uint8_t* p) { return static_cast<uint16_t>((p[0] << 8) | p[1]); }
}  // namespace
template <MotorModel M>
MotorProtocol<MotorKind::DM, M>::MotorProtocol(const MotorInstanceConfig<MotorKind::DM>& c,
                                               const MotorInstallSpec& i, MotorState& s, DmModule m)
    : config_(c),
      install_(i),
      state_(s),
      mapper_(TransmissionMapper::FromExternalRatio(i.external_ratio, 1.0f)),
      transport_(c.port),
      module_(m) {}
template <MotorModel M>
float MotorProtocol<MotorKind::DM, M>::TorqueRange() const {
  return config_.mit_torque_range_nm > 0 ? config_.mit_torque_range_nm
                                         : MotorTraits<MotorKind::DM, M>::kPeakTorque;
}
template <MotorModel M>
int8_t MotorProtocol<MotorKind::DM, M>::Register() {
  if (registered_) return DEVICE_OK;
  if (!IsValidAddress(transport_, config_) || !ValidInstall(install_) ||
      !mr::component::math::is_positive_scalar(TorqueRange()))
    return DEVICE_ERR;
  const uint32_t keys[] = {config_.master_id, config_.can_id,
                           static_cast<uint32_t>(config_.can_id) + 0x100U,
                           static_cast<uint32_t>(config_.can_id) + 0x200U};
  if (!config_.port->ClaimEndpoints(this, keys, 4)) return DEVICE_ERR;
  const auto result = transport_.RegisterLatest(config_.master_id);
  if (result != DEVICE_OK) {
    config_.port->ReleaseEndpoints(this);
    return result;
  }
  registered_ = true;
  state_.protocol_state = MotorProtocolState::Registered;
  return DEVICE_OK;
}
template <MotorModel M>
FeedbackRead MotorProtocol<MotorKind::DM, M>::Decode(DmFeedback& f) {
  CanFrame x{};
  const auto result = transport_.ReadLatest(config_.master_id, &x);
  if (result == kCanNoFrame) return FeedbackRead::kNoNew;
  if (result != DEVICE_OK || x.dlc != 8 || x.format != CanFrame::Format::Standard ||
      x.id != config_.master_id ||
      !IsFreshFeedback(transport_.NowMs(), x.timestamp_ms, config_.feedback_timeout_ms))
    return FeedbackRead::kInvalid;
  if (have_feedback_ && x.sequence == last_sequence_ && x.timestamp_ms == last_feedback_tick_)
    return FeedbackRead::kNoNew;
  last_sequence_ = x.sequence;
  last_feedback_tick_ = x.timestamp_ms;
  have_feedback_ = true;
  state_.last_feedback_tick = x.timestamp_ms;
  state_.feedback_sequence = x.sequence;
  state_.feedback_valid = true;
  f.position = Read16(x.data + 1);
  f.velocity = static_cast<uint16_t>((x.data[3] << 4) | (x.data[4] >> 4));
  f.torque = static_cast<uint16_t>(((x.data[4] & 15U) << 8) | x.data[5]);
  f.status = x.data[0] >> 4;
  f.mos_temperature = x.data[6];
  f.rotor_temperature = x.data[7];
  return FeedbackRead::kNew;
}
template <MotorModel M>
void MotorProtocol<MotorKind::DM, M>::RefreshState(const DmFeedback& f) {
  const float pmax = config_.mit_position_range_rad, vmax = config_.mit_velocity_range_rad_s;
  const float sign = config_.reverse ? -1.0f : 1.0f;
  const float p = sign * FloatFromRaw(f.position, -pmax, pmax, 16);
  const float v = sign * FloatFromRaw(f.velocity, -vmax, vmax, 12);
  const float t = sign * FloatFromRaw(f.torque, -TorqueRange(), TorqueRange(), 12);
  if (!tracker_.initialized()) tracker_.Sync(p);
  const float accumulated = tracker_.Accumulate(p, v, 2 * pmax, kDefaultRotorResyncDeltaRad);
  state_.position_rad = mapper_.ToOutputPosition(accumulated);
  state_.position_single_turn_rad = mr::component::math::wrap_to_pi(state_.position_rad);
  state_.position_valid = true;
  state_.velocity_rad_s = mapper_.ToOutputVelocity(v);
  state_.torque_nm = mapper_.ToOutputTorque(t);  // Native protocol already reports Nm.
  state_.temperature_c = f.rotor_temperature;
  state_.device_temperature_c = f.mos_temperature;
  state_.online = true;
  state_.protocol_status_code = f.status;
  state_.protocol_fault = (f.status >= 8U && f.status <= 14U) ? (1UL << (f.status - 8U)) : 0U;
  state_.protocol_state = state_.protocol_fault ? MotorProtocolState::Fault
                                                : (f.status == 1U ? MotorProtocolState::Enabled
                                                                  : MotorProtocolState::Disabled);
}
template <MotorModel M>
int8_t MotorProtocol<MotorKind::DM, M>::Update() {
  DmFeedback f{};
  const auto result = Decode(f);
  if (result == FeedbackRead::kNoNew)
    return KeepFeedbackUntilTimeout(state_, transport_.NowMs(), config_.feedback_timeout_ms);
  if (result == FeedbackRead::kInvalid) {
    if (state_.online) ++state_.feedback_lost_count;
    MarkFeedbackUnavailable(state_);
    return DEVICE_ERR_NO_DEV;
  }
  feedback_ = f;
  RefreshState(f);
  return DEVICE_OK;
}
template <MotorModel M>
int8_t MotorProtocol<MotorKind::DM, M>::Send(const CanFrame& f) const {
  return transport_.SendStandard(f);
}
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
  if (result == DEVICE_OK) {
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
  if (result == DEVICE_OK) {
    state_.protocol_state = MotorProtocolState::Disabled;
  }
  return result;
}

template <MotorModel M>
int8_t MotorProtocol<MotorKind::DM, M>::Relax() {
  const int8_t result = Disable();
  if (result == DEVICE_OK) {
    state_.protocol_state = MotorProtocolState::Relaxed;
  }
  return result;
}
template <MotorModel M>
int8_t MotorProtocol<MotorKind::DM, M>::SetZero() {
  CanFrame f{};
  f.id = config_.can_id;
  f.dlc = 8;
  for (uint8_t& b : f.data) b = 0xff;
  f.data[7] = 0xfe;
  const int8_t r = Send(f);
  if (r == DEVICE_OK) {
    ClearPendingCommand();
    tracker_.Reset();
    ResetZeroedState(state_);
  }
  return r;
}
template <MotorModel M>
int8_t MotorProtocol<MotorKind::DM, M>::CommitCommand() {
  if (pending_ == PendingCommandType::None) {
    state_.last_commit_ok = true;
    return DEVICE_OK;
  }
  CanFrame f{};
  f.id = config_.can_id;
  f.dlc = 8;
  const float sign = config_.reverse ? -1.0f : 1.0f;
  if (pending_ == PendingCommandType::Mit) {
    const uint16_t p = Encode(sign * pending_mit_position_, -config_.mit_position_range_rad,
                              config_.mit_position_range_rad, 16);
    const uint16_t v = Encode(sign * pending_mit_velocity_, -config_.mit_velocity_range_rad_s,
                              config_.mit_velocity_range_rad_s, 12);
    const uint16_t kp = Encode(pending_kp_, 0, 500, 12), kd = Encode(pending_kd_, 0, 5, 12);
    const uint16_t t = Encode(sign * pending_torque_, -TorqueRange(), TorqueRange(), 12);
    f.data[0] = p >> 8;
    f.data[1] = p;
    f.data[2] = v >> 4;
    f.data[3] = (v << 4) | (kp >> 8);
    f.data[4] = kp;
    f.data[5] = kd >> 4;
    f.data[6] = (kd << 4) | (t >> 8);
    f.data[7] = t;
  } else {
    const bool position = pending_ == PendingCommandType::Position;
    const float a = sign * (position ? pending_position_ : pending_velocity_);
    memcpy(f.data, &a, 4);
    if (position) {
      // This is a speed magnitude, independent of installation direction.
      memcpy(f.data + 4, &pending_velocity_limit_, 4);
      f.id += 0x100U;
    } else {
      f.dlc = 4;
      f.id += 0x200U;
    }
  }
  const auto result = Send(f);
  state_.last_commit_ok = result == DEVICE_OK;
  if (result == DEVICE_OK) ClearPendingCommand();
  return result;
}
template <MotorModel M>
void MotorProtocol<MotorKind::DM, M>::ClearPendingCommand() {
  pending_ = PendingCommandType::None;
  state_.command_pending = false;
}
template <MotorModel M>
int8_t MotorProtocol<MotorKind::DM, M>::SetTorque(float t) {
  return SetMIT(state_.position_rad, 0, 0, 0, t);
}
template <MotorModel M>
int8_t MotorProtocol<MotorKind::DM, M>::SetVelocity(float velocity) {
  const float native = mapper_.ToRotorVelocity(velocity);
  if (!IsFiniteCommand(velocity) || !IsFiniteCommand(native)) return DEVICE_ERR;
  pending_velocity_ =
      mr::component::math::abs_clip_scalar(native, MotorTraits<MotorKind::DM, M>::kMaxVelocity);
  pending_ = PendingCommandType::Velocity;
  state_.command_pending = true;
  return DEVICE_OK;
}
template <MotorModel M>
int8_t MotorProtocol<MotorKind::DM, M>::SetPosition(float position, float max_velocity) {
  const float p = mapper_.ToRotorPosition(position), v = mapper_.ToRotorVelocity(max_velocity);
  if (!IsFiniteCommand(position) || !IsFiniteCommand(max_velocity) || max_velocity < 0 ||
      !IsFiniteCommand(p) || !IsFiniteCommand(v))
    return DEVICE_ERR;
  pending_position_ =
      mr::component::math::abs_clip_scalar(p, MotorTraits<MotorKind::DM, M>::kMaxPosition);
  pending_velocity_limit_ = max_velocity > 0 ? mr::component::math::abs_clip_scalar(
                                                   v, MotorTraits<MotorKind::DM, M>::kMaxVelocity)
                                             : MotorTraits<MotorKind::DM, M>::kMaxVelocity;
  pending_ = PendingCommandType::Position;
  state_.command_pending = true;
  return DEVICE_OK;
}
template <MotorModel M>
int8_t MotorProtocol<MotorKind::DM, M>::SetMIT(float position, float velocity, float kp, float kd,
                                               float torque) {
  const float p = mapper_.ToRotorPosition(position), v = mapper_.ToRotorVelocity(velocity);
  const float t = mapper_.ToTorqueCurrent(torque);
  const float native_kp = (kp / mapper_.ratio) / mapper_.ratio;
  const float native_kd = (kd / mapper_.ratio) / mapper_.ratio;
  if (!IsFiniteCommand(position) || !IsFiniteCommand(velocity) || !IsFiniteCommand(kp) ||
      !IsFiniteCommand(kd) || !IsFiniteCommand(torque) || kp < 0 || kd < 0 || !IsFiniteCommand(p) ||
      !IsFiniteCommand(v) || !IsFiniteCommand(t) || !IsFiniteCommand(native_kp) ||
      !IsFiniteCommand(native_kd))
    return DEVICE_ERR;
  pending_mit_position_ = mr::component::math::abs_clip_scalar(p, config_.mit_position_range_rad);
  pending_mit_velocity_ = mr::component::math::abs_clip_scalar(v, config_.mit_velocity_range_rad_s);
  pending_torque_ = mr::component::math::abs_clip_scalar(t, TorqueRange());
  pending_kp_ = native_kp;
  pending_kd_ = native_kd;
  pending_ = PendingCommandType::Mit;
  state_.command_pending = true;
  return DEVICE_OK;
}
template <MotorModel M>
int8_t MotorProtocol<MotorKind::DM, M>::ClearFault() {
  ClearPendingCommand();
  CanFrame f{};
  f.id = config_.can_id;
  f.dlc = 8;
  for (auto& byte : f.data) byte = 0xff;
  f.data[7] = 0xfb;
  // Do not clear the observed fault optimistically; wait for real feedback.
  return Send(f);
}
#define INSTANTIATE(M) template class MotorProtocol<MotorKind::DM, M>;
INSTANTIATE(MotorModel::J10010)
INSTANTIATE(MotorModel::J10010L) INSTANTIATE(MotorModel::J10422P) INSTANTIATE(MotorModel::J3507)
    INSTANTIATE(MotorModel::J4310) INSTANTIATE(MotorModel::J4310P) INSTANTIATE(MotorModel::J4340)
        INSTANTIATE(MotorModel::J4340P) INSTANTIATE(MotorModel::J6006)
            INSTANTIATE(MotorModel::J6248P) INSTANTIATE(MotorModel::J8006)
                INSTANTIATE(MotorModel::J8009) INSTANTIATE(MotorModel::J8009P)
                    INSTANTIATE(MotorModel::H3510)
}  // namespace mr::motor
