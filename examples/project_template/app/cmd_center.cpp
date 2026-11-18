#include "app/cmd_center.hpp"

#include <algorithm>
#include <cmath>

#include "app/robot_config.hpp"
#include "component/cmd_center/cmd_center.hpp"

namespace app {

// ==================== 项目模式选择回调 ====================
// 初始化时显式绑定到 select_chassis_mode；输入适配器不包含拨杆映射策略。
ChassisMode SelectChassisMode(void*, const RemoteInput& remote) {
  switch (remote.right_switch) {
    case RemoteSwitch::kMiddle:
      return remote.left_switch == RemoteSwitch::kDown
          ? ChassisMode::kStairAssist : ChassisMode::kManual;
    case RemoteSwitch::kDown:
      return ChassisMode::kAuto;
    case RemoteSwitch::kUp:
    case RemoteSwitch::kUnknown:
      return ChassisMode::kDisabled;
  }
  return ChassisMode::kDisabled;
}

namespace {

// ==================== 1. 私有状态与通用工具 ====================
// 下面的输入、条件和 Sink 共享这份状态；跨 app 共享的消息类型在 app_messages.hpp。
struct State {
  CommandCenterBindings bindings{};
  RemoteInput remote{};
  NucInputSnapshot nuc{};
  CommandCenterStatus status{};
  ChassisCommand command{};
  bool remote_enabled = false;
};

bool Fresh(uint32_t now, uint32_t received, uint32_t timeout) {
  return uint32_t(now - received) <= timeout;
}

float ApplyDeadzone(float value) {
  return std::fabs(value) > params::kRemoteDeadzone ? value : 0.0f;
}

// ==================== 2. 输入适配：快照、有效性和模式 ====================
// Center 每次 tick 先更新输入；online 决定路由能否取到该输入，
// timestamp_ms 保留原始接收时间。在线与是否获准控制分别判断。
struct Dr16Input {
  explicit Dr16Input(State* state) : state(state) {}

  bool update(cmd::Context& ctx) {
    const bool available = state->bindings.read_remote(
        state->bindings.context, state->remote);
    const auto& remote = state->remote;
    auto& status = state->status;
    status.remote = remote;
    status.remote_fresh = available && remote.online &&
        Fresh(ctx.now_ms(), remote.updated_ms, params::kRemoteTimeoutMs) &&
        std::isfinite(remote.left_x) && std::isfinite(remote.left_y) &&
        std::isfinite(remote.right_x) && std::isfinite(remote.right_y);
    if (!status.remote_fresh) {
      status.armed = false;
    } else if (remote.left_switch == params::kRemoteArmPosition) {
      status.armed = true;
    }
    state->remote_enabled = status.remote_fresh && status.armed &&
        (remote.left_switch == params::kRemoteEnablePositionA ||
         remote.left_switch == params::kRemoteEnablePositionB);
    status.chassis_mode = ChassisMode::kDisabled;
    if (state->remote_enabled && state->bindings.select_chassis_mode) {
      status.chassis_mode = state->bindings.select_chassis_mode(
          state->bindings.context, remote);
    }
    return true;
  }

  bool online() const { return state->status.remote_fresh; }
  uint32_t timestamp_ms() const { return state->remote.updated_ms; }
  State* state;
};

// NUC 只检查数据可用性；是否允许自动控制，由后面的路由条件决定。
struct NucInput {
  explicit NucInput(State* state) : state(state) {}
  bool update(cmd::Context& ctx) {
    const auto read = state->bindings.read_nuc;
    const bool available = read && read(state->bindings.context, state->nuc);
    const auto& nuc = state->nuc;
    state->status.nuc_fresh = available && nuc.valid &&
        Fresh(ctx.now_ms(), nuc.received_ms, params::kNucTimeoutMs) &&
        std::isfinite(nuc.vx_mps) && std::isfinite(nuc.vy_mps) &&
        std::isfinite(nuc.wz_rad_s);
    return true;
  }
  bool online() const { return state->status.nuc_fresh; }
  uint32_t timestamp_ms() const { return state->nuc.received_ms; }
  State* state;
};

// ==================== 3. 全局安全规则 ====================
// 全局授权高于任何路由优先级；NUC 不能绕过遥控器的失联/解锁保护。
struct RobotStopRule {
  bool operator()(const cmd::Context& ctx) const {
    const auto* remote = ctx.input<Dr16Input>();
    return !remote || !remote->state->remote_enabled;
  }
};

// ==================== 4. 路由授权条件 ====================
// 条件只读取已更新的模式，不修改状态；授权通过后才生成候选命令。
struct AllowManualChassis {
  bool operator()(const cmd::Context& ctx) const {
    const auto* remote = ctx.input<Dr16Input>();
    if (!remote) return false;
    const auto mode = remote->state->status.chassis_mode;
    return mode == ChassisMode::kManual || mode == ChassisMode::kStairAssist;
  }
};

struct AllowAutoChassis {
  bool operator()(const cmd::Context& ctx) const {
    const auto* remote = ctx.input<Dr16Input>();
    return remote && remote->state->status.chassis_mode == ChassisMode::kAuto;
  }
};

// ==================== 5. 候选命令构造：输入到业务命令的映射 ====================
// Builder 各自生成完整命令，由 Center 仲裁；不在这里发送 CAN 或操作电机。
struct BuildManualChassis {
  bool operator()(const Dr16Input& input, cmd::Context& ctx,
                  ChassisCommand& out) const {
    const auto& state = *input.state;
    const auto& remote = state.remote;
    out.input_fresh = true;
    out.enabled = true;
    out.received_ms = ctx.now_ms();
    out.source_received_ms = remote.updated_ms;
    out.source = CommandSource::kDr16;
    out.vx_mps = ApplyDeadzone(remote.right_y) * params::kRemoteMaxVxMps;
    out.vy_mps = -ApplyDeadzone(remote.right_x) * params::kRemoteMaxVyMps;
    if (state.status.chassis_mode == ChassisMode::kManual) {
      out.wz_rad_s = -ApplyDeadzone(remote.left_x) * params::kRemoteMaxWzRadS;
    } else {
      out.stair_enabled = true;
      out.front_target_rate_rad_s =
          ApplyDeadzone(remote.left_y) * params::kStairTargetRateMaxRadS;
      out.support_target_rate_rad_s =
          ApplyDeadzone(remote.left_x) * params::kStairTargetRateMaxRadS;
      const float track_speed = ApplyDeadzone(remote.right_y) * params::kTrackMaxSpeedRadS;
      out.left_track_speed_rad_s = track_speed;
      out.right_track_speed_rad_s = track_speed;
    }
    return true;
  }
};

struct BuildAutoChassis {
  bool operator()(const NucInput& input, cmd::Context& ctx,
                  ChassisCommand& out) const {
    const auto& nuc = input.state->nuc;
    out.vx_mps = std::clamp(nuc.vx_mps, -params::kMaxChassisVxMps, params::kMaxChassisVxMps);
    out.vy_mps = std::clamp(nuc.vy_mps, -params::kMaxChassisVyMps, params::kMaxChassisVyMps);
    out.wz_rad_s = std::clamp(nuc.wz_rad_s, -params::kMaxChassisWzRadS, params::kMaxChassisWzRadS);
    out.enabled = true;
    out.input_fresh = true;
    out.received_ms = ctx.now_ms();
    out.source_received_ms = nuc.received_ms;
    out.source = CommandSource::kNuc;
    // NUC 当前仅控制底盘平面速度，不授权上台阶机构。
    return true;
  }
};

// ==================== 6. 安全命令与输出 Sink ====================
// 全局安全规则触发或没有有效候选时，输出未使能命令；不保留旧运动目标。
struct SafeChassisCommand {
  bool operator()(cmd::Context& ctx, ChassisCommand& out) const {
    out = {};
    out.received_ms = ctx.now_ms();
    return true;
  }
};

// Sink 接收实际胜出的命令；同一所有者任务随后按值取走本周期结果。
struct ChassisCommandSink {
  State* state;
  bool operator()(const ChassisCommand& command) const {
    state->command = command;
    return true;
  }
};

// ==================== 7. 服务装配与周期执行 ====================
// Init 只注册一次，Update 每周期执行；只有这里持有具体的 Center 类型。
class CommandCenterService final {
 public:
  bool Init(const CommandCenterBindings& bindings) {
    if (configured_) {
      const auto& old = state_.bindings;
      return !center_.overflowed() && old.context == bindings.context &&
          old.read_remote == bindings.read_remote && old.read_nuc == bindings.read_nuc &&
          old.select_chassis_mode == bindings.select_chassis_mode;
    }
    if (!bindings.read_remote) return false;
    state_.bindings = bindings;

    // 7.1 注册输入和全局安全规则。
    center_.input<Dr16Input>("dr16", &state_);
    center_.input<NucInput>("nuc", &state_);
    center_.global().safety<RobotStopRule>();

    // 7.2 为底盘输出配置安全命令、同级冲突策略和手动/自动路由。
    // when 决定参与资格，priority 决定有效候选的优先级。
    center_.output<ChassisCommand>("chassis", ChassisCommandSink{&state_})
        .safe<SafeChassisCommand>()
        .arbitrate(cmd::HighestPriority{cmd::TieBreak::RejectConflict})
        .routes(
            cmd::from<Dr16Input, BuildManualChassis>()
                .when<AllowManualChassis>().priority(cmd::Priority::Manual),
            cmd::from<NucInput, BuildAutoChassis>()
                .when<AllowAutoChassis>().priority(cmd::Priority::Auto));
    configured_ = true;
    state_.status.overflowed = center_.overflowed();
    state_.status.initialized = !state_.status.overflowed;
    return state_.status.initialized;
  }

  ChassisCommand Update(uint32_t now_ms) {
    // 7.3 先准备安全默认值；未初始化时也能返回确定的停止命令。
    auto& status = state_.status;
    status.updated_ms = now_ms;
    state_.command = {};
    state_.command.received_ms = now_ms;
    if (!status.initialized) {
      status.safety_reason = CommandSafetyReason::kNotInitialized;
      return state_.command;
    }
    // 7.4 更新全部输入、执行安全检查和路由仲裁，再交给 Sink。
    center_.tick(now_ms).publish();
    status.publish_ok = center_.last_publish_ok();
    status.overflowed = center_.overflowed();
    if (!status.publish_ok || status.overflowed) {
      state_.command = {};
      state_.command.received_ms = now_ms;
    }
    // 7.5 从最终输出记录实际来源和安全原因，而非从候选 Builder 推断。
    status.chassis_source = state_.command.source;
    if (status.overflowed) status.safety_reason = CommandSafetyReason::kConfigurationError;
    else if (!status.publish_ok) status.safety_reason = CommandSafetyReason::kPublishFailed;
    else if (!status.remote_fresh) status.safety_reason = CommandSafetyReason::kRemoteUnavailable;
    else if (!state_.remote_enabled) status.safety_reason = CommandSafetyReason::kNotEnabled;
    else if (status.chassis_source == CommandSource::kNone)
      status.safety_reason = CommandSafetyReason::kNoValidRoute;
    else status.safety_reason = CommandSafetyReason::kNone;
    return state_.command;
  }

  CommandCenterStatus status() const { return state_.status; }

 private:
  // 两路输入、一个输出；总路由槽包含安全规则和 safe builder，留出扩展容量。
  State state_{};
  cmd::Center<2, 1, 8> center_{};
  bool configured_ = false;
};

// ==================== 8. 实例生命周期 ====================
// 固定实例保留布防状态、输入快照和注册关系，避免每周期重新构造。
CommandCenterService& Service() {
  static CommandCenterService service;
  return service;
}

}  // namespace

// ==================== 9. 对外接口 ====================
// 外部只依赖消息和服务接口，不接触输入适配器、路由或 Center 内部对象。
bool InitCommandCenter(const CommandCenterBindings& bindings) {
  return Service().Init(bindings);
}

ChassisCommand UpdateCommandCenter(uint32_t now_ms) {
  return Service().Update(now_ms);
}

CommandCenterStatus GetCommandCenterStatus() { return Service().status(); }

}  // namespace app
