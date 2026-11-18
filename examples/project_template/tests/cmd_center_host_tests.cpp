#include "app/cmd_center.hpp"
#include "app/robot_config.hpp"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string_view>

namespace {
struct Inputs {
  app::RemoteInput remote{};
  app::NucInputSnapshot nuc{};
  bool remote_available = true;
  bool nuc_available = true;
  app::ChassisMode selected_mode = app::ChassisMode::kAuto;
  uint32_t selector_calls = 0;
};

app::ChassisMode CustomSelectMode(void* context, const app::RemoteInput&) {
  auto& inputs = *static_cast<Inputs*>(context);
  ++inputs.selector_calls;
  return inputs.selected_mode;
}

bool ReadRemote(void* context, app::RemoteInput& out) {
  auto& inputs = *static_cast<Inputs*>(context);
  if (!inputs.remote_available) return false;
  out = inputs.remote;
  return true;
}

bool ReadNuc(void* context, app::NucInputSnapshot& out) {
  auto& inputs = *static_cast<Inputs*>(context);
  if (!inputs.nuc_available) return false;
  out = inputs.nuc;
  return true;
}

void Near(float actual, float expected) { assert(std::fabs(actual - expected) < 1.0e-5f); }

void Safe(const app::ChassisCommand& command) {
  assert(!command.enabled && !command.stair_enabled && !command.input_fresh);
  assert(command.source == app::CommandSource::kNone);
  Near(command.vx_mps, 0.0f);
  Near(command.vy_mps, 0.0f);
  Near(command.wz_rad_s, 0.0f);
  Near(command.front_target_rate_rad_s, 0.0f);
  Near(command.support_target_rate_rad_s, 0.0f);
  Near(command.left_track_speed_rad_s, 0.0f);
  Near(command.right_track_speed_rad_s, 0.0f);
}

app::ChassisCommand Sample(Inputs& inputs, uint32_t now, app::RemoteSwitch position) {
  inputs.remote.updated_ms = now;
  inputs.remote.left_switch = position;
  ++inputs.remote.sequence;
  return app::UpdateCommandCenter(now);
}

void TestManual(Inputs& inputs) {
  using app::RemoteSwitch;
  inputs.remote.online = true;
  inputs.remote.right_switch = RemoteSwitch::kMiddle;
  Safe(Sample(inputs, 10, RemoteSwitch::kMiddle));  // 启动时中档不能直接使能。
  Safe(Sample(inputs, 12, RemoteSwitch::kUp));
  assert(app::GetCommandCenterStatus().armed);
  inputs.remote.right_y = 0.5f;
  inputs.remote.right_x = -0.25f;
  inputs.remote.left_x = 0.5f;
  auto command = Sample(inputs, 14, RemoteSwitch::kMiddle);
  assert(command.enabled && command.source == app::CommandSource::kDr16);
  Near(command.vx_mps, 0.75f);
  Near(command.vy_mps, 0.375f);
  Near(command.wz_rad_s, -2.0f);
  inputs.remote.left_y = 0.25f;
  command = Sample(inputs, 16, RemoteSwitch::kDown);
  assert(command.stair_enabled);
  Near(command.vx_mps, 0.75f);
  Near(command.wz_rad_s, 0.0f);
  Near(command.front_target_rate_rad_s, 2.0f);
  Near(command.support_target_rate_rad_s, 4.0f);
  Near(command.left_track_speed_rad_s, 7.5f);
  Near(command.right_track_speed_rad_s, 7.5f);
  inputs.remote.right_y = 0.05f;
  inputs.remote.right_x = -0.049f;
  command = Sample(inputs, 18, RemoteSwitch::kMiddle);
  Near(command.vx_mps, 0.0f);
  Near(command.vy_mps, 0.0f);
  command = app::UpdateCommandCenter(20);
  assert(command.received_ms == 20 && command.source_received_ms == 18);
  Safe(app::UpdateCommandCenter(119));  // 原始输入超时，不能被生成时间掩盖。
  assert(!app::GetCommandCenterStatus().armed);
  Safe(Sample(inputs, 120, RemoteSwitch::kMiddle));
  Safe(Sample(inputs, 122, RemoteSwitch::kUp));
  assert(Sample(inputs, 124, RemoteSwitch::kMiddle).enabled);
  inputs.remote_available = false;
  Safe(app::UpdateCommandCenter(126));
  inputs.remote_available = true;
  Safe(Sample(inputs, 128, RemoteSwitch::kMiddle));
  Safe(Sample(inputs, 130, RemoteSwitch::kUp));
  inputs.remote.right_y = std::numeric_limits<float>::quiet_NaN();
  Safe(Sample(inputs, 132, RemoteSwitch::kMiddle));
  inputs.remote.right_y = 0.0f;
  Safe(Sample(inputs, UINT32_MAX - 5u, RemoteSwitch::kUp));
  inputs.remote.left_switch = RemoteSwitch::kMiddle;
  assert(app::UpdateCommandCenter(3u).enabled);  // 本板毫秒计数回绕。
  inputs.remote.right_switch = RemoteSwitch::kDown;
  Safe(Sample(inputs, 5u, RemoteSwitch::kMiddle));  // 未绑定 NUC 读取回调。
  assert(app::GetCommandCenterStatus().chassis_mode == app::ChassisMode::kAuto);
  inputs.remote.right_switch = RemoteSwitch::kMiddle;
  assert(Sample(inputs, 7u, RemoteSwitch::kMiddle).source == app::CommandSource::kDr16);
}

void TestMultipleSources(Inputs& inputs) {
  using app::RemoteSwitch;
  inputs.remote.online = true;
  inputs.remote.right_switch = RemoteSwitch::kMiddle;
  inputs.nuc = {0.2f, 0.3f, 0.4f, 10u, 1u, true};
  Safe(Sample(inputs, 10, RemoteSwitch::kUp));
  inputs.remote.right_y = 0.5f;
  auto command = Sample(inputs, 12, RemoteSwitch::kMiddle);
  // NUC 在线且 Auto 优先级更高，但未获模式授权，仍使用手动。
  assert(command.source == app::CommandSource::kDr16);
  Near(command.vx_mps, 0.75f);
  inputs.remote.right_switch = RemoteSwitch::kDown;
  command = Sample(inputs, 14, RemoteSwitch::kMiddle);
  assert(command.source == app::CommandSource::kNuc);
  Near(command.vx_mps, 0.2f);
  assert(command.source_received_ms == 10);
  assert(!command.stair_enabled);
  inputs.nuc.vx_mps = 100.0f;
  inputs.nuc.vy_mps = -100.0f;
  inputs.nuc.wz_rad_s = 100.0f;
  command = Sample(inputs, 16, RemoteSwitch::kMiddle);
  Near(command.vx_mps, app::params::kMaxChassisVxMps);
  Near(command.vy_mps, -app::params::kMaxChassisVyMps);
  Near(command.wz_rad_s, app::params::kMaxChassisWzRadS);
  Safe(Sample(inputs, 111, RemoteSwitch::kMiddle));  // NUC 超时不退回手动。
  auto status = app::GetCommandCenterStatus();
  assert(status.chassis_mode == app::ChassisMode::kAuto);
  assert(status.chassis_source == app::CommandSource::kNone && !status.nuc_fresh);
  inputs.nuc.received_ms = 112;
  inputs.nuc.vx_mps = std::numeric_limits<float>::infinity();
  Safe(Sample(inputs, 112, RemoteSwitch::kMiddle));
  inputs.remote.right_switch = RemoteSwitch::kMiddle;
  assert(Sample(inputs, 114, RemoteSwitch::kMiddle).source == app::CommandSource::kDr16);
  inputs.remote.right_switch = RemoteSwitch::kDown;
  inputs.nuc.vx_mps = 0.2f;
  inputs.nuc.received_ms = 116;
  assert(Sample(inputs, 116, RemoteSwitch::kMiddle).source == app::CommandSource::kNuc);
  inputs.remote.online = false;
  Safe(Sample(inputs, 118, RemoteSwitch::kMiddle));  // 自动也受遥控全局授权约束。
  inputs.remote.online = true;
  Safe(Sample(inputs, 120, RemoteSwitch::kMiddle));
  Safe(Sample(inputs, 122, RemoteSwitch::kUp));
  assert(Sample(inputs, 124, RemoteSwitch::kMiddle).enabled);
  inputs.remote.right_switch = RemoteSwitch::kUp;
  Safe(Sample(inputs, 126, RemoteSwitch::kMiddle));
  inputs.remote.right_switch = RemoteSwitch::kUnknown;
  Safe(Sample(inputs, 128, RemoteSwitch::kMiddle));
  inputs.remote.right_switch = RemoteSwitch::kDown;
  assert(Sample(inputs, 130, RemoteSwitch::kDown).source == app::CommandSource::kNuc);
  assert(!app::UpdateCommandCenter(130).stair_enabled);  // 自动模式不授权上台阶机构。
  inputs.remote.right_switch = RemoteSwitch::kMiddle;
  const auto stair = Sample(inputs, 132, RemoteSwitch::kDown);
  assert(stair.source == app::CommandSource::kDr16 && stair.stair_enabled);
}
}  // namespace

int main(int argc, char** argv) {
  Safe(app::UpdateCommandCenter(0));
  assert(!app::InitCommandCenter({}));
  const bool multiple = argc > 1 && std::string_view(argv[1]) == "multi";
  const bool no_selector = argc > 1 && std::string_view(argv[1]) == "no-selector";
  const bool custom_selector = argc > 1 && std::string_view(argv[1]) == "custom-selector";
  Inputs inputs;
  const app::CommandCenterBindings bindings{
      &inputs, ReadRemote, (multiple || custom_selector) ? ReadNuc : nullptr,
      no_selector ? nullptr : (custom_selector ? CustomSelectMode : app::SelectChassisMode)};
  assert(app::InitCommandCenter(bindings));
  assert(app::InitCommandCenter(bindings));
  auto changed = bindings;
  changed.context = nullptr;
  assert(!app::InitCommandCenter(changed));
  if (custom_selector) {
    inputs.remote.online = true;
    inputs.remote.right_switch = app::RemoteSwitch::kMiddle;
    inputs.nuc = {0.2f, 0.3f, 0.4f, 10u, 1u, true};
    Safe(Sample(inputs, 10, app::RemoteSwitch::kUp));
    assert(inputs.selector_calls == 0);  // 未使能时不执行模式回调。
    assert(Sample(inputs, 12, app::RemoteSwitch::kMiddle).source == app::CommandSource::kNuc);
    assert(inputs.selector_calls == 1);  // 自定义回调决定模式，无内置拨杆后备策略。
    inputs.selected_mode = app::ChassisMode::kManual;
    inputs.remote.right_switch = app::RemoteSwitch::kDown;
    assert(Sample(inputs, 14, app::RemoteSwitch::kMiddle).source == app::CommandSource::kDr16);
    inputs.nuc_available = false;
    inputs.selected_mode = app::ChassisMode::kAuto;
    Safe(Sample(inputs, 16, app::RemoteSwitch::kMiddle));
    inputs.nuc_available = true;
    inputs.nuc.valid = false;
    Safe(Sample(inputs, 18, app::RemoteSwitch::kMiddle));
  } else if (no_selector) {
    inputs.remote.online = true;
    inputs.remote.right_switch = app::RemoteSwitch::kMiddle;
    Safe(Sample(inputs, 10, app::RemoteSwitch::kUp));
    Safe(Sample(inputs, 12, app::RemoteSwitch::kMiddle));
    const auto status = app::GetCommandCenterStatus();
    assert(status.armed && status.chassis_mode == app::ChassisMode::kDisabled);
    assert(status.safety_reason == app::CommandSafetyReason::kNoValidRoute);
  } else if (multiple) TestMultipleSources(inputs);
  else TestManual(inputs);
  assert(app::GetCommandCenterStatus().publish_ok);
  assert(!app::GetCommandCenterStatus().overflowed);
}
