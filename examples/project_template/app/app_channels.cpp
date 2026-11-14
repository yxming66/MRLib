#include "app/app_channels.hpp"
#include <cmath>
#include <type_traits>
#include "bsp/time.h"
#include "FreeRTOS.h"
#include "queue.h"

namespace {
StaticQueue_t remote_control, command_control, status_control;
alignas(8) uint8_t remote_storage[sizeof(app::RemoteInput)];
alignas(8) uint8_t command_storage[sizeof(app::ChassisCommand)];
alignas(8) uint8_t status_storage[sizeof(app::ChassisStatus)];
QueueHandle_t remote_queue = nullptr;
QueueHandle_t command_queue = nullptr;
QueueHandle_t status_queue = nullptr;
static_assert(std::is_trivially_copyable_v<app::RemoteInput>);
static_assert(std::is_trivially_copyable_v<app::ChassisCommand>);
static_assert(std::is_trivially_copyable_v<app::ChassisStatus>);
}  // namespace

namespace app {
bool InitChannels() {
  if (remote_queue || command_queue || status_queue) return false;
  remote_queue = xQueueCreateStatic(1, sizeof(RemoteInput), remote_storage, &remote_control);
  command_queue = xQueueCreateStatic(1, sizeof(ChassisCommand), command_storage, &command_control);
  status_queue = xQueueCreateStatic(1, sizeof(ChassisStatus), status_storage, &status_control);
  return remote_queue && command_queue && status_queue;
}
bool PublishRemoteInput(const RemoteInput& input) {
  if (!remote_queue || !std::isfinite(input.left_x) || !std::isfinite(input.left_y) ||
      !std::isfinite(input.right_x) || !std::isfinite(input.right_y) ||
      !std::isfinite(input.channel5)) return false;
  return xQueueOverwrite(remote_queue, &input) == pdPASS;
}
bool ReadRemoteInput(RemoteInput& input) {
  return remote_queue && xQueuePeek(remote_queue, &input, 0) == pdPASS;
}
bool PublishChassisCommand(const ChassisCommand& command) {
  if (!command_queue || !std::isfinite(command.vx_mps) ||
      !std::isfinite(command.vy_mps) || !std::isfinite(command.wz_rad_s)) return false;
  auto local = command;
  local.received_ms = BSP_TIME_Get_ms();
  return xQueueOverwrite(command_queue, &local) == pdPASS;
}
bool TryReceiveChassisCommand(ChassisCommand& command) {
  return command_queue && xQueueReceive(command_queue, &command, 0) == pdPASS;
}
bool PublishChassisStatus(const ChassisStatus& status) {
  return status_queue && xQueueOverwrite(status_queue, &status) == pdPASS;
}
bool ReadChassisStatus(ChassisStatus& status) {
  return status_queue && xQueuePeek(status_queue, &status, 0) == pdPASS;
}
}  // namespace app
