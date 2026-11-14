# DR16 remote control task example

`RemoteControlTask` adapts the library's DR16 driver into a board-neutral, latest-value `RemoteInput` snapshot. It runs as its own blocking FreeRTOS thread because `DR16_WaitDmaCplt` waits on UART callbacks; do not run it inside a fast `PeriodicTask` or from an ISR.

## Project integration

1. Configure the receiver UART in Cube for the DR16 protocol (typically 100 kbaud, 8 data bits, even parity, 1 stop bit). Enable the RX DMA in normal mode and the UART/DMA interrupts.
2. Keep `board/uart_endpoints.c` and register the receiver endpoint, for example `{"uart5", &huart5}`. The name must match `board::bindings::kRemoteUart`.
3. Install the board's `BSP_DMA_Configure` validation/cache hook before starting the receiver. It must accept the DR16 driver's private aligned RX buffer in DMA-accessible memory and perform the required H7 cache maintenance. DMA start is rejected if the BSP has no hook.
4. Add `app/remote_control_task.cpp` and `User/device/dr16.c` to the project sources. Also compile `User/bsp/uart.c` and `User/bsp/dma.c`, and include the UART endpoint source in `board_adapter`.
5. `app_channels.cpp` now creates the one-slot remote input queue with the other app channels. Start the remote task once from the running `Task_Init` after `InitChannels` and after UART/DMA setup:

```cpp
#include "app/remote_control_task.hpp"
#include "board/bindings.hpp"

static app::RemoteControlTask remote_task;
const BSP_UART_t remote_uart = BSP_UART_FindEndpoint(board::bindings::kRemoteUart);
if (!remote_task.Start(remote_uart)) {
  // Record failure; all consumers must keep their outputs disabled.
}
```

The DR16 driver is a single-instance driver. `DR16_Init` runs inside `Task_RemoteControl`, so its UART callbacks signal the correct thread. Do not initialize DR16 elsewhere or share that UART with another driver.

## Consuming the snapshot

The task converts the four sticks and channel 5 to normalized floats and converts both switches to `RemoteSwitch`. It publishes an offline zeroed snapshot after a receive timeout or parse failure. Consumers should still check the timestamp in case the receive task or queue stops updating:

```cpp
app::RemoteInput remote{};
const uint32_t now_ms = BSP_TIME_Get_ms();
const bool have_remote = app::ReadRemoteInput(remote);
const bool fresh = have_remote && remote.online &&
    uint32_t(now_ms - remote.updated_ms) <= 30u;

app::ChassisCommand command{};
if (fresh && remote.left_switch == app::RemoteSwitch::kUp) {
  command.vx_mps = remote.left_y * max_forward_mps;
  command.vy_mps = remote.left_x * max_left_mps;
  command.wz_rad_s = remote.right_x * max_yaw_rate_rad_s;
  command.enabled = true;
}
app::PublishChassisCommand(command);
```

The axis signs and switch that enables motion are project choices; verify them against the transmitter and robot coordinate frame. `RemoteInput` is an in-memory app message, not a wire protocol. A different receiver can publish this same normalized structure, keeping chassis and gimbal tasks independent of the radio model. For a gimbal, map sticks to desired yaw/pitch and publish its command only while the snapshot is fresh. On stale/offline input, publish a disabled or neutral command so downstream tasks can enforce their own safe-stop policy.
