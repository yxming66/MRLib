# DR16 接收任务

RemoteControlTask 独立阻塞等待 UART DMA 通知，将数据转成 RemoteInput 最新快照。它不选择输入源、不直接发布底盘命令；映射和仲裁见 [CmdCenter 示例](cmd_center.md)。

## 接入步骤

1. Cube 配置 UART 为 DR16 的 100 kbaud、8 个有效数据位、偶校验、1 停止位。STM32 HAL 通常需 9 位字长加偶校验才能得到 8 个有效数据位。启用 RX DMA normal 模式及 UART/DMA 中断。
2. 修改 board/uart_endpoints.c 的句柄和名称，与 board::bindings::kRemoteUart 一致。在全部 MX 初始化之后、启动调度器前调用 Board_RegisterUartEndpoints。
3. 链接脚本将驱动的 .dma_buffer 放在 DMA 可访问 SRAM，不能放 DTCM。启动接收任务前安装 BSP_DMA_Configure 校验和缓存维护钩子，满足 User/bsp/dma.h 的契约。缺少钩子时 BSP 拒绝 DMA；模板不替项目猜测内存布局。
4. 复制模板 CMake 后，配置时添加 -DMR_EXAMPLE_ENABLE_DR16=ON。它加入接收任务、dr16.c、BSP UART/DMA、UART 登记源，并启用 Task_Init 的启动代码。

Task_Init 已包含启动调用及返回值检查：

```cpp
app::RemoteControlTask::Instance().Start(
    BSP_UART_FindEndpoint(board::bindings::kRemoteUart));
```

DR16_Init 在接收线程内部执行，才能保存正确的通知目标。不要在别处再次初始化 DR16 或共享此 UART；驱动是单实例，任务也只提供一个固定实例。

Start 成功只表示线程已创建。线程内驱动初始化失败时保存离线快照并退出；接收超时/解析失败也保存清零、online=false 的快照。接收任务停止时，CmdCenter 仍会检查时间戳，超时后停止并清除布防。

## 快照如何进入 CmdCenter

```cpp
bool ReadRemote(void*, app::RemoteInput& input) {
  return app::RemoteControlTask::Instance().ReadLatest(input);
}
```

接收任务独占写，StoreLatest/ReadLatest 使用 FreeRTOS 短临界区复制整个结构。读取不移除快照，支持多个观察者，只能在任务上下文调用。Center 读取回调不等待下一帧，不阻塞周期任务。

摇杆/channel5 为归一化值，拨杆为 RemoteSwitch，updated_ms 是本板接收时间。它是板内 C++ 消息，不可直接 memcpy 为板间协议。

默认 MR_EXAMPLE_ENABLE_DR16=OFF：不编译或启动 DR16，ReadRemote 返回不可用，Center 输出安全命令。NUC 需要项目另外实现接收快照和回调。
