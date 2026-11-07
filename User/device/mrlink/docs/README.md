# MRLink 接入与迁移

通用协议核心在 `User/component/communication/mrlink`，构建目标 `mr_component_mrlink`。
只链接该目标不需要 HAL、CMSIS、RTOS、SPSC 容器或动态内存。设备目录中的旧头只是转发入口。
UART/FDCAN/USB 通道是单独的平台适配，应用显式持有并连接资源。

## 协议与并发

帧格式保留 `[header0][header1][payload_len:u8][cmd:u8][payload][CRC16_lo][CRC16_hi]`。
默认头为 ASCII MR，CRC 可选，反射多项式 0x1021、初值 FFFF、无 xorout。
payload 上限 64，最大帧 70；RX 存储至少最大帧加一个字节，TX 至少最大帧。
C 与 C++ 共享同一真实布局，实例及外部存储禁止使用期间复制/移动。

`Init` 后默认为同一执行上下文。ISR 喂入、任务消费时，启动接收前调用
`SetSynchronization` 安装配对的 IRQ save/restore hooks；库不隐式关闭全局中断。
只支持一个生产者、一个消费任务。Reset/重绑定要求两端停用。
PushBytes 部分接收返回 `MRLINK_ERR_BUF_FULL`，统计记录实际接收/丢弃数；它不调用业务回调。
所有业务和错误回调都在 Parse/Dispatch 内、临界区外执行。

Parse 返回一个有效 wire 帧，同时调用该帧 handler；长度订阅不匹配也消费该帧并报告错误。
Dispatch 默认预算 16 帧/1024 消费字节；剩余数据下次继续处理。
回调可以 PushBytes/Build，递归 Parse/Dispatch、注册、注销和 Reset 返回 BUSY。
不在回调中重新初始化实例。回调应短且非阻塞；预算无法限制业务回调自身耗时。
使用 `GetStatsSnapshot` 读取任务一致快照。payload 是字节序列，不能强转为一个 C++ 对象。

## C++ 消息接口

只保留 `mr::link::BasicInstance<容量>`、默认 `Instance`、自带缓冲的 `Bus<>`。
`mrlink_lite.hpp` 转发至同一实现；traits 和 codec 特化统一放到 `mr::link`。
不再使用 std::function；handler 为 `void(const T&, void*)`，状态由应用 ctx 提供。
`MessageTraits<T>::topic` 或 `T::topic` 指定命令；`MessageCodec<T>` 显式给出
`size`、`Encode(const T&, uint8_t*, uint16_t)` 和 `Decode(const uint8_t*, uint16_t, T*)`。
`wire::Put/GetU16/U32/F32` 提供小端字段编解码，F32 要求 IEEE754 binary32。

旧的 native struct memcpy 只有在应用明确特化
`template<> struct MessageCodec<Old> : LegacyNativeCodec<Old> {};` 后才启用。
该兼容方式仍要求收发双方的端序、padding、字段布局相同，不能作为跨平台协议格式。
旧 `Latest<T>()` 内部指针删除，使用 `CopyLatest<T>(&value, &sequence)`。
注销/替换订阅同时清除缓存，注册失败保留旧订阅及快照。

传输通过 `BindTransport` 或 `SetWriter` 注入。send 返回 true 前必须复制或消费全部字节。
直接持有 Build 的 TX scratch 给 DMA 不符合接口契约；使用拥有发送缓冲的硬件 channel，
或在应用自行实现固定发送队列。PollRx 会限制读取量到 RX 空间，并在无新输入时继续 Dispatch。


## 平台通道与 DMA 接入

实现移至 `User/bsp/mrlink`，`bsp/adapters/mrlink.hpp` 将它转换为 portable Transport，
并提供配对的 IRQ save/restore 同步钩子。旧 device 通道头只做转发。
所有 channel 必须零初始化，使用期间禁止复制/移动；先停用消费任务，再调用 Deinit。
Deinit 失败必须保留对象/缓冲，重试或处理硬件故障，不能释放后让 DMA/ISR 继续访问。

UART 由 board 在启动时登记硬件，十个中性槽位均有 trampoline；初始化独占 BSP 回调资源，和 DR16 等设备冲突时明确失败。
重复初始化返回 BUSY，不能以初始化覆盖活动 DMA。TX 使用对齐的独立缓冲，完成或确认取消后才可复用；
RX 从 DMA 槽位分段复制，scratch 小于接收块时不丢尾部。
UART 错误 ISR 只尝试非阻塞重启；需要 HAL abort 的恢复延迟到任务 Poll/Send。
TX 通知仅作为短通知信号，可能来自 ISR、错误恢复任务或 FDCAN 提交上下文；不要在通知里重入通道 API。

USB 明确选择 synchronous_send（返回前已消费/复制）或异步提交。异步模式必须提供
transactional register_tx_callbacks 和同步 stop：失败注册不保留回调，完成/错误通知表示已停止使用
TX 缓冲，stop 成功保证 DMA 和回调均已解除。库拥有异步 TX 数据直到这些条件之一满足。
板级 USB 回调负责 USB 控制器自己的 DMA/Cache 策略，send 不得阻塞。

FDCAN 校验 ID/数据帧种类，按 BSP 的实际字节数接收；保留不足 scratch 的剩余数据。
其 tx_done 仅表示 BSP 已复制/接纳发送，并非总线 ACK。Deinit 释放自己登记的接收 ID；
必须先停用所有该 ID 的应用读者，不能在另一个任务仍阻塞读队列时删除。
最大完整协议帧不能超过 channel.max_frame_size（CAN FD 最大 64）。CRC 开启时 payload 最大 58；
classic CAN 最大完整帧 8，BSP 会拒绝超过该控制器帧格式的长度。
当前没有 CAN 分片；FDCAN padding 按噪声消费。BSP 的 ID parser 若自定义，必须为此通道保持原始 ID 键。

DMA 统一通过 `bsp/dma.h` 的 BSP_DMA_Configure 安装 board hook。未配置时拒绝 UART/SPI DMA，
不会假定 H7 的 DTCM/SRAM、Cache 和 MPU 布局。hook 验证 DMA 可达内存，按板级策略处理 prepare TX、
prepare RX 和 complete RX；失败的 complete 不通知数据就绪。
只支持 normal DMA；circular DMA 需要另一套写指针/事件协议，当前显式拒绝。
Cacheable RX 整个缓冲及其行填充在完成/成功取消前归 DMA 独占，half callback 只能读取已完成部分。

BMI088 改为 32 字节对齐的独占 staging，再复制到 CPU 原始数据区；DR16 也使用独占 32 字节缓冲。
二者传输启动失败不报告成功，超时取消失败保持所有权；DR16_Deinit 显式解除资源登记。
这些静态 DMA buffer 必须由 board 链接到控制器可达内存，单纯对齐不能让 DTCM 变得可达。

可编译的显式装配例子见 `examples/mrlink`。应用需要加入 UART/SPI 的 `bsp/dma.c`；
平台通道源为 `bsp/mrlink/channel.c`、`channel_uart.c`、`channel_usb.c`、`channel_fdcan.c`，
并链接原 BSP FDCAN/RTOS/内存/计时及 HAL 支持。不要再编译已删除的 device/mrlink/src/mrlink.c。
