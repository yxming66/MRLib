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
