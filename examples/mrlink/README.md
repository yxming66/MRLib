# 手动装配 MRLink

`app/telemetry.hpp` 是可移植的六字节消息定义与小端 codec；`board/uart_link.hpp` 显式组合
库对象、通道、UART DMA 槽位；`board/h7_dma_policy.hpp` 是按实际链接器/MPU区域配置的 H7 Cache hook 示例。
没有固定的可达 RAM 地址，board 必须从实际 DMA 控制器与链接结果给出 DmaRegion 表。

启动顺序：

1. HAL/Cube 初始化 UART、normal DMA 和相关中断；board 调用 BSP_UART_Configure 登记命名硬件端点，详见 [UART 绑定](../../docs/UART_BINDINGS.md)。中断优先级应满足所用 RTOS 的 ISR API 限制。
2. 把 ChannelAndLink、BMI088/DR16 等 DMA 缓冲放在控制器可达内存，Cacheable 缓冲独占整行。
   初始化静态 DmaPolicy 与 DmaRegion 表，调用 BSP_DMA_Configure(board::SynchronizeDma, &policy)。
3. 声明长期存活且禁止复制的 board::ChannelAndLink，检查 Begin(BSP_UART_FindEndpoint("usart1")) 返回值；使用 YAML 时可传生成的逻辑端点查找结果。
   若它在初始化通道后启动失败，仍须 Stop 成功才能释放。
4. 在同一消费任务注册 link.OnLatest<app::Telemetry>()，循环调用 link.Poll()，
   通过 CopyLatest(&value, &sequence) 获取新快照。要发送时调用 link.Publish(app::Telemetry{...})，
   返回 false 表示当前未接受，是否重试或丢弃由 app 决定。
5. 停止消费任务后检查 Stop()；失败保持对象和缓冲存活，处理故障后重试。

消费任务中定义回调可用静态函数 `void OnTelemetry(const app::Telemetry&, void* ctx)`；
将业务状态显式作为 ctx 传入 OnLatest，业务不接触 DMA/库内部缓存。

协议目标链接 `mr_component_mrlink`；HAL 通道按接入说明显式加入 board 的源文件。
`py -3 tools/check_user_io_h7.py --board-root <H723 Cube工程目录>` 用真实外部头检查该例子和实际 User C 源；
它只在库的 build 目录生成对象文件，不修改参考 Cube 工程。它验证编译，不代表完整固件链接或硬件运行。
