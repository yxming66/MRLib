# 项目时间源

User/bsp/time.c 不再引用 TIM2、htim2 或 HAL tick。项目直接链接实现 bsp/time_source.h 的三个函数，无运行期注册表或对象工厂：

```c
bool Board_TimeIsReady(void);
uint64_t Board_TimeNowUs(void);
int8_t Board_TimeDelayMs(uint32_t ms);
```

board 负责在设备使用前初始化时间源，文档说明精度、溢出、中断和休眠约束。未 ready 时 BSP 读数为 0、延时返回错误。Board_TimeNowUs 必须提供同一单调微秒 epoch，并能被任务及声明支持的 ISR 安全读取。Board_TimeDelayMs 用于调度器启动前的等待，返回 BSP_OK/BSP_ERR，不调用 RTOS。

BSP_TIME_Get/Get_us 返回 64 位微秒；Get_ms 从同一个微秒 epoch 除以 1000 再转 uint32，约 49.7 天回绕。计算有界时间差仍采用无符号减法。不得混用不同时间源的时间戳。微秒 epoch 不再随 HAL 的 32 位 tick 回绕。

RTOS running 时 BSP_TIME_Delay_ms 使用 CMSIS tick 频率向上取整，至少等待 1 tick，拒绝换算溢出；启动前转给 board。微秒延时仍为忙等，只供极短硬件时序，在任务或启动代码中使用，不在 ISR、屏蔽 IRQ 或暂停时间源时使用，也不能代替周期任务调度。

## H723 示例

examples/h7_integration/board/time_source.c 继续复用 Cube HAL 的 TIM2：1 MHz、向上计数、ARR=999、更新中断开启，不额外占用定时器。main 在最终时钟设置完成后、其他外设初始化之前调用 Board_TimeInit；它检查 APB1 /2、实际 HCLK、PSC、ARR 和运行状态。HAL_TIM_PeriodElapsedCallback 直接转交 Board_TimeOnUpdate，后者只接受准确的 &htim2，并继续 HAL_IncTick，保持 Cube HAL 延时工作。

独立的 64 位毫秒累计值在每次 TIM2 更新回调递增。读取时用短临界区取得累计值、更新标志和计数器，补偿尚未进入回调以及读取期间发生的一次溢出。初始化以当时 HAL tick 为起点；ready 后不再依赖 HAL tick 的数值，HAL tick 跨 UINT32_MAX 不影响 64 位时间。

Board_TimeInit 将 TIM2_IRQn 优先级设置为 0：HAL handler 先清更新标志、再进入用户回调，其他普通 ISR 不能在两者之间抢占并读取旧 epoch。TIM2 回调不得调用 FreeRTOS API。NMI/HardFault 不支持调用此时间源。

TIM2 中断最坏延迟必须小于 1 ms，包括 PRIMASK/FAULTMASK 屏蔽区、优先级 0 的其他 ISR 及 TIM2 自身处理时间。一个更新标志无法记录多次溢出，超过该约束会丢时间。读取函数保护 64 位共享值，但不意味着可以无限期关闭中断。

此实现不支持运行期变更时钟、重配 TIM2、HAL_SuspendTick、停止 TIM2 的 tickless idle 或 Stop 低功耗后的连续计时。需要这些能力时替换 board 实现并验证其恢复策略，不修改模块或设备接口。启动前 board 延时拒绝 ISR/屏蔽 IRQ 上下文。

## 接入与维护

项目显式编译 board/time_source.c 并链接库的 bsp/time.c；保留本项目的 time_source.h 初始化声明。更换时间源只维护这份板级实现及相应 IRQ/时钟启动逻辑，无需改业务、生成模板或平行参数格式。新板可以使用其他硬件计数器，但必须满足相同时间单位、单调性与上下文合同。

主机测试验证未就绪、错误配置、pending 溢出、读取期间溢出、HAL tick 回绕、IRQ 状态保留及非法启动前延时；H723 示例已编译和链接。尚未测量板上精度、最坏 IRQ 延迟或低功耗行为。
