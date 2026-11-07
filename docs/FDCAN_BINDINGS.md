# FDCAN 板级接入

库保留 CAN 统一接口和 H7 HAL FDCAN 实现，但不再引用 hfdcan1/2/3、不配置硬件过滤器。项目 board 负责 Cube 初始化、位时序、消息 RAM 布局/容量、过滤器、FIFO 路由及 NVIC；app 选择命名总线。换用途只修改 board/app，不增加库内用途枚举。

## 普通 C 登记

在所有相关 MX_FDCANx_Init 完成后、控制器启动前执行：

```c
#include "bsp/fdcan.h"
#include "fdcan.h"  /* 项目 Cube 头 */

int Board_RegisterFdcanEndpoints(void) {
    const BSP_FDCAN_Endpoint_t buses[] = {
        {"motor_bus", &hfdcan2, BSP_FDCAN_RX_FIFO1},
    };
    if (BSP_FDCAN_Configure(buses, 1) != BSP_OK) return 0;
    /* 此处调用项目的 HAL_FDCAN_ConfigFilter/ConfigGlobalFilter，检查结果。
       所有接收过滤规则必须路由到上面声明的 FIFO。 */
    return Board_ConfigureMotorBusFilters();
}
```

Board_ConfigureMotorBusFilters 由项目定义。完整可编译实现见 examples/h7_integration/board/fdcan_endpoints.c。

登记表复制到容量为 3 的固定数组；名称字符串和 HAL 句柄须长期有效。只允许启动期成功登记一次，不支持运行期换绑。BSP_FDCAN_1/2/3（以及 BSP_CAN_1/2/3）表示登记顺序的槽位，不再表示物理 FDCAN 编号。推荐在登记后通过 BSP_CAN_FindEndpoint("motor_bus") 选择，并检查查找结果。不要在 main 之前的全局构造中缓存查找结果；H7 示例采用函数内 static，仍为固定存储。

Configure 整表检查通过才发布，拒绝重名、同一物理实例的重复/复制句柄、非 READY 状态、无效 FIFO、发送资源不足，以及 HAL 已计算的消息 RAM 区间重叠。所选接收 FIFO 必须有元素；Classic 要求至少 8 字节 RAM 元素，FD 要求 64 字节，才能支持本接口声明的最大载荷。HAL 初始化后不得再修改句柄配置或消息 RAM 布局。

## 过滤器和共享 RAM

登记可在过滤器 RAM 写入之前执行，让重叠/别名检查先运行。它不撤销 Cube 已执行的 HAL_Init，也不替代 board 对共享 RAM 总容量及未登记控制器的所有权检查。应先规划所有控制器的 MessageRAMOffset 与资源数，再初始化，避免 HAL_Init 清除其他控制器的 RAM。

H723 示例检查 RAM 位于本芯片 SRAMCAN 的 0x2800 字节范围，采用标准/扩展各一个 mask 过滤器，索引均为 0：两类过滤器有独立索引空间。示例广泛接收数据帧，由软件 ID 订阅分流；实际项目可改成更严格的规则。过滤器路由必须与端点 rx_fifos 一致，不能把帧送入未启用通知的 FIFO。登记后若任一过滤器配置失败，示例直接进入 Error_Handler，不继续初始化或允许应用启动；这不是支持重复调用的跨硬件事务。

## 启动、接收与恢复

在 RTOS 已运行的任务中，先调用一次 BSP_FDCAN_Init，再登记设备/接收 ID。BSP_CAN_Init 兼容映射仍可使用。Init 先准备软件状态和互斥量，再启用通知、启动已登记控制器；失败则撤销本次通知并停止本次控制器。HAL 回滚失败会锁住初始化，需复位；回滚成功可在同一登记表上重试。board 不应自行提前 Start 或重初始化这些控制器。

Cube IRQ 继续调用 HAL_FDCAN_IRQHandler；库定义 HAL 回调，不要在项目重复定义。NVIC 优先级必须允许 CMSIS/FreeRTOS 的 ISR 队列操作。回调只分发准确登记的句柄与对应 FIFO，不扫描其他总线；每次消耗进入回调时的 FIFO 帧数快照，最多 64 帧，期间新到帧交给后续中断。持续高负载的 IRQ 时间和丢帧仍需板上测量。

RegisterCallback 是附加观察回调，不替换 BSP 内部接收，也不能覆盖已有观察者；UnregisterCallback 必须匹配原函数。观察者和 ID parser 在 ISR 内执行，不得阻塞。接收节点、latest 快照及队列所有权仍由 BSP 管理；latest 读写和订阅链表发布使用 IRQ 保护。

任务中的发送/读取按需尝试恢复；显式 BSP_FDCAN_Recover 也可请求恢复。恢复只操作对应已登记控制器，不调用 HAL_Init、不重写过滤器或共享 RAM，不伪造 HAL READY 状态。Abort/Stop/通知/Start 错误使恢复失败，发送随即返回错误；接收读取仍可取得之前缓存的帧，不能把读取成功作为总线恢复成功的证明。恢复丢弃本总线待发软件队列，应用按实际反馈年龄决定故障与重新提交策略。

每条总线的发送、读取与恢复由项目在单个任务中串行安排，库不提供跨任务的总线操作仲裁。订阅登记/释放也须串行；释放前停止该 ID 的所有应用读取者。Init/订阅/释放/恢复不是 ISR 接口。

## 依赖与迁移

保留 bsp/can.h 及设备 CanPort 适配；新增 BSP_CAN_Configure/FindEndpoint 映射。旧项目删除 BSP_FDCANx_ENABLE、FIFO/过滤器生成宏，改为 board 清单和过滤函数；不能再依赖槽位数字等于物理编号。

直接接入需编译 bsp/fdcan.c、bsp/time.c、bsp/mm.c，提供 H7 HAL、CMSIS-RTOS2 及项目 Board_Time* 实现。既有 mr_add_stm32h7_bsp 的 can 组件选择这些源，board 的 fdcan_endpoints.c/time_source.c 仍须项目显式编译。当前实现仍依赖 H7 HAL，不是任意 MCU 的通用 CAN 后端。

端点表和 TX 缓冲为固定容量，但 ID 节点、CMSIS 接收队列和互斥量保留启动期动态分配；本批不宣称整个 BSP 无堆。主机测试覆盖登记、回滚、接收隔离、FD 帧及恢复失败，真实 IRQ/总线故障与吞吐尚未板测。
