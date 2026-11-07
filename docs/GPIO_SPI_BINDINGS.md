# GPIO / SPI 与 BMI088 板级接入

GPIO/SPI 不再保存设备用途或引用 Cube 的引脚宏、hspi2。board 提供普通 C 硬件表，app 将查找结果交给设备。不使用 YAML、生成器或运行期设备工厂。

## board 登记

在 Cube 初始化后、设备/任务启动前调用一次：

```c
#include "bsp/gpio.h"
#include "bsp/spi.h"
#include "main.h"
#include "spi.h"

static const BSP_GPIO_Endpoint_t pins[] = {
    {"imu_accl_cs", ACCL_CS_GPIO_Port, ACCL_CS_Pin, BSP_GPIO_OUTPUT},
    {"imu_gyro_cs", GYRO_CS_GPIO_Port, GYRO_CS_Pin, BSP_GPIO_OUTPUT},
    {"imu_accl_irq", ACCL_INT_GPIO_Port, ACCL_INT_Pin, BSP_GPIO_EXTI},
    {"imu_gyro_irq", GYRO_INT_GPIO_Port, GYRO_INT_Pin, BSP_GPIO_EXTI},
};
static const BSP_SPI_Endpoint_t buses[] = {{"sensor_bus", &hspi2}};
if (BSP_GPIO_Configure(pins, 4) != BSP_OK ||
    BSP_SPI_Configure(buses, 1) != BSP_OK) Error_Handler();
```

表被复制，名称和 HAL 句柄/端口必须持续有效。GPIO 最多 32 端点、SPI 最多 6 总线，全部使用静态数组。空名/句柄、重复名称或物理端点、无效参数均失败；整表校验后发布，不部分更新，成功后不能重配。中性槽位不代表管脚号或 SPI 编号。

GPIO 每端点只允许一个 pin bit；拒绝相同 port/pin 重复登记，也拒绝不同 port 占用同一条 EXTI 线。普通输入/输出在不同 port 使用相同 bit 合法，例如 PC10 阀门和 PE10 中断。Write/Toggle 只接受 OUTPUT，输入模式不会被误写；Read 非法端点返回 false，不提供错误诊断。

SPI 登记要求 HAL_READY、8 位主机、双线模式、关闭硬件 CRC，保证 size 表示字节数。引脚、极性/相位、频率、NSS、DMA、IRQ 优先级与时钟仍由 Cube/board 设置并验证。DMA 双向均要求 NORMAL、内存和外设 BYTE 对齐；不支持 circular 或重叠的 TX/RX 缓冲。DMA Cache/内存可达策略仍由 BSP_DMA_Configure 提供。

## EXTI 与回调

当前 GPIO 实现面向 H7 Cortex-M7，通过 EXTI_D1 的单线掩码控制中断，不调用 HAL_NVIC_DisableIRQ。这样屏蔽 PE10 不会同时屏蔽 PE12 或 PA15。board 负责 EXTI 路由、边沿、NVIC 优先级/使能和 IRQ handler 对 HAL 的调用。

登记时只屏蔽已登记的 EXTI 线并清除其 pending，保留其他线、GPIO 模式和输出电平。此后先 RegisterCallback 再 EnableIRQ；EnableIRQ 丢弃屏蔽期间的 pending。没有回调时不能使能。回调在 ISR 执行，应短小且使用允许的 RTOS ISR API。

回调不允许覆盖：GPIO 安装/卸载时必须先屏蔽该线；UnregisterCallback 要求传入原函数指针。SPI 回调修改要求总线空闲，无待处理 RX DMA；同样通过匹配函数卸载。HAL SPI 回调只按准确登记的句柄分发，复制一个 Instance 相同的句柄不会取得原端点回调。

每条 SPI 总线由项目安排一个所有者，包含 CS、寄存器地址和数据阶段。当前接口不提供多设备仲裁，也不自动切换模式/频率。BMI088 需要独占总线。回调冲突检查只能保护已登记的回调，不能识别绕过 BSP 的外部使用者。

## app 连接 BMI088

在传感器所属的 RTOS 任务中执行，校准数据长期有效：

```c
#include "device/bmi088.h"
static BMI088_t imu;
static const BMI088_Cali_t calibration = {{0.0f, 0.0f, 0.0f}};
const BMI088_Config_t hardware = {
    .spi = BSP_SPI_FindEndpoint("sensor_bus"),
    .accl_cs = BSP_GPIO_FindEndpoint("imu_accl_cs"),
    .gyro_cs = BSP_GPIO_FindEndpoint("imu_gyro_cs"),
    .accl_int = BSP_GPIO_FindEndpoint("imu_accl_irq"),
    .gyro_int = BSP_GPIO_FindEndpoint("imu_gyro_irq"),
};
if (BMI088_Init(&imu, &calibration, &hardware) != DEVICE_OK) {
    /* Stop this service and report initialization failure. */
}
```

配置会被复制，检查总线/引脚角色、重复引脚和校准值。驱动仍为一个实例、一个任务，内部 DMA/解析缓冲固定存储，不在堆上创建设备；采集和等待必须由初始化任务调用。启动前先取得回调，不覆盖其他消费者；失败只回滚本次资源，芯片检查和每次配置写入失败均返回错误。若 HAL 无法恢复空闲总线，会保留已取得回调并拒绝再初始化，需由项目复位处理，不伪装为已释放。当前没有运行期换绑、卸载或多实例 API。

BMI088 配置 INT1/INT3 为低电平有效，board 应配置下降沿。H7 示例在自己的 io_endpoints.c 中覆盖外部 Cube 的旧上升沿设置；外部 Cube 工程未修改。示例将 PC10/PC11 明确登记为两路电磁阀，PA2/PA0 为继电器，不再存在旧表中的重复错绑。

使用 CMake 时选择 `COMPONENTS gpio spi`，spi 自动携带 dma 源；使用 BMI088 时项目还需显式添加 User/device/bmi088.c，并提供 time/RTOS 依赖与数学库。H7 整机当前仅登记这些端点，未启动 IMU 服务；全量实现检查编译 BMI088，主机回归运行真实源的初始化/采集路径，不能将此解释为整机已启用传感器控制。

新增普通 SPI 设备，只需实现设备驱动、在 board 增加硬件登记，并在 app 传入端点；需要共享总线时另行设计明确的仲裁契约。新增 GPIO 用途只修改 board/app，不扩展库的用途枚举或生成模板。

## 旧接口迁移与验证边界

- 删除 GPIO 的 CS/电源/按键/继电器等用途枚举，以及 BSP_GPIO_NONE。使用 FindEndpoint；可选设备由 app 判断是否存在，不用无效 GPIO 假装操作成功。
- 删除 BSP_SPI_BMI088，改为显式选择 SPI 端点。
- BMI088_Init 增加 config 参数；删除从未实现的 BMI088_Restart 声明。
- GPIO/SPI 的 RegisterCallback 不再覆盖；SPI 不再用 NULL 静默移除，改用匹配函数的 UnregisterCallback。
- 不在项目中重复实现库接管的 HAL_GPIO_EXTI_Callback/HAL_SPI_*Callback；Cube IRQ handler 仍负责调用 HAL。

主机回归检查映射、逐线掩码、回调冲突、初始化回滚及现有 DMA 生命周期；H723 全量实现与公共头通过编译，两份 H7 示例完成链接。未进行板上电平/边沿、SPI 波形、DMA/IRQ 时序或故障恢复验证。其他 H7 内核或芯片系列需替换实际 EXTI/HAL 适配。
