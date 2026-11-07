# PWM 板级登记与设备接入

PWM 驱动不再保存蜂鸣器、加热、机械臂/推杆舵机的用途枚举，也不引用 htim1/3/12。项目在 board 中给出硬件表，设备通过查找结果接入。整个过程是普通 C/C++，不需要 YAML 或生成器。

## board 提供硬件

Cube 完成 PWM1、向上计数和引脚配置后，输出启动前登记一次：

```c
#include "bsp/pwm.h"
#include "tim.h"  /* Cube handles belong to board */
static const BSP_PWM_Endpoint_t endpoints[] = {
    {"tone_output", &htim12, TIM_CHANNEL_2, 240000000, 16, true},
    {"servo_a", &htim1, TIM_CHANNEL_1, 240000000, 16, false},
    {"servo_b", &htim1, TIM_CHANNEL_3, 240000000, 16, false},
};
if (BSP_PWM_Configure(endpoints, 3) != BSP_OK) Error_Handler();
```

240 MHz 和 16 位只是本示例的实际板级参数，不是库默认值。board 必须确认实际定时器内核时钟、计数位宽、通道存在、PWM1/向上计数模式和引脚；库支持 TIM_CHANNEL_1..4，不支持反相/互补、中心对齐或外部计数时钟。H7 示例在 board/pwm_endpoints.c 中检查 APB1/APB2 二分频，再用 HCLK 给出实际时钟。

驱动最多存储 16 个端点，使用固定数组，不分配堆。复制整表后发布，名称字符串/句柄必须持续有效；空名、无效句柄/通道/时钟/位宽、未就绪或正在计数的定时器、重复名称或重复物理通道会失败，不发布半张表。成功后禁止重新配置。BSP_PWM_0..15 是中性槽位，不能当作 TIM 编号。

## 共享定时器

多个通道可共享一个定时器，但必须使用同一个 HAL 句柄、一致的时钟/位宽，并全部设置 allow_frequency_change=false。它们共用 PSC/ARR，周期由 Cube/board 设置；各通道可以独立改占空比、脉宽、启停。BSP_PWM_SetFreq 对这类端点返回错误，寄存器不变，避免另一只舵机的周期随之改变。

设置 allow_frequency_change=true 表示项目保证独占整个定时器，包含未登记通道、编码器、捕获、中断和 DMA 等用户。登记表只能发现表内冲突，不能发现绕过 BSP 的外部用户。调用者按定时器串行访问，不在 ISR 或多个任务同时改参数。动态改变内核时钟后当前表失效；本接口不支持运行期时钟切换。

## app 连接设备

```c
BSP_PWM_Channel_t output = BSP_PWM_FindEndpoint("tone_output");
BUZZER_t buzzer = {0};
if (BUZZER_Init(&buzzer, output) != DEVICE_OK) { /* initialization failed */ }
/* Explicit output request: check each return value. */
BUZZER_Set(&buzzer, 440.0f, 0.5f);
BUZZER_Start(&buzzer);
```

名称也可以像 H7 示例一样集中放到 board/bindings.hpp。换接线时修改 Cube、board 表与用途常量，模块/播放器无需增加型号或用途分支。蜂鸣器初始化检查端点存在并确认已停止；失败不标记 online。改音调需要独占且允许变频的定时器。

SetFreq 根据实际时钟、16/32 位计数容量和 16 位 PSC 选择最近的整数周期，保持占空比比例；它会触发更新事件、重置相位，不保证无毛刺切频。新周期至少两个计数，保留最高 ARR 值以保证 100% 占空比可用。使用前核对实际可表示频率和精度。

SetComp 拒绝 NaN/Inf，有限值限幅到 [0,1]。SetPulseUs 根据当前 PSC 计算，向下取整到计数 tick，并限幅到周期。已有满量程 ARR+1 无法装入 CCR 时返回错误，避免 100% 输出溢出成零。HAL 启停失败如实返回。

## 旧 API 迁移

- 删除 BSP_PWM_BUZZER/IMU_HEAT/ARM_SERVO/ROD_SERVO，改为 FindEndpoint，不需要扩展库枚举。
- BSP_PWM_GetChannel 返回 uint32_t，非法端点返回 UINT32_MAX；GetHandle 返回 NULL，其他操作返回错误。GetAutoReloadPreload 的非法值为 0，不能用它判断有效性。
- 删除仓库中无调用者的 BSP_PWM_Start_DMA/Stop_DMA。旧包装没有缓冲所有权/Cache/完成通知契约；需要波形 DMA 时由 board 显式实现并验证，普通 PWM 接口不承诺此能力。

主机回归编译真实 PWM/蜂鸣器代码，检查真实寄存器计算和 HAL 返回值；H723 示例登记四路并完成固件链接。未进行实机波形、引脚、PWM 极性、占空比或共享通道时序验证。
