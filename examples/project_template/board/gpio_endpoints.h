#pragma once
#ifdef __cplusplus
extern "C" {
#endif
// 在对应 MX_*_Init 后调用一次；成功返回 1，失败返回 0。
int Board_RegisterGpioEndpoints(void);
#ifdef __cplusplus
}
#endif
