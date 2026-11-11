#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
/* MX_FREERTOS_Init 在调度器启动前调用，只创建 Task_Init。 */
int App_CreateInitTask(void);
typedef enum {
  APP_NOT_STARTED = 0,
  APP_INITIALIZING,
  APP_TASKS_CREATED,
  APP_CHANNEL_INIT_FAILED,
  APP_CAN_INIT_FAILED,
  APP_CHASSIS_TASK_CREATE_FAILED,
  APP_CHASSIS_FEEDBACK_REGISTER_FAILED,
  APP_CHASSIS_STATUS_PUBLISH_FAILED
} App_State;
typedef struct {
  App_State state;
  int8_t can_init_result;
  int8_t chassis_registration_result;
} App_Diagnostics;
/* 只供调试器查看。任务间交换业务数据使用 app_channels.hpp。 */
extern volatile App_Diagnostics g_app;
#ifdef __cplusplus
}
#endif
