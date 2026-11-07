#ifndef __APP_H__
#define __APP_H__

#include "Task_Init.h"

#ifdef __cplusplus
extern "C" {
#endif

extern TaskHandle_t Chassis_Task_Handle;

void Chassis_Task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif
