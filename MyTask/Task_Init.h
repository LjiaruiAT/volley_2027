#ifndef _TASK_INIT_H_
#define _TASK_INIT_H_

#include "config.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "stdio.h"
#include "math.h"
#include "usart.h"
#include "dataFrame.h"
#include "usb_upper.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t Left_Key_Up;
    uint8_t Left_Key_Down;
    uint8_t Left_Key_Left;
    uint8_t Left_Key_Right;
    uint8_t Left_Switch_Up;
    uint8_t Left_Switch_Down;
    uint8_t Left_Broadside_Key;

    uint8_t Right_Key_Up;
    uint8_t Right_Key_Down;
    uint8_t Right_Key_Left;
    uint8_t Right_Key_Right;
    uint8_t Right_Switch_Up;
    uint8_t Right_Switch_Down;
    uint8_t Right_Broadside_Key;
} hw_key_t;

typedef struct {
    float Ex;
    float Ey;
    float Eomega;
    hw_key_t First, Second;
} Remote_Handle_t;

extern Remote_Handle_t Remote_Control;

void Task_Init(void);
void Key_Parse(uint32_t key, hw_key_t *out);
void Remote_Analysis(void);
void Remote_Analysis_Task(void *pvParameters);

extern TaskHandle_t Remote_Analysis_Handle;

#ifdef __cplusplus
}
#endif

#endif
