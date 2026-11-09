#ifndef __USB_UPPER_H__
#define __USB_UPPER_H__

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HOST_FRAME_HEAD     0xAA
#define HOST_FRAME_TAIL     0xBB
#define HOST_FRAME_LEN      14u

typedef struct {
    float vx;
    float vy;
    float omega;
} UsbUpper_Data_t;

extern UsbUpper_Data_t   g_upper_data;
extern SemaphoreHandle_t g_upper_sem;
extern volatile uint32_t g_upper_tx_drop;    /*!< 回声发送被丢弃的次数         */
extern volatile uint32_t g_upper_frame_bad;  /*!< 头尾合法但数值非法的帧数     */

void UsbUpper_Init(void);
void UsbUpper_RecvCallback(uint8_t *src, uint16_t size);
void UsbUpper_Send(float vx, float vy, float omega);

#ifdef __cplusplus
}
#endif

#endif
