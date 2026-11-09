#include "Task_Init.h"
#include "app.h"
#include "unitree.h"
#include "usb_trans.h"

Remote_Handle_t Remote_Control;
TaskHandle_t Remote_Analysis_Handle;

volatile uint32_t g_task_ret[3] = {0};

void Key_Parse(uint32_t key, hw_key_t *out)
{
    out->Right_Switch_Up     = (key & KEY_Right_Switch_Up)     ? 1 : 0;
    out->Right_Switch_Down   = (key & KEY_Right_Switch_Down)   ? 1 : 0;

    out->Right_Key_Up        = (key & KEY_Right_Key_Up)        ? 1 : 0;
    out->Right_Key_Down      = (key & KEY_Right_Key_Down)      ? 1 : 0;
    out->Right_Key_Left      = (key & KEY_Right_Key_Left)      ? 1 : 0;
    out->Right_Key_Right     = (key & KEY_Right_Key_Right)     ? 1 : 0;

    out->Right_Broadside_Key = (key & KEY_Right_Broadside_Key) ? 1 : 0;

    out->Left_Switch_Up      = (key & KEY_Left_Switch_Up)      ? 1 : 0;
    out->Left_Switch_Down    = (key & KEY_Left_Switch_Down)    ? 1 : 0;

    out->Left_Key_Up         = (key & KEY_Left_Key_Up)         ? 1 : 0;
    out->Left_Key_Down       = (key & KEY_Left_Key_Down)       ? 1 : 0;
    out->Left_Key_Left       = (key & KEY_Left_Key_Left)       ? 1 : 0;
    out->Left_Key_Right      = (key & KEY_Left_Key_Right)      ? 1 : 0;

    out->Left_Broadside_Key  = (key & KEY_Left_Broadside_Key)  ? 1 : 0;
}

void Task_Init(void)
{
    Unitree_Init();

    g_task_ret[0] = (uint32_t)xTaskCreate(Remote_Analysis_Task,
                                          "Remote_Analysis_Task",
                                          400,
                                          NULL,
                                          4,
                                          &Remote_Analysis_Handle);

    g_task_ret[1] = (uint32_t)xTaskCreate(Chassis_Task,
                                          "Chassis_Task",
                                          512,
                                          NULL,
                                          4,
                                          &Chassis_Task_Handle);

    g_task_ret[2] = (uint32_t)xTaskCreate(Unitree_Task,
                                          "Unitree_Task",
                                          512,
                                          NULL,
                                          5,
                                          &Unitree_Task_Handle);
}

#define UPPER_LINK_TIMEOUT_MS   100u

void Remote_Analysis(void)
{
    static TickType_t upper_last_tick  = 0;
    static uint8_t    upper_link_alive = 0u;

    if ((g_upper_sem == NULL) && (USB_CDC_IsConfigured() != 0u)) {
        UsbUpper_Init();
    }

    Remote_Control.Second = Remote_Control.First;

    if ((g_upper_sem != NULL) && (xSemaphoreTake(g_upper_sem, 0) == pdTRUE)) {
        Remote_Control.Ex     = g_upper_data.vx;
        Remote_Control.Ey     = g_upper_data.vy;
        Remote_Control.Eomega = g_upper_data.omega;
        upper_last_tick       = xTaskGetTickCount();
        upper_link_alive      = 1u;
    }

    /* 上位机断线保护：超过 UPPER_LINK_TIMEOUT_MS 没有新帧就立刻停车，
     * 否则底盘会一直按最后一条速度指令跑下去。 */
    if ((upper_link_alive != 0u) &&
        ((TickType_t)(xTaskGetTickCount() - upper_last_tick) >
         pdMS_TO_TICKS(UPPER_LINK_TIMEOUT_MS))) {
        Remote_Control.Ex     = 0.0f;
        Remote_Control.Ey     = 0.0f;
        Remote_Control.Eomega = 0.0f;
        upper_link_alive      = 0u;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
}

void Remote_Analysis_Task(void *pvParameters)
{
    (void)pvParameters;

    for (;;) {
        Remote_Analysis();
    }
}
