#include "unitree.h"

RS485_t unitree_bus;

Joint_t unitree_joint[UNITREE_MOTOR_NUM] = {
    {
        .motor      = { .motor_id = UNITREE_MOTOR_ID0, .rs485 = &unitree_bus },
        .inv_motor  = 1,
        .pos_offset = 0.0f,
        .exp_rad    = 0.0f,
        .exp_omega  = 0.0f,
        .exp_torque = 0.0f,
        .Kp         = 0.0f,
        .Kd         = 0.0f,
    },
};

TaskHandle_t Unitree_Task_Handle;

volatile uint32_t g_unitree_cnt = 0u;
volatile uint32_t g_unitree_rx  = 0u;

void Unitree_Init(void)
{
    RS485Init(&unitree_bus, &huart6, UNITREE_DE_PORT, UNITREE_DE_PIN);
}

void Unitree_GetState(uint8_t idx, float *rad, float *omega, float *torque)
{
    Joint_t *j;
    float inv;

    if (idx >= UNITREE_MOTOR_NUM) {
        return;
    }

    j   = &unitree_joint[idx];
    inv = (float)j->inv_motor;

    if (rad != NULL) {
        *rad = inv * (j->motor.state.rad - j->pos_offset) / UNITREE_GEAR_RATIO;
    }
    if (omega != NULL) {
        *omega = inv * j->motor.state.velocity / UNITREE_GEAR_RATIO;
    }
    if (torque != NULL) {
        *torque = inv * j->motor.state.torque * UNITREE_GEAR_RATIO;
    }
}

void Unitree_Task(void *pvParameters)
{
    TickType_t last_wake = xTaskGetTickCount();

    (void)pvParameters;

    for (;;) {
        g_unitree_cnt++;

        for (uint8_t i = 0; i < UNITREE_MOTOR_NUM; i++) {
            Joint_t *j = &unitree_joint[i];

            GoMotorSend(&j->motor,
                        j->exp_torque / UNITREE_GEAR_RATIO * j->inv_motor,
                        j->exp_omega  * UNITREE_GEAR_RATIO * j->inv_motor,
                        j->exp_rad    * UNITREE_GEAR_RATIO * j->inv_motor + j->pos_offset,
                        j->Kp, j->Kd);

            g_unitree_rx = GoMotorRecv(&j->motor);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(UNITREE_TASK_PERIOD_MS));
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UNITREE_UART.Instance) {
        RS485SendIRQ_Handler(&unitree_bus, huart);
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart->Instance == UNITREE_UART.Instance) {
        RS485RecvIRQ_Handler(&unitree_bus, huart, size);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    volatile uint32_t dr;

    if (huart->Instance != UNITREE_UART.Instance) {
        return;
    }

    HAL_UART_DMAStop(huart);

    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE)) {
        dr = huart->Instance->DR;
        (void)dr;
    }

    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_FLUSH_DRREGISTER(huart);
}
