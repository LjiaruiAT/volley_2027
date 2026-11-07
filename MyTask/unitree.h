#ifndef __UNITREE_H__
#define __UNITREE_H__

#include "Task_Init.h"
#include "485_bus.h"
#include "go_motor.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    GO_MotorHandle_t motor;

    int8_t inv_motor;
    float  pos_offset;

    float exp_rad;
    float exp_omega;
    float exp_torque;

    float Kp;
    float Kd;
} Joint_t;

extern RS485_t      unitree_bus;
extern Joint_t      unitree_joint[UNITREE_MOTOR_NUM];
extern TaskHandle_t Unitree_Task_Handle;

extern volatile uint32_t g_unitree_cnt;
extern volatile uint32_t g_unitree_rx;

void Unitree_Init(void);
void Unitree_Task(void *pvParameters);


#ifdef __cplusplus
}
#endif

#endif
