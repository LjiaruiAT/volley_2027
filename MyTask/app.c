#include "app.h"
#include "motor.h"
#include "motorEx.h"
#include "PID_old.h"
#include "CANDrive.h"
#include "can.h"
#include <math.h>

Motor3508Ex_t steer_1 = { .ID = 0x201, .hcan = &hcan2 };
Motor3508Ex_t steer_2 = { .ID = 0x202, .hcan = &hcan2 };
Motor3508Ex_t steer_3 = { .ID = 0x203, .hcan = &hcan2 };

TaskHandle_t Chassis_Task_Handle;

static int16_t motor_current_buf[4] = {0};

static float RampAccel(float target, float current, float accel_step)
{
    float diff;

    if (target * current < 0.0f) {
        return 0.0f;
    }

    diff = target - current;

    if (fabsf(target) > fabsf(current)) {
        if (diff > accel_step)  diff = accel_step;
        if (diff < -accel_step) diff = -accel_step;
        return current + diff;
    }

    return target;
}

void Chassis_Task(void *pvParameters)
{
    TickType_t last_wake;
    float Ex_ref = 0.0f;

    (void)pvParameters;

    steer_1.vel_pid.Kp = 22.0f;
    steer_1.vel_pid.Ki = 2.3f;
    steer_1.vel_pid.Kd = 1.0f;
    steer_1.vel_pid.limit = 16384.0f;
    steer_1.vel_pid.output_limit = 10000.0f;

    steer_2.vel_pid = steer_1.vel_pid;
    steer_3.vel_pid = steer_1.vel_pid;

    last_wake = xTaskGetTickCount();

    for (;;) {
        float v1, v2, v3;
        float wheel1_cmd, wheel2_cmd, wheel3_cmd;

        Ex_ref = RampAccel(Remote_Control.Ex, Ex_ref, 0.005f);

        v1 =  0.5f * Ex_ref - 0.8660254f * Remote_Control.Ey - LENGTH * Remote_Control.Eomega;
        v2 = -0.5f * Ex_ref - 0.8660254f * Remote_Control.Ey + LENGTH * Remote_Control.Eomega;
        v3 = -Ex_ref - LENGTH * Remote_Control.Eomega;

        wheel1_cmd = -(v1 * MPS_TO_MOTOR_RPM);
        wheel2_cmd =  (v2 * MPS_TO_MOTOR_RPM);
        wheel3_cmd = -(v3 * MPS_TO_MOTOR_RPM);

        PID_Control2((float)steer_1.motor.Speed, wheel1_cmd, &steer_1.vel_pid);
        PID_Control2((float)steer_2.motor.Speed, wheel2_cmd, &steer_2.vel_pid);
        PID_Control2((float)steer_3.motor.Speed, wheel3_cmd, &steer_3.vel_pid);

        motor_current_buf[0] = (int16_t)steer_1.vel_pid.pid_out;
        motor_current_buf[1] = (int16_t)steer_2.vel_pid.pid_out;
        motor_current_buf[2] = (int16_t)steer_3.vel_pid.pid_out;

        MotorSend(&hcan2, 0x200, motor_current_buf);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(2));
    }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    uint8_t Recv[8] = {0};
    uint32_t ID = CAN_Receive_DataFrame(hcan, Recv);

    Motor3508Recv(&steer_1, hcan, ID, Recv);
    Motor3508Recv(&steer_2, hcan, ID, Recv);
    Motor3508Recv(&steer_3, hcan, ID, Recv);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    (void)hcan;
}
