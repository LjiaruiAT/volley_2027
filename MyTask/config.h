#ifndef __CONFIG_H__
#define __CONFIG_H__

#define PI                    3.14159265359f
#define ANGLE2RAD(x)          ((x) * PI / 180.0f)

#define MAX_ROBOT_VEL         4.0f
#define MAX_ROBOT_OMEGA       (PI * 0.3f)
#define LENGTH                0.457f
#define WHEEL_RADIUS          0.075f

#define M3508_GEAR_RATIO      19.2032f
#define MPS_TO_MOTOR_RPM      (M3508_GEAR_RATIO * 60.0f / (2.0f * PI * WHEEL_RADIUS))

#define REMOTE_FIGER          1647.0f
#define KEY_RISING_EDGE(cur, last, field)  ((cur.field == 1) && (last.field == 0))

#define UNITREE_UART             huart6
#define UNITREE_DE_PORT          GPIOA
#define UNITREE_DE_PIN           GPIO_PIN_4
#define UNITREE_MOTOR_NUM        1
#define UNITREE_MOTOR_ID0        0x01
#define UNITREE_GEAR_RATIO       6.33f
#define UNITREE_TASK_PERIOD_MS   5

#endif
