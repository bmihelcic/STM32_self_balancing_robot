/*
 * sbr_pid_control.h
 *
 *  Created on: Jun 22, 2023
 *      Author: brane
 */

#ifndef APP_INC_PID_CONTROL_H_
#define APP_INC_PID_CONTROL_H_

void StartPidControllerTask(void const *argument);

#endif /* APP_INC_PID_CONTROL_H_ */

/*
 * pid_control.h
 *
 *  Created on: Jul 7, 2021
 *      Author: branimir
 */

#ifndef INC_PID_CONTROL_H_
#define INC_PID_CONTROL_H_

#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "app_cfg.h"

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float pid_error;
    float pid_error_prev;
    float pid_p;
    float pid_i;
    float pid_d;
    float pid_total;
    uint32_t time_stamp;
    uint32_t time_stamp_prev;
    uint32_t time_delta;
    uint8_t is_initialized;
    float set_point;
    float process_variable;
} pid_handle_S;

typedef struct
{
    message_id_t id;
    union
    {
        uint8_t command;
        float imu_robot_angle;
        float master_angle_set_point;
    } data;
} pid_rx_message_t;

void PID_CONTROL_Thread(void const *argument);

#endif /* INC_PID_CONTROL_H_ */
