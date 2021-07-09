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
#include "MPU6050.h"

typedef struct PID_handle_STRUCT {
    volatile float Kp;
    volatile float Ki;
    volatile float Kd;
    float pid_error;
    float pid_error_prev;
    float pid_p;
    float pid_i;
    float pid_d;
    float pid_total;
    uint32_t time_stamp;
    uint32_t time_stamp_prev;
    uint32_t time_delta;
}PID_handle_S;

void PID_init(PID_handle_S*);
void PID_controlHandler(PID_handle_S*, float, float);

#endif /* INC_PID_CONTROL_H_ */
