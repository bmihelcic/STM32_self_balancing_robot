/*
 * pid_control.c
 *
 *  Created on: Jul 7, 2021
 *      Author: branimir
 */
#include "pid_control.h"

void PID_init(pidControlHandle_S *pidHandlePtr) {
    pidHandlePtr->Kp = 100.0f;
    pidHandlePtr->Ki = 50.0f;
    pidHandlePtr->Kd = 70.0f;
}

void PID_controlHandler(pidControlHandle_S *pidHandlePtr, float set_point, float process_variable) {
    pidHandlePtr->time_stamp = HAL_GetTick();
    pidHandlePtr->time_delta = pidHandlePtr->time_stamp - pidHandlePtr->time_stamp_prev;

    pidHandlePtr->pid_error = set_point - process_variable;
    /* calculate p of pid */
    pidHandlePtr->pid_p = pidHandlePtr->Kp * pidHandlePtr->pid_error;
    /* limit p */
    if (pidHandlePtr->pid_p > 1023.0f) {
        pidHandlePtr->pid_p = 1023.0f;
    } else if (pidHandlePtr->pid_p < -1023.0f) {
        pidHandlePtr->pid_p = -1023.0f;
    }
    /* calculate i of pid if error is small */
    if (pidHandlePtr->pid_error > -2 && pidHandlePtr->pid_error < 2 && pidHandlePtr->pid_error != 0) {
        pidHandlePtr->pid_i = pidHandlePtr->pid_i + pidHandlePtr->Ki * pidHandlePtr->pid_error;
    } else {
        pidHandlePtr->pid_i = 0.0f;
    }
    /* limit i of pid */
    if (pidHandlePtr->pid_i < -1023.0f) {
        pidHandlePtr->pid_i = -1023.0f;
    } else if (pidHandlePtr->pid_i > 1023.0f) {
        pidHandlePtr->pid_i = 1023.0f;
    }
    /* calculate d of pid */
    pidHandlePtr->pid_d = pidHandlePtr->Kd * (pidHandlePtr->pid_error - pidHandlePtr->pid_error_prev) / pidHandlePtr->time_delta;
    /* limit d of pid */
    if (pidHandlePtr->pid_d < -1023.0f) {
        pidHandlePtr->pid_d = -1023.0f;
    } else if (pidHandlePtr->pid_d > 1023.0f) {
        pidHandlePtr->pid_d = 1023.0f;
    }
    /* calculate pid_total (sum of p,i and d) */
    pidHandlePtr->pid_total = pidHandlePtr->pid_p + pidHandlePtr->pid_i + pidHandlePtr->pid_d;
    if (pidHandlePtr->pid_total < 0.0f) {
        pidHandlePtr->pid_total = -(pidHandlePtr->pid_total);
        }

    pidHandlePtr->pid_error_prev = pidHandlePtr->pid_error;
    pidHandlePtr->time_stamp_prev = pidHandlePtr->time_stamp;
}

