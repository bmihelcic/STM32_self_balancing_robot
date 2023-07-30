/**
 ******************************************************************************
 * File Name          : sbr_pid_control.c
 * Description        : PID controller source file
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2023 Branimir Mihelčić
 * All rights reserved.</center></h2>
 *
 * This software component is licensed under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************/
#include "cmsis_os.h"
#include "pid_control.h"
#include "printf.h"
#include "app_cfg.h"

#define PID_ZERO  (0)

static uint8_t pid_message_queue_buffer[50];
static osStaticMessageQDef_t pid_message_queue_cb;
static pid_handle_S pid_handle;
static osMessageQStaticDef(pid_message_queue, 10, sizeof(pid_message_t), pid_message_queue_buffer, &pid_message_queue_cb);
osMessageQId pid_message_queue_id;

void pid_init();
void pid_control_process(float set_point, float process_variable);
void pid_control_message_listener();

/**
 * @brief Function implementing the pid control thread.
 * @param argument: Not used
 * @retval None
 */
void PID_CONTROL_Thread(void const *argument)
{
    const uint32_t delay_ms = (1000 / CFG_PID_CONTROL_FREQ_HZ);
    uint32_t os_delay_prev_wake_time;

    pid_init();

    if (1u == pid_handle.is_initialized) {
        os_delay_prev_wake_time = osKernelSysTick();
        printf("pid control init success\n");
        while (1) {
            pid_control_message_listener();
            pid_control_process(pid_handle.set_point, pid_handle.process_variable);
            osDelayUntil(&os_delay_prev_wake_time, delay_ms);
        }
    } else {
        printf("pid control init fail\n");
        while (1);
    }
}

void pid_init()
{
    pid_handle.Kp = 100.0f;
    pid_handle.Ki = 50.0f;
    pid_handle.Kd = 70.0f;

    pid_message_queue_id = osMessageCreate(osMessageQ(pid_message_queue), osThreadGetId());

    if (NULL != pid_message_queue_id) {
        pid_handle.is_initialized = 1u;
    } else {
        pid_handle.is_initialized = 0u;
    }
}

void pid_control_process(float set_point, float process_variable)
{
    pid_handle.time_stamp = osKernelSysTick();
    pid_handle.time_delta = pid_handle.time_stamp - pid_handle.time_stamp_prev;

    pid_handle.pid_error = set_point - process_variable;
    /* calculate p of pid */
    pid_handle.pid_p = pid_handle.Kp * pid_handle.pid_error;
    /* limit p */
    if (pid_handle.pid_p > CFG_PID_KP_HIGH_LIMIT) {
        pid_handle.pid_p = CFG_PID_KP_HIGH_LIMIT;
    } else if (pid_handle.pid_p < CFG_PID_KP_LOW_LIMIT) {
        pid_handle.pid_p = CFG_PID_KP_LOW_LIMIT;
    }
    /* calculate i of pid if error is small */
    if ((pid_handle.pid_error > -2) && (pid_handle.pid_error < 2)
            && (pid_handle.pid_error != PID_ZERO)) {
        pid_handle.pid_i = pid_handle.pid_i + pid_handle.Ki * pid_handle.pid_error;
    } else {
        pid_handle.pid_i = PID_ZERO;
    }
    /* limit i of pid */
    if (pid_handle.pid_i < CFG_PID_KI_LOW_LIMIT) {
        pid_handle.pid_i = CFG_PID_KI_LOW_LIMIT;
    } else if (pid_handle.pid_i > CFG_PID_KI_HIGH_LIMIT) {
        pid_handle.pid_i = CFG_PID_KI_HIGH_LIMIT;
    }
    /* calculate d of pid */
    pid_handle.pid_d = pid_handle.Kd * (pid_handle.pid_error - pid_handle.pid_error_prev)
            / pid_handle.time_delta;
    /* limit d of pid */
    if (pid_handle.pid_d < CFG_PID_KD_LOW_LIMIT) {
        pid_handle.pid_d = CFG_PID_KD_LOW_LIMIT;
    } else if (pid_handle.pid_d > CFG_PID_KD_HIGH_LIMIT) {
        pid_handle.pid_d = CFG_PID_KD_HIGH_LIMIT;
    }
    /* calculate pid_total (sum of p,i and d) */
    pid_handle.pid_total = pid_handle.pid_p + pid_handle.pid_i + pid_handle.pid_d;

    if (pid_handle.pid_total < PID_ZERO) {
        pid_handle.pid_total = -(pid_handle.pid_total);
    }

    pid_handle.pid_error_prev = pid_handle.pid_error;
    pid_handle.time_stamp_prev = pid_handle.time_stamp;
}

void pid_control_message_listener()
{
    osEvent pid_message_queue_event;
    pid_message_t *pid_message_ptr;

    pid_message_queue_event = osMessageGet(pid_message_queue_id, 0);
    if (osEventMessage == pid_message_queue_event.status) {
        pid_message_ptr = (pid_message_t*) pid_message_queue_event.value.p;
        pid_handle.set_point = pid_message_ptr->rx_set_point;
        pid_handle.process_variable = pid_message_ptr->rx_current_point;
    }
}

