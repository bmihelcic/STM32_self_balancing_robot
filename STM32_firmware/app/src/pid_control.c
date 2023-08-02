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
#include "app_cfg.h"
#include "os_resources.h"
#include "message_buffer.h"
#include "log.h"

#define PID_ZERO  (0)

static pid_handle_S pid_handle;

void pid_init();
void pid_control_process(float set_point, float process_variable);
void pid_control_rx_message_listener();
static void pid_parse_rx_message(pid_rx_message_t *msg);

/**
 * @brief Function implementing the pid control thread.
 * @param argument: Not used
 * @retval None
 */
void PID_CONTROL_Thread(void const *argument)
{
    uint32_t os_delay_prev_wake_time;

    pid_init();

    if (1u == pid_handle.is_initialized) {

        os_delay_prev_wake_time = osKernelSysTick();

        if (pdTRUE == xSemaphoreTake(uart_mutex,
                                     portMAX_DELAY)) {
            LOG_Transmit_Blocking("pid control init success\n");
            xSemaphoreGive(uart_mutex);
        }

        while (1) {
            pid_control_rx_message_listener();
            pid_control_process(pid_handle.set_point,
                                pid_handle.process_variable);
            osDelayUntil(&os_delay_prev_wake_time,
                         CFG_PID_CONTROL_FREQ_MS);
        }
    } else {
        if (pdTRUE == xSemaphoreTake(uart_mutex,
                                     portMAX_DELAY)) {
            LOG_Transmit_Blocking("pid control init fail\n");
            xSemaphoreGive(uart_mutex);
        }
        while (1);
    }
}

void pid_init()
{
    pid_handle.Kp = 100.0f;
    pid_handle.Ki = 50.0f;
    pid_handle.Kd = 70.0f;
    pid_handle.is_initialized = 1u;
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
    if ((pid_handle.pid_error > -2) && (pid_handle.pid_error < 2) && (pid_handle.pid_error != PID_ZERO)) {
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
    pid_handle.pid_d = pid_handle.Kd * (pid_handle.pid_error - pid_handle.pid_error_prev) / pid_handle.time_delta;
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

void pid_control_rx_message_listener()
{
    pid_rx_message_t rx_message;

    if (0 != xMessageBufferReceive(pid_rx_message_buffer_handle,
                                   &rx_message,
                                   sizeof(rx_message),
                                   0)) {
        pid_parse_rx_message(&rx_message);
    }
}

static void pid_parse_rx_message(pid_rx_message_t *msg)
{
    switch (msg->id)
    {
        case COMMAND_ID:
            switch (msg->data.command)
            {
            }

            break;
        case IMU_ID:
            pid_handle.process_variable = msg->data.imu_robot_angle;
            break;
        case MASTER_ID:
            pid_handle.set_point = msg->data.master_angle_set_point;
            break;
        default:
            break;
    }
}

