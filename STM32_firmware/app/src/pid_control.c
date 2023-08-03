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
#include "master.h"

#define PID_ZERO  (0)

static pid_handle_S pid_handle;

static void pid_init();
static void pid_control_loop();
static void pid_rx_message_listener();
static void pid_parse_rx_message(pid_rx_message_t *msg);
static void pid_update_log_message();
static void pid_send_pid_total_to_master();

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
            pid_rx_message_listener();
            pid_control_loop();
            pid_send_pid_total_to_master();
            pid_update_log_message();
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

static void pid_init()
{
    pid_handle.Kp = 100.0f;
    pid_handle.Ki = 50.0f;
    pid_handle.Kd = 70.0f;
    pid_handle.set_point = CFG_INITIAL_UPRIGHT_ROBOT_ANGLE;
    pid_handle.is_initialized = 1u;
}

static void pid_control_loop()
{
    pid_handle.time_stamp = osKernelSysTick();
    pid_handle.time_delta = pid_handle.time_stamp - pid_handle.time_stamp_prev;

    pid_handle.pid_error = pid_handle.set_point - pid_handle.process_variable;
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

    pid_handle.pid_error_prev = pid_handle.pid_error;
    pid_handle.time_stamp_prev = pid_handle.time_stamp;
}

static void pid_rx_message_listener()
{
    pid_rx_message_t rx_message;

    if (0 != xMessageBufferReceive(pid_rx_message_buffer_handle,
                                   &rx_message,
                                   sizeof(rx_message),
                                   0)) {
        pid_parse_rx_message(&rx_message);
    }
}

static void pid_update_log_message()
{
    log_rx_message_t log_message;
    static uint32_t loop_counter = 0;

    loop_counter++;

    log_message.id = PID_ID;
    log_message.data.pid.pid_error = pid_handle.pid_error;
    log_message.data.pid.pid_total = pid_handle.pid_total;

    if (pdTRUE == xSemaphoreTake(log_message_buffer_mutex,
                                 10)) {
        if(loop_counter % 10 == 0) {
            // slow message
            log_message.id = PID_SLOW_ID;
            log_message.data.pid_slow.pid_Kp = pid_handle.Kp;
            log_message.data.pid_slow.pid_Ki = pid_handle.Ki;
            log_message.data.pid_slow.pid_Kd = pid_handle.Kd;
        }
        xMessageBufferSend(log_rx_message_buffer_handle,
                           &log_message,
                           sizeof(log_message),
                           10);
        xSemaphoreGive(log_message_buffer_mutex);
    }
}

static void pid_send_pid_total_to_master()
{
    master_rx_message_t master_message;

    master_message.id = PID_ID;
    master_message.data.pid_total = pid_handle.pid_total;

    if (pdTRUE == xSemaphoreTake(master_message_buffer_mutex,
                                 10)) {
        xMessageBufferSend(master_rx_message_buffer_handle,
                           &master_message,
                           sizeof(master_message),
                           10);
        xSemaphoreGive(master_message_buffer_mutex);
    }
}

static void pid_parse_rx_message(pid_rx_message_t *msg)
{
    switch (msg->id)
    {
        case COMMAND_ID:
            switch (msg->data.command)
            {
                case COMMAND_PID_RAISE_P:
                    pid_handle.Kp++;
                    break;
                case COMMAND_PID_RAISE_I:
                    pid_handle.Ki += 0.1;
                    break;
                case COMMAND_PID_RAISE_D:
                    pid_handle.Kd++;
                    break;
                case COMMAND_PID_RAISE_P10:
                    pid_handle.Kp += 10;
                    break;
                case COMMAND_PID_RAISE_I1:
                    pid_handle.Ki += 1;
                    break;
                case COMMAND_PID_RAISE_D10:
                    pid_handle.Kd += 10;
                    break;
                case COMMAND_PID_LOWER_P:
                    pid_handle.Kp--;
                    break;
                case COMMAND_PID_LOWER_I:
                    pid_handle.Ki -= 0.1;
                    break;
                case COMMAND_PID_LOWER_D:
                    pid_handle.Kd--;
                    break;
                case COMMAND_PID_LOWER_P10:
                    pid_handle.Kp -= 10;
                    break;
                case COMMAND_PID_LOWER_I1:
                    pid_handle.Ki -= 1;
                    break;
                case COMMAND_PID_LOWER_D10:
                    pid_handle.Kd -= 10;
                    break;
                default:
                    break;
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

