/**
 ******************************************************************************
 * File Name          : sbr_master.c
 * Description        : Self Balancing Robot master source file
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
#include "master.h"
#include "os_resources.h"
#include "app_cfg.h"
#include "message_buffer.h"
#include "log.h"
#include "pid_control.h"
#include "motor_control.h"

static master_handle_s master_handle;

static void master_init();
static void master_parse_rx_message(master_rx_message_t *msg);
static void master_send_set_point_to_pid();
static void master_drive_motors();
static void master_power_off_motors();

/**
 * @brief Function implementing the master thread.
 * @param argument: Not used
 * @retval None
 */
void MASTER_Thread(void const *argument)
{
    uint32_t os_delay_prev_wake_time;
    master_rx_message_t rx_message;

    master_init();

    if (1u == master_handle.is_initialized) {

        if (pdTRUE == xSemaphoreTake(uart_mutex,
                                     portMAX_DELAY)) {
            LOG_Transmit_Blocking("master init success\n");
            xSemaphoreGive(uart_mutex);
        }

        os_delay_prev_wake_time = osKernelSysTick();

        while (1) {
            if (0 != xMessageBufferReceive(master_rx_message_buffer_handle,
                                           &rx_message,
                                           sizeof(rx_message),
                                           100)) {
                master_parse_rx_message(&rx_message);
            }

            if (1 == master_handle.master_start_stop) {
                master_drive_motors();
            } else {
                master_power_off_motors();
            }

            osDelayUntil(&os_delay_prev_wake_time,
                         CFG_MASTER_FREQ_MS);
        }
    } else {
        if (pdTRUE == xSemaphoreTake(uart_mutex,
                                     portMAX_DELAY)) {
            LOG_Transmit_Blocking("master init fail\n");
            xSemaphoreGive(uart_mutex);
        }
        while (1) {
            osDelay(100);
        }
    }
}

static void master_init()
{
    master_handle.robot_angle_set_point = CFG_INITIAL_UPRIGHT_ROBOT_ANGLE;
    master_handle.master_start_stop = 0;
    master_handle.is_initialized = 1u;
}

static void master_parse_rx_message(master_rx_message_t *msg)
{
    switch (msg->id)
    {
        case COMMAND_ID:
            switch (msg->data.command)
            {
                case COMMAND_MASTER_START_STOP:
                    master_handle.master_start_stop = !master_handle.master_start_stop;
                    break;
                case COMMAND_MASTER_ANGLE_SET_POINT_PLUS:
                    master_handle.robot_angle_set_point += 0.5f;
                    master_send_set_point_to_pid();
                    break;
                case COMMAND_MASTER_ANGLE_SET_POINT_MINUS:
                    master_handle.robot_angle_set_point -= 0.5f;
                    master_send_set_point_to_pid();
                    break;
                default:
                    break;
            }

            break;
        case PID_ID:
            master_handle.pid_total_value = msg->data.pid_total;
            break;
        default:
            break;
    }
}

static void master_send_set_point_to_pid()
{
    pid_rx_message_t pid_message;

    pid_message.id = MASTER_ID;
    pid_message.data.master_angle_set_point = master_handle.robot_angle_set_point;

    if (pdTRUE == xSemaphoreTake(pid_message_buffer_mutex,
                                 10)) {
        xMessageBufferSend(pid_rx_message_buffer_handle,
                           &pid_message,
                           sizeof(pid_message),
                           10);
        xSemaphoreGive(pid_message_buffer_mutex);
    }
}

static void master_drive_motors()
{
    if (master_handle.pid_total_value > 0) {
        MOTORS_Turn_Counter_Clockwise((uint16_t) master_handle.pid_total_value);
    } else {
        MOTORS_Turn_Clockwise((uint16_t) (-master_handle.pid_total_value));
    }

}

static void master_power_off_motors()
{
    MOTORS_Power_Off();
}
