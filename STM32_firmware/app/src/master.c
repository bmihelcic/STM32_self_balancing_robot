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
#include "stdio.h"

static master_handle_s master_handle;

static void master_init();
static void master_parse_rx_message(master_rx_message_t *msg);

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
            sprintf(uart_tx_buffer, "master init success\n");
            LOG_Transmit_Blocking();
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

            osDelayUntil(&os_delay_prev_wake_time,
                         CFG_MASTER_FREQ_MS);
        }
    } else {
        if (pdTRUE == xSemaphoreTake(uart_mutex,
                                     portMAX_DELAY)) {
            sprintf(uart_tx_buffer, "master init fail\n");
            LOG_Transmit_Blocking();
            xSemaphoreGive(uart_mutex);
        }
        while (1) {
            osDelay(100);
        }
    }
}

static void master_init()
{
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
                    break;
                case COMMAND_MASTER_ANGLE_SET_POINT_MINUS:
                    master_handle.robot_angle_set_point -= 0.5f;
                    break;
                default:
                    break;
            }

            break;
        case PID_ID:
            break;
        default:
            break;
    }
}
