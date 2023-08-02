/**
 ******************************************************************************
 * File Name          : sbr_command.c
 * Description        : Self Balancing Robot commands handler source file
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
#include "stm32f1xx_hal.h"
#include "command.h"
#include "printf.h"
#include "app_cfg.h"
#include "stdint.h"
#include "master.h"
#include "app_cfg.h"
#include "os_resources.h"
#include "pid_control.h"
#include "message_buffer.h"
#include "bsp.h"
#include "log.h"

extern UART_HandleTypeDef huart1;

command_handle_S command_handle;
static uint8_t uart_rx_byte; // registered in uart driver for byte reception, do not write to it from application!

static void command_init();
static void command_handler(uint8_t rx_command);
void command_rx_callback(UART_HandleTypeDef *huart);

/**
 * @brief Function implementing the command thread.
 * @param argument: Not used
 * @retval None
 */
void COMMAND_Thread(void const *argument)
{
    uint8_t received_command;

    command_init();

    if (1u == command_handle.is_initialized) {
        if (pdTRUE == xSemaphoreTake(uart_mutex,
                                     portMAX_DELAY)) {
            LOG_Transmit_Blocking("command init success\n");
            xSemaphoreGive(uart_mutex);
        }
        while (1) {
            if (0 != xMessageBufferReceive(command_rx_message_buffer_handle,
                                           &received_command,
                                           sizeof(received_command),
                                           100)) {
                command_handler(received_command);
            }
            osDelay(50);
        }
    } else {
        if (pdTRUE == xSemaphoreTake(uart_mutex,
                                     portMAX_DELAY)) {
            LOG_Transmit_Blocking("command init fail\n");
            xSemaphoreGive(uart_mutex);
        }
        while (1) {
            osDelay(1000);
        }
    }
}

void command_rx_callback(UART_HandleTypeDef *huart)
{
    xMessageBufferSendFromISR(command_rx_message_buffer_handle,
                              &uart_rx_byte,
                              sizeof(uart_rx_byte),
                              NULL);
    HAL_UART_Receive_IT(&huart1,
                        &uart_rx_byte,
                        sizeof(uart_rx_byte));

}

static void command_init()
{
    if (HAL_OK != HAL_UART_Receive_IT(&huart1,
                                      &uart_rx_byte,
                                      sizeof(uart_rx_byte))) {
        goto exitErr;
    }

    if (HAL_OK != HAL_UART_RegisterCallback(&huart1,
                                            HAL_UART_RX_COMPLETE_CB_ID,
                                            command_rx_callback)) {
        goto exitErr;
    }

    command_handle.is_initialized = 1u;
    return;

exitErr:
    command_handle.is_initialized = 0u;
    return;
}

static void command_handler(uint8_t rx_command)
{
    master_rx_message_t master_message = { .id = COMMAND_ID, };
    pid_rx_message_t pid_message = { .id = COMMAND_ID, };

    switch (rx_command)
    {
        case COMMAND_MASTER_START_STOP:
        case COMMAND_MASTER_ANGLE_SET_POINT_PLUS:
        case COMMAND_MASTER_ANGLE_SET_POINT_MINUS:
            master_message.data.command = rx_command;
            xMessageBufferSend(master_rx_message_buffer_handle,
                               &master_message,
                               sizeof(master_message),
                               10);
            break;
        case COMMAND_PID_RAISE_P:
        case COMMAND_PID_LOWER_P:
        case COMMAND_PID_RAISE_I:
        case COMMAND_PID_LOWER_I:
        case COMMAND_PID_RAISE_D:
        case COMMAND_PID_LOWER_D:
        case COMMAND_PID_RAISE_P10:
        case COMMAND_PID_RAISE_I1:
        case COMMAND_PID_RAISE_D10:
        case COMMAND_PID_LOWER_P10:
        case COMMAND_PID_LOWER_I1:
        case COMMAND_PID_LOWER_D10:
            pid_message.data.command = rx_command;
            xQueueSend(pid_rx_message_buffer_handle,
                       &pid_message,
                       10);
            break;
        case COMMAND_LOG_ON_OFF:
            break;
        default:
            break;
    }
}

