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

extern UART_HandleTypeDef huart1;
extern osMutexId uart_mutex_id;

osMessageQId command_message_queue_id;
command_handle_S command_handle;

uint8_t command_message_queue_buffer[50];
osStaticMessageQDef_t command_message_queue_cb;

osMessageQStaticDef(command_message_queue,
                    100,
                    uint8_t,
                    command_message_queue_buffer,
                    &command_message_queue_cb);

void command_init();

/**
 * @brief Function implementing the command thread.
 * @param argument: Not used
 * @retval None
 */
void COMMAND_Thread(void const *argument)
{
    osEvent command_message_event;
    uint8_t command;

    command_init();

    if (1u == command_handle.is_initialized) {
        printf("command init success\n");
        while (1) {
            command_message_event = osMessageGet(command_message_queue_id,
                                                 100);
            if(osEventMessage == command_message_event.status) {
                command = (uint8_t)command_message_event.value.v;
            }
            osDelay(10);
        }
    } else {
        printf("command init fail\n");
        while (1);
    }
}

void command_init()
{
    command_message_queue_id = osMessageCreate(osMessageQ(command_message_queue),
                                               osThreadGetId());

    if (NULL == command_message_queue_id) {
        goto exitErr;
    }

    command_handle.is_initialized = 1u;
    return;

exitErr:
    command_handle.is_initialized = 0u;
    return;
}

void COMMAND_Rx_Callback(UART_HandleTypeDef *huart)
{
    volatile uint32_t received_command = COMMAND_DO_NOTHING;
    received_command = (huart->Instance->DR & 0xFF);
    osMessagePut(command_message_queue_id,
                 received_command,
                 0);

}
