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

extern UART_HandleTypeDef huart1;
extern osMutexId uart_mutex_id;

command_handle_S command_handle;

void command_init();
pUART_CallbackTypeDef command_rx_callback(UART_HandleTypeDef *huart);

/**
 * @brief Function implementing the command thread.
 * @param argument: Not used
 * @retval None
 */
void COMMAND_Thread(void const *argument)
{
    command_init();

    if (1u == command_handle.is_initialized) {
        printf("command init success\n");
        while (1) {
            osDelay(1);
        }
    } else {
        printf("command init fail\n");
        while (1);
    }
}

void command_init()
{
    if (HAL_OK != HAL_UART_RegisterCallback(&huart1,
                                            HAL_UART_RX_COMPLETE_CB_ID,
                                            command_rx_callback)) {
        goto exitErr;
    }

    command_handle.is_initialized = 1u;

exitErr:
    command_handle.is_initialized = 0u;
}

pUART_CallbackTypeDef command_rx_callback(UART_HandleTypeDef *huart)
{
    volatile uint32_t received_command = COMMAND_DO_NOTHING;
    received_command = (huart->Instance->DR & 0xFF);
    osMessagePut(command_message_queue_id, received_command, 0);

}
