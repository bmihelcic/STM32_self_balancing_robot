/**
 ******************************************************************************
 * File Name          : sbr_command.c
 * Description        : Self Balancing Robot commands header file
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
#ifndef APP_INC_COMMAND_H_
#define APP_INC_COMMAND_H_

#include "stm32f1xx_hal.h"

typedef struct
{
    uint8_t is_initialized;
} command_handle_S;

typedef struct
{
    uint8_t rx_command;
} command_message_S;


void COMMAND_Thread(void const *argument);
void COMMAND_Rx_Callback(UART_HandleTypeDef *huart);

#endif /* APP_INC_COMMAND_H_ */
