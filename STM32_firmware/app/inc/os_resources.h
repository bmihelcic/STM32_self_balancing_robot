/**
 ******************************************************************************
 * File Name          : os_resources.h
 * Description        : OS resources header file.
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
#ifndef APP_INC_OS_RESOURCES_H_
#define APP_INC_OS_RESOURCES_H_

#include "cmsis_os.h"

extern QueueHandle_t master_rx_message_buffer_handle;
extern QueueHandle_t command_rx_message_buffer_handle;
extern QueueHandle_t pid_rx_message_buffer_handle;
extern QueueHandle_t log_rx_message_buffer_handle;

int OS_RESOURCES_Init();

#endif /* APP_INC_OS_RESOURCES_H_ */
