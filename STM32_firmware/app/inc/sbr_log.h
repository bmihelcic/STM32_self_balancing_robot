/**
 ******************************************************************************
 * File Name          : sbr_log.c
 * Description        : Self Balancing Robot logging module header file
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

#ifndef APP_INC_SBR_LOG_H_
#define APP_INC_SBR_LOG_H_

#include "stm32f1xx_hal.h"

void StartLogTask(void const *argument);

typedef struct sbr_log_mpu6050_message_STRUCT {
    float gyro_angle;
    float accel_angle;
    uint8_t angle_critical;
} sbr_log_mpu6050_message_S;

typedef struct sbr_log_handle_STRUCT {
    UART_HandleTypeDef *uart_handle_ptr;

} sbr_log_handle_S;

#endif /* APP_INC_SBR_LOG_H_ */
