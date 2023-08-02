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

#ifndef APP_INC_LOG_H_
#define APP_INC_LOG_H_

#include "stm32f1xx_hal.h"
#include "app_cfg.h"

void LOG_Thread(void const *argument);

typedef struct {
    float gyro_angle;
    float accel_angle;
    uint8_t angle_critical;
} log_tx_message_S;

typedef struct {
    module_id_t id;
    union
    {
        uint8_t command;
        float pid_total;
        float pid_error;
        float imu_gyro_angle;
        float imu_accel_angle;
    } data;
} log_rx_message_t;

typedef struct sbr_log_handle_STRUCT {
    UART_HandleTypeDef *uart_handle_ptr;
    log_tx_message_S tx_message;
    uint8_t log_enabled;
    uint8_t is_initialized;
} log_handle_S;

void LOG_Transmit_Blocking(const char *log_string);

#endif /* APP_INC_LOG_H_ */
