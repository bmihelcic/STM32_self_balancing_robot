/**
 ******************************************************************************
 * File Name          : sbr_log.c
 * Description        : Self Balancing Robot logging module source file
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
/**
 * @brief Function implementing the logTask thread.
 * @param argument: Not used
 * @retval None
 */
#include "log.h"
#include "stm32f1xx_hal.h"
#include "string.h"
#include "stdio.h"
#include "cmsis_os.h"
#include "message_buffer.h"
#include "printf.h"
#include "imu.h"
#include "app_cfg.h"
#include "os_resources.h"

extern UART_HandleTypeDef huart1;
extern osMutexId uart_mutex_id;

static log_handle_S log_handle;

static void log_init();
static void log_check_rx_message_buffer();
static void log_parse_rx_message(log_rx_message_t *msg);
static void log_send_tx_message();

/**
 * @brief Function implementing log thread.
 * @param argument: Not used
 * @retval None
 */
void LOG_Thread(void const *argument)
{
    uint32_t os_delay_prev_wake_time;

    log_init();

    if (1u == log_handle.is_initialized) {
        if (pdTRUE == xSemaphoreTake(uart_mutex,
                                     portMAX_DELAY)) {
            LOG_Transmit_Blocking("log init success\n");
            xSemaphoreGive(uart_mutex);
        }

        os_delay_prev_wake_time = osKernelSysTick();
        while (1) {

            if (1 == log_handle.log_enabled) {
                log_check_rx_message_buffer();

                if (pdTRUE == xSemaphoreTake(uart_mutex,
                                             portMAX_DELAY)) {
                    log_send_tx_message();
                    xSemaphoreGive(uart_mutex);
                }
            }
            osDelayUntil(&os_delay_prev_wake_time,
                         CFG_LOG_FREQ_MS);
        }
    } else {
        LOG_Transmit_Blocking("log init fail\n");
        while (1) {
            osDelay(1000);
        }
    }
}

void LOG_Transmit_Blocking(const char *log_string)
{
    uint16_t log_string_size;

    log_string_size = strlen(log_string);
    if (log_string_size <= CFG_UART_TX_BUFFER_SIZE) {
        sprintf(uart_tx_buffer,
                log_string);
        HAL_UART_Transmit(&huart1,
                          (uint8_t*) uart_tx_buffer,
                          log_string_size,
                          100);
    }

}

static void log_init()
{
    log_handle.uart_handle_ptr = &huart1;
    log_handle.log_enabled = 1u;

    if (NULL != log_handle.uart_handle_ptr) {
        log_handle.is_initialized = 1u;
    } else {
        log_handle.is_initialized = 0u;
    }
}

static void log_check_rx_message_buffer()
{
    log_rx_message_t rx_message;

    if (0 != xMessageBufferReceive(log_rx_message_buffer_handle,
                                   &rx_message,
                                   sizeof(rx_message),
                                   10)) {
        log_parse_rx_message(&rx_message);
    }
}

static void log_parse_rx_message(log_rx_message_t *msg)
{
    switch (msg->id)
    {
        case COMMAND_ID:
            switch (msg->data.command)
            {
                case COMMAND_LOG_ON_OFF:
                    log_handle.log_enabled = !log_handle.log_enabled;
            }

            break;
        case IMU_ID:
            log_handle.tx_message.imu_accel_angle = msg->data.imu.imu_accel_angle;
            log_handle.tx_message.imu_gyro_angle = msg->data.imu.imu_gyro_angle;
            break;
        case PID_ID:
            log_handle.tx_message.pid_error = msg->data.pid.pid_error;
            log_handle.tx_message.pid_total = msg->data.pid.pid_total;
            break;
        default:
            break;
    }
}

static void log_send_tx_message()
{
    char tx_buff[50];

    sprintf(tx_buff,
            "%.2f %.2f %.2f %.2f\n",
            log_handle.tx_message.imu_gyro_angle,
            log_handle.tx_message.imu_accel_angle,
            log_handle.tx_message.pid_error,
            log_handle.tx_message.pid_total);
    LOG_Transmit_Blocking(tx_buff);
}
