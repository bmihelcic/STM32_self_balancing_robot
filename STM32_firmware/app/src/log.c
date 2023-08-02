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

extern UART_HandleTypeDef huart1;
extern osMutexId uart_mutex_id;

static log_handle_S log_handle;

static void log_init();

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
//        printf("log init success\n");
        os_delay_prev_wake_time = osKernelSysTick();
        while (1) {
//            log_handle.tx_message.gyro_angle = IMU_Get_Gyro_Angle();
//            log_handle.tx_message.accel_angle = IMU_Get_Accel_Angle();
//            log_handle.tx_message.angle_critical = IMU_Is_Angle_Critical();

//            printf("ga=%.2f aa=%.2f %d\n",
//                   log_handle.tx_message.gyro_angle,
//                   log_handle.tx_message.accel_angle,
//                   log_handle.tx_message.angle_critical);
            osDelayUntil(&os_delay_prev_wake_time,
                         CFG_LOG_FREQ_MS);
        }
    } else {
        printf("log init fail\n");
        while (1){
            osDelay(1000);
        }
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

/* Low level function for printing a char. Needed for printf() */
void _putchar(char character)
{
    HAL_UART_Transmit(&huart1,
                      (uint8_t*) &character,
                      sizeof(character),
                      100);
}

