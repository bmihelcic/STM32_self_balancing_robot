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
#include "stm32f1xx_hal.h"
#include "sbr_log.h"
#include "string.h"
#include "stdio.h"
#include "cmsis_os.h"
#include "message_buffer.h"
#include "printf.h"

// Used to dimension the array used to hold the messages.  The available space
// will actually be one less than this, so 999.
#define STORAGE_SIZE_BYTES 1000

extern UART_HandleTypeDef huart1;
static sbr_log_handle_S log_handle;
MessageBufferHandle_t xMessageMPU6050SharedBuffer;
// Defines the memory that will actually hold the messages within the message
// buffer.
static uint8_t ucStorageBuffer[STORAGE_SIZE_BYTES];
// The variable used to hold the message buffer structure.
StaticMessageBuffer_t xMessageBufferStruct;

static int sbr_log_init(sbr_log_handle_S *handle_ptr);

void StartLogTask(void const *argument)
{
    sbr_log_init(&log_handle);
    sbr_log_mpu6050_message_S mpu6050_message;
    size_t xReceivedBytes;

    printf("Sbr log task init\n");

    for (;;)
    {
        if (pdFALSE == xMessageBufferIsEmpty(xMessageMPU6050SharedBuffer))
        {
            xReceivedBytes = xMessageBufferReceive(xMessageMPU6050SharedBuffer,
                    &mpu6050_message, sizeof(mpu6050_message), 100);
            if (xReceivedBytes > 0)
            {
                if (sizeof(mpu6050_message) == xReceivedBytes)
                {
                    printf("ga=%.2f aa=%.2f %d\n",
                            mpu6050_message.gyro_angle,
                            mpu6050_message.accel_angle,
                            mpu6050_message.angle_critical);
                }
            }
        }
    }
}

static int sbr_log_init(sbr_log_handle_S *handle_ptr)
{
    int ret_val = 0;
    handle_ptr->uart_handle_ptr = &huart1;

    xMessageMPU6050SharedBuffer = xMessageBufferCreateStatic(
            sizeof(ucStorageBuffer), ucStorageBuffer, &xMessageBufferStruct);

    return ret_val;
}

/* Low level function for printing a char. Needed for printf() */
void _putchar(char character)
{
    HAL_UART_Transmit(&huart1, (uint8_t*) &character, sizeof(character), 100);
}

