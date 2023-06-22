/**
 ******************************************************************************
 * File Name          : mpu6050.c
 * Description        : MPU6050 accelerometer and gyroscope source file
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

#include "../inc/mpu6050.h"

/**
 * @brief Function implementing the mpu6050Task thread.
 * @param argument: Not used
 * @retval None
 */
void StartMpu6050Task(void const *argument)
{
    /* Infinite loop */
    for (;;)
    {
        osDelay(1);
    }
}
