/**
 ******************************************************************************
 * File Name          : motor_control.c
 * Description        : Self balancing robot motor control functions
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

#include "../inc/motor_control.h"

/**
 * @brief Function implementing the motorCtrlTask thread.
 * @param argument: Not used
 * @retval None
 */
void StartMotorCtrlTask(void const *argument)
{
    /* Infinite loop */
    for (;;)
    {
        osDelay(1);
    }
}

