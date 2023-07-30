/**
 ******************************************************************************
 * File Name          : sbr_led.c
 * Description        : Self Balancing Robot LEDs handler source file
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
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "bsp_cfg.h"
/**
 * @brief Function implementing led thread.
 * @param argument: Not used
 * @retval None
 */
void LED_Thread(void const *argument)
{
    for (;;)
    {
        HAL_GPIO_TogglePin(BLUEPILL_LED_Port, BLUEPILL_LED_Pin);
        osDelay(500);
    }
}
