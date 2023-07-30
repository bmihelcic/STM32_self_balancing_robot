/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main source file
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

#include <led.h>
#include "mcu.h"
#include "cmsis_os.h"
#include "sbr_pid_control.h"
#include "sbr_command.h"
#include "sbr_log.h"
#include "sbr_master.h"
#include "motor_control.h"
#include "mpu6050.h"
#include "printf.h"

osThreadId led_thread_id;
uint32_t led_thread_buffer[128];
osStaticThreadDef_t led_thread_control_block;


/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    mcu_init();

    osThreadStaticDef(LED, LED_Thread, osPriorityLow, 0, 128,
            led_thread_buffer, &led_thread_control_block);
    led_thread_id = osThreadCreate(osThread(LED), NULL);


   /* Start scheduler */
    osKernelStart();
    /* We should never get here as control is now taken by the scheduler */
}
