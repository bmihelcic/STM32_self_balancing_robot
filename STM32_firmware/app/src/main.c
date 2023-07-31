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
#include <log.h>
#include "mcu.h"
#include "cmsis_os.h"
#include "printf.h"
#include "imu.h"
#include "pid_control.h"
#include "command.h"

osThreadId led_thread_id;
uint32_t led_thread_buffer[128];
osStaticThreadDef_t led_thread_control_block;

osThreadId log_thread_id;
uint32_t log_thread_buffer[128];
osStaticThreadDef_t log_thread_control_block;

osThreadId imu_thread_id;
uint32_t imu_thread_buffer[128];
osStaticThreadDef_t imu_thread_control_block;

osThreadId pid_thread_id;
uint32_t pid_thread_buffer[128];
osStaticThreadDef_t pid_thread_control_block;

osThreadId command_thread_id;
uint32_t command_thread_buffer[128];
osStaticThreadDef_t command_thread_control_block;

osThreadStaticDef(LED, LED_Thread, osPriorityLow, 0, configMINIMAL_STACK_SIZE,
        led_thread_buffer, &led_thread_control_block);
osThreadStaticDef(LOG, LOG_Thread, osPriorityLow, 0, configMINIMAL_STACK_SIZE,
        log_thread_buffer, &log_thread_control_block);
osThreadStaticDef(IMU, IMU_Thread, osPriorityAboveNormal, 0, configMINIMAL_STACK_SIZE,
        imu_thread_buffer, &imu_thread_control_block);
osThreadStaticDef(PID, PID_CONTROL_Thread, osPriorityAboveNormal, 0, configMINIMAL_STACK_SIZE,
        pid_thread_buffer, &pid_thread_control_block);
osThreadStaticDef(COMMAND, COMMAND_Thread, osPriorityAboveNormal, 0, configMINIMAL_STACK_SIZE,
        command_thread_buffer, &command_thread_control_block);


/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    mcu_init();

    led_thread_id = osThreadCreate(osThread(LED), NULL);
    log_thread_id = osThreadCreate(osThread(LOG), NULL);
    imu_thread_id = osThreadCreate(osThread(IMU), NULL);
    pid_thread_id = osThreadCreate(osThread(PID), NULL);
    command_thread_id = osThreadCreate(osThread(COMMAND), NULL);

   /* Start scheduler */
    osKernelStart();
    /* We should never get here as control is now taken by the scheduler */
}
