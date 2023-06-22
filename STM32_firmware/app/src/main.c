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

#include "../../bsp/inc/motor_control.h"
#include "../../bsp/inc/mpu6050.h"
#include "mcu.h"
#include "cmsis_os.h"
#include "sbr_pid_control.h"
#include "sbr_command.h"
#include "sbr_log.h"
#include "sbr_master.h"
#include "sbr_led.h"

osThreadId defaultTaskHandle;
uint32_t defaultTaskBuffer[128];
osStaticThreadDef_t defaultTaskControlBlock;
osThreadId mpu6050TaskHandle;
uint32_t mpu6050TaskBuffer[128];
osStaticThreadDef_t mpu6050TaskControlBlock;
osThreadId pidCtrlTaskHandle;
uint32_t pidControllerTaBuffer[128];
osStaticThreadDef_t pidControllerTaControlBlock;
osThreadId sbrCmdTaskHandle;
uint32_t sbrCommandHandlBuffer[128];
osStaticThreadDef_t sbrCommandHandlControlBlock;
osThreadId motorCtrlTaskHandle;
uint32_t motorCtrlTaskBuffer[128];
osStaticThreadDef_t motorCtrlTaskControlBlock;
osThreadId sbrMasterTaskHandle;
uint32_t sbrMasterTaskBuffer[128];
osStaticThreadDef_t sbrMasterTaskControlBlock;
osThreadId logTaskHandle;
uint32_t logTaskBuffer[128];
osStaticThreadDef_t logTaskControlBlock;
osThreadId ledTaskHandle;
uint32_t ledTaskBuffer[128];
osStaticThreadDef_t ledTaskControlBlock;

void StartDefaultTask(void const *argument);

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    mcu_init();

    /* Create the thread(s) */
    /* definition and creation of defaultTask */
    osThreadStaticDef(defaultTask, StartDefaultTask, osPriorityLow, 0, 128,
            defaultTaskBuffer, &defaultTaskControlBlock);
    defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

    /* definition and creation of mpu6050Task */
    osThreadStaticDef(mpu6050Task, StartMpu6050Task, osPriorityHigh, 0, 128,
            mpu6050TaskBuffer, &mpu6050TaskControlBlock);
    mpu6050TaskHandle = osThreadCreate(osThread(mpu6050Task), NULL);

    /* definition and creation of pidCtrlTask */
    osThreadStaticDef(pidCtrlTask, StartPidControllerTask, osPriorityHigh, 0,
            128, pidControllerTaBuffer, &pidControllerTaControlBlock);
    pidCtrlTaskHandle = osThreadCreate(osThread(pidCtrlTask), NULL);

    /* definition and creation of sbrMasterTask */
    osThreadStaticDef(sbrMasterTask, StartSbrMasterTask, osPriorityAboveNormal,
            0, 128, sbrMasterTaskBuffer, &sbrMasterTaskControlBlock);
    sbrMasterTaskHandle = osThreadCreate(osThread(sbrMasterTask), NULL);

    /* definition and creation of debugMsgTask */
    osThreadStaticDef(logTask, StartLogTask, osPriorityLow, 0,
            128, logTaskBuffer, &logTaskControlBlock);
    logTaskHandle = osThreadCreate(osThread(logTask), NULL);

    /* definition and creation of sbrCmdTask */
    osThreadStaticDef(sbrCmdTask, StartSbrCommandTask, osPriorityLow, 0, 128,
            sbrCommandHandlBuffer, &sbrCommandHandlControlBlock);
    sbrCmdTaskHandle = osThreadCreate(osThread(sbrCmdTask), NULL);

    /* definition and creation of motorCtrlTask */
    osThreadStaticDef(motorCtrlTask, StartMotorCtrlTask, osPriorityHigh, 0, 128,
            motorCtrlTaskBuffer, &motorCtrlTaskControlBlock);
    motorCtrlTaskHandle = osThreadCreate(osThread(motorCtrlTask), NULL);

    /* definition and creation of ledTask */
    osThreadStaticDef(ledTask, StartLedTask, osPriorityLow, 0, 128,
            ledTaskBuffer, &ledTaskControlBlock);
    ledTaskHandle = osThreadCreate(osThread(ledTask), NULL);

    /* Start scheduler */
    osKernelStart();
    /* We should never get here as control is now taken by the scheduler */
}

/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
void StartDefaultTask(void const *argument)
{
    /* Infinite loop */
    for (;;)
    {
        osDelay(1);
    }
}
