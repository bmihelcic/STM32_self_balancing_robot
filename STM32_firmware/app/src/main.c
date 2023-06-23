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

#include "mcu.h"
#include "cmsis_os.h"
#include "sbr_pid_control.h"
#include "sbr_command.h"
#include "sbr_log.h"
#include "sbr_master.h"
#include "sbr_led.h"
#include "motor_control.h"
#include "mpu6050.h"
#include "printf.h"

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


/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    mcu_init();

    /* Create the thread(s) */
    printf("Creating mpu6050 task...\n");
    /* definition and creation of mpu6050Task */
    osThreadStaticDef(mpu6050Task, StartMpu6050Task, osPriorityHigh, 0, 128,
            mpu6050TaskBuffer, &mpu6050TaskControlBlock);
    mpu6050TaskHandle = osThreadCreate(osThread(mpu6050Task), NULL);

    printf("Creating pid control task...\n");
    /* definition and creation of pidCtrlTask */
    osThreadStaticDef(pidCtrlTask, StartPidControllerTask, osPriorityHigh, 0,
            128, pidControllerTaBuffer, &pidControllerTaControlBlock);
    pidCtrlTaskHandle = osThreadCreate(osThread(pidCtrlTask), NULL);

    printf("Creating sbr master task...\n");
    /* definition and creation of sbrMasterTask */
    osThreadStaticDef(sbrMasterTask, StartSbrMasterTask, osPriorityAboveNormal,
            0, 128, sbrMasterTaskBuffer, &sbrMasterTaskControlBlock);
    sbrMasterTaskHandle = osThreadCreate(osThread(sbrMasterTask), NULL);

    printf("Creating log task...\n");
    /* definition and creation of logTask */
    osThreadStaticDef(logTask, StartLogTask, osPriorityLow, 0,
            128, logTaskBuffer, &logTaskControlBlock);
    logTaskHandle = osThreadCreate(osThread(logTask), NULL);

    printf("Creating sbr command task...\n");
    /* definition and creation of sbrCmdTask */
    osThreadStaticDef(sbrCmdTask, StartSbrCommandTask, osPriorityLow, 0, 128,
            sbrCommandHandlBuffer, &sbrCommandHandlControlBlock);
    sbrCmdTaskHandle = osThreadCreate(osThread(sbrCmdTask), NULL);

    printf("Creating motor control task...\n");
    /* definition and creation of motorCtrlTask */
    osThreadStaticDef(motorCtrlTask, StartMotorCtrlTask, osPriorityHigh, 0, 128,
            motorCtrlTaskBuffer, &motorCtrlTaskControlBlock);
    motorCtrlTaskHandle = osThreadCreate(osThread(motorCtrlTask), NULL);

    printf("Creating led task...\n");
    /* definition and creation of ledTask */
    osThreadStaticDef(ledTask, StartLedTask, osPriorityLow, 0, 128,
            ledTaskBuffer, &ledTaskControlBlock);
    ledTaskHandle = osThreadCreate(osThread(ledTask), NULL);

    /* Start scheduler */
    printf("Starting kernel!\n");
    osKernelStart();
    /* We should never get here as control is now taken by the scheduler */
}
