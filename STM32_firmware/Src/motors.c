/**
  ******************************************************************************
  * File Name          : motors.c
  * Description        : Self balancing robot motor control functions
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 Branimir Mihelčić
  * All rights reserved.</center></h2>
  *
  * This software component is licensed under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
#include "motors.h"
#include "main.h"

void MOTORS_init(MOTORS_handle_S *motorsHandlePtr, TIM_HandleTypeDef *timerHandlePtr) {
    motorsHandlePtr->timerPtr = timerHandlePtr;
}

void MOTORS_turnClockwise(MOTORS_handle_S *motorsHandlePtr, uint16_t v) {
    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_SET);
    motorsHandlePtr->timerPtr->Instance->CCR1 = v;
    motorsHandlePtr->timerPtr->Instance->CCR2 = v;
}

void MOTORS_turnCounterClockwise(MOTORS_handle_S *motorsHandlePtr, uint16_t v) {
    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
    motorsHandlePtr->timerPtr->Instance->CCR1 = v;
    motorsHandlePtr->timerPtr->Instance->CCR2 = v;
}

void MOTORS_powerOff(MOTORS_handle_S *motorsHandlePtr) {
    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
    motorsHandlePtr->timerPtr->Instance->CCR1 = 0;
    motorsHandlePtr->timerPtr->Instance->CCR2 = 0;
}

