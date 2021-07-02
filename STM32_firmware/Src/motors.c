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

/*
 * @brief Function to turn wheels clockwise utilizing LN298N module (H-bridge)
 * @param htim: TIM Base handle
 * @param v: speed, pwm duty cycle
 *              <0, timer period>
 */
void MOTORS_turnClockwise(TIM_HandleTypeDef *htim, uint16_t v) {
    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_SET);
    htim->Instance->CCR1 = v;
    htim->Instance->CCR2 = v;
}

/*
 * @brief Function to turn wheels counter clockwise utilizing LN298N module (H-bridge)
 * @param htim: TIM Base handle
 * @param v: speed/pwm duty cycle
 *              <0, timer period>
 */
void MOTORS_turnCounterClockwise(TIM_HandleTypeDef *htim, uint16_t v) {
    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
    htim->Instance->CCR1 = v;
    htim->Instance->CCR2 = v;
}

/*
 * @brief Function to turn off wheels utilizing LN298N module (H-bridge)
 * @param htim: TIM Base handle
 */
void MOTORS_powerOff(TIM_HandleTypeDef *htim) {
    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
    htim->Instance->CCR1 = 0;
    htim->Instance->CCR2 = 0;
}

