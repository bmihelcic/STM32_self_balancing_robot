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
#include "bsp_cfg.h"
#include "stm32f1xx_hal.h"

extern TIM_HandleTypeDef htim3;

void MOTORS_Init() {
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}

void MOTORS_Turn_Clockwise(uint16_t v) {
    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_SET);
    htim3.Instance->CCR1 = v;
    htim3.Instance->CCR2 = v;
}

void MOTORS_Turn_Counter_Clockwise(uint16_t v) {
    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
    htim3.Instance->CCR1 = v;
    htim3.Instance->CCR2 = v;
}

void MOTORS_Power_Off() {
    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
    htim3.Instance->CCR1 = 0;
    htim3.Instance->CCR2 = 0;
}

