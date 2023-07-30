/**
 ******************************************************************************
 * File Name          : bsp_cfg.h
 * Description        : board support package configuration file
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
#ifndef BSP_CONFIG_BSP_CFG_H_
#define BSP_CONFIG_BSP_CFG_H_

#include "stm32f1xx_hal.h"

#define BLUEPILL_LED_Pin        GPIO_PIN_13
#define BLUEPILL_LED_Port       GPIOC
#define MOTOR_L_ENABLE_Pin      GPIO_PIN_6
#define MOTOR_L_ENABLE_Port     GPIOA
#define MOTOR_R_ENABLE_Pin      GPIO_PIN_7
#define MOTOR_R_ENABLE_Port     GPIOA
#define IN1_Pin GPIO_PIN_12
#define IN1_GPIO_Port GPIOB
#define IN2_Pin GPIO_PIN_13
#define IN2_GPIO_Port GPIOB
#define IN3_Pin GPIO_PIN_14
#define IN3_GPIO_Port GPIOB
#define IN4_Pin GPIO_PIN_15
#define IN4_GPIO_Port GPIOB

#define CFG_MPU6050_CALIBRATION_SAMPLES_NUM  (500)

#define CFG_IMU_FREQ_HZ (200)
#define CFG_IMU_MAX_ANGLE (125.0f)
#define CFG_IMU_MIN_ANGLE (65.0f)

#endif /* BSP_CONFIG_BSP_CFG_H_ */
