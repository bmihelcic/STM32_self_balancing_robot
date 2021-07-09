/**
  ******************************************************************************
  * File Name          : MPU6050.h
  * Description        : Self balancing robot MPU6050 functions
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


#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include <stdint.h>
#include "stm32f1xx_hal.h"

#define MPU6050_ADDR  (0x68 << 1)
#define PWR_MGMT_1    (0x6B)
#define CONFIG        (0x1A)
#define GYRO_CONFIG   (0x1B)
#define ACCEL_CONFIG  (0x1C)

typedef struct MPU6050_handle_STRUCT {
    I2C_HandleTypeDef *i2cHandlePtr;
    int16_t accel_x;
    int16_t accel_z;
    int16_t gyro_y;
    int32_t gyro_offset_x;
    int32_t gyro_offset_y;
    float accel_angle;
    float gyro_angle;
    uint8_t isAngleCritical;
}MPU6050_handle_S;

void MPU6050_init(MPU6050_handle_S*, I2C_HandleTypeDef *hi2c);
void MPU6050_Handler(MPU6050_handle_S*);

#endif /* INC_MPU6050_H_ */
