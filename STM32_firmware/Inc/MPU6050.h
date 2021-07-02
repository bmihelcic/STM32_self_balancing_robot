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

void MPU6050_setReg(I2C_HandleTypeDef*, uint8_t, uint8_t);
int16_t MPU6050_getAccel_X(I2C_HandleTypeDef*);
int16_t MPU6050_getAccel_Y(I2C_HandleTypeDef*);
int16_t MPU6050_getAccel_Z(I2C_HandleTypeDef*);
int16_t MPU6050_getGyro_X(I2C_HandleTypeDef*);
int16_t MPU6050_getGyro_Y(I2C_HandleTypeDef*);
int16_t MPU6050_getGyro_Z(I2C_HandleTypeDef*);

#endif /* INC_MPU6050_H_ */
