/**
  ******************************************************************************
  * File Name          : my_functions.h
  * Description        : Self balancing robot specific functions, e.g. motor control
  *                      MPU6050 functions, etc.
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

#ifndef MY_FUNCTIONS_H_
#define MY_FUNCTIONS_H_

/* MPU6050 */
#define MPU6050_ADDR  (0x68 << 1)
#define PWR_MGMT_1    (0x6B)
#define CONFIG        (0x1A)
#define GYRO_CONFIG   (0x1B)
#define ACCEL_CONFIG  (0x1C)

#define TRUE    (1u)
#define FALSE   (0u)

void turnMotorsClockwise(TIM_HandleTypeDef*, uint16_t);
void turnMotorsCounterClockwise(TIM_HandleTypeDef*, uint16_t);
void resetMotors(TIM_HandleTypeDef*);
void MPU6050_setReg(I2C_HandleTypeDef*, uint8_t, uint8_t);
int16_t MPU6050_getAccel_X(I2C_HandleTypeDef*);
int16_t MPU6050_getAccel_Y(I2C_HandleTypeDef*);
int16_t MPU6050_getAccel_Z(I2C_HandleTypeDef*);
int16_t MPU6050_getGyro_X(I2C_HandleTypeDef*);
int16_t MPU6050_getGyro_Y(I2C_HandleTypeDef*);
int16_t MPU6050_getGyro_Z(I2C_HandleTypeDef*);

#endif /* MY_FUNCTIONS_H_ */
