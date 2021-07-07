/**
  ******************************************************************************
  * File Name          : MPU6050.c
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
#include "MPU6050.h"
#include "main.h"
#include <math.h>

static void MPU6050_setReg(MPU6050_handle_S*, uint8_t, uint8_t);
static int16_t MPU6050_getAccel_X(MPU6050_handle_S*);
static int16_t MPU6050_getAccel_Y(MPU6050_handle_S*);
static int16_t MPU6050_getAccel_Z(MPU6050_handle_S*);
static int16_t MPU6050_getGyro_X(MPU6050_handle_S*);
static int16_t MPU6050_getGyro_Y(MPU6050_handle_S*);
static int16_t MPU6050_getGyro_Z(MPU6050_handle_S*);
/*
 * MPU6050 initialization function
 */
void MPU6050_init(MPU6050_handle_S *mpu6050HandlePtr, I2C_HandleTypeDef *i2cHandlePtr) {
    mpu6050HandlePtr->i2cHandlePtr = i2cHandlePtr;
    MPU6050_setReg(mpu6050HandlePtr, PWR_MGMT_1, 0x00);
    MPU6050_setReg(mpu6050HandlePtr, GYRO_CONFIG, 0x00);
    MPU6050_setReg(mpu6050HandlePtr, ACCEL_CONFIG, 0x08);
    MPU6050_setReg(mpu6050HandlePtr, CONFIG, 0x03);

    for (size_t i = 0; i < 500; i++) {
        if (i % 10 == 0) {
            HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        }
        mpu6050HandlePtr->gyro_offset_x += MPU6050_getGyro_X(mpu6050HandlePtr);
        mpu6050HandlePtr->gyro_offset_y += MPU6050_getGyro_Y(mpu6050HandlePtr);
        HAL_Delay(4);
    }
    mpu6050HandlePtr->gyro_offset_x /= 500;
    mpu6050HandlePtr->gyro_offset_y /= 500;
}

void MPU6050_Handler(MPU6050_handle_S *mpu6050HandlePtr) {
    mpu6050HandlePtr->accel_x = MPU6050_getAccel_X(mpu6050HandlePtr);
    mpu6050HandlePtr->accel_z = MPU6050_getAccel_Z(mpu6050HandlePtr);
    mpu6050HandlePtr->gyro_y = MPU6050_getGyro_Y(mpu6050HandlePtr) - mpu6050HandlePtr->gyro_offset_y;
    mpu6050HandlePtr->accel_angle = (atan2((double) mpu6050HandlePtr->accel_x, -(double) mpu6050HandlePtr->accel_z) * (180.0f / M_PI));

    if ((TRUE == mpu6050HandlePtr->isAngleCritical) && (mpu6050HandlePtr->accel_angle > 70.0f) && (mpu6050HandlePtr->accel_angle < 120.0f)) {
        mpu6050HandlePtr->isAngleCritical = FALSE;
        mpu6050HandlePtr->gyro_angle = mpu6050HandlePtr->accel_angle;
    }
    mpu6050HandlePtr->gyro_angle += ((float) mpu6050HandlePtr->gyro_y / 13100.0f);
    mpu6050HandlePtr->gyro_angle = mpu6050HandlePtr->gyro_angle * 0.996f + mpu6050HandlePtr->accel_angle * 0.004f;

    if ((mpu6050HandlePtr->gyro_angle < 65.0f) || (mpu6050HandlePtr->gyro_angle > 125.0f)) {
        mpu6050HandlePtr->isAngleCritical = TRUE;
    } else {
        mpu6050HandlePtr->isAngleCritical = FALSE;
    }
}

static void MPU6050_setReg(MPU6050_handle_S *mpu6050HandlePtr, uint8_t reg, uint8_t data) {
    uint8_t i2c_buff[2] = { reg, data };
    HAL_I2C_Master_Transmit(mpu6050HandlePtr->i2cHandlePtr, MPU6050_ADDR, i2c_buff, 2, 25);
}

static int16_t MPU6050_getAccel_X(MPU6050_handle_S *mpu6050HandlePtr) {
    uint8_t reg[1] = { 0x3B };
    uint8_t accel_x_buff[2];
    HAL_I2C_Master_Transmit(mpu6050HandlePtr->i2cHandlePtr, MPU6050_ADDR, reg, 1, 25);
    HAL_I2C_Master_Receive(mpu6050HandlePtr->i2cHandlePtr, MPU6050_ADDR, accel_x_buff, 2, 25);
    return ((accel_x_buff[0] << 8) | accel_x_buff[1]);
}

static int16_t MPU6050_getAccel_Y(MPU6050_handle_S *mpu6050HandlePtr) {
    uint8_t reg[1] = { 0x3D };
    uint8_t accel_y_buff[2];
    HAL_I2C_Master_Transmit(mpu6050HandlePtr->i2cHandlePtr, MPU6050_ADDR, reg, 1, 25);
    HAL_I2C_Master_Receive(mpu6050HandlePtr->i2cHandlePtr, MPU6050_ADDR, accel_y_buff, 2, 25);
    return ((accel_y_buff[0] << 8) | accel_y_buff[1]);
}

static int16_t MPU6050_getAccel_Z(MPU6050_handle_S *mpu6050HandlePtr) {
    uint8_t reg[1] = { 0x3F };
    uint8_t accel_z_buff[2];
    HAL_I2C_Master_Transmit(mpu6050HandlePtr->i2cHandlePtr, MPU6050_ADDR, reg, 1, 25);
    HAL_I2C_Master_Receive(mpu6050HandlePtr->i2cHandlePtr, MPU6050_ADDR, accel_z_buff, 2, 25);
    return ((accel_z_buff[0] << 8) | accel_z_buff[1]);
}

static int16_t MPU6050_getGyro_X(MPU6050_handle_S *mpu6050HandlePtr) {
    uint8_t reg[1] = { 0x43 };
    uint8_t gyro_x_buff[2];
    HAL_I2C_Master_Transmit(mpu6050HandlePtr->i2cHandlePtr, MPU6050_ADDR, reg, 1, 25);
    HAL_I2C_Master_Receive(mpu6050HandlePtr->i2cHandlePtr, MPU6050_ADDR, gyro_x_buff, 2, 25);
    return ((gyro_x_buff[0] << 8) | gyro_x_buff[1]);
}

static int16_t MPU6050_getGyro_Y(MPU6050_handle_S *mpu6050HandlePtr) {
    uint8_t reg[1] = { 0x45 };
    uint8_t gyro_y_buff[2];
    HAL_I2C_Master_Transmit(mpu6050HandlePtr->i2cHandlePtr, MPU6050_ADDR, reg, 1, 25);
    HAL_I2C_Master_Receive(mpu6050HandlePtr->i2cHandlePtr, MPU6050_ADDR, gyro_y_buff, 2, 25);
    return ((gyro_y_buff[0] << 8) | gyro_y_buff[1]);
}

static int16_t MPU6050_getGyro_Z(MPU6050_handle_S *mpu6050HandlePtr) {
    uint8_t reg[1] = { 0x47 };
    uint8_t gyro_z_buff[2];
    HAL_I2C_Master_Transmit(mpu6050HandlePtr->i2cHandlePtr, MPU6050_ADDR, reg, 1, 25);
    HAL_I2C_Master_Receive(mpu6050HandlePtr->i2cHandlePtr, MPU6050_ADDR, gyro_z_buff, 2, 25);
    return ((gyro_z_buff[0] << 8) | gyro_z_buff[1]);
}
