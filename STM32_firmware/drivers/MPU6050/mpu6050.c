/**
 ******************************************************************************
 * File Name          : mpu6050.c
 * Description        : MPU6050 accelerometer and gyroscope source file.
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

#include "mpu6050.h"
#include "stm32f1xx_hal.h"

#define I2C_TIMEOUT    (HAL_MAX_DELAY)

static MPU6050_handle_S mpu6050_handle;

static mpu6050_err_t mpu6050_set_reg(uint8_t reg, uint8_t data);
/*
 * MPU6050 initialization function
 */
mpu6050_err_t MPU6050_Init(I2C_HandleTypeDef *i2c_handle_ptr)
{
    mpu6050_err_t ret_val = MPU6050_INIT_FAIL;

    mpu6050_handle.i2c_handle_ptr = i2c_handle_ptr;
    if (MPU6050_ERR_OK != mpu6050_set_reg(MPU6050_REG_PWR_MGMT_1,
                                          0x00)) {
        goto exit;
    }
    if (MPU6050_ERR_OK != mpu6050_set_reg(MPU6050_REG_GYRO_CONFIG,
                                          ((uint8_t) MPU6050_GYRO_RANGE_250DPS << 3))) {
        goto exit;
    }
    if (MPU6050_ERR_OK != mpu6050_set_reg(MPU6050_REG_ACCEL_CONFIG,
                                          ((uint8_t) MPU6050_ACCELEROMETER_RANGE_2G << 3))) {
        goto exit;
    }
    if (MPU6050_ERR_OK != mpu6050_set_reg(MPU6050_REG_CONFIG,
                                          0x03)) {
        goto exit;
    }

    mpu6050_handle.accel_range = MPU6050_ACCELEROMETER_RANGE_2G;
    mpu6050_handle.gyro_range = MPU6050_GYRO_RANGE_250DPS;

    ret_val = MPU6050_ERR_OK;

    return ret_val;

exit:
    return ret_val;
}

int16_t MPU6050_Read_Accel_X()
{
    uint8_t reg[1] = { MPU6050_REG_ACCEL_XOUT_H };
    uint8_t accel_x_buff[2];
    HAL_I2C_Master_Transmit(mpu6050_handle.i2c_handle_ptr,
                            MPU6050_ADDR,
                            reg,
                            1,
                            I2C_TIMEOUT);
    HAL_I2C_Master_Receive(mpu6050_handle.i2c_handle_ptr,
                           MPU6050_ADDR,
                           accel_x_buff,
                           2,
                           I2C_TIMEOUT);
    return ((accel_x_buff[0] << 8) | accel_x_buff[1]);
}

int16_t MPU6050_Read_Accel_Y()
{
    uint8_t reg[1] = { MPU6050_REG_ACCEL_YOUT_H };
    uint8_t accel_y_buff[2];
    HAL_I2C_Master_Transmit(mpu6050_handle.i2c_handle_ptr,
                            MPU6050_ADDR,
                            reg,
                            1,
                            I2C_TIMEOUT);
    HAL_I2C_Master_Receive(mpu6050_handle.i2c_handle_ptr,
                           MPU6050_ADDR,
                           accel_y_buff,
                           2,
                           I2C_TIMEOUT);
    return ((accel_y_buff[0] << 8) | accel_y_buff[1]);
}

int16_t MPU6050_Read_Accel_Z()
{
    uint8_t reg[1] = { MPU6050_REG_ACCEL_ZOUT_H };
    uint8_t accel_z_buff[2];
    HAL_I2C_Master_Transmit(mpu6050_handle.i2c_handle_ptr,
                            MPU6050_ADDR,
                            reg,
                            1,
                            I2C_TIMEOUT);
    HAL_I2C_Master_Receive(mpu6050_handle.i2c_handle_ptr,
                           MPU6050_ADDR,
                           accel_z_buff,
                           2,
                           I2C_TIMEOUT);
    return ((accel_z_buff[0] << 8) | accel_z_buff[1]);
}

int16_t MPU6050_Read_Gyro_X()
{
    uint8_t reg[1] = { MPU6050_REG_GYRO_XOUT_H };
    uint8_t gyro_x_buff[2];
    HAL_I2C_Master_Transmit(mpu6050_handle.i2c_handle_ptr,
                            MPU6050_ADDR,
                            reg,
                            1,
                            I2C_TIMEOUT);
    HAL_I2C_Master_Receive(mpu6050_handle.i2c_handle_ptr,
                           MPU6050_ADDR,
                           gyro_x_buff,
                           2,
                           I2C_TIMEOUT);
    return ((gyro_x_buff[0] << 8) | gyro_x_buff[1]);
}

int16_t MPU6050_Read_Gyro_Y()
{
    uint8_t reg[1] = { MPU6050_REG_GYRO_YOUT_H };
    uint8_t gyro_y_buff[2];
    HAL_I2C_Master_Transmit(mpu6050_handle.i2c_handle_ptr,
                            MPU6050_ADDR,
                            reg,
                            1,
                            I2C_TIMEOUT);
    HAL_I2C_Master_Receive(mpu6050_handle.i2c_handle_ptr,
                           MPU6050_ADDR,
                           gyro_y_buff,
                           2,
                           I2C_TIMEOUT);
    return ((gyro_y_buff[0] << 8) | gyro_y_buff[1]);
}

int16_t MPU6050_Read_Gyro_Z()
{
    uint8_t reg[1] = { MPU6050_REG_GYRO_ZOUT_H };
    uint8_t gyro_z_buff[2];
    HAL_I2C_Master_Transmit(mpu6050_handle.i2c_handle_ptr,
                            MPU6050_ADDR,
                            reg,
                            1,
                            I2C_TIMEOUT);
    HAL_I2C_Master_Receive(mpu6050_handle.i2c_handle_ptr,
                           MPU6050_ADDR,
                           gyro_z_buff,
                           2,
                           I2C_TIMEOUT);
    return ((gyro_z_buff[0] << 8) | gyro_z_buff[1]);
}

mpu6050_gyroscope_range_t MPU6050_Get_Gyro_Range()
{
    return mpu6050_handle.gyro_range;
}

static mpu6050_err_t mpu6050_set_reg(uint8_t reg, uint8_t data)
{
    mpu6050_err_t ret_val;
    uint8_t i2c_buff[2] = { reg, data };

    if (HAL_OK != HAL_I2C_Master_Transmit(mpu6050_handle.i2c_handle_ptr,
                                          MPU6050_ADDR,
                                          i2c_buff,
                                          2,
                                          I2C_TIMEOUT)) {
        ret_val = MPU6050_I2C_FAIL;
    } else {
        ret_val = MPU6050_ERR_OK;
    }

    return ret_val;
}
