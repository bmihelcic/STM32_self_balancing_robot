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
#include "bsp_cfg.h"
#include <math.h>
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#define I2C_TIMEOUT    (50u)

extern I2C_HandleTypeDef hi2c1;

void mpu6050_init(MPU6050_handle_S *mpu6050_handle_ptr,
        I2C_HandleTypeDef *i2c_handle_ptr);
static void MPU6050_set_reg(MPU6050_handle_S*, uint8_t, uint8_t);
static int16_t MPU6050_get_accel_x(MPU6050_handle_S*);
static int16_t MPU6050_get_accel_y(MPU6050_handle_S*);
static int16_t MPU6050_get_accel_z(MPU6050_handle_S*);
static int16_t MPU6050_get_gyro_x(MPU6050_handle_S*);
static int16_t MPU6050_get_gyro_Y(MPU6050_handle_S*);
static int16_t MPU6050_get_gyro_z(MPU6050_handle_S*);
static void MPU6050_handler(MPU6050_handle_S *mpu6050_handle_ptr);
inline static void mpu6050_calculate_gyro_factor(
        MPU6050_handle_S *mpu6050_handle_ptr);

/**
 * @brief Function implementing the mpu6050Task thread.
 * @param argument: Not used
 * @retval None
 */
void StartMpu6050Task(void const *argument)
{
    MPU6050_handle_S mpu6050_handle;
    const uint32_t delay_ms = (1000 / CONFIG_MPU6050_UPDATE_FREQ_HZ);

    mpu6050_init(&mpu6050_handle, &hi2c1);

    /* Infinite loop */
    for (;;)
    {
        MPU6050_handler(&mpu6050_handle);
        osDelay(delay_ms);
    }
}

/*
 * MPU6050 initialization function
 */
void mpu6050_init(MPU6050_handle_S *mpu6050_handle_ptr,
        I2C_HandleTypeDef *i2c_handle_ptr)
{
    mpu6050_handle_ptr->i2c_handle_ptr = i2c_handle_ptr;
    MPU6050_set_reg(mpu6050_handle_ptr, MPU6050_REG_PWR_MGMT_1, 0x00);
    MPU6050_set_reg(mpu6050_handle_ptr, MPU6050_REG_GYRO_CONFIG,
            ((uint8_t) MPU6050_GYRO_RANGE_250DPS << 3));
    MPU6050_set_reg(mpu6050_handle_ptr, MPU6050_REG_ACCEL_CONFIG,
            ((uint8_t) MPU6050_ACCELEROMETER_RANGE_2G << 3));
    MPU6050_set_reg(mpu6050_handle_ptr, MPU6050_REG_CONFIG, 0x03);

    mpu6050_handle_ptr->accel_range = MPU6050_ACCELEROMETER_RANGE_2G;
    mpu6050_handle_ptr->gyro_range = MPU6050_GYRO_RANGE_250DPS;

    mpu6050_calculate_gyro_factor(mpu6050_handle_ptr);

    for (size_t i = 0; i < CONFIG_MPU6050_CALIBRATION_SAMPLES_NUM; i++)
    {
        mpu6050_handle_ptr->gyro_offset_x += MPU6050_get_gyro_x(
                mpu6050_handle_ptr);
        mpu6050_handle_ptr->gyro_offset_y += MPU6050_get_gyro_Y(
                mpu6050_handle_ptr);
        osDelay(5);
    }
    mpu6050_handle_ptr->gyro_offset_x /= CONFIG_MPU6050_CALIBRATION_SAMPLES_NUM;
    mpu6050_handle_ptr->gyro_offset_y /= CONFIG_MPU6050_CALIBRATION_SAMPLES_NUM;
}

/**
 * Calculate gyroscope value change factor
 * Check MPU6050 product specification, page 12
 * Factor is different for different reading frequency
 */
inline static void mpu6050_calculate_gyro_factor(
        MPU6050_handle_S *mpu6050_handle_ptr)
{
    switch (mpu6050_handle_ptr->gyro_range) {
    case MPU6050_GYRO_RANGE_250DPS:
        mpu6050_handle_ptr->gyro_val_change_factor = 131.0f
                * CONFIG_MPU6050_UPDATE_FREQ_HZ;
        break;
    case MPU6050_GYRO_RANGE_500DPS:
        mpu6050_handle_ptr->gyro_val_change_factor = 65.5f
                * CONFIG_MPU6050_UPDATE_FREQ_HZ;
        break;
    case MPU6050_GYRO_RANGE_1000DPS:
        mpu6050_handle_ptr->gyro_val_change_factor = 32.8f
                * CONFIG_MPU6050_UPDATE_FREQ_HZ;
        break;
    case MPU6050_GYRO_RANGE_2000DPS:
        mpu6050_handle_ptr->gyro_val_change_factor = 16.4f
                * CONFIG_MPU6050_UPDATE_FREQ_HZ;
        break;
    }
}

static void MPU6050_handler(MPU6050_handle_S *mpu6050_handle_ptr)
{
    mpu6050_handle_ptr->accel_x = MPU6050_get_accel_x(mpu6050_handle_ptr);
    mpu6050_handle_ptr->accel_z = MPU6050_get_accel_z(mpu6050_handle_ptr);
    mpu6050_handle_ptr->gyro_y = MPU6050_get_gyro_Y(mpu6050_handle_ptr)
            - mpu6050_handle_ptr->gyro_offset_y;
    mpu6050_handle_ptr->accel_angle = (atan2(
            (double) mpu6050_handle_ptr->accel_x,
            -(double) mpu6050_handle_ptr->accel_z) * (180.0f / M_PI));

    if ((1 == mpu6050_handle_ptr->is_angle_critical)
            && (mpu6050_handle_ptr->accel_angle > 70.0f)
            && (mpu6050_handle_ptr->accel_angle < 120.0f))
    {
        mpu6050_handle_ptr->is_angle_critical = 0;
        mpu6050_handle_ptr->gyro_angle = mpu6050_handle_ptr->accel_angle;
    }
    mpu6050_handle_ptr->gyro_angle += ((float) mpu6050_handle_ptr->gyro_y
            / mpu6050_handle_ptr->gyro_val_change_factor);
    mpu6050_handle_ptr->gyro_angle = mpu6050_handle_ptr->gyro_angle * 0.996f
            + mpu6050_handle_ptr->accel_angle * 0.004f;

    if ((mpu6050_handle_ptr->gyro_angle < CONFIG_MPU6050_MIN_ANGLE)
            || (mpu6050_handle_ptr->gyro_angle > CONFIG_MPU6050_MAX_ANGLE))
    {
        mpu6050_handle_ptr->is_angle_critical = 1;
    } else
    {
        mpu6050_handle_ptr->is_angle_critical = 0;
    }
}

static void MPU6050_set_reg(MPU6050_handle_S *mpu6050_handle_ptr, uint8_t reg,
        uint8_t data)
{
    uint8_t i2c_buff[2] = { reg, data };
    HAL_I2C_Master_Transmit(mpu6050_handle_ptr->i2c_handle_ptr, MPU6050_ADDR,
            i2c_buff, 2, I2C_TIMEOUT);
}

static int16_t MPU6050_get_accel_x(MPU6050_handle_S *mpu6050_handle_ptr)
{
    uint8_t reg[1] = { MPU6050_REG_ACCEL_XOUT_H };
    uint8_t accel_x_buff[2];
    HAL_I2C_Master_Transmit(mpu6050_handle_ptr->i2c_handle_ptr, MPU6050_ADDR,
            reg, 1, I2C_TIMEOUT);
    HAL_I2C_Master_Receive(mpu6050_handle_ptr->i2c_handle_ptr, MPU6050_ADDR,
            accel_x_buff, 2, I2C_TIMEOUT);
    return ((accel_x_buff[0] << 8) | accel_x_buff[1]);
}

static int16_t MPU6050_get_accel_y(MPU6050_handle_S *mpu6050_handle_ptr)
{
    uint8_t reg[1] = { MPU6050_REG_ACCEL_YOUT_H };
    uint8_t accel_y_buff[2];
    HAL_I2C_Master_Transmit(mpu6050_handle_ptr->i2c_handle_ptr, MPU6050_ADDR,
            reg, 1, I2C_TIMEOUT);
    HAL_I2C_Master_Receive(mpu6050_handle_ptr->i2c_handle_ptr, MPU6050_ADDR,
            accel_y_buff, 2, I2C_TIMEOUT);
    return ((accel_y_buff[0] << 8) | accel_y_buff[1]);
}

static int16_t MPU6050_get_accel_z(MPU6050_handle_S *mpu6050_handle_ptr)
{
    uint8_t reg[1] = { MPU6050_REG_ACCEL_ZOUT_H };
    uint8_t accel_z_buff[2];
    HAL_I2C_Master_Transmit(mpu6050_handle_ptr->i2c_handle_ptr, MPU6050_ADDR,
            reg, 1, I2C_TIMEOUT);
    HAL_I2C_Master_Receive(mpu6050_handle_ptr->i2c_handle_ptr, MPU6050_ADDR,
            accel_z_buff, 2, I2C_TIMEOUT);
    return ((accel_z_buff[0] << 8) | accel_z_buff[1]);
}

static int16_t MPU6050_get_gyro_x(MPU6050_handle_S *mpu6050_handle_ptr)
{
    uint8_t reg[1] = { MPU6050_REG_GYRO_XOUT_H };
    uint8_t gyro_x_buff[2];
    HAL_I2C_Master_Transmit(mpu6050_handle_ptr->i2c_handle_ptr, MPU6050_ADDR,
            reg, 1, I2C_TIMEOUT);
    HAL_I2C_Master_Receive(mpu6050_handle_ptr->i2c_handle_ptr, MPU6050_ADDR,
            gyro_x_buff, 2, I2C_TIMEOUT);
    return ((gyro_x_buff[0] << 8) | gyro_x_buff[1]);
}

static int16_t MPU6050_get_gyro_Y(MPU6050_handle_S *mpu6050_handle_ptr)
{
    uint8_t reg[1] = { MPU6050_REG_GYRO_YOUT_H };
    uint8_t gyro_y_buff[2];
    HAL_I2C_Master_Transmit(mpu6050_handle_ptr->i2c_handle_ptr, MPU6050_ADDR,
            reg, 1, I2C_TIMEOUT);
    HAL_I2C_Master_Receive(mpu6050_handle_ptr->i2c_handle_ptr, MPU6050_ADDR,
            gyro_y_buff, 2, I2C_TIMEOUT);
    return ((gyro_y_buff[0] << 8) | gyro_y_buff[1]);
}

static int16_t MPU6050_get_gyro_z(MPU6050_handle_S *mpu6050_handle_ptr)
{
    uint8_t reg[1] = { MPU6050_REG_GYRO_ZOUT_H };
    uint8_t gyro_z_buff[2];
    HAL_I2C_Master_Transmit(mpu6050_handle_ptr->i2c_handle_ptr, MPU6050_ADDR,
            reg, 1, I2C_TIMEOUT);
    HAL_I2C_Master_Receive(mpu6050_handle_ptr->i2c_handle_ptr, MPU6050_ADDR,
            gyro_z_buff, 2, I2C_TIMEOUT);
    return ((gyro_z_buff[0] << 8) | gyro_z_buff[1]);
}
