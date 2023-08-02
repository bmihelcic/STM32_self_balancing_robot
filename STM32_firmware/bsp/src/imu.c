/**
 ******************************************************************************
 * File Name          : sbr_imu.c
 * Description        : Self Balancing Robot Inertial Measurement Unit source file
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
#include "imu.h"
#include "log.h"
#include "cmsis_os.h"
#include "message_buffer.h"
#include "printf.h"
#include "bsp_cfg.h"
#include "mpu6050.h"
#include <math.h>
#include "os_resources.h"

extern I2C_HandleTypeDef hi2c1;
static imu_handle_S imu_handle;

void imu_init();
void imu_proccess_sensor_data();
int imu_calc_gyro_scale_factor(float *scale_factor);

void IMU_Thread(void const *argument)
{
    uint32_t os_delay_prev_wake_time;

    imu_init();

    os_delay_prev_wake_time = osKernelSysTick();

    if (1u == imu_handle.is_initialized) {
        if (pdTRUE == xSemaphoreTake(uart_mutex,
                                     portMAX_DELAY)) {
            LOG_Transmit_Blocking("imu init success\n");
            xSemaphoreGive(uart_mutex);
        }

        while (1) {
            imu_proccess_sensor_data();
            osDelayUntil(&os_delay_prev_wake_time,
                         CFG_IMU_FREQ_MS);
        }
    } else {
        if (pdTRUE == xSemaphoreTake(uart_mutex,
                                     portMAX_DELAY)) {
            LOG_Transmit_Blocking("imu init fail\n");
            xSemaphoreGive(uart_mutex);
        }
        while (1) {
            osDelay(1000);
        }
    }
}

float IMU_Get_Gyro_Angle()
{
    return imu_handle.gyro_angle;
}

float IMU_Get_Accel_Angle()
{
    return imu_handle.accel_angle;
}

uint8_t IMU_Is_Angle_Critical()
{
    return imu_handle.is_angle_critical;
}

void imu_init()
{
    imu_handle.is_initialized = 0u;

    if (MPU6050_ERR_OK == MPU6050_Init(&hi2c1)) {
        for (size_t i = 0; i < CFG_MPU6050_CALIBRATION_SAMPLES_NUM; i++) {
            imu_handle.gyro_offset_x += MPU6050_Read_Gyro_X();
            imu_handle.gyro_offset_y += MPU6050_Read_Gyro_Y();
        }
        imu_handle.gyro_offset_x /= CFG_MPU6050_CALIBRATION_SAMPLES_NUM;
        imu_handle.gyro_offset_y /= CFG_MPU6050_CALIBRATION_SAMPLES_NUM;
        if (0 == imu_calc_gyro_scale_factor(&imu_handle.gyro_val_change_factor)) {
            imu_handle.is_initialized = 1u;
        }
    }
}

void imu_proccess_sensor_data()
{
    int16_t accel_x;
    int16_t accel_z;
    int16_t gyro_y;

    accel_x = MPU6050_Read_Accel_X();
    accel_z = MPU6050_Read_Accel_Z();
    gyro_y = MPU6050_Read_Gyro_Y() - imu_handle.gyro_offset_y;

    imu_handle.accel_angle = (atan2((double) accel_x,
                                    -(double) accel_z) * (180.0f / M_PI));

    if ((1 == imu_handle.is_angle_critical) && (imu_handle.accel_angle > 70.0f) && (imu_handle.accel_angle < 120.0f)) {
        imu_handle.is_angle_critical = 0;
        imu_handle.gyro_angle = imu_handle.accel_angle;
    }

    imu_handle.gyro_angle += ((float) gyro_y / imu_handle.gyro_val_change_factor);
    imu_handle.robot_angle = imu_handle.gyro_angle * 0.996f + imu_handle.accel_angle * 0.004f;

    if ((imu_handle.robot_angle < CFG_IMU_MIN_ANGLE) || (imu_handle.robot_angle > CFG_IMU_MAX_ANGLE)) {
        imu_handle.is_angle_critical = 1;
    } else {
        imu_handle.is_angle_critical = 0;
    }
}

/**
 * Calculate gyroscope value change factor
 * Check MPU6050 product specification, page 12
 * Factor is different for different reading frequency
 */
int imu_calc_gyro_scale_factor(float *scale_factor)
{
    int ret_val = -1;

    if (NULL != scale_factor) {
        switch (MPU6050_Get_Gyro_Range())
        {
            case MPU6050_GYRO_RANGE_250DPS:
                *scale_factor = 131.0f * CFG_IMU_FREQ_HZ;
                ret_val = 0;
                break;
            case MPU6050_GYRO_RANGE_500DPS:
                *scale_factor = 65.5f * CFG_IMU_FREQ_HZ;
                ret_val = 0;
                break;
            case MPU6050_GYRO_RANGE_1000DPS:
                *scale_factor = 32.8f * CFG_IMU_FREQ_HZ;
                ret_val = 0;
                break;
            case MPU6050_GYRO_RANGE_2000DPS:
                *scale_factor = 16.4f * CFG_IMU_FREQ_HZ;
                ret_val = 0;
                break;
            default:
                *scale_factor = 0.0f;
                break;
        }
    }

    return ret_val;
}

