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
#include "app_cfg.h"
#include "pid_control.h"

extern I2C_HandleTypeDef hi2c1;
static imu_handle_S imu_handle;

void imu_init();
void imu_proccess_sensor_data();
int imu_calc_gyro_scale_factor(float *scale_factor);
static void imu_send_angle_to_pid_control();
static void imu_update_log_message();

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
            imu_send_angle_to_pid_control();
            imu_update_log_message();
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

static void imu_send_angle_to_pid_control()
{
    pid_rx_message_t pid_message;

    pid_message.id = IMU_ID;
    pid_message.data.imu_robot_angle = imu_handle.robot_angle;

    if (pdTRUE == xSemaphoreTake(pid_message_buffer_mutex,
                                 10)) {
        xMessageBufferSend(pid_rx_message_buffer_handle,
                           &pid_message,
                           sizeof(pid_message),
                           20);
        xSemaphoreGive(pid_message_buffer_mutex);
    }
}

static void imu_update_log_message()
{
    log_rx_message_t log_message;

    log_message.id = IMU_ID;
    log_message.data.imu.imu_accel_angle = imu_handle.accel_angle;
    log_message.data.imu.imu_gyro_angle = imu_handle.gyro_angle;

    if (pdTRUE == xSemaphoreTake(log_message_buffer_mutex,
                                 10)) {
        xMessageBufferSend(log_rx_message_buffer_handle,
                           &log_message,
                           sizeof(log_message),
                           10);
        xSemaphoreGive(log_message_buffer_mutex);
    }
}
