/**
 ******************************************************************************
 * File Name          : sbr_imu.c
 * Description        : Self Balancing Robot Inertial Measurement Unit header file
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
#ifndef APP_INC_IMU_H_
#define APP_INC_IMU_H_

#include "stdint.h"

typedef struct
{
    int32_t gyro_offset_x;
    int32_t gyro_offset_y;
    float accel_angle;
    float gyro_angle;
    float robot_angle;
    float gyro_val_change_factor;
    uint8_t is_angle_critical;
    uint8_t is_initialized;
} imu_handle_S;

void IMU_Thread(void const *argument);
float IMU_Get_Gyro_Angle();
float IMU_Get_Accel_Angle();
uint8_t IMU_Is_Angle_Critical();

#endif /* APP_INC_IMU_H_ */
