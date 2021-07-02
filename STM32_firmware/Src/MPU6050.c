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

/*
 * @brief Sets value to MPU6050 register
 * @param hi2c: Pointer to a I2C_HandleTypeDef structure that contains
 *                 the configuration information for the specified I2C.
 * @param reg:  register to write into
 * @param data: value to write in
 *
 */
void MPU6050_setReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t data) {
    uint8_t i2c_buff[2] = { reg, data };
    HAL_I2C_Master_Transmit(hi2c, MPU6050_ADDR, i2c_buff, 2, 25);
}

/*
 * @brief Get accelerometer x axis value
 * @param hi2c: Pointer to a I2C_HandleTypeDef structure that contains
 *                 the configuration information for the specified I2C.
 * @return: x axis accelerometer value
 */
int16_t MPU6050_getAccel_X(I2C_HandleTypeDef *hi2c) {
    uint8_t reg[1] = { 0x3B };
    uint8_t accel_x_buff[2];
    HAL_I2C_Master_Transmit(hi2c, MPU6050_ADDR, reg, 1, 25);
    HAL_I2C_Master_Receive(hi2c, MPU6050_ADDR, accel_x_buff, 2, 25);
    return ((accel_x_buff[0] << 8) | accel_x_buff[1]);
}

/*
 * @brief Get accelerometer y axis value
 * @param hi2c: Pointer to a I2C_HandleTypeDef structure that contains
 *                 the configuration information for the specified I2C.
 * @return: y axis accelerometer value
 */
int16_t MPU6050_getAccel_Y(I2C_HandleTypeDef *hi2c) {
    uint8_t reg[1] = { 0x3D };
    uint8_t accel_y_buff[2];
    HAL_I2C_Master_Transmit(hi2c, MPU6050_ADDR, reg, 1, 25);
    HAL_I2C_Master_Receive(hi2c, MPU6050_ADDR, accel_y_buff, 2, 25);
    return ((accel_y_buff[0] << 8) | accel_y_buff[1]);
}

/*
 * @brief Get accelerometer z axis value
 * @param hi2c: Pointer to a I2C_HandleTypeDef structure that contains
 *                 the configuration information for the specified I2C.
 * @return: z axis accelerometer value
 */
int16_t MPU6050_getAccel_Z(I2C_HandleTypeDef *hi2c) {
    uint8_t reg[1] = { 0x3F };
    uint8_t accel_z_buff[2];
    HAL_I2C_Master_Transmit(hi2c, MPU6050_ADDR, reg, 1, 25);
    HAL_I2C_Master_Receive(hi2c, MPU6050_ADDR, accel_z_buff, 2, 25);
    return ((accel_z_buff[0] << 8) | accel_z_buff[1]);
}

/*
 * @brief Get gyro x axis value
 * @param hi2c: Pointer to a I2C_HandleTypeDef structure that contains
 *                 the configuration information for the specified I2C.
 * @return: x axis gyro value
 */
int16_t MPU6050_getGyro_X(I2C_HandleTypeDef *hi2c) {
    uint8_t reg[1] = { 0x43 };
    uint8_t gyro_x_buff[2];
    HAL_I2C_Master_Transmit(hi2c, MPU6050_ADDR, reg, 1, 25);
    HAL_I2C_Master_Receive(hi2c, MPU6050_ADDR, gyro_x_buff, 2, 25);
    return ((gyro_x_buff[0] << 8) | gyro_x_buff[1]);
}

/*
 * @brief Get gyro y axis value
 * @param hi2c: Pointer to a I2C_HandleTypeDef structure that contains
 *                 the configuration information for the specified I2C.
 * @return: y axis gyro value
 */
int16_t MPU6050_getGyro_Y(I2C_HandleTypeDef *hi2c) {
    uint8_t reg[1] = { 0x45 };
    uint8_t gyro_y_buff[2];
    HAL_I2C_Master_Transmit(hi2c, MPU6050_ADDR, reg, 1, 25);
    HAL_I2C_Master_Receive(hi2c, MPU6050_ADDR, gyro_y_buff, 2, 25);
    return ((gyro_y_buff[0] << 8) | gyro_y_buff[1]);
}

/*
 * @brief Get gyro z axis value
 * @param hi2c: Pointer to a I2C_HandleTypeDef structure that contains
 *                 the configuration information for the specified I2C.
 * @return: z axis gyro value
 */
int16_t MPU6050_getGyro_Z(I2C_HandleTypeDef *hi2c) {
    uint8_t reg[1] = { 0x47 };
    uint8_t gyro_z_buff[2];
    HAL_I2C_Master_Transmit(hi2c, MPU6050_ADDR, reg, 1, 25);
    HAL_I2C_Master_Receive(hi2c, MPU6050_ADDR, gyro_z_buff, 2, 25);
    return ((gyro_z_buff[0] << 8) | gyro_z_buff[1]);
}
