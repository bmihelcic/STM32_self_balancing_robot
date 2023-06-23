/**
 ******************************************************************************
 * File Name          : mpu6050.h
 * Description        : MPU6050 accelerometer and gyroscope header file
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
 ******************************************************************************
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "stm32f1xx_hal.h"
#include "stdint.h"
#include "sbr_log.h"

#define MPU6050_ADDR  (0x68 << 1)
/**
 * @brief chip information definition
 */
#define CHIP_NAME                 "TDK MPU6050"        /**< chip name */
#define MANUFACTURER_NAME         "TDK"                /**< manufacturer name */
#define SUPPLY_VOLTAGE_MIN        2.375f               /**< chip min supply voltage */
#define SUPPLY_VOLTAGE_MAX        3.46f                /**< chip max supply voltage */
#define MAX_CURRENT               3.9f                 /**< chip max current */
#define TEMPERATURE_MIN           -40.0f               /**< chip min operating temperature */
#define TEMPERATURE_MAX           85.0f                /**< chip max operating temperature */
#define DRIVER_VERSION            1000                 /**< driver version */

/**
 * @brief chip register definition
 */
#define MPU6050_REG_SELF_TEST_X         0x0D        /**< self test x register */
#define MPU6050_REG_SELF_TEST_Y         0x0E        /**< self test y register */
#define MPU6050_REG_SELF_TEST_Z         0x0F        /**< self test z register */
#define MPU6050_REG_SELF_TEST_A         0x10        /**< self test a register */
#define MPU6050_REG_SMPRT_DIV           0x19        /**< smprt div register */
#define MPU6050_REG_CONFIG              0x1A        /**< configure register */
#define MPU6050_REG_GYRO_CONFIG         0x1B        /**< gyro configure register */
#define MPU6050_REG_ACCEL_CONFIG        0x1C        /**< accel configure register */
#define MPU6050_REG_MOTION_THRESHOLD    0x1F        /**< motion threshold register */
#define MPU6050_REG_MOTION_DURATION     0x20        /**< motion duration register */
#define MPU6050_REG_FIFO_EN             0x23        /**< fifo enable register */
#define MPU6050_REG_I2C_MST_CTRL        0x24        /**< i2c master ctrl register */
#define MPU6050_REG_I2C_MST_STATUS      0x36        /**< i2c master status register */
#define MPU6050_REG_I2C_MST_DELAY_CTRL  0x67        /**< i2c master delay ctrl register */
#define MPU6050_REG_I2C_SLV0_ADDR       0x25        /**< iic slave0 address register */
#define MPU6050_REG_I2C_SLV0_REG        0x26        /**< iic slave0 reg register */
#define MPU6050_REG_I2C_SLV0_CTRL       0x27        /**< iic slave0 ctrl register */
#define MPU6050_REG_I2C_SLV0_DO         0x63        /**< iic slave0 do register */
#define MPU6050_REG_I2C_SLV1_ADDR       0x28        /**< iic slave1 address register */
#define MPU6050_REG_I2C_SLV1_REG        0x29        /**< iic slave1 reg register */
#define MPU6050_REG_I2C_SLV1_CTRL       0x2A        /**< iic slave1 ctrl register */
#define MPU6050_REG_I2C_SLV1_DO         0x64        /**< iic slave1 do register */
#define MPU6050_REG_I2C_SLV2_ADDR       0x2B        /**< iic slave2 address register */
#define MPU6050_REG_I2C_SLV2_REG        0x2C        /**< iic slave2 reg register */
#define MPU6050_REG_I2C_SLV2_CTRL       0x2D        /**< iic slave2 ctrl register */
#define MPU6050_REG_I2C_SLV2_DO         0x65        /**< iic slave2 do register */
#define MPU6050_REG_I2C_SLV3_ADDR       0x2E        /**< iic slave3 address register */
#define MPU6050_REG_I2C_SLV3_REG        0x2F        /**< iic slave3 reg register */
#define MPU6050_REG_I2C_SLV3_CTRL       0x30        /**< iic slave3 ctrl register */
#define MPU6050_REG_I2C_SLV3_DO         0x66        /**< iic slave3 do register */
#define MPU6050_REG_I2C_SLV4_ADDR       0x31        /**< iic slave4 address register */
#define MPU6050_REG_I2C_SLV4_REG        0x32        /**< iic slave4 reg register */
#define MPU6050_REG_I2C_SLV4_CTRL       0x34        /**< iic slave4 ctrl register */
#define MPU6050_REG_I2C_SLV4_DO         0x33        /**< iic slave4 do register */
#define MPU6050_REG_I2C_SLV4_DI         0x35        /**< iic slave4 di register */
#define MPU6050_REG_EXT_SENS_DATA_00    0x49        /**< extern sensor data 00 register */
#define MPU6050_REG_INT_PIN_CFG         0x37        /**< interrupt pin configure register */
#define MPU6050_REG_INT_ENABLE          0x38        /**< interrupt enable register */
#define MPU6050_REG_INT_STATUS          0x3A        /**< interrupt status register */
#define MPU6050_REG_ACCEL_XOUT_H        0x3B        /**< accel xout high register */
#define MPU6050_REG_ACCEL_XOUT_L        0x3C        /**< accel xout low register */
#define MPU6050_REG_ACCEL_YOUT_H        0x3D        /**< accel yout high register */
#define MPU6050_REG_ACCEL_YOUT_L        0x3E        /**< accel yout low register */
#define MPU6050_REG_ACCEL_ZOUT_H        0x3F        /**< accel zout high register */
#define MPU6050_REG_ACCEL_ZOUT_L        0x40        /**< accel zout low register */
#define MPU6050_REG_TEMP_OUT_H          0x41        /**< temp high register */
#define MPU6050_REG_TEMP_OUT_L          0x42        /**< temp low register */
#define MPU6050_REG_GYRO_XOUT_H         0x43        /**< gyro xout high register */
#define MPU6050_REG_GYRO_XOUT_L         0x44        /**< gyro xout low register */
#define MPU6050_REG_GYRO_YOUT_H         0x45        /**< gyro yout high register */
#define MPU6050_REG_GYRO_YOUT_L         0x46        /**< gyro yout low register */
#define MPU6050_REG_GYRO_ZOUT_H         0x47        /**< gyro zout high register */
#define MPU6050_REG_GYRO_ZOUT_L         0x48        /**< gyro zout low register */
#define MPU6050_REG_SIGNAL_PATH_RESET   0x68        /**< signal path reset register */
#define MPU6050_REG_USER_CTRL           0x6A        /**< user ctrl register */
#define MPU6050_REG_PWR_MGMT_1          0x6B        /**< power management 1 register */
#define MPU6050_REG_PWR_MGMT_2          0x6C        /**< power management 2 register */
#define MPU6050_REG_BANK_SEL            0x6D        /**< bank sel register */
#define MPU6050_REG_MEM                 0x6F        /**< memory register */
#define MPU6050_REG_PROGRAM_START       0x70        /**< program start register */
#define MPU6050_REG_FIFO_COUNTH         0x72        /**< fifo count high threshold register */
#define MPU6050_REG_FIFO_COUNTL         0x73        /**< fifo count low threshold register */
#define MPU6050_REG_R_W                 0x74        /**< fifo read write data register */
#define MPU6050_REG_WHO_AM_I            0x75        /**< who am I register */

/**
 * @brief mpu6050 accelerometer range enumeration definition
 */
typedef enum {
    MPU6050_ACCELEROMETER_RANGE_2G = 0x00, /**< ±2 g */
    MPU6050_ACCELEROMETER_RANGE_4G = 0x01, /**< ±4 g */
    MPU6050_ACCELEROMETER_RANGE_8G = 0x02, /**< ±8 g */
    MPU6050_ACCELEROMETER_RANGE_16G = 0x03, /**< ±16 g */
} mpu6050_accelerometer_range_t;
/**
 * @brief mpu6050 gyroscope range enumeration definition
 */
typedef enum {
    MPU6050_GYRO_RANGE_250DPS = 0x00, /**< ±250 dps */
    MPU6050_GYRO_RANGE_500DPS = 0x01, /**< ±500 dps */
    MPU6050_GYRO_RANGE_1000DPS = 0x02, /**< ±1000 dps */
    MPU6050_GYRO_RANGE_2000DPS = 0x03, /**< ±2000 dps */
} mpu6050_gyroscope_range_t;

typedef struct MPU6050_handle_STRUCT {
    I2C_HandleTypeDef *i2c_handle_ptr;
    int16_t accel_x;
    int16_t accel_z;
    int16_t gyro_y;
    int32_t gyro_offset_x;
    int32_t gyro_offset_y;
    float accel_angle;
    float gyro_angle;
    mpu6050_accelerometer_range_t accel_range;
    mpu6050_gyroscope_range_t gyro_range;
    float gyro_val_change_factor;
    uint8_t is_angle_critical;
    sbr_log_mpu6050_message_S tx_log_message;
} MPU6050_handle_S;

void StartMpu6050Task(void const *argument);

#endif /* INC_MPU6050_H_ */
