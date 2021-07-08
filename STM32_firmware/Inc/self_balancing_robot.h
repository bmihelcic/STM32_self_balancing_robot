/*
 * self_balancing_robot.h
 *
 *  Created on: Jul 7, 2021
 *      Author: branimir
 */

#ifndef SRC_SELF_BALANCING_ROBOT_H_
#define SRC_SELF_BALANCING_ROBOT_H_

#include "motors.h"
#include "pid_control.h"
#include "MPU6050.h"

typedef struct selfBalancingRobot_STRUCT {
    volatile uint8_t robot_shutdown;
    volatile uint8_t update_pid;
    volatile uint8_t send_important_data;
    volatile uint8_t send_non_important_data;
    float robot_angle;
    float req_robot_angle;
    uint8_t robot_crashed;
    pidControlHandle_S pidHandle;
    MPU6050_handle_S mpu6050Handle;
    MOTORS_handle_S motorsHandle;
}selfBalancingRobot_S;

#endif /* SRC_SELF_BALANCING_ROBOT_H_ */
