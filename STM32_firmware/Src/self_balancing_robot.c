/*
 * self_balancing_robot.c
 *
 *  Created on: Jul 9, 2021
 *      Author: branimir
 */

#include "main.h"
#include "conf.h"
#include "self_balancing_robot.h"

void SELF_BALANCING_ROBOT_handler(SELF_BALANCING_ROBOT_handle_S *handlePtr) {
    // Read accelerometer and gyroscope values
    MPU6050_Handler(&handlePtr->mpu6050Handle);
    handlePtr->robot_angle = handlePtr->mpu6050Handle.gyro_angle;
    handlePtr->robot_crashed = handlePtr->mpu6050Handle.isAngleCritical;
    if(TRUE == handlePtr->robot_crashed) {
        handlePtr->motorsHandle.motorsEnabled = FALSE;
    }
    // Run PID control handler
    PID_controlHandler(&handlePtr->pidHandle, handlePtr->req_robot_angle, handlePtr->robot_angle);
    handlePtr->update_pid = FALSE;

    if ((TRUE != handlePtr->robot_crashed) &&
       (TRUE == handlePtr->motorsHandle.motorsEnabled))
    {
        if (handlePtr->robot_angle > (handlePtr->req_robot_angle + SET_POINT_DEVIATION)) {
            MOTORS_turnClockwise(&handlePtr->motorsHandle, (uint16_t)handlePtr->pidHandle.pid_total);
        } else if (handlePtr->robot_angle < (handlePtr->req_robot_angle - SET_POINT_DEVIATION)) {
            MOTORS_turnCounterClockwise(&handlePtr->motorsHandle, (uint16_t)handlePtr->pidHandle.pid_total);
        } else {
            MOTORS_powerOff(&handlePtr->motorsHandle);
        }
    } else {
        MOTORS_powerOff(&handlePtr->motorsHandle);
        handlePtr->pidHandle.pid_total = 0.0f;
    }
}
