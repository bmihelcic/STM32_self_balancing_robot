/**
 ******************************************************************************
 * File Name          : motor_control.h
 * Description        : Module which handles motor control.
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

#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_

#include "stdint.h"

void MOTORS_Init();
void MOTORS_Turn_Clockwise(uint16_t v);
void MOTORS_Turn_Counter_Clockwise(uint16_t v);
void MOTORS_Power_Off();

#endif /* INC_MOTOR_CONTROL_H_ */
