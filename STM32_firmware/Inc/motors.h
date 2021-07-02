/**
  ******************************************************************************
  * File Name          : motors.h
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


#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

#include <stdint.h>
#include "main.h"
#include "stm32f1xx_hal.h"

void MOTORS_turnClockwise(TIM_HandleTypeDef*, uint16_t);
void MOTORS_turnCounterClockwise(TIM_HandleTypeDef*, uint16_t);
void MOTORS_powerOff(TIM_HandleTypeDef*);

#endif /* INC_MOTORS_H_ */
