/**
 ******************************************************************************
 * @file    stm32f1xx_isr.h
 * @brief   This file contains the headers of the interrupt handlers.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
#ifndef __STM32F1xx_ISR_H
#define __STM32F1xx_ISR_H

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void DebugMon_Handler(void);
void TIM1_UP_IRQHandler(void);
void USART1_IRQHandler(void);

#endif /* __STM32F1xx_ISR_H */
