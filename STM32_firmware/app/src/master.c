/**
 ******************************************************************************
 * File Name          : sbr_master.c
 * Description        : Self Balancing Robot master source file
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
#include "cmsis_os.h"

extern osMessageQId pid_message_queue_id;

/**
 * @brief Function implementing the master thread.
 * @param argument: Not used
 * @retval None
 */
void MASTER_Thread(void const *argument)
{
    for (;;)
    {
        osDelay(1);
    }
}
