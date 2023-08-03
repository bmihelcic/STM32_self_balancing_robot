/*
 * sbr_master.h
 *
 *  Created on: Jun 22, 2023
 *      Author: brane
 */

#ifndef APP_INC_MASTER_H_
#define APP_INC_MASTER_H_

#include "app_cfg.h"

typedef enum
{
    MASTER_OFF = 0u,
    MASTER_INIT,
    MASTER_RUN,
    MASTER_STOP,
    MASTER_ERROR,
} master_states_e;

typedef struct
{
    master_states_e master_state;
    uint8_t master_start_stop;
    uint8_t is_initialized;
    float robot_angle_set_point;
    float pid_total_value;
} master_handle_s;

typedef struct
{
    message_id_t id;
    union
    {
        uint8_t command;
        float pid_total;
    } data;

} master_rx_message_t;

void MASTER_Thread(void const *argument);

#endif /* APP_INC_MASTER_H_ */
