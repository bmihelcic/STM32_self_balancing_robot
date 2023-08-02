/*
 * app_cfg.h
 *
 *  Created on: Jun 22, 2023
 *      Author: Branimir Mihelčić
 */

#ifndef APP_CONFIG_APP_CFG_H_
#define APP_CONFIG_APP_CFG_H_

#define COMMAND_DO_NOTHING                         (0x00)
#define COMMAND_MASTER_START_STOP                  (0xA5)
#define COMMAND_MASTER_ANGLE_SET_POINT_PLUS        (0x43)
#define COMMAND_MASTER_ANGLE_SET_POINT_MINUS       (0x44)
#define COMMAND_PID_RAISE_P                        (0x30)
#define COMMAND_PID_RAISE_I                        (0x31)
#define COMMAND_PID_RAISE_D                        (0x32)
#define COMMAND_PID_RAISE_P10                      (0x33)
#define COMMAND_PID_RAISE_I1                       (0x34)
#define COMMAND_PID_RAISE_D10                      (0x35)
#define COMMAND_PID_LOWER_P                        (0x36)
#define COMMAND_PID_LOWER_I                        (0x37)
#define COMMAND_PID_LOWER_D                        (0x38)
#define COMMAND_PID_LOWER_P10                      (0x39)
#define COMMAND_PID_LOWER_I1                       (0x40)
#define COMMAND_PID_LOWER_D10                      (0x41)
#define COMMAND_MOTORS_TOGGLE                      (0x42)
#define COMMAND_LOG_ON_OFF                         (0x45)

#define SET_POINT_DEVIATION                 (0.0f)

#define CFG_INITIAL_UPRIGHT_ROBOT_ANGLE     (84.7f)      // Robot angle at which it should stand upright

/* MASTER configuration */
#define CFG_MASTER_FREQ_MS                  (10)         // Master module run frequency in ms

/* LOG configuration*/
#define CFG_LOG_FREQ_MS                     (10)         // Log module run frequency in ms

/* PID configuration */
#define CFG_PID_CONTROL_FREQ_MS             (5)          // Pid module run frequency in ms
#define CFG_PID_KP_HIGH_LIMIT               (1023.0f)
#define CFG_PID_KP_LOW_LIMIT                (-1023.0f)
#define CFG_PID_KI_HIGH_LIMIT               (1023.0f)
#define CFG_PID_KI_LOW_LIMIT                (-1023.0f)
#define CFG_PID_KD_HIGH_LIMIT               (1023.0f)
#define CFG_PID_KD_LOW_LIMIT                (-1023.0f)

#define CFG_UART_TX_BUFFER_SIZE             (100)       // Size in bytes of uart tx buffer

typedef enum
{
    MASTER_ID = 0u,
    COMMAND_ID,
    PID_ID,
    LOG_ID,
    IMU_ID,
} module_id_t;

#endif /* APP_CONFIG_APP_CFG_H_ */
