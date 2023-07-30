/*
 * app_cfg.h
 *
 *  Created on: Jun 22, 2023
 *      Author: brane
 */

#ifndef APP_CONFIG_APP_CFG_H_
#define APP_CONFIG_APP_CFG_H_

#define COMMAND_SHUTDOWN            (0x2E)
#define COMMAND_DO_NOTHING          (0x00)
#define COMMAND_RAISE_P             (0x30)
#define COMMAND_RAISE_I             (0x31)
#define COMMAND_RAISE_D             (0x32)
#define COMMAND_RAISE_P10           (0x33)
#define COMMAND_RAISE_I1            (0x49)
#define COMMAND_RAISE_D10           (0x34)
#define COMMAND_LOWER_P             (0x35)
#define COMMAND_LOWER_I             (0x36)
#define COMMAND_LOWER_D             (0x37)
#define COMMAND_LOWER_P10           (0x38)
#define COMMAND_LOWER_I1            (0x69)
#define COMMAND_LOWER_D10           (0x39)
#define COMMAND_ENABLE_MOTORS       (0x0E)
#define COMMAND_SET_POINT_PLUS      (0x51)
#define COMMAND_SET_POINT_MINUS     (0x52)


#define SET_POINT_DEVIATION         (0.0f)

#define IMPORTANT_DATA_FREQ         (40)      // Send important data every 40 milliseconds
#define NON_IMPORTANT_DATA_FREQ     (1000)    // Send non important data every 1 second
#define PID_UPDATE_FREQ             (10)      // Update pid variables every 10ms

#define INITIAL_UPRIGHT_ROBOT_ANGLE (84.7f)   // Robot angle at which it should stand upright
#define NV_MEMORY_ADDRESS           (0x800F000)


#define IMPORTANT_DATA      (0u)
#define NON_IMPORTANT_DATA  (1u)

#define TRUE    (1u)
#define FALSE   (0u)


#define CONFIG_MAIN_LOG_LEVEL     2    // 0 - all debug off, 1 - important debug on, 2 - all debug on

#endif /* APP_CONFIG_APP_CFG_H_ */
