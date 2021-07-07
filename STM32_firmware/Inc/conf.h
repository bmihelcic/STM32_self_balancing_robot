/*
 * conf.h
 *
 *  Created on: Jun 26, 2021
 *      Author: brane
 */

#ifndef INC_CONF_H_
#define INC_CONF_H_

#define SET_POINT_DEVIATION         (0.0f)

#define IMPORTANT_DATA_FREQ         (40)      // Send important data every 40 milliseconds
#define NON_IMPORTANT_DATA_FREQ     (1000)    // Send non important data every 1 second
#define PID_UPDATE_FREQ             (10)      // Update pid variables every 10ms

#define INITIAL_UPRIGHT_ROBOT_ANGLE (84.7f)   // Robot angle at which it should stand upright
#define NV_MEMORY_ADDRESS           (0x800F000)

#define COMMAND_SHUTDOWN            (0x2E)
#define COMMAND_CLEAR               (0x00)
#define COMMAND_RAISE_P             (0x30)
#define COMMAND_RAISE_I             (0x31)
#define COMMAND_RAISE_D             (0x32)
#define COMMAND_RAISE_P10           (0x33)
#define COMMAND_RAISE_D10           (0x34)
#define COMMAND_LOWER_P             (0x35)
#define COMMAND_LOWER_I             (0x36)
#define COMMAND_LOWER_D             (0x37)
#define COMMAND_LOWER_P10           (0x38)
#define COMMAND_LOWER_D10           (0x39)
#define COMMAND_ENABLE_MOTORS       (0x0E)
#define COMMAND_SET_POINT_PLUS      (0x51)
#define COMMAND_SET_POINT_MINUS     (0x52)

#endif /* INC_CONF_H_ */
