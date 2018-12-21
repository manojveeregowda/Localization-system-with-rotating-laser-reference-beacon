/*
 * beacon.h
 *
 *  Created on: Nov 27, 2018
 *      Author: sulicat
 */

#ifndef BEACON_H_
#define BEACON_H_

#include <stdint.h>
#include "homing_sensor/homing.h"
#include "motor_driver/stepper_motor.h"


uint32_t beacon_omega;
extern uint8_t GLOBAL_START_MOTOR;

void beacon_init();
void beacon_loop();
void beacon_set_omega();
void beacon_stop();     // NA
void beacon_start();    // NA
void beacon_reset();    // NA

void initHW_beacon();
inline void boot_driver( void );
void wait_microsecond( uint32_t us );





#endif /* BEACON_H_ */
