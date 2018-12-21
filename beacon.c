/*
 * beacon.h
 *
 *  Created on: Nov 27, 2018
 *      Author: sulicat
 */

#include "beacon.h"
#include "globalDefines.h"
#include "homing_sensor/homing.h"
#include "motor_driver/stepper_motor.h"


extern uint32_t beacon_omega;

void beacon_init(){
    beacon_omega = 100000;
}

void beacon_loop(){
    RED_LED = 1;
    waitMicrosecond(beacon_omega);
    RED_LED = 0;
    waitMicrosecond(beacon_omega);
}

void beacon_set_omega( uint32_t new ){
    beacon_omega = new;
    set_motor_speed( (uint16_t)new );
}

/*void SET_OMEGA( uint32_t new_omega ){
    beacon_omega = new_omega;
}*/




void initHW_beacon(){
    stepper_motor_init_hw();
    homing_sensor_init_hw();
}

inline void boot_driver( void ){
    // Alert of the boot sequence
    BLUE_LED = 1;
    wait_microsecond( 500000 );
    BLUE_LED = 0;
    wait_microsecond( 500000 );
    BLUE_LED = 1;
    wait_microsecond( 500000 );
    BLUE_LED = 0;

    // Boot up the motor
    boot_motor_driver();
}

void wait_microsecond( uint32_t us ){
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             NOP");                  // 1
    __asm("             B    WMS_LOOP0");       // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}
