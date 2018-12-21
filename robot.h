/*
 * robot.h
 *
 *  Created on: Nov 29, 2018
 *      Author: sulicat
 */

#ifndef ROBOT_H_
#define ROBOT_H_

// HARDWARE TARGET:
//    Target Platform: EK-TM4C123GXL Evaluation Board
//    Target uC:       TM4C123GH6PM
//    System Clock:    40 MHz
//    PWM Divisor:     64
//    Motor Driver:    SN754410

// GPIO CONFIGURATION:
//    Red LED:                 PF1 (Internal to board.)
//    Green LED:               PF3 (Internal to board.)
//    Blue LED:                PF2 (Internal to board.)
//    SW1 Pushbutton:          PF4 (Internal to board.)
//    Stepper Motor A1 PWM:    PB5 Green wire
//    Stepper Motor A2 PWM:    PB6 Blue wire
//    Stepper Motor B1 PWM:    PE4 Orange wire
//    Stepper Motor B2 PWM:    PE5 White wire
//    XBee RF TX/RX:           PB0
//    XBee RF TX/RX:           PB1

#include <stdint.h>
#include <stdbool.h>
#include "TM4C123GH6PM.h"

void wait_microsecond( uint32_t us );

// Hardware bit-band addresses.
#define RED_LED     ( *( ( volatile uint32_t* )( 0x42000000 + ( 0x400253FC - 0x40000000 ) * 32 + 1 * 4 ) ) )
#define GREEN_LED   ( *( ( volatile uint32_t* )( 0x42000000 + ( 0x400253FC - 0x40000000 ) * 32 + 3 * 4 ) ) )
#define BLUE_LED    ( *( ( volatile uint32_t* )( 0x42000000 + ( 0x400253FC - 0x40000000 ) * 32 + 2 * 4 ) ) )
#define PUSH_BUTTON ( *( ( volatile uint32_t *)( 0x42000000 + ( 0x400253FC - 0x40000000 ) * 32 + 4 * 4 ) ) )

// Hardware PWM comparator addresses.

#define MOTOR_PIN_1 PWM0_1_CMPB_R // Green wire  - PB5
#define MOTOR_PIN_2 PWM0_0_CMPA_R // Blue wire   - PB6 (PD0)
#define MOTOR_PIN_3 PWM0_2_CMPA_R // Orange wire - PE4
#define MOTOR_PIN_4 PWM0_2_CMPB_R // White wire  - PE5

// Discretes.
#define MAX_SPEED        1023   // Duty cycle numerator out of 1023.
// Duty cycle for robot #1.
//    #define LEFT_REG_SPEED  512    // Duty cycle numerator out of 1023.
//    #define RIGHT_REG_SPEED 650    // Duty cycle numerator out of 1023.
// Duty cycle for robot #2.
//    #define LEFT_REG_SPEED  750     // Duty cycle numerator out of 1023.
//    #define RIGHT_REG_SPEED 650     // Duty cycle numerator out of 1023.


// test
#define LEFT_REG_SPEED  800     // Duty cycle numerator out of 1023.
#define RIGHT_REG_SPEED 1023     // Duty cycle numerator out of 1023.

#define ON       1
#define OFF      0
#define FORWARD  1
#define REVERSE  2
#define STOP     3

// Globals.
uint8_t leftStatus;
uint8_t rightStatus;

// ----------------------------------------------------------------------------
void initHW_robot();
// Alert of the boot sequence.
inline void boot_driver( void );
// Alert of button pushed.
inline void reset_button( void );
// Blocking function for SW1.
void wait_pressed( void );
void wait_microsecond( uint32_t us );
// Left motor control functions.
void leftForward (uint16_t speed);
void leftReverse (uint16_t speed);
void leftStop (void);
// Right motor control functions.
void rightForward (uint16_t speed);
void rightReverse (uint16_t speed);
void rightStop (void);
void motorControl (uint8_t left, uint8_t right);



#endif /* ROBOT_H_ */
