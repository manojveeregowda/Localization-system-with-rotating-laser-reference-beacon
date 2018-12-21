/*
 * xbee.c
 *
 *  Created on: Nov 7, 2018
 *      Author: sulicat
 */

#include <stdint.h>
#include "TM4C123GH6PM.h"
#include "globalDefines.h"
#include "robot.h"

char RF_INPUT[80];
int rf_input_ptr = 0;
uint8_t RF_ARGS[MAX_NUM_OF_INPUT_ARGS];
uint8_t RF_INPUT_WRITE_POINT = 0;
uint8_t RF_INPUT_ARG_COUNT = 0;

int dev_1_omega = 100;
int dev_2_omega = 100;
int dev_3_omega = 100;
int started = 0;

void initHW_rf(){
    // init to 115200 8N1
    // receive interrupt would be nice
    // RX = B0
    // TX = B1

}

void uart1_Isr(){
    char c = UART1_DR_R & 0xFF;
    //putcUart0(c);
    parseInputRFChar(c);
}

void parseInputRFChar(char c){
    if( RF_INPUT_WRITE_POINT >= MAX_INPUT_LENGTH - 2 ){
        put_s_UART0("[ERROR] RF input overflow\n\r");
        RF_INPUT[ RF_INPUT_WRITE_POINT + 1 ] = '\0';

    // if uppercase letter || symbol || number
    }else if( c >= ' ' && c <= 'Z'  ){
        RF_INPUT[ RF_INPUT_WRITE_POINT ] = c;
        RF_INPUT_WRITE_POINT += 1;
        putcUart0( c );

    // if lower case letter -> convert to upper case then add
    }else if( c >= 'a' && c <= 'z' ){
        c = c - 32;
        RF_INPUT[ RF_INPUT_WRITE_POINT ] = c;
        RF_INPUT_WRITE_POINT += 1;
        putcUart0( c );

    // deal with backspace
    }else if( c == 8 && RF_INPUT_WRITE_POINT > 0 ){
        RF_INPUT_WRITE_POINT -= 1;
        putcUart0( c );

    // carriege return
    }else if( c == 13 ){
        RF_INPUT[ RF_INPUT_WRITE_POINT ] = '\0';
        RF_INPUT_WRITE_POINT = 0;
        put_s_UART0( "\n\r" );
        runRFInput();
    }
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart1( char c ){
    while  (UART1_FR_R & UART_FR_TXFF );
    UART1_DR_R = c;
}
// Blocking function that writes a string when the UART buffer is not full
void put_s_UART1(char* str){
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
      putcUart1(str[i]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart1(){
    while (UART1_FR_R & UART_FR_RXFE);
    return UART1_DR_R & 0xFF;
}


void runRFInput(){
    int i = 1;
    int RF_INPUT_ARG_COUNT = 0;
    if( strlen( RF_INPUT ) >= 1 ){
        RF_INPUT_ARG_COUNT = 1;
        RF_ARGS[0] = 0;

        while( RF_INPUT[i] != '\0' ){
            // if the character is a delimiter  IE: anything not a letter/number or period
            if( ! ((RF_INPUT[i]  >= 'A' && RF_INPUT[i] <= 'Z') || (RF_INPUT[i] >= '0' && RF_INPUT[i] <= '9') || RF_INPUT[i] == '.' || RF_INPUT[i] == '-' || RF_INPUT[i] == '&') ){
                RF_INPUT[i] = '\0';

            }else if( RF_INPUT[ i - 1 ] == '\0' ){
                if( RF_INPUT_ARG_COUNT < MAX_NUM_OF_INPUT_ARGS ){
                    RF_ARGS[ RF_INPUT_ARG_COUNT ] = i;
                    RF_INPUT_ARG_COUNT += 1;
                }
            }
            i++;
        }
    }else{
        put_s_UART0( "Please Enter a valid String \n\r");
    }

    // check inputs
    uint8_t error = 0;


    if( RF_INPUT_ARG_COUNT == 1){
        if( strcmp(RF_INPUT, "START") == 0 ){
            TIMER1_CTL_R |= TIMER_CTL_TAEN;
            started = 1;
        }

    }else if( RF_INPUT_ARG_COUNT == 2){
        if( strcmp(RF_INPUT, "ROBOT") == 0 && strcmp(RF_INPUT + RF_ARGS[1], "FORWARD") == 0){
            motorControl(FORWARD,FORWARD);

        }else if( strcmp(RF_INPUT, "ROBOT") == 0 && strcmp(RF_INPUT + RF_ARGS[1], "BACKWARD") == 0 ){
            motorControl(REVERSE,REVERSE);

        }else if( strcmp(RF_INPUT, "ROBOT") == 0 && strcmp(RF_INPUT + RF_ARGS[1], "RIGHT") == 0 ){
            motorControl(FORWARD,REVERSE);

        }else if( strcmp(RF_INPUT, "ROBOT") == 0 && strcmp(RF_INPUT + RF_ARGS[1], "LEFT") == 0 ){
            motorControl(REVERSE,FORWARD);

        }else if( strcmp(RF_INPUT, "ROBOT") == 0 && strcmp(RF_INPUT + RF_ARGS[1], "STOP") == 0 ){
            motorControl(STOP, STOP);

        }else if( strcmp(RF_INPUT, "ROBOT") == 0 && strcmp(RF_INPUT + RF_ARGS[1], "WIGGLE") == 0 ){
            motorControl(REVERSE,FORWARD);
            waitMicrosecond(2000000);
            motorControl(FORWARD,REVERSE);
            waitMicrosecond(500000);
            motorControl(FORWARD, FORWARD);
            waitMicrosecond(2000000);
            motorControl(STOP, STOP);
        }

    }else if( RF_INPUT_ARG_COUNT == 4){
        if( strcmp(RF_INPUT, "SET") == 0 && strcmp(RF_INPUT + RF_ARGS[1], "OMEGA") == 0 ){

            if( (RF_INPUT + RF_ARGS[3])[0] == '1' ){
                dev_1_omega = atoi(RF_INPUT + RF_ARGS[2]);

            }else if( (RF_INPUT + RF_ARGS[3])[0] == '2' ){
                dev_2_omega = atoi(RF_INPUT + RF_ARGS[2]);

            }else if( (RF_INPUT + RF_ARGS[3])[0] == '3' ){
                dev_3_omega = atoi(RF_INPUT + RF_ARGS[2]);
            }

        }
    }

    BLUE_LED = 1;
    waitMicrosecond(50000);
    BLUE_LED = 0;
    waitMicrosecond(50000);

    // error parsing
    if(error){
        put_s_UART0( "Could not parse input\n\r");
    }
}


