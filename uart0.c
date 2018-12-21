/*
 * uart0.c
 *
 *  Created on: Nov 19, 2018
 *      Author: sulicat
 */

#include <stdint.h>
#include "TM4C123GH6PM.h"
#include "globalDefines.h"

char INPUT[80];
int input_ptr = 0;
uint8_t ARGS[MAX_NUM_OF_INPUT_ARGS];
uint8_t INPUT_WRITE_POINT = 0;
uint8_t CURRENT_INPUT_ARG_COUNT = 0;

void initHW_uart0(){
    SYSCTL_RCGC1_R  |= SYSCTL_RCGC1_UART0;
    SYSCTL_RCGC2_R  |= SYSCTL_RCGC2_GPIOA;

    GPIO_PORTA_DEN_R    |= 0b11;
    GPIO_PORTA_AFSEL_R  |= 0b11;
    GPIO_PORTA_PCTL_R   |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;


    // Configure UART0 to  baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0;                                                // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                                 // use system clock (40 MHz)
    UART0_IBRD_R = 21;                                              // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                                              // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;                // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;    // enable TX, RX, and module
}


// Blocking function that returns with serial data once the buffer is not empty
char getcUart0(){
    while (UART0_FR_R & UART_FR_RXFE);
    return UART0_DR_R & 0xFF;
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c){
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = c;
    //putcUart1(c);
}

// Blocking function that writes a string when the UART buffer is not full
void put_s_UART0(char* str){
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
      putcUart0(str[i]);
}

/*************************************************************************************************************/
void run_input(){
    // asd
    int i = 1;
    int CURRENT_INPUT_ARG_COUNT = 0;
    if( strlen( INPUT ) > 1 ){
        CURRENT_INPUT_ARG_COUNT = 1;
        ARGS[0] = 0;

        while( INPUT[i] != '\0' ){
            // if the character is a delimiter  IE: anything not a letter/number or period
            if( ! ((INPUT[i]  >= 'A' && INPUT[i] <= 'Z') || (INPUT[i] >= '0' && INPUT[i] <= '9') || INPUT[i] == '.' || INPUT[i] == '-' || INPUT[i] == '&') ){
                INPUT[i] = '\0';

            }else if( INPUT[ i - 1 ] == '\0' ){
                if( CURRENT_INPUT_ARG_COUNT < MAX_NUM_OF_INPUT_ARGS ){
                    ARGS[ CURRENT_INPUT_ARG_COUNT ] = i;
                    CURRENT_INPUT_ARG_COUNT += 1;
                }
            }
            i++;
        }
    }else{
        put_s_UART0( "Please Enter a valid String \n\r");
    }

    // check inputs
    uint8_t error = 0;


    if( CURRENT_INPUT_ARG_COUNT == 1){
        if( strcmp(INPUT, "NODES") == 0 ){
            put_s_UART0( "[OK] Printing Discovered Nodes... \n\r");
            put_s_UART0( "[Done] \n\r");

        }else if( strcmp(INPUT, "SYNC") == 0 ){
            put_s_UART0( "[OK] Syncing ... \n\r");
            put_s_UART0( "[Done] \n\r");
        }

    }else if( CURRENT_INPUT_ARG_COUNT == 4 ){
        if( strcmp(INPUT, "SET") == 0 && strcmp(INPUT + ARGS[1], "OMEGA") == 0 && strcmp(INPUT + ARGS[3], "0") == 0 ){
            put_s_UART0( "[OK] Setting Omega for id=1 to: ");
            put_s_UART0( INPUT+ARGS[2] );
            put_s_UART0( "\n\r");
            put_s_UART1( "SET OMEGA ");
            put_s_UART1( INPUT+ARGS[2] );

        }
    }

    // error parsing
    if(error){
        put_s_UART0( "Could not parse input\n\r");
    }
}


void parseInputChar(char c){
    if( INPUT_WRITE_POINT >= MAX_INPUT_LENGTH - 2 ){
        put_s_UART0("Your Input was too long. Will parse it as it is.\n\r");
        INPUT[ INPUT_WRITE_POINT + 1 ] = '\0';

    // if uppercase letter || symbol || number
    }else if( c >= ' ' && c <= 'Z'  ){
        INPUT[ INPUT_WRITE_POINT ] = c;
        INPUT_WRITE_POINT += 1;
        putcUart0( c );

    // if lower case letter -> convert to upper case then add
    }else if( c >= 'a' && c <= 'z' ){
        c = c - 32;
        INPUT[ INPUT_WRITE_POINT ] = c;
        INPUT_WRITE_POINT += 1;
        putcUart0( c );

    // deal with backspace
    }else if( c == 8 && INPUT_WRITE_POINT > 0 ){
        INPUT_WRITE_POINT -= 1;
        putcUart0( c );

    // carriege return
    }else if( c == 13 ){
        INPUT[INPUT_WRITE_POINT] = '\0';
        INPUT_WRITE_POINT = 0;
        put_s_UART0( "\n\r" );
        run_input();
    }
}







