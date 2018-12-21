/*
 * xbee.h
 *
 *  Created on: Nov 19, 2018
 *      Author: sulicat
 */

#ifndef XBEE_H_
#define XBEE_H_

void initHW_rf();
void uart1_Isr();
void putcUart1( char c );
void put_s_UART1(char* str);
char getcUart1();
void parseInputRFChar(char c);
void runRFInput();


#endif /* XBEE_H_ */
