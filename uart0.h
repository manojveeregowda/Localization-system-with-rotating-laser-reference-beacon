/*
 * uart0.h
 *
 *  Created on: Nov 19, 2018
 *      Author: sulicat
 */

#ifndef UART0_H_
#define UART0_H_


void initHW_uart0();
char getcUart0();
void putcUart0(char c);
void put_s_UART0(char* str);
void run_input();


#endif /* UART0_H_ */
