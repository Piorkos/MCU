/*
 * print_to_uart.c
 *
 *  Created on: Mar 12, 2021
 *      Author: dominikpiorkowski
 */


#ifndef __PRINTF_TO_UART_H__
#define __PRINTF_TO_UART_H__

#include "usart.h"

int __io_putchar(char ch)
{
	HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 1000);
	return ch;
}

#endif /*__PRINTF_TO_UART_H__*/

