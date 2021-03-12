/*
 * printf_to_uart.cpp
 *
 *  Created on: Mar 10, 2021
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
