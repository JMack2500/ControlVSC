 /*
 * UART.h
 *
 *  Created on: Apr 17, 2024
 *      Author: baseb
 */

#ifndef INC_UART_H_
#define INC_UART_H_

void LPUART_GPIO_Init(void);
void UART_Init(void);
void UART_Write(char data[]);
uint8_t UART_Read(void);

#endif /* INC_UART_H_ */
