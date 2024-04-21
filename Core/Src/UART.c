/*
 * UART.c
 *
 *  Created on: Apr 17, 2024
 *      Author: baseb
 */

#include "stm32l4xx_hal.h"
#include <string.h>

void sendIntArray(int array[], int size) {	// UART helper function
  // Create a temporary character array to store the ASCII representation
  // Convert each integer to ASCII and store in the temporary array
  int index = 0;
  for (int i = 0; i < size; i++) {
         // Assuming maximum of 5 characters per integer + 1 for delimiter
	  char tempArray[6];
         //Convert integer to ASCII with a comma delimiter
	  sprintf(&tempArray[index], "%d, ", array[i]);
	  UART_Write(tempArray); // send the number
  }
}

void UART_Write(char data[])
{
	for(int i = 0 ; i<strlen(data); i++){
		// Send the data
		LPUART1->TDR = data[i];
		// Wait until the transmission is complete
		while (!(LPUART1->ISR & USART_ISR_TC_Msk));
              // Wait for TC to SET.. This indicates that the data has been transmitted
	}
}

uint8_t UART_Read(void)
{
	uint8_t temp = 0;
// Wait for RXNE to SET.. This indicates that the data has been Received
	if(!(LPUART1->ISR & (1<<5))){
		temp = LPUART1->RDR;
		LPUART1->ISR &= ~USART_ISR_RXNE;
	}else{
		for(int i = 0 ; i <100000; i++);		// timeout
	}
	return temp;
}

void UART_Init(void)
{
	RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN; 		// Enable LPUART1 clock
	LPUART1->CR1 = 0x00; // Clear ALL
	LPUART1->CR1 &= ~(USART_CR1_UE); // Disable LPUART
	LPUART1->BRR = 0xcd70; // Baud rate of 115200, PCLK1 at 40MHz
	LPUART1->CR1 &= ~(USART_CR1_M | USART_CR1_PCE | USART_CR1_OVER8);
	LPUART1->CR1 |= USART_CR1_RE; // RE=1.. Enable the Receiver
	LPUART1->CR1 |= USART_CR1_TE; // TE=1.. Enable Transmitter
	PWR->CR2 |= PWR_CR2_IOSV;
	LPUART1->CR1 |= USART_CR1_UE; // UE=1... Enable LPUSART
}

void LPUART_GPIO_Init(void)
{
	//PWR->CR2 |= PWR_CR2_IOSV;
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOGEN);
	GPIOG->MODER &= ~( (3<<14) | (3<<16) );
	GPIOG->MODER |= ( (2<<14) | (2<<16) );		// set the mode to AF
	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT2 | GPIO_OTYPER_OT3);
	GPIOG->AFR[0] |= (8<<28);					// AF low for PG7
	GPIOG->AFR[1] |= (8<<0);					// AF HIGH for PG8
}
