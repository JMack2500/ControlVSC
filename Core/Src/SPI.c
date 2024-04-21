/*
 * SPI.c
 *
 *  Created on: Apr 17, 2024
 *      Author: baseb
 */

#include "stm32l4xx_hal.h"

// SPI functions
void SPI1_Write(uint16 word) {	// Write a 16 bit word out of SPI1
	data = (SPI_DR_GN | SPI_DR_SD | word);
	if(SPI1->SR & SPI_TXCRCR_TXCRC){
		SPI1->DR &= ~(SPI_DR_DR_Msk);
		SPI1->DR |= (data);
	}
}

void SPI_Init( void ) {
    // SPI config as specified @ STM32L4 RM0351 rev.9 p.1459
    // build control registers CR1 & CR2 for SPI control of peripheral DAC
    // assumes no active SPI xmits & no recv data in process (BSY=0)
    // CR1 (reset value = 0x0000)
    SPI1->CR1 &= ~( SPI_CR1_SPE );        		 // disable SPI for config
    SPI1->CR1 &= ~( SPI_CR1_RXONLY );     		 // recv-only OFF
    SPI1->CR1 &= ~( SPI_CR1_LSBFIRST );   		 // data bit order MSb:LSb
    SPI1->CR1 &= ~( SPI_CR1_CPOL | SPI_CR1_CPHA ); // SCLK polarity:phase = 0:0
    SPI1->CR1 |=     SPI_CR1_MSTR;         		 // MCU is SPI controller
    // CR2 (reset value = 0x0700 : 8b data)
    SPI1->CR2 &= ~( SPI_CR2_TXEIE | SPI_CR2_RXNEIE ); // disable FIFO intrpts
    SPI1->CR2 &= ~( SPI_CR2_FRF);         		 // Moto frame format
    SPI1->CR2 |=     SPI_CR2_NSSP;         		 // auto-generate NSS pulse
    SPI1->CR2 |=     SPI_CR2_DS;           		 // 16-bit data
    SPI1->CR2 |=     SPI_CR2_SSOE;         		 // enable SS output
    // CR1
    SPI1->CR1 |=     SPI_CR1_SPE;          		 // re-enable SPI for ops
}
