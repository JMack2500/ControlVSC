#include "stm32l4xx_hal.h"
int callback_state = 0;
void DMA_Init(void)
{
// 1. Enable DMA1 Channel
	RCC-> AHB1ENR |= 1<<0;
// 2. Enable DMA Interrupts
	DMA1_Channel1->CCR |= (1<<1)|(1<<2)|(1<<3);  // TCIE, HTIE, TEIE Enabled
// 3. Set the Data Direction
	DMA1_Channel1->CCR &= ~(1<<4);   // Read From Peripheral
// 4. Enable the circular mode (CIRC)
	DMA1_Channel1->CCR |= 1<<5;
// 5. Enable the Memory Increment (MINC)
	DMA1_Channel1->CCR |= 1<<7;
	//DMA1_Channel1->CCR |= 1<<6;  // peripheral increment
// 6. Set the Peripheral data size (PSIZE)
	//DMA1_Channel1->CCR &= ~(3<<8);  // 00 : 8 Bit Data
	DMA1_Channel1->CCR |= (1<<8);  // 01 : 16 Bit Data
// 7. Set the Memory data size (MSIZE)
	//DMA1_Channel1->CCR &= ~(3<<10);  // 00 : 8 Bit Data
	DMA1_Channel1->CCR |= (1<<10);  // 01 : 16 Bit Data
// 8. Set the Priority Level
	DMA1_Channel1->CCR &= ~(0<<12);  // PLL = 0 , PLM = 1 ,PLH = 2 ,PLVH = 3
// 9. select DMA channel CSELR
	DMA1_CSELR -> CSELR &= ~(15<<0);  // 0000 for ADC1
	NVIC->ISER[0] = (1 << (DMA1_Channel1_IRQn & 0x1F));
	NVIC_SetPriority(DMA1_Channel1_IRQn, 2);
	__enable_irq();
}
void DMA_Config (uint32_t srcAdd, uint32_t destAdd, uint16_t datasize)
{
	// 1. Set the data size in CNDTR Register
	DMA1_Channel1->CNDTR = datasize ;
	// 2. Set the  peripheral address in PAR Register
	DMA1_Channel1->CPAR = srcAdd;
	// 3. Set the  Memory address in MAR Register
	DMA1_Channel1->CMAR = destAdd;
	// 4. Enable the DMA1
	DMA1_Channel1->CCR |= 1<<0;
}

void DMA1_Channel1_IRQHandler (void)
{
	if ((DMA1->ISR) &(1<<2))  // If the Half Transfer Complete Interrupt is set
	{
		GPIOC->ODR ^= GPIO_PIN_1;
		DMA1->IFCR |= (1<<2);
		callback_state = 1;
	}
	if ((DMA1->ISR)&(1<<1))  // If the Transfer Complete Interrupt is set
	{
		GPIOC->ODR ^= GPIO_PIN_1;
		DMA1->IFCR |= (1<<1);
		callback_state = 2;
	}
}
