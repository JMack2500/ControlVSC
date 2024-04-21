/*
 * DMA.h
 *
 *  Created on: Apr 17, 2024
 *      Author: baseb
 */

#ifndef INC_DMA_H_
#define INC_DMA_H_

// Function Boiler Plates
void DMA_Init(void);
void DMA_Config (uint32_t srcAdd, uint32_t destAdd, uint16_t datasize);

#endif /* INC_DMA_H_ */
