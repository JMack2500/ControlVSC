/* USER CODE BEGIN Header */
/**
 * Voltage Source Converter Control Program
 * Authors: Thomas Mack,
 * Date: 4/17/2024 begin
 * Description:
 * 	The STM32 is interfaced with an ATM90E32A (energy metering IC), through SPI.
 * 	Calls must be made to the ATM90 to receive data.
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

// Includes
#include "main.h"
#include "math.h"
#include "arm_math.h"
#include "DMA.h"
#include "UART.h"
#include "arm_const_structs.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// Global Variables
extern int callback_state;
#define ArraySize 8192
#define fft_size 4096


// Initialize variables
uint16_t myArray1[ArraySize];
float32_t fft_in_buf[fft_size];
float32_t fft_out_buf[fft_size];
arm_rfft_fast_instance_f32 fft_handler;
uint32_t maxIndex;
int freqs[fft_size/2];
float32_t mags[(fft_size/2)/20];
char amp[37] = " "; // Initialize amp to an empty string
uint32_t ind = 0;
float32_t  maxValue;
uint32_t maxIndex;
char freqBuf[64];
int arrayToLED[32];		// array used to plot LED matrix data

// Prototype Function Declarations
void DoFFT(void);
void plot_UART(void);
void sendIntArray(int array[], int size);
void runInitializations(void);

// Main Function, initialize then loop
int main(void)
{
	runInitializations();		// Initialize various services
	DMA_Config ((uint32_t) &(ADC1->DR) , (uint32_t) myArray1, ArraySize);	// Config DMA for
	seupInterrupte();
	calibrateATM90E();

	while (1)	// Continuous running state
	{
		  int fft_in_ptr = 0;
		  if (callback_state == 1) {			// DMA half transfer callback
                        // copy from the first half of array
			  for (int i=0; i< 2048*2; i++) {
				  fft_in_buf[fft_in_ptr] =  (float32_t) (myArray1[i]);
				  fft_in_ptr++;
			  }
			  DoFFT();
		  }
		  if (callback_state == 2) {			// DMA full transfer callback
                        // copy from the second half of array
			  for (int i= 2048*2; i< 4096*2; i++) {
				  fft_in_buf[fft_in_ptr] =  (float32_t) (myArray1[i]);
				  fft_in_ptr++;
			  }
			  DoFFT();
		  }
	}
}

// Initialization functions
void runInitializations(void) {
	HAL_Init();
	SystemClock_Config();	// main.c
	//SysTick_Init(); Necessary if using delay_us()
	GPIO_Init();			// main.c
	SPI_Init();				// SPI.c
	timer2_Init();			// main.c
	LPUART_GPIO_Init();		// UART.c
	UART_Init(); 			// UART.c
	arm_rfft_fast_init_f32(&fft_handler, fft_size);
	DMA_Init();				// DMA.c
}

void GPIO_Init(void){
	/* GPIO Ports Clock Enable */
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
	/* GPIO Connections:
	 * To Sensor Board:
	 *	~RESET (active low)
	 *	WARN
	 * From Sensor Board:
	 * 	DC Voltage (Voltage Div)
	 * 	DC Current (Hall Effect)
	 * To Driver Board:
	 * 	ENABLE	(active high???)
	 * 	1 (gate driver controls)
	 * 	2
	 * 	3
	 */

	/*Setting up lED output*/
	GPIOB->MODER &= ~ ( GPIO_MODER_MODE7) ;
	GPIOC->MODER &= ~ ( GPIO_MODER_MODE1) ;
	GPIOC->MODER &= ~ ( GPIO_MODER_MODE3) ;		// NRZ pin not used
	GPIOB->MODER |= ( (1 << GPIO_MODER_MODE7_Pos));		// blue led
	GPIOC->MODER |= ( (1 << GPIO_MODER_MODE1_Pos));		// DMA toggling pin
	GPIOC->MODER |= ( (1 << GPIO_MODER_MODE3_Pos));    // NRZ pin
}

void timer2_Init(void)
{
	RCC->APB1ENR1 |= (1<<0);		  	 // Enable Tim2 clock
	//TIM2 ->PSC = 2;				   // pre-scaler 16 bit value 23112
	//TIM2 ->ARR =  11556;		  // auto-relod value 16 bit value	4096 hz
	//TIM2 ->ARR =  1000;		  // auto-relod value 16 bit value 40 KHz sampling
	TIM2 ->ARR =  1075;		  // auto-relod value 16 bit value 2048 Hz sampling
	TIM2->CR2 |= TIM_CR2_MMS_1;      // Set timer to trigger ADC conversion
	TIM2 ->CR1 |= (1<<0);		   // Enable Tim2 counter
}

void ADC_Init(void)
{
    // 1. Enable ADC clock and select clock source
    RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;
    ADC123_COMMON->CCR |= ADC_CCR_CKMODE_0; // HCLK/1
    // 2. Disable ADC deep power down and enable internal voltage regulator
    ADC1->CR &= ~ADC_CR_DEEPPWD;
    ADC1->CR |= ADC_CR_ADVREGEN;
    for(uint16_t i = 0; i < 1000; i++); // Wait for regulator to settle
    // 3. Configure ADC for single-ended mode and perform calibration
    ADC1->CR &= ~(ADC_CR_ADEN | ADC_CR_ADCALDIF);
    ADC1->CR |= ADC_CR_ADCAL;
    while(ADC1->CR & ADC_CR_ADCAL);
    ADC1->DIFSEL &= ~(ADC_DIFSEL_DIFSEL_1);
    // 4. Enable ADC ready flag and wait for it to clear
    ADC1->ISR |= ADC_ISR_ADRDY;
    ADC1->CR |= ADC_CR_ADEN;
    while(!(ADC1->ISR & ADC_ISR_ADRDY));
    ADC1->ISR |= ADC_ISR_ADRDY;
    // 5. Configure ADC sequence and sampling time
    ADC1->SQR1 = (ADC1->SQR1 & ~(ADC_SQR1_SQ1_Msk | ADC_SQR1_L_Msk)) |
    (1 << ADC_SQR1_SQ1_Pos); // PC0 is channel 1 or 10
    ADC1->SMPR2 |= (0 << ADC_SMPR2_SMP10_Pos); // 6.5 ADC clock cycles
    // 6. Enable end of conversion interrupt and clear flag
    ADC1->IER |= ADC_IER_EOC;
    ADC1->ISR &= ~ADC_ISR_EOC;
    // 7. Enable GPIOC clock and configure PC0 as analog input
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
    GPIOC->MODER |= GPIO_MODER_MODE0_Msk; // Analog mode
    //8. config ADC1 DFSMCFG  and offset to get signed 16 bit p.549
    ADC1->IER |= (1<<4);  // enable overrun mode
    ADC1->CFGR |= (1<<12);  // enable overrun mode to rewrite new data

    ADC1->CFGR |= ADC_CFGR_EXTEN_0; // set EXTEN bits to 0b01 (i.e., rising edge triggered)
    // set EXTSEL bits to 0b1011 (i.e., TIM2 TRGO)
    ADC1->CFGR |= ADC_CFGR_EXTSEL_3 | ADC_CFGR_EXTSEL_1 | ADC_CFGR_EXTSEL_0;
    //9. Enable DMA for ADC1
    ADC1->CFGR |= (ADC_CFGR_DMAEN );
    ADC1->CFGR |= (ADC_CFGR_DMACFG);
    ADC1->CR |= ADC_CR_ADSTART;
}

void DoFFT(void)
{
    // Apply the Hanning window to the input buffer
    for (int i = 0; i < fft_size; i++)
    {
        float32_t window = 0.5 * (1 - cos((2 * M_PI * i) / (float)(fft_size - 1)));
        fft_in_buf[i] = fft_in_buf[i] * window;
    }
    // Perform the FFT on the input buffer
    arm_rfft_fast_f32(&fft_handler, fft_in_buf, fft_out_buf, 0);
    // Calculate the magnitudes of the complex FFT output
    arm_cmplx_mag_f32(fft_out_buf, fft_out_buf, fft_size);
    // Scale the magnitude values to account for the DC component and Nyquist frequency
    fft_out_buf[0] = fft_out_buf[0] / fft_size;    // DC component
    // Nyquist frequency
    fft_out_buf[fft_size / 2] = fft_out_buf[fft_size / 2] / fft_size;
    for (int i = 1; i < fft_size / 2; i++)
    {
        //fft_out_buf[i] = fft_out_buf[i] * 2 / fft_size;
    	freqs[i] = fft_out_buf[i] * 2 / 44000;
    }
    plot_UART();
}

void plot_UART(void)
{
	// average the 2048 array into 102
    for (int j = 0; j < (fft_size / 2) / 20; j++) {
        int sum = 0;
        for (int k = j * 20; k < (j + 1) * 20; k++) {
            sum += freqs[k];
        }
        mags[j] = sum / 20;				// save the average
    }
    float32_t maxMag;
    arm_max_f32(mags, (fft_size / 2) / 20 - 1, &maxMag,&maxIndex);
// this for loop is used to plot the frequency data to the LED matrix
// only frequencies neighboring the maximum frequency are plotted
    for (int i= maxIndex-15; i < maxIndex+15; i++)
    {
    	arrayToLED[i] = mags[i];
    }
    sendIntArray(arrayToLED, 32);

// the commented block below is used for ploting the frequency spectrum b/n 0 - 22 kHz
    //UART_Write("\x1B[2J"); // clear terminal
//    UART_Write("\x1B[H");  // set cursor home
//
//    UART_Write("|\r\n"); // Print the y-axis line
//
//    for (int i = 35; i >= 0; i--) {
//        UART_Write("|");
//
//        for (int j = 0; j < (fft_size / 2) / 20; j++) {
//            if (mags[j] > (maxMag / 36.0) * i) {
//                UART_Write("#");
//            } else {
//                UART_Write(" ");
//            }
//        }
//
//        UART_Write("\r\n");
//    }
//
//
//	UART_Write("|_________________________________________________________________"
//			"__________________________________\r\n");
//	UART_Write("0                      5k                      10k                "
//			"    15k                    20k     (Hz) \r\n");
//
//	// re-scale the frequency before sending by maxIndex times 10.7 and 20
//	// because array compressing & averaging
//	sprintf(freqBuf,"The input frequency is: %lu Hz\r\n" , maxIndex*218);
//	UART_Write(freqBuf);
}

// configure SysTick timer for use with delay_us().
// warning: breaks HAL_delay() by disabling interrupts for shorter delay timing.
void SysTick_Init(void) {
	SysTick->CTRL |= (SysTick_CTRL_ENABLE_Msk |     	// enable SysTick Timer
                      SysTick_CTRL_CLKSOURCE_Msk); 	// select CPU clock
	SysTick->CTRL &= ~(SysTick_CTRL_TICKINT_Msk);  	// disable interrupt
}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 71;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
