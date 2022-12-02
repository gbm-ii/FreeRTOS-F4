/*
 * board.h
 * resource definitions for KA-Nucleo_Multisensor on Nucleo-F4xx
 *  Created on: Dec 1, 2022
 *      Author: gbm
 */

#ifndef INC_BOARD_H_
#define INC_BOARD_H_

#include "ka_nuc_multis.h"
#include "stm32f4yy.h"

#define SYSCLK_FREQ	84000000u

//========================================================================
// OneWire peripherals
#define OWTIM	TIM3
#define OWTIM_IRQHandler	TIM3_IRQHandler
#define OW_PIN	OW_BIT
//========================================================================
// LED Mpx peripherals
#define LEDMpx_TIM	TIM1
#define LEDMpx_IRQn	TIM1_UP_TIM10_IRQn
#define LEDMpx_IRQHandler	TIM1_UP_TIM10_IRQHandler
#define LEDMpx_DMACh	DMA1_Channel7
#define LEDMpx_DMARq	6
#define LEDMpxOff_DMACh	DMA1_Channel1
#define LEDMpxFad_DMACh	DMA1_Channel4

#define	LIS_SPI_RX_DMACH DMA1_Channel2
#define	LIS_SPI_TX_DMACH DMA1_Channel3

#define	USART2_RX_DMACH DMA1_Channel6
#define	USART2_TX_DMACH DMA1_Channel7
#define	USART_DMARq 2

#endif /* INC_BOARD_H_ */

