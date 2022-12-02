/*
ST Nucleo + KA-Nucleo-Multisensor Board
7-segment mpx display, soft refresh, brightness control
gbm, 2017-2022
*/

#include "board.h"
#include "disp.h"

uint32_t display[NDIGITS] = {DigAct(DIG0_MSK), DigAct(DIG1_MSK), DigAct(DIG2_MSK), DigAct(DIG3_MSK)};
uint32_t cpxctrl[NDIGITS];

// Display stuff
// encode table for digits 0..9
static const uint8_t encode_digit[] = {
	SEG_A_MSK | SEG_B_MSK | SEG_C_MSK | SEG_D_MSK | SEG_E_MSK | SEG_F_MSK,	// 0
	SEG_B_MSK | SEG_C_MSK,	// 1
	SEG_A_MSK | SEG_B_MSK | SEG_D_MSK | SEG_E_MSK | SEG_G_MSK,	// 2
	SEG_A_MSK | SEG_B_MSK | SEG_C_MSK | SEG_D_MSK | SEG_G_MSK,	// 3
	SEG_B_MSK | SEG_C_MSK | SEG_F_MSK | SEG_G_MSK,	// 4
	SEG_A_MSK | SEG_C_MSK | SEG_D_MSK | SEG_F_MSK | SEG_G_MSK,	// 5
	SEG_A_MSK | SEG_C_MSK | SEG_D_MSK | SEG_E_MSK | SEG_F_MSK | SEG_G_MSK,	// 6
	SEG_A_MSK | SEG_B_MSK | SEG_C_MSK,	// 7
	SEG_A_MSK | SEG_B_MSK | SEG_C_MSK | SEG_D_MSK | SEG_E_MSK | SEG_F_MSK | SEG_G_MSK,	// 8
	SEG_A_MSK | SEG_B_MSK | SEG_C_MSK | SEG_D_MSK | SEG_F_MSK | SEG_G_MSK,	// 9
	SEG_A_MSK | SEG_B_MSK | SEG_C_MSK | SEG_E_MSK | SEG_F_MSK | SEG_G_MSK,	// A
	SEG_C_MSK | SEG_D_MSK | SEG_E_MSK | SEG_F_MSK | SEG_G_MSK,	// B
	SEG_A_MSK | SEG_D_MSK | SEG_E_MSK | SEG_F_MSK,	// C
	SEG_B_MSK | SEG_C_MSK | SEG_D_MSK | SEG_E_MSK | SEG_G_MSK,	// d
	SEG_A_MSK | SEG_D_MSK | SEG_E_MSK | SEG_F_MSK | SEG_G_MSK,	// E
	SEG_A_MSK | SEG_E_MSK | SEG_F_MSK | SEG_G_MSK,	// F
	SEG_A_MSK | SEG_C_MSK | SEG_D_MSK | SEG_E_MSK | SEG_F_MSK,	// G
	SEG_B_MSK | SEG_C_MSK | SEG_E_MSK | SEG_F_MSK | SEG_G_MSK,	// H
	SEG_E_MSK | SEG_F_MSK,	// I
	SEG_B_MSK | SEG_C_MSK | SEG_D_MSK | SEG_E_MSK,	// J
};
static const uint16_t digmask[] = {DIG0_MSK, DIG1_MSK, DIG2_MSK, DIG3_MSK};

void blank_digit(uint8_t pos)
{
	display[pos] = DigAct(digmask[pos]) | SegAct(0);
}

void set_digit(uint8_t pos, uint8_t val)
{
	display[pos] = DigAct(digmask[pos]) | SegAct(encode_digit[val]);
}

void disp_init(void)
{
#ifdef STM32F4
	RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_TIM4EN | RCC_APB1ENR_TIM3EN
		| RCC_APB1ENR_USART2EN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_TIM1EN;
#else
	RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN | RCC_APB1ENR1_TIM4EN | RCC_APB1ENR1_TIM3EN
		| RCC_APB1ENR1_USART2EN;
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN
		| RCC_AHB2ENR_ADCEN;
#endif
	// GPIOC - display and button
	GPIOC->MODER = BF2A(0, GPIO_MODER_OUT) & BF2A(1, GPIO_MODER_OUT) & BF2A(2, GPIO_MODER_OUT) & BF2A(3, GPIO_MODER_OUT)
		 & BF2A(4, GPIO_MODER_OUT) & BF2A(5, GPIO_MODER_OUT) & BF2A(6, GPIO_MODER_OUT) & BF2A(7, GPIO_MODER_OUT)
		 & BF2A(8, GPIO_MODER_OUT) & BF2A(9, GPIO_MODER_OUT) & BF2A(10, GPIO_MODER_OUT) & BF2A(11, GPIO_MODER_OUT)
		 & BF2A(13, GPIO_MODER_IN);
	GPIOC->PUPDR = BF2(13, GPIO_PUPDR_PU);	// Nucleo64 button
	// LED multiplexing timer
	LEDMpx_TIM->PSC = SYSCLK_FREQ / MPX_FREQ / MPX_STEPS - 1;
	LEDMpx_TIM->ARR = MPX_STEPS - 1;

	LEDMpx_TIM->DIER = TIM_DIER_CC1IE | TIM_DIER_UIE;
	// interrupts and sleep control
	NVIC_EnableIRQ(LEDMpx_IRQn);
#ifdef STM32F4
	NVIC_EnableIRQ(TIM1_CC_IRQn);
#endif
	LEDMpx_TIM->CR1 = TIM_CR1_CEN;
//	set_digit(0, 1);
//	set_digit(1, 3);
//	set_digit(2, 5);
//	set_digit(3, 7);
}

void LEDMpx_IRQHandler(void)
{
	uint32_t tim_sr = LEDMpx_TIM->SR;
	LEDMpx_TIM->SR = ~(TIM_SR_UIF | TIM_SR_CC1IF);
	
	if (tim_sr & TIM_SR_UIF)
	{
		static uint8_t dig;
		dig = (dig + 1) % NDIGITS;
		LEDMpx_PORT->BSRR = display[dig];
		
		// CPX
		CPX_PORT->BSRR = CPX_BSRR_OFF | CPX_LED3_MSK >> dig;
		CPX_PORT->MODER |= cpxctrl[dig];
		
		static uint8_t tdiv;
		if (++tdiv == MPX_FREQ / 100)
		{
			tdiv = 0;
//			run_every_10ms();
		}
	}
	if (tim_sr & TIM_SR_CC1IF)
	{
//		LEDMpx_PORT->BSRR = DigAct(0) | SegAct(0);
//		CPX_PORT->MODER &= CPX_MODER_OFF;
	}
}

#ifdef STM32F4
void TIM1_CC_IRQHandler(void)
{
	LEDMpx_IRQHandler();
}
#endif
