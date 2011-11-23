/*
 * pios_clocks.c
 *
 *  Created on: Nov 12, 2011
 *      Author: msmith
 */

/**
 * Clock initialisation and lookup functions.
 */

#include <stm32f4xx_rcc.h>
#include <pios_stm32f4xx.h>

static RCC_ClocksTypeDef	clocks;

uint32_t
PIOS_CLOCK_Frequency(void *periph_base)
{
	switch ((uintptr_t)periph_base) {
	case (uintptr_t)SPI2:
	case (uintptr_t)SPI3:
	case (uintptr_t)UART4:
	case (uintptr_t)UART5:
	case (uintptr_t)USART2:
	case (uintptr_t)USART3:
		return clocks.PCLK1_Frequency;

	case (uintptr_t)TIM2:
	case (uintptr_t)TIM3:
	case (uintptr_t)TIM4:
	case (uintptr_t)TIM5:
	case (uintptr_t)TIM6:
	case (uintptr_t)TIM7:
	case (uintptr_t)TIM12:
	case (uintptr_t)TIM13:
	case (uintptr_t)TIM14:
		return clocks.PCLK1_Frequency * 2;	// XXX unless APB1 prescaler is 1

	case (uintptr_t)ADC:
	case (uintptr_t)ADC1:
	case (uintptr_t)ADC2:
	case (uintptr_t)ADC3:
	case (uintptr_t)SDIO:
	case (uintptr_t)SPI1:
	case (uintptr_t)USART1:
	case (uintptr_t)USART6:
		return clocks.PCLK2_Frequency;

	case (uintptr_t)TIM1:
	case (uintptr_t)TIM8:
	case (uintptr_t)TIM9:
	case (uintptr_t)TIM10:
	case (uintptr_t)TIM11:
		return clocks.PCLK2_Frequency * 2;	// XXX unless APB2 prescaler is 1

	case PIOS_CLOCK_SYSTICK:
		return clocks.HCLK_Frequency / 8;

	case PIOS_CLOCK_CPU:
		return clocks.SYSCLK_Frequency;

	default:
		return 0;
	}
}

void
PIOS_CLOCK_Init(void)
{

	/*
	 * Turn on all the peripheral clocks.
	 * Micromanaging clocks makes no sense given the power situation in the system, so
	 * light up everything we might reasonably use here and just leave it on.
	 */
	RCC_AHB1PeriphClockCmd(
			       RCC_AHB1Periph_GPIOA |
			       RCC_AHB1Periph_GPIOB |
			       RCC_AHB1Periph_GPIOC |
			       RCC_AHB1Periph_GPIOD |
			       RCC_AHB1Periph_GPIOE |
			       RCC_AHB1Periph_GPIOF |
			       RCC_AHB1Periph_GPIOG |
			       RCC_AHB1Periph_GPIOH |
			       RCC_AHB1Periph_GPIOI |
			       RCC_AHB1Periph_CRC |
			       RCC_AHB1Periph_FLITF |
			       RCC_AHB1Periph_SRAM1 |
			       RCC_AHB1Periph_SRAM2 |
			       RCC_AHB1Periph_BKPSRAM |
			       RCC_AHB1Periph_DMA1 |
			       RCC_AHB1Periph_DMA2 |
			       //RCC_AHB1Periph_ETH_MAC |			No ethernet
			       //RCC_AHB1Periph_ETH_MAC_Tx |
			       //RCC_AHB1Periph_ETH_MAC_Rx |
			       //RCC_AHB1Periph_ETH_MAC_PTP |
			       //RCC_AHB1Periph_OTG_HS |			No high-speed USB (requires external PHY)
			       //RCC_AHB1Periph_OTG_HS_ULPI |		No ULPI PHY (see above)
			0, ENABLE);
	RCC_AHB2PeriphClockCmd(
			       //RCC_AHB2Periph_DCMI |				No camera   @todo might make sense later for basic vision support?
			       //RCC_AHB2Periph_CRYP |				No crypto
			       //RCC_AHB2Periph_HASH |				No hash generator
			       //RCC_AHB2Periph_RNG |				No random numbers @todo might be good to have later if entropy is desired
			       RCC_AHB2Periph_OTG_FS |
			0, ENABLE);
	RCC_AHB3PeriphClockCmd(
			       //RCC_AHB3Periph_FSMC |				No external static memory
			0, ENABLE);
	RCC_APB1PeriphClockCmd(
			       RCC_APB1Periph_TIM2 |
			       RCC_APB1Periph_TIM3 |
			       RCC_APB1Periph_TIM4 |
			       RCC_APB1Periph_TIM5 |
			       RCC_APB1Periph_TIM6 |
			       RCC_APB1Periph_TIM7 |
			       RCC_APB1Periph_TIM12 |
			       RCC_APB1Periph_TIM13 |
			       RCC_APB1Periph_TIM14 |
			       RCC_APB1Periph_WWDG |
			       RCC_APB1Periph_SPI2 |
			       RCC_APB1Periph_SPI3 |
			       RCC_APB1Periph_USART2 |
			       RCC_APB1Periph_USART3 |
			       RCC_APB1Periph_UART4 |
			       RCC_APB1Periph_UART5 |
			       RCC_APB1Periph_I2C1 |
			       RCC_APB1Periph_I2C2 |
			       RCC_APB1Periph_I2C3 |
			       RCC_APB1Periph_CAN1 |
			       RCC_APB1Periph_CAN2 |
			       RCC_APB1Periph_PWR |
			       RCC_APB1Periph_DAC |
			0, ENABLE);

	RCC_APB2PeriphClockCmd(
			       RCC_APB2Periph_TIM1 |
			       RCC_APB2Periph_TIM8 |
			       RCC_APB2Periph_USART1 |
			       RCC_APB2Periph_USART6 |
			       RCC_APB2Periph_ADC |
			       RCC_APB2Periph_ADC1 |
			       RCC_APB2Periph_ADC2 |
			       RCC_APB2Periph_ADC3 |
			       RCC_APB2Periph_SDIO |
			       RCC_APB2Periph_SPI1 |
			       RCC_APB2Periph_SYSCFG |
			       RCC_APB2Periph_TIM9 |
			       RCC_APB2Periph_TIM10 |
			       RCC_APB2Periph_TIM11 |
			0, ENABLE);

	/* calculate and save the bus clocks */
	RCC_GetClocksFreq(&clocks);
}
