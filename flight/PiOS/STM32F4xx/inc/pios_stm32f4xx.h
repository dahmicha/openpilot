/*
 * pios_i2c_config.h
 *
 *  Created on: November 12, 2011
 *      Author: Michael Smith
 */

/**
 * \file	PiOS internal functions for the STM32F4xx family.
 */

/**
 * Initialise clocks, increases performance to nominal.
 */
void		PIOS_CLOCK_Init(void);

/**
 * Return the clock frequency supplied to the given peripheral
 *
 * \param	periph_base		Pointer to the peripheral's register block,
 *							or one of the PIOS_CLOCK_* defines below.
 */
uint32_t	PIOS_CLOCK_Frequency(void *periph_base);

#define PIOS_CLOCK_SYSTICK		10
#define PIOS_CLOCK_CPU			11

