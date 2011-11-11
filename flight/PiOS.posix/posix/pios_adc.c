/**
 ******************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup   PIOS_ADC ADC Functions
 * @brief STM32F4xx ADC PIOS interface
 * @{
 *
 * @file       pios_adc.c  
 * @author     Michael Smith Copyright (C) 2011.
 * @brief      Analog to Digital converstion routines
 * @see        The GNU Public License (GPL) Version 3
 *****************************************************************************/
/* 
 * This program is free software; you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License as published by 
 * the Free Software Foundation; either version 3 of the License, or 
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY 
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License 
 * for more details.
 * 
 * You should have received a copy of the GNU General Public License along 
 * with this program; if not, write to the Free Software Foundation, Inc., 
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

/*
 * @note This is a stripped-down ADC driver intended primarily for sampling
 * voltage and current values.  Samples are averaged over the period between
 * fetches so that relatively accurate measurements can be obtained without
 * forcing higher-level logic to poll aggressively.
 *
 * @todo This module needs more work to be more generally useful.  It should
 * almost certainly grow callback support so that e.g. voltage and current readings
 * can be shipped out for coulomb counting purposes.  The F1xx interface presumes
 * use with analog sensors, but that implementation largely dominates the ADC
 * resources.  Rather than commit to a new API without a defined use case, we
 * should stick to our lightweight subset until we have a better idea of what's needed.
 */

#include "pios.h"

extern struct pios_adc_cfg pios_adc_cfg;

static void init_pins(void);
static void init_dma(void);
static void init_timer(void);
static void init_adc(void);

struct adc_accumulator {
	uint32_t		accumulator;
	uint32_t		count;
};

#define PIOS_ADC_NUM_PINS 2
#define PIOS_ADC_MAX_SAMPLES 20

static struct adc_accumulator accumulator[PIOS_ADC_NUM_PINS];

static uint16_t adc_raw_buffer[2][PIOS_ADC_MAX_SAMPLES][PIOS_ADC_NUM_PINS];

static void
init_pins(void)
{

}

static void
init_dma(void)
{

}

static void
init_timer(void)
{

}

static void
init_adc(void)
{

}

/**
 * @brief Init the ADC.
 */
void PIOS_ADC_Init()
{
	init_pins();
	init_dma();
	init_timer();
	init_adc();
}

/**
 * @brief Configure the ADC to run at a fixed oversampling
 * @param[in] oversampling the amount of oversampling to run at
 */
void PIOS_ADC_Config(uint32_t oversampling)
{
	/* we ignore this */
}

/**
 * Returns value of an ADC Pin
 * @param[in] pin number
 * @return ADC pin value averaged over the set of samples since the last reading.
 * @return -1 if pin doesn't exist
 */
int32_t PIOS_ADC_PinGet(uint32_t pin)
{
	int32_t	result = 0;

	/* Check if pin exists */
	if (pin >= PIOS_ADC_NUM_PINS) {
		return -1;
	}
	
	return result;
}

#if defined(PIOS_INCLUDE_FREERTOS)
/**
 * @brief Register a queue to add data to when downsampled 
 * @note Not currently supported.
 */
void PIOS_ADC_SetQueue(xQueueHandle data_queue) 
{
	// XXX it might make sense? to do this
}
#endif

/**
 * @brief Return the address of the downsampled data buffer
 * @note Not currently supported.
 */
float * PIOS_ADC_GetBuffer(void)
{
	return NULL;
}

/**
 * @brief Return the address of the raw data data buffer 
 * @note Not currently supported.
 */
int16_t * PIOS_ADC_GetRawBuffer(void)
{
	return NULL;
}

/**
 * @brief Return the amount of over sampling
 * @note Not currently supported (always returns 1)
 */
uint8_t PIOS_ADC_GetOverSampling(void)
{
	return 1;
}

/**
 * @brief Set the fir coefficients.  Takes as many samples as the 
 * current filter order plus one (normalization)
 *
 * @param new_filter Array of adc_oversampling floats plus one for the
 * filter coefficients
 * @note Not currently supported.
 */
void PIOS_ADC_SetFIRCoefficients(float * new_filter)
{
	// not implemented
}

/**
 * @brief accumulate the data for each of the channels.
 */
void accumulate(uint16_t *buffer, uint32_t count)
{
	

	// XXX callback?
}

/**
 * @brief Interrupt on buffer flip.
 *
 * The hardware is done with the 'other' buffer, so we can pass it to the accumulator.
 */
void PIOS_ADC_DMA_Handler(void)
{
}

/** 
 * @}
 * @}
 */
