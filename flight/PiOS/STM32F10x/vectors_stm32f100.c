/**
 * @file	startup.c
 * @brief	I/O peripheral vectors for STM32F100
 */

/** interrupt handler */
typedef const void	(vector)(void);

/** default interrupt handler */
static void
default_io_handler(void)
{
	for (;;) ;
}

/** prototypes an interrupt handler */
#define HANDLER(_name)	extern vector _name __attribute__((weak, alias("default_io_handler")))

HANDLER(WWDG_IRQHandler);                 // Window WatchDog
HANDLER(PVD_IRQHandler);                  // PVD through EXTI Line detection
HANDLER(TAMPER_IRQHandler);               // Tamper
HANDLER(RTC_WKUP_IRQHandler);             // RTC Wakeup through the EXTI line
HANDLER(FLASH_IRQHandler);                // FLASH
HANDLER(RCC_IRQHandler);                  // RCC
HANDLER(EXTI0_IRQHandler);                // EXTI Line0
HANDLER(EXTI1_IRQHandler);                // EXTI Line1
HANDLER(EXTI2_IRQHandler);                // EXTI Line2
HANDLER(EXTI3_IRQHandler);                // EXTI Line3
HANDLER(EXTI4_IRQHandler);                // EXTI Line4
HANDLER(DMA1_Channel1_IRQHandler);        // DMA1 Channel 1
HANDLER(DMA1_Channel2_IRQHandler);        // DMA1 Channel 2
HANDLER(DMA1_Channel3_IRQHandler);        // DMA1 Channel 3
HANDLER(DMA1_Channel4_IRQHandler);        // DMA1 Channel 4
HANDLER(DMA1_Channel5_IRQHandler);        // DMA1 Channel 5
HANDLER(DMA1_Channel6_IRQHandler);        // DMA1 Channel 6
HANDLER(DMA1_Channel7_IRQHandler);        // DMA1 Channel 7
HANDLER(ADC1_2_IRQHandler);               // ADC1, ADC2
HANDLER(EXTI9_5_IRQHandler);              // External Line[9:5]s
HANDLER(TIM1_BRK_IRQHandler);             // TIM1 Break
HANDLER(TIM1_UP_IRQHandler);              // TIM1 Update
HANDLER(TIM1_TRG_COM_IRQHandler);         // TIM1 Trigger and Commutation
HANDLER(TIM1_CC_IRQHandler);              // TIM1 Capture Compare
HANDLER(TIM2_IRQHandler);                 // TIM2
HANDLER(TIM3_IRQHandler);                 // TIM3
HANDLER(TIM4_IRQHandler);                 // TIM4
HANDLER(I2C1_EV_IRQHandler);              // I2C1 Event
HANDLER(I2C1_ER_IRQHandler);              // I2C1 Error
HANDLER(I2C2_EV_IRQHandler);              // I2C2 Event
HANDLER(I2C2_ER_IRQHandler);              // I2C2 Error
HANDLER(SPI1_IRQHandler);                 // SPI1
HANDLER(SPI2_IRQHandler);                 // SPI2
HANDLER(USART1_IRQHandler);               // USART1
HANDLER(USART2_IRQHandler);               // USART2
HANDLER(USART3_IRQHandler);               // USART3
HANDLER(EXTI15_10_IRQHandler);            // External Line[15:10]s
HANDLER(RTC_Alarm_IRQHandler);            // RTC Alarm (A and B) through EXTI Line
HANDLER(CEC_IRQHandler);                  //
HANDLER(TIM12_IRQHandler);                // TIM12
HANDLER(TIM13_IRQHandler);                // TIM13
HANDLER(TIM14_IRQHandler);                // TIM14
HANDLER(FSMC_IRQHandler);                 // FSMC
HANDLER(TIM5_IRQHandler);                 // TIM5
HANDLER(SPI3_IRQHandler);                 // SPI3
HANDLER(UART4_IRQHandler);               // UART4
HANDLER(UART5_IRQHandler);               // UART5
HANDLER(TIM6_DAC_IRQHandler);             // TIM6 and DAC1&2 underrun errors
HANDLER(TIM7_IRQHandler);                 // TIM7
HANDLER(DMA2_Channel1_IRQHandler);        // DMA2 Channel 1
HANDLER(DMA2_Channel2_IRQHandler);        // DMA2 Channel 2
HANDLER(DMA2_Channel3_IRQHandler);        // DMA2 Channel 3
HANDLER(DMA2_Channel4_5_IRQHandler);      // DMA2 Channel 4

/** stm32f100 interrupt vector table */
vector *io_vectors[] __attribute__((section(".io_vectors"))) = {
	WWDG_IRQHandler,			// exception #16
	PVD_IRQHandler,
	TAMPER_IRQHandler,
	RTC_WKUP_IRQHandler,
	FLASH_IRQHandler,
	RCC_IRQHandler,
	EXTI0_IRQHandler,
	EXTI1_IRQHandler,
	EXTI2_IRQHandler,
	EXTI3_IRQHandler,
	EXTI4_IRQHandler,
	DMA1_Channel1_IRQHandler,
	DMA1_Channel2_IRQHandler,
	DMA1_Channel3_IRQHandler,
	DMA1_Channel4_IRQHandler,
	DMA1_Channel5_IRQHandler,
	DMA1_Channel6_IRQHandler,
	DMA1_Channel7_IRQHandler,
	ADC1_2_IRQHandler,
	0,
	0,
	0,
	0,
	EXTI9_5_IRQHandler,
	TIM1_BRK_IRQHandler,
	TIM1_UP_IRQHandler,
	TIM1_TRG_COM_IRQHandler,
	TIM1_CC_IRQHandler,
	TIM2_IRQHandler,
	TIM3_IRQHandler,
	TIM4_IRQHandler,
	I2C1_EV_IRQHandler,
	I2C1_ER_IRQHandler,
	I2C2_EV_IRQHandler,
	I2C2_ER_IRQHandler,
	SPI1_IRQHandler,
	SPI2_IRQHandler,
	USART1_IRQHandler,
	USART2_IRQHandler,
	USART3_IRQHandler,
	EXTI15_10_IRQHandler,
	RTC_Alarm_IRQHandler,
	CEC_IRQHandler,
	TIM12_IRQHandler,
	TIM13_IRQHandler,
	TIM14_IRQHandler,
	0,
	0,
	FSMC_IRQHandler,
	0,
	TIM5_IRQHandler,
	SPI3_IRQHandler,
	UART4_IRQHandler,
	UART5_IRQHandler,
	TIM6_DAC_IRQHandler,
	TIM7_IRQHandler,
	DMA2_Channel1_IRQHandler,
	DMA2_Channel2_IRQHandler,
	DMA2_Channel3_IRQHandler,
	DMA2_Channel4_5_IRQHandler,
};
