/**
 ******************************************************************************
 * @addtogroup OpenPilotSystem OpenPilot System
 * @{
 * @addtogroup OpenPilotCore OpenPilot Core
 * @{
 * @file       pios_board.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      Defines board hardware for the OpenPilot Version 1.1 hardware.
 * @see        The GNU Public License (GPL) Version 3
 *
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


#ifndef STM32205_PX2FMU_H_
#define STM32205_PX2FMU_H_


//------------------------
// BOOTLOADER_SETTINGS
//------------------------

//#define FUNC_ID				1
//#define HW_VERSION			01

#define BOARD_TYPE				0x05	// PX2 FMU
#define BOARD_REVISION			0x01 	// prototype (should be a strapping option... silly to hardcode it here)
//#define HW_VERSION			(BOARD_TYPE << 8) | BOARD_REVISION

#define MEM_SIZE				(1024 * 1024)
#define SIZE_OF_DESCRIPTION		(uint8_t)100
#define START_OF_USER_CODE		(uint32_t)0x08010000 // NOTE: must match FLASH ORIGIN in link_STM32F2xx_application.ld
#define SIZE_OF_CODE			(uint32_t) (MEM_SIZE-(START_OF_USER_CODE-0x08000000)-SIZE_OF_DESCRIPTION)

#define BOARD_READABLE			TRUE
#define BOARD_WRITABLE			TRUE

//------------------------
// WATCHDOG_SETTINGS
//------------------------
#define PIOS_WATCHDOG_TIMEOUT	250
#define PIOS_WDG_REGISTER		RTC_BKP_DR4
#define PIOS_WDG_ACTUATOR		0x0001
#define PIOS_WDG_STABILIZATION 	0x0002
#define PIOS_WDG_AHRS			0x0004
#define PIOS_WDG_MANUAL			0x0008
#define PIOS_WDG_ATTITUDE		0x0010

//------------------------
// TELEMETRY
//------------------------
#define TELEM_QUEUE_SIZE		20
#define PIOS_TELEM_STACK_SIZE	1200

//------------------------
// MAVLINK
//------------------------
#define MAVLINK_QUEUE_SIZE		20
#define PIOS_MAVLINK_STACK_SIZE	2400

//------------------------
// PIOS_LED
//------------------------
#define PIOS_LED_LED1_GPIO_PORT                 GPIOB
#define PIOS_LED_LED1_GPIO_PIN                  GPIO_Pin_15
#define PIOS_LED_LED2_GPIO_PORT                 GPIOB
#define PIOS_LED_LED2_GPIO_PIN                  GPIO_Pin_14
#define PIOS_LED_NUM                            2
#define PIOS_LED_PORTS                          { PIOS_LED_LED1_GPIO_PORT, PIOS_LED_LED2_GPIO_PORT }
#define PIOS_LED_PINS                           { PIOS_LED_LED1_GPIO_PIN, PIOS_LED_LED2_GPIO_PIN }

//------------------------
// PIOS_SPI
// See also pios_board.c
//------------------------
#define PIOS_SPI_MAX_DEVS		2
#define SPI_CS_GYRO				0
#define SPI_CS_ACCEL			1

//------------------------
// PIOS_I2C
// See also pios_board.c
//------------------------
#define PIOS_I2C_MAX_DEVS		3
extern uint32_t pios_i2c_main_adapter_id;
#define PIOS_I2C_MAIN_ADAPTER	(pios_i2c_main_adapter_id)
extern uint32_t pios_i2c_esc_adapter_id;
#define PIOS_I2C_ESC_ADAPTER	(pios_i2c_esc_adapter_id)
extern uint32_t pios_i2c_external_adapter_id;
#define PIOS_I2C_EXTERNAL_ADAPTER	(pios_i2c_external_adapter_id)

/* I2C as available on the PX2 FMU */
#include <pios_i2c_config.h>

/* motor control */
#define I2C1_CONFIG()	\
		I2C_CONFIG(I2C1, GPIOB, GPIO_Pin_8, GPIOB, GPIO_Pin_9)

/* onboard sensors and EEPROM, IO comms */
#define I2C2_CONFIG()	\
		I2C_CONFIG(I2C2, GPIOB, GPIO_Pin_10, GPIOB, GPIO_Pin_11)

/* offboard sensors */
#define I2C3_CONFIG()	\
		I2C_CONFIG(I2C3, GPIOA, GPIO_Pin_8, GPIOC, GPIO_Pin_9)

#define PIOS_I2C_EEPROM_ADDRESS					0x50		/* on the MAIN bus */
#define PIOS_I2C_EEPROM_SIZE					(16 * 1024)

//------------------------
// PIOS_BMP085
//------------------------
#define PIOS_BMP085_EOC_GPIO_PORT               GPIOB
#define PIOS_BMP085_EOC_GPIO_PIN                GPIO_Pin_0
#define PIOS_BMP085_EOC_PORT_SOURCE             EXTI_PortSourceGPIOB
#define PIOS_BMP085_EOC_PIN_SOURCE              EXTI_PinSource0
#define PIOS_BMP085_EOC_EXTI_LINE               EXTI_Line0
#define PIOS_BMP085_EOC_IRQn                    EXTI0_IRQn
#define PIOS_BMP085_EOC_PRIO                    PIOS_IRQ_PRIO_LOW
#define PIOS_BMP085_OVERSAMPLING                3

//------------------------
// PIOS_HMC5883
//------------------------
#define PIOS_HMC5883_DRDY_GPIO_PORT				GPIOB
#define PIOS_HMC5883_DRDY_GPIO_PIN				GPIO_Pin_1
#define PIOS_HMC5883_DRDY_PORT_SOURCE			EXTI_PortSourceGPIOB
#define PIOS_HMC5883_DRDY_PIN_SOURCE			EXTI_PinSource1
#define PIOS_HMC5883_DRDY_EXTI_LINE				EXTI_Line1
#define PIOS_HMC5883_DRDY_IRQn					EXTI1_IRQn
#define PIOS_HMC5883_DRDY_PRIO					PIOS_IRQ_PRIO_HIGH

//-------------------------
// PIOS_USART
//-------------------------
#define PIOS_USART_MAX_DEVS             4

#define PIOS_USART_RX_BUFFER_SIZE       512
#define PIOS_USART_TX_BUFFER_SIZE       512

/* USARTs as available on the PX2 FMU */
#include <pios_usart_config.h>

#define USART1_CONFIG(_baudrate)	\
		USART_CONFIG(USART1, _baudrate, GPIOB, GPIO_Pin_7, GPIOB, GPIO_Pin_6)

#define USART2_CONFIG(_baudrate)	\
		USART_CONFIG(USART2, _baudrate, GPIOA, GPIO_Pin_3, GPIOA, GPIO_Pin_2)

#define UART5_CONFIG(_baudrate) 	\
		USART_CONFIG(UART5, _baudrate, GPIOD, GPIO_Pin_2, GPIOC, GPIO_Pin_12)

#define USART6_CONFIG(_baudrate)	\
		USART_CONFIG(USART6, _baudrate, GPIOC, GPIO_Pin_7, GPIOC, GPIO_Pin_6)

//-------------------------
// PIOS_COM
//
// See also pios_board.c
//-------------------------
#define PIOS_COM_MAX_DEVS               (PIOS_USART_MAX_DEVS + 1)	// +1 for USB

#define PIOS_COM_TELEM_BAUDRATE         57600
//#define PIOS_COM_TELEM_BAUDRATE         921600
extern uint32_t pios_com_telem_rf_id;
#define PIOS_COM_TELEM_RF               (pios_com_telem_rf_id)

#define PIOS_COM_GPS_BAUDRATE           38400
extern uint32_t pios_com_gps_id;
#define PIOS_COM_GPS                    (pios_com_gps_id)

extern uint32_t pios_com_telem_usb_id;
#define PIOS_COM_TELEM_USB              (pios_com_telem_usb_id)

#define PIOS_COM_CONTROL_BAUDRATE		921600
extern uint32_t pios_com_control_id;
#define PIOS_COM_CONTROL				(pios_com_control_id)

#define PIOS_COM_AUX_BAUDRATE           57600
extern uint32_t pios_com_aux_id;
#define PIOS_COM_AUX                    (pios_com_aux_id)

#define PIOS_COM_DEBUG                  PIOS_COM_TELEM_RF // Was PIOS_COM_AUX

//-------------------------
// Delay Timer
//-------------------------
#define PIOS_DELAY_TIMER				TIM2
#define PIOS_DELAY_TIMER_RCC_FUNC		do {} while(0)

//-------------------------
// System Settings
//-------------------------
#define PIOS_MASTER_CLOCK				SYSCLK_FREQ	// XXX should really get this procedurally

//-------------------------
// Interrupt Priorities
//-------------------------
#define PIOS_IRQ_PRIO_LOW				12			// lower than RTOS
#define PIOS_IRQ_PRIO_MID				8			// higher than RTOS
#define PIOS_IRQ_PRIO_HIGH				5			// for SPI, ADC, I2C etc...
#define PIOS_IRQ_PRIO_HIGHEST			4			// for USART etc...

//------------------------
// PIOS_RCVR
// See also pios_board.c
//------------------------
#define PIOS_RCVR_MAX_DEVS				1			// number of receiver devices
#define PIOS_RCVR_MAX_CHANNELS			12			// number of channels (total)

//-------------------------
// Receiver PPM input
//-------------------------
#define PIOS_PPM_GPIO_PORT				GPIOA
#define PIOS_PPM_GPIO_PIN				GPIO_Pin_9
#define PIOS_PPM_GPIO_REMAP				GPIO_AF_TIM1
#define PIOS_PPM_TIM_CHANNEL			TIM_Channel_2
#define PIOS_PPM_TIM_CCR				TIM_IT_CC2
#define PIOS_PPM_TIM					TIM1
#define PIOS_PPM_TIM_IRQ				TIM1_CC_IRQn
#define PIOS_PPM_NUM_INPUTS				8  //Could be more if needed

//-------------------------
// Servo outputs
//-------------------------
#define PIOS_SERVO_UPDATE_HZ            50
#define PIOS_SERVOS_INITIAL_POSITION    0		// default to sending no signal


//-------------------------
// ADC
// PIOS_ADC_PinGet(0) = External voltage
// PIOS_ADC_PinGet(1) = AUX1 (PX2IO external pressure port)
// PIOS_ADC_PinGet(2) = AUX2 (Current sensor, if available)
// PIOS_ADC_PinGet(3) = AUX3
// PIOS_ADC_PinGet(4) = VREF
// PIOS_ADC_PinGet(5) = Temperature sensor
//-------------------------

#define PIOS_DMA_PIN_CONFIG												\
	{																	\
		{GPIOC,	GPIO_Pin_0,	ADC_Channel_10},							\
		{GPIOC, GPIO_Pin_1,	ADC_Channel_11},							\
		{GPIOC,	GPIO_Pin_2,	ADC_Channel_12},							\
		{GPIOC, GPIO_Pin_3,	ADC_Channel_13},							\
		{NULL,	0,			ADC_Channel_Vrefint},		/* Voltage reference */\
		{NULL,	0,			ADC_Channel_TempSensor}		/* Temperature sensor */\
	}

/* we have to do all this to satisfy the PIOS_ADC_MAX_SAMPLES define in pios_adc.h */
/* which is annoying because this then determines the rate at which we generate buffer turnover events */
/* the objective here is to get enough buffer space to support 100Hz averaging rate */
#define PIOS_ADC_NUM_CHANNELS		6
#define PIOS_ADC_MAX_OVERSAMPLING	10
#define PIOS_ADC_USE_ADC2			0


//-------------------------
// GPIO
//-------------------------
#define PIOS_GPIO_1_PORT                        GPIOC
#define PIOS_GPIO_1_PIN                         GPIO_Pin_4
#define PIOS_GPIO_2_PORT                        GPIOC
#define PIOS_GPIO_2_PIN                         GPIO_Pin_5
#define PIOS_GPIO_3_PORT                        GPIOC
#define PIOS_GPIO_3_PIN							GPIO_Pin_8
#define PIOS_GPIO_PORTS                         { PIOS_GPIO_1_PORT, PIOS_GPIO_2_PORT, PIOS_GPIO_3_PORT}
#define PIOS_GPIO_PINS                          { PIOS_GPIO_1_PIN, PIOS_GPIO_2_PIN, PIOS_GPIO_3_PIN}
#define PIOS_GPIO_NUM                           3

#define GPIO_EXT1								0
#define GPIO_EXT2								1
#define GPIO_PC8								2

//-------------------------
// USB
//-------------------------
#define PIOS_USB_ENABLED                        1
#define PIOS_USB_DETECT_GPIO_PORT               GPIOA
#define PIOS_USB_DETECT_GPIO_PIN                GPIO_Pin_10
#define PIOS_USB_DETECT_EXTI_LINE               EXTI_Line10
#define PIOS_IRQ_USB_PRIORITY                   PIOS_IRQ_PRIO_MID
#define PIOS_USB_RX_BUFFER_SIZE                 512
#define PIOS_USB_TX_BUFFER_SIZE                 512

/**
 * glue macros for file IO
 * STM32 uses DOSFS for file IO
 */
#define PIOS_FOPEN_READ(filename,file)					DFS_OpenFile(&PIOS_SDCARD_VolInfo, (uint8_t *)filename, DFS_READ, PIOS_SDCARD_Sector, &file) != DFS_OK
#define PIOS_FOPEN_WRITE(filename,file)					DFS_OpenFile(&PIOS_SDCARD_VolInfo, (uint8_t *)filename, DFS_WRITE, PIOS_SDCARD_Sector, &file) != DFS_OK
#define PIOS_FREAD(file,bufferadr,length,resultadr)     DFS_ReadFile(file, PIOS_SDCARD_Sector, (uint8_t*)bufferadr, resultadr, length) != DFS_OK
#define PIOS_FWRITE(file,bufferadr,length,resultadr)    DFS_WriteFile(file, PIOS_SDCARD_Sector, (uint8_t*)bufferadr, resultadr, length)
#define PIOS_FCLOSE(file)               				DFS_Close(&file)
#define PIOS_FUNLINK(filename)          				DFS_UnlinkFile(&PIOS_SDCARD_VolInfo, (uint8_t *)filename, PIOS_SDCARD_Sector)


#endif /* STM32205_PX2FMU_H_ */
/**
 * @}
 * @}
 */
