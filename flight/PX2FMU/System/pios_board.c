/**
 ******************************************************************************
 * @addtogroup OpenPilotSystem OpenPilot System
 * @{
 * @addtogroup OpenPilotCore OpenPilot Core
 * @{
 *
 * @file       pios_board.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      Defines board specific static initializers for hardware for the OpenPilot board.
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

#include <pios.h>
#include <openpilot.h>
#include <uavobjectsinit.h>

#include <pios_usart_priv.h>
#include <pios_spi_priv.h>
#include <pios_com_priv.h>
#include <pios_ppm_priv.h>
#include <pios_adc_priv.h>
#include <pios_i2c_priv.h>
#include <pios_rcvr_priv.h>
#include <pios_buzzer_priv.h>

/* XXX these should be somewhere else */
#define PIOS_COM_TELEM_RF_RX_BUF_LEN	192
#define PIOS_COM_TELEM_RF_TX_BUF_LEN	192
#define PIOS_COM_GPS_RX_BUF_LEN			96
#define PIOS_COM_GPS_TX_BUF_LEN			96    /* FIXME XXX any value below 96 will KILL all communication on that port. Why? */
#define PIOS_COM_AUX_RX_BUF_LEN			96
#define PIOS_COM_AUX_TX_BUF_LEN			96
#define PIOS_COM_CONTROL_RX_BUF_LEN		96
#define PIOS_COM_CONTROL_TX_BUF_LEN		96


/* XXX this should be more comprehensively abstracted */
void PIOS_SPI_main_irq_handler(void);
void DMA2_Stream0_IRQ_Handler(void) __attribute__((alias("PIOS_SPI_main_irq_handler")));
void DMA2_Stream3_IRQ_Handler(void) __attribute__((alias("PIOS_SPI_main_irq_handler")));
static const struct pios_spi_cfg pios_spi_main_cfg = {
    .regs    = SPI1,
    .remap   = GPIO_AF_SPI1,
    .use_crc = false,
    .init    = {
        .SPI_Mode              = SPI_Mode_Master,
        .SPI_Direction         = SPI_Direction_2Lines_FullDuplex,
        .SPI_DataSize          = SPI_DataSize_8b,
        .SPI_NSS               = SPI_NSS_Soft,
        .SPI_FirstBit          = SPI_FirstBit_MSB,
        .SPI_CRCPolynomial     = 7,
        .SPI_CPOL              = SPI_CPOL_High,
        .SPI_CPHA              = SPI_CPHA_2Edge,
        .SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8,
    },
    .dma     = {
        /* .ahb_clk - not required */
        .irq = {
            .flags   = (DMA_IT_TCIF3 | DMA_IT_TEIF3 | DMA_IT_HTIF3),
            .init    = {
                .NVIC_IRQChannel                   = DMA2_Stream0_IRQn,
                .NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_HIGH,
                .NVIC_IRQChannelSubPriority        = 0,
                .NVIC_IRQChannelCmd                = ENABLE,
            },
        },
        .rx = {
            .channel = DMA2_Stream0,
            .init = {
                .DMA_Channel            = DMA_Channel_3,
                .DMA_PeripheralBaseAddr = (uint32_t)&(SPI1->DR),
                /* .DMA_Memory0BaseAddr */
                .DMA_DIR                = DMA_DIR_PeripheralToMemory,
                /* .DMA_BufferSize */
                .DMA_PeripheralInc      = DMA_PeripheralInc_Disable,
                /* .DMA_BufferSize */
                .DMA_PeripheralInc      = DMA_PeripheralInc_Disable,
                .DMA_MemoryInc          = DMA_MemoryInc_Enable,
                .DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte,
                .DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte,
                .DMA_Mode               = DMA_Mode_Normal,
                .DMA_Priority           = DMA_Priority_High,
                .DMA_FIFOMode           = DMA_FIFOMode_Disable,
                /* .DMA_FIFOThreshold */
                .DMA_MemoryBurst        = DMA_MemoryBurst_Single,
                .DMA_PeripheralBurst    = DMA_PeripheralBurst_Single,
            },
        },
        .tx = {
            .channel = DMA2_Stream3,
            .init = {
                .DMA_Channel            = DMA_Channel_3,
                /* .DMA_Memory0BaseAddr */
                .DMA_PeripheralBaseAddr = (uint32_t)&(SPI1->DR),
                .DMA_DIR                = DMA_DIR_MemoryToPeripheral,
                /* .DMA_BufferSize */
                .DMA_PeripheralInc      = DMA_PeripheralInc_Disable,
                .DMA_MemoryInc          = DMA_MemoryInc_Enable,
                .DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte,
                .DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte,
                .DMA_Mode               = DMA_Mode_Normal,
                .DMA_Priority           = DMA_Priority_High,
                .DMA_FIFOMode           = DMA_FIFOMode_Disable,
                /* .DMA_FIFOThreshold */
                .DMA_MemoryBurst        = DMA_MemoryBurst_Single,
                .DMA_PeripheralBurst    = DMA_PeripheralBurst_Single,
            },
        },
    },
    .sclk = {
        .gpio = GPIOA,
        .init = {
            .GPIO_Pin   = GPIO_Pin_5,
            .GPIO_Mode  = GPIO_Mode_AF,
            .GPIO_Speed = GPIO_Speed_100MHz,
            .GPIO_OType = GPIO_OType_PP,
            .GPIO_PuPd  = GPIO_PuPd_UP,
        },
    },
    .miso = {
        .gpio = GPIOA,
        .init = {
            .GPIO_Pin   = GPIO_Pin_6,
            .GPIO_Mode  = GPIO_Mode_AF,
            .GPIO_Speed = GPIO_Speed_50MHz,
            .GPIO_OType = GPIO_OType_PP,
            .GPIO_PuPd  = GPIO_PuPd_UP,
        },
    },
    .mosi = {
        .gpio = GPIOA,
        .init = {
            .GPIO_Pin   = GPIO_Pin_7,
            .GPIO_Mode  = GPIO_Mode_AF,
            .GPIO_Speed = GPIO_Speed_50MHz,
            .GPIO_OType = GPIO_OType_PP,
            .GPIO_PuPd  = GPIO_PuPd_UP,
        },
    },
    .slave_count = 2,
    .ssel = {
        {
            .gpio = GPIOC,
            .init = {
                .GPIO_Pin   = GPIO_Pin_14,
                .GPIO_Mode  = GPIO_Mode_OUT,
                .GPIO_Speed = GPIO_Speed_50MHz,
                .GPIO_OType = GPIO_OType_PP,
                .GPIO_PuPd  = GPIO_PuPd_UP,
            },
        },
        {
            .gpio = GPIOC,
            .init = {
                .GPIO_Pin   = GPIO_Pin_15,
                .GPIO_Mode  = GPIO_Mode_OUT,
                .GPIO_Speed = GPIO_Speed_50MHz,
                .GPIO_OType = GPIO_OType_PP,
                .GPIO_PuPd  = GPIO_PuPd_UP,
            },
        },
    },
};

#if defined(PIOS_INCLUDE_SDCARD)
void PIOS_SPI_sdcard_irq_handler(void);
void DMA1_Stream0_IRQ_Handler(void) __attribute__((alias("PIOS_SPI_sdcard_irq_handler")));
void DMA1_Stream3_IRQ_Handler(void) __attribute__((alias("PIOS_SPI_sdcard_irq_handler")));
static const struct pios_spi_cfg pios_spi_sdcard_cfg = {
	    .regs    = SPI3,
	    .remap   = GPIO_AF_SPI3,
	    .use_crc = true,
	    .init    = {
	        .SPI_Mode              = SPI_Mode_Master,
	        .SPI_Direction         = SPI_Direction_2Lines_FullDuplex,
	        .SPI_DataSize          = SPI_DataSize_8b,
	        .SPI_NSS               = SPI_NSS_Soft,
	        .SPI_FirstBit          = SPI_FirstBit_MSB,
	        .SPI_CRCPolynomial     = 7,
	        .SPI_CPOL              = SPI_CPOL_High,
	        .SPI_CPHA              = SPI_CPHA_2Edge,
	        .SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256,
	    },
	    .dma     = {
	        /* .ahb_clk - not required */
	        .irq = {
	            .flags   = (DMA_IT_TCIF0 | DMA_IT_TEIF0 | DMA_IT_HTIF5),
	            .init    = {
	                .NVIC_IRQChannel                   = DMA1_Stream0_IRQn,
	                .NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_HIGH,
	                .NVIC_IRQChannelSubPriority        = 0,
	                .NVIC_IRQChannelCmd                = ENABLE,
	            },
	        },
	        .rx = {
	            .channel = DMA1_Stream0,
	            .init = {
	                .DMA_Channel            = DMA_Channel_0,
	                .DMA_PeripheralBaseAddr = (uint32_t)&(SPI3->DR),
	                /* .DMA_Memory0BaseAddr */
	                .DMA_DIR                = DMA_DIR_PeripheralToMemory,
	                /* .DMA_BufferSize */
	                .DMA_PeripheralInc      = DMA_PeripheralInc_Disable,
	                /* .DMA_BufferSize */
	                .DMA_PeripheralInc      = DMA_PeripheralInc_Disable,
	                .DMA_MemoryInc          = DMA_MemoryInc_Enable,
	                .DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte,
	                .DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte,
	                .DMA_Mode               = DMA_Mode_Normal,
	                .DMA_Priority           = DMA_Priority_High,
	                .DMA_FIFOMode           = DMA_FIFOMode_Disable,
	                /* .DMA_FIFOThreshold */
	                .DMA_MemoryBurst        = DMA_MemoryBurst_Single,
	                .DMA_PeripheralBurst    = DMA_PeripheralBurst_Single,
	            },
	        },
	        .tx = {
	            .channel = DMA1_Stream7,
	            .init = {
	                .DMA_Channel            = DMA_Channel_0,
	                /* .DMA_Memory0BaseAddr */
	                .DMA_PeripheralBaseAddr = (uint32_t)&(SPI3->DR),
	                .DMA_DIR                = DMA_DIR_MemoryToPeripheral,
	                /* .DMA_BufferSize */
	                .DMA_PeripheralInc      = DMA_PeripheralInc_Disable,
	                .DMA_MemoryInc          = DMA_MemoryInc_Enable,
	                .DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte,
	                .DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte,
	                .DMA_Mode               = DMA_Mode_Normal,
	                .DMA_Priority           = DMA_Priority_High,
	                .DMA_FIFOMode           = DMA_FIFOMode_Disable,
	                /* .DMA_FIFOThreshold */
	                .DMA_MemoryBurst        = DMA_MemoryBurst_Single,
	                .DMA_PeripheralBurst    = DMA_PeripheralBurst_Single,
	            },
	        },
	    },
	    .sclk = {
	        .gpio = GPIOC,
	        .init = {
	            .GPIO_Pin   = GPIO_Pin_10,
	            .GPIO_Mode  = GPIO_Mode_AF,
	            .GPIO_Speed = GPIO_Speed_100MHz,
	            .GPIO_OType = GPIO_OType_PP,
	            .GPIO_PuPd  = GPIO_PuPd_UP,
	        },
	    },
	    .miso = {
	        .gpio = GPIOC,
	        .init = {
	            .GPIO_Pin   = GPIO_Pin_11,
	            .GPIO_Mode  = GPIO_Mode_AF,
	            .GPIO_Speed = GPIO_Speed_50MHz,
	            .GPIO_OType = GPIO_OType_PP,
	            .GPIO_PuPd  = GPIO_PuPd_UP,
	        },
	    },
	    .mosi = {
	        .gpio = GPIOB,
	        .init = {
	            .GPIO_Pin   = GPIO_Pin_5,
	            .GPIO_Mode  = GPIO_Mode_AF,
	            .GPIO_Speed = GPIO_Speed_50MHz,
	            .GPIO_OType = GPIO_OType_PP,
	            .GPIO_PuPd  = GPIO_PuPd_UP,
	        },
	    },
	    .slave_count = 1,
	    .ssel = {
	        {
	            .gpio = GPIOA,
	            .init = {
	                .GPIO_Pin   = GPIO_Pin_4,
	                .GPIO_Mode  = GPIO_Mode_OUT,
	                .GPIO_Speed = GPIO_Speed_50MHz,
	                .GPIO_OType = GPIO_OType_PP,
	                .GPIO_PuPd  = GPIO_PuPd_UP,
	            },
	        },
	    },
	};

uint32_t pios_spi_sdcard_id;
void PIOS_SPI_sdcard_irq_handler(void)
{
  /* Call into the generic code to handle the IRQ for this specific device */
	PIOS_SPI_IRQ_Handler(pios_spi_sdcard_id);
}
#endif

#if defined(PIOS_INCLUDE_BUZZER) && !defined(PIOS_INCLUDE_SERVO)
const struct pios_buzzer_cfg pios_buzzer_cfg = {
	.tim_base_init = {
		.TIM_Prescaler = ((PIOS_MASTER_CLOCK /2) / 20000000) - 1,	// TIM5 Frequency is 20 MHz
		.TIM_CounterMode = TIM_CounterMode_CenterAligned1,
		.TIM_ClockDivision = TIM_CKD_DIV1,
		.TIM_Period = 57333,
		.TIM_RepetitionCounter = 0						//not used (only TIM1 & TIM8)
	},
	.tim_oc_init = {
		.TIM_OCMode = TIM_OCMode_PWM1,
		.TIM_OutputState = TIM_OutputState_Enable,
		.TIM_Pulse = 57333/2,
		.TIM_OCPolarity = TIM_OCPolarity_High,
		.TIM_OutputNState = TIM_OutputNState_Disable,	//not used (only TIM1 & TIM8)
		.TIM_OCNPolarity = TIM_OCPolarity_High,			//not used (only TIM1 & TIM8)
		.TIM_OCIdleState = TIM_OCIdleState_Reset,		//not used (only TIM1 & TIM8)
		.TIM_OCNIdleState = TIM_OCNIdleState_Reset		//not used (only TIM1 & TIM8)
	},
	.gpio_init = {
		.GPIO_Pin   = GPIO_Pin_1,
		.GPIO_Mode  = GPIO_Mode_AF,
		.GPIO_Speed = GPIO_Speed_2MHz,
		.GPIO_OType = GPIO_OType_PP,
		.GPIO_PuPd  = GPIO_PuPd_NOPULL,
	},
	.timer = TIM5,
	.channel = TIM_Channel_2,
	.port = GPIOA,						//use PA1 - USART2_RTS is not available!
	.pin_source = GPIO_PinSource1,
	.af = GPIO_AF_TIM5
};
#endif

uint32_t pios_spi_main_id;
void PIOS_SPI_main_irq_handler(void)
{
  /* Call into the generic code to handle the IRQ for this specific device */
	PIOS_SPI_IRQ_Handler(pios_spi_main_id);
}



/*
 * ADC system
 */
void PIOS_ADC_handler(void)
{
	PIOS_ADC_DMA_Handler();
}
void DMA2_Stream4_IRQHandler() __attribute__ ((alias("PIOS_ADC_handler")));
const struct pios_adc_cfg pios_adc_cfg = {
	.dma = {
		.irq = {
			.flags   = (DMA_FLAG_TCIF4 | DMA_FLAG_TEIF4 | DMA_FLAG_HTIF4),
			.init    = {
				.NVIC_IRQChannel		= DMA2_Stream4_IRQn
			},
		},
		/* XXX there is secret knowledge here regarding the use of ADC1 by the pios_adc code */
		.rx = {
			.channel = DMA2_Stream4,	// stream0 may be used by SPI1
			.init    = {
				.DMA_Channel			= DMA_Channel_0,
				.DMA_PeripheralBaseAddr	= (uint32_t) & ADC1->DR
			},
		}
	}, 
	.half_flag = DMA_IT_HTIF4,
	.full_flag = DMA_IT_TCIF4,
};

struct pios_adc_dev pios_adc_devs[] = {
	{
		.cfg = &pios_adc_cfg,
		.callback_function = NULL,
		.data_queue = NULL
	},
};

uint8_t pios_adc_num_devices = NELEMENTS(pios_adc_devs);

/*
 * TELEMETRY1 USART
 */
const struct pios_usart_cfg pios_usart_telem_cfg = USART1_CONFIG(PIOS_COM_TELEM_BAUDRATE);

/*
 * GPS USART
 */
const struct pios_usart_cfg pios_usart_gps_cfg = USART6_CONFIG(PIOS_COM_GPS_BAUDRATE);

/*
 * TELEMETRY2 / AUX USART
 */
const struct pios_usart_cfg pios_usart_aux_cfg = USART2_CONFIG(PIOS_COM_AUX_BAUDRATE);

/*
 * TELEMETRY3 / CONTROL USART
 */
const struct pios_usart_cfg pios_usart_control_cfg = UART5_CONFIG(PIOS_COM_CONTROL_BAUDRATE);


/*
 * PPM Input
 */
#if defined(PIOS_INCLUDE_PPM)
void TIM1_CC_IRQHandler();
void TIM1_CC_IRQHandler() __attribute__ ((alias ("PIOS_TIM1_CC_irq_handler")));
const struct pios_ppm_cfg pios_ppm_cfg = {
	.tim_base_init = {
		.TIM_Prescaler = (PIOS_MASTER_CLOCK / 1000000) - 1,
		.TIM_ClockDivision = TIM_CKD_DIV1,
		.TIM_CounterMode = TIM_CounterMode_Up,
		.TIM_Period = 0xFFFF,
		.TIM_RepetitionCounter = 0x0000,
	},
	.tim_ic_init = {
			.TIM_ICPolarity = TIM_ICPolarity_Rising,
			.TIM_ICSelection = TIM_ICSelection_DirectTI,
			.TIM_ICPrescaler = TIM_ICPSC_DIV1,
			.TIM_ICFilter = 0x0,
			.TIM_Channel = TIM_Channel_2,
	},
	.remap = GPIO_AF_TIM1,
	.port = GPIOA,
	.gpio_init = {
			.GPIO_Pin   = GPIO_Pin_9,
			.GPIO_Mode  = GPIO_Mode_AF,
			.GPIO_Speed = GPIO_Speed_2MHz,
			.GPIO_OType = GPIO_OType_OD,
			.GPIO_PuPd  = GPIO_PuPd_UP
	},
	.irq = {
		.init    = {
			.NVIC_IRQChannel                   = TIM1_CC_IRQn,
			.NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_MID,
			.NVIC_IRQChannelSubPriority        = 0,
			.NVIC_IRQChannelCmd                = ENABLE,
		},
	},
	.timer = PIOS_PPM_TIM,
	.ccr = PIOS_PPM_TIM_CCR,
};

void PIOS_TIM1_CC_irq_handler()
{
	PIOS_PPM_irq_handler();
}

#endif //PPM



// PWM / Servo output
#ifdef PIOS_INCLUDE_SERVO
/*
 * Servo outputs
 */
#include <pios_servo_priv.h>
static const struct pios_servo_channel pios_servo_channels[] = {
	{
		.timer = TIM5,
		.channel = TIM_Channel_1,
		.pin = GPIO_Pin_0,
		.pin_source = GPIO_PinSource0,
		.port = GPIOA,
	},
	{
		.timer = TIM5,
		.channel = TIM_Channel_2,
		.pin = GPIO_Pin_1,
		.pin_source = GPIO_PinSource1,
		.port = GPIOA,
	},
	{
		.timer = TIM5,
		.channel = TIM_Channel_3,
		.pin = GPIO_Pin_2,
		.pin_source = GPIO_PinSource2,
		.port = GPIOA,
	},
	{
		.timer = TIM5,
		.channel = TIM_Channel_4,
		.pin = GPIO_Pin_3,
		.pin_source = GPIO_PinSource3,
		.port = GPIOA,
	}
};

const struct pios_servo_cfg pios_servo_cfg = {
	.tim_base_init = {
		/* Prescaler will be set in pios_servo.c */
		.TIM_ClockDivision = TIM_CKD_DIV1,
		.TIM_CounterMode = TIM_CounterMode_Up,
		.TIM_Period = ((1000000 / PIOS_SERVO_UPDATE_HZ) - 1),
		.TIM_RepetitionCounter = 0x0000,
	},
	.tim_oc_init = {
		.TIM_OCMode = TIM_OCMode_PWM1,
		.TIM_OutputState = TIM_OutputState_Enable,
		.TIM_OutputNState = TIM_OutputNState_Disable,
		.TIM_Pulse = PIOS_SERVOS_INITIAL_POSITION,
		.TIM_OCPolarity = TIM_OCPolarity_High,
		.TIM_OCNPolarity = TIM_OCPolarity_High,
		.TIM_OCIdleState = TIM_OCIdleState_Reset,
		.TIM_OCNIdleState = TIM_OCNIdleState_Reset,
	},
	.gpio_init = {
		.GPIO_Mode = GPIO_Mode_AF,
		.GPIO_OType = GPIO_OType_PP,
		.GPIO_PuPd  = GPIO_PuPd_NOPULL,
		.GPIO_Speed = GPIO_Speed_2MHz,
	},
	.remap = GPIO_AF_TIM5,
	.channels = pios_servo_channels,
	.num_channels = NELEMENTS(pios_servo_channels),
};
#endif



/*
 * I2C Adapters
 */

/* XXX this should be more comprehensively abstracted */

/* I2C ESCs connected via the multi-connector */
void PIOS_I2C_esc_adapter_ev_irq_handler(void);
void PIOS_I2C_esc_adapter_er_irq_handler(void);
void I2C1_EV_IRQHandler() __attribute__ ((alias ("PIOS_I2C_esc_adapter_ev_irq_handler")));
void I2C1_ER_IRQHandler() __attribute__ ((alias ("PIOS_I2C_esc_adapter_er_irq_handler")));

const struct pios_i2c_adapter_cfg pios_i2c_esc_adapter_cfg = I2C1_CONFIG();

uint32_t pios_i2c_esc_adapter_id;
void PIOS_I2C_esc_adapter_ev_irq_handler(void)
{
	/* Call into the generic code to handle the IRQ for this specific device */
	PIOS_I2C_EV_IRQ_Handler(pios_i2c_esc_adapter_id);
}

void PIOS_I2C_esc_adapter_er_irq_handler(void)
{
  /* Call into the generic code to handle the IRQ for this specific device */
  PIOS_I2C_ER_IRQ_Handler(pios_i2c_esc_adapter_id);
}

/* Onboard sensor bus + IO comms */
void PIOS_I2C_main_adapter_ev_irq_handler(void);
void PIOS_I2C_main_adapter_er_irq_handler(void);
void I2C2_EV_IRQHandler() __attribute__ ((alias ("PIOS_I2C_main_adapter_ev_irq_handler")));
void I2C2_ER_IRQHandler() __attribute__ ((alias ("PIOS_I2C_main_adapter_er_irq_handler")));

const struct pios_i2c_adapter_cfg pios_i2c_main_adapter_cfg = I2C2_CONFIG();

uint32_t pios_i2c_main_adapter_id;
void PIOS_I2C_main_adapter_ev_irq_handler(void)
{
	/* Call into the generic code to handle the IRQ for this specific device */
	PIOS_I2C_EV_IRQ_Handler(pios_i2c_main_adapter_id);
}

void PIOS_I2C_main_adapter_er_irq_handler(void)
{
  /* Call into the generic code to handle the IRQ for this specific device */
  PIOS_I2C_ER_IRQ_Handler(pios_i2c_main_adapter_id);
}

/* Off-board sensor bus */
void PIOS_I2C_external_adapter_ev_irq_handler(void);
void PIOS_I2C_external_adapter_er_irq_handler(void);
void I2C3_EV_IRQHandler() __attribute__ ((alias ("PIOS_I2C_external_adapter_ev_irq_handler")));
void I2C3_ER_IRQHandler() __attribute__ ((alias ("PIOS_I2C_external_adapter_er_irq_handler")));

const struct pios_i2c_adapter_cfg pios_i2c_external_adapter_cfg = I2C3_CONFIG();

uint32_t pios_i2c_external_adapter_id;
void PIOS_I2C_external_adapter_ev_irq_handler(void)
{
	/* Call into the generic code to handle the IRQ for this specific device */
	PIOS_I2C_EV_IRQ_Handler(pios_i2c_external_adapter_id);
}

void PIOS_I2C_external_adapter_er_irq_handler(void)
{
  /* Call into the generic code to handle the IRQ for this specific device */
  PIOS_I2C_ER_IRQ_Handler(pios_i2c_external_adapter_id);
}

struct pios_rcvr_channel_map pios_rcvr_channel_to_id_map[PIOS_RCVR_MAX_CHANNELS];
uint32_t pios_rcvr_max_channel;

/*
 * COM interfaces
 */

extern const struct pios_com_driver pios_usb_com_driver;

uint32_t pios_com_telem_rf_id;
uint32_t pios_com_telem_usb_id;
uint32_t pios_com_gps_id;
uint32_t pios_com_aux_id;
uint32_t pios_com_control_id;

/*
 * Do the allocation work that PIOS_COM_Init is too lazy to do for itself.
 */
static void
usart_init(uint32_t *id, const struct pios_usart_cfg * cfg, size_t rx_buffer, size_t tx_buffer)
{
	uint32_t	usart_id;
	void		*rx = NULL, *tx = NULL;

	PIOS_Assert(!PIOS_USART_Init(&usart_id, cfg));
	if (rx_buffer)
		PIOS_Assert((rx = pvPortMalloc(rx_buffer)));
	if (tx_buffer)
		PIOS_Assert((tx = pvPortMalloc(tx_buffer)));
	PIOS_Assert(!PIOS_COM_Init(id, &pios_usart_com_driver, usart_id, rx, rx_buffer, tx, tx_buffer));
}


/**
 * PIOS_Board_Init()
 * initializes all the core subsystems on this specific hardware
 * called from System/openpilot.c
 */
void PIOS_Board_Init(void)
{
	uint32_t	rcvr_id;

	/* Initialise USARTs */
	usart_init(&pios_com_telem_rf_id, &pios_usart_telem_cfg,   PIOS_COM_TELEM_RF_RX_BUF_LEN, PIOS_COM_TELEM_RF_TX_BUF_LEN);
	usart_init(&pios_com_gps_id,      &pios_usart_gps_cfg,     PIOS_COM_GPS_RX_BUF_LEN,      PIOS_COM_GPS_TX_BUF_LEN);
#ifndef PIOS_INCLUDE_SERVO
	// USART2 and SERVO outputs are on the same four pins
	usart_init(&pios_com_aux_id,      &pios_usart_aux_cfg,     PIOS_COM_AUX_RX_BUF_LEN,      PIOS_COM_AUX_TX_BUF_LEN);
#endif
	usart_init(&pios_com_control_id,  &pios_usart_control_cfg, PIOS_COM_CONTROL_RX_BUF_LEN,  PIOS_COM_CONTROL_TX_BUF_LEN);


	PIOS_COM_SendString(PIOS_COM_DEBUG, "\r\n\r\nFMU starting: USART ");

	/* Internal settings support - must come up before UAVO */
	if (PIOS_I2C_Init(&pios_i2c_main_adapter_id, &pios_i2c_main_adapter_cfg)) {
		PIOS_DEBUG_Assert(0);
	}
	PIOS_EEPROM_Attach(PIOS_I2C_MAIN_ADAPTER, PIOS_I2C_EEPROM_ADDRESS);
	PIOS_EEPROM_Init();
	PIOS_FLASHFS_Init(&PIOS_EEPROM_Driver);

	/* Initialize UAVObject libraries */
	EventDispatcherInitialize();
	UAVObjInitialize();

	/* Initialize the alarms library */
	AlarmsInitialize();

	/* Initialize the task monitor library */
	TaskMonitorInitialize();

	/* GPIO init */
	PIOS_GPIO_Init();

	/* Initialise SPI */
	if (PIOS_SPI_Init(&pios_spi_main_id, &pios_spi_main_cfg)) {
		PIOS_DEBUG_Assert(0);
	}
	PIOS_L3G4200_Attach(pios_spi_main_id);
	PIOS_LIS331_Attach(pios_spi_main_id);

	/* sdcard init */
#if defined(PIOS_INCLUDE_SDCARD)
//	if (PIOS_SPI_Init(&pios_spi_sdcard_id, &pios_spi_sdcard_cfg)) {
//		PIOS_DEBUG_Assert(0);
//	}
//	PIOS_SDCARD_Init(pios_spi_sdcard_id);
//
//	FILEINFO file;
//	uint32_t res;
//
//	res = PIOS_SDCARD_MountFS(0);
//	PIOS_COM_SendFormattedString(PIOS_COM_DEBUG, "PIOS_SDCARD_MountFS Res: %d\n", (int) res);
//
//	res = DFS_OpenFile(&PIOS_SDCARD_VolInfo, (uint8_t *)("test/test.txt"), DFS_READ, PIOS_SDCARD_Sector, &file);
//	PIOS_COM_SendFormattedString(PIOS_COM_DEBUG, "DFS_OpenFile Res: %u\n", (unsigned int) res);
//
//	static uint8_t buffer[200];
//	if(!PIOS_FOPEN_READ("test/test.txt", file))
//	{
//		PIOS_SDCARD_ReadLine(&file, buffer, 199);
//	}
//	PIOS_COM_SendFormattedString(PIOS_COM_DEBUG, "%s\n", (char *)buffer);

#endif

	PIOS_COM_SendString(PIOS_COM_DEBUG, "SPI ");

	if (PIOS_I2C_Init(&pios_i2c_esc_adapter_id, &pios_i2c_esc_adapter_cfg)) {
		PIOS_DEBUG_Assert(0);
	}

	if (PIOS_I2C_Init(&pios_i2c_external_adapter_id, &pios_i2c_external_adapter_cfg)) {
		PIOS_DEBUG_Assert(0);
	}

	PIOS_COM_SendString(PIOS_COM_DEBUG, "I2C ");
	// Clear I2C alarm
	AlarmsClear(SYSTEMALARMS_ALARM_I2C);
	PIOS_ADC_Init();
	PIOS_COM_SendString(PIOS_COM_DEBUG, "ADC ");

	// XXX in theory we need to do this... but why?  This is crap.  It should be somewhere very far from here.
	//uint8_t inputmode;
	//ManualControlSettingsInputModeGet(&inputmode);
	PIOS_PPM_Init();
	if (PIOS_RCVR_Init(&rcvr_id, &pios_ppm_rcvr_driver, 0)) {
		PIOS_Assert(0);
	}
	for (uint8_t i = 0; i < PIOS_PPM_NUM_INPUTS && pios_rcvr_max_channel < NELEMENTS(pios_rcvr_channel_to_id_map); i++) {
		pios_rcvr_channel_to_id_map[pios_rcvr_max_channel].id = rcvr_id;
		pios_rcvr_channel_to_id_map[pios_rcvr_max_channel].channel = i;
		pios_rcvr_max_channel++;
	}

	PIOS_COM_SendString(PIOS_COM_DEBUG, "PPM ");

#if defined(PIOS_INCLUDE_USB_HID)
	PIOS_USB_HID_Init(0);
#if defined(PIOS_INCLUDE_COM)
	if (PIOS_COM_Init(&pios_com_telem_usb_id, &pios_usb_com_driver, 0)) {
		PIOS_DEBUG_Assert(0);
	}
	PIOS_COM_SendString(PIOS_COM_DEBUG, "USB ");
#endif	/* PIOS_INCLUDE_COM */
#endif  /* PIOS_INCLUDE_USB_HID */

	PIOS_IAP_Init();

#if defined(PIOS_INCLUDE_WDG)
	PIOS_WDG_Init();
#endif /* PIOS_INCLUDE_WDG */

#if defined(PIOS_INCLUDE_BUZZER) && !defined(PIOS_INCLUDE_SERVO)
	PIOS_Buzzer_Init();
#endif

#if defined(PIOS_INCLUDE_SERVO)
	PIOS_Servo_Init();
#endif

	PIOS_COM_SendString(PIOS_COM_DEBUG, "Hardware init done.\r\n");
}

/**
 * @}
 */
