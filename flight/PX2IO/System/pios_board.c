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
#include <pios_com_priv.h>
#include <pios_ppm_priv.h>
#include <pios_servo_priv.h>
#include <pios_spektrum_priv.h>
#include <pios_usart_priv.h>
#include <pios_rtc_priv.h>
#include <pios_rcvr_priv.h>
#include <pios_i2c_slave.h>

/*
 * Clocking
 */
const struct pios_clock_cfg px2io_clock_config = {
		.source				= RCC_PLLSource_PREDIV1,
		.refclock_frequency = HSE_VALUE,
		.refclock_prescale	= RCC_PREDIV1_Div12,
		.pll_multiply		= RCC_PLLMul_12,
		.hclk_prescale		= RCC_SYSCLK_Div1,
		.pclk1_prescale		= RCC_HCLK_Div1,
		.pclk2_prescale		= RCC_HCLK_Div1,
		.adc_prescale		= RCC_PCLK2_Div2,
};

/*
 * AUX USART
 */
const struct pios_usart_cfg pios_usart_aux_cfg = {
  .regs = USART1,
  .init = {
	.USART_BaudRate            = PIOS_COM_AUX_BAUDRATE,
    .USART_WordLength          = USART_WordLength_8b,
    .USART_Parity              = USART_Parity_No,
    .USART_StopBits            = USART_StopBits_1,
    .USART_HardwareFlowControl = USART_HardwareFlowControl_None,
    .USART_Mode                = USART_Mode_Rx | USART_Mode_Tx,
  },
  .irq = {
    .init    = {
      .NVIC_IRQChannel                   = USART1_IRQn,
      .NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_MID,
      .NVIC_IRQChannelSubPriority        = 0,
      .NVIC_IRQChannelCmd                = ENABLE,
    },
  },
  .rx   = {
    .gpio = GPIOA,
    .init = {
      .GPIO_Pin   = GPIO_Pin_10,
      .GPIO_Speed = GPIO_Speed_2MHz,
      .GPIO_Mode  = GPIO_Mode_IPU,
    },
  },
  .tx   = {
    .gpio = GPIOA,
    .init = {
      .GPIO_Pin   = GPIO_Pin_9,
      .GPIO_Speed = GPIO_Speed_2MHz,
      .GPIO_Mode  = GPIO_Mode_AF_PP,
    },
  },
};

#if PIOS_INCLUDE_SPEKTRUM
# error Need Spektrum config
#endif

/**
 * Pios servo configuration structures
 *
 * Note: early prototypes do not have the last two channels available,
 * as they are routed elsewhere (as ADC inputs).
 */
const struct pios_servo_channel pios_servo_channels[] = {
	{	// channel 1
		.timer = TIM2,
		.port = GPIOA,
		.channel = TIM_Channel_1,
		.pin = GPIO_Pin_0,
	}, 
	{	// channel 2
		.timer = TIM2,
		.port = GPIOA,
		.channel = TIM_Channel_2,
		.pin = GPIO_Pin_1,
	}, 
	{	// channel 3
		.timer = TIM16,
		.port = GPIOB,
		.channel = TIM_Channel_1,
		.pin = GPIO_Pin_8,
	}, 
	{	// channel 4
		.timer = TIM17,
		.port = GPIOB,
		.channel = TIM_Channel_1,
		.pin = GPIO_Pin_9,
	}, 
	{	// channel 5
		.timer = TIM3,
		.port = GPIOA,
		.channel = TIM_Channel_1,
		.pin = GPIO_Pin_6,
	}, 
	{	// channel 6
		.timer = TIM3,
		.port = GPIOA,
		.channel = TIM_Channel_2,
		.pin = GPIO_Pin_7,
	}, 
	{	// channel 7
		.timer = TIM3,
		.port = GPIOB,
		.channel = TIM_Channel_3,
		.pin = GPIO_Pin_0,
	}, 
	{	// channel 8
		.timer = TIM3,
		.port = GPIOB,
		.channel = TIM_Channel_4,
		.pin = GPIO_Pin_1,
	}, 	
};

const struct pios_servo_cfg pios_servo_cfg = {
	.tim_base_init = {
		.TIM_Prescaler = (PIOS_MASTER_CLOCK / 1000000) - 1,
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
		.GPIO_Mode = GPIO_Mode_AF_PP,
		.GPIO_Speed = GPIO_Speed_2MHz,
	},
	.remap = 0,
	.channels = pios_servo_channels,
	.num_channels = NELEMENTS(pios_servo_channels),
};

#if defined(PIOS_INCLUDE_PPM)
/*
 * PPM Input
 */
void TIM1_CC_IRQHandler();
void TIM1_CC_IRQHandler() __attribute__ ((alias ("PIOS_TIM1_CC_irq_handler")));
const struct pios_ppm_cfg pios_ppm_cfg = {
	.tim_base_init = {
		.TIM_Prescaler = (PIOS_MASTER_CLOCK / 1000000) - 1,	/* For 1 uS accuracy */
		.TIM_ClockDivision = TIM_CKD_DIV1,
		.TIM_CounterMode = TIM_CounterMode_Up,
		.TIM_Period = 0xFFFF,
		.TIM_RepetitionCounter = 0x0000,
	},
	.tim_ic_init = {
			.TIM_Channel = TIM_Channel_2,
			.TIM_ICPolarity = TIM_ICPolarity_Rising,
			.TIM_ICSelection = TIM_ICSelection_DirectTI,
			.TIM_ICPrescaler = TIM_ICPSC_DIV1,
			.TIM_ICFilter = 0x0,
	},
	.gpio_init = {
			.GPIO_Pin = GPIO_Pin_9,
			.GPIO_Mode = GPIO_Mode_IPD,
			.GPIO_Speed = GPIO_Speed_2MHz,
	},
	.remap = 0,
	.irq = {
		.init    = {
			.NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_MID,
			.NVIC_IRQChannelSubPriority        = 0,
			.NVIC_IRQChannelCmd                = ENABLE,
		},
	},
	.timer = TIM1,
	.port = GPIOA,
	.ccr = TIM_IT_CC2,
};

void PIOS_TIM1_CC_irq_handler()
{
	PIOS_PPM_irq_handler();
}
#endif

#if defined(PIOS_INCLUDE_RTC)
/*
 * Realtime Clock (RTC)
 */
#include <pios_rtc_priv.h>

void PIOS_RTC_IRQ_Handler (void);
void RTC_IRQHandler() __attribute__ ((alias ("PIOS_RTC_IRQ_Handler")));
static const struct pios_rtc_cfg pios_rtc_main_cfg = {
	.clksrc = RCC_RTCCLKSource_HSE_Div128,
	.prescaler = 100,
	.irq = {
		.init = {
			.NVIC_IRQChannel                   = RTC_IRQn,
			.NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_MID,
			.NVIC_IRQChannelSubPriority        = 0,
			.NVIC_IRQChannelCmd                = ENABLE,
		  },
	},
};

void PIOS_RTC_IRQ_Handler (void)
{
	PIOS_RTC_irq_handler ();
}

#endif


/*
 * I2C slave interface
 */
static uint32_t	pios_i2c_slave_id = 0;
static void
I2C_SLAVE_EV_IRQ_Handler(void)
{
	PIOS_I2C_SLAVE_EV_IRQ_Handler(pios_i2c_slave_id);
}
void I2C1_EV_IRQHandler() __attribute__((alias ("I2C_SLAVE_EV_IRQ_Handler")));

static void
I2C_SLAVE_ER_IRQ_Handler(void)
{
	PIOS_I2C_SLAVE_ER_IRQ_Handler(pios_i2c_slave_id);
}
void I2C1_ER_IRQHandler() __attribute__((alias ("I2C_SLAVE_ER_IRQ_Handler")));

static const struct pios_i2c_adapter_cfg i2c_slave_cfg = {
		.regs	= I2C1,
		.init = {
				.I2C_ClockSpeed				= 400000,
				.I2C_Mode					= I2C_Mode_I2C,
				.I2C_DutyCycle				= I2C_DutyCycle_2,
				.I2C_OwnAddress1			= 0x10,
				.I2C_Ack					= I2C_Ack_Enable,
				.I2C_AcknowledgedAddress	= I2C_AcknowledgedAddress_7bit,

		},
		.scl = {
				.gpio = GPIOB,
				.init = {
						.GPIO_Pin   = GPIO_Pin_6,
						.GPIO_Speed = GPIO_Speed_10MHz,
						.GPIO_Mode  = GPIO_Mode_AF_OD,
				},
		},
		.sda = {
				.gpio = GPIOB,
				.init = {
						.GPIO_Pin   = GPIO_Pin_7,
						.GPIO_Speed = GPIO_Speed_10MHz,
						.GPIO_Mode  = GPIO_Mode_AF_OD,
				},
		},
		.event = {
				.init = {
						.NVIC_IRQChannel                   = I2C1_EV_IRQn,
						.NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_HIGHEST,
						.NVIC_IRQChannelSubPriority        = 0,
						.NVIC_IRQChannelCmd                = ENABLE,
				},
		},
		.error = {
				.init = {
						.NVIC_IRQChannel                   = I2C1_ER_IRQn,
						.NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_HIGHEST,
						.NVIC_IRQChannelSubPriority        = 0,
						.NVIC_IRQChannelCmd                = ENABLE,
				},
		},
};

uint32_t pios_com_aux_id;
#if defined(PIOS_INCLUDE_SPEKTRUM)
uint32_t pios_com_spektrum_id;
#endif

struct pios_rcvr_channel_map pios_rcvr_channel_to_id_map[PIOS_RCVR_MAX_CHANNELS];
uint32_t pios_rcvr_max_channel;

/**
 * PIOS_Board_Init()
 */
void PIOS_Board_Init(void)
{

	/* Delay system */
	PIOS_DELAY_Init();	

	/* Initialize UAVObject libraries */
//	EventDispatcherInitialize();
//	UAVObjInitialize();
//	HwSettingsInitialize();
//	ManualControlSettingsInitialize();

#if defined(PIOS_INCLUDE_RTC)
	/* Initialize the real-time clock and its associated tick */
	PIOS_RTC_Init(&pios_rtc_main_cfg);
#endif
	
	/* Initialize the alarms library */
//	AlarmsInitialize();

	/* Initialize the task monitor library */
//	TaskMonitorInitialize();

#if 0 // XXX this is all very wrong now
#if defined(PIOS_INCLUDE_SBUS)
		{
			uint32_t pios_usart_sbus_id;
			if (PIOS_USART_Init(&pios_usart_sbus_id, &pios_usart_sbus_main_cfg)) {
				PIOS_Assert(0);
			}

			uint32_t pios_sbus_id;
			if (PIOS_SBUS_Init(&pios_sbus_id, &pios_sbus_cfg, &pios_usart_com_driver, pios_usart_sbus_id)) {
				PIOS_Assert(0);
			}
		}
#endif	/* PIOS_INCLUDE_SBUS */
#if defined(PIOS_INCLUDE_SPEKTRUM)
		{
			uint32_t pios_usart_spektrum_id;
			if (PIOS_USART_Init(&pios_usart_spektrum_id, &pios_usart_spektrum_main_cfg)) {
				PIOS_Assert(0);
			}

			uint32_t pios_spektrum_id;
			if (PIOS_SPEKTRUM_Init(&pios_spektrum_id, &pios_spektrum_main_cfg, &pios_usart_com_driver, pios_usart_spektrum_id, 0)) {
				PIOS_Assert(0);
			}
		}
#endif	/* PIOS_INCLUDE_SPEKTRUM */
#endif

#if defined(PIOS_COM_AUX)
	uint32_t pios_usart_aux_id;
	if (PIOS_USART_Init(&pios_usart_aux_id, &pios_usart_aux_cfg)) {
		PIOS_DEBUG_Assert(0);
	}
	static uint8_t rx_buffer[128];
	static uint8_t tx_buffer[128];

	if (PIOS_COM_Init(&pios_com_aux_id, &pios_usart_com_driver, pios_usart_aux_id,
			  rx_buffer, 128,
			  tx_buffer, 128)) {
		PIOS_Assert(0);
	}
#endif
	PIOS_COM_SendFormattedString(PIOS_COM_AUX, "PX2IO starting...\r\n");

	/* Bring up the I2C slave interface */
	PIOS_I2C_Slave_Init(pios_i2c_slave_id, &i2c_slave_cfg);

	PIOS_Servo_Init();
//	PIOS_ADC_INIT();
	PIOS_GPIO_Init();

	/* Configure the selected receiver */
	/* XXX we don't have these settings until we have config from FMU ... */
#if defined(PIOS_INCLUDE_PPM)
#if !defined(PIOS_INCLUDE_RTC)
# error PPM requires RTC
#endif
		PIOS_PPM_Init();
		uint32_t pios_ppm_rcvr_id;
		if (PIOS_RCVR_Init(&pios_ppm_rcvr_id, &pios_ppm_rcvr_driver, 0)) {
			PIOS_Assert(0);
		}
		for (uint8_t i = 0;
		     i < PIOS_PPM_NUM_INPUTS && pios_rcvr_max_channel < NELEMENTS(pios_rcvr_channel_to_id_map);
		     i++) {
			pios_rcvr_channel_to_id_map[pios_rcvr_max_channel].id = pios_ppm_rcvr_id;
			pios_rcvr_channel_to_id_map[pios_rcvr_max_channel].channel = i;
			pios_rcvr_max_channel++;
		}
#endif	/* PIOS_INCLUDE_PPM */
#if defined(PIOS_INCLUDE_SPEKTRUM)
		if (hwsettings_cc_mainport == HWSETTINGS_CC_MAINPORT_SPEKTRUM ||
		    hwsettings_cc_flexiport == HWSETTINGS_CC_FLEXIPORT_SPEKTRUM) {
			uint32_t pios_spektrum_rcvr_id;
			if (PIOS_RCVR_Init(&pios_spektrum_rcvr_id, &pios_spektrum_rcvr_driver, 0)) {
				PIOS_Assert(0);
			}
			for (uint8_t i = 0;
			     i < PIOS_SPEKTRUM_NUM_INPUTS && pios_rcvr_max_channel < NELEMENTS(pios_rcvr_channel_to_id_map);
			     i++) {
				pios_rcvr_channel_to_id_map[pios_rcvr_max_channel].id = pios_spektrum_rcvr_id;
				pios_rcvr_channel_to_id_map[pios_rcvr_max_channel].channel = i;
				pios_rcvr_max_channel++;
			}
		}
#endif	/* PIOS_INCLUDE_SPEKTRUM */
#if defined(PIOS_INCLUDE_SBUS)
		if (hwsettings_cc_mainport == HWSETTINGS_CC_MAINPORT_SBUS) {
			uint32_t pios_sbus_rcvr_id;
			if (PIOS_RCVR_Init(&pios_sbus_rcvr_id, &pios_sbus_rcvr_driver, 0)) {
				PIOS_Assert(0);
			}
			for (uint8_t i = 0;
			     i < SBUS_NUMBER_OF_CHANNELS && pios_rcvr_max_channel < NELEMENTS(pios_rcvr_channel_to_id_map);
			     i++) {
				pios_rcvr_channel_to_id_map[pios_rcvr_max_channel].id = pios_sbus_rcvr_id;
				pios_rcvr_channel_to_id_map[pios_rcvr_max_channel].channel = i;
				pios_rcvr_max_channel++;
			}
		}
#endif  /* PIOS_INCLUDE_SBUS */

	//PIOS_WDG_Init();
}

/**
 * @}
 */
