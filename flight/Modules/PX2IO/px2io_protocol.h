/*
 * protocol.h
 *
 *  Created on: Oct 17, 2011
 *      Author: msmith
 */

#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#define PX2IO_I2C_ADDR	0x10

/*
 * Trivial I2C protocol for IO.
 */

#define IOP_PWM_CHANNELS					8
#define IOP_RADIO_CHANNELS					8
#define IOP_ADC_CHANNELS					2

#define IOP_GPIO_POWER_0					(1<<0)
#define IOP_GPIO_POWER_1					(1<<1)
#define IOP_GPIO_SERVO_POWER				(1<<7)
#define IOP_GPIO_RELAY_0					(1<<8)
#define IOP_GPIO_RELAY_1					(1<<9)

#define IOP_GPIO_PUSHBUTTON_LED				(1<<16)

#define IOP_STATUS_ACCESSORY_OVERCURRENT	(1<<0)
#define IOP_STATUS_SERVO_OVERCURRENT		(1<<7)
#define IOP_STATUS_PUSHBUTTON				(1<<16)

#define IOP_ADC_VIN							0

struct iop_set {
	uint32_t		gpio_bits;
	uint16_t		channel_values[IOP_PWM_CHANNELS];
} __attribute__((packed));

struct iop_get {
	uint32_t		status_bits;
	uint16_t		adc_inputs[IOP_ADC_CHANNELS];
	uint16_t		channel_values[IOP_RADIO_CHANNELS];
};

#endif /* PROTOCOL_H_ */
