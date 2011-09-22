/*
 * pios_i2c_slave.h
 *
 *  Created on: Jul 29, 2011
 *      Author: msmith
 */

#ifndef PIOS_I2C_SLAVE_H_
#define PIOS_I2C_SLAVE_H_

#include <pios_i2c_priv.h>	// for struct pios_i2c_adapter_cfg

enum pios_i2c_slave_event {
	PIOS_I2C_SLAVE_RECEIVE,			//< addressed by master wanting to write
	PIOS_I2C_SLAVE_BUFFER_FULL,		//< master has filled the buffer in a supplied txn
	PIOS_I2C_SLAVE_RECEIVE_DONE,	//< master has stopped sending data
	PIOS_I2C_SLAVE_TRANSMIT,		//< addressed by master wanting to read
	PIOS_I2C_SLAVE_TRANSMIT_DONE,	//< master has stopped reading data

	PIOS_I2C_SLAVE_ERROR			//< an error ocurred, any current transaction has been discarded
};

struct pios_i2c_slave_txn {
	uint32_t	len;
	uint8_t		*buf;
};

typedef void (* pios_i2c_slave_callback)(uint32_t i2c_id, enum pios_i2c_slave_event event, uint32_t arg);

extern int	PIOS_I2C_Slave_Init(uint32_t i2c_id, const struct pios_i2c_adapter_cfg *cfg);
extern void PIOS_I2C_Slave_Open(uint32_t i2c_id, pios_i2c_slave_callback callback);
extern void	PIOS_I2C_SLAVE_Enable(uint32_t i2c_id, bool enabled);
extern void PIOS_I2C_SLAVE_Transfer(uint32_t i2c_id, struct pios_i2c_slave_txn txn_list[], uint32_t num_txns);

extern void PIOS_I2C_SLAVE_EV_IRQ_Handler(uint32_t i2c_id);
extern void PIOS_I2C_SLAVE_ER_IRQ_Handler(uint32_t i2c_id);

#endif /* PIOS_I2C_SLAVE_H_ */
