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
#include <pios_i2c_slave.h>
#include <px2io_protocol.h>
#include <FreeRTOS.h>

#include "msheap/msheap.h"

/* Task Priorities */
#define INIT_TASK_PRIORITY		(tskIDLE_PRIORITY + configMAX_PRIORITIES - 1)	// max priority
#define PROTOCOL_TASK_PRIORITY	(tskIDLE_PRIORITY + 2)
#define FAILSAFE_TASK_PRIORITY	(tskIDLE_PRIORITY + 3)

/* Global Variables */

/* Local Variables */
#define INIT_TASK_STACK			(1024 / 4)
#define PROTOCOL_TASK_STACK		(512 / 4)
#define FAILSAFE_TASK_STACK		(512 / 4)
static xTaskHandle initTaskHandle;
static xTaskHandle protocolTaskHandle;
static xTaskHandle failsafeTaskHandle;

/* Function Prototypes */
static void initTask(void *parameters);
static void protocolTask(void *parameters);
static void failsafeTask(void *parameters);

/* Prototype of PIOS_Board_Init() function */
extern void PIOS_Board_Init(void);

int main()
{
	int	result;

	/* initialise the heap */
	vPortInitialiseBlocks();

	/* core PIOS init */
	PIOS_SYS_Init();

	/* create the init task */
	result = xTaskCreate(initTask,
						(const signed char *)"init",
						INIT_TASK_STACK,
						NULL,
						INIT_TASK_PRIORITY,
						&initTaskHandle);
	PIOS_Assert(result == pdPASS);

	/* Start the FreeRTOS scheduler */
	vTaskStartScheduler();

	/* either we failed to start the scheduler, or it has returned unexpectedly */
	/* XXX might actually want to reboot here and hope the failure was transient? */
	PIOS_DELAY_Init();
	PIOS_LED_Off(LED1);
	PIOS_LED_On(LED2);
	for(;;) {
		PIOS_LED_Toggle(LED1);
		PIOS_LED_Toggle(LED2);
		PIOS_DELAY_WaitmS(100);
	}

	return 0;
}

/**
 * Initialisation task.
 *
 * Runs board and module initialisation, then terminates.
 */
void
initTask(void *parameters)
{
	/* board driver init */
	PIOS_Board_Init();

	/* Initialize modules */
	MODULE_INITIALISE_ALL;

	/* start tasks */
	xTaskCreate(protocolTask,
				(const signed char *)"protocol",
				PROTOCOL_TASK_STACK,
				NULL,
				PROTOCOL_TASK_PRIORITY,
				&protocolTaskHandle);
	//TaskMonitorAdd(TASKINFO_RUNNING_PROTOCOL, protocolTaskHandle);
	//PIOS_WDG_RegisterFlag(PIOS_WDG_PROTOCOL);

	xTaskCreate(failsafeTask,
				(const signed char *)"failsafe",
				FAILSAFE_TASK_STACK,
				NULL,
				FAILSAFE_TASK_PRIORITY,
				&failsafeTaskHandle);
	//TaskMonitorAdd(TASKINFO_RUNNING_PROTOCOL, failsafeTaskHandle);
	//PIOS_WDG_RegisterFlag(PIOS_WDG_FAILSAFE);

	/* terminate this task */
	vTaskDelete(NULL);
}

/*
 * Control protocol handling
 */

/*
 * We have three copies total of the config.  One private to the callback
 * that is incrementally updated as the I2C transaction progresses, one global
 * copy that can be copied from safely while in a critical section, and one
 * private to the protocol task that is used as a reference when handling a
 * configuration change.
 */
static struct iop_set		current_config;
static volatile bool		config_changed;

/*
 * Similarly there are three copies total of the status.  One private to the
 * protocol task that is used to assemble the current status, one global copy
 * that is updated in a critical section, and one that is used as a buffer while
 * transmitting over I2C.
 */
static struct iop_get		current_status;

static void
protocol_callback(uint32_t i2c_id, enum pios_i2c_slave_event event, uint32_t arg)
{
	static struct pios_i2c_slave_txn txn;
	static struct iop_set			new_config;
	static struct iop_get			status;

	switch (event) {
	case PIOS_I2C_SLAVE_TRANSMIT:
		memcpy(&status, &current_status, sizeof(status));
		txn.buf = (void *)&status;
		txn.len = sizeof(status);
		PIOS_I2C_SLAVE_Transfer(0, &txn, 1);
		break;

	case PIOS_I2C_SLAVE_TRANSMIT_DONE:
		// XXX nothing to do here
		break;

	case PIOS_I2C_SLAVE_RECEIVE:
		txn.buf = (void *)&current_config;
		txn.len = sizeof(new_config);
		PIOS_I2C_SLAVE_Transfer(0, &txn, 1);
		break;

	case PIOS_I2C_SLAVE_RECEIVE_DONE:
		if (arg != sizeof(new_config))
			break;
		memcpy((void *)&current_config, &new_config, sizeof(new_config));
		config_changed = true;
		break;

	default:
		// not interested
		break;
	}
}

static void
protocolTask(void *parameters)
{
	struct iop_set		new_config;
	struct iop_get	new_status;

	PIOS_COM_SendFormattedString(PIOS_COM_DEBUG, "protocol task start\r\n");

	// set up the I2C slave callback
	PIOS_I2C_Slave_Open(0, protocol_callback);

	for (;;) {
		// has the config been updated?
		if (config_changed) {

			PIOS_LED_On(LED1);

			// make a copy of the new config
			vPortEnterCritical();
			config_changed = false;
			memcpy(&new_config, &current_config, sizeof(new_config));
			vPortExitCritical();

			// set new servo values
			for (int i = 0; i < IOP_PWM_CHANNELS; i++)
				PIOS_Servo_Set(i, new_config.channel_values[i]);

			// set GPIOs ... yay, more needless verbosity
			if (new_config.gpio_bits & IOP_GPIO_RELAY_0) {
				PIOS_GPIO_On(0);
			} else {
				PIOS_GPIO_Off(0);
			}
			if (new_config.gpio_bits & IOP_GPIO_RELAY_0) {
				PIOS_GPIO_On(1);
			} else {
				PIOS_GPIO_Off(1);
			}
			if (new_config.gpio_bits & IOP_GPIO_POWER_0) {
				PIOS_GPIO_On(2);
			} else {
				PIOS_GPIO_Off(2);
			}
			if (new_config.gpio_bits & IOP_GPIO_POWER_1) {
				PIOS_GPIO_On(3);
			} else {
				PIOS_GPIO_Off(3);
			}
			// XXX ignore servo power for now... not sure how best to handle it
			if (new_config.gpio_bits & IOP_GPIO_PUSHBUTTON_LED) {
				PIOS_LED_On(LED3);
			} else {
				PIOS_LED_Off(LED3);
			}

			PIOS_LED_Off(LED1);
		} else {
			// XXX handle FMU-not-talking situation (failsafe reset)
			// PIOS_LED_Off(LED1);
		}

		// XXX update new_status with recent events

		new_status.status_bits = 0;
		for (int i = 0; i < IOP_ADC_CHANNELS; i++)
			new_status.adc_inputs[i] = 0;	// XXX
		for (int i = 0; i < IOP_RADIO_CHANNELS; i++)
			new_status.channel_values[i] = 0;

		// atomically copy new status to where the I2C callback will see it
		vPortEnterCritical();
		memcpy(&current_status, &new_status, sizeof(current_status));
		vPortExitCritical();

		// give up our quantum if there's anyone else that wants it
		vPortYieldFromISR();
	}
}

static void
failsafeTask(void *parameters)
{
	PIOS_COM_SendFormattedString(PIOS_COM_DEBUG, "failsafe task start\r\n");

	for (;;) {
		PIOS_LED_Toggle(LED2);
		vTaskDelay(1000 / portTICK_RATE_MS);
	}
}

void
vApplicationTickHook(void)
{
	// XXX this might be a bit drastic later on
	msheap_check();
}

/**
 * @}
 * @}
 */

