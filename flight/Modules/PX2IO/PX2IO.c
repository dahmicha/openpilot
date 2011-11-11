/**
 * Driver for the PX2IO expansion board.
 */

#include "pios.h"
#include "actuatorcommand.h"
#include "px2io_protocol.h"

#define PX2IO_POLL_INTERVAL_TICKS	(20 / portTICK_RATE_MS)

static xTaskHandle	px2ioTaskHandle;
static void			px2ioTask(void *parameters);

static xQueueHandle px2ioQueue;

int32_t
PX2IOStart(void)
{
	// Start the PX2IO comms task
	xTaskCreate(px2ioTask, (signed char *)"PX2IO", STACK_SIZE_BYTES/4, NULL, PX2IO_TASK_PRIORITY, &px2ioTaskHandle);
	//TaskMonitorAdd(TASKINFO_RUNNIN_PX2IO, px2ioTaskHandle);
	//PIOS_WDG_RegisterFlag(PIOS_WDG_PX2IO);

	return 0;
}

int32
PX2IOInitialize(void)
{

	px2ioQueue = xQueueCreate(MAX_QUEUE_SIZE, sizeof(UAVObjEvent));
	ActuatorCommandConnectQueue(px2ioQueue);

	// we could/should watch the update rate changes as well, but for now we fix @ 50Hz

	return 0;
}

MODULE_INITCALL(PX2IOInitialize, PX2IOStart);

static bool
px2io_get(struct iop_get *get)
{
	struct pios_i2c_txn txn_list = {
			.info = __func__,
			.addr = PX2IO_I2C_ADDR,
			.rw = PIOS_I2C_TXN_READ,
			.len = sizeof(*get),
			.buf = get
	};

	return PIOS_I2C_Transfer(PIOS_I2C_MAIN_ADAPTER, &txn_list, 1);
}

static bool
px2io_set(struct iop_set *set)
{
	struct pios_i2c_txn txn_list = {
			.info = __func__,
			.addr = PX2IO_I2C_ADDR,
			.rw = PIOS_I2C_TXN_WRITE,
			.len = sizeof(*set),
			.buf = set
	};

	return PIOS_I2C_Transfer(PIOS_I2C_MAIN_ADAPTER, &txn_list, 1);
}

static void
px2ioTask(void *parameters)
{
	portTickType		lastSysTime;
	UAVObjEvent			ev;
	ActuatorCommandData	command;
	int					channel;
	struct iop_set		set;
	struct iop_get		get;

	PIOS_COM_SendString(PIOS_COM_DEBUG, "PX2IO task running\r\n");

	// task loop
	lastSysTime = xTaskGetTickCount();
	for (;;) {

		//PIOS_WDG_UpdateFlag(PIOS_WDG_PX2IO);

		// Block waiting for an update on the ActuatorCommand object
		// XXX do we want to handle failsafe here?  The mixer already does
		// it, and PX2IO should as well...
		//
		// Note that blocking on the queue means that the update rate will
		// change to suit the mixer config, within reason.
		//
		xQueueReceive(px2ioQueue, &ev, 0);
		ActuatorCommandGet(&command);

		// Build the new set command
		memset(&set, 0, sizeof(set));
		for (channel = 0; channel < min(IOP_PWM_CHANNELS, ACTUATORCOMMAND_CHANNEL_NUMELEM); channel++) {
			set.channel_values[channel] = command.Channel[channel];
		}
		// XXX GPIO bits

		px2io_set(&set);	// XXX check for success/failure ... what would we do?

		// fetch status from PX2IO
		px2io_get(&get);

		// XXX publish get results as an object?

	}
}
