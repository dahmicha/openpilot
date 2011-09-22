/*
 * pios_i2c_slave.c
 *
 *  Created on: Jul 29, 2011
 *      Author: msmith
 */

#include <pios_i2c_slave.h>

/**
 * States implemented by the I2C slave FSM.
 */
enum fsm_state {
	BAD_PHASE,			// must be zero, default exit on a bad state transition

	WAIT_FOR_MASTER,

	/* write from master */
	WAIT_FOR_DATA,
	RECEIVE_DATA,
	RECEIVE_COMPLETE,

	/* read from master */
	WAIT_TO_SEND,
	SEND_DATA,
	SEND_COMPLETE,

	NUM_STATES
};

/**
 * Events recognised by the I2C slave FSM.
 */
enum fsm_event {
	/* automatic transition */
	AUTO,

	/* write from master */
	ADDRESSED_WRITE,
	BYTE_RECEIVED,
	STOP_RECEIVED,

	/* read from master */
	ADDRESSED_READ,
	BYTE_SENDABLE,
	ACK_FAILED,

	NUM_EVENTS
};

/**
 * Context for the I2C slave FSM
 */
struct fsm_context {
	uint32_t		i2c_id;
	I2C_TypeDef		*regs;
	enum fsm_state	state;

	pios_i2c_slave_callback callback;

	struct pios_i2c_slave_txn	*txn_list;
	uint32_t		txn_remaining;
	uint32_t		txn_data_offset;

};

/**
 * Structure defining one FSM state and its outgoing transitions.
 */
struct fsm_transition {
		void (*handler)(struct fsm_context *ctx);
		enum fsm_state next_state[NUM_EVENTS];
};

static void fsm_event(struct fsm_context *ctx, enum fsm_event event);

static void	go_bad(struct fsm_context *ctx);
static void	go_wait_master(struct fsm_context *ctx);

static void	go_wait_data(struct fsm_context *ctx);
static void	go_receive_data(struct fsm_context *ctx);
static void	go_receive_done(struct fsm_context *ctx);

static void	go_wait_send(struct fsm_context *ctx);
static void	go_send_data(struct fsm_context *ctx);
static void	go_send_done(struct fsm_context *ctx);

/**
 * The FSM state graph.
 */
struct fsm_transition fsm[NUM_STATES] = {
		[BAD_PHASE] = {
				.handler = go_bad,
				.next_state = {
						[AUTO] = WAIT_FOR_MASTER,
				},
		},

		[WAIT_FOR_MASTER] = {
				.handler = go_wait_master,
				.next_state = {
						[ADDRESSED_WRITE] = WAIT_FOR_DATA,
						[ADDRESSED_READ] = WAIT_TO_SEND,
				},
		},

		/* write from master */
		[WAIT_FOR_DATA] = {
				.handler = go_wait_data,
				.next_state = {
						[BYTE_RECEIVED] = RECEIVE_DATA,
						[STOP_RECEIVED] = WAIT_FOR_MASTER,
				},
		},
		[RECEIVE_DATA] = {
				.handler = go_receive_data,
				.next_state = {
						[BYTE_RECEIVED] = RECEIVE_DATA,
						[STOP_RECEIVED] = RECEIVE_COMPLETE,
				},
		},
		[RECEIVE_COMPLETE] = {
				.handler = go_receive_done,
				.next_state = {
						[AUTO] = WAIT_FOR_MASTER,
				},
		},

		/* buffer send */
		[WAIT_TO_SEND] = {
			.handler = go_wait_send,
			.next_state = {
					[BYTE_SENDABLE] = SEND_DATA,
			},
		},
		[SEND_DATA] = {
				.handler = go_send_data,
				.next_state = {
						[BYTE_SENDABLE] = SEND_DATA,
						[ACK_FAILED] = SEND_COMPLETE,
				},
		},
		[SEND_COMPLETE] = {
				.handler = go_send_done,
				.next_state = {
						[AUTO] = WAIT_FOR_MASTER,
				},
		},
};

static struct fsm_context context;	// XXX need an array for > 1 device

static struct fsm_context *context_for_id(uint32_t i2c_id)
{
	// XXX
	return	&context;
}

int
PIOS_I2C_Slave_Init(uint32_t i2c_id, const struct pios_i2c_adapter_cfg *cfg)
{
	struct fsm_context	*ctx = context_for_id(i2c_id);

	ctx->i2c_id = i2c_id;
	ctx->regs = cfg->regs;

	/* Enable the associated peripheral clock */
	/* XXX this is bogus, clocks should always be on */
	switch ((uint32_t)cfg->regs) {
	case (uint32_t) I2C1:
		/* Enable I2C peripheral clock (APB1 == slow speed) */
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
		break;
	case (uint32_t) I2C2:
		/* Enable I2C peripheral clock (APB1 == slow speed) */
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
		break;
	default:
		return -1;
	}

	// do pin init
	GPIO_Init(cfg->sda.gpio, &(cfg->sda.init));
	GPIO_Init(cfg->scl.gpio, &(cfg->scl.init));

	// do i2c init but start disabled
	I2C_DeInit(ctx->regs);
	I2C_Init(ctx->regs, &cfg->init);

	return 0;
}

void
PIOS_I2C_Slave_Open(uint32_t i2c_id, pios_i2c_slave_callback callback)
{
	struct fsm_context	*ctx = context_for_id(i2c_id);

	// hook up the callback
	ctx->callback = callback;

	// and open for business
	PIOS_I2C_SLAVE_Enable(i2c_id, true);
}

void
PIOS_I2C_SLAVE_Enable(uint32_t i2c_id, bool enabled)
{
	struct fsm_context	*ctx = context_for_id(i2c_id);

	// @todo consider interlock with FSM here?
	if (enabled) {
		// reinit the FSM
		ctx->state = BAD_PHASE;
		fsm_event(ctx, AUTO);

		// enable the controller
		I2C_Cmd(ctx->regs, ENABLE);
		I2C_ClearITPendingBit(ctx->regs, I2C_IT_BUF | I2C_IT_EVT | I2C_IT_ERR);
		I2C_ITConfig(ctx->regs, I2C_IT_BUF | I2C_IT_EVT | I2C_IT_ERR, ENABLE);
	} else {

		// disable the controller
		I2C_ITConfig(ctx->regs, I2C_IT_BUF | I2C_IT_EVT | I2C_IT_ERR, DISABLE);
		I2C_Cmd(ctx->regs, DISABLE);
	}
}


void
PIOS_I2C_SLAVE_Transfer(uint32_t i2c_id, struct pios_i2c_slave_txn txn_list[], uint32_t num_txns)
{
	struct fsm_context	*ctx = context_for_id(i2c_id);

	PIOS_Assert(txn_list != NULL);
	PIOS_Assert(num_txns > 0);

	// update the current transfer details
	ctx->txn_list = txn_list;
	ctx->txn_remaining = num_txns;
	ctx->txn_data_offset = 0;
}

void
PIOS_I2C_SLAVE_EV_IRQ_Handler(uint32_t i2c_id)
{
	struct fsm_context	*ctx = context_for_id(i2c_id);
	uint32_t			event;

	// fetch the most recent event
	event = I2C_GetLastEvent(I2C1);

	// generate FSM events based on I2C events
	switch (event) {
	case I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED:
		fsm_event(ctx, ADDRESSED_WRITE);
		break;

	case I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED:
		fsm_event(ctx, ADDRESSED_READ);
		break;

	case I2C_EVENT_SLAVE_BYTE_RECEIVED:
		fsm_event(ctx, BYTE_RECEIVED);
		break;

	case I2C_EVENT_SLAVE_STOP_DETECTED:
		fsm_event(ctx, STOP_RECEIVED);
		break;

	case I2C_EVENT_SLAVE_BYTE_TRANSMITTING:
	//case I2C_EVENT_SLAVE_BYTE_TRANSMITTED:
		fsm_event(ctx, BYTE_SENDABLE);
		break;

	case I2C_EVENT_SLAVE_ACK_FAILURE:
		fsm_event(ctx, ACK_FAILED);
		break;

	default:
		break;
	}
}

void
PIOS_I2C_SLAVE_ER_IRQ_Handler(uint32_t i2c_id)
{
	struct fsm_context	*ctx = context_for_id(i2c_id);

	// clear the flag in the hardware
	I2C_ClearFlag(ctx->regs, I2C_FLAG_BERR);

	// reset the FSM
	ctx->state = BAD_PHASE;
	fsm_event(ctx, AUTO);

	// tell our client that an error occurred
	ctx->callback(i2c_id, PIOS_I2C_SLAVE_ERROR, 0);
}

/**
 * Update the FSM with an event
 *
 * @param ctx	FSM context.
 * @param event	New event.
 */
static void
fsm_event(struct fsm_context *ctx, enum fsm_event event)
{
	// move to the next state
	ctx->state = fsm[ctx->state].next_state[event];

	// call the state entry handler
	if (fsm[ctx->state].handler) {
		fsm[ctx->state].handler(ctx);
	}
}

static void
go_bad(struct fsm_context *ctx)
{
	fsm_event(ctx, AUTO);
}

/**
 * Wait for the master to address us.
 *
 * @param ctx	FSM context.
 */
static void
go_wait_master(struct fsm_context *ctx)
{
	// clear transaction state
	ctx->txn_list = NULL;
	ctx->txn_remaining = 0;


	// (re)enable the peripheral, clear the stop event flag in
	// case we just finished receiving data
	I2C_Cmd(ctx->regs, ENABLE);

	// clear the ACK failed flag in case we just finished sending data
	I2C_ClearFlag(ctx->regs, I2C_FLAG_AF);
}

/**
 * Prepare to receive a data byte.
 *
 * @param ctx	FSM context.
 */
static void
go_wait_data(struct fsm_context *ctx)
{
	// Tell the client we are about to receive
	//
	// We expect them to call PIOS_I2C_SLAVE_Transfer if they want to capture the data.
	//
	ctx->callback(ctx->i2c_id, PIOS_I2C_SLAVE_RECEIVE, 0);
}

/**
 * Receive a data byte.
 *
 * @param ctx	FSM context.
 */
static void
go_receive_data(struct fsm_context *ctx)
{
	uint8_t	d;

	// fetch the byte
	d = I2C_ReceiveData(ctx->regs);

	// if we don't have a txn_list, nobody wants the data
	if (ctx->txn_list == NULL) {
		return;
	}

	// capture the byte
	ctx->txn_list->buf[ctx->txn_data_offset] = d;

	// increment the buffer pointer and check for overflow into the next buffer
	if (++ctx->txn_data_offset > ctx->txn_list->len) {

		// check for another buffer
		if (--ctx->txn_remaining > 0) {

			// use the next buffer
			ctx->txn_list++;
			ctx->txn_data_offset = 0;
		} else {

			// assume that since the buffer list has run out, we no longer care about data
			ctx->txn_list = NULL;

			// Tell the client we have run out of buffer space - they may want to call
			// PIOS_I2C_SLAVE_Transfer to set up a new buffer based on what they have
			// received so far.
			ctx->callback(ctx->i2c_id, PIOS_I2C_SLAVE_BUFFER_FULL, 0);
		}
	}
}

/**
 * The master has stopped transmitting, tell the client that reception has finished.
 *
 * @param ctx	FSM context.
 */
static void
go_receive_done(struct fsm_context *ctx)
{
	// Tell the client that the transmitter has stopped
	ctx->callback(ctx->i2c_id, PIOS_I2C_SLAVE_RECEIVE_DONE, 0);

	// kick along to the next state
	fsm_event(ctx, AUTO);
}

/**
 * Wait to be able to send data.
 *
 * @param ctx	FSM context.
 */
static void
go_wait_send(struct fsm_context *ctx)
{
	// Call the client and give them the chance to set up a transfer.
	ctx->callback(ctx->i2c_id, PIOS_I2C_SLAVE_TRANSMIT, 0);
}

/**
 * Send data or a pad byte.
 *
 * @param ctx	FSM context
 */
static void
go_send_data(struct fsm_context *ctx)
{
	uint8_t	d = 0xff;

	if (ctx->txn_list) {
		d = ctx->txn_list->buf[ctx->txn_data_offset];

		// increment the buffer pointer and check for wrap into the next buffer
		if (++ctx->txn_data_offset > ctx->txn_list->len) {

			// check for another buffer
			if (--ctx->txn_remaining > 0) {

				// use the next buffer
				ctx->txn_list++;
				ctx->txn_data_offset = 0;
			} else {

				// there are no more buffers
				ctx->txn_list = NULL;
			}
		}
	}
	I2C_SendData(ctx->regs, d);
}

/**
 * The master has stopped accepting data, tell the client that transmission is finished.
 *
 * @param ctx
 */
static void
go_send_done(struct fsm_context *ctx)
{
	// Tell the client that transmission has stopped
	ctx->callback(ctx->i2c_id, PIOS_I2C_SLAVE_TRANSMIT_DONE, 0);

	// kick along to the next state
	fsm_event(ctx, AUTO);
}
