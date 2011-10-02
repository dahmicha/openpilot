/*
 * dfu.c
 *
 *  Created on: Sep 30, 2011
 *      Author: msmith
 */

#include <stdbool.h>
#include <string.h>

#include "pios.h"
#include "dfu.h"

extern bool try_boot;

enum PKTState {
	PKT_WAIT_PRE0	= 0,
	PKT_WAIT_PRE1	= 1,
	PKT_WAIT_PRE2	= 2,
	PKT_WAIT_PRE3	= 3,
	PKT_GET_PKT		= 4,
};
static const char preamble[] = { 's', 'D', 'F', 'U' };

static int 					_pkt_state = PKT_WAIT_PRE0;	// packet parser machine state
static enum DFUState		_dfu_state = dfuIDLE;		// DFU machine state
static uint32_t				_port;
static uint32_t				_dfu_status = errOK;
static uint32_t				_address;				// program space address
static uint32_t				_address_max;
static uint8_t				*_codespace;

static struct DFUHeader		_command;
static uint8_t				_data[DFU_MAX_DATA];

static void dfu_process(void);

typedef void	(* dfu_state_handler)(void);

static void		dfu_handle_idle(void);
static void		dfu_handle_dnload_sync(void);
static void		dfu_handle_dnload_idle(void);
static void		dfu_handle_manifest_sync(void);
static void		dfu_handle_upload_idle(void);
static void		dfu_handle_error(void);

static void		dfu_go_dnload(void);
static void		dfu_go_upload(void);

static void		dfu_go_error(void);
static void		dfu_reply(void);

static const dfu_state_handler	_handlers[] = {
		dfu_go_error,				// appIDLE
		dfu_go_error,				// appDETACH
		dfu_handle_idle,			// dfuIDLE
		dfu_handle_dnload_sync,		// dfuDNLOAD_SYNC
		dfu_go_error,				// dfuDNBUSY
		dfu_handle_dnload_idle,		// dfuDNLOAD_IDLE
		dfu_handle_manifest_sync,	// dfuMANIFEST_SYNC
		dfu_go_error,				// dfuMANIFEST
		dfu_go_error,				// dfuMANIFEST_WAIT_RESET
		dfu_handle_upload_idle,		// dfuUPLOAD_IDLE
		dfu_handle_error,			// dfuERROR
};

void
dfu_init(uint32_t port, uint32_t code, uint32_t code_size)
{
	_port = port;
	_codespace = (uint8_t *)code;
	_address_max = code_size;
}

void
dfu_tick(void)
{
	int ready = PIOS_COM_ReceiveBufferUsed(_port);	// size of ready data
	uint8_t	c;

	switch (_pkt_state) {

	// preamble detector
	case PKT_WAIT_PRE0:
	case PKT_WAIT_PRE1:
	case PKT_WAIT_PRE2:
	case PKT_WAIT_PRE3:
		if (ready < 1)
			break;
		PIOS_COM_ReceiveBuffer(_port, &c, 1, 0);
		if (c != preamble[_pkt_state]) {
			_pkt_state = PKT_WAIT_PRE0;
		} else {
			_pkt_state++;
		}
		break;

	// command packet reception
	case PKT_GET_PKT:
		if (ready < sizeof(_command))
			break;
		PIOS_COM_ReceiveBuffer(_port, (void *)&_command, sizeof(_command), 0);
		dfu_process();
		_pkt_state = PKT_WAIT_PRE0;
		break;
	}
}

static void
dfu_process()
{
	// handle universal commands
	switch (_command.command) {

	case DFU_GETSTATE:
		dfu_reply();
		break;

	case DFU_ABORT:
		_dfu_state = dfuIDLE;
		_dfu_status = errOK;
		break;

	default:
		_handlers[_dfu_state]();
		break;
	}
}

static void
dfu_reply(void)
{
	struct DFUStatusData	reply;

	reply.state = _dfu_state;
	reply.status = _dfu_status;
	PIOS_COM_SendBuffer(_port, (uint8_t *)preamble, sizeof(preamble));
	PIOS_COM_SendBuffer(_port, (uint8_t *)&reply, sizeof(reply));
}

static void
dfu_reply_info(void)
{
	struct DFUDescriptor	desc;

	desc.attributes = 0x7;	// read/write, communicates after manifestation
	desc.transfer_size = DFU_MAX_DATA;
	desc.vendor = 0x0;		// XXX vendor ID
	desc.product = 0x0;		// XXX product ID
	desc.device = 0x0;		// XXX board ID
	strncpy(desc.name, "PX2FMU", sizeof(desc.name));
	strncpy(desc.serial, "SERIAL", sizeof(desc.serial));
	PIOS_COM_SendBuffer(_port, (uint8_t *)preamble, sizeof(preamble));
	PIOS_COM_SendBuffer(_port, (uint8_t *)&desc, sizeof(desc));
	_dfu_status = errOK;
}


static void
dfu_go_error(void)
{
	_dfu_state = dfuERROR;
}

static void
dfu_handle_error(void)
{
	if (_command.command == DFU_CLRSTATUS)
		_dfu_state = dfuIDLE;
}

static void
dfu_handle_idle(void)
{
	switch (_command.command) {
	case DFU_DNLOAD:
		_address = 0;
		// XXX erase all of program space here
		dfu_go_dnload();
		break;
	case DFU_UPLOAD:
		_address = 0;
		dfu_go_upload();
		break;
	case DFU_GETSTATUS:
		dfu_reply();
		break;
	case DFU_GET_INFO:
		dfu_reply_info();
		break;
	default:
		dfu_go_error();
		break;
	}
}

static void
dfu_handle_dnload_sync(void)
{
	switch (_command.command) {
	case DFU_GETSTATUS:
		_dfu_state = dfuDNLOAD_IDLE;
		dfu_reply();
		break;
	default:
		dfu_go_error();
		break;
	}
}

static void
dfu_handle_dnload_idle(void)
{
	switch (_command.command) {
	case DFU_GETSTATUS:
		dfu_reply();
		break;
	case DFU_DNLOAD:
		dfu_go_dnload();
		break;
	default:
		dfu_go_error();
		break;
	}
}

static void
dfu_handle_upload_idle(void)
{
	switch (_command.command) {
	case DFU_GETSTATUS:
		dfu_reply();
		break;
	case DFU_UPLOAD:
		dfu_go_upload();
		break;
	default:
		dfu_go_error();
		break;
	}
}

static void
dfu_handle_manifest_sync(void)
{
	switch (_command.command) {
	case DFU_GETSTATUS:
		_dfu_state = dfuMANIFEST;
		dfu_reply();
		try_boot = true;
		break;
	default:
		dfu_go_error();
		break;
	}
}

static void
dfu_go_dnload(void)
{
	if ((_command.length > DFU_MAX_DATA)|| ((_address + _command.length) > _address_max)) {
		_dfu_status = errADDRESS;
		dfu_go_error();
		return;
	}
	// upload is complete
	if (_command.length == 0) {
		_dfu_state = dfuMANIFEST_SYNC;
		return;
	}

	// get data
	PIOS_COM_ReceiveBuffer(_port, _data, _command.length, 0);

	// XXX program _command.length bytes in _data to _address
	_address += _command.length;
	_dfu_status = errOK;
	_dfu_state = dfuDNLOAD_SYNC;
}

static void
dfu_go_upload(void)
{
	if ((_command.length > DFU_MAX_DATA)|| ((_address + _command.length) > _address_max)) {
		_dfu_status = errADDRESS;
		dfu_go_error();
		return;
	}
	_dfu_status = errOK;
	if (_command.length == 0) {
		_dfu_state = dfuIDLE;
		return;
	}
	memcpy(_data, _codespace + _address, _command.length);
	_address += _command.length;
	PIOS_COM_SendBuffer(_port, (uint8_t *)preamble, sizeof(preamble));
	PIOS_COM_SendBuffer(_port, _data, _command.length);

	_dfu_state = dfuUPLOAD_IDLE;
}
