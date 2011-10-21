/*
 * dfu.h
 *
 *  Created on: Sep 30, 2011
 *      Author: msmith
 */

#ifndef DFU_H_
#define DFU_H_

#include <stdint.h>
/*
 * Serial DFU
 *
 * This protocol implements a fairly literal interpretation of the USB
 * DFU protocol over bidirectional async serial.
 *
 * Each packet in either direction is preceded by a 4-byte preamble, 'sDFU'.
 * Host->device packets then contain a DFUHeader struct, which conveys the
 * same information as a DFU request in the USB version of the protocol.
 * Device->host packet contents are specific to the request that generates them.
 *
 * The DFU state machine is implemented with a few variations:
 * - serial cannot stall, so transitions that would normally result in a stall
 *   are just NOPs; the device generates no result, and usually enters the
 *   dfuError state.
 * - serial has no metadata interface, so a new command DFU_GET_INFO returns
 *   a DFUDescriptor structure that contains the essential fields from the
 *   DFU device USB descriptors.
 * - serial has no concept of disconnect or reset, so the device will reboot
 *   as soon as the MANIFEST_SYNC reply is sent.
 *
 */

/* DFU commands */
#define	DFU_DETACH		0
#define	DFU_DNLOAD		1
#define	DFU_UPLOAD		2
#define	DFU_GETSTATUS	3
#define	DFU_CLRSTATUS	4
#define	DFU_GETSTATE	5
#define	DFU_ABORT		6

#define DFU_GET_INFO	100	// fetch the DFU descriptor

/*
 * Serial encapsulation of DFU commands.
 *
 * This is a fairly literal translation of the USB DFU packet,
 * though the wIndex field is omitted as it's not relevant.
 *
 * Each packet is preceded by a 4-byte preamble, 'sDFU'.
 */

struct DFUHeader {
	uint32_t	command;	// DFU command opcode
	uint32_t	value;		// DFU protocol wValue field
	uint32_t	length;		// DFU protocol wLength field
	uint8_t		data[];		// DFU protocol Data field
};

struct DFUDescriptor {
	uint32_t	attributes;		// DFU protocol bmAttributes field
	uint32_t	transfer_size;	// DFU protocol wTransferSize field
	uint32_t	vendor;			// idVendor value
	uint32_t	product;		// idProduct value
	uint32_t	device;			// bcdDevice value
	char		name[32];		// iProduct string
	char		serial[32];		// iSerial string
};

enum DFUState {
	appIDLE				= 0,
	appDETACH			= 1,
	dfuIDLE				= 2,
	dfuDNLOAD_SYNC		= 3,
	dfuDNBUSY			= 4,
	dfuDNLOAD_IDLE		= 5,
	dfuMANIFEST_SYNC	= 6,
	dfuMANIFEST			= 7,
	dfuMANIFEST_WAIT_RESET = 8,
	dfuUPLOAD_IDLE		= 9,
	dfuERROR			= 10,
};

enum DFUStatus {
	errOK				= 0,
	errTARGET			= 1,
	errFILE				= 2,
	errWRITE			= 3,
	errERASE			= 4,
	errCHECK_ERASE		= 5,
	errPROG				= 6,
	errVERIFY			= 7,
	errADDRESS			= 8,
	errNOTDONE			= 9,
	errFIRMWARE			= 10,
	errVENDOR			= 11,
	errUNKNOWN			= 14,
	errSTALLED			= 15
};

struct DFUStatusData {
	uint32_t	status;		// DFU protocol bStatus field
	uint32_t	state;		// DFU protocol bState field
};

#define DFU_MAX_DATA	64		// largest data block we will process

extern void	dfu_init(uint32_t port, uint32_t code, uint32_t code_size);
extern void	dfu_tick(void);


#endif /* DFU_H_ */
