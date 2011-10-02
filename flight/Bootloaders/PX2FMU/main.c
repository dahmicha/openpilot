/**
 ******************************************************************************
 * @addtogroup OpenPilotBL OpenPilot Bootloader
 * @brief	Bootloader for the PX2FMU board.
 *
 * @{
 * @file       main.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      This is the file with the main function of the OpenPilot BootLoader
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

#include <stdbool.h>
#include <pios.h>
#include <pios_board_info.h>
#include "dfu.h"
#include "pios_iap.h"
#include "dcc_stdio.h"

extern void PIOS_Board_Init(void);

bool		try_boot = false;

#define LED_TOGGLE_INTERVAL		25000	// 25ms = 20Hz blink rate

static void jump_to_app();
static void	do_jump(uint32_t stacktop, uint32_t entrypoint) __attribute__((noreturn));

static bool	use_usb = false;

int main(void)
{
	uint32_t	led_last_toggle;

	/* do basic PiOS init */
	PIOS_SYS_Init();

	/* assume that we are going to try to boot */
	try_boot = true;

	/* init the IAP helper and check for a boot-to-DFU request */
	PIOS_IAP_Init();
	if (PIOS_IAP_CheckRequest()) {
		try_boot = false;
		PIOS_IAP_ClearRequest();
	}

	/* add checks for force-dfu controls here, e.g. talk to IO */

	/* if DFU not forced, try to jump to the app */
	if (try_boot)
		jump_to_app();

	/* if we get here, either jumping to the app failed or the app requested DFU mode */

	/* configure the board for bootloader use */
	PIOS_Board_Init();
	PIOS_LED_Off(LED1);
	PIOS_LED_Off(LED2);
	led_last_toggle = PIOS_DELAY_GetuS();
	PIOS_COM_SendString(PIOS_COM_DEBUG, "FMU BL\r\n");

	/* init the serial DFU engine */
	dfu_init(PIOS_COM_TELEM_RF, pios_board_info_blob.fw_base, pios_board_info_blob.fw_size);

	/* check whether USB is connected */
	PIOS_DELAY_WaitmS(10);	/* let the pin settle */
	use_usb = GPIO_ReadInputDataBit(PIOS_USB_DETECT_GPIO_PORT, PIOS_USB_DETECT_GPIO_PIN) != 0;
	if (use_usb) {
		// XXX set up the USB DFU downloader
	}

	/* bootloader loop */
	for (;;) {

		/* blink the LED */
		if (PIOS_DELAY_GetuSSince(led_last_toggle) > LED_TOGGLE_INTERVAL) {
			PIOS_LED_Toggle(LED1);
			led_last_toggle = PIOS_DELAY_GetuS();
		}

		/* kick the serial DFU engine */
		dfu_tick();

		/* has the host told us to try booting? */
		if (try_boot) {
			jump_to_app();
			/* for now, lock up here */
			for (;;)
				;
		}
	}
}

static void
do_jump(uint32_t stacktop, uint32_t entrypoint)
{
	asm volatile(
			"msr msp, %0	\n"
			"bx	%1			\n"
			: : "r" (stacktop), "r" (entrypoint) : );
	/* just to keep noreturn happy */
	for (;;) ;
}

void
jump_to_app()
{
	const uint32_t *fw_vec = (uint32_t *)(pios_board_info_blob.fw_base);

	if ((pios_board_info_blob.magic == PIOS_BOARD_INFO_BLOB_MAGIC) &&
		((fw_vec[0] & 0x2FF00000) == 0x20000000)) {

		FLASH_Lock();

		// XXX reset all peripherals here?
		do_jump(fw_vec[0], fw_vec[1]);
	}
	/* don't try again... */
	try_boot = false;
}
