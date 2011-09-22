/**
 ******************************************************************************
 * @addtogroup PX2IO BOOTLOADER
 * @brief Bootloader for the PX2IO board
 *
 * @{ 
 * @addtogroup PX2IO_BOOTLOADER_Main
 * @brief Main function which does the hardware dependent stuff
 * @{ 
 *
 *
 * @file       main.c
 * @author     Copyright (c) 2011, Michael Smith
 * @brief
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
#include <pios_board_info.h>
#include <bl_fsm.h>		/* lfsm_state */

extern void PIOS_Board_Init(void);

typedef void	(*pFunction)(void);
pFunction		Jump_To_Application;
uint32_t		JumpAddress;

void			jump_to_app();
uint8_t			jumpFW = FALSE;

/**
 * @brief Startup logic.
 */
int main()
{
	bool	boot_app = true;

	/* do basic system init - clocks, GPIOs, LEDs */
	PIOS_SYS_Init();

	/* start the delay system - should be in SYS_Init */
	PIOS_DELAY_Init();

	/* check for an external request to enter update mode here, set GO_dfu if requested */
	if (GPIO_ReadInputDataBit(GPIO_PORT_SafetySwitch, GPIO_PIN_SafetySwitch) == Bit_SET) {
		boot_app = false;

		/* turn on the safety switch LED to tell the user that we got their request */
		PIOS_LED_On(LED3);
	}

	/* check for program-requested update mode */
	PIOS_IAP_Init();
	if (PIOS_IAP_CheckRequest()) {
		boot_app = false;
		PIOS_IAP_ClearRequest();
	}

	/* update not requested, start the app immediately - may return if app is invalid */
	if (boot_app)
		jump_to_app();

	/* bring up the rest of the board as required for the updater */
	PIOS_Board_Init();
	PIOS_LED_On(LED1);
	i2c_fsm();
	return 0;
}

void
jump_to_app()
{
	const struct pios_board_info * bdinfo = &pios_board_info_blob;

	PIOS_LED_On(LED1);
	if (((*(__IO uint32_t*) bdinfo->fw_base) & 0x2FFE0000) == 0x20000000) { /* Jump to user application */
		RCC_APB2PeriphResetCmd(0xffffffff, ENABLE);
		RCC_APB1PeriphResetCmd(0xffffffff, ENABLE);
		RCC_APB2PeriphResetCmd(0xffffffff, DISABLE);
		RCC_APB1PeriphResetCmd(0xffffffff, DISABLE);
		//_SetCNTR(0); // clear interrupt mask
		//_SetISTR(0); // clear all requests

		JumpAddress = *(__IO uint32_t*) (bdinfo->fw_base + 4);
		Jump_To_Application = (pFunction) JumpAddress;
		/* Initialize user application's Stack Pointer */
		__set_MSP(*(__IO uint32_t*) bdinfo->fw_base);
		Jump_To_Application();
	} else {
		return;
	}
}
