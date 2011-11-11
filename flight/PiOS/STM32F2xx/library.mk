#
# Rules to (help) build the F2xx device support.
#

#
# Directory containing this makefile
#
PIOS_DEVLIB			:=	$(dir $(lastword $(MAKEFILE_LIST)))

#
# Hardcoded linker script names for now
#
LINKER_SCRIPTS_APP	 =	$(PIOS_DEVLIB)/link_STM32F2xx_OP_memory.ld \
						$(PIOS_DEVLIB)/link_STM32F2xx_sections.ld

LINKER_SCRIPTS_BL	 =	$(PIOS_DEVLIB)/link_STM32F2xx_OP_memory.ld \
						$(PIOS_DEVLIB)/link_STM32F2xx_sections.ld

#
# Compiler options implied by the F2xx
#
CDEFS				+= -DSTM32F2XX
CDEFS				+= -DHSE_VALUE=$(OSCILLATOR_FREQ)
CDEFS 				+= -DUSE_STDPERIPH_DRIVER
ARCHFLAGS			+= -mcpu=cortex-m3

#
# PIOS device library source and includes
#
SRC					+=	$(wildcard $(PIOS_DEVLIB)/*.c)
EXTRAINCDIRS		+=	$(PIOS_DEVLIB)/inc

#
# CMSIS for the F2
#
CMSIS_DIR			:=	$(PIOS_DEVLIB)/Libraries/CMSIS
SRC					+=	$(wildcard $(CMSIS_DIR)/Core/CM3/*.c)
EXTRAINCDIRS		+=	$(CMSIS_DIR)/Core/CM3

#
# ST Peripheral library
#
PERIPHLIB			 =	$(PIOS_DEVLIB)/Libraries/STM32F2xx_StdPeriph_Driver
SRC					+=	$(wildcard $(PERIPHLIB)/src/*.c)
EXTRAINCDIRS		+=	$(PERIPHLIB)/inc

#
# FreeRTOS
#
# If the application has included the generic FreeRTOS support, then add in
# the device-specific pieces of the code.
#
ifneq ($(FREERTOS_DIR),)
FREERTOS_PORTDIR	:=	$(PIOS_DEVLIB)/Libraries/FreeRTOS/Source
SRC					+=	$(wildcard $(FREERTOS_PORTDIR)/portable/GCC/ARM_CM3/*.c)
EXTRAINCDIRS		+=	$(FREERTOS_PORTDIR)/portable/GCC/ARM_CM3
endif

