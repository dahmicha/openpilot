 #####
 # Project: PX2 OpenPilot
 #
 # Makefile for the OpenPilot flight application on the PX2 FMU
 #
 # (c) 2011 Michael Smith
 # The OpenPilot Team, http://www.openpilot.org, Copyright (C) 2009.
 #
 # 
 # This program is free software; you can redistribute it and/or modify
 # it under the terms of the GNU General Public License as published by
 # the Free Software Foundation; either version 3 of the License, or
 # (at your option) any later version.
 #
 # This program is distributed in the hope that it will be useful, but
 # WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 # or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 # for more details.
 #
 # You should have received a copy of the GNU General Public License along
 # with this program; if not, write to the Free Software Foundation, Inc.,
 # 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 #####
 
 
 
 
 
########################## BEGIN OF USER CONFIGURATION
 
 # Use UAVTALK or MAVLINK
TELEMETRY_PROTOCOL	?= UAVTALK

# Debugging (YES/NO) ?
DEBUG ?= YES

# Set to YES when using Code Sourcery toolchain on Windows
CODE_SOURCERY ?= NO
 
# List of modules to include
# NOTE: The telemetry module and system module are included automatically
MODULES	= PX2AttitudeTL Actuator Stabilization ManualControl Altitude GPS Battery
 
# Deactivated modules: PX2Buzzer
 
########################## END OF USER CONFIGURATION
 
 
 
 
 
 

WHEREAMI := $(dir $(lastword $(MAKEFILE_LIST)))
TOP      := $(realpath $(WHEREAMI)/../../)

BOARD_NAME	= PX2FMU

include $(TOP)/make/firmware-defs.mk
include $(TOP)/make/boards/$(BOARD_NAME)/board-info.mk

# Target file name (without extension).
TARGET := fw_$(BOARD_NAME)

# Directory for output files (lst, obj, dep, elf, sym, map, hex, bin etc.)
OUTDIR := $(TOP)/build/$(TARGET)

# Remove command is different for Code Sourcery on Windows
ifeq ($(CODE_SOURCERY), YES)
REMOVE_CMD = cs-rm
else
REMOVE_CMD = rm -rf
endif

FLASH_TOOL = OPENOCD

# Enable Guidance only once REALLY in use: Guidance
# Telemetry must be last to grab the optional modules
ifeq ($(TELEMETRY_PROTOCOL), MAVLINK)
MODULES += MAVLink
else
MODULES += Telemetry
endif
PYMODULES = FlightPlan
#MODULES = PX2Test

# Paths
FLIGHT = $(TOP)/flight

OPSYSTEM = $(WHEREAMI)/System
OPSYSTEMINC = $(OPSYSTEM)/inc

ifeq ($(TELEMETRY_PROTOCOL), MAVLINK)
MAVLINK = $(FLIGHT)/Libraries/mavlink/include
MAVLINKCOMMON = $(MAVLINK)/common
MAVLINKMISSIONLIB = $(MAVLINK)/../missionlib
else
OPUAVTALK = $(FLIGHT)/UAVTalk
OPUAVTALKINC = $(OPUAVTALK)/inc
endif

OPUAVOBJ = $(FLIGHT)/UAVObjects
OPUAVOBJINC = $(OPUAVOBJ)/inc

OPTESTS  = $(WHEREAMI)/Tests

OPMODULEDIR = $(FLIGHT)/Modules

FLIGHTLIB = $(FLIGHT)/Libraries
FLIGHTLIBINC = $(FLIGHTLIB)/inc

PIOS = $(FLIGHT)/PiOS
PIOSINC = $(PIOS)/inc
PIOSBOARDS = $(PIOS)/Boards
PIOSCOMMON = $(PIOS)/Common
PIOSCOMMONLIB = $(PIOSCOMMON)/Libraries

DOXYGENDIR = $(FLIGHT)/Doc/Doxygen

PYMITE = $(FLIGHTLIB)/PyMite
PYMITELIB = $(PYMITE)/lib
PYMITEPLAT = $(PYMITE)/platform/openpilot
PYMITETOOLS = $(PYMITE)/tools
PYMITEVM = $(PYMITE)/vm
PYMITEINC = $(PYMITEVM)
PYMITEINC += $(PYMITEPLAT)
PYMITEINC += $(OUTDIR)

FLIGHTPLANLIB = $(OPMODULEDIR)/FlightPlan/lib
FLIGHTPLANS = $(OPMODULEDIR)/FlightPlan/flightplans

UAVOBJSYNTHDIR = $(OUTDIR)/../uavobject-synthetics/flight

# optional component libraries
include $(PIOSCOMMONLIB)/FreeRTOS/library.mk
include $(PIOSCOMMONLIB)/dosfs/library.mk
include $(PIOSCOMMONLIB)/msheap/library.mk

# XXX this should be removed when the PIOS code can be built without being configured
# or otherwise cherry-picked by the application
EXTRAINCDIRS +=	$(PIOS) $(PIOS)/inc
SRC	+=	$(PIOSCOMMON)/pios_bmp085.c \
		$(PIOSCOMMON)/pios_com.c \
		$(PIOSCOMMON)/pios_crc.c \
		$(PIOSCOMMON)/pios_eeprom.c \
		$(PIOSCOMMON)/pios_flashfs_objlist.c \
		$(PIOSCOMMON)/pios_hmc5883.c \
		$(PIOSCOMMON)/pios_i2c_esc.c \
		$(PIOSCOMMON)/pios_iap.c \
		$(PIOSCOMMON)/pios_l3g4200.c \
		$(PIOSCOMMON)/pios_lis331.c \
		$(PIOSCOMMON)/pios_rcvr.c \
		$(PIOSCOMMON)/pios_sdcard.c \
		$(PIOSCOMMON)/printf-stdarg.c

# target support library should come last
ifneq ($(M4),)
include $(PIOS)/STM32F4xx/library.mk
else
include $(PIOS)/STM32F2xx/library.mk
endif

# List C source files here. (C dependencies are automatically generated.)
# use file-extension c for "c-only"-files

# UNCOMMENT THE LINE BELOW TO PERFORM LOAD TESTING, SEE Tests/test_cpuload.c
#TESTAPP = test_cpuload
# UNCOMMENT THE LINE BELOW TO PERFORM FPU TESTING, SEE Tests/test_cpuload.c
#TESTAPP = test_fpu

ifndef TESTAPP

## PyMite files and modules
SRC += $(OUTDIR)/pmlib_img.c
SRC += $(OUTDIR)/pmlib_nat.c
SRC += $(OUTDIR)/pmlibusr_img.c
SRC += $(OUTDIR)/pmlibusr_nat.c
PYSRC += $(wildcard ${PYMITEVM}/*.c)
PYSRC += $(wildcard ${PYMITEPLAT}/*.c)
PYSRC += ${foreach MOD, ${PYMODULES}, ${wildcard ${OPMODULEDIR}/${MOD}/*.c}}
SRC += $(PYSRC)

## MODULES
SRC += ${foreach MOD, ${MODULES}, ${wildcard ${OPMODULEDIR}/${MOD}/*.c}}
## OPENPILOT CORE:
SRC += ${OPMODULEDIR}/System/systemmod.c
SRC += $(OPSYSTEM)/openpilot.c
SRC += $(OPSYSTEM)/pios_board.c
SRC += $(OPSYSTEM)/alarms.c
SRC += $(OPSYSTEM)/taskmonitor.c
ifeq ($(TELEMETRY_PROTOCOL), UAVTALK)
SRC += $(OPUAVTALK)/uavtalk.c
endif
SRC += $(OPUAVOBJ)/uavobjectmanager.c
SRC += $(OPUAVOBJ)/eventdispatcher.c

else

## TESTCODE
SRC += $(OPTESTS)/test_common.c
SRC += $(OPTESTS)/$(TESTAPP).c
endif

#ifeq ($(DEBUG),YES)
SRC += $(OPSYSTEM)/dcc_stdio.c
SRC += $(OPSYSTEM)/cm3_fault_handlers.c
#endif

## UAVOBJECTS
ifndef TESTAPP
#include $(UAVOBJSYNTHDIR)/Makefile.inc
include $(WHEREAMI)/UAVObjects.inc
ifeq ($(TELEMETRY_PROTOCOL), MAVLINK)
include $(WHEREAMI)/UAVObjectsMAVLink.inc
SRC += $(UAVOBJMAVLINKSRC)
endif
SRC += $(UAVOBJSRC)
endif

## MAVLINK MISSION LIB
ifeq ($(TELEMETRY_PROTOCOL), MAVLINK)
SRC += $(MAVLINKMISSIONLIB)/waypoints.c
endif

## Libraries for flight calculations
SRC += $(FLIGHTLIB)/fifo_buffer.c
SRC += $(FLIGHTLIB)/WorldMagModel.c
SRC += $(FLIGHTLIB)/CoordinateConversions.c

# List C source files here which must be compiled in ARM-Mode (no -mthumb).
# use file-extension c for "c-only"-files
SRCARM =

# List C++ source files here.
# use file-extension .cpp for C++-files (not .C)
CPPSRC =

# List C++ source files here which must be compiled in ARM-Mode.
# use file-extension .cpp for C++-files (not .C)
#CPPSRCARM = $(TARGET).cpp
CPPSRCARM =

# List Assembler source files here.
# Make them always end in a capital .S. Files ending in a lowercase .s
# will not be considered source files but generated files (assembler
# output from the compiler), and will be deleted upon "make clean"!
# Even though the DOS/Win* filesystem matches both .s and .S the same,
# it will preserve the spelling of the filenames, and gcc itself does
# care about how the name is spelled on its command-line.
ASRC	=

# List Assembler source files here which must be assembled in ARM-Mode..
ASRCARM =

# List any extra directories to look for include files here.
#    Each directory must be seperated by a space.
EXTRAINCDIRS  += $(OPSYSTEM)
EXTRAINCDIRS  += $(OPSYSTEMINC)
ifeq ($(TELEMETRY_PROTOCOL), MAVLINK)
EXTRAINCDIRS  += $(MAVLINK)
EXTRAINCDIRS  += $(MAVLINKCOMMON)
EXTRAINCDIRS  += $(MAVLINKMISSIONLIB)
else
EXTRAINCDIRS  += $(OPUAVTALK)
EXTRAINCDIRS  += $(OPUAVTALKINC)
endif
EXTRAINCDIRS  += $(OPUAVOBJ)
EXTRAINCDIRS  += $(OPUAVOBJINC)
EXTRAINCDIRS  += $(UAVOBJSYNTHDIR)
EXTRAINCDIRS  += $(FLIGHTLIBINC)
EXTRAINCDIRS  += $(PIOSBOARDS)
EXTRAINCDIRS  += $(PYMITEINC)

EXTRAINCDIRS += ${foreach MOD, ${MODULES} ${PYMODULES}, $(OPMODULEDIR)/${MOD}/inc} ${OPMODULEDIR}/System/inc


# List any extra directories to look for library files here.
# Also add directories where the linker should search for
# includes from linker-script to the list
#     Each directory must be seperated by a space.
EXTRA_LIBDIRS =

# Extra Libraries
#    Each library-name must be seperated by a space.
#    i.e. to link with libxyz.a, libabc.a and libefsl.a:
#    EXTRA_LIBS = xyz abc efsl
# for newlib-lpc (file: libnewlibc-lpc.a):
#    EXTRA_LIBS = newlib-lpc
EXTRA_LIBS =

# Optimization level, can be [0, 1, 2, 3, s].
# 0 = turn off optimization. s = optimize for size.
# (Note: 3 is not always the best optimization level. See avr-libc FAQ.)

ifeq ($(DEBUG),YES)
OPT = 0
else
OPT = s
endif

# Output format. (can be ihex or binary or both)
#  binary to create a load-image in raw-binary format i.e. for SAM-BA,
#  ihex to create a load-image in Intel hex format
#LOADFORMAT = ihex
#LOADFORMAT = binary
LOADFORMAT = both

# Debugging format.
DEBUGF = dwarf-2

# Place project-specific 
# (define) and/or
# -U options for C here.
#CDEFS += -DMAVLINK_SEPARATE_HELPERS
ifeq ($(TELEMETRY_PROTOCOL), MAVLINK)
CDEFS += -DMAVLINK_ENABLED
CDEFS += -DMAVLINK_USE_CONVENIENCE_FUNCTIONS
CDEFS += -DMAVLINK_CRC_EXTRA
CDEFS += $(UAVOBJMAVLINKDEFINE)
else
CDEFS += -DUAVTALK_ENABLED
endif
CDEFS += -DPX2MODE
CDEFS += -DPX2FMU
CDEFS += -DSYSCLK_FREQ=$(SYSCLK_FREQ)
CDEFS += -DUSE_$(BOARD)

# Place project-specific -D and/or -U options for
# Assembler with preprocessor here.
ADEFS = -D__ASSEMBLY__

# Compiler flag to set the C Standard level.
# c89   - "ANSI" C
# gnu89 - c89 plus GCC extensions
# c99   - ISO C99 standard (not yet fully implemented)
# gnu99 - c99 plus GCC extensions
CSTANDARD = -std=gnu99

#-----

# Compiler flags.

# Flags for C and C++ (arm-elf-gcc/arm-elf-g++)
ifeq ($(DEBUG),YES)
CFLAGS += -g$(DEBUGF) -DDEBUG
CFLAGS += -finstrument-functions -ffixed-r10
endif

# common architecture-specific flags from the device-specific library makefile
CFLAGS += $(ARCHFLAGS)

# FMU has enough memory to always enable optional objects
CFLAGS += -DDIAGNOSTICS

CFLAGS += -O$(OPT)
CFLAGS += $(CDEFS)
CFLAGS += $(patsubst %,-I%,$(EXTRAINCDIRS)) -I.
CFLAGS += -ffunction-sections -fdata-sections

CFLAGS += -mapcs-frame
CFLAGS += -fomit-frame-pointer
ifeq ($(CODE_SOURCERY), YES)
CFLAGS += -fpromote-loop-indices
endif

CFLAGS += -Wall
CFLAGS += -Werror
CFLAGS += -Wa,-adhlns=$(addprefix $(OUTDIR)/, $(notdir $(addsuffix .lst, $(basename $<))))
# Compiler flags to generate dependency files:
CFLAGS += -MD -MP -MF $(OUTDIR)/dep/$(@F).d

# flags only for C
CONLYFLAGS += $(CSTANDARD)

# Assembler flags.
#  -Wa,...:    tell GCC to pass this to the assembler.
#  -ahlns:     create listing
ASFLAGS  = $(ARCHFLAGS) -mthumb -I. -x assembler-with-cpp
ASFLAGS += $(ADEFS)
ASFLAGS += -Wa,-adhlns=$(addprefix $(OUTDIR)/, $(notdir $(addsuffix .lst, $(basename $<))))
ASFLAGS += $(patsubst %,-I%,$(EXTRAINCDIRS))

MATH_LIB = -lm

# Linker flags.
#  -Wl,...:     tell GCC to pass this to linker.
#    -Map:      create map file
#    --cref:    add cross reference to  map file
LDFLAGS += -nostartfiles -Wl,-Map=$(OUTDIR)/$(TARGET).map,--cref,--gc-sections
LDFLAGS += $(patsubst %,-L%,$(EXTRA_LIBDIRS))
LDFLAGS += $(patsubst %,-l%,$(EXTRA_LIBS))
LDFLAGS += $(MATH_LIB)

# Linker scripts
LDFLAGS += $(addprefix -T,$(LINKER_SCRIPTS_APP))

# Define programs and commands.
REMOVE  = $(REMOVE_CMD) -f
PYTHON  = python

# List of all source files.
ALLSRC     = $(ASRCARM) $(ASRC) $(SRCARM) $(SRC) $(CPPSRCARM) $(CPPSRC)
# List of all source files without directory and file-extension.
ALLSRCBASE = $(notdir $(basename $(ALLSRC)))

# Define all object files.
ALLOBJ     = $(addprefix $(OUTDIR)/, $(addsuffix .o, $(ALLSRCBASE)))

# Define all listing files (used for make clean).
LSTFILES   = $(addprefix $(OUTDIR)/, $(addsuffix .lst, $(ALLSRCBASE)))
# Define all depedency-files (used for make clean).
DEPFILES   = $(addprefix $(OUTDIR)/dep/, $(addsuffix .o.d, $(ALLSRCBASE)))

# Default target.
all: gccversion build

ifeq ($(LOADFORMAT),ihex)
build: elf hex lss sym
else
ifeq ($(LOADFORMAT),binary)
build: elf bin lss sym
else
ifeq ($(LOADFORMAT),both)
build: elf hex bin lss sym
else
$(error "$(MSG_FORMATERROR) $(FORMAT)")
endif
endif
endif

# Generate intermediate code
gencode: ${OUTDIR}/pmlib_img.c ${OUTDIR}/pmlib_nat.c ${OUTDIR}/pmlibusr_img.c ${OUTDIR}/pmlibusr_nat.c ${OUTDIR}/pmfeatures.h 

$(PYSRC): gencode

# Generate code for PyMite
${OUTDIR}/pmlib_img.c ${OUTDIR}/pmlib_nat.c ${OUTDIR}/pmlibusr_img.c ${OUTDIR}/pmlibusr_nat.c ${OUTDIR}/pmfeatures.h: $(wildcard ${PYMITELIB}/*.py) $(wildcard ${PYMITEPLAT}/*.py) $(wildcard ${FLIGHTPLANLIB}/*.py) $(wildcard ${FLIGHTPLANS}/*.py) 
	@echo $(MSG_PYMITEINIT) $(call toprel, $@)
	@$(PYTHON) $(PYMITETOOLS)/pmImgCreator.py -f $(PYMITEPLAT)/pmfeatures.py -c -s --memspace=flash -o $(OUTDIR)/pmlib_img.c --native-file=$(OUTDIR)/pmlib_nat.c $(PYMITELIB)/list.py $(PYMITELIB)/dict.py $(PYMITELIB)/__bi.py $(PYMITELIB)/sys.py $(PYMITELIB)/string.py $(wildcard $(FLIGHTPLANLIB)/*.py)
	@$(PYTHON) $(PYMITETOOLS)/pmGenPmFeatures.py $(PYMITEPLAT)/pmfeatures.py > $(OUTDIR)/pmfeatures.h
	@$(PYTHON) $(PYMITETOOLS)/pmImgCreator.py -f $(PYMITEPLAT)/pmfeatures.py -c -u -o $(OUTDIR)/pmlibusr_img.c --native-file=$(OUTDIR)/pmlibusr_nat.c $(FLIGHTPLANS)/test.py

# Link: create ELF output file from object files.
$(eval $(call LINK_TEMPLATE, $(OUTDIR)/$(TARGET).elf, $(ALLOBJ)))

# Assemble: create object files from assembler source files.
$(foreach src, $(ASRC), $(eval $(call ASSEMBLE_TEMPLATE, $(src))))

# Assemble: create object files from assembler source files. ARM-only
$(foreach src, $(ASRCARM), $(eval $(call ASSEMBLE_ARM_TEMPLATE, $(src))))

# Compile: create object files from C source files.
$(foreach src, $(SRC), $(eval $(call COMPILE_C_TEMPLATE, $(src))))

# Compile: create object files from C source files. ARM-only
$(foreach src, $(SRCARM), $(eval $(call COMPILE_C_ARM_TEMPLATE, $(src))))

# Compile: create object files from C++ source files.
$(foreach src, $(CPPSRC), $(eval $(call COMPILE_CPP_TEMPLATE, $(src))))

# Compile: create object files from C++ source files. ARM-only
$(foreach src, $(CPPSRCARM), $(eval $(call COMPILE_CPP_ARM_TEMPLATE, $(src))))

# Compile: create assembler files from C source files. ARM/Thumb
$(eval $(call PARTIAL_COMPILE_TEMPLATE, SRC))

# Compile: create assembler files from C source files. ARM only
$(eval $(call PARTIAL_COMPILE_ARM_TEMPLATE, SRCARM))

$(OUTDIR)/$(TARGET).bin.o: $(OUTDIR)/$(TARGET).bin

$(eval $(call OPFW_TEMPLATE,$(OUTDIR)/$(TARGET).bin,$(BOARD_TYPE),$(BOARD_REVISION)))

# Add jtag targets (program and wipe)
$(eval $(call JTAG_TEMPLATE,$(OUTDIR)/$(TARGET).bin,$(FW_BANK_BASE),$(FW_BANK_SIZE)))

.PHONY: elf lss sym hex bin bino opfw
elf:  $(OUTDIR)/$(TARGET).elf
lss:  $(OUTDIR)/$(TARGET).lss
sym:  $(OUTDIR)/$(TARGET).sym
hex:  $(OUTDIR)/$(TARGET).hex
bin:  $(OUTDIR)/$(TARGET).bin
bino: $(OUTDIR)/$(TARGET).bin.o
opfw: $(OUTDIR)/$(TARGET).opfw

# Display sizes of sections.
$(eval $(call SIZE_TEMPLATE, $(OUTDIR)/$(TARGET).elf))

# Generate Doxygen documents
docs:
	doxygen  $(DOXYGENDIR)/doxygen.cfg

# Install: install binary file with prefix/suffix into install directory
install: $(OUTDIR)/$(TARGET).opfw
ifneq ($(INSTALL_DIR),)
	@echo $(MSG_INSTALLING) $(call toprel, $<)
	$(V1) mkdir -p $(INSTALL_DIR)
	$(V1) $(INSTALL) $< $(INSTALL_DIR)/$(INSTALL_PFX)$(TARGET)$(INSTALL_SFX).opfw
else
	$(error INSTALL_DIR must be specified for $@)
endif

# Target: clean project.
clean: clean_list

clean_list :
	@echo $(MSG_CLEANING)
	$(V1) $(REMOVE) $(OUTDIR)/$(TARGET).map
	$(V1) $(REMOVE) $(OUTDIR)/$(TARGET).elf
	$(V1) $(REMOVE) $(OUTDIR)/$(TARGET).hex
	$(V1) $(REMOVE) $(OUTDIR)/$(TARGET).bin
	$(V1) $(REMOVE) $(OUTDIR)/$(TARGET).sym
	$(V1) $(REMOVE) $(OUTDIR)/$(TARGET).lss
	$(V1) $(REMOVE) $(OUTDIR)/$(TARGET).bin.o
	$(V1) $(REMOVE) $(wildcard $(OUTDIR)/*.c)
	$(V1) $(REMOVE) $(wildcard $(OUTDIR)/*.h)
	$(V1) $(REMOVE) $(ALLOBJ)
	$(V1) $(REMOVE) $(LSTFILES)
	$(V1) $(REMOVE) $(DEPFILES)
	$(V1) $(REMOVE) $(SRC:.c=.s)
	$(V1) $(REMOVE) $(SRCARM:.c=.s)
	$(V1) $(REMOVE) $(CPPSRC:.cpp=.s)
	$(V1) $(REMOVE) $(CPPSRCARM:.cpp=.s)


# Create output files directory
# all known MS Windows OS define the ComSpec environment variable
ifdef ComSpec
$(shell md $(subst /,\\,$(OUTDIR)) 2>NUL)
else
$(shell mkdir -p $(OUTDIR) 2>/dev/null)
endif

# Include the dependency files.
ifdef ComSpec
-include $(shell md $(subst /,\\,$(OUTDIR))\dep 2>NUL) $(wildcard $(OUTDIR)/dep/*)
else
-include $(shell mkdir -p $(OUTDIR) 2>/dev/null) $(shell mkdir $(OUTDIR)/dep 2>/dev/null) $(wildcard $(OUTDIR)/dep/*)
endif

# Listing of phony targets.
.PHONY : all build clean clean_list gencode install

