##
#
# Makefile for ARM Cortex cross-compiling
#
#
#  @file     Makefile
#  @brief    General Makefile for Cortex-M Processors
#  @version  V1.3
#  @date     24/10/2020
#
#  @note     CMSIS library used
#
#  @note     options
#   @param build       generate binary file
#   @param flash       transfer binary file to target (aliases=burn|deploy)
#   @param force-flash recover board when flash can not be written
#   @param disassembly generate assembly listing in a .dump file
#   @param size        list size of executable sections
#   @param nm          list symbols of executable
#   @param edit        open source files in a editor
#   @param gdbserver   start debug daemon (Start before debug session)
#   @param debug       enter a debug session (one of below)
#   @param  gdb        enter a debug session using gdb
#   @param  ddd        enter a debug session using ddd (GUI)
#   @param  nemiver    enter a debug session using nemiver (GUI)
#   @param  tui        enter a debug session using gdb in text UI
#   @param doxygen     generate doc files (alias=docs)
#   @param clean       clean all generated files
#   @param help        print options (default)
#
#  @note    Ported to Windows
#
#
###############################################################################
# Main parameters                                                             #
###############################################################################

#
# Compatibility Windows/Linux
#
ifeq ($(OS),Windows_NT)
HOSTOS :=Windows
else
HOSTOS :=$(shell uname -s)
endif
#
# Program name
#
PROGNAME=blink

#
# Defines the part type that this project uses.
#
PART=STM32L476
# Used to define part in header file
PARTCLASS=STM32L476xx
# Used to find correct CMSIS include file
PARTCLASSCMSIS=STM32L4xx

#
# Path
#
ifeq ($(HOSTOS),Windows)
# OpenOCD Dir
OPENOCDDIR=C:\Program Files (x86)\OpenOCD-20200729\share\openocd
# CMSIS Dir
CMSISDIR=C:\STM32Cube\STM32CubeL4\Drivers\CMSIS
else
# OpenOCD Dir
OPENOCDDIR=/usr/share/openocd
# CMSIS Dir
CMSISDIR=../../STM32CubeL4/Drivers/CMSIS
endif


#
# UCOS Configuration
#

# Micrium Dir
UCOS_DIR=../../../Micrium/Software
#UCOS_DIR=.

# uCOS-III is the main requisite
UCOS_SRCDIR=$(UCOS_DIR)/uCOS-III/Source
#UCOS_PORTDIR=$(UCOS_DIR)/Ports/ARM-Cortex-M3/Generic/GNU
UCOS_PORTDIR=$(UCOS_DIR)/uCOS-III/Ports/ARM-Cortex-M/GNU

# uC/CPU is needed by uC/OS
UCCPU_DIR=$(UCOS_DIR)/uC-CPU
#UCCPU_DIR=./uC-CPU
UCCPU_SRCDIR=$(UCCPU_DIR)
UCCPU_PORTDIR=$(UCCPU_DIR)/Ports/ARM-Cortex-M/GNU

# uC/Lib is needed by uC/CPU
UCLIB_DIR=$(UCOS_DIR)/uC-LIB
UCLIB_SRCDIR=$(UCLIB_DIR)
UCLIB_PORTDIR=$(UCLIB_DIR)/ARM-Cortex-M3/GNU


# Virtual path
VPATH=$(UCOS_SRCDIR):$(UCOS_PORTDIR) $(UCCPU_SRCDIR) $(UCCPU_PORTDIR) $(UCLIB_SRCDIR) $(UCLIB_PORTDIR)

# UCOS Include Path
UCOS_INCLUDEPATH=   $(UCOS_SRCDIR)  $(UCOS_PORTDIR) \
                    $(UCCPU_SRCDIR) $(UCCPU_PORTDIR)   \
                    $(UCLIB_SRCDIR) $(UCLIB_PORTDIR)

# UCOS Source Files
UCOS_FILES=         os_cfg_app.c    os_mutex.c      os_stat.c       os_tmr.c     \
                    os_core.c       os_mem.c        os_prio.c       os_task.c    \
                    os_dbg.c        os_q.c          os_tick.c    \
                    os_flag.c       os_msg.c        os_sem.c        os_time.c    \
                    os_var.c
UCOS_PORTCFILES=    os_cpu_c.c
UCOS_PORTASMFILES=  os_cpu_a.s

# uCCPU Source Files
UCCPU_FILES=        cpu_core.c
UCCPU_PORTFILES=    cpu_c.c
UCCPU_PORTASMFILES= cpu_a.s

# uCLIB Source Files
UCLIB_FILES=        lib_math.c      lib_mem.c       lib_str.c       lib_ascii.c
UCLIB_PORTFILES=
uCLIB_PORTASMFILES= libmem_a.s


# All ucOS Source Files
UCOS_SRCFILES=  $(addprefix $(UCOS_SRCDIR)/, $(UCOS_FILES))                  \
                $(addprefix $(UCOS_PORTDIR)/, $(UCOS_PORTCFILES))            \
                $(addprefix $(UCOS_PORTDIR)/, $(UCOS_PORTASMFILES))          \
                $(addprefix $(UCCPU_SRCDIR)/, $(UCCPU_FILES))                \
                $(addprefix $(UCCPU_PORTDIR)/, $(UCCPU_PORTFILES))           \
                $(addprefix $(UCCPU_PORTDIR)/, $(UCCPU_PORTASMFILES))        \
                $(addprefix $(UCLIB_SRCDIR)/, $(UCLIB_FILES))                \
                $(addprefix $(UCLIB_PORTDIR)/, $(UCLIB_PORTFILES))           \
                $(addprefix $(UCLIB_PORTDIR)/, $(UCLIB_PORTASMFILES))        \

# All ucOS Object Files
UCOS_OBJFILES=  $(addprefix $(OBJDIR)/, $(UCOS_FILES:.c=.o))                 \
                $(addprefix $(OBJDIR)/, $(UCOS_PORTCFILES:.c=.o))            \
                $(addprefix $(OBJDIR)/, $(UCOS_PORTASMFILES:.s=.o))          \
                $(addprefix $(OBJDIR)/, $(UCCPU_FILES:.c=.o))                \
                $(addprefix $(OBJDIR)/, $(UCCPU_PORTFILES:.c=.o))            \
                $(addprefix $(OBJDIR)/, $(UCCPU_PORTASMFILES:.s=.o))         \
                $(addprefix $(OBJDIR)/, $(UCLIB_FILES:.c=.o))                \
                $(addprefix $(OBJDIR)/, $(UCLIB_PORTFILES:.c=.o))            \
                $(addprefix $(OBJDIR)/, $(UCLIB_PORTASMFILES:.s=.o))



#
# Source files
#
SRCFILES=$(wildcard *.c)
#SRCFILES= main.c

#
# Suppress warnings
# Comment out to have verbose output
#MAKEFLAGS+= --silent
.SILENT:


#
# Include the common make definitions.
#
PREFIX:=arm-none-eabi


#
# Object files
#
OBJFILES=$(addprefix ${OBJDIR}/,$(notdir $(SRCFILES:.c=.o))) $(UCOS_OBJFILES)


#
# For gcc-arm-embedded
#

#--------------------------------------------------------------------------
#|   ARM core | Command Line Options                       | multilib     |
#| / ARM arch |                                            |              |
#|------------|--------------------------------------------|--------------|
#| Cortex-M0+ | -mthumb -mcpu=cortex-m0plus                | armv6-m      |
#| Cortex-M0  | -mthumb -mcpu=cortex-m0                    |              |
#| Cortex-M1  | -mthumb -mcpu=cortex-m1                    |              |
#|            |--------------------------------------------|              |
#|            | -mthumb -march=armv6-m                     |              |
#|------------|--------------------------------------------|--------------|
#| Cortex-M3  | -mthumb -mcpu=cortex-m3                    | armv7-m      |
#|            |--------------------------------------------|              |
#|            | -mthumb -march=armv7-m                     |              |
#|------------|--------------------------------------------|--------------|
#| Cortex-M4  | -mthumb -mcpu=cortex-m4                    | armv7e-m     |
#| (No FP)    |--------------------------------------------|              |
#|            | -mthumb -march=armv7e-m                    |              |
#|------------|--------------------------------------------|--------------|
#| Cortex-M4  | -mthumb -mcpu=cortex-m4 -mfloat-abi=softfp | armv7e-m     |
#| (Soft FP)  | -mfpu=fpv4-sp-d16                          | /softfp      |
#|            |--------------------------------------------|              |
#|            | -mthumb -march=armv7e-m -mfloat-abi=softfp |              |
#|            | -mfpu=fpv4-sp-d16                          |              |
#|------------|--------------------------------------------|--------------|
#| Cortex-M4  | -mthumb -mcpu=cortex-m4 -mfloat-abi=hard   | armv7e-m     |
#| (Hard FP)  | -mfpu=fpv4-sp-d16                          | /fpu         |
#|            |--------------------------------------------|              |
#|            | -mthumb -march=armv7e-m -mfloat-abi=hard   |              |
#|            | -mfpu=fpv4-sp-d16                          |              |
#|------------|--------------------------------------------|--------------|
#| Cortex-M7  | -mthumb -mcpu=cortex-m7                    | armv7e-m     |
#| (No FP)    |--------------------------------------------|              |
#|            | -mthumb -march=armv7e-m                    |              |
#|------------|--------------------------------------------|--------------|
#| Cortex-M7  | -mthumb -mcpu=cortex-m7 -mfloat-abi=softfp | armv7e-m     |
#| (Soft FP)  | -mfpu=fpv5-sp-d16                          | /softfp      |
#|            |--------------------------------------------| /fpv5-sp-d16 |
#|            | -mthumb -march=armv7e-m -mfloat-abi=softfp |              |
#|            | -mfpu=fpv5-sp-d16                          |              |
#|            |--------------------------------------------|--------------|
#|            | -mthumb -mcpu=cortex-m7 -mfloat-abi=softfp | armv7e-m     |
#|            | -mfpu=fpv5-d16                             | /softfp      |
#|            |--------------------------------------------| /fpv5-d16    |
#|            | -mthumb -march=armv7e-m -mfloat-abi=softfp |              |
#|            | -mfpu=fpv5-d16                             |              |
#|------------|--------------------------------------------|--------------|
#| Cortex-M7  | -mthumb -mcpu=cortex-m7 -mfloat-abi=hard   | armv7e-m     |
#| (Hard FP)  | -mfpu=fpv5-sp-d16                          | /fpu         |
#|            |--------------------------------------------| /fpv5-sp-d16 |
#|            | -mthumb -march=armv7e-m -mfloat-abi=hard   |              |
#|            | -mfpu=fpv5-sp-d16                          |              |
#|            |--------------------------------------------|--------------|
#|            | -mthumb -mcpu=cortex-m7 -mfloat-abi=hard   | armv7e-m     |
#|            | -mfpu=fpv5-d16                             | /fpu         |
#|            |--------------------------------------------| /fpv5-d16    |
#|            | -mthumb -march=armv7e-m -mfloat-abi=hard   |              |
#|            | -mfpu=fpv5-d16                             |              |
#|------------|--------------------------------------------|--------------|
#| ARMv8-M    | -mthumb -march=armv8-m.base                | armv8-m.base |
#| Baseline   |                                            |              |
#|------------|--------------------------------------------|--------------|
#| ARMv8-M    | -mthumb -march=armv8-m.main                | armv8-m.main |
#| Mainline   |                                            |              |
#| (No FP)    |                                            |              |
#|------------|--------------------------------------------|--------------|
#| ARMv8-M    | -mthumb -march=armv8-m.main                | armv8-m.main |
#| Mainline   | -mfloat-abi=softfp -mfpu=fpv5-sp-d16       | /softfp      |
#| (Soft FP)  |                                            | /fpv5-sp-d16 |
#|            |--------------------------------------------|--------------|
#|            | -mthumb -march=armv8-m.main                | armv8-m.main |
#|            | -mfloat-abi=softfp -mfpu=fpv5-d16          | /softfp      |
#|            |                                            | /fpv5-d16    |
#|------------|--------------------------------------------|--------------|
#| ARMv8-M    | -mthumb -march=armv8-m.main                | armv8-m.main |
#| Mainline   | -mfloat-abi=hard -mfpu=fpv5-sp-d16         | /fpu         |
#| (Hard FP)  |                                            | /fpv5-sp-d16 |
#|            |--------------------------------------------|--------------|
#|            | -mthumb -march=armv8-m.main                | armv8-m.main |
#|            | -mfloat-abi=hard -mfpu=fpv5-d16            | /fpu         |
#|            |                                            | /fpv5-d16    |
#|------------|--------------------------------------------|--------------|
#| Cortex-R4  | [-mthumb] -march=armv7-r                   | armv7-ar     |
#| Cortex-R5  |                                            | /thumb       |
#| Cortex-R7  |                                            |              |
#| Cortex-R8  |                                            |              |
#| (No FP)    |                                            |              |
#|------------|--------------------------------------------|--------------|
#| Cortex-R4  | [-mthumb] -march=armv7-r -mfloat-abi=softfp| armv7-ar     |
#| Cortex-R5  | -mfpu=vfpv3-d16                            | /thumb       |
#| Cortex-R7  |                                            | /softfp      |
#| Cortex-R8  |                                            |              |
#| (Soft FP)  |                                            |              |
#|------------|--------------------------------------------|--------------|
#| Cortex-R4  | [-mthumb] -march=armv7-r -mfloat-abi=hard  | armv7-ar     |
#| Cortex-R5  | -mfpu=vfpv3-d16                            | /thumb       |
#| Cortex-R7  |                                            | /fpu         |
#| Cortex-R8  |                                            |              |
#| (Hard FP)  |                                            |              |
#|------------|--------------------------------------------|--------------|
#| Cortex-A*  | [-mthumb] -march=armv7-a                   | armv7-ar     |
#| (No FP)    |                                            | /thumb       |
#|------------|--------------------------------------------|--------------|
#| Cortex-A*  | [-mthumb] -march=armv7-a -mfloat-abi=softfp| armv7-ar     |
#| (Soft FP)  | -mfpu=vfpv3-d16                            | /thumb       |
#|            |                                            | /softfp      |
#|------------|--------------------------------------------|--------------|
#| Cortex-A*  | [-mthumb] -march=armv7-a -mfloat-abi=hard  | armv7-ar     |
#| (Hard FP)  | -mfpu=vfpv3-d16                            | /thumb       |
#|            |                                            | /fpu         |
#--------------------------------------------------------------------------


# Set the compiler CPU/FPU options.
#
CPUFLAGS=-mcpu=cortex-m4
FPUFLAGS=-mfpu=fpv4-sp-d16 -mfloat-abi=softfp

#
# Terminal application (used to open new windows in debug)
#
#TERMAPP=xterm
TERMAPP=gnome-terminal

#
# Folder for object files
#
OBJDIR=gcc


#
# Main target
#
##default: help
default: build

#
# Default debugger
#
debug: gdb

#
# Serial terminal communication
#
TTYTERM=/dev/ttyACM0
TTYBAUD=9600
#
# Serial terminal emulator
#
# Use one of configuration below
# cu
#TTYPROG=cu
#TTYPARMS=-l $(TTYTERM) -s $(TTYBAUD)
# screen
#TTYPROG=screen
#TTYPARMS= $(TTYTERM) $(TTYBAUD)
# minicom
TTYPROG=minicom
TTYPARMS=-D $(TTYTERM) -b $(TTYBAUD)
# putty
#TTYPROG=putty
# tip
#TTYPROG=tip
#TTYPARMS=-$(TTYBAUD) $(TTYTERM)

#
# The command to flash the device
#
#
ifeq ($(HOSTOS),Windows)
FLASHER=ST-LINK_CLI
#FLASHER=openocd
else
FLASHER=st-flash
#FLASHER=openocd
endif


#
# Terminal application (used to open new windows in debug)
#
#TERMAPP=xterm
TERMAPP=gnome-terminal

#
# Editor to be used
#
EDITOR=gedit



###############################################################################
# Generally it is not needed to modify the lines below                        #
###############################################################################

###############################################################################
# Commands                                                                    #
###############################################################################

#
# The command for calling the compiler.
#
CC=${PREFIX}-gcc

#
# The command for calling the library archiver.
#
AR=${PREFIX}-ar

#
# The command for calling the linker.
#
LD=${PREFIX}-ld

#
# Debugger
#
GDB=$(PREFIX)-gdb

#
# Tool to generate documentation
#
DOXYGEN=doxygen

#
# The command for extracting images from the linked executables.
#
OBJCOPY=${PREFIX}-objcopy

#
# The command for disassembly
#
OBJDUMP=${PREFIX}-objdump

#
# The command for listing size of code
#
OBJSIZE=${PREFIX}-size

#
# The command for listing symbol table
#
OBJNM=${PREFIX}-nm


###############################################################################
# Commands parameters                                                         #
###############################################################################

#
# The flags passed to the linker.
#
LDFLAGS=--gc-sections

#
# The flags passed to the debugger
#
GDBFLAGS=-x $(GDBINIT) -n

#
# Parameters for OpenOCD
#
OPENOCD=openocd
OPENOCDBOARD=$(OPENOCDDIR)/scripts/board/stm32l4discovery.cfg
OPENOCDFLASHSCRIPT=$(OBJDIR)/flash.ocd

#
# Configuration file for GDB
#
GDBINIT=$(OBJDIR)/gdbinit

#
# Flags for disassembler
#
ODFLAGS=-S -D

###############################################################################
# Compilation parameters                                                      #
###############################################################################

#
# Get the location of libgcc.a from the GCC front-end.
#
LIBGCC:=${shell ${CC} ${CFLAGS} -print-libgcc-file-name}

#
# Get the location of libc.a from the GCC front-end.
#
LIBC:=${shell ${CC} ${CFLAGS} -print-file-name=libc.a}

#
# Get the location of libm.a from the GCC front-end.
#
LIBM:=${shell ${CC} ${CFLAGS} -print-file-name=libm.a}

#
# Object files
#
OBJFILES=$(addprefix ${OBJDIR}/,$(SRCFILES:.c=.o))
OBJFILES+= $(UCOS_OBJFILES)

#
# Include path
#
CMSISDEVINCDIR=${CMSISDIR}/Device/ST/$(PARTCLASSCMSIS)/Include
CMSISINCDIR=${CMSISDIR}/Include
INCLUDEPATH= . \
             $(UCOS_INCLUDEPATH) \
             ${CMSISDEVINCDIR} \
             ${CMSISINCDIR}

#
#
# The flags passed to the assembler.
#
AFLAGS=-mthumb \
       ${CPUFLAGS}  \
       ${FPUFLAGS}  \
       $(addprefix -I ,${INCLUDEPATH})           \

#
# The flags passed to the compiler.
#
CFLAGS=-mthumb             \
       ${CPUFLAGS}              \
       ${FPUFLAGS}              \
       $(addprefix -I ,${INCLUDEPATH})           \
       -D${PARTCLASS}            \
       -ffunction-sections \
       -fdata-sections     \
       -std=c99            \
       -pedantic           \
       -DPART_${PART}      \
        $(UCOS_FLAGS)

#
# Tell the compiler to include debugging information if the DEBUG environment
# variable is set.
#
ifneq ($DEBUG,)
CFLAGS+=-g -D DEBUG -O0
else
CFLAGS+=-Os
endif

#
# Flags needed to generate dependency information
#
DEPFLAGS=-MT $@  -MMD -MP -MF ${OBJDIR}/$(notdir $(@:.o=.d))

#
# Linker script
#
ifeq ($(HOSTOS),Windows)
#LINKERSCRIPT=$(PROGNAME).ld
LINKERSCRIPT= $(PART).ld
LINKERSCRIPT= stm32l476.ld
else
#LINKERSCRIPT=$(PROGNAME).ld
LINKERSCRIPT=$(shell echo $(PART)| tr A-Z a-z).ld
endif
#
# Entry Point
#
ENTRY=Reset_Handler

###############################################################################
# RULES                                                                       #
###############################################################################

#
# The rule for building the object file from each C source file.
#
${OBJDIR}/%.o: %.c
	@echo "  Compiling           ${notdir ${<}}";
	${CC} -c ${CFLAGS} -D${OBJDIR} ${DEPFLAGS} -o ${@} ${<}

#
# The rule for building the object file from each assembly source file.
#
${OBJDIR}/%.o: %.s
	@echo "  Assembling          ${notdir ${<}}";
	${CC} -c ${AFLAGS} -D${OBJDIR} -o ${@} -c ${<}
${OBJDIR}/%.o: %.S
	@echo "  Assembling          ${notdir ${<}}";
	${CC} -c ${AFLAGS} -D${OBJDIR} -o ${@} -c ${<}

#
# The rule for creating an object library.
#
${OBJDIR}/%.a:
	@echo "  Archiving           ${@}";
	${AR} -cr ${@} ${^}

#
# The rule for linking the application.
#
${OBJDIR}/%.axf:  $(OBJFILES)
	@echo "  Linking             ${@} ";
	${LD} -T '${LINKERSCRIPT}' --entry '${ENTRY}' \
	      ${LDFLAGS} -o ${@} $(OBJFILES) \
	      '${LIBM}' '${LIBC}' '${LIBGCC}'

${OBJDIR}/%.bin: ${OBJDIR}/%.axf
	@echo "  Generating binary   ${@} ";
	${OBJCOPY} -O binary ${^} ${@}


#
# Rules for compiling FreeRTOS
#
${OBJDIR}/tasks.o: ${FREERTOSDIR}/Source/tasks.c
	@echo "  Compiling           $(notdir ${<})";
	${CC} -c ${CFLAGS} -D${OBJDIR} ${DEPFLAGS} -o ${@} ${<}
${OBJDIR}/queue.o: ${FREERTOSDIR}/Source/queue.c
	@echo "  Compiling           $(notdir ${<})";
	${CC} -c ${CFLAGS} -D${OBJDIR} ${DEPFLAGS} -o ${@} ${<}
${OBJDIR}/list.o: ${FREERTOSDIR}/Source/list.c
	@echo "  Compiling           $(notdir ${<})";
	${CC} -c ${CFLAGS} -D${OBJDIR} ${DEPFLAGS} -o ${@} ${<}
${OBJDIR}/timers.o: ${FREERTOSDIR}/Source/timers.c
	@echo "  Compiling           $(notdir ${<})";
	${CC} -c ${CFLAGS} -D${OBJDIR} ${DEPFLAGS} -o ${@} ${<}
${OBJDIR}/croutine.o: ${FREERTOSDIR}/Source/croutine.c
	@echo "  Compiling           $(notdir ${<})";
	${CC} -c ${CFLAGS} -D${OBJDIR} ${DEPFLAGS} -o ${@} ${<}
${OBJDIR}/heap_1.o:${FREERTOSDIR}/Source/portable/MemMang/heap_1.c
	@echo "  Compiling           $(notdir ${<})";
	${CC} -c ${CFLAGS} -D${OBJDIR} ${DEPFLAGS} -o ${@} ${<}
${OBJDIR}/heap_2.o:${FREERTOSDIR}/Source/portable/MemMang/heap_2.c
	@echo "  Compiling           $(notdir ${<})";
	${CC} -c ${CFLAGS} -D${OBJDIR} ${DEPFLAGS} -o ${@} ${<}
${OBJDIR}/heap_3.o:${FREERTOSDIR}/Source/portable/MemMang/heap_3.c
	@echo "  Compiling           $(notdir ${<})";
	${CC} -c ${CFLAGS} -D${OBJDIR} ${DEPFLAGS} -o ${@} ${<}
${OBJDIR}/heap_4.o:${FREERTOSDIR}/Source/portable/MemMang/heap_4.c
	@echo "  Compiling           $(notdir ${<})";
	${CC} -c ${CFLAGS} -D${OBJDIR} ${DEPFLAGS} -o ${@} ${<}
${OBJDIR}/heap_5.o:${FREERTOSDIR}/Source/portable/MemMang/heap_5.c
	@echo "  Compiling           $(notdir ${<})";
	${CC} -c ${CFLAGS} -D${OBJDIR} ${DEPFLAGS} -o ${@} ${<}
${OBJDIR}/port.o:${FREERTOSDIR}/Source/portable/GCC/ARM_CM4F/port.c
	@echo "  Compiling           $(notdir ${<})";
	${CC} -c ${CFLAGS} -D${OBJDIR} ${DEPFLAGS} -o ${@} ${<}


###############################################################################
# TARGETS                                                                     #
###############################################################################

#
# help menu
#
help: usage
usage:
	@echo "Options are:"
	@echo "build:       generate binary file"
	@echo "flash:       transfer binary file to target (aliases=burn|deploy)"
	@echo "force-flash: recover board when flash can not be written"
	@echo "disassembly: generate assembly listing in a .dump file"
	@echo "size:        list size of executable sections"
	@echo "nm:          list symbols of executable"
	@echo "edit:        open source files in a editor"
	@echo "gdbserver:   start debug daemon (Start before debug session)"
	@echo "debug:       enter a debug session (one of below)"
	@echo " gdb:        enter a debug session using gdb"
	@echo " ddd:        enter a debug session using ddd (GUI)"
	@echo " nemiver:    enter a debug session using nemiver (GUI)"
	@echo " tui:        enter a debug session using gdb in text UI"
	@echo "doxygen:     generate doc files (alias=docs)"
	@echo "clean:       clean all generated files"
	@echo "help:        print options (default)"

#
# The default rule, which causes the $(PROGNAME) example to be built.
#
build: ${OBJDIR} ${OBJDIR}/$(PROGNAME).bin
	echo "Done."

#
# The rule to clean out all the build products.
#
ifeq ($(HOSTOS),Windows)
clean:
	del /q /f /s  ${wildcard *~}  > nul 2>&1
	rmdir /q /f /s  html latex docs $(OBJDIR) > nul 2>&1
	echo "Done."
else
clean:
	rm -rf ${OBJDIR} ${wildcard *~} html latex docs
	echo "Done."
endif

#
# The rule to transfer binary to board
#
# For Linux st-flash or OpenOCD
# For Windows ST_LINK_CLI

flash: deploy
burn: deploy
ifeq ($(FLASHER),st-flash)
deploy: ${OBJDIR}/$(PROGNAME).bin
	sudo ${FLASHER} write $^ 0x08000000
endif
ifeq ($(FLASHER),openocd)
deploy: ${OBJDIR}/$(PROGNAME).bin $(OPENOCDFLASHSCRIPT)
	sudo ${FLASHER} -f $(OPENOCDBOARD) -f $(OPENOCDFLASHSCRIPT)
endif
ifeq ($(FLASHER),ST-LINK_CLI)
deploy: ${OBJDIR}/$(PROGNAME).bin
	${FLASHER} -c swd -p $^ 0x08000000 -Rst -v
endif

#
# Force write to flash memory. Useful in case of recurring write errors
#

ifeq ($(FLASHER),st-flash)
force-flash: ${OBJDIR}/$(PROGNAME).bin
	echo "Press RESET during write"
	sleep 3
	sudo ${FLASHER} --reset write  $^ 0x8000000
endif
ifeq ($(FLASHER),openocd)
	echo "Press RESET during write"
	sleep 3
	sudo ${FLASHER} --reset write  $^ 0x8000000
endif
ifeq ($(FLASHER),ST-LINK_CLI)
force-flash: ${OBJDIR}/$(PROGNAME).bin
	echo "Press RESET during write"
	sleep 3
	${FLASHER} -c swd -p $^ 0x08000000 -Rst -v
endif
#
# Disassembling
#
disassembly:$(OBJDIR)/$(PROGNAME).dump
dump: disassembly
$(OBJDIR)/$(PROGNAME).dump: $(OBJDIR)/$(PROGNAME).axf
	@echo "  Disassembling       ${^} and storing in $(OBJDIR)/$(PROGNAME).dump"
	$(OBJDUMP) $(ODFLAGS) $^ > $(OBJDIR)/$(PROGNAME).dump

#
# List size
#
size: $(OBJDIR)/$(PROGNAME).axf
	$(OBJSIZE) $^

#
# List symbols
#
nm: $(OBJDIR)/$(PROGNAME).axf
	$(OBJNM) $^

#
# The rule to create the target directory.
#
ifeq ($(HOSTOS),Windows)
${OBJDIR}:
	mkdir ${OBJDIR}
else
${OBJDIR}:
	mkdir -p ${OBJDIR}
endif

#
# Rules for building the $(PROGNAME) example.
#
${OBJDIR}/$(PROGNAME).bin: ${OBJDIR}/$(PROGNAME).axf
${OBJDIR}/$(PROGNAME).axf: ${OBJFILES}

#
# Open files in editor windows
#
edit:
	$(EDITOR) Makefile *.c *.h *.ld &

#
# Debug command
#
gdb: $(OBJDIR)/$(PROGNAME).bin $(GDBINIT)
	$(GDB) $(GDBFLAGS) $(OBJDIR)/$(PROGNAME).axf

#
# iDebug command with text UI
#
tui: $(OBJDIR)/$(PROGNAME).bin $(GDBINIT)
	$(GDB) -tui $(GDBFLAGS) $(OBJDIR)/$(PROGNAME).axf

#
# iDebug command with text UI
#
cgdb: $(OBJDIR)/$(PROGNAME).bin $(GDBINIT)
	cgdb -d `which $(GDB)`  -x $(OBJDIR)/gdbinit $(OBJDIR)/$(PROGNAME).axf


#
# Debug using GUI
#
ddd: $(OBJDIR)/$(PROGNAME).bin $(GDBINIT)
	ddd --debugger "$(GDB) $(GDBFLAGS)" $(OBJDIR)/$(PROGNAME).axf

#
# Debug using kdbg GUI
#
#kdbg: $(OBJDIR)/$(PROGNAME).bin $(GDBINIT)
	kdbg  -r localhost:4242 $(OBJDIR)/$(PROGNAME).axf

#
# Debug using nemiver GUI
#
nemiver: $(OBJDIR)/$(PROGNAME).bin $(GDBINIT)
	nemiver  --remote=localhost:4242  --gdb-binary=`which $(GDB)`     $(OBJDIR)/$(PROGNAME).axf


#
# Start debug demon
#
gdbserver:
ifeq ($(FLASHER),st-flash)
	if [ X"`pidof st-util`" != X ]; then kill `pidof st-util`; fi
endif
ifeq ($(FLASHER),openocd)
	$(TERMAPP) -e "openocd -x XXXXXXXXXXX" &
endif
ifeq ($(FLASHER),ST-LINK_CLI)
	$(TERMAPP) -e "openocd -x XXXXXXXXXXX" &
endif


#
# Debugger initialization scripts
#
$(GDBINIT):
ifeq ($(FLASHER),st-flash)
	echo "# Run this script using gdb source command" > $(GDBINIT)
	echo "target extended-remote localhost:4242" >> $(GDBINIT)
	echo "break main" >> $(GDBINIT)
	echo "continue" >> $(GDBINIT)
endif
ifeq ($(FLASHER),openocd)
	echo "# Run this script using gdb source command" > $(GDBINIT)
	echo "target remote localhost:3333" >> $(GDBINIT)
	echo "break main" >> $(GDBINIT)
	echo "continue" >> $(GDBINIT)
endif
ifeq ($(FLASHER),ST-LINK_CLI)
	echo "# Run this script using gdb source command" > $(GDBINIT)
	echo "target extended-remote localhost:4242" >> $(GDBINIT)
	echo "break main" >> $(GDBINIT)
	echo "continue" >> $(GDBINIT)
endif

$(OPENOCDFLASHSCRIPT):
	echo "reset halt" > $(OPENOCDFLASHSCRIPT)
	echo "flash probe 0" >> $(OPENOCDFLASHSCRIPT)
	echo "flash write_image erase ${OBJDIR}/$(PROGNAME).bin 0" >> $(OPENOCDFLASHSCRIPT)
	echo "reset run" >> $(OPENOCDFLASHSCRIPT)

$(OPENOCDDEBUGSCRIPT):
	echo "monitor reset halt" > $(OPENOCDFLASHSCRIPT)
	echo "monitor flash probe 0" >> $(OPENOCDFLASHSCRIPT)
	echo "monitor flash write_image erase ${OBJDIR}/$(PROGNAME).bin 0" >> $(OPENOCDFLASHSCRIPT)
	echo "monitor reset run" >> $(OPENOCDFLASHSCRIPT)


#
# Generate documentation using doxygen
#
docs: doxygen
doxygen:
	$(DOXYGEN) *.c *.h

#
# These labels are not files !!!
#
.PHONY: nm size dump disassembly deploy burn clean editor force-flash gdbserver
.PHONY: help build nemiver ddd gdb tui doxygen docs flash

#
# Dependencies
#
-include $(OBJFILES:%.o=%.d)
