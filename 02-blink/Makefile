##
#
# Makefile for ARM Cortex cross-compiling
#
#
#  @file     Makefile
#  @brief    General Makefile for Cortex-M Processors
#  @version  V1.2
#  @date     20/04/2017
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
#

###############################################################################
# Main parameters                                                             #
###############################################################################

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
# Source files
#
SRCFILES=$(wildcard *.c)
#SRCFILES= main.c


#
# Include the common make definitions.
#
PREFIX:=arm-none-eabi

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
default: help
#default: build

#
# Default debugger
#
debug: gdb

#
# Suppress warnings
# Comment out to have verbose output
#MAKEFLAGS+= --silent
.SILENT:

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
FLASHER=st-flash
#FLASHER=openocd


#
# Terminal application (used to open new windows in debug)
#
#TERMAPP=xterm
TERMAPP=gnome-terminal

#
# Editor to be used
#
EDITOR=gedit

#
# Path
#
OPENOCDDIR=/usr/share/openocd
# CMSIS Dir
CMSISDIR=../../STM32Cube_FW_L4_V1.2.0/Drivers/CMSIS


#
# Additional libraries
#
#

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

#
# Include path
#
CMSISDEVINCDIR=${CMSISDIR}/Device/ST/$(PARTCLASSCMSIS)/Include
CMSISINCDIR=${CMSISDIR}/Include
INCLUDEPATH=${CMSISDEVINCDIR} ${CMSISINCDIR} 

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
       -DPART_${PART}

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
DEPFLAGS=-MT $@  -MMD -MP -MF ${OBJDIR}/$*.d

#
# Linker script
#
#LINKERSCRIPT=$(PROGNAME).ld
LINKERSCRIPT=$(shell echo $(PART)| tr A-Z a-z).ld

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
	@echo "  Compiling           ${<}";
	${CC} -c ${CFLAGS} -D${OBJDIR} ${DEPFLAGS} -o ${@} ${<}

#
# The rule for building the object file from each assembly source file.
#
${OBJDIR}/%.o: %.S
	@echo "  Assembling          ${<}";
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

#
# The rule to clean out all the build products.
#
clean:
	rm -rf ${OBJDIR} ${wildcard *~} html latex docs  && echo "Done."

#
# The rule to transfer binary to board
#
flash: deploy
burn: deploy
ifeq ($(FLASHER),st-flash)
deploy: ${OBJDIR}/$(PROGNAME).bin
	sudo ${FLASHER} write $^ 0x08000000
else
deploy: ${OBJDIR}/$(PROGNAME).bin $(OPENOCDFLASHSCRIPT)
	sudo ${FLASHER} -f $(OPENOCDBOARD) -f $(OPENOCDFLASHSCRIPT)
endif

#
# Force write to flash memory. Useful in case of recurring write errors
#
force-flash: ${OBJDIR}/$(PROGNAME).bin
	echo "Press RESET during write"
	sleep 3
	sudo ${FLASHER} --reset write  $^ 0x8000000

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
${OBJDIR}:
	mkdir -p ${OBJDIR}

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
#	kdbg  -r localhost:4242 $(OBJDIR)/$(PROGNAME).axf

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
	$(TERMAPP) -e "st-util" &
else
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
else
	echo "# Run this script using gdb source command" > $(GDBINIT)
	echo "target remote localhost:3333" >> $(GDBINIT)
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
