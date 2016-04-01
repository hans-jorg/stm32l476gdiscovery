
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
# CMSIS Dir
#
CMSISDIR=../STM32Cube_FW_L4_V1.2.0/Drivers/CMSIS

###############################################################################
# Generally it is not needed to modify the following lines                    #
###############################################################################

#
# Main target
#
default: all

#
# Coment to have verbose output
MAKEFLAGS+= --silent

#
# Folder for object files
#
OBJDIR=gcc

#
# Object files
#
OBJFILES=$(addprefix ${OBJDIR}/,$(SRCFILES:.c=.o))

# Set the compiler CPU/FPU options.
#
CPUFLAGS=-mcpu=cortex-m4
FPUFLAGS=-mfpu=fpv4-sp-d16 -mfloat-abi=softfp

#
# Include path
#
CMSISDEVINCDIR=${CMSISDIR}/Device/ST/$(PARTCLASSCMSIS)/Include
CMSISINCDIR=${CMSISDIR}/Include
INCLUDEPATH=${CMSISDEVINCDIR} ${CMSISINCDIR}

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
# Terminal application (used to open new windows in debug)
#
#TERMAPP=xterm
TERMAPP=gnome-terminal

#
# The command to flash the device
#
FLASHER=st-flash
#FLASHER=openocd

#
# Parameters for OpenOCD
#
OPENOCDBOARD=/usr/share/openocd/scripts/board/stm32l4discovery.cfg
OPENOCDFLASHSCRIPT=$(OBJDIR)/flash.ocd
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
# The flags passed to the linker.
#
LDFLAGS=--gc-sections

#
# Debugger
#
GDB=$(PREFIX)-gdb

#
# The flags passed to the debugger
#
GDBFLAGS=-x $(GDBINIT) -n

#
#
#
GDBINIT=$(OBJDIR)/gdbinit

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
# The command for extracting images from the linked executables.
#
OBJCOPY=${PREFIX}-objcopy

#
# The command for disassembly
#
OBJDUMP=${PREFIX}-objdump

#
# Flags for disassembler
#
ODFLAGS=-S -D

#
# The command for listing size of code
#
OBJSIZE=${PREFIX}-size

#
# The command for listing symbol table
#
OBJNM=${PREFIX}-nm

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
# Suppress warnings in protothread
#
CFLAGS+= -Wuninitialized -Werror

#
#
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

#
# Editor to be used
#
EDITOR=gedit

#
# Tool to generate documentation
#
DOXYGEN=doxygen
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



#
# The default rule, which causes the $(PROGNAME) example to be built.
#
all: ${OBJDIR} ${OBJDIR}/$(PROGNAME).bin

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
	sudo ${FLASHER} write $^ 0x8000000
else
deploy: ${OBJDIR}/$(PROGNAME).bin $(OPENOCDFLASHSCRIPT)
	sudo ${FLASHER} -f $(OPENOCDBOARD) -f $(OPENOCDFLASHSCRIPT)
endif

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
# List size
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
debug: $(OBJDIR)/$(PROGNAME).bin $(GDBINIT)
	$(GDB) $(GDBFLAGS) $(OBJDIR)/$(PROGNAME).axf

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
	$(DOXYGEN) doxygen.cfg *.c *.h

.PHONY: nm size dump disassembly deploy burn clean all editor doxygen docs
#
# Dependencies
#
-include $(OBJFILES:%.o=%.d)
