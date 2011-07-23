# uDMX MAKEFILE
#version 1.1 2006-08-20 me@anyma.ch
#version 1.2 2011-07-22 malte@maltepoeggel.de

# derived from:
# Makefile for AVR function library development and examples
# Author: Pascal Stang
#
# For those who have never heard of makefiles: a makefile is essentially a
# script for compiling your code.  Most C/C++ compilers in the world are
# command line programs and this is even true of programming environments
# which appear to be windows-based (like Microsoft Visual C++).  Although
# you could use AVR-GCC directly from the command line and try to remember
# the compiler options each time, using a makefile keeps you free of this
# tedious task and automates the process.
#
# For those just starting with AVR-GCC and not used to using makefiles,
# I've added some extra comments above several of the makefile fields which
# you will have to deal with.

########### change this lines according to your project ##################
#put the name of the target mcu here (at90s8515, at90s8535, attiny22, atmega603 etc.)
	
	MCU = atmega8
	F_CPU = 12000000
	
	AVRDUDE = avrdude -c usbasp -p $(MCU)


#put the name of the target file here (without extension)
#  Your "target" file is your C source file that is at the top level of your code.
#  In other words, this is the file which contains your main() function.

	TRG = uDMX


#put your C sourcefiles here 
#  Here you must list any C source files which are used by your target file.
#  They will be compiled in the order you list them, so it's probably best
#  to list $(TRG).c, your top-level target file, last.

	SRC = usbdrv/usbdrv.c $(TRG).c


#put additional assembler source file here
#  The ASRC line allows you to list files which contain assembly code/routines that
#  you would like to use from within your C programs.  The assembly code must be
#  written in a special way to be usable as a function from your C code.

	ASRC = usbdrv/usbdrvasm.S


#additional libraries and object files to link
#  Libraries and object files are collections of functions which have already been
#  compiled.  If you have such files, list them here, and you will be able to use
#  use the functions they contain in your target program.

	LIB	=

#additional includes to compile
	INC	= 

#assembler flags
	ASFLAGS = -Wa, -gstabs

#compiler flags
	CPFLAGS	= -g -Os -Wall -Wstrict-prototypes -I$(AVRLIB) -Wa,-ahlms=$(<:.c=.lst)

#linker flags
	LDFLAGS = -Wl,-Map=$(TRG).map,--cref
#	LDFLAGS = -Wl,-Map=$(TRG).map,--cref -lm

	
########### you should not need to change the following  #############
#----------------------------------------------------------------------------------
# ARM-GCC standard Makefile
# This makefile is to be used by including it from a project-specific makefile
# which defines the source files and compiler/linker options
#
# Written by Pascal Stang
# Based on Volker Oth's AVR makefiles of jan.2000
# ---------------------------------------------------------------------------------



###### BLOCK 1) define some variables based on the AVR base path in $(AVR) #######

	CC	= avr-gcc
	AS	= avr-gcc -x assembler-with-cpp	
	RM	= rm -f
	RN	= mv
	CP	= cp
	BIN	= avr-objcopy
	SIZE	= avr-size
	INCDIR	= .
#	LIBDIR	= $(AVR)/avr/lib
#	SHELL   = $(AVR)/bin/sh.exe


###### BLOCK 2) output format can be srec, ihex (avrobj is always created) #######

	FORMAT = ihex


###### BLOCK 3) define all project specific object files ######

	SRC	+= $(addprefix $(AVRLIB)/,$(AVRLIB_SRC))
	OBJ	= $(ASRC:.s=.o) $(SRC:.c=.o) 
	CPFLAGS += -DF_CPU=$(F_CPU) -mmcu=$(MCU)
	ASFLAGS += -DF_CPU=$(F_CPU) -mmcu=$(MCU)
	LDFLAGS += -DF_CPU=$(F_CPU) -mmcu=$(MCU)
  
###### BLOCK 4) this defines the aims of the make process ######

#all:	$(TRG).obj $(TRG).elf $(TRG).hex  $(TRG).cof $(TRG).eep $(TRG).ok
all:	$(TRG).elf $(TRG).hex $(TRG).eep $(TRG).ok


###### BLOCK 5) compile: instructions to create assembler and/or object files from C source ######

%.o : %.c 
	$(CC) -c $(CPFLAGS) -I$(INCDIR) $< -o $@

%.s : %.c
	$(CC) -S $(CPFLAGS) -I$(INCDIR) $< -o $@


###### BLOCK 6) assemble: instructions to create object file from assembler files ######

%.o : %.s
	$(AS) -c $(ASFLAGS) -I$(INCDIR) $< -o $@


###### BLOCK 7)  link: instructions to create elf output file from object files ######
%.elf: $(OBJ)
	$(CC) $(OBJ) $(LIB) $(LDFLAGS) -o $@

###### BLOCK 8) create avrobj file from elf output file ######

%.obj: %.elf
	$(BIN) -O avrobj -R .eeprom $< $@


###### BLOCK 9) create bin (.hex and .eep) files from elf output file ######

%.hex: %.elf
	$(BIN) -O $(FORMAT) -R .eeprom $< $@

%.eep: %.elf
	$(BIN) -j .eeprom --set-section-flags=.eeprom="alloc,load" --change-section-lma .eeprom=0 -O $(FORMAT) $< $@

%.cof: %.elf
	$(BIN)  -O ihex \
		--change-section-address   .data-0x800000 \
		--change-section-address    .bss-0x800000 \
		--change-section-address .noinit-0x800000 \
		--change-section-address .eeprom-0x810000 \
		$< $@


###### BLOCK 10) If all other steps compile ok then echo "Errors: none" ######

%ok:
	$(SIZE) $(TRG).elf
	@echo "Errors: none" 


###### BLOCK 11)  make instruction to delete created files ######

clean:
	$(RM) $(TRG).o
	$(RM) $(SRC:.c=.s)
	$(RM) $(SRC:.c=.lst)
	$(RM) $(TRG).map
	$(RM) $(TRG).elf
	$(RM) $(TRG).cof
	$(RM) $(TRG).obj
	$(RM) $(TRG).a90
	$(RM) $(TRG).hex
	$(RM) $(TRG).sym
	$(RM) $(TRG).eep
	$(RM) $(TRG).hex
	$(RM) *.bak
	$(RM) *.log
	@echo "Errors: none"
	
size:
	$(SIZE) $(TRG).elf

flash:	all
	$(AVRDUDE) -U flash:w:$(TRG).hex
	
fuse:
	$(AVRDUDE) -U hfuse:w:0xc0:m -U lfuse:w:0xbf:m
	
###### dependecies, add any dependencies you need here ###################
#  Dependencies tell the compiler which files in your code depend on which
#  other files.  When you change a piece of code, the dependencies allow
#  the compiler to intelligently figure out which files are affected and
#  need to be recompiled.  You should only list the dependencies of *.o 
#  files.  For example: uart.o is the compiled output of uart.c and uart.h
#  and therefore, uart.o "depends" on uart.c and uart.h.  But the code in
#  uart.c also uses information from global.h, so that file should be listed
#  in the dependecies too.  That way, if you alter global.h, uart.o will be
#  recompiled to take into account the changes.

$(TRG).o		: $(TRG).c
