ROOT=$(shell pwd)
PROJ_NAME = stm32f3_HID_for_real


###################################################
# Set toolchain
TC = arm-none-eabi

# Set Tools
CC			= $(TC)-gcc
AR			= $(TC)-ar
OBJCOPY		= $(TC)-objcopy
OBJDUMP		= $(TC)-objdump
SIZE		= $(TC)-size

###################################################
# Set Sources
USB_SRCS	= $(wildcard Libraries/STM32_USB-FS-Device_Driver/src/*.c)
LIB_SRCS	= $(wildcard Libraries/STM32F30x_StdPeriph_Driver/src/*.c) $(wildcard Libraries/STM32F3_Discovery/*.c) 
USER_SRCS	= $(wildcard src/*.c) $(wildcard src/*/*.c)

# Set Objects
#LIB_OBJS	= $(LIB_SRCS:.c=.o)
#USER_OBJS	= $(USER_SRCS:.c=.o) startup.o
#USER_OBJS	= $(USER_SRCS:.c=.o) src/startup_stm32f30x.o

USER_SOURCES = $(wildcard src/*.c)
STM32F3_Discovery_SOURCES = $(wildcard Libraries/STM32F3_Discovery/*.c)
STM32F30x_StdPeriph_Driver_SOURCES = $(wildcard Libraries/STM32F30x_StdPeriph_Driver/src/*.c)
STM32_USBFS_Driver_SOURCES = $(wildcard Libraries/STM32_USB-FS-Device_Driver/src/*.c)

SOURCES = $(USER_SOURCES) \
		  $(STM32F3_Discovery_SOURCES) \
		  $(STM32F30x_StdPeriph_Driver_SOURCES) \
		  $(STM32_USBFS_Driver_SOURCES)


# Set Include Paths
INCLUDES 	= -I./inc/ \
			-I./Libraries/CMSIS/Device/ST/STM32F30x/Include/ \
			-I./Libraries/STM32F30x_StdPeriph_Driver/inc/ \
			-I./Libraries/STM32_USB-FS-Device_Driver/inc/ \
			-I./Libraries/STM32F3_Discovery/ \
			-I./Libraries/CMSIS/Include/
					
# Set Libraries
LIBS		= -lm -lc

###################################################
# Set Board
MCU 		= -mthumb -mcpu=cortex-m4
FPU 		= -mfpu=fpv4-sp-d16 -mfloat-abi=hard
DEFINES 	= -DSTM32F3XX -DUSE_STDPERIPH_DRIVER -DARM_MATH_CM4
DEBUG		= -g
OPTIMIZE	= -O0

# Set Compilation and Linking Flags
CFLAGS 		= $(MCU) $(FPU) $(DEBUG) -Wall -std=gnu90 $(OPTIMIZE) -ffunction-sections -fdata-sections
FPU_CFLAGS 	= -L/home/jasdf/bin/gcc-arm-none-eabi-4_6-2012q2/lib/fpu/
ASFLAGS 	= $(MCU) $(FPU) $(DEBUG) -Wa,--warn -x assembler-with-cpp
LDFLAGS 	= $(MCU) $(FPU) $(DEBUG) -gdwarf-2 -Tstm32f30_flash.ld -Xlinker --gc-sections -Wl,-Map=$(PROJ_NAME).map $(LIBS)
CPPFLAGS	= $(INCLUDES) $(DEFINES)

###################################################
# Default Target

.SILENT: 

all: $(PROJ_NAME).bin info

# Clean Target
clean:
	rm -f $(PROJ_NAME).elf
	rm -f $(PROJ_NAME).bin
	rm -f $(PROJ_NAME).map
	@rm -f *.d *.o *.a $(subst .c,.d,$(SOURCES)) $(subst .c,.o,$(SOURCES))


stm32f3_discovery.a: $(subst .c,.o,$(STM32F3_Discovery_SOURCES))
	@echo [A] $@
	@$(AR) rcs $@ $^

stm32f30x_stdperiph_driver.a: $(subst .c,.o,$(STM32F30x_StdPeriph_Driver_SOURCES))
	@echo [A] $@
	@$(AR) rcs $@ $^

stm32_usbfs_driver.a: $(subst .c,.o,$(STM32_USBFS_Driver_SOURCES))
	@echo [A] $@
	@$(AR) rcs $@ $^

# elf Target
$(PROJ_NAME).elf: $(subst .c,.o,$(USER_SOURCES)) stm32f3_discovery.a stm32f30x_stdperiph_driver.a stm32_usbfs_driver.a startup.o
	@echo [O] $@
	@$(CC) $(LDFLAGS) $(FPU_CFLAGS) -nostartfiles $^ -o $@ 

-include $(subst .c,.d,$(SOURCES))

%.d: %.c
	@echo [CC] $@
	@$(CC) $(CFLAGS) $(FPU_CFLAGS) -M $(CPPFLAGS) $< > $@.$$$$;\
	sed 's,\($*\)\.o[ :]*,\1.o $@ : ,g' < $@.$$$$ > $@; \
	rm -f $@.$$$$;

startup.o: ./Libraries/CMSIS/Device/ST/STM32F30x/Source/Templates/gcc_ride7/startup_stm32f30x.s
	@echo [CC] $@
	@$(CC) -c $(ASFLAGS) $(FPU_CFLAGS) $(CPPFLAGS) $< -o $@

# bin Target
$(PROJ_NAME).bin: $(PROJ_NAME).elf
	@echo [O] $@
	@$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin

# Display Memory Usage Info
info: $(PROJ_NAME).elf
	@$(SIZE) --format=berkeley $(PROJ_NAME).elf

install-ocd: $(PROJ_NAME).elf
	@$(shell killall -9 openocd)
	@$(shell openocd -f /usr/local/share/openocd/scripts/board/stm32f3discovery.cfg -c init -c"reset halt" -c"flash erase_sector 0 0 127" -c"flash write_image $(PROJ_NAME).elf" -c"reset init" -c"reset run" -c"exit")

#ocd-shell: stm32f3.bin
#	telnet localhost 4444

#$(PROJ_NAME).hex: $(PROJ_NAME).elf
#	@$(OBJCOPY) -O ihex $(PROJ_NAME).elf $(PROJ_NAME).hex
#	@echo $@

#$(PROJ_NAME).lst: $(PROJ_NAME).elf
#	@$(OBJDUMP) -h -S $(PROJ_NAME).elf > $(PROJ_NAME).lst
#	@echo $@

