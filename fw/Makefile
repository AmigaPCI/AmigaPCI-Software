# AmigaPCI STM32 Rev 1

BOARD_REV ?= 1

SRCS	:= main.c gpio.c led.c timer.c printf.c uart.c \
	   mem_access.c readline.c cmdline.c cmds.c pcmds.c \
	   utils.c scanf.c stm32flash.c version.c config.c \
	   clock.c crc32.c crc8.c usb.c kbrst.c keyboard.c mouse.c \
	   adc.c fan.c irq.c power.c rtc.c sensor.c amigartc.c msg.c \
	   hiden.c joystick.c i2c.c button.c spi.c spi_flash.c
SRCS    += libopencm3_stm32f2/adc_common_v1.c \
	   libopencm3_stm32f2/adc_common_v1_multi.c \
	   libopencm3_stm32f2/adc_common_f47.c
# STM32F2 library is missing ADC source and headers

#STACK := tinyusb
STACK := stmcubemx

# TinyUSB support
ifeq ($(STACK),tinyusb)
TINYUSB_DIR := tinyusb
TU	 := $(TINYUSB_DIR)/src
TU_HDR   := $(TU)/tusb.h
USB_SRCS := $(wildcard $(TU)/*.c $(TU)/*/*.c $(TU)/*/*/*.c $(TU)/*/*/*/*.c tinyusb.c tinyusb_hid.c)
DEFS	     += -I. -I$(TU) -DCFG_TUSB_MCU=OPT_MCU_STM32F2 -DUSE_TINYUSB
USB_DEFS     := -std=c23 -Ist-micro -Icubemx/Drivers/CMSIS/Include \
		-Icubemx/Drivers/CMSIS/Device/ST/STM32F2xx/Include \
		-DCFG_TUSB_CONFIG_FILE=\"tusb_config.h\"
USB_HEADERS  := tusb_config.h
ifeq (,$(wildcard $(TU_HDR)))
SKIP_LINK := 1
endif
endif

# ST-Micro CubeMX support
ifeq ($(STACK),stmcubemx)
CUBE	 := cubemx
CUBEUHL	 := $(CUBE)/Middlewares/ST/STM32_USB_Host_Library
CUBEHAL  := $(CUBE)/Drivers/STM32F2xx_HAL_Driver/Src
DEFS	 += -DUSE_STMCUBEUSB
USB_SRCS := $(wildcard $(CUBEUHL)/*/*/*.c $(CUBEUHL)/*/*/*/*.c \
	      $(CUBEHAL)/stm32f2xx_hal_hcd.c $(CUBEHAL)/stm32f2xx_ll_usb.c \
	      cubeusb.c)

USB_SRCS := $(filter-out %usbh_conf_template.c,$(USB_SRCS))
USB_DEFS += \
	    -Ist-micro \
	    -I$(CUBEUHL)/Core/Inc -I$(CUBE)/USB_HOST/Target -I. \
	    -I$(CUBEUHL)/Class/AUDIO/Inc \
	    -I$(CUBEUHL)/Class/HUB \
	    -I$(CUBEUHL)/Class/CDC/Inc -I$(CUBEUHL)/Class/HID/Inc \
	    -I$(CUBEUHL)/Class/MSC/Inc -I$(CUBEUHL)/Class/MTP/Inc \
	    -I$(CUBEUHL)/Class/XUSB \
	    -I$(CUBE)/Drivers/CMSIS/Device/ST/STM32F2xx/Include \
	    -I$(CUBE)/Drivers/CMSIS/Include \
	    -I$(CUBE)/Drivers/STM32F2xx_HAL_Driver/Inc \
	    -I$(CUBE)/Core/Inc -I$(CUBE)/USB_HOST/App \
	    -Wno-maybe-uninitialized -Wno-redundant-decls
USB_HEADERS := $(wildcard $(CUBEUHL)/*/*/*.h $(CUBEUHL)/*/*/*/*.h \
	         $(CUBEHAL)/../Inc/*.h)
endif

OBJDIR := objs
#OBJS   := $(SRCS:%.c=$(OBJDIR)/%.o)
EXTRA_OBJDIRS := $(OBJDIR)/libopencm3_stm32f2

QUIET  := @
VERBOSE :=

# If verbose is specified with no other targets, then build everything
ifeq ($(MAKECMDGOALS),verbose)
verbose: all
endif
ifeq (,$(filter verbose timed, $(MAKECMDGOALS)))
QUIET   := @
else
QUIET   :=
VERBOSE := -v
endif

# libopencm3
OPENCM3_DIR := /usr/lib/libopencm3
ifeq (,$(wildcard $(OPENCM3_DIR)))
OPENCM3_DIR := libopencm3
endif
OPENCM3_HEADER := $(OPENCM3_DIR)/include/libopencm3/stm32/rcc.h

all: bin size

define DEPEND_SRC
# The following line creates a rule for an object file to depend on a
# given source file.
$(patsubst %,$(2)/%,$(filter-out $(2)/%,$(basename $(1)).o)): $(1)
# The following line adds that object to macro containing the list of objects
$(3) += $(patsubst %,$(2)/%,$(filter-out $(2)/%,$(basename $(1)).o))
USB_OBJDIRS += $(OBJDIR)/$(dir $(1))
endef

$(foreach SRCFILE,$(SRCS),$(eval $(call DEPEND_SRC,$(SRCFILE),$(OBJDIR),OBJS)))
$(foreach SRCFILE,$(USB_SRCS),$(eval $(call DEPEND_SRC,$(SRCFILE),$(OBJDIR),USB_OBJS)))
USB_OBJDIRS := $(sort $(USB_OBJDIRS))
$(USB_OBJS): | $(USB_OBJDIRS)
$(USB_OBJS):: CFLAGS += $(USB_DEFS)
$(USB_OBJS): $(USB_HEADERS)

# Our output name
BINARY = $(OBJDIR)/fw

# Linker script for our MCU
LDSCRIPT = fw.ld

LIBNAME		:= opencm3_stm32f2
ARCH_FLAGS	:= -mthumb -mcpu=cortex-m3 -msoft-float -mfix-cortex-m3-ldrd
DEFS		+= -DSTM32F2 -DSTM32F205 -DSTM32F205xx
DEFS		+= -DEMBEDDED_CMD

OPENCM3_LIB := $(OPENCM3_DIR)/lib/lib$(LIBNAME).a

# Where the Black Magic Probe is attached
BMP_PORT = /dev/ttyACM0

CUBE_PORT ?= /dev/ttyUSB0
CUBE_CLI  ?= /usr/local/STMCubeProgrammer/bin/STM32_Programmer_CLI
DFU_UTIL=dfu-util
ST_BUILD_DIR=/usr/stlink
ifeq (,$(wildcard $(ST_BUILD_DIR)))
ST_BUILD_DIR=stutils
endif
ST_TOOLS_PATH=$(ST_BUILD_DIR)/build/Release/bin
OS := $(shell uname -s)

NOW  := $(shell date +%s)
ifeq ($(OS),Darwin)
DATE := $(shell date -j -f %s $(NOW)  '+%Y-%m-%d')
TIME := $(shell date -j -f %s $(NOW)  '+%H:%M:%S')
else
DATE := $(shell date -d "@$(NOW)" '+%Y-%m-%d')
TIME := $(shell date -d "@$(NOW)" '+%H:%M:%S')
endif


## Boilerplate

# Compiler configuration
PREFIX		?= arm-none-eabi
CC		:= $(PREFIX)-gcc
CXX		:= $(PREFIX)-g++
LD		:= $(PREFIX)-gcc
AR		:= $(PREFIX)-ar
AS		:= $(PREFIX)-as
SIZE		:= $(PREFIX)-size
OBJCOPY		:= $(PREFIX)-objcopy
OBJDUMP		:= $(PREFIX)-objdump
GDB		:= $(PREFIX)-gdb
STFLASH		= $(shell which st-flash)
OPT		:= -O2
DEBUG		:= -ggdb3
CSTD		?= -std=gnu11

# C flags
TGT_CFLAGS	+= $(OPT) $(CSTD) $(DEBUG)
TGT_CFLAGS	+= $(ARCH_FLAGS)
TGT_CFLAGS	+= -Wextra -Wshadow -Wimplicit-function-declaration
TGT_CFLAGS	+= -Wredundant-decls -Wmissing-prototypes -Wstrict-prototypes
TGT_CFLAGS	+= -fno-common -ffunction-sections -fdata-sections
TGT_CFLAGS	+= -ffreestanding
TGT_CFLAGS	+= -DBUILD_DATE=\"$(DATE)\" -DBUILD_TIME=\"$(TIME)\"
TGT_CFLAGS	+= -Wa,-a > $(@:.o=.lst)

# C++ flags
TGT_CXXFLAGS	+= $(OPT) $(CXXSTD) $(DEBUG)
TGT_CXXFLAGS	+= $(ARCH_FLAGS)
TGT_CXXFLAGS	+= -Wextra -Wshadow -Wredundant-decls -Weffc++
TGT_CXXFLAGS	+= -fno-common -ffunction-sections -fdata-sections
TGT_CXXFLAGS	+= -std=c++11

# C & C++ preprocessor common flags
TGT_CPPFLAGS	+= -MD
TGT_CPPFLAGS	+= -Wall -Wundef -pedantic
TGT_CPPFLAGS	+= $(DEFS)
TGT_CPPFLAGS	+= -Wno-unused-parameter -Wno-variadic-macros
#TGT_CPPFLAGS	+= -fanalyzer -Wno-analyzer-out-of-bounds

# Linker flags
TGT_LDFLAGS		+= --static -nostartfiles -nostdlib
TGT_LDFLAGS		+= -T$(LDSCRIPT)
TGT_LDFLAGS		+= $(ARCH_FLAGS) $(DEBUG)
TGT_LDFLAGS		+= -Wl,-Map=$(OBJDIR)/$*.map -Wl,--cref
TGT_LDFLAGS		+= -Wl,--gc-sections
TGT_LDFLAGS		+= -Wl,-no-warn-rwx-segments
ifeq ($(V),99)
TGT_LDFLAGS		+= -Wl,--print-gc-sections
endif

#TGT_CFLAGS	+= -g
#TGT_LDFLAGS	+= -g

# Used libraries
DEFS		+= -Ilibopencm3_stm32f2/include -I$(OPENCM3_DIR)/include
LDFLAGS		+= -L$(OPENCM3_DIR)/lib
LDLIBS		+= -l$(LIBNAME)
LDLIBS		+= -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group

.SUFFIXES: .elf .bin .hex .srec .list .map .images

size: $(BINARY).size
elf: $(BINARY).elf
bin: $(BINARY).bin
hex: $(BINARY).hex
srec: $(BINARY).srec
list: $(BINARY).list
flashbmp: $(BINARY).flashbmp

$(OBJDIR)/%.bin: $(OBJDIR)/%.elf
	@echo Building $@
	$(QUIET)$(OBJCOPY) -Obinary $< $@

$(OBJDIR)/%.hex: $(OBJDIR)/%.elf
	@echo Building $@
	$(QUIET)$(OBJCOPY) -Oihex $< $@

$(OBJDIR)/%.srec: $(OBJDIR)/%.elf
	@echo Building $@
	$(QUIET)$(OBJCOPY) -Osrec $< $@

$(OBJDIR)/%.list: $(OBJDIR)/%.elf
	@echo Building $@
	$(QUIET)$(OBJDUMP) -S $< > $@

$(OBJDIR)/%.elf: $(OBJS) $(USB_OBJS) $(LDSCRIPT) $(OPENCM3_LIB)
	@echo Building $@
ifeq (,$(SKIP_LINK))
	$(QUIET)$(LD) $(TGT_LDFLAGS) $(LDFLAGS) $(OBJS) $(USB_OBJS) $(LDLIBS) -o $@
endif

# Get argument options for objcopy -O
#     arm-none-eabi-ld --print-output-format
# Get argument options for objcopy -B
#     arm-none-eabi-ld --verbose | sed -n 's/OUTPUT_ARCH(\([^()*]*\))/\1/p'
# Show sections and symbols in usbdfu_bin.o
#     arm-none-eabi-objdump -x objs/usbdfu_bin.o

$(OBJDIR)/%.o: %.c
	@echo Building $@
	$(QUIET)$(CC) $(TGT_CFLAGS) $(CFLAGS) $(TGT_CPPFLAGS) $(CPPFLAGS) -o $@ -c $(*).c

$(OBJDIR)/%.o: %.cxx
	@echo Building $@
	$(QUIET)$(CXX) $(TGT_CXXFLAGS) $(CXXFLAGS) $(TGT_CPPFLAGS) $(CPPFLAGS) -o $@ -c $(*).cxx

$(OBJDIR)/%.o: %.cpp
	@echo Building $@
	$(QUIET)$(CXX) $(TGT_CXXFLAGS) $(CXXFLAGS) $(TGT_CPPFLAGS) $(CPPFLAGS) -o $@ -c $(*).cpp

$(OBJDIR)/%.size: $(OBJDIR)/%.elf
	@echo "Output code size:"
	@$(SIZE) -A -d $< | grep -Ee 'text|data|bss' | awk ' \
    function human(x) { \
        if (x<1000) {return x} else {x/=1024} \
        s="kMGTEPZY"; \
        while (x>=1000 && length(s)>1) \
            {x/=1024; s=substr(s,2)} \
        return int(x+0.5) substr(s,1,1) \
    } \
	{printf("%10s %8s\n", $$1, human($$2))} \
'
$(OBJDIR)/version.o: $(filter-out $(OBJDIR)/version.o, $(OBJS) $(USB_OBJS))

$(sort $(OBJS) $(USB_OBJS)): Makefile $(OPENCM3_HEADER) $(OPENCM3_DIR)/include/libopencm3/stm32/f2/nvic.h | $(OBJDIR) $(EXTRA_OBJDIRS)

$(OBJDIR) $(USB_OBJDIRS) $(EXTRA_OBJDIRS):
	$(QUIET)mkdir -p $@

$(OPENCM3_DIR)/Makefile:
	@echo Cloning LibOpenCM3 for $@
	git clone https://github.com/libopencm3/libopencm3
	@(cd libopencm3; git checkout 87a080c94ce67643216464821c752c1c406c6414)

$(OPENCM3_DIR)/include/%: | $(OPENCM3_DIR)/Makefile
	@echo Building $@
	$(QUIET)make -C $(OPENCM3_DIR) -j8

$(OPENCM3_LIB): $(OPENCM3_HEADER)

ifeq ($(STACK),tinyusb)
$(TU_HDR):
	@echo Cloning TinyUSB for $@
	git clone https://github.com/hathach/tinyusb
	# Build a second time now that all sources can be discovered
	$(MAKE) $(MAKEFLAGS)

$(USB_OBJS): $(TU_HDR)
endif

%.flashbmp: %.elf
	$(GDB) --batch \
		-ex 'target extended-remote $(BMP_PORT)' \
		-x bmp_flash.scr \
		$(OBJDIR)/$*.elf

clean clean-all:
	@echo Cleaning
	$(QUIET)$(RM) -rf $(OBJDIR)

# Dependencies
-include $(OBJS:.o=.d)

UDEV_DIR        := /etc/udev/rules.d
UDEV_FILENAMES  := 70-st-link.rules
UDEV_FILE_PATHS := $(UDEV_FILENAMES:%=$(UDEV_DIR)/%)
ifneq (,$(wildcard $(UDEV_DIR)))
$(UDEV_FILE_PATHS) &:
	sudo cp -np udev/* $(UDEV_DIR)
	sudo udevadm control --reload
endif

# Write STM32 flash using DFU mode
just-dfu:
	$(DFU_UTIL) --device 0483:df11 --alt 0 --download $(BINARY).bin --dfuse-address 0x08000000:leave

just-dfuser:
	$(CUBE_CLI) -c port=$(CUBE_PORT) br=115200 -v -w $(BINARY).bin 0x08000000
	$(CUBE_CLI) -c port=$(CUBE_PORT) br=115200 -g 0x08000000

dfu-unprotect: $(UDEV_FILE_PATHS)
	$(DFU_UTIL) -s 0:force:unprotect -a 0 --device 0483:df11 --alt 0 --download $(BINARY).bin

dfu-clobber: $(UDEV_FILE_PATHS)
	$(DFU_UTIL) -s 0:force:unprotect --alt 1 --download flash_clobber.bin -Z 0x10 --dfuse-address 0x1ffff800:leave

#dfu-clobber: $(UDEV_FILE_PATHS)
#	$(DFU_UTIL) -s 0:force:unprotect --alt 1 --download flash_clobber.bin -Z 0xd --dfuse-address 0x1ffff800:leave

# Write STM32 flash using ST-Link
just-flash: $(ST_TOOLS_PATH)/st-flash | $(BINARY).bin $(UDEV_FILE_PATHS)
	$(ST_TOOLS_PATH)/st-flash $(ST_ARGS) --reset write $(BINARY).bin 0x08000000

# Erase STM32 flash using ST-Link
just-erase: $(ST_TOOLS_PATH)/st-flash | $(UDEV_FILE_PATHS)
	$(ST_TOOLS_PATH)/st-flash $(ST_ARGS) --flash=0x100000 erase 0x08000000 0x3c000

# Protect/unprotect/clobber flash by writing the option byte area
# The following can also be done by the STM32CubeProgrammer.
# F105 unlocked  1ffff800: a5 5a 07 f8 00 ff 00 ff ff 00 ff 00 ff 00 ff 00
# F105 protect   1ffff800: 00 ff 07 f8 ff 00 ff 00 00 ff 00 ff 00 ff 00 ff
# F105 clobber   1ffff800: 00 00 00 00 00 03 00 03 00 ff 00 ff 00 ff 00 ff
#
# F205 unlocked  1fffc000: ff aa 00 55 ff aa 00 55 ff ff 00 00 ff ff 00 00
# F205 protect   1fffc000: ff aa 00 55 ff aa 00 55 00 f0 00 00 ff ff 00 00

# Select specific programmer ("make stinfo" to get serial)
# export ST_ARGS="--serial 57FF6E064975545225502187"		# amiga1
# export ST_ARGS="--serial 303030303030303030303031"		# amiga2

just-unprotect: $(ST_TOOLS_PATH)/st-flash | $(UDEV_FILE_PATHS)
	$(ST_TOOLS_PATH)/st-flash $(ST_ARGS) --reset write flash_unprotect.bin 0x1fffc000
just-protect: $(ST_TOOLS_PATH)/st-flash | $(UDEV_FILE_PATHS)
	$(ST_TOOLS_PATH)/st-flash $(ST_ARGS) --reset write flash_protect.bin 0x1fffc000
just-clobber: $(ST_TOOLS_PATH)/st-flash | $(UDEV_FILE_PATHS)
	$(ST_TOOLS_PATH)/st-flash $(ST_ARGS) --reset write flash_clobber.bin 0x1fffc000

dfu: all $(UDEV_FILE_PATHS) just-dfu
dfuser: all just-dfuser
flash: all just-flash

# Connect to remote STM32 via ST-Link (follow with "make gdb")
stlink: $(ST_TOOLS_PATH)/st-flash | $(UDEV_FILE_PATHS)
	$(ST_TOOLS_PATH)/st-util $(ST_ARGS) --no-reset

stinfo reset: $(ST_TOOLS_PATH)/st-flash | $(UDEV_FILE_PATHS)
	$(ST_TOOLS_PATH)/st-info --probe

# Get ST-TOOLS
$(ST_BUILD_DIR) get-stutils:
	@echo Cloning stutils
	git clone https://github.com/texane/stlink.git stutils

# Build ST-TOOLS
$(ST_TOOLS_PATH)/st-flash build-stutils: | $(ST_BUILD_DIR)
	@echo Building stutils
	make -C $(ST_BUILD_DIR) CMAKEFLAGS="-DCMAKE_INSTALL_PREFIX=. -DCMAKE_INSTALL_FULL_DATADIR=." -j4
	ln -s ../.. $(ST_BUILD_DIR)/build/Release/stlink
	ln -s . $(ST_BUILD_DIR)/build/Release/share

gdb:
	gdb -q -x .gdbinit $(BINARY).elf

.PHONY: images clean get-stutils build_stutils stlink dfu flash just-flash just-unprotect just-dfu just-dfuser dfu-unprotect size elf bin hex srec list udev-files verbose
