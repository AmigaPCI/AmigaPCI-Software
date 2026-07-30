#
# Makefile to build AmigaOS/68k tools using Bebbo's GCC cross-compiler.
#

BEC_PROG     := bec
BECKY_PROG   := becky
BEC_TOOL     := bectool
FLASH_PROG   := apciflash
ACONF_PROG   := apciaconf
ASCAN_PROG   := apciscan
PCI_PROG     := pci
APCIROM_PROG := apcirom

ALL_PROGS    := $(BEC_PROG) $(BECKY_PROG) $(FLASH_PROG) $(ACONF_PROG) $(ASCAN_PROG) $(PCI_PROG) $(APCIROM_PROG)

OBJDIR       := objs
ROM_OBJDIR   := objs.rom
CRC32_C      := ../fw/crc32.c
BEC_SRCS     := bec.c becmsg.c $(CRC32_C)
BEC_HDRS     := becmsg.h ../fw/crc32.h ../fw/bec_cmd.h
BECKY_SRCS   := becky.c becmsg.c $(CRC32_C)
BECKY_HDRS   := becmsg.h ../fw/crc32.h ../fw/bec_cmd.h \
	        ../fw/amiga_kbd_codes.h ../fw/hid_kbd_codes.h
FLASH_SRCS   := apciflash.c cpu_control.c
FLASH_HDRS   := cpu_control.h
ACONF_SRCS   := apciaconf.c
ACONF_HDRS   :=
ASCAN_SRCS   := apciscan.c pci_access.c
ASCAN_HDRS   := pci_access.h
PCI_SRCS     := pci.c pci_access.c strtox.c
PCI_HDRS     := pci_access.h
APCIROM_SRCS := apcirom.c my_createtask.c apcibanner.c printf.c rom_end.c
APCIROM_HDRS :=

CC      := m68k-amigaos-gcc
STRIP   := m68k-amigaos-strip
CFLAGS  := -Wall -Wextra -Wno-pointer-sign -Wno-format -Wno-strict-aliasing
CFLAGS  += -Wno-sign-compare -fomit-frame-pointer -I../fw -mcpu=68060 -DAMIGAOS
CFLAGS_ROM := -DAPCIROM -fbaserel -resident -mcpu=68060
LDFLAGS = -Xlinker -Map=$(OBJDIR)/$@.map -Wa,-a > $(OBJDIR)/$@.lst -mcrt=clib2 -lgcc -lc -lamiga
LDFLAGS_ROM = -nostdlib -lgcc -lc -lamiga -Xlinker --verbose -Tapcirom.ld -mcrt=clib2 -fbaserel

BUILD_TIMESTAMP := $(shell date '+%Y-%m-%d %H:%M:%S')
DATE := $(word 1,$(BUILD_TIMESTAMP))
TIME := $(word 2,$(BUILD_TIMESTAMP))
CFLAGS += -DBUILD_DATE=\"$(DATE)\" -DBUILD_TIME=\"$(TIME)\"

VERSION := $(shell awk '/Version/{print $$2}' ../fw/version.c)
CFLAGS += -DVERSION=\"$(VERSION)\"
PROGVER := bec_$(VERSION)

NDK_PATH := /opt/amiga/m68k-amigaos/ndk-include
VASM     := vasmm68k_mot

CFLAGS  += -Os
QUIET   := @

#LDFLAGS += -g
#CFLAGS  += -g

# If verbose is specified with no other targets, then build everything
ifeq ($(MAKECMDGOALS),verbose)
verbose: all
endif
ifeq (,$(filter verbose timed, $(MAKECMDGOALS)))
QUIET   := @
else
QUIET   :=
endif

ifeq (, $(shell which $(CC) 2>/dev/null ))
$(error "No $(CC) in PATH: maybe do PATH=$$PATH:/opt/amiga13/bin")
endif

all: $(ALL_PROGS)
	@:

gdb:
	m68k-amigaos-gdb $(BEC_PROG)

TEST :=
define DEPEND_SRC
# The following line creates a rule for an object file to depend on a
# given source file.
$(patsubst %,$(2)/%,$(filter-out $(2)/%,$(basename $(notdir $(1))).o)) $(filter $(2)/%,$(basename $(1)).o): $(1)
# The following line adds that object to macro containing the list of objects
$(3) += $(patsubst %,$(2)/%,$(filter-out $(2)/%,$(basename $(notdir $(1))).o))
endef

$(foreach SRCFILE,$(BEC_SRCS),$(eval $(call DEPEND_SRC,$(SRCFILE),$(OBJDIR),BEC_OBJS)))
$(foreach SRCFILE,$(BECKY_SRCS),$(eval $(call DEPEND_SRC,$(SRCFILE),$(OBJDIR),BECKY_OBJS)))
$(foreach SRCFILE,$(FLASH_SRCS),$(eval $(call DEPEND_SRC,$(SRCFILE),$(OBJDIR),FLASH_OBJS)))
$(foreach SRCFILE,$(ACONF_SRCS),$(eval $(call DEPEND_SRC,$(SRCFILE),$(OBJDIR),ACONF_OBJS)))
$(foreach SRCFILE,$(ASCAN_SRCS),$(eval $(call DEPEND_SRC,$(SRCFILE),$(OBJDIR),ASCAN_OBJS)))
$(foreach SRCFILE,$(PCI_SRCS),$(eval $(call DEPEND_SRC,$(SRCFILE),$(OBJDIR),PCI_OBJS)))
$(foreach SRCFILE,$(APCIROM_SRCS),$(eval $(call DEPEND_SRC,$(SRCFILE),$(ROM_OBJDIR),APCIROM_OBJS)))

OBJS := $(sort $(BEC_OBJS) $(BECKY_OBJS) $(FLASH_OBJS) $(ACONF_OBJS) $(ASCAN_OBJS) $(PCI_OBJS) $(APCIROM_OBJS))

$(BEC_OBJS): $(BEC_HDRS)
$(BECKY_OBJS): $(BECKY_HDRS)
$(FLASH_OBJS): $(FLASH_HDRS)
$(ACONF_OBJS): $(ACONF_HDRS)
$(ASCAN_OBJS): $(ASCAN_HDRS)
$(PCI_OBJS): $(PCI_HDRS)
$(APCIROM_OBJS): $(APCIROM_HDRS) | $(ROM_OBJDIR)
$(APCIROM_OBJS):: CFLAGS += $(CFLAGS_ROM) -DNO_DEBUG
$(ACONF_PROG):: LDFLAGS := -Xlinker -Map=$(OBJDIR)/$@.map -Wa,-a -noixemul > $(OBJDIR)/$@.lst

$(OBJS): Makefile | $(OBJDIR)
	@echo Building $@
	$(QUIET)$(CC) $(CFLAGS) -c $(filter %.c,$^) -Wa,-a,-ad >$(@:.o=.lst) -o $@

$(BEC_PROG): $(BEC_OBJS)
$(BECKY_PROG): $(BECKY_OBJS)
$(FLASH_PROG): $(FLASH_OBJS)
$(ACONF_PROG): $(ACONF_OBJS)
$(ASCAN_PROG): $(ASCAN_OBJS)
$(PCI_PROG): $(PCI_OBJS)
$(BEC_TOOL): $(BEC_OBJS)
$(APCIROM_PROG): $(APCIROM_OBJS) apcirom.ld

$(BEC_PROG) $(BECKY_PROG) $(FLASH_PROG) $(ACONF_PROG) $(ASCAN_PROG) $(PCI_PROG) $(BEC_TOOL):
	@echo Building $@
	$(QUIET)$(CC) $^ $(LDFLAGS) -o $@

$(APCIROM_PROG):
	@echo Building $@
	$(QUIET)$(CC) $(filter %.o,$^) $(LDFLAGS_ROM) -Xlinker -Map=$(ROM_OBJDIR)/$@.map -Wa,-a,-ad > $(ROM_OBJDIR)/$@.lst -nostartfiles -o $(ROM_OBJDIR)/$@
	$(QUIET)$(STRIP) -o $@ $(ROM_OBJDIR)/$@

$(OBJDIR) $(ROM_OBJDIR):
	mkdir -p $@

ZIPFILE := $(PROGVER).zip
LHAFILE := $(PROGVER).lha
DISK	:= $(PROGVER).adf

zip:
	@echo Building $(ZIPFILE)
	$(QUIET)rm -rf $(ZIPFILE) $(PROGVER)
	$(QUIET)mkdir $(PROGVER)
	$(QUIET)cp -p $(ALL_PROGS) $(PROGVER)/
	$(QUIET)zip -rq $(ZIPFILE) $(PROGVER)
	$(QUIET)rm -rf $(PROGVER)

lha:
	@echo Building $(LHAFILE)
	$(QUIET)rm -rf $(LHAFILE) $(PROGVER)
	$(QUIET)mkdir $(PROGVER)
	$(QUIET)cp -p $(ALL_PROGS) $(PROGVER)/
	$(QUIET)lha -aq2 $(LHAFILE) $(PROGVER)
	$(QUIET)rm -rf $(PROGVER)

adf:
	xdftool $(DISK) format "$(PROGVER)"
	xdftool $(DISK) makedir c
	xdftool $(DISK) write bec c/bec
	xdftool $(DISK) boot install

clean clean-all:
	@echo Cleaning
	@rm -rf $(OBJS) $(OBJDIR) $(ROM_OBJDIR)

FLINT_FILE=flexelint.lnt
flint:
	flexelint -v -w3 -I/opt/amiga/m68k-amigaos/ndk-include -I/opt/amiga/m68k-amigaos/sys-include -I/opt/amiga/m68k-amigaos/clib2/include flexelint.lnt $(BEC_SRCS)
