HOSTBEC_PROG=hostbec
HOSTBEC_SRCS=hostbec.c ../fw/version.c ../fw/crc32.c
CC := gcc
CFLAGS  := -O2 -pthread -Wall -Wpedantic -Wno-variadic-macros
LDFLAGS := -O2 -lpthread
OS := $(shell uname -s)

NOW  := $(shell date +%s)
ifeq ($(OS),Darwin)
DATE := $(shell date -j -f %s $(NOW)  '+%Y-%m-%d')
TIME := $(shell date -j -f %s $(NOW)  '+%H:%M:%S')
else
DATE := $(shell date -d "@$(NOW)" '+%Y-%m-%d')
TIME := $(shell date -d "@$(NOW)" '+%H:%M:%S')
USB_HDR := /usr/include/usb.h
endif
CFLAGS += -DBUILD_DATE=\"$(DATE)\" -DBUILD_TIME=\"$(TIME)\"

VERSION := $(shell awk '/Version/{print $$2}' ../fw/version.c)
CFLAGS += -DVERSION=\"$(VERSION)\"

# Get the default target operating system
ifneq (,$(filter $(OS),Windows_NT Windows win))
    TARGET_OS ?= Windows_NT
else
    TARGET_OS ?= $(OS)
endif

# Windows
ifneq (,$(filter $(TARGET_OS),Windows_NT Windows win win32 win64))
    CFLAGS := $(filter-out -DALLOW_CREATE_LINK, $(CFLAGS))
    # -static works for Vista+, but doesn't work for XP
    LDFLAGS += -lws2_32 -lwinmm -static
    OBJDIR := objs.win64
    HOSTBEC_SRCS += mingw-realpath.c
    CC := x86_64-w64-mingw32-gcc
endif
ifneq (,$(filter $(TARGET_OS),win32))
    # Windows 32-bit
    LDFLAGS += -Lmingw_7.5.0_i686  # old version of pthread library for Win XP
    OBJDIR := objs.win32
    CC := i686-w64-mingw32-gcc
endif
ifneq (,$(filter $(TARGET_OS),Windows_NT Windows win win32 win64))
    HOSTBEC_PROG := $(HOSTBEC_PROG).exe
endif

# Linux
ifeq ($(TARGET_OS),Linux)
    CFLAGS += -DLINUX
    UNAME_M := $(shell uname -m)
    OBJDIR := objs.$(UNAME_M)
endif

# MacOS
ifeq ($(TARGET_OS),Darwin)
#   CFLAGS += -DOSX -target x86_64-apple-macos10.12
    CFLAGS += -DOSX #-lxml2 -I/Library/Developer/CommandLineTools/SDKs/MacOSX.sdk/usr/include/libxml2
    OBJDIR := objs.mac
endif

#CFLAGS += -fanalyzer

HOSTBEC_OPROG := $(OBJDIR)/$(HOSTBEC_PROG)

#ifneq ($(TARGET_OS),$(OS))
#    $(info HOST=$(OS) TARGET=$(TARGET_OS))
#endif

# If verbose is specified with no other targets, then build everything
ifeq ($(MAKECMDGOALS),verbose)
verbose: all
endif
ifeq (,$(filter verbose timed, $(MAKECMDGOALS)))
QUIET   := @
else
QUIET   :=
endif

#HOSTBEC_OBJS  := $(HOSTBEC_SRCS:%.c=$(OBJDIR)/%.o)

nativeprog: $(HOSTBEC_OPROG)
	@:

all: $(HOSTBEC_OPROG) win32 win64
	@:

win32:
	$(MAKE) TARGET_OS=win32
win win64:
	$(MAKE) TARGET_OS=win64

define DEPEND_SRC
# The following line creates a rule for an object file to depend on a
# given source file.
$(patsubst %,$(2)/%,$(filter-out $(2)/%,$(basename $(notdir $(1))).o)) $(filter $(2)/%,$(basename $(1)).o): $(1)
# The following line adds that object to macro containing the list of objects
$(3) += $(patsubst %,$(2)/%,$(filter-out $(2)/%,$(basename $(notdir $(1))).o))
endef

$(foreach SRCFILE,$(HOSTBEC_SRCS),$(eval $(call DEPEND_SRC,$(SRCFILE),$(OBJDIR),HOSTBEC_OBJS)))

$(OBJDIR)/hostbec.o: | $(USB_HDR)
$(OBJDIR)/version.o: $(filter-out $(OBJDIR)/version.o,$(HOSTBEC_OBJS)) Makefile

$(HOSTBEC_OPROG): $(HOSTBEC_OBJS)
	@echo Building $@
	$(QUIET)$(CC) -o $@ $(HOSTBEC_OBJS) $(LDFLAGS)
	@rm -f $(HOSTBEC_PROG)
	@ln -s $@

$(HOSTBEC_OBJS): Makefile | $(OBJDIR)
	@echo Building $@
	$(QUIET)$(CC) $(CFLAGS) -c $(filter %.c,$^) -o $@

$(OBJDIR):
	@echo Creating $@
	$(QUIET)mkdir $@

$(USB_HDR):
	echo "You must install the libusb development package"
	echo "On Fedora:   dnf install libusb-devel"
	echo "On Raspbian: apt-get install libusb-dev"
	exit 1

clean:
	@echo Cleaning
	$(QUIET)rm -rf $(HOSTBEC_OPROG) $(OBJDIR)

clean-all: clean
	@$(MAKE) TARGET_OS=win32 clean
	@$(MAKE) TARGET_OS=win64 clean

verbose:

.PHONY: all clean clean-all verbose nativeprog
