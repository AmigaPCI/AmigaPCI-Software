#
# Makefile to build APCI STM32 firmware, host software, and Amiga software
#

MAKE ?= make
MAKEFLAGS += --no-print-directory

ifeq (,$(VER))
VERSION := $(shell awk '/Version/{print $$2}' fw/version.c)
else
VERSION := $(VER)
endif

all: build-sw build-fw build-amiga build-3rdparty
clean: clean-sw clean-fw clean-amiga clean-3rdparty

build-fw build-amiga build-sw build-3rdparty:
	@echo
	@echo "* Building in $(@:build-%=%)"
	@$(MAKE) -C $(@:build-%=%) all

clean-sw clean-fw clean-amiga clean-3rdparty:
	@echo
	@echo "* Cleaning in $(@:clean-%=%)"
	@$(MAKE) -C $(@:clean-%=%) clean-all

# ---------------------------------------------------------------

RELEASE_DIR := release/apci_$(VERSION)
RELEASE_LHA := release/apci_$(VERSION)_amiga.lha
RELEASE_ADF := release/apci_$(VERSION).adf
RELEASE_ZIP := release/apci_$(VERSION).zip

RELEASE_SUBDIR := $(patsubst release/%,%,$(RELEASE_DIR))
$(info RELEASE_SUBDIR = $(RELEASE_SUBDIR))

RELEASE_TARGETS :=
RELEASE_DIRS :=
define RELEASE_IT
ifneq (,$(wildcard $(1)))
RELEASE_TARGETS += $(RELEASE_DIR)/$(2)
RELEASE_DIRS += $(dir $(RELEASE_DIR)/$(2))
$(RELEASE_DIR)/$(2): $(1)
endif
endef
$(eval $(call RELEASE_IT,sw/objs.x86_64/hostbec,sw/hostbec.linux_x86_64))
$(eval $(call RELEASE_IT,sw/objs.mac/hostbec,sw/hostbec.mac))
$(eval $(call RELEASE_IT,sw/objs.armv7l/hostbec,sw/hostbec.pi32))
$(eval $(call RELEASE_IT,sw/objs.aarch64/hostbec,sw/hostbec.pi64))
$(eval $(call RELEASE_IT,sw/objs.win32/hostbec.exe,sw/hostbec_win32.exe))
$(eval $(call RELEASE_IT,sw/objs.win64/hostbec.exe,sw/hostbec_win64.exe))
$(eval $(call RELEASE_IT,sw/program_fpgas.sh,sw/program_fpgas.sh))
$(eval $(call RELEASE_IT,fw/dfu.release,fw/dfu.bat))
$(eval $(call RELEASE_IT,fw/objs/fw.bin,fw/fw.bin))
$(eval $(call RELEASE_IT,fw/objs/fw.elf,fw/fw.elf))
$(eval $(call RELEASE_IT,fw/Makefile.release,fw/Makefile))
$(eval $(call RELEASE_IT,fw/.gdbinit,fw/.gdbinit))
$(eval $(call RELEASE_IT,amiga/apciaconf,amiga/apciaconf))
$(eval $(call RELEASE_IT,amiga/apciflash,amiga/apciflash))
$(eval $(call RELEASE_IT,amiga/apcirom,amiga/apcirom))
$(eval $(call RELEASE_IT,amiga/apciscan,amiga/apciscan))
$(eval $(call RELEASE_IT,amiga/bec,amiga/bec))
$(eval $(call RELEASE_IT,amiga/becky,amiga/Becky))
$(eval $(call RELEASE_IT,amiga/Becky.info,amiga/Becky.info))
$(eval $(call RELEASE_IT,amiga/welcome,amiga/welcome))
$(eval $(call RELEASE_IT,amiga/program_flash,amiga/program_flash))
$(eval $(call RELEASE_IT,amiga/pci,amiga/pci))
$(eval $(call RELEASE_IT,amiga/Startup-Sequence,other/Startup-Sequence))
$(eval $(call RELEASE_IT,amiga/system-configuration,other/system-configuration))
$(eval $(call RELEASE_IT,3rdparty/lide/amigapci-lide.device,lide/lide.device))
#$(eval $(call RELEASE_IT,3rdparty/lide/apci-lide-F0_working_2025_11_13.rom,lide/lide.rom))
$(eval $(call RELEASE_IT,3rdparty/lide/apci-lide-F0_2025_11_14.rom,lide/lide.rom))
#$(eval $(call RELEASE_IT,3rdparty/lide/amigapci-lide.rom,lide/lide.rom))
$(eval $(call RELEASE_IT,3rdparty/med/med,3rdparty/med))
$(eval $(call RELEASE_IT,README.md,README.md))
$(eval $(call RELEASE_IT,LICENSE.md,LICENSE.md))
$(foreach DOC,$(wildcard doc/*.txt doc/*.md),$(eval $(call RELEASE_IT,$(DOC),$(DOC))))

RELEASE_DIRS := $(sort $(RELEASE_DIR) $(RELEASE_DIRS))

ifneq (,$(wildcard $(RELEASE_LHA))$(wildcard $(RELEASE_ZIP))$(wildcard $(RELEASE_ADF)))
release:
	@echo $(wildcard $(RELEASE_LHA) $(RELEASE_ZIP) $(RELEASE_ADF)) already exist
else
release: all
	@$(MAKE) do_release

do_release: populating $(RELEASE_TARGETS) $(RELEASE_LHA) $(RELEASE_ZIP) $(RELEASE_ADF)

AMIGA_RELS := $(RELEASE_SUBDIR)/amiga/* $(RELEASE_SUBDIR)/3rdparty/* \
              $(RELEASE_SUBDIR)/lide/* \
	      $(RELEASE_SUBDIR)/README.md $(RELEASE_SUBDIR)/LICENSE.md
$(RELEASE_LHA): all populating $(EXTERN_TARGETS) $(RELEASE_TARGETS)
	@echo "* Building $@"
	@rm -f $@
	(cd release; lha -aq2 $(notdir $@) $(AMIGA_RELS))

$(RELEASE_ZIP): all populating $(EXTERN_TARGETS) $(RELEASE_TARGETS)
	@echo "* Building $@"
	@rm -f $@
	(cd release; zip -rq $(notdir $@) $(RELEASE_SUBDIR))

$(RELEASE_ADF): all populating $(EXTERN_TARGETS) $(RELEASE_TARGETS)
	@echo "* Building $@"
	@rm -f $@
	xdftool $@ format "apci_$(VERSION)"
	xdftool $@ makedir C
	xdftool $@ makedir S
	xdftool $@ makedir Devs
	for f in $(RELEASE_DIR)/lide/*; do \
		xdftool $@ write "$$f" Devs/$$(basename "$$f"); \
	done
	for f in $(RELEASE_DIR)/amiga/*; do \
		[[ $$f == */pci ]] && continue; \
		xdftool $@ write "$$f" C/$$(basename "$$f"); \
	done
	xdftool $@ write $(RELEASE_DIR)/other/Startup-Sequence S/Startup-Sequence; \
	xdftool $@ write $(RELEASE_DIR)/other/system-configuration Devs/system-configuration; \
	xdftool $@ protect C/program_flash srwe
	xdftool $@ boot install

populating:
	@echo "* Populating $(RELEASE_DIR)"

$(RELEASE_TARGETS): | $(RELEASE_DIRS)
	cp -p $^ $@
endif
$(RELEASE_DIRS):
	mkdir -p $@

# Suppress make's built-in implicit rules for amiga binaries so they don't get
# rebuilt with the host compiler (cc) instead of the cross-compiler.
# These are built by the amiga sub-make via build-amiga.
amiga/apciaconf amiga/apciflash amiga/apcirom amiga/apciscan amiga/bec amiga/becky: ;

.PHONY: build-sw build-fw build-amiga clean-sw clean-fw clean-amiga all clean release populating
