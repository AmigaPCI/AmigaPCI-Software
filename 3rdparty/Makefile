LIDE_DIR    := lide
LIDE_DEVICE := $(LIDE_DIR)/amigapci-lide.device
LIDE_ROM    := $(LIDE_DIR)/amigapci-lide.rom

MED_DIR     := med
MED_PROG    := $(MED_DIR)/med

GENERATED_PROGS := $(LIDE_DEVICE) $(LIDE_ROM) $(MED_PROG)
ALL_DIRS        := $(LIDE_DIR) $(MED_DIR)

all: $(GENERATED_PROGS)
	@:

clean clean-all:
	@echo Cleaning
	@rm -rf $(GENERATED_PROGS)

WGET_FLAGS := --no-verbose --no-use-server-timestamps
$(LIDE_DEVICE): Makefile | $(LIDE_DIR)
	wget $(WGET_FLAGS) https://github.com/LIV2/lide.device/releases/latest/download/amigapci-lide.device -O $@
	@touch $@

$(LIDE_ROM): Makefile | $(LIDE_DIR)
	wget $(WGET_FLAGS) https://github.com/LIV2/lide.device/releases/download/Release-40.12/amigapci-lide.rom -O $@
	@touch $@

$(MED_PROG): Makefile | $(MED_DIR)
	wget $(WGET_FLAGS) https://github.com/cdhooper/med/releases/download/release_1.3/med.amiga -O $@
	@touch $@

$(ALL_DIRS):
	mkdir -p $@
