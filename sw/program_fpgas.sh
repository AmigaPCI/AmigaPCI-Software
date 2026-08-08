#!/usr/bin/env bash
#
# This script can be used to program AmigaPCI FPGA EEPROMs through
# BEC. The user is presented with a menu, where one or all FPGAs
# may be updated.
#

HOSTBEC=hostbec
DEFAULT_MB_BINS=../../AmigaPCI-Mainboard/Verilog/Release/BINs
DEFAULT_MBD_BINS=../../AmigaPCI-Mainboard/Verilog/Daily/Bins
DEFAULT_CPU_BINS=../../AmigaPCI-LBC040/Verilog/Release/BINs
DEFAULT_CPUD_BINS=../../AmigaPCI-LBC040/Verilog/Daily/Bins
DEFAULT_DEV_PREFIX=/dev/ttyUSB
PROGCONFIG=.progconfig
MODE=program
OS=$(uname -s)
ARCH=$(uname -m)
if [[ $OS == "Linux" && -f hostbec.linux.$ARCH ]]; then
    HOSTBEC=./hostbec.linux.$ARCH
elif [[ $OS == "Darwin" && -f ./hostbec.mac ]]; then
    HOSTBEC=./hostbec.mac
fi

if [[ -f "$PROGCONFIG" ]]; then
    [[ -z "$DEV" ]] &&
        DEV=$(awk -F= '/^DEV=/ { print $2 }' "$PROGCONFIG")
    [[ -z "$MB_BINS" ]] &&
        MB_BINS=$(awk -F= '/^MB_BINS=/ { print $2 }' "$PROGCONFIG")
    [[ -z "$MBD_BINS" ]] &&
        MBD_BINS=$(awk -F= '/^MBD_BINS=/ { print $2 }' "$PROGCONFIG")
    [[ -z "$CPU_BINS" ]] &&
        CPU_BINS=$(awk -F= '/^CPU_BINS=/ { print $2 }' "$PROGCONFIG")
    [[ -z "$CPUD_BINS" ]] &&
        CPUD_BINS=$(awk -F= '/^CPUD_BINS=/ { print $2 }' "$PROGCONFIG")
fi
[[ ! -d "$MB_BINS" ]] &&
    MB_BINS=~/projects/amiga_pci/AmigaPCI/Verilog/Release/BINs
[[ ! -d "$MBD_BINS" ]] &&
    MBD_BINS=~/projects/amiga_pci/AmigaPCI/Verilog/Daily/BINs
[[ ! -d "$CPU_BINS" ]] &&
    CPU_BINS=~/projects/amiga_pci/AmigaPCI-LBC040/Verilog/Release/BINs
[[ ! -d "$CPUD_BINS" ]] &&
    CPUD_BINS=~/projects/amiga_pci/AmigaPCI-LBC040/Verilog/Daily/BINs

CHANGED=0
if [[ $PROMPT_ALL != "" || ! -e $DEV ]]; then
    PROMPT_DEFAULT="$DEFAULT_DEV_PREFIX"
    [[ $PROMPT_ALL != "" && ! -z $DEV ]] && PROMPT_DEFAULT="$DEV"
    echo "The following devices are attached: "
    for DEVICE in "$DEFAULT_DEV_PREFIX"*; do
        [[ -e "$DEVICE" ]] && echo "    $DEVICE"
    done
    while read -r -e -i "$PROMPT_DEFAULT" -p "Device name to use: " DEV ; do
        [[ -e "$DEV" ]] && break
        echo "$DEV does not exist"
    done
    CHANGED=1
fi

if [[ $PROMPT_ALL != "" || ! -d $MB_BINS ]]; then
    PROMPT_DEFAULT="$DEFAULT_MB_BINS"
    [[ $PROMPT_ALL != "" && -d $MB_BINS ]] && PROMPT_DEFAULT="$MB_BINS"
    while read -r -e -i "$PROMPT_DEFAULT" -p "Path to MB FPGA release binaries: " MB_BINS ; do
        [[ -d "$MB_BINS" ]] && break
        echo "$MB_BINS does not exist"
    done
    CHANGED=1
fi

if [[ $PROMPT_ALL != "" || ! -d $MBD_BINS ]]; then
    PROMPT_DEFAULT="$DEFAULT_MBD_BINS"
    [[ $PROMPT_ALL != "" && -d $MBD_BINS ]] && PROMPT_DEFAULT="$MBD_BINS"
    while read -r -e -i "$PROMPT_DEFAULT" -p "Path to MB FPGA daily binaries: " MBD_BINS ; do
        [[ -d "$MBD_BINS" ]] && break
        echo "$MBD_BINS does not exist"
    done
    CHANGED=1
fi

if [[ $PROMPT_ALL != "" || ! -d $CPU_BINS ]]; then
    PROMPT_DEFAULT="$DEFAULT_CPU_BINS"
    [[ $PROMPT_ALL != "" && -d $CPU_BINS ]] && PROMPT_DEFAULT="$CPU_BINS"
    while read -r -e -i "$PROMPT_DEFAULT" -p "Path to CPU FPGA release binaries: " CPU_BINS ; do
        [[ -d "$CPU_BINS" ]] && break
        echo "$CPU_BINS does not exist"
    done
    CHANGED=1
fi

if [[ $PROMPT_ALL != "" || ! -d $CPUD_BINS ]]; then
    PROMPT_DEFAULT="$DEFAULT_CPUD_BINS"
    [[ $PROMPT_ALL != "" && -d $CPUD_BINS ]] && PROMPT_DEFAULT="$CPUD_BINS"
    while read -r -e -i "$PROMPT_DEFAULT" -p "Path to CPU FPGA daily binaries: " CPUD_BINS ; do
        [[ -d "$CPUD_BINS" ]] && break
        echo "$CPUD_BINS does not exist"
    done
    CHANGED=1
fi

if [[ "$CHANGED" == "1" ]]; then
    (
        echo "DEV=$DEV"
        echo "MB_BINS=$MB_BINS"
        echo "MBD_BINS=$MBD_BINS"
        echo "CPU_BINS=$CPU_BINS"
        echo "CPUD_BINS=$CPUD_BINS"
    ) > "$PROGCONFIG"
    echo "Updated $PROGCONFIG"
fi

FILE_SPI=("" "0,0" "1,0" "2,0" "3,0" "4,0" "5,0")
FILE_NAME=("" U110_TOP_bitmap.bin U109_TOP_bitmap.bin U712_TOP_bitmap.bin
           U409_TOP_bitmap.bin U111_TOP_bitmap.bin U400_TOP_bitmap.bin)
FILE_RELEASE=("" "$MB_BINS" "$MB_BINS" "$MB_BINS" "$MB_BINS"
              "$CPU_BINS" "$CPU_BINS")
FILE_DAILY=("" "$MBD_BINS" "$MBD_BINS" "$MBD_BINS" "$MBD_BINS"
            "$CPUD_BINS" "$CPUD_BINS")

NUM_FILES=6
do_cmd() {
    printf ' %q' "$@"
    printf '\n'
    "$@"
}

do_stat() {
    if [[ $OS == "Darwin" ]]; then
        stat -f "%Sc" -t "%Y-%m-%d %H:%M:%S" "$1"
    else
        stat -c '%.19y' "$1"
    fi
}

show_files() {
    echo "    #   SPI  FILE                 Type     Last Modified"
    for ((i = 1; i <= NUM_FILES;  i++)); do
        FILEPATH="${FILE_RELEASE[$i]}/${FILE_NAME[$i]}"
        SPI="${FILE_SPI[$i]}"
        SHORTNAME="${FILEPATH##*/}"
        DTIME=$(do_stat "${FILEPATH}")
        TYPE="Release"
        echo "    $i   $SPI  $SHORTNAME  $TYPE  $DTIME"

        FILEPATH="${FILE_DAILY[$i]}/${FILE_NAME[$i]}"
        if [[ -f $FILEPATH ]]; then
            SHORTNAME="${FILEPATH##*/}"
            DTIME=$(do_stat "${FILEPATH}")
            TYPE="Daily  "
            echo "    ${i}D  $SPI  $SHORTNAME  $TYPE  $DTIME"
        fi
    done
    if [[ "$MODE" == "program" ]]; then
        echo "    A        Program All"
    else
        echo "    A        Verify All"
    fi
    echo "    B        Program BEC (STM32) using serial DFU"
    echo "    C        Power Cycle Amiga"
    echo "    I        Show FPGA programming instructions"
    echo "    S        Change path or device settings"
    echo "    R        Reset Amiga"
    if [[ "$MODE" == "program" ]]; then
        echo "    V        Verify Mode"
    else
        echo "    P        Program Mode"
    fi
    echo "    X        Exit"
}

bec_state() {
    "$HOSTBEC" -d "$DEV" -t "" > /dev/null
    "$HOSTBEC" -d "$DEV" -t amiga status |
    awk 'BEGIN { SAW=0 }
        /Power state/ { print "    " $0; sub(/\r/, "",$3); STATE=$3; SAW=1 }
        END {
            if (SAW == 0) {
                print "    WARNING: No BEC response at this device"
            } else {
                if (STATE != "On") print("    WARNING: You must power on the board before programming");
            }
        }'
}

program_single() {
    IDX="$1"
    DAILY="$2"
    if [[ "$MODE" == "program" ]]; then
        ARGS="-wvy"
    else
        ARGS="-vy"
    fi
    FILEPATH="${FILE_RELEASE[$IDX]}/${FILE_NAME[$IDX]}"
    if [[ "$DAILY" == "daily" ]]; then
        FILEPATH_D="${FILE_DAILY[$IDX]}/${FILE_NAME[$IDX]}"
        if [[ -f $FILEPATH_D ]]; then
            FILEPATH="$FILEPATH_D"
        fi
    fi
    SPI="${FILE_SPI[$IDX]}"
    SHORTNAME="${FILEPATH##*/}"
    SHORTERNAME="${SHORTNAME%%_*}"
    echo "------------------------ $IDX $SHORTERNAME ------------------------"
    do_cmd "$HOSTBEC" -d "$DEV" "$ARGS" -a "$SPI" "$FILEPATH"
}

program_all() {
    DAILY="$1"
    for ((i = 1; i <= NUM_FILES;  i++)); do
        program_single "$i" "$DAILY" || return 1
    done
}

reset_amiga() {
    "$HOSTBEC" -d "$DEV" -t reset amiga
}

power_cycle_amiga() {
    "$HOSTBEC" -d "$DEV" -t power cycle
}

program_stm32() {
    STATE=$(bec_state)
    if [[ "$STATE" == *"Power state"* ]]; then
        USE_PG_JUMPER=
    else
        USE_PG_JUMPER=Yes
    fi

    LOADER=
    LOADER1=$(which stm32loader 2>/dev/null)
    LOADER2=$(which STM32_Programmer_CLI 2>/dev/null)
    [[ -z $LOADER2 ]] && LOADER2=$(which /usr/local/STMCubeProgrammer/bin/STM32_Programmer_CLI 2>/dev/null)
    [[ ! -z $LOADER2 ]] && LOADER=2
    [[ ! -z $LOADER1 ]] && LOADER=1
    if [[ -z $LOADER ]]; then
        echo "You must have one of stm32loader or STM32_Programmer_CLI installed."
        echo "Install STM32_Programmer_CLI by installing STM32CubeProg from:"
        echo "    https://www.st.com/en/development-tools/stm32cubeprog.html"
        echo "or install stm32loader using:"
        echo "    pip install stm32loader"
        exit 1
    fi
    FW=
    [[ -f ../fw/fw.bin ]] && FW=../fw/fw.bin
    [[ -f ../fw/objs/fw.bin ]] && FW=../fw/objs/fw.bin
    if [[ -z $FW ]]; then
        echo "No firmware fw.bin found in ../fw"
        exit 1
    fi

    if [[ ! -z $USE_PG_JUMPER ]]; then
        show_stm32_programming_instructions
    else
        echo "BEC STM32 will send a \"reset dfu\" command to enter DFU mode"
    fi
    echo
    echo "Press enter to proceed with programming"
    read
    if [[ -z $USE_PG_JUMPER ]]; then
        # Attempt to put BEC in DFU mode by command
        OUTPUT=$("$HOSTBEC" -d "$DEV" -t reset dfu)
        if [[ "$OUTPUT" != *"Resetting to DFU"* ]]; then
            echo "Failed to put STM32 into DFU mode."
            exit 1
        fi
        sleep 1
    fi
    if [[ $LOADER == 1 ]]; then
        do_cmd $LOADER1 --port $DEV -b 115200 -a 0x08000000 -w -v $FW -g 0x08000000 -f F4
    elif [[ $LOADER == 2 ]]; then
        do_cmd $LOADER2 -c port=$DEV br=115200 -v -w $FW 0x08000000
        do_cmd $LOADER2 -c port=$DEV br=115200 -g 0x08000000
    fi
    echo "Done. Install the J209 RN jumper and do a 20 second power cycle."
    echo "It is not necessary to power cycle if the STM32 status LED is now on."
}

show_fpga_programming_instructions() {
    echo "1. Connect an FTDI TTL-to-USB serial adapter to the STM32 DEBUG"
    echo "   header."
    echo "2. Connect a power supply to your AmigaPCI and program the STM32"
    echo "   firmware using either an ST-Link or serial DFU."
    echo "3. If you are programming a completely blank FPGA, you must use a"
    echo "   F-F dupont jumper to connect CRESET and GND on that FPGA's"
    echo "   programming header. You may jumper one or all FPGAs at the "
    echo "   same time."
    echo "4. Power on the AmigaPCI (you can use 'C' to cause this."
    echo "5. Select from the menu each FPGA which has not been programmed."
    echo "6. Power cycle the AmigaPCI."
    echo
    read -p "Press Enter."
}

show_stm32_programming_instructions() {
    echo "1. Power off AmigaPCI and wait 20 seconds."
    echo "2. Remove the jumper from the J209 PG/RN header."
    echo "3. Connect an FTDI TTL-to-USB serial adapter to the STM32 DEBUG"
    echo "   header."
    echo "4. Power on AmigaPCI."
}

show_menu()
{
    echo "================================================================"
    echo "MB_BINS=$MB_BINS"
    echo "MBD_BINS=$MBD_BINS"
    echo "CPU_BINS=$CPU_BINS"
    echo "CPUD_BINS=$CPUD_BINS"
    echo "DEV=$DEV"
    bec_state
    echo "    Mode: $MODE"
    echo
    show_files
}

show_menu
while read -r -p "Enter file number to $MODE: " WHICH; do
    case "$WHICH" in
        [1-$NUM_FILES])
            program_single "$WHICH"
            ;;
        [1-$NUM_FILES][dD])
            program_single "${WHICH%[dD]}" daily
            ;;
        [aA])
            program_all release
            ;;
        [bB])
            program_stm32
            ;;
        [cC])
            power_cycle_amiga
            ;;
        [iI])
            show_fpga_programming_instructions
            ;;
        [qQxX])
            exit 0
            ;;
        [sS])
            PROMPT_ALL=1 exec "$0" "$@"
            ;;
        [rR])
            reset_amiga
            ;;
        [pP])
            MODE=program
            ;;
        [vV])
            MODE=verify
            ;;
        "")
            ;;
        *)
            echo "Invalid option \"$WHICH\""
            ;;
    esac
    show_menu
done
