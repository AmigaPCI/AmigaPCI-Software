#!/usr/bin/env bash
#
# This script can be used to program AmigaPCI FPGA EEPROMs through
# BEC. The user is presented with a menu, where one or all FPGAs
# may be updated.
#

HOSTBEC=hostbec
DEFAULT_MB_BINS=../../AmigaPCI/Verilog/Release/BINs
DEFAULT_MBD_BINS=../../AmigaPCI/Verilog/Daily/Bins
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

[ ! -d $MB_BINS ] && MB_BINS=~/projects/amiga_pci/AmigaPCI/Verilog/Release/BINs
[ ! -d $MBD_BINS ] && MB_BINS=~/projects/amiga_pci/AmigaPCI/Verilog/Daily/BINs
[ ! -d $CPU_BINS ] && CPU_BINS=~/projects/amiga_pci/AmigaPCI-LBC040/Verilog/Release/BINs
[ ! -d $CPUD_BINS ] && CPU_BINS=~/projects/amiga_pci/AmigaPCI-LBC040/Verilog/Daily/BINs
if [ -f $PROGCONFIG ]; then
    [ -z $DEV ] && DEV=$(awk -F= '/^DEV/ { print $2 }' $PROGCONFIG)
    [ -z $MB_BINS ] && MB_BINS=$(awk -F= '/^MB_BINS/ { print $2 }' $PROGCONFIG)
    [ -z $MBD_BINS ] && MBD_BINS=$(awk -F= '/^MBD_BINS/ { print $2 }' $PROGCONFIG)
    [ -z $CPU_BINS ] && CPU_BINS=$(awk -F= '/^CPU_BINS/ { print $2 }' $PROGCONFIG)
    [ -z $CPUD_BINS ] && CPUD_BINS=$(awk -F= '/^CPUD_BINS/ { print $2 }' $PROGCONFIG)
fi

CHANGED=0
if [[ $PROMPT_ALL != "" || -z $DEV ]]; then
    PROMPT_DEFAULT="$DEFAULT_DEV_PREFIX"
    [[ $PROMPT_ALL != "" && ! -z $DEV ]] && PROMPT_DEFAULT="$DEV"
    echo "The following devices are attached: "
    ls -1 "$DEFAULT_DEV_PREFIX"* | sed -e 's/^/    /'
    while read -ei "$PROMPT_DEFAULT" -p "Device name to use: " DEV ; do
        [ -e $DEV ] && break
        echo "$DEV does not exist"
    done
    CHANGED=1
fi

if [[ $PROMPT_ALL != "" || -z $MB_BINS ]]; then
    PROMPT_DEFAULT="$DEFAULT_MB_BINS "
    [[ $PROMPT_ALL != "" && -d $MB_BINS ]] && PROMPT_DEFAULT="$MB_BINS"
    while read -ei "$PROMPT_DEFAULT" -p "Path to MB FPGA release binaries: " MB_BINS ; do
        [ -d $MB_BINS"/" ] && break
        echo "$MB_BINS does not exist"
    done
    CHANGED=1
fi

if [[ $PROMPT_ALL != "" || -z $MBD_BINS ]]; then
    PROMPT_DEFAULT="$DEFAULT_MBD_BINS"
    [[ $PROMPT_ALL != "" && -d $MBD_BINS ]] && PROMPT_DEFAULT="$MBD_BINS"
    while read -ei "$PROMPT_DEFAULT" -p "Path to MB FPGA daily binaries: " MBD_BINS ; do
        [ -d $MBD_BINS"/" ] && break
        echo "$MBD_BINS does not exist"
    done
    CHANGED=1
fi

if [[ $PROMPT_ALL != "" || -z $CPU_BINS ]]; then
    PROMPT_DEFAULT="$DEFAULT_CPU_BINS"
    [[ $PROMPT_ALL != "" && -d $CPU_BINS ]] && PROMPT_DEFAULT="$CPU_BINS"
    while read -ei "$PROMPT_DEFAULT" -p "Path to CPU FPGA release binaries: " CPU_BINS ; do
        [ -d $CPU_BINS"/" ] && break
        echo "$CPU_BINS does not exist"
    done
    CHANGED=1
fi

if [[ $PROMPT_ALL != "" || -z $CPUD_BINS ]]; then
    PROMPT_DEFAULT="$DEFAULT_CPUD_BINS"
    [[ $PROMPT_ALL != "" && -d $CPUD_BINS ]] && PROMPT_DEFAULT="$CPUD_BINS"
    while read -ei "$PROMPT_DEFAULT" -p "Path to CPU FPGA daily binaries: " CPUD_BINS ; do
        [ -d $CPUD_BINS"/" ] && break
        echo "$CPUD_BINS does not exist"
    done
    CHANGED=1
fi

if [[ "$CHANGED" == "1" ]]; then
    (
        echo "DEV="$DEV
        echo "MB_BINS="$MB_BINS
        echo "MBD_BINS="$MBD_BINS
        echo "CPU_BINS="$CPU_BINS
        echo "CPUD_BINS="$CPUD_BINS
    )> $PROGCONFIG
    echo Updated $PROGCONFIG
fi

declare -A FILES

FILES[1,0]=0,0
FILES[1,1]=U110_TOP_bitmap.bin
FILES[1,2]=$MB_BINS
FILES[1,3]=$MBD_BINS
FILES[2,0]=1,0
FILES[2,1]=U109_TOP_bitmap.bin
FILES[2,2]=$MB_BINS
FILES[2,3]=$MBD_BINS
FILES[3,0]=2,0
FILES[3,1]=U712_TOP_bitmap.bin
FILES[3,2]=$MB_BINS
FILES[3,3]=$MBD_BINS
FILES[4,0]=3,0
FILES[4,1]=U409_TOP_bitmap.bin
FILES[4,2]=$MB_BINS
FILES[4,3]=$MBD_BINS
FILES[5,0]=4,0
FILES[5,1]=U111_TOP_bitmap.bin
FILES[5,2]=$CPU_BINS
FILES[5,3]=$CPUD_BINS
FILES[6,0]=5,0
FILES[6,1]=U400_TOP_bitmap.bin
FILES[6,2]=$CPU_BINS
FILES[6,3]=$CPUD_BINS

NUM_FILES=6
NUM_FILES_MINUS_ONE=$((NUM_FILES - 1))

do_cmd() {
    echo $*
    $*
    return $?
}

do_stat() {
    if [[ $OS == "Darwin" ]]; then
        stat -f "%Sc" -t "%Y-%m-%d %H:%M:%S" $1
    else
        stat -c '%.19y' $1
    fi
}

show_files() {
    echo "    #   SPI  FILE                 Type     Last Modified"
    for ((i = 1; i <= NUM_FILES;  i++)); do
        FILEPATH="${FILES[$i,2]}/${FILES[$i,1]}"
        SPI="${FILES[$i,0]}"
        SHORTNAME="${FILEPATH##*/}"
        DTIME=$(do_stat "${FILEPATH}")
        TYPE="Release"
        echo "    $i   $SPI  $SHORTNAME  $TYPE  $DTIME"

        FILEPATH="${FILES[$i,3]}/${FILES[$i,1]}"
        if [[ -f $FILEPATH ]]; then
            SHORTNAME="${FILEPATH##*/}"
            DTIME=$(do_stat "${FILEPATH}")
            TYPE="Daily  "
            echo "    "$i"D  $SPI  $SHORTNAME  $TYPE  $DTIME"
        fi
    done
    if [[ "$MODE" == "program" ]]; then
        echo "    A        Program All"
    else
        echo "    A        Verify All"
    fi
    echo "    C        Power Cycle Amiga"
    echo "    R        Reset Amiga"
    if [[ "$MODE" == "program" ]]; then
        echo "    V        Verify Mode"
    else
        echo "    P        Program Mode"
    fi
    echo "    S        Change path or device settings"
    echo "    X        Exit"
}

bec_state() {
    $HOSTBEC -d $DEV -t amiga status |
    awk 'BEGIN { SAW=0 } /Power state/ { print "    " $0; SAW=1 } END { if (SAW == 0) print "    WARNING: No BEC response at this device" }'
}

program_single() {
    IDX="$1"
    DAILY="$2"
    if [[ "$MODE" == "program" ]]; then
        ARGS="-wvy"
    else
        ARGS="-vy"
    fi
    FILEPATH="${FILES[$IDX,2]}/${FILES[$IDX,1]}"
    if [[ "$DAILY" == "daily" ]]; then
        FILEPATH_D="${FILES[$IDX,3]}/${FILES[$IDX,1]}"
        if [[ -f $FILEPATH_D ]]; then
            FILEPATH="$FILEPATH_D"
        fi
    fi
    SPI="${FILES[$IDX,0]}"
    SHORTNAME="${FILEPATH##*/}"
    SHORTERNAME="${SHORTNAME%%_*}"
    echo "------------------------ $IDX $SHORTERNAME ------------------------"
    do_cmd $HOSTBEC -d $DEV $ARGS -a $SPI $FILEPATH
}

program_all() {
    DAILY="$1"
    for ((i = 1; i <= NUM_FILES;  i++)); do
        program_single $i $FAILY || return 1
    done
}

reset_amiga() {
    $HOSTBEC -d $DEV -t reset amiga
}

power_cycle_amiga() {
    $HOSTBEC -d $DEV -t power cycle
}

show_menu()
{
    echo "================================================================"
    echo "MB_BINS="$MB_BINS
    echo "MBD_BINS="$MBD_BINS
    echo "CPU_BINS="$CPU_BINS
    echo "CPUD_BINS="$CPUD_BINS
    echo "DEV=$DEV"
    bec_state
    echo "    Mode: $MODE"
    echo
    show_files
}

show_menu
while read -p "Enter file number to $MODE: " WHICH; do
    case "$WHICH" in
        [1-$NUM_FILES])
            program_single $WHICH
            ;;
        [1-$NUM_FILES][dD])
            program_single ${WHICH%[dD]} daily
            ;;
        [aA])
            program_all
            ;;
        [cC])
            power_cycle_amiga
            ;;
        [qQxX])
            exit 0
            ;;
        [sS])
            PROMPT_ALL=1 exec $0 $*
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
