#!/bin/bash -p
#
# This script can be used to program AmigaPCI FPGA EEPROMs through
# BEC. The user is presented with a menu, where one or all FPGAs
# may be updated.
#

HOSTBEC=hostbec
DEFAULT_MB_BINS=../../AmigaPCI/Verilog/Release/BINs
DEFAULT_CPU_BINS=../../AmigaPCI-LBC040/Verilog/Release/BINs
DEFAULT_DEV_PREFIX=/dev/ttyUSB
PROGCONFIG=.progconfig
[ ! -d $MB_BINS ] && MB_BINS=~/projects/amigapci/AmigaPCI/Verilog/Release/BINs
[ ! -d $CPU_BINS ] && CPU_BINS=~/projects/amigapci/AmigaPCI-LBC040/Verilog/Release/BINs
if [ -f $PROGCONFIG ]; then
    [ -z $DEV ] && DEV=$(awk -F= '/^DEV/ { print $2 }' $PROGCONFIG)
    [ -z $MB_BINS ] && MB_BINS=$(awk -F= '/^MB_BINS/ { print $2 }' $PROGCONFIG)
    [ -z $CPU_BINS ] && CPU_BINS=$(awk -F= '/^CPU_BINS/ { print $2 }' $PROGCONFIG)
fi

if [ -z $DEV ]; then
    echo "The following devices are attached: "
    ls -1 "$DEFAULT_DEV_PREFIX"* | sed -e 's/^/    /'
    while read -ei $DEFAULT_DEV_PREFIX -p "Enter device name to use: " DEV ; do
        [ -e $DEV ] && break
        echo "$DEV does not exist"
    done
fi

if [ -z $MB_BINS ]; then
    while read -ei $DEFAULT_MB_BINS -p "Enter path to MB FPGA binaries: " MB_BINS ; do
        [ -d $MB_BINS"/" ] && break
        echo "$MB_BINS does not exist"
    done
fi

if [ -z $CPU_BINS ]; then
    while read -ei $DEFAULT_CPU_BINS -p "Enter path to CPU FPGA binaries: " CPU_BINS ; do
        [ -d $CPU_BINS"/" ] && break
        echo "$CPU_BINS does not exist"
    done
fi

(
    echo "DEV="$DEV
    echo "MB_BINS="$MB_BINS
    echo "CPU_BINS="$CPU_BINS
)> $PROGCONFIG

declare -A FILES

FILES[1,0]=$MB_BINS/U110_TOP_bitmap.bin
FILES[1,1]=0,0
FILES[2,0]=$MB_BINS/U109_TOP_bitmap.bin
FILES[2,1]=1,0
FILES[3,0]=$MB_BINS/U712_TOP_bitmap.bin
FILES[3,1]=2,0
FILES[4,0]=$MB_BINS/U409_TOP_bitmap.bin
FILES[4,1]=3,0
FILES[5,0]=$CPU_BINS/U111_TOP_bitmap.bin
FILES[5,1]=4,0
FILES[6,0]=$CPU_BINS/U400_TOP_bitmap.bin
FILES[6,1]=5,0

do_cmd() {
    echo $*
    $*
    return $?
}

show_files() {
    echo "    #  SPI  FILE                 Last Modified"
    for ((i = 1; i <= NUM_FILES;  i++)); do
        FILEPATH="${FILES[$i,0]}"
        SPI="${FILES[$i,1]}"
        SHORTNAME="${FILEPATH##*/}"
        DTIME=`stat -c '%.19y' ${FILEPATH}`
        echo "    $i  $SPI  $SHORTNAME  $DTIME"
    done
    echo "    A       Program All"
    echo "    R       Reset Amiga"
    echo "    P       Power Cycle Amiga"
    echo "    X       Exit"
}

bec_state() {
    $HOSTBEC -d $DEV -t amiga status |
    awk 'BEGIN { SAW=0 } /Power state/ { print "    " $0; SAW=1 } END { if (SAW == 0) print "    WARNING: No BEC response at this device" }'
}

program_single() {
    IDX="$1"
    FILEPATH="${FILES[$IDX,0]}"
    SPI="${FILES[$i,1]}"
    SHORTNAME="${FILEPATH##*/}"
    SHORTERNAME="${SHORTNAME%%_*}"
    echo "------------------------ $IDX $SHORTERNAME ------------------------"
    do_cmd $HOSTBEC -d $DEV -wvy -a 0,0 $FILEPATH
}

program_all() {
    for ((i = 1; i <= NUM_FILES;  i++)); do
        program_single $i || return 1
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
    echo "MB_BINS=$MB_BINS"
    echo "CPU_BINS=$CPU_BINS"
    echo "DEV=$DEV"
    bec_state
    echo
    show_files
}

NUM_FILES=${#FILES[@]}
NUM_FILES=$((NUM_FILES / 2))
NUM_FILES_MINUS_ONE=$((NUM_FILES - 1))

show_menu
while read -p "Enter file number to program: " WHICH; do
    case "$WHICH" in
        [1-$NUM_FILES])
            program_single $WHICH
            ;;
        [aA])
            program_all
            ;;
        [qQxX])
            exit 0
            ;;
        [rR])
            reset_amiga
            ;;
        [pP])
            power_cycle_amiga
            ;;
        "")
            ;;
        *)
            echo "Invalid option \"$WHICH\""
            ;;
    esac
    show_menu
done
