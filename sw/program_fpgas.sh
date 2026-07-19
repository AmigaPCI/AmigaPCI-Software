#!/bin/bash -p
#
# This script can be used to program AmigaPCI FPGA EEPROMs through
# BEC. The user is presented with a menu, where one or all FPGAs
# may be updated.
#

HOSTBEC=hostbec
MB=../../AmigaPCI/Verilog/Release/BINs
CPU=../../AmigaPCI-LBC040/Verilog/Release/BINs
[ ! -d $MB ] && MB=~/projects/amigapci/AmigaPCI/Verilog/Release/BINs
[ ! -d $CPU ] && CPU=~/projects/amigapci/AmigaPCI-LBC040/Verilog/Release/BINs
if [ -z $DEV ]; then
    DEV=`ls -1 /dev/ttyUSB* | tail -1`
    if [[ ! -e $DEV ]]; then
        echo 'No /dev/ttyUSB* device found'
        exit 1
    fi
fi

declare -A FILES

FILES[1,0]=$MB/U110_TOP_bitmap.bin
FILES[1,1]=0,0
FILES[2,0]=$MB/U109_TOP_bitmap.bin
FILES[2,1]=1,0
FILES[3,0]=$MB/U712_TOP_bitmap.bin
FILES[3,1]=2,0
FILES[4,0]=$MB/U409_TOP_bitmap.bin
FILES[4,1]=3,0
FILES[5,0]=$CPU/U111_TOP_bitmap.bin
FILES[5,1]=4,0
FILES[6,0]=$CPU/U400_TOP_bitmap.bin
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
    echo "    X       Exit"
}

program_single() {
    IDX="$1"
    FILEPATH="${FILES[$IDX,0]}"
    SPI="${FILES[$i,1]}"
    SHORTNAME="${FILEPATH##*/}"
    SHORTERNAME="${SHORTNAME%%_*}"
    echo "------------------------ $IDX $SHORTERNAME ------------------------"
    do_cmd hostbec -d $DEV -wvy -a 0,0 $FILEPATH
}

program_all() {
    for ((i = 1; i <= NUM_FILES;  i++)); do
        program_single $i || return 1
    done
}


NUM_FILES=${#FILES[@]}
NUM_FILES=$((NUM_FILES / 2))
NUM_FILES_MINUS_ONE=$((NUM_FILES - 1))

echo "Using DEV=$DEV for programming"
echo
show_files
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
        *)
            echo "Invalid option \"$WHICH\""
            ;;
    esac
done

# Need to do Do U110 again

#do_cmd U110 hostbec -d $DEV -wvy -a 0,0 $MB/U110_TOP_bitmap.bin &&
#do_cmd U109 hostbec -d $DEV -wvy -a 1,0 $MB/U109_TOP_bitmap.bin &&
#do_cmd U712 hostbec -d $DEV -wvy -a 2,0 $MB/U712_TOP_bitmap.bin &&
#do_cmd U409 hostbec -d $DEV -wvy -a 3,0 $MB/U409_TOP_bitmap.bin &&
#do_cmd U111 hostbec -d $DEV -wvy -a 4,0 $CPU/U111_TOP_bitmap.bin &&
#do_cmd U400 hostbec -d $DEV -wvy -a 5,0 $CPU/U400_TOP_bitmap.bin &&
echo Done
