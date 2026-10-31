set confirm off

define flash
target extended-remote localhost:4242
load
quit
end

define connect
target extended-remote localhost:4242
end

define loadu
monitor reset halt
load objs/updater.elf
set $pc = *(unsigned int*)0x20000004
set $sp = *(unsigned int*)0x20000000
end

document connect
Connect to the GDB server running on the local host.
You should first connect USB to the target and then enter:
    make stlink
end

printf "\nflash - connect, flash firmware, and exit.\n"
printf "connect - attach to local GDB server (port 4242).\n"
printf "load    - reload firmware.\n"
printf ""
printf "Note: st-util must already be started (make stlink).\n"
