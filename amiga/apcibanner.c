#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <clib/dos_protos.h>
#include <clib/exec_protos.h>
#include <clib/alib_protos.h>
#include <fcntl.h>
#include <exec/execbase.h>
#include <exec/memory.h>
#include <exec/types.h>
#include <exec/ports.h>
#include <exec/nodes.h>
#include <exec/libraries.h>

#include <devices/timer.h>

#include <libraries/dos.h>
#include <libraries/dosextens.h>
#include <libraries/filehandler.h>

#include "cpu_control.h"

const char *version = "\0$VER: apcirom "VERSION" ("BUILD_DATE") \xA9 Chris Hooper";

int
main(int argc, char *argv[])
{
    (void) argc;
    (void) argv;
    printf("AmigaPCI\n");
}
