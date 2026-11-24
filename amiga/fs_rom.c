/*
 * This is free and unencumbered software released into the public domain.
 * See the LICENSE file for additional details.
 *
 * Designed by Chris Hooper in 2025.
 *
 * ---------------------------------------------------------------------
 *
 * becfsrom: ROM resident module which starts becfs (the AmigaPCI BEC USB
 * storage filesystem) after DOS has started.
 *
 * This is derived from KickSmash fs_rom.c (smashfsrom), less two things
 * which becfs does not need:
 *   - The message code does not need to be copied to RAM: BEC messages
 *     go through the RTC or keyboard, not the ROM bus.
 *   - There is no boot module. The BEC USB devices may not be ready when
 *     the Amiga cold-starts, so becfs volumes are not bootable.
 */

#include <exec/types.h>
#include <exec/libraries.h>
#include <exec/resident.h>
#include <string.h>
#include <dos/dos.h>
#include <dos/dostags.h>      // NP_Entry, etc
#include <dos/dosextens.h>    // struct Process
#include <utility/utility.h>  // UTILITYNAME

#include <exec/execbase.h>
#include <libraries/filehandler.h>
#include <proto/exec.h>
#include <clib/alib_protos.h>

#include <inline/exec.h>
#include <inline/dos.h>
#include <inline/macros.h>

#include "printf.h"
#include "fs_vol.h"

#define ROM_VERSION 1

#define ARRAY_SIZE(x) ((sizeof (x) / sizeof ((x)[0])))

#define RawPutChar(___c) \
        LP1NR(0x204, RawPutChar , UBYTE, ___c, d0, , SysBase)

extern struct DosLibrary *DOSBase;

void rom_end(void);  // End of the ROM module (rom_end.c, linked last)
void rom_main(struct ExecBase *SysBase asm("a6"));
extern const char BecfsRomID[];

struct Task *my_CreateTask(CONST_STRPTR name,LONG pri, CONST APTR init_pc,
                           ULONG stack_size, void *userdata);

const struct Resident resident = {
    RTC_MATCHWORD,         // rt_MatchWord - word to match on (ILLEGAL)
    (void *) &resident,    // rt_MatchTag - pointer to the above
    rom_end,               // rt_EndSkip - address to continue scan
    RTF_AFTERDOS,          // rt_Flags - various tag flags
    ROM_VERSION,           // rt_Version - release version number
    NT_UNKNOWN,            // rt_Type - type of module
    5,                     // rt_Pri - initialization priority
    (char *) "becfsrom",   // rt_Name - pointer to node name
    (char *) BecfsRomID,   // rt_IdString - pointer to identification
    rom_main               // rt_Init - pointer to init code
};

const char BecfsRomID[] = "becfsrom "VERSION" ("BUILD_DATE")\r\n";

typedef unsigned int uint;

void
dputs(const char *str)
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds="
    struct ExecBase *SysBase = *(struct ExecBase **)4UL;
#pragma GCC diagnostic pop

    while (*str != '\0')
        RawPutChar(*str++);
}

/*
 * Boot node support (see fs_vol.c) is not used by becfs: there are no
 * becboot device nodes.
 */
becboot_dev_t *
becboot_dev(struct DeviceNode *dn)
{
    (void) dn;
    return (NULL);
}

void becboot_add(const char *name, int bootpri);
void
becboot_add(const char *name, int bootpri)
{
    (void) name;
    (void) bootpri;
}

static int
call_main(void)
{
    extern int main(int argc, char *argv[]);
    extern uint8_t flag_output;
    char *args[] = { NULL, "-dd" };

    /* Globals are now available */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds="
    SysBase = *(struct ExecBase **)4UL;
#pragma GCC diagnostic pop

    /* Make becfs debug output (becfsrom_d only) go to the serial port */
    flag_output = 2;

    /* Execute command line main() */
    return (main(ARRAY_SIZE(args), args));
}

int __regargs
rom_process_entry(void)
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds="
    struct ExecBase *SysBase = *(struct ExecBase **)4UL;
#pragma GCC diagnostic pop
    struct Process *proc;
    uint8_t *globals;
    int rc;

    proc = (struct Process *) FindTask(NULL);
    if (proc == NULL)
        return (0);

    globals = (uint8_t *) proc->pr_ExitData;
    if (globals == NULL)
        return (0);

    /* Got globals pointer */
    __asm("move.l %0,a4" :: "r" (globals));

    /* Globals are now available */
    extern struct Library * __UtilityBase;
    __UtilityBase = OpenLibrary(UTILITYNAME, 0);

    rc = call_main();
    Forbid();
    return (rc);
}

/*
 * globals_alloc
 * -------------
 * Allocates and initializes a copy of the data and BSS segments for
 * becfs. Returns the value for a4 (-fbaserel), or NULL.
 */
static uint8_t *
globals_alloc(struct ExecBase *SysBase)
{
    uint8_t *globals;
    void    *data_start;
    uint     data_size;
    uint     bss_size;
    __asm("lea __sdata,%0"      : "=a" (data_start) ::);
    __asm("lea ___data_size,%0" : "=a" (data_size) ::);
    __asm("lea ___bss_size,%0"  : "=a" (bss_size) ::);

    if ((globals = AllocVec(data_size + bss_size, MEMF_PUBLIC)) == NULL) {
        dputs("becfsrom: AllocVec fail\n");
        return (NULL);
    }

    /* Must compile with -fbaserel so a4 will be used for globals */
    memcpy(globals, data_start, data_size);
    memset(globals + data_size, 0, bss_size);

    return (globals + 0x7ffe);  // offset gcc applies to a4-relative globals
}

/*
 * rom_task_entry
 * --------------
 * Task which starts the becfs process. CreateNewProcTags() crashes the
 * caller if called directly from rom_main() (see KickSmash fs_rom.c).
 */
int
rom_task_entry(void)
{
    uint8_t *globals;
    struct Task *task;
    struct Process *child;
    struct DosLibrary *DOSBase;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds="
    struct ExecBase *SysBase = *(struct ExecBase **)4UL;
#pragma GCC diagnostic pop
    __asm("movem.l d0-d7/a0-a6,-(sp)");

    task = FindTask(NULL);
    globals = task->tc_UserData;
    __asm("move.l %0,a4" :: "r" (globals));   // Point a4 at globals

    DOSBase = (struct DosLibrary *) OpenLibrary(DOSNAME, 34);
    if (DOSBase == NULL) {
        dputs("NULL DOSBase\n");
        goto task_entry_fail;
    }

    child = CreateNewProcTags(NP_Entry, (ULONG) rom_process_entry,
                              NP_Name, (ULONG) "becfs",
                              NP_StackSize, 8192,
                              NP_Priority, -1,
                              NP_CloseInput, FALSE,
                              NP_CloseOutput, FALSE,
                              NP_CopyVars, FALSE,
                              NP_ExitData, (ULONG) globals,
                              TAG_END);

    CloseLibrary((struct Library *)DOSBase);
    if (child == NULL)
        dputs("Failed to start becfs process\n");

task_entry_fail:
    __asm("movem.l (sp)+,d0-d7/a0-a6");
    return (0);
}

/*
 * rom_main
 * --------
 * Entry for ROM module, called by exec startup code.
 * The a6 register is SysBase
 */
void
rom_main(struct ExecBase *SysBase asm("a6"))
{
    uint8_t *globals;
    struct Task *task;

    if ((globals = globals_alloc(SysBase)) == NULL)
        return;
    __asm("move.l %0,a4" :: "r" (globals));

    task = my_CreateTask("becfs", 10, rom_task_entry, 4096, globals);
    if (task == NULL)
        dputs("Failed to start becfs task\n");
}
