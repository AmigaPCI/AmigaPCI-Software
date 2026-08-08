/*
 * AmigaPCI utilities disk welcome banner.
 *
 * This replaces a long Startup-Sequence full of Echo commands with one
 * small process that writes the same text directly to the console.
 *
 * Originally written by Stefan Reinauer for the ReA4091 project.
 */

#include <dos/dos.h>
#include <exec/types.h>
#include <proto/dos.h>

#ifndef BUILD_TIME
#define BUILD_TIME ""
#endif

#ifndef BUILD_DATE
#define BUILD_DATE "2026"
#endif

#define ARRAY_SIZE(x) (sizeof (x) / sizeof ((x)[0]))

typedef struct {
    const char *device_path;
    const char * const *banner;
} banner_def_t;

static BPTR out;

static const char * const banner[] = {
":::'###::::'##::::'##:'####::'######::::::'###::::'########:::'######::'####:",
"::'## ##::: ###::'###:. ##::'##... ##::::'## ##::: ##.... ##:'##... ##:. ##::",
":'##:. ##:: ####'####:: ##:: ##:::..::::'##:. ##:: ##:::: ##: ##:::..::: ##::",
"'##:::. ##: ## ### ##:: ##:: ##::'####:'##:::. ##: ########:: ##:::::::: ##::",
" #########: ##. #: ##:: ##:: ##::: ##:: #########: ##.....::: ##:::::::: ##::",
" ##.... ##: ##:.:: ##:: ##:: ##::: ##:: ##.... ##: ##:::::::: ##::: ##:: ##::",
" ##:::: ##: ##:::: ##:'####:. ######::: ##:::: ##: ##::::::::. ######::'####:",
"..:::::..::..:::::..::....:::......::::..:::::..::..::::::::::......:::....::",
};

static const char * const common_lines[] = {
    "                 Welcome to the AmigaPCI Test and Utilities Disk.",
    "",
    "     This disk contains the following tools:",
    "",
    "       program_flash - Script to program flash with lide.device driver",
    "       bec           - Interact with STM32 Board Environment Controller",
    "       Becky         - Program USB HID (keyboard and mouse) mappings",
    "       apciaconf     - Create fake Zorro entries for PCI area",
    "       apciflash     - Program AmigaPCI flash",
    "       apciscan      - Scan PCI and assign addresses",
//  "       pci           - PCI debug utility",
};

static unsigned int
str_len(const char *str)
{
    const char *ptr = str;

    while (*ptr != '\0')
        ptr++;
    return ((unsigned int)(ptr - str));
}

static void
write_text(const char *text)
{
    Write(out, (APTR)text, str_len(text));
}

static void
write_newline(void)
{
    static const char newline[] = "\n";

    Write(out, (APTR)newline, 1);
}

static void
write_line(const char *line)
{
    write_text(line);
    write_newline();
}

static void
write_spaces(unsigned int count)
{
    static const char spaces[] =
        "                                                                                ";

    while (count > 0) {
        unsigned int chunk = count;

        if (chunk > sizeof (spaces) - 1)
            chunk = sizeof (spaces) - 1;
        Write(out, (APTR)spaces, chunk);
        count -= chunk;
    }
}

static void
write_centered_line(const char *line)
{
    unsigned int len = str_len(line);

    if (len < 80)
        write_spaces((80 - len) / 2);
    write_line(line);
}

static void
write_lines(const char * const *lines, unsigned int count)
{
    unsigned int pos;

    for (pos = 0; pos < count; pos++)
        write_line(lines[pos]);
}

#if 0
static int
path_exists(const char *path)
{
    BPTR lock = Lock((STRPTR)path, ACCESS_READ);

    if (lock == 0)
        return (0);
    UnLock(lock);
    return (1);
}
#endif

int
main(void)
{
    static const char clear_screen[] = "\x1b[0;0H\x1b[2J\n";
    static const char version[] = "Version "VERSION" built "
                                  BUILD_DATE" "BUILD_TIME"\n";

    out = Output();
    if (out == 0)
        return (20);

    Write(out, (APTR)clear_screen, sizeof (clear_screen) - 1);

    write_lines(banner, ARRAY_SIZE(banner));

    write_newline();
    write_centered_line(version);
    write_newline();

    write_lines(common_lines, ARRAY_SIZE(common_lines));

    return (0);
}
