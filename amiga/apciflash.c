/*
 * apciflash
 * ----------
 * Utility to write / read the extra flash on an AmigaPCI board.
 *
 * Copyright 2025 Chris Hooper. This program and source may be used
 * and distributed freely, for any purpose which benefits the Amiga
 * community. Commercial use of the binary, source, or algorithms requires
 * prior written approval from Chris Hooper <amiga@cdh.eebugs.com>.
 * All redistributions must retain this Copyright notice.
 *
 * DISCLAIMER: THE SOFTWARE IS PROVIDED "AS-IS", WITHOUT ANY WARRANTY.
 * THE AUTHOR ASSUMES NO LIABILITY FOR ANY DAMAGE ARISING OUT OF THE USE
 * OR MISUSE OF THIS UTILITY OR INFORMATION REPORTED BY THIS UTILITY.
 */
const char *version = "\0$VER: apciflash "VERSION" ("BUILD_DATE") © Chris Hooper";

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <ctype.h>
#include <string.h>
#include <fcntl.h>
#include <exec/execbase.h>
#include <exec/memory.h>
#include <clib/dos_protos.h>
#include <proto/exec.h>
#include <inline/mmu.h>
#include <mmu/mmubase.h>
#include <mmu/context.h>
#include <mmu/config.h>
#include <mmu/mmutags.h>
#include "cpu_control.h"


#define STATUS_OK               0
#define STATUS_FAIL             1
#define STATUS_PRG_FAIL         2
#define STATUS_PRG_TIMEOUT      3
#define STATUS_BAD_DATA         4

#define ROM_BANKS               4
#define BANK_SIZE               (512 << 10)  // Each bank is 512 KB

#define DUMP_VALUE_UNASSIGNED   0xffffffff

static int flash_id(uint32_t *dev);


/*
 * gcc clib2 headers are bad (for example, no stdint definitions) and are
 * not being included by our build.  Because of that, we need to fix up
 * some stdio definitions.
 */
extern struct iob ** __iob;
#undef stdout
#define stdout ((FILE *)__iob[1])

struct ExecBase *DOSBase;
struct Device   *TimerBase;
static struct    timerequest TimeRequest;

#define BIT(x) (1U << (x))
#define ARRAY_SIZE(x) ((sizeof (x) / sizeof ((x)[0])))

#define VALUE_UNASSIGNED 0xffffffff

#define ROM_WINDOW_SIZE   (512 << 10)  // 512 KB
#define MAX_CHUNK         (16 << 10)   // 16 KB

static const char cmd_options[] =
    "usage: apciflash <options>\n"
    "   debug         show debug output (-d)\n"
    "   erase <opt>   erase flash (-e ?, bank, ...)\n"
    "   identify      identify flash part (-i])\n"
    "   read <opt>    read from flash (-r ?, bank, file, ...)\n"
    "   test          test flash ID (-t) [-l <loops>]\n"
    "   verify <opt>  verify flash matches file (-v ?, bank, file, ...)\n"
    "   write <opt>   write to flash (-w ?, bank, file, ...)\n";

static const char cmd_read_options[] =
    "flash read options\n"
    "   addr <hex>   starting address (-a)\n"
    "   bank <num>   flash bank on which to operate (-b)\n"
    "   dump         save hex/ASCII instead of binary (-d)\n"
    "   file <name>  file where to save content (-f)\n"
    "   len <hex>    length to read in bytes (-l)\n"
    "   swap <mode>  byte swap mode (1032, 2301, 3210) (-s)\n"
    "   yes          skip prompt (-y)\n";

static const char cmd_write_options[] =
    "flash write options\n"
    "   addr <hex>   starting address (-a)\n"
    "   bank <num>   flash bank on which to operate (-b)\n"
//  "   dump         save hex/ASCII instead of binary (-d)\n"
    "   file <name>  file from which to read (-f)\n"
    "   len <hex>    length to program in bytes (-l)\n"
    "   noremap      skip automatic MMU remapping of ROM to RAM (-n)\n"
    "   swap <mode>  byte swap mode (1032, 2301, 3210) (-s)\n"
    "   yes          skip prompt (-y)\n";

static const char cmd_verify_options[] =
    "flash verify options\n"
    "   addr <hex>   starting address (-a)\n"
    "   bank <num>   flash bank on which to operate (-b)\n"
//  "   dump         save hex/ASCII instead of binary (-d)\n"
    "   file <name>  file to verify against (-f)\n"
    "   len <hex>    length to read in bytes (-l)\n"
    "   swap <mode>  byte swap mode (1032, 2301, 3210) (-s)\n"
    "   yes          skip prompt (-y)\n";

static const char cmd_erase_options[] =
    "flash erase options\n"
    "   addr <hex>   starting address (-a)\n"
    "   bank <num>   flash bank on which to operate (-b)\n"
    "   len <hex>    length to erase in bytes (-l)\n"
    "   noremap      skip automatic MMU remapping of ROM to RAM (-n)\n"
    "   yes          skip prompt (-y)\n";

typedef struct {
    const char *const short_name;
    const char *const long_name;
} long_to_short_t;
long_to_short_t long_to_short_main[] = {
    { "-d", "debug" },
    { "-e", "erase" },
    { "-i", "inquiry" },
    { "-i", "identify" },
    { "-i", "id" },
    { "-q", "quiet" },
    { "-r", "read" },
    { "-t", "test" },
    { "-v", "verify" },
    { "-w", "write" },
};

long_to_short_t long_to_short_erase[] = {
    { "-a", "addr" },
    { "-b", "bank" },
    { "-d", "debug" },
    { "-h", "?" },
    { "-h", "help" },
    { "-l", "len" },
    { "-l", "length" },
    { "-n", "noremap" },
    { "-y", "yes" },
};

long_to_short_t long_to_short_readwrite[] = {
    { "-a", "addr" },
    { "-b", "bank" },
    { "-D", "debug" },
    { "-d", "dump" },
    { "-f", "file" },
    { "-h", "?" },
    { "-h", "help" },
    { "-l", "len" },
    { "-l", "length" },
    { "-n", "noremap" },
    { "-s", "swap" },
    { "-y", "yes" },
    { "-r", "read" },
    { "-v", "verify" },
    { "-w", "write" },
};

BOOL __check_abort_enabled = 0;       // Disable gcc clib2 ^C break handling
uint flag_debug = 0;
uint8_t flag_quiet = 0;

static const uint32_t bank_to_base_addr[] = {
    0x00a80000,  // Bank 0
    0x00b00000,  // Bank 1
    0x00e00000,  // Bank 2
    0x00f00000,  // Bank 3
};

static const char *const status_strings[] = {
    "OK",                     // STATUS_OK
    "FAIL",                   // STATUS_FAIL
    "PRG_FAIL",               // STATUS_PRG_FAIL
    "PRG_TIMEOUT",            // STATUS_TIMEOUT
    "BAD_DATA",               // STATUS_BAD_DATA
};

/*
 * status_string
 * -------------
 * Converts status value to a readable string
 *
 * status is the error status code to convert
 */
const char *
status_string(uint status)
{
    static char buf[32];
    const char *str = "Unknown";

    if (status < ARRAY_SIZE(status_strings))
        str = status_strings[status];

    sprintf(buf, "%d %s", status, str);
    return (buf);
}

/*
 * is_user_abort
 * -------------
 * Check for user break input (^C)
 */
static BOOL
is_user_abort(void)
{
    if (SetSignal(0, 0) & SIGBREAKF_CTRL_C) {
        static uint8_t printed = 0;
        if (printed++ == 0)
            printf("^C Abort\n");
        return (1);
    }
    return (0);
}

static void
usage(void)
{
    printf("%s\n\n%s", version + 7, cmd_options);
}

static void
show_rom_banks(void)
{
    uint bank;
    printf("  Bank  Address range\n"
           "  ----  -------------------\n");
    for (bank = 0; bank < ARRAY_SIZE(bank_to_base_addr); bank++)
        printf("%5d   %08x - %08x\n", bank, bank_to_base_addr[bank],
               bank_to_base_addr[bank] + BANK_SIZE - 1);
}

const char *
long_to_short(const char *ptr, long_to_short_t *ltos, uint ltos_count)
{
    uint cur;

    for (cur = 0; cur < ltos_count; cur++)
        if (strcmp(ptr, ltos[cur].long_name) == 0)
            return (ltos[cur].short_name);
    return (ptr);
}

static void
print_us_diff(uint64_t start, uint64_t end)
{
    uint64_t diff = end - start;
    uint32_t diff2;
    char *scale = "ms";

    if ((diff >> 32) != 0)  // Finished before started?
        diff = 0;
    if (diff >= 100000) {
        diff /= 1000;
        scale = "sec";
    }
    diff2 = diff / 10;
    printf("%u.%02u %s\n", diff2 / 100, diff2 % 100, scale);
}

static char
printable_ascii(uint8_t ch)
{
    if (ch >= ' ' && ch <= '~')
        return (ch);
    if (ch == '\t' || ch == '\r' || ch == '\n' || ch == '\0')
        return (' ');
    return ('.');
}

/*
 * dump_memory
 * -----------
 * Display hex and ASCII dump of data at the specified memory location.
 *
 * buf is address of the data.
 * len is the number of bytes to display.
 * dump_base is either an address/offset of DUMP_VALUE_UNASSIGNED if
 *     it should not be printed.
 */
void
dump_memory(void *buf, uint len, uint dump_base)
{
    uint pos;
    uint strpos;
    char str[20];
    uint32_t *src = buf;

    len = (len + 3) / 4;
    if (dump_base != DUMP_VALUE_UNASSIGNED)
        printf("%05x:", dump_base);
    for (strpos = 0, pos = 0; pos < len; pos++) {
        uint32_t val = src[pos];
        printf(" %08x", val);
        str[strpos++] = printable_ascii(val >> 24);
        str[strpos++] = printable_ascii(val >> 16);
        str[strpos++] = printable_ascii(val >> 8);
        str[strpos++] = printable_ascii(val);
        if ((pos & 3) == 3) {
            str[strpos] = '\0';
            strpos = 0;
            printf(" %s\n", str);
            if ((dump_base != DUMP_VALUE_UNASSIGNED) && ((pos + 1) < len)) {
                dump_base += 16;
                printf("%05x:", dump_base);
            }
        }
    }
    if ((pos & 3) != 0) {
        str[strpos] = '\0';
        printf("%*s%s\n", (4 - (pos & 3)) * 9 + 1, "", str);
    }
}

static int
OpenTimer(void)
{
    int rc;

    rc = OpenDevice((STRPTR)TIMERNAME, UNIT_MICROHZ,
                    (struct IORequest *) &TimeRequest, 0L);
    if (rc != 0) {
        printf("Timer open failed\n");
        return (rc);
    }

    TimerBase = (struct Device *) TimeRequest.tr_node.io_Device;
    return (0);
}

static void
CloseTimer(void)
{
    if (TimerBase == NULL)
        return;
    CloseDevice((struct IORequest *) &TimeRequest);
    TimerBase = NULL;
}

static void
exit_cleanup(int exitcode)
{
    CloseTimer();
    exit(exitcode);
}

static uint
getsystime(uint *usec)
{
    if (TimerBase == NULL) {
        *usec = 0;
        return (0);
    }

    TimeRequest.tr_node.io_Command = TR_GETSYSTIME;
    DoIO((struct IORequest *) &TimeRequest);
    *usec = TimeRequest.tr_time.tv_micro;
    return (TimeRequest.tr_time.tv_secs);
}


static uint64_t
get_usec_time(void)
{
    uint usec;
    uint sec = getsystime(&usec);
    return ((uint64_t) sec * 1000000 + usec);
}

typedef struct {
    uint32_t ci_id;       // Vendor code
    char     ci_dev[16];  // ID string for display
} chip_ids_t;

static const chip_ids_t chip_ids[] = {
    { 0x00bf234e, "SST39VF1602C" },  // Microchip 2MB top boot
    { 0x00bf234f, "SST39VF1601C" },  // Microchip 2MB bottom boot
    { 0x00000000, "Unknown" },       // Must remain last
};

typedef struct {
    uint16_t cb_chipid;   // Chip id code
    uint8_t  cb_bbnum;    // Boot block number (0=Bottom boot)
    uint8_t  cb_bsize;    // Common block size in Kwords (typical 32K)
    uint8_t  cb_ssize;    // Boot block sector size in Kwords (typical 4K)
    uint8_t  cb_map;      // Boot block sector erase map
} chip_blocks_t;

/*
 * The bits of cb_map explain the configuration of "sub-blocks" in the boot
 * block. The sub-block term here does not correspond to the datasheet, which
 * simply calls all eraseable blocks, whatever size they may be, blocks.
 * The value of cb_map is computed based on the size of each adjacent
 * "sub-block", where the higher order bits represent the higher addresses
 * in the block. Say the boot block has an organization of 16K, 4K, 4K, 8K,
 * where 8K is the first of those sectors in the physical address map.
 *     Block  Size (KWords)  KWord Address
 *     3      16             04000 - 07FFF
 *     2      4              03000 - 03FFF
 *     1      4              02000 - 02FFF
 *     0      8              00000 - 01FFF
 *
 * Since cb_ssize is 4, we represent block 0 (size 8) using two bits.
 * The low order bit of a sub-block is always 1. The higher order bit(s)
 * are always 0. So block 0 is represented as 01. The rest of the bits can
 * be computed similarly. Size 4 is represented as 1, and 16 as 0001.
 *
 *     Block  Size (KWords)  KWord Address   Bits
 *     3      16             04000 - 07FFF   0001
 *     2      4              03000 - 03FFF   1
 *     1      4              02000 - 02FFF   1
 *     0      8              00000 - 01FFF   01
 *
 * We then assemble those bits into a byte: 00011101, which is 0x1d
 */
static const chip_blocks_t chip_blocks[] = {
    { 0x234e, 31, 32, 4, 0x71 },  // 01110001 8K 4K 4K 16K (top)
    { 0x234f,  0, 32, 4, 0x1d },  // 00011101 16K 4K 4K 8K (bottom)
    { 0x234f,  0, 32, 4, 0x1d },  // Default to bottom boot
};

static const chip_blocks_t *
get_chip_block_info(uint32_t chipid)
{
    uint16_t cid = (uint16_t) chipid;
    uint pos;

    /* Search for exact match */
    for (pos = 0; pos < ARRAY_SIZE(chip_blocks) - 1; pos++)
        if (chip_blocks[pos].cb_chipid == cid)
            break;

    return (&chip_blocks[pos]);
}

const char *
ee_id_string(uint32_t id)
{
    uint pos;

    for (pos = 0; pos < ARRAY_SIZE(chip_ids) - 1; pos++)
        if (chip_ids[pos].ci_id == id)
            break;

    if (pos == ARRAY_SIZE(chip_ids)) {
        uint16_t cid = id & 0xffff;
        for (pos = 0; pos < ARRAY_SIZE(chip_ids) - 1; pos++)
            if ((chip_ids[pos].ci_id & 0xffff) == cid)
                break;
    }
    return (chip_ids[pos].ci_dev);
}

static void
show_test_state(const char * const name, int state)
{
    if (state == 0) {
        if (!flag_quiet)
            printf("PASS\n");
        return;
    }

    if (!flag_quiet || (state != -1))
        printf("  %-15s ", name);

    if (state == -1) {
        fflush(stdout);
        return;
    }
    printf("FAIL\n");
}

static int
test_flash_id(void)
{
    int rc = 0;
    uint32_t dev_first;
    uint32_t dev_again;
    uint     count = 0;
    const char *id_str;
    const chip_blocks_t *cb;

    show_test_state("Flash ID", -1);
    rc = flash_id(&dev_first);
    if (rc != 0) {
        printf("%08x", dev_first);
        goto test_flash_id_fail;
    }

    id_str = ee_id_string(dev_first);
    if (strcmp(id_str, "Unknown") == 0) {
        printf("Failed to identify device (%08x)", dev_first);
        rc = STATUS_BAD_DATA;
    }
    if (rc != 0)
        goto test_flash_id_fail;

    cb = get_chip_block_info(dev_first);
    if (cb == NULL) {
        printf("Failed to determine erase block information for %08x\n",
               dev_first);
        rc = STATUS_BAD_DATA;
        goto test_flash_id_fail;
    }

    for (count = 1; count < 100; count++) {
        rc = flash_id(&dev_again);
        if (rc != 0) {
            printf("%08x", dev_first);
            break;
        }
        if (dev_again != dev_first) {
            printf("  Flash Dev1 ID %08x != first read %08x",
                   dev_again, dev_first);
            rc = 1;
            break;
        }
    }

test_flash_id_fail:
    if (rc == 0) {
        if (flag_quiet)
            return (rc);
        printf("PASS  %08x\n", dev_again);
    } else {
        printf(" at read %u\n", count);
        show_test_state("Flash ID", rc);
    }

    return (rc);
}

static uint
addr_to_bank(uint *addr, uint *bank)
{
    uint cur;
    for (cur = 0; cur < ARRAY_SIZE(bank_to_base_addr); cur++) {
        if ((*addr >= bank_to_base_addr[cur]) &&
            (*addr < bank_to_base_addr[cur] + ROM_WINDOW_SIZE)) {
            *addr -= bank_to_base_addr[cur];
            *bank = cur;
            return (0);
        }
    }
    printf("Specified address %x is not within a window:   \n");
    for (cur = 0; cur < ARRAY_SIZE(bank_to_base_addr); cur++)
        printf(" %08x\n", bank_to_base_addr[cur]);
    printf("\n");
    return (1);
}

/*
 * flash_read_mode() returns the flash to normal read mode.
 */
static void
flash_read_mode(void)
{
    *ADDR32(bank_to_base_addr[0]) = 0xf0;
    cia_spin(CIA_USEC(2));
}

static int
flash_id(uint32_t *dev)
{
    int      rc = 0;

    SUPERVISOR_STATE_ENTER();
    INTERRUPTS_DISABLE();
    CACHE_DISABLE_DATA();
    MMU_DISABLE();

    *ADDR16(bank_to_base_addr[0] + 0xaaa) = 0xaa;
    *ADDR16(bank_to_base_addr[0] + 0x554) = 0x55;
    *ADDR16(bank_to_base_addr[0] + 0xaaa) = 0x90;
    cia_spin(CIA_USEC(2));
    *dev = *ADDR32(bank_to_base_addr[0]);
    flash_read_mode();

    CACHE_FLUSH();
    MMU_RESTORE();
    CACHE_RESTORE_STATE();
    INTERRUPTS_ENABLE();
    SUPERVISOR_STATE_EXIT();

    return (rc);
}

static int
flash_show_id(void)
{
    int      rc;
    uint32_t flash_dev;
    const char *id_str;

    rc = flash_id(&flash_dev);
    if (rc != 0) {
        printf("Flash id failure (%s)\n", status_string(rc));
        return (rc);
    }

    id_str = ee_id_string(flash_dev);

    if (strcmp(id_str, "Unknown") == 0) {
        printf("Failed to identify flash (%08x)\n", flash_dev);
        return (STATUS_BAD_DATA);
    }

    if (flag_quiet)
        return (rc);

    printf("Flash ID: %08x %s\n", flash_dev, id_str);
    return (rc);
}

/*
 * are_you_sure() prompts the user to confirm that an operation is intended.
 *
 * @param  [in]  None.
 *
 * @return       TRUE  - User has confirmed (Y).
 * @return       FALSE - User has denied (N).
 */
int
are_you_sure(const char *prompt)
{
    int ch;
ask_again:
    printf("%s - are you sure? (y/n) ", prompt);
    fflush(stdout);
    while ((ch = getchar()) != EOF) {
        if (is_user_abort())
            return (FALSE);
        if ((ch == 'y') || (ch == 'Y'))
            return (TRUE);
        if ((ch == 'n') || (ch == 'N'))
            return (FALSE);
        if (!isspace(ch))
            goto ask_again;
    }
    return (FALSE);
}

static uint
get_file_size(const char *filename)
{
    struct FileInfoBlock fb;
    BPTR lock;
    fb.fib_Size = VALUE_UNASSIGNED;
    lock = Lock(filename, ACCESS_READ);
    if (lock == 0L) {
        printf("Lock %s failed\n", filename);
        return (VALUE_UNASSIGNED);
    }
    if (Examine(lock, &fb) == 0) {
        printf("Examine %s failed\n", filename);
        UnLock(lock);
        return (VALUE_UNASSIGNED);
    }
    UnLock(lock);
    return (fb.fib_Size);
}

#define SWAPMODE_A500  0xA500   // Amiga 16-bit ROM format
#define SWAPMODE_A3000 0xA3000  // Amiga 32-bit ROM format

#define SWAP_TO_ROM    0  // Bytes originated in a file (to be written in ROM)
#define SWAP_FROM_ROM  1  // Bytes originated in ROM (to be written to a file)

/*
 * execute_swapmode() swaps bytes in the specified buffer according to the
 *                    currently active swap mode.
 *
 * @param  [io]  buf      - Buffer to modify.
 * @param  [in]  len      - Length of data in the buffer.
 * @global [in]  dir      - Image swap direction (SWAP_TO_ROM or SWAP_FROM_ROM)
 * @global [in]  swapmode - Swap operation to perform (0123, 3210, etc)
 * @return       None.
 */
static void
execute_swapmode(uint8_t *buf, uint len, uint dir, uint swapmode)
{
    uint    pos;
    uint8_t temp;
    static const uint8_t str_f94e1411[] = { 0xf9, 0x4e, 0x14, 0x11 };
    static const uint8_t str_11144ef9[] = { 0x11, 0x14, 0x4e, 0xf9 };
    static const uint8_t str_1411f94e[] = { 0x14, 0x11, 0xf9, 0x4e };
    static const uint8_t str_4ef91114[] = { 0x4e, 0xf9, 0x11, 0x14 };

    switch (swapmode) {
        case 0:
        case 0123:
            return;  // Normal (no swap)
        swap_1032:
        case 1032:
            /* Swap adjacent bytes in 16-bit words */
            for (pos = 0; pos < len - 1; pos += 2) {
                temp         = buf[pos + 0];
                buf[pos + 0] = buf[pos + 1];
                buf[pos + 1] = temp;
            }
            return;
        swap_2301:
        case 2301:
            /* Swap adjacent (16-bit) words */
            for (pos = 0; pos < len - 3; pos += 4) {
                temp         = buf[pos + 0];
                buf[pos + 0] = buf[pos + 2];
                buf[pos + 2] = temp;
                temp         = buf[pos + 1];
                buf[pos + 1] = buf[pos + 3];
                buf[pos + 3] = temp;
            }
            return;
        swap_3210:
        case 3210:
            /* Swap bytes in 32-bit longs */
            for (pos = 0; pos < len - 3; pos += 4) {
                temp         = buf[pos + 0];
                buf[pos + 0] = buf[pos + 3];
                buf[pos + 3] = temp;
                temp         = buf[pos + 1];
                buf[pos + 1] = buf[pos + 2];
                buf[pos + 2] = temp;
            }
            return;
        case SWAPMODE_A500:
            if (dir == SWAP_TO_ROM) {
                /* Need bytes in order: 14 11 f9 4e */
                if (memcmp(buf, str_1411f94e, 4) == 0)
                    return;  // Already in desired order
                if (memcmp(buf, str_11144ef9, 4) == 0) {
                    printf("Swap mode 2301\n");
                    goto swap_2301;  // Swap adjacent 16-bit words
                }
            }
            if (dir == SWAP_FROM_ROM) {
                /* Need bytes in order: 11 14 4e f9 */
                if (memcmp(buf, str_11144ef9, 4) == 0)
                    return;  // Already in desired order
                if (memcmp(buf, str_1411f94e, 4) == 0) {
                    printf("Swap mode 1032\n");
                    goto swap_1032;  // Swap odd/even bytes
                }
            }
            goto unrecognized;
        case SWAPMODE_A3000:
            if (dir == SWAP_TO_ROM) {
                /* Need bytes in order: f9 4e 14 11 */
                if (memcmp(buf, str_f94e1411, 4) == 0)
                    return;  // Already in desired order
                if (memcmp(buf, str_11144ef9, 4) == 0) {
                    printf("Swap mode 3210\n");
                    goto swap_3210;  // Swap bytes in 32-bit longs
                }
                if (memcmp(buf, str_1411f94e, 4) == 0) {
                    printf("Swap mode 2301\n");
                    goto swap_2301;  // Swap adjacent 16-bit words
                }
                if (memcmp(buf, str_4ef91114, 4) == 0) {
                    printf("Swap mode 1032\n");
                    goto swap_1032;  // Swap odd/even bytes
                }
            }
            if (dir == SWAP_FROM_ROM) {
                /* Need bytes in order: 11 14 4e f9 */
                if (memcmp(buf, str_11144ef9, 4) == 0)
                    return;  // Already in desired order
                if (memcmp(buf, str_f94e1411, 4) == 0) {
                    printf("Swap mode 3210\n");
                    goto swap_3210;  // Swap bytes in 32-bit longs
                }
                if (memcmp(buf, str_4ef91114, 4) == 0) {
                    printf("Swap mode 2301\n");
                    goto swap_2301;  // Swap adjacent 16-bit words
                }
                if (memcmp(buf, str_1411f94e, 4) == 0) {
                    printf("Swap mode 1032\n");
                    goto swap_1032;  // Swap odd/even bytes
                }
            }
unrecognized:
            printf("Unrecognized Amiga ROM format: %02x %02x %02x %02x\n",
                   buf[0], buf[1], buf[2], buf[3]);
            exit_cleanup(EXIT_FAILURE);
    }
}

static uint
read_from_flash(uint bank, uint addr, void *buf, uint len)
{
    SUPERVISOR_STATE_ENTER();
    INTERRUPTS_DISABLE();
    CACHE_DISABLE_DATA();
    MMU_DISABLE();

    memcpy(buf, (void *) bank_to_base_addr[bank] + addr, len);

    CACHE_FLUSH();
    MMU_RESTORE();
    CACHE_RESTORE_STATE();
    INTERRUPTS_ENABLE();
    SUPERVISOR_STATE_EXIT();

    return (0);
}

static int
wait_for_flash_done(uint addr, uint erase_mode)
{
    uint32_t status;
    uint32_t lstatus;
    uint     spins = erase_mode ? 1000000 : 50000; // 1 sec or 50ms
    uint     spin_count = 0;
    int      same_count = 0;
    int      see_fail_count = 0;

    cia_spin(1);
    lstatus = *ADDR16(addr);
    while (spin_count < spins) {
        status = *ADDR16(addr);

        if (status == lstatus) {
            if (same_count++ >= 2) {
                /* Same for 3 tries */
                if (erase_mode && (status != 0xffff)) {
                    /* Something went wrong -- block protected? */
                    return (STATUS_PRG_FAIL);
                }
                return (0);
            }
        } else {
            same_count = 0;
            lstatus = status;
        }

        if (status & BIT(5))  // Program / erase failure
            if (see_fail_count++ > 5)
                break;
    }

    if (status & BIT(5)) {
        /* Program / erase failure */
        return (STATUS_PRG_FAIL);
    }

    /* Program / erase timeout */
    return (STATUS_PRG_TIMEOUT);
}

static uint
write_to_flash(uint bank, uint addr, void *buf, uint len)
{
    uint rc = 0;
    uint xlen;
    uint8_t *xbuf = buf;
    uint16_t data;

    SUPERVISOR_STATE_ENTER();
    INTERRUPTS_DISABLE();
    CACHE_DISABLE_DATA();
    MMU_DISABLE();

    /* Write flash data */
    while (len > 0) {
        xlen = len;
        if (xlen > 2)
            xlen = 2;
        data = (xbuf[0] << 8) | xbuf[1];

        /* Send write command */
        *ADDR16(bank_to_base_addr[0] + 0xaaa)   = 0x00aa;
        *ADDR16(bank_to_base_addr[0] + 0x554)   = 0x0055;
        *ADDR16(bank_to_base_addr[0] + 0xaaa)   = 0x00a0;
        *ADDR16(bank_to_base_addr[bank] + addr) = data;

        rc = wait_for_flash_done(bank_to_base_addr[0], 0);
        if (rc != 0)
            break;

        len  -= xlen;
        xbuf += xlen;
        addr += xlen;
    }

    /* Restore flash to read mode */
    flash_read_mode();

    CACHE_FLUSH();
    MMU_RESTORE();
    CACHE_RESTORE_STATE();
    INTERRUPTS_ENABLE();
    SUPERVISOR_STATE_EXIT();
    return (rc);
}

static uint
erase_flash_block(uint bank, uint addr)
{
    uint rc = 0;

    if (flag_debug)
        printf("erase at %x %x\n", bank, addr);

    SUPERVISOR_STATE_ENTER();
    INTERRUPTS_DISABLE();
    CACHE_DISABLE_DATA();
    MMU_DISABLE();

    /* Send erase command */
    *ADDR16(bank_to_base_addr[0] + 0xaaa)   = 0x00aa;
    *ADDR16(bank_to_base_addr[0] + 0x554)   = 0x0055;
    *ADDR16(bank_to_base_addr[0] + 0xaaa)   = 0x0080;
    *ADDR16(bank_to_base_addr[0] + 0xaaa)   = 0x00aa;
    *ADDR16(bank_to_base_addr[0] + 0x554)   = 0x0055;
//  *ADDR16(bank_to_base_addr[bank] + addr) = 0x0050;  // Sector Erase
    *ADDR16(bank_to_base_addr[bank] + addr) = 0x0030;  // Block Erase
//  *ADDR16(bank_to_base_addr[0] + 0x554)   = 0x0010;  // Chip Erase

    cia_spin(CIA_USEC(10));
    rc = wait_for_flash_done(bank_to_base_addr[0] + addr, 0);

    /* Restore flash to read mode */
    flash_read_mode();

    CACHE_FLUSH();
    MMU_RESTORE();
    CACHE_RESTORE_STATE();
    INTERRUPTS_ENABLE();
    SUPERVISOR_STATE_EXIT();
    return (rc);
}

static uint
get_flash_bsize(const chip_blocks_t *cb, uint flash_addr)
{
    uint flash_bsize = cb->cb_bsize << (10 + 1);  // bsize is in 16-bit words
    uint flash_bnum  = flash_addr / flash_bsize;
    if (flag_debug) {
        printf("Erase at %x bnum=%x: flash_bsize=%x flash_bbnum=%x\n",
               flash_addr, flash_bnum, flash_bsize, cb->cb_bbnum);
    }
    if (flash_bnum == cb->cb_bbnum) {
        /*
         * Boot block area has variable sub-block size.
         *
         * The map is 8 bits which are arranged in order such that Bit 0
         * represents the first sub-block and Bit 7 represents the last
         * sub-block.
         *
         * If a given bit is 1, this is the start of an erase block.
         * If a given bit is 0, then it is a continuation of the previous
         * bit's erase block.
         */
        uint bboff = flash_addr & (flash_bsize - 1);
        uint bsnum = (bboff / cb->cb_ssize) >> (10 + 1);
        uint first_snum = bsnum;
        uint last_snum = bsnum;
        uint smap = cb->cb_map;
        if (flag_debug)
            printf(" bblock bb_off=%x snum=%x s_map=%x\n", bboff, bsnum, smap);
        flash_bsize = 0;
        /* Find first bit of this map */
        while (first_snum > 0) {
            if (smap & BIT(first_snum))  // Found base
                break;
            first_snum--;
        }
        while (++last_snum < 8) {
            if (smap & BIT(last_snum))  // Found next base
                break;
        }
        flash_bsize = (cb->cb_ssize * (last_snum - first_snum)) << (10 + 1);
        if (flag_debug) {
            printf(" first_snum=%x last_snum=%x bb_ssize=%x\n",
                   first_snum, last_snum, flash_bsize);
        }
    } else if (flag_debug) {
        printf(" normal block %x\n", flash_bsize);
    }
    return (flash_bsize);
}

static uint
lib_is_loaded(const char *name)
{
    struct List *libs = &SysBase->LibList;
    struct Library *lib;
    struct Node *node;

    for (node = libs->lh_Head; node->ln_Succ != NULL; node = node->ln_Succ) {
        lib = (struct Library *)node;
        if (strcmp(name, lib->lib_Node.ln_Name) == 0)
            return (1);
    }
    return (0);
}

/*
 * Flash memory to RAM remapping code taken from example code by
 * LIV2 (Matt Harlum).
 *
 * These functions are used to prevent an Amiga crash when writing
 * to flash which has active Amiga code executing from it. The
 * solution is to first copy that flash to RAM, then use the MMU
 * to remap the logical flash address to the physical RAM copy.
 */
static struct MMUBase *MMUBase;

/*
 * RelocateRomBank
 * ---------------
 * Relocate the specified ROM (flash) address to RAM by allocating a
 * suitable size block of RAM, copying its current contents and then
 * configuring the MMU to remap.
 */
static bool
RelocateRomBank(struct MMUContext *ctx, struct MMUContext *sctx, void *src)
{
    void *dest;
    void *phys;
    ULONG pageSize = GetPageSize(ctx);
    ULONG size = BANK_SIZE;
    ULONG psize;

    if (GetProperties(ctx, (ULONG)src, TAG_DONE) & MAPP_REMAPPED)
        return (true);  // Already re-mapped; do nothing

    dest = AllocAligned(BANK_SIZE, MEMF_REVERSE | MEMF_FAST, pageSize);
    if (dest == NULL)
        return (false);

    CopyMemQuick(src, dest, size);
    phys = dest;
    psize = size;

    /* Get physical location of mirror */
    if ((PhysicalLocation(ctx, &phys, &psize) != 0) &&
        SetProperties(ctx, MAPP_REMAPPED | MAPP_COPYBACK | MAPP_ROM,
                      -1, (ULONG)src, size,
                      MAPTAG_DESTINATION, (ULONG)dest,
                      TAG_DONE) &&
        SetProperties(ctx, MAPP_ROM, MAPP_ROM, (ULONG)phys, size,
                      TAG_DONE)) {
        /* Success */
        printf("Remapped flash at %08x to RAM\n", (ULONG)src);

        /* Now do the same for Supervisor context */
        if (GetProperties(sctx, (ULONG)src, TAG_DONE) & MAPP_REMAPPED)
            return (true);  // Already re-mapped; do nothing

        if ((PhysicalLocation(sctx, &phys, &psize) != 0) &&
            SetProperties(sctx, MAPP_REMAPPED | MAPP_COPYBACK | MAPP_ROM,
                          -1, (ULONG)src, size,
                          MAPTAG_DESTINATION, (ULONG)dest,
                          TAG_DONE)) {
            SetProperties(sctx, MAPP_ROM, MAPP_ROM, (ULONG)phys, size,
                          TAG_DONE);
        }

        return (true);
    } else {
        /* Failure */
        FreeMem(dest, size);
        return (false);
    }
}

/*
 * remap_flash_to_ram
 * ------------------
 * Remap the specified flash bank to RAM by copying its current contents.
 */
static uint
remap_flash_to_ram(uint bank)
{
    uint baseaddr = bank_to_base_addr[bank];
    char mmu;
    uint rc = 1;  // default to fail
    struct MMUContext *ctx;
    struct MMUContext *sctx;
    struct MinList *ctxl = NULL;
    struct MinList *sctxl = NULL;

#define MMU_LIB_MIN_VER 41
    MMUBase = (struct MMUBase *) OpenLibrary(MMU_NAME, MMU_LIB_MIN_VER);
    if (MMUBase == NULL) {
        printf("Failed to open mmu.library %u\n", MMU_LIB_MIN_VER);
        return (1);
    }

    mmu = GetMMUType();
    if (!mmu) {
        printf("MMU is required for flash remap.\n");
        goto fail_close_library;
    }
    ctx  = DefaultContext();  // User context
    sctx = SuperContext(ctx); // Supervisor context
    LockContextList();
    LockMMUContext(ctx);
    LockMMUContext(sctx);

    if (((ctxl = GetMapping(ctx)) == NULL) ||
        ((sctxl = GetMapping(sctx)) == NULL)) {
        printf("Failed to lock MMU context\n");
        goto fail_unlock_mmu;
    }

#if 0
    if (RelocateRomBank(ctx, (void *) baseaddr) &&
        RelocateRomBank(sctx, (void *) baseaddr) &&
        RebuildTree(ctx) &&
        RebuildTree(sctx)) {
    } else {
        SetPropertyList(ctx, ctxl);
        SetPropertyList(sctx, sctxl);
    }
#else
    if (RelocateRomBank(ctx, sctx, (void *) baseaddr) &&
        RebuildTree(ctx) &&
        RebuildTree(sctx)) {
    } else {
        SetPropertyList(ctx, ctxl);
        SetPropertyList(sctx, sctxl);
    }
#endif

    rc = 0; // Successfully remapped bank

fail_unlock_mmu:
    if (sctxl != NULL)
        ReleaseMapping(sctx, sctxl);
    if (ctxl != NULL)
        ReleaseMapping(ctx, ctxl);

    UnlockMMUContext(sctx);
    UnlockMMUContext(ctx);
    UnlockContextList();

fail_close_library:
    CloseLibrary((struct Library *)MMUBase);

    return (rc);
}

/*
 * remap_flash_to_ram_check
 * ------------------------
 * Verify mmu.library is available for memory remapping
 */
static uint
remap_flash_to_ram_check(uint bank)
{
    uint baseaddr = bank_to_base_addr[bank];

    if (!lib_is_loaded("mmu.library")) {
        printf("mmu.library is required for flash remap.\n"
               "If you are sure the Amiga is not using flash at %08x,\n"
               "you may add the noremap (-n) option to skip remapping.\n",
               baseaddr);
        return (1);
    }
    return (0);
}

/*
 * flash_erase
 * -----------
 * Erase specified flash bank.
 */
static int
flash_erase(uint bank, uint addr, uint len, uint flag_yes, uint flag_noremap)
{
    int         rc;
    const chip_blocks_t *cb;
    const char *id_str;
    uint64_t    time_start;
    uint64_t    time_end;
    uint32_t    flash_dev;
    uint        tlen = 0;
    uint        dot_count = 0;
    uint        dot_iters = 1;
    uint        dot_max;
    uint        flash_start_addr;
    uint        flash_end_addr;
    uint        flash_start_bsize;
    uint        flash_end_bsize;

    /* Acquire flash id to get block erase zones */
    rc = flash_id(&flash_dev);
    if (rc != 0) {
        printf("Flash id failure (%s)\n", status_string(rc));
        return (rc);
    }

    id_str = ee_id_string(flash_dev);
    if (flag_debug)
        printf("    %08x %s\n", flash_dev, id_str);

    if (strcmp(id_str, "Unknown") == 0) {
        printf("Failed to identify flash (%08x)\n", flash_dev);
        rc = STATUS_BAD_DATA;
    }
    if (rc != 0)
        return (rc);

    cb = get_chip_block_info(flash_dev);
    if (cb == NULL) {
        printf("Failed to determine erase block information for %08x\n",
               flash_dev);
        return (STATUS_BAD_DATA);
    }

    flash_start_addr  = bank * ROM_WINDOW_SIZE + addr;
    flash_end_addr    = bank * ROM_WINDOW_SIZE + addr + len - 1;
    flash_start_bsize = get_flash_bsize(cb, flash_start_addr);
    flash_end_bsize   = get_flash_bsize(cb, flash_end_addr);

    if (flag_debug)
        printf("pre saddr=%x eaddr=%x\n", flash_start_addr, flash_end_addr);

    /* Round start address down and end address up, then compute length */
    flash_start_addr = flash_start_addr & ~(flash_start_bsize - 1);
    flash_end_addr   = (flash_end_addr | (flash_end_bsize - 1)) + 1;
    len = flash_end_addr - flash_start_addr;
    addr = addr & ~(flash_start_bsize - 1);

    if (flag_debug) {
        printf("saddr=%x sbsize=%x\n", flash_start_addr, flash_start_bsize);
        printf("eaddr=%x ebsize=%x\n", flash_end_addr, flash_end_bsize);
    }

    if (!flag_noremap && remap_flash_to_ram_check(bank))
        return (1);

    printf("Erase bank=%u addr=%x len=%x\n", bank, addr, len);
    if ((!flag_yes) && (!are_you_sure("Proceed")))
        return (1);

    if (!flag_noremap && remap_flash_to_ram(bank))
        return (1);

    dot_max = (len + MAX_CHUNK - 1) / MAX_CHUNK;
    while (dot_max > 50) {
        dot_max >>= 1;
        dot_iters <<= 1;
    }
    printf("Erase  [%*s]\rErase  [", dot_max, "");
    fflush(stdout);

    time_start = get_usec_time();

    bank += addr / ROM_WINDOW_SIZE;
    addr &= (ROM_WINDOW_SIZE - 1);

    while (len > 0) {
        uint xlen = get_flash_bsize(cb, bank * ROM_WINDOW_SIZE + addr);

        rc = erase_flash_block(bank, addr);
        if (rc != 0) {
            printf("\nErase failure (%s)\n", status_string(rc));
            break;
        }
        if (is_user_abort()) {
            rc = 2;
            break;
        }

        tlen += xlen;
        len  -= xlen;
        addr += xlen;
        if (addr >= ROM_WINDOW_SIZE) {
            addr -= ROM_WINDOW_SIZE;
            bank++;
        }
        if (tlen >= MAX_CHUNK) {
            while (tlen >= MAX_CHUNK) {
                tlen -= MAX_CHUNK;
                if (++dot_count == dot_iters) {
                    dot_count = 0;
                    printf(".");
                }
            }
            fflush(stdout);
        }
    }
    if (rc == 0) {
        while (tlen >= MAX_CHUNK) {
            tlen -= MAX_CHUNK;
            if (++dot_count == dot_iters) {
                dot_count = 0;
                printf(".");
            }
        }
        printf("]\n");
        time_end = get_usec_time();
        printf("Erase complete in ");
        print_us_diff(time_start, time_end);
    }

    return (rc);
}

/*
 * cmd_erase
 * ---------
 * Erase flash (interpret user options and direct erase)
 */
int
cmd_erase(int argc, char *argv[])
{
    const char *ptr;
    uint        addr = VALUE_UNASSIGNED;
    uint        bank = VALUE_UNASSIGNED;
    uint        len  = VALUE_UNASSIGNED;
    uint        flag_noremap = 0;
    uint        flag_yes = 0;
    uint        rc = 1;
    uint        bank_size = BANK_SIZE;
    int         arg;
    int         pos;

    for (arg = 1; arg < argc; arg++) {
        ptr = long_to_short(argv[arg], long_to_short_erase,
                            ARRAY_SIZE(long_to_short_erase));
        if (*ptr == '-') {
            for (++ptr; *ptr != '\0'; ptr++) {
                switch (*ptr) {
                    case 'a':  // addr
                        if (++arg >= argc) {
                            show_rom_banks();
                            printf("apciflash %s -%s requires an option\n",
                                   argv[0], ptr);
                            goto usage;
                        }
                        pos = 0;
                        if ((sscanf(argv[arg], "%x%n", &addr, &pos) != 1) ||
                            (pos == 0) || (argv[arg][pos] != '\0')) {
                            printf("Invalid argument \"%s\" for %s -%s\n",
                                   argv[arg], argv[0], ptr);
                            show_rom_banks();
                            goto usage;
                        }
                        break;
                    case 'b':  // bank
                        if (++arg >= argc) {
                            show_rom_banks();
                            printf("apciflash %s -%s requires an option\n",
                                   argv[0], ptr);
                            goto usage;
                        }
                        pos = 0;
                        if ((sscanf(argv[arg], "%x%n", &bank, &pos) != 1) ||
                            (pos == 0) || (argv[arg][pos] != '\0') ||
                            (bank >= ROM_BANKS)) {
                            printf("Invalid argument \"%s\" for %s -%s\n",
                                   argv[arg], argv[0], ptr);
                            show_rom_banks();
                            goto usage;
                        }
                        break;
                    case 'd':  // debug
                        flag_debug++;
                        break;
                    case 'h':  // help
                        rc = 0;
usage:
                        printf("%s", cmd_erase_options);
                        return (rc);
                    case 'l':  // length
                        if (++arg >= argc) {
                            printf("apciflash %s -%s requires an option\n",
                                   argv[0], ptr);
                        }
                        pos = 0;
                        if ((sscanf(argv[arg], "%x%n", &len, &pos) != 1) ||
                            (pos == 0) || (argv[arg][pos] != '\0')) {
                            printf("Invalid argument \"%s\" for %s -%s\n",
                                   argv[arg], argv[0], ptr);
                            goto usage;
                        }
                        break;
                    case 'n':  // noremap
                        flag_noremap++;
                        break;
                    case 'y':  // yes
                        flag_yes++;
                        break;
                    default:
                        printf("Unknown argument %s \"-%s\"\n",
                               argv[0], ptr);
                        goto usage;
                }
            }
        } else {
            printf("Unknown argument %s \"%s\"\n",
                   argv[0], ptr);
            goto usage;
        }
    }

    if (addr == VALUE_UNASSIGNED)
        addr = 0;

    if ((addr > ROM_WINDOW_SIZE) && addr_to_bank(&addr, &bank)) {
        rc = 1;
        goto usage;
    }

    if (bank == VALUE_UNASSIGNED) {
        printf("You must supply a bank number\n");
        goto usage;
    }
    bank_size = BANK_SIZE;

    if (len == VALUE_UNASSIGNED) {
        /* Get length from size of bank */
        len = bank_size - (addr & (bank_size - 1));
    } else if (len > bank_size) {
        printf("Specified length 0x%x is greater than bank size 0x%x\n",
               len, bank_size);
        return (1);
    } else if (addr + len > bank_size) {
        printf("Specified address + length (0x%x) overflows bank (size 0x%x)\n",
               addr + len, bank_size);
        return (1);
    }

    return (flash_erase(bank, addr, len, flag_yes, flag_noremap));
}


/*
 * cmd_readwrite
 * -------------
 * Read from flash and write to file or read from file and write to flash.
 * This function also performs the verify operation (read from file and
 * verify flash contents).
 */
int
cmd_readwrite(int argc, char *argv[])
{
    const char *ptr;
    const char *filename = NULL;
    uint64_t    time_start;
    uint64_t    time_rw_end;
    uint64_t    time_end;
    int         arg;
    int         pos;
    int         bytes;
    uint        flag_dump = 0;
    uint        flag_noremap = 0;
    uint        flag_yes = 0;
    uint        addr = VALUE_UNASSIGNED;
    uint        bank = VALUE_UNASSIGNED;
    uint        len  = VALUE_UNASSIGNED;
    uint        start_addr;
    uint        start_bank;
    uint        start_len;
    uint        file_is_stdio = 0;
    uint        rc = 1;
    uint        bank_size;
    FILE       *file;
    uint        swapmode = 0123;  // no swap
    uint        writemode = 0;
    uint        verifymode = 0;
    uint        readmode = 0;
    uint        dot_count = 1;
    uint        dot_iters = 1;
    uint        dot_max;
    uint8_t    *buf;
    uint8_t    *rbuf = NULL;

    /* First Just determine the read/write/verify mode */
    for (arg = 0; arg < argc; arg++) {
        ptr = long_to_short(argv[arg], long_to_short_readwrite,
                            ARRAY_SIZE(long_to_short_readwrite));
        if (*ptr == '-') {
            for (++ptr; *ptr != '\0'; ptr++) {
                switch (*ptr) {
                    case 'r':  // read
                        readmode = 1;
                        break;
                    case 'v':  // verify
                        verifymode = 1;
                        break;
                    case 'w':  // write
                        writemode = 1;
                        break;
                }
            }
        }
    }

    for (arg = 0; arg < argc; arg++) {
        ptr = long_to_short(argv[arg], long_to_short_readwrite,
                            ARRAY_SIZE(long_to_short_readwrite));
        if (*ptr == '-') {
            for (++ptr; *ptr != '\0'; ptr++) {
                switch (*ptr) {
                    case 'a':  // addr
                        if (++arg >= argc) {
                            show_rom_banks();
                            printf("apciflash %s -%s requires an option\n",
                                   argv[0], ptr);
                            goto usage;
                        }
                        pos = 0;
                        if ((sscanf(argv[arg], "%x%n", &addr, &pos) != 1) ||
                            (pos == 0) || (argv[arg][pos] != '\0')) {
                            printf("Invalid argument \"%s\" for %s -%s\n",
                                   argv[arg], argv[0], ptr);
                            show_rom_banks();
                            goto usage;
                        }
                        break;
                    case 'b':  // bank
                        if (++arg >= argc) {
                            show_rom_banks();
                            printf("apciflash %s -%s requires an option\n",
                                   argv[0], ptr);
                            goto usage;
                        }
                        pos = 0;
                        if ((sscanf(argv[arg], "%x%n", &bank, &pos) != 1) ||
                            (pos == 0) || (argv[arg][pos] != '\0') ||
                            (bank >= ROM_BANKS)) {
                            printf("Invalid argument \"%s\" for %s -%s\n",
                                   argv[arg], argv[0], ptr);
                            show_rom_banks();
                            goto usage;
                        }
                        break;
                    case 'D':  // debug
                        flag_debug++;
                        break;
                    case 'd':  // dump
                        flag_dump++;
                        break;
                    case 'f':  // file
                        if (++arg >= argc) {
                            printf("apciflash %s -%s requires an option\n",
                                   argv[0], ptr);
                            goto usage;
                        }
                        filename = argv[arg];
                        if (strcmp(filename, "-") == 0)
                            file_is_stdio++;
                        break;
                    case 'h':  // help
                        rc = 0;
usage:
                        if (writemode)
                            printf("%s", cmd_write_options);
                        else if (readmode)
                            printf("%s", cmd_read_options);
                        else
                            printf("%s", cmd_verify_options);
                        return (rc);
                    case 'l':  // length
                        if (++arg >= argc) {
                            printf("apciflash %s -%s requires an option\n",
                                   argv[0], ptr);
                            goto usage;
                        }
                        pos = 0;
                        if ((sscanf(argv[arg], "%x%n", &len, &pos) != 1) ||
                            (pos == 0) || (argv[arg][pos] != '\0')) {
                            printf("Invalid argument \"%s\" for %s -%s\n",
                                   argv[arg], argv[0], ptr);
                            goto usage;
                        }
                        break;
                    case 'n':  // noremap
                        flag_noremap++;
                        break;
                    case 's':  // swap
                        if (++arg >= argc) {
                            printf("apciflash %s -%s requires an option\n",
                                   argv[0], ptr);
                            goto usage;
                        }
                        if ((strcasecmp(argv[arg], "a3000") == 0) ||
                            (strcasecmp(argv[arg], "a4000") == 0) ||
                            (strcasecmp(argv[arg], "a3000t") == 0) ||
                            (strcasecmp(argv[arg], "a4000t") == 0) ||
                            (strcasecmp(argv[arg], "a1200") == 0)) {
                            swapmode = SWAPMODE_A3000;
                            break;
                        }
                        if ((strcasecmp(argv[arg], "a500") == 0) ||
                            (strcasecmp(argv[arg], "a600") == 0) ||
                            (strcasecmp(argv[arg], "a1000") == 0) ||
                            (strcasecmp(argv[arg], "a2000") == 0) ||
                            (strcasecmp(argv[arg], "cdtv") == 0)) {
                            swapmode = SWAPMODE_A500;
                            break;
                        }
                        pos = 0;
                        if ((sscanf(argv[arg], "%u%n", &swapmode, &pos) != 1) ||
                            (pos == 0) || (argv[arg][pos] != '\0') ||
                            ((swapmode != 0123) && (swapmode != 1032) &&
                             (swapmode != 2301) && (swapmode != 3210))) {
                            printf("Invalid argument \"%s\" for %s -%s\n",
                                   argv[arg], argv[0], ptr);
                            printf("Use 1032, 2301, or 3210\n");
                            return (1);
                        }
                        break;
                    case 'y':  // yes
                        flag_yes++;
                        break;
                    case 'r':  // read
                    case 'v':  // verify
                    case 'w':  // write
                        /* These were handled in the first pass */
                        break;
                    default:
                        printf("Unknown argument %s \"-%s\"\n",
                               argv[0], ptr);
                        goto usage;
                }
            }
        } else {
            printf("Unknown argument %s \"%s\"\n",
                   argv[0], ptr);
            goto usage;
        }
    }
    if (filename == NULL) {
        if (flag_dump) {
            file_is_stdio++;
            filename = "-";
        } else {
            printf("You must supply a filename");
            if (readmode)
                printf(" or - for stdout");
            printf("\n");
            goto usage;
        }
    }
    if (addr == VALUE_UNASSIGNED)
        addr = 0;

    if ((addr > ROM_WINDOW_SIZE) && addr_to_bank(&addr, &bank)) {
        rc = 1;
        goto usage;
    }

    if (bank == VALUE_UNASSIGNED) {
        printf("You must supply a bank number\n");
        goto usage;
    }
    if (flag_dump && !file_is_stdio) {
        printf("Can only dump ASCII text to stdout\n");
        return (1);
    }

    bank_size = 512 << 10;

    if (len == VALUE_UNASSIGNED) {
        if (readmode) {
            /* Get length from size of bank */
            len = bank_size;
        } else {
            /* Get length from size of file */
            len = get_file_size(filename);
            if (len == VALUE_UNASSIGNED) {
                return (1);
            }
        }
    }
    if (len > bank_size) {
        printf("Length 0x%x is greater than bank size 0x%x\n",
               len, bank_size);
        return (1);
    } else if (addr + len > bank_size) {
        printf("Length 0x%x + address overflows bank (size 0x%x)\n",
               addr + len, bank_size);
        return (1);
    }

    if (!readmode && file_is_stdio) {
        printf("STDIO not supported for this mode\n");
        return (1);
    }

    if (writemode) {
        if (addr == 0)
            printf("Erase and ");
        printf("Write bank=%u addr=%x len=%x from ", bank, addr, len);
        if (file_is_stdio)
            printf("stdin");
        else
            printf("file=\"%s\"", filename);
        printf("\n");
    } else {
        if (readmode)
            printf("Read");
        else
            printf("Verify");
        printf(" bank=%u addr=%x len=%x ", bank, addr, len);
        if (readmode)
            printf("to ");
        else
            printf("matches ");
        if (file_is_stdio)
            printf("stdout");
        else
            printf("file=\"%s\"", filename);
        if (flag_dump)
            printf(" (ASCII dump)");
        printf("\n");
    }
    if (len > ROM_WINDOW_SIZE) {
        printf("Length %x is larger than bank size %x\n", len, ROM_WINDOW_SIZE);
        return (1);
    }
    if ((addr > ROM_WINDOW_SIZE) && addr_to_bank(&addr, &bank)) {
        return (1);
    }

    if (writemode && !flag_noremap && remap_flash_to_ram_check(bank))
        return (1);

    if ((!flag_yes) && (!file_is_stdio || (flag_dump && (len >= 0x1000))) &&
        (!are_you_sure("Proceed"))) {
        return (1);
    }

    if (writemode && !flag_noremap && remap_flash_to_ram(bank))
        return (1);

    buf = AllocMem(MAX_CHUNK, MEMF_PUBLIC);
    if (buf == NULL) {
        printf("Failed to allocate 0x%x bytes\n", MAX_CHUNK);
        return (1);
    }

    if (file_is_stdio) {
        file = stdout;
    } else {
        char *filemode;
        if (writemode || !readmode)
            filemode = "r";
        else if (readmode && verifymode)
            filemode = "w+";
        else
            filemode = "w";

        file = fopen(filename, filemode);
        if (file == NULL) {
            printf("Failed to open \"%s\", for %s\n", filename,
                   readmode ? "write" : "read");
            rc = 1;
            goto fail_end;
        }

        if (writemode || verifymode) {
            rbuf = AllocVec(len, MEMF_PUBLIC);
            if (rbuf == NULL) {
                printf("Failed to allocate 0x%x bytes\n", MAX_CHUNK);
                rc = 1;
                goto fail_end;
            }
            bytes = fread(rbuf, 1, len, file);
            if (bytes < (int) len) {
                printf("\nFailed to read %u bytes from %s\n", len, filename);
                rc = 1;
                goto fail_end;
            }
        }
    }

    rc = 0;

    if (writemode && (addr == 0)) {
        /* Autoerase */
        rc = flash_erase(bank, addr, len, 1, flag_noremap);
        if (rc != 0)
            goto fail_end;
    }
    time_start = get_usec_time();

    start_bank = bank;
    start_addr = 0;
    start_len = len;

    dot_max = (len + MAX_CHUNK - 1) / MAX_CHUNK;
    while (dot_max > 50) {
        dot_max >>= 1;
        dot_iters <<= 1;
    }
    if (readmode || writemode) {
        uint offset = 0;
        dot_count = 0;
        if (!file_is_stdio) {
            if (readmode)
                printf("Read   [%*s]\rRead   [", dot_max, "");
            else
                printf("Write  [%*s]\rWrite  [", dot_max, "");
            fflush(stdout);
        }
        while (len > 0) {
            uint xlen = len;
            if (xlen > MAX_CHUNK)
                xlen = MAX_CHUNK;
            if (xlen > ROM_WINDOW_SIZE - addr) {
                xlen = ROM_WINDOW_SIZE - addr;
            }

            if (readmode) {
                /* Read from flash */
                rc = read_from_flash(bank, addr, buf, xlen);
                if (rc != 0) {
                    printf("\nFlash read failure (%s)\n", status_string(rc));
                    break;
                }
            }

            execute_swapmode(buf, xlen, SWAP_FROM_ROM, swapmode);

            if (writemode) {
                /* Write to flash */
                rc = write_to_flash(bank, addr, rbuf + offset, xlen);
                if (rc != 0) {
                    printf("\nFlash write failure (%s)\n", status_string(rc));
                    break;
                }
            } else {
                /* Output to file or stdout */
                if (file_is_stdio) {
                    dump_memory((uint32_t *)buf, xlen, addr);
                } else {
                    bytes = fwrite(buf, 1, xlen, file);
                    if (bytes < (int) xlen) {
                        printf("\nFailed to write all bytes to %s\n", filename);
                        rc = 1;
                        break;
                    }
                }
            }
            if ((!file_is_stdio) && (++dot_count == dot_iters)) {
                dot_count = 0;
                printf(".");
                fflush(stdout);
            }
            if (is_user_abort()) {
                rc = 2;
                goto fail_end;
            }

            len    -= xlen;
            addr   += xlen;
            offset += xlen;
            if (addr >= ROM_WINDOW_SIZE) {
                addr -= ROM_WINDOW_SIZE;
                bank++;
            }
        }
    }
    time_rw_end = get_usec_time();
    if (!file_is_stdio && (rc == 0) && (readmode || writemode)) {
        printf("]\n%s complete in ", writemode ? "Write" : "Read");
        print_us_diff(time_start, time_rw_end);
    }

    if (verifymode && (rc == 0)) {
        uint offset = 0;
        dot_count = 0;
        if (!file_is_stdio) {
            printf("Verify [%*s]\rVerify [", dot_max, "");
            fflush(stdout);
        }

        /* Restart positions */
        fseek(file, 0, SEEK_SET);
        bank = start_bank;
        addr = start_addr;
        len  = start_len;

        while (len > 0) {
            uint8_t *vbuf = rbuf + offset;
            uint xlen = len;
            if (xlen > MAX_CHUNK)
                xlen = MAX_CHUNK;
            if (xlen > ROM_WINDOW_SIZE - addr) {
                xlen = ROM_WINDOW_SIZE - addr;
            }

            /* Read from flash */
            rc = read_from_flash(bank, addr, buf, xlen);
            if (rc != 0) {
                printf("\nFlash read failure (%s)\n", status_string(rc));
                break;
            }
            execute_swapmode(buf, xlen, SWAP_FROM_ROM, swapmode);

            if (memcmp(buf, vbuf, xlen) != 0) {
                uint pos;
                uint32_t *buf1 = (uint32_t *) buf;
                uint32_t *buf2 = (uint32_t *) vbuf;
                printf("\nVerify failure at bank %x address %x\n", bank, addr);
                for (pos = 0; pos < xlen / 4; pos++) {
                    if (buf1[pos] != buf2[pos]) {
                        if (rc++ < 5) {
                            printf("    %05x: %08x != file %08x\n",
                                   addr + pos * 4, buf1[pos], buf2[pos]);
                        }
                    }
                }
                printf("    %u miscompares\n", rc);
                goto fail_end;
            }
            if (!file_is_stdio) {
                printf(".");
                fflush(stdout);
            }

            len    -= xlen;
            addr   += xlen;
            offset += xlen;
            if (addr >= ROM_WINDOW_SIZE) {
                addr -= ROM_WINDOW_SIZE;
                bank++;
            }
        }
    }
    if (!file_is_stdio && (rc == 0) && verifymode) {
        time_end = get_usec_time();
        fclose(file);
        if (verifymode) {
            printf("]\n%s complete in ", "Verify");
            print_us_diff(time_rw_end, time_end);
        }
    }
fail_end:
    FreeMem(buf, MAX_CHUNK);
    if (rbuf != NULL)
        FreeVec(rbuf);
    return (rc);
}

int
main(int argc, char *argv[])
{
    int      arg;
    uint     loop;
    uint     loops = 1;
    uint     flag_inquiry = 0;
    uint     flag_test = 0;
    uint     errs = 0;
    uint     do_multiple = 0;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds="
    SysBase = *(struct ExecBase **)4UL;
#pragma GCC diagnostic pop
    cpu_control_init();  // cpu_type, SysBase

    OpenTimer();
    for (arg = 1; arg < argc; arg++) {
        const char *ptr;
        ptr = long_to_short(argv[arg], long_to_short_main,
                            ARRAY_SIZE(long_to_short_main));
        if (*ptr == '-') {
            for (++ptr; *ptr != '\0'; ptr++) {
                switch (*ptr) {
                    case 'd':  // debug
                        flag_debug++;
                        break;
                    case 'e':  // erase flash
                        exit_cleanup(cmd_erase(argc - arg, argv + arg));
                        break;
                    case 'i':  // inquiry
                        flag_inquiry++;
                        break;
                    case 'l':  // loop count
                        if (++arg >= argc) {
                            printf("-%s requires an argument\n", ptr);
                            exit_cleanup(1);
                        }
                        loops = atoi(argv[arg]);
                        break;
                    case 'q':  // quiet
                        flag_quiet++;
                        break;
                    case 'r':  // read flash to file
                        exit_cleanup(cmd_readwrite(argc - arg, argv + arg));
                        break;
                    case 't':  // test id
                        flag_test++;
                        break;
                    case 'v':  // verify file with flash
                        exit_cleanup(cmd_readwrite(argc - arg, argv + arg));
                        break;
                    case 'w':  // write file to flash
                        exit_cleanup(cmd_readwrite(argc - arg, argv + arg));
                        break;
                    default:
                        printf("Unknown argument %s\n", ptr);
                        usage();
                        exit_cleanup(1);
                }
            }
        } else {
            printf("Error: unknown argument %s\n", ptr);
            usage();
            exit_cleanup(1);
        }
    }

    if (flag_inquiry + flag_test == 0) {
        printf("You must specify an operation to perform\n");
        usage();
        exit_cleanup(1);
    }
    if (flag_inquiry & (flag_inquiry - 1))
        do_multiple = 1;
    if (flag_inquiry + flag_test > 1)
        do_multiple = 1;

    for (loop = 0; loop < loops; loop++) {
        if (loops > 1) {
            if (flag_quiet) {
                if ((loop & 0xff) == 0) {
                    printf(".");
                    fflush(stdout);
                }
            } else {
                printf("Pass %-4u ", loop + 1);
                if (do_multiple)
                    printf("\n");
            }
        }
        if (flag_inquiry && flash_show_id() && ((loops == 1) || (loop > 1))) {
            errs++;
            break;
        }

        if (is_user_abort())
            goto end_now;

        if (flag_test && test_flash_id() && ((loops == 1) || (loop > 1))) {
            errs++;
            break;
        }

        if (is_user_abort())
            goto end_now;
    }
    if (loop < loops) {
        printf("Failed");
end_now:
        if (loops > 1)
            printf(" at pass %u", loop + 1);
        if (errs != 0)
            printf(" (%u errors)", errs);
        printf("\n");
    } else if (flag_quiet && (errs == 0)) {
        printf("Pass %u done\n", loop);
    }
    exit_cleanup(errs ? EXIT_FAILURE : EXIT_SUCCESS);
}
