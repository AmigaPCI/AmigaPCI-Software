/*
 * bec
 * ---
 * Utility to perform various operations with AmigaPCI STM32 Board Environment
 * Controller (BEC).
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
const char *version = "\0$VER: bec "VERSION" ("BUILD_DATE") \xA9 Chris Hooper";

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <fcntl.h>
#include <exec/types.h>
#include <dos/dos.h>
#include <clib/dos_protos.h>
#include <inline/timer.h>
#include <inline/exec.h>
#include <inline/dos.h>
#include <exec/execbase.h>
#include <exec/memory.h>
#include <devices/inputevent.h>
#include <devices/input.h>
#include "util.h"
#include "amiga_kbd_codes.h"
#include "../fw/bec_cmd.h"
#include "becmsg.h"
#include "crc32.h"

#ifndef ADDR8
#define ADDR8(x)    ((volatile uint8_t *)  ((uintptr_t)(x)))
#define ADDR16(x)   ((volatile uint16_t *) ((uintptr_t)(x)))
#define ADDR32(x)   ((volatile uint32_t *) ((uintptr_t)(x)))
#endif
#ifndef VADDR8
#define VADDR8(x)    ((volatile uint8_t *)  ((uintptr_t)(x)))
#define VADDR16(x)   ((volatile uint16_t *) ((uintptr_t)(x)))
#define VADDR32(x)   ((volatile uint32_t *) ((uintptr_t)(x)))
#endif

/*
 * gcc clib2 headers are bad (for example, no stdint definitions) and are
 * not being included by our build.  Because of that, we need to fix up
 * some stdio definitions.
 */
extern struct iob ** __iob;
#undef stdout
#define stdout ((FILE *)__iob[1])

struct ExecBase *DOSBase;
#if 0
struct Device   *TimerBase;
static struct    timerequest TimeRequest;
#endif

extern struct ExecBase *SysBase;

#define BIT(x) (1U << (x))
#define ARRAY_SIZE(x) ((sizeof (x) / sizeof ((x)[0])))

#define VALUE_UNASSIGNED 0xffffffff

#define TEST_LOOPBACK_BUF 4096
#define TEST_LOOPBACK_MAX 64
#define MEM_LOOPS         1000000
#define ROM_WINDOW_SIZE   (512 << 10)  // 512 KB
#define MAX_CHUNK         (16 << 10)   // 16 KB
#define FW_READ_CHUNK     (8 << 10)    // 8 KB
#define FW_READ_DOTS      32           // Width of fwread progress bar
#define FW_READ_RETRIES   5            // Attempts per fwread transfer
#define FW_WRITE_CHUNK    64           // Bytes per BEC_CMD_FW_WRITE
#define FW_WRITE_RETRIES  5            // Attempts per fwwrite transfer
#define FW_INFO_MAGIC     0x42454346   // "BECF" firmware info structure
#define FW_INFO_MIN_SIZE  20           // Minimum firmware info struct size
#define STM32_FLASH_BASE  0x08000000   // BEC flash address of firmware

#define swap32(x) __builtin_bswap32(x)

static const char cmd_options[] =
    "usage: bec <options>\n"
    "   debug         show debug output (-d)\n"
    "   fwread <fn>   read BEC firmware to file (-R)\n"
    "   fwverify <fn> verify BEC firmware matches file (-V)\n"
    "   fwwrite <fn>  write BEC firmware from file (-W)\n"
    "   identify      identify Board Environment Controller (BEC)\n"
    "   loop <num>    repeat the command a specified number of times (-l)\n"
    "   quiet         minimize test output\n"
    "   set <n> <v>   set BEC value <n>=\"name\" and <v> is string (-s)\n"
    "   term          open BEC firmware terminal [-T]\n"
    "   test[0123]    do interface test (-t)\n";

typedef struct {
    const char *const short_name;
    const char *const long_name;
} long_to_short_t;
long_to_short_t long_to_short_main[] = {
    { "-d", "debug" },
    { "-i", "inquiry" },
    { "-i", "identify" },
    { "-i", "id" },
    { "-l", "loop" },
    { "-q", "quiet" },
    { "-R", "fwread" },
    { "-s", "set" },
    { "-t", "test" },
    { "-T", "term" },
    { "-V", "fwverify" },
    { "-W", "fwwrite" },
    { "-z", "z" },
};

BOOL __check_abort_enabled = 0;       // Disable gcc clib2 ^C break handling
uint flag_debug = 0;
uint8_t flag_quiet = 0;
uint8_t *test_loopback_buf = NULL;

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

char *
ull_to_str(unsigned long long val, char *buf, int bufsize)
{
    char *ptr = buf + bufsize;

    if (ptr == buf)
        return (NULL);  // no space
    *(--ptr) = '\0';

    while (val != 0) {
        if (ptr == buf)
            return (NULL);  // overflow
        *(--ptr) = (val % 10) + '0';
        val /= 10;

    }
    return (ptr);
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

/*
 * rand32
 * ------
 * Very simple pseudo-random number generator
 */
static uint32_t rand_seed = 0;
static uint32_t
rand32(void)
{
    rand_seed = (rand_seed * 25173) + 13849;
    return (rand_seed);
}

/*
 * srand32
 * -------
 * Very simple random number seed
 */
static void
srand32(uint32_t seed)
{
    rand_seed = seed;
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

/* RTC offsets for Ricoh RP5C01 in AmigaPCI */
#define RP_ONE_SEC   (0x0 * 4 + 1)  // M0 Second One's
#define RP_TEN_SEC   (0x1 * 4 + 1)  // M0 Second Ten's
#define RP_ONE_MIN   (0x2 * 4 + 1)  // M0 Minute One's
#define RP_TEN_MIN   (0x3 * 4 + 1)  // M0 Minute Ten's
#define RP_ONE_HOUR  (0x4 * 4 + 1)  // M0 Hour One's
#define RP_TEN_HOUR  (0x5 * 4 + 1)  // M0 Hour Ten's
#define RP_DOW       (0x6 * 4 + 1)  // M0 Day of Week
#define RP_ONE_DAY   (0x7 * 4 + 1)  // M0 Day of Month One's
#define RP_TEN_DAY   (0x8 * 4 + 1)  // M0 Day of Month Ten's
#define RP_ONE_MONTH (0x9 * 4 + 1)  // M0 Month of Year One's
#define RP_TEN_MONTH (0xa * 4 + 1)  // M0 Month of Year Ten's
#define RP_ONE_YEAR  (0xb * 4 + 1)  // M0 Year of Century One's
#define RP_TEN_YEAR  (0xc * 4 + 1)  // M0 Year of Century Ten's
#define RP_MODE      (0xd * 4 + 1)  // M0 Mode register
#define RP_TEST      (0xe * 4 + 1)  // M0 Test register
#define RP_RESET     (0xf * 4 + 1)  // M0 Reset controller, etc
#define RP_MAGIC     (0x9 * 4 + 1)  // M1 AmigaPCI magic register
#define RP_MAGIC_HI  (0x0 * 4 + 1)  // M1 AmigaPCI magic register hi
#define RP_MAGIC_LO  (0x1 * 4 + 1)  // M1 AmigaPCI magic register lo
#define RP_M1_12_24  (0xa * 4 + 1)  // M1 12/24 Hour select (0=AM, 1=24, 2=PM)
#define RP_M1_LEAP   (0xb * 4 + 1)  // M1 Leap Year Counter (increments w/ year

#define RP_MODE_M0         0      /* Mode 0 */
#define RP_MODE_M1         1      /* Mode 1 */
#define RP_MODE_BLK10      2      /* RAM Block 10 */
#define RP_MODE_BLK11      3      /* RAM Block 11 */
#define RP_MODE_ALARM_EN   BIT(2) /* Alarm Enable */
#define RP_MODE_TIMER_EN   BIT(3) /* Timer Enable */
#define RP_RESET_TIMER_ALL BIT(0) /* Reset all alarm registers */
#define RP_RESET_TIMER_SEC BIT(1) /* Reset divider stage for sec and smaller */
#define RP_RESET_16HZ_OFF  BIT(2) /* Turn off 16Hz clock pulse */
#define RP_RESET_1HZ_OFF   BIT(3) /* Turn off 1Hz clock pulse */

#define RTC_REG(x) (*((volatile uint8_t *) 0xdc0000 + x))

#define INTERRUPTS_DISABLE() if (irq_disabled++ == 0) \
                                 Disable()  /* Disable interrupts */
#define INTERRUPTS_ENABLE()  if (--irq_disabled == 0) \
                                 Enable()   /* Enable Interrupts */
unsigned int irq_disabled;


#define DUMP_VALUE_UNASSIGNED 0xffffffff

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
    uint8_t *src = buf;

    if (len > 0x100) {
        printf("Bad len %x\n", len);
        len = 0x100;
    }
    if (dump_base != DUMP_VALUE_UNASSIGNED)
        printf("%05x:", dump_base);
    for (pos = 0; pos < len; pos++) {
        printf(" %02x", src[pos]);

        if ((pos & 0xf) == 0xf)
            printf("\n");
    }
    if ((pos & 0xf) != 0x0)
        printf("\n");
}

static uint64_t
bec_time(void)
{
    uint64_t usecs;
    if (send_cmd_retry(BEC_CMD_UPTIME, NULL, 0, &usecs, sizeof (usecs), NULL))
        return (0);
    return (usecs);
}

static uint
bec_identify(void)
{
    uint64_t   usecs;
    uint       sec;
    uint       usec;
    bec_id_t id;
    uint       rlen;
    uint       rc;

    memset(&id, 0, sizeof (id));
    rc = send_cmd_retry(BEC_CMD_ID, NULL, 0, &id, sizeof (id), &rlen);

    if (rc != 0) {
        printf("Reply message failure: (%s)\n", bec_err(rc));
        if (flag_debug) {
            if (rlen > 256)
                rlen = 256;
            dump_memory(&id, rlen, DUMP_VALUE_UNASSIGNED);
        }
        return (rc);
    }
    if (flag_quiet == 0) {
        char board_type[32];
        switch (id.bid_board_type) {
            case 0:
                strcpy(board_type, "Unknown");
                break;
            case 1:
                strcpy(board_type, "AmigaPCI");
                break;
            case 2:
                strcpy(board_type, "APCIDev");
                break;
            case 3:
                strcpy(board_type, "KeyJAM");
                break;
            default:
                sprintf(board_type, "Unknown Board (%u)", id.bid_board_type);
                break;
        }
        printf("ID\n");
        printf("  BEC %u.%u built %02u%02u-%02u-%02u %02u:%02u:%02u\n",
               id.bid_version[0], id.bid_version[1],
               id.bid_date[0], id.bid_date[1],
               id.bid_date[2], id.bid_date[3],
               id.bid_time[0], id.bid_time[1], id.bid_time[2]);
        printf("  %s Rev %u  Size %u  Serial \"%s\"  Name \"%s\"\n",
               board_type, id.bid_board_rev, id.bid_size,
               id.bid_serial, id.bid_name);
    }

    usecs = bec_time();
    if (flag_quiet == 0) {
        if (usecs != 0) {
            sec  = usecs / 1000000;
            usec = usecs % 1000000;
            printf("  Uptime: %u.%06u sec\n", sec, usec);
        }
    }

#if 0
    uint8_t nvdata[32];
    uint16_t what = 0x0020;  // Get all 32 bytes of NV data
    rc = send_cmd(BEC_CMD_GET | BEC_GET_NV, &what, sizeof (what),
                   nvdata, sizeof (nvdata), &rlen);
    printf("  NV Data:");
    if (rc == 0) {
        uint pos;
        for (pos = 0; pos < sizeof (nvdata); pos++) {
            if (pos == 16)
                printf("\n          ");
            printf(" %02x", nvdata[pos]);
        }
        printf("\n");
    } else {
        if (rc == KS_STATUS_UNKCMD)
            printf(" Unavailable\n");
        else
            printf(" Fail (%s)\n", bec_err(rc));
    }
#endif

    return (0);
}

static const uint8_t test_pattern[] = {
    0xaa, 0x55, 0xcc, 0x33,
    0xee, 0x11, 0xff, 0x00,
    0x01, 0x02, 0x04, 0x08,
    0x10, 0x20, 0x40, 0x80,
    0xfe, 0xfd, 0xfb, 0xf7,
    0xef, 0xdf, 0xbf, 0x7f,
};

static int
bec_test_pattern(void)
{
    uint8_t reply_buf[64];
    uint start;
    uint pos;
    uint err_count = 0;
    uint rlen = 0;
    uint rc;

    show_test_state("Test pattern", -1);

    memset(reply_buf, 0, sizeof (reply_buf));
    rc = send_cmd(BEC_CMD_TESTPATT, NULL, 0, reply_buf,
                  sizeof (reply_buf), &rlen);
    if (rc != 0) {
        printf("Reply message failure: (%s)\n", bec_err(rc));
        if (flag_debug) {
            if (rlen > 256)
                rlen = 256;
            dump_memory(reply_buf, rlen, DUMP_VALUE_UNASSIGNED);
        }
        show_test_state("Test pattern", rc);
        return (rc);
    }

    for (start = 0; start < ARRAY_SIZE(reply_buf); start++)
        if (reply_buf[start] == test_pattern[0])  // Found first data
            break;

    if (start == ARRAY_SIZE(reply_buf)) {
        printf("No test pattern marker [%08x] in reply\n", test_pattern[0]);
fail:
        dump_memory(reply_buf, sizeof (reply_buf), DUMP_VALUE_UNASSIGNED);
        show_test_state("Test pattern", 1);
        return (1);
    }

    if (start != 0) {
        printf("Pattern start 0x%x is not at beginning of buffer\n",
               start);
        err_count++;
    }
    for (pos = 0; pos < ARRAY_SIZE(test_pattern); pos++) {
        if (reply_buf[start + pos] != test_pattern[pos]) {
            printf("At pos=%x reply %08x != expected %08x (diff %08x)\n", pos,
                   reply_buf[start + pos], test_pattern[pos],
                   reply_buf[start + pos] ^ test_pattern[pos]);
            if (err_count > 6)
                goto fail;
        }
    }
    if (err_count > 6)
        goto fail;
    if (flag_debug > 1)
        dump_memory(reply_buf, sizeof (reply_buf), DUMP_VALUE_UNASSIGNED);
    show_test_state("Test pattern", 0);
    return (0);
}

static int
bec_test_loopback(void)
{
    uint8_t   *tx_buf;
    uint8_t   *rx_buf;
    uint       cur;
    uint       rc;
    uint       count;
    uint       rlen = 0;
    uint       nums = (rand32() % (TEST_LOOPBACK_MAX - 1)) + 1;
    uint64_t   time_start;
    uint64_t   time_end;

    show_test_state("Test loopback", -1);

    tx_buf = &test_loopback_buf[0];
    rx_buf = &test_loopback_buf[TEST_LOOPBACK_BUF];
    memset(rx_buf, 0, TEST_LOOPBACK_MAX * 4);
    for (cur = 0; cur < nums; cur++)
        tx_buf[cur] = (uint8_t) (rand32() >> 8);

    /* Measure IOPS */
    INTERRUPTS_DISABLE();
    time_start = bec_time();
    for (count = 0; count < 10; count++) {
        rc = send_cmd(BEC_CMD_LOOPBACK, tx_buf, 4,
                      rx_buf, TEST_LOOPBACK_MAX, &rlen);
        if ((rc != BEC_CMD_LOOPBACK))
            break;
    }
    time_end = bec_time();
    INTERRUPTS_ENABLE();

    if (rc == BEC_CMD_LOOPBACK) {
        /* Test loopback accuracy */
        rc = send_cmd(BEC_CMD_LOOPBACK, tx_buf, nums,
                      rx_buf, TEST_LOOPBACK_MAX, &rlen);
    }
    if (rc == BEC_CMD_LOOPBACK) {
        rc = 0;
    } else {
        printf("FAIL: (%s)\n", bec_err(rc));
fail_handle_debug:
        if (flag_debug) {
            uint pos;
            uint8_t *txp = (uint8_t *)tx_buf;
            uint8_t *rxp = (uint8_t *)rx_buf;
            for (pos = 0; pos < nums; pos++)
                if (rxp[pos] != txp[pos])
                    break;
            dump_memory(tx_buf, nums, DUMP_VALUE_UNASSIGNED);
            if (pos < nums) {
                printf("--- Tx (0x%x) above; Rx (0x%x) below; "
                       "first diff at 0x%x ---\n",
                       nums, rlen, pos);
                dump_memory(rx_buf, rlen, DUMP_VALUE_UNASSIGNED);
            } else {
                printf("Tx and Rx buffers (len=0x%x) match\n", nums);
            }
        }
        goto fail_loopback;
    }
    if (rlen != nums) {
        printf("FAIL: rlen=%u != sent %u\n", rlen, nums);
        rc = BEC_STATUS_REPLYLEN;
        goto fail_handle_debug;
    }
    for (cur = 0; cur < nums; cur++) {
        if (rx_buf[cur] != tx_buf[cur]) {
            if (rc++ == 0)
                printf("\nLoopback data miscompare\n");
            if (rc < 5) {
                printf("    [%02x] %02x != expected %02x\n",
                       cur, rx_buf[cur], tx_buf[cur]);
            }
        }
    }
    if (rc >= 4)
        printf("%u miscompares\n", rc);
    if ((rc == 0) && (flag_quiet == 0)) {
        uint diff = (uint) (time_end - time_start);
        if (diff == 0)
            diff = 1;
        printf("PASS  %u IOPS\n", 1000000 * count / diff);
        return (0);
    }
fail_loopback:
    show_test_state("Test loopback", rc);
    return (rc);
}

static int
bec_test_loopback_perf(void)
{
    const uint lb_size  = 220;
    const uint xfers    = 20;
    const uint lb_alloc = lb_size + BEC_MSG_HDR_LEN + BEC_MSG_CRC_LEN;
    uint8_t   *txbuf;
    uint8_t   *rxbuf;
    uint       cur;
    uint       rc;
    uint       diff;
    uint       total;
    uint       perf;
    uint64_t   time_start;
    uint64_t   time_end;

    show_test_state("Loopback perf", -1);

    txbuf = AllocMem(lb_alloc, MEMF_PUBLIC);
    if (txbuf == NULL) {
        printf("Memory allocation failure\n");
        return (1);
    }
    rxbuf = AllocMem(lb_alloc, MEMF_PUBLIC);
    if (rxbuf == NULL) {
        printf("Memory allocation failure\n");
        rc = 1;
        goto cleanup;
    }
    memset(rxbuf, 0x00, lb_size);
    for (cur = 0; cur < lb_size; cur++)
        txbuf[cur] = cur + 4;

    time_start = bec_time();

    for (cur = 0; cur < xfers; cur++) {
        rc = send_cmd(BEC_CMD_LOOPBACK, txbuf, lb_size, rxbuf, lb_size, NULL);
        if (rc == BEC_CMD_LOOPBACK) {
            rc = 0;
        } else {
            printf("FAIL: (%s) at pass %u\n", bec_err(rc), cur);
            if (flag_debug) {
                dump_memory(rxbuf, lb_size, DUMP_VALUE_UNASSIGNED);
            }
            goto cleanup;
        }
    }

    if (flag_quiet == 0) {
        time_end = bec_time();
        diff = (uint) (time_end - time_start);
        if (diff == 0)
            diff = 1;
        total = xfers * (lb_size + BEC_MSG_HDR_LEN + BEC_MSG_CRC_LEN);
        perf = total * 1000 / diff;
        perf *= 2;  // Write data + Read (reply) data
        printf("PASS  %u KB/sec\n", perf);
    }

cleanup:
    if (txbuf != NULL)
        FreeMem(txbuf, lb_alloc);
    if (rxbuf != NULL)
        FreeMem(rxbuf, lb_alloc);
    return (rc);
}

static uint
calc_kb_sec(uint usecs, uint bytes)
{
    if (usecs == 0)
        usecs = 1;
    while (bytes > 4000000) {
        bytes >>= 1;
        usecs >>= 1;
    }
    return (bytes * 1000 / usecs);
}

static int
bec_test_commands(void)
{
    int errs = 0;
    int rc;
    int rc2;
    uint rlen;
    __attribute__((aligned(4))) uint8_t buf[256];

    show_test_state("Commands", -1);

    /* BEC_CMD_NOP */
    rc = send_cmd(BEC_CMD_NOP, NULL, 0, NULL, 0, NULL);
    if (rc != 0) {
        printf("FAIL: NOP (%s)\n", bec_err(rc));
        errs++;
        goto test_commands_fail;
    }

    /* BEC_CMD_ID */
    bec_id_t id;
    rc = send_cmd(BEC_CMD_ID, NULL, 0, &id, sizeof (id), &rlen);
    if ((rc != 0) || (id.bid_rev == 0x0000) || (id.bid_rev > 0x0002)) {
        printf("FAIL: ID Rev=%04x (%s)\n", id.bid_rev, bec_err(rc));
        errs++;
    }

    /* BEC_CMD_UPTIME */
    uint64_t usecs1;
    uint64_t usecs2;
    rc = send_cmd(BEC_CMD_UPTIME, NULL, 0, &usecs1, sizeof (usecs1), NULL);
    Delay(1);
    rc2 = send_cmd(BEC_CMD_UPTIME, NULL, 0, &usecs2, sizeof (usecs2), NULL);

    if ((rc != 0) || (rc2 != 0)) {
        printf("FAIL: UPTIME (%s) (%s)\n", bec_err(rc), bec_err(rc2));
        errs++;
        goto test_commands_fail;

    } else {
        uint64_t diff = usecs2 - usecs1;
        if ((diff < 19500) || (diff > 200000)) {
            printf("FAIL: UPTIME not accurate: 20000 expected, but got %s: ",
                   ull_to_str(diff, buf, sizeof (buf)));
            print_us_diff(usecs1, usecs2);
            errs++;
        }
    }

#if 0
    /* BEC_CMD_GET */
    uint count;
    uint8_t nvdata1[32];
    uint8_t nvdata2[32];
    uint16_t what = 0x0010;  // Get 16 bytes of NV data
    rc = send_cmd(BEC_CMD_GET | KS_GET_NV, &what, sizeof (what),
                  nvdata1, sizeof (nvdata1), &rlen);
    if (rc != 0) {
fail_get_nv:
        printf("FAIL: GET NV (%s)\n", bec_err(rc));
        errs++;
    } else {
        for (count = 0; count < 10; count++) {
            rc = send_cmd(BEC_CMD_GET | KS_GET_NV, &what, sizeof (what),
                          nvdata2, sizeof (nvdata2), &rlen);
            if (rc != 0)
                goto fail_get_nv;
            if (memcmp(nvdata1, nvdata2, what) != 0) {
                printf("FAIL: GET NV miscompare:\n");
                printf("  Pass 1: ");
                dump_memory(nvdata1, what, DUMP_VALUE_UNASSIGNED);
                printf("  Pass 2: ");
                dump_memory(nvdata2, what, DUMP_VALUE_UNASSIGNED);
                errs++;
                break;
            }
        }
    }
#endif

    /* BEC_CMD_CONS_OUTPUT */
    uint16_t maxlen = 0;
    buf[0] = 0xa5;
    buf[1] = 0x5a;
    rc = send_cmd(BEC_CMD_CONS_OUTPUT, &maxlen, sizeof (maxlen),
                  buf, sizeof (buf), &rlen);
    if (rc != 0) {
        printf("FAIL: CONS_OUTPUT (%s)\n", bec_err(rc));
        errs++;
    } else if ((rlen != 0) || (buf[0] != 0xa5) || (buf[1] != 0x5a)) {
        printf("FAIL: CONS_OUTPUT rlen=%u buf=%02x %02x\n",
               rlen, buf[0], buf[1]);
        errs++;
    }

test_commands_fail:
    show_test_state("Commands", errs);

    return (errs);
}


static int
bec_test(uint mask)
{
    int rc = 0;
    if (mask & BIT(0)) {
        rc = bec_test_pattern();
        if (rc != 0)
            return (rc);
    }

    if (is_user_abort())
        return (1);

    if (mask & BIT(1)) {
        rc = bec_test_loopback();
        if (rc != 0)
            return (rc);
    }

    if (is_user_abort())
        return (1);

    if (mask & BIT(2)) {
        rc = bec_test_loopback_perf();
        if (rc != 0)
            return (rc);
    }

    if (is_user_abort())
        return (1);

    if (mask & BIT(3)) {
        rc = bec_test_commands();
        if (rc != 0)
            return (rc);
    }

    return (0);
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

#if 0
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
#endif

/*
 * cmd_set
 * -------
 * Set a KickSmash value, such as board name, or NVRAM value available to
 * AmigaOS.
 */
int
cmd_set(int argc, char *argv[])
{
    int rc;
    int arg;

    if (argc < 2) {
cmd_set_usage:
        printf("set name <string>    - up to 15 characters for board name\n"
               "set nv<n> <value...> - set non-volatile data\n"
               "    nv0 is switcher timeout and nv1 is ROM bank to use\n");
        return (1);
    }
    if (strcmp(argv[1], "name") == 0) {
        char buf[30];
        int len = 0;
        int maxlen;
        uint bufpos = 5;
#define BOARD_NAME_MAX 15
        strcpy(buf, "name");
        for (arg = 2; arg < argc; arg++) {
            int clen = strlen(argv[arg]);
            if (len > 0)
                buf[bufpos + len++] = ' ';

            if (len + clen >= BOARD_NAME_MAX) {
                printf("Name too long: \"");
                for (arg = 2; arg < argc; arg++) {
                    if (arg != 2)
                        printf(" ");
                    printf("%s", argv[arg]);
                }
                printf("\"\n");
                return (1);
            }
            maxlen = BOARD_NAME_MAX - len;
            strncpy(buf + bufpos + len, argv[arg], maxlen);
            len += clen;
        }
        buf[bufpos + len] = '\0';
        rc = send_cmd(BEC_CMD_SET, buf, bufpos + len + 1, NULL, 0, NULL);
        if (rc != 0) {
            printf("Failed to set board name: (%s)\n", bec_err(rc));
        } else {
            printf("Set board name to \"%s\"\n", buf);
        }
        return (rc);
    } else {
        printf("Unknown set %s\n", argv[1]);
        goto cmd_set_usage;
    }
    return (0);
}

/*
 * fw_read_chunk
 * -------------
 * Read up to FW_READ_CHUNK bytes of BEC firmware at the specified offset
 * using BEC_CMD_FW_READ. The reply header is placed at the start of rbuf,
 * which must be sizeof (bec_fw_rw_t) + FW_READ_CHUNK bytes, followed by
 * the data. The BEC may return less data than requested.
 *
 * Returns the number of bytes read, or 0 on failure.
 */
static uint
fw_read_chunk(uint offset, uint fw_size, uint8_t *rbuf, uint *retries)
{
    bec_fw_rw_t  req;
    bec_fw_rw_t *rhdr = (bec_fw_rw_t *) rbuf;
    uint         rbuf_size = sizeof (bec_fw_rw_t) + FW_READ_CHUNK;
    uint         rlen;
    uint         len;
    uint         rc;
    uint         tries;

    memset(&req, 0, sizeof (req));
    req.bfrw_offset = offset;
    len = fw_size - offset;
    if (len > FW_READ_CHUNK)
        len = FW_READ_CHUNK;
    req.bfrw_size = len;

    /* Reads have no side effects, so retry any failure */
    for (tries = 0; tries < FW_READ_RETRIES; tries++) {
        rc = send_cmd_retry(BEC_CMD_FW_READ, &req, sizeof (req),
                            rbuf, rbuf_size, &rlen);
        if (rc == 0)
            break;
        (*retries)++;
        if (flag_debug)
            printf("\nFW read at offset 0x%x retry: (%s)\n",
                   offset, bec_err(rc));
    }
    if (rc != 0) {
        printf("\nFW read at offset 0x%x failed: (%s)\n",
               offset, bec_err(rc));
        return (0);
    }
    len = rhdr->bfrw_size;
    if ((rlen < sizeof (*rhdr)) || (len == 0) ||
        (len > rlen - sizeof (*rhdr)) || (len > fw_size - offset) ||
        (rhdr->bfrw_offset != offset)) {
        printf("\nFW read at offset 0x%x bad reply: "
               "rlen=%u offset=0x%x size=%u\n",
               offset, rlen, rhdr->bfrw_offset, len);
        return (0);
    }
    return (len);
}

/*
 * cmd_fwread
 * ----------
 * Read the running BEC firmware image and write it to the specified file.
 * The firmware size is acquired by BEC_CMD_ID. The BEC may return less
 * data per BEC_CMD_FW_READ than requested.
 */
static int
cmd_fwread(int argc, char *argv[])
{
    const char  *filename;
    bec_id_t     id;
    uint8_t     *rbuf;
    uint         offset;
    uint         fw_size;
    uint         rbuf_size = sizeof (bec_fw_rw_t) + FW_READ_CHUNK;
    uint         rlen;
    uint         len;
    uint         rc;
    uint         retries = 0;
    uint         dots = 0;
    uint         dots_now;
    uint64_t     time_start;
    uint64_t     time_end;
    int          errs = 0;
    BPTR         lock;
    FILE        *fp = NULL;

    if (argc < 2) {
        printf("fwread requires a filename argument\n");
        return (1);
    }
    filename = argv[1];

    lock = Lock(filename, ACCESS_READ);
    if (lock != 0L) {
        char prompt[80];
        UnLock(lock);
        snprintf(prompt, sizeof (prompt), "Overwrite existing %s", filename);
        if (!are_you_sure(prompt))
            return (1);
    }

    memset(&id, 0, sizeof (id));
    rc = send_cmd_retry(BEC_CMD_ID, NULL, 0, &id, sizeof (id), &rlen);
    if (rc != 0) {
        printf("BEC ID failure: (%s)\n", bec_err(rc));
        return (rc);
    }
    fw_size = id.bid_size;
    if (fw_size == 0) {
        printf("BEC reported firmware size 0 (firmware may be too old)\n");
        return (1);
    }

    rbuf = AllocMem(rbuf_size, MEMF_PUBLIC);
    if (rbuf == NULL) {
        printf("Failed to allocate %u bytes\n", rbuf_size);
        return (1);
    }

    if (flag_quiet == 0) {
        printf("Reading [%*s]\rReading [", FW_READ_DOTS, "");
        fflush(stdout);
    }
    time_start = bec_time();

    for (offset = 0; offset < fw_size; offset += len) {
        if (is_user_abort()) {
            errs++;
            break;
        }
        len = fw_read_chunk(offset, fw_size, rbuf, &retries);
        if (len == 0) {
            errs++;
            break;
        }
        /* Don't create the file until BEC has provided data */
        if (fp == NULL) {
            fp = fopen(filename, "wb");
            if (fp == NULL) {
                printf("\nFailed to open %s for write\n", filename);
                errs++;
                break;
            }
        }
        if (fwrite(rbuf + sizeof (bec_fw_rw_t), 1, len, fp) != len) {
            printf("\nWrite to %s failed\n", filename);
            errs++;
            break;
        }
        dots_now = (offset + len) * FW_READ_DOTS / fw_size;
        if ((flag_quiet == 0) && (dots_now > dots)) {
            for (; dots < dots_now; dots++)
                printf(".");
            fflush(stdout);
        }
    }
    time_end = bec_time();
    if (flag_quiet == 0) {
        if (errs == 0)
            printf("]");
        printf("\n");
    }

    if ((fp != NULL) && (fclose(fp) != 0)) {
        printf("Close of %s failed\n", filename);
        errs++;
    }
    FreeMem(rbuf, rbuf_size);

    if (errs == 0) {
        printf("Read %u bytes of firmware to %s (%u KB/sec) in ",
               fw_size, filename,
               calc_kb_sec((uint) (time_end - time_start), fw_size));
        print_us_diff(time_start, time_end);
    }
    if (retries != 0)
        printf("%u read retries were required\n", retries);
    return (errs ? 1 : 0);
}

/*
 * fw_file_identify
 * ----------------
 * Locate the firmware info structure (FW_INFO_MAGIC) in a firmware image
 * read from disk, then fill in the version, build date/time, and size
 * fields of the supplied bec_id_t from it, as BEC_CMD_ID would report
 * for that firmware.
 *
 * Returns 0 on success, 1 if the image does not appear to be valid.
 */
static int
fw_file_identify(const uint8_t *buf, uint buf_size, bec_id_t *id)
{
    uint        off;
    uint        vptr_off;
    uint        str_off;
    uint        ver[2];
    uint        date[3];
    uint        time[3];
    uint32_t    fi_struct_size;
    uint32_t    fi_fw_size;
    const char *str;
    const char *built;

    for (off = 0; off + FW_INFO_MIN_SIZE <= buf_size; off += 4) {
        if (swap32(*ADDR32(buf + off)) != FW_INFO_MAGIC)
            continue;
        fi_struct_size = swap32(*ADDR32(buf + off + 4));
        fi_fw_size     = swap32(*ADDR32(buf + off + 16));
        if ((swap32(*ADDR32(buf + off + 8)) == STM32_FLASH_BASE + off) &&
            (fi_struct_size >= FW_INFO_MIN_SIZE) &&
            (fi_fw_size <= buf_size) &&
            (off + fi_struct_size <= fi_fw_size)) {
            break;
        }
    }
    if (off + FW_INFO_MIN_SIZE > buf_size) {
        printf("Firmware info not found in file\n");
        return (1);
    }

    /* fi_version_str is the flash address of a pointer to version_str */
    vptr_off = swap32(*ADDR32(buf + off + 12)) - STM32_FLASH_BASE;
    if ((vptr_off > buf_size - 4) ||
        ((str_off = swap32(*ADDR32(buf + vptr_off)) - STM32_FLASH_BASE) >=
         buf_size) ||
        (memchr(buf + str_off, '\0', buf_size - str_off) == NULL)) {
        printf("Firmware version string not found in file\n");
        return (1);
    }

    /* Format: "Version 1.0+ built YYYY-MM-DD HH:MM:SS" */
    str = (const char *) buf + str_off;
    built = strstr(str, " built ");
    if ((sscanf(str, "Version %u.%u", &ver[0], &ver[1]) != 2) ||
        (built == NULL) ||
        (sscanf(built, " built %u-%u-%u %u:%u:%u", &date[0], &date[1],
                &date[2], &time[0], &time[1], &time[2]) != 6)) {
        printf("Could not parse firmware version \"%s\"\n", str);
        return (1);
    }

    memset(id, 0, sizeof (*id));
    id->bid_version[0] = ver[0];
    id->bid_version[1] = ver[1];
    id->bid_date[0]    = date[0] / 100;
    id->bid_date[1]    = date[0] % 100;
    id->bid_date[2]    = date[1];
    id->bid_date[3]    = date[2];
    id->bid_time[0]    = time[0];
    id->bid_time[1]    = time[1];
    id->bid_time[2]    = time[2];
    id->bid_size       = fi_fw_size;
    return (0);
}

/*
 * fw_id_show
 * ----------
 * Display firmware version, build date/time, and size from a bec_id_t.
 */
static void
fw_id_show(const char *what, const bec_id_t *id)
{
    printf("%-8s BEC %u.%u built %02u%02u-%02u-%02u %02u:%02u:%02u  "
           "Size %u\n", what,
           id->bid_version[0], id->bid_version[1],
           id->bid_date[0], id->bid_date[1], id->bid_date[2], id->bid_date[3],
           id->bid_time[0], id->bid_time[1], id->bid_time[2], id->bid_size);
}

/*
 * fw_id_match
 * -----------
 * Returns TRUE if the firmware identity fields of two bec_id_t match.
 */
static int
fw_id_match(const bec_id_t *id1, const bec_id_t *id2)
{
    return ((memcmp(id1->bid_version, id2->bid_version,
                    sizeof (id1->bid_version)) == 0) &&
            (memcmp(id1->bid_date, id2->bid_date,
                    sizeof (id1->bid_date)) == 0) &&
            (memcmp(id1->bid_time, id2->bid_time, 3) == 0) &&
            (id1->bid_size == id2->bid_size));
}

/*
 * fw_file_load
 * ------------
 * Read a firmware image file into memory allocated with AllocMem() and
 * identify it with fw_file_identify(). The caller must FreeMem() the
 * buffer when done.
 *
 * Returns 0 on success, 1 on failure (no buffer is then allocated).
 */
static int
fw_file_load(const char *filename, uint8_t **fbufp, uint *fbuf_sizep,
             bec_id_t *file_id)
{
    uint8_t *fbuf;
    uint     fbuf_size;
    long     flen;
    FILE    *fp;

    fp = fopen(filename, "rb");
    if (fp == NULL) {
        printf("Failed to open %s\n", filename);
        return (1);
    }
    if ((fseek(fp, 0, SEEK_END) != 0) || ((flen = ftell(fp)) <= 0) ||
        (fseek(fp, 0, SEEK_SET) != 0)) {
        printf("Failed to get size of %s\n", filename);
        fclose(fp);
        return (1);
    }
    fbuf_size = flen;
    fbuf = AllocMem(fbuf_size, MEMF_PUBLIC);
    if (fbuf == NULL) {
        printf("Failed to allocate %u bytes\n", fbuf_size);
        fclose(fp);
        return (1);
    }
    if (fread(fbuf, 1, fbuf_size, fp) != fbuf_size) {
        printf("Failed to read %s\n", filename);
        fclose(fp);
        FreeMem(fbuf, fbuf_size);
        return (1);
    }
    fclose(fp);

    if (fw_file_identify(fbuf, fbuf_size, file_id) != 0) {
        FreeMem(fbuf, fbuf_size);
        return (1);
    }
    if (file_id->bid_size != fbuf_size) {
        printf("Note: %s is %u bytes; firmware image is %u bytes\n",
               filename, fbuf_size, file_id->bid_size);
    }
    *fbufp = fbuf;
    *fbuf_sizep = fbuf_size;
    return (0);
}

/*
 * cmd_fwwrite
 * -----------
 * Write a firmware image file to the BEC. The BEC firmware updater is
 * started with BEC_CMD_FW_UNLOCK, the image is sent with BEC_CMD_FW_WRITE,
 * and BEC_CMD_FW_LOCK restarts the BEC in the new firmware, which is
 * then verified against the file with BEC_CMD_ID.
 */
static int
cmd_fwwrite(int argc, char *argv[])
{
    const char  *filename;
    bec_id_t     id;
    bec_id_t     file_id;
    bec_fw_rw_t  rhdr;
    bec_fw_rw_t *req;
    uint8_t     *fbuf = NULL;
    uint8_t      wbuf[sizeof (bec_fw_rw_t) + FW_WRITE_CHUNK];
    uint         fbuf_size = 0;
    uint         offset;
    uint         rlen;
    uint         len;
    uint         rc;
    uint         tries;
    uint         retries = 0;
    uint         dots = 0;
    uint         dots_now;
    uint64_t     time_start;
    uint64_t     time_end;
    int          errs = 0;
    int          in_updater;

    if (argc < 2) {
        printf("fwwrite requires a filename argument\n");
        return (1);
    }
    filename = argv[1];

    if (fw_file_load(filename, &fbuf, &fbuf_size, &file_id) != 0)
        return (1);

    /* Identify the running firmware */
    memset(&id, 0, sizeof (id));
    rc = send_cmd_retry(BEC_CMD_ID, NULL, 0, &id, sizeof (id), &rlen);
    if (rc != 0) {
        printf("BEC ID failure: (%s)\n", bec_err(rc));
        errs++;
        goto fwwrite_end;
    }
    in_updater = (strcmp(id.bid_serial, "Updater") == 0);
    fw_id_show(in_updater ? "Updater" : "Running", &id);
    fw_id_show("File", &file_id);

    if (!are_you_sure("Write BEC firmware")) {
        errs++;
        goto fwwrite_end;
    }

    /* Start the firmware updater, unless it's already running */
    if (in_updater) {
        printf("BEC firmware updater is already running\n");
    } else {
        rc = send_cmd(BEC_CMD_FW_UNLOCK, NULL, 0, NULL, 0, NULL);
        Delay(2);
        if (rc != 0) {
            printf("BEC FW unlock failure: (%s)\n", bec_err(rc));
            errs++;
            goto fwwrite_end;
        }
        memset(&id, 0, sizeof (id));
        rc = send_cmd_retry(BEC_CMD_ID, NULL, 0, &id, sizeof (id), &rlen);
        if (rc != 0) {
            printf("BEC ID failure after unlock: (%s)\n", bec_err(rc));
            errs++;
            goto fwwrite_end;
        }
        if (strcmp(id.bid_serial, "Updater") != 0) {
            printf("BEC firmware updater did not start (serial \"%s\")\n",
                   id.bid_serial);
            errs++;
            goto fwwrite_end;
        }
    }

    if (flag_quiet == 0) {
        printf("Writing [%*s]\rWriting [", FW_READ_DOTS, "");
        fflush(stdout);
    }
    /*
     * AmigaOS time can't be used here, as the keyboard message interface
     * disables interrupts for most of each transfer.
     */
    time_start = bec_time();

    req = (bec_fw_rw_t *) wbuf;
    for (offset = 0; offset < file_id.bid_size; offset += len) {
        if (is_user_abort()) {
            errs++;
            break;
        }
        len = file_id.bid_size - offset;
        if (len > FW_WRITE_CHUNK)
            len = FW_WRITE_CHUNK;
        memset(req, 0, sizeof (*req));
        req->bfrw_offset = offset;
        req->bfrw_size = len;
        memcpy(wbuf + sizeof (*req), fbuf + offset, len);

        for (tries = 0; tries < FW_WRITE_RETRIES; tries++) {
            bec_rx_timeout = 600;  // Added timeout for sector erase
            rc = send_cmd_retry(BEC_CMD_FW_WRITE, wbuf, sizeof (*req) + len,
                                &rhdr, sizeof (rhdr), &rlen);
            bec_rx_timeout = 0;  // Added timeout for sector erase
            if (rc == 0)
                break;
            retries++;
            if (flag_debug)
                printf("\nFW write at offset 0x%x retry: (%s)\n",
                       offset, bec_err(rc));
        }
        if (rc != 0) {
            printf("\nFW write at offset 0x%x failed: (%s)\n",
                   offset, bec_err(rc));
            errs++;
            break;
        }
        if ((rlen < sizeof (rhdr)) || (rhdr.bfrw_offset != offset) ||
            (rhdr.bfrw_size != len)) {
            printf("\nFW write at offset 0x%x bad reply: "
                   "rlen=%u offset=0x%x size=%u\n",
                   offset, rlen, rhdr.bfrw_offset, rhdr.bfrw_size);
            errs++;
            break;
        }
        dots_now = (offset + len) * FW_READ_DOTS / file_id.bid_size;
        if ((flag_quiet == 0) && (dots_now > dots)) {
            for (; dots < dots_now; dots++)
                printf(".");
            fflush(stdout);
        }
    }
    time_end = bec_time();
    if (flag_quiet == 0) {
        if (errs == 0)
            printf("]");
        printf("\n");
    }
    if (retries != 0)
        printf("%u write retries were required\n", retries);
    if (errs != 0) {
        printf("Firmware write incomplete; BEC updater is still running\n");
        goto fwwrite_end;
    }
    printf("Wrote %u bytes of firmware from %s", file_id.bid_size, filename);
    if ((time_start != 0) && (time_end > time_start)) {
        printf(" (%u KB/sec) in ",
               calc_kb_sec((uint) (time_end - time_start), file_id.bid_size));
        print_us_diff(time_start, time_end);
    } else {
        printf(" (BEC uptime unavailable for timing)\n");
    }

    /* Restart BEC in the new firmware */
    rc = send_cmd(BEC_CMD_FW_LOCK, NULL, 0, NULL, 0, NULL);
    if (rc != 0) {
        printf("BEC FW lock failure: (%s)\n", bec_err(rc));
        errs++;
        goto fwwrite_end;
    }

    /* BEC needs time to reboot */
    Delay(5);  // 100 msec

    /* Wait for the new firmware to start, up to about 10 more seconds */
    for (tries = 0; tries < 50; tries++) {
        if (tries > 0)
            Delay(2);
        memset(&id, 0, sizeof (id));
        rc = send_cmd(BEC_CMD_ID, NULL, 0, &id, sizeof (id), &rlen);
        if ((rc == 0) && (strcmp(id.bid_serial, "Updater") != 0))
            break;
    }
    if (rc != 0) {
        printf("BEC ID failure after update: (%s)\n", bec_err(rc));
        errs++;
        goto fwwrite_end;
    }
    if (strcmp(id.bid_serial, "Updater") == 0) {
        printf("BEC is still running the firmware updater\n");
        errs++;
        goto fwwrite_end;
    }
    fw_id_show("Running", &id);
    if (fw_id_match(&id, &file_id)) {
        printf("BEC firmware update verified\n");
    } else {
        printf("BEC firmware does not match %s\n", filename);
        errs++;
    }

fwwrite_end:
    if (fbuf != NULL)
        FreeMem(fbuf, fbuf_size);
    return (errs ? 1 : 0);
}

/*
 * cmd_fwverify
 * ------------
 * Compare a firmware image file with the firmware in BEC flash. If the
 * version, build date/time, or size reported by BEC_CMD_ID differ from
 * the file, they are reported as not matching without reading the flash.
 * Otherwise, the flash is read with BEC_CMD_FW_READ and compared.
 */
static int
cmd_fwverify(int argc, char *argv[])
{
    const char  *filename;
    bec_id_t     id;
    bec_id_t     file_id;
    uint8_t     *fbuf;
    uint8_t     *rbuf;
    uint         fbuf_size;
    uint         rbuf_size = sizeof (bec_fw_rw_t) + FW_READ_CHUNK;
    uint         fw_size;
    uint         offset;
    uint         pos;
    uint         len;
    uint         rlen;
    uint         rc;
    uint         retries = 0;
    uint         miscompares = 0;
    uint         first_bad = 0;
    uint         dots = 0;
    uint         dots_now;
    uint64_t     time_start;
    uint64_t     time_end;
    int          errs = 0;
    int          in_updater;

    if (argc < 2) {
        printf("fwverify requires a filename argument\n");
        return (1);
    }
    filename = argv[1];

    if (fw_file_load(filename, &fbuf, &fbuf_size, &file_id) != 0)
        return (1);
    fw_size = file_id.bid_size;

    memset(&id, 0, sizeof (id));
    rc = send_cmd_retry(BEC_CMD_ID, NULL, 0, &id, sizeof (id), &rlen);
    if (rc != 0) {
        printf("BEC ID failure: (%s)\n", bec_err(rc));
        errs++;
        goto fwverify_end;
    }

    /*
     * The updater reports its own version and build date/time, but the
     * size of the firmware in flash, so only the size can be checked.
     */
    in_updater = (strcmp(id.bid_serial, "Updater") == 0);
    fw_id_show(in_updater ? "Updater" : "Running", &id);
    fw_id_show("File", &file_id);
    if (in_updater ? (id.bid_size != fw_size) : !fw_id_match(&id, &file_id)) {
        printf("BEC firmware does not match %s\n", filename);
        errs++;
        goto fwverify_end;
    }

    rbuf = AllocMem(rbuf_size, MEMF_PUBLIC);
    if (rbuf == NULL) {
        printf("Failed to allocate %u bytes\n", rbuf_size);
        errs++;
        goto fwverify_end;
    }

    if (flag_quiet == 0) {
        printf("Verifying [%*s]\rVerifying [", FW_READ_DOTS, "");
        fflush(stdout);
    }
    time_start = bec_time();

    for (offset = 0; offset < fw_size; offset += len) {
        if (is_user_abort()) {
            errs++;
            break;
        }
        len = fw_read_chunk(offset, fw_size, rbuf, &retries);
        if (len == 0) {
            errs++;
            break;
        }
        for (pos = 0; pos < len; pos++) {
            if (rbuf[sizeof (bec_fw_rw_t) + pos] != fbuf[offset + pos]) {
                if (miscompares++ == 0)
                    first_bad = offset + pos;
            }
        }
        dots_now = (offset + len) * FW_READ_DOTS / fw_size;
        if ((flag_quiet == 0) && (dots_now > dots)) {
            for (; dots < dots_now; dots++)
                printf(".");
            fflush(stdout);
        }
    }
    time_end = bec_time();
    if (flag_quiet == 0) {
        if (errs == 0)
            printf("]");
        printf("\n");
    }
    FreeMem(rbuf, rbuf_size);

    if (retries != 0)
        printf("%u read retries were required\n", retries);
    if (errs != 0) {
        printf("Firmware verify incomplete\n");
    } else if (miscompares != 0) {
        printf("BEC firmware does not match %s: %u bytes differ, "
               "first at offset 0x%x\n", filename, miscompares, first_bad);
        errs++;
    } else {
        printf("Verified %u bytes of firmware with %s (%u KB/sec) in ",
               fw_size, filename,
               calc_kb_sec((uint) (time_end - time_start), fw_size));
        print_us_diff(time_start, time_end);
    }

fwverify_end:
    FreeMem(fbuf, fbuf_size);
    return (errs ? 1 : 0);
}

/*
 * update_qualifier
 * ----------------
 * Handle Amiga scancodes for shift, control, alt, etc to update the
 * qualifier field which must be presented to input.librayr in the
 * injected scancode input.
 */
static void
update_qualifier(uint scancode, UWORD *qualifier)
{
    uint key_up = scancode & 0x80;
    uint mask = 0;
    switch (scancode & 0x7f) {
        case AS_LEFTSHIFT:
            mask = IEQUALIFIER_LSHIFT;
            break;
        case AS_RIGHTSHIFT:
            mask = IEQUALIFIER_RSHIFT;
            break;
        case AS_CAPSLOCK:
            mask = IEQUALIFIER_CAPSLOCK;
            break;
        case AS_CTRL:
            mask = IEQUALIFIER_CONTROL;
            break;
        case AS_LEFTALT:
            mask = IEQUALIFIER_LALT;
            break;
        case AS_RIGHTALT:
            mask = IEQUALIFIER_RALT;
            break;
        case AS_LEFTAMIGA:
            mask = IEQUALIFIER_LCOMMAND;
            break;
        case AS_RIGHTAMIGA:
            mask = IEQUALIFIER_RCOMMAND;
            break;
        default:
            return;
    }
    if (key_up)
        *qualifier &= ~mask;
    else
        *qualifier |= mask;
}

/*
 * inject_key_scancode
 * -------------------
 * Inject the specified keyboard scancode into the Amiga input stream.
 */
static void
inject_key_scancode(uint scancode)
{
    struct IOStdReq *ir;
    struct InputEvent ie;
    struct MsgPort *iport;
    static UWORD qualifier;

    iport = CreateMsgPort();
    if (iport == 0)
        return;  // Failed to create message port
    ir = (struct IOStdReq *)CreateIORequest(iport, sizeof(struct IOStdReq));
    if (OpenDevice("input.device", 0, (struct IORequest *)ir, 0) != 0) {
        DeleteMsgPort(iport);
        return;  // Failed to open device
    }

    update_qualifier(scancode, &qualifier);
    ie.ie_NextEvent          = 0;
    ie.ie_Class              = IECLASS_RAWKEY;
    ie.ie_SubClass           = 0;
    ie.ie_Code               = scancode;
    ie.ie_Qualifier          = qualifier;
    ie.ie_position.ie_addr   = 0;
    ie.ie_TimeStamp.tv_secs  = 0;
    ie.ie_TimeStamp.tv_micro = 0;

    ir->io_Command = IND_WRITEEVENT;
    ir->io_Data    = &ie;
    ir->io_Length  = sizeof (ie);
    DoIO((struct IORequest *) ir);
    CloseDevice((struct IORequest *) ir);
    DeleteIORequest(ir);
    DeleteMsgPort(iport);
}

/*
 * poll_bec_for_amiga_scancodes
 * ----------------------------
 * Requests captured scancodes from BEC firmware. These scancodes are
 * normally automatically sent through the Amiga keyboard port. They
 * are instead received through the BEC protocol and injected into the
 * Amiga input stream. This is done so that keystrokes which normally
 * flow asynchronously over the keyboard serial port do not collide with
 * BEC messages sent from the Amiga, such as BEC_CMD_CONS_OUTPUT, which
 * is rapidly polled by bec_term() to capture BEC console output.
 */
static uint
poll_bec_for_amiga_scancodes(void)
{
    uint            rc;
    uint            rlen;
    uint8_t         replybuf[48];
    uint            pos;
    bec_poll_t      req;
    bec_poll_t     *rep = (bec_poll_t *)replybuf;
    uint8_t        *data;

    req.bkm_source  = BKM_SOURCE_AMIGA_SCANCODE;
    req.bkm_count   = 16;
    req.bkm_timeout = 500;  // msec timeout if not polling_for_scancodes again

    rep->bkm_count = 0;
    rc = send_cmd(BEC_CMD_POLL_INPUT, &req, sizeof (req),
                  replybuf, sizeof (replybuf), &rlen);
    if ((rc != 0) || (rep->bkm_count == 0))
        return (0);

    data = (uint8_t *) (rep + 1);
    for (pos = 0; pos < rep->bkm_count * 2; pos += 2)
        inject_key_scancode(data[pos]);

    return (0);
}

/*
 * cmd_term
 * --------
 * Open a KickSmash terminal.
 */
static int
cmd_term(int argc, char *argv[])
{
    int ch;
    int arg;
    uint rlen;
    uint rc = 0;
    uint fail_count = 0;
    uint is_poll = 0;
    uint count = 0;
    ULONG ihandle = Input();
    __attribute__((aligned(4))) uint8_t buf[64];
    uint8_t maxlen = sizeof (buf) - 4;
    uint poll_delay = 0;
    uint poll_count = 0;
    uint interactive;
    uint tick_last = 0;
    uint tick_count = 0;
    uint tick_now;

    if (argc > 1) {
        interactive = 0;
        buf[0] = 'U' - '@';  // ^U
        buf[1] = '\n';       // ^M Return
        if (send_cmd_retry(BEC_CMD_CONS_INPUT, buf, 2, NULL, 0, &rlen) != 0)
            return (1);
        buf[0] = ' ';  // ^M Return
        for (arg = 1; arg < argc; arg++) {
            if (send_cmd_retry(BEC_CMD_CONS_INPUT, argv[arg],
                               strlen(argv[arg]), NULL, 0, &rlen) != 0) {
                return (1);
            }
            if (arg == argc - 1)
                buf[0] = '\n';  // ^M Return
            if (send_cmd_retry(BEC_CMD_CONS_INPUT, buf, 1, NULL, 0, &rlen) != 0)
                return (1);
        }
    } else {
        interactive = 1;
        printf("Press ^X to exit\n");
        SetMode(Input(), 1);
    }
    setvbuf(stdout, NULL, _IONBF, 0);  // Raw output mode
    SetMode(Output(), 1);

#define KEY_CTRL_X 0x18
    tick_last = cia_ticks();
    while (fail_count < 10) {
        /* Poll for keystroke input */
        if (interactive && WaitForChar(ihandle, 0)) {
            if (Read(ihandle, buf, 1) == 0) {
                setvbuf(stdout, NULL, _IOLBF, 0);  // Line output mode
                printf("\nRead fail\n");
                break;
            }
            ch = buf[0];
            if (ch == KEY_CTRL_X) {
                printf("\n");
                break;
            }
            rc = send_cmd_retry(BEC_CMD_CONS_INPUT, buf, 1, NULL, 0, &rlen);
            if (rc != 0) {
                fail_count++;
                continue;
            } else {
                if (fail_count > 0)
                    fail_count--;
                poll_delay = 0;
            }
        }

        /* Poll BEC for raw keystroke input */
        if (poll_bec_for_amiga_scancodes())
            break;

        tick_now = cia_ticks();  // CIA counts downward
        tick_count += (uint16_t) (tick_last - tick_now);
        tick_last = tick_now;
        if ((count++ & 0x7) != 0)
            continue;  // Poll infrequently for BEC output

        if (poll_count < poll_delay) {
            poll_count++;
            continue;
        }
        poll_count = 0;

        /* Poll for BEC Controler output */
        maxlen = sizeof (buf) - 2;
        rc = send_cmd_retry(BEC_CMD_CONS_OUTPUT, &maxlen, sizeof (maxlen),
                            buf, sizeof (buf), &rlen);
#if 0
        if (rc == MSG_STATUS_BAD_CRC) {
            uint pos;
            printf("[Bad CRC rc=%d l=%04x s=????", (int) rc, rlen);
            for (pos = 0; pos < rlen + 8; pos++)
                printf(" %02x", buf[pos]);
            printf("]\n");
        }
#endif
        if (rc != 0) {
            is_poll = 1;
            fail_count++;
            continue;
        } else {
            if (fail_count > 0)
                fail_count--;
        }
        if (rlen > 0) {
            count = 0;  // Got output -- poll again right away
            poll_delay = 0;
            tick_count = 0;
            if (Write(Output(), buf, rlen) == 0) {
                setvbuf(stdout, NULL, _IOLBF, 0);  // Line output mode
                printf("\nWrite fail\n");
                break;
            }
        } else {
            poll_delay++;
            if (!interactive && (tick_count > 800000)) {
                /* More than 1 second with no additional output */
                break;
            }
        }
    }

    /* Restore cooked mode */
    if (interactive)
        SetMode(Input(), 0);
    SetMode(Output(), 0);
    setvbuf(stdout, NULL, _IOLBF, 0);  // Line output mode

    if (rc != 0) {
        printf("\n%s Fail (%s)\n", is_poll ? "Poll" : "Send", bec_err(rc));
        return (EXIT_FAILURE);
    }
    return (0);
}

#if 0
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
    CloseDevice((struct IORequest *) &TimeRequest);
    TimerBase = NULL;
}
#endif

#if 0
static void
setsystime(uint sec, uint usec)
{
    TimeRequest.tr_node.io_Command = TR_SETSYSTIME;
    TimeRequest.tr_time.tv_secs = sec;
    TimeRequest.tr_time.tv_micro = usec;
    DoIO((struct IORequest *) &TimeRequest);
}

static uint
getsystime(uint *usec)
{
    TimeRequest.tr_node.io_Command = TR_GETSYSTIME;
    DoIO((struct IORequest *) &TimeRequest);
    *usec = TimeRequest.tr_time.tv_micro;
    return (TimeRequest.tr_time.tv_secs);
}

static void
showdatestamp(struct DateStamp *ds, uint usec)
{
    struct DateTime  dtime;
    char             datebuf[32];
    char             timebuf[32];

    dtime.dat_Stamp.ds_Days   = ds->ds_Days;
    dtime.dat_Stamp.ds_Minute = ds->ds_Minute;
    dtime.dat_Stamp.ds_Tick   = ds->ds_Tick;
    dtime.dat_Format          = FORMAT_DOS;
    dtime.dat_Flags           = 0x0;
    dtime.dat_StrDay          = NULL;
    dtime.dat_StrDate         = datebuf;
    dtime.dat_StrTime         = timebuf;
    DateToStr(&dtime);
    printf("%s %s.%06u\n", datebuf, timebuf, usec);
}

static void
showsystime(uint sec, uint usec)
{
    struct DateStamp ds;
    uint min  = sec / 60;
    uint day  = min / (24 * 60);

    ds.ds_Days   = day;
    ds.ds_Minute = min % (24 * 60);
    ds.ds_Tick   = (sec % 60) * TICKS_PER_SECOND;
    showdatestamp(&ds, usec);
}
#endif

#define POT0DAT   VADDR16(0x00dff012)  // Counter for proportional input port 0
#define POTGOR    VADDR16(0x00dff016)  // Prop. pin and start counters (read)
#define POTGO     VADDR16(0x00dff034)  // Prop. pin and start counters (write)

#define JOY0DAT   VADDR16(0x00dff00a)  // Counter for digital mouse input port 0

static int
cmd_z(void)
{
    uint16_t potgo_orig = *POTGOR;
    uint16_t potgo_x;
    uint16_t joydat_1;
    uint16_t joydat_2;

    printf("orig=%04x\n", potgo_orig);
    if ((potgo_orig & (BIT(8) | BIT(10))) != (BIT(8) | BIT(10))) {
        printf("Mouse button already pressed: %04x\n", potgo_orig);
        return (1);
    }
    joydat_1 = *JOY0DAT;
    INTERRUPTS_DISABLE();
    *POTGO = (potgo_orig & 0xf000) | BIT(9) | BIT(11);  // Drop middle and right mouse buttons
    cia_spin(50);
    joydat_2 = *JOY0DAT;
    potgo_x = *POTGOR;
    cia_spin(50);
    *POTGO = potgo_orig;
    INTERRUPTS_ENABLE();
    printf("dat = %04x %04x [%04x]\n", joydat_1, joydat_2, potgo_x);
    return (0);
}

/*
 * exit_rc
 * -------
 * Exits with the AmigaDOS return code for a command's status:
 * RETURN_OK if rc is 0, RETURN_ERROR otherwise.
 */
static void __attribute__((noreturn))
exit_rc(int rc)
{
    exit(rc ? RETURN_ERROR : RETURN_OK);
}

int
main(int argc, char *argv[])
{
    int      arg;
    uint     loop;
    uint     loops = 1;
    uint     flag_inquiry = 0;
    uint     flag_test = 0;
    uint     flag_test_mask = 0;
    uint     flag_z = 0;
    uint     errs = 0;
    uint     aborted = 0;
    uint     do_multiple = 0;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds="
    SysBase = *(struct ExecBase **)4UL;
#pragma GCC diagnostic pop

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
                    case 'i':  // inquiry
                        flag_inquiry++;
                        break;
                    case 'k':
                        bec_msg_interface++;
                        break;
                    case 'l':  // loop count
                        if (++arg >= argc) {
                            printf("%s requires an argument\n", ptr);
                            exit(RETURN_ERROR);
                        }
                        loops = atoi(argv[arg]);
                        break;
                    case 'q':  // quiet
                        flag_quiet++;
                        break;
                    case 'R':  // fwread
                        exit_rc(cmd_fwread(argc - arg, argv + arg));
                        break;
                    case 's':  // set
                        exit_rc(cmd_set(argc - arg, argv + arg));
                        break;
                    case '0':  // pattern test
                    case '1':  // loopback test
                    case '2':  // loopback perf test
                    case '3':  // Commands
                        flag_test_mask |= BIT(*ptr - '0');
                        flag_test++;
                        break;
                    case 't':  // test
                        flag_test++;
                        break;
                    case 'T':  // BEC Controler firmware terminal
                        exit_rc(cmd_term(argc - 1, argv + 1));
                        break;
                    case 'V':  // fwverify
                        exit_rc(cmd_fwverify(argc - arg, argv + arg));
                        break;
                    case 'W':  // fwwrite
                        exit_rc(cmd_fwwrite(argc - arg, argv + arg));
                        break;
                    case 'z':  // z cmd
                        flag_z++;
                        break;
                    default:
                        printf("Unknown argument %s\n", ptr);
                        usage();
                        exit(RETURN_ERROR);
                }
            }
        } else if ((*ptr >= '0') && (*ptr <= '3') && (ptr[1] == '\0')) {
            flag_test_mask |= BIT(*ptr - '0');
            flag_test++;
        } else {
            printf("Error: unknown argument %s\n", ptr);
            usage();
            exit(RETURN_ERROR);
        }
    }

    if ((flag_inquiry | flag_test | flag_z) == 0) {
        printf("You must specify an operation to perform\n");
        usage();
        exit(RETURN_ERROR);
    }
    if (flag_test_mask == 0)
        flag_test_mask = ~0;
    if (flag_test) {
        srand32(time(NULL));
        test_loopback_buf = AllocMem(TEST_LOOPBACK_BUF * 2, MEMF_PUBLIC);
        if (flag_test_mask & (flag_test_mask - 1))
            do_multiple = 1;
    }
    if (flag_inquiry & (flag_inquiry - 1))
        do_multiple = 1;
    if (!!flag_test + !!flag_inquiry > 1)
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
        if (flag_inquiry) {
            if (flag_inquiry &&
                bec_identify() && ((loops == 1) || (loop > 1))) {
                errs++;
                break;
            }
        }
        if (flag_test) {
            if (bec_test(flag_test_mask) && ((loops == 1) || (loop > 1))) {
                errs++;
                break;
            }
        }
        if (flag_z) {
            if (cmd_z() != 0) {
                errs++;
                break;
            }
        }
        if (is_user_abort()) {
            aborted++;
            goto end_now;
        }
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
    if (test_loopback_buf != NULL)
        FreeMem(test_loopback_buf, TEST_LOOPBACK_BUF * 2);

    if (errs != 0)
        exit(RETURN_ERROR);
    exit(aborted ? RETURN_WARN : RETURN_OK);
}
