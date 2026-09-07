/*
 * linetest - exercise MC68040 bus transfer patterns to fast and chip RAM on
 * the AmigaPCI and verify the results.
 *
 * Every test fills a destination buffer with a known pattern using one access
 * pattern, flushes the caches, and verifies the buffer with the data cache
 * disabled so that verification uses plain long word reads. Mismatches are
 * counted per beat (long word 0..3 of each 16 byte line) and the failing data
 * bits are accumulated.
 *
 *   move.l   long word writes to fast RAM (single writes, or cache hits that
 *            are pushed as line writes when the 68040.library set copyback)
 *   movem    13 register MOVEM bursts to fast RAM, the write half of the
 *            lide.device sector copy
 *   movemcp  chip RAM to fast RAM copy with 13 register MOVEMs, the full
 *            lide.device sector copy with chip RAM standing in for the port
 *   movemff  fast RAM to fast RAM copy with 13 register MOVEMs (SysInfo style)
 *   movemfc  fast RAM to chip RAM copy with 13 register MOVEMs: back to back
 *            single writes into chip RAM on the mainboard
 *   movelfc  the same with move.l, one write at a time
 *   chipwd   chip RAM only: dense writes from registers, sparse read back
 *   chiprd   chip RAM only: sparse writes, dense read back
 *   chipdd   chip RAM only: dense writes, dense read back
 *            (the chip tests use a counting pattern; a mismatch reports
 *            which long word's data was found, i.e. the displacement)
 *
 * A canary buffer is placed, when the memory is free, at the fast RAM address
 * that mirrors the chip RAM destination (same SDRAM bank, row and column bits).
 * A U400 that starts a cycle for a chip RAM write by mistake stores the CPU's
 * data there; the canary is checked after every chip RAM write test.
 *   move16   MOVE16 from chip RAM: line writes to fast RAM only
 *   move16f  MOVE16 from fast RAM: line reads and line writes
 *
 * Usage: linetest [SIZE=kbytes] [PASSES=n] [ADDR=0xhex] [VERBOSE] [ONLY=name]
 *   SIZE    buffer size in KB (default 256; twice that much chip RAM is used)
 *   PASSES  passes over all patterns (default 2)
 *   ADDR    absolute address for the fast RAM destination (default AllocMem)
 *   VERBOSE print every mismatch (default: the first eight per test)
 *   ONLY    run only the named test
 *
 * Requires a 68040 and exec V37+. Build:
 *   m68k-amigaos-gcc -O2 -m68040 -noixemul -Wall -o linetest linetest.c -lamiga
 */
#include <exec/types.h>
#include <exec/memory.h>
#include <exec/execbase.h>
#include <proto/exec.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern struct ExecBase *SysBase;

/* ------------------------------------------------------------------ CACR */

ULONG new_cacr_arg;
ULONG old_cacr_ret;
ULONG tc_ret;

/* Runs in supervisor mode via Supervisor(). Pushes and invalidates both
 * caches, then loads the requested CACR value. Naked so that nothing sits
 * between the exception frame and the rte. */
__asm(
"    .text\n"
"    .globl _super_cacr_stub\n"
"_super_cacr_stub:\n"
"    movec   cacr,d0\n"
"    move.l  d0,_old_cacr_ret\n"
"    movec   tc,d0\n"
"    move.l  d0,_tc_ret\n"
"    cpusha  bc\n"
"    move.l  _new_cacr_arg,d0\n"
"    movec   d0,cacr\n"
"    nop\n"
"    rte\n");
extern ULONG super_cacr_stub(void);

static ULONG set_cacr(ULONG value)
{
    new_cacr_arg = value;
    Supervisor((ULONG (*)())super_cacr_stub);
    return old_cacr_ret;
}

#define CACR_EDC 0x80000000UL   /* 68040 data cache enable */
#define CACR_EIC 0x00008000UL   /* 68040 instruction cache enable */

/* -------------------------------------------------------------- patterns */

/* The address based patterns use the fast RAM destination's address for
 * every buffer, so that reference, source and destination hold the same
 * values wherever they live. */
static ULONG pattern_base;

static void fill_movel(ULONG *dst, ULONG bytes, int pattern, ULONG pass)
{
    volatile ULONG *p = dst;
    ULONG n = bytes / 4;
    ULONG a = pattern_base;
    ULONG x = 0x9E3779B9UL ^ pass;
    ULONG i;

    for (i = 0; i < n; i++, a += 4) {
        ULONG v;
        switch (pattern) {
        case 0:  v = (a ^ 0xA5A5A5A5UL) + pass * 0x01010101UL; break;
        case 1:  v = (i & 1) ? 0xFFFFFFFFUL : 0x00000000UL; break;
        case 2:  v = (i & 1) ? 0x00000000UL : 0xFFFFFFFFUL; break;
        case 3:  v = (i & 1) ? 0xAAAAAAAAUL : 0x55555555UL; break;
        case 4:  v = 1UL << (i & 31); break;
        case 5:  x ^= x << 13; x ^= x >> 17; x ^= x << 5; v = x; break;
        default: v = ((a >> 2) & 3) << 28 | (a & 0x0FFFFFFFUL); break;
        }
        *p++ = v;
    }
}

/* -------------------------------------------------------------- copies */

/* 13 register MOVEM copy, 52 bytes per step, as in lide.device. The source
 * pointer advances too (lide reads a fixed port address, chip RAM stands in). */
static void copy_movem13(const void *src, void *dst, ULONG bytes)
{
    ULONG n = bytes / 52;
    __asm volatile(
        "1:\n\t"
        "movem.l (%0),d0-d7/a1-a4/a6\n\t"
        "movem.l d0-d7/a1-a4/a6,(%1)\n\t"
        "lea     52(%0),%0\n\t"
        "lea     52(%1),%1\n\t"
        "subq.l  #1,%2\n\t"
        "bne.s   1b"
        : "+a" (src), "+a" (dst), "+m" (n)
        :
        : "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7", "a1", "a2", "a3", "a4", "a6", "memory");
}

static void copy_move16(const void *src, void *dst, ULONG bytes)
{
    ULONG n = bytes / 16;
    __asm volatile(
        "1:\n\t"
        "move16 (%0)+,(%1)+\n\t"
        "subq.l #1,%2\n\t"
        "bne.s  1b"
        : "+a" (src), "+a" (dst), "+d" (n)
        :
        : "memory");
}

/* ---------------------------------------------------------------- verify */

struct result {
    ULONG errors;
    ULONG beat[4];
    ULONG bits;
    ULONG bitcount[32];
};

static int verbose = 0;

static void verify(const ULONG *expect, const ULONG *got, ULONG bytes,
                   struct result *r, const char *what)
{
    const volatile ULONG *g = got;
    ULONG n = bytes / 4;
    ULONG i, shown = 0;

    memset(r, 0, sizeof(*r));
    for (i = 0; i < n; i++) {
        ULONG e = expect[i];
        ULONG v = g[i];
        if (v != e) {
            ULONG x = e ^ v;
            int b;
            r->errors++;
            r->beat[i & 3]++;
            r->bits |= x;
            for (b = 0; b < 32; b++)
                if (x & (1UL << b)) r->bitcount[b]++;
            if (verbose || shown < 8) {
                printf("  %s: %08lx expected %08lx got %08lx xor %08lx (beat %lu)\n",
                       what, (ULONG)&got[i], e, v, x, (ULONG)(i & 3));
                shown++;
            }
        }
    }
}

static void add_result(struct result *t, const struct result *r)
{
    int i;
    t->errors += r->errors;
    for (i = 0; i < 4; i++) t->beat[i] += r->beat[i];
    t->bits |= r->bits;
    for (i = 0; i < 32; i++) t->bitcount[i] += r->bitcount[i];
}

static void print_result(const char *what, const struct result *r)
{
    printf("  %-8s %8lu errors", what, r->errors);
    fflush(stdout);
    if (r->errors) {
        int b, worst = 0;
        printf("  beats [%lu %lu %lu %lu]  bits %08lx",
               r->beat[0], r->beat[1], r->beat[2], r->beat[3], r->bits);
        for (b = 1; b < 32; b++)
            if (r->bitcount[b] > r->bitcount[worst]) worst = b;
        printf("  worst bit D%d (%lu)", worst, r->bitcount[worst]);
    }
    printf("\n");
    fflush(stdout);
}

/* ----------------------------------------------------------------- tests */

enum { T_MOVEL, T_MOVEM, T_MOVEMCP, T_MOVEMFF, T_MOVEMFC, T_MOVELFC, T_CHIPWD, T_CHIPRD, T_CHIPDD, T_MOVE16, T_MOVE16F, T_COUNT };
static const char *tname[T_COUNT] = { "move.l", "movem", "movemcp", "movemff", "movemfc", "movelfc", "chipwd", "chiprd", "chipdd", "move16", "move16f" };

/* ---- chip RAM only tests: value for index i is seed + i * step ---- */

/* About 7us of nothing, with cached code so no bus activity. */
static void sparse_delay(void)
{
    __asm volatile("move.w #100,d0\n\t1:\tdbra d0,1b" ::: "d0", "cc");
}

static void seq_write(ULONG *dst, ULONG n, ULONG seed, ULONG step, int dense)
{
    if (dense) {
        __asm volatile(
            "1:\n\t"
            "move.l %2,(%0)+\n\t"
            "add.l  %3,%2\n\t"
            "subq.l #1,%1\n\t"
            "bne.s  1b"
            : "+a" (dst), "+d" (n), "+d" (seed)
            : "d" (step)
            : "memory", "cc");
    } else {
        volatile ULONG *p = dst;
        ULONG i;
        for (i = 0; i < n; i++) { p[i] = seed; seed += step; sparse_delay(); }
    }
}

static void seq_verify(const ULONG *got, ULONG n, ULONG seed, ULONG step, int dense,
                       struct result *r, const char *what)
{
    const volatile ULONG *g = got;
    ULONG i, shown = 0, v, e = seed;
    memset(r, 0, sizeof(*r));
    for (i = 0; i < n; i++, e += step) {
        v = g[i];
        if (!dense) sparse_delay();
        if (v != e) {
            ULONG x = e ^ v;
            long d;
            int b, found = 0;
            r->errors++;
            r->beat[i & 3]++;
            r->bits |= x;
            for (b = 0; b < 32; b++) if (x & (1UL << b)) r->bitcount[b]++;
            if (verbose || shown < 8) {
                printf("  %s: %08lx expected %08lx got %08lx xor %08lx", what, (ULONG)&got[i], e, v, x);
                for (d = -8; d <= 8 && !found; d++) {
                    if (d && v == e + (ULONG)d * step) { printf(" = data of long word %+ld", d); found = 1; }
                }
                printf("\n");
                shown++;
            }
        }
    }
}

static ULONG *dst, *src_chip, *src_fast, *dst_chip, *ref;
static ULONG *canary;
static ULONG canary_size;
static ULONG size;
static ULONG cacr, cacr_nodata;

static void copy_movel(const ULONG *src, ULONG *dst, ULONG bytes)
{
    volatile ULONG *d = dst;
    ULONG n = bytes / 4, i;
    for (i = 0; i < n; i++) d[i] = src[i];
}

/* Canary in the fast RAM mirror of the chip RAM destination. */
static void canary_fill(void)
{
    volatile ULONG *p = canary;
    ULONG i;
    if (!canary) return;
    for (i = 0; i < canary_size / 4; i++) p[i] = 0xC0FFEE00UL + i;
    CacheClearU();
}

static void canary_check(const char *what)
{
    const volatile ULONG *p = canary;
    ULONG i, bad = 0;
    if (!canary) return;
    set_cacr(cacr_nodata);
    for (i = 0; i < canary_size / 4; i++) {
        if (p[i] != 0xC0FFEE00UL + i) {
            if (bad < 6)
                printf("  %s: CANARY HIT at %08lx: %08lx (chip write of %08lx landed in fast RAM)\n",
                       what, (ULONG)&canary[i], p[i], (ULONG)dst_chip + i * 4);
            bad++;
        }
    }
    set_cacr(cacr);
    if (bad) {
        printf("  %s: %lu canary long words overwritten: U400 ran cycles for chip RAM writes\n", what, bad);
        canary_fill();
    }
}

/* Run one test: prepare, fill the destination the given way, verify. */
static int run_test(int t, int pattern, ULONG pass, struct result *r)
{
    ULONG *target = dst;
    const char *what = tname[t];

    if ((t == T_MOVEMFF || t == T_MOVE16F) && !src_fast) return 0;
    if ((t == T_MOVEMFC || t == T_MOVELFC || t == T_CHIPWD || t == T_CHIPRD || t == T_CHIPDD) && !dst_chip) return 0;

    if (t == T_CHIPWD || t == T_CHIPRD || t == T_CHIPDD) {
        /* Pattern selects the seed and step: unique values per long word. */
        static const ULONG seeds[7] = { 0x00000000UL, 0xFFFFFFFFUL, 0x00000000UL, 0x55555555UL, 0x00000001UL, 0x9E3779B9UL, 0x10000000UL };
        static const ULONG steps[7] = { 0x00000001UL, 0xFFFFFFFFUL, 0x11111111UL, 0x55555555UL, 0x01010101UL, 0x7F4A7C15UL, 0x10000004UL };
        ULONG seed = seeds[pattern] ^ (pass * 0x01000000UL), step = steps[pattern];
        int dense_w = (t != T_CHIPRD), dense_r = (t != T_CHIPWD);
        Forbid();
        seq_write(dst_chip, size / 4, seed, step, dense_w);
        CacheClearU();
        set_cacr(cacr_nodata);
        seq_verify(dst_chip, size / 4, seed, step, dense_r, r, what);
        set_cacr(cacr);
        canary_check(what);
        Permit();
        return 1;
    }

    Forbid();
    switch (t) {
    case T_MOVEL:
        fill_movel(dst, size, pattern, pass);
        break;
    case T_MOVEM:
        /* Source is the reference buffer: reads are cache hits or fills,
         * the interesting part is the 13 long word write burst. */
        fill_movel(dst, size, 6, ~pass);
        CacheClearU();
        copy_movem13(ref, dst, size);
        break;
    case T_MOVEMCP:
        fill_movel(src_chip, size, pattern, pass);
        fill_movel(dst, size, 6, ~pass);
        CacheClearU();
        copy_movem13(src_chip, dst, size);
        break;
    case T_MOVEMFF:
        fill_movel(src_fast, size, pattern, pass);
        fill_movel(dst, size, 6, ~pass);
        CacheClearU();
        copy_movem13(src_fast, dst, size);
        break;
    case T_MOVEMFC:
        target = dst_chip;
        fill_movel(src_fast ? src_fast : ref, size, pattern, pass);
        fill_movel(dst_chip, size, 6, ~pass);
        CacheClearU();
        copy_movem13(src_fast ? src_fast : ref, dst_chip, size);
        break;
    case T_MOVELFC:
        target = dst_chip;
        fill_movel(src_fast ? src_fast : ref, size, pattern, pass);
        fill_movel(dst_chip, size, 6, ~pass);
        CacheClearU();
        copy_movel(src_fast ? src_fast : ref, dst_chip, size);
        break;
    case T_MOVE16:
        fill_movel(src_chip, size, pattern, pass);
        fill_movel(dst, size, 6, ~pass);
        CacheClearU();
        copy_move16(src_chip, dst, size);
        break;
    case T_MOVE16F:
        fill_movel(src_fast, size, pattern, pass);
        fill_movel(dst, size, 6, ~pass);
        CacheClearU();
        copy_move16(src_fast, dst, size);
        break;
    }
    CacheClearU();
    set_cacr(cacr_nodata);
    verify(ref, target, size, r, what);
    set_cacr(cacr);
    if (target == dst_chip) canary_check(what);
    Permit();
    return 1;
}

/* ------------------------------------------------------------------ main */

int main(int argc, char **argv)
{
    ULONG passes = 2;
    ULONG addr = 0;
    ULONG *dst_raw = NULL, *src_chip_raw = NULL, *src_fast_raw = NULL, *dst_chip_raw = NULL, *ref_raw = NULL;
    const char *only = NULL;
    struct result total[T_COUNT], r;
    ULONG pass;
    int i, t, rc = 0, ran[T_COUNT];
    static const char *pname[] = { "address", "0/F", "F/0", "5/A", "walk1", "random", "beat" };

    size = 256 * 1024;
    for (i = 1; i < argc; i++) {
        if (strncmp(argv[i], "SIZE=", 5) == 0)        size = strtoul(argv[i] + 5, NULL, 0) * 1024;
        else if (strncmp(argv[i], "PASSES=", 7) == 0) passes = strtoul(argv[i] + 7, NULL, 0);
        else if (strncmp(argv[i], "ADDR=", 5) == 0)   addr = strtoul(argv[i] + 5, NULL, 0);
        else if (strncmp(argv[i], "ONLY=", 5) == 0)   only = argv[i] + 5;
        else if (strcmp(argv[i], "VERBOSE") == 0)     verbose = 1;
        else {
            printf("usage: linetest [SIZE=kbytes] [PASSES=n] [ADDR=0xhex] [VERBOSE] [ONLY=name]\n");
            return 5;
        }
    }
    size = (size / 208) * 208;       /* multiple of 16 and of 52 */
    if (size < 4160) size = 4160;

    if (!(SysBase->AttnFlags & AFF_68040)) {
        printf("linetest: needs a 68040\n");
        return 10;
    }

    if (addr) {
        dst_raw = AllocAbs(size, (APTR)addr);
        dst = dst_raw;
        if (!dst || ((ULONG)dst & 15)) {
            printf("linetest: cannot AllocAbs %lu bytes at %08lx (16 byte aligned?)\n", size, addr);
            rc = 10; goto out;
        }
    } else {
        dst_raw = AllocMem(size + 16, MEMF_FAST | MEMF_PUBLIC);
        if (!dst_raw) { printf("linetest: no %lu bytes of fast RAM\n", size); rc = 10; goto out; }
        dst = (ULONG *)(((ULONG)dst_raw + 15) & ~15UL);
    }
    src_chip_raw = AllocMem(size + 16, MEMF_CHIP | MEMF_PUBLIC);
    if (!src_chip_raw) { printf("linetest: no %lu bytes of chip RAM (use SIZE=)\n", size); rc = 10; goto out; }
    src_chip = (ULONG *)(((ULONG)src_chip_raw + 15) & ~15UL);
    dst_chip_raw = AllocMem(size + 16, MEMF_CHIP | MEMF_PUBLIC);
    if (dst_chip_raw) dst_chip = (ULONG *)(((ULONG)dst_chip_raw + 15) & ~15UL);
    src_fast_raw = AllocMem(size + 16, MEMF_FAST | MEMF_PUBLIC);
    if (src_fast_raw) src_fast = (ULONG *)(((ULONG)src_fast_raw + 15) & ~15UL);
    /* Canary at the fast RAM mirror of the chip destination, as large as is free. */
    if (dst_chip) {
        ULONG mirror = 0x08000000UL | ((ULONG)dst_chip & 0x03FFFFFFUL);
        for (canary_size = size; canary_size >= 4096; canary_size /= 2) {
            canary = AllocAbs(canary_size, (APTR)mirror);
            if (canary) break;
        }
        if (!canary) canary_size = 0;
    }
    ref_raw = AllocMem(size + 16, MEMF_ANY | MEMF_PUBLIC);
    if (!ref_raw) { printf("linetest: no memory for the reference buffer\n"); rc = 10; goto out; }
    ref = (ULONG *)(((ULONG)ref_raw + 15) & ~15UL);

    /* Exec preserves the code of the last alert across a warm reboot. */
    printf("last alert: %08lx (task %08lx)%s\n",
           SysBase->LastAlert[0], SysBase->LastAlert[1],
           ((ULONG)SysBase->LastAlert[0] == 0xFFFFFFFFUL) ? " none since power on" : "");

    pattern_base = (ULONG)dst;

    cacr = set_cacr(0);
    set_cacr(cacr);
    printf("linetest: %lu KB per buffer\n  dst fast %08lx  src chip %08lx  dst chip %08lx  src fast %08lx  ref %08lx\n",
           size / 1024, (ULONG)dst, (ULONG)src_chip, (ULONG)dst_chip, (ULONG)src_fast, (ULONG)ref);
    if (canary)
        printf("  canary %08lx (%lu KB) mirrors the chip destination in fast RAM\n", (ULONG)canary, canary_size / 1024);
    else
        printf("  no canary: the fast RAM mirror of the chip destination is in use\n");
    canary_fill();
    printf("CACR %08lx: I-cache %s, D-cache %s; TC %08lx: MMU %s\n", cacr,
           (cacr & CACR_EIC) ? "on" : "off", (cacr & CACR_EDC) ? "on" : "off",
           tc_ret, (tc_ret & 0x8000) ? "on (68040.library cache modes apply)" : "off (default cacheable write-through)");
    if (!(cacr & CACR_EDC))
        printf("note: data cache is off, fast RAM writes are single writes and nothing is pushed\n");
    cacr_nodata = cacr & ~CACR_EDC;

    memset(total, 0, sizeof(total));
    memset(ran, 0, sizeof(ran));

    for (pass = 0; pass < passes; pass++) {
        int pattern;
        for (pattern = 0; pattern < 7; pattern++) {
            fill_movel(ref, size, pattern, pass);
            printf("pass %lu pattern %s\n", pass + 1, pname[pattern]);
            for (t = 0; t < T_COUNT; t++) {
                if (only && strcmp(only, tname[t]) != 0) continue;
                /* Announce before running so a silent hang names the test. */
                printf("  running %s\r", tname[t]);
                fflush(stdout);
                if (!run_test(t, pattern, pass, &r)) continue;
                ran[t] = 1;
                print_result(tname[t], &r);
                add_result(&total[t], &r);
            }
        }
    }

    printf("\nsummary over %lu passes of %lu KB:\n", passes, size / 1024);
    for (t = 0; t < T_COUNT; t++) {
        if (!ran[t]) continue;
        print_result(tname[t], &total[t]);
        if (total[t].errors) rc = 5;
    }
    printf(rc ? "FAIL\n" : "PASS\n");

out:
    if (ref_raw) FreeMem(ref_raw, size + 16);
    if (canary) FreeMem(canary, canary_size);
    if (src_fast_raw) FreeMem(src_fast_raw, size + 16);
    if (dst_chip_raw) FreeMem(dst_chip_raw, size + 16);
    if (src_chip_raw) FreeMem(src_chip_raw, size + 16);
    if (dst_raw) FreeMem(dst_raw, addr ? size : size + 16);
    return rc;
}
