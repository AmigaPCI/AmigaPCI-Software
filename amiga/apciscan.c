/*
 * apcirom
 * -------
 * Utility to scan and enumerate Amiga Firebird PCI configuration space.
 *
 * Copyright 2025 Chris Hooper.  This program and source may be used
 * and distributed freely, for any purpose which benefits the Amiga
 * community. Commercial use of the binary, source, or algorithms requires
 * prior written or email approval from Chris Hooper <amiga@cdh.eebugs.com>.
 * All redistributions must retain this Copyright notice.
 *
 * DISCLAIMER: THE SOFTWARE IS PROVIDED "AS-IS", WITHOUT ANY WARRANTY.
 * THE AUTHOR ASSUMES NO LIABILITY FOR ANY DAMAGE ARISING OUT OF THE USE
 * OR MISUSE OF THIS UTILITY OR INFORMATION REPORTED BY THIS UTILITY.
 */
#include <stdio.h>
#include <stdlib.h>
#include <proto/exec.h>
#include <proto/expansion.h>
#include "util.h"
#include "pci_access.h"

#define BIT(x)         (1U << (x))

#define ALIGN_DOWN(addr, alignment) ((addr) & ~((alignment) - 1))
#define ALIGN_UP(addr, alignment)   ALIGN_DOWN((addr) + (alignment) - 1, \
                                               alignment)
#define SIZE_1MB (1 << 20)
#define SIZE_4KB (4 << 10)

struct Library *PCIBase;

typedef struct pci_dev pci_dev_t;
typedef struct pci_dev {
    uint16_t   pd_vendor;
    uint16_t   pd_device;
    uint32_t   pd_bar[7];
    uint32_t   pd_bar_size[7];
    uint32_t   pd_size;         // size of all BARs + bars of children
    uint8_t    pd_bar_type[7];
    uint8_t    pd_htype;
    uint8_t    pd_bus;
    uint8_t    pd_dev;
    uint8_t    pd_func;
    uint8_t    pd_allocated;    // Mask of BARs which have allocated addresses
    pci_dev_t *pd_child;
    pci_dev_t *pd_next;
} pci_dev_t;

static pci_dev_t  pci_root_dev;
static pci_dev_t *pci_root = &pci_root_dev;
static uint8_t    pci_max_bus = 0;
static uint32_t   pci_alloc_mem_base;
static uint32_t   pci_alloc_mem_max;
static uint32_t   pci_alloc_mem_pre_base;
static uint32_t   pci_alloc_mem_pre_max;
static uint32_t   pci_alloc_io_base;
static uint32_t   pci_alloc_io_max;
static uint       firestorm_mode = 0;

/*
 * pci_discover
 * ------------
 * Perform a depth-first search to locate devices, assign bus numbers,
 * and determine total BAR sizes.
 */
static void
pci_discover(uint8_t bus, pci_dev_t *parent_dev)
{
    uint        dev;
    uint        func;
    uint8_t     header;
    uint16_t    vendor;
    uint16_t    device;
    uint32_t    vd;
    pci_dev_t  *cur;
    pci_dev_t **parent_next = &parent_dev->pd_child;
    uint        maxdev = (bus == 0) ? PCI_MAX_PHYS_SLOT : PCI_MAX_DEV;

    for (dev = 0; dev < maxdev; dev++) {
        for (func = 0; func < PCI_MAX_FUNC; func++) {
            vd = pci_read32v(bus, dev, func, PCI_OFF_VENDOR);

            /* Check Vendor / Device for device presence */
            if ((vd == 0xffffffff) || (vd == 0x00000000)) {
                if (func == 0)
                    break; // Don't probe further if no primary function
                continue;
            }
            vendor = (uint16_t) vd;
            device = vd >> 16;
            printf("%x.%x.%x %04x.%04x\n",
                   bus, dev, func, vendor, device);

            cur = AllocMem(sizeof (*cur), MEMF_PUBLIC | MEMF_CLEAR);
            if (cur == NULL)
                break;
            cur->pd_vendor = vendor;
            cur->pd_device = device;
            cur->pd_child = NULL;
            cur->pd_next = NULL;
            cur->pd_bus = bus;
            cur->pd_dev = dev;
            cur->pd_func = func;
            *parent_next = cur;
            parent_next = &cur->pd_next;
            header = pci_read8(bus, dev, func, PCI_OFF_HEADERTYPE);
            cur->pd_htype = header;
            uint is_bridge = (cur->pd_htype & 0x7F) == 1;
            uint max_bars = is_bridge ? 3 : 7;  // Bridge has only two BARs

            /*
             * Set device's PCI latency timer to 0x80 because of
             * Zorro III design.  XXX: Research if this is optimal.
             */
            pci_write8(bus, dev, func, PCI_OFF_LATENCYTIMER, 0x80);

            /* Determine BAR size */
            uint total_size = 0;
            uint bar;
            for (bar = 0; bar < max_bars; bar++) {
                uint     bar_offset = PCI_OFF_BAR0 + (bar * 4);
                uint32_t old_val;
                uint32_t size_mask = 0;

                if (bar == max_bars - 1) {
                    /* Optional ROM BAR */
                    bar_offset = is_bridge ? PCI_OFF_BR_ROM_BAR :
                                             PCI_OFF_ROM_BAR;
                }

                old_val = pci_read32(bus, dev, func, bar_offset);
                pci_write32(bus, dev, func, bar_offset, 0xffffffff);
                size_mask = pci_read32(bus, dev, func, bar_offset);
                pci_write32(bus, dev, func, bar_offset, old_val);

                if (size_mask != 0) {
                    uint32_t size = ~(size_mask & 0xFFFFFFF0) + 1;
                    cur->pd_bar_size[bar] = size;
                    if (bar == max_bars - 1)
                        cur->pd_bar_type[bar] = BIT(7);  // Exp ROM BAR
                    else
                        cur->pd_bar_type[bar] = size_mask & 0xf;
                    total_size += size;
                    printf("  [%u] size %08x type %02x\n", bar, size,
                           cur->pd_bar_type[bar]);
                    if ((size_mask & (BIT(0) | BIT(1) | BIT(2))) == BIT(2)) {
                        /* 64-bit memory BAR */
                        // XXX: Need to handle 64-bit size here??
                        bar++;  // Skip upper bits of this BAR
                    }
                }
            }

            /* Bridge Detection & Recursive Scan */
            if ((header & 0x7F) == 1) {  // Type 1 is a Bridge
                uint8_t sec_bus = pci_max_bus + 1;
                uint8_t latency = 0x80;
                uint32_t bus_value;
                if (sec_bus == 0)
                    break;  // No more buses available
                pci_max_bus = sec_bus;

                bus_value = bus | (sec_bus << 8) | (0xff << 16) |
                            (latency << 24);
                pci_write32(bus, dev, func, PCI_OFF_BR_PRI_BUS, bus_value);

                printf("  Child Bus %02x\n", sec_bus);
                pci_discover(sec_bus, cur);

                bus_value = bus | (sec_bus << 8) | (pci_max_bus << 16) |
                            (latency << 24);
                pci_write32(bus, dev, func, PCI_OFF_BR_PRI_BUS, bus_value);

                /* Round up to next 1 MB boundary */
                cur->pd_size = ALIGN_UP(cur->pd_size, SIZE_1MB);
            }
            if (parent_dev != NULL) {
                parent_dev->pd_size += total_size + cur->pd_size;
            }

            if (!(header & 0x80) && func == 0)
                break; // Not multi-function
        }
    }
}

static uint32_t
get_next_mem_base(uint barsize)
{
    uint32_t next = ALIGN_UP(pci_alloc_mem_base, barsize);
    if (next <= pci_alloc_mem_max) {
        pci_alloc_mem_base = next;
        return (next);
    }
    /* Can not allocate this device */
    return (0xffffffff);
}

#if 0
static uint32_t
get_next_mem_pre_base(uint barsize)
{
    uint32_t next = ALIGN_UP(pci_alloc_mem_pre_base, barsize);
    if (next <= pci_alloc_mem_pre_max) {
        pci_alloc_mem_base = next;
        return (next);
    }
    /* Can not allocate this device */
    return (0xffffffff);
}
#endif

static uint32_t
get_next_io_base(uint barsize)
{
    uint32_t next = ALIGN_UP(pci_alloc_io_base, barsize);
    if (next <= pci_alloc_io_max) {
        pci_alloc_io_base = next;
        return (next);
    }
    /* Can not allocate this device */
    return (0xffffffff);
}

/*
 * pci_allocate
 * ------------
 * Perform a allocation of the entire PCI tree. Devices or bridges with
 * their respective subtrees with the largest required size are allocated
 * first.
 */
static void
pci_allocate(pci_dev_t *parent_dev)
{
    pci_dev_t *cur;
    pci_dev_t *maxdev;

    while (1) {
        uint maxsize = 0;
        int  maxbar = -1;
        maxdev = NULL;

        /* Select the next largest BAR or sub-bus to allocate */
        for (cur = parent_dev->pd_child; cur != NULL; cur = cur->pd_next) {
            uint is_bridge = (cur->pd_htype & 0x7F) == 1;
            uint max_bars = is_bridge ? 3 : 7;  // Bridge has only two BARs
            uint bar;
            for (bar = 0; bar < max_bars; bar++) {
                if (cur->pd_allocated & BIT(bar))
                    continue;  // Already allocated
                if (maxsize < cur->pd_bar_size[bar]) {
                    maxsize = cur->pd_bar_size[bar];
                    maxdev  = cur;
                    maxbar  = bar;
                }
            }
            if (is_bridge && ((cur->pd_allocated & BIT(7)) == 0) &&
                ((maxsize < cur->pd_size) || (maxdev == NULL))) {
                /* Subordinate bus has larger demand */
                maxsize = cur->pd_size;
                maxdev  = cur;
                maxbar  = -1;  // Allocate downstream
            }
        }
        if (maxdev == NULL)
            break;  // No more devices to allocate

        printf("alloc %x.%x.%x ",
               maxdev->pd_bus, maxdev->pd_dev, maxdev->pd_func);
#if 0
        printf("maxsize=%08x ", maxsize);
#endif
        if (maxbar == -1) {
            /* Allocate subordinate bus */
            pci_alloc_mem_base = ALIGN_UP(pci_alloc_mem_base, SIZE_1MB);
            pci_alloc_io_base  = ALIGN_UP(pci_alloc_io_base, SIZE_4KB);
            uint32_t start_mem = pci_alloc_mem_base;
            uint32_t start_io  = pci_alloc_io_base;
            uint32_t end_mem;
            uint32_t end_io;
            printf("Bridge\n", start_mem, start_io);

            pci_allocate(maxdev);

            /* Align allocators to bridge alignment requirement */
            pci_alloc_mem_base = ALIGN_UP(pci_alloc_mem_base, SIZE_1MB);
            pci_alloc_io_base  = ALIGN_UP(pci_alloc_io_base, SIZE_4KB);
            end_mem = pci_alloc_mem_base;
            end_io  = pci_alloc_io_base;
            printf("      %x.%x.%x MEMW=%x-%x  IOW=%x-%x\n",
                   maxdev->pd_bus, maxdev->pd_dev, maxdev->pd_func,
                   start_mem, end_mem, start_io, end_io);

            if (start_mem == end_mem) {
                /* No downstream memory space */
                start_mem = 0xffffffff;
                end_mem = 0;
            } else {
                end_mem -= SIZE_1MB;
            }
            if (start_io == end_io) {
                /* No downstream I/O space */
                start_io = 0xffffffff;
                end_io = 0;
            } else {
                end_io -= SIZE_4KB;
            }
            /* Handle I/O window */
            pci_write8(maxdev->pd_bus, maxdev->pd_dev, maxdev->pd_func,
                       PCI_OFF_BR_IO_BASE, (start_io >> 8) & 0xf0);
            pci_write8(maxdev->pd_bus, maxdev->pd_dev, maxdev->pd_func,
                       PCI_OFF_BR_IO_LIMIT, (end_io >> 8) & 0xf0);
            pci_write16(maxdev->pd_bus, maxdev->pd_dev, maxdev->pd_func,
                        PCI_OFF_BR_IO_BASE_U, start_io >> 16);
            pci_write16(maxdev->pd_bus, maxdev->pd_dev, maxdev->pd_func,
                        PCI_OFF_BR_IO_LIMIT_U, end_io >> 16);

            /* Handle memory window */
            pci_write16(maxdev->pd_bus, maxdev->pd_dev, maxdev->pd_func,
                        PCI_OFF_BR_W32_BASE, start_mem >> 16);
            pci_write16(maxdev->pd_bus, maxdev->pd_dev, maxdev->pd_func,
                        PCI_OFF_BR_W32_LIMIT, end_mem >> 16);

            /* Disable 64-bit pre-fetchable memory window */
            pci_write16(maxdev->pd_bus, maxdev->pd_dev, maxdev->pd_func,
                        PCI_OFF_BR_W64_BASE, 0xffff);
            pci_write16(maxdev->pd_bus, maxdev->pd_dev, maxdev->pd_func,
                        PCI_OFF_BR_W64_LIMIT, 0x0000);
            pci_write32(maxdev->pd_bus, maxdev->pd_dev, maxdev->pd_func,
                        PCI_OFF_BR_W64_BASE_U, 0xffffffff);
            pci_write32(maxdev->pd_bus, maxdev->pd_dev, maxdev->pd_func,
                        PCI_OFF_BR_W64_LIMIT_U, 0x00000000);
            maxdev->pd_allocated |= BIT(7);
        } else {
            /* Allocate BAR on this device */
            uint8_t  bartype    = maxdev->pd_bar_type[maxbar];
            uint32_t barsize    = maxdev->pd_bar_size[maxbar];
            uint     bar_offset = PCI_OFF_BAR0 + 4 * (maxbar);

            /*
             * XXX: Not yet taking into account pci_alloc_mem_max
             *      For compatibility with E3B Firestorm, the PCI config
             *      space and I/O mapped range begins at 0x1fc00000.
             *      If the next allocation would enter this range, then
             *      the BAR allocation request can not be met.
             *      In the case of AmigaPCI, that range may be skipped,
             *      and further allocations may be made up to the maximum
             *      0xff000000 address.
             */
            if (bartype & BIT(7)) {
                /* ROM BAR */
                if (get_next_mem_base(barsize) == 0xffffffff)
                    goto can_not_map_bar;
                printf("ROM      base=%08x", pci_alloc_mem_base);
                maxdev->pd_bar[maxbar] = pci_alloc_mem_base;
                uint is_bridge = (maxdev->pd_htype & 0x7F) == 1;
                bar_offset = is_bridge ? PCI_OFF_BR_ROM_BAR : PCI_OFF_ROM_BAR;

                pci_write32v(maxdev->pd_bus, maxdev->pd_dev, maxdev->pd_func,
                             bar_offset, pci_alloc_mem_base | BIT(0));
                pci_alloc_mem_base += barsize;
            } else if (bartype & BIT(0)) {
                /* I/O space BAR */
                if (get_next_io_base(barsize) == 0xffffffff)
                    goto can_not_map_bar;
                printf("BAR%u IO  base=%08x", maxbar, pci_alloc_io_base);
                maxdev->pd_bar[maxbar] = pci_alloc_io_base;
                pci_write32v(maxdev->pd_bus, maxdev->pd_dev, maxdev->pd_func,
                             bar_offset, pci_alloc_io_base);
                pci_alloc_io_base += barsize;
            } else {
                /* Memory space BAR */
                if (get_next_mem_base(barsize) == 0xffffffff)
                    goto can_not_map_bar;
                printf("BAR%u MEM base=%08x", maxbar, pci_alloc_mem_base);
                maxdev->pd_bar[maxbar] = pci_alloc_mem_base;
                pci_write32v(maxdev->pd_bus, maxdev->pd_dev, maxdev->pd_func,
                             bar_offset, pci_alloc_mem_base);
                if ((bartype & (BIT(2) | BIT(1))) == BIT(2)) {
                    /* 64-bit memory BAR */
                    pci_write32v(maxdev->pd_bus, maxdev->pd_dev,
                                 maxdev->pd_func, bar_offset + 4, 0x00000000);
                    maxdev->pd_allocated |= BIT(maxbar + 1);
                }
                pci_alloc_mem_base += barsize;
            }
            printf(" size=%08x\n", barsize);
can_not_map_bar:
            maxdev->pd_allocated |= BIT(maxbar);
        }
    }
}

static void
pci_enable(pci_dev_t *parent_dev)
{
    pci_dev_t *cur;
    for (cur = parent_dev->pd_child; cur != NULL; cur = cur->pd_next) {
        uint is_bridge = (cur->pd_htype & 0x7F) == 1;
        if (is_bridge) {
#if 0
            /* Disable bridge secondary side error reporting */
            pci_write16(cur->pd_bus, cur->pd_dev, cur->pd_func,
                        PCI_OFF_BR_CONTROL, BIT(0));  // Enable parity
#endif

            pci_enable(cur);

            /* Enable bridge secondary side error reporting */
            pci_write16(cur->pd_bus, cur->pd_dev, cur->pd_func,
                        PCI_OFF_BR_CONTROL,
                        BIT(0) |  // Enable parity
                        BIT(1) |  // Enable SERR#
//                      BIT(5) |  // Enable Master Abort reporting
                        BIT(11) | // Enable SERR# on timer expiration
                        0);
            /*
             * XXX: Master Abort reporting slows down PCI scan for commands
             *      such as "pci ls"
             *      I'm not sure whether this should be enabled by default
             */
        }

        /* Enable I/O space, Memory space, Bus Mastering, and error reporting */
        pci_write16(cur->pd_bus, cur->pd_dev, cur->pd_func, PCI_OFF_CMD,
                    BIT(0) |  // Enable I/O Space
                    BIT(1) |  // Enable Memory Space
                    BIT(2) |  // Enable Bus Master
                    BIT(6) |  // Enable Parity
                    BIT(8));  // Enable SERR#
    }
}


/*
 * pci_scan
 * --------
 * Strategy
 * 1. Discover: Depth-first search to locate devices, assign bus numbers,
 *    and determine total BAR sizes.
 * 2. Allocate: Assign BARs based on buses with largest sizes first.
 * 3. Enable: Enable I/O and memory space on all devices.
 */
static void
pci_scan(void)
{
    if (pci_bridge_is_present() == 0) {
        printf("No PCI bridge located\n");
        return;
    }
    pci_bridge_control(0, -1, -1, -1, FLAG_BRIDGE_RESET);
    if (bridge_type != BRIDGE_TYPE_AMIGAPCI)
        firestorm_mode = 1;
    if (firestorm_mode) {
        /* Prometheus / Firestorm */
        pci_alloc_mem_base = 0x00000000;
        pci_alloc_mem_max  = 0x1fc00000;
        /* config space starts after this point */
    } else {
        /* AmigaPCI */
        pci_alloc_mem_base     = 0x80000000;
        pci_alloc_mem_max      = 0x9fc00000;
        pci_alloc_mem_pre_base = 0xa0000000;
        pci_alloc_mem_pre_max  = 0xff000000;
    }
    pci_alloc_io_base  = 0x00000000;
    pci_alloc_io_max   = 0x00200000;  // Firestorm (0x20000000 - 0x1fe00000)
    pci_discover(0, &pci_root_dev);
    pci_allocate(&pci_root_dev);
    pci_enable(&pci_root_dev);
}

/*
 * free_pci_tree
 * -------------
 * Deallocate PCI tree structure
 */
static void
free_pci_tree(pci_dev_t *cur)
{
    cur = cur->pd_child;
    while (cur != NULL) {
        pci_dev_t *temp = cur;
        if (cur->pd_child != NULL)
            free_pci_tree(cur);
        cur = cur->pd_next;
        FreeMem(temp, sizeof (*temp));
    }
}

/*
 * print_pci_tree
 * --------------
 * Display the PCI tree.
 */
static void
print_pci_tree(pci_dev_t *cur, uint indent)
{
    cur = cur->pd_child;
    while (cur != NULL) {
        uint rindent = 0;
        uint bar;
        if (indent < 10)
            rindent = 10 - indent;
        printf("%*s%x.%x.%x %*s%04x.%04x",
               indent, "" , cur->pd_bus, cur->pd_dev, cur->pd_func,
               rindent, "", cur->pd_vendor, cur->pd_device);

        uint is_bridge = (cur->pd_htype & 0x7F) == 1;
        uint max_bars = is_bridge ? 3 : 7;  // Bridge has only two BARs
        for (bar = 0; bar < max_bars; bar++) {
            uint bartype = cur->pd_bar_type[bar];
            uint addr = cur->pd_bar[bar];
            if (cur->pd_bar_size[bar] == 0)
                continue;
            printf(" ");
            if (bartype & BIT(7)) {
                printf("ROM=");
            } else if (bartype & BIT(0)) {
                printf("IO=");
            } else {
                if ((bartype & (BIT(2) | BIT(1))) == BIT(2)) {
                    /* 64-bit BAR */
                    bar++;
                    printf("M64=");
                    if (cur->pd_bar[bar] != 0)
                        printf("%x:", cur->pd_bar[bar]);
                } else {
                    printf("M32=");
                }
            }
            printf("%x", addr);
        }
        if (is_bridge)
            printf(" Bridge");
        printf("\n");
        if (cur->pd_child != NULL)
            print_pci_tree(cur, indent + 2);
        cur = cur->pd_next;
    }
}

static void
usage(void)
{
    printf("apciscan options\n"
           "   -f assign BARs as Firestorm would want them\n");
}

int
main(int argc, char *argv[])
{
    int arg;
    (void) argc;
    (void) argv;
    for (arg = 1; arg < argc; arg++) {
        char *ptr = argv[arg];
        if (*ptr == '-') {
            for (++ptr; *ptr != '\0'; ptr++) {
                switch (*ptr) {
                    case 'f':  // Firestorm mode
                        firestorm_mode = 1;
                        break;
                    default:
                        printf("Unknown argument %s\n", ptr);
                        usage();
                        exit(1);
                }
            }
        } else {
            printf("Error: unknown argument %s\n", ptr);
            usage();
            exit(1);
        }
    }

    pci_scan();

    printf("Tree%17s %08x\n", "", pci_root->pd_size);
    print_pci_tree(pci_root, 2);

    free_pci_tree(pci_root);
    pci_root = NULL;
    return (0);
}
