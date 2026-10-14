#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <exec/types.h>

#include <libraries/expansionbase.h>
#include <clib/exec_protos.h>
#include <clib/expansion_protos.h>
#include <inline/expansion.h>
#include <proto/dos.h>
#include "util.h"
#include "cpu_control.h"
#include "pci_access.h"

#ifndef BIT
#define BIT(x)        (1 << (x))
#endif

#define swap16(x) __builtin_bswap16(x)
#define swap32(x) __builtin_bswap32(x)

#define ZORRO_MFG_MATAY         0xad47
#define ZORRO_PROD_MATAY_BD     0x0001
#define ZORRO_MFG_E3B           0x0e3b
#define ZORRO_PROD_FIRESTORM    0x00c8

#define FS_PCI_ADDR_CONFIG0     0x1fc00000  // Normal config space
#define FS_PCI_ADDR_CONFIG1     0x1fd00000  // Config1 space
#define FS_PCI_ADDR_IO          0x1fe00000
#define FS_PCI_ADDR_CONTROL     0x1fc08000  // Control (PCI reset) register

#define FS_PCI_CONTROL_NO_RESET 0x80000000  // 0=Bridge in reset
#define FS_PCI_CONTROL_EN_INTS  0x40000000  // 1=Interrupts enabled

#define MAYTAY_PCI_ADDR_CONFIG  0x000f0000

#define BRIDGE_TYPE_UNKNOWN   0
#define BRIDGE_TYPE_MAYTAY    1
#define BRIDGE_TYPE_FIRESTORM 2
#define BRIDGE_TYPE_AMIGAPCI  3

uint8_t *bridge_pci0_base;
uint8_t *bridge_pci1_base;
void    *bridge_io_base;
void    *bridge_mem_base;
uint32_t bridge_map_base;
static void    *bridge_control_reg;
uint8_t  bridge_type = BRIDGE_TYPE_UNKNOWN;
uint16_t bridge_zorro_mfg;
uint16_t bridge_zorro_prod;
struct ConfigDev *pci_zorro_cdev = NULL;

static const char * const expansion_library = "expansion.library";
struct Library *ExpansionBase;

/*
 * pci_find_root_bridge
 * --------------------
 * Finds the specified PCI root bridge by number.
 */
static void *
pci_find_root_bridge(uint bridge_num)
{
    struct ConfigDev    *cdev = NULL;
    uint8_t             *base = NULL;
    uint                 cur_bridge = 0;

    ExpansionBase = OpenLibrary((CONST_STRPTR) expansion_library, 0);
    if (ExpansionBase == NULL) {
        printf("Could not open %s\n", expansion_library);
        return (NULL);
    }
    bridge_zorro_mfg  = ZORRO_MFG_MATAY;
    bridge_zorro_prod = ZORRO_PROD_MATAY_BD;
    for (cur_bridge = 0; cur_bridge < 8; cur_bridge++) {
        cdev = FindConfigDev(cdev, bridge_zorro_mfg, bridge_zorro_prod);
        if (cdev == NULL)
            break;
        if (cur_bridge == bridge_num) {
            base = cdev->cd_BoardAddr;
            bridge_map_base = (uintptr_t) base;
            bridge_pci0_base = base + MAYTAY_PCI_ADDR_CONFIG;
            bridge_pci1_base = base + MAYTAY_PCI_ADDR_CONFIG;
            bridge_io_base = base;
            bridge_mem_base = base;
            bridge_control_reg = NULL;
            bridge_type = BRIDGE_TYPE_MAYTAY;
            goto done;
        }
    }
    bridge_zorro_mfg  = ZORRO_MFG_E3B;
    bridge_zorro_prod = ZORRO_PROD_FIRESTORM;
    for (; cur_bridge < 8; cur_bridge++) {
        cdev = FindConfigDev(cdev, bridge_zorro_mfg, bridge_zorro_prod);
        if (cdev == NULL)
            break;
        if (cur_bridge == bridge_num) {
            base = cdev->cd_BoardAddr;
            bridge_map_base  = (uintptr_t) base;
            bridge_pci0_base = base + FS_PCI_ADDR_CONFIG0;
            bridge_pci1_base = base + FS_PCI_ADDR_CONFIG1;
            bridge_io_base = base + FS_PCI_ADDR_IO;
            bridge_control_reg = base + FS_PCI_ADDR_CONTROL;
            if (base == ADDR8(0x80000000)) {
                bridge_type = BRIDGE_TYPE_AMIGAPCI;
                bridge_mem_base = 0;
            } else {
                bridge_type = BRIDGE_TYPE_FIRESTORM;
                bridge_mem_base = base;
            }
            break;
        }
    }

done:
    pci_zorro_cdev = cdev;
    CloseLibrary(ExpansionBase);

    return (base);
}

/*
 * pci_bridge_is_present
 * ---------------------
 * Returns non-zero if a PCI bridge is present in the system.
 */
int
pci_bridge_is_present(void)
{
    static uint8_t did_pci_init = 0;

    if (did_pci_init == 0) {
        if (pci_find_root_bridge(0) != NULL)
            return (1);
        did_pci_init = 1;
    }
    return (0);
}

/*
 * pci_cfg_base
 * ------------
 * Get the base address of configuration space for the specified PCI
 * bus, device, function, and register offset.
 */
void *
pci_cfg_base(uint bus, uint dev, uint func, uint off)
{
    if (pci_bridge_is_present() == 0)
        return (NULL);
    if (bus == 0) {
        if (bridge_pci0_base == NULL)
            return (NULL);
        if (dev <= 3)
            return (bridge_pci0_base + (0x10000 << dev) + (func << 8) + off);

        if ((dev == 4) && (bridge_type == BRIDGE_TYPE_AMIGAPCI)) {
            /* Only AmigaPCI has 4 slots */
            return (bridge_pci0_base + 0x30000 + (func << 8) + off);
        } else {
            return (bridge_pci0_base);  // Fail with no slot selected
        }
    }
    if ((bridge_pci1_base == NULL) || (bus > 15))
        return (NULL);
    return (bridge_pci1_base + (bus << 16) + (dev << 11) + (func << 8) + off);
}

/*
 * pci_read8
 * ---------
 * Read a 8-bit value from PCI configuration space.
 */
uint8_t
pci_read8(uint bus, uint dev, uint func, uint off)
{
    void *addr = pci_cfg_base(bus, dev, func, off);
    if (addr == NULL)
        return (0xff);
    return (*ADDR8(addr));
}

/*
 * pci_read16
 * ----------
 * Read a 16-bit value from PCI configuration space.
 */
uint16_t
pci_read16(uint bus, uint dev, uint func, uint off)
{
    void *addr = pci_cfg_base(bus, dev, func, off);
    if (addr == NULL)
        return (0xffff);
    return (swap16(*ADDR16(addr)));
}

/*
 * pci_read32
 * ----------
 * Read a 32-bit value from PCI configuration space.
 */
uint32_t
pci_read32(uint bus, uint dev, uint func, uint off)
{
    void *addr = pci_cfg_base(bus, dev, func, off);

    if (addr == NULL)
        return (0xffffffff);
    return (swap32(*ADDR32(addr)));
}

/*
 * pci_write8
 * ----------
 * Write a 8-bit value to PCI configuration space.
 */
void
pci_write8(uint bus, uint dev, uint func, uint off, uint8_t value)
{
    void *addr = pci_cfg_base(bus, dev, func, off);
    if (addr == NULL)
        return;
    *ADDR8(addr) = value;
}

/*
 * pci_write16
 * -----------
 * Write a 16-bit value to PCI configuration space.
 */
void
pci_write16(uint bus, uint dev, uint func, uint off, uint16_t value)
{
    void *addr = pci_cfg_base(bus, dev, func, off);
    if (addr == NULL)
        return;
    *ADDR16(addr) = swap16(value);
}

/*
 * pci_write32
 * -----------
 * Write a 32-bit value to PCI configuration space.
 */
void
pci_write32(uint bus, uint dev, uint func, uint off, uint32_t value)
{
    void *addr = pci_cfg_base(bus, dev, func, off);
    if (addr == NULL)
        return;

    *ADDR32(addr) = swap32(value);

    /*
     * The below will flush the write
     *
     * XXX: cdh: Why is this necessary with Firestorm and maybe others?
     *      If the following read is not done, then a subsequent read
     *      will return the just written data rather than actual data
     *      from the PCI config register.
     */
    (void) *VADDR32(addr + 0x100);
}

/*
 * pci_read32v
 * -----------
 * Convenience function to read a 32-bit value from PCI configuration space,
 * filtering out bad values returned when the previous PCI access timeout
 * is still pending.
 */
uint32_t
pci_read32v(uint bus, uint dev, uint func, uint off)
{
    void    *addr = pci_cfg_base(bus, dev, func, off);
    uint32_t rval;
    uint32_t nval;
    uint     diffs = 0;

    if (addr == NULL)
        return (0xffffffff);

    rval = *ADDR32(addr);
    CacheClearE((void *)addr, 4, CACRF_ClearD);  // Work around 68030 bug
    while (rval != (nval = *ADDR32(addr))) {
        rval = nval;
        if (++diffs > 5)
            break;
        CacheClearE((void *)addr, 4, CACRF_ClearD);  // Work around 68030 bug
    }
    return (swap32(rval));
}

/*
 * pci_write32v
 * ------------
 * Convenience function to write a 32-bit value to PCI configuration space,
 * verifying that the write was successful.
 */
void
pci_write32v(uint bus, uint dev, uint func, uint off, uint32_t wval)
{
    void    *addr = pci_cfg_base(bus, dev, func, off);
    uint     diffs = 0;

    wval = swap32(wval);
    do {
        *ADDR32(addr) = wval;
        CacheClearE((void *)addr, 4, CACRF_ClearD);  // Work around 68030 bug
        if (*ADDR32(addr) == wval)
            break;
    } while (diffs++ < 5);
}

/*
 * pci_read
 * --------
 * Function to read any quantity of bytes from PCI configuration space.
 * This is a convenience function for APIs which just specify a byte count.
 */
uint32_t
pci_read(uint bus, uint dev, uint func, uint offset, uint mode)
{
    if (pci_bridge_is_present() == 0)
        return (0xffffffff);

    switch (mode) {
        case 1:
            return (pci_read8(bus, dev, func, offset));
        case 2:
            return (pci_read16(bus, dev, func, offset));
        default:
        case 4:
            return (pci_read32(bus, dev, func, offset));
    }
}

/*
 * pci_write
 * ---------
 * Function to write any quantity of bytes to PCI configuration space.
 * This is a convenience function for APIs which just specify a byte count.
 */
void
pci_write(uint bus, uint dev, uint func, uint offset, uint mode, uint32_t value)
{
    if (pci_bridge_is_present() == 0)
        return;

    switch (mode) {
        case 1:
            pci_write8(bus, dev, func, offset, value);
            break;
        case 2:
            pci_write16(bus, dev, func, offset, value);
            break;
        case 4:
            pci_write32(bus, dev, func, offset, value);
            break;
    }
}

/*
 * pci_amiga_read
 * --------------
 * Function to read 1, 2, or 4 bytes from Amiga PCI configuration space.
 * This is a convenience function for APIs which just specify a byte count.
 */
static void
pci_amiga_read(uint bus, uint dev, uint func, uint offset, uint mode,
               void *value)
{
    switch (mode) {
        case 1:
            *ADDR8(value) = pci_read8(bus, dev, func, offset);
            break;
        case 2:
            *ADDR16(value) = pci_read16(bus, dev, func, offset);
            break;
        case 4:
            *ADDR32(value) = pci_read32(bus, dev, func, offset);
            break;
    }
}

/*
 * pci_amiga_write
 * ---------------
 * Function to write 1, 2, or 4 bytes to Amiga PCI configuration space.
 * This is a convenience function for APIs which just specify a byte count.
 */
static void
pci_amiga_write(uint bus, uint dev, uint func, uint offset, uint mode,
                void *value)
{
    switch (mode) {
        case 1:
            pci_write8(bus, dev, func, offset, *ADDR8(value));
            break;
        case 2:
            pci_write16(bus, dev, func, offset, *ADDR16(value));
            break;
        case 4:
            pci_write32(bus, dev, func, offset, *ADDR32(value));
            break;
    }
}

/*
 * pci_read_buf
 * ------------
 * Function to read any quantity of bytes from PCI configuration space.
 * This is a convenience function for APIs which just specify a byte count.
 */
rc_t
pci_read_buf(uint bus, uint dev, uint func, uint offset, uint width, void *bufp)
{
    rc_t rc = RC_SUCCESS;

    if (pci_bridge_is_present() == 0)
        return (RC_NO_DATA);

    while (width > 0) {
        uint len = width;
        if (offset & 1)
            len = 1;
        else if (len == 3)
            len = 2;
        else if (len > 4)
            len = 4;

        pci_amiga_read(bus, dev, func, offset, len, bufp);
        width  -= len;
        offset += len;
        bufp = (void *)((uintptr_t)bufp + len);
    }
    return (rc);
}

/*
 * pci_write_buf
 * -------------
 * Function to write any quantity of bytes to PCI configuration space.
 * This is a convenience function for APIs which just specify a byte count.
 */
rc_t
pci_write_buf(uint bus, uint dev, uint func, uint offset, uint width,
              void *bufp)
{
    rc_t rc = RC_SUCCESS;

    if (pci_bridge_is_present() == 0)
        return (RC_NO_DATA);

    while (width > 0) {
        uint len = width;
        if (offset & 1)
            len = 1;
        else if (len == 3)
            len = 2;
        else if (len > 4)
            len = 4;
        pci_amiga_write(bus, dev, func, offset, len, bufp);
        width  -= len;
        offset += len;
        bufp = (void *)((uintptr_t)bufp + len);
    }
    return (rc);
}

/*
 * pci_bridge_control
 * ------------------
 * Function to control the specified PCI bridge. Currently only reset of
 * the downstream devices is supported.
 */
void
pci_bridge_control(int pci_bridge, int bus, int dev, int func, uint flags)
{
    int bridge_num;
    if ((bus != -1) && (dev != -1) && (func != -1)) {
        uint32_t val = pci_read32(bus, dev, func, PCI_OFF_REVISION);
        if ((val >> 16) != PCI_CLASS_PCI_BRIDGE) {
            printf("PCI %x.%x.%x is not a bridge\n", bus, dev, func);
            return;
        }
        printf("PCI reset subordinate of %x.%x.%x\n", bus, dev, func);
        val = pci_read16(bus, dev, func, PCI_OFF_BR_CONTROL);

        /* Force reset */
        val |= PCI_BRIDGE_CTL_BUS_RESET;
        pci_write16(bus, dev, func, PCI_OFF_BR_CONTROL, val);
        Delay(1);

        /* Release reset */
        val &= ~PCI_BRIDGE_CTL_BUS_RESET;
        pci_write16(bus, dev, func, PCI_OFF_BR_CONTROL, val);
        return;
    }
    for (bridge_num = 0; bridge_num < PCI_MAX_BUS; bridge_num++) {
        if ((pci_bridge != -1) && (pci_bridge != bridge_num))
            continue;
        if (pci_find_root_bridge(bridge_num) == NULL) {
            if (pci_bridge != -1)
                printf("Could not find bridge %d\n", pci_bridge);
            break;
        }
        if (bridge_control_reg == NULL) {
            if (pci_bridge != -1) {
                printf("Bridge %d does not have reset capability\n",
                       pci_bridge);
            }
            continue;
        }
        if (flags & FLAG_BRIDGE_RESET_HOLD) {
            *ADDR32(bridge_control_reg) &= ~FS_PCI_CONTROL_NO_RESET;
        }
        if (flags & FLAG_BRIDGE_RESET) {
            *ADDR32(bridge_control_reg) &= ~FS_PCI_CONTROL_NO_RESET;
            Delay(1);   // 20 ms
            *ADDR32(bridge_control_reg) |= FS_PCI_CONTROL_NO_RESET |
                                           FS_PCI_CONTROL_EN_INTS;
            Delay(15);  // 300 ms
        }
    }
}

static uint
is_multifunction_device(uint bus, uint dev, uint func)
{
    uint32_t val;
    uint8_t htype;
    val = pci_read32(bus, dev, func, PCI_OFF_HEADERTYPE & ~3);
    htype = val >> 16;
    if (((htype & BIT(7)) == 0) || (val == 0xffffffff))
        return (0);  // Not a multifunction device
    return (1);
}

int
pci_scan_cb(pci_scan_cb_t callback)
{
    uint    bus;
    uint    dev;
    uint    func;
    uint    maxbus = PCI_MAX_BUS;

    if (pci_bridge_is_present() == 0)
        return (1);

    for (bus = 0; bus <= maxbus; bus++) {
        uint maxdev = PCI_MAX_DEV;  // PCI supports 32 devs per bus
        if (bus == 0)
            maxdev = PCI_MAX_PHYS_SLOT;  // 5 physical slots

        for (dev = 0; dev < maxdev; dev++) {
            for (func = 0; func < PCI_MAX_FUNC; func++) {
                uint32_t vd;
                uint16_t vendor;
                uint16_t device;

                vd = pci_read32v(bus, dev, func, PCI_OFF_VENDOR);
                if ((vd == 0xffffffff) || (vd == 0x00000000)) {
                    if (func == 0) {
                        if ((bus > 0) && (dev == 0) && (maxbus == PCI_MAX_BUS))
                            maxbus = bus + 1;  // Just probe 1 more bus
                        break;
                    }

                    goto skip_and_check_htype;
                }
                vendor = (uint16_t) vd;
                device = vd >> 16;
                if (callback(bus, dev, func, vendor, device))
                    return (0);
skip_and_check_htype:
                if ((func == 0) && !is_multifunction_device(bus, dev, func))
                    break;  // Not a multifunction device
            }
        }
    }
    return (0);
}
