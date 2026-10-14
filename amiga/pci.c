/*
 * PCI    Version 0.2 2025-11-11
 * -----------------------------
 * Utility to inspect Amiga Firebird PCI configuration space.
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

const char *version = "\0$VER: PCI "VERSION" ("BUILD_DATE") � Chris Hooper";

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <clib/expansion_protos.h>
#include "util.h"
#include "pci_access.h"
#include "cpu_control.h"
#include "strtox.h"

#define ARRAY_SIZE(x)  ((sizeof (x) / sizeof ((x)[0])))
#define BIT(x)         (1U << (x))

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

#define FLAG_DUMP               0x00000001  // Raw dump
#define FLAG_DDUMP              0x00000002  // Dump all 256 bytes
#define FLAG_VERBOSE            0x00000004  // Verbose display of all registers
#define FLAG_VERBOSE_IO         0x00000008  // Also display I/O range in tree
#define FLAG_NO_TRANSLATE       0x00000010  // No translate to CPU address
#define FLAG_SHOW_CADDR         0x00000020  // Show device config base address
#define FLAG_PCI_LS             0x00000040  // Do ls display
#define FLAG_PCI_STATUS         0x00000080  // Show PCI device error status
#define FLAG_PCI_CLEAR          0x00000100  // Clear PCI device error status
#define FLAG_PCI_STATUS_ALL     0x00000200  // Display all registers (not used)
#define FLAG_PCI_CAP            0x00000400  // Get specific capability
#define FLAG_PCI_ECAP           0x00000800  // Get specific extended capability
#define FLAG_PCI_TREE           0x00001000  // Show tree view

#define PCI_ANY_ID              0xffffffff

/* PCIe packet header */
#define TLP_FORMAT_64BIT_ADDR           0x01 // 64-bit address specified
#define TLP_FORMAT_WRITE                0x02 // Write specified
#define TLP_FORMAT_COMPL_WDATA          0x02 // Completion with data
#define TLP_TYPE_MEMORY                 0x00 // Memory access
#define TLP_TYPE_LOCKED_READ            0x01 // Locked memory read access
#define TLP_TYPE_IO                     0x02 // I/O access
#define TLP_TYPE_CONFIG_0               0x04 // Config type 0 access
#define TLP_TYPE_CONFIG_1               0x05 // Config type 1 access
#define TLP_TYPE_COMPLETION             0x0a // Completion
#define TLP_TYPE_LOCKED_COMPL           0x0b // Locked Completion
#define TLP_TYPE_MESSAGE                0x10 // Message

/* Offsets within a PCI header */
#define PCI_ERR_HEADER_LOG	28	/* Header Log Register (16 bytes) */


/* PCI error status */
#define PCI_STATUS_ERRORS (PCI_STATUS_PARITY         | \
                           PCI_STATUS_SIG_TGT_ABORT  | \
                           PCI_STATUS_REC_TGT_ABORT  | \
                           PCI_STATUS_REC_MSTR_ABORT | \
                           PCI_STATUS_SIG_SYS_ERROR  | \
                           PCI_STATUS_DET_PARITY)

#define PCIE_STATUS_ERRORS (PCI_EXP_DEVSTA_CED  | \
                            PCI_EXP_DEVSTA_NFED | \
                            PCI_EXP_DEVSTA_FED  | \
                            PCI_EXP_DEVSTA_URD)

#define PCIE_LINK_ERRORS   (PCI_EXP_LNKSTA_LT   | \
                            PCI_EXP_LNKSTA_LBMS | \
                            PCI_EXP_LNKSTA_LABS)

#define PCIE_LINK_ERRORS2  (PCI_EXP_LNKSTA2_EQRST)

#define SIZE_1MB (1U << 20)

typedef unsigned int uint;
typedef const char * const bits_t;

extern struct ExecBase *SysBase;
static uint flags;

/* PCI Command (0x4) */
static bits_t bits_pci_command[] = {
    "En_IO", "En_Memory", "En_BusMaster", "En_Special",
    "En_MWI", "En_Snoop", "En_Parity", "En_AddrStep",
    "En_SERR#", "En_FastBtoB", "", "",
    "", "", "", "",
};

/* PCI Primary Status (0x6) */
static bits_t bits_pci_status_primary[] = {
    "", "", "",
    "Int Pending",
    "Cap List",
    "66 MHz Capable",
    "UDF Support",
    "Fast Back-to-Back Capable",
    "Master Data Parity Error",
    NULL,
    "DEVSEL:0=fast,1=medium,2=slow,3=reserved",
    "Signaled Target Abort",
    "Received Target Abort",
    "Received Master Abort",
    "Signaled System Error",
    "Detected Parity Error",
};

/** PCI Bridge Secondary Status (0x1e). */
static bits_t bits_pci_status_secondary[] = {
    "", "", "", "",
    "", "", "", "",
    "Master Data Parity Error",
    "",
    "",
    "Signaled Target Abort",
    "Received Target Abort",
    "Received Master Abort",
    "Received System Error",
    "Detected parity Error",
};

/* PCI Max_Lat, Min_Gnt, Interrupt Pin, Interrupt Line (0x3c) */
static bits_t bits_pci_lat_gnt_int[] = {
    NULL, NULL, NULL, NULL,
    NULL, NULL, NULL, "Line",
    NULL, NULL, NULL, NULL,
    NULL, NULL, NULL, "Pin:1=INTA#,2=INTB#,3=INTC#,4=INTD#",
};

static void
zorro_show_mfgprod(void)
{
    if (bridge_type == BRIDGE_TYPE_AMIGAPCI) {
        printf(" AmigaPCI");
    } else if (bridge_zorro_mfg == ZORRO_MFG_MATAY) {
        printf(" Matay");
        if (bridge_zorro_prod == ZORRO_PROD_MATAY_BD)
            printf(" Prometheus");
    } else if (bridge_zorro_mfg == ZORRO_MFG_E3B) {
        printf(" E3B");
        if (bridge_zorro_prod == ZORRO_PROD_FIRESTORM)
            printf(" Firestorm");
    } else {
        printf("Unknown");
    }
}

static const struct {
    uint16_t          pv_vendor;
    const char *const pv_name;
} pci_vendors[] = {
    { 0x1025, "Acer"          },
    { 0x9004, "Adaptec"       },
    { 0x9005, "Adaptec"       },
    { 0x1317, "ADMtek"        },
    { 0x12ae, "Alteon"        },
    { 0x1172, "Altera"        },
    { 0x1022, "AMD"           },
    { 0x1b21, "ASMedia"       },
    { 0x1a03, "ASPEED"        },
    { 0x1002, "ATI"           },
    { 0x168c, "Atheros"       },
    { 0x1969, "Attansic"      },
    { 0x14e4, "Broadcom"      },
    { 0x1166, "Broadcom"      },
    { 0x1013, "Cirrus"        },
    { 0x1011, "DEC"           },
    { 0x1028, "Dell"          },
    { 0x1092, "Diamond"       },
    { 0x10b7, "3Com"          },
    { 0x0e11, "Compaq"        },
    { 0x102c, "Chips&Tech"    },
    { 0x121a, "3Dfx"          },
    { 0x3d3d, "3DLabs"        },
    { 0x10df, "Emulex"        },
    { 0x1274, "Ensoniq"       },
    { 0x13a8, "Exar"          },
    { 0x103c, "HP"            },
    { 0x1014, "IBM"           },
    { 0x111d, "IDT"           },
    { 0x1283, "IntegratedTech" },
    { 0x8086, "Intel"         },
    { 0x197b, "JMicron"       },
    { 0x1000, "LSI"           },
    { 0x102a, "LSI"           },
    { 0x11ab, "Marvell"       },
    { 0x1b4b, "Marvell"       },
    { 0x102b, "Matrox"        },
    { 0x1033, "NEC"           },
    { 0x10de, "NVIDIA"        },
    { 0x1415, "Oxford"        },
    { 0x12d8, "Pericom"       },
    { 0x10b5, "PLX"           },
    { 0x11f8, "PMC-Sierra"    },
    { 0x104a, "Promise"       },
    { 0x1045, "OPTi"          },
    { 0x1077, "QLogic"        },
    { 0x10ec, "Realtek"       },
    { 0x1912, "Renesas"       },
    { 0x17d5, "S2IO"          },
    { 0x5333, "S3"            },
    { 0x144d, "Samsung"       },
    { 0x1095, "SiliconImage"  },
    { 0x1039, "SiS"           },
    { 0x104c, "TI"            },
    { 0x1023, "Trident"       },
    { 0x1106, "VIA"           },
    { 0x10ee, "Xilinx"        },
};

static const struct {
    uint16_t          pv_vendor;
    uint16_t          pv_device;
    const char *const pv_name;
} pci_devices[] = {
    { 0x121a, 0x0003, "Banshee 16MB" },
    { 0x121a, 0x0005, "Voodoo3" },
    { 0x121a, 0x0036, "Voodoo3 2000" },
    { 0x121a, 0x0057, "Voodoo3 3000" },
    { 0x121a, 0x0009, "Voodoo4/5" },
    { 0x5333, 0x5631, "Virge DX 86C375" },
    { 0x5333, 0x8a01, "Virge 86C325" },
};

static const struct {
    uint32_t          pc_class_value;
    const char *const pc_name;
} pci_classes[] = {
    { 0x000000, "Legacy" },
    { 0x000100, "VGA" },
    { 0x010000, "SCSI" },
    { 0x010100, "IDE" },
    { 0x010200, "Floppy" },
    { 0x010300, "IPI" },
    { 0x010400, "RAID" },
    { 0x010520, "ATA ADMA SS" },
    { 0x010530, "ATA ADMA Continuous" },
    { 0x010600, "SATA" },
    { 0x010601, "SATA AHCI" },
    { 0x010602, "SATA SSBI" },
    { 0x010700, "SAS" },
    { 0x010701, "SAS SSBI" },
    { 0x010800, "NVM" },
    { 0x010801, "NVM HCI" },
    { 0x010802, "NVMe" },
    { 0x018000, "Other Mass Storage" },
    { 0x020000, "Ethernet" },
    { 0x020100, "Token Ring" },
    { 0x020200, "FDDI" },
    { 0x020300, "ATM" },
    { 0x020400, "ISDN" },
    { 0x020500, "WordFip" },
    { 0x020600, "PICMG" },
    { 0x028000, "Other Network" },
    { 0x030000, "VGA" },
    { 0x030001, "XGA" },
    { 0x030002, "3D" },
    { 0x038000, "Other display" },
    { 0x040000, "Video" },
    { 0x040100, "Audio" },
    { 0x040200, "Telephony" },
    { 0x040300, "HD Audio" },
    { 0x048000, "Other Multimedia" },
    { 0x050000, "RAM" },
    { 0x050100, "Flash" },
    { 0x058000, "Other Memory" },
    { 0x060000, "Host bridge" },
    { 0x060100, "ISA bridge" },
    { 0x060200, "EISA bridge" },
    { 0x060300, "MCA bridge" },
    { 0x060400, "PCI bridge" },
    { 0x060401, "PCI SubDecode bridge" },
    { 0x060500, "PCMCIA bridge" },
    { 0x060600, "NuBus bridge" },
    { 0x060700, "CardBus bridge" },
    { 0x060800, "RACEway bridge" },
    { 0x060940, "Semi-transparent bridge" },
    { 0x060980, "Semi-transparent reverse bridge" },
    { 0x060a00, "InfiniBand-PCI bridge" },
    { 0x060b00, "ASI-PCI bridge" },
    { 0x060b01, "ASI-PCI portal bridge" },
    { 0x068000, "Other bridge" },
    { 0x070000, "Serial" },
    { 0x070001, "16450 Serial" },
    { 0x070002, "16550 Serial" },
    { 0x070003, "16650 Serial" },
    { 0x070004, "16750 Serial" },
    { 0x070005, "16850 Serial" },
    { 0x070006, "16950 Serial" },
    { 0x070100, "Parallel" },
    { 0x070101, "BiDir Parallel" },
    { 0x070102, "ECP 1.x Parallel" },
    { 0x070103, "IEEE1284" },
    { 0x0701fe, "IEEE1284 target" },
    { 0x070200, "Multiport serial" },
    { 0x070300, "Modem" },
    { 0x070301, "Hayes 16450 model" },
    { 0x070302, "Hayes 16550 model" },
    { 0x070303, "Hayes 16650 model" },
    { 0x070304, "Hayes 16750 model" },
    { 0x070400, "GPIB IEEE488.1/2" },
    { 0x070500, "Smart Card" },
    { 0x078000, "Communication" },
    { 0x080000, "8259 PIC" },
    { 0x080001, "ISA PIC" },
    { 0x080002, "EISA PIC" },
    { 0x080010, "IOAPIC" },
    { 0x080020, "IOxAPIC" },
    { 0x080100, "8237 DMA" },
    { 0x080101, "ISA DMA" },
    { 0x080102, "EISA DMA" },
    { 0x080200, "8254 timer" },
    { 0x080201, "ISA timer" },
    { 0x080202, "EISA timers" },
    { 0x080203, "HP timer" },
    { 0x080300, "RTC" },
    { 0x080301, "ISA RTC" },
    { 0x080400, "PCI HP Controller" },
    { 0x080500, "SD Host" },
    { 0x080600, "IOMMU" },
    { 0x088000, "Other system peripheral" },
    { 0x090000, "Keyboard" },
    { 0x090100, "Digitizer" },
    { 0x090200, "Mouse" },
    { 0x090300, "Scanner" },
    { 0x090400, "Gameport" },
    { 0x098000, "Input Controller" },
    { 0x0a0000, "Docking Station" },
    { 0x0b0000, "386" },
    { 0x0b0100, "486" },
    { 0x0b0200, "Pentium" },
    { 0x0b1000, "Alpha" },
    { 0x0b2000, "PowerPC" },
    { 0x0b3000, "MIPS" },
    { 0x0b4000, "Co-Processor" },
    { 0x0b8000, "Other Processor" },
    { 0x0c0000, "1394 Firewire" },
    { 0x0c0010, "1394 Firewire OHCI" },
    { 0x0c0100, "ACCESS.bus" },
    { 0x0c0200, "SSA" },
    { 0x0c0300, "USB UHCI" },
    { 0x0c0301, "USB OHCI" },
    { 0x0c0310, "USB2 OHCI" },
    { 0x0c0320, "USB2 EHCI" },
    { 0x0c0380, "USB2" },
    { 0x0c03fe, "USB device" },
    { 0x0c0400, "Fibre Channel" },
    { 0x0c0500, "SMBus" },
    { 0x0c0600, "InfiniBand" },
    { 0x0c0700, "IPMI SMIC" },
    { 0x0c0701, "IPMI Keyboard" },
    { 0x0c0702, "IPMI Block Device" },
    { 0x0c0800, "SERCOS" },
    { 0x0c0900, "CANbus" },
    { 0x0c8000, "Serial Bus" },
    { 0x0d0000, "iRDA" },
    { 0x0d0100, "Consumer IR" },
    { 0x0d0110, "UWB Radio" },
    { 0x0d1000, "Wireless" },
    { 0x0d1100, "Bluetooth" },
    { 0x0d1200, "Broadband" },
    { 0x0d2000, "Ethernet 5 GHz" },
    { 0x0d2100, "Ethernet 2.4 GHz" },
    { 0x0d8000, "Other Wireless" },
    { 0x0e0000, "IIO" },
    { 0x0f0100, "Satt. TV" },
    { 0x0f0200, "Satt. Audio" },
    { 0x0f0300, "Satt. Voice" },
    { 0x0f0400, "Satt. Data" },
    { 0x0f8000, "Other Satt." },
    { 0x100000, "Network/compute crypto" },
    { 0x101000, "Entertainment crypto" },
    { 0x108000, "Other crypto" },
    { 0x110000, "DPIO" },
    { 0x110100, "Performance Counter" },
    { 0x111000, "Comm Sync" },
    { 0x112000, "Management" },
    { 0x118000, "Misc Acq/Signal Proc" },
};

const char *
pci_vendor(uint16_t vendor)
{
    uint pos;
    for (pos = 0; pos < ARRAY_SIZE(pci_vendors); pos++) {
        if (pci_vendors[pos].pv_vendor == vendor) {
            return (pci_vendors[pos].pv_name);
        }
    }
    return ("Unknown");
}

static void
pci_show_vendordevice(uint16_t vendor, uint16_t device)
{
    uint pos;
    for (pos = 0; pos < ARRAY_SIZE(pci_vendors); pos++)
        if (pci_vendors[pos].pv_vendor == vendor)
            break;

    if (pos >= ARRAY_SIZE(pci_vendors))
        return;

    printf(" %s", pci_vendors[pos].pv_name);

    for (pos = 0; pos < ARRAY_SIZE(pci_devices); pos++)
        if ((pci_devices[pos].pv_vendor == vendor) &&
            (pci_devices[pos].pv_device == device)) {
            printf(" %s", pci_devices[pos].pv_name);
            return;
        }

    printf(" %04x", device);
}

static void
pci_show_class(uint32_t classrev)
{
    uint        pos;
    uint        class_value = (uint) (classrev >> 8);
    const char *pc_base    = NULL;

    for (pos = 0; pos < ARRAY_SIZE(pci_classes); pos++) {
        uint cv = pci_classes[pos].pc_class_value;
        if (((cv & 0xff) == 0) &&
            ((class_value >> 8) == (cv >> 8))) {
            pc_base = pci_classes[pos].pc_name;
        }
        if (class_value == cv) {
            printf(" %s", pci_classes[pos].pc_name);
            return;
        }
    }

    printf(" %s", pc_base);
}

static const struct {
    uint8_t     dwords;
    const char *name;
} pci_caps[] = {
    [0x00] = { 0x01, "NULL" },
    [0x01] = { 0x02, "Power Management" },
    [0x02] = { 0x01, "Accelerated Graphics Port (AGP)" },
    [0x03] = { 0x02, "Vital Product Data (VPD)" },
    [0x04] = { 0x01, "Slot Identification" },
    [0x05] = { 0x03, "Message Signaled Interrupts (MSI)" },
    [0x06] = { 0x01, "CompactPCI Hotswap" },
    [0x07] = { 0x02, "PCI-X" },
    [0x08] = { 0x01, "HyperTransport" },
    [0x09] = { 0x08, "Vendor-specific" },
    [0x0a] = { 0x01, "Debug Port" },
    [0x0b] = { 0x01, "CompactPCI Central Resource" },
    [0x0c] = { 0x01, "Standard Hotplug Controller" },
    [0x0d] = { 0x02, "Bridge subsystem vendor/device ID" },
    [0x0e] = { 0x01, "AGP Target bridge" },
    [0x0f] = { 0x01, "Secure Device" },
    [0x10] = { 0x0d, "PCI Express" },
    [0x11] = { 0x03, "MSI-X" },
    [0x12] = { 0x02, "SATA Data/Index Config" },
    [0x13] = { 0x02, "Advanced Features" },
    [0x14] = { 0x08, "Enhanced Allocation" },
    [0x15] = { 0x09, "Flattening Portal Bridge" },
};

static const struct {
    uint        dwords;
    const char *name;
} pcie_caps[] = {
    [0x00] = { 0x01, "NULL" },
    [0x01] = { 0x0b, "Advanced Error Reporting (AER)" },
    [0x02] = { 0x0d, "Virtual Channel (VC)" },
    [0x03] = { 0x03, "Device Serial Number" },
    [0x04] = { 0x04, "Power Budgeting" },
    [0x05] = { 0x0c, "Root Complex Link Declaration" },
    [0x06] = { 0x03, "Root Complex Internal Link Ctrl" },
    [0x07] = { 0x03, "Root Complex Event Collector EP" },
    [0x08] = { 0x0a, "Multi-func Virtual Channel (MFVC)" },
    [0x09] = { 0x0a, "Virtual Channel (VC)" },
    [0x0a] = { 0x05, "Root Complex Reg. Block (RCRB)" },
    [0x0b] = { 0x05, "Vendor-specific (VSEC)" },
    [0x0c] = { 0x02, "Config Access Correlation (CAC)" },
    [0x0d] = { 0x03, "Access Control Services (ACS)" },
    [0x0e] = { 0x02, "Alternative Routing-ID (ARI)" },
    [0x0f] = { 0x02, "Addr Translation Services (ATS)" },
    [0x10] = { 0x10, "Single Root I/O Virt (SR-IOV)" },
    [0x11] = { 0x02, "Multi-Root I/O Virt (MR-IOV)" },
    [0x12] = { 0x0c, "Multicast" },
    [0x13] = { 0x04, "Page Request" },
    [0x14] = { 0x02, "AMD Reserved" },
    [0x15] = { 0x06, "Resizable Bar" },
    [0x16] = { 0x0c, "Dynamic Power Allocation (DPA)" },
    [0x17] = { 0x04, "TLP Processing Hints (TPH)" },
    [0x18] = { 0x03, "Latency Tolerance Reporting (LTR)" },
    [0x19] = { 0x13, "Secondary PCI Express" },
    [0x1a] = { 0x02, "Protocol Multiplexing (PMUX)" },
    [0x1b] = { 0x02, "PASID" },
    [0x1c] = { 0x01, "LN Requester" },
    [0x1d] = { 0x10, "DPC" },
    [0x1e] = { 0x05, "L1 PM Substates" },
    [0x1f] = { 0x03, "Precision Time Management (PTM)" },
    [0x21] = { 0x04, "FRS Queuing" },
    [0x22] = { 0x03, "Readiness Time Reporting" },
    [0x23] = { 0x05, "Designated Vendor-Specific (DVSEC)" },
    [0x24] = { 0x06, "VF Resizable BAR" },
    [0x25] = { 0x03, "Data Link Feature" },
    [0x26] = { 0x10, "Physical Layer 16 GT/s" },
    [0x27] = { 0x22, "Lane Margining" },
    [0x28] = { 0x08, "Hierarchy ID" },
    [0x29] = { 0x04, "Native PCIe Enclosure Management (NPEM)" },
    [0x2a] = { 0x10, "Physical Layer 32 GT/s" },
    [0x2b] = { 0x05, "Alternate Protocol" },
    [0x2c] = { 0x05, "SFI" },
    [0x2d] = { 0x07, "Shadow Functions" },
    [0x2e] = { 0x06, "Data Object Exchange" },
    [0x2f] = { 0x04, "Device 3" },
    [0x30] = { 0x0d, "Integrity.Data.Encrpt (IDE)" },
    [0x31] = { 0x10, "Physical Layer 64 GT/s" },
    [0x32] = { 0x0e, "Flit Logging" },
    [0x33] = { 0x08, "Flit Performance" },
    [0x34] = { 0x08, "Flit Error Injection" },
};

/* PCI Power Management Capabilities (PMC offset 0x2) */
static bits_t bits_pm_cap[] = {
    NULL, NULL, ":3=v1.2",
    "PME Clock req",
    "Bit4",
    "Device-Specific Init req",
    NULL, NULL,
    "Aux:0=0mA,1=55mA,2=100mA,3=160mA,4=220mA,5=270mA,6=320mA,7=375mA",
    "D1",
    "D2",
    "D0_PME",
    "D1_PME",
    "D2_PME",
    "D3hot_PME",
    "D3cold_PME",
};

/* PCI Power Management Control/Status Register (PMCSR offset 0x4) */
static bits_t bits_pm_csr[] = {
    NULL, NULL, "Powerstate:0=D0,1=D1,2=D2,3=D3", "Bit2",
    "No_Soft_Reset", "Bit4", "Bit5", "Bit6",
    "PME_En", NULL, NULL, NULL,
    "Data_Select:0=D0 Power,1=D1 Power,2=D2 Power,3=D3 Power,4=D4 Power,"
    "5=D5 Power,6=D6 Power,7=D7 Power,8=Cmn Logic Power", NULL,
    "Data_Scale:0=Unknown,1=0.1x Watts,2=0.01x Watts,3=0.001x Watts",
    "PME_Asserted",
};

/* PCI Power Management Bridge Support Extensions (PMCSR_BSE offset 0x6) */
static bits_t bits_pm_bse[] = {
    "Bit0", "Bit1", "Bit2", "Bit3", "Bit4", "Bit5",
    "D3hot:0=Remove power(B3),1=Stop clock(B2)",
    "Bus Power/Clock:0=Disabled,1=Enabled",
    NULL, NULL, NULL, NULL, NULL, NULL, NULL, "Data",
};

/* PCI-X Command Register (PCI-X capability offset 0x2) */
static bits_t bits_pcix_cmd[] = {
    "DParityRecovery", "RelaxedOrdering",
        NULL, "Max Read:0=512,1=1024,2=2048,3=4096",
    NULL, NULL, "Max Split:0=1,1=2,2=3,3=4,4=8,5=12,6=16,7=32", "Bit7",
    "Bit8", "Bit9", "Bit10", "Bit11",
    "Bit12", "Bit13", "Bit14", "Bit15",
};

/* PCI-X Status Register (PCI-X capability offset 0x4) */
static bits_t bits_pcix_status[] = {
    NULL, NULL, "Func", NULL,
        NULL, NULL, NULL, "Dev",
    NULL, NULL, NULL, NULL, NULL, NULL, NULL, "Bus",
    "64-bit capable", "Max:0=66MHz,1=133MHz",
        "Split Completion Discard", "Unexpected Split Completion",
    "Bridge", NULL, "Max Read:0=512,1=1024,2=2048,3=4096", NULL,
    NULL, "Max Split:0=1,1=2,2=3,3=4,4=8,5=12,6=16,7=32", NULL, NULL,
    "Max ADQ/CRead:0=8/1KB,1=16/2KB,2=32/4KB,3=64/8KB,"
    "4=128/16KB,5=256/32KB,6=512/64KB,7=1024/128KB",
    "Split Completion Error", "Bit30", "Bit31"
};

/* PCI Express Capabilities (offset 0x2) */
static bits_t bits_pcie_cap[] = {
    NULL, NULL, NULL, "Version",
    NULL, NULL, NULL, "Type:0=PCIe Endpoint,1=Legacy PCIe Endpoint,"
    "4=Root Port,5=Switch Upstream Port,6=Switch Downstream Port,"
    "7=PCIe-to-PCI bridge,8=PCI-to-PCIe bridge",
    "Has Slot",
    NULL, NULL, NULL, NULL, "MSI Vector:0=",
    "Bit14", "Bit15",
};

/* PCI Express Device Capabilities (offset 0x4) */
static bits_t bits_pcie_devcap[] = {
    NULL, NULL,
    "Max Payload_RO:0=128,1=256,2=512,3=1024,4=2048,5=4096,6=Rsvd(6),7=Rsvd(7)",
    NULL, "Phantom Bits",
    "Tag Field:0=5-bit, 1=8-bit",
    NULL, NULL, "Max L0s Latency:0=64ns,1=128ns,2=256ns,"
    "3=512ns,4=1us,5=2us,6=4us,7=unlimited",
    NULL, NULL, "Max L1s Latency:0=1us,1=2us,2=4us,"
    "3=8us,4=16us,5=32us,6=64us,7=unlimited",
    "",  // Undefined (was Attention Button depressed)
    "",  // Undefined (was Attention Indicator implemented)
    "",  // Undefined (was Power Indicator implemented)
    "",  // Role-Based Error Reporting -- all PCI 2.0+ devices report this.
    "Bit16",
    "Bit17",
    NULL, NULL, NULL, NULL, NULL, NULL, NULL, "Power Limit Value",
    NULL, "Power Limit Scale:0=1.0x,1=0.1x,2=0.01x,3=0.001x",
    "Function Reset supported",
    "Bit29",
    "Bit30",
    "Bit31",
};

/* PCI Express Device Control (offset 0x8) */
static bits_t bits_pcie_devcontrol[] = {
    "Report Correctable",
    "Report Non-Fatal",
    "Report Fatal",
    "Report Unsupported Request",
    "Relaxed Ordering",
    NULL, NULL,
    "Max Payload:0=128,1=256,2=512,3=1024,4=2048,5=4096,6=Rsvd(6),7=Rsvd(7)",
    "Extended Tag",
    "Phantom Functions",
    "Aux Power",
    "Snoop",
    NULL, NULL,
    "Max Read Size:0=128,1=256,2=512,3=1024,4=2048,5=4096,6=Rsvd(6),7=Rsvd(7)",
    "Bridge Config Retry",
};

/* PCI Express Link Capabilities (offset 0xc) */
static bits_t bits_pcie_linkcap[] = {
    NULL, NULL, NULL, "Max Link:1=2.5 GT/s,2=5 GT/s,3=8 GT/s",
    NULL, NULL, NULL, NULL, NULL,
    "Max Width:1=x1,2=x2,4=x4,8=x8,c=x12,10=x16,20=x32",
    NULL, "ASPM:0=,1=L0s,2=L1,3=L0s and L1",
    NULL, NULL, "L0s Exit Latency:0=<64ns,1=<128ns,2=<256ns,3=<512ns,"
    "4=<1us,5=<2us,6=<4ns,7=>4ns",
    NULL, NULL, "L1s Exit Latency:0=<1us,1=<2us,2=<4us,3=<u8s,4=<16us,"
    "5=<32us,6=<64us,7=>64us",
    "Allows CLKREQ#",
    "Surprise Down Notify Capable",
    "DLL Active Notify Capable",
    "Bandwidth Notify Capable",
    "ASPM Compliant",
    "Bit23",
    NULL, NULL, NULL, NULL, NULL, NULL, NULL, "Port",
};

/* PCI Express Link Control (offset 0x10) */
static bits_t bits_pcie_linkcontrol[] = {
    NULL, "ASPM:0=,1=Enable L0s,2=Enable L1,3=Enable L0 & L1",
    "Bit1",
    "Read Completion Bound:0=64,1=128",
    ":1=Link Disabled,0=Link Enabled",
    "Link Retrain",
    "Common Clock Config",
    "Extended Sync",
    "Clock Power Mgmt:0=Disable,1=Enable",
    "HW Auto Width:0=Enable,1=Disable",
    "Bandwidth Management IRQ:0=Disable,1=Enable",
    "Auto Bandwidth IRQ:0=Disable,1=Enable",
    "Bit12",
    "Bit13",
    "Bit14",
    "Bit15",
};

/* PCI Express Slot Capabilities (offset 0x14) */
static bits_t bits_pcie_slotcap[] = {
    "Attention Button Present",
    "Power Controller Present",
    "MRL Sensor Present",
    "Attention Indicator Present",
    "Power Indicator Present",
    "Hot-Plug Surprise Possible",
    "Hot-Plug Capable",
    NULL, NULL, NULL, NULL, NULL, NULL, NULL,
    "Power Limit:0=,f0=250W,f1=275W,f2=300W",
    NULL, "Power Scale:0=1.0x,1=0.1x,2=0.01x,3=0.001x",
    "EMech Interlock Present",
    "No HP Completed Support",
    NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
    "Slot",
};

/* PCI Express Slot Control (offset 0x18) */
static bits_t bits_pcie_slotcontrol[] = {
    "Enable Attention Button Detect",
    "Enable Power Fault Detect",
    "Enable MRL Sensor Detect",
    "Enable Presence Detect",
    "Enable Command Completed IRQ",
    "Enable Hot-Plug IRQ",
    NULL, "Attention LED:0=,1=On,2=Blink,3=",
    NULL, "Power LED:0=,1=On,2=Blink,3=",
    "Enable Power Controller",
    "Enable EMech Interlock",
    "Enable DLL Active State Changed IRQ",
    "Bit13",
    "Bit14",
    "Bit15",
};

/* PCI Express Root Control (offset 0x1c) */
static bits_t bits_pcie_root_control[] = {
    "Enable SERR on Correctable",
    "Enable SERR on Non-Fatal",
    "Enable SERR on Fatal",
    "Enable PME IRQ",
    "Enable CRS Software Visibility",
    "Bit5",
    "Bit6",
    "Bit7",
};

/* PCI Express Root Capabilities (offset 0x1e) */
static bits_t bits_pcie_rootcap[] = {
    "CSR Software Visible",
    "Bit1",
    "Bit2",
    "Bit3",
    "Bit4",
    "Bit5",
    "Bit6",
    "Bit7",
};

/* PCI Express Root Status (offset 0x20) */
static bits_t bits_pcie_root_status[] = {
    NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
    NULL, NULL, NULL, NULL, NULL, NULL, NULL, "PME Requester ID:0=",
    "PME Asserted",
    "PME Pending",
    "Bit18",
    "Bit19",
    "Bit20",
    "Bit21",
    "Bit22",
    "Bit23",
    "Bit24",
    "Bit25",
    "Bit26",
    "Bit27",
    "Bit28",
    "Bit29",
    "Bit30",
    "Bit31",
};

/* PCI Express Device Capabilities 2 (offset 0x24) */
static bits_t bits_pcie_devcap2[] = {
    NULL, NULL, NULL, "Supported Timeout Range:0=50us to 50ms,1=50us to 10ms,"
    "2=10ms to 250ms,3=50us to 250ms,4=250ms to 4s,6=10ms to 4s,"
    "7=50us to 4s,8=4s to 64s,e=10ms to 64s,f=50us to 64s",
    "Can Disable Completion Timeout",
    "Can Forward ARI",
    "Can Route AtomicOp",
    "Can Complete 32-bit AtomicOp",
    "Can Complete 64-bit AtomicOp",
    "Can Complete 128-bit CAS",
    "No RO-enabled PR-PR Passing",
    "Can LTR",
    NULL, "TPH Complete:0=,1=Supported,2=Rsvd,3=Extended",
    "Bit14",
    "Bit15",
    "Bit16",
    "Bit17",
    "Bit18",
    "Bit19",
    "Support Extended Fmt Field",
    "Support End-End TLP Prefix",
    NULL, "Max End-END TLP Prefixes:0=",
    "Bit24",
    "Bit25",
    "Bit26",
    "Bit27",
    "Bit28",
    "Bit29",
    "Bit30",
    "Bit31",
};

/* PCI Express Device Control 2 (offset 0x28) */
static bits_t bits_pcie_devcontrol2[] = {
    NULL, NULL, NULL, "Timeout Range:0=50us to 50ms,1=50us to 100us,"
    "2=1ms to 10ms,5=16ms to 55ms,6=64ms to 210ms,9=260ms to 900ms,"
    "10=1s to 3.5s,13=4s to 13s,14=17s to 64s",
    "Disable Completion Timeout",
    "Forward ARI",
    "Enable AtomicOp Requester",
    "Enable AtomicOp Egress Blocking",
    "Enable IDO Request",
    "Enable IDO Completion",
    "Enable LTR",
    "Bit11",
    "Bit12",
    NULL, "OBFF:0=,1=MSI-A,2=MSI-B,3=WAKE#",
    "Enable End-End TLP Blocking",
};

/* PCI Express Link Capabilities 2 (offset 0x2c) */
static bits_t bits_pcie_linkcap2[] = {
    "Bit0",
    "Support 2.5 GT/s",
    "Support 5.0 GT/s",
    "Support 8.0 GT/s",
    "Bit4",
    "Bit5",
    "Bit6",
    "Bit7",
    "Support Crosslink",
    "Bit9",
    "Bit10",
    "Bit11",
    "Bit12",
    "Bit13",
    "Bit14",
    "Bit15",
};

/* PCI Express Link Control 2 (offset 0x30) */
static bits_t bits_pcie_linkcontrol2[] = {
    NULL, NULL, NULL, "Target Link:0=,1=2.5 GT/s,2=5 GT/s,3=8 GT/s",
    "Enter Compliance",
    "HW Auto Speed",
    "Selectable De-emphasis",
    NULL, NULL, "Transmit Margin:0=",
    "Enter Modified Compliance",
    "Compliance SOS",
    NULL, NULL, NULL, "De-emphasis:0=-6dB,1=-3.5dB",
};

/* PCI Express TPH Requester Capability (offset 0x4) */
static bits_t bits_pcie_tph_cap[] = {
    "Can No ST Mode",
    "Can IRQ Vector Mode",
    "Can Dev Specific Mode",
    "Bit3",
    "Bit4",
    "Bit5",
    "Bit6",
    "Bit7",
    "Can Extended TPH Requester",
    NULL, "ST Table:0=,1=In TPH Capability,2=In MSI-X Table,3=Reserved",
    "Bit11",
    "Bit12",
    "Bit13",
    "Bit14",
    "Bit15",
    NULL, NULL, NULL, NULL,
    NULL, NULL, NULL, NULL,
    NULL, NULL, "ST Table Size",
    "Bit27",
    "Bit28",
    "Bit29",
    "Bit30",
    "Bit31",
};

/* PCI Express TPH Requester Capability (offset 0x8) */
static bits_t bits_pcie_tph_control[] = {
    NULL, NULL, "ST Mode:0=,1=IRQ Vector,2=Device Specific",
    "Bit3",
    "Bit4",
    "Bit5",
    "Bit6",
    "Bit7",
    NULL, "TPH Requester:0=,1=Enable,2=Reserved,3=Extended",
    "Bit10",
    "Bit11",
    "Bit12",
    "Bit13",
    "Bit14",
    "Bit15",
};

/* PCI Express Secondary PCIE Extended Link Control (offset 0x4) */
static bits_t bits_pcie_cap2_link_control3[] = {
    "Perform Equalization",
    "Enable Link Eq. IRQ",
    "Bit2",
    "Bit3",
    "Bit4",
    "Bit5",
    "Bit6",
    "Bit7",
};

/* PCI Express Secondary PCIE Lane Equalization (offset 0xc + lane * 2) */
static bits_t bits_pcie_cap2_lane_eq_control[] = {
    NULL, NULL, NULL, "DS Tx:0=-6/ps0,1=-3.5/ps0,"
        "2=-4.5/ps0,3=-2.5/ps0,4=0/ps0,5=0/ps2,6=0/ps2.5,"
        "7=-6/ps3.5,8=-3.5/ps3.5,9=0/ps3.5,0xa=-4.5/ps0",
    NULL, NULL, "DS Rx:0=-6,1=-7,2=-8,3=-9,4=-10,"
        "5=-11,6=-12,7=Reserved(7)",
    "Bit7",
    NULL, NULL, NULL, "US Tx:0=-6/ps0,1=-3.5/ps0,"
        "2=-4.5/ps0,3=-2.5/ps0,4=0/ps0,5=0/ps2,6=0/ps2.5,"
        "7=-6/ps3.5,8=-3.5/ps3.5,9=0/ps3.5,0xa=-4.5/ps0",
    NULL, NULL, "US Rx:0=-6,1=-7,2=-8,3=-9,4=-10,"
        "5=-11,6=-12,7=Reserved(7)",
    "Bit15",
};

/* Access Control Services Capability (offset 0x4) */
static bits_t bits_pcie_acs_cap[] = {
    "Source Validation",
    "Translation Blocking",
    "P2P Request Redirect",
    "P2P Completion Redirect",
    "Upstream Forwarding",
    "P2P Egress Control",
    "Direct Translated P2P",
    "Bit7",
    NULL, NULL, NULL, NULL, NULL, NULL, NULL, "Egress Control Vector Size:0=",
};

/* Access Control Services Control (offset 0x6) */
static bits_t bits_pcie_acs_ctl[] = {
    "Source Validation",
    "Translation Blocking",
    "P2P Request Redirect",
    "P2P Completion Redirect",
    "Upstream Forwarding",
    "P2P Egress Control",
    "Direct Translated P2P",
    "Bit7",
};

/* Access Control Services Egress Control Vector (offset 0x8) */
static bits_t bits_pcie_acs_ecv[] = {
    NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
    NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
    NULL, NULL, NULL, NULL, NULL, NULL, NULL, "Egress Control Vector:0=",
    "Bit24",
    "Bit25",
    "Bit26",
    "Bit27",
    "Bit28",
    "Bit29",
    "Bit30",
    "Bit31",
};

/* PCI Express Multicast Capability (offset 0x4) */
static bits_t bits_pcie_multicast_cap[] = {
    NULL, NULL, NULL, NULL, NULL, "Max_Group:0=",
    "Bit6",
    "Bit7",
    NULL, NULL, NULL, NULL, NULL, "Window_Size:0=",
    "Bit14",
    "Supports ECRC Regeneration",
};

/* PCI Express Multicast Control (offset 0x6) */
static bits_t bits_pcie_multicast_control[] = {
    NULL, NULL, NULL, NULL, NULL, "Num_Group:",
    "Bit6",
    "Bit7",
    "Bit8",
    "Bit9",
    "Bit10",
    "Bit11",
    "Bit12",
    "Bit13",
    "Bit14",
    "Enable",
};

/* PCI Express AER Capabilities and Control (offset 0x18) */
static bits_t bits_pcie_aer_cap_control[] = {
    NULL, NULL, NULL, NULL, "First Error Pointer:0=",
    "Can Generate ECRC",
    "Generate ECRC",
    "Can Check ECRC",
    "Check ECRC",
    "Can Record Multiple Headers",
    "Record Multiple Headers",
    "TLP Prefix Log Present",
    "Bit12",
    "Bit13",
    "Bit14",
    "Bit15",
};

/* PCI Express AER Root Error Command (offset 0x2c) */
static bits_t bits_pcie_aer_root_error_command[] = {
    "Report Correctable Errors",
    "Report Non-Fatal Errors",
    "Report Fatal Errors",
    "Bit3",
    "Bit4",
    "Bit5",
    "Bit6",
    "Bit7",
};

/* PCI Express Port VC Capability 1 (offset 0x4) */
static bits_t bits_vc_port_vc_cap1[] = {
    NULL, NULL, "Max VC",
    "Bit3",
    NULL, NULL, "Lowpri VCs",
    "Bit7",
    NULL, "Refclk:0=100ns",
    NULL, "Arbitration Table:0=1-bit,1=2-bit,2=4-bit,3=8-bit",
    "Bit12", "Bit13", "Bit14", "Bit15",
};

/* PCI Express Port VC Capability 2 (offset 0x8) */
static bits_t bits_vc_port_vc_cap2[] = {
    "Supports HW Fixed Arbitration",
    "Supports Weighted Round Robin",
    "Supports 64-phase WRR",
    "Supports 128-phase WRR",
    "Bit4", "Bit5", "Bit6", "Bit7",
};

/* PCI Express Port VC Control (offset 0xc) */
static bits_t bits_vc_port_vc_ctrl[] = {
    "Load VC Arbitration Table",
    NULL, NULL, "VC Arbitration Select",
    "Bit4", "Bit5", "Bit6", "Bit7",
};

/* PCI Express Port VC[n] Resource Capability (offset 0x4 + vc * 0xc) */
static bits_t bits_vc_port_vc0_rcap[] = {
    "Supports HW Fixed Arbitration",
    "Supports 32-phase Weighted Round Robin",
    "Supports 64-phase WRR",
    "Supports 128-phase WRR",
    "Supports Time-based 128-phase WRR",
    "Supports 256-phase WRR",
    "Bit6", "Bit7", "Bit8", "Bit9", "Bit10", "Bit11", "Bit12", "Bit13",
    "Advanced Packet Switching", "Reject Snoop",
    NULL, NULL, NULL, NULL, NULL, NULL, "Max Time Slots",
    "Bit23",
    NULL, NULL, NULL, NULL, NULL, NULL, NULL, "PAT Offset",
};

/* PCI Express Port VC[n] Resource Control (offset 0x8 + vc * 0xc) */
static bits_t bits_vc_port_vc0_rctrl[] = {
    "TC0", "TC1", "TC2", "TC3", "TC4", "TC5", "TC6", "TC7",
    "Bit8", "Bit9", "Bit10", "Bit11", "Bit12", "Bit13", "Bit14", "Bit15",
    "LoadPat",
    NULL, NULL, "Arb:0=HW Fixed,"
        "1=32-phase Weighted Round Robin,2=64-phase WRR,3=128-phase WRR,"
        "4=Time-based 128-phase WRR,5=256-phase WRR,6=Reserved(6),"
        "7=Reserved(7)",
    "Bit20", "Bit21", "Bit22", "Bit23",
    NULL, NULL, "VC ID",
    "Bit27", "Bit28", "Bit29", "Bit30",
    ":0=Disable,1=Enable"
};

/* PCI Express Port VC[n] Resource Status (offset 0xe + vc * 0xc) */
static bits_t bits_vc_port_vc0_rstat[] = {
    "Arb Table Incoherent",
    "VC initialization not complete",
    "Bit18", "Bit19", "Bit20", "Bit21", "Bit22", "Bit23",
};

/* PCIe AER Uncorrectable Error Status (AERs+0x04). */
bits_t bits_pcie_aer_uncorrectable_status[] = {
    "Training Error",
    "Bit1", "Bit2", "Bit3",
    "Data Link Protocol Error",
    "Surprise Down",
    "Bit6", "Bit7", "Bit8", "Bit9", "Bit10", "Bit11",
    "Poisoned TLP",
    "Flow Control Protocol Error",
    "Completion Timeout",
    "Completer Abort",
    "Unexpected Completion",
    "Receiver Overflow",
    "Malformed TLP",
    "ECRC Error",
    "Unsupported Request",
    "ACS Violation",
    "Uncorrectable Internal Error",
    "Multicast Blocked TLP",
    "AtomicOp Egress Blocked",
    "TLP Prefix Blocked",
    "Bit26", "Bit27", "Bit28", "Bit29", "Bit30", "Bit31",
};

/* PCIe AER Device Status (AERs+0xa). */
bits_t bits_pcie_device_status[] = {
    "Correctable Error",
    "Non-Fatal Error",
    "Fatal Error",
    "Unsupported Request",
    "", // "Aux Power Detected",
    "Transactions Pending",
    "Bit6", "Bit7",
};

/* PCIe AER Correctable Error Status (AERs+0x10). */
bits_t bits_pcie_aer_correctable_status[] = {
    "Receiver Error",
    "Bit1", "Bit2", "Bit3",
    "Bit4", "Bit5",
    "Bad TLP",
    "Bad DLLP",
    "Replay Number Rollover",
    "Bit9", "Bit10", "Bit11",
    "Replay Timer Timeout",
    "Advisory Non-Fatal",
    "Correctable Internal Error",
    "Header Log Overflow",
};

/* PCIe AER Link Status upper 16 bits (AERs+0x14). */
bits_t bits_pcie_link_status[] = {
    NULL, NULL, NULL, "Link Speed:0=Reserved,1=2.5,2=5.0,3=8.0,?=Reserved",
    NULL, NULL, NULL, NULL, NULL,
    "Link Width:0=DOWN,1=x1,2=x2,4=x4,8=x8,10=x16,20=x32",
    "Bit10",
    "Link Training",
    "", // "Same XTAL clocks",
    "Active", // "DL_Active",
    "UERR Speed/Width Change",
    "CERR Speed/Width Change",
};

/* PCIe AER Slot Status (AERS+0x1a). */
bits_t bits_pcie_slot_status[] = {
    "Attention Button",
    "Power Fault",
    "MRL Sensor Changed",
    "Presence Detect Changed",
    "Command Completed",
    "MRL Sensor Open",
    "Slot Occupied",
    "Emech Interlock Engaged",
    "DLL Active State Changed",
    "Bit9",
    "Bit10",
    "Bit11",
    "Bit12",
    "Bit13",
    "Bit14",
    "Bit15",
};

/* PCIe AER Root Port Error Status (AERs+0x30). */
bits_t bits_pcie_aer_root_error_status[] = {
    "Correctable Received",
    "Multiple Correctable Received",
    "Fatal or Non-fatal Received",
    "Multiple Fatal or Non-fatal Received",
    "First Uncorrectable Fatal",
    "Non-Fatal Messages Received",
    "Fatal Messages Received",
    "Bit7",
    "Bit8",
    "Bit9",
    "Bit10",
    "Bit11",
    "Bit12",
    "Bit13",
    "Bit14",
    "Bit15",
    "Bit16",
    "Bit17",
    "Bit18",
    "Bit19",
    "Bit20",
    "Bit21",
    "Bit22",
    "Bit23",
    "Bit24",
    "Bit25",
    "Bit26",
    NULL, NULL, NULL, NULL, "MSG",
};

/* PCIe AER Link Status 2 (AERs+0x32). */
bits_t bits_pcie_link_status_2[] = {
    "",  // "Current De-emphasis level:0=-6Db,1=-3.5Db",
    "",  // "Equalization Complete",
    "",  // "Equalization Phase 1 Successful",
    "",  // "Equalization Phase 2 Successful",
    "",  // "Equalization Phase 3 Successful",
    "Link Equalization Reset",
    "Bit6", "Bit7",
};

/* PCI Express AER Uncorrectable Error Severity (offset 0xc) */
static bits_t bits_pcie_aer_uncorrectable_severity[] = {
    "Bit0",
    "Bit1",
    "Bit2",
    "Bit3",
    "Fatal on Data Link Protocol",
    "Fatal on Surprise Down",
    "Bit6",
    "Bit7",
    "Bit8",
    "Bit9",
    "Bit10",
    "Bit11",
    "Poisoned TLP",
    "Flow Control Protocol",
    "Completion Timeout",
    "Completer Abort",
    "Unexpected Completion",
    "Receiver Overflow",
    "Malformed TLP",
    "ECRC",
    "Unsupported Request",
    "ACS Violation",
    "Uncorrectable Internal Error",
    "MC Blocked TLP",
    "AtomicOp Egress Blocked",
    "TLP Prefix Blocked",
    "Bit26",
    "Bit27",
    "Bit28",
    "Bit29",
    "Bit30",
    "Bit31",
};

/*
 * lookup_list_t - Structure to contain a value and corresponding name.
 *
 *   - ll_value   - Value
 *   - ll_name    - String name
 */
typedef struct {
    uint8_t           ll_value;
    const char *const ll_name;
} lookup_list_t;

/* All known PCI Express TLP header message types. */
static const lookup_list_t tlp_message_types[] = {
    { 0x00, "Unlock"                    }, // Unlock Completer
    { 0x14, "PM_Active_State_Nak"       }, // Msg to receiver
    { 0x18, "PM_PME"                    }, // Sent upstream by PME requester
    { 0x19, "PME_Turn_Off"              }, // Broadcast Downstream
    { 0x1b, "PME_TO_Ack"                }, // Ack of Timeout sent upstream
    { 0x20, "Assert_INTA"               }, // Assert virtual wire INTA
    { 0x21, "Assert_INTB"               }, // Assert virtual wire INTB
    { 0x22, "Assert_INTC"               }, // Assert virtual wire INTC
    { 0x23, "Assert_INTD"               }, // Assert virtual wire INTD
    { 0x24, "Deassert_INTA"             }, // Deassert virtual wire INTA
    { 0x25, "Deassert_INTB"             }, // Deassert virtual wire INTB
    { 0x26, "Deassert_INTC"             }, // Deassert virtual wire INTC
    { 0x27, "Deassert_INTD"             }, // Deassert virtual wire INTD
    { 0x30, "ERR_COR"                   }, // Device detected PCIe correctable
    { 0x31, "ERR_NONFATAL"              }, // Device detected PCIe non-fatal
    { 0x33, "ERR_FATAL"                 }, // Device detected PCIe fatal
    { 0x40, "Attention_Indicator_Off"   }, // switch msg -> dev
    { 0x41, "Attention_Indicator_On"    }, // switch msg -> dev
    { 0x43, "Attention_Indicator_Blink" }, // switch msg -> dev
    { 0x44, "Power_Indicator_Off"       }, // switch msg -> dev
    { 0x45, "Power_Indicator_On"        }, // switch msg -> dev
    { 0x47, "Power_Indicator_Blink"     }, // switch msg -> dev
    { 0x48, "Attention_Button_Pressed"  }, // dev msg -> switch
    { 0x50, "Set_Slot_Power_Limit"      }, // of upstream port
    { 0x7e, "Vendor_Defined_Type_0"     }, // defined by dev vendor
    { 0x7f, "Vendor_Defined_Type_1"     }, // defined by dev vendor
};
#define NUM_TLP_MESSAGE_TYPES ARRAY_SIZE(tlp_message_types)

/* All known PCI Express TLP header completion types. */
static const lookup_list_t tlp_completion_types[] = {
    { 0x0,  "Success"                   }, // Successful completion (SC)
    { 0x1,  "Unsupported Request"       }, // Unsupported Request (UR)
    { 0x2,  "Config Request Retry"      }, // Configuration Request Retry (CRS)
    { 0x4,  "Completer Abort"           }, // Completer Abort (CA)
};
#define NUM_TLP_COMPLETION_TYPES ARRAY_SIZE(tlp_completion_types)

/* All known PCI Express TLP header routing types. */
static const lookup_list_t tlp_routing_types[] = {
    { 0x0,  "to root"                   }, // Routed to Root Complex
    { 0x1,  "by address"                }, // Routed by Address
    { 0x2,  "by ID"                     }, // Routed by ID
    { 0x3,  "root broadcast"            }, // Broadcast from Root Complex
    { 0x4,  "local"                     }, // Local - Terminate at Receiver
    { 0x5,  "gather to root"            }, // Gathered & routed to Root Complex
};
#define NUM_TLP_ROUTING_TYPES ARRAY_SIZE(tlp_routing_types)

#define MAX_EXPECTED_PCI_CAPS 30
#define MAX_DWORDS            32
#define MAX_BUF               4096

static uint
print_bits(uint32_t value, uint mode, bits_t bits[])
{
    uint     i;
    uint     bitvalue;
    uint     num_bits   = mode * 8;
    uint     maxmode    = mode;
    uint     count      = 0;
    uint     subvalue   = 0;
    uint     accumulate = 0;
    uint     zeros      = mode * 2;
    uint     spaces     = maxmode * 2 - zeros;
    uint32_t mask       = ((1UL << (mode * 8)) - 1);
    uint32_t svalue     = value;

    /* Show header */
    if (spaces > 8)
        spaces = 0;

    /* Mask off high bits */
    if (mask == 0)
        mask = (uint32_t) -1;
    value &= mask;

    printf("[%0*lx]%*s ", zeros, (unsigned long) value, spaces, "");

    for (i = 0; i < num_bits; i++) {
        bitvalue  = svalue & 1;
        subvalue |= (bitvalue << accumulate);
        svalue  >>= 1;
        if (bits == NULL || bits[i] == NULL) {
            accumulate++;
            continue;  // Bit is NULL -- remember this value for next bit
        }
        if (bits[i][0] == '\0') {
            subvalue   = 0;
            accumulate = 0;
            continue;  // Empty string -- don't show or accumulate
        } else if (strchr(bits[i], '=') != NULL) {
            accumulate++;
        }
        if ((bitvalue == 0) && (accumulate == 0))
            continue;  // Bit not set, and no accumulated value -- don't show
        if (count++ > 0) {
            /* Comma separate */
            printf(", ");
        }
        if (accumulate > 0) {
            const char *key = strchr(bits[i], ':');
            if (key == NULL) {
                /* Simple "<name>=<value>" */
                if (bits[i][0] == '\0')
                    printf("%x", subvalue);
                else
                    printf("%s=%x", bits[i], subvalue);
            } else {
                /* Complex "<name>=<string>" from lookup */
                const    char *tmp = NULL;
                const    char *name_end = key;
                uint     key_value;
                /*
                 * Format of bits[] in this case is:
                 *    "<name>:<val0>=<name0>,<val1>=<name1>,..."
                 *
                 * Example:
                 *    ...
                 *    NULL,
                 *    "speed:0=low,1=mid,2=high,3=reserved"
                 *    ...
                 */
                key++;
                while (*key != '\0') {
                    if (*key == '?')  // Catch-all is "?=string"
                        break;
                    int count = strtox(key, 16, &key_value);
                    if ((count == 0) || (key[count] != '=')) {
                        key = NULL;
                        break;
                    }
                    if (key_value == subvalue)
                        break;
                    tmp = strchr(key, ',');      // Go to next possible match
                    if (tmp == NULL) {
                        key = NULL;
                        break;  // No more possible matches -- just show number
                    }
                    key = tmp + 1;
                }
                if (key != NULL)
                    tmp = strchr(key, '=');
                else
                    tmp = NULL;
                if (tmp != NULL) {
                    /* Found equals -- string to display follows this */
                    key = tmp + 1;
                    tmp = strchr(tmp, ',');      // Stop output at comma
                    if (tmp == NULL)             // ...or end of string
                        tmp = key + strlen(key);

                    if ((int) (tmp - key) > 0) {
                        /* Have string to display */
                        if (name_end > bits[i]) {
                            /* More than just equals sign */
                            printf("%.*s=", (int) (name_end - bits[i]), bits[i]);
                        }
                        printf("%.*s", (int) (tmp - key), key);
                    } else if (count > 0) {
                        /* Back up -- nothing to display this time */
                        count = 0;
                    }
                } else {
                    /* No equals or no match -- just show number */
                    if (name_end > bits[i]) {
                        /* More than just equals sign */
                        printf("%.*s=", (int) (name_end - bits[i]), bits[i]);
                    }
                    printf("0x%x", subvalue);
                }
            }
        } else {
            printf("%s", bits[i]);
        }
        subvalue   = 0;
        accumulate = 0;
    }
    if (bits == NULL)
        printf("[%0*lx]%*s", zeros, (unsigned long) value, spaces, "");
    printf("\n");
    return (count);
}

static void
printspaces(void)
{
    printf("        ");
}

static void
printindentnum(uint num)
{
    printspaces();
    printf("%02x:", num);
}

static void
show_bits(const char *name, uint64_t value, uint mode, bits_t bits[])
{
    printspaces();
    printf("%s", name);

    print_bits(value, mode, bits);
}

static rc_t
pci_vpd_fetch(uint bus, uint dev, uint func, uint offset, uint pos,
              uint32_t *buf)
{
    uint     vpd_control = offset + PCI_VPD_ADDR;
    uint     vpd_data    = offset + PCI_VPD_DATA;
    uint     timeout     = 1000000;

    if (pos > PCI_VPD_ADDR_MASK)
        return (RC_FAILURE);

    /* Request VPD byte */
    pci_write16(bus, dev, func, vpd_control, pos);

    /* Wait until data has been retrieved */
    while ((pci_read16(bus, dev, func, vpd_control) & PCI_VPD_ADDR_F) == 0) {
        if (timeout-- == 0) {
            printf("Timeout reading VPD from %x.%x.%x.%x\n",
                   bus, dev, func, vpd_control);
            return (RC_TIMEOUT);
        }
    }

    /* Read retrieved data value */
    *buf = pci_read32(bus, dev, func, vpd_data);

    return (RC_SUCCESS);
}

static void
pci_vpd_dump(uint bus, uint dev, uint func, uint offset)
{
    uint32_t value;
    uint     pos;

    for (pos = 0; pos < 8; pos++) {
        if (pci_vpd_fetch(bus, dev, func, offset, pos, &value) != RC_SUCCESS)
            return;
        printf("    VPD %02x = %08x\n", pos, value);
    }
}

static uint32_t
translate_mem_address(uint32_t addr)
{
    if (((addr & 0x80000000) == 0) && ((flags & FLAG_NO_TRANSLATE) == 0))
        return (addr + (uintptr_t) bridge_mem_base);
    return (addr);
}

static void
pci_show_cap(uint bus, uint dev, uint func, uint32_t cap_pos, uint32_t value)
{
#define MAX_DWORDS 32
    uint8_t  cap = value & 0xff;
    uint     num_dwords;
    uint     word;
    uint32_t dword[MAX_DWORDS];

    printf("    PCI Cap %02x at %02x ", cap, cap_pos);
    if (cap < ARRAY_SIZE(pci_caps)) {
        printf(pci_caps[cap].name);
        num_dwords = pci_caps[cap].dwords;
    } else {
        printf("Unknown");
        num_dwords = 1;
    }

    dword[0] = value;
    if (num_dwords > MAX_DWORDS)
        num_dwords = MAX_DWORDS;
    if (pci_read_buf(bus, dev, func, cap_pos + 4, (num_dwords - 1) * 4,
                     dword + 1) != RC_SUCCESS) {
        return;
    }

    for (word = 0; word < num_dwords; word++) {
        if ((word & 0x7) == 0) {
            printf("\n");
            printindentnum(word * 4);
        }
        printf(" %08x", dword[word]);
    }
    printf("\n");

    switch (cap) {
        case PCI_CAP_ID_PM: {   // 0x01 Power Management
            show_bits("00: CAP", dword[0] >> 16, 2, bits_pm_cap);
            show_bits("04: CSR", dword[1] & 0xffff, 2, bits_pm_csr);
            show_bits("06: BSE", (dword[1] >> 16) & 0xff, 2, bits_pm_bse);
            break;
        }
        case PCI_CAP_ID_AGP:    // 0x02 Accelerated Graphics Port
            break;
        case PCI_CAP_ID_VPD:    // 0x03 Vital Product Data
            pci_vpd_dump(bus, dev, func, cap_pos);
            break;
        case PCI_CAP_ID_SLOTID: // 0x04 Slot Identification
            break;
        case PCI_CAP_ID_MSI: {  // 0x05 Message Signaled Interrupts
            uint     control         = dword[0] >> 16;
            uint     enable          = control & PCI_MSI_FLAGS_ENABLE;
            uint     cap_64          = control & PCI_MSI_FLAGS_64BIT;
            uint     per_vec_masking = control & PCI_MSI_FLAGS_MASKBIT;
            uint     a_addr;
            uint     a_data;
            uint64_t addr;
            uint32_t data;
            if (cap_64 || per_vec_masking) {
                addr = ((uint64_t) dword[2] << 32) | dword[1];
                data = dword[3];
                a_addr = 0x8;
                a_data = 0xc;
            } else {
                addr = dword[1];
                data = dword[2];
                a_addr = 0x4;
                a_data = 0x8;
            }
            printindentnum(a_addr);
            printf(" Message Addr=", a_addr);
            if ((addr >> 32) != 0)
                printf("%02x:", (uint32_t) (addr >> 32));
            printf("%08x  %02x: Data=%04x", (uint32_t) addr, a_data, data);
            printf("    Max MSI=%x  ", 1 << ((control >> 1) & 7));
            if (enable)
                printf("Enabled  MSI=%x\n", 1 << ((control >> 4) & 3));
            else
                printf("Disabled\n");
            if (per_vec_masking) {
                printspaces();
                printf("0c: Mask Bits=%08x  10: Pending Bits=%08x\n",
                       dword[3], dword[4]);
            }
            break;
        }
        case PCI_CAP_ID_CHSWP:  // 0x06 CompactPCI HotSwap
            break;
        case PCI_CAP_ID_PCIX: { // 0x07 PCI-X
            show_bits("02: CMD   ", dword[0] >> 16, 2, bits_pcix_cmd);
            show_bits("04: STATUS", dword[1], 4, bits_pcix_status);
            break;
        }
        case PCI_CAP_ID_HT:     // 0x08 HyperTransport
            break;
        case PCI_CAP_ID_VNDR:   // 0x09 Vendor specific
            break;
        case PCI_CAP_ID_DBG:    // 0x0A Debug port
            break;
        case PCI_CAP_ID_CCRC:   // 0x0B CompactPCI Central Resource Control
            break;
        case PCI_CAP_ID_SHPC:   // 0x0C PCI Standard Hot-Plug Controller
            break;
        case PCI_CAP_ID_SSVID:  // 0x0D Bridge subsystem vendor/device ID
            printspaces();
            printf("04: subsystem vendor=%04x device=%04x\n",
                   dword[1] & 0xffff, dword[1] >> 16);
            break;
        case PCI_CAP_ID_AGP3:   // 0x0E AGP Target PCI-PCI bridge
            break;
        case PCI_CAP_ID_PCIE: {  // 0x10 PCI Express
            uint port_flags = dword[0] >> 16;
            uint port_type  = (port_flags & PCI_EXP_FLAGS_TYPE) >> 4;
            show_bits("00: PCIECAP ", dword[0] >> 16, 2, bits_pcie_cap);
            show_bits("04: DEVCAP  ", dword[1], 4, bits_pcie_devcap);
            show_bits("08: DEVCTRL ", dword[2], 2, bits_pcie_devcontrol);
            show_bits("0a: DEVSTAT ", dword[2] >> 16, 1,
                      bits_pcie_device_status);
            show_bits("0c: LINKCAP ", dword[3], 4, bits_pcie_linkcap);
            show_bits("10: LINKCTRL", dword[4], 2, bits_pcie_linkcontrol);
            show_bits("12: LINKSTAT", dword[4] >> 16, 2,
                      bits_pcie_link_status);
            if (port_flags & PCI_EXP_FLAGS_SLOT) {
                if (dword[5] != 0) {
                    show_bits("14: SLOTCAP ", dword[5], 4,
                              bits_pcie_slotcap);
                }
                show_bits("18: SLOTCTRL", dword[6], 2,
                          bits_pcie_slotcontrol);
                show_bits("1a: SLOTSTAT", dword[6] >> 16, 2,
                          bits_pcie_slot_status);
            }
            if (port_type == PCI_EXP_TYPE_ROOT_PORT) {
                show_bits("1c: ROOTCTRL", dword[7], 1,
                          bits_pcie_root_control);
                show_bits("1e: ROOTCAP ", dword[7] >> 16, 1,
                          bits_pcie_rootcap);
                show_bits("20: ROOTSTAT", dword[8], 4,
                          bits_pcie_root_status);
            }
            if ((port_flags & PCI_EXP_FLAGS_VERS) >= 2) {
                /* PCIe version 2+ */
                show_bits("24: DEVCAP2 ", dword[9], 4, bits_pcie_devcap2);
                show_bits("28: DEVCTRL2", dword[10], 2,
                          bits_pcie_devcontrol2);
                show_bits("2c: LNKCAP2 ", dword[11], 2,
                          bits_pcie_linkcap2);
                show_bits("30: LNKCTRL2", dword[12], 2,
                          bits_pcie_linkcontrol2);
                show_bits("32: LNKSTAT2", dword[12] >> 16, 1,
                          bits_pcie_link_status_2);
            }
            break;
        }
        case PCI_CAP_ID_MSIX: { // 0x11 MSI-X
            uint     control   = dword[0] >> 16;
            uint     bir       = dword[2] & 0x7;
            uint     entries   = control & (BIT(11) - 1);
            uint32_t table_off = dword[2] & ~0x7U;
            printspaces();
            printf("00: Table 0x%x entries in BAR%x+%04x [",
                    entries, bir, table_off);
            if (bir >= 6) {
                printf("invalid] ");
            } else {
                uint     bar_offset = PCI_OFF_BAR0 + bir * 4;
                uint32_t addr = pci_read32(bus, dev, func, bar_offset);

                addr = translate_mem_address(addr & ~0xf);
                printf("%08x] ", addr);
            }
            if (dword[1] != 0)
                printf("|Value=%08x  ", dword[1]);
            if (control & BIT(15))
                printf("Enabled\n");
            else
                printf("Disabled\n");
            break;
        }
        case PCI_CAP_ID_AF:     // 0x13 PCI Advanced Features
            printspaces();
            printf("00: TP_CAP=%x FLR_CAP=%x Length=%x\n",
                    dword[0] & 1, !!(dword[0] & 2), (dword[0] >> 16) & 0xff);
            printspaces();
            printf("05: TP=%x\n", (dword[1] >> 8) & 1);
            break;
        default:
            break;
    }
}

static void
pci_show_caps(uint bus, uint dev, uint func, uint p_cap)
{
    uint8_t cap_pos;
    uint32_t value;
    uint16_t status = pci_read16(bus, dev, func, PCI_OFF_STATUS);

    if ((status & PCI_STATUS_HAS_CAPS) == 0)
        return;

    for (cap_pos = pci_read8(bus, dev, func, PCI_OFF_CAP_LIST);
         (cap_pos > PCI_OFF_CAP_LIST) && (cap_pos <= 0xfc);
         cap_pos = (value >> 8) & 0xfc) {

        value = pci_read32(bus, dev, func, cap_pos);
        if (p_cap != PCI_ANY_ID) {
            uint8_t cap = value & 0xff;
            if (cap != p_cap)
                continue;
        }
        pci_show_cap(bus, dev, func, cap_pos, value);
    }
}

static uint
pci_mmio_read(uint32_t cfgbase, uint offset, uint mode)
{
    switch (mode) {
        case 1:
            return (*VADDR8(cfgbase + offset));
        case 2: {
            uint16_t tvalue = *VADDR16(cfgbase + offset);
            return (swap16(tvalue));
        }
        case 4: {
            uint32_t tvalue = *VADDR32(cfgbase + offset);
            return (swap32(tvalue));
        }
    }
    return (0);  // bug
}

static void
pci_mmio_read_block(uint32_t cfgbase, uint offset, uint count, uint32_t *buf)
{
    while (count-- > 0) {
        *(buf++) = pci_mmio_read(cfgbase, offset, 4);
        offset += 4;
    }
}

uint
pci_mmio_find_cap(uint32_t cfgbase, uint8_t cap)
{
    uint16_t status = pci_mmio_read(cfgbase, PCI_OFF_CMD, 4) >> 16;
    uint32_t value;
    uint     pos;
    uint     count = 0;

    if ((status & PCI_STATUS_HAS_CAPS) == 0)
        return (0);  // Device does not have PCI capabilities list

    /* Get start of capabilities list */
    value = pci_mmio_read(cfgbase, PCI_OFF_CAP_LIST, 4);
    if (value == 0xffffffff)
        return (0);
    pos = value & 0xfc;

    while ((pos != 0x00) && (count++ < 20)) {
        value = pci_mmio_read(cfgbase, pos, 4);
        if (value == 0xffffffff)
            return (0);
        if ((value & 0xff) == cap)
            return (pos);
        pos = (value >> 8) & 0xfc;
    }
    return (0);
}

static void
pci_print_bdf(uint32_t dev, uint show_offset)
{
    uint pci_bus    = dev >> 24;
    uint pci_dev    = PCI_SLOT(dev >> 16);
    uint pci_func   = PCI_FUNC(dev >> 16);
    uint pci_offset = dev & 0xffff;

    printf("%x.%x.%x", pci_bus, pci_dev, pci_func);
    if (show_offset)
        printf(".%x", pci_offset);
}

static void
pci_print_device(uint bus, uint dev, uint func, uint offset,
                 uint pad, const char * const name)
{
#define PCI_DEV_NAME_PAD_LEN 11
    int name_len = PCI_DEV_NAME_PAD_LEN;

    printf("%x.%x.%x.%03x ", bus, dev, func, offset);

    if (offset > 0xffff)
        name_len -= 2;
    else if (offset > 0xfff)
        name_len -= 1;

    if (name == NULL) {
        /* Pad if necessary */
        if (pad)
            printf("%*s", PCI_DEV_NAME_PAD_LEN - name_len, "");
    } else {
        printf("%*s", pad ? name_len + 1 : 1, name);
    }
}

static void
pci_print_str(uint code, const lookup_list_t list[],
              uint count, const char * const type)
{
    uint i;
    for (i = 0; i < count; i++) {
        if (list[i].ll_value == code)
            break;
    }
    if (i < count)
        printf(list[i].ll_name);
    else
        printf("Unknown %s(%x)", type, code);
}

void
pci_decode_hdr_log(uint cfgbase, const uint32_t *log, uint offset)
{
    uint        log_all_zero    = TRUE;  // Assume all zero
    uint        log_all_ff      = TRUE;  // Assume all 0xffffffff
    uint        requester_index = 0;     // 0=Do not show requester
    uint        completer_index = 0;     // 0=Do not show completer
    uint        tlp_format;
    uint        tlp_type;
    uint        tlp_length;
    uint        i;

    /* Process TLP log */
    for (i = 0; i < 4; i++) {
        if (log[i] != 0)
            log_all_zero = FALSE;
        if (log[i] != 0xffffffff)
            log_all_ff = FALSE;
    }

    /* Check for all 00 data (nothing logged) */
    if (log_all_zero)
        return;  // Nothing to be done

    printindentnum(offset);

    /* Check for all FF data (invalid) */
    if (log_all_ff) {
        printf(" fails to respond to config access\n");
        return;  // Access failure
    }

    printf("[%08x %08x %08x %08x]\n", log[0], log[1], log[2], log[3]);

    /* Determine the format of this TLP */
    tlp_format = (log[0] >> 29) & 0x03;

    /* Determine the type of this TLP */
    tlp_type = (log[0] >> 24) & 0x1f;
    if ((tlp_type & 0x18) == 0x10)
        tlp_type = 0x10;  // Filter out low 3 bits of type for Message Request

    /* Determine the length of this TLP in bytes */
    tlp_length = log[0] & 0x3ff;
    if (tlp_length == 0)
        tlp_length = 0x400;

    /* Determine the transfer width and start byte offset */
    uint mode_bytes       = 0;
    uint first_byte       = 0;
    uint tlp_byte_enables = log[1] & 0xff; // [0:3]=1st DWORD; [4:7]=Last DWORD

    for (i = tlp_byte_enables; i != 0; i >>= 1) {
        if (i & 1)
            mode_bytes++;
        else if (mode_bytes > 0)
            break;
        else
            first_byte++;
    }
    if (tlp_length > 2) {
        /* DWORDs between the first and last DWORD */
        mode_bytes += (tlp_length - 2) << 2;
    }

    if (cfgbase != 0)
        printf("%*s", 24, "");

    switch (tlp_type) {
        case TLP_TYPE_MEMORY: {  /* 0x00: Memory access */
            uint32_t   log2;
            uint32_t   log3;
            uint64_t   addr;

            printf("Memory %s ",
                   (tlp_format & TLP_FORMAT_WRITE) ? "write" : "read");
decode_memory_access:
            log2 = log[2];
            log3 = log[3];
            printf("len=%x", mode_bytes);

            if (((tlp_format & TLP_FORMAT_64BIT_ADDR) == 0) &&
                 (tlp_format & TLP_FORMAT_WRITE)) {
                /* TLP has 32-bit address, so log[3] is first 32-bit value */
                printf(" [");
                if (first_byte > 0) {
                    printf("%.*s", first_byte * 2, "........");
                }
                for (i = 4 - first_byte; i > 0; i--) {
                    printf("%02x", (uint8_t) log3);
                    log3 >>= 8;
                }
                if (tlp_length > 1) {
                    printf("...");
                }
                printf("]");
            }

            if (tlp_format & TLP_FORMAT_64BIT_ADDR)
                log3 += first_byte;
            else
                log2 += first_byte;

            /* Display first 32 bits of address */
            printf(" at %08x", log2);
            addr = log2;

            if (tlp_format & TLP_FORMAT_64BIT_ADDR) {
                /* TLP has 64-bit address.  Display low 32 bits */
                printf(":%08x", log3);
                addr = (addr << 32) | log3;
            }
#if 0
            /* Report which device was targeted. */
            target_dev = pci_find_addr(addr);
            if (target_dev != NULL) {
                printf(" target=%x.%x.%x",
                        target_dev->pd_bus, target_dev->pd_dev,
                        target_dev->pd_func);
            }
#endif
            requester_index = 1;
            break;
        }

        case TLP_TYPE_LOCKED_READ:  /* 0x01: Locked memory read access */
            printf("Locked memory read ");
            goto decode_memory_access;

        case TLP_TYPE_IO:  /* 0x02: I/O access */
            printf("I/O %s ",
                   (tlp_format & TLP_FORMAT_WRITE) ? "write" : "read");
            goto decode_memory_access;

        case TLP_TYPE_CONFIG_0:  /* 0x04: Config type 0 access */
        case TLP_TYPE_CONFIG_1:  /* 0x05: Config type 1 access */
            printf("Config Type %d %s len=%u",
                   (tlp_type == TLP_TYPE_CONFIG_0) ? 0 : 1,
                   (tlp_format & TLP_FORMAT_WRITE) ? "write" : "read",
                   mode_bytes);
            if (tlp_format & TLP_FORMAT_WRITE) {  // write
                uint max_byte = (mode_bytes > 4) ? 4 : mode_bytes;
                printf(" [");
                for (i = 0; i < max_byte; i++) {
                    uint pos = 4 - first_byte - max_byte + i;
                    printf("%02x", ((uint8_t *) &log[3])[pos]);
                }
                if (i < mode_bytes)
                    printf("...");
                printf("]");
            }
            printf(" at ");
            pci_print_bdf(log[2] + first_byte, TRUE);
            requester_index = 1;
            break;

        case TLP_TYPE_COMPLETION: {  /* 0x0a: Completion */
            uint tlp_status;
            uint with_data;

decode_completion:
            with_data = tlp_format & TLP_FORMAT_COMPL_WDATA;
            printf("Completion%s: ", with_data ? " W/Data" : "");
            tlp_status = (log[1] >> 13) & 0x7;
            pci_print_str(tlp_status, tlp_completion_types,
                          NUM_TLP_COMPLETION_TYPES, "completion");
            if (with_data && tlp_status == 0) {
                printf(" len=%x", tlp_length * 4);
            }
            completer_index = 1;
            requester_index = 2;
            break;
        }
        case TLP_TYPE_LOCKED_COMPL:  /* 0x0b: Locked Completion */
            printf("Locked ");
            goto decode_completion;

        case TLP_TYPE_MESSAGE: {
            uint tlp_msg_code = log[1] & 0xff;
            uint tlp_routing  = (log[0] >> 24) & 0x7;
            uint with_data    = tlp_format & TLP_FORMAT_COMPL_WDATA;

            printf("MessageRequest%s ", with_data ? " W/Data" : "");
            if ((tlp_format & TLP_FORMAT_64BIT_ADDR) == 0) {
                printf("BadTLPfmt(%x) ", tlp_format);
            }
            pci_print_str(tlp_routing, tlp_routing_types,
                          NUM_TLP_ROUTING_TYPES, "routing");
            printf(" ");
            pci_print_str(tlp_msg_code, tlp_message_types,
                          NUM_TLP_MESSAGE_TYPES, "msg");

            requester_index = 1;
            break;
        }

        default:
            printf("unknown type %02x", tlp_type);
            break;
    }
    /* Decode the requester if required */
    if (requester_index != 0) {
        printf(" req=");
        pci_print_bdf(log[requester_index], FALSE);
    }
    /* Decode the completer if required */
    if (completer_index != 0) {
        printf(" comp=");
        pci_print_bdf(log[completer_index], FALSE);
    }
    printf("\n");
}

static void
pci_show_ecap(uint32_t cfgbase, uint offset, uint32_t value)
{
    uint     num_dwords;
    uint     pos;
    uint     cap = value & 0xffff;
    uint     fetched_dwords;
    uint32_t dword[MAX_DWORDS];

    printf("    PCI ECap %02x at %03x ", cap, offset);
    if (cap < ARRAY_SIZE(pcie_caps)) {
        printf("%s", pcie_caps[cap].name);
        num_dwords = pcie_caps[cap].dwords;
    } else {
        printf("Unknown");
        num_dwords = 1;
    }

    dword[0] = value;
    if (num_dwords > MAX_DWORDS)
        num_dwords = MAX_DWORDS;

    pci_mmio_read_block(cfgbase, offset + 4, num_dwords - 1, dword + 1);
    fetched_dwords = num_dwords;

    /* Implement any dword extensions here based on cap type */
    switch (cap) {
        case 0x0001:   // Advanced Error Reporting (AER)
            /* This is a hack and is only valid for a root port */
            num_dwords = 0x0e;
            break;
        case 0x000b:   // Vendor-specific (VSEC)
            num_dwords = ((dword[1] >> 20) + 3) / 4;  // dwords rounded up
            break;
        default:
            break;
    }

    /* Retrieve further dwords if necessary */
    if (num_dwords > MAX_DWORDS)
        num_dwords = MAX_DWORDS;
    if (num_dwords > fetched_dwords) {
        pci_mmio_read_block(cfgbase, offset + 4 * fetched_dwords,
                            num_dwords - fetched_dwords,
                            dword + fetched_dwords);
    }

    for (pos = 0; pos < num_dwords && pos < MAX_DWORDS; pos++) {
        if ((pos & 0x7) == 0) {
            printf("\n");
            printindentnum(pos * 4);
        }
        printf(" %08x", dword[pos]);
    }
    printf("\n");

    switch (cap) {
        case 0x0001: { // Advanced Error Reporting (AER)
            show_bits("04: UERR    ", dword[1], 2,
                      bits_pcie_aer_uncorrectable_status);
            show_bits("08: UERRMASK", dword[2], 2,
                      bits_pcie_aer_uncorrectable_status);
            show_bits("0c: UERRSEV ", dword[3], 4,
                      bits_pcie_aer_uncorrectable_severity);
            show_bits("10: CERR    ", dword[4], 2,
                      bits_pcie_aer_correctable_status);
            show_bits("14: CERRMASK", dword[5], 2,
                      bits_pcie_aer_correctable_status);
            show_bits("18: CAP_CTRL", dword[6], 2,
                      bits_pcie_aer_cap_control);
            pci_decode_hdr_log(cfgbase, &dword[7], 0x1c);
            if (num_dwords >= 0xe) {
                show_bits("2c: ROOTCMD ", dword[11], 1,
                          bits_pcie_aer_root_error_command);
                show_bits("30: ROOTSTAT", dword[12], 4,
                          bits_pcie_aer_root_error_status);
            }
            break;
        }
        case 0x0002: {   // Virtual Channel (VC)
            uint vc_count = dword[1] & 7;
            uint vc;

            show_bits("04: PORT_VCCAP1 ", dword[1], 2,
                      bits_vc_port_vc_cap1);
            show_bits("08: PORT_VCCAP2 ", dword[2], 1,
                      bits_vc_port_vc_cap2);
            if (dword[3] != 0) {
                printspaces();
                printf("0c: PORT_VCSTAT  Arb Table=%scoherent\n",
                        (dword[3] & (1 << 16)) ? "in" : "");
            }
            show_bits("10: PORT_VCCTRL ", dword[3] >> 16, 1,
                      bits_vc_port_vc_ctrl);

            for (vc = 0; vc <= vc_count; vc++) {
                char buf[32];
                uint vc_off  = 4 + vc * 3;
                sprintf(buf, "%02x: VC%d RCAP  ", vc_off, vc);
                show_bits(buf, dword[vc_off + 0], 4,
                          bits_vc_port_vc0_rcap);
                sprintf(buf, "%02x: VC%d RCTRL ", vc_off + 0x4, vc);
                show_bits(buf, dword[vc_off + 1], 4,
                          bits_vc_port_vc0_rctrl);
                sprintf(buf, "%02x: VC%d RSTAT ", vc_off + 0x8, vc);
                show_bits(buf, dword[vc_off + 2] >> 16, 1,
                          bits_vc_port_vc0_rstat);
            }
            break;
        }
        case 0x0003: { // Device Serial Number
            uint64_t qword = *VADDR32(dword + 1) |
                             (((uint64_t) *VADDR32(dword + 5)) << 32);
            printspaces();
            printf("04: Company=%06jx Device=%010jx\n",
                   qword >> 40, qword & 0xffffffffff);
            break;
        }
        case 0x0004: {   // Power Budgeting
            uint base_power  = (dword[2] & 0xff);
            uint data_scale  = (dword[2] >> 8) & 3;
            uint pm_substate = (dword[2] >> 10) & 3;
            uint pm_state    = (dword[2] >> 13) & 3;
            uint op_cond     = (dword[2] >> 15) & 7;
            uint power_rail  = (dword[2] >> 18) & 7;

            /* Apply data_scale: 00=1.0x 01=0.1x 10=0.01x 11=0.001x */
            if ((data_scale == 0) && (base_power >= 0xf0)) {
                switch (base_power) {
                    case 0xf0:
                        base_power = 239 * 1000;  // < 250 W
                        break;
                    case 0xf1:
                        base_power = 250 * 1000;  // < 275 W
                        break;
                    case 0xf2:
                        base_power = 275 * 1000;  // < 300 W
                        break;
                    default:  // Reserved for > 300
                        base_power = 300 * 1000;  // > 300 W
                        break;
                }
            } else {
                base_power *= 1000;
                while (data_scale-- > 0)
                    base_power /= 10;
            }

            printspaces();
            printf("04: Index=%x\n",
                   dword[1] & 0xff);
            printspaces();
            printf("08: pm_state=D%u.%u %s %s ",
                   pm_state,
                   pm_substate,
                   (power_rail == 0) ? "12V" :
                   (power_rail == 1) ? "3.3V" :
                   (power_rail == 2) ? "1.8V" :
                   (power_rail == 7) ? "Thermal" : "?",
                   (op_cond == 0) ? "PME_Auxiliary" :
                   (op_cond == 1) ? "Auxiliary" :
                   (op_cond == 2) ? "Idle" :
                   (op_cond == 3) ? "Sustained" :
                   (op_cond == 7) ? "Maximum" : "?");
            printf("power=%u.%03u W%s\n",
                   base_power, base_power % 1000, (dword[3] & 1) ? " allocated" : "");
            break;
        }
        case 0x0005:   // Root Complex Link Declaration
            break;
        case 0x0006:   // Root Complex Internal Link Ctrl
            break;
        case 0x0007:   // Root Complex Event Collector Endpoint
            break;
        case 0x0008:   // Multi-function Virtual Channel (MFVC)
            break;
        case 0x0009:   // Virtual Channel (VC)
            break;
        case 0x000a:   // Root Complex Reg. Block (RCRB)
            break;
        case 0x000b:   // Vendor-specific (VSEC)
            printspaces();
            printf("04: ID=%04x Rev=%x Length=%x\n",
                   dword[1] & 0xffff, (dword[1] >> 16) & 3, dword[1] >> 20);
            break;
        case 0x000c:   // Config Access Correlation (CAC)
            break;
        case 0x000d:   // Access Control Services (ACS)
            show_bits("04: ACSCAP ", dword[1], 2, bits_pcie_acs_cap);
            show_bits("06: ACSCTL ", dword[1] >> 16, 1, bits_pcie_acs_ctl);
            show_bits("08: ACSECV ", dword[2], 4, bits_pcie_acs_ecv);
            break;
        case 0x000e:   // Alternative Routing-ID (ARI)
            break;
        case 0x000f:   // Addr Translation Services (ATS)
            break;
        case 0x0010:   // Single Root I/O Virt (SR-IOV)
            break;
        case 0x0011:   // Multi-Root I/O Virt (MR-IOV)
            break;
        case 0x0012:   // Multicast
            show_bits("04: MCCAP ", dword[1], 2, bits_pcie_multicast_cap);
            show_bits("06: MCCTRL", dword[1] >> 16, 2,
                      bits_pcie_multicast_control);
            if (dword[2] != 0) {
                printspaces();
                printf("08: MC_Index=0x%x Base=%08x:%08x\n",
                       dword[2] & 0x3f, dword[3], dword[2] & ~0x3fU);
            }
            if ((dword[5] != 0) || (dword[4] != 0)) {
                printspaces();
                printf("10: MC_Receive=%08x:%08x\n",
                       dword[5], dword[4]);
            }
            if ((dword[7] != 0) || (dword[6] != 0)) {
                printspaces();
                printf("18: MC_Block_All=%08x:%08x\n",
                       dword[7], dword[6]);
            }
            if ((dword[9] != 0) || (dword[8] != 0)) {
                printspaces();
                printf("20: MC_Block_Untranslated=%08x:%08x\n",
                       dword[9], dword[8]);
            }
            if ((dword[11] != 0) || (dword[10] != 0)) {
                uint sizebits = dword[10] & 0x3fU;
                uint64_t size = (sizebits < 6) ? 0 : (1ULL << sizebits);
                if (size != 0) {
                    printspaces();
                    printf("28: MC_Overlay_Size=0x%jx BAR=%08x:%08x\n",
                           size, dword[11], dword[10] & ~0x3fU);
                }
            }
            break;
        case 0x0013:   // Page Request
            break;
        case 0x0014:   // AMD Reserved
            break;
        case 0x0015:   // Resizable Bar
            break;
        case 0x0016:   // Dynamic Power Allocation (DPA)
            break;
        case 0x0017:   // TLP Processing Hints (TPH)
            show_bits("04: TPHCAP ", dword[1], 4, bits_pcie_tph_cap);
            show_bits("04: TPHCTRL", dword[1], 2, bits_pcie_tph_control);
            break;
        case 0x0018: {  // Latency Tolerance Reporting (LTR)
            uint     value;
            uint     scale;
            uint64_t max_snoop;
            uint64_t max_no_snoop;
            /*
             * latency_scale is as follows:
             *    0 = 1 ns
             *    1 = 32 ns
             *    2 = 1024 ns
             *    3 = 32768 ns
             *    4 = 1048576 ns
             *    5 = 33554432 ns
             *    6 = reserved
             *    7 = reserved
             */
            /* Calculate Max Snoop Latency */
            value  = dword[1] & 0x3ff;
            scale  = (dword[1] >> 10) & 0x7;
            max_snoop = value * (1 << (scale * 5));
            /* Calculate Max No-Snoop Latency */
            value  = (dword[1] >> 16) & 0x3ff;
            scale  = (dword[1] >> 26) & 0x7;
            max_no_snoop = value * (1 << (scale * 5));
            if ((max_snoop != 0) || (max_no_snoop != 0)) {
                printspaces();
                printf("04: Max Snoop Latency=%llu ns"
                       "  Max No-Snoop Latency=%llu ns\n",
                       (long long) max_snoop, (long long) max_no_snoop);
            }
            break;
        }
        case 0x0019: {  // Secondary PCI Express
            uint pos = 0x0c;
            char buf[32];
            uint16_t *wptr = (uint16_t *) &dword[3];
            uint printed = 0;;
            show_bits("04: LNKCTRL3", dword[1], 1,
                      bits_pcie_cap2_link_control3);

            printspaces();
            printf("08: LANEERR [%08x] ", dword[2]);
            for (uint lane = 0; lane < 32; lane++) {
                if (dword[2] & BIT(lane)) {
                    if (printed++)
                        printf(", ");
                    printf("Lane%u", lane);
                }
            }
            if (printed)
                printf("\n");

            for (uint lane = 0; lane < 16; lane++, pos += 2) {
                sprintf(buf, "%02x: LANE%02dEQ dB", pos, lane);
                show_bits(buf, wptr[lane], 2, bits_pcie_cap2_lane_eq_control);
            }
            break;
        }
        case 0x001a:   // Protocol Multiplexing (PMUX)
            break;
        default:
            break;
    }
}

static void
pcie_show_ecaps(uint32_t cfgbase, uint pcie_offset, uint desired_ecap)
{
    uint     count = 0;
    uint     pos = pcie_offset;
    uint     cap;
    uint32_t value;

    for (count = 0; count < MAX_EXPECTED_PCI_CAPS; count++) {
        if (pos < 0x100 || pos >= 0x1000)
            break;  // Invalid pointer / no more capabilities

        /* Read next capability value */
        value = pci_mmio_read(cfgbase, pos, 4);
        if (value == 0xffffffff)
            return;

        cap = value & 0xffff;
        if (cap == 0x0000)
            break;  // End of capabilities list

        if ((desired_ecap == (uint)PCI_ANY_ID) ||
            ((cap | 0x10000) == desired_ecap))
            pci_show_ecap(cfgbase, pos, (uint32_t) value);

        /* Move to next PCIe capability */
        pos = (uint16_t) (value >> 20);
    }
}

static void
pcie_show_regs(uint32_t cfgbase, uint p_cap)
{
    uint pcie_offset = pci_mmio_find_cap(cfgbase, PCI_CAP_ID_PCIE);
    if (pcie_offset == 0)
        return;  // Can't find PCIe capability

    pcie_show_ecaps(cfgbase, PCI_OFF_ECAP, p_cap);
}

extern struct ExecBase *DOSBase;

static void
pci_show_device_specific(uint bus, uint dev, uint func,
                         uint16_t vendor, uint16_t device,
                         int p_cap, uint class)
{
    if ((vendor == 0x10b5) && (class == PCI_CLASS_PCI_BRIDGE)) {
        /* BAR0 of PLX bridge contains config space + extended registers */
        uint32_t cfgbase = pci_read32v(bus, dev, func, PCI_OFF_BAR0);
        uint32_t cmd     = pci_read32(bus, dev, func, PCI_OFF_CMD);
        uint16_t pvendor;
        uint16_t pdevice;
        uint32_t pvd;

        if (cmd & BIT(1)) {
            cfgbase = translate_mem_address(cfgbase & ~0xf);
            pvd = pci_mmio_read(cfgbase, PCI_OFF_VENDOR, 4);
            pvendor = (uint16_t) pvd;
            pdevice = pvd >> 16;

            if (pvd == 0xffffffff)
                return;  // BAR not present?
            if ((vendor != pvendor) || (device != pdevice)) {
                printf("PLX MMIO BAR does not match: %04x.%04x != %04x.%04x\n",
                       vendor, device, pvendor, pdevice);
                return;  // Vendor/Device in MMIO BAR does not match
            }
            pcie_show_regs(cfgbase, p_cap);
        }
    }
    (void) device;
}

static rc_t
pci_check_reg(uint p_bus, uint p_dev, uint p_func,
              const char * const name,
              uint offset, uint fmode, bits_t bits[],
              uint *errs, uint32_t mask)
{
    uint     mode = fmode & 0xff;
    uint32_t value;
    rc_t     rc;

    if ((mask == 0) ||
        ((flags & (FLAG_PCI_STATUS_ALL | FLAG_VERBOSE)))) {
        mask = ~0UL;
    }

    value = pci_read(p_bus, p_dev, p_func, offset, mode);
    if (((mode == 4) && (value == 0xffffffff)) ||
        ((mode == 2) && (value == 0xffff))) {
        /* Failed to read device */
        value = pci_read(p_bus, p_dev, p_func, offset, mode);
        if (((mode == 4) && (value == 0xffffffff)) ||
            ((mode == 2) && (value == 0xffff))) {
            uint16_t vid;
            vid = pci_read(p_bus, p_dev, p_func, PCI_OFF_VENDOR, 2);
            if (vid == 0xffff) {
                rc = RC_FAILURE;
                pci_print_device(p_bus, p_dev, p_func, offset, TRUE, name);
                printf("           fails to respond to config access\n");
                (*errs)++;
                return (rc);
            }
        }
    }

    if ((flags & FLAG_PCI_CLEAR) && ((value & mask) != 0)) {
        /* User requested that status should be cleared. */
        uint32_t cvalue;
        uint32_t cleared;
        uint32_t wvalue = value;

#if 0
        /* Handle clear of registers which are not RW1C. */
        dev_check_special_clear(dev, offset, &wvalue);
#endif

        pci_write(p_bus, p_dev, p_func, offset, mode, wvalue);

        cvalue = pci_read(p_bus, p_dev, p_func, offset, mode);

        /* Only report bits which were cleared */
        cleared = value & ~cvalue;
        value = (value & ~mask) | cleared;
    }

    if (((value & mask) != 0) || (flags & FLAG_PCI_STATUS_ALL)) {
        /* Value changed or all display requested */
        pci_print_device(p_bus, p_dev, p_func, offset, TRUE, name);
        print_bits(value, mode, bits);
        if ((value & mask) != 0) {
            (*errs)++;
        } else if (flags & FLAG_PCI_STATUS_ALL) {
            /* No bits printed */
            printf("[%0*llx]\n", mode * 2, (unsigned long long) value);
        }
    }

    return (RC_SUCCESS);
}


static void
pci_status(uint bus, uint dev, uint func, uint classrev)
{
    uint     errs = 0;

    /* Check primary PCI status register */
    if (pci_check_reg(bus, dev, func, "PRI_STAT", PCI_OFF_STATUS, 2,
                      bits_pci_status_primary, &errs, PCI_STATUS_ERRORS))
        return;

    if ((classrev >> 16) == PCI_CLASS_PCI_BRIDGE) {
        /* Check bridge secondary side registers */
        pci_check_reg(bus, dev, func, "SEC_STAT", PCI_OFF_BR_SEC_STATUS, 2,
                      bits_pci_status_secondary, &errs,
                      PCI_STATUS_ERRORS);
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

static uint32_t
get_bar_size(uint bus, uint dev, uint func, uint bar_offset, uint32_t *base,
             uint interpret_size)
{
    uint32_t size;

    Disable();  // Disable interrupts
    *base = pci_read32v(bus, dev, func, bar_offset);
    pci_write32(bus, dev, func, bar_offset, 0xffffffff);
    size = pci_read32(bus, dev, func, bar_offset);
    pci_write32v(bus, dev, func, bar_offset, *base);
    Enable();  // Enable interrupts

    if (interpret_size) {
        if (size & 1)
            size &= ~0x3;    // Drop I/O BAR type bits
        else
            size &= ~0xf;    // Drop Memory type bits
        size &= (0 - size);  // Convert mask to size
    }
    return (size);
}

static void
lspci(uint p_bus, uint p_dev, uint p_func, uint p_vendor, uint p_device,
      uint p_cap)
{
    uint    bus;
    uint    dev;
    uint    func;
    uint    off;
    uint    maxbus = PCI_MAX_BUS;
    uint    found = 0;
    char   *bustype = "Zorro";

    if (bridge_type == BRIDGE_TYPE_AMIGAPCI)
        bustype = "MB";

    if (flags & ~(FLAG_PCI_STATUS | FLAG_PCI_CLEAR | FLAG_PCI_CAP | FLAG_PCI_ECAP)) {
        /* Regular "ls" listing: show banner and root port */
        printf("B.D.F  Vend.Dev  _BAR_ ____Base____ ____Size____ Description\n");
        if (((p_bus == 0) || (p_bus == PCI_ANY_ID)) &&
             (p_dev == PCI_ANY_ID) && (p_func == PCI_ANY_ID) &&
             (p_vendor == PCI_ANY_ID) && (p_device == PCI_ANY_ID)) {
            printf("%x      %04x.%04x %-5s %12x %12x",
                   0, bridge_zorro_mfg, bridge_zorro_prod, bustype,
                   (uint32_t) pci_zorro_cdev->cd_BoardAddr,
                   (uint32_t) pci_zorro_cdev->cd_BoardSize);
            zorro_show_mfgprod();
            printf("\n");
        }
    }

    for (bus = 0; bus <= maxbus; bus++) {
        uint maxdev = PCI_MAX_DEV;  // PCI supports 32 devs per bus
        if (bus == 0)
            maxdev = PCI_MAX_PHYS_SLOT;  // 5 physical slots
        if ((p_bus != PCI_ANY_ID) && (p_bus != bus))
            continue;

        for (dev = 0; dev < maxdev; dev++) {
            uint32_t tbase;
            if ((p_dev != PCI_ANY_ID) && (p_dev != dev))
                continue;
            if (flags & FLAG_SHOW_CADDR) {
                tbase = (uintptr_t) pci_cfg_base(bus, dev, 0, 0);
                printf("%27s%08x%13x PCI Config %u.%u.0\n", "",
                       tbase, 0x100, bus, dev);
            }

            for (func = 0; func < PCI_MAX_FUNC; func++) {
                uint32_t vd;
                uint16_t vendor;
                uint16_t device;
                uint32_t classrev;
                uint32_t base;
                uint32_t size;
                uint     bar_offset;
                uint     printed;
                uint     bar;
                uint     maxbar = 6;
                uint32_t cmd;

                if ((p_func != PCI_ANY_ID) && (p_func != func))
                    goto skip_and_check_htype;

                vd = pci_read32v(bus, dev, func, PCI_OFF_VENDOR);
                vendor = (uint16_t) vd;
                device = vd >> 16;

                /* Check Vendor / Device for device presence */
                if ((vd == 0xffffffff) || (vd == 0x00000000)) {
                    if (func == 0) {
                        if ((bus > 0) && (dev == 0) && (maxbus == PCI_MAX_BUS))
                            maxbus = bus + 1;  // Just probe 1 more bus
                        break;
                    }
                    goto skip_and_check_htype;
                }
                if (maxbus < bus + 2)
                    maxbus = bus + 2;  // Found device -- also try next bus

                if ((p_vendor != PCI_ANY_ID) && (p_vendor != vendor))
                    continue;
                if ((p_device != PCI_ANY_ID) && (p_device != device))
                    continue;
                cmd = pci_read32(bus, dev, func, PCI_OFF_CMD);
                if (cmd == 0xffffffff)
                    cmd = 0;  // HW failure

                found++;

                classrev = pci_read32(bus, dev, func, PCI_OFF_REVISION);
                if ((classrev >> 16) == PCI_CLASS_PCI_BRIDGE)
                    maxbar = 2;

                if (flags & (FLAG_PCI_STATUS | FLAG_PCI_CLEAR)) {
                    pci_status(bus, dev, func, classrev);
                }
                if ((flags & ~(FLAG_PCI_STATUS | FLAG_PCI_CLEAR)) == 0) {
                    /* No display options specified, so end here */
                    goto skip_and_check_htype;
                }
                printf("%x.%x.%x%s %04x.%04x", bus, dev, func, (dev < 0x10) ? " " : "", vendor, device);
                printed = 0;
                if (p_cap != PCI_ANY_ID) {
                    printf("\n");
                    goto just_show_caps;
                }
                for (bar = 0; bar < maxbar; bar++) {
                    uint32_t pbase;
                    uint32_t bar_offset;
                    uint     bar_active;
                    const char *btype;

                    bar_offset = PCI_OFF_BAR0 + bar * 4;
                    size = get_bar_size(bus, dev, func, bar_offset, &base, 1);
                    if (size == 0)
                        continue;  // Not a writeable BAR
                    pbase = base;
                    if (pbase & BIT(0)) {
                        /* I/O space */
                        btype = "I/O";
                        base &= ~0x3;
                        if ((flags & FLAG_NO_TRANSLATE) == 0)
                            base += (uint32_t) bridge_io_base;
                        bar_active = cmd & BIT(0);  // I/O space enabled
                    } else {
                        /* Memory space */
                        btype = "M32";
                        if ((pbase & (BIT(2) | BIT(1))) == BIT(2))
                            btype = "M64";
                        base = translate_mem_address(base & ~0xf);
                        bar_active = cmd & BIT(1);  // Memory Space enabled
                    }
                    if (printed++ != 0)
                        printf("%16s", "");
                    printf(" %x %s%c", bar, btype, bar_active ? ' ' : '-');
                    if (((pbase & 1) == 0) &&
                        ((pbase & (BIT(2) | BIT(1))) == BIT(2))) {
                        uint32_t basehigh;
                        uint32_t sizehigh;
                        /* 64-bit BAR */
                        bar++;
                        bar_offset += 4;
                        sizehigh = get_bar_size(bus, dev, func, bar_offset,
                                                &basehigh, 0);
                        if (sizehigh == 0xffffffff)
                            sizehigh = 0;  // Overflow
                        else
                            sizehigh &= (0 - sizehigh);  // Convert mask to size
                        printf("%3x:%08x", basehigh, base, sizehigh, size);
                        if (sizehigh == 0)
                            printf(" %12x", size);
                        else
                            printf(" %3x:%08x", sizehigh, size);
                    } else {
                        printf("%12x %12x", base, size);
                    }
                    if (printed == 1) {  // First line
                        pci_show_vendordevice(vendor, device);
                        pci_show_class(classrev);
                    }
                    printf("\n");
                }

                /* Show expansion ROM */
                if ((classrev >> 16) == PCI_CLASS_PCI_BRIDGE)
                    bar_offset = PCI_OFF_BR_ROM_BAR;
                else
                    bar_offset = PCI_OFF_ROM_BAR;

                size = get_bar_size(bus, dev, func, bar_offset, &base, 0);
                if (size & 1) {
                    /* BAR exists */
                    uint bar_active;
                    size &= ~1;
                    size &= (0 - size);
                    base = translate_mem_address(base);
                    if (printed++ != 0)
                        printf("%16s", "");
                    bar_active = cmd & BIT(1);  // Memory Space enabled
                    printf("   ROM%c", bar_active ? ' ' : '-');
                    if (base & 1) {
                        /* BAR is enabled */
                        printf("%12x %12x", base & ~1, size);
                    } else {
                        printf("%12s %12x", "-", size);
                    }
                    if (printed == 1) {  // First line
                        pci_show_vendordevice(vendor, device);
                        pci_show_class(classrev);
                    }
                    printf("\n");
                }

                /* Handle bridge memory and I/O windows */
                if ((classrev >> 16) == PCI_CLASS_PCI_BRIDGE) {
                    uint32_t temp;
                    uint32_t base;
                    uint32_t limit;
                    uint32_t size;
                    temp = pci_read32(bus, dev, func, PCI_OFF_BR_IO_BASE);
                    base = (temp & 0xf0) << 8;
                    limit = (temp & 0xf000) + 0x1000;

                    if (temp & 1) {
                        /* 32-bit IO window */
                        temp = pci_read32(bus, dev, func, PCI_OFF_BR_IO_BASE_U);
                        base  += ((temp & 0x0000ffff) << 16);
                        limit += (temp & 0xffff0000);
                    }
                    if (limit <= base)
                        size = 0;
                    else
                        size = limit - base;
                    if ((flags & FLAG_NO_TRANSLATE) == 0)
                        base += (uint32_t) bridge_io_base;
                    if (printed)
                        printf("%16s", "");
                    printf("   WIO ");
                    if (size == 0)
                        printf("%12s %12x", "-", size);
                    else
                        printf("%12x %12x", base, size);

                    printed++;
                    if (printed == 1) {  // First line
                        pci_show_vendordevice(vendor, device);
                        pci_show_class(classrev);
                    }
                    printf("\n");

                    temp = pci_read32(bus, dev, func, PCI_OFF_BR_W32_BASE);
                    base  = (temp & 0x0000fff0) << 16;
                    limit = temp & 0xfff00000;
                    if (limit < base) {
                        size = 0;
                    } else {
                        limit += SIZE_1MB;
                        size = limit - base;
                    }
                    base = translate_mem_address(base);
                    printf("%22s ", "W32");
                    if (size == 0)
                        printf("%12s %12x\n", "-", size);
                    else
                        printf("%12x %12x\n", base, size);

                    temp = pci_read32(bus, dev, func, PCI_OFF_BR_W64_BASE);
                    base  = (temp & 0x0000fff0) << 16;
                    limit = temp & 0xfff00000;
                    if (limit < base) {
                        size = 0;
                    } else {
                        limit += SIZE_1MB;
                        size = limit - base;
                    }
                    printf("%22s ", "W64");
                    if (temp & 0x1) {
                        /* 64-bit prefetchable window */
                        uint32_t base_u;
                        uint32_t limit_u;
                        uint32_t size_u;
                        base_u = pci_read32(bus, dev, func,
                                            PCI_OFF_BR_W64_BASE_U);
                        limit_u = pci_read32(bus, dev, func,
                                             PCI_OFF_BR_W64_LIMIT_U);
                        if (base < bridge_map_base) { // wrapped
                            base_u++;
                            limit_u++;
                        }

                        /*
                         * If limit_u < base_u, then this window is not valid.
                         * If limit_u == base_u and limit < base, then this
                         * window is not valid.
                         * Otherwise, the window and size are valid.
                         */
                        if ((limit_u < base_u) ||
                            ((limit_u == base_u) && (limit < base))) {
                            printf("%12s %12x\n", "-", 0);
                        } else {
                            base = translate_mem_address(base);
                            size_u = limit_u - base_u;
                            printf("%3x:%08x %3x:%08x\n",
                                   base_u, base, size_u, size);
                        }
                    } else {
                        /* 32-bit prefetchable window */
                        if (size == 0) {
                            printf("%12s %12x\n", "-", size);
                        } else {
                            base = translate_mem_address(base);
                            printf("%12x %12x\n", base, size);
                        }
                    }
                    if ((classrev >> 16) == PCI_CLASS_PCI_BRIDGE) {
                        uint32_t sub = pci_read32(bus, dev, func,
                                                  PCI_OFF_BR_PRI_BUS);
                        if (flags & FLAG_VERBOSE) {
                            printf("    Bus %02x  SecBus %02x  SubBus %02x\n",
                               (uint8_t) sub, (uint8_t) (sub >> 8),
                               (uint8_t) (sub >> 16));
                        }
                        sub >>= 16;
                        sub &= 0xff;
                        if (maxbus < sub)
                            maxbus = sub;
                    }
                } else {
                    if (printed == 0) {  // Only the b.d.f and vendor printed
                        pci_show_vendordevice(vendor, device);
                        pci_show_class(classrev);
                        printf("\n");
                    }
                }
                if (flags & FLAG_VERBOSE) {
                    uint16_t status;
                    uint32_t subsys;
                    subsys = pci_read32(bus, dev, func, PCI_OFF_SUBSYSTEM_VID);
                    if ((subsys != 0) && (subsys != 0xffffffff)) {
                        printf("    Subsystem %04x.%04x ",
                               (uint16_t) subsys, subsys >> 16);
                        pci_show_vendordevice((uint16_t) subsys, subsys >> 16);
                        printf("\n");
                    }
                    printf("    CMD       ");
                    print_bits(pci_read16(bus, dev, func, PCI_OFF_CMD),
                               2, bits_pci_command);
                    printf("    STATUS    ");
                    status = pci_read16(bus, dev, func, PCI_OFF_STATUS);
                    print_bits(status, 2, bits_pci_status_primary);
                    printf("    Interrupt ");
                    print_bits(pci_read32(bus, dev, func, PCI_OFF_INT_LINE),
                               2, bits_pci_lat_gnt_int);

just_show_caps:
                    pci_show_caps(bus, dev, func, p_cap);
                    pci_show_device_specific(bus, dev, func, vendor, device,
                                             p_cap, classrev >> 16);
                }
                if (flags & FLAG_DUMP) {
                    uint omax = 64;
                    if (flags & FLAG_DDUMP)
                        omax = 256;
                    for (off = 0; off < omax; off++) {
                        if ((off & 0x0f) == 0)
                            printindentnum(off);
                        printf(" %02x", pci_read8(bus, dev, func, off));
                        if ((off & 0x0f) == 0x0f)
                            printf("\n");
                    }
                }

skip_and_check_htype:
                if (func == 0) {
                    if (!is_multifunction_device(bus, dev, func))
                        break;  // Not a multifunction device
                }
            }
        }
    }
    if (found == 0) {
        if ((p_bus != PCI_ANY_ID) &&
            (p_dev != PCI_ANY_ID) &&
            (p_func != PCI_ANY_ID)) {
            printf("%x.%x.%x not found\n", p_bus, p_dev, p_func);
        } else {
            printf("No device found\n");
        }
    }
}

static uint
lspci_tree_r(uint p_bus, uint p_dev, uint p_func, uint p_vendor, uint p_device,
             uint indent)
{
    uint     dev;
    uint     func;
    uint     secbus;
    uint     maxbus;
    uint     is_bridge;
    uint32_t classrev;
    uint16_t vendor;
    uint16_t device;
    uint32_t addr_min = 0xffffffff;
    uint32_t addr_max = 0;
    uint32_t ioaddr_min = 0xffffffff;
    uint32_t ioaddr_max = 0;
    uint32_t vd = pci_read32v(p_bus, p_dev, p_func, PCI_OFF_VENDOR);

    /* Check Vendor / Device for device presence */
    if ((vd == 0xffffffff) || (vd == 0x00000000))
        return (1);

    vendor = (uint16_t) vd;
    device = vd >> 16;

    if ((p_vendor != PCI_ANY_ID) && (p_vendor != vendor))
        return (0);
    if ((p_device != PCI_ANY_ID) && (p_device != device))
        return (0);

    uint rindent = 8 - indent - ((p_dev > 0xf) ? 1 : 0);
    printf("%*s%x.%x.%x%*s %04x.%04x ",
           indent, "", p_bus, p_dev, p_func, rindent, "", vendor, device);

    /* Is this a bridge? */
    classrev = pci_read32(p_bus, p_dev, p_func, PCI_OFF_REVISION);
    is_bridge = (classrev >> 16) == PCI_CLASS_PCI_BRIDGE;
    if (is_bridge) {
        uint32_t sub = pci_read32(p_bus, p_dev, p_func, PCI_OFF_BR_PRI_BUS);
        secbus = (uint8_t) (sub >> 8);
        maxbus = (uint8_t) (sub >> 16);

        if (secbus == maxbus)
            printf("[%02x]   ", secbus);
        else
            printf("[%02x-%02x]", secbus, maxbus);
    } else {
        /* Not a bridge */
        printf("       ");
    }

    if (flags & FLAG_VERBOSE) {
        /* Show memory range */
        uint     bar;
        uint     bar_offset;
        uint     maxbar = is_bridge ? 3 : 7;
        uint32_t base;
        uint32_t pbase;
        uint32_t size;

        for (bar = 0; bar < maxbar; bar++) {
            bar_offset = PCI_OFF_BAR0 + bar * 4;
            if (bar == maxbar - 1) {
                /* Optional ROM BAR */
                bar_offset = is_bridge ? PCI_OFF_BR_ROM_BAR :
                                         PCI_OFF_ROM_BAR;
            }

            size = get_bar_size(p_bus, p_dev, p_func, bar_offset, &pbase, 1);
            base = pbase;
            if (size == 0)
                continue;  // Not a writeable BAR
            if ((bar != maxbar - 1) && (pbase & BIT(0))) {
                /* I/O BAR */
                base &= ~0x3;
                if (size != 0) {
                    if (ioaddr_min > base)
                        ioaddr_min = base;
                    if (ioaddr_max < base + size)
                        ioaddr_max = base + size;
                }
            } else {
                /* Memory BAR */
                base &= ~0xf;
                if (((pbase & 1) == 0) &&
                    ((pbase & (BIT(2) | BIT(1))) == BIT(2))) {
                    /* 64-bit BAR -- ignore size if > 4 GB */
                    bar++;
                }
                base = translate_mem_address(base);
                if (size != 0) {
                    if (addr_min > base)
                        addr_min = base;
                    if (addr_max < base + size)
                        addr_max = base + size;
                }
            }
        }
        if (is_bridge) {
            /* Handle the bridge window registers */
            uint32_t temp;
            uint64_t base;
            uint32_t limit;
            uint64_t size;

            /* Try the 32-bit window */
            temp = pci_read32(p_bus, p_dev, p_func, PCI_OFF_BR_W32_BASE);
            base  = ((temp & 0x0000fff0) << 16);
            limit = ((temp & 0xfff00000U)) + 0x00100000;
            if (limit <= base)
                size = 0;
            else
                size = limit - base;
            base = translate_mem_address(base);

            if (size == 0) {
                /* Try the 64-bit window */
                temp = pci_read32(p_bus, p_dev, p_func, PCI_OFF_BR_W64_BASE);
                base  = ((temp & 0x0000fff0) << 16);
                limit = ((temp & 0xfff00000U)) + 0x00100000;
                if (limit <= base)
                    size = 0;
                else
                    size = limit - base;

                base = translate_mem_address(base);
                if (temp & 0x1) {
                    /* 64-bit prefetchable window */
                    uint32_t base_u = pci_read32(p_bus, p_dev, p_func,
                                                 PCI_OFF_BR_W64_BASE_U);
                    if (base_u != 0)
                        size = 0;  // Base is > 4GB, so ignore this window
                }
            }
            if (size != 0) {
                if (addr_min > base)
                    addr_min = base;
                if (addr_max < base + size)
                    addr_max = base + size;
            }

            /* Try the I/O window */
            temp = pci_read32(p_bus, p_dev, p_func, PCI_OFF_BR_IO_BASE);
            base = (temp & 0xf0) << 8;
            limit = (temp & 0xf000) + 0x1000;
            if (temp & 1) {
                /* 32-bit IO window */
                temp = pci_read32(p_bus, p_dev, p_func,
                                  PCI_OFF_BR_IO_BASE_U);
                base  += ((temp & 0x0000ffff) << 16);
                limit += (temp & 0xffff0000);
            }

            if (limit <= base)
                size = 0;
            else
                size = limit - base;

            if (size != 0) {
                if (ioaddr_min > base)
                    ioaddr_min = base;
                if (ioaddr_max < base + size)
                    ioaddr_max = base + size;
            }
        }
        if (addr_min < addr_max) {
            printf(" %08x-%08x", addr_min, addr_max);
        } else if (((flags & FLAG_VERBOSE_IO) == 0) &&
                 (ioaddr_min < ioaddr_max)) {
            if ((flags & FLAG_NO_TRANSLATE) == 0) {
                ioaddr_min += (uint32_t) bridge_io_base;
                ioaddr_max += (uint32_t) bridge_io_base;
            }
            printf(" %08x-%08x", ioaddr_min, ioaddr_max);
        } else {
            printf("%18s", "");
        }
        if (flags & FLAG_VERBOSE_IO) {
            if (ioaddr_min < ioaddr_max)
                printf(" %4x-%-5x", ioaddr_min, ioaddr_max);
            else
                printf("%11s", "");
        }
    }

    if (flags & FLAG_SHOW_CADDR) {
        uint32_t tbase = (uintptr_t) pci_cfg_base(p_bus, p_dev, p_func, 0);
        printf(" %08x", tbase);
    }
    pci_show_vendordevice(vendor, device);
    pci_show_class(classrev);
    printf("\n");
    if (is_bridge) {
        uint16_t br_control;
        if ((secbus <= p_bus) || (secbus == 0xff) || (maxbus < secbus))
            return (0);  // Downstream bus has not been configured

        /* Disable bridge secondary side error reporting */
        br_control = pci_read16(p_bus, p_dev, p_func, PCI_OFF_BR_CONTROL);
        pci_write16(p_bus, p_dev, p_func, PCI_OFF_BR_CONTROL, BIT(0));

        for (dev = 0; dev < PCI_MAX_DEV; dev++) {
            for (func = 0; func < PCI_MAX_FUNC; func++) {
                lspci_tree_r(secbus, dev, func, PCI_ANY_ID, PCI_ANY_ID,
                             indent + 2);
            }
        }

        /* Restore bridge secondary side error reporting */
        pci_write16(p_bus, p_dev, p_func, PCI_OFF_BR_CONTROL, br_control);
    }

    return (0);
}

static void
lspci_tree(uint bus, uint p_dev, uint p_func, uint p_vendor, uint p_device)
{
    uint    dev;
    uint    func;
    uint    maxdev;

    printf("Bus.Dev.Func  Vend.Dev  Buses   ");
    if (flags & FLAG_VERBOSE)
        printf("Memory Range      ");
    if (flags & FLAG_VERBOSE_IO)
        printf("I/O Range  ");
    if (flags & FLAG_SHOW_CADDR)
        printf("Config   ");
    printf("Description\n");

    if (bus == PCI_ANY_ID)
        bus = 0;

    maxdev = PCI_MAX_DEV;  // PCI supports 32 devices per bus
    if (bus == 0)
        maxdev = PCI_MAX_PHYS_SLOT;  // 5 physical slots

    /* Do tree of each device which is in a physical slot (bus 0) */
    for (dev = 0; dev < maxdev; dev++) {
        if ((p_dev != PCI_ANY_ID) && (p_dev != dev))
            continue;
        for (func = 0; func < PCI_MAX_FUNC; func++) {
            if ((p_func == PCI_ANY_ID) || (p_func == func)) {
                lspci_tree_r(bus, dev, func, p_vendor, p_device, 0);
            }
            if (func == 0) {
                if (!is_multifunction_device(bus, dev, func))
                    break;  // Not a multifunction device
            }
        }
    }
}

static rc_t
parse_pci_bdf(const char *arg, uint *p_bus, uint *p_dev, uint *p_func)
{
    const char *str = arg;
    int         count;

    /* Process PCI bus */
    if (*str == '.') {
        *p_bus = PCI_ANY_ID;
        str++;
    } else if (*str == '*') {
        *p_bus = PCI_ANY_ID;
        if (*(++str) == '.')
            str++;
    } else {
        count = strtox(str, 16, p_bus);
        if ((count < 1) || (count > 2) || (str[count] != '.')) {
            printf("Invalid PCI %s '%s' in PCI '%s'\n", "bus", str, arg);
            return (RC_BAD_PARAM);
        }
        str += count + 1;
    }

    /* Process PCI device */
    if (*str == '.') {
        *p_dev = PCI_ANY_ID;
        str++;
    } else if (*str == '*') {
        *p_bus = PCI_ANY_ID;
        if (*(++str) == '.')
            str++;
    } else {
        count = strtox(str, 16, p_dev);
        if ((count < 1) || (count > 2) || (str[count] != '.')) {
            printf("Invalid PCI %s '%s' in PCI '%s'\n", "device", str, arg);
            return (RC_BAD_PARAM);
        }
        str += count + 1;
    }

    /* Process PCI function */
    if (*str == '\0') {
        *p_func = PCI_ANY_ID;
    } else if (*str == '*') {
        *p_func = PCI_ANY_ID;
        str++;
    } else {
        count = strtox(str, 16, p_func);
        if ((count < 1) || (count > 2) || (str[count] != '\0')) {
            printf("Invalid PCI %s '%s' in PCI '%s'\n", "function", str, arg);
            return (RC_BAD_PARAM);
        }
        str += count;
    }
    return (RC_SUCCESS);
}

static rc_t
parse_pci_vendev(const char *arg, uint *p_vendor, uint *p_device)
{
    const char *str = arg;
    uint        count;

    /* Process PCI vendor */
    if (*str == '.') {
        *p_vendor = PCI_ANY_ID;
        str++;
    } else if (*str == '*') {
        *p_vendor = PCI_ANY_ID;
        if (*(++str) == '.')
            str++;
    } else {
        count = strtox(str, 16, p_vendor);
        if ((count < 1) || (count > 4) || (str[count] != '.')) {
            printf("Invalid PCI %s '%s' in PCI '%s'\n", "vendor", str, arg);
            return (RC_BAD_PARAM);
        }
        str += count + 1;
    }

    /* Process PCI device */
    if (*str == '\0') {
        *p_device = PCI_ANY_ID;
    } else if (*str == '*' || *str == '-') {
        *p_device = PCI_ANY_ID;
        str++;
    } else {
        count = strtox(str, 16, p_device);
        if ((count < 1) || (count > 4) || (str[count] != '\0')) {
            printf("Invalid PCI %s '%s' in PCI '%s'\n", "device", str, arg);
            return (RC_BAD_PARAM);
        }
        str += count;
    }
    return (RC_SUCCESS);
}

static const char cmd_options[] =
    "usage: pci <options>\n"
    "pci clear         clear PCI status registers (-c)\n"
    "pci help          show this help text (-h)\n"
    "pci ls [opts]     display PCI devices\n"
    "    opts: -a      show config space addresses\n"
    "          -d      show raw config space bytes\n"
    "          -n      do not translate PCI space addresses to CPU space\n"
    "          -t      display tree view of buses and devices\n"
    "          -v      verbose (decode everything)\n"
    "          b.d.f   specific bus, device, and/or function\n"
    "          v.d     specific vendor and/or device\n"
    "          cap c   specific PCI capability (or ecap x for extended cap)\n"
    "pci reset [b.d.f] reset the PCI bridge (-r) or (-rr) to hold in reset\n"
    "pci status        show PCI status registers (-s)\n"
    "";

typedef struct {
    const char *const short_name;
    const char *const long_name;
} long_to_short_t;
static const long_to_short_t long_to_short_main[] = {
    { "-d", "dump" },
    { "-k", "cap" },
    { "-K", "ecap" },
    { "-c", "clear" },
    { "-h", "help" },
    { "-l", "ls" },
    { "-r", "reset" },
    { "-s", "status" },
    { "-t", "tree" },
};

const char *
long_to_short(const char *ptr, const long_to_short_t *ltos, uint ltos_count)
{
    uint cur;

    for (cur = 0; cur < ltos_count; cur++)
        if (strcmp(ptr, ltos[cur].long_name) == 0)
            return (ltos[cur].short_name);
    return (ptr);
}

int
main(int argc, char **argv)
{
    int arg;
    int  pci_reset_bus_num  = -1;
    uint flag_pci_ls        = 0;
    uint flag_pci_tree      = 0;
    uint flag_pci_reset     = 0;
    uint bus                = PCI_ANY_ID;
    uint dev                = PCI_ANY_ID;
    uint func               = PCI_ANY_ID;
    uint vendor             = PCI_ANY_ID;
    uint device             = PCI_ANY_ID;
    uint cap                = PCI_ANY_ID;

    for (arg = 1; arg < argc; arg++) {
        const char *ptr;
        ptr = long_to_short(argv[arg], long_to_short_main,
                            ARRAY_SIZE(long_to_short_main));
        if (*ptr == '-') {
            while (*(++ptr) != '\0') {
                switch (*ptr) {
                    case 'a':
                        flags |= FLAG_SHOW_CADDR;
                        break;
                    case 'c':
                        flags |= FLAG_PCI_CLEAR;
                        break;
                    case 'd':
                        if (flags & FLAG_DUMP)
                            flags |= FLAG_DDUMP;
                        flags |= FLAG_DUMP;
                        break;
                    case 'k':
                        flags |= FLAG_PCI_CAP;
                        break;
                    case 'K':
                        flags |= FLAG_PCI_CAP | FLAG_PCI_ECAP;
                        break;
                    case 'l':
                        flags |= FLAG_PCI_LS;
                        break;
                    case 'n':
                        flags |= FLAG_NO_TRANSLATE;
                        break;
                    case 'r':
                        flag_pci_reset++;
                        break;
                    case 's':
                        flags |= FLAG_PCI_STATUS;
                        break;
                    case 't':
                        flag_pci_tree++;
                        break;
                    case 'v':
                        if (flags & FLAG_VERBOSE)
                            flags |= FLAG_VERBOSE_IO;
                        flags |= FLAG_VERBOSE;
                        break;
                    default:
                        goto usage;
                }
            }
        } else if ((flags & FLAG_PCI_CAP) && (cap == PCI_ANY_ID))  {
            int count = strtox(ptr, 16, &cap);
            if ((count < 1) || (count > 2) || (ptr[count] != '\0')) {
                printf("Unknown argument \"%s\"\n", ptr);
                goto usage;
            }
            if (flags & FLAG_PCI_ECAP)
                cap |= 0x10000;
        } else {
            /* Attempt to parse bus.dev.func */
            char *tmp;
            if (((tmp = strchr(ptr, '.')) != NULL) &&
                (strchr(tmp + 1, '.') != NULL)) {
                if ((bus != PCI_ANY_ID) || (dev != PCI_ANY_ID) ||
                    (func != PCI_ANY_ID)) {
                    printf("Only a single bus.dev.func may be provided\n");
                    exit(1);
                }

                if (parse_pci_bdf(ptr, &bus, &dev, &func) == RC_SUCCESS)
                    continue;
            } else if (strchr(ptr, '.') != NULL) {
                if ((vendor != PCI_ANY_ID) || (device != PCI_ANY_ID)) {
                    printf("Only a single vendor.device may be provided\n");
                    exit(1);
                }
                if (parse_pci_vendev(ptr, &vendor, &device) == RC_SUCCESS)
                    continue;
            }
            printf("Unknown argument \"%s\"\n", ptr);
usage:
            printf(cmd_options);
            exit(1);
        }
    }
    if (pci_bridge_is_present() == 0) {
        printf("Could not find PCI Root Bridge\n");
        exit(1);
    }

    if (flags != 0)
        flag_pci_ls++;

    if (!flag_pci_ls && !flag_pci_tree && !flag_pci_reset)
        goto usage;

    if (flag_pci_tree) {
        lspci_tree(bus, dev, func, vendor, device);
    } else if (flag_pci_ls) {
        lspci(bus, dev, func, vendor, device, cap);
    } else if (flag_pci_reset) {
        pci_bridge_control(pci_reset_bus_num, bus, dev, func,
                           (flag_pci_reset > 1) ?
                           FLAG_BRIDGE_RESET_HOLD : FLAG_BRIDGE_RESET);
    }

    exit(0);
}
