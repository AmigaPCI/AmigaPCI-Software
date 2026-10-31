/*
 * This is free and unencumbered software released into the public domain.
 * See the LICENSE file for additional details.
 *
 * Designed by Chris Hooper in 2025.
 *
 * ---------------------------------------------------------------------
 *
 * ST-Micro CubeMX USB Host library handling.
 */

#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "config.h"
#include "printf.h"
#include "utils.h"
#include "timer.h"
#include "clock.h"
#include "hiden.h"
#include "joystick.h"
#include "keyboard.h"
#include "mouse.h"
#include "cubeusb.h"
#include "usb.h"
#include <usbh_cdc.h>  // CDC
#include <usbh_hid.h>  // HID
#include <usbh_msc.h>  // MSC
#include <usbh_mtp.h>  // MTP
#include <usbh_hub.h>  // HUB
#include <usbh_xusb.h>  // XUSB
#include <usbh_hid_usage.h>  // HID_USAGE_*

void otg_fs_isr(void);
void otg_hs_ep1_out_isr(void);
void otg_hs_ep1_in_isr(void);
void otg_hs_wkup_isr(void);
void otg_hs_isr(void);

struct {
    uint otg_fs_ints;
    uint otg_hs_ints;
    uint otg_hs_ep1_in_ints;
    uint otg_hs_ep1_out_ints;
    uint otg_hs_wkup_ints;
} usb_stats;

/* USB port state */
typedef enum {
    APPLICATION_IDLE = 0,
    APPLICATION_START,
    APPLICATION_READY,
    APPLICATION_RUNNING,
    APPLICATION_DISCONNECT
} appstate_type;

static const char * const host_appstate[] = {
    "IDLE", "START", "READY", "RUNNING",
    "DISCONNECT",
};

static struct {
    uint64_t           recovery_timer;
    uint8_t            recovery_state;
    uint8_t            disabled;
    uint8_t            connected;
    volatile uint8_t   reset_pending;   // Set in IRQ; handled in cubeusb_poll()
    volatile uint8_t   clk_reset_pending; // PHY clock changed; port re-reset
    volatile uint8_t   clk_resetting;   // Re-reset in progress
    uint8_t            reinit_pending;  // Re-init port to retry enumeration
    uint8_t            enum_retries;    // Root device enumeration retries
    uint64_t           reinit_timer;    // When to re-init
    uint64_t           enum_retry_window; // Retry count resets after this
    volatile uint      frame;
    uint32_t           rec_episodes;    // Root port recoveries started
    uint32_t           rec_reinits;     // Port re-inits made by recovery
    uint32_t           rec_ok;          // Recoveries which succeeded
    uint32_t           rec_fail;        // Recoveries which gave up
} usbport[2];

USBH_HandleTypeDef *usb_root[2];

static void cubeusb_init_port(uint port);
static void cubeusb_shutdown_port(uint port);
// static uint hp_cur[2];  // Next handle to be processed by process_usb_ports()
static USBH_HandleTypeDef *phost_cur[2];  // Next handle to be processed

/*
 * Wall-clock bound on staying with one device while its control transfer
 * is in flight (see process_usb_ports()). phost->Timer can't be used for
 * this alone: it only advances on SOF, and there are no SOFs if the root
 * port has been disabled.
 */
#define USB_CTL_STAY_MAX_MS 3000

/*
 * Address 0 lock, per STM32 port (bus). Only one device on a bus may be
 * at the default address (0) at a time, so a hub must hold this from the
 * time it resets a port until the device on that port has been assigned
 * its address and configured. Without it, two hubs (e.g. the two hub
 * chips in a multi-port hub) resetting ports at the same time put two
 * devices at address 0, and both answer the same requests.
 * (Linux: usb_bus.devnum_next / hub address0_mutex.)
 */
#define USB_ADDR0_HOLD_MAX_MS 15000
static USBH_HandleTypeDef *addr0_owner[2];
static uint64_t            addr0_deadline[2];
static USBH_HandleTypeDef *ctl_stay_handle[2];
static uint64_t            ctl_stay_deadline[2];

/*
 * USB device addresses in use on each STM32 physical port (bus).
 * Bit N set means address N is assigned. Address 0 is the default
 * (unassigned) address and is never allocated.
 */
static uint32_t usb_addr_used[2][128 / 32];

/*
 * _hHCD[0] is the Full speed (OTG_FS) port
 * _hHCD[1] is the High speed (OTG_HS) port
 */
#ifndef USB_HOST_TOCAL
#define USB_HOST_TOCAL 7  // GUSBCFG.TOCAL (0-7); see USBH_LL_Init()
#endif

HCD_HandleTypeDef _hHCD[2];  // FS and HS device handles

/* Raw register access for a host port (HCD handle) */
#define USBx_HC_RAW(hhcd, ch) \
    ((USB_OTG_HostChannelTypeDef *)((uintptr_t)(hhcd)->Instance + \
                           USB_OTG_HOST_CHANNEL_BASE + \
                           (ch) * USB_OTG_HOST_CHANNEL_SIZE))
#define USBx_HPRT0_RAW(hhcd) \
    (*(volatile uint32_t *)((uintptr_t)(hhcd)->Instance + \
                            USB_OTG_HOST_PORT_BASE))

#ifdef USB_TRACE
/*
 * USB trace (USB_TRACE, defined in the Makefile).
 *
 * Two kinds of record are kept:
 *
 * 1. Low-speed enumeration records (per root port connect). For diagnosing
 *    low-speed devices which fail enumeration intermittently (e.g. the
 *    MacAlly MKEYE keyboard). Each connect on a root port starts a new
 *    record of port and channel events, timestamped. When the attempt
 *    ends, the record is kept:
 *      - as the last failure, if a low-speed device disconnected before its
 *        class became active;
 *      - as the last success, when a low-speed device's class becomes
 *        active (recording then stops until the next connect).
 *    Records are linear from connect (the start of an attempt is what
 *    matters); events past USB_TRACE_ENTRIES are counted, not stored.
 *
 * 2. A drop ring: the most recent USB_TRACE_RING events of both ports,
 *    recorded all the time, including hub port events (port status and
 *    change bits, hub port resets, child attach/detach) which the
 *    per-connect records don't see. When a link drops at runtime
 *    (unexpected root port disable, a hub reporting a connection / enable
 *    / over-current change on a port with a device, or an active device
 *    being detached), the ring records USB_TRACE_POST more events and is
 *    then frozen, so it holds what led up to the drop. "usb trace" prints
 *    it and re-arms the ring. On a root port disable, the host port and
 *    channel registers are also captured in the interrupt handler, before
 *    anything is torn down.
 *
 * Every event records the frame number and the time left in the frame
 * (HFNUM.FTREM converted to microseconds), to show where in the 1 ms frame
 * a transaction was queued, completed or failed. Channel interrupt events
 * also record which device / endpoint the channel was programmed for (the
 * channels other than EP0 are assigned dynamically, see "Virtual pipes"),
 * and whether it was a low-speed (PRE) transaction.
 *
 * A NAK and the channel-halted interrupt after it are collapsed into an
 * earlier NAK on the same channel and endpoint with a repeat count (one NAK
 * per frame while a device NAKs; interrupt endpoint polling NAKs every
 * bInterval). Interrupt endpoint submits are not recorded.
 *
 * Nothing is printed from interrupt context or the USB poll path.
 *
 * All trace code, including the hooks in stm32f2xx_hal_hcd.c and
 * usbh_hub.c and the "usb trace" command in pcmds.c, is under
 * #ifdef USB_TRACE. At runtime nothing is recorded or sampled unless the
 * USBTrace debug flag (DF_USB_TRACE) is set; "usb trace" still shows what
 * was recorded while it was set.
 */
#define UTR_ENABLED() ((config.debug_flag & DF_USB_TRACE) != 0)

#define USB_TRACE_ENTRIES 64
#ifndef USB_TRACE_RING
#define USB_TRACE_RING    96   // Drop ring entries (24 bytes each)
#endif
#define USB_TRACE_POST    8    // Events still recorded after a drop
#define UTR_COLLAPSE_BACK 6    // How far back to look for a NAK run

typedef enum {
    UTR_CONN = 1,   // Port connect detected (a=hprt0)
    UTR_RESET,      // Port reset issued (a=1 start, 0 end)
    UTR_PEN,        // Port enabled (a=hprt0)
    UTR_PDIS,       // Port disabled (a=hprt0, b=HFNUM)
    UTR_DISC,       // Disconnect (a=hprt0)
    UTR_SUB,        // Channel submit (a=ep_type|dir|token|ls|epnum,
                    //                 b=len|addr)
    UTR_INT,        // Channel interrupt (a=HCINT | endpoint << 16,
                    //                    b=HCTSIZ)
    UTR_RX,         // IN packet received (a=bytes)
    UTR_ACTIVE,     // Device class active (enumeration complete)
    UTR_HUB,        // Hub event (a=kind << 16 | hub addr << 8 | hub port,
                    //            b=data)
    UTR_IPXFR,      // Incomplete periodic transfer (a=GINTSTS)
} utr_ev_t;

/* UTR_HUB event kinds */
#define UTRH_STATUS  1   // GET_PORT_STATUS result (b=status | change << 16)
#define UTRH_BITMAP  2   // Status change endpoint bitmap (b=bitmap)
#define UTRH_RESET   3   // SET_FEATURE(PORT_RESET) done
#define UTRH_ATTACH  4   // Child attach started (b=lowspeed)
#define UTRH_DETACH  5   // Child detached (b=addr | active << 8 |
                         //                 gState << 16)
#define UTRH_DETACH_ACTIVE 0x100

/* Hub wPortChange bits */
#define UTR_C_PORT_CONNECTION  0x0001
#define UTR_C_PORT_ENABLE      0x0002
#define UTR_C_PORT_OVER_CURR   0x0008

/* Why the drop ring was frozen */
#define UTR_DROP_ROOT    1   // Root port disabled unexpectedly
#define UTR_DROP_HUBPORT 2   // Hub port change on a port with a device
#define UTR_DROP_DETACH  3   // Active device detached by the hub driver

typedef struct {
    uint32_t tick;        // Low 32 bits of timer tick
    uint32_t last_tick;   // Last repeat (NAK runs)
    uint32_t a;
    uint32_t b;
    uint16_t frame;       // HFNUM.FRNUM (13:0); bit 15 = STM32 port
    uint16_t rem;         // Microseconds left in the frame (HFNUM.FTREM)
    uint16_t last_rem;    // rem at the last repeat
    uint8_t  repeat;      // Additional identical NAKs collapsed (saturates)
    uint8_t  ch_ev;       // Channel (high nibble), event (low nibble)
} utr_ent_t;

typedef struct {
    utr_ent_t ent[USB_TRACE_ENTRIES];
    uint      count;      // Entries stored
    uint      dropped;    // Events not stored (record full)
    uint32_t  attempt;    // Attempt number on its port
    uint64_t  start_tick; // Time of connect (full tick)
    uint8_t   port;
    uint8_t   valid;
} utr_rec_t;

static utr_rec_t utr_live[2];  // Recording, per port
static utr_rec_t utr_fail;     // Last failed low-speed attempt
static utr_rec_t utr_ok;       // Last successful low-speed attempt

static struct {
    volatile uint8_t active;   // Recording into utr_live[port]
    volatile uint8_t ls_seen;  // Low-speed traffic in this attempt
    volatile uint8_t ok;       // This attempt reached class active
    uint32_t attempts;         // Connects with low-speed traffic
    uint32_t fails;
    uint32_t oks;
} utr[2];

static volatile uint8_t utr_printing;  // Don't replace records mid-print
static volatile uint32_t utr_skipped;  // Records not kept while printing

/* Drop ring */
#define UTR_RING_ARMED  0   // Recording
#define UTR_RING_DRAIN  1   // Drop seen; recording USB_TRACE_POST more
#define UTR_RING_FROZEN 2   // Holds a drop; not recording
#define UTR_RING_PAUSED 3   // Being printed ("usb trace ring")

static struct {
    utr_ent_t ent[USB_TRACE_RING];
    uint      head;        // Next slot to write
    uint      count;       // Valid entries
    uint32_t  seq;         // Events stored since armed
    uint32_t  drop_seq;    // seq at the drop
    uint64_t  drop_tick;
    uint32_t  drop_a;
    uint32_t  drop_b;
    uint32_t  drops;       // Drops seen since cleared (incl. while frozen)
    volatile uint8_t state;
    uint8_t   post;        // Events left to record after the drop
    uint8_t   reason;
    uint8_t   port;
} utr_ring;

/* Host registers captured in the interrupt handler at a root port disable */
#define UTR_SNAP_CH 12
typedef struct {
    uint8_t  valid;
    uint32_t count;        // Root port disables seen on this port
    uint64_t tick;
    uint32_t hprt0;        // As read by the port interrupt handler
    uint32_t hfnum;
    uint32_t hfir;
    uint32_t hcfg;
    uint32_t gintsts;
    uint32_t hnptxsts;
    uint32_t hptxsts;
    uint32_t hcchar[UTR_SNAP_CH];
    uint32_t hcint[UTR_SNAP_CH];
    uint32_t hctsiz[UTR_SNAP_CH];
} utr_pdsnap_t;

static utr_pdsnap_t utr_pdsnap[2];
static utr_pdsnap_t utr_pdsnap_noena[2];  // First disable with no channel enabled

/*
 * Root port drop summaries. For every unexpected root port disable (not
 * only the one the drop ring holds), record what was last submitted and
 * how long ago, and how long ago a hub port was last reset / a device
 * attached behind a hub, so the timing of all drops in a session can be
 * compared. Times are timer ticks (low 32 bits).
 */
static struct {
    uint32_t sub_tick;     // Last channel submit (any type)
    uint32_t sub_ep;       // Its target (utr_hcchar_ep() format)
    uint16_t sub_rem;      // Microseconds left in its frame
    uint8_t  sub_ch;
    uint8_t  sub_setup;    // 1 = SETUP token
    uint8_t  have_sub;
    uint8_t  have_reset;
    uint8_t  have_attach;
    uint8_t  stall_checked; // Stall sampler has looked at this submit
    uint8_t  reset_hub;    // Hub address and port of the last port reset
    uint8_t  reset_port;
    uint32_t reset_tick;   // SET_FEATURE(PORT_RESET) completed
    uint32_t attach_tick;  // Hub driver attach() of a child
} utr_last[2];

#define UTR_DSUM_MAX 16
typedef struct {
    uint32_t tick;         // Disable (low 32 bits of tick)
    uint32_t ms;           // Disable, ms after boot
    uint32_t sub_tick;
    uint32_t sub_ep;
    uint32_t reset_tick;
    uint32_t attach_tick;
    uint32_t sub_hctsiz;   // HCTSIZ of the last submit's channel at disable
    uint16_t sub_rem;
    uint16_t ena_mask;     // Channels enabled (CHENA) at disable
    uint8_t  port;
    uint8_t  sub_ch;
    uint8_t  sub_setup;
    uint8_t  flags;        // 1 have_sub, 2 have_reset, 4 have_attach
    uint8_t  reset_hub;
    uint8_t  reset_port;
} utr_dsum_t;

static utr_dsum_t utr_dsum[UTR_DSUM_MAX];
static uint       utr_dsum_count;     // Total recorded (index = count % MAX)

/*
 * Low-speed transfer stall sampler.
 *
 * A low-speed transaction through a hub (PRE, SETUP + DATA0 + handshake)
 * takes about 120 us. If a non-interrupt low-speed transfer is still active
 * on its channel UTR_STALL_AFTER_US after it was submitted, and there was
 * room for it in that frame, cubeusb_poll() busy-samples the root port line
 * state (HPRT.PLSTS) until the channel finishes, the port is disabled, or
 * UTR_STALL_MAX_US passes. That shows whether the bus was idle (J) while
 * the core waited, or something (the host, or a hub repeating a device)
 * kept it busy (K / SE0) up to the end of the frame. Interrupts stay
 * enabled while sampling, so there are gaps for interrupt handlers.
 */
#define UTR_STALL_AFTER_US 200
#define UTR_STALL_MIN_REM  250   // Skip submits with less room in the frame
#define UTR_STALL_MAX_US   1500
#define UTR_STALL_KEEP     8

typedef struct {
    uint32_t sub_tick;     // utr_last[].sub_tick of the sampled transfer
    uint32_t n[4];         // Samples per line state
    uint16_t first_nonj;   // us after start, 0xffff = none
    uint16_t last_nonj;    // us after start of the last non-J sample
    uint16_t dur_us;
    uint16_t changes;
    uint8_t  end;          // 1 channel done, 2 port disabled, 3 timeout
    uint8_t  valid;
} utr_pre_t;

typedef struct {
    uint32_t ms;           // When sampling started, ms after boot
    uint32_t sub_ep;       // Transfer target (utr_hcchar_ep() format)
    uint32_t n[4];         // Samples per line state: SE0, J, K, SE1
    uint32_t hnptxsts;     // At sampling start
    uint32_t hctsiz;       // At sampling start
    uint16_t max_run[4];   // Longest run of each line state, us
    uint16_t sub_rem;      // us left in the frame at submit
    uint16_t start_us;     // Sampling started this long after submit
    uint16_t dur_us;       // Sampled for
    uint16_t first_nonj;   // First non-J seen, us after submit (0xffff none)
    uint16_t changes;      // Line state changes seen
    uint8_t  sub_ch;
    uint8_t  sub_setup;
    uint8_t  port;
    uint8_t  end;          // 1 transfer done, 2 port disabled, 3 timeout
    uint8_t  have_pre;     // pre is from this transfer's start
    utr_pre_t pre;         // Start sample (usb_trace_started())
} utr_stall_t;

static utr_stall_t utr_stall[UTR_STALL_KEEP];
static uint        utr_stall_count;

#define USBx_HOST_RAW(hhcd) \
    ((USB_OTG_HostTypeDef *)((uintptr_t)(hhcd)->Instance + \
                             USB_OTG_HOST_BASE))

static uint
utr_port(const HCD_HandleTypeDef *hhcd)
{
    return ((hhcd == &_hHCD[0]) ? 0 : 1);
}

/* Microseconds left in the current frame, from HFNUM.FTREM and HFIR */
static uint16_t
utr_frame_rem_us(HCD_HandleTypeDef *hhcd, uint32_t hfnum)
{
    uint32_t hfir  = USBx_HOST_RAW(hhcd)->HFIR & 0xffff;
    uint32_t ftrem = hfnum >> 16;

    if (hfir == 0)
        return (0xffff);
    return ((uint16_t) ((ftrem * 1000) / hfir));
}

/*
 * Compress the HCCHAR fields which identify a channel's target into 15
 * bits: device address (6:0), endpoint number (10:7), IN (11),
 * low-speed / PRE (12), endpoint type (14:13).
 */
static uint32_t
utr_hcchar_ep(uint32_t hcchar)
{
    return (((hcchar >> 22) & 0x7f) |                // DAD
            (((hcchar >> 11) & 0xf) << 7) |          // EPNUM
            (((hcchar >> 15) & 1) << 11) |           // EPDIR
            (((hcchar >> 17) & 1) << 12) |           // LSDEV
            (((hcchar >> 18) & 3) << 13));           // EPTYP
}

static void
utr_put(utr_ent_t *e, uint8_t ev, uint8_t ch, uint32_t a, uint32_t b,
        uint32_t tick, uint16_t frame, uint16_t rem)
{
    e->tick      = tick;
    e->last_tick = tick;
    e->frame     = frame;
    e->rem       = rem;
    e->last_rem  = rem;
    e->repeat    = 0;
    e->ch_ev     = (uint8_t) (((ch & 0xf) << 4) | (ev & 0xf));
    e->a         = a;
    e->b         = b;
}

/*
 * Try to absorb a channel interrupt into an earlier NAK entry for the same
 * port, channel and endpoint, looking back past other channels' interrupt
 * entries only. <newest> is the index of the newest entry, <avail> the
 * number of entries which may be examined. Returns 1 if absorbed.
 */
static uint
utr_collapse(utr_ent_t *base, uint size, uint newest, uint avail, uint port,
             uint ch, uint32_t a, uint32_t b, uint32_t tick, uint16_t rem)
{
    uint32_t hcint = a & 0xffff;
    uint k;

    if (avail > UTR_COLLAPSE_BACK)
        avail = UTR_COLLAPSE_BACK;
    for (k = 0; k < avail; k++) {
        utr_ent_t *e = &base[(newest + size - k) % size];
        uint e_ev = e->ch_ev & 0xf;
        uint e_ch = e->ch_ev >> 4;

        if (e_ev != UTR_INT)
            return (0);
        if (((uint) (e->frame >> 15) != port) || (e_ch != ch))
            continue;  // Another channel's interrupt; look further back
        if ((((e->a & 0xffff) & ~USB_OTG_HCINT_CHH) != USB_OTG_HCINT_NAK) ||
            ((e->a >> 16) != (a >> 16))) {
            return (0);
        }
        if (hcint == USB_OTG_HCINT_CHH)
            return (1);  // Expected halt after the NAK
        if (((hcint & ~USB_OTG_HCINT_CHH) == USB_OTG_HCINT_NAK) &&
            (e->b == b)) {
            if (e->repeat != 0xff)
                e->repeat++;
            e->last_tick = tick;
            e->last_rem  = rem;
            return (1);
        }
        return (0);
    }
    return (0);
}

static void
utr_add(HCD_HandleTypeDef *hhcd, uint8_t ev, uint8_t ch, uint32_t a,
        uint32_t b)
{
    uint port = utr_port(hhcd);
    utr_rec_t *r = &utr_live[port];
    uint32_t tick;
    uint32_t hfnum;
    uint16_t frame;
    uint16_t rem;
    uint32_t flags;

    if (!UTR_ENABLED())
        return;
    if (!utr[port].active && (utr_ring.state != UTR_RING_ARMED) &&
        (utr_ring.state != UTR_RING_DRAIN)) {
        return;
    }

    tick  = (uint32_t) timer_tick_get();
    hfnum = USBx_HOST_RAW(hhcd)->HFNUM;
    frame = (uint16_t) ((hfnum & 0x3fff) | (port << 15));
    rem   = utr_frame_rem_us(hhcd, hfnum);

    flags = __get_PRIMASK();
    __disable_irq();

    /* Per-connect low-speed enumeration record */
    if (utr[port].active) {
        if ((ev == UTR_INT) && (r->count > 0) &&
            utr_collapse(r->ent, USB_TRACE_ENTRIES, r->count - 1, r->count,
                         port, ch, a, b, tick, rem)) {
            /* Absorbed into a NAK run */
        } else if (r->count >= USB_TRACE_ENTRIES) {
            r->dropped++;
        } else {
            utr_put(&r->ent[r->count++], ev, ch, a, b, tick, frame, rem);
        }
    }

    /* Drop ring */
    if ((utr_ring.state == UTR_RING_ARMED) ||
        (utr_ring.state == UTR_RING_DRAIN)) {
        uint newest = (utr_ring.head + USB_TRACE_RING - 1) % USB_TRACE_RING;
        uint absorbed = 0;

        if (utr_ring.count > 0) {
            utr_ent_t *e = &utr_ring.ent[newest];
            if (ev == UTR_INT) {
                absorbed = utr_collapse(utr_ring.ent, USB_TRACE_RING, newest,
                                        utr_ring.count, port, ch, a, b,
                                        tick, rem);
            } else if ((ev == UTR_IPXFR) &&
                       ((e->ch_ev & 0xf) == UTR_IPXFR) &&
                       ((uint) (e->frame >> 15) == port)) {
                if (e->repeat != 0xff)
                    e->repeat++;
                e->last_tick = tick;
                e->last_rem  = rem;
                absorbed = 1;
            }
        }
        if (!absorbed) {
            utr_put(&utr_ring.ent[utr_ring.head], ev, ch, a, b, tick, frame,
                    rem);
            utr_ring.head = (utr_ring.head + 1) % USB_TRACE_RING;
            if (utr_ring.count < USB_TRACE_RING)
                utr_ring.count++;
            utr_ring.seq++;
            if (utr_ring.state == UTR_RING_DRAIN) {
                if ((utr_ring.post == 0) || (--utr_ring.post == 0))
                    utr_ring.state = UTR_RING_FROZEN;
            }
        }
    }
    __set_PRIMASK(flags);
}

/* A link dropped: keep the ring once USB_TRACE_POST more events are in */
static void
utr_ring_drop(uint port, uint reason, uint32_t a, uint32_t b)
{
    uint32_t flags;

    if (!UTR_ENABLED())
        return;
    flags = __get_PRIMASK();
    __disable_irq();
    utr_ring.drops++;
    if (utr_ring.state == UTR_RING_ARMED) {
        utr_ring.reason    = (uint8_t) reason;
        utr_ring.port      = (uint8_t) port;
        utr_ring.drop_tick = timer_tick_get();
        utr_ring.drop_a    = a;
        utr_ring.drop_b    = b;
        utr_ring.drop_seq  = utr_ring.seq;
        utr_ring.post      = USB_TRACE_POST;
        utr_ring.state     = (USB_TRACE_POST > 0) ? UTR_RING_DRAIN :
                                                    UTR_RING_FROZEN;
    }
    __set_PRIMASK(flags);
}

static void
utr_ring_rearm(void)
{
    uint32_t flags = __get_PRIMASK();
    __disable_irq();
    utr_ring.head  = 0;
    utr_ring.count = 0;
    utr_ring.seq   = 0;
    utr_ring.post  = 0;
    utr_ring.state = UTR_RING_ARMED;
    utr_pdsnap[0].valid = 0;
    utr_pdsnap[1].valid = 0;
    __set_PRIMASK(flags);
}

/* Connect: start a new record */
static void
utr_start(uint port)
{
    utr_rec_t *r = &utr_live[port];

    utr[port].active  = 0;
    if (!UTR_ENABLED())
        return;
    utr[port].ls_seen = 0;
    utr[port].ok      = 0;
    r->count      = 0;
    r->dropped    = 0;
    r->port       = (uint8_t) port;
    r->valid      = 1;
    r->start_tick = timer_tick_get();
    utr[port].active = 1;
}

/* Keep the live record as the last failure or last success */
static void
utr_keep(uint port, utr_rec_t *dst)
{
    if (utr_printing) {
        utr_skipped++;
        return;
    }
    memcpy(dst, &utr_live[port], sizeof (*dst));
}

/* Disconnect (interrupt context) */
static void
utr_stop(uint port)
{
    if (!utr[port].active)
        return;
    utr[port].active = 0;
    if (utr[port].ls_seen && !utr[port].ok) {
        utr[port].fails++;
        utr_live[port].attempt = utr[port].attempts;
        utr_keep(port, &utr_fail);
    }
}

/* A device's class became active (task context) */
static void
utr_class_active(USBH_HandleTypeDef *phost)
{
    uint port = phost->id & 1;

    if (!UTR_ENABLED())
        return;
    if (phost->device.speed != USBH_SPEED_LOW)
        return;  // e.g. a hub in front of the low-speed device
    if (!utr[port].active || utr[port].ok || !utr[port].ls_seen)
        return;
    utr_add(&_hHCD[port], UTR_ACTIVE, 0, 0, 0);
    utr[port].ok = 1;
    utr[port].oks++;
    utr[port].active = 0;  // Keep the start of the successful attempt
    utr_live[port].attempt = utr[port].attempts;
    utr_keep(port, &utr_ok);
}

static void
utr_print_hcint(uint32_t v)
{
    static const char * const names[] = {
        "XFRC", "CHH", "AHBERR", "STALL", "NAK", "ACK", "NYET", "TXERR",
        "BBERR", "FRMOR", "DTERR"
    };
    int bit;
    for (bit = 0; bit < ARRAY_SIZE(names); bit++)
        if (v & BIT(bit))
            printf(" %s", names[bit]);
}

static const char * const utr_eptype[] = { "CTRL", "ISOC", "BULK", "INTR" };

/* Print a compressed endpoint (see utr_hcchar_ep()) as [addr:ep TYPE LS] */
static void
utr_print_ep(uint32_t ep)
{
    printf("[%lu:%02lx %s%s]",
           (unsigned long) (ep & 0x7f),
           (unsigned long) (((ep >> 7) & 0xf) | (((ep >> 11) & 1) << 7)),
           utr_eptype[(ep >> 13) & 3], (ep & BIT(12)) ? " LS" : "");
}

static void
utr_print_hprt(uint32_t v)
{
    static const char * const pls[] = { "SE0", "J", "K", "SE1" };
    static const char * const spd[] = { "HS", "FS", "LS", "?" };
    if (v & USB_OTG_HPRT_PCSTS)   printf(" CONN");
    if (v & USB_OTG_HPRT_PCDET)   printf(" CONNDET");
    if (v & USB_OTG_HPRT_PENA)    printf(" ENA");
    if (v & USB_OTG_HPRT_PENCHNG) printf(" ENCHG");
    if (v & USB_OTG_HPRT_POCA)    printf(" OC");
    if (v & USB_OTG_HPRT_POCCHNG) printf(" OCCHG");
    if (v & USB_OTG_HPRT_PRES)    printf(" RESUME");
    if (v & USB_OTG_HPRT_PSUSP)   printf(" SUSP");
    if (v & USB_OTG_HPRT_PRST)    printf(" RST");
    if (v & USB_OTG_HPRT_PPWR)    printf(" PWR");
    printf(" line=%s %s", pls[(v >> 10) & 3], spd[(v >> 17) & 3]);
}

static void
utr_print_hub_status(uint32_t v)
{
    static const char * const st[] = {
        "CONN", "EN", "SUSP", "OC", "RST", NULL, NULL, NULL,
        "PWR", "LS", "HS", "TEST", "IND"
    };
    static const char * const ch[] = {
        "C_CONN", "C_EN", "C_SUSP", "C_OC", "C_RST"
    };
    int bit;
    for (bit = 0; bit < ARRAY_SIZE(st); bit++)
        if ((st[bit] != NULL) && (v & BIT(bit)))
            printf(" %s", st[bit]);
    for (bit = 0; bit < ARRAY_SIZE(ch); bit++)
        if (v & BIT(bit + 16))
            printf(" %s", ch[bit]);
}

static void
utr_print_ent(const utr_ent_t *e, uint32_t t0, uint show_port)
{
    uint ev = e->ch_ev & 0xf;
    uint ch = e->ch_ev >> 4;

    printf("  %8lu us ", (unsigned long) timer_tick_to_usec(
                                             (uint32_t) (e->tick - t0)));
    if (show_port)
        printf("U%u ", e->frame >> 15);
    printf("f%04x r%3u ", e->frame & 0x3fff, e->rem);
    switch (ev) {
        case UTR_CONN:
            printf("CONNECT hprt0=%08lx\n", (unsigned long) e->a);
            break;
        case UTR_RESET:
            printf("PORT RESET %s\n", e->a ? "start" : "end");
            break;
        case UTR_PEN:
            printf("PORT ENABLED hprt0=%08lx\n", (unsigned long) e->a);
            break;
        case UTR_PDIS:
            printf("PORT DISABLED hprt0=%08lx", (unsigned long) e->a);
            utr_print_hprt(e->a);
            printf("\n");
            break;
        case UTR_DISC:
            printf("DISCONNECT hprt0=%08lx\n", (unsigned long) e->a);
            break;
        case UTR_SUB: {
            uint type  = e->a & 3;
            uint dir   = (e->a >> 4) & 1;
            uint token = (e->a >> 8) & 1;
            uint ls    = (e->a >> 9) & 1;
            uint epnum = (e->a >> 12) & 0xf;
            printf("ch%u [%lu:%02x %s%s] SUBMIT %s %s len=%lu\n", ch,
                   (unsigned long) (e->b >> 16), epnum | (dir << 7),
                   utr_eptype[type], ls ? " LS" : "",
                   dir ? "IN" : "OUT", token ? "DATA" : "SETUP",
                   (unsigned long) (e->b & 0xffff));
            break;
        }
        case UTR_INT:
            printf("ch%u ", ch);
            utr_print_ep(e->a >> 16);
            printf(" INT %04lx", (unsigned long) (e->a & 0xffff));
            utr_print_hcint(e->a & 0xffff);
            printf("  xfrsiz=%lu pktcnt=%lu",
                   (unsigned long) (e->b & 0x7ffff),
                   (unsigned long) ((e->b >> 19) & 0x3ff));
            if (e->repeat != 0) {
                printf("  x%u%s until %lu us r%u",
                       e->repeat + 1U, (e->repeat == 0xff) ? "+" : "",
                       (unsigned long) timer_tick_to_usec(
                           (uint32_t) (e->last_tick - t0)), e->last_rem);
            }
            printf("\n");
            break;
        case UTR_RX:
            printf("ch%u RX %lu bytes\n", ch, (unsigned long) e->a);
            break;
        case UTR_ACTIVE:
            printf("CLASS ACTIVE (enumeration complete)\n");
            break;
        case UTR_HUB: {
            uint kind = e->a >> 16;
            uint addr = (e->a >> 8) & 0x7f;
            uint hport = e->a & 0xff;
            printf("HUB %u", addr);
            switch (kind) {
                case UTRH_STATUS:
                    printf(" port %u STATUS %04lx change %04lx", hport,
                           (unsigned long) (e->b & 0xffff),
                           (unsigned long) (e->b >> 16));
                    utr_print_hub_status(e->b);
                    break;
                case UTRH_BITMAP:
                    printf(" status change bitmap %02lx",
                           (unsigned long) e->b);
                    break;
                case UTRH_RESET:
                    printf(" port %u RESET issued", hport);
                    break;
                case UTRH_ATTACH:
                    printf(" port %u ATTACH %s", hport,
                           e->b ? "low-speed" : "full-speed");
                    break;
                case UTRH_DETACH:
                    printf(" port %u DETACH addr %lu gState %lu%s", hport,
                           (unsigned long) (e->b & 0xff),
                           (unsigned long) (e->b >> 16),
                           (e->b & UTRH_DETACH_ACTIVE) ? " (was active)" : "");
                    break;
                default:
                    printf(" ? kind=%u", kind);
                    break;
            }
            printf("\n");
            break;
        }
        case UTR_IPXFR:
            printf("INCOMPLETE PERIODIC (IPXFR) GINTSTS=%08lx",
                   (unsigned long) e->a);
            if (e->repeat != 0) {
                printf("  x%u%s until %lu us",
                       e->repeat + 1U, (e->repeat == 0xff) ? "+" : "",
                       (unsigned long) timer_tick_to_usec(
                           (uint32_t) (e->last_tick - t0)));
            }
            printf("\n");
            break;
        default:
            printf("? ev=%u\n", ev);
            break;
    }
}

static void
utr_dump(const char *what, const utr_rec_t *r)
{
    uint32_t t0;
    uint cur;

    if (!r->valid) {
        printf("No %s recorded\n", what);
        return;
    }
    printf("USB%u %s: attempt %lu, %lu s after boot, %u events",
           r->port, what, (unsigned long) r->attempt,
           (unsigned long) (timer_tick_to_usec(r->start_tick) / 1000000),
           r->count);
    if (r->dropped != 0)
        printf(" (+%u not stored)", r->dropped);
    printf(":\n");

    t0 = (r->count > 0) ? r->ent[0].tick : 0;
    for (cur = 0; cur < r->count; cur++)
        utr_print_ent(&r->ent[cur], t0, 0);
}

static void
utr_dump_pdsnap_s(uint port, const utr_pdsnap_t *s, const char *what)
{
    uint nch = _hHCD[port].Init.Host_channels;
    uint ch;

    if (!s->valid)
        return;
    printf("USB%u root port disable%s captured %lu ms after boot:\n"
           "  hprt0=%08lx", port, what,
           (unsigned long) (timer_tick_to_usec(s->tick) / 1000),
           (unsigned long) s->hprt0);
    utr_print_hprt(s->hprt0);
    printf("\n  HFNUM=%08lx (frame %04lx, FTREM %lu of HFIR %lu = %lu us "
           "left)\n",
           (unsigned long) s->hfnum, (unsigned long) (s->hfnum & 0x3fff),
           (unsigned long) (s->hfnum >> 16),
           (unsigned long) (s->hfir & 0xffff),
           (unsigned long) ((s->hfir & 0xffff) ?
                            ((s->hfnum >> 16) * 1000) / (s->hfir & 0xffff) :
                            0));
    if (((s->hfnum & 0x3fff) == 0x3fff) &&
        ((s->hfnum >> 16) == (s->hfir & 0xffff))) {
        printf("  (frame counter already stopped by the disable; see the "
               "drop summary for timing)\n");
    }
    printf("  HCFG=%08lx GINTSTS=%08lx\n",
           (unsigned long) s->hcfg, (unsigned long) s->gintsts);
    printf("  HNPTXSTS=%08lx (NP queue top: ch%lu type %lu%s, %lu free)\n",
           (unsigned long) s->hnptxsts,
           (unsigned long) ((s->hnptxsts >> 27) & 0xf),
           (unsigned long) ((s->hnptxsts >> 25) & 3),
           (s->hnptxsts & BIT(24)) ? " term" : "",
           (unsigned long) ((s->hnptxsts >> 16) & 0xff));
    printf("  HPTXSTS=%08lx  (P queue top: ch%lu type %lu%s %s, %lu free)\n",
           (unsigned long) s->hptxsts,
           (unsigned long) ((s->hptxsts >> 27) & 0xf),
           (unsigned long) ((s->hptxsts >> 25) & 3),
           (s->hptxsts & BIT(24)) ? " term" : "",
           (s->hptxsts & BIT(31)) ? "odd" : "even",
           (unsigned long) ((s->hptxsts >> 16) & 0xff));
    if (nch > UTR_SNAP_CH)
        nch = UTR_SNAP_CH;
    for (ch = 0; ch < nch; ch++) {
        if ((s->hcchar[ch] == 0) && (s->hcint[ch] == 0))
            continue;
        printf("  ch%u HCCHAR=%08lx ", ch, (unsigned long) s->hcchar[ch]);
        utr_print_ep(utr_hcchar_ep(s->hcchar[ch]));
        printf("%s%s HCTSIZ=%08lx HCINT=%04lx",
               (s->hcchar[ch] & USB_OTG_HCCHAR_CHENA) ? " ENA" : "",
               (s->hcchar[ch] & USB_OTG_HCCHAR_ODDFRM) ? " ODD" : "",
               (unsigned long) s->hctsiz[ch], (unsigned long) s->hcint[ch]);
        utr_print_hcint(s->hcint[ch]);
        printf("\n");
    }
}

static void
utr_dump_pdsnap(uint port)
{
    const utr_pdsnap_t *s = &utr_pdsnap[port];

    if (s->count == 0)
        return;
    printf("USB%u root port disables: %lu\n", port, (unsigned long) s->count);
    utr_dump_pdsnap_s(port, s, "");
}

/*
 * Low-speed transfer start sampler. Right after a non-interrupt
 * low-speed transfer is started on a channel (USB_HC_StartXfer()), sample
 * the root port line state for up to UTR_PRE_MAX_US or until the channel
 * is no longer enabled. A normal PRE + SETUP + DATA0 + ACK through the hubs
 * takes about 120 us and shows as K / SE0 activity, so this both checks
 * that line sampling sees bus traffic and shows, for a transfer which
 * later stalls, whether anything was sent at all.
 */
#define UTR_PRE_MAX_US 300


static utr_pre_t utr_pre_last[2];
static struct {
    uint32_t sampled;
    uint32_t done_active;   // Finished in time, bus activity seen
    uint32_t done_idle;     // Finished in time, no activity seen
    uint32_t open_active;   // Still active at the limit, activity seen
    uint32_t open_idle;     // Still active at the limit, no activity seen
} utr_pre_stats;

void usb_trace_started(HCD_HandleTypeDef *hhcd, uint32_t chnum);

/* Low-speed OUT starts (from the start sampler): [SETUP][hung] */
static uint32_t utr_lsout_starts[2][2];

/* Called from the HAL right after USB_HC_StartXfer() (task context) */
void
usb_trace_started(HCD_HandleTypeDef *hhcd, uint32_t chnum)
{
    uint port = utr_port(hhcd);
    volatile USB_OTG_HostChannelTypeDef *hc = USBx_HC_RAW(hhcd, chnum);
    utr_pre_t *pr = &utr_pre_last[port];
    uint32_t hcchar = hc->HCCHAR;
    uint32_t t0;
    uint32_t now;
    uint last = 0xff;

    if (!UTR_ENABLED())
        return;
    if (__get_IPSR() != 0)
        return;                                    // Not from an IRQ
    if ((hcchar & USB_OTG_HCCHAR_LSDEV) == 0)
        return;                                    // Not low-speed / PRE
    if (((hcchar & USB_OTG_HCCHAR_EPTYP) >> 18) == EP_TYPE_INTR)
        return;
    if ((USBx_HPRT0_RAW(hhcd) & USB_OTG_HPRT_PENA) == 0)
        return;

    memset(pr, 0, sizeof (*pr));
    pr->sub_tick   = utr_last[port].sub_tick;
    pr->first_nonj = 0xffff;
    t0 = (uint32_t) timer_tick_get();
    for (;;) {
        uint32_t hprt = USBx_HPRT0_RAW(hhcd);
        uint32_t chr  = hc->HCCHAR;
        uint     ls   = (hprt >> 10) & 3;
        uint32_t us;

        now = (uint32_t) timer_tick_get();
        us  = (uint32_t) timer_tick_to_usec((uint32_t) (now - t0));
        pr->n[ls]++;
        if (ls != 1) {
            if (pr->first_nonj == 0xffff)
                pr->first_nonj = (uint16_t) us;
            pr->last_nonj = (uint16_t) us;
        }
        if ((last != 0xff) && (ls != last))
            pr->changes++;
        last = ls;
        if ((hprt & USB_OTG_HPRT_PENA) == 0) {
            pr->end = 2;
            break;
        }
        if ((chr & USB_OTG_HCCHAR_CHENA) == 0) {
            pr->end = 1;
            break;
        }
        if (us > UTR_PRE_MAX_US) {
            pr->end = 3;
            break;
        }
    }
    pr->dur_us = (uint16_t) timer_tick_to_usec((uint32_t) (now - t0));
    pr->valid  = 1;

    /* Low-speed OUT (SETUP or data): did it start, or hang? */
    if ((hcchar & USB_OTG_HCCHAR_EPDIR) == 0) {
        uint setup = (hhcd->hc[chnum].data_pid == HC_PID_SETUP);
        if ((pr->end == 3) && (pr->first_nonj == 0xffff))
            utr_lsout_starts[setup][1]++;    // Hung: nothing on the bus
        else if ((pr->end == 1) || (pr->first_nonj != 0xffff))
            utr_lsout_starts[setup][0]++;    // Started
    }

    utr_pre_stats.sampled++;
    if (pr->end == 1) {
        if (pr->first_nonj != 0xffff)
            utr_pre_stats.done_active++;
        else
            utr_pre_stats.done_idle++;
    } else if (pr->end == 3) {
        if (pr->first_nonj != 0xffff)
            utr_pre_stats.open_active++;
        else
            utr_pre_stats.open_idle++;
    }
}

static void
utr_print_pre(const utr_pre_t *pr)
{
    static const char * const ends[] = {
        "?", "channel done", "PORT DISABLED", "still active"
    };
    static const char * const lsn[] = { "SE0", "J", "K", "SE1" };
    uint k;

    printf("     from start: %u us ->  %s; line:", pr->dur_us,
           ends[(pr->end < 4) ? pr->end : 0]);
    for (k = 0; k < 4; k++)
        printf(" %s %lu", lsn[k], (unsigned long) pr->n[k]);
    printf("; %u changes", pr->changes);
    if (pr->first_nonj != 0xffff) {
        printf("; activity +%u..+%u us", pr->first_nonj, pr->last_nonj);
    } else {
        printf("; NO activity");
    }
    printf("\n");
}

/* Called from cubeusb_poll() (task context) */
static void
utr_stall_check(uint port)
{
    HCD_HandleTypeDef *hhcd = &_hHCD[port];
    volatile USB_OTG_HostChannelTypeDef *hc;
    utr_stall_t *st;
    uint32_t sub_tick;
    uint32_t t0;
    uint32_t run_start;
    uint32_t now;
    uint32_t elapsed;
    uint last = 0xff;
    uint ch;

    if (!UTR_ENABLED())
        return;
    if (!utr_last[port].have_sub || utr_last[port].stall_checked)
        return;
    if ((utr_last[port].sub_ep & BIT(12)) == 0) {      // Not low-speed/PRE
        utr_last[port].stall_checked = 1;
        return;
    }
    if (((utr_last[port].sub_ep >> 13) & 3) == EP_TYPE_INTR) {
        utr_last[port].stall_checked = 1;
        return;
    }
    ch = utr_last[port].sub_ch;
    if (ch >= hhcd->Init.Host_channels) {
        utr_last[port].stall_checked = 1;
        return;
    }
    hc = USBx_HC_RAW(hhcd, ch);
    if ((hc->HCCHAR & USB_OTG_HCCHAR_CHENA) == 0) {
        utr_last[port].stall_checked = 1;              // Finished normally
        return;
    }
    sub_tick = utr_last[port].sub_tick;
    elapsed = (uint32_t) timer_tick_to_usec(
                  (uint32_t) ((uint32_t) timer_tick_get() - sub_tick));
    if (elapsed < UTR_STALL_AFTER_US)
        return;
    utr_last[port].stall_checked = 1;
    if (utr_last[port].sub_rem < UTR_STALL_MIN_REM)
        return;  // May legitimately have been deferred to the next frame
    if ((USBx_HPRT0_RAW(hhcd) & USB_OTG_HPRT_PENA) == 0)
        return;

    st = &utr_stall[utr_stall_count % UTR_STALL_KEEP];
    memset(st, 0, sizeof (*st));
    st->port       = (uint8_t) port;
    st->sub_ch     = (uint8_t) ch;
    st->sub_ep     = utr_last[port].sub_ep;
    st->sub_rem    = utr_last[port].sub_rem;
    st->sub_setup  = utr_last[port].sub_setup;
    st->start_us   = (uint16_t) ((elapsed > 0xffff) ? 0xffff : elapsed);
    st->first_nonj = 0xffff;
    st->hnptxsts   = hhcd->Instance->HNPTXSTS;
    st->hctsiz     = hc->HCTSIZ;
    st->ms         = (uint32_t) (timer_tick_to_usec(timer_tick_get()) / 1000);
    if (utr_pre_last[port].valid &&
        (utr_pre_last[port].sub_tick == sub_tick)) {
        st->have_pre = 1;
        st->pre = utr_pre_last[port];
    }

    t0 = (uint32_t) timer_tick_get();
    run_start = t0;
    for (;;) {
        uint32_t hprt  = USBx_HPRT0_RAW(hhcd);
        uint32_t hcchr = hc->HCCHAR;
        uint     ls    = (hprt >> 10) & 3;
        uint32_t since;

        now = (uint32_t) timer_tick_get();
        st->n[ls]++;
        if (ls != last) {
            if (last != 0xff) {
                uint32_t run = (uint32_t) timer_tick_to_usec(
                                   (uint32_t) (now - run_start));
                if (run > st->max_run[last])
                    st->max_run[last] = (uint16_t) ((run > 0xffff) ?
                                                    0xffff : run);
                st->changes++;
            }
            if ((ls != 1) && (st->first_nonj == 0xffff)) {
                since = (uint32_t) timer_tick_to_usec(
                            (uint32_t) (now - sub_tick));
                st->first_nonj = (uint16_t) ((since > 0xfffe) ? 0xfffe :
                                             since);
            }
            last = ls;
            run_start = now;
        }
        if ((hprt & USB_OTG_HPRT_PENA) == 0) {
            st->end = 2;
            break;
        }
        if ((hcchr & USB_OTG_HCCHAR_CHENA) == 0) {
            st->end = 1;
            break;
        }
        if (timer_tick_to_usec((uint32_t) (now - t0)) > UTR_STALL_MAX_US) {
            st->end = 3;
            break;
        }
    }
    if (last != 0xff) {
        uint32_t run = (uint32_t) timer_tick_to_usec(
                           (uint32_t) (now - run_start));
        if (run > st->max_run[last])
            st->max_run[last] = (uint16_t) ((run > 0xffff) ? 0xffff : run);
    }
    st->dur_us = (uint16_t) timer_tick_to_usec((uint32_t) (now - t0));
    utr_stall_count++;
}

static void
utr_dump_stall(void)
{
    static const char * const ends[] = {
        "?", "transfer finished", "PORT DISABLED", "timeout"
    };
    static const char * const lsn[] = { "SE0", "J", "K", "SE1" };
    uint n = (utr_stall_count < UTR_STALL_KEEP) ? utr_stall_count :
                                                  UTR_STALL_KEEP;
    uint i;
    uint k;

    if (n == 0)
        return;
    printf("Stalled low-speed transfers (%u total, last %u shown):\n",
           utr_stall_count, n);
    for (i = utr_stall_count - n; i < utr_stall_count; i++) {
        const utr_stall_t *st = &utr_stall[i % UTR_STALL_KEEP];
        printf(" #%u USB%u %lu ms: ch%u ", i + 1, st->port,
               (unsigned long) st->ms, st->sub_ch);
        utr_print_ep(st->sub_ep);
        printf("%s, %u us left at submit; sampled from +%u us for %u us "
               "-> %s\n",
               st->sub_setup ? " SETUP" : "", st->sub_rem, st->start_us,
               st->dur_us, ends[(st->end < 4) ? st->end : 0]);
        printf("     line:");
        for (k = 0; k < 4; k++) {
            printf(" %s %lu (max %u us)", lsn[k],
                   (unsigned long) st->n[k], st->max_run[k]);
        }
        printf("; %u changes", st->changes);
        if (st->first_nonj != 0xffff)
            printf("; first non-J +%u us", st->first_nonj);
        printf("\n     at start: HNPTXSTS=%08lx HCTSIZ=%08lx\n",
               (unsigned long) st->hnptxsts, (unsigned long) st->hctsiz);
        if (st->have_pre)
            utr_print_pre(&st->pre);
    }
}

/* Print a signed microsecond value */
static void
utr_print_sus(int32_t us)
{
    if (us < 0)
        printf("-%lu", (unsigned long) -us);
    else
        printf("+%lu", (unsigned long) us);
}

static void
utr_dump_dsum(void)
{
    uint n = (utr_dsum_count < UTR_DSUM_MAX) ? utr_dsum_count : UTR_DSUM_MAX;
    uint first = utr_dsum_count - n;
    uint i;

    if (n == 0)
        return;
    printf("Root port drop summary (%u total, last %u shown):\n",
           utr_dsum_count, n);
    for (i = first; i < utr_dsum_count; i++) {
        const utr_dsum_t *d = &utr_dsum[i % UTR_DSUM_MAX];
        printf(" #%u USB%u %lu ms:", i + 1, d->port, (unsigned long) d->ms);
        if (d->flags & 1) {
            uint32_t ago = (uint32_t) timer_tick_to_usec(
                               (uint32_t) (d->tick - d->sub_tick));
            printf(" last submit ch%u ", d->sub_ch);
            utr_print_ep(d->sub_ep);
            printf("%s %lu us before with %u us left in its frame "
                   "(disable at ",
                   d->sub_setup ? " SETUP" : "", (unsigned long) ago,
                   d->sub_rem);
            utr_print_sus((int32_t) ago - (int32_t) d->sub_rem);
            printf(" us vs that frame's end), pktcnt now %lu;",
                   (unsigned long) ((d->sub_hctsiz >> 19) & 0x3ff));
        }
        if (d->flags & 2) {
            printf(" hub %u port %u reset %lu ms before;",
                   d->reset_hub, d->reset_port,
                   (unsigned long) (timer_tick_to_usec(
                       (uint32_t) (d->tick - d->reset_tick)) / 1000));
        }
        if (d->flags & 4) {
            printf(" attach %lu us before;",
                   (unsigned long) timer_tick_to_usec(
                       (uint32_t) (d->tick - d->attach_tick)));
        }
        printf(" ENA ch mask %04x\n", d->ena_mask);
    }
}

static void
utr_dump_ring(uint frozen)
{
    static const char * const reasons[] = {
        "?", "root port disabled", "hub port change with device attached",
        "active device detached"
    };
    uint32_t t0;
    uint32_t seq;
    uint start;
    uint cur;

    if (utr_ring.count == 0) {
        printf("Drop ring empty\n");
        return;
    }
    start = (utr_ring.head + USB_TRACE_RING - utr_ring.count) %
            USB_TRACE_RING;
    seq   = utr_ring.seq - utr_ring.count;
    t0    = utr_ring.ent[start].tick;
    if (frozen) {
        printf("Last drop: USB%u %s, %lu ms after boot "
               "(%08lx %08lx); %lu drops since cleared\n",
               utr_ring.port,
               reasons[(utr_ring.reason < ARRAY_SIZE(reasons)) ?
                       utr_ring.reason : 0],
               (unsigned long) (timer_tick_to_usec(utr_ring.drop_tick) /
                                1000),
               (unsigned long) utr_ring.drop_a,
               (unsigned long) utr_ring.drop_b,
               (unsigned long) utr_ring.drops);
    }
    printf("Drop ring: %u events (r = us left in frame):\n", utr_ring.count);
    for (cur = 0; cur < utr_ring.count; cur++, seq++) {
        if (frozen && (seq == utr_ring.drop_seq))
            printf("  ---- drop detected here ----\n");
        utr_print_ent(&utr_ring.ent[(start + cur) % USB_TRACE_RING], t0, 1);
    }
}

/* "usb trace" command */
void
usb_trace_show(void)
{
    uint port;
    uint state;

    if (!UTR_ENABLED())
        printf("USB trace is off (debug flag USBTrace); nothing new is "
               "recorded\n");
    utr_printing = 1;
    for (port = 0; port < 2; port++) {
        printf("USB%u low-speed attempts %lu: %lu failed, %lu succeeded\n",
               port, (unsigned long) utr[port].attempts,
               (unsigned long) utr[port].fails,
               (unsigned long) utr[port].oks);
    }
    utr_dump("last failure", &utr_fail);
    utr_dump("last success", &utr_ok);
    for (port = 0; port < 2; port++)
        utr_dump_pdsnap(port);
    utr_dump_dsum();
    if (utr_pre_stats.sampled != 0) {
        printf("Low-speed transfer start sampling: %lu sampled; finished "
               "within %u us: %lu with bus activity, %lu without; still "
               "active: %lu with activity, %lu without\n",
               (unsigned long) utr_pre_stats.sampled, UTR_PRE_MAX_US,
               (unsigned long) utr_pre_stats.done_active,
               (unsigned long) utr_pre_stats.done_idle,
               (unsigned long) utr_pre_stats.open_active,
               (unsigned long) utr_pre_stats.open_idle);
        printf("Low-speed OUT starts: SETUP %lu started, %lu hung; "
               "OUT data %lu started, %lu hung\n",
               (unsigned long) utr_lsout_starts[1][0],
               (unsigned long) utr_lsout_starts[1][1],
               (unsigned long) utr_lsout_starts[0][0],
               (unsigned long) utr_lsout_starts[0][1]);
    }
    utr_dump_stall();

    state = utr_ring.state;
    if (state == UTR_RING_FROZEN) {
        utr_dump_ring(1);
        utr_ring_rearm();
        printf("Drop ring re-armed\n");
    } else if (state == UTR_RING_DRAIN) {
        printf("Drop just detected; still recording (run again)\n");
    } else {
        printf("No drop captured (%lu drops since cleared); "
               "\"usb trace ring\" shows the live ring\n",
               (unsigned long) utr_ring.drops);
    }
    utr_printing = 0;
    if (utr_skipped != 0) {
        printf("(%lu records not kept while printing)\n",
               (unsigned long) utr_skipped);
        utr_skipped = 0;
    }
}

/* "usb trace [ring|clear|sum|drops]" */
void
usb_trace_cmd(int argc, char * const *argv)
{
    if (argc < 1) {
        usb_trace_show();
        return;
    }
    if (strcmp(argv[0], "ring") == 0) {
        uint state = utr_ring.state;
        if (state == UTR_RING_ARMED) {
            utr_ring.state = UTR_RING_PAUSED;  // Not recording while printed
            utr_dump_ring(0);
            utr_ring.state = UTR_RING_ARMED;
        } else {
            utr_dump_ring(state == UTR_RING_FROZEN);
        }
    } else if (strcmp(argv[0], "clear") == 0) {
        uint32_t flags = __get_PRIMASK();
        __disable_irq();
        memset(&utr_fail, 0, sizeof (utr_fail));
        memset(&utr_ok, 0, sizeof (utr_ok));
        for (uint port = 0; port < 2; port++) {
            utr[port].attempts = 0;
            utr[port].fails = 0;
            utr[port].oks = 0;
            utr_pdsnap[port].count = 0;
            utr_pdsnap_noena[port].valid = 0;
            usbport[port].rec_episodes = 0;
            usbport[port].rec_reinits = 0;
            usbport[port].rec_ok = 0;
            usbport[port].rec_fail = 0;
        }
        utr_ring.drops = 0;
        utr_dsum_count = 0;
        utr_stall_count = 0;
        memset(&utr_pre_stats, 0, sizeof (utr_pre_stats));
        memset(utr_lsout_starts, 0, sizeof (utr_lsout_starts));
        __set_PRIMASK(flags);
        utr_ring_rearm();
        printf("USB trace cleared\n");
    } else if (strcmp(argv[0], "drops") == 0) {
        utr_dump_pdsnap(0);
        utr_dump_pdsnap(1);
        utr_dump_pdsnap_s(0, &utr_pdsnap_noena[0], " with no channel enabled");
        utr_dump_pdsnap_s(1, &utr_pdsnap_noena[1], " with no channel enabled");
        utr_dump_dsum();
        utr_dump_stall();
    } else if (strcmp(argv[0], "sum") == 0) {
        uint port;
        printf("SUM t=%lus setup=%lu/%lu out=%lu/%lu",
               (unsigned long) (timer_tick_to_usec(timer_tick_get()) /
                                1000000),
               (unsigned long) utr_lsout_starts[1][0],
               (unsigned long) utr_lsout_starts[1][1],
               (unsigned long) utr_lsout_starts[0][0],
               (unsigned long) utr_lsout_starts[0][1]);
        for (port = 0; port < 2; port++) {
            if ((utr_pdsnap[port].count == 0) &&
                (usbport[port].rec_episodes == 0))
                continue;
            printf(" USB%u drops=%lu rec=%lu reinit=%lu ok=%lu fail=%lu",
                   port, (unsigned long) utr_pdsnap[port].count,
                   (unsigned long) usbport[port].rec_episodes,
                   (unsigned long) usbport[port].rec_reinits,
                   (unsigned long) usbport[port].rec_ok,
                   (unsigned long) usbport[port].rec_fail);
        }
        printf("%s\n", UTR_ENABLED() ? "" : " (trace off)");
    } else {
        printf("usb trace [ring|clear|sum|drops]\n");
    }
}

/* Called from the HAL (stm32f2xx_hal_hcd.c) */
void usb_trace_submit(HCD_HandleTypeDef *hhcd, uint32_t chnum,
                      uint32_t direction, uint32_t ep_type, uint32_t token,
                      uint32_t length);
void usb_trace_rx(HCD_HandleTypeDef *hhcd, uint32_t chnum, uint32_t bytes);
void usb_trace_port_disabled(HCD_HandleTypeDef *hhcd, uint32_t hprt0);

void
usb_trace_submit(HCD_HandleTypeDef *hhcd, uint32_t chnum, uint32_t direction,
                 uint32_t ep_type, uint32_t token, uint32_t length)
{
    uint port = utr_port(hhcd);
    uint32_t hcchar;
    uint32_t hfnum;

    if (!UTR_ENABLED())
        return;
    hcchar = USBx_HC_RAW(hhcd, chnum)->HCCHAR;
    hfnum  = USBx_HOST_RAW(hhcd)->HFNUM;

    /* Remember the last submit of any type, for the drop summary */
    utr_last[port].sub_tick  = (uint32_t) timer_tick_get();
    utr_last[port].sub_ep    = utr_hcchar_ep(hcchar);
    utr_last[port].sub_rem   = utr_frame_rem_us(hhcd, hfnum);
    utr_last[port].sub_ch    = (uint8_t) chnum;
    utr_last[port].sub_setup = (token == 0) && (ep_type == EP_TYPE_CTRL);
    utr_last[port].have_sub  = 1;
    utr_last[port].stall_checked = 0;

    if (ep_type == EP_TYPE_INTR)
        return;  // Don't let interrupt polling flood the trace
    if ((hhcd->hc[chnum].speed == HCD_DEVICE_SPEED_LOW) &&
        utr[port].active && !utr[port].ls_seen) {
        utr[port].ls_seen = 1;
        utr[port].attempts++;
    }
    utr_add(hhcd, UTR_SUB, chnum,
            (ep_type & 3) | ((direction & 1) << 4) | ((token & 1) << 8) |
            (((hcchar >> 17) & 1) << 9) |                  // LSDEV (PRE)
            (((hcchar >> 11) & 0xf) << 12),                // EPNUM
            (length & 0xffff) | ((uint32_t) hhcd->hc[chnum].dev_addr << 16));
}

void
usb_trace_rx(HCD_HandleTypeDef *hhcd, uint32_t chnum, uint32_t bytes)
{
    utr_add(hhcd, UTR_RX, chnum, bytes, 0);
}

/* Capture the host port and channel registers into a snapshot */
static void
utr_pdsnap_fill(HCD_HandleTypeDef *hhcd, utr_pdsnap_t *s, uint32_t hprt0,
                uint32_t hfnum)
{
    USB_OTG_HostTypeDef *host = USBx_HOST_RAW(hhcd);
    uint nch = hhcd->Init.Host_channels;
    uint ch;

    s->tick     = timer_tick_get();
    s->hprt0    = hprt0;
    s->hfnum    = hfnum;
    s->hfir     = host->HFIR;
    s->hcfg     = host->HCFG;
    s->gintsts  = hhcd->Instance->GINTSTS;
    s->hnptxsts = hhcd->Instance->HNPTXSTS;
    s->hptxsts  = host->HPTXSTS;
    if (nch > UTR_SNAP_CH)
        nch = UTR_SNAP_CH;
    for (ch = 0; ch < nch; ch++) {
        s->hcchar[ch] = USBx_HC_RAW(hhcd, ch)->HCCHAR;
        s->hcint[ch]  = USBx_HC_RAW(hhcd, ch)->HCINT;
        s->hctsiz[ch] = USBx_HC_RAW(hhcd, ch)->HCTSIZ;
    }
    for (; ch < UTR_SNAP_CH; ch++) {
        s->hcchar[ch] = 0;
        s->hcint[ch]  = 0;
        s->hctsiz[ch] = 0;
    }
    s->valid = 1;
}

/*
 * Root port enable dropped (interrupt context, called from the HAL port
 * interrupt handler before anything else is done). Capture the host port
 * and channel registers. The capture which goes with the frozen drop ring
 * is kept: a new one is taken only while the ring is armed, or if none is
 * held.
 */
void
usb_trace_port_disabled(HCD_HandleTypeDef *hhcd, uint32_t hprt0)
{
    uint port = utr_port(hhcd);
    utr_pdsnap_t *s = &utr_pdsnap[port];
    USB_OTG_HostTypeDef *host = USBx_HOST_RAW(hhcd);
    uint32_t hfnum = host->HFNUM;
    uint nch = hhcd->Init.Host_channels;
    uint ch;
    uint noena = 0;

    if (!UTR_ENABLED())
        return;
    s->count++;

    /* Drop summary (not for our own re-reset after a PHY clock change) */
    if (!usbport[port].clk_resetting && (hprt0 & USB_OTG_HPRT_PCSTS)) {
        utr_dsum_t *d = &utr_dsum[utr_dsum_count % UTR_DSUM_MAX];
        uint64_t now = timer_tick_get();
        uint nchs = (nch > 16) ? 16 : nch;

        memset(d, 0, sizeof (*d));
        d->tick        = (uint32_t) now;
        d->ms          = (uint32_t) (timer_tick_to_usec(now) / 1000);
        d->port        = (uint8_t) port;
        d->sub_tick    = utr_last[port].sub_tick;
        d->sub_ep      = utr_last[port].sub_ep;
        d->sub_rem     = utr_last[port].sub_rem;
        d->sub_ch      = utr_last[port].sub_ch;
        d->sub_setup   = utr_last[port].sub_setup;
        d->reset_tick  = utr_last[port].reset_tick;
        d->reset_hub   = utr_last[port].reset_hub;
        d->reset_port  = utr_last[port].reset_port;
        d->attach_tick = utr_last[port].attach_tick;
        d->flags       = (utr_last[port].have_sub ? 1 : 0) |
                         (utr_last[port].have_reset ? 2 : 0) |
                         (utr_last[port].have_attach ? 4 : 0);
        if (utr_last[port].sub_ch < nch)
            d->sub_hctsiz = USBx_HC_RAW(hhcd, utr_last[port].sub_ch)->HCTSIZ;
        for (ch = 0; ch < nchs; ch++)
            if (USBx_HC_RAW(hhcd, ch)->HCCHAR & USB_OTG_HCCHAR_CHENA)
                d->ena_mask |= (uint16_t) BIT(ch);
        noena = (d->ena_mask == 0);
        utr_dsum_count++;
    }

    if (!s->valid || (utr_ring.state == UTR_RING_ARMED))
        utr_pdsnap_fill(hhcd, s, hprt0, hfnum);
    if (noena && !utr_pdsnap_noena[port].valid)
        utr_pdsnap_fill(hhcd, &utr_pdsnap_noena[port], hprt0, hfnum);
    utr_add(hhcd, UTR_PDIS, 0, hprt0, hfnum);
}

/* Called from the hub driver (usbh_hub.c), task context */
void usb_trace_hub_status(USBH_HandleTypeDef *hub, uint hub_port,
                          uint32_t status_change);
void usb_trace_hub_bitmap(USBH_HandleTypeDef *hub, uint32_t bitmap);
void usb_trace_hub_reset(USBH_HandleTypeDef *hub, uint hub_port);
void usb_trace_hub_attach(USBH_HandleTypeDef *hub, uint hub_port,
                          uint lowspeed);
void usb_trace_hub_detach(USBH_HandleTypeDef *hub, uint hub_port,
                          USBH_HandleTypeDef *child);

static uint32_t
utr_hub_a(USBH_HandleTypeDef *hub, uint kind, uint hub_port)
{
    return (((uint32_t) kind << 16) |
            ((uint32_t) (hub->device.address & 0x7f) << 8) |
            (hub_port & 0xff));
}

/* status_change: wPortStatus | wPortChange << 16 */
void
usb_trace_hub_status(USBH_HandleTypeDef *hub, uint hub_port,
                     uint32_t status_change)
{
    uint port = hub->id & 1;
    uint32_t a = utr_hub_a(hub, UTRH_STATUS, hub_port);
    uint32_t change = status_change >> 16;

    if (!UTR_ENABLED())
        return;
    utr_add(&_hHCD[port], UTR_HUB, 0, a, status_change);
    if (change & (UTR_C_PORT_CONNECTION | UTR_C_PORT_ENABLE |
                  UTR_C_PORT_OVER_CURR)) {
        USBH_HandleTypeDef *child = get_portdev_handle(hub, hub_port, 0);
        if ((child != NULL) && child->valid)
            utr_ring_drop(port, UTR_DROP_HUBPORT, a, status_change);
    }
}

void
usb_trace_hub_bitmap(USBH_HandleTypeDef *hub, uint32_t bitmap)
{
    utr_add(&_hHCD[hub->id & 1], UTR_HUB, 0, utr_hub_a(hub, UTRH_BITMAP, 0),
            bitmap);
}

void
usb_trace_hub_reset(USBH_HandleTypeDef *hub, uint hub_port)
{
    uint port = hub->id & 1;

    if (!UTR_ENABLED())
        return;
    utr_last[port].reset_tick = (uint32_t) timer_tick_get();
    utr_last[port].reset_hub  = (uint8_t) (hub->device.address & 0x7f);
    utr_last[port].reset_port = (uint8_t) hub_port;
    utr_last[port].have_reset = 1;
    utr_add(&_hHCD[hub->id & 1], UTR_HUB, 0,
            utr_hub_a(hub, UTRH_RESET, hub_port), 0);
}

void
usb_trace_hub_attach(USBH_HandleTypeDef *hub, uint hub_port, uint lowspeed)
{
    if (!UTR_ENABLED())
        return;
    utr_last[hub->id & 1].attach_tick = (uint32_t) timer_tick_get();
    utr_last[hub->id & 1].have_attach = 1;
    utr_add(&_hHCD[hub->id & 1], UTR_HUB, 0,
            utr_hub_a(hub, UTRH_ATTACH, hub_port), lowspeed ? 1 : 0);
}

void
usb_trace_hub_detach(USBH_HandleTypeDef *hub, uint hub_port,
                     USBH_HandleTypeDef *child)
{
    uint port = hub->id & 1;
    uint32_t a = utr_hub_a(hub, UTRH_DETACH, hub_port);
    uint32_t b = (child->device.address & 0xff) |
                 ((uint32_t) child->gState << 16);

    if (!UTR_ENABLED())
        return;
    if (child->gState == HOST_CLASS)
        b |= UTRH_DETACH_ACTIVE;
    utr_add(&_hHCD[port], UTR_HUB, 0, a, b);
    if (b & UTRH_DETACH_ACTIVE)
        utr_ring_drop(port, UTR_DROP_DETACH, a, b);
}


#endif /* USB_TRACE */

/*
 * HOST_StateTypeDef gState gstate
 *   0 IDLE                  1 DEV_WAIT_FOR_ATTACHMENT
 *   2 DEV_ATTACHED          3 DEV_DISCONNECTED
 *   4 DETECT_DEVICE_SPEED   5 ENUMERATION
 *   6 CLASS_REQUEST         7 INPUT
 *   8 SET_CONFIGURATION     9 SET_WAKEUP_FEATURE
 *  10 CHECK_CLASS          11 CLASS
 *  12 SUSPENDED            13 ABORT_STATE
 *
 * ENUM_StateTypeDef EnumState estate
 *   0 IDLE                     1 GET_FULL_DEV_DESC
 *   2 SET_ADDR                 3 GET_CFG_DESC
 *   4 GET_FULL_CFG_DESC        5 GET_MFC_STRING_DESC
 *   6 GET_PRODUCT_STRING_DESC  7 GET_SERIALNUM_STRING_DESC
 *   8 DONE
 *
 * CMD_StateTypeDef RequestState rstate
 *   0 IDLE                     1 SEND
 *   2 WAIT                     3 ERROR
 */

/*
 * Decode for HOST_USER_* state macros
 *   1 HOST_USER_SELECT_CONFIGURATION
 *   2 HOST_USER_CLASS_ACTIVE
 *   3 HOST_USER_CLASS_SELECTED
 *   4 HOST_USER_CONNECTION
 *   5 HOST_USER_DISCONNECTION
 *   6 HOST_USER_UNRECOVERED_ERROR
 */
static const char * const host_user_types[] = {
    "Unknown", "SELECT_CONF", "ACTIVE", "SELECTED",
    "CONNECT", "DISCONNECT", "ERROR"
};

/*
 * HOST_CLASS is the normal idle gstate. It runs background processing
 * for the whatever class (HID, HUB, etc) of the device.
 */
static const char * const host_gstate_types[] = {
    "HOST_IDLE", "HOST_RESET", "HOST_DEV_WAIT_FOR_ATTACHMENT",
        "HOST_DEV_ATTACHED", "HOST_DEV_DISCONNECTED",
    "HOST_DETECT_DEVICE_SPEED", "HOST_ENUMERATION",
        "HOST_CLASS_REQUEST", "HOST_INPUT",
    "HOST_SET_CONFIGURATION", "HOST_SET_WAKEUP_FEATURE",
        "HOST_CHECK_CLASS", "HOST_CLASS",
    "HOST_SUSPENDED", "HOST_ABORT_STATE"
};

/*
 * This state machine runs while gState is HOST_ENUMERATION.
 *
 * phost->gState = HOST_ENUMERATION
 * phost->EnumState
 */
static const char * const host_enumstate_types[] = {
    "ENUM_IDLE", "ENUM_GET_FULL_DEV_DESC",
        "ENUM_SET_ADDR", "ENUM_GET_CFG_DESC",
    "ENUM_GET_FULL_CFG_DESC", "ENUM_GET_MFC_STRING_DESC",
        "ENUM_GET_PRODUCT_STRING_DESC", "ENUM_GET_SERIALNUM_STRING_DESC",
        "ENUM_DONE",
};

/*
 * CMD_SEND is the actual "idle" state. It means that the state
 * machine is ready to send.
 * CMD_WAIT means that a command is in progress, and that the
 *      control state machine is active
 */
static const char * const host_requeststate_types[] = {
    "CMD_IDLE", "CMD_SEND", "CMD_WAIT", "CMD_ERROR"
};

static const char * const host_controlstate_types[] = {
    "CTRL_IDLE", "CTRL_SETUP",
        "CTRL_SETUP_WAIT", "CTRL_DATA_IN",
    "CTRL_DATA_IN_WAIT", "CTRL_DATA_OUT",
        "CTRL_DATA_OUT_WAIT", "CTRL_STATUS_IN",
    "CTRL_STATUS_IN_WAIT", "CTRL_STATUS_OUT",
        "CTRL_STATUS_OUT_WAIT", "CTRL_ERROR",
    "CTRL_ERROR_RECOVERY", "CTRL_STALLED",
        "CTRL_COMPLETE",
};

static const char * const hid_state_types[] = {
    "HID_INIT", "HID_VENDOR", "HID_GET_REPORT",
    "HID_SEND_DATA", "HID_BUSY", "HID_GET_DATA", "HID_SYNC",
    "HID_POLL", "HID_ERROR", "HID_NO_SUPPORT"
};

static const char * const hid_ctl_state_types[] = {
    "HID_REQ_INIT", "HID_REQ_IDLE",
    "HID_REQ_GET_REPORT_DESC", "HID_REQ_GET_HID_DESC",
    "HID_REQ_SET_IDLE", "HID_REQ_SET_PROTOCOL", "HID_REQ_SET_REPORT"
};

static const char * const hub_state_types[] = {
    "HUB_IDLE", "HUB_SYNC",
        "HUB_BUSY", "HUB_GET_DATA",
    "HUB_POLL", "HUB_LOOP_PORT_CHANGED",
        "HUB_LOOP_PORT_ENUM", "HUB_LOOP_PORT_WAIT",
    "HUB_PORT_CHANGED", "HUB_C_PORT_CONNECTION",
        "HUB_C_PORT_RESET", "HUB_C_PORT_ENABLE",
    "HUB_RESET_DEVICE",
    "HUB_DEV_ATTACHED", "HUB_DEV_DETACHED",
        "HUB_C_PORT_OVER_CURRENT", "HUB_C_PORT_SUSPEND",
    "HUB_PORT_POWER_OFF", "HUB_PORT_POWER_ON",
    "HUB_ERROR",
};

static const char * const hub_ctl_state_types[] = {
    "HUB_REQ_IDLE", "HUB_REQ_GET_DESCRIPTOR",
        "HUB_REQ_SET_POWER", "HUB_WAIT_PWRGOOD",
    "HUB_REQ_DONE",
};

static const char * const xusb_state_types[] = {
    "XUSB_INIT", "XUSB_FEATURE_REQUEST", "XUSB_IDLE", "XUSB_SEND_DATA",
    "XUSB_BUSY", "XUSB_GET_DATA", "XUSB_SYNC", "XUSB_POLL",
    "XUSB_ERROR" "XUSB_NO_SUPPORT",
};

uint
get_port(USBH_HandleTypeDef *phost)
{
    uint port;

    while (phost->parent != NULL)
        phost = phost->parent;

    for (port = 0; port < 2; port++)
        if (phost == usb_root[port])
            return (port);

    printf("[Can't find phost %p]", (void *) phost);
    return (0);
}

static USBH_HandleTypeDef *
alloc_port_handle(uint port)
{
    USBH_HandleTypeDef *node = USBH_malloc(sizeof (*node));
    if (node != NULL)
        memset(node, 0, sizeof (*node));
    return (node);
}

/*
 * usb_alloc_address() - allocate a USB device address which is unique on
 *                       the bus (STM32 physical port) of the given handle.
 *
 * Previously the hub port number was used as the device address, which
 * collides as soon as two hubs are present (port 4 of hub A and port 4 of
 * hub B both became address 4), and hub port 5 collided with the root
 * device's fixed address (USBH_DEVICE_ADDRESS).
 */
uint8_t
usb_alloc_address(USBH_HandleTypeDef *phost)
{
    uint port = phost->id & 1;
    uint addr;

    usb_free_address(phost);
    for (addr = 1; addr < 128; addr++) {
        if ((usb_addr_used[port][addr / 32] & BIT(addr % 32)) == 0) {
            usb_addr_used[port][addr / 32] |= BIT(addr % 32);
            phost->address = addr;
            return (addr);
        }
    }
    printf("USB%u out of device addresses\n", port);
    return (0);
}

void
usb_free_address(USBH_HandleTypeDef *phost)
{
    uint port = phost->id & 1;
    uint addr = phost->address;

    if ((addr != 0) && (addr < 128))
        usb_addr_used[port][addr / 32] &= ~BIT(addr % 32);
    phost->address = 0;
}

/*
 * usb_addr0_acquire() - try to take the bus address 0 lock for a hub.
 *
 * Returns 1 if the hub now holds (or already held) the lock, 0 if another
 * hub holds it. A lock held too long is assumed stale and taken over.
 */
/*
 * Root port enumeration retry.
 *
 * A device attached directly to an STM32 port which fails enumeration was
 * previously left in HOST_ABORT_STATE until unplugged. Devices behind a
 * hub are retried by the hub driver with a port reset; for the root port,
 * re-initialize the port (the same path as port-disable recovery), which
 * resets the bus and restarts connection detection and enumeration.
 */
#define USB_ROOT_ENUM_RETRIES     3
#define USB_ROOT_RETRY_WINDOW_MS  60000

void
usb_root_enum_failed(USBH_HandleTypeDef *phost)
{
    uint port = phost->id & 1;

    if (timer_tick_has_elapsed(usbport[port].enum_retry_window))
        usbport[port].enum_retries = 0;
    if (usbport[port].enum_retries >= USB_ROOT_ENUM_RETRIES) {
        printf("USB%u enumeration failed; giving up after %u port re-inits\n",
               port, usbport[port].enum_retries);
        return;
    }
    usbport[port].enum_retries++;
    usbport[port].enum_retry_window =
        timer_tick_plus_msec(USB_ROOT_RETRY_WINDOW_MS);
    usbport[port].reinit_timer =
        timer_tick_plus_msec(500 * usbport[port].enum_retries);
    usbport[port].reinit_pending = 1;
    printf("USB%u enumeration failed; port re-init retry %u\n",
           port, usbport[port].enum_retries);
}

void
usb_root_enum_ok(USBH_HandleTypeDef *phost)
{
    usbport[phost->id & 1].enum_retries = 0;
}

uint
usb_addr0_acquire(USBH_HandleTypeDef *hub)
{
    uint port = hub->id & 1;

    if ((addr0_owner[port] != NULL) && (addr0_owner[port] != hub)) {
        if (!timer_tick_has_elapsed(addr0_deadline[port]))
            return (0);
        printf("USB%s address 0 lock timeout; taking it from USB%s\n",
               usb_get_path(hub, NULL, -1),
               usb_get_path(addr0_owner[port], NULL, -1));
    }
    if (addr0_owner[port] != hub) {
        addr0_owner[port] = hub;
        addr0_deadline[port] = timer_tick_plus_msec(USB_ADDR0_HOLD_MAX_MS);
    }
    return (1);
}

void
usb_addr0_release(USBH_HandleTypeDef *hub)
{
    uint port = hub->id & 1;

    if (addr0_owner[port] == hub)
        addr0_owner[port] = NULL;
}

USBH_HandleTypeDef *
get_portdev_handle(USBH_HandleTypeDef *phost, uint dev, uint alloc)
{
    USBH_HandleTypeDef *cur;

    if (phost == NULL)
        return (NULL);

    /*
     * Search for device on the parent hub. The devices attached to a hub
     * are the hub's child list. This previously started the search at
     * phost itself (and walked phost's siblings), so a hub attached to
     * port N of another hub would "find" itself when a device appeared
     * on its own port N, leading to the hub tearing itself down and a
     * double free in detach().
     */
    for (cur = phost->child; cur != NULL; cur = cur->next) {
        if (cur->hubdev == dev)
            return (cur);
    }
    if (alloc) {
        uint port = phost->id;
        cur = alloc_port_handle(port);
        if (cur == NULL)
            return (NULL);
#if 0
        printf("alloc handle %x for %s\n", (uintptr_t) cur,
               usb_get_path(phost, NULL, -1));
#endif
        cur->id = phost->id;
        cur->hubdev = dev;
        cur->parent = phost;
        if (phost->child == NULL) {
            phost->child = cur;
        } else {
            USBH_HandleTypeDef *tcur = phost->child;
            while (tcur->next != NULL)
                tcur = tcur->next;
            tcur->next = cur;
        }
    }
    return (cur);
}

static void
release_usb_handle(USBH_HandleTypeDef *handle)
{
    USBH_HandleTypeDef *cur;
    if (handle == NULL)
        return;

    if ((handle->parent != NULL) && handle->valid && (handle->pUser != NULL))
        handle->pUser(handle, HOST_USER_DISCONNECTION);

    if (handle->pActiveClass != NULL) {
        handle->pActiveClass->DeInit(handle);
        handle->pActiveClass = NULL;
    }

    /* Don't leave the poll loop pointing at a freed handle */
    for (uint p = 0; p < 2; p++) {
        if (addr0_owner[p] == handle)
            addr0_owner[p] = NULL;
        if (phost_cur[p] == handle)
            phost_cur[p] = NULL;
        if (ctl_stay_handle[p] == handle)
            ctl_stay_handle[p] = NULL;
    }

    if (handle->parent != NULL) {
        /* Hub-attached device: free its address and class copies */
        usb_free_address(handle);
        while (handle->ClassNumber > 0) {
            handle->ClassNumber--;
            USBH_free(handle->pClass[handle->ClassNumber]);
        }
        for (uint i = 0; i < USBH_MAX_NUM_INTERFACES; i++) {
            USBH_free(handle->USBH_ClassTypeDef_pData[i]);
            handle->USBH_ClassTypeDef_pData[i] = NULL;
        }
    }

    if (handle->parent == NULL) {
        /* Root port */
        uint curport;
        for (curport = 0; curport < 2; curport++) {
            if (usb_root[curport] == handle) {
                usb_root[curport] = NULL;
                break;
            }
        }
        if (curport == 2) {
            printf("Handle %x has no parent, but is not root port\n",
                   (uintptr_t) handle);
        }
    } else {
        cur = handle->parent->child;
        if (cur == handle) {
            /* Disconnect first port */
            handle->parent->child = handle->next;
        } else {
            /* Find port to disconnect */
            for (; cur->next != NULL; cur = cur->next)
                if (cur->next == handle) {
                    cur->next = handle->next;
                    break;
                }
            if (cur->next == NULL) {
                printf("Didn't find handle %x!\n", (uintptr_t) handle);
            }
        }
    }
    USBH_free(handle);
}

static void
release_usb_handles(USBH_HandleTypeDef *handle)
{
    if (handle == NULL)
        return;

    USBH_HandleTypeDef *child = handle->child;
    while (child != NULL) {
        USBH_HandleTypeDef *next = child->next;
        release_usb_handles(child);
        handle->child = child = next;
    }
    release_usb_handle(handle);
}

static void
USBH_UserProcess(USBH_HandleTypeDef *phost, uint8_t id)
{
    dprintf(DF_USB_CONN, "USB%s %s\n", usb_get_path(phost, NULL, id),
            (id < ARRAY_SIZE(host_user_types)) ?
            host_user_types[id] : "Unknown");
    switch (id) {
        case HOST_USER_SELECT_CONFIGURATION:
            /*
             * Can be used to select one of multiple device configurations,
             * if the device implements multiple. An example is a composite
             * device supporting either HID or CDC. The host can pick one
             * of the two by setting here:
             *     phost->device.CfgDesc.bConfigurationValue
             */
            break;
        case HOST_USER_CLASS_ACTIVE:
#ifdef USB_TRACE
            utr_class_active(phost);
#endif
            phost->appstate = APPLICATION_READY;
            phost->hid_ready_timer = timer_tick_plus_msec(500);
            break;
        case HOST_USER_CLASS_SELECTED:
            break;
        case HOST_USER_CONNECTION:
            phost->appstate = APPLICATION_START;
            break;
        case HOST_USER_DISCONNECTION:
            usb_keyboard_count -= phost->keyboard_count;
            usb_mouse_count    -= phost->mouse_count;
            usb_joystick_count -= phost->joystick_count;

            phost->keyboard_count = 0;
            phost->mouse_count = 0;
            phost->joystick_count = 0;
            phost->hid_ready_timer = 0;
            phost->appstate = APPLICATION_DISCONNECT;
            break;
        case HOST_USER_UNRECOVERED_ERROR:
            break;
        default:
            break;
    }
}

void
otg_fs_isr(void)
{
//  printf("Ifs");
    usb_stats.otg_fs_ints++;
    HAL_HCD_IRQHandler(&_hHCD[0]);
}

void
otg_hs_ep1_in_isr(void)
{
    usb_stats.otg_hs_ep1_in_ints++;
    printf("[Ihsepi]");
    HAL_HCD_IRQHandler(&_hHCD[1]);
}

void
otg_hs_ep1_out_isr(void)
{
    usb_stats.otg_hs_ep1_out_ints++;
    printf("[Ihsepo]");
    HAL_HCD_IRQHandler(&_hHCD[1]);
}

void
otg_hs_wkup_isr(void)
{
    usb_stats.otg_hs_wkup_ints++;
    printf("[Ihsw]");
    HAL_HCD_IRQHandler(&_hHCD[1]);
}

void
otg_hs_isr(void)
{
//  printf("Ihs");
    usb_stats.otg_hs_ints++;
    HAL_HCD_IRQHandler(&_hHCD[1]);
}

void
usb_show_stats(void)
{
    printf("FS ints         %u\n"
           "HS ints         %u\n"
           "HS EP1 IN ints  %u\n"
           "HS EP1 OUT ints %u\n"
           "HS WKUP ints    %u\n",
           usb_stats.otg_fs_ints, usb_stats.otg_hs_ints,
           usb_stats.otg_hs_ep1_in_ints, usb_stats.otg_hs_ep1_out_ints,
           usb_stats.otg_hs_wkup_ints);
}

/*
 * USB Class Codes
 *
 * These codes define both the bDeviceClass field in the Device Descriptor
 * and the bInterfaceClass field in the Interface Descriptor.
 */
static const char *const usb_class_codes[] =
{
    "IF",       // 0x00
    "Audio",    // 0x01 AC_CLASS
    "CDC",      // 0x02 USB_CDC_CLASS
    "HID",      // 0x03 USB_HID_CLASS
    "Unknown",  // 0x04
    "PID",      // 0x05
    "IMG",      // 0x06 USB_MTP_CLASS
    "Printer",  // 0x07
    "Storage",  // 0x08 USB_MSC_CLASS
    "HUB",      // 0x09
    "CDC-Data", // 0x0a DATA_INTERFACE_CLASS_CODE
    "SmartCrd", // 0x0b
    "Eth",      // 0x0c
    "Security", // 0x0d
    "Video",    // 0x0e
    "PHealth",  // 0x0f
    "AV",       // 0x10
    "Bboard",   // 0x11
    "C-Bridge", // 0x12
    "USB-Bulk", // 0x13
    "MCTP",     // 0x14
                // 0xdc Diagnostic
                // 0xfe Application-specific
                // 0xff Vendor-specific
};

static const char *
get_devclass_str(uint8_t devclass)
{
    if (devclass < ARRAY_SIZE(usb_class_codes))
        return (usb_class_codes[devclass]);
    else if (devclass == 0xfe)
        return ("App");
    else if (devclass == 0xff)
        return ("Vendor");
    else
        return ("Unknown");
}

static const char *const usb_hid_usage_codes[] =
{
    "Undefined",      // 0x00
    "Pointer",        // 0x01
    "Mouse",          // 0x02
    "Reserved",       // 0x03
    "Joystick",       // 0x04
    "Gamepad",        // 0x05
    "Keyboard",       // 0x06
    "Keypad",         // 0x07
    "M-X Controller", // 0x08
};

static const char *
get_hid_usage_str(uint8_t usage)
{
    if (usage < ARRAY_SIZE(usb_hid_usage_codes))
        return (usb_hid_usage_codes[usage]);
    switch (usage) {
        case HID_USAGE_SYSCTL:
            return ("Sysctl");
        default:
            return ("Unknown");
    }
}

static void
usb_ls_classes(USBH_HandleTypeDef *phost, uint verbose)
{
    uint ifnum;
    uint numif = phost->device.CfgDesc.bNumInterfaces;
    uint ifclass;
    uint ifsclass;
    uint ifproto;
    uint i;
    char *protostr;
    char *ifsclass_str;
    uint show_config = verbose & 0x100;

    if (numif > USBH_MAX_NUM_INTERFACES)
        numif = USBH_MAX_NUM_INTERFACES;

    for (ifnum = 0; ifnum < numif; ifnum++) {
        void *data = NULL;
        uint classnum;

        ifclass = phost->device.CfgDesc.Itf_Desc[ifnum].bInterfaceClass;
        ifsclass = phost->device.CfgDesc.Itf_Desc[ifnum].bInterfaceSubClass;
        ifproto  = phost->device.CfgDesc.Itf_Desc[ifnum].bInterfaceProtocol;
        protostr = "";
        ifsclass_str = "";

        for (classnum = 0; classnum < phost->ClassNumber; classnum++) {
            if (ifclass == phost->pClass[classnum]->ClassCode) {
                data = phost->pClass[classnum]->pData;
                break;
            }
        }
        switch (ifclass) {
            case USB_HID_CLASS:
                switch (ifproto) {
                    case HID_MOUSE_BOOT_CODE:
                        protostr = "Mouse";
                        break;
                    case HID_KEYBRD_BOOT_CODE:
                        protostr = "Keyboard";
                        break;
                    default:
                        protostr = "Unknown";
                        break;
                }
                switch (ifsclass) {
                    case 0:
                        ifsclass_str = "None";
                        break;
                    case 1:
                        ifsclass_str = "Boot Interface";
                        break;
                }
                break;
        }
        printf(" %2s IF %u: class=%x %s subclass=%02x %s protocol=%02x %s\n",
               (phost->iface_waiting & BIT(ifnum)) ? "W>" : "",
               ifnum, ifclass, get_devclass_str(ifclass),
               ifsclass, ifsclass_str,
               ifproto, protostr);
        if (ifclass == 0)
            continue;
        if (verbose & 0xff) {
            switch (ifclass) {
                case USB_HID_CLASS: {
                    HID_HandleTypeDef *HID_Handle = data;
                    for (; HID_Handle != NULL; HID_Handle = HID_Handle->next) {
                        if (HID_Handle->interface != ifnum)
                            continue;

                        if (HID_Handle == NULL) {
                            printf("          NULL HID Handle\n");
                            continue;
                        }

                        printf("          OutEp=%x InEp=%x state=%s "
                               "ctl_state=%s\n",
                               HID_Handle->OutEp, HID_Handle->InEp,
                               (HID_Handle->state <
                                ARRAY_SIZE(hid_state_types)) ?
                               hid_state_types[HID_Handle->state] : "Unknown",
                               (HID_Handle->ctl_state <
                                ARRAY_SIZE(hid_ctl_state_types)) ?
                               hid_ctl_state_types[HID_Handle->ctl_state] :
                               "Unknown");
                        printf("          timer=%lx dataready=%x idmouse=%x "
                               "idconsumer=%x idsysctl=%x\n",
                               HID_Handle->timer, HID_Handle->DataReady,
                               HID_Handle->HID_RDesc.id_mouse,
                               HID_Handle->HID_RDesc.id_consumer,
                               HID_Handle->HID_RDesc.id_sysctl);
                        printf("          usage=%s",
                               get_hid_usage_str(HID_Handle->HID_RDesc.usage));
                        if (HID_Handle->HID_RDesc.pos_keymod ||
                            HID_Handle->HID_RDesc.pos_keynkro) {
                            uint which = HID_Handle->HID_RDesc.pos_keynkro &
                                         BIT(15);
                            printf("  keymod=%u %ckro=%u",
                                   HID_Handle->HID_RDesc.pos_keymod,
                                   which ? 'n' : '6',
                                   HID_Handle->HID_RDesc.pos_keynkro & 0x7fff);
                        }
                        printf("\n");
                        if (HID_Handle->HID_RDesc.bits_x ||
                            HID_Handle->HID_RDesc.bits_y ||
                            HID_Handle->HID_RDesc.bits_wheel ||
                            HID_Handle->HID_RDesc.bits_ac_pan) {
                            printf("          x=%u [%u]  y=%u [%u]  "
                                   "wheel=%u [%u]  pan=%u [%u]  offset=%d\n",
                                   HID_Handle->HID_RDesc.pos_x,
                                   HID_Handle->HID_RDesc.bits_x,
                                   HID_Handle->HID_RDesc.pos_y,
                                   HID_Handle->HID_RDesc.bits_y,
                                   HID_Handle->HID_RDesc.pos_wheel,
                                   HID_Handle->HID_RDesc.bits_wheel,
                                   HID_Handle->HID_RDesc.pos_ac_pan,
                                   HID_Handle->HID_RDesc.bits_ac_pan,
                                   HID_Handle->HID_RDesc.offset_xy);
                        }
                        if (HID_Handle->HID_RDesc.num_buttons > 0) {
                            printf("          buttons=");
                            for (i = 0; i < HID_Handle->HID_RDesc.num_buttons; i++)
                                printf("%u ", HID_Handle->HID_RDesc.pos_button[i]);
                            printf("\n");
                        }
                        if ((HID_Handle->HID_RDesc.num_keys > 0) ||
                            (HID_Handle->HID_RDesc.bits_sysctl > 0)) {
                            printf("          mmkeys=");
                            for (i = 0; i < HID_Handle->HID_RDesc.num_keys; i++)
                                printf("%u [%u]  ", HID_Handle->HID_RDesc.pos_key[i], HID_Handle->HID_RDesc.bits_key);
                            printf(" sysctl=%u [%u]\n", HID_Handle->HID_RDesc.pos_sysctl, HID_Handle->HID_RDesc.bits_sysctl);
                        }
                        for (i = 0; i < 4; i++) {
                            if (HID_Handle->HID_RDesc.pos_jpad[i]) {
                                printf("          jpad=");
                                for (i = 0; i < 4; i++)
                                    printf("%u ", HID_Handle->HID_RDesc.pos_jpad[i]);
                                printf("\n");
                                break;
                            }
                        }
                        if (HID_Handle->HID_RDesc.num_mmbuttons > 0) {
                            printf("          mmbutton=");
                            for (i = 0; i < HID_Handle->HID_RDesc.num_mmbuttons; i++)
                                printf("%u[%u]=%x ", HID_Handle->HID_RDesc.pos_mmbutton[i], HID_Handle->HID_RDesc.id_mmbutton[i], HID_Handle->HID_RDesc.val_mmbutton[i]);
                            printf("\n");
                        }
                    }
                    break;
                }
                case USB_XUSB_CLASS: {
                    XUSB_Handle_t *XUSB_Handle = data;
                    for (; XUSB_Handle != NULL; XUSB_Handle = XUSB_Handle->next) {
                        if ((XUSB_Handle->interface != ifnum) ||
                            (XUSB_Handle->interface != ifnum))
                            continue;

                        if (XUSB_Handle == NULL) {
                            printf("          NULL XUSB Handle\n");
                            continue;
                        }
                        printf("          OutEp=%x InEp=%x state=%s \n",
                               XUSB_Handle->OutEp, XUSB_Handle->InEp,
                               (XUSB_Handle->state <
                                ARRAY_SIZE(xusb_state_types)) ?
                               xusb_state_types[XUSB_Handle->state] :
                               "Unknown");
                        printf("          timer=%lx dataready=%x\n",
                               XUSB_Handle->timer, XUSB_Handle->DataReady);
                    }
                    break;
                }
                case USB_HUB_CLASS: {
                    HUB_HandleTypeDef *HUB_Handle;
                    HUB_Handle = phost->USBH_ClassTypeDef_pData[0];

                    if (HUB_Handle == NULL) {
                        printf("          NULL HUB Handle\n");
                        break;
                    }
                    printf("          Change=%02x CurPort=%x NumPort=%x "
                           "PwrGood=%04x\n",
                           HUB_Handle->change.val, HUB_Handle->cur_port,
                           HUB_Handle->num_ports, HUB_Handle->pwr_good);
                    printf("          InEp=%x state=%s "
                           "ctl_state=%s\n",
                           HUB_Handle->InEp,
                           (HUB_Handle->state <
                            ARRAY_SIZE(hub_state_types)) ?
                           hub_state_types[HUB_Handle->state] : "Unknown",
                           (HUB_Handle->ctl_state <
                            ARRAY_SIZE(hub_ctl_state_types)) ?
                           hub_ctl_state_types[HUB_Handle->ctl_state] :
                           "Unknown");
                    printf("          timer=%lx DataReady=%x poll=%x "
                           "length=%x\n",
                           HUB_Handle->timer, HUB_Handle->DataReady,
                           HUB_Handle->poll, HUB_Handle->length);
                    break;
                }
            }
        }
        if (show_config) {
            USBH_DeviceTypeDef *dev = &phost->device;
            USBH_CfgDescTypeDef *cd = &dev->CfgDesc;
            USBH_InterfaceDescTypeDef *id = &cd->Itf_Desc[ifnum];
            uint ep;
            printf("          IDesc bLength=%x bDescriptorType=%x "
                   "bInterfaceNumber=%x\n",
                   id->bLength, id->bDescriptorType, id->bInterfaceNumber);
            printf("                bAlternateSetting=%x bNumEndpoints=%x "
                   "bInterfaceClass=%x\n",
                   id->bAlternateSetting, id->bNumEndpoints,
                   id->bInterfaceClass);
            printf("                bInterfaceSubClass=%x "
                   "bInterfaceProtocol=%x iInterface=%x\n",
                   id->bInterfaceSubClass, id->bInterfaceProtocol,
                   id->iInterface);
            for (ep = 0; ep < id->bNumEndpoints; ep++) {
                USBH_EpDescTypeDef *ed = &id->Ep_Desc[ep];
                printf("          EDesc%u bLength=%x bDescriptorType=%x "
                       "bEndpointAddress=%x\n",
                       ep, ed->bLength, ed->bDescriptorType,
                       ed->bEndpointAddress);
                printf("                 bmAttributes=%x wMaxPacketSize=%x "
                       "bInterval=%x\n",
                       ed->bmAttributes, ed->wMaxPacketSize,
                       ed->bInterval);
            }
        }
    }
}

char *
usb_get_path(USBH_HandleTypeDef *phost, char *buf, int interface)
{
    char   *bufcur;
    uint8_t hub_stack[16];
    uint8_t depth = 0;
    static  char localbuf[32];

    if (buf == NULL)
        buf = localbuf;
    bufcur = buf;
    while (phost != NULL) {
        hub_stack[depth++] = phost->hubdev;
        phost = phost->parent;
    }
    while (depth > 0) {
        depth--;
        bufcur += sprintf(bufcur, "%x", hub_stack[depth]);
        if (depth != 0)
            *(bufcur++) = '.';
    }
    if (interface >= 0) {
        bufcur += sprintf(bufcur, "[%x]", interface);
    }
    *bufcur = '\0';
    return (buf);
}

void
usb_ls(uint verbose)
{
    uint    cur;
    uint    port;
    uint    show_config = verbose & 0x100;
    uint    hub_depth = 0;
    uint    classnum;
    uint8_t devclass;
    uint8_t hub_stack[16];

    for (port = 0; port < 2; port++) {
        USBH_HandleTypeDef *phost = usb_root[port];

        while (phost != NULL) {
            hub_stack[hub_depth] = phost->hubdev;
            devclass = phost->device.DevDesc.bDeviceClass;
            for (cur = 0; cur <= hub_depth; cur++)
                printf("%s%u", (cur == 0) ? "" : ".", hub_stack[cur]);
            printf(" %04x:%04x ",
                   phost->device.DevDesc.idVendor,
                   phost->device.DevDesc.idProduct);

            if (phost->valid == 0) {
                printf("NOT VALID\n");
                goto dev_next;
            }
            if (phost->device.is_connected == 0) {
                printf("No device\n");
                goto dev_next;
            }

            printf("addr=%x %s: %s",
                   phost->device.address,
                   phost->device.manufacturer_string,
                   phost->device.product_string);
            if (phost_cur[port] == phost)
                printf(" <<");

            for (classnum = 0; classnum < phost->ClassNumber; classnum++) {
                if (devclass == phost->pClass[classnum]->ClassCode)
                    break;
            }
            if (show_config) {
                USBH_DeviceTypeDef *dev = &phost->device;
                USBH_DevDescTypeDef *dd = &dev->DevDesc;
                USBH_CfgDescTypeDef *cd = &dev->CfgDesc;
                printf("\n          enabled=%x connected=%x disconnected=%x "
                       "re-enumerated=%x",
                       dev->PortEnabled, dev->is_connected,
                       dev->is_disconnected, dev->is_ReEnumerated);
                printf("\n          Dev bLength=%x bDescriptorType=%x "
                       "bDeviceClass=%x bDeviceSubClass=%x",
                       dd->bLength, dd->bDescriptorType,
                       dd->bDeviceClass, dd->bDeviceSubClass);
                printf("\n              bDeviceProtocol=%x bcdUSB=%x "
                       "bcdDevice=%x bNumConfigurations=%x",
                       dd->bcdUSB, dd->bDeviceProtocol, dd->bcdDevice,
                       dd->bNumConfigurations);
                printf("\n          Cfg bLength=%x bDescriptorType=%x "
                       "wTotalLength=%x bNumInterfaces=%x ",
                       cd->bLength, cd->bDescriptorType, cd->wTotalLength,
                       cd->bNumInterfaces);
                printf("\n              bConfigurationValue=%x02 "
                       "bmAttributes=%02x bMaxPower=%x",
                       cd->bConfigurationValue, cd->bmAttributes,
                       cd->bMaxPower);
            }
            if (verbose & 0xff) {
                printf("\n          class=%x %s ",
                       devclass, get_devclass_str(devclass));
                printf(" gstate=%s ",
                       (phost->gState < ARRAY_SIZE(host_gstate_types)) ?
                       host_gstate_types[phost->gState] : "Unknown");
                printf(" rstate=%s",
                       (phost->RequestState <
                        ARRAY_SIZE(host_requeststate_types)) ?
                       host_requeststate_types[phost->RequestState] :
                       "Unknown");
                printf("\n          estate=%s ",
                       (phost->EnumState < ARRAY_SIZE(host_enumstate_types)) ?
                       host_enumstate_types[phost->EnumState] : "Unknown");
                printf(" ctlstate=%s ",
                       (phost->Control.state <
                        ARRAY_SIZE(host_controlstate_types)) ?
                       host_controlstate_types[phost->Control.state] :
                       "Unknown");
                printf(" app=%s",
                       host_appstate[phost->appstate]);
                printf("\n          polls=%lu  usec %llu max %llu",
                       phost->poll_count, timer_tick_to_usec(phost->tick_total),
                       timer_tick_to_usec(phost->tick_max));
                printf(" ifs=%u hub=%x hubifs=%x",
                       phost->device.CfgDesc.bNumInterfaces,
                       phost->hub, phost->interfaces);
                printf("\n          lastreq=%02x val=%02x ind=%02x len=%02x",
                       phost->Control.setup.b.bRequest,
                       phost->Control.setup.b.wValue.w,
                       phost->Control.setup.b.wIndex.w,
                       phost->Control.setup.b.wLength.w);
                printf(" out=%u in=%u",
                       phost->Control.pipe_out, phost->Control.pipe_in);
                if (phost->busy) {
                    printf("  BUSY");
                    if (phost->busy > 1)
                        printf(" %x", phost->busy);
                }
            }
            printf("\n");
            usb_ls_classes(phost, verbose);
dev_next:
            if (phost->child != NULL) {
                phost = phost->child;
                hub_depth++;
            } else if (phost->next != NULL) {
                phost = phost->next;
            } else while ((phost = phost->parent) != NULL) {
                if (hub_depth > 0)
                    hub_depth--;
                if (phost->next != NULL) {
                    phost = phost->next;
                    break;
                }
            }
        }
    }
}

__attribute__((format(__printf__, 1, 2)))
void
usb_usrlog(const char *fmt, ...)
{
    va_list args;

    if (config.debug_flag & DF_USB_USRLOG) {
        va_start(args, fmt);
        (void) vprintf(fmt, args);
        va_end(args);
        printf("\n");
    }
}

/*
 * phost_which() determines whether the specified device is device 0 or 1,
 *               which in the context of mouse or joystick means that it's
 *               targeting Amiga port 0 (typically mouse) or port 1
 *               (typically joystick).
 */
static uint
phost_which(USBH_HandleTypeDef *phost)
{
    /*
     * XXX: This code needs to determine which device this is, based on
     *      an association selection that the user saved.
     *
     *      My thinking right now is that there will be an array of up
     *      to 16 runtime phost values that this code will walk first.
     *      If a match is found, the selected "which" value is returned.
     *      If nothing is found, then this code will then compute a
     *      CRC based on the config data. The CRC will be used to look
     *      up the desired which value from saved config.
     *      Will need a way to present CRC values to Becky so they
     *      can be associated.
     */
    return (0);
}

void
USBH_XUSB_EventCallback(USBH_HandleTypeDef *phost, XUSB_Handle_t *XUSB_Handle)
{
    static uint32_t last[4];
    static uint8_t last_was_joypad;
    static uint32_t buttons;
    static uint8_t joypad;
    static int16_t mouse_x;
    static int16_t mouse_y;
    static int16_t wheel_x;
    static int16_t wheel_y;
    static uint64_t throttle_timer;
    XUSB_MISC_Info_t info;
    uint32_t *data = (uint32_t *)XUSB_Handle->pData;
    uint      which = phost_which(phost);
    uint pos;

    if (memcmp(last, data, sizeof (last)) == 0) {
        if (last_was_joypad) {
            if (joypad | buttons) {
                joystick_action(which, joypad & BIT(0), joypad & BIT(1),
                                joypad & BIT(2), joypad & BIT(3), buttons);
            }
        } else {
            /* Replay mouse */
            int16_t twheel_x;
            int16_t twheel_y;
            if (timer_tick_has_elapsed(throttle_timer)) {
                if ((wheel_x > 1) || (wheel_x < -1) ||
                    (wheel_y > 1) || (wheel_y < -1)) {
                    throttle_timer = timer_tick_plus_msec(50);
                } else {
                    throttle_timer = timer_tick_plus_msec(200);
                }
                twheel_x = wheel_x;
                twheel_y = wheel_y;
            } else {
                twheel_x = 0;
                twheel_y = 0;
            }
            if (mouse_x | mouse_y | wheel_x | wheel_y | buttons) {
                mouse_action(which, mouse_x, mouse_y, twheel_y, twheel_x);
                mouse_action_button(which, buttons);
            }
        }
        return;
    }

    if (config.debug_flag & DF_USB_DECODE_MISC) {
        for (pos = 0; pos < ARRAY_SIZE(last); pos++) {
            last[pos] = data[pos];
            printf(" %08lx", data[pos]);
        }
        printf(" ");
    }

    USBH_XUSB_DecodeReport(phost, XUSB_Handle, &info);
    if (info.joypad ||
        (last_was_joypad &&
         (info.mouse_x == 0) && (info.mouse_y == 0) &&
         (info.wheel_x == 0) && (info.wheel_y == 0))) {
        joypad = info.joypad;
        buttons = info.buttons;
        joystick_action(which, joypad & BIT(0), joypad & BIT(1),
                        joypad & BIT(2), joypad & BIT(3), buttons);
        last_was_joypad = 1;
    } else {
        int16_t twheel_x;
        int16_t twheel_y;
        mouse_x = info.mouse_x;
        mouse_y = info.mouse_y;
        wheel_x = info.wheel_x;
        wheel_y = info.wheel_y;
        buttons = info.buttons;
        if (timer_tick_has_elapsed(throttle_timer)) {
            if ((wheel_x > 1) || (wheel_x < -1) ||
                (wheel_y > 1) || (wheel_y < -1)) {
                throttle_timer = timer_tick_plus_msec(50);
            } else {
                throttle_timer = timer_tick_plus_msec(200);
            }
            twheel_x = wheel_x;
            twheel_y = wheel_y;
        } else {
            twheel_x = 0;
            twheel_y = 0;
        }
        dprintf(DF_USB_DECODE_MISC, "%d %d %d %d ",
                mouse_x, mouse_y, twheel_x, twheel_y);
        mouse_action(which, mouse_x, mouse_y, twheel_y, twheel_x);
        mouse_action_button(which, buttons);
        last_was_joypad = 0;
    }
    dprintf(DF_USB_DECODE_MISC, "\n");
}

void
USBH_HID_EventCallback(USBH_HandleTypeDef *phost, HID_HandleTypeDef *HID_Handle)
{
    uint    which = phost_which(phost);
    uint8_t usage = HID_Handle->HID_RDesc.usage;
    HID_TypeTypeDef devtype;

    devtype = USBH_HID_GetDeviceType(phost, HID_Handle->interface);
    if ((usage == HID_USAGE_KBD) || (usage == HID_USAGE_SYSCTL))
        devtype = HID_KEYBOARD;
    else if ((usage == HID_USAGE_MOUSE) || (usage == HID_USAGE_POINTER))
        devtype = HID_MOUSE;
    else if ((usage == HID_USAGE_JOYSTICK) || (usage == HID_USAGE_GAMEPAD))
        devtype = HID_UNKNOWN;

    if ((devtype == HID_MOUSE) || (devtype == HID_UNKNOWN)) {
        /* Mouse or Generic HID (refer to the HID report descriptor) */
        HID_MISC_Info_TypeDef info;
        if (USBH_HID_DecodeReport(phost, HID_Handle, devtype, &info) != USBH_OK)
            return;
        if ((info.usage == HID_USAGE_GAMEPAD) ||
            (info.usage == HID_USAGE_JOYSTICK)) {
            uint8_t up    = info.jpad & BIT(0);
            uint8_t down  = info.jpad & BIT(1);
            uint8_t left  = info.jpad & BIT(2);
            uint8_t right = info.jpad & BIT(3);
            if (config.flags & CF_GAMEPAD_MOUSE) {
                mouse_action(which, info.x, info.y, -info.wheel, -info.ac_pan);
                mouse_action_button(which, info.buttons);
            } else {
                up    |= (info.y < 0);
                down  |= (info.y > 0);
                left  |= (info.x < 0);
                right |= (info.x > 0);
            }
            joystick_action(which, up, down, left, right, info.buttons);
        } else {
            mouse_action(which, info.x, info.y, -info.wheel, -info.ac_pan);
            mouse_action_button(which, info.buttons);
            keyboard_usb_input_mm(info.mm_key, ARRAY_SIZE(info.mm_key));
            keyboard_usb_input_sysctl(info.sysctl);
        }
    } else if (devtype == HID_KEYBOARD) {  // Keyboard
        HID_Keyboard_Info_TypeDef info;

        if (USBH_HID_DecodeKeyboard(phost, HID_Handle, &info) != USBH_OK)
            return;
        keyboard_usb_input((usb_keyboard_report_t *) &info);
        keyboard_usb_input_mm(info.mm_key, ARRAY_SIZE(info.mm_key));
    } else {
        printf("USB%s Event\n", usb_get_path(phost, NULL, -1));
    }
}

static int
is_usb_joystick(USBH_HandleTypeDef *phost, uint ifnum)
{
    uint8_t ifclass = phost->device.CfgDesc.Itf_Desc[ifnum].bInterfaceClass;
    uint classnum;
    void *data = NULL;
    for (classnum = 0; classnum < phost->ClassNumber; classnum++) {
        if (ifclass == phost->pClass[classnum]->ClassCode) {
            data = phost->pClass[classnum]->pData;
            break;
        }
    }

    if (ifclass == USB_HID_CLASS) {
        if (data != NULL) {
            HID_HandleTypeDef *HID_Handle = data;
            switch (HID_Handle->HID_RDesc.usage) {
                case HID_USAGE_GAMEPAD:
                case HID_USAGE_JOYSTICK:
                    return (1);
            }
        }
    } else if (ifclass == USB_XUSB_CLASS) {
        /* Check for XBox-360 controller */
        if ((USBH_XUSB_CLASS)->Probe(phost) == USBH_OK)
            return (1);
    }
    return (0);
}

static void
handle_discovery(USBH_HandleTypeDef *phost, uint port, uint devnum)
{
    uint iface;
    uint numif = phost->device.CfgDesc.bNumInterfaces;
    uint devtype;
    uint8_t ifclass;

    if (phost->appstate == APPLICATION_READY) {
        if (!timer_tick_has_elapsed(phost->hid_ready_timer))
            return;

        for (iface = 0; iface < numif; iface++) {
            ifclass = phost->device.CfgDesc.Itf_Desc[iface].bInterfaceClass;
            if (ifclass == USB_HID_CLASS)
                devtype = USBH_HID_GetDeviceType(phost, iface);
            else
                devtype = 0;
            dprintf(DF_USB_CONN, "USB%s type %x %s\n",
                    usb_get_path(phost, NULL, iface), devtype,
                    (devtype == HID_KEYBOARD) ? "Keyboard" :
                    (devtype == HID_MOUSE) ? "Mouse" : "Unknown");
            if (devtype == HID_KEYBOARD) {
                phost->keyboard_count++;
                usb_keyboard_count++;
            } else if (devtype == HID_MOUSE) {
                phost->mouse_count++;
                usb_mouse_count++;
            } else if (is_usb_joystick(phost, iface)) {
                phost->joystick_count++;
                usb_joystick_count++;
                break;
            }
        }
        if ((usb_mouse_count > 0) || (usb_joystick_count > 0))
            hiden_set(1);
        phost->appstate = APPLICATION_RUNNING;
    }
}

/*
 * USBH_reset_state_machine() - drop all devices below the root of a port
 *                              after the root device has disconnected.
 *
 * This may be called from interrupt context (HAL_HCD_Disconnect_Callback),
 * so it only flags the request; usb_reset_pending() does the work from
 * cubeusb_poll(). It previously called release_usb_handles(usb_root[port])
 * directly, which freed the root handle (and the whole device tree) from
 * the interrupt handler, while hhcd->pData still pointed at the root and
 * the main loop could be in USBH_Process() on one of the freed handles.
 * The root handle itself must stay: it is the HCD's handle for the port
 * and the core state machine (HOST_DEV_DISCONNECTED) re-enumerates on it.
 */
static void
USBH_reset_state_machine(int port)
{
    if ((port < 0) || (port > 1))
        return;
    usbport[port].reset_pending = 1;
}

static void
usb_reset_pending(uint port)
{
    USBH_HandleTypeDef *root;

    if (usbport[port].reset_pending == 0)
        return;
    usbport[port].reset_pending = 0;

    root = usb_root[port];
    if (root != NULL)
        USBH_remove_subdevices(root);
    phost_cur[port] = NULL;
    ctl_stay_handle[port] = NULL;
}

static void
host_hc_init(USB_OTG_GlobalTypeDef *USBx, uint ch_num, uint epnum,
             uint dev_address, uint speed, uint ep_type, uint mps)
{
    uint32_t USBx_BASE = (uint32_t) USBx;
    uint32_t HCcharEpDir;
    uint32_t HCcharLowSpeed;
    uint32_t HostCoreSpeed;

    /* Program the HCCHAR register */
    if ((epnum & 0x80U) == 0x80U) {
        HCcharEpDir = (0x1U << 15) & USB_OTG_HCCHAR_EPDIR;
    } else {
        HCcharEpDir = 0U;
    }
#if 0
    HAL_HCD_HC_ClearHubInfo(hhcd, ch_num);
#endif

    HostCoreSpeed = USB_GetHostSpeed(USBx);

    /* LS device plugged to HUB */
    if ((speed == HPRT0_PRTSPD_LOW_SPEED) &&
        (HostCoreSpeed != HPRT0_PRTSPD_LOW_SPEED)) {
        HCcharLowSpeed = (0x1U << 17) & USB_OTG_HCCHAR_LSDEV;
    } else {
        HCcharLowSpeed = 0U;
    }

    USBx_HC(ch_num)->HCCHAR = ((dev_address << 22) & USB_OTG_HCCHAR_DAD) |
                              (((epnum & 0x7FU) << 11) & USB_OTG_HCCHAR_EPNUM) |
                              ((ep_type << 18) & USB_OTG_HCCHAR_EPTYP) |
                              (mps & USB_OTG_HCCHAR_MPSIZ) |
                              USB_OTG_HCCHAR_MC_0 | HCcharEpDir |
                              HCcharLowSpeed;

    if ((ep_type == EP_TYPE_INTR) || (ep_type == EP_TYPE_ISOC)) {
        USBx_HC(ch_num)->HCCHAR |= USB_OTG_HCCHAR_ODDFRM;
    }
}

/*
 * @brief  USBH_switch_to_dev
 *         Setup endpoint with selected device info
 * @param  phost: Host handle
 * @retval Status
 */
HAL_StatusTypeDef
USBH_switch_to_dev(USBH_HandleTypeDef *phost)
{
    HCD_HandleTypeDef *hhcd = &_hHCD[phost->id];
    uint ch_out = phost->Control.pipe_out;
    uint ch_in  = phost->Control.pipe_in;

#if 0
    if ((hhcd->hc[ch_out].dev_addr != phost->device.address) ||
        (hhcd->hc[ch_in].dev_addr != phost->device.address)) {
       printf("USB%s switch_to_dev\n", usb_get_path(phost, NULL, -1));
    }
#endif
    /*
     * The control channels are shared by every device on this port, and
     * USB_HC_StartXfer() does not reprogram the device address in HCCHAR;
     * it reuses whatever was last written. So the channels must be
     * reprogrammed when switching to a different device.
     *
     * But they must NOT be rewritten while a transfer is in flight on
     * them. This function is called before every USBH_Process() pass,
     * including the repeated passes on the same device while it waits
     * for a control transfer to complete. Rewriting HCCHAR under an
     * active transaction is not allowed by the core, and the window is
     * much larger for a low-speed device behind a hub (PRE + 1.5 Mbit/s)
     * than for full-speed devices. So only reprogram when something
     * actually changed. HAL_HCD_HC_Init() (USBH_OpenControlPipes) keeps
     * hhcd->hc[] up to date, so it is used as the record of what the
     * hardware currently holds.
     */
    uint need_init = (hhcd->pData != phost) ||
                     (hhcd->hc[ch_out].dev_addr != phost->device.address) ||
                     (hhcd->hc[ch_in].dev_addr != phost->device.address) ||
                     (hhcd->hc[ch_out].max_packet != phost->Control.pipe_size) ||
                     (hhcd->hc[ch_in].max_packet != phost->Control.pipe_size) ||
                     (hhcd->hc[ch_out].speed != phost->device.speed) ||
                     (hhcd->hc[ch_in].speed != phost->device.speed);

    hhcd->pData = phost;
    phost->pData = hhcd;

    if (!need_init)
        return (HAL_OK);

    if (phost->RequestState == CMD_WAIT) {
        /* Should not happen: the poll loop doesn't switch mid-transfer */
        printf("USB%s switch_to_dev during control transfer\n",
               usb_get_path(phost, NULL, -1));
        return (HAL_OK);
    }

    __HAL_LOCK(hhcd);

    hhcd->hc[ch_out].dev_addr   = phost->device.address;
    hhcd->hc[ch_out].max_packet = phost->Control.pipe_size;
    hhcd->hc[ch_out].speed      = phost->device.speed;
    hhcd->hc[ch_out].do_ping    = 0U;
    hhcd->hc[ch_out].ch_num     = ch_out;
    hhcd->hc[ch_out].ep_num     = 0;
    hhcd->hc[ch_out].ep_is_in   = 0;
    hhcd->hc[ch_out].ep_type    = EP_TYPE_CTRL;

    hhcd->hc[ch_in].dev_addr    = phost->device.address;
    hhcd->hc[ch_in].max_packet  = phost->Control.pipe_size;
    hhcd->hc[ch_in].speed       = phost->device.speed;
    hhcd->hc[ch_in].do_ping     = 0U;
    hhcd->hc[ch_in].ch_num      = ch_in;
    hhcd->hc[ch_in].ep_num      = 0;
    hhcd->hc[ch_in].ep_is_in    = 1;
    hhcd->hc[ch_in].ep_type     = EP_TYPE_CTRL;

    /* EP0 IN is 0x80 and EP0 OUT is always 0x00 (not phost->OutEp) */
    host_hc_init(hhcd->Instance, ch_in, 0x80, phost->device.address,
                 phost->device.speed, USBH_EP_CONTROL,
                 phost->Control.pipe_size);
    host_hc_init(hhcd->Instance, ch_out, 0x00, phost->device.address,
                 phost->device.speed, USBH_EP_CONTROL,
                 phost->Control.pipe_size);

    __HAL_UNLOCK(hhcd);

    return (HAL_OK);
}

USBH_HandleTypeDef *
USBH_get_root_device(USBH_HandleTypeDef *phost)
{
    while (phost->parent != NULL)
        phost = phost->parent;
    return (phost);
}

void
USBH_remove_subdevices(USBH_HandleTypeDef *phost)
{
    /*
     * release_usb_handles() releases a handle and its descendants, but
     * not its siblings, so walk the whole child list here.
     */
    while (phost->child != NULL)
        release_usb_handles(phost->child);
}


#define LOG(...) printf(__VA_ARGS__)

static void
process_usb_ports(uint port)
{
    USBH_HandleTypeDef *phost;
    uint64_t tick_enter;
    uint64_t tick_diff;

    phost = phost_cur[port];
    if (phost == NULL)
        phost = usb_root[port];

    if (phost != NULL) {
        // XXX: This might need work. Should be able to do discovery
        //      when phost doesn't exist. The phost should get
        //      created during discovery, I would think.

        /* Handle device class "initialization done" discovery */
        handle_discovery(phost, port, 0);
    }

    if (phost != NULL) {
        switch (phost->valid) {
            case 0:  // Not valid
                break;
            case 1:
                tick_enter = timer_tick_get();
                USBH_switch_to_dev(phost);
                USBH_Process(phost);
                tick_diff = timer_tick_get() - tick_enter;
                if (phost->tick_max < tick_diff)
                    phost->tick_max = tick_diff;
                phost->tick_total += tick_diff;
                phost->poll_count++;

                /*
                 * A control transfer in flight owns the shared EP0
                 * channels. Switching to another device now would
                 * reprogram the channel's device address under the
                 * transfer (NAK retries would then go to the wrong
                 * device). USBH_CtlReq() times out after 2000 ticks;
                 * the bound here only protects against a class driver
                 * which abandons a request without calling it again.
                 * Note that phost->Timer only advances while this
                 * handle is the current one (hhcd->pData).
                 */
                if ((phost->RequestState == CMD_WAIT) &&
                    (phost->Timer - phost->ctl_timer < 2500)) {
                    if (ctl_stay_handle[port] != phost) {
                        ctl_stay_handle[port] = phost;
                        ctl_stay_deadline[port] =
                            timer_tick_plus_msec(USB_CTL_STAY_MAX_MS);
                    }
                    if (!timer_tick_has_elapsed(ctl_stay_deadline[port]))
                        return;
                    printf("USB%s control transfer stalled; moving on\n",
                           usb_get_path(phost, NULL, -1));
                }
                ctl_stay_handle[port] = NULL;

                if (phost->busy) {
                    /* Don't go to next device until this one is done */
                    phost->poll_no_progress++;
                    if (phost->poll_no_progress < 500)
                        return;
                } else {
                    /* This one is no longer busy */
                    phost->poll_no_progress = 0;
                }
                break;
            case 3:
                LOG("USB%s PROCESSING ATTACH\n", usb_get_path(phost, NULL, -1));

                phost->valid = 1;
                break;
            default:
                printf("USB%s Unknown valid %u for cur=%u\n",
                       usb_get_path(phost, NULL, -1),
                       phost->valid, phost->hubdev);
                break;
        }
        /* Locate the next phost */
        if (phost->child != NULL) {
            phost = phost->child;
        } else if (phost->next != NULL) {
            phost = phost->next;
        } else while ((phost = phost->parent) != NULL) {
            if (phost->next != NULL) {
                phost = phost->next;
                break;
            }
        }
    }
    phost_cur[port] = phost;
}

#if 0
static void
wait_sof(uint port)
{
    uint frame = usbport[port].frame;
    uint64_t timeout = timer_tick_plus_msec(1000);
    while (frame == usbport[port].frame) {
        if (timer_tick_has_elapsed(timeout)) {
            printf("USB%u SOF timeout\n", port);
            break;
        }
    }
}
#endif

void
cubeusb_poll(void)
{
    /* USB Host Background task */
    for (uint port = 0; port < 2; port++) {
        /* Deferred device tree teardown requested from interrupt context */
        usb_reset_pending(port);
#ifdef USB_TRACE
        /* Sample the bus if a low-speed transfer is taking too long */
        utr_stall_check(port);
#endif

        /*
         * Port came up after reset, but the HAL had to change the FS/LS
         * PHY clock select for the device's speed (low-speed device
         * directly on the port). The core needs another port reset
         * after that before the bus is usable. The host state machine
         * keeps waiting for PortEnabled, which follows this reset.
         */
        if (usbport[port].clk_reset_pending) {
            usbport[port].clk_reset_pending = 0;
            if (usbport[port].connected) {
                dprintf(DF_USB_CONN, "USB%u PHY clock changed; reset port\n",
                        port);
                usbport[port].clk_resetting = 1;  // Cleared on PortEnabled
                HAL_HCD_HaltAllChannels(&_hHCD[port]);
#ifdef USB_TRACE
                utr_add(&_hHCD[port], UTR_RESET, 0, 1, 0);
#endif
                (void) HAL_HCD_ResetPort(&_hHCD[port]);
#ifdef USB_TRACE
                utr_add(&_hHCD[port], UTR_RESET, 0, 0, 0);
#endif
            }
        }

        /* Root device enumeration retry: re-initialize the port */
        if (usbport[port].reinit_pending &&
            timer_tick_has_elapsed(usbport[port].reinit_timer)) {
            usbport[port].reinit_pending = 0;
            cubeusb_shutdown_port(port);
            cubeusb_init_port(port);
        }
#if 1
        if ((usbport[port].recovery_state != 0) &&
            timer_tick_has_elapsed(usbport[port].recovery_timer)) {
            /*
             * recovery_state is 1 + the number of re-init attempts made.
             * It used to be incremented twice per attempt, so the
             * "tries" / "attempts" counts printed were double the real
             * number of port re-inits (and the limits were half).
             * USB_RECOVERY_FAST attempts are made quickly, then slower
             * ones, up to USB_RECOVERY_MAX in all (the same schedule as
             * before: 5 fast + 5 slow).
             */
            const uint usb_recovery_fast = 5;
            const uint usb_recovery_max  = 10;
            uint attempts = usbport[port].recovery_state - 1;
            if (usbport[port].connected == 0) {
                /* Port is no longer connected (device pull?) */
                printf("USB is not connected\n");
                USBH_LL_Disconnect(_hHCD[port].pData);
                USBH_reset_state_machine(port);
                usbport[port].recovery_state = 0;
                break;
            }
            if (usbport[port].disabled == 0) {
                /* Port has recovered */
                printf("Recovered USB%u after %u port re-init%s\n",
                       port, attempts,
                       (attempts == 1) ? "" : "s");
                usbport[port].rec_ok++;
                usbport[port].recovery_state = 0;
                break;
            }
            if (attempts < usb_recovery_max) {
                uint msec;
                printf("\nUSB%u port disabled; re-init %u of %u\n",
                       port, attempts + 1, usb_recovery_max);
                if (attempts == 0)
                    usbport[port].rec_episodes++;
                usbport[port].rec_reinits++;
                usbport[port].disabled = 0;
                cubeusb_shutdown_port(port);
//              wait_sof(port);
                cubeusb_init_port(port);
                usbport[port].recovery_state++;
                attempts++;

                if (attempts < usb_recovery_fast)
                    msec = 1024 * (attempts + 1);
                else
                    msec = 10000 + 2048 * attempts;
                usbport[port].recovery_timer = timer_tick_plus_msec(msec);
            } else {
                /*
                 * Give up. Previously the device tree was left in place:
                 * a device which was mid-enumeration kept its control
                 * transfer pending on a disabled port forever (no SOFs,
                 * so its timeout never ran), printing "control transfer
                 * stalled; moving on" every few seconds. Release
                 * everything below the root; unplugging and replugging
                 * the device (a connect interrupt) starts over.
                 */
                printf("USB%u port recovery failed after %u port "
                       "re-inits; replug to retry\n",
                       port, attempts);
                usbport[port].rec_fail++;
                if (usb_root[port] != NULL)
                    USBH_remove_subdevices(usb_root[port]);
                phost_cur[port] = NULL;
                ctl_stay_handle[port] = NULL;
                usbport[port].recovery_state = 0;
            }
        }
#endif
        /* Handle host and hub port processing */
        process_usb_ports(port);
    }
}

#if 0
uint32_t
HAL_GetTick(void)
{
    printf("HAL_GetTick %lu\n", uwTick);
    return (uwTick);
}

HAL_TickFreqTypeDef
HAL_GetTickFreq(void)
{
    printf("HAL_GetTickFreq\n");
    return (uwTickFreq);
}

uint32_t
HAL_GetTickPrio(void)
{
    printf("HAL_GetTickPrio\n");
    return (uwTickPrio);
}

void
HAL_ResumeTick(void)
{
    /* Enable SysTick Interrupt */
    printf("HAL_ResumeTick\n");
    SysTick->CTRL  |= SysTick_CTRL_TICKINT_Msk;
}

void
HAL_SuspendTick(void)
{
    /* Disable SysTick Interrupt */
    printf("HALSuspendTick\n");
    SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
}
#endif

void
HAL_Delay(uint32_t Delay)
{
    timer_delay_msec(Delay);
}

void
HAL_HCD_MspInit(HCD_HandleTypeDef *hhcd)
{
//  printf("HAL_HCD_MspInit\n");
}

void
HAL_HCD_MspDeInit(HCD_HandleTypeDef *hhcd)
{
    printf("HAL_HCD_MspDeInit\n");
}

void
HAL_HCD_SOF_Callback(HCD_HandleTypeDef *hhcd)
{
    int port = (hhcd == &_hHCD[0]) ? 0 : 1;
    usbport[port].frame++;
    USBH_LL_IncTimer(hhcd->pData);
}

/*
 * Host channel event statistics, per STM32 physical port, for diagnosing
 * transfer errors. Counted from the HAL channel interrupt handler.
 */
typedef struct {
    uint32_t rx_pkts;   // IN data packets received without error
    uint32_t xacterr;   // CRC, timeout, bit stuff, false EOP
    uint32_t babble;
    uint32_t toggle;    // Data toggle error
    uint32_t stall;
    uint32_t frmor;     // Frame overrun
    uint32_t nak;
    uint32_t incper;    // Incomplete periodic transfer (GINTSTS.IPXFR)
} usb_hc_stats_t;

static volatile usb_hc_stats_t usb_hc_stats[2];

/* Called from the HAL HCD interrupt handler (stm32f2xx_hal_hcd.c) */
void usb_count_hc_int(HCD_HandleTypeDef *hhcd, uint32_t chnum, uint32_t hcint);
void usb_count_hc_rx(HCD_HandleTypeDef *hhcd, uint32_t chnum);
void usb_count_incomplete_periodic(HCD_HandleTypeDef *hhcd);

void
usb_count_hc_int(HCD_HandleTypeDef *hhcd, uint32_t chnum, uint32_t hcint)
{
    volatile usb_hc_stats_t *st = &usb_hc_stats[(hhcd == &_hHCD[0]) ? 0 : 1];
    (void) chnum;

#ifdef USB_TRACE
    /* Record which device / endpoint the channel is programmed for */
    utr_add(hhcd, UTR_INT, chnum,
            (USBx_HC_RAW(hhcd, chnum)->HCINT & 0xffff) |
            (utr_hcchar_ep(USBx_HC_RAW(hhcd, chnum)->HCCHAR) << 16),
            USBx_HC_RAW(hhcd, chnum)->HCTSIZ);
#endif

    if (hcint & USB_OTG_HCINT_TXERR)
        st->xacterr++;
    if (hcint & USB_OTG_HCINT_BBERR)
        st->babble++;
    if (hcint & USB_OTG_HCINT_DTERR)
        st->toggle++;
    if (hcint & USB_OTG_HCINT_STALL)
        st->stall++;
    if (hcint & USB_OTG_HCINT_FRMOR)
        st->frmor++;
    if (hcint & USB_OTG_HCINT_NAK)
        st->nak++;
}

void
usb_count_hc_rx(HCD_HandleTypeDef *hhcd, uint32_t chnum)
{
    (void) chnum;
    usb_hc_stats[(hhcd == &_hHCD[0]) ? 0 : 1].rx_pkts++;
}

/*
 * GINTSTS.IPXFR: a periodic (interrupt endpoint) transaction queued for a
 * frame was not completed in that frame. Called from the HAL interrupt
 * handler before it clears the flag.
 */
void
usb_count_incomplete_periodic(HCD_HandleTypeDef *hhcd)
{
    usb_hc_stats[(hhcd == &_hHCD[0]) ? 0 : 1].incper++;
#ifdef USB_TRACE
    utr_add(hhcd, UTR_IPXFR, 0, hhcd->Instance->GINTSTS, 0);
#endif
}

/*
 * usb_hc_stats_report() - print and clear channel event counts for the
 *                         physical port of the given device.
 */
void
usb_hc_stats_report(USBH_HandleTypeDef *phost)
{
    uint port = phost->id & 1;
    volatile usb_hc_stats_t *st = &usb_hc_stats[port];

    printf("USB%u channel stats: rx=%lu xacterr=%lu babble=%lu toggle=%lu "
           "stall=%lu frmor=%lu nak=%lu incper=%lu\n", port,
           st->rx_pkts, st->xacterr, st->babble, st->toggle,
           st->stall, st->frmor, st->nak, st->incper);
    memset((void *) st, 0, sizeof (*st));
}

void
HAL_HCD_Connect_Callback(HCD_HandleTypeDef *hhcd)
{
    int port = (hhcd == &_hHCD[0]) ? 0 : (hhcd == &_hHCD[1]) ? 1 : -1;
    dprintf(DF_USB_CONN, "USB%d HAL_HCD_Connect\n", port);
    if (port < 0)
        return;
    usbport[port].connected = 1;
#ifdef USB_TRACE
    utr_start(port);
    utr_add(hhcd, UTR_CONN, 0, USBx_HPRT0_RAW(hhcd), 0);
#endif
    USBH_LL_Connect(hhcd->pData);
}

void
HAL_HCD_Disconnect_Callback(HCD_HandleTypeDef *hhcd)
{
    int port = (hhcd == &_hHCD[0]) ? 0 : (hhcd == &_hHCD[1]) ? 1 : -1;
    dprintf(DF_USB_CONN, "USB%d HAL_HCD_Disconnect\n", port);
    if (port < 0)
        return;
    usbport[port].connected = 0;
    /* Nothing downstream remains; don't let a transfer resume later */
    HAL_HCD_HaltAllChannels(hhcd);
#ifdef USB_TRACE
    utr_add(hhcd, UTR_DISC, 0, USBx_HPRT0_RAW(hhcd), 0);
    utr_stop(port);
#endif
    usbport[port].clk_reset_pending = 0;
    usbport[port].clk_resetting = 0;
    USBH_LL_Disconnect(hhcd->pData);
#if 1
//  XXX: CDH - disable below line to debug EasySMX game controller
    USBH_reset_state_machine(port);
#endif
}

void
HAL_HCD_HC_NotifyURBChange_Callback(HCD_HandleTypeDef *hhcd, uint8_t chnum,
                                    HCD_URBStateTypeDef urb_state)
{
//  printf("HAL_HCD_HC_NotifyURBChange_Callback\n");
#if (USBH_USE_OS == 1)
    /* To be used with OS to sync URB state with the global state machine */
    USBH_LL_NotifyURBChange(hhcd->pData);
#endif
}

void
HAL_HCD_PortEnabled_Callback(HCD_HandleTypeDef *hhcd)
{
    int port = (hhcd == &_hHCD[0]) ? 0 : (hhcd == &_hHCD[1]) ? 1 : -1;
    dprintf(DF_USB_CONN, "USB%d HAL_HCD_PortEnabled\n", port);
    if (port >= 0)
        usbport[port].clk_resetting = 0;
#ifdef USB_TRACE
    utr_add(hhcd, UTR_PEN, 0, USBx_HPRT0_RAW(hhcd), 0);
#endif
    USBH_LL_PortEnabled(hhcd->pData);
}

void
HAL_HCD_PortClockChanged_Callback(HCD_HandleTypeDef *hhcd)
{
    int port = (hhcd == &_hHCD[0]) ? 0 : (hhcd == &_hHCD[1]) ? 1 : -1;
    dprintf(DF_USB_CONN, "USB%d HAL_HCD_PortClockChanged\n", port);
    if (port < 0) {
        HAL_HCD_PortEnabled_Callback(hhcd);
        return;
    }
    usbport[port].clk_reset_pending = 1;  // Reset from cubeusb_poll()
}

void
HAL_HCD_PortDisabled_Callback(HCD_HandleTypeDef *hhcd)
{
    int port = (hhcd == &_hHCD[0]) ? 0 : (hhcd == &_hHCD[1]) ? 1 : -1;
    USBH_HandleTypeDef *phost = hhcd->pData;
    port = phost->id;
    dprintf(DF_USB_CONN, "USB%s HAL_HCD_PortDisabled\n",
            usb_get_path(phost, NULL, -1));
    /*
     * USB_TRACE: the UTR_PDIS event and a register capture were recorded
     * by usb_trace_port_disabled() in the port interrupt handler, before
     * this callback.
     */
    USBH_LL_PortDisabled(phost);
    if (usbport[port].clk_resetting) {
        /* Expected: our own re-reset after a PHY clock change */
        return;
    }
    if (usbport[port].connected) {
        usbport[port].disabled = 1;  // Mark it as (unexpected) disabled
#ifdef USB_TRACE
        utr_ring_drop(port, UTR_DROP_ROOT, USBx_HPRT0_RAW(hhcd), 0);
#endif
        /*
         * Everything below the root is lost with the port. Tear it down
         * now (from cubeusb_poll(), via reset_pending) rather than at the
         * next re-init, which can be seconds away: until then a device
         * that was mid-enumeration sat on a control transfer that can't
         * complete (no SOFs on a disabled port) and printed "control
         * transfer stalled; moving on" every few seconds.
         */
        USBH_reset_state_machine(port);

        if (usbport[port].recovery_state == 0) {
            usbport[port].recovery_state = 1;  // Kickstart recovery
            usbport[port].recovery_timer = timer_tick_plus_msec(10);
        }
    }
}

USBH_StatusTypeDef
USBH_LL_Init(USBH_HandleTypeDef *phost)
{
    /* Init USB_IP */
    if (phost->id == HOST_FS) {
        /* Link the OTG_FS driver to the stack */
        _hHCD[0].pData = phost;
        _hHCD[0].Instance = USB_OTG_FS;
        _hHCD[0].Init.Host_channels = 8;
        _hHCD[0].Init.speed = HCD_SPEED_FULL;
        _hHCD[0].Init.dma_enable = DISABLE;
        _hHCD[0].Init.phy_itface = HCD_PHY_EMBEDDED;
        _hHCD[0].Init.Sof_enable = DISABLE;
        phost->pData = &_hHCD[0];

    } else if (phost->id == HOST_HS) {
        /* Link the OTG_HS driver to the stack */
        _hHCD[1].pData = phost;
        _hHCD[1].Instance = USB_OTG_HS;
        _hHCD[1].Init.Host_channels = 12;
        _hHCD[1].Init.speed = HCD_SPEED_FULL;
        _hHCD[1].Init.dma_enable = DISABLE;
        _hHCD[1].Init.phy_itface = USB_OTG_EMBEDDED_PHY;
        _hHCD[1].Init.Sof_enable = DISABLE;
        _hHCD[1].Init.low_power_enable = DISABLE;
        _hHCD[1].Init.vbus_sensing_enable = DISABLE;
        _hHCD[1].Init.use_external_vbus = DISABLE;
        phost->pData = &_hHCD[1];

    } else {
        return (USBH_FAIL);
    }
    if (HAL_HCD_Init(phost->pData) != HAL_OK) {
        printf("HAL_HCD_Init OTG_HS error\n");
        return (USBH_FAIL);
    }

    /*
     * FS timeout calibration: extra PHY clocks added to the full-speed
     * inter-packet (response) timeout to account for delays outside the
     * core. The HAL leaves this at 0. Linux dwc2 uses 7 for FS PHYs. A
     * low-speed device behind a hub (hub repeater delay + LS turnaround)
     * can otherwise reply just after the host gives up, which the core
     * reports as a transaction error (XACTERR).
     */
    {
        HCD_HandleTypeDef *hhcd = phost->pData;
        uint32_t usbcfg = hhcd->Instance->GUSBCFG;
        usbcfg &= ~USB_OTG_GUSBCFG_TOCAL;
        usbcfg |= (USB_HOST_TOCAL << USB_OTG_GUSBCFG_TOCAL_Pos) &
                  USB_OTG_GUSBCFG_TOCAL;
        hhcd->Instance->GUSBCFG = usbcfg;
    }
    USBH_LL_SetTimer(phost, HAL_HCD_GetCurrentFrame(phost->pData));
    return (USBH_OK);
}

/*
 * Virtual pipes
 *
 * The STM32F2 OTG_FS core has only 8 host channels (OTG_HS has 12), but
 * a few HID devices behind a hub easily need more pipes than that (e.g.
 * 2 control + 2 hub + 3 Corsair + 3 Logitech = 10). The ST host library
 * uses pipe numbers directly as hardware channel numbers, so pipes beyond
 * the channel count were programmed into channels that don't exist (and
 * writes to "channel 8/9" may land on channels 0/1, the control
 * channels).
 *
 * Here pipe numbers are logical. Pipes 0 and 1 (the shared EP0 control
 * pipes) map directly to channels 0 and 1. Every other pipe is bound to a
 * free hardware channel only while it has a transfer in flight, and
 * released when the transfer reaches a terminal state (the hardware
 * channel is halted by then). The endpoint parameters, data toggles,
 * last URB state and transfer count are kept per logical pipe.
 * (Linux dwc2 assigns channels to endpoints dynamically in the same way.)
 *
 * If no channel is free, a submit is treated as NAKed (URB_NOTREADY); the
 * class drivers already retry on their next poll interval.
 */
#define VPIPE_FIXED  2      // Pipes 0..1 are fixed (EP0 control)
#define VPIPE_NONE   0xff

typedef struct {
    uint8_t  open;
    uint8_t  ch;            // Bound hardware channel, or VPIPE_NONE
    uint8_t  epnum;
    uint8_t  dev_address;
    uint8_t  speed;
    uint8_t  ep_type;
    uint16_t mps;
    uint8_t  toggle_in;
    uint8_t  toggle_out;
    uint8_t  urb_state;     // Last URB state (valid while unbound)
    uint32_t xfer_count;    // Last transfer count (valid while unbound)
} vpipe_t;

static vpipe_t vpipe[2][USBH_MAX_PIPES_NBR];
static uint8_t vch_owner[2][16];   // Logical pipe bound to channel

static inline uint
vp_port(USBH_HandleTypeDef *phost)
{
    return (phost->id & 1);
}

static void
vpipe_reset_port(uint port)
{
    memset(vpipe[port], 0, sizeof (vpipe[port]));
    for (uint i = 0; i < USBH_MAX_PIPES_NBR; i++)
        vpipe[port][i].ch = VPIPE_NONE;
    memset(vch_owner[port], VPIPE_NONE, sizeof (vch_owner[port]));
}

/* Save state from the bound channel and release it */
static void
vpipe_unbind(uint port, vpipe_t *vp)
{
    HCD_HandleTypeDef *hhcd = &_hHCD[port];
    uint ch = vp->ch;

    if (ch == VPIPE_NONE)
        return;
    vp->toggle_in  = hhcd->hc[ch].toggle_in;
    vp->toggle_out = hhcd->hc[ch].toggle_out;
    vp->urb_state  = hhcd->hc[ch].urb_state;
    vp->xfer_count = hhcd->hc[ch].xfer_count;
    vch_owner[port][ch] = VPIPE_NONE;
    vp->ch = VPIPE_NONE;
}

/* Bind a free hardware channel to a logical pipe; returns 0 if none free */
static uint
vpipe_bind(uint port, uint pipe)
{
    HCD_HandleTypeDef *hhcd = &_hHCD[port];
    vpipe_t *vp = &vpipe[port][pipe];
    uint nch = hhcd->Init.Host_channels;
    uint ch;

    if (vp->ch != VPIPE_NONE)
        return (1);
    if (nch > 16)
        nch = 16;
    for (ch = VPIPE_FIXED; ch < nch; ch++)
        if (vch_owner[port][ch] == VPIPE_NONE)
            break;
    if (ch >= nch)
        return (0);

    vch_owner[port][ch] = pipe;
    vp->ch = ch;
    (void) HAL_HCD_HC_Init(hhcd, ch, vp->epnum, vp->dev_address, vp->speed,
                           vp->ep_type, vp->mps);
    hhcd->hc[ch].toggle_in  = vp->toggle_in;
    hhcd->hc[ch].toggle_out = vp->toggle_out;
    hhcd->hc[ch].ErrCnt     = 0;
    hhcd->hc[ch].xfer_count = 0;
    hhcd->hc[ch].urb_state  = URB_IDLE;
    return (1);
}

/* Is a URB state final, with the hardware channel no longer active? */
static uint
vpipe_terminal(uint ep_type, uint state)
{
    switch (state) {
        case URB_DONE:
        case URB_ERROR:
        case URB_STALL:
            return (1);
        case URB_NOTREADY:
            /*
             * For interrupt/isoc, NAK and retried errors halt the channel.
             * For bulk the HAL re-activates the channel itself, so the
             * transfer is still in progress.
             */
            return ((ep_type == EP_TYPE_INTR) || (ep_type == EP_TYPE_ISOC));
        default:
            return (0);
    }
}

USBH_URBStateTypeDef
USBH_LL_GetURBState(USBH_HandleTypeDef *phost, uint8_t pipe)
{
    uint port = vp_port(phost);
    vpipe_t *vp;

    if (pipe >= USBH_MAX_PIPES_NBR)
        return (USBH_URB_ERROR);
    if (pipe < VPIPE_FIXED)
        return (USBH_URBStateTypeDef) HAL_HCD_HC_GetURBState(phost->pData,
                                                             pipe);
    vp = &vpipe[port][pipe];
    if (vp->ch != VPIPE_NONE) {
        uint state = _hHCD[port].hc[vp->ch].urb_state;
        if (vpipe_terminal(vp->ep_type, state))
            vpipe_unbind(port, vp);
        return ((USBH_URBStateTypeDef) state);
    }
    return ((USBH_URBStateTypeDef) vp->urb_state);
}

void
USBH_LL_SetURBState(USBH_HandleTypeDef *phost, uint8_t pipe,
                    USBH_URBStateTypeDef state)
{
    uint port = vp_port(phost);
    vpipe_t *vp;

    if (pipe >= USBH_MAX_PIPES_NBR)
        return;
    if (pipe < VPIPE_FIXED) {
        HCD_HandleTypeDef *hhcd = phost->pData;
        hhcd->hc[pipe].urb_state = (USB_URBStateTypeDef) state;
        return;
    }
    vp = &vpipe[port][pipe];
    vp->urb_state = state;
    if (vp->ch != VPIPE_NONE)
        _hHCD[port].hc[vp->ch].urb_state = (USB_URBStateTypeDef) state;
}

USBH_StatusTypeDef
USBH_LL_DriverVBUS(USBH_HandleTypeDef *phost, uint8_t state)
{
    return (USBH_OK);
}

USBH_StatusTypeDef
USBH_LL_SetToggle(USBH_HandleTypeDef *phost, uint8_t pipe, uint8_t toggle)
{
    uint port = vp_port(phost);
    vpipe_t *vp;

    if (pipe >= USBH_MAX_PIPES_NBR)
        return (USBH_FAIL);
    if (pipe < VPIPE_FIXED) {
        HCD_HandleTypeDef *hhcd = phost->pData;
        if (hhcd->hc[pipe].ep_is_in) {
            hhcd->hc[pipe].toggle_in = toggle;
        } else {
            hhcd->hc[pipe].toggle_out = toggle;
        }
        return (USBH_OK);
    }
    vp = &vpipe[port][pipe];
    if (vp->epnum & 0x80)
        vp->toggle_in = toggle;
    else
        vp->toggle_out = toggle;
    if (vp->ch != VPIPE_NONE) {
        if (vp->epnum & 0x80)
            _hHCD[port].hc[vp->ch].toggle_in = toggle;
        else
            _hHCD[port].hc[vp->ch].toggle_out = toggle;
    }
    return (USBH_OK);
}

uint8_t
USBH_LL_GetToggle(USBH_HandleTypeDef *phost, uint8_t pipe)
{
    uint port = vp_port(phost);
    vpipe_t *vp;

    if (pipe >= USBH_MAX_PIPES_NBR)
        return (0);
    if (pipe < VPIPE_FIXED) {
        HCD_HandleTypeDef *hhcd = phost->pData;
        if (hhcd->hc[pipe].ep_is_in)
            return (hhcd->hc[pipe].toggle_in);
        return (hhcd->hc[pipe].toggle_out);
    }
    vp = &vpipe[port][pipe];
    if (vp->ch != VPIPE_NONE) {
        if (vp->epnum & 0x80)
            return (_hHCD[port].hc[vp->ch].toggle_in);
        return (_hHCD[port].hc[vp->ch].toggle_out);
    }
    return ((vp->epnum & 0x80) ? vp->toggle_in : vp->toggle_out);
}

static USBH_StatusTypeDef
USBH_Get_USB_Status(HAL_StatusTypeDef hal_status)
{
    USBH_StatusTypeDef usb_status = USBH_OK;

    switch (hal_status) {
        case HAL_OK:
            usb_status = USBH_OK;
            break;
        case HAL_ERROR:
            usb_status = USBH_FAIL;
            break;
        case HAL_BUSY:
            usb_status = USBH_BUSY;
            break;
        case HAL_TIMEOUT:
            usb_status = USBH_FAIL;
            break;
        default:
            usb_status = USBH_FAIL;
            break;
    }
    return (usb_status);
}

/*
 * usb_ctl_abort() - put the shared EP0 channels back into a known idle
 *                   state after a control transfer was abandoned (timeout).
 *
 * Halts a control channel only if it is still enabled (halting an idle
 * channel in slave mode would enable it), waits briefly for the halt to
 * complete, and forces USBH_switch_to_dev() to reprogram the channels.
 */
void
usb_ctl_abort(USBH_HandleTypeDef *phost)
{
    uint port = vp_port(phost);
    HCD_HandleTypeDef *hhcd = &_hHCD[port];
    uint32_t USBx_BASE = (uint32_t) hhcd->Instance;
    uint8_t chans[2] = { phost->Control.pipe_out, phost->Control.pipe_in };

    for (uint i = 0; i < 2; i++) {
        uint ch = chans[i];
        if (ch >= VPIPE_FIXED)
            continue;
        if (USBx_HC(ch)->HCCHAR & USB_OTG_HCCHAR_CHENA) {
            uint64_t timeout = timer_tick_plus_msec(5);
            (void) HAL_HCD_HC_Halt(hhcd, ch);
            while (USBx_HC(ch)->HCCHAR & USB_OTG_HCCHAR_CHENA)
                if (timer_tick_has_elapsed(timeout))
                    break;
        }
        hhcd->hc[ch].urb_state = URB_IDLE;
        hhcd->hc[ch].ErrCnt = 0;
        hhcd->hc[ch].dev_addr = 0xff;  // Force reprogram on next switch
    }
}

uint32_t
USBH_LL_GetLastXferSize(USBH_HandleTypeDef *phost, uint8_t pipe)
{
    uint port = vp_port(phost);
    vpipe_t *vp;

    if (pipe >= USBH_MAX_PIPES_NBR)
        return (0);
    if (pipe < VPIPE_FIXED)
        return (HAL_HCD_HC_GetXferCount(phost->pData, pipe));
    vp = &vpipe[port][pipe];
    if (vp->ch != VPIPE_NONE)
        return (_hHCD[port].hc[vp->ch].xfer_count);
    return (vp->xfer_count);
}

USBH_StatusTypeDef
USBH_LL_OpenPipe(USBH_HandleTypeDef *phost, uint8_t pipe_num, uint8_t epnum,
                 uint8_t dev_address, uint8_t speed, uint8_t ep_type,
                 uint16_t mps)
{
    HAL_StatusTypeDef hal_status;
    USBH_StatusTypeDef usb_status;
    uint port = vp_port(phost);
    vpipe_t *vp;

    if (pipe_num >= USBH_MAX_PIPES_NBR)
        return (USBH_FAIL);
    if (pipe_num < VPIPE_FIXED) {
        hal_status = HAL_HCD_HC_Init(phost->pData, pipe_num, epnum,
                                     dev_address, speed, ep_type, mps);
        usb_status = USBH_Get_USB_Status(hal_status);
        return (usb_status);
    }

    /* Logical pipe: record parameters; a channel is bound on submit */
    vp = &vpipe[port][pipe_num];
    if (vp->ch != VPIPE_NONE) {
        (void) HAL_HCD_HC_Halt(&_hHCD[port], vp->ch);
        vpipe_unbind(port, vp);
    }
    vp->open        = 1;
    vp->epnum       = epnum;
    vp->dev_address = dev_address;
    vp->speed       = speed;
    vp->ep_type     = ep_type;
    vp->mps         = mps;
    vp->toggle_in   = 0;
    vp->toggle_out  = 0;
    vp->urb_state   = URB_IDLE;
    vp->xfer_count  = 0;
    return (USBH_OK);
}

USBH_StatusTypeDef
USBH_LL_ClosePipe(USBH_HandleTypeDef *phost, uint8_t pipe)
{
    HAL_StatusTypeDef hal_status;
    uint port = vp_port(phost);
    vpipe_t *vp;

    if (pipe >= USBH_MAX_PIPES_NBR)
        return (USBH_FAIL);
    if (pipe < VPIPE_FIXED) {
        hal_status = HAL_HCD_HC_Halt(phost->pData, pipe);
        return (USBH_Get_USB_Status(hal_status));
    }
    vp = &vpipe[port][pipe];
    if (vp->ch != VPIPE_NONE) {
        (void) HAL_HCD_HC_Halt(&_hHCD[port], vp->ch);
        vpipe_unbind(port, vp);
    }
    vp->open = 0;
    vp->urb_state = URB_IDLE;
    return (USBH_OK);
}

USBH_StatusTypeDef
USBH_LL_SubmitURB(USBH_HandleTypeDef *phost, uint8_t pipe, uint8_t direction,
                  uint8_t ep_type, uint8_t token, uint8_t *pbuff,
                  uint16_t length, uint8_t do_ping)
{
    HAL_StatusTypeDef hal_status;
    uint port = vp_port(phost);
    vpipe_t *vp;

    if (pipe >= USBH_MAX_PIPES_NBR)
        return (USBH_FAIL);
    if (pipe < VPIPE_FIXED) {
        hal_status = HAL_HCD_HC_SubmitRequest(phost->pData, pipe, direction,
                                              ep_type, token, pbuff, length,
                                              do_ping);
        return (USBH_Get_USB_Status(hal_status));
    }

    vp = &vpipe[port][pipe];
    if (!vp->open)
        return (USBH_FAIL);
    if (!vpipe_bind(port, pipe)) {
        /* No free hardware channel: behave as if the device NAKed */
        vp->urb_state = URB_NOTREADY;
        return (USBH_OK);
    }
    hal_status = HAL_HCD_HC_SubmitRequest(&_hHCD[port], vp->ch, direction,
                                          ep_type, token, pbuff, length,
                                          do_ping);
    return (USBH_Get_USB_Status(hal_status));
}

USBH_StatusTypeDef
USBH_LL_Start(USBH_HandleTypeDef *phost)
{
    HAL_StatusTypeDef hal_status = HAL_HCD_Start(phost->pData);

    return (USBH_Get_USB_Status(hal_status));
}

USBH_StatusTypeDef
USBH_LL_Stop(USBH_HandleTypeDef *phost)
{
    HAL_StatusTypeDef hal_status = HAL_HCD_Stop(phost->pData);

    return (USBH_Get_USB_Status(hal_status));
}

/* It's not clear that this code from STM32F4HUB is needed */
USBH_StatusTypeDef
USBH_LL_StopHC(USBH_HandleTypeDef *phost, uint8_t chnum)
{
    /* chnum is a (logical) pipe number */
    uint port = vp_port(phost);
    vpipe_t *vp;

    if (chnum >= USBH_MAX_PIPES_NBR)
        return (USBH_FAIL);
    if (chnum < VPIPE_FIXED) {
        HAL_HCD_StopHC(phost->pData, chnum);
        return (USBH_OK);
    }
    vp = &vpipe[port][chnum];
    if (vp->ch != VPIPE_NONE) {
        HAL_HCD_StopHC(&_hHCD[port], vp->ch);
        vpipe_unbind(port, vp);
    }
    return (USBH_OK);
}

USBH_SpeedTypeDef
USBH_LL_GetSpeed(USBH_HandleTypeDef *phost)
{
    USBH_SpeedTypeDef speed = USBH_SPEED_FULL;

    switch (HAL_HCD_GetCurrentSpeed(phost->pData)) {
        case 0:
            speed = USBH_SPEED_HIGH;
            break;
        case 1:
            speed = USBH_SPEED_FULL;
            break;
        case 2:
            speed = USBH_SPEED_LOW;
            break;
        default:
            speed = USBH_SPEED_FULL;
            break;
    }
    return (speed);
}

void *
USBH_malloc(size_t size)
{
    void *ptr;
    __disable_irq();
    ptr = malloc(size);
    __enable_irq();
#undef MALLOC_DEBUG
#ifdef MALLOC_DEBUG
    printf("+ MALLOC(%x %x)\n", size, (uintptr_t) ptr);
#endif
    return (ptr);
}

void
USBH_free(void *ptr)
{
#ifdef MALLOC_DEBUG
    size_t size = 0;
    if (ptr != NULL) {
        size = ((uint32_t *)ptr)[-1];
        size = (size & ~1) - 8;
    }
    printf("- FREE(%x %x)\n", size, (uintptr_t) ptr);
#endif
    __disable_irq();
    free(ptr);
    __enable_irq();
}


/*
 * Select the low-speed PHY clock before resetting a port with a low-speed
 * device on it.
 *
 * The port reports the attached device's speed (HPRT.PSPD) at connect.
 * Previously the port was always reset with the 48 MHz clock selected;
 * the HAL then switched to 6 MHz when the port enabled and a second reset
 * was issued. For about 3 ms between the two resets the port ran with
 * the full-speed clock and a low-speed device attached. The MacAlly
 * MKEYE keyboard came out of that sequence unresponsive in almost every
 * attempt (it reset itself at a fixed ~393 ms after the first reset,
 * whatever was sent to it). Selecting the clock first means one reset and
 * no full-speed-clocked frames. Set USB_LS_CLOCK_BEFORE_RESET to 0 to
 * restore the old behavior.
 */
#ifndef USB_LS_CLOCK_BEFORE_RESET
#define USB_LS_CLOCK_BEFORE_RESET 1
#endif

static void
usb_preset_phy_clock(HCD_HandleTypeDef *hhcd)
{
#if USB_LS_CLOCK_BEFORE_RESET
    uint32_t hprt0 = USBx_HPRT0_RAW(hhcd);

    if (hhcd->Init.phy_itface != USB_OTG_EMBEDDED_PHY)
        return;
    if (((hprt0 & USB_OTG_HPRT_PCSTS) != 0) &&
        ((hprt0 & USB_OTG_HPRT_PSPD) == (HPRT0_PRTSPD_LOW_SPEED << 17))) {
        (void) USB_InitFSLSPClkSel(hhcd->Instance, HCFG_6_MHZ);
    }
#else
    (void) hhcd;
#endif
}

USBH_StatusTypeDef
USBH_LL_ResetPort(USBH_HandleTypeDef *phost)
{
    HAL_StatusTypeDef hal_status = HAL_OK;

    /* A port reset takes down everything on the port */
    HAL_HCD_HaltAllChannels(phost->pData);
    usb_preset_phy_clock(phost->pData);
#ifdef USB_TRACE
    utr_add(phost->pData, UTR_RESET, 0, 1, 0);
#endif
    hal_status = HAL_HCD_ResetPort(phost->pData);
#ifdef USB_TRACE
    utr_add(phost->pData, UTR_RESET, 0, 0, 0);
#endif

    return (USBH_Get_USB_Status(hal_status));
}

void
USBH_Delay(uint32_t Delay)
{
    timer_delay_msec(Delay);
}

/*
 * Use the following call to send data to the HID device. Can be used
 * for setting LEDs, etc. Need to research the data format.
 *    USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, report, len);
 *
 * There is also this call:
 *    USBH_HID_SetReport()
 */

USBH_StatusTypeDef
USBH_register_class(USBH_HandleTypeDef *handle, USBH_ClassTypeDef *class,
                    uint size)
{
    USBH_ClassTypeDef *nclass = USBH_malloc(size);
    memcpy(nclass, class, size);
    nclass->pData = NULL;
    return (USBH_RegisterClass(handle, nclass));
}

void
cubeusb_init_port(uint port)
{
    USBH_HandleTypeDef *handle;
    USBH_HandleTypeDef *old = usb_root[port];
    const char *pname = (port == 0) ? "FS" : "HS";

    /*
     * Free the previous root handle (port recovery path). It is only safe
     * once cubeusb_shutdown_port() has stopped the host controller, which
     * it marks by freeing Pipes.
     */
    if (old != NULL) {
        if (old->Pipes == NULL) {
            usb_root[port] = NULL;
            USBH_free(old);
        } else {
            printf("USB%u init: previous root handle still active\n", port);
        }
    }
    phost_cur[port] = NULL;
    ctl_stay_handle[port] = NULL;

    handle = alloc_port_handle(port);
    uint     host_dev = (port == 0) ? HOST_FS : HOST_HS;
    int rc;

    usb_root[port] = handle;
    handle->valid = 1;
    handle->hubdev = port;
    handle->id = port;
    vpipe_reset_port(port);
    memset(usb_addr_used[port], 0, sizeof (usb_addr_used[port]));
    usb_alloc_address(handle);              // Assigned device address
    handle->Pipes = USBH_malloc(sizeof (uint32_t) * USBH_MAX_PIPES_NBR);

    rc = USBH_Init(handle, USBH_UserProcess, host_dev);
    if (rc != USBH_OK) {
        printf("USB%u %s init fail: %d\n", port, pname, rc);
        return;
    }
    if (USBH_register_class(handle, USBH_CDC_CLASS, sizeof (*USBH_CDC_CLASS)) ||
//      USBH_register_class(handle, USBH_MSC_CLASS, sizeof (*USBH_MSC_CLASS)) ||
        USBH_register_class(handle, USBH_HID_CLASS, sizeof (*USBH_HID_CLASS)) ||
        USBH_register_class(handle, USBH_HUB_CLASS, sizeof (*USBH_HUB_CLASS)) ||
//      USBH_register_class(handle, USBH_MTP_CLASS, sizeof (*USBH_MTP_CLASS)) ||
        USBH_register_class(handle, USBH_XUSB_CLASS,
                                                sizeof (*USBH_XUSB_CLASS))) {
        printf("USB%u %s register fail\n", port, pname);
        return;
    }
    if (USBH_Start(handle) != USBH_OK) {
        printf("USB%u %s start fail\n", port, pname);
        return;
    }
}

void
cubeusb_init(void)
{
    uint port;

    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();

    /*
     * usb_root[] is zero at boot. It is not cleared here: after
     * cubeusb_shutdown() (e.g. "usb reset") it still
     * holds the shut-down root handles, which cubeusb_init_port() frees.
     * Clearing it leaked them.
     */

#undef SINGLE_PORT
#ifdef SINGLE_PORT
    cubeusb_init_port(1);
#else
    for (port = 0; port < 2; port++)
        cubeusb_init_port(port);
#endif
}

static void
cubeusb_shutdown_port(uint port)
{
    USBH_HandleTypeDef *handle = usb_root[port];
    if ((handle == NULL) || (handle->Pipes == NULL))
        return;

    /*
     * Release every device below the root (hubs and their children),
     * and the root's own class, before stopping the port. Previously the
     * old device tree was left in place, and the poll loop (phost_cur)
     * kept processing stale handles after the port was re-initialized.
     */
    USBH_remove_subdevices(handle);
    if (handle->pActiveClass != NULL) {
        handle->pActiveClass->DeInit(handle);
        handle->pActiveClass = NULL;
    }
    phost_cur[port] = NULL;
    ctl_stay_handle[port] = NULL;
    addr0_owner[port] = NULL;

    USBH_DeInit(handle);

    /*
     * XXX: if this code is called from the user input handler (interrupt),
     *      then the USB interrupt can't run until the command returns.
     */
    while (handle->ClassNumber-- > 0)
        USBH_free(handle->pClass[handle->ClassNumber]);
    USBH_free(handle->Pipes);
    handle->Pipes = NULL;
    handle->valid = 0;
}

void
cubeusb_shutdown(void)
{
    for (uint port = 0; port < 2; port++)
        cubeusb_shutdown_port(port);

// XXX: need to deinit hub ports?
}
