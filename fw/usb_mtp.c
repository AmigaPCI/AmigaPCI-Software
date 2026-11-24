/*
 * This is free and unencumbered software released into the public domain.
 * See the LICENSE file for additional details.
 *
 * Designed by Chris Hooper in 2026.
 *
 * ---------------------------------------------------------------------
 *
 * USB MTP (Media Transfer Protocol) device access.
 *
 * Transactions are started asynchronously in the MTP class driver and
 * advanced by the normal USB host polling (cubeusb_poll(), from
 * main_poll()). The functions here wait for completion by running
 * main_poll(), so the keyboard, mouse and the rest of the firmware keep
 * running during a transaction (as usb_msc.c does). They must not be
 * called from interrupt context or from inside the USB host stack.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "main.h"
#include "printf.h"
#include "timer.h"
#include "cubeusb.h"
#include "utils.h"
#include "usb_mtp.h"
#include <usbh_core.h>
#include <usbh_mtp.h>

#define USB_MTP_READY_WAIT_MS   3000   // Wait for device to become idle
#define USB_MTP_XFER_BASE_MS   90000   // Transaction timeout ...
#define USB_MTP_XFER_KB_MS        50   // ... plus this per KB of data

static MTP_HandleTypeDef *
mtp_handle(USBH_HandleTypeDef *phost)
{
    return ((MTP_HandleTypeDef *) phost->pActiveClass->pData);
}

uint
usb_mtp_count(void)
{
    uint count = 0;
    while (cubeusb_mtp_get(count) != NULL)
        count++;
    return (count);
}

int
usb_mtp_info(uint dev, usb_mtp_info_t *info)
{
    USBH_HandleTypeDef *phost = cubeusb_mtp_get(dev);
    MTP_HandleTypeDef  *mh;

    memset(info, 0, sizeof (*info));
    if (phost == NULL)
        return (USB_MTP_NODEV);
    mh = mtp_handle(phost);
    info->ready    = USBH_MTP_IsReady(phost);
    info->nstorage = info->ready ? mh->nstorage : 0;
    info->ops      = mh->ops;
    strncpy(info->manufacturer, mh->manufacturer,
            sizeof (info->manufacturer) - 1);
    strncpy(info->model, mh->model, sizeof (info->model) - 1);
    strncpy(info->path, usb_get_path(phost, NULL, -1), sizeof (info->path) - 1);
    return (USB_MTP_OK);
}

int
usb_mtp_storage(uint dev, uint idx, usb_mtp_storage_t *st)
{
    USBH_HandleTypeDef *phost = cubeusb_mtp_get(dev);
    MTP_HandleTypeDef  *mh;
    MTP_StorageTypeDef *ms;

    memset(st, 0, sizeof (*st));
    if (phost == NULL)
        return (USB_MTP_NODEV);
    if (!USBH_MTP_IsReady(phost))
        return (USB_MTP_NOT_READY);
    mh = mtp_handle(phost);
    if (idx >= mh->nstorage)
        return (USB_MTP_NODEV);
    ms = &mh->storage[idx];
    st->id        = ms->id;
    st->access    = ms->access;
    st->capacity  = ms->capacity;
    st->freespace = ms->freespace;
    strncpy(st->desc, ms->desc, sizeof (st->desc) - 1);
    strncpy(st->label, ms->label, sizeof (st->label) - 1);
    return (USB_MTP_OK);
}

/*
 * usb_mtp_run() - run one transaction, waiting for completion.
 *
 * The device handle may be freed by main_poll() if the device is
 * unplugged, so after every poll the MTP de-init counter is checked
 * before the handle is used again.
 */
static int
usb_mtp_run(uint dev, uint16_t code, uint nparam, const uint32_t *params,
            uint dir, void *buf, uint32_t buflen,
            void (*sink)(void *arg, const uint8_t *data, uint32_t len),
            void *sink_arg, usb_mtp_result_t *res)
{
    USBH_HandleTypeDef *phost = cubeusb_mtp_get(dev);
    USBH_StatusTypeDef  st;
    MTP_XactTypeDef    *x;
    uint32_t gen = USBH_MTP_DeInitCount;
    uint64_t deadline;

    memset(res, 0, sizeof (*res));
    if (phost == NULL)
        return (USB_MTP_NODEV);
    if ((nparam > 5) || ((dir != USB_MTP_DIR_NONE) && (buf == NULL)))
        return (USB_MTP_BAD_PARAM);

    /* Wait for the device to finish init, or a previous error recovery */
    deadline = timer_tick_plus_msec(USB_MTP_READY_WAIT_MS);
    for (;;) {
        if (sink != NULL)
            st = USBH_MTP_StartXactSink(phost, code, nparam, params, buf,
                                        buflen, sink, sink_arg);
        else
            st = USBH_MTP_StartXact(phost, code, nparam, params, dir, buf,
                                    buflen);
        if (st == USBH_OK)
            break;
        if (st != USBH_BUSY)
            return (USB_MTP_IO_ERROR);
        if (timer_tick_has_elapsed(deadline))
            return (USB_MTP_NOT_READY);
        main_poll();
        if (gen != USBH_MTP_DeInitCount)
            return (USB_MTP_REMOVED);
    }

    deadline = timer_tick_plus_msec(USB_MTP_XFER_BASE_MS +
                                    USB_MTP_XFER_KB_MS * (buflen / 1024));
    for (;;) {
        main_poll();
        if (gen != USBH_MTP_DeInitCount)
            return (USB_MTP_REMOVED);
        st = USBH_MTP_XactStatus(phost);
        if (st != USBH_BUSY)
            break;
        if (timer_tick_has_elapsed(deadline)) {
            x = USBH_MTP_Xact(phost);
            printf("MTP op %04x timeout: xstate=%u pos=%lu total=%lu "
                   "len=%lu xfer=%u\n", code, x->state, x->pos, x->total,
                   x->len, x->xfer_len);
            for (uint i = 0; i < x->ntrace; i++)
                printf(" %04x", x->trace[i]);
            printf("\n");
            USBH_MTP_XactAbort(phost);
            return (USB_MTP_TIMEOUT);
        }
    }
    if (st != USBH_OK)
        return (USB_MTP_IO_ERROR);
    x = USBH_MTP_Xact(phost);
    res->resp  = x->resp;
    res->nresp = x->nresp;
    memcpy(res->rparam, x->rparam, sizeof (res->rparam));
    res->len   = x->len;
    res->total = x->total;
    return (USB_MTP_OK);
}

/*
 * usb_mtp_xact() - run an MTP operation. Returns USB_MTP_OK if the
 *                  transaction completed; res->resp then has the
 *                  device's response code.
 */
int
usb_mtp_xact(uint dev, uint16_t code, uint nparam, const uint32_t *params,
             uint dir, void *buf, uint32_t buflen, usb_mtp_result_t *res)
{
    return (usb_mtp_run(dev, code, nparam, params, dir, buf, buflen,
                        NULL, NULL, res));
}

/* As usb_mtp_xact(), with IN data passed to sink() as it arrives */
int
usb_mtp_xact_sink(uint dev, uint16_t code, uint nparam,
                  const uint32_t *params, void *buf, uint32_t buflen,
                  void (*sink)(void *arg, const uint8_t *data, uint32_t len),
                  void *sink_arg, usb_mtp_result_t *res)
{
    return (usb_mtp_run(dev, code, nparam, params, USB_MTP_DIR_IN, buf,
                        buflen, sink, sink_arg, res));
}

/* Changes whenever any MTP device is removed */
uint32_t
usb_mtp_generation(void)
{
    return (USBH_MTP_DeInitCount);
}

/* Changes whenever an MTP device reports that objects or storage changed */
uint32_t
usb_mtp_changes(void)
{
    return (USBH_MTP_ChangeCount);
}

const char *
usb_mtp_strerror(int rc)
{
    switch (rc) {
        case USB_MTP_OK:        return ("OK");
        case USB_MTP_NODEV:     return ("no such device");
        case USB_MTP_NOT_READY: return ("device not ready");
        case USB_MTP_IO_ERROR:  return ("I/O error");
        case USB_MTP_TIMEOUT:   return ("timeout");
        case USB_MTP_REMOVED:   return ("device removed");
        case USB_MTP_BAD_PARAM: return ("invalid parameter");
        default:                return ("unknown error");
    }
}

static const char *
mtp_state_str(uint state)
{
    static const char * const states[] = {
        "CheckName", "OpenSession", "GetDeviceInfo", "GetStorageIDs",
        "GetStorageInfo", "Idle", "Transaction", "Recovery", "NotMTP",
        "Failed",
    };
    return ((state < (uint) ARRAY_SIZE(states)) ? states[state] : "Unknown");
}

static void
show_storage(MTP_StorageTypeDef *ms)
{
    printf("%08lx %s%s%s%s %lu MB, %lu MB free%s\n", ms->id, ms->desc,
           (ms->label[0] != '\0') ? " [" : "", ms->label,
           (ms->label[0] != '\0') ? "]" : "",
           (uint32_t) (ms->capacity >> 20), (uint32_t) (ms->freespace >> 20),
           (ms->access != PTP_AC_ReadWrite) ? " (read-only)" : "");
}

/* Detail for "usb ls" verbose */
void
usb_mtp_show(USBH_HandleTypeDef *phost)
{
    MTP_HandleTypeDef *mh = mtp_handle(phost);
    uint i;

    printf("          MTP %s %s  state=%s ops=%04x InEp=%x/%u OutEp=%x/%u "
           "IntEp=%x\n", mh->manufacturer, mh->model,
           mtp_state_str(mh->state), mh->ops, mh->DataInEp,
           mh->DataInEpSize, mh->DataOutEp, mh->DataOutEpSize,
           mh->NotificationEp);
    for (i = 0; i < mh->nstorage; i++) {
        printf("          Storage ");
        show_storage(&mh->storage[i]);
    }
}

/* Listing for "fs devs" */
void
usb_mtp_list(void)
{
    USBH_HandleTypeDef *phost;
    uint dev;
    uint i;

    for (dev = 0; (phost = cubeusb_mtp_get(dev)) != NULL; dev++) {
        MTP_HandleTypeDef *mh = mtp_handle(phost);
        printf("MTP%u: USB%-6s %s %s", dev, usb_get_path(phost, NULL, -1),
               mh->manufacturer, mh->model);
        if (!USBH_MTP_IsReady(phost)) {
            printf("  %s\n", mtp_state_str(mh->state));
            continue;
        }
        printf("%s\n", (mh->ops & MTP_OPS_EDIT) ? "" : "  (read-only)");
        for (i = 0; i < mh->nstorage; i++) {
            printf("  %u.%u: ", dev, i);
            show_storage(&mh->storage[i]);
        }
    }
}
