/**
  ******************************************************************************
  * @file    usbh_mtp.c
  * @author  MCD Application Team
  * @brief   This file is the MTP Layer Handlers for USB Host MTP class.
  *
  * @verbatim
  *
  *          ===================================================================
  *                                MTP Class  Description
  *          ===================================================================
  *           This module manages the MTP class following the
  *           "Media Transfer Protocol (MTP) specification Version 1.11 April 6th, 2011".
  *           the implmentation is compatible with the PTP model as an extension
  *           of the existing Picture Transfer Protocol defined by the ISO 15740 specification
  *
  *  @endverbatim
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                      www.st.com/SLA0044
  *
  ******************************************************************************
  *
  * AmigaPCI BEC rework (see usbh_mtp.h):
  *   - Transactions are run by a non-blocking engine (MTP_XactRun), one
  *     step per pass of the host state machine, so the other devices on
  *     the port keep being serviced. Data phases of any size are handled
  *     with a bounded bounce buffer.
  *   - Session open, device info and storage info are read at attach.
  *   - Android devices present MTP as a vendor-specific interface (class
  *     0xFF) named "MTP"; MTP_VendorClass claims those.
  *   - Bulk OUT is sent one packet per URB (the HAL's non-DMA OUT path
  *     can't refill the TX FIFO for multi-packet transfers).
  */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdlib.h>
#include "usbh_mtp.h"
#include <stdbool.h>
#include "timer.h"

#define MTP_EVENT_POLL_MS     50U     /* Event endpoint poll interval */
#define MTP_INITBUF_SIZE      1536U   /* DeviceInfo / StorageInfo buffer */
#define MTP_HDR_LEN           12U     /* Container header length */

volatile uint32_t USBH_MTP_DeInitCount;
volatile uint32_t USBH_MTP_ChangeCount;

static USBH_StatusTypeDef USBH_MTP_Probe(USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_MTP_VendorProbe(USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_MTP_InterfaceInit(USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_MTP_InterfaceDeInit(USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_MTP_Process(USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_MTP_ClassRequest(USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_MTP_SOFProcess(USBH_HandleTypeDef *phost);

USBH_ClassTypeDef  MTP_Class =
{
  "MTP",
  USB_MTP_CLASS,
  USBH_MTP_Probe,
  USBH_MTP_InterfaceInit,
  USBH_MTP_InterfaceDeInit,
  USBH_MTP_ClassRequest,
  USBH_MTP_Process,
  USBH_MTP_SOFProcess,
  NULL,
};

USBH_ClassTypeDef  MTP_VendorClass =
{
  "MTP",
  USB_MTP_VENDOR_CLASS,
  USBH_MTP_VendorProbe,
  USBH_MTP_InterfaceInit,
  USBH_MTP_InterfaceDeInit,
  USBH_MTP_ClassRequest,
  USBH_MTP_Process,
  USBH_MTP_SOFProcess,
  NULL,
};

static MTP_HandleTypeDef *
mtp_handle(USBH_HandleTypeDef *phost)
{
  return (MTP_HandleTypeDef *) phost->pActiveClass->pData;
}

/* ------------------------------------------------------------------------ */
/* Little-endian dataset access, bounded by the dataset length              */

static uint32_t get_le(const uint8_t *buf, uint32_t len, uint32_t *off,
                       uint32_t size)
{
  uint32_t val = 0U;
  uint32_t i;

  if (*off + size > len)
  {
    *off = len + 1U;  /* Mark as overrun */
    return 0U;
  }
  for (i = 0U; i < size; i++)
  {
    val |= (uint32_t) buf[*off + i] << (8U * i);
  }
  *off += size;
  return val;
}

static uint64_t get_le64(const uint8_t *buf, uint32_t len, uint32_t *off)
{
  uint64_t lo = get_le(buf, len, off, 4U);
  uint64_t hi = get_le(buf, len, off, 4U);
  return lo | (hi << 32);
}

/*
 * MTP_GetString
 *     Decode a PTP string (count byte, then UTF-16LE characters including
 *     the NUL) at *off into UTF-8 at dst (NUL-terminated, truncated to
 *     dstlen). dst may be NULL to skip the string.
 */
uint32_t MTP_GetString(const uint8_t *buf, uint32_t len, uint32_t *off,
                       char *dst, uint32_t dstlen)
{
  uint32_t nchars = get_le(buf, len, off, 1U);
  uint32_t pos = 0U;
  uint32_t i;

  for (i = 0U; i < nchars; i++)
  {
    uint32_t ch = get_le(buf, len, off, 2U);
    if (*off > len)
    {
      break;
    }
    if ((ch >= 0xD800U) && (ch < 0xDC00U))
    {
      /* High surrogate: combine with the low surrogate which follows */
      uint32_t lo = get_le(buf, len, off, 2U);
      i++;
      ch = 0x10000U + ((ch - 0xD800U) << 10) + (lo & 0x3FFU);
    }
    if ((ch == 0U) || (dst == NULL))
    {
      continue;
    }
    if (ch < 0x80U)
    {
      if (pos + 1U < dstlen)
      {
        dst[pos++] = (char) ch;
      }
    }
    else if (ch < 0x800U)
    {
      if (pos + 2U < dstlen)
      {
        dst[pos++] = (char) (0xC0U | (ch >> 6));
        dst[pos++] = (char) (0x80U | (ch & 0x3FU));
      }
    }
    else if (ch < 0x10000U)
    {
      if (pos + 3U < dstlen)
      {
        dst[pos++] = (char) (0xE0U | (ch >> 12));
        dst[pos++] = (char) (0x80U | ((ch >> 6) & 0x3FU));
        dst[pos++] = (char) (0x80U | (ch & 0x3FU));
      }
    }
    else
    {
      if (pos + 4U < dstlen)
      {
        dst[pos++] = (char) (0xF0U | (ch >> 18));
        dst[pos++] = (char) (0x80U | ((ch >> 12) & 0x3FU));
        dst[pos++] = (char) (0x80U | ((ch >> 6) & 0x3FU));
        dst[pos++] = (char) (0x80U | (ch & 0x3FU));
      }
    }
  }
  if ((dst != NULL) && (dstlen > 0U))
  {
    dst[pos] = '\0';
  }
  return pos;
}

/* Skip a PTP array (32-bit count, then count elements of size bytes) */
static uint32_t skip_array(const uint8_t *buf, uint32_t len, uint32_t *off,
                           uint32_t size)
{
  uint32_t count = get_le(buf, len, off, 4U);
  if (*off <= len)
  {
    if ((uint64_t) count * size > (uint64_t) (len - *off))
    {
      *off = len + 1U;
    }
    else
    {
      *off += count * size;
    }
  }
  return count;
}

/* ------------------------------------------------------------------------ */
/* Interface matching                                                       */

static int8_t MTP_FindEp(USBH_HandleTypeDef *phost, uint8_t itf,
                         uint8_t in, uint8_t type)
{
  USBH_InterfaceDescTypeDef *pif = &phost->device.CfgDesc.Itf_Desc[itf];
  uint8_t ep;

  for (ep = 0U; (ep < pif->bNumEndpoints) && (ep < USBH_MAX_NUM_ENDPOINTS); ep++)
  {
    USBH_EpDescTypeDef *ed = &pif->Ep_Desc[ep];
    if ((((ed->bEndpointAddress & 0x80U) != 0U) == (in != 0U)) &&
        ((ed->bmAttributes & 0x03U) == type) && (ed->wMaxPacketSize > 0U))
    {
      return (int8_t) ep;
    }
  }
  return -1;
}

/* Does interface itf have the MTP endpoints (bulk in + out, interrupt in)? */
static uint8_t MTP_HasEndpoints(USBH_HandleTypeDef *phost, uint8_t itf)
{
  return (MTP_FindEp(phost, itf, 1U, USBH_EP_BULK) >= 0) &&
         (MTP_FindEp(phost, itf, 0U, USBH_EP_BULK) >= 0) &&
         (MTP_FindEp(phost, itf, 1U, USBH_EP_INTERRUPT) >= 0);
}

static USBH_StatusTypeDef USBH_MTP_Probe(USBH_HandleTypeDef *phost)
{
  USBH_InterfaceDescTypeDef *pif = &phost->device.CfgDesc.Itf_Desc[0];

  if ((pif->bInterfaceClass == USB_MTP_CLASS) && MTP_HasEndpoints(phost, 0U))
  {
    return USBH_OK;
  }
  return USBH_FAIL;
}

/*
 * Android MTP: vendor class, subclass 0xFF, protocol 0, three endpoints.
 * Other vendor devices may look the same (some USB Ethernet adapters),
 * so the interface name is also checked ("MTP") before anything is sent
 * on the bulk pipes.
 */
static USBH_StatusTypeDef USBH_MTP_VendorProbe(USBH_HandleTypeDef *phost)
{
  USBH_InterfaceDescTypeDef *pif = &phost->device.CfgDesc.Itf_Desc[0];

  if ((pif->bInterfaceClass == USB_MTP_VENDOR_CLASS) &&
      (pif->bInterfaceSubClass == 0xFFU) &&
      (pif->bInterfaceProtocol == 0x00U) &&
      (pif->iInterface != 0U) &&
      MTP_HasEndpoints(phost, 0U))
  {
    return USBH_OK;
  }
  return USBH_FAIL;
}

uint8_t USBH_MTP_IsMTP(USBH_HandleTypeDef *phost)
{
  return (phost->pActiveClass != NULL) &&
         (phost->pActiveClass->BgndProcess == USBH_MTP_Process) &&
         (phost->pActiveClass->pData != NULL);
}

uint8_t USBH_MTP_IsReady(USBH_HandleTypeDef *phost)
{
  if (!USBH_MTP_IsMTP(phost) || (phost->gState != HOST_CLASS))
  {
    return 0U;
  }
  return mtp_handle(phost)->is_ready;
}

/* ------------------------------------------------------------------------ */
/* Init / DeInit                                                            */

static USBH_StatusTypeDef USBH_MTP_InterfaceInit(USBH_HandleTypeDef *phost)
{
  USBH_InterfaceDescTypeDef *pif;
  MTP_HandleTypeDef *MTP_Handle;
  int8_t ep_in, ep_out, ep_int;
  const uint8_t itf = 0U;

  pif = &phost->device.CfgDesc.Itf_Desc[itf];
  ep_in  = MTP_FindEp(phost, itf, 1U, USBH_EP_BULK);
  ep_out = MTP_FindEp(phost, itf, 0U, USBH_EP_BULK);
  ep_int = MTP_FindEp(phost, itf, 1U, USBH_EP_INTERRUPT);
  if ((ep_in < 0) || (ep_out < 0) || (ep_int < 0))
  {
    USBH_UsrLog("USB%s MTP: missing endpoints", usb_get_path(phost, NULL, -1));
    return USBH_FAIL;
  }
  if (USBH_SelectInterface(phost, itf) != USBH_OK)
  {
    return USBH_FAIL;
  }

  MTP_Handle = (MTP_HandleTypeDef *) USBH_malloc(sizeof (MTP_HandleTypeDef));
  phost->pActiveClass->pData = MTP_Handle;
  if (MTP_Handle == NULL)
  {
    USBH_UsrLog("USB%s MTP: no memory", usb_get_path(phost, NULL, -1));
    return USBH_FAIL;
  }
  USBH_memset(MTP_Handle, 0, sizeof (MTP_HandleTypeDef));
  MTP_Handle->interface  = pif->bInterfaceNumber;
  MTP_Handle->iInterface = pif->iInterface;

  MTP_Handle->NotificationEp     = pif->Ep_Desc[ep_int].bEndpointAddress;
  MTP_Handle->NotificationEpSize = pif->Ep_Desc[ep_int].wMaxPacketSize;
  if (MTP_Handle->NotificationEpSize > sizeof (MTP_Handle->evbuf))
  {
    MTP_Handle->NotificationEpSize = sizeof (MTP_Handle->evbuf);
  }
  MTP_Handle->DataInEp      = pif->Ep_Desc[ep_in].bEndpointAddress;
  MTP_Handle->DataInEpSize  = pif->Ep_Desc[ep_in].wMaxPacketSize & 0x7FFU;
  MTP_Handle->DataOutEp     = pif->Ep_Desc[ep_out].bEndpointAddress;
  MTP_Handle->DataOutEpSize = pif->Ep_Desc[ep_out].wMaxPacketSize & 0x7FFU;
  if ((MTP_Handle->DataInEpSize > MTP_XBUF_SIZE) ||
      (MTP_Handle->DataOutEpSize > MTP_XBUF_SIZE) ||
      (MTP_Handle->DataOutEpSize <= MTP_HDR_LEN))
  {
    USBH_UsrLog("USB%s MTP: unsupported packet size",
                usb_get_path(phost, NULL, -1));
    return USBH_FAIL;
  }

  MTP_Handle->NotificationPipe = USBH_AllocPipe(phost, MTP_Handle->NotificationEp);
  MTP_Handle->DataInPipe       = USBH_AllocPipe(phost, MTP_Handle->DataInEp);
  MTP_Handle->DataOutPipe      = USBH_AllocPipe(phost, MTP_Handle->DataOutEp);

  USBH_OpenPipe(phost, MTP_Handle->NotificationPipe, MTP_Handle->NotificationEp,
                phost->device.address, phost->device.speed, USB_EP_TYPE_INTR,
                MTP_Handle->NotificationEpSize);
  USBH_OpenPipe(phost, MTP_Handle->DataInPipe, MTP_Handle->DataInEp,
                phost->device.address, phost->device.speed, USB_EP_TYPE_BULK,
                MTP_Handle->DataInEpSize);
  USBH_OpenPipe(phost, MTP_Handle->DataOutPipe, MTP_Handle->DataOutEp,
                phost->device.address, phost->device.speed, USB_EP_TYPE_BULK,
                MTP_Handle->DataOutEpSize);
  USBH_LL_SetToggle(phost, MTP_Handle->NotificationPipe, 0U);
  USBH_LL_SetToggle(phost, MTP_Handle->DataInPipe, 0U);
  USBH_LL_SetToggle(phost, MTP_Handle->DataOutPipe, 0U);

  MTP_Handle->state = (pif->bInterfaceClass == USB_MTP_VENDOR_CLASS) ?
                      MTP_CHECK_NAME : MTP_OPENSESSION;
  MTP_Handle->x.state  = MTP_XS_DONE;
  MTP_Handle->x.status = USBH_OK;
  MTP_Handle->ev_time  = timer_tick_get();
  return USBH_OK;
}

static USBH_StatusTypeDef USBH_MTP_InterfaceDeInit(USBH_HandleTypeDef *phost)
{
  MTP_HandleTypeDef *MTP_Handle = mtp_handle(phost);

  /* Tell anyone waiting on a transaction that the device left */
  USBH_MTP_DeInitCount++;

  if (MTP_Handle == NULL)
  {
    return USBH_OK;
  }
  if (MTP_Handle->DataOutPipe != 0U)
  {
    USBH_ClosePipe(phost, MTP_Handle->DataOutPipe);
    USBH_FreePipe(phost, MTP_Handle->DataOutPipe);
  }
  if (MTP_Handle->DataInPipe != 0U)
  {
    USBH_ClosePipe(phost, MTP_Handle->DataInPipe);
    USBH_FreePipe(phost, MTP_Handle->DataInPipe);
  }
  if (MTP_Handle->NotificationPipe != 0U)
  {
    USBH_ClosePipe(phost, MTP_Handle->NotificationPipe);
    USBH_FreePipe(phost, MTP_Handle->NotificationPipe);
  }
  free(MTP_Handle->initbuf);
  USBH_free(MTP_Handle);
  phost->pActiveClass->pData = NULL;
  return USBH_OK;
}

static USBH_StatusTypeDef USBH_MTP_ClassRequest(USBH_HandleTypeDef *phost)
{
  UNUSED(phost);
  return USBH_OK;
}

static USBH_StatusTypeDef USBH_MTP_SOFProcess(USBH_HandleTypeDef *phost)
{
  UNUSED(phost);
  return USBH_OK;
}

/* ------------------------------------------------------------------------ */
/* Transaction engine                                                       */

static void MTP_XactBegin(MTP_HandleTypeDef *MTP_Handle, uint16_t code,
                          uint8_t nparam, const uint32_t *params, uint8_t dir,
                          uint8_t *buf, uint32_t buflen)
{
  MTP_XactTypeDef *x = &MTP_Handle->x;

  USBH_memset(x, 0, sizeof (*x));
  x->code   = code;
  x->nparam = (nparam > 5U) ? 5U : nparam;
  if (params != NULL)
  {
    USBH_memcpy(x->param, params, x->nparam * sizeof (uint32_t));
  }
  x->dir    = dir;
  x->buf    = buf;
  x->buflen = (buf == NULL) ? 0U : buflen;
  x->first  = 1U;
  x->state  = MTP_XS_CMD;
  x->status = USBH_BUSY;
}

static void put_le32(uint8_t *p, uint32_t val)
{
  p[0] = (uint8_t) val;
  p[1] = (uint8_t) (val >> 8);
  p[2] = (uint8_t) (val >> 16);
  p[3] = (uint8_t) (val >> 24);
}

/* Write a container header at p */
static void put_hdr(uint8_t *p, uint32_t len, uint16_t type, uint16_t code,
                    uint32_t tid)
{
  put_le32(p, len);
  p[4] = (uint8_t) type;
  p[5] = (uint8_t) (type >> 8);
  p[6] = (uint8_t) code;
  p[7] = (uint8_t) (code >> 8);
  put_le32(p + 8, tid);
}

/* Parse a response container at p (n bytes) into the transaction */
static void MTP_XactResponse(MTP_XactTypeDef *x, const uint8_t *p, uint32_t n)
{
  uint32_t off = 0U;
  uint32_t clen = get_le(p, n, &off, 4U);
  uint32_t i;

  (void) get_le(p, n, &off, 2U);  /* Type */
  x->resp = (uint16_t) get_le(p, n, &off, 2U);
  (void) get_le(p, n, &off, 4U);  /* Transaction ID */
  if (clen > n)
  {
    clen = n;
  }
  x->nresp = (clen > MTP_HDR_LEN) ? (uint8_t) ((clen - MTP_HDR_LEN) / 4U) : 0U;
  if (x->nresp > 5U)
  {
    x->nresp = 5U;
  }
  for (i = 0U; i < x->nresp; i++)
  {
    x->rparam[i] = get_le(p, n, &off, 4U);
  }
  x->status = USBH_OK;
  x->state  = MTP_XS_DONE;
}

static void MTP_XactFail(USBH_HandleTypeDef *phost, uint8_t stalled)
{
  MTP_HandleTypeDef *MTP_Handle = mtp_handle(phost);

  MTP_Handle->x.status = USBH_FAIL;
  MTP_Handle->x.resp   = 0U;
  MTP_Handle->x.state  = MTP_XS_DONE;
  if (stalled)
  {
    MTP_Handle->rr_step = 0U;
    MTP_Handle->state   = MTP_RECOVERY;
  }
}

/*
 * MTP_XactRun
 *     Advance the current transaction one step. Returns its status
 *     (USBH_BUSY while it is still running).
 */
static USBH_StatusTypeDef MTP_XactRun(USBH_HandleTypeDef *phost)
{
  MTP_HandleTypeDef *MTP_Handle = mtp_handle(phost);
  MTP_XactTypeDef   *x = &MTP_Handle->x;
  uint8_t           *xbuf = (uint8_t *) MTP_Handle->xbuf;
  uint16_t           mps_in  = MTP_Handle->DataInEpSize;
  uint16_t           mps_out = MTP_Handle->DataOutEpSize;
  USBH_URBStateTypeDef urb;
  uint32_t           n;
  uint32_t           i;

  switch (x->state)
  {
    case MTP_XS_CMD:
      if (x->first)
      {
        /* OpenSession is transaction 0; the session's first is then 1 */
        if (x->code == PTP_OC_OpenSession)
        {
          MTP_Handle->transaction_id = 0U;
        }
        else
        {
          MTP_Handle->transaction_id++;
        }
      }
      n = MTP_HDR_LEN + 4U * x->nparam;
      put_hdr(xbuf, n, PTP_USB_CONTAINER_COMMAND, x->code,
              MTP_Handle->transaction_id);
      for (i = 0U; i < x->nparam; i++)
      {
        put_le32(xbuf + MTP_HDR_LEN + 4U * i, x->param[i]);
      }
      (void) USBH_BulkSendData(phost, xbuf, (uint16_t) n,
                               MTP_Handle->DataOutPipe, 1U);
      x->state = MTP_XS_CMD_WAIT;
      break;

    case MTP_XS_CMD_WAIT:
      urb = USBH_LL_GetURBState(phost, MTP_Handle->DataOutPipe);
      if (urb == USBH_URB_DONE)
      {
        x->first = 1U;
        if (x->dir == MTP_DATA_OUT)
        {
          x->state = MTP_XS_DOUT;
        }
        else if (x->dir == MTP_DATA_IN)
        {
          x->state = MTP_XS_DIN;
        }
        else
        {
          x->state = MTP_XS_RESP;
        }
      }
      else if (urb == USBH_URB_NOTREADY)
      {
        x->first = 0U;
        x->state = MTP_XS_CMD;   /* Resend */
      }
      else if ((urb == USBH_URB_STALL) || (urb == USBH_URB_ERROR))
      {
        MTP_XactFail(phost, 1U);
      }
      break;

    case MTP_XS_DOUT:
      /*
       * One packet per URB. The first packet carries the container
       * header and the start of the data; a zero-length packet ends a
       * data phase which is a multiple of the packet size.
       */
      if (x->first == 1U)
      {
        n = x->buflen;
        if (n > (uint32_t) mps_out - MTP_HDR_LEN)
        {
          n = (uint32_t) mps_out - MTP_HDR_LEN;
        }
        put_hdr(xbuf, MTP_HDR_LEN + x->buflen, PTP_USB_CONTAINER_DATA,
                x->code, MTP_Handle->transaction_id);
        if (n > 0U)
        {
          USBH_memcpy(xbuf + MTP_HDR_LEN, x->buf, n);
        }
        x->xfer_len = (uint16_t) n;
        (void) USBH_BulkSendData(phost, xbuf, (uint16_t) (MTP_HDR_LEN + n),
                                 MTP_Handle->DataOutPipe, 1U);
      }
      else if (x->pos < x->buflen)
      {
        n = x->buflen - x->pos;
        if (n > mps_out)
        {
          n = mps_out;
        }
        x->xfer_len = (uint16_t) n;
        (void) USBH_BulkSendData(phost, x->buf + x->pos, (uint16_t) n,
                                 MTP_Handle->DataOutPipe, 1U);
      }
      else
      {
        x->first = 2U;   /* Zero-length packet */
        x->xfer_len = 0U;
        (void) USBH_BulkSendData(phost, xbuf, 0U, MTP_Handle->DataOutPipe, 1U);
      }
      x->state = MTP_XS_DOUT_WAIT;
      break;

    case MTP_XS_DOUT_WAIT:
      urb = USBH_LL_GetURBState(phost, MTP_Handle->DataOutPipe);
      if (urb == USBH_URB_DONE)
      {
        if (x->first == 2U)
        {
          x->state = MTP_XS_RESP;
          break;
        }
        x->pos  += x->xfer_len;
        x->first = 0U;
        if (x->pos < x->buflen)
        {
          x->state = MTP_XS_DOUT;
        }
        else if (((MTP_HDR_LEN + x->buflen) % mps_out) == 0U)
        {
          /* MTP_XS_DOUT sends a ZLP when pos == buflen and first != 1 */
          x->state = MTP_XS_DOUT;
        }
        else
        {
          x->state = MTP_XS_RESP;
        }
      }
      else if (urb == USBH_URB_NOTREADY)
      {
        x->state = MTP_XS_DOUT;   /* Resend the same packet */
      }
      else if ((urb == USBH_URB_STALL) || (urb == USBH_URB_ERROR))
      {
        MTP_XactFail(phost, 1U);
      }
      break;

    case MTP_XS_DIN:
    {
      /*
       * Never ask for more than the rest of the data container: some
       * devices (Samsung) don't send the zero-length packet which should
       * end a data phase that is a multiple of the packet size, and send
       * the response container right after, which a longer URB would
       * swallow. The first URB is one packet (it holds the header, which
       * gives the length).
       */
      uint32_t space = x->buflen - x->len;
      uint32_t rem;
      uint32_t max = (uint32_t) mps_in * 256U;
      if (max > 0xFFFFU)
      {
        max = (0xFFFFU / mps_in) * mps_in;
      }
      if (x->first)
      {
        rem = mps_in;
      }
      else
      {
        rem = x->total - x->pos;
        rem = ((rem + mps_in - 1U) / mps_in) * mps_in;  /* Whole packets */
      }
      if (rem > max)
      {
        rem = max;
      }
      if (!x->first && (space >= mps_in))
      {
        /* Directly into the caller's buffer */
        n = (space > rem) ? rem : space;
        n = (n / mps_in) * mps_in;
        x->xfer_direct = 1U;
        x->xfer_len = (uint16_t) n;
        (void) USBH_BulkReceiveData(phost, x->buf + x->len, (uint16_t) n,
                                    MTP_Handle->DataInPipe);
      }
      else
      {
        /* First packet (container header), or the buffer is full */
        n = (MTP_XBUF_SIZE / mps_in) * mps_in;
        if (n > rem)
        {
          n = rem;
        }
        x->xfer_direct = 0U;
        x->xfer_len = (uint16_t) n;
        (void) USBH_BulkReceiveData(phost, xbuf, (uint16_t) n,
                                    MTP_Handle->DataInPipe);
      }
      x->state = MTP_XS_DIN_WAIT;
      break;
    }

    case MTP_XS_DIN_WAIT:
      urb = USBH_LL_GetURBState(phost, MTP_Handle->DataInPipe);
      if (urb == USBH_URB_DONE)
      {
        if (x->ntrace < 8U)
        {
          x->trace[x->ntrace++] = (uint16_t) ((x->state << 12) |
              (USBH_LL_GetLastXferSize(phost, MTP_Handle->DataInPipe) & 0xFFFU));
        }
        const uint8_t *src = xbuf;
        n = USBH_LL_GetLastXferSize(phost, MTP_Handle->DataInPipe);
        if (n > x->xfer_len)
        {
          n = x->xfer_len;
        }
        if (x->first)
        {
          uint32_t off = 0U;
          uint32_t clen;
          uint16_t type;
          if (n == 0U)
          {
            x->state = MTP_XS_DIN;   /* Stray zero-length packet */
            break;
          }
          if (n < MTP_HDR_LEN)
          {
            MTP_XactFail(phost, 1U);
            break;
          }
          clen = get_le(xbuf, n, &off, 4U);
          type = (uint16_t) get_le(xbuf, n, &off, 2U);
          if (type == PTP_USB_CONTAINER_RESPONSE)
          {
            /* The device skipped the data phase (operation failed) */
            MTP_XactResponse(x, xbuf, n);
            break;
          }
          if ((type != PTP_USB_CONTAINER_DATA) || (clen < MTP_HDR_LEN))
          {
            MTP_XactFail(phost, 1U);
            break;
          }
          x->total = clen - MTP_HDR_LEN;
          x->first = 0U;
          src = xbuf + MTP_HDR_LEN;
          n  -= MTP_HDR_LEN;
          x->pos += n;
          i = x->buflen - x->len;
          if (i > n)
          {
            i = n;
          }
          if (i > 0U)
          {
            USBH_memcpy(x->buf + x->len, src, i);
            x->len += i;
          }
          n += MTP_HDR_LEN;   /* For the short packet test below */
        }
        else if (x->xfer_direct)
        {
          x->len += n;
          x->pos += n;
        }
        else
        {
          i = x->buflen - x->len;
          if (i > n)
          {
            i = n;
          }
          if (i > 0U)
          {
            USBH_memcpy(x->buf + x->len, xbuf, i);
            x->len += i;
          }
          x->pos += n;
        }
        if ((x->sink != NULL) && (x->len > 0U))
        {
          x->sink(x->sink_arg, x->buf, x->len);
          x->len = 0U;
        }
        if (x->pos >= x->total)
        {
          /*
           * All data: the response is next. (A short packet before that
           * isn't the end: some devices send the header on its own.)
           */
          x->state = MTP_XS_RESP;
        }
        else
        {
          x->state = MTP_XS_DIN;
        }
      }
      else if ((urb == USBH_URB_STALL) || (urb == USBH_URB_ERROR))
      {
        MTP_XactFail(phost, 1U);
      }
      break;

    case MTP_XS_RESP:
      x->xfer_len = mps_in;
      (void) USBH_BulkReceiveData(phost, xbuf, mps_in, MTP_Handle->DataInPipe);
      x->state = MTP_XS_RESP_WAIT;
      break;

    case MTP_XS_RESP_WAIT:
      urb = USBH_LL_GetURBState(phost, MTP_Handle->DataInPipe);
      if (urb == USBH_URB_DONE)
      {
        if (x->ntrace < 8U)
        {
          x->trace[x->ntrace++] = (uint16_t) ((x->state << 12) |
              (USBH_LL_GetLastXferSize(phost, MTP_Handle->DataInPipe) & 0xFFFU));
        }
        uint32_t off = 4U;
        n = USBH_LL_GetLastXferSize(phost, MTP_Handle->DataInPipe);
        if (n == 0U)
        {
          /* Zero-length packet ending the data phase */
          x->state = MTP_XS_RESP;
          break;
        }
        if ((n < MTP_HDR_LEN) ||
            (get_le(xbuf, n, &off, 2U) != PTP_USB_CONTAINER_RESPONSE))
        {
          MTP_XactFail(phost, 1U);
          break;
        }
        MTP_XactResponse(x, xbuf, n);
      }
      else if ((urb == USBH_URB_STALL) || (urb == USBH_URB_ERROR))
      {
        MTP_XactFail(phost, 1U);
      }
      break;

    case MTP_XS_DONE:
    default:
      break;
  }
  return x->status;
}

/* ------------------------------------------------------------------------ */
/* Events                                                                   */

__weak void USBH_MTP_EventsCallback(USBH_HandleTypeDef *phost,
                                    uint32_t event, uint32_t param)
{
  UNUSED(phost);
  UNUSED(event);
  UNUSED(param);
}

static void MTP_Events(USBH_HandleTypeDef *phost)
{
  MTP_HandleTypeDef *MTP_Handle = mtp_handle(phost);
  USBH_URBStateTypeDef urb;

  if (!MTP_Handle->ev_active)
  {
    if (!timer_tick_has_elapsed(MTP_Handle->ev_time))
    {
      return;
    }
    (void) USBH_InterruptReceiveData(phost, (uint8_t *) MTP_Handle->evbuf,
                                     (uint8_t) MTP_Handle->NotificationEpSize,
                                     MTP_Handle->NotificationPipe);
    MTP_Handle->ev_active = 1U;
    return;
  }

  urb = USBH_LL_GetURBState(phost, MTP_Handle->NotificationPipe);
  if (urb == USBH_URB_DONE)
  {
    const uint8_t *p = (const uint8_t *) MTP_Handle->evbuf;
    uint32_t n = USBH_LL_GetLastXferSize(phost, MTP_Handle->NotificationPipe);
    MTP_Handle->ev_active = 0U;
    MTP_Handle->ev_time = timer_tick_get();   /* Poll again right away */
    if (n >= MTP_HDR_LEN)
    {
      uint32_t off = 6U;
      uint32_t code  = get_le(p, n, &off, 2U);
      uint32_t param = 0U;
      off = 12U;
      if (n >= 16U)
      {
        param = get_le(p, n, &off, 4U);
      }
      switch (code)
      {
        case PTP_EC_StoreAdded:
        case PTP_EC_StoreRemoved:
          MTP_Handle->rescan = 1U;
          USBH_MTP_ChangeCount++;
          break;
        case PTP_EC_ObjectAdded:
        case PTP_EC_ObjectRemoved:
        case PTP_EC_ObjectInfoChanged:
        case PTP_EC_StorageInfoChanged:
        case PTP_EC_DeviceReset:
          USBH_MTP_ChangeCount++;
          break;
        default:
          break;
      }
      USBH_MTP_EventsCallback(phost, code, param);
    }
  }
  else if ((urb == USBH_URB_NOTREADY) || (urb == USBH_URB_ERROR) ||
           (urb == USBH_URB_STALL))
  {
    /* NAK (no event pending) or error: the channel has halted */
    MTP_Handle->ev_active = 0U;
    MTP_Handle->ev_time = timer_tick_plus_msec(MTP_EVENT_POLL_MS);
  }
}

/* ------------------------------------------------------------------------ */
/* Dataset decoding                                                         */

static void MTP_DecodeDeviceInfo(MTP_HandleTypeDef *MTP_Handle,
                                 const uint8_t *buf, uint32_t len)
{
  uint32_t off = 8U;   /* StandardVersion, VendorExtensionID / Version */
  uint32_t count;
  uint32_t i;
  uint16_t ops = 0U;

  (void) MTP_GetString(buf, len, &off, NULL, 0U);   /* VendorExtensionDesc */
  off += 2U;                                         /* FunctionalMode */
  count = get_le(buf, len, &off, 4U);                /* OperationsSupported */
  for (i = 0U; (i < count) && (off < len); i++)
  {
    switch (get_le(buf, len, &off, 2U))
    {
      case PTP_OC_GetPartialObject:
        ops |= MTP_OPS_PARTIAL;
        break;
      case MTP_OC_ANDROID_GetPartialObject64:
        ops |= MTP_OPS_PARTIAL64;
        break;
      case MTP_OC_ANDROID_BeginEditObject:
        ops |= MTP_OPS_EDIT;
        break;
      case PTP_OC_SendObjectInfo:
        ops |= MTP_OPS_SENDINFO;
        break;
      case PTP_OC_DeleteObject:
        ops |= MTP_OPS_DELETE;
        break;
      case PTP_OC_MoveObject:
        ops |= MTP_OPS_MOVE;
        break;
      case PTP_OC_SetObjectPropValue:
        ops |= MTP_OPS_SETPROP;
        break;
      case PTP_OC_GetObjectPropValue:
        ops |= MTP_OPS_GETPROP;
        break;
      case PTP_OC_GetObjPropList:
        ops |= MTP_OPS_PROPLIST;
        break;
      default:
        break;
    }
  }
  MTP_Handle->ops = ops;
  (void) skip_array(buf, len, &off, 2U);   /* EventsSupported */
  (void) skip_array(buf, len, &off, 2U);   /* DevicePropertiesSupported */
  (void) skip_array(buf, len, &off, 2U);   /* CaptureFormats */
  (void) skip_array(buf, len, &off, 2U);   /* PlaybackFormats */
  (void) MTP_GetString(buf, len, &off, MTP_Handle->manufacturer,
                       sizeof (MTP_Handle->manufacturer));
  (void) MTP_GetString(buf, len, &off, MTP_Handle->model,
                       sizeof (MTP_Handle->model));
}

static void MTP_DecodeStorageInfo(MTP_StorageTypeDef *st,
                                  const uint8_t *buf, uint32_t len)
{
  uint32_t off = 0U;

  st->type      = (uint16_t) get_le(buf, len, &off, 2U);
  st->fstype    = (uint16_t) get_le(buf, len, &off, 2U);
  st->access    = (uint16_t) get_le(buf, len, &off, 2U);
  st->capacity  = get_le64(buf, len, &off);
  st->freespace = get_le64(buf, len, &off);
  off += 4U;   /* FreeSpaceInObjects */
  (void) MTP_GetString(buf, len, &off, st->desc, sizeof (st->desc));
  (void) MTP_GetString(buf, len, &off, st->label, sizeof (st->label));
}

/* ------------------------------------------------------------------------ */
/* Background process                                                       */

/* Run an initialization transaction; returns its status */
static USBH_StatusTypeDef MTP_InitXact(USBH_HandleTypeDef *phost,
                                       uint16_t code, uint8_t nparam,
                                       uint32_t param, uint8_t dir)
{
  MTP_HandleTypeDef *MTP_Handle = mtp_handle(phost);

  if (MTP_Handle->x.state == MTP_XS_DONE)
  {
    if (MTP_Handle->x.status != USBH_BUSY)
    {
      /* Start it */
      if ((dir == MTP_DATA_IN) && (MTP_Handle->initbuf == NULL))
      {
        MTP_Handle->initbuf = malloc(MTP_INITBUF_SIZE);
        if (MTP_Handle->initbuf == NULL)
        {
          return USBH_FAIL;
        }
      }
      MTP_XactBegin(MTP_Handle, code, nparam, &param, dir,
                    MTP_Handle->initbuf, MTP_INITBUF_SIZE);
      MTP_Handle->x.status = USBH_BUSY;
    }
  }
  if (MTP_XactRun(phost) == USBH_BUSY)
  {
    return USBH_BUSY;
  }
  /* Done; the next MTP_InitXact() call starts a new transaction */
  if ((MTP_Handle->x.status == USBH_OK) &&
      ((MTP_Handle->x.resp == PTP_RC_OK) ||
       ((code == PTP_OC_OpenSession) &&
        (MTP_Handle->x.resp == PTP_RC_SessionAlreadyOpened))))
  {
    MTP_Handle->x.status = USBH_OK;
    return USBH_OK;
  }
  MTP_Handle->x.status = USBH_OK;
  USBH_UsrLog("USB%s MTP op %04x failed: resp %04x",
              usb_get_path(phost, NULL, -1), code, MTP_Handle->x.resp);
  return USBH_FAIL;
}

/* An init step failed: give up (the device stays attached, unused) */
static void MTP_InitFailed(USBH_HandleTypeDef *phost)
{
  MTP_HandleTypeDef *MTP_Handle = mtp_handle(phost);

  free(MTP_Handle->initbuf);
  MTP_Handle->initbuf = NULL;
  if (MTP_Handle->state == MTP_RECOVERY)
  {
    return;   /* Endpoints are cleared first; init is then retried */
  }
  MTP_Handle->state = MTP_FAILED;
}

static USBH_StatusTypeDef USBH_MTP_Process(USBH_HandleTypeDef *phost)
{
  MTP_HandleTypeDef *MTP_Handle = mtp_handle(phost);
  USBH_StatusTypeDef status;
  uint8_t           *xbuf = (uint8_t *) MTP_Handle->xbuf;

  if (MTP_Handle == NULL)
  {
    return USBH_FAIL;
  }
  if (MTP_Handle->is_ready)
  {
    MTP_Events(phost);
  }

  switch (MTP_Handle->state)
  {
    case MTP_CHECK_NAME:
      status = USBH_Get_StringDesc(phost, MTP_Handle->iInterface, xbuf, 0xFFU);
      if (status == USBH_BUSY)
      {
        break;
      }
      if ((status == USBH_OK) && (strncmp((char *) xbuf, "MTP", 3) == 0))
      {
        MTP_Handle->state = MTP_OPENSESSION;
      }
      else
      {
        MTP_Handle->state = MTP_NOT_MTP;
        USBH_UsrLog("USB%s vendor interface is not MTP",
                    usb_get_path(phost, NULL, -1));
      }
      break;

    case MTP_OPENSESSION:
      MTP_Handle->is_ready = 0U;
      MTP_Handle->session_id = 1U;  /* Session 0 is not valid */
      status = MTP_InitXact(phost, PTP_OC_OpenSession, 1U,
                            MTP_Handle->session_id, MTP_DATA_NONE);
      if (status == USBH_OK)
      {
        MTP_Handle->state = MTP_GETDEVICEINFO;
      }
      else if (status != USBH_BUSY)
      {
        MTP_InitFailed(phost);
      }
      break;

    case MTP_GETDEVICEINFO:
      status = MTP_InitXact(phost, PTP_OC_GetDeviceInfo, 0U, 0U, MTP_DATA_IN);
      if (status == USBH_OK)
      {
        MTP_DecodeDeviceInfo(MTP_Handle, MTP_Handle->initbuf,
                             MTP_Handle->x.len);
        MTP_Handle->state = MTP_GETSTORAGEIDS;
      }
      else if (status != USBH_BUSY)
      {
        MTP_InitFailed(phost);
      }
      break;

    case MTP_GETSTORAGEIDS:
      status = MTP_InitXact(phost, PTP_OC_GetStorageIDs, 0U, 0U, MTP_DATA_IN);
      if (status == USBH_OK)
      {
        const uint8_t *buf = MTP_Handle->initbuf;
        uint32_t len = MTP_Handle->x.len;
        uint32_t off = 0U;
        uint32_t count = get_le(buf, len, &off, 4U);
        uint32_t i;

        MTP_Handle->nstorage = 0U;
        for (i = 0U; (i < count) && (off < len) &&
                     (MTP_Handle->nstorage < MTP_MAX_STORAGE); i++)
        {
          uint32_t id = get_le(buf, len, &off, 4U);
          /* Low 16 bits zero: a storage slot with no media */
          if ((id & 0xFFFFU) != 0U)
          {
            MTP_Handle->storage[MTP_Handle->nstorage++].id = id;
          }
        }
        MTP_Handle->cur_storage = 0U;
        MTP_Handle->state = MTP_GETSTORAGEINFO;
      }
      else if (status != USBH_BUSY)
      {
        MTP_InitFailed(phost);
      }
      break;

    case MTP_GETSTORAGEINFO:
      if (MTP_Handle->cur_storage >= MTP_Handle->nstorage)
      {
        free(MTP_Handle->initbuf);
        MTP_Handle->initbuf = NULL;
        MTP_Handle->rescan = 0U;
        MTP_Handle->state = MTP_IDLE;
        if (!MTP_Handle->is_ready)
        {
          USBH_UsrLog("USB%s MTP %s %s: %u storage", usb_get_path(phost, NULL, -1),
                      MTP_Handle->manufacturer, MTP_Handle->model,
                      MTP_Handle->nstorage);
        }
        MTP_Handle->is_ready = 1U;
        USBH_MTP_ChangeCount++;
        phost->pUser(phost, HOST_USER_CLASS_ACTIVE);
        break;
      }
      status = MTP_InitXact(phost, PTP_OC_GetStorageInfo, 1U,
                            MTP_Handle->storage[MTP_Handle->cur_storage].id,
                            MTP_DATA_IN);
      if (status == USBH_OK)
      {
        MTP_DecodeStorageInfo(&MTP_Handle->storage[MTP_Handle->cur_storage],
                              MTP_Handle->initbuf, MTP_Handle->x.len);
        MTP_Handle->cur_storage++;
      }
      else if (status != USBH_BUSY)
      {
        MTP_InitFailed(phost);
      }
      break;

    case MTP_IDLE:
      if (MTP_Handle->rescan)
      {
        /* Storage was added or removed: re-read the storage list */
        MTP_Handle->is_ready = 0U;
        MTP_Handle->state = MTP_GETSTORAGEIDS;
      }
      break;

    case MTP_XACT:
      if (MTP_XactRun(phost) != USBH_BUSY)
      {
        if (MTP_Handle->state == MTP_XACT)
        {
          MTP_Handle->state = MTP_IDLE;
        }
        if ((MTP_Handle->x.status == USBH_OK) &&
            (MTP_Handle->x.resp == PTP_RC_SessionNotOpen))
        {
          /* The device lost the session (reset?): open it again */
          MTP_Handle->state = MTP_OPENSESSION;
          MTP_Handle->is_ready = 0U;
        }
      }
      break;

    case MTP_RECOVERY:
      /* Clear Feature HALT on both bulk endpoints */
      if (MTP_Handle->rr_step == 0U)
      {
        status = USBH_ClrFeature(phost, MTP_Handle->DataInEp);
      }
      else
      {
        status = USBH_ClrFeature(phost, MTP_Handle->DataOutEp);
      }
      if (status == USBH_BUSY)
      {
        break;
      }
      if (++MTP_Handle->rr_step < 2U)
      {
        break;
      }
      USBH_LL_SetToggle(phost, MTP_Handle->DataInPipe, 0U);
      USBH_LL_SetToggle(phost, MTP_Handle->DataOutPipe, 0U);
      MTP_Handle->rr_step = 0U;
      MTP_Handle->x.state = MTP_XS_DONE;
      MTP_Handle->state = MTP_Handle->is_ready ? MTP_IDLE : MTP_OPENSESSION;
      break;

    case MTP_NOT_MTP:
    case MTP_FAILED:
    default:
      break;
  }
  return USBH_OK;
}

/* ------------------------------------------------------------------------ */
/* User API                                                                 */

USBH_StatusTypeDef USBH_MTP_StartXact(USBH_HandleTypeDef *phost,
                                      uint16_t code, uint8_t nparam,
                                      const uint32_t *params, uint8_t dir,
                                      uint8_t *buf, uint32_t buflen)
{
  MTP_HandleTypeDef *MTP_Handle;

  if (!USBH_MTP_IsMTP(phost) || (phost->device.is_connected == 0U))
  {
    return USBH_FAIL;
  }
  MTP_Handle = mtp_handle(phost);
  if ((MTP_Handle->state == MTP_FAILED) || (MTP_Handle->state == MTP_NOT_MTP))
  {
    return USBH_FAIL;
  }
  if ((phost->gState != HOST_CLASS) || (MTP_Handle->state != MTP_IDLE) ||
      !MTP_Handle->is_ready)
  {
    return USBH_BUSY;
  }
  MTP_XactBegin(MTP_Handle, code, nparam, params, dir, buf, buflen);
  MTP_Handle->state = MTP_XACT;
  return USBH_OK;
}

USBH_StatusTypeDef USBH_MTP_StartXactSink(USBH_HandleTypeDef *phost,
                                          uint16_t code, uint8_t nparam,
                                          const uint32_t *params,
                                          uint8_t *buf, uint32_t buflen,
                                          void (*sink)(void *arg,
                                                       const uint8_t *data,
                                                       uint32_t len),
                                          void *sink_arg)
{
  USBH_StatusTypeDef status;

  status = USBH_MTP_StartXact(phost, code, nparam, params, MTP_DATA_IN,
                              buf, buflen);
  if (status == USBH_OK)
  {
    mtp_handle(phost)->x.sink     = sink;
    mtp_handle(phost)->x.sink_arg = sink_arg;
  }
  return status;
}

USBH_StatusTypeDef USBH_MTP_XactStatus(USBH_HandleTypeDef *phost)
{
  if (!USBH_MTP_IsMTP(phost))
  {
    return USBH_FAIL;
  }
  return mtp_handle(phost)->x.status;
}

MTP_XactTypeDef *USBH_MTP_Xact(USBH_HandleTypeDef *phost)
{
  if (!USBH_MTP_IsMTP(phost))
  {
    return NULL;
  }
  return &mtp_handle(phost)->x;
}

/*
 * USBH_MTP_XactAbort
 *     Abandon a transaction (timeout). The bulk channels are halted before
 *     returning, so the hardware no longer accesses the caller's buffer.
 *     The endpoints are then cleared in the background.
 */
void USBH_MTP_XactAbort(USBH_HandleTypeDef *phost)
{
  MTP_HandleTypeDef *MTP_Handle;

  if (!USBH_MTP_IsMTP(phost))
  {
    return;
  }
  MTP_Handle = mtp_handle(phost);
  if (MTP_Handle->state != MTP_XACT)
  {
    return;
  }
  (void) USBH_ClosePipe(phost, MTP_Handle->DataInPipe);
  (void) USBH_ClosePipe(phost, MTP_Handle->DataOutPipe);
  (void) USBH_OpenPipe(phost, MTP_Handle->DataOutPipe, MTP_Handle->DataOutEp,
                       phost->device.address, phost->device.speed,
                       USB_EP_TYPE_BULK, MTP_Handle->DataOutEpSize);
  (void) USBH_OpenPipe(phost, MTP_Handle->DataInPipe, MTP_Handle->DataInEp,
                       phost->device.address, phost->device.speed,
                       USB_EP_TYPE_BULK, MTP_Handle->DataInEpSize);
  MTP_XactFail(phost, 1U);
}
