/**
  ******************************************************************************
  * @file    usbh_mtp.h
  * @author  MCD Application Team
  * @brief   This file contains all the prototypes for the usbh_mtp.c
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
  * AmigaPCI BEC: reworked from the ST MTP class. The ST PTP transaction
  * layer (usbh_mtp_ptp.c) is no longer used: it received datasets into a
  * fixed 1 KB container with no bounds check, and its API functions spun
  * in busy loops. Transactions are now run by a bounded, non-blocking
  * engine advanced from the class background process, as is done for MSC
  * (see usbh_msc.h). usbh_mtp_ptp.h is still used for the PTP / MTP
  * constants.
  */

/* Define to prevent recursive  ----------------------------------------------*/
#ifndef __USBH_MTP_H
#define __USBH_MTP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbh_mtp_ptp.h"
#include "usbh_core.h"

/* Still Image class (PTP, and MTP devices which use it) */
#define USB_MTP_CLASS                 0x06U
/* Android presents MTP as a vendor-specific interface named "MTP" */
#define USB_MTP_VENDOR_CLASS          0xFFU

#define MTP_MAX_STORAGE               4U      /* Storage units tracked */
#define MTP_XBUF_SIZE                 512U    /* Bounce buffer (>= max packet) */
#define MTP_NAME_MAX                  40U     /* Storage / device name length */

/* Android MTP extensions (used for writing part of a file) */
#define MTP_OC_ANDROID_GetPartialObject64   0x95C1U
#define MTP_OC_ANDROID_SendPartialObject    0x95C2U
#define MTP_OC_ANDROID_TruncateObject       0x95C3U
#define MTP_OC_ANDROID_BeginEditObject      0x95C4U
#define MTP_OC_ANDROID_EndEditObject        0x95C5U

/* Operations the device supports (MTP_HandleTypeDef.ops) */
#define MTP_OPS_PARTIAL       0x0001U   /* GetPartialObject */
#define MTP_OPS_PARTIAL64     0x0002U   /* Android GetPartialObject64 */
#define MTP_OPS_EDIT          0x0004U   /* Android Begin/EndEditObject,
                                           SendPartialObject, TruncateObject */
#define MTP_OPS_SENDINFO      0x0008U   /* SendObjectInfo + SendObject */
#define MTP_OPS_DELETE        0x0010U   /* DeleteObject */
#define MTP_OPS_MOVE          0x0020U   /* MoveObject */
#define MTP_OPS_SETPROP       0x0040U   /* SetObjectPropValue */
#define MTP_OPS_GETPROP       0x0080U   /* GetObjectPropValue */
#define MTP_OPS_PROPLIST      0x0100U   /* GetObjectPropList */

typedef enum
{
  MTP_CHECK_NAME = 0,     /* Vendor class: check the interface name is "MTP" */
  MTP_OPENSESSION,
  MTP_GETDEVICEINFO,
  MTP_GETSTORAGEIDS,
  MTP_GETSTORAGEINFO,
  MTP_IDLE,
  MTP_XACT,               /* Running a transaction for the user */
  MTP_RECOVERY,           /* Clearing halted endpoints after an error */
  MTP_NOT_MTP,            /* Vendor interface which is not MTP: inert */
  MTP_FAILED,             /* Initialization failed: inert */
}
MTP_StateTypeDef;

/* Transaction direction of the data phase */
#define MTP_DATA_NONE   0U
#define MTP_DATA_IN     1U
#define MTP_DATA_OUT    2U

typedef enum
{
  MTP_XS_CMD = 0,
  MTP_XS_CMD_WAIT,
  MTP_XS_DOUT,
  MTP_XS_DOUT_WAIT,
  MTP_XS_DIN,
  MTP_XS_DIN_WAIT,
  MTP_XS_RESP,
  MTP_XS_RESP_WAIT,
  MTP_XS_DONE,
}
MTP_XactStateTypeDef;

/* One PTP transaction: command, optional data phase, response */
typedef struct
{
  uint16_t   code;          /* Operation code */
  uint8_t    nparam;        /* Number of command parameters */
  uint8_t    dir;           /* MTP_DATA_* */
  uint32_t   param[5];
  uint8_t   *buf;           /* IN: data destination; OUT: data source */
  uint32_t   buflen;        /* IN: size of buf; OUT: bytes to send */
  uint32_t   len;           /* IN: bytes stored in buf */
  uint32_t   total;         /* IN: data payload size the device reported */
  uint32_t   pos;           /* Data payload bytes transferred so far */
  uint16_t   xfer_len;      /* Length of the URB in flight */
  uint8_t    xfer_direct;   /* URB in flight is directly into buf */
  uint8_t    state;         /* MTP_XactStateTypeDef */
  uint8_t    first;         /* Next data IN URB is the first */
  uint8_t    nresp;         /* Number of response parameters */
  uint16_t   resp;          /* Response code */
  uint32_t   rparam[5];     /* Response parameters */
  USBH_StatusTypeDef status;  /* USBH_BUSY while running; OK or FAIL */
  /*
   * Optional data IN consumer: when set, received data is passed to it
   * as it arrives (buf is then only a staging buffer), so a data phase
   * may be larger than buf.
   */
  void     (*sink)(void *arg, const uint8_t *data, uint32_t len);
  void      *sink_arg;
  uint8_t    ntrace;        /* Debug: URB completions recorded */
  uint16_t   trace[8];      /* Debug: (state << 12) | length */
}
MTP_XactTypeDef;

typedef struct
{
  uint32_t  id;
  uint16_t  type;               /* PTP_ST_* */
  uint16_t  fstype;             /* PTP_FST_* */
  uint16_t  access;             /* PTP_AC_* */
  uint64_t  capacity;           /* Bytes */
  uint64_t  freespace;          /* Bytes */
  char      desc[MTP_NAME_MAX]; /* StorageDescription (UTF-8) */
  char      label[MTP_NAME_MAX];/* VolumeLabel (UTF-8) */
}
MTP_StorageTypeDef;

/* Structure for MTP process */
typedef struct _MTP_Process
{
  uint8_t               DataInPipe;
  uint8_t               DataOutPipe;
  uint8_t               NotificationPipe;

  uint8_t               DataOutEp;
  uint8_t               DataInEp;
  uint8_t               NotificationEp;

  uint16_t              DataOutEpSize;
  uint16_t              DataInEpSize;
  uint16_t              NotificationEpSize;

  uint8_t               interface;      /* bInterfaceNumber */
  uint8_t               iInterface;     /* Interface name string index */
  MTP_StateTypeDef      state;
  uint8_t               rr_step;        /* MTP_RECOVERY step */
  uint8_t               is_ready;
  uint8_t               cur_storage;    /* Init: storage being read */
  uint8_t               nstorage;       /* Number of storage units */
  uint8_t               rescan;         /* Storage set changed: re-read */
  uint8_t               ev_active;      /* Event interrupt URB in flight */
  uint16_t              ops;            /* MTP_OPS_* */
  uint32_t              session_id;
  uint32_t              transaction_id;
  uint64_t              ev_time;        /* Next event poll (timer ticks) */

  MTP_StorageTypeDef    storage[MTP_MAX_STORAGE];
  char                  manufacturer[MTP_NAME_MAX];
  char                  model[MTP_NAME_MAX];

  MTP_XactTypeDef       x;              /* Current transaction */
  uint8_t              *initbuf;        /* Data buffer during init */
  uint32_t              xbuf[MTP_XBUF_SIZE / 4];  /* Bounce buffer */
  uint32_t              evbuf[64 / 4];  /* Event interrupt data */
}
MTP_HandleTypeDef;

extern USBH_ClassTypeDef  MTP_Class;
extern USBH_ClassTypeDef  MTP_VendorClass;
#define USBH_MTP_CLASS          &MTP_Class
#define USBH_MTP_VENDOR_CLASS   &MTP_VendorClass

/* Incremented whenever any MTP device is de-initialized (removed) */
extern volatile uint32_t USBH_MTP_DeInitCount;
/* Incremented when an MTP device reports objects or storage changed */
extern volatile uint32_t USBH_MTP_ChangeCount;

/* Is the device's active class MTP? */
uint8_t            USBH_MTP_IsMTP(USBH_HandleTypeDef *phost);
uint8_t            USBH_MTP_IsReady(USBH_HandleTypeDef *phost);

/*
 * Asynchronous transactions. Start one with USBH_MTP_StartXact(), then
 * poll USBH_MTP_XactStatus() until it is no longer USBH_BUSY. The
 * transaction (including the response code and parameters) can then be
 * examined with USBH_MTP_Xact(). For MTP_DATA_IN, buf receives up to
 * buflen bytes of the data payload (the rest is discarded; x->total is
 * the size the device sent). For MTP_DATA_OUT, buflen bytes of buf are
 * sent. buf must stay valid until the transaction completes, or until
 * USBH_MTP_XactAbort() returns.
 *
 * USBH_MTP_StartXact() returns USBH_BUSY if the device is not idle yet.
 */
USBH_StatusTypeDef USBH_MTP_StartXact(USBH_HandleTypeDef *phost,
                                      uint16_t code, uint8_t nparam,
                                      const uint32_t *params, uint8_t dir,
                                      uint8_t *buf, uint32_t buflen);
USBH_StatusTypeDef USBH_MTP_StartXactSink(USBH_HandleTypeDef *phost,
                                          uint16_t code, uint8_t nparam,
                                          const uint32_t *params,
                                          uint8_t *buf, uint32_t buflen,
                                          void (*sink)(void *arg,
                                                       const uint8_t *data,
                                                       uint32_t len),
                                          void *sink_arg);
USBH_StatusTypeDef USBH_MTP_XactStatus(USBH_HandleTypeDef *phost);
MTP_XactTypeDef   *USBH_MTP_Xact(USBH_HandleTypeDef *phost);
void               USBH_MTP_XactAbort(USBH_HandleTypeDef *phost);

/* Decode a PTP string at *off of buf (len bytes) to UTF-8 */
uint32_t MTP_GetString(const uint8_t *buf, uint32_t len, uint32_t *off,
                       char *dst, uint32_t dstlen);

void USBH_MTP_EventsCallback(USBH_HandleTypeDef *phost, uint32_t event,
                             uint32_t param);

#ifdef __cplusplus
}
#endif

#endif /* __USBH_MTP_H */
