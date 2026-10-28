/*
 * @file    usbh_xusb.c
 * @author  Chris Hooper
 *
 * This module implements XUSB (XBOX 360, etc) game controller input
 */

#include "usbh_xusb.h"
#include <stdbool.h>
#include "config.h"
#include "timer.h"
#include "utils.h"

#define DEBUG_XUSB_PROTOCOL
#ifdef DEBUG_XUSB_PROTOCOL
#define PPRINTF(...) dprintf(DF_USB, __VA_ARGS__);
#else
#define PPRINTF(...) do { } while (0)
#endif

#define BIT(x) (1U << (x))

static USBH_StatusTypeDef USBH_XUSB_Probe(USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_XUSB_InterfaceInit(USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_XUSB_InterfaceDeInit(USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_XUSB_ClassRequest(USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_XUSB_Process(USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_XUSB_SOFProcess(USBH_HandleTypeDef *phost);
static void USBH_XUSB_PrepareBuf(USBH_HandleTypeDef *phost,
                                 XUSB_Handle_t *XUSB_Handle);

/*
 *                          bType bReq wVal wInd wLen Def Adr Cfg Comments
 * GET_STATUS (Device)       80    00  0000 0000 0002  S   V   V  Bogus status
 * GET_STATUS (Interface)    81    00  0000 0000 0002  S   S   S  Do not support
 * GET_STATUS (Endpoint)     82    00  0000 0000 0002  S   S  V/S Only EP0
 * CLEAR_FEATURE (Device)    00    01  0001 0000 0000  S   V   V
 * CLEAR_FEATURE (Interface) 01    01  0001 0000 0000  S   S   S
 * CLEAR_FEATURE (Endpoint)  02    01  0001 0000 0000  S  V/S  V
 * SET_FEATURE (Device)      00    03  0001 0000 0000  S   V   V
 * SET_FEATURE (Interface)   01    03  0001 0000 0000  S   S   S
 * SET_FEATURE (Endpoint)    02    03  0001 0000 0000  S  V/S  S
 * SET_ADDRESS               00    05  Addr 0000 0000  V   V   S
 * GET_DESCRIPTOR (Device)   80    06  0100 0000 0000  V   V   S
 * GET_DESCRIPTOR (Config)   80    06  0200 0000 0000  V   V   S
 * GET_CONFIGURATION         80    08  0000 0000 0001  S   V   V
 * SET_CONFIGURATION         00    09  Conf 0000 0001  S   V   V  Conf=00 or 01
 * SET_DESCRIPTOR (All)      00    07  0100 0000 Leng  S   S   S  No Support
 * SET_DESCRIPTOR (All)      01    07  0100 0000 Leng  S   S   S  No Support
 * SET_DESCRIPTOR (All)      02    07  0100 0000 Leng  S   S   S  No Support
 * GET_INTERFACE             81    0a  0000 0000 0001  S   S   S  No alt Iface
 * SET_INTERFACE             01    0b  0000 0000 0001  S   S   S  No alt Iface
 *
 * Vendor-specific
 * SET_CONTROL               41    00  Ctrl bIF# 0000  S   S   S  Set control
 * GET_DEVICE_ID             c0    01  0000 0000 0004  S   S   V  Get serial #
 * SET_BIND_INFO             40    01  0001 0000 0007  S   S   V  Wireless bind
 * XBox360 Magic             c1    01  0100 0000 0014
 *    c1 = Direction: D2H, USB_REQ_TYPE_VENDOR, USB_REQ_RECIPIENT_INTERFACE
 *    01 = USB_REQ_CLEAR_FEATURE
 *    0100 = ?
 *    0000 = ?
 *    0014 = ?
 *
 * HID
 * GET_REPORT (input)        a1    01  01xx bIF# wLen  S   S   V  input report
 * SET_REPORT (Output)       21    09  0200 bIF# wLen  S   S   V  set control
 * GET_CAPABILITIES          ??    ??  01xx bIF# wLen  S   S   S  set control
 *
 * Get_MS_OS_Descriptor      80    06  03ee 0000 0012  S   V   V  Get MS descr
 * Get_Ext_Conf_Descriptor   c0   bZVc 0000 0004 0028  S   V   V  Get Ext descr
 *
 * MS OS String Descriptor:
 * Offset Field              Size  Value   Description
 * 0      bLength               1  12      Length of descriptor
 * 1      bDescriptorType       1  03      String Descriptor
 * 2      qwSignature          14  MSFT100 Signature field
 * 16     bMS_VendorCode        1  Custom  Vendor code for OS Feature Desc
 * 17     bPad                  1  00      Pad field
 *
 * Extended Compatible ID Descriptor (wIndex 0x0004):
 * Offset Field              Size Value Description
 * 0      dwLength              4  0028  Length of descriptor
 * 4      bcdVersion            2  0100  Version 1.0
 * 6      wIndex                2  0004  Extended Configuration Descriptor
 * 8      bCount                1  01    Number of Function Sections
 * 9      RESERVED              7  00000000000000 Reserved
 * 16     bFirstInterfaceNumber 1  00    Start Interface number for function
 * 17     bNumInterfaces        1  Numb  Same value as part of cfg desc
 * 18     compatibleID          8  5855534231300000 "XUSB10"
 * 26     subCompatibleID       8  0000000000000000 (none)
 * 34     RESERVED              6  000000000000     Reserved
 *
 * XBox-360 controller reports
 *
 *  Bit   Byte/Bit  Meaning
 *  16      2 / 0   Joypad Up
 *  17      2 / 1   Joypad Down
 *  18      2 / 2   Joypad Left
 *  19      2 / 3   Joypad Right
 *  20      2 / 4   Start
 *  21      2 / 5   Select
 *  24      3 / 0   Left top button
 *  25      3 / 1   Right top button
 *  28      3 / 4   Button B
 *  29      3 / 5   Button A
 *  30      3 / 6   Button Y
 *  31      3 / 7   Button X
 *  32-39   4       Left bottom button
 *  40-47   5       Right bottom button
 *  48-63   6-7     Left joystick L-R, Can just use byte 7: 0=Center
 *                  80=FullLeft, C0=HalfLeft, 40=HalfRight, ff7f=FullRight
 *  64-79   8-9     Left joystick U-D, Can just use byte 9: 0=Center
 *                  80=FullDown, C0=HalfDown, 40=HalfUp, ff7f=FullUp
 *  80-95   10-11   Right joystick L-R, Can just use byte 11: 0=Center
 *                  80=FullLeft, C0=HalfLeft, 40=HalfRight, ff7f=FullRight
 *  96-111  12-13   Right joystick U-D, Can just use byte 13: 0=Center
 *                  79=FullDown, C0=HalfDown, 40=HalfUp, ff7f=FullUp
 */

USBH_ClassTypeDef XUSB_Class =
{
    "XUSB",
    USB_XUSB_CLASS,
    USBH_XUSB_Probe,
    USBH_XUSB_InterfaceInit,
    USBH_XUSB_InterfaceDeInit,
    USBH_XUSB_ClassRequest,
    USBH_XUSB_Process,
    USBH_XUSB_SOFProcess,
    NULL,
};

static USBH_StatusTypeDef
send_xbox360_feature_request(USBH_HandleTypeDef *phost)
{
    /* 0x01=INTERFACE  0x03=SET_FEATURE  0x02=? */
    uint8_t const feature[] = { 0x01, 0x03, 0x02 };
    uint8_t pipe_num = 0x03;

    return (USBH_InterruptSendData(phost, (uint8_t *) feature,
                                   sizeof (feature), pipe_num));
}

static USBH_StatusTypeDef
USBH_XUSB_InterfaceInit_ll(USBH_HandleTypeDef *phost, uint8_t interface)
{
    USBH_StatusTypeDef status;
    XUSB_Handle_t *XUSB_Handle;
    uint8_t max_ep;
    uint8_t num = 0U;

    status = USBH_SelectInterface(phost, interface);
    if (status != USBH_OK)
        return (USBH_FAIL);

    XUSB_Handle = (XUSB_Handle_t *)USBH_malloc(sizeof (XUSB_Handle_t));
    if (XUSB_Handle == NULL) {
        USBH_DbgLog("Cannot allocate memory for XUSB Handle");
        return (USBH_FAIL);
    }

    /* Initialize XUSB handler */
    USBH_memset(XUSB_Handle, 0, sizeof (XUSB_Handle_t));

#if 0
    /* pData list in reverse order */
    XUSB_Handle->next = phost->pActiveClass->pData;
    phost->pActiveClass->pData = XUSB_Handle;
#else
    /*
     * pData list must be in insertion order because some XUSB devices
     * such as the Dell USB Hub Keyboard require that interfaces be
     * processed in order.
     */
    XUSB_Handle_t *prev = (XUSB_Handle_t *) phost->pActiveClass->pData;

    XUSB_Handle->next = NULL;

    if (prev == NULL) {
        phost->pActiveClass->pData = XUSB_Handle;
    } else {
        while (prev->next != NULL)
                prev = prev->next;
        prev->next = XUSB_Handle;
    }
#endif

    XUSB_Handle->interface  = interface;
    XUSB_Handle->state      = XUSB_INIT;
    XUSB_Handle->ep_addr    = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[0].bEndpointAddress;
    XUSB_Handle->length     = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[0].wMaxPacketSize;
    XUSB_Handle->length_max = phost->device.DevDesc.bMaxPacketSize;
    XUSB_Handle->poll       = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[0].bInterval;

    if (XUSB_Handle->poll < XUSB_MIN_POLL) {
        XUSB_Handle->poll = XUSB_MIN_POLL;
    }

    /* Find the number of EPs in the Interface Descriptor */
    max_ep = phost->device.CfgDesc.Itf_Desc[interface].bNumEndpoints;
    if (max_ep > USBH_MAX_NUM_ENDPOINTS)
        max_ep = USBH_MAX_NUM_ENDPOINTS;

    /* Decode endpoint IN and OUT address from interface descriptor */
    for (num = 0U; num < max_ep; num++) {
        if (phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[num].bEndpointAddress & 0x80U) {
            XUSB_Handle->InEp = (phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[num].bEndpointAddress);
            XUSB_Handle->InPipe = USBH_AllocPipe(phost, XUSB_Handle->InEp);

            /* Open pipe for IN endpoint */
            USBH_OpenPipe(phost, XUSB_Handle->InPipe, XUSB_Handle->InEp,
                          phost->device.address, phost->device.speed,
                          USB_EP_TYPE_INTR, XUSB_Handle->length_max);

            USBH_LL_SetToggle(phost, XUSB_Handle->InPipe, 0U);

        } else {
            XUSB_Handle->OutEp = (phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[num].bEndpointAddress);
            XUSB_Handle->OutPipe = USBH_AllocPipe(phost, XUSB_Handle->OutEp);

            /* Open pipe for OUT endpoint */
            USBH_OpenPipe(phost, XUSB_Handle->OutPipe, XUSB_Handle->OutEp,
                          phost->device.address, phost->device.speed,
                          USB_EP_TYPE_INTR, XUSB_Handle->length);

            USBH_LL_SetToggle(phost, XUSB_Handle->OutPipe, 0U);
        }
    }

    return (USBH_OK);
}

/* X-Box 360 compatible controllers */
struct {
    uint16_t vendor;
    uint16_t product;
} match_table[] = {
    { 0x045e, 0x028e },  // Microsoft X-Box 360 Controller
    { 0x046d, 0xc242 },  // Logitech Chillstream Controller
    { 0x0738, 0x4716 },  // Mad Catz Wired Xbox 360 Controller
    { 0x0738, 0x4738 },  // Mad Catz Wired Xbox 360 Controller (SFIV)
    { 0x0e6f, 0x0006 },  // Pelican 'TSZ' Wired Xbox 360 Controller
    { 0x0e6f, 0x0201 },  // Pelican PL-3601 'TSZ' Wired Xbox 360 Controller
    { 0x12ab, 0x0004 },  // Honey Bee Xbox360 Dancepad
    { 0x0e6f, 0x0105 },  // HSM3 Xbox360 Dancepad
    { 0x1430, 0x4748 },  // RedOctane Guitar Hero X-plorer
    { 0x146b, 0x0601 },  // BigBen Interactive XBOX 360 Controller
    { 0x1bad, 0x0002 },  // Harmonix Rock Band Guitar
    { 0x1bad, 0x0003 },  // Harmonix Rock Band Drumkit
    { 0x0f0d, 0x0016 },  // Hori Real Arcade Pro.EX
    { 0x0f0d, 0x000d },  // Hori Fighting Stick EX2
};

static USBH_StatusTypeDef
USBH_XUSB_Probe(USBH_HandleTypeDef *phost)
{
    uint16_t vendor  = phost->device.DevDesc.idVendor;
    uint16_t product = phost->device.DevDesc.idProduct;
    uint     pos;

    for (pos = 0; pos < ARRAY_SIZE(match_table); pos++) {
        if ((vendor == match_table[pos].vendor) &&
            (product == match_table[pos].product)) {
            return (USBH_OK);
        }
    }
    return (USBH_FAIL);
}

/**
 * @brief  USBH_XUSB_InterfaceInit
 *         The function init the XUSB class.
 * @param  phost: Host handle
 * @retval USBH Status
 */
static USBH_StatusTypeDef
USBH_XUSB_InterfaceInit(USBH_HandleTypeDef *phost)
{
    uint8_t interface;
    USBH_StatusTypeDef status = USBH_FAIL;
    USBH_StatusTypeDef t_status;
    uint numif = phost->device.CfgDesc.bNumInterfaces;
    uint16_t vendor = phost->device.DevDesc.idVendor;
    uint16_t product = phost->device.DevDesc.idProduct;

    if (vendor != 0x045e) { // Microsoft
        printf("USB%s Unrecognized USB vendor %04x\n",
               usb_get_path(phost, NULL, -1), vendor);
        return (USBH_FAIL);
    }
    switch (product) {
        case 0x028e:  // XBox 360 controller
            break;
        default:
            printf("USB%s Unrecognized USB product %04x.%04x\n",
                   usb_get_path(phost, NULL, -1), vendor, product);
            printf("Hopefully it acts like the XBox 360 controller\n");
            break;
    }

    if (numif > USBH_MAX_NUM_INTERFACES)
        numif = USBH_MAX_NUM_INTERFACES;
    for (interface = 0; interface < numif; interface++) {
        t_status = USBH_XUSB_InterfaceInit_ll(phost, interface);
        if (t_status == USBH_OK)
            status = t_status;
    }

    if (status == USBH_FAIL) { /* No Valid Interface */
        USBH_DbgLog("Cannot Find the interface for %s class.",
                    phost->pActiveClass->Name);
    }
    return (status);
}

static USBH_StatusTypeDef
USBH_XUSB_InterfaceDeInit_ll(USBH_HandleTypeDef *phost)
{
    XUSB_Handle_t *XUSB_Handle = (XUSB_Handle_t *) phost->pActiveClass->pData;

    if (XUSB_Handle == NULL)
        return (USBH_FAIL);

    if (XUSB_Handle->InPipe != 0x00U) {
        USBH_LL_StopHC(phost, XUSB_Handle->InPipe);

        USBH_ClosePipe(phost, XUSB_Handle->InPipe);
        USBH_FreePipe(phost, XUSB_Handle->InPipe);
        XUSB_Handle->InPipe = 0U;     /* Reset the pipe as Free */
    }

    if (XUSB_Handle->OutPipe != 0x00U) {
        USBH_LL_StopHC(phost, XUSB_Handle->OutPipe);

        USBH_ClosePipe(phost, XUSB_Handle->OutPipe);
        USBH_FreePipe(phost, XUSB_Handle->OutPipe);
        XUSB_Handle->OutPipe = 0U;     /* Reset the pipe as Free */
    }

    phost->pActiveClass->pData = XUSB_Handle->next;
    USBH_free(XUSB_Handle);
    return (USBH_OK);
}

/**
 * @brief  USBH_XUSB_InterfaceDeInit
 *         The function DeInit the Pipes used for the XUSB class.
 * @param  phost: Host handle
 * @retval USBH Status
 */
static USBH_StatusTypeDef
USBH_XUSB_InterfaceDeInit(USBH_HandleTypeDef *phost)
{
    USBH_StatusTypeDef status = USBH_FAIL;

    printf("USBH_XUSB_InterfaceDeInit\n");
    while (phost->pActiveClass->pData != NULL)
        status = USBH_XUSB_InterfaceDeInit_ll(phost);

    return (status);
}

/**
 * @brief  USBH_XUSB_ClassRequest
 *         The function is responsible for handling Standard requests
 *         for XUSB class.
 * @param  phost: Host handle
 * @retval USBH Status
 */
static USBH_StatusTypeDef
USBH_XUSB_ClassRequest(USBH_HandleTypeDef *phost)
{
    (void) phost;
    phost->pUser(phost, HOST_USER_CLASS_ACTIVE);
    return (USBH_OK);
}

static USBH_StatusTypeDef
USBH_XUSB_Process_ll(USBH_HandleTypeDef *phost, XUSB_Handle_t *XUSB_Handle)
{
    USBH_StatusTypeDef status = USBH_OK;
    uint32_t XferSize;

    switch (XUSB_Handle->state) {
        case XUSB_INIT:
            USBH_XUSB_PrepareBuf(phost, XUSB_Handle);
            XUSB_Handle->state = XUSB_FEATURE_REQUEST;

#if (USBH_USE_OS == 1U)
            phost->os_msg = (uint32_t)USBH_URB_EVENT;
#if (osCMSIS < 0x20000U)
            (void) osMessagePut(phost->os_event, phost->os_msg, 0U);
#else
            (void) osMessageQueuePut(phost->os_event, &phost->os_msg, 0U, NULL);
#endif
#endif
            memset(XUSB_Handle->pData, 0, XUSB_Handle->length);
            break;

        case XUSB_FEATURE_REQUEST:
            if (send_xbox360_feature_request(phost) == USBH_OK) {
                XUSB_Handle->state = XUSB_IDLE;
            }
            break;

        case XUSB_IDLE:
            status = USBH_XUSB_GetReport(phost, XUSB_Handle->InPipe);
            if (status == USBH_OK) {
                XUSB_Handle->state = XUSB_SYNC;

            } else if (status == USBH_BUSY) {
                XUSB_Handle->state = XUSB_IDLE;

            } else if (status == USBH_NOT_SUPPORTED) {
                PPRINTF(" ->NOSUP");
                XUSB_Handle->state = XUSB_SYNC;
                status = USBH_OK;

            } else {
                PPRINTF(" ->ERROR");
                if (++XUSB_Handle->error_count < 5) {
                    XUSB_Handle->state = XUSB_FEATURE_REQUEST;
                } else {
                    XUSB_Handle->state = XUSB_ERROR;
                }
                status = USBH_FAIL;
            }

#if (USBH_USE_OS == 1U)
            phost->os_msg = (uint32_t)USBH_URB_EVENT;
#if (osCMSIS < 0x20000U)
            (void) osMessagePut(phost->os_event, phost->os_msg, 0U);
#else
            (void) osMessageQueuePut(phost->os_event, &phost->os_msg, 0U, NULL);
#endif
#endif
            break;

        case XUSB_SYNC:
            /* Sync with start of Even Frame */
            if (phost->Timer & 1U) {
                XUSB_Handle->state = XUSB_GET_DATA;
            }

#if (USBH_USE_OS == 1U)
            phost->os_msg = (uint32_t)USBH_URB_EVENT;
#if (osCMSIS < 0x20000U)
            (void) osMessagePut(phost->os_event, phost->os_msg, 0U);
#else
            (void) osMessageQueuePut(phost->os_event, &phost->os_msg, 0U, NULL);
#endif
#endif
            break;

        case XUSB_GET_DATA:
            USBH_InterruptReceiveData(phost, XUSB_Handle->pData,
                                      (uint8_t)XUSB_Handle->length_max,
                                      XUSB_Handle->InPipe);

            XUSB_Handle->state = XUSB_POLL;
            XUSB_Handle->timer = phost->Timer;
            XUSB_Handle->DataReady = 0U;
            break;

        case XUSB_POLL:
            if (USBH_LL_GetURBState(phost, XUSB_Handle->InPipe) == USBH_URB_DONE) {
                XferSize = USBH_LL_GetLastXferSize(phost, XUSB_Handle->InPipe);

                if ((XUSB_Handle->DataReady == 0U) && (XferSize != 0U)) {
                    XUSB_Handle->DataReady = 1U;
                    USBH_XUSB_EventCallback(phost, XUSB_Handle);

#if (USBH_USE_OS == 1U)
                    phost->os_msg = (uint32_t)USBH_URB_EVENT;
#if (osCMSIS < 0x20000U)
                    (void) osMessagePut(phost->os_event, phost->os_msg, 0U);
#else
                    (void) osMessageQueuePut(phost->os_event, &phost->os_msg, 0U, NULL);
#endif
#endif
                }
            } else {
                /* IN Endpoint Stalled */
                if (USBH_LL_GetURBState(phost, XUSB_Handle->InPipe) == USBH_URB_STALL) {
                    /* Issue Clear Feature on interrupt IN endpoint */
                    if (USBH_ClrFeature(phost, XUSB_Handle->ep_addr) == USBH_OK) {
                        /* Change state to issue next IN token */
                        XUSB_Handle->state = XUSB_GET_DATA;
                    }
                }
            }
            break;

        default:
            break;
    }

    return (status);
}

/**
 * @brief  USBH_XUSB_Process
 *         The function is for managing state machine for XUSB data transfers
 *         It is the background processor for the class (BgndProcess).
 * @param  phost: Host handle
 * @retval USBH Status
 */
static USBH_StatusTypeDef
USBH_XUSB_Process(USBH_HandleTypeDef *phost)
{
    uint bit = 0;
    USBH_StatusTypeDef status;
    XUSB_Handle_t *XUSB_Handle =
                  (XUSB_Handle_t *) phost->pActiveClass->pData;

    while (XUSB_Handle != NULL) {
        if (phost->iface_waiting) {
            printf(" Waiting %lx", phost->iface_waiting);
            if (phost->iface_waiting & BIT(bit)) {
                status = USBH_XUSB_Process_ll(phost, XUSB_Handle);
                if (status == USBH_OK)
                    phost->iface_waiting &= ~BIT(bit);
            }
        } else {
            status = USBH_XUSB_Process_ll(phost, XUSB_Handle);
            if (status == USBH_BUSY) {
                break;  // Check again later
            }
        }
        bit++;
        XUSB_Handle = XUSB_Handle->next;
    }
    return (phost->iface_waiting ? USBH_FAIL: USBH_OK);
}

/**
 * @brief  USBH_XUSB_SOFProcess
 *         The function is for managing the SOF Process
 * @param  phost: Host handle
 * @retval USBH Status
 */
static USBH_StatusTypeDef
USBH_XUSB_SOFProcess(USBH_HandleTypeDef *phost)
{
    XUSB_Handle_t *XUSB_Handle = (XUSB_Handle_t *) phost->pActiveClass->pData;

    if (XUSB_Handle->state == XUSB_POLL) {
        if ((phost->Timer - XUSB_Handle->timer) >= XUSB_Handle->poll) {
            XUSB_Handle->state = XUSB_GET_DATA;

#if (USBH_USE_OS == 1U)
            phost->os_msg = (uint32_t)USBH_URB_EVENT;
#if (osCMSIS < 0x20000U)
            (void) osMessagePut(phost->os_event, phost->os_msg, 0U);
#else
            (void) osMessageQueuePut(phost->os_event, &phost->os_msg, 0U, NULL);
#endif
#endif
        }
    }

    return (USBH_OK);
}

USBH_StatusTypeDef
USBH_XUSB_GetReport(USBH_HandleTypeDef *phost, uint8_t pipe_num)
{
    /*
     * Custom Vendor request for XBox360 controller
     *  0xc1 = D2H, TYPE_VENDOR, INTERFACE
     *  0x01 = GET
     *  0x0100 = ?
     *  0x0000 = ?
     *  0x0014 = ?
     */
    static const uint8_t vendor_req[] = { 0xc1, 0x01, 0x00, 0x01,
                                          0x00, 0x00, 0x14, 0x00 };

    return (USBH_CtlSendSetup(phost, (uint8_t *)vendor_req, pipe_num));
}

/**
 * @brief  The function is a callback about XUSB Data events
 * @param  phost: Selected device
 * @retval None
 */
__weak void
USBH_XUSB_EventCallback(USBH_HandleTypeDef *phost, XUSB_Handle_t *XUSB_Handle)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(phost);
    UNUSED(XUSB_Handle);
}

#include "config.h"
#include "usbh_hid_mouse.h"

static int
readbits(void *ptr, uint startbit, uint bits, uint is_signed)
{
    uint byte = startbit / 8;
    uint bitoff = startbit % 8;
    uint mask = BIT(bits) - 1;
    uint val = ((*(uint *) ((uintptr_t) ptr + byte)) >> bitoff) & mask;

    if (is_signed && (val & BIT(bits - 1)))
        val |= (0 - BIT(bits));  // Sign-extend negative

    return (val);
}

/**
 * @brief  USBH_XUSB_DecodeReport
 *         The function gets and decodes mouse and generic data.
 * @param  phost: Host handle
 * @retval USBH Status
 */
void
USBH_XUSB_DecodeReport(USBH_HandleTypeDef *phost, XUSB_Handle_t *XUSB_Handle,
                       XUSB_MISC_Info_t *report_info)
{
    uint32_t *report_data;
    int8_t val;

    if (XUSB_Handle == NULL)
        return;

    if (XUSB_Handle->length == 0U)
        return;

    /* Report data is what was most recently received */
    report_data = (uint32_t *)XUSB_Handle->pData;

    val = readbits(report_data, 56, 8, 1);   // Left joystick L-R
    report_info->wheel_x = val / 8;
    val = readbits(report_data, 72, 8, 1);   // Left joystick U-D
    report_info->wheel_y = val / 8;
    val = readbits(report_data, 88, 8, 1);   // Right joystick L-R
    report_info->mouse_x = val / 32;
    val = readbits(report_data, 104, 8, 1);  // Right joystick U-D
    report_info->mouse_y = val / 32;
    report_info->joypad = readbits(report_data, 16, 4, 0);

    /*
     * Bit 0 = Button B Red
     * Bit 1 = Button A Green
     * Bit 2 = Button Y Yellow
     * Bit 3 = Button X Blue
     * Bit 4 = Start
     * Bit 5 = Select
     * Bit 6 = Left joystick center depressed
     * Bit 7 = Right joystick center depressed
     * Bit 8 = Left top button
     * Bit 9 = Right top button
     * Bit 10 = Right checker pattern button
     * Bit 11 = Left bottom button "trigger"
     * Bit 12 = Right bottom button "trigger"
     */
    report_info->buttons = readbits(report_data, 28, 4, 0) |
                           (readbits(report_data, 20, 4, 0) << 4) |
                           (readbits(report_data, 24, 3, 0) << 8) |
                           (readbits(report_data, 32, 1, 0) << 11) |
                           (readbits(report_data, 40, 1, 0) << 12);
}

static uint32_t xusb_rx_report_buf[2][XUSB_QUEUE_SIZE * 4];

static void
USBH_XUSB_PrepareBuf(USBH_HandleTypeDef *phost, XUSB_Handle_t *XUSB_Handle)
{
    uint port = phost->id;

    if (XUSB_Handle->length_max > sizeof (xusb_rx_report_buf[port]))
        XUSB_Handle->length_max = sizeof (xusb_rx_report_buf[port]);

    XUSB_Handle->pData = (uint8_t *)(void *) xusb_rx_report_buf[port];
}
