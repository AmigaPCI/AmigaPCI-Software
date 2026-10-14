/**
  ******************************************************************************
  * @file    usbh_hid.c
  * @author  MCD Application Team
  * @brief   This file is the HID Layer Handlers for USB Host HID class.
  *
  * @verbatim
  *
  *          ===================================================================
  *                                HID Class  Description
  *          ===================================================================
  *           This module manages the HID class V1.11 following the "Device Class Definition
  *           for Human Interface Devices (HID) Version 1.11 Jun 27, 2001".
  *           This driver implements the following aspects of the specification:
  *             - The Boot Interface Subclass
  *             - The Mouse and Keyboard protocols
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
  */

/* BSPDependencies
- "stm32xxxxx_{eval}{discovery}{nucleo_144}.c"
- "stm32xxxxx_{eval}{discovery}_io.c"
- "stm32xxxxx_{eval}{discovery}{adafruit}_lcd.c"
- "stm32xxxxx_{eval}{discovery}_sdram.c"
EndBSPDependencies */

/* Includes ------------------------------------------------------------------*/
#include "usbh_hid.h"
#include "usbh_hid_parser.h"

#include <stdbool.h>
#include "config.h"
#include "timer.h"
#include "utils.h"
#include "usb.h"
#include "irq.h"
#define DEBUG_HIDREPORT_DESCRIPTOR
#ifdef DEBUG_HIDREPORT_DESCRIPTOR
#define DPRINTF(...) dprintf(DF_USB_REPORT, __VA_ARGS__)
#else
#define DPRINTF(...)
#endif

#undef DEBUG_HID_PROTOCOL
#ifdef DEBUG_HID_PROTOCOL
#define PPRINTF(...) dprintf(DF_USB, __VA_ARGS__);
#else
#define PPRINTF(...) do { } while (0)
#endif

/** @addtogroup USBH_LIB
* @{
*/

/** @addtogroup USBH_CLASS
* @{
*/

/** @addtogroup USBH_HID_CLASS
* @{
*/

/** @defgroup USBH_HID_CORE
* @brief    This file includes HID Layer Handlers for USB Host HID class.
* @{
*/

/** @defgroup USBH_HID_CORE_Private_TypesDefinitions
* @{
*/
/**
* @}
*/


/** @defgroup USBH_HID_CORE_Private_Defines
* @{
*/
/**
* @}
*/


/** @defgroup USBH_HID_CORE_Private_Macros
* @{
*/
/**
* @}
*/


/** @defgroup USBH_HID_CORE_Private_Variables
* @{
*/

/**
* @}
*/


/** @defgroup USBH_HID_CORE_Private_FunctionPrototypes
* @{
*/

static USBH_StatusTypeDef USBH_HID_InterfaceInit(USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_HID_InterfaceDeInit(USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_HID_ClassRequest(USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_HID_Process(USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_HID_SOFProcess(USBH_HandleTypeDef *phost);
static void  USBH_HID_ParseHIDDesc(HID_DescTypeDef *desc, uint8_t *buf);

extern USBH_StatusTypeDef USBH_HID_MouseInit(USBH_HandleTypeDef *phost, HID_HandleTypeDef *HID_Handle);
extern USBH_StatusTypeDef USBH_HID_KeybdInit(USBH_HandleTypeDef *phost, HID_HandleTypeDef *HID_Handle);

USBH_ClassTypeDef  HID_Class =
{
  "HID",
  USB_HID_CLASS,
  NULL,
  USBH_HID_InterfaceInit,
  USBH_HID_InterfaceDeInit,
  USBH_HID_ClassRequest,
  USBH_HID_Process,
  USBH_HID_SOFProcess,
  NULL,
};
/**
* @}
*/


/** @defgroup USBH_HID_CORE_Private_Functions
* @{
*/


/** @defgroup USBH_HID_MOUSE_Private_Functions
  * @{
  */
void USBH_HID_Process_HIDReportDescriptor(USBH_HandleTypeDef *phost, HID_HandleTypeDef *HID_Handle)
{
    uint pos;
    uint desc_len = HID_Handle->HID_Desc.wItemLength;
    uint coll_depth = 0;
    uint8_t report_size = 0;  // in bits
    uint8_t report_count = 0;
    uint8_t usage_page = 0;
    uint16_t usage = 0;
    uint    log_min = 0;
    uint    log_max = 0;
    uint    bitpos = 0;  // First byte is usually record #
    uint    button = 0;
    uint    temp;
    uint    value;
    HID_RDescTypeDef *rd = &HID_Handle->HID_RDesc;
    uint16_t usage_array[16];
    uint8_t  usage_count = 0;
    uint8_t  report_id;
    const uint8_t *desc = (const uint8_t *) phost->device.Data;
#ifdef DEBUG_HIDREPORT_DESCRIPTOR
    const char *spaces = "                 ";
#endif
    DPRINTF("USB%u.%u.%u Process_HIDReportDescriptor rlen=%x\n",
            get_port(phost), phost->address, HID_Handle->interface, desc_len);

    if (desc_len > 512)
        desc_len = 512;  // Let's not go crazy here

#if 0
    /* Request HID Report Descriptor (some mice require 20ms) */
    uint timeout = 100;
    while (USBH_HID_GetHIDReportDescriptor(phost, HID_Handle->interface,
                                           desc_len) != USBH_OK) {
        if (--timeout == 0) {
            printf("USB%u.%u.%u Get report descriptor failed\n",
                   get_port(phost), phost->address, HID_Handle->interface);
            return;
        }
        timer_delay_msec(1);
    }
#endif

#if 0
    for (pos = 0; pos < desc_len; pos++)
        printf(" %02x", desc[pos]);
    printf("\n");
#endif
    for (pos = 0; pos < desc_len; ) {
        const uint8_t tag = desc[pos] >> 4;
        const uint8_t type = (desc[pos] >> 2) & 3;
        const uint8_t size = desc[pos] & 3;
        uint cur;

        if ((type == 0) && (tag == 0)) {
            pos++;
            continue;
        }
        DPRINTF("%02x:", pos);
        for (temp = 0; temp <= 2; temp++) {
            if (temp <= size) {
                DPRINTF(" %02x", desc[pos + temp]);
            } else {
                DPRINTF("   ");
            }
        }
        pos++;
        value = 0;
        for (cur = 0; cur < size; cur++)
            value |= (desc[pos + cur] << (8 * cur));
        DPRINTF(" type %x tag %x size %x  ", type, tag, size);
        switch (type) {
            case HID_ITEM_TYPE_MAIN:  // type 0
                switch (tag) {
                    case HID_MAIN_ITEM_TAG_INPUT:
                        DPRINTF("%.*s%s", coll_depth, spaces, "INPUT");
                        DPRINTF(" size=%u count=%u", report_size, report_count);
                        while ((usage_count < report_count) &&
                               (usage_count < ARRAY_SIZE(usage_array))) {
                            usage_array[usage_count++] = usage;
                        }
                        for (temp = 0; temp < report_count; temp++) {
                            int x = temp;
                            if (value & BIT(0)) {
                                /* Constant data */
                                DPRINTF(" C");
                            } else if (usage_page == HID_USAGE_PAGE_BUTTON) {
                                DPRINTF(" Button%u=%u", button, bitpos);
                                if (button < ARRAY_SIZE(rd->pos_button))
                                    rd->pos_button[button] = bitpos;
                                button++;
                                rd->num_buttons = button;
                            } else if (usage_page == HID_USAGE_PAGE_CONSUMER) {
                                DPRINTF(" Consumer usage=%x", usage_array[x]);
                                switch (usage_array[x]) {
                                    case 0x01:  // Generic control
                                        if (rd->num_keys <
                                            ARRAY_SIZE(rd->pos_key)) {
                                            rd->bits_key = report_size;
                                            rd->pos_key[rd->num_keys] = bitpos;
                                            rd->num_keys++;
                                        }
                                        DPRINTF(" key=%u", bitpos);
                                        break;
                                    case HID_USAGE_AC_PAN:
                                        DPRINTF(" AC_PAN=%u", bitpos);
                                        rd->pos_ac_pan = bitpos;
                                        rd->bits_ac_pan = report_size;
                                        break;
                                    default: {
                                        /* Attempt to handle as MM Button */
                                        uint num = rd->num_mmbuttons;
                                        if (report_size != 1)
                                            break;
                                        if (num >= ARRAY_SIZE(rd->val_mmbutton))
                                            break;
                                        DPRINTF(" MM %x %x=%u", num,
                                                usage_array[x], bitpos);
                                        rd->val_mmbutton[num] = usage_array[x];
                                        rd->pos_mmbutton[num] = bitpos;
                                        rd->id_mmbutton[num] = report_id;
                                        rd->num_mmbuttons++;
                                    }
                                }
                            } else if (x <= usage_count) {
                                switch (usage_array[x]) {
                                    case HID_USAGE_X:
                                        if ((value & BIT(2)) == 0) {
                                            DPRINTF(" Absolute");
                                            rd->dev_flag |= DEV_FLAG_ABSOLUTE;
                                        }
                                        DPRINTF(" X=%u", bitpos);
                                        rd->pos_x = bitpos;
                                        rd->bits_x = report_size;
                                        rd->offset_xy = -(log_min +
                                                          log_max) / 2;
//                                      printf("offset_xy=%d\n", rd->offset_xy);
                                        break;
                                    case HID_USAGE_Y:
                                        DPRINTF(" Y=%u", bitpos);
                                        rd->pos_y = bitpos;
                                        rd->bits_y = report_size;
                                        break;
                                    case HID_USAGE_WHEEL:
                                        DPRINTF(" WHEEL=%u", bitpos);
                                        rd->pos_wheel = bitpos;
                                        rd->bits_wheel = report_size;
                                        break;
                                    case HID_USAGE_SYSCTL:
                                    case HID_USAGE_SLEEP:
                                    case HID_USAGE_PWDOWN:
                                    case HID_USAGE_WAKEUP:
                                        DPRINTF(" SYSCTL=%u", bitpos);
                                        rd->pos_sysctl = bitpos;
                                        rd->bits_sysctl = report_size;
                                        if (rd->bits_sysctl > 16)
                                            rd->bits_sysctl = 16;
                                        break;
                                    case HID_USAGE_KBD:
                                        if (x != 0)  // Not first cell
                                            break;
                                        if ((report_size == 1) &&
                                            (report_count == 8)) {
                                            rd->pos_keymod = bitpos;
                                            DPRINTF(" KEYMOD=%u", bitpos);
                                        } else if ((report_size == 8) &&
                                                   (report_count == 6)) {
                                            rd->pos_keynkro = bitpos;
                                            DPRINTF(" KEY6KRO=%u", bitpos);
                                        } else if ((report_size == 1) &&
                                            (report_count > 64)) {
                                            /* Typical count: 152 */
                                            rd->pos_keynkro = bitpos | BIT(15);
                                            DPRINTF(" KEYNKRO=%u", bitpos);
                                        }
                                        break;
                                }
                            }
                            bitpos += report_size;
                        }
                        break;
                    case HID_MAIN_ITEM_TAG_OUTPUT:
                        DPRINTF("%.*s%s", coll_depth, spaces, "OUTPUT");
                        // End output feature
                        break;
                    case HID_MAIN_ITEM_TAG_COLLECTION:
                        DPRINTF("%.*s%s", coll_depth, spaces, "COLL");
                        coll_depth++;
                        break;
                    case HID_MAIN_ITEM_TAG_FEATURE:
                        DPRINTF("%.*s%s", coll_depth, spaces, "FEATURE");
                        break;
                    case HID_MAIN_ITEM_TAG_ENDCOLLECTION:
                        if (coll_depth > 0)
                            coll_depth--;
                        if (coll_depth == 0) {
                            bitpos = 0;
                            usage = 0;
                            report_count = 0;
                        }
                        DPRINTF("%.*s%s", coll_depth, spaces, "END COLL");
                        break;
                }
                usage_count = 0;
                break;
            case HID_ITEM_TYPE_GLOBAL:  // type 1
                switch (tag) {
                    case HID_GLOBAL_ITEM_TAG_USAGE_PAGE:
                        usage_page = desc[pos];
                        DPRINTF("%.*s%s", coll_depth, spaces, "USAGE PAGE");
                        switch (usage_page) {
                            case HID_USAGE_PAGE_GEN_DES:
                                /* Mouse X, Y */
                                DPRINTF(" Generic Desktop");
                                break;
                            case HID_USAGE_PAGE_GAME_CTR:
                                DPRINTF(" Game Controller");
                                break;
                            case HID_USAGE_PAGE_KEYB:
                                DPRINTF(" Keyboard");
                                break;
                            case HID_USAGE_PAGE_LED:
                                DPRINTF(" LED");
                                break;
                            case HID_USAGE_PAGE_BUTTON:
                                DPRINTF(" Button");
                                break;
                            case HID_USAGE_PAGE_CONSUMER:
                                DPRINTF(" Consumer");
                                break;
                            case HID_USAGE_PAGE_BARCODE:
                                DPRINTF(" Barcode");
                                break;
                        }
                        break;
                    case HID_GLOBAL_ITEM_TAG_LOG_MIN:
                        DPRINTF("%.*s%s", coll_depth, spaces, "LOG MIN");
                        log_min = value;
                        break;
                    case HID_GLOBAL_ITEM_TAG_LOG_MAX:
                        DPRINTF("%.*s%s", coll_depth, spaces, "LOG MAX");
                        log_max = value;
                        break;
                    case HID_GLOBAL_ITEM_TAG_PHY_MIN:
                        DPRINTF("%.*s%s", coll_depth, spaces, "PHY MIN");
                        break;
                    case HID_GLOBAL_ITEM_TAG_PHY_MAX:
                        DPRINTF("%.*s%s", coll_depth, spaces, "PHY MAX");
                        break;
                    case HID_GLOBAL_ITEM_TAG_UNIT_EXPONENT:
                        DPRINTF("%.*s%s", coll_depth, spaces, "UNIT EXP");
                        break;
                    case HID_GLOBAL_ITEM_TAG_UNIT:
                        DPRINTF("%.*s%s", coll_depth, spaces, "UNIT");
                        break;
                    case HID_GLOBAL_ITEM_TAG_REPORT_SIZE:
                        report_size = desc[pos];  // bits per report
                        DPRINTF("%.*s%s", coll_depth, spaces, "REP SIZE");
                        DPRINTF("=%u bits", report_size);
                        break;
                    case HID_GLOBAL_ITEM_TAG_REPORT_ID:
                        DPRINTF("%.*s%s", coll_depth, spaces, "REP ID");
                        DPRINTF("=%u", desc[pos]);
                        report_id = desc[pos];
                        switch (usage_page) {
                            case HID_USAGE_PAGE_GEN_DES:
                                switch (usage) {
                                    case HID_USAGE_POINTER:
                                    case HID_USAGE_MOUSE:
                                        rd->id_mouse = report_id;
                                        break;
                                    case HID_USAGE_SYSCTL:
                                        rd->id_sysctl = report_id;
                                        break;
                                }
                                break;
                            case HID_USAGE_PAGE_CONSUMER:
                                rd->id_consumer = report_id;
                                break;
                        }
                        bitpos += 8;
                        break;
                    case HID_GLOBAL_ITEM_TAG_REPORT_COUNT:
                        report_count = desc[pos];  // number of reports
                        DPRINTF("%.*s%s", coll_depth, spaces, "REP COUNT");
                        DPRINTF("=%u", report_count);
                        break;
                    case HID_GLOBAL_ITEM_TAG_PUSH:
                        DPRINTF("%.*s%s", coll_depth, spaces, "PUSH");
                        break;
                    case HID_GLOBAL_ITEM_TAG_POP:
                        DPRINTF("%.*s%s", coll_depth, spaces, "POP");
                        break;
                        break;
                }
                break;
            case HID_ITEM_TYPE_LOCAL:  // type 2
                switch (tag) {
                    case HID_LOCAL_ITEM_TAG_USAGE:
                        DPRINTF("%.*s%s", coll_depth, spaces, "USAGE");
                        usage = value;
                        switch (usage_page) {
                            case HID_USAGE_PAGE_GEN_DES:
                                switch (usage) {
                                    case HID_USAGE_POINTER:
                                        DPRINTF(" Pointer");
                                        break;
                                    case HID_USAGE_MOUSE:
                                        DPRINTF(" Mouse");
                                        break;
                                    case HID_USAGE_JOYSTICK:
                                        DPRINTF(" Joystick");
                                        break;
                                    case HID_USAGE_GAMEPAD:
                                        DPRINTF(" Gamepad");
                                        break;
                                    case HID_USAGE_KBD:
                                        DPRINTF(" Keyboard");
                                        break;
                                    case HID_USAGE_X:
                                        DPRINTF(" X");
                                        break;
                                    case HID_USAGE_Y:
                                        DPRINTF(" Y");
                                        break;
                                    case HID_USAGE_Z:
                                        DPRINTF(" Z");
                                        break;
                                    case HID_USAGE_RX:
                                        DPRINTF(" RX");
                                        break;
                                    case HID_USAGE_RY:
                                        DPRINTF(" RY");
                                        break;
                                    case HID_USAGE_RZ:
                                        DPRINTF(" RZ");
                                        break;
                                    case HID_USAGE_WHEEL:
                                        DPRINTF(" WHEEL");
                                        break;
                                    case HID_USAGE_SYSCTL:
                                        DPRINTF(" SYSCTL");
                                        break;
                                    case HID_USAGE_PWDOWN:
                                        DPRINTF(" PWDOWN");
                                        break;
                                    case HID_USAGE_SLEEP:
                                        DPRINTF(" SLEEP");
                                        break;
                                    case HID_USAGE_WAKEUP:
                                        DPRINTF(" WAKEUP");
                                        break;
                                }
                                break;
                            case HID_USAGE_PAGE_CONSUMER:
                                switch (usage) {
                                    case 0x01:
                                        DPRINTF(" Control");
                                        break;
                                    case HID_USAGE_AC_PAN:
                                        DPRINTF(" AC_PAN");
                                        break;
                                }
                                break;
                        }
                        if (coll_depth == 0) {
                            if (rd->usage == 0)
                                rd->usage = usage;
                        } else {
                            if (usage_count < ARRAY_SIZE(usage_array))
                                usage_array[usage_count++] = usage;
                        }
                        break;
                    case HID_LOCAL_ITEM_TAG_USAGE_MIN:
                        DPRINTF("%.*s%s", coll_depth, spaces, "USAGEMIN");
                        break;
                    case HID_LOCAL_ITEM_TAG_USAGE_MAX:
                        DPRINTF("%.*s%s", coll_depth, spaces, "USAGEMAX");
                        break;
                    case HID_LOCAL_ITEM_TAG_DESIGNATOR_INDEX:
                        DPRINTF("%.*s%s", coll_depth, spaces, "DES INDEX");
                        break;
                    case HID_LOCAL_ITEM_TAG_DESIGNATOR_MIN:
                        DPRINTF("%.*s%s", coll_depth, spaces, "DES MIN");
                        break;
                    case HID_LOCAL_ITEM_TAG_DESIGNATOR_MAX:
                        DPRINTF("%.*s%s", coll_depth, spaces, "DES MAX");
                        break;
                    case HID_LOCAL_ITEM_TAG_STRING_INDEX:
                    case HID_LOCAL_ITEM_TAG_STRING_MIN:
                    case HID_LOCAL_ITEM_TAG_STRING_MAX:
                    case HID_LOCAL_ITEM_TAG_DELIMITER:
                        DPRINTF("%.*s%s", coll_depth, spaces, "DELIM");
                        break;
                        break;
                }
                break;
            case HID_ITEM_TYPE_RESERVED:  // type 3
                break;
        }
        DPRINTF("\n");
        pos += size;
    }
}


void timer_delay_msec(uint msec);
#define BIT(x) (1U << (x))


static USBH_StatusTypeDef USBH_HID_InterfaceInit_ll(USBH_HandleTypeDef *phost, uint8_t interface)
{
  USBH_StatusTypeDef status;
  HID_HandleTypeDef *HID_Handle;
  uint8_t max_ep;
  uint8_t num = 0U;

  status = USBH_SelectInterface(phost, interface);

  if (status != USBH_OK)
  {
    return USBH_FAIL;
  }

  HID_Handle = (HID_HandleTypeDef *)USBH_malloc(sizeof(HID_HandleTypeDef));
  if (HID_Handle == NULL)
  {
    USBH_DbgLog("Cannot allocate memory for HID Handle");
    return USBH_FAIL;
  }

  /* Initialize hid handler */
  USBH_memset(HID_Handle, 0, sizeof(HID_HandleTypeDef));

#if 0
  /* pData list in reverse order */
  HID_Handle->next = phost->pActiveClass->pData;
  phost->pActiveClass->pData = HID_Handle;
#else
  /*
   * pData list must be in insertion order because some HID devices
   * such as the Dell USB Hub Keyboard require that interfaces be
   * processed in order.
   */
  HID_HandleTypeDef *prev = (HID_HandleTypeDef *) phost->pActiveClass->pData;

  HID_Handle->next = NULL;

  if (prev == NULL) {
    phost->pActiveClass->pData = HID_Handle;
  } else {
    while (prev->next != NULL)
        prev = prev->next;
    prev->next = HID_Handle;
  }
#endif

  HID_Handle->interface = interface;
  HID_Handle->state = HID_NO_SUPPORT;

  /* Decode Bootclass Protocol: Mouse or Keyboard */
  if (phost->device.CfgDesc.Itf_Desc[interface].bInterfaceProtocol == HID_KEYBRD_BOOT_CODE)
  {
    USBH_UsrLog("USB%u.%u.%u Keyboard device found", get_port(phost), phost->address, interface);
    HID_Handle->Init = USBH_HID_KeybdInit;
  }
  else if (phost->device.CfgDesc.Itf_Desc[interface].bInterfaceProtocol == HID_MOUSE_BOOT_CODE)
  {
    USBH_UsrLog("USB%u.%u.%u Mouse device found", get_port(phost), phost->address, interface);
    HID_Handle->Init = USBH_HID_MouseInit;
  }
  else
  {
    USBH_UsrLog("USB%u.%u.%u Generic device found", get_port(phost), phost->address, interface);
#if 0
    USBH_UsrLog("Protocol not supported.");
    return USBH_FAIL;
#endif
    HID_Handle->Init = USBH_HID_GenericInit;
  }

  HID_Handle->state     = HID_INIT;
  HID_Handle->ctl_state = HID_REQ_INIT;
  HID_Handle->ep_addr   = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[0].bEndpointAddress;
  HID_Handle->length    = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[0].wMaxPacketSize;
  HID_Handle->length_max = phost->device.DevDesc.bMaxPacketSize;
  HID_Handle->poll      = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[0].bInterval;

  if (HID_Handle->poll  < HID_MIN_POLL)
  {
    HID_Handle->poll = HID_MIN_POLL;
  }

  /* Check fo available number of endpoints */
  /* Find the number of EPs in the Interface Descriptor */
  /* Choose the lower number in order not to overrun the buffer allocated */
  max_ep = ((phost->device.CfgDesc.Itf_Desc[interface].bNumEndpoints <= USBH_MAX_NUM_ENDPOINTS) ?
             phost->device.CfgDesc.Itf_Desc[interface].bNumEndpoints : USBH_MAX_NUM_ENDPOINTS);

#if 0
  printf("HID IF %x InEp=%x InPipe=%x OutEp=%x OutPipe=%x\n",
          HID_Handle->interface, HID_Handle->InEp, HID_Handle->InPipe,
          HID_Handle->OutEp, HID_Handle->OutPipe);
#endif

  /* Decode endpoint IN and OUT address from interface descriptor */
  for (num = 0U; num < max_ep; num++)
  {
    if (phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[num].bEndpointAddress & 0x80U)
    {
      HID_Handle->InEp = (phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[num].bEndpointAddress);
      HID_Handle->InPipe = USBH_AllocPipe(phost, HID_Handle->InEp);

      /* Open pipe for IN endpoint */
      USBH_OpenPipe(phost, HID_Handle->InPipe, HID_Handle->InEp, phost->device.address,
                    phost->device.speed, USB_EP_TYPE_INTR, HID_Handle->length_max);

      USBH_LL_SetToggle(phost, HID_Handle->InPipe, 0U);
    }
    else
    {
      HID_Handle->OutEp = (phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[num].bEndpointAddress);
      HID_Handle->OutPipe  = USBH_AllocPipe(phost, HID_Handle->OutEp);

      /* Open pipe for OUT endpoint */
      USBH_OpenPipe(phost, HID_Handle->OutPipe, HID_Handle->OutEp, phost->device.address,
                    phost->device.speed, USB_EP_TYPE_INTR, HID_Handle->length);

      USBH_LL_SetToggle(phost, HID_Handle->OutPipe, 0U);
    }
  }

  return USBH_OK;
}

/**
  * @brief  USBH_HID_InterfaceInit
  *         The function init the HID class.
  * @param  phost: Host handle
  * @retval USBH Status
  */
static USBH_StatusTypeDef USBH_HID_InterfaceInit(USBH_HandleTypeDef *phost)
{
  uint8_t interface;
  USBH_StatusTypeDef status = USBH_FAIL;
  USBH_StatusTypeDef t_status;
  uint numif = phost->device.CfgDesc.bNumInterfaces;

  if (numif > USBH_MAX_NUM_INTERFACES)
      numif = USBH_MAX_NUM_INTERFACES;
  for (interface = 0; interface < numif; interface++) {
      t_status = USBH_HID_InterfaceInit_ll(phost, interface);
      if (t_status == USBH_OK)
          status = t_status;
  }

  if (status == USBH_FAIL) { /* No Valid Interface */
    USBH_DbgLog("Cannot Find the interface for %s class.", phost->pActiveClass->Name);
  }
  return (status);
}

static USBH_StatusTypeDef USBH_HID_InterfaceDeInit_ll(USBH_HandleTypeDef *phost)
{
  HID_HandleTypeDef *HID_Handle = (HID_HandleTypeDef *) phost->pActiveClass->pData;

  if (HID_Handle == NULL)
    return USBH_FAIL;

  if (HID_Handle->InPipe != 0x00U)
  {
    USBH_LL_StopHC(phost, HID_Handle->InPipe);

    USBH_ClosePipe(phost, HID_Handle->InPipe);
    USBH_FreePipe(phost, HID_Handle->InPipe);
    HID_Handle->InPipe = 0U;     /* Reset the pipe as Free */
  }

  if (HID_Handle->OutPipe != 0x00U)
  {
    USBH_LL_StopHC(phost, HID_Handle->OutPipe);

    USBH_ClosePipe(phost, HID_Handle->OutPipe);
    USBH_FreePipe(phost, HID_Handle->OutPipe);
    HID_Handle->OutPipe = 0U;     /* Reset the pipe as Free */
  }

  phost->pActiveClass->pData = HID_Handle->next;
  USBH_free(HID_Handle);

  return USBH_OK;
}

/**
  * @brief  USBH_HID_InterfaceDeInit
  *         The function DeInit the Pipes used for the HID class.
  * @param  phost: Host handle
  * @retval USBH Status
  */
static USBH_StatusTypeDef USBH_HID_InterfaceDeInit(USBH_HandleTypeDef *phost)
{
  USBH_StatusTypeDef status = USBH_FAIL;

  while (phost->pActiveClass->pData != NULL)
    status = USBH_HID_InterfaceDeInit_ll(phost);

  return status;
}

/**
  * @brief  USBH_HID_ClassRequest_ll
  *         The function is responsible for handling Standard requests
  *         for HID class.
  * @param  phost: Host handle
  * @retval USBH Status
  */
static USBH_StatusTypeDef USBH_HID_ClassRequest_ll(USBH_HandleTypeDef *phost, HID_HandleTypeDef *HID_Handle)
{
  USBH_StatusTypeDef status = USBH_BUSY;
  USBH_StatusTypeDef tstatus;

  /* Switch HID state machine */
  switch (HID_Handle->ctl_state)
  {
    case HID_REQ_INIT:
        HID_Handle->ctl_state = HID_REQ_GET_HID_DESC;
        HID_Handle->timer = phost->Timer;
        break;
    case HID_REQ_GET_HID_DESC:
      /* Get HID Desc */
      tstatus = USBH_HID_GetHIDDescriptor(phost, HID_Handle->interface, USB_HID_DESC_SIZE);
      if (tstatus == USBH_OK) {
        USBH_HID_ParseHIDDesc(&HID_Handle->HID_Desc, phost->device.Data);
        HID_Handle->ctl_state = HID_REQ_GET_REPORT_DESC;
        HID_Handle->error_count = 0;

      } else if (tstatus == USBH_BUSY) {
        if (phost->Timer - HID_Handle->timer > 3000)
            goto get_hid_descriptor_failed;

        if ((phost->Timer - HID_Handle->timer > 1000) &&
            (HID_Handle->error_count == 0)) {
          HID_Handle->error_count++;
          printf("USB%u.%u.%u busy too long for get HID descriptor\n",
                 get_port(phost), phost->address, HID_Handle->interface);

          /* Try again */
          phost->RequestState = CMD_SEND;
          HID_Handle->ctl_state = HID_REQ_GET_HID_DESC;
        }
      } else {
get_hid_descriptor_failed:
        printf("USB%u.%u.%u failed get HID descriptor\n",
               get_port(phost), phost->address, HID_Handle->interface);

        /* Assume keyboard */
        HID_Handle->HID_Desc.bLength                  = 0x0009;
        HID_Handle->HID_Desc.bDescriptorType          = 0x21;
        HID_Handle->HID_Desc.bcdHID                   = 0x0110;
        HID_Handle->HID_Desc.bCountryCode             = 0x00;
        HID_Handle->HID_Desc.bNumDescriptors          = 0x01;
        HID_Handle->HID_Desc.bReportDescriptorType    = 0x22;
        HID_Handle->HID_Desc.wItemLength              = 0x0045;

        HID_Handle->ctl_state = HID_REQ_GET_REPORT_DESC;
      }

      break;
    case HID_REQ_GET_REPORT_DESC:
      /* Get Report Desc */
      tstatus = USBH_HID_GetHIDReportDescriptor(phost, HID_Handle->interface, HID_Handle->HID_Desc.wItemLength);
      if (tstatus == USBH_OK) {
        /* The descriptor is available in phost->device.Data */
        USBH_HID_Process_HIDReportDescriptor(phost, HID_Handle);
        HID_Handle->ctl_state = HID_REQ_SET_IDLE;

      } else if (tstatus != USBH_BUSY) {
        printf("USB%u.%u.%u failed get HID report descriptor\n",
               get_port(phost), phost->address, HID_Handle->interface);
        HID_Handle->ctl_state = HID_REQ_SET_IDLE;
      }
      break;

    case HID_REQ_SET_IDLE:
      /* set Idle */
      tstatus = USBH_HID_SetIdle(phost, 0U, 0U);
      if ((tstatus == USBH_OK) || (tstatus == USBH_NOT_SUPPORTED)) {
        HID_Handle->ctl_state = HID_REQ_SET_PROTOCOL;

      } else if (tstatus != USBH_BUSY) {
        printf("USB%u.%u.%u failed setidle\n",
               get_port(phost), phost->address, HID_Handle->interface);
        HID_Handle->ctl_state = HID_REQ_SET_PROTOCOL;
      }
      break;

    case HID_REQ_SET_PROTOCOL:
      /* set report protocol */
      status = USBH_HID_SetProtocol(phost, 1U);
      if ((status == USBH_OK) || (status == USBH_NOT_SUPPORTED)) {
        HID_Handle->ctl_state = HID_REQ_IDLE;

        /* all requests performed */
        phost->pUser(phost, HOST_USER_CLASS_ACTIVE);
        status = USBH_OK;

      } else if ((status != USBH_BUSY)) {
        printf("USB%u.%u.%u failed setprotocol\n",
               get_port(phost), phost->address, HID_Handle->interface);
        HID_Handle->ctl_state = HID_REQ_IDLE;

        /* all requests performed */
        phost->pUser(phost, HOST_USER_CLASS_ACTIVE);
        status = USBH_OK;
      }
      break;

    case HID_REQ_IDLE:
    default:
      break;
  }

  return status;
}

/**
  * @brief  USBH_HID_ClassRequest
  *         The function is responsible for handling Standard requests
  *         for HID class.
  * @param  phost: Host handle
  * @retval USBH Status
  */
static USBH_StatusTypeDef USBH_HID_ClassRequest(USBH_HandleTypeDef *phost)
{
    uint bit = 0;
    USBH_StatusTypeDef status;
    HID_HandleTypeDef *HID_Handle = (HID_HandleTypeDef *) phost->pActiveClass->pData;

    /*
     * Serialize class requests because only one may be active
     * on the USB host at any time.
     */
    while (HID_Handle != NULL) {
        if (phost->iface_waiting) {
            if (phost->iface_waiting & BIT(bit)) {
                status = USBH_HID_ClassRequest_ll(phost, HID_Handle);
                if (status != USBH_BUSY)
                    phost->iface_waiting &= ~BIT(bit);
            }
        } else {
            status = USBH_HID_ClassRequest_ll(phost, HID_Handle);
            if (status == USBH_BUSY) {
                phost->iface_waiting |= BIT(bit);
                break;  // Check again later
            }
        }
        bit++;
        HID_Handle = HID_Handle->next;
    }
    return (phost->iface_waiting ? USBH_BUSY: USBH_OK);
}

static void
USBH_OpenEpPipes(USBH_HandleTypeDef *phost, HID_HandleTypeDef *HID_Handle)
{
    uint8_t interface = HID_Handle->interface;
    USBH_InterfaceDescTypeDef *ifd = &phost->device.CfgDesc.Itf_Desc[interface];
    uint max_ep = (ifd->bNumEndpoints <= USBH_MAX_NUM_ENDPOINTS) ?
                   ifd->bNumEndpoints : USBH_MAX_NUM_ENDPOINTS;

    for (uint num = 0U; num < max_ep; num++) {
      if (ifd->Ep_Desc[num].bEndpointAddress & 0x80U) {
        /* Open pipe for IN endpoint */
        USBH_OpenPipe(phost, HID_Handle->InPipe, HID_Handle->InEp,
                      phost->device.address, phost->device.speed,
                      USB_EP_TYPE_INTR, HID_Handle->length_max);
      } else {
        /* Open pipe for OUT endpoint */
        USBH_OpenPipe(phost, HID_Handle->OutPipe, HID_Handle->OutEp,
                      phost->device.address, phost->device.speed,
                      USB_EP_TYPE_INTR, HID_Handle->length);
      }
    }
}

static USBH_StatusTypeDef USBH_HID_Process_ll(USBH_HandleTypeDef *phost, HID_HandleTypeDef *HID_Handle)
{
  USBH_StatusTypeDef status = USBH_OK;
  uint32_t XferSize;

  switch (HID_Handle->state)
  {
    case HID_INIT:
      status = HID_Handle->Init(phost, HID_Handle);
      if (status != USBH_OK) {
        printf("USB%u.%u.%u HID init failure\n",
               get_port(phost), phost->address, HID_Handle->interface);
        HID_Handle->state = HID_ERROR;
        break;
      }
      HID_Handle->state = HID_VENDOR;
      USBH_OpenEpPipes(phost, HID_Handle);

      USBH_LL_SetToggle(phost, HID_Handle->InPipe, 0U);

#if (USBH_USE_OS == 1U)
      phost->os_msg = (uint32_t)USBH_URB_EVENT;
#if (osCMSIS < 0x20000U)
      (void)osMessagePut(phost->os_event, phost->os_msg, 0U);
#else
      (void)osMessageQueuePut(phost->os_event, &phost->os_msg, 0U, NULL);
#endif
#endif
      break;

    case HID_VENDOR:
      /* Execute vendor-specific code */
      if (HID_Handle->Vendor != NULL)
        status = HID_Handle->Vendor(phost, HID_Handle);

      if (status == USBH_OK) {
        switch (USBH_HID_GetDeviceType(phost, HID_Handle->interface)) {
            case HID_KEYBOARD:
            case HID_MOUSE:
                /*
                 * Some keyboards and mice react badly to GET_REPORT, so
                 * skip it for all of them. The Corsair K55 is one example.
                 */
                HID_Handle->state = HID_SYNC;
                break;
            default:
                HID_Handle->state = HID_GET_REPORT;
                break;
        }
      }
      break;

    case HID_GET_REPORT:
      // HID_Handle pData and length_max are updated in the HID protocol handler
      status = USBH_HID_GetReport(phost, 0x01U, 0U, HID_Handle->pData, (uint8_t)HID_Handle->length_max);
      if (status == USBH_OK)
      {
        HID_Handle->state = HID_SYNC;
      }
      else if (status == USBH_BUSY)
      {
        /* Stay in same state */
      }
      else if (status == USBH_NOT_SUPPORTED)
      {
        PPRINTF(" ->NOSUP");
        HID_Handle->state = HID_SYNC;
        status = USBH_OK;
      }
      else if (status == USBH_TIMEOUT)
      {
        printf("USB%u.%u.%u HID GetReport timeout\n",
               get_port(phost), phost->address, HID_Handle->interface);
        status = USBH_OK;
        HID_Handle->state = HID_SYNC;
      }
      else
      {
        PPRINTF(" ->ERROR");
        if (++HID_Handle->error_count < 5) {
          HID_Handle->state = HID_ERROR;
        } else {
          HID_Handle->state = HID_ERROR;
        }
        status = USBH_FAIL;
      }

#if (USBH_USE_OS == 1U)
      phost->os_msg = (uint32_t)USBH_URB_EVENT;
#if (osCMSIS < 0x20000U)
      (void)osMessagePut(phost->os_event, phost->os_msg, 0U);
#else
      (void)osMessageQueuePut(phost->os_event, &phost->os_msg, 0U, NULL);
#endif
#endif
      break;

    case HID_ERROR:
      /* Terminal state */
      break;

    case HID_SYNC:
      /* Sync with start of Even Frame */
      if (phost->Timer & 1U)
      {
        HID_Handle->state = HID_GET_DATA;
      }

#if (USBH_USE_OS == 1U)
      phost->os_msg = (uint32_t)USBH_URB_EVENT;
#if (osCMSIS < 0x20000U)
      (void)osMessagePut(phost->os_event, phost->os_msg, 0U);
#else
      (void)osMessageQueuePut(phost->os_event, &phost->os_msg, 0U, NULL);
#endif
#endif
      break;

    case HID_GET_DATA: {
      if (USBH_LL_GetURBState(phost, HID_Handle->InPipe) == USBH_URB_DONE) {
        goto do_hid_poll;
      }
      USBH_InterruptReceiveData(phost, HID_Handle->pData,
                                (uint8_t)HID_Handle->length_max,
                                HID_Handle->InPipe);

      HID_Handle->timer = phost->Timer;
      HID_Handle->state = HID_POLL;
      HID_Handle->DataReady = 0U;
      break;
    }

    case HID_POLL:
do_hid_poll:
      if (USBH_LL_GetURBState(phost, HID_Handle->InPipe) == USBH_URB_DONE)
      {
        /* Mark the URB as received */
        USBH_LL_SetURBState(phost, HID_Handle->InPipe, USBH_URB_IDLE);

        XferSize = USBH_LL_GetLastXferSize(phost, HID_Handle->InPipe);

        if ((HID_Handle->DataReady == 0U) && (XferSize != 0U))
        {
          USBH_HID_FifoWrite(&HID_Handle->fifo, HID_Handle->pData, XferSize);
          HID_Handle->DataReady = 1U;
          USBH_HID_EventCallback(phost, HID_Handle);

#if (USBH_USE_OS == 1U)
          phost->os_msg = (uint32_t)USBH_URB_EVENT;
#if (osCMSIS < 0x20000U)
          (void)osMessagePut(phost->os_event, phost->os_msg, 0U);
#else
          (void)osMessageQueuePut(phost->os_event, &phost->os_msg, 0U, NULL);
#endif
#endif
        }
      }
      else
      {
        /* IN Endpoint Stalled */
        if (USBH_LL_GetURBState(phost, HID_Handle->InPipe) == USBH_URB_STALL)
        {
          /* Issue Clear Feature on interrupt IN endpoint */
          if (USBH_ClrFeature(phost, HID_Handle->ep_addr) == USBH_OK)
          {
            /* Change state to issue next IN token */
            HID_Handle->state = HID_GET_DATA;
          }
        }
      }
      break;

    default:
      break;
  }

  return status;
}

/**
  * @brief  USBH_HID_Process
  *         The function is for managing state machine for HID data transfers
  *         It is the background processor for the class (BgndProcess).
  * @param  phost: Host handle
  * @retval USBH Status
  */
static USBH_StatusTypeDef USBH_HID_Process(USBH_HandleTypeDef *phost)
{
    uint bit = 0;
    USBH_StatusTypeDef status;
    HID_HandleTypeDef *HID_Handle =
                       (HID_HandleTypeDef *) phost->pActiveClass->pData;

    while (HID_Handle != NULL) {
        if (phost->iface_waiting) {
            if (phost->iface_waiting & BIT(bit)) {
                status = USBH_HID_Process_ll(phost, HID_Handle);
                if (status != USBH_BUSY)
                    phost->iface_waiting &= ~BIT(bit);
            }
        } else {
            status = USBH_HID_Process_ll(phost, HID_Handle);
            if (status == USBH_BUSY) {
                break;  // Check again later
            }
        }
        bit++;
        HID_Handle = HID_Handle->next;
    }
    return (phost->iface_waiting ? USBH_BUSY: USBH_OK);
}

static USBH_StatusTypeDef USBH_HID_SOFProcess_ll(USBH_HandleTypeDef *phost, HID_HandleTypeDef *HID_Handle)
{
  if (HID_Handle->state == HID_POLL)
  {
    if ((phost->Timer - HID_Handle->timer) >= HID_Handle->poll)
    {
      HID_Handle->state = HID_GET_DATA;

#if (USBH_USE_OS == 1U)
      phost->os_msg = (uint32_t)USBH_URB_EVENT;
#if (osCMSIS < 0x20000U)
      (void)osMessagePut(phost->os_event, phost->os_msg, 0U);
#else
      (void)osMessageQueuePut(phost->os_event, &phost->os_msg, 0U, NULL);
#endif
#endif
    }
  }
  return USBH_OK;
}

/**
  * @brief  USBH_HID_SOFProcess
  *         The function is for managing the SOF Process
  * @param  phost: Host handle
  * @retval USBH Status
  */
static USBH_StatusTypeDef USBH_HID_SOFProcess(USBH_HandleTypeDef *phost)
{
  USBH_StatusTypeDef tstatus;
  USBH_StatusTypeDef status = USBH_OK;
  HID_HandleTypeDef *HID_Handle = (HID_HandleTypeDef *) phost->pActiveClass->pData;

  while (HID_Handle != NULL) {
    tstatus = USBH_HID_SOFProcess_ll(phost, HID_Handle);
    if (status == USBH_OK)
        status = tstatus;
    HID_Handle = HID_Handle->next;
  }
  return status;
}

/**
* @brief  USBH_HID_GetHIDReportDescriptor
  *         Issue report Descriptor command to the device. Once the response
  *         received, parse the report descriptor and update the status.
  * @param  phost: Host handle
  * @param  iface: Which interface on the device
  * @param  Length : HID Report Descriptor Length
  * @retval USBH Status
  */
USBH_StatusTypeDef USBH_HID_GetHIDReportDescriptor(USBH_HandleTypeDef *phost,
                                                   uint16_t iface, uint16_t length)
{

  USBH_StatusTypeDef status;

  status = USBH_GetDescriptor(phost,
                              USB_REQ_RECIPIENT_INTERFACE | USB_REQ_TYPE_STANDARD,
                              USB_DESC_HID_REPORT, iface,
                              phost->device.Data,
                              length);

  /* HID report descriptor is available in phost->device.Data.
  In case of USB Boot Mode devices for In report handling ,
  HID report descriptor parsing is not required.
  In case, for supporting Non-Boot Protocol devices and output reports,
  user may parse the report descriptor*/


  return status;
}


/**
  * @brief  USBH_Get_HID_Descriptor
  *         Issue HID Descriptor command to the device. Once the response
  *         received, parse the report descriptor and update the status.
  * @param  phost: Host handle
  * @param  iface: Which interface on the device
  * @param  Length : HID Descriptor Length
  * @retval USBH Status
  */
USBH_StatusTypeDef USBH_HID_GetHIDDescriptor(USBH_HandleTypeDef *phost,
                                             uint16_t iface, uint16_t length)
{
  USBH_StatusTypeDef status;

  status = USBH_GetDescriptor(phost,
                              USB_REQ_RECIPIENT_INTERFACE | USB_REQ_TYPE_STANDARD,
                              USB_DESC_HID, iface,
                              phost->device.Data,
                              length);

  return status;
}

/**
  * @brief  USBH_Set_Idle
  *         Set Idle State.
  * @param  phost: Host handle
  * @param  duration: Duration for HID Idle request
  * @param  reportId : Targeted report ID for Set Idle request
  * @retval USBH Status
  */
USBH_StatusTypeDef USBH_HID_SetIdle(USBH_HandleTypeDef *phost,
                                    uint8_t duration,
                                    uint8_t reportId)
{

  phost->Control.setup.b.bmRequestType = USB_H2D | USB_REQ_RECIPIENT_INTERFACE | \
                                         USB_REQ_TYPE_CLASS;


  phost->Control.setup.b.bRequest = USB_HID_SET_IDLE;
  phost->Control.setup.b.wValue.w = (uint16_t)(((uint32_t)duration << 8U) | (uint32_t)reportId);

  phost->Control.setup.b.wIndex.w = 0U;
  phost->Control.setup.b.wLength.w = 0U;

  PPRINTF("SetIdle\n");
  return USBH_CtlReq(phost, 0U, 0U);
}


/**
  * @brief  USBH_HID_Set_Report
  *         Issues Set Report
  * @param  phost: Host handle
  * @param  reportType  : Report type to be sent
  * @param  reportId    : Targeted report ID for Set Report request
  * @param  reportBuff  : Report Buffer
  * @param  reportLen   : Length of data report to be send
  * @retval USBH Status
  */
USBH_StatusTypeDef USBH_HID_SetReport(USBH_HandleTypeDef *phost,
                                      uint8_t reportType,
                                      uint8_t reportId,
                                      uint8_t *reportBuff,
                                      uint8_t reportLen)
{

  phost->Control.setup.b.bmRequestType = USB_H2D | USB_REQ_RECIPIENT_INTERFACE | \
                                         USB_REQ_TYPE_CLASS;


  phost->Control.setup.b.bRequest = USB_HID_SET_REPORT;
  phost->Control.setup.b.wValue.w = (uint16_t)(((uint32_t)reportType << 8U) | (uint32_t)reportId);

  phost->Control.setup.b.wIndex.w = 0U;
  phost->Control.setup.b.wLength.w = reportLen;

  PPRINTF("SetReport\n");
  return USBH_CtlReq(phost, reportBuff, (uint16_t)reportLen);
}


/**
  * @brief  USBH_HID_GetReport
  *         retreive Set Report
  * @param  phost: Host handle
  * @param  reportType  : Report type to be sent
  * @param  reportId    : Targeted report ID for Set Report request
  * @param  reportBuff  : Report Buffer
  * @param  reportLen   : Length of data report to be send
  * @retval USBH Status
  */
USBH_StatusTypeDef USBH_HID_GetReport(USBH_HandleTypeDef *phost,
                                      uint8_t reportType,
                                      uint8_t reportId,
                                      uint8_t *reportBuff,
                                      uint8_t reportLen)
{

  phost->Control.setup.b.bmRequestType = USB_D2H | USB_REQ_RECIPIENT_INTERFACE | \
                                         USB_REQ_TYPE_CLASS;


  phost->Control.setup.b.bRequest = USB_HID_GET_REPORT;
  phost->Control.setup.b.wValue.w = (uint16_t)(((uint32_t)reportType << 8U) | (uint32_t)reportId);

  phost->Control.setup.b.wIndex.w = 0U;
  phost->Control.setup.b.wLength.w = reportLen;

  return USBH_CtlReq(phost, reportBuff, (uint16_t)reportLen);
}

/**
  * @brief  USBH_Set_Protocol
  *         Set protocol State.
  * @param  phost: Host handle
  * @param  protocol : Set Protocol for HID : boot/report protocol
  * @retval USBH Status
  */
USBH_StatusTypeDef USBH_HID_SetProtocol(USBH_HandleTypeDef *phost,
                                        uint8_t protocol)
{
  phost->Control.setup.b.bmRequestType = USB_H2D | USB_REQ_RECIPIENT_INTERFACE
                                         | USB_REQ_TYPE_CLASS;

  phost->Control.setup.b.bRequest = USB_HID_SET_PROTOCOL;
  phost->Control.setup.b.wValue.w = protocol;

  phost->Control.setup.b.wIndex.w = 0U;
  phost->Control.setup.b.wLength.w = 0U;

  PPRINTF("SetProtocol %u\n", protocol);
  return USBH_CtlReq(phost, 0U, 0U);

}

/**
  * @brief  USBH_ParseHIDDesc
  *         This function Parse the HID descriptor
  * @param  desc: HID Descriptor
  * @param  buf: Buffer where the source descriptor is available
  * @retval None
  */
static void  USBH_HID_ParseHIDDesc(HID_DescTypeDef *desc, uint8_t *buf)
{

  desc->bLength                  = *(uint8_t *)(buf + 0);
  desc->bDescriptorType          = *(uint8_t *)(buf + 1);
  desc->bcdHID                   =  LE16(buf + 2);
  desc->bCountryCode             = *(uint8_t *)(buf + 4);
  desc->bNumDescriptors          = *(uint8_t *)(buf + 5);
  desc->bReportDescriptorType    = *(uint8_t *)(buf + 6);
  desc->wItemLength              =  LE16(buf + 7);
}

/**
  * @brief  USBH_HID_GetDeviceType
  *         Return Device function.
  * @param  phost: Host handle
  * @param  iface: Which interface on the device
  * @retval HID function: HID_MOUSE / HID_KEYBOARD
  */
HID_TypeTypeDef USBH_HID_GetDeviceType(USBH_HandleTypeDef *phost, uint16_t iface)
{
  HID_TypeTypeDef   type = HID_UNKNOWN;
  uint8_t InterfaceProtocol;

  if (phost->gState == HOST_CLASS)
  {
    InterfaceProtocol = phost->device.CfgDesc.Itf_Desc[iface].bInterfaceProtocol;
    if (InterfaceProtocol == HID_KEYBRD_BOOT_CODE) {
      type = HID_KEYBOARD;
    } else if (InterfaceProtocol == HID_MOUSE_BOOT_CODE) {
      type = HID_MOUSE;
    }
  }
  return type;
}


#if 0
/**
  * @brief  USBH_HID_GetPollInterval
  *         Return HID device poll time
  * @param  phost: Host handle
  * @retval poll time (ms)
  */
uint8_t USBH_HID_GetPollInterval(USBH_HandleTypeDef *phost)
{
  uint8_t interval = 0U;
  HID_HandleTypeDef *HID_Handle = (HID_HandleTypeDef *) phost->pActiveClass->pData;

  if ((phost->gState == HOST_CLASS_REQUEST) ||
      (phost->gState == HOST_INPUT) ||
      (phost->gState == HOST_SET_CONFIGURATION) ||
      (phost->gState == HOST_CHECK_CLASS) ||
      ((phost->gState == HOST_CLASS)))
  {
    while (HID_Handle != NULL) {
      if ((interval == 0) || (interval > HID_Handle->poll))
        interval = HID_Handle->poll;
      HID_Handle = HID_Handle->next;
    }
  }
  return (interval);
}
#endif
/**
  * @brief  USBH_HID_FifoInit
  *         Initialize FIFO.
  * @param  f: Fifo address
  * @param  buf: Fifo buffer
  * @param  size: Fifo Size
  * @retval none
  */
void USBH_HID_FifoInit(FIFO_TypeDef *f, uint8_t *buf, uint16_t size)
{
  f->head = 0U;
  f->tail = 0U;
  f->lock = 0U;
  f->size = size;
  f->buf = buf;
}

void USBH_HID_FifoFlush(FIFO_TypeDef *f)
{
  f->head = 0U;
  f->tail = 0U;
  f->lock = 0U;
}

/**
  * @brief  USBH_HID_FifoRead
  *         Read from FIFO.
  * @param  f: Fifo address
  * @param  buf: read buffer
  * @param  nbytes: number of item to read
  * @retval number of read items
  */
uint16_t USBH_HID_FifoRead(FIFO_TypeDef *f, void *buf, uint16_t nbytes)
{
  uint16_t i;
  uint8_t *p;

  p = (uint8_t *) buf;

  if (f->lock == 0U)
  {
    f->lock = 1U;

    for (i = 0U; i < nbytes; i++)
    {
      if (f->tail != f->head)
      {
        *p++ = f->buf[f->tail];
        f->tail++;

        if (f->tail == f->size)
        {
          f->tail = 0U;
        }
      }
      else
      {
        f->lock = 0U;
        return i;
      }
    }
  }

  f->lock = 0U;

  return nbytes;
}

/**
  * @brief  USBH_HID_FifoWrite
  *         Write To FIFO.
  * @param  f: Fifo address
  * @param  buf: read buffer
  * @param  nbytes: number of item to write
  * @retval number of written items
  */
uint16_t USBH_HID_FifoWrite(FIFO_TypeDef *f, void *buf, uint16_t  nbytes)
{
  uint16_t i;
  uint8_t *p;

  p = (uint8_t *) buf;

  if (f->lock == 0U)
  {
    f->lock = 1U;

    for (i = 0U; i < nbytes; i++)
    {
      if ((f->head + 1U == f->tail) ||
          ((f->head + 1U == f->size) && (f->tail == 0U)))
      {
        f->lock = 0U;
        return i;
      }
      else
      {
        f->buf[f->head] = *p++;
        f->head++;

        if (f->head == f->size)
        {
          f->head = 0U;
        }
      }
    }
  }

  f->lock = 0U;

  return nbytes;
}

/**
* @brief  The function is a callback about HID Data events
*  @param  phost: Selected device
* @retval None
*/
__weak void USBH_HID_EventCallback(USBH_HandleTypeDef *phost, HID_HandleTypeDef *HID_Handle)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(phost);
  UNUSED(HID_Handle);
}
/**
* @}
*/

/**
* @}
*/

/**
* @}
*/


/**
* @}
*/


/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

#include "config.h"
#include "usbh_hid_mouse.h"

static int
readbits(void *ptr, uint startbit, uint bits, uint is_signed)
{
    uint byte = startbit / 8;
    uint bitoff = startbit % 8;
    uint mask = BIT(bits) - 1;
    uint val = ((*(uint *) ((uintptr_t) ptr + byte)) >> bitoff) & mask;

    if (is_signed &&
        (val & BIT(bits - 1))) {
        val |= (0 - BIT(bits));  // Sign-extend negative
    }

    return (val);
}

/*
 * readbits_joy() converts joystick values from the retronicdesign.com
 *                Atari C64 Amiga Joystick v3.2       ID e501.0810
 */
static int
readbits_joy(void *ptr, uint startbit, uint bits, int offset)
{
    int val = readbits(ptr, startbit, bits, 0) + offset;
    int range = abs(offset) / 4;
    if (val < 0 - range)
        val = -1;
    else if (val > 0 + range)
        val = 1;
    else
        val = 0;
#if 0
    switch (val) {
        case 0:             // 0x00
            val = -1;
            break;
        default:
        case  127:          // 0x7f
        case -128:          // 0x80
            val = 0;
            break;
        case -1:            // 0xff
            val = 1;
            break;
    }
#endif
    return (val);
}

static uint8_t
readbits_jpad(void *ptr, uint16_t *startbits)
{
    uint pos;
    uint jpad = 0;
    for (pos = 0; pos < 4; pos++)
        jpad |= ((!!readbits(ptr, startbits[pos], 1, 0)) << pos);
    return (jpad);
}


static uint32_t
readbits_buttons(void *ptr, HID_RDescTypeDef *rd)
{
    uint button;
    uint32_t buttons = 0;
    uint num_buttons = rd->num_buttons;

    if (num_buttons > ARRAY_SIZE(rd->pos_button))
        num_buttons = ARRAY_SIZE(rd->pos_button);

    for (button = 0; button < num_buttons; button++) {
        if ((button < ARRAY_SIZE(rd->pos_button)) &&
            readbits(ptr, rd->pos_button[button], 1, 0)) {
            buttons |= BIT(button);
        }
    }
    return (buttons);
}

/**
  * @brief  USBH_HID_DecodeReport
  *         The function gets and decodes mouse and generic data.
  * @param  phost: Host handle
  * @retval USBH Status
  */
USBH_StatusTypeDef
USBH_HID_DecodeReport(USBH_HandleTypeDef *phost, HID_HandleTypeDef *HID_Handle, HID_TypeTypeDef devtype, HID_MISC_Info_TypeDef *report_info)
{
    HID_RDescTypeDef *rd = &HID_Handle->HID_RDesc;
    uint32_t          report_data[4];
    static uint32_t   report_last[ARRAY_SIZE(report_data)];
    uint16_t          len = sizeof (report_data);
    uint button;
    uint rlen;

    if (HID_Handle == NULL)
        return USBH_FAIL;

    if (HID_Handle->length == 0U)
        return USBH_FAIL;

    if (len > HID_Handle->length)
        len = HID_Handle->length;
    memset(&report_data, 0, sizeof (report_data));
    rlen = USBH_HID_FifoRead(&HID_Handle->fifo, &report_data, len);
    USBH_HID_FifoFlush(&HID_Handle->fifo);  // Discard excess data

    /* Fill report */
    if (rlen > 0) {
        uint cur;
        uint8_t id = report_data[0];
        memset(report_info, 0, sizeof (*report_info));
        report_info->usage = rd->usage;

        if (((rd->id_mouse != 0) && (rd->id_mouse == id)) ||
            ((rd->id_mouse == 0) && (devtype == HID_MOUSE))) {
            int16_t x;
            int16_t y;
            int     pdiv_x;
            int     pdiv_y;
            int     mul_x;
            int     div_x;
            int     mul_y;
            int     div_y;
            int     calc;
            static  int accum_x;
            static  int accum_y;
is_mouse:
            dprintf(DF_USB_DECODE_MOUSE, "\n%08lx %08lx ",
                    report_data[0], report_data[1]);

            report_info->buttons = readbits_buttons(report_data, rd);
            x = readbits(report_data, rd->pos_x, rd->bits_x, 1);
            y = readbits(report_data, rd->pos_y, rd->bits_y, 1);
            if (rd->dev_flag & DEV_FLAG_ABSOLUTE) {
                static int16_t x_last;
                static int16_t y_last;
                int16_t        temp;
                temp = x;
                x -= x_last;
                x_last = temp;

                temp = y;
                y -= y_last;
                y_last = temp;

                mul_x = config.amouse_mul_x;
                div_x = config.amouse_div_x;
                mul_y = config.amouse_mul_y;
                div_y = config.amouse_div_y;
                if (div_x == 0)
                    div_x = 4;
                if (div_y == 0)
                    div_y = 4;

                /* Assume "tablet" has higher resolution than screen */
                pdiv_x = 8;
                pdiv_y = 16;
            } else {
                mul_x = config.rmouse_mul_x;
                div_x = config.rmouse_div_x;
                mul_y = config.rmouse_mul_y;
                div_y = config.rmouse_div_y;
                if (div_x == 0)
                    div_x = 2;
                if (div_y == 0)
                    div_y = 2;

                /* Assume relative mouse has same resolution as screen */
                pdiv_x = 1;
                pdiv_y = 1;
            }
            if (mul_x == 0)
                mul_x = 1;
            if (mul_y == 0)
                mul_y = 1;

            /*
             * The below code handles fractional mouse movements by
             * remembering the remainder from previous mouse input.
             */
            accum_x       += ((int) x) * mul_x;
            calc           = accum_x / (pdiv_x * div_x);
            accum_x       -= (calc * div_x);
            if (calc < INT8_MIN)
                calc = INT8_MIN;
            else if (calc > INT8_MAX)
                calc = INT8_MAX;
            if (accum_x < INT8_MIN)
                accum_x = INT8_MIN;
            else if (accum_x > INT8_MAX)
                accum_x = INT8_MAX;
            report_info->x = calc;

            accum_y       += ((int) y) * mul_y;
            calc           = accum_y / (pdiv_y * div_y);
            accum_y       -= (calc * div_y);
            if (calc < INT8_MIN)
                calc = INT8_MIN;
            else if (calc > INT8_MAX)
                calc = INT8_MAX;
            if (accum_y < INT8_MIN)
                accum_y = INT8_MIN;
            else if (accum_y > INT8_MAX)
                accum_y = INT8_MAX;
            report_info->y = calc;

            if (rd->bits_wheel != 0) {
                report_info->wheel = readbits(report_data, rd->pos_wheel,
                                              rd->bits_wheel, 1);
            }
            if (rd->bits_ac_pan != 0) {
                report_info->ac_pan = readbits(report_data, rd->pos_ac_pan,
                                               rd->bits_ac_pan, 1);
            }
        } else if ((rd->id_consumer != 0) && (rd->id_consumer == id)) {
            dprintf(DF_USB_DECODE_MISC, "mmkey");
            for (cur = 0; cur < rd->num_keys; cur++) {
                if (rd->pos_key[cur] == 0)
                    continue;
                report_info->mm_key[cur] =
                    readbits(report_data, rd->pos_key[cur], rd->bits_key, 1);
                dprintf(DF_USB_DECODE_MISC, " %02x", report_info->mm_key[cur]);
            }
        } else if ((rd->id_sysctl != 0) && (rd->id_sysctl == id)) {
            report_info->sysctl = readbits(report_data,
                                           rd->pos_sysctl, rd->bits_sysctl, 0);
            dprintf(DF_USB_DECODE_MISC, "Sysctl %x", report_info->sysctl);
        } else if ((rd->id_mouse == 0) && (rd->usage == HID_USAGE_MOUSE)) {
            goto is_mouse;
        } else if ((rd->usage == HID_USAGE_JOYSTICK) ||
                   (rd->usage == HID_USAGE_GAMEPAD)) {
            report_info->buttons = readbits_buttons(report_data, rd);
            report_info->x = readbits_joy(report_data, rd->pos_x,
                                          rd->bits_x, rd->offset_xy);
            report_info->y = readbits_joy(report_data, rd->pos_y,
                                          rd->bits_y, rd->offset_xy);
            if (rd->pos_jpad[0] != 0) {
                report_info->flags |= MI_FLAG_HAS_JPAD;  // Has joypad
                report_info->jpad = readbits_jpad(report_data,
                                                  rd->pos_jpad);
            }
            if (rd->bits_ac_pan != 0) {
                report_info->ac_pan = readbits_joy(report_data,
                                                   rd->pos_ac_pan,
                                                   rd->bits_ac_pan,
                                                   rd->offset_xy);
            }
            if (rd->bits_wheel != 0) {
                report_info->wheel = readbits_joy(report_data,
                                                  rd->pos_wheel,
                                                  rd->bits_wheel,
                                                  rd->offset_xy);
            }
            if (config.debug_flag & DF_USB_DECODE_JOY) {
                if (rd->pos_jpad[0] &&
                    (phost->device.DevDesc.idVendor == 0x057e) &&
                    (phost->device.DevDesc.idProduct == 0x2009)) {
                    /* EasySMX PC USB controller adds timestamp to report */
                    report_data[0] &= ~0x00ff00;  // Clobber timestamp
                }
                if (memcmp(report_data, report_last, 12) != 0) {
                    memcpy(report_last, report_data, sizeof (report_last));
                    printf("\n%08lx %08lx %08lx",
                           report_data[0], report_data[1], report_data[2]);
                    printf(" [%d %d %d %d] ", report_info->x, report_info->y,
                           report_info->ac_pan, report_info->wheel);
                }
            }
        } else {
            if ((config.debug_flag & DF_USB_DECODE_JOY) &&
                (memcmp(report_data, report_last, sizeof (report_data)) != 0)) {
                printf("\n%08lx %08lx %08lx %08lx",
                       report_data[0], report_data[1], report_data[2],
                       report_data[3]);
                memcpy(report_last, report_data, sizeof (report_last));
            }
//          dprintf(DF_USB_DECODE_MISC, "Misc ID %x", id);
        }
        cur = 0;
        for (button = 0; button < rd->num_mmbuttons; button++) {
            uint val;
            if (id != rd->id_mmbutton[button])
                continue;
            val = readbits(report_data, rd->pos_mmbutton[button], 1, 0);
            if (val != 0) {
                if (cur == 0)
                    dprintf(DF_USB_DECODE_MISC, "MMKEY");
                dprintf(DF_USB_DECODE_MISC, " %x", rd->val_mmbutton[button]);
                report_info->mm_key[cur++] = rd->val_mmbutton[button];
                if (cur == ARRAY_SIZE(report_info->mm_key))
                    break;
            }
        }
        return USBH_OK;
    }
    return   USBH_FAIL;
}

/**
  * @brief  USBH_HID_DecodeKeyboard
  *         Retrieve and decode keyboard input
  * @param  phost: Host handle
  * @retval keyboard information
  */
USBH_StatusTypeDef USBH_HID_DecodeKeyboard(USBH_HandleTypeDef *phost, HID_HandleTypeDef *HID_Handle, HID_Keyboard_Info_TypeDef *report_info)
{
    uint recvlen;
    uint len;
    uint32_t report_data[6];
    HID_RDescTypeDef *rd = &HID_Handle->HID_RDesc;

    if (HID_Handle == NULL)
        return (USBH_FAIL);

    if (HID_Handle->length == 0U)
        return (USBH_FAIL);

    /* Fill report */
    len = HID_Handle->length;
    if (len > sizeof (report_data))
        len = sizeof (report_data);

    memset(report_data, 0, sizeof (*report_data));
    recvlen = USBH_HID_FifoRead(&HID_Handle->fifo, &report_data, len);
    USBH_HID_FifoFlush(&HID_Handle->fifo);  // Discard excess data
    if (config.debug_flag & DF_USB_DECODE_KBD) {
        uint pos;
        uint8_t *ptr = (uint8_t *) report_data;
        for (pos = 0; pos < recvlen; pos++) {
            if ((pos & 3) == 0) {
                if (pos == 0)
                    printf("\n");
                else
                    printf(" ");
            }
            printf("%02x", *(ptr++));
        }
        printf(" ");
    }

    if (recvlen > 0) {
        uint cur;
        uint8_t id = report_data[0];
        memset(report_info, 0, sizeof (*report_info));
        if ((rd->id_consumer != 0) && (rd->id_consumer == id)) {
            dprintf(DF_USB_DECODE_MISC, "mmkey");
            for (cur = 0; cur < rd->num_keys; cur++) {
                if (rd->pos_key[cur] == 0)
                    continue;
                report_info->mm_key[cur] =
                    readbits(report_data, rd->pos_key[cur], rd->bits_key, 1);
                dprintf(DF_USB_DECODE_MISC, " %02x", report_info->mm_key[cur]);
            }
        } else if (rd->pos_keynkro & 0x8000) {
            uint byte_keymod  = rd->pos_keymod / 8;
            uint pos = 0;
            uint pressed = 0;
            uint pos_keynkro = rd->pos_keynkro & 0x7fff;
            memset(report_info, 0, sizeof (*report_info));
            memcpy(&report_info->modifier,
                   ((uint8_t *) report_data) + byte_keymod, 1);
            for (pos = pos_keynkro; pos < recvlen * 8; ) {
                if (((pos & 31) == 0) && (report_data[pos / 32] == 0)) {
                    /* Nothing set in these bits */
                    pos += 32;
                    continue;
                }
                if (report_data[pos / 32] & BIT(pos & 31)) {
                    uint scancode = pos - pos_keynkro;
                    report_info->keycode[pressed] = scancode;
                    if (++pressed == 6)
                        break;  // buffer full
                }
                pos++;
            }
        } else if ((rd->pos_keymod != 0) || (rd->pos_keynkro != 0)) {
            uint byte_keymod  = rd->pos_keymod / 8;
            uint byte_key6kro = rd->pos_keynkro / 8;
            memset(report_info, 0, sizeof (*report_info));
            memcpy(&report_info->modifier,
                   ((uint8_t *) report_data) + byte_keymod, 1);
            memcpy(&report_info->keycode,
                   ((uint8_t *) report_data) + byte_key6kro, 6);
        } else {
            memcpy(report_info, report_data, sizeof (*report_info));
        }
        return (USBH_OK);
    }

    return (USBH_FAIL);
}

static uint32_t hid_rx_report_buf[2][HID_QUEUE_SIZE * 4];

void
USBH_HID_PrepareFifo(USBH_HandleTypeDef *phost, HID_HandleTypeDef *HID_Handle)
{
    uint port = phost->id;

    if (HID_Handle->length_max > sizeof (hid_rx_report_buf[port]))
        HID_Handle->length_max = sizeof (hid_rx_report_buf[port]);

    memset(hid_rx_report_buf[port], 0, sizeof (hid_rx_report_buf[port]));

    HID_Handle->pData = (uint8_t *)(void *) hid_rx_report_buf[port];
    USBH_HID_FifoInit(&HID_Handle->fifo, phost->device.Data,
                      sizeof (hid_rx_report_buf[port]));
}
