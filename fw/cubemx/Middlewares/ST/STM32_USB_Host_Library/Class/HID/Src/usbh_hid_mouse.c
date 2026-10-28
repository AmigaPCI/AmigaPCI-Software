/**
  ******************************************************************************
  * @file    usbh_hid_mouse.c
  * @author  MCD Application Team
  * @brief   This file is the application layer for USB Host HID Mouse Handling.
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
#include "usbh_hid_mouse.h"
#include "usbh_hid_parser.h"


/** @addtogroup USBH_LIB
  * @{
  */

/** @addtogroup USBH_CLASS
  * @{
  */

/** @addtogroup USBH_HID_CLASS
  * @{
  */

/** @defgroup USBH_HID_MOUSE
  * @brief    This file includes HID Layer Handlers for USB Host HID class.
  * @{
  */

/** @defgroup USBH_HID_MOUSE_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */


/** @defgroup USBH_HID_MOUSE_Private_Defines
  * @{
  */
/**
  * @}
  */


/** @defgroup USBH_HID_MOUSE_Private_Macros
  * @{
  */
/**
  * @}
  */

/** @defgroup USBH_HID_MOUSE_Private_FunctionPrototypes
  * @{
  */
#if 0
static USBH_StatusTypeDef USBH_HID_MouseDecode(USBH_HandleTypeDef *phost, HID_HandleTypeDef *HID_Handle);
#endif

/**
  * @}
  */


/** @defgroup USBH_HID_MOUSE_Private_Variables
  * @{
  */
#if 0
HID_MOUSE_Info_TypeDef    mouse_info;
uint32_t                  mouse_report_data[2];
uint32_t                  mouse_rx_report_buf[HID_QUEUE_SIZE * 2];
#endif

#if 0
/* Structures defining how to access items in a HID mouse report */
/* Access button 1 state. */
static const HID_Report_ItemTypedef prop_b1 =
{
  (uint8_t *)(void *)mouse_report_data + 0, /*data*/
  1,     /*size*/
  0,     /*shift*/
  0,     /*count (only for array items)*/
  0,     /*signed?*/
  0,     /*min value read can return*/
  1,     /*max value read can return*/
  0,     /*min value device can report*/
  1,     /*max value device can report*/
  1      /*resolution*/
};

/* Access button 2 state. */
static const HID_Report_ItemTypedef prop_b2 =
{
  (uint8_t *)(void *)mouse_report_data + 0, /*data*/
  1,     /*size*/
  1,     /*shift*/
  0,     /*count (only for array items)*/
  0,     /*signed?*/
  0,     /*min value read can return*/
  1,     /*max value read can return*/
  0,     /*min value device can report*/
  1,     /*max value device can report*/
  1      /*resolution*/
};

/* Access button 3 state. */
static const HID_Report_ItemTypedef prop_b3 =
{
  (uint8_t *)(void *)mouse_report_data + 0, /*data*/
  1,     /*size*/
  2,     /*shift*/
  0,     /*count (only for array items)*/
  0,     /*signed?*/
  0,     /*min value read can return*/
  1,     /*max value read can return*/
  0,     /*min vale device can report*/
  1,     /*max value device can report*/
  1      /*resolution*/
};

/* Access x coordinate change. */
static const HID_Report_ItemTypedef prop_x =
{
  (uint8_t *)(void *)mouse_report_data + 1, /*data*/
  8,     /*size*/
  0,     /*shift*/
  0,     /*count (only for array items)*/
  1,     /*signed?*/
  0,     /*min value read can return*/
  0xFFFF,/*max value read can return*/
  0,     /*min vale device can report*/
  0xFFFF,/*max value device can report*/
  1      /*resolution*/
};

/* Access y coordinate change. */
static const HID_Report_ItemTypedef prop_y =
{
  (uint8_t *)(void *)mouse_report_data + 2, /*data*/
  8,     /*size*/
  0,     /*shift*/
  0,     /*count (only for array items)*/
  1,     /*signed?*/
  0,     /*min value read can return*/
  0xFFFF,/*max value read can return*/
  0,     /*min vale device can report*/
  0xFFFF,/*max value device can report*/
  1      /*resolution*/
};
#endif


/**
  * @}
  */

#define ARRAY_SIZE(x) (int)((sizeof (x) / sizeof ((x)[0])))
#define BIT(x) (1U << (x))

#include "config.h"

/**
  * @brief  USBH_HID_MouseInit
  *         The function init the HID mouse.
  * @param  phost: Host handle
  * @retval USBH Status
  */
USBH_StatusTypeDef USBH_HID_MouseInit(USBH_HandleTypeDef *phost, HID_HandleTypeDef *HID_Handle)
{
  dprintf(DF_USB_MOUSE, "USB%s mouseinit\n",
          usb_get_path(phost, NULL, HID_Handle->interface));
  HID_RDescTypeDef *rd = &HID_Handle->HID_RDesc;

  /* Defaults (hopefully these are BOOT MODE mouse settings) */
  if (rd->num_buttons == 0) {
    rd->pos_button[0] = 8 + 0;
    rd->pos_button[1] = 8 + 1;
    rd->pos_button[2] = 8 + 2;
    rd->num_buttons   = 3;
  }
  if ((rd->pos_x == 0) && (rd->pos_y == 0)) {
    rd->pos_x         = 8 + 8;
    rd->pos_y         = 8 + 16;
    rd->pos_wheel     = 8 + 24;
    rd->bits_x     = 8;
    rd->bits_y     = 8;
    rd->bits_wheel = 8;
  }

#if 0
  memset(&mouse_info, 0, sizeof (mouse_info));
  memset(mouse_report_data, 0, sizeof (mouse_report_data));
  memset(mouse_rx_report_buf, 0, sizeof (mouse_rx_report_buf));

  if (HID_Handle->length > sizeof(mouse_report_data))
  {
    HID_Handle->length = sizeof(mouse_report_data);
  }
  HID_Handle->pData = (uint8_t *)(void *)mouse_rx_report_buf;
  USBH_HID_FifoInit(&HID_Handle->fifo, phost->device.Data, HID_QUEUE_SIZE * sizeof(mouse_report_data));
#else
  USBH_HID_PrepareFifo(phost, HID_Handle);
#endif

  return USBH_OK;
}

#if 0
/**
  * @brief  USBH_HID_GetMouseInfo
  *         The function return mouse information.
  * @param  phost: Host handle
  * @retval mouse information
  */
HID_MOUSE_Info_TypeDef *USBH_HID_GetMouseInfo(USBH_HandleTypeDef *phost, HID_HandleTypeDef *HID_Handle)
{
  if (USBH_HID_MouseDecode(phost, HID_Handle) == USBH_OK)
  {
    return &mouse_info;
  }
  else
  {
    return NULL;
  }
}

static int
readbits(void *ptr, uint startbit, uint bits)
{
    uint byte = startbit / 8;
    uint bitoff = startbit % 8;
    uint mask = BIT(bits) - 1;
    uint val = ((*(uint *) ((uintptr_t) ptr + byte)) >> bitoff) & mask;

    if (val & BIT(bits - 1))
        val |= (0 - BIT(bits));  // Sign-extend negative

    return (val);
}

/**
  * @brief  USBH_HID_MouseDecode
  *         The function decode mouse data.
  * @param  phost: Host handle
  * @retval USBH Status
  */
static USBH_StatusTypeDef USBH_HID_MouseDecode(USBH_HandleTypeDef *phost, HID_HandleTypeDef *HID_Handle)
{
  HID_RDescTypeDef *rd = &HID_Handle->HID_RDesc;
  if (HID_Handle == NULL)
  {
    return USBH_FAIL;
  }

  if (HID_Handle->length == 0U)
  {
    return USBH_FAIL;
  }
  /*Fill report */
  if (USBH_HID_FifoRead(&HID_Handle->fifo, &mouse_report_data, HID_Handle->length) ==  HID_Handle->length)
  {
    uint button;
    uint cur;
    uint8_t id = mouse_report_data[0];
    uint32_t buttons = 0;
    dprintf(DF_USB_MOUSE, "\n%08lx %08lx ",
            mouse_report_data[0], mouse_report_data[1]);
    for (button = 0; button < rd->num_buttons; button++) {
        if ((button < ARRAY_SIZE(rd->pos_button)) &&
            readbits(mouse_report_data, rd->pos_button[button], 1))
            buttons |= BIT(button);
    }
    memset(&mouse_info, 0, sizeof (mouse_info));
    if ((rd->id_mouse == 0) || (rd->id_mouse == id)) {
        mouse_info.buttons = buttons;
        mouse_info.x = readbits(mouse_report_data, rd->pos_x, rd->bits_x);
        mouse_info.y = readbits(mouse_report_data, rd->pos_y, rd->bits_y);
        mouse_info.wheel = readbits(mouse_report_data, rd->pos_wheel,
                                    rd->bits_wheel);
        mouse_info.ac_pan = readbits(mouse_report_data, rd->pos_ac_pan,
                                     rd->bits_ac_pan);
    } else if ((rd->id_consumer != 0) && (rd->id_consumer == id)) {
        dprintf(DF_USB_MOUSE, "mmkey");
        for (cur = 0; cur < rd->num_keys; cur++) {
            if (rd->pos_key[cur] == 0)
                continue;
            mouse_info.mm_key[cur] = readbits(mouse_report_data,
                                              rd->pos_key[cur], rd->bits_key);
            dprintf(DF_USB_MOUSE, " %x", mouse_info.mm_key[cur]);
        }
    } else if ((rd->id_sysctl != 0) && (rd->id_sysctl == id)) {
        mouse_info.sysctl = readbits(mouse_report_data,
                                     rd->pos_sysctl, rd->bits_sysctl);
        dprintf(DF_USB_MOUSE, "Sysctl %x", mouse_info.sysctl);
    }
    return USBH_OK;
  }
  return   USBH_FAIL;
}
#endif

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
