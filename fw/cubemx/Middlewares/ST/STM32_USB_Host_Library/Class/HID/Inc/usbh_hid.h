/**
  ******************************************************************************
  * @file    usbh_hid.h
  * @author  MCD Application Team
  * @brief   This file contains all the prototypes for the usbh_hid.c
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

/* Define to prevent recursive  ----------------------------------------------*/
#ifndef __USBH_HID_H
#define __USBH_HID_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbh_core.h"

typedef struct _HID_HandleTypeDef HID_HandleTypeDef;
#include "usbh_hid_mouse.h"
#include "usbh_hid_keybd.h"

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
  * @brief This file is the Header file for usbh_hid.c
  * @{
  */


/** @defgroup USBH_HID_CORE_Exported_Types
  * @{
  */

// #define HID_MIN_POLL                                10U
#define HID_MIN_POLL                                2U  // with 1ms pooling we have too many babble errors
#define HID_REPORT_SIZE                             16U
#define HID_MAX_USAGE                               10U
#define HID_MAX_NBR_REPORT_FMT                      10U
#define HID_QUEUE_SIZE                              10U

#define  HID_ITEM_LONG                              0xFEU

#define  HID_ITEM_TYPE_MAIN                         0x00U
#define  HID_ITEM_TYPE_GLOBAL                       0x01U
#define  HID_ITEM_TYPE_LOCAL                        0x02U
#define  HID_ITEM_TYPE_RESERVED                     0x03U


#define  HID_MAIN_ITEM_TAG_INPUT                    0x08U
#define  HID_MAIN_ITEM_TAG_OUTPUT                   0x09U
#define  HID_MAIN_ITEM_TAG_COLLECTION               0x0AU
#define  HID_MAIN_ITEM_TAG_FEATURE                  0x0BU
#define  HID_MAIN_ITEM_TAG_ENDCOLLECTION            0x0CU


#define  HID_GLOBAL_ITEM_TAG_USAGE_PAGE             0x00U
#define  HID_GLOBAL_ITEM_TAG_LOG_MIN                0x01U
#define  HID_GLOBAL_ITEM_TAG_LOG_MAX                0x02U
#define  HID_GLOBAL_ITEM_TAG_PHY_MIN                0x03U
#define  HID_GLOBAL_ITEM_TAG_PHY_MAX                0x04U
#define  HID_GLOBAL_ITEM_TAG_UNIT_EXPONENT          0x05U
#define  HID_GLOBAL_ITEM_TAG_UNIT                   0x06U
#define  HID_GLOBAL_ITEM_TAG_REPORT_SIZE            0x07U
#define  HID_GLOBAL_ITEM_TAG_REPORT_ID              0x08U
#define  HID_GLOBAL_ITEM_TAG_REPORT_COUNT           0x09U
#define  HID_GLOBAL_ITEM_TAG_PUSH                   0x0AU
#define  HID_GLOBAL_ITEM_TAG_POP                    0x0BU


#define  HID_LOCAL_ITEM_TAG_USAGE                   0x00U
#define  HID_LOCAL_ITEM_TAG_USAGE_MIN               0x01U
#define  HID_LOCAL_ITEM_TAG_USAGE_MAX               0x02U
#define  HID_LOCAL_ITEM_TAG_DESIGNATOR_INDEX        0x03U
#define  HID_LOCAL_ITEM_TAG_DESIGNATOR_MIN          0x04U
#define  HID_LOCAL_ITEM_TAG_DESIGNATOR_MAX          0x05U
#define  HID_LOCAL_ITEM_TAG_STRING_INDEX            0x07U
#define  HID_LOCAL_ITEM_TAG_STRING_MIN              0x08U
#define  HID_LOCAL_ITEM_TAG_STRING_MAX              0x09U
#define  HID_LOCAL_ITEM_TAG_DELIMITER               0x0AU


/* States for HID State Machine */
typedef enum
{
  HID_INIT = 0,
  HID_VENDOR,
  HID_GET_REPORT,
  HID_SEND_DATA,
  HID_BUSY,
  HID_GET_DATA,
  HID_SYNC,
  HID_POLL,
  HID_ERROR,
  HID_NO_SUPPORT,       // Unsupported HID device
}
HID_StateTypeDef;

typedef enum
{
  HID_REQ_INIT = 0,
  HID_REQ_IDLE,
  HID_REQ_GET_REPORT_DESC,
  HID_REQ_GET_HID_DESC,
  HID_REQ_SET_IDLE,
  HID_REQ_SET_PROTOCOL,
  HID_REQ_SET_REPORT,

}
HID_CtlStateTypeDef;

typedef enum
{
  HID_MOUSE    = 0x01,
  HID_KEYBOARD = 0x02,
  HID_UNKNOWN = 0xFF,
}
HID_TypeTypeDef;


typedef  struct  _HID_ReportData
{
  uint8_t   ReportID;
  uint8_t   ReportType;
  uint16_t  UsagePage;
  uint32_t  Usage[HID_MAX_USAGE];
  uint32_t  NbrUsage;
  uint32_t  UsageMin;
  uint32_t  UsageMax;
  int32_t   LogMin;
  int32_t   LogMax;
  int32_t   PhyMin;
  int32_t   PhyMax;
  int32_t   UnitExp;
  uint32_t  Unit;
  uint32_t  ReportSize;
  uint32_t  ReportCnt;
  uint32_t  Flag;
  uint32_t  PhyUsage;
  uint32_t  AppUsage;
  uint32_t  LogUsage;
}
HID_ReportDataTypeDef;

typedef  struct  _HID_ReportIDTypeDef
{
  uint8_t  Size;         /* Report size return by the device id            */
  uint8_t  ReportID;     /* Report Id                                      */
  uint8_t  Type;         /* Report Type (INPUT/OUTPUT/FEATURE)             */
} HID_ReportIDTypeDef;

typedef struct  _HID_CollectionTypeDef
{
  uint32_t                       Usage;
  uint8_t                        Type;
  struct _HID_CollectionTypeDef  *NextPtr;
} HID_CollectionTypeDef;


typedef  struct  _HID_AppCollectionTypeDef
{
  uint32_t               Usage;
  uint8_t                Type;
  uint8_t                NbrReportFmt;
  HID_ReportDataTypeDef  ReportData[HID_MAX_NBR_REPORT_FMT];
} HID_AppCollectionTypeDef;


typedef struct _HIDDescriptor
{
  uint8_t   bLength;
  uint8_t   bDescriptorType;
  uint16_t  bcdHID;               /* indicates what endpoint this descriptor is describing */
  uint8_t   bCountryCode;        /* specifies the transfer type. */
  uint8_t   bNumDescriptors;     /* specifies the transfer type. */
  uint8_t   bReportDescriptorType;    /* Maximum Packet Size this endpoint is capable of sending or receiving */
  uint16_t  wItemLength;          /* is used to specify the polling interval of certain transfers. */
}
HID_DescTypeDef;

#define DEV_FLAG_ABSOLUTE 0x01  // Device gives absolute position (not relative)

typedef struct _HIDRDescriptor
{
  uint16_t   usage;            // Top level Usage
  uint16_t   dev_flag;         // Device flags
  uint16_t   pos_x;            // Position of Mouse X movement
  uint16_t   pos_y;            // Position of Mouse Y movement
  uint16_t   pos_wheel;        // Position of Mouse wheel movement
  uint16_t   pos_ac_pan;       // Position of Mouse left-right pan movement
  uint16_t   pos_button[16];   // Position of Mouse buttons
  uint16_t   pos_key[2];       // Position of Multimedia key
  uint16_t   pos_jpad[4];      // Joystick/pad button positions U D L R
  uint16_t   pos_sysctl;       // Position of System control key
  uint16_t   pos_keymod;       // Position of keyboard modifiers
  uint16_t   pos_keynkro;      // Position of keyboard 6-key or n-key rollover
  uint16_t   pos_mmbutton[20]; // Position of Multimedia button
  uint16_t   val_mmbutton[20]; // MM Key value of Multimedia button
  int16_t    offset_xy;        // Offset to add to mouse x / y / wheel / pan
  uint8_t    id_mmbutton[20];  // Report ID code for each button
  uint8_t    num_mmbuttons;    // Number of Multimedia buttons
  uint8_t    num_buttons;      // Number of mouse buttons
  uint8_t    num_keys;         // Number of multimedia key positions
  uint8_t    bits_x;           // Number of bits for Mouse X movement
  uint8_t    bits_y;           // Number of bits for Mouse Y movement
  uint8_t    bits_wheel;       // Number of bits for Mouse wheel movement
  uint8_t    bits_ac_pan;      // Number of bits for Mouse left-right movement
  uint8_t    bits_key;         // Number of bits for Multimedia key
  uint8_t    bits_sysctl;      // Number of bits for System control key
  uint8_t    id_mouse;         // Report ID code for Mouse movement
  uint8_t    id_consumer;      // Report ID code for Consumer control
  uint8_t    id_sysctl;        // Report ID code for System control
}
HID_RDescTypeDef;

typedef struct
{
  uint8_t  *buf;
  uint16_t  head;
  uint16_t tail;
  uint16_t size;
  uint8_t  lock;
} FIFO_TypeDef;


/* Structure for HID handle */
struct _HID_HandleTypeDef
{
  uint8_t              interface;  // USB device interface for this handle
  uint8_t              OutPipe;
  uint8_t              InPipe;
  HID_StateTypeDef     state;
  uint8_t              OutEp;
  uint8_t              InEp;
  HID_CtlStateTypeDef  ctl_state;
  FIFO_TypeDef         fifo;
  uint8_t              *pData;
  uint16_t             length;
  uint16_t             length_max;
  uint8_t              ep_addr;
  uint16_t             poll;
  uint32_t             timer;
  uint8_t              DataReady;
  uint8_t              error_count;
  HID_DescTypeDef      HID_Desc;
  HID_RDescTypeDef     HID_RDesc;
  USBH_StatusTypeDef(* Init)(USBH_HandleTypeDef *phost, HID_HandleTypeDef *HID_Handle);
  USBH_StatusTypeDef(* Vendor)(USBH_HandleTypeDef *phost, HID_HandleTypeDef *HID_Handle);  // Vendor-specific init
  struct _HID_HandleTypeDef *next;
};

/**
  * @}
  */

/** @defgroup USBH_HID_CORE_Exported_Defines
  * @{
  */

/* HID class request codes */
#define USB_HID_GET_REPORT                            0x01U
#define USB_HID_GET_IDLE                              0x02U
#define USB_HID_GET_PROTOCOL                          0x03U
#define USB_HID_SET_REPORT                            0x09U
#define USB_HID_SET_IDLE                              0x0AU
#define USB_HID_SET_PROTOCOL                          0x0BU




/* HID Class Codes */
#define USB_HID_CLASS                                 0x03U

/* Interface Descriptor field values for HID Boot Protocol */
#define HID_BOOT_CODE                                 0x01U
#define HID_KEYBRD_BOOT_CODE                          0x01U
#define HID_MOUSE_BOOT_CODE                           0x02U


/**
  * @}
  */

/** @defgroup USBH_HID_CORE_Exported_Macros
  * @{
  */
/**
  * @}
  */

/** @defgroup USBH_HID_CORE_Exported_Variables
  * @{
  */
extern USBH_ClassTypeDef  HID_Class;
#define USBH_HID_CLASS    &HID_Class
/**
  * @}
  */

/** @defgroup USBH_HID_CORE_Exported_FunctionsPrototype
  * @{
  */

USBH_StatusTypeDef USBH_HID_SetReport(USBH_HandleTypeDef *phost,
                                      uint8_t reportType,
                                      uint8_t reportId,
                                      uint8_t *reportBuff,
                                      uint8_t reportLen);

USBH_StatusTypeDef USBH_HID_GetReport(USBH_HandleTypeDef *phost,
                                      uint8_t reportType,
                                      uint8_t reportId,
                                      uint8_t *reportBuff,
                                      uint8_t reportLen);

USBH_StatusTypeDef USBH_HID_GetHIDReportDescriptor(USBH_HandleTypeDef *phost,
                                                   uint16_t iface, uint16_t length);

USBH_StatusTypeDef USBH_HID_GetHIDDescriptor(USBH_HandleTypeDef *phost,
                                             uint16_t iface, uint16_t length);

USBH_StatusTypeDef USBH_HID_SetIdle(USBH_HandleTypeDef *phost,
                                    uint8_t duration,
                                    uint8_t reportId);

USBH_StatusTypeDef USBH_HID_SetProtocol(USBH_HandleTypeDef *phost,
                                        uint8_t protocol);

void USBH_HID_EventCallback(USBH_HandleTypeDef *phost, HID_HandleTypeDef *HID_Handle);

HID_TypeTypeDef USBH_HID_GetDeviceType(USBH_HandleTypeDef *phost, uint16_t iface);

uint8_t USBH_HID_GetPollInterval(USBH_HandleTypeDef *phost);

void USBH_HID_FifoInit(FIFO_TypeDef *f, uint8_t *buf, uint16_t size);
void USBH_HID_FifoFlush(FIFO_TypeDef *f);

uint16_t  USBH_HID_FifoRead(FIFO_TypeDef *f, void *buf, uint16_t  nbytes);

uint16_t  USBH_HID_FifoWrite(FIFO_TypeDef *f, void *buf, uint16_t nbytes);

void USBH_HID_Process_HIDReportDescriptor(USBH_HandleTypeDef *phost, HID_HandleTypeDef *HID_Handle);

#define MI_FLAG_HAS_JPAD BIT(0)  // Device has joypad

typedef struct
{
  uint16_t             usage;       // Usage type for this report
  uint8_t              flags;       // Device flag bits
  uint8_t              jpad;        // Joystick pad directions (U D L R)
  uint32_t             buttons;     // Mouse buttons
  int8_t               x;           // Mouse X movement
  int8_t               y;           // Mouse Y movement
  int8_t               wheel;       // Mouse Wheel movement
  int8_t               ac_pan;      // Mouse Left-Right movement
  uint16_t             sysbuttons;  // System buttons (power, sleep, wake)
  uint16_t             mm_key[2];   // Multimedia key(s)
  uint16_t             sysctl;      // System control button(s)
}
HID_MISC_Info_TypeDef;

typedef struct {
  uint8_t              modifier;    // Keyboard modifier keys
  uint8_t              reserved;    // Reserved for OEM use, always set to 0
  uint8_t              keycode[6];  // Key codes of the currently pressed keys
  uint16_t             mm_key[2];   // Multimedia key(s)
}
HID_Keyboard_Info_TypeDef;

USBH_StatusTypeDef USBH_HID_DecodeReport(USBH_HandleTypeDef *phost, HID_HandleTypeDef *HID_Handle, HID_TypeTypeDef devtype, HID_MISC_Info_TypeDef *report_info);
USBH_StatusTypeDef USBH_HID_DecodeKeyboard(USBH_HandleTypeDef *phost, HID_HandleTypeDef *HID_Handle, HID_Keyboard_Info_TypeDef *report_info);
void USBH_HID_PrepareFifo(USBH_HandleTypeDef *phost, HID_HandleTypeDef *HID_Handle);


/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __USBH_HID_H */

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

