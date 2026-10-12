/**
  ******************************************************************************
  * @file    usbh_hub.h
  * @author  Mori
  * @version
  * @date
  * @brief   Definitions used in the USB Hub support
  ******************************************************************************
  * Copyright (c) 2016 Mori
  *
  * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
  * associated documentation files (the "Software"), to deal in the Software without restriction,
  * including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
  * and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so,
  * subject to the following conditions:
  *
  * The above copyright notice and this permission notice shall be included in all copies or substantial
  * portions of the Software.
  *
  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT
  * LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
  * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
  * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
  * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
  *
  ******************************************************************************
*/
#ifndef __USBH_HUB_H
#define __USBH_HUB_H

/* Includes ------------------------------------------------------------------*/
#include "usbh_core.h"

#define USB_HUB_CLASS     					 0x09
#define HUB_MIN_POLL	  					 200

#define MAX_HUB_PORTS 						 5
#define HUB_ENUM_RETRIES                     3    // Port resets on enum failure
#define HUB_CONNECT_DEBOUNCE_MS              100  // USB 2.0 7.1.7.3 TATTDB
#define HUB_RESET_RECOVERY_MS                10   // USB 2.0 7.1.7.5 TRSTRCY
#define HUB_POWER_CYCLE_RETRY                2    // Retry # which power-cycles
#define HUB_POWER_OFF_MIN_MS                 500  // Min port power-off time

/* wHubCharacteristics: Logical Power Switching Mode (bits 1:0) */
#define HUB_CHAR_LPSM_MASK                   0x0003
#define HUB_CHAR_LPSM_GANGED                 0x0000
#define HUB_CHAR_LPSM_INDIVIDUAL             0x0001

#define USB_DESCRIPTOR_HUB                   0x29
#define USB_REQUEST_GET_DESCRIPTOR           0x06
#define HUB_FEATURE_SEL_PORT_POWER           0x08

#define USB_DEVICE_REQUEST_SET   			 0x00
#define USB_DEVICE_REQUEST_GET   			 0x01
#define USB_REQUEST_CLEAR_FEATURE   		 0x01
#define USB_REQUEST_SET_FEATURE     		 0x03
#define USB_REQUEST_GET_STATUS          	 0x00

#define HUB_FEAT_SEL_PORT_CONNECTION 		 0x00
#define HUB_FEAT_SEL_C_HUB_LOCAL_POWER       0x00
#define HUB_FEAT_SEL_C_HUB_OVER_CURRENT      0x01

#define HUB_FEAT_SEL_PORT_CONN         		 0x00
#define HUB_FEAT_SEL_PORT_ENABLE             0x01
#define HUB_FEAT_SEL_PORT_SUSPEND            0x02
#define HUB_FEAT_SEL_PORT_OVER_CURRENT       0x03
#define HUB_FEAT_SEL_PORT_RESET              0x04
#define HUB_FEAT_SEL_PORT_POWER              0x08
#define HUB_FEAT_SEL_PORT_LOW_SPEED          0x09
#define HUB_FEAT_SEL_C_PORT_CONNECTION       0x10
#define HUB_FEAT_SEL_C_PORT_ENABLE           0x11
#define HUB_FEAT_SEL_C_PORT_SUSPEND          0x12
#define HUB_FEAT_SEL_C_PORT_OVER_CURRENT     0x13
#define HUB_FEAT_SEL_C_PORT_RESET            0x14
#define HUB_FEAT_SEL_PORT_INDICATOR          0x16

typedef enum
{
	HUB_IDLE= 0,
	HUB_SYNC,
	HUB_BUSY,
	HUB_GET_DATA,
	HUB_POLL,
	HUB_LOOP_PORT_CHANGED,
	HUB_LOOP_PORT_ENUM,
	HUB_LOOP_PORT_WAIT,
	HUB_PORT_CHANGED,
	HUB_C_PORT_CONNECTION,
	HUB_C_PORT_RESET,
	HUB_RESET_DEVICE,
	HUB_DEV_ATTACHED,
	HUB_DEV_DETACHED,
	HUB_C_PORT_OVER_CURRENT,
	HUB_C_PORT_SUSPEND,
	HUB_PORT_POWER_OFF,
	HUB_PORT_POWER_ON,
	HUB_ERROR,

} HUB_StateTypeDef;

typedef enum
{
	HUB_REQ_IDLE = 0,
	HUB_REQ_GET_DESCRIPTOR,
	HUB_REQ_SET_POWER,
	HUB_WAIT_PWRGOOD,
	HUB_REQ_DONE,
}
HUB_CtlStateTypeDef;

typedef struct __attribute__ ((packed)) _USB_PORT_CHANGE
{
    union
    {
        struct
        {
			uint8_t     PORT_1    : 1;
			uint8_t     PORT_2    : 1;
			uint8_t     PORT_3    : 1;
			uint8_t     PORT_4    : 1;
			uint8_t     PORT_5    : 1;
			uint8_t     PORT_6    : 1;
			uint8_t     PORT_7    : 1;
			uint8_t     PORT_8    : 1;

        } bPorts;

        uint8_t val;
    };

} USB_PORT_CHANGE;

struct _USB_HUB_PORT_STATUS;

/*
 * Structure for HUB process
 *
 * All per-hub state lives here (not in globals indexed by STM32 physical
 * port), so that multiple hubs on the same physical port -- including
 * cascaded hubs such as two GL850G chips in one 7-port hub -- each keep
 * their own state.
 */
typedef struct _HUB_Process
{
  uint8_t              InPipe;
  HUB_StateTypeDef     state;
  uint8_t              InEp;
  HUB_CtlStateTypeDef  ctl_state;
  uint8_t              buffer[20];
  uint16_t             length;
  uint8_t              ep_addr;
  uint16_t             poll;
  uint32_t             timer;
  uint8_t              DataReady;

  USB_PORT_CHANGE      change;        // Ports with pending status change
  uint8_t              cur_port;      // Hub port currently being serviced
  uint8_t              num_ports;     // Number of downstream ports
  uint16_t             pwr_good;      // Power-on to power-good time (ms)
  uint8_t              req_port;      // Port for HUB_REQ_SET_POWER loop
  uint8_t              desc_state;    // get_hub_descriptor() state
  uint8_t              enum_retries;  // Port resets done for failed enum
  uint8_t              need_debounce; // New connection: debounce before reset
  uint8_t              power_cycling; // Port power cycle in progress
  uint16_t             hub_chars;     // wHubCharacteristics
  struct _USB_HUB_PORT_STATUS *change_info;  // Points into buffer[]
} HUB_HandleTypeDef;


extern USBH_ClassTypeDef  HUB_Class;
#define USBH_HUB_CLASS    &HUB_Class

typedef struct __attribute__ ((packed)) _USB_HUB_DESCRIPTOR
{
	uint8_t  bLength;               // Length of this descriptor.
	uint8_t  bDescriptorType;       // Descriptor Type, value: 29H for hub descriptor
	uint8_t  bNbrPorts;             // Number of downstream facing ports that this hub supports
	uint16_t wHubCharacteristics;   //
    uint8_t  bPwrOn2PwrGood;        // Time (in 2 ms intervals) from the time the power-on sequence begins on a port until power is good on that port
    uint8_t  bHubContrCurrent;      // Maximum current requirements of the Hub Controller electronics in mA
    uint8_t  DeviceRemovable;       // Indicates if a port has a removable device attached.
    uint8_t  PortPwrCtrlMask;       // This field exists for reasons of compatibility with software written for 1.0 compliant devices.

} USB_HUB_DESCRIPTOR;

typedef struct __attribute__ ((packed)) _USB_HUB_PORT_STATUS
{
    union
    {
        struct
        {
        	uint8_t     PORT_CONNECTION      : 1;
        	uint8_t     PORT_ENABLE          : 1;
        	uint8_t     PORT_SUSPEND         : 1;
        	uint8_t     PORT_OVER_CURRENT    : 1;
        	uint8_t     PORT_RESET           : 1;
        	uint8_t     RESERVED_1           : 3;
        	uint8_t     PORT_POWER           : 1;
        	uint8_t     PORT_LOW_SPEED       : 1;
        	uint8_t     PORT_HIGH_SPEED      : 1;
        	uint8_t     PORT_TEST            : 1;
        	uint8_t     PORT_INDICATOR       : 1;
        	uint8_t     RESERVED_2           : 3;
        };

        uint16_t val;

    }   wPortStatus;

    union
    {
        struct
        {
        	uint8_t     C_PORT_CONNECTION    : 1;
        	uint8_t     C_PORT_ENABLE        : 1;
        	uint8_t     C_PORT_SUSPEND       : 1;
        	uint8_t     C_PORT_OVER_CURRENT  : 1;
        	uint8_t     C_PORT_RESET         : 1;
        	uint16_t    RESERVED             : 11;
        };

        uint16_t val;

    }   wPortChange;

} USB_HUB_PORT_STATUS;



#endif	// __USBH_HUB_H
