/**
  ******************************************************************************
  * @file    usbh_hub.c
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
#include "usbh_hub.h"
#include "usbh_hid.h"

#ifndef BIT
#define BIT(x)      (1U << (x))
#endif

//#include "usbh_msc.h"
// #include "stm32f4xx_hal.h"
// #include "stm32f4xx_hal_hcd.h"

// extern USBH_HandleTypeDef hUSBHost[5];
extern HCD_HandleTypeDef _hHCD[2];

static USBH_StatusTypeDef get_hub_descriptor(USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef set_hub_port_power(USBH_HandleTypeDef *phost, uint8_t hub_port);
static USBH_StatusTypeDef hub_request(USBH_HandleTypeDef *phost, uint8_t request, uint8_t feature, uint8_t dataDirection, uint8_t porta, uint8_t *buffer, uint16_t size);
static USBH_StatusTypeDef set_hub_request(USBH_HandleTypeDef *phost, uint8_t request, uint8_t feature, uint8_t hub_port);
static USBH_StatusTypeDef get_hub_request(USBH_HandleTypeDef *phost, uint8_t request, uint8_t feature, uint8_t porta, uint8_t *buffer, uint16_t size);
static USBH_StatusTypeDef clear_port_feature(USBH_HandleTypeDef *phost, uint8_t feature, uint8_t hub_port);
static USBH_StatusTypeDef set_port_feature(USBH_HandleTypeDef *phost, uint8_t feature, uint8_t hub_port);
static void clear_port_changed(HUB_HandleTypeDef *HUB_Handle, uint8_t hub_port);
static uint8_t get_port_changed(HUB_HandleTypeDef *HUB_Handle);
static uint8_t port_changed(USBH_HandleTypeDef *phost, HUB_HandleTypeDef *HUB_Handle, uint8_t *b);
static void detach(USBH_HandleTypeDef *phost, uint16_t idx);
static void attach(USBH_HandleTypeDef *phost, uint16_t idx, uint8_t lowspeed);
static uint8_t attach_is_complete(USBH_HandleTypeDef *phost, uint16_t idx);
static void debug_port(USBH_HandleTypeDef *phost, HUB_HandleTypeDef *HUB_Handle);

static USBH_StatusTypeDef USBH_HUB_InterfaceInit  (USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_HUB_InterfaceDeInit  (USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_HUB_ClassRequest(USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_HUB_Process(USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_HUB_SOFProcess(USBH_HandleTypeDef *phost);

USBH_ClassTypeDef  HUB_Class =
{
        "HUB",
        USB_HUB_CLASS,
        NULL,
        USBH_HUB_InterfaceInit,
        USBH_HUB_InterfaceDeInit,
        USBH_HUB_ClassRequest,
        USBH_HUB_Process,
        USBH_HUB_SOFProcess,
        NULL,
};

/*
 * Hub state is kept per hub in HUB_HandleTypeDef. It used to be kept in
 * globals indexed by STM32 physical port, which allowed only one hub per
 * port; a second (e.g. cascaded) hub would clobber the first hub's state.
 */

#include "printf.h"
#define LOG1(...) printf(__VA_ARGS__)

static USBH_StatusTypeDef USBH_HUB_InterfaceInit (USBH_HandleTypeDef *phost)
{
    uint8_t interface;

    USBH_StatusTypeDef status = USBH_FAIL ;
    HUB_HandleTypeDef *HUB_Handle;

    interface = USBH_FindInterface(phost, phost->pActiveClass->ClassCode, 0x00, 0x00);

    if(interface == 0xFF) /* No Valid Interface */
    {
            status = USBH_FAIL;
            USBH_DbgLog ("Cannot Find the interface for %s class.", phost->pActiveClass->Name);
    }
    else
    {
        HUB_Handle = (HUB_HandleTypeDef *)USBH_malloc(sizeof(HUB_HandleTypeDef));
        if (HUB_Handle == NULL)
            return (USBH_FAIL);
        memset(HUB_Handle, 0, sizeof (HUB_HandleTypeDef));
        phost->USBH_ClassTypeDef_pData[0] = HUB_Handle;

        phost->interfaces = 1;

        USBH_SelectInterface(phost, interface);

        HUB_Handle->state     = HUB_IDLE;
        HUB_Handle->ctl_state = HUB_REQ_IDLE;
        HUB_Handle->ep_addr   = phost->device.CfgDesc.Itf_Desc[phost->device.current_interface].Ep_Desc[0].bEndpointAddress;
        HUB_Handle->length    = phost->device.CfgDesc.Itf_Desc[phost->device.current_interface].Ep_Desc[0].wMaxPacketSize;
        HUB_Handle->poll      = phost->device.CfgDesc.Itf_Desc[phost->device.current_interface].Ep_Desc[0].bInterval;

        if (HUB_Handle->poll  < HUB_MIN_POLL)
            HUB_Handle->poll = HUB_MIN_POLL;

USBH_UsrLog("device POLL %d, LEN %d", HUB_Handle->poll, HUB_Handle->length);

        if(phost->device.CfgDesc.Itf_Desc[phost->device.current_interface].Ep_Desc[0].bEndpointAddress & 0x80)
        {
            HUB_Handle->InEp = (phost->device.CfgDesc.Itf_Desc[phost->device.current_interface].Ep_Desc[0].bEndpointAddress);
            HUB_Handle->InPipe  = USBH_AllocPipe(phost, HUB_Handle->InEp);

            // Open pipe for IN endpoint
            USBH_OpenPipe  (phost, HUB_Handle->InPipe, HUB_Handle->InEp, phost->device.address, phost->device.speed, USB_EP_TYPE_INTR, HUB_Handle->length);

            USBH_LL_SetToggle (phost, HUB_Handle->InPipe, 0);
        }

        phost->hub = 1;

        status = USBH_OK;
    }

    return status;
}

static USBH_StatusTypeDef USBH_HUB_InterfaceDeInit (USBH_HandleTypeDef *phost )
{
    USBH_UsrLog("USB%u.%u USBH_HUB_InterfaceDeInit", get_port(phost), phost->address);

    HUB_HandleTypeDef *HUB_Handle = phost->USBH_ClassTypeDef_pData[0];

    usb_addr0_release(phost);

    if (HUB_Handle == NULL)
        return USBH_OK;

    if(HUB_Handle->InPipe != 0x00)
    {
            USBH_ClosePipe (phost, HUB_Handle->InPipe);
            USBH_FreePipe  (phost, HUB_Handle->InPipe);
            HUB_Handle->InPipe = 0;     // Reset the pipe as Free
    }

    /*
     * Clear the pointer after freeing. detach() and the hub release code
     * also free USBH_ClassTypeDef_pData[], so leaving it set here caused
     * a double free (heap corruption).
     */
    USBH_free (HUB_Handle);
    phost->USBH_ClassTypeDef_pData[0] = NULL;

    return USBH_OK;
}

static USBH_StatusTypeDef USBH_HUB_ClassRequest(USBH_HandleTypeDef *phost)
{
    USBH_StatusTypeDef status = USBH_BUSY;
    HUB_HandleTypeDef *HUB_Handle = phost->USBH_ClassTypeDef_pData[0];

    switch (HUB_Handle->ctl_state)
    {
        case HUB_REQ_IDLE:
        case HUB_REQ_GET_DESCRIPTOR:
                HUB_Handle->req_port = 1;
                if(get_hub_descriptor(phost) == USBH_OK) {
                        if (HUB_Handle->num_ports == 0)
                                HUB_Handle->ctl_state = HUB_WAIT_PWRGOOD;
                        else
                                HUB_Handle->ctl_state = HUB_REQ_SET_POWER;
                }
                break;

        case HUB_REQ_SET_POWER:
                // Turn on power for each hub port...
                if(set_hub_port_power(phost, HUB_Handle->req_port) == USBH_OK)
                {
                        // Reach last port
                        if(HUB_Handle->req_port >= HUB_Handle->num_ports)
                                HUB_Handle->ctl_state = HUB_WAIT_PWRGOOD;
                        else
                                HUB_Handle->req_port++;
                }
                break;

        case HUB_WAIT_PWRGOOD:
                USBH_Delay(HUB_Handle->pwr_good);
                HUB_Handle->ctl_state = HUB_REQ_DONE;
                break;

        case HUB_REQ_DONE:
                USBH_UsrLog("USB%s HUB PORTS ENABLED: %d  chars=%04x "
                            "(%s port power switching)",
                            usb_get_path(phost, NULL, -1), HUB_Handle->num_ports,
                            HUB_Handle->hub_chars,
                            ((HUB_Handle->hub_chars & HUB_CHAR_LPSM_MASK) ==
                             HUB_CHAR_LPSM_INDIVIDUAL) ? "per-port" :
                            ((HUB_Handle->hub_chars & HUB_CHAR_LPSM_MASK) ==
                             HUB_CHAR_LPSM_GANGED) ? "ganged" : "no");
                USBH_UsrLog("=============================================");
                status = USBH_OK;
                break;
    }

    return status;
}

static USBH_StatusTypeDef
USBH_HUB_Process(USBH_HandleTypeDef *phost)
{
    USBH_StatusTypeDef status = USBH_OK;
    HUB_HandleTypeDef *HUB_Handle = (HUB_HandleTypeDef *)phost->USBH_ClassTypeDef_pData[0];

    if (HUB_Handle == NULL)
        return (USBH_FAIL);

    switch (HUB_Handle->state) {
        case HUB_IDLE:
            usb_addr0_release(phost);
            HUB_Handle->cur_port = 0;
            HUB_Handle->state = HUB_SYNC;
            break;

        case HUB_SYNC:
            /* Sync with start of Even Frame */
            if (phost->Timer & 1)
                HUB_Handle->state = HUB_GET_DATA;
            break;

        case HUB_GET_DATA:
            USBH_InterruptReceiveData(phost, HUB_Handle->buffer,
                                      HUB_Handle->length, HUB_Handle->InPipe);
            HUB_Handle->state = HUB_POLL;
            HUB_Handle->timer = phost->Timer;
            HUB_Handle->DataReady = 0;
            break;

        case HUB_POLL: {
            USBH_URBStateTypeDef state;
            state = USBH_LL_GetURBState(phost, HUB_Handle->InPipe);
            if (state == USBH_URB_DONE) {
// LOG1("^");
                if (HUB_Handle->DataReady == 0) {
                    HUB_Handle->DataReady = 1;
                    if (port_changed(phost, HUB_Handle, HUB_Handle->buffer))
                        HUB_Handle->state = HUB_LOOP_PORT_CHANGED;
                    else
                        HUB_Handle->state = HUB_GET_DATA;
                }
            } else if (state == USBH_URB_STALL) { /* IN Endpoint Stalled */
// LOG1("s");
                // Issue Clear Feature on interrupt IN endpoint
                if( (USBH_ClrFeature(phost, HUB_Handle->ep_addr)) == USBH_OK) {
                    // Change state to issue next IN token
                    HUB_Handle->state = HUB_GET_DATA;
                }
            } else if (state == USBH_URB_ERROR) {
// LOG1("=");
            }
            break;
        }

        case HUB_LOOP_PORT_CHANGED:

            HUB_Handle->cur_port = get_port_changed(HUB_Handle);
            if (HUB_Handle->cur_port > 0) {
                USBH_UsrLog("USB%s HUB_CurPort %d",
                            usb_get_path(phost, NULL, -1), HUB_Handle->cur_port);

                clear_port_changed(HUB_Handle, HUB_Handle->cur_port);
                HUB_Handle->state = HUB_PORT_CHANGED;
            } else {
                HUB_Handle->state = HUB_IDLE;
            }
            break;

        case HUB_PORT_CHANGED:
            // uses EP0
            status = get_hub_request(phost, USB_REQUEST_GET_STATUS,
                                     HUB_FEAT_SEL_PORT_CONN, HUB_Handle->cur_port,
                                     HUB_Handle->buffer,
                                     sizeof (USB_HUB_PORT_STATUS));
            if (status == USBH_OK) {
                HUB_Handle->change_info = (USB_HUB_PORT_STATUS *)HUB_Handle->buffer;

                debug_port(phost, HUB_Handle);

                if (HUB_Handle->change_info->wPortStatus.PORT_POWER) {
                    if (HUB_Handle->change_info->wPortChange.C_PORT_OVER_CURRENT) {
                        printf("USB%s OVER_CURRENT\n",
                               usb_get_path(phost, NULL, HUB_Handle->cur_port));
                        HUB_Handle->state = HUB_C_PORT_OVER_CURRENT;
                        break;
                    }

                    if (HUB_Handle->change_info->wPortChange.C_PORT_SUSPEND) {
                        printf("USB%s C_PORT_SUSPEND\n",
                               usb_get_path(phost, NULL, HUB_Handle->cur_port));
                        HUB_Handle->state = HUB_C_PORT_SUSPEND;
                        break;
                    }

                    if (HUB_Handle->change_info->wPortChange.C_PORT_CONNECTION) {
                        HUB_Handle->state = HUB_C_PORT_CONNECTION;
                    } else {
                        if (HUB_Handle->change_info->wPortStatus.PORT_CONNECTION) {
                            if (HUB_Handle->change_info->wPortStatus.PORT_RESET) {
                                HUB_Handle->state = HUB_PORT_CHANGED;
                                printf("USB%s PORT_RESET\n",
                                       usb_get_path(phost, NULL,
                                                    HUB_Handle->cur_port));
                            } else if (HUB_Handle->change_info->wPortChange.C_PORT_RESET) {
                                HUB_Handle->state = HUB_C_PORT_RESET;
                            } else if (HUB_Handle->change_info->wPortStatus.PORT_ENABLE) {
                                // Device Attached
/*                              if (HUB_Handle->change_info->wPortStatus.PORT_LOW_SPEED) {
                                    HUB_Handle->state = HUB_LOOP_PORT_CHANGED;
                                } else */
                                    HUB_Handle->state = HUB_DEV_ATTACHED;
                            } else {
                                printf("USB%s RESET_DEVICE\n",
                                       usb_get_path(phost, NULL, -1));
                                HUB_Handle->state = HUB_RESET_DEVICE;
                            }
                        } else {
                            printf("USB%s DETACHED\n",
                                   usb_get_path(phost, NULL, -1));
                            // Device Detached
                            HUB_Handle->state = HUB_DEV_DETACHED;
                        }
                    }
                }
            } else if (status == USBH_NOT_SUPPORTED) {
                printf("USB%s get status not supported for port %u\n",
                       usb_get_path(phost, NULL, -1),
                       HUB_Handle->cur_port);
                HUB_Handle->state = HUB_C_PORT_CONNECTION;
            }
            break;

        case HUB_C_PORT_SUSPEND:
            printf("USB%s HUB_C_PORT_SUSPEND\n", usb_get_path(phost, NULL, -1));
            if (clear_port_feature(phost, HUB_FEAT_SEL_C_PORT_SUSPEND,
                                   HUB_Handle->cur_port) == USBH_OK) {
                HUB_Handle->state = HUB_PORT_CHANGED;
            }
            break;

        case HUB_C_PORT_OVER_CURRENT:
            printf("USB%s HUB_C_PORT_OVER_CURRENT\n",
                   usb_get_path(phost, NULL, -1));
            if (clear_port_feature(phost, HUB_FEAT_SEL_C_PORT_OVER_CURRENT,
                                   HUB_Handle->cur_port) == USBH_OK) {
                HUB_Handle->state = HUB_PORT_CHANGED;
            }
            break;

        case HUB_C_PORT_CONNECTION:
            if (clear_port_feature(phost, HUB_FEAT_SEL_C_PORT_CONNECTION,
                                                            HUB_Handle->cur_port) == USBH_OK) {
                /* New connection (or disconnection) event */
                HUB_Handle->need_debounce = 1;
                if (HUB_Handle->power_cycling) {
                    /* Reconnect caused by our own port power cycle */
                    HUB_Handle->power_cycling = 0;
                } else {
                    HUB_Handle->enum_retries = 0;
                }
                HUB_Handle->state = HUB_PORT_CHANGED;
            }
            break;

        case HUB_C_PORT_RESET:
            if (clear_port_feature(phost, HUB_FEAT_SEL_C_PORT_RESET,
                                                        HUB_Handle->cur_port) == USBH_OK) {
                HUB_Handle->state = HUB_PORT_CHANGED;
            }
            break;

        case HUB_RESET_DEVICE:
            /*
             * Resetting the port puts its device at address 0. Wait until
             * no other hub on this bus has a device at address 0.
             */
            if (!usb_addr0_acquire(phost))
                break;
            if (HUB_Handle->need_debounce) {
                /*
                 * USB 2.0 7.1.7.3: wait at least 100 ms (TATTDB) after a
                 * connection is detected before resetting the port, so
                 * the device's power and connection are stable. This
                 * matters with bus-powered hubs and devices which are
                 * slow to come up.
                 */
                USBH_Delay(HUB_CONNECT_DEBOUNCE_MS);
                HUB_Handle->need_debounce = 0;
            }
            if (set_port_feature(phost, HUB_FEAT_SEL_PORT_RESET, HUB_Handle->cur_port) == USBH_OK) {
                USBH_Delay(150);
                HUB_Handle->state = HUB_PORT_CHANGED;
            }
            break;

        case HUB_DEV_ATTACHED:
            USBH_UsrLog("USB%s HUB_DEV_ATTACHED %s",
                        usb_get_path(phost, NULL, -1),
                        HUB_Handle->change_info->wPortStatus.PORT_LOW_SPEED ?
                        "lowspeed" : "highspeed");

            HUB_Handle->state = HUB_LOOP_PORT_ENUM;

            /* USB 2.0 7.1.7.5: reset recovery time before first request */
            USBH_Delay(HUB_RESET_RECOVERY_MS);
            /*
             * XXX: Before initiating an attach of a low speed device,
             *      it might be necessary to call HAL_HCD_HC_SetHubInfo()
             *      to set up hub information in the STM32 hardware. The
             *      doc is not very clear. It's apparently needed if the
             *      host is in high speed mode and the target is a LS or
             *      FS device, as the hub needs to proxy for the host.
             */
            attach(phost, HUB_Handle->cur_port, HUB_Handle->change_info->wPortStatus.PORT_LOW_SPEED);
            break;

        case HUB_DEV_DETACHED:
            USBH_UsrLog("USB%s HUB_DEV_DETACHED",
                        usb_get_path(phost, NULL, -1));

            HUB_Handle->state = HUB_LOOP_PORT_WAIT;
            detach(phost, HUB_Handle->cur_port);
            break;

        case HUB_LOOP_PORT_ENUM:
            /*
             * Wait for HUB port to complete enumeration before moving
             * on to other ports.
             */
            if (attach_is_complete(phost, HUB_Handle->cur_port)) {
                USBH_HandleTypeDef *pphost;
                pphost = get_portdev_handle(phost, HUB_Handle->cur_port, 0);
                if ((pphost != NULL) && pphost->valid && pphost->enum_failed &&
                    (HUB_Handle->enum_retries < HUB_ENUM_RETRIES)) {
                    /*
                     * Enumeration failed. Reset the port and try again
                     * rather than leaving the device dead until it is
                     * unplugged.
                     */
                    HUB_Handle->enum_retries++;
                    printf("USB%s enumeration failed; port reset retry %u\n",
                           usb_get_path(pphost, NULL, -1),
                           HUB_Handle->enum_retries);
                    detach(phost, HUB_Handle->cur_port);
                    if ((HUB_Handle->enum_retries == HUB_POWER_CYCLE_RETRY) &&
                        ((HUB_Handle->hub_chars & HUB_CHAR_LPSM_MASK) ==
                         HUB_CHAR_LPSM_INDIVIDUAL)) {
                        /*
                         * A bus reset doesn't always recover a device
                         * which came up in a bad state; some only
                         * recover after losing power. Like Linux
                         * (hub_port_connect), power-cycle the port
                         * partway through the retries, but only on a hub
                         * with per-port power switching, so other ports'
                         * devices aren't affected.
                         */
                        printf("USB%s power cycling port %u\n",
                               usb_get_path(phost, NULL, -1),
                               HUB_Handle->cur_port);
                        HUB_Handle->state = HUB_PORT_POWER_OFF;
                    } else {
                        USBH_Delay(100);
                        HUB_Handle->state = HUB_RESET_DEVICE;
                    }
                } else {
                    if ((pphost != NULL) && pphost->valid && !pphost->enum_failed)
                        HUB_Handle->enum_retries = 0;
                    HUB_Handle->power_cycling = 0;
                    HUB_Handle->state = HUB_LOOP_PORT_WAIT;
                }
            }
            break;

        case HUB_PORT_POWER_OFF:
            if (clear_port_feature(phost, HUB_FEAT_SEL_PORT_POWER,
                                   HUB_Handle->cur_port) == USBH_OK) {
                uint32_t off_ms = HUB_Handle->pwr_good * 2;
                if (off_ms < HUB_POWER_OFF_MIN_MS)
                    off_ms = HUB_POWER_OFF_MIN_MS;
                USBH_Delay(off_ms);
                HUB_Handle->state = HUB_PORT_POWER_ON;
            }
            break;

        case HUB_PORT_POWER_ON:
            if (set_port_feature(phost, HUB_FEAT_SEL_PORT_POWER,
                                 HUB_Handle->cur_port) == USBH_OK) {
                USBH_Delay(HUB_Handle->pwr_good);
                /*
                 * The device reconnects; let the normal status path see
                 * C_PORT_CONNECTION, debounce, reset and attach. The
                 * power_cycling flag keeps that reconnect from resetting
                 * the retry count.
                 */
                HUB_Handle->power_cycling = 1;
                HUB_Handle->state = HUB_PORT_CHANGED;
            }
            break;

        case HUB_LOOP_PORT_WAIT:
            /* Done with this port: its device (if any) has an address */
            usb_addr0_release(phost);
            USBH_Delay(10);
            HUB_Handle->state = HUB_LOOP_PORT_CHANGED;
            break;

        case HUB_ERROR:
        default:
            break;
    }

    return status;
}

static USBH_StatusTypeDef
USBH_HUB_SOFProcess(USBH_HandleTypeDef *phost)
{
/*  if (!phost->hub) {
        USBH_UsrLog("EEEERRRRRRROOORRRRRRR");
        return USBH_OK;
    }
*/

    HUB_HandleTypeDef *HUB_Handle = (HUB_HandleTypeDef *)phost->USBH_ClassTypeDef_pData[0];

    if (HUB_Handle == NULL)
        return USBH_OK;

//  if(HUB_Handle->poll != 255)
//  USBH_UsrLog("ERR %d %d", HUB_Handle->poll, HUB_Handle->length);

    if (HUB_Handle->state == HUB_POLL) {
        if ((phost->Timer - HUB_Handle->timer) >= HUB_Handle->poll) {
            HUB_Handle->state = HUB_GET_DATA;
        }
    }

    return USBH_OK;
}

static USBH_StatusTypeDef
get_hub_descriptor(USBH_HandleTypeDef *phost)
{
    USBH_StatusTypeDef status = USBH_BUSY;

    HUB_HandleTypeDef *HUB_Handle = phost->USBH_ClassTypeDef_pData[0];

    switch (HUB_Handle->desc_state) {
        case 0:
            phost->Control.setup.b.bmRequestType =
                        USB_D2H | USB_REQ_RECIPIENT_DEVICE | USB_REQ_TYPE_CLASS;
            phost->Control.setup.b.bRequest      = USB_REQ_GET_DESCRIPTOR;
            phost->Control.setup.b.wValue.bw.msb = 0;
            phost->Control.setup.b.wValue.bw.lsb = USB_DESCRIPTOR_HUB;
            phost->Control.setup.b.wIndex.w      = 0;
            phost->Control.setup.b.wLength.w     = sizeof (USB_HUB_DESCRIPTOR);

            if (USBH_CtlReq(phost, HUB_Handle->buffer,
                            sizeof (USB_HUB_DESCRIPTOR)) == USBH_OK) {
                HUB_Handle->desc_state = 1;
            }
            break;

        case 1: {
            USB_HUB_DESCRIPTOR *HUB_Desc = (USB_HUB_DESCRIPTOR *)
                                           HUB_Handle->buffer;
            HUB_Handle->num_ports = (HUB_Desc->bNbrPorts > MAX_HUB_PORTS) ?
                                    MAX_HUB_PORTS : HUB_Desc->bNbrPorts;
            HUB_Handle->pwr_good  = (HUB_Desc->bPwrOn2PwrGood * 2);
            HUB_Handle->hub_chars = HUB_Desc->wHubCharacteristics;
            HUB_Handle->desc_state = 0;
            status = USBH_OK;
            break;
        }
    }

    return status;
}

static USBH_StatusTypeDef
hub_request(USBH_HandleTypeDef *phost, uint8_t request, uint8_t feature,
            uint8_t dataDirection, uint8_t porta, uint8_t *buffer,
            uint16_t size)
{
    uint8_t bmRequestType = (dataDirection == USB_DEVICE_REQUEST_GET) ?
                                            USB_D2H : USB_H2D; // 0x23

    phost->Control.setup.b.bmRequestType =
                bmRequestType | USB_REQ_RECIPIENT_OTHER | USB_REQ_TYPE_CLASS;
    phost->Control.setup.b.bRequest      = request;
    phost->Control.setup.b.wValue.bw.msb = feature;
    phost->Control.setup.b.wValue.bw.lsb = 0;
    phost->Control.setup.b.wIndex.bw.msb = porta;
    phost->Control.setup.b.wIndex.bw.lsb = 0;
    phost->Control.setup.b.wLength.w     = size;

    return USBH_CtlReq(phost, buffer, size);

//  USBH_StatusTypeDef res = USBH_FAIL;
//  do {
//      res = USBH_CtlReq(phost, buffer, size);
//  } while (res != USBH_OK && res != USBH_FAIL);
//  return res;
}

static USBH_StatusTypeDef
set_hub_port_power(USBH_HandleTypeDef *phost, uint8_t hub_port)
{
    return hub_request(phost, USB_REQ_SET_FEATURE, HUB_FEATURE_SEL_PORT_POWER,
                       USB_DEVICE_REQUEST_SET, hub_port, 0, 0);
}

static USBH_StatusTypeDef
get_hub_request(USBH_HandleTypeDef *phost, uint8_t request, uint8_t feature,
                uint8_t porta, uint8_t *buffer, uint16_t size)
{
    return hub_request(phost, request, feature, USB_DEVICE_REQUEST_GET,
                       porta,      buffer, size);
}

static USBH_StatusTypeDef
set_hub_request(USBH_HandleTypeDef *phost, uint8_t request, uint8_t feature,
                uint8_t hub_port)
{
    return hub_request(phost, request, feature, USB_DEVICE_REQUEST_SET,
                       hub_port, 0, 0);
}

static USBH_StatusTypeDef
clear_port_feature(USBH_HandleTypeDef *phost, uint8_t feature, uint8_t hub_port)
{
    return set_hub_request(phost, USB_REQUEST_CLEAR_FEATURE, feature, hub_port);
}

static USBH_StatusTypeDef
set_port_feature(USBH_HandleTypeDef *phost, uint8_t feature, uint8_t hub_port)
{
    return set_hub_request(phost, USB_REQUEST_SET_FEATURE, feature, hub_port);
}

static void
clear_port_changed(HUB_HandleTypeDef *HUB_Handle, uint8_t hub_port)
{
    if (hub_port == 0)
        return;

    HUB_Handle->change.val &= ~BIT(hub_port - 1);
}

static uint8_t
get_port_changed(HUB_HandleTypeDef *HUB_Handle)
{
    uint bit;
    for (bit = 0; bit < 8; bit++)
        if (HUB_Handle->change.val & BIT(bit))
            return (bit + 1);

    return (0);
}

static uint8_t
port_changed(USBH_HandleTypeDef *phost, HUB_HandleTypeDef *HUB_Handle, uint8_t *b)
{
    if (b[0] != 0x00) {
        HUB_Handle->change.val |= (b[0] >> 1);

        USBH_UsrLog("USB%s PORT STATUS CHANGE [0x%02X] [%d %d %d %d]",
                    usb_get_path(phost, NULL, -1),
                    b[0], HUB_Handle->change.bPorts.PORT_1, HUB_Handle->change.bPorts.PORT_2, HUB_Handle->change.bPorts.PORT_3, HUB_Handle->change.bPorts.PORT_4);

        return (HUB_Handle->change.val != 0);
    }
    return 0;
}

/*
 * detach() - tear down the device on port <idx> of hub <phost>.
 *
 * <phost> must be the HUB's handle (not the child's handle).
 */
static void
detach(USBH_HandleTypeDef *phost, uint16_t idx)
{
    USBH_HandleTypeDef *pphost = get_portdev_handle(phost, idx, 0);
    if ((pphost != NULL) && (pphost->valid)) {
        USBH_UsrLog("USB%s detach id %u",
                    usb_get_path(pphost, NULL, -1), pphost->id);

        /* If the detached device is itself a hub, remove its children */
        USBH_remove_subdevices(pphost);

        if (pphost->pUser != NULL) {
            pphost->pUser(pphost, HOST_USER_DISCONNECTION);
        }

        if (pphost->pActiveClass != NULL) {
            pphost->pActiveClass->DeInit(pphost);
            pphost->pActiveClass = NULL;
        }

        /* Free per-device class copies made by USBH_register_class() */
        while (pphost->ClassNumber > 0) {
            pphost->ClassNumber--;
            USBH_free(pphost->pClass[pphost->ClassNumber]);
            pphost->pClass[pphost->ClassNumber] = NULL;
        }

        usb_free_address(pphost);
        pphost->busy        = 0;
        pphost->valid       = 0;

        uint32_t i;
        for (i = 0; i < USBH_MAX_DATA_BUFFER; i++) {
            pphost->device.Data[i] = 0;
        }

        for (i = 0; i < pphost->interfaces; ++i) {
            USBH_free(pphost->USBH_ClassTypeDef_pData[i]);
            pphost->USBH_ClassTypeDef_pData[i] = NULL;
        }

        pphost->interfaces = 0;
        pphost->device.is_connected = 0;
    }
}

static void
attach(USBH_HandleTypeDef *phost, uint16_t idx, uint8_t lowspeed)
{
    USBH_HandleTypeDef *pphost = get_portdev_handle(phost, idx, 1);
    if (pphost == NULL) {
        printf("USB%s attach: out of memory\n", usb_get_path(phost, NULL, -1));
        return;
    }
    pphost->hubdev = idx;
    USBH_UsrLog("USB%s attach", usb_get_path(pphost, NULL, -1));

printf("hub alloc handle %x\n", (uintptr_t) pphost);
    if (pphost->valid) {
        USBH_UsrLog("USB%s ATTACH ERROR, ALREADY ATTACHED (%x)",
                    usb_get_path(pphost, NULL, -1), pphost->valid);
        detach(phost, idx);  // phost is the hub; idx is the hub port
    }

//  memset(pphost, 0, sizeof (*pphost))
    pphost->id                  = phost->id;
    pphost->address             = usb_alloc_address(pphost);
    pphost->hub                 = 0;
    pphost->pActiveClass        = NULL;
    pphost->ClassNumber         = 0;
    pphost->Pipes               = phost->Pipes;
    pphost->pUser               = phost->pUser;
    pphost->EnumState           = ENUM_IDLE;
    pphost->RequestState        = CMD_SEND;
    pphost->Timer               = 0;
    pphost->Control.errorcount  = 0;
    pphost->Control.state       = CTRL_SETUP;
    pphost->Control.pipe_size = lowspeed ? USBH_MPS_LOWSPEED: USBH_MPS_DEFAULT;
    pphost->device.address      = USBH_ADDRESS_DEFAULT;
    pphost->device.speed        = lowspeed ? USBH_SPEED_LOW : USBH_SPEED_FULL;
    pphost->device.is_connected = 1;
    pphost->hubdev              = idx;

    HCD_HandleTypeDef *phHCD =  &_hHCD[pphost->id];
    USBH_LL_SetTimer (pphost, HAL_HCD_GetCurrentFrame(phHCD));

    /* Link the registered classes to the USB Host handle */
    uint class;
    for (class = 0; class < phost->ClassNumber; class++)
        if (phost->pClass[class] != NULL) {
            USBH_register_class(pphost, phost->pClass[class],
                                sizeof (USBH_ClassTypeDef));
        }
    pphost->ClassNumber = class;

    pphost->gState = HOST_ENUMERATION;

    pphost->Control.pipe_out = phost->Control.pipe_out;
    pphost->Control.pipe_in  = phost->Control.pipe_in;

    uint8_t i;
    for (i = 0; i < USBH_MAX_NUM_INTERFACES; ++i)
        pphost->USBH_ClassTypeDef_pData[i] = NULL;

    pphost->interfaces = 0;
    pphost->enum_failed = 0;
    pphost->busy  = 1;
    pphost->valid = 3;
}

static uint8_t
attach_is_complete(USBH_HandleTypeDef *phost, uint16_t idx)
{
    USBH_HandleTypeDef *pphost = get_portdev_handle(phost, idx, 0);

    /* Don't wait forever on a device which went away or failed to attach */
    if ((pphost == NULL) || (pphost->valid == 0) || pphost->enum_failed)
        return (1);

    /*
     * Wait through SET_CONFIGURATION (and the optional remote wakeup
     * request) as well as enumeration, so a failure there is also seen
     * by the hub and can be retried with a port reset.
     */
    if (pphost->EnumState != ENUM_DONE)
        return (0);
    switch (pphost->gState) {
        case HOST_ENUMERATION:
        case HOST_INPUT:
        case HOST_SET_CONFIGURATION:
        case HOST_SET_WAKEUP_FEATURE:
            return (0);
        default:
            return (1);
    }
}

static void
debug_port(USBH_HandleTypeDef *phost, HUB_HandleTypeDef *HUB_Handle)
{
#if 1
    uint8_t *buff = HUB_Handle->buffer;
    USB_HUB_PORT_STATUS *info = HUB_Handle->change_info;
    LOG1("USB%s PORT STATUS [%02X %02X %02X %02X] ",
         usb_get_path(phost, NULL, HUB_Handle->cur_port),
         buff[0], buff[1], buff[2], buff[3]);

    if (info->wPortStatus.PORT_CONNECTION)       LOG1("PORT_CONNECTION ");
    if (info->wPortStatus.PORT_ENABLE)           LOG1("PORT_ENABLE ");
    if (info->wPortStatus.PORT_SUSPEND)          LOG1("PORT_SUSPEND ");
    if (info->wPortStatus.PORT_OVER_CURRENT)     LOG1("PORT_OVER_CURRENT ");
    if (info->wPortStatus.PORT_RESET)            LOG1("PORT_RESET ");
    if (info->wPortStatus.PORT_POWER)            LOG1("PORT_POWER ");
    if (info->wPortStatus.PORT_LOW_SPEED)        LOG1("PORT_LOW_SPEED ");
    if (info->wPortStatus.PORT_HIGH_SPEED)       LOG1("PORT_HIGH_SPEED ");
    if (info->wPortStatus.PORT_TEST)             LOG1("PORT_TEST ");
    if (info->wPortStatus.PORT_INDICATOR)        LOG1("PORT_INDICATOR ");

    if (info->wPortChange.C_PORT_CONNECTION)     LOG1("C_PORT_CONNECTION ");
    if (info->wPortChange.C_PORT_ENABLE)         LOG1("C_PORT_ENABLE ");
    if (info->wPortChange.C_PORT_SUSPEND)        LOG1("C_PORT_SUSPEND ");
    if (info->wPortChange.C_PORT_OVER_CURRENT)   LOG1("C_PORT_OVER_CURRENT ");
    if (info->wPortChange.C_PORT_RESET)          LOG1("C_PORT_RESET ");
    LOG1("\r\n");
#endif
}
