/**
  ******************************************************************************
  * @file    usbh_hid_generic.c
  ******************************************************************************
  */

#include "usbh_hid_mouse.h"
#include "usbh_hid_parser.h"
#include "config.h"

#if 0
extern uint32_t                  mouse_report_data[2];
extern uint32_t                  mouse_rx_report_buf[2];
#endif


/* Nintendo Switch Pro Controller */
static USBH_StatusTypeDef
USBH_HID_vendor_nintendo_switch_pro(USBH_HandleTypeDef *phost,
                                    HID_HandleTypeDef *HID_Handle)
{
    USBH_StatusTypeDef status;

    static const uint8_t nspro_init_data[] = {
        2, 0x80, 0x02,
        2, 0x80, 0x03,
        2, 0x80, 0x02,
        2, 0x80, 0x04,
        2, 0x80, 0x04,
        /* The below command starts data flowing */
        11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02,
        16, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x10, 0x10, 0x80, 0x00, 0x00, 0x02,
        16, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x10, 0x1b, 0x80, 0x00, 0x00, 0x02,
        16, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x10, 0x12, 0x80, 0x00, 0x00, 0x09,
        16, 0x01, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x10, 0x46, 0x60, 0x00, 0x00, 0x09,
        16, 0x01, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x10, 0x26, 0x80, 0x00, 0x00, 0x02,
#if 1
        16, 0x01, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x10, 0x20, 0x60, 0x00, 0x00, 0x18,
#endif
        /* The below are needed for the 8Bitdo in Nintendo mode */
        12, 0x01, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x40, 0x01,
        12, 0x01, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x03, 0x30,
        12, 0x01, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x48, 0x01,
        12, 0x01, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x30, 0x01,
        16, 0x01, 0x0b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x38, 0x01, 0x80, 0x00, 0x11, 0x11,
        63, 0x80, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
#if 0
        /* These leave the 8Bitdo in Nintendo mode stuck vibrating */
        10, 0x10, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        10, 0x10, 0x0d, 0x00, 0x01, 0x40, 0x40, 0x00, 0x01, 0x40, 0x40,
        10, 0x10, 0x0e, 0x00, 0x01, 0x40, 0x40, 0x00, 0x01, 0x40, 0x40,
#endif
        0,
    };
    const uint8_t pipe_num = 0x03;
    static uint16_t npos = 0;
    uint8_t len = nspro_init_data[npos];
    if (len == 0) {
        npos = 0;
        printf("END\n");

        /*
         * Override HID report descriptor settings
         *
         * For mapping convenience, the buttons are placed in the same order
         * as those of the XBOX-360 controller.
         */
        HID_RDescTypeDef *rd = &HID_Handle->HID_RDesc;
        memset(rd, 0, sizeof (*rd));
        rd->usage = HID_USAGE_GAMEPAD;
        rd->bits_x         = 12;
        rd->bits_y         = 12;
        rd->bits_ac_pan    = 12;
        rd->bits_wheel     = 12;
        rd->pos_x          = 48;  // Left Joystick L/R
        rd->pos_y          = 60;  // Left Joystick U/D
        rd->pos_ac_pan     = 72;  // Right Joystick L/R
        rd->pos_wheel      = 84;  // Right Joystick U/D
        rd->pos_button[0]  = 27;  // Button B Red
        rd->pos_button[1]  = 26;  // Button A Green
        rd->pos_button[2]  = 25;  // Button Y Yellow
        rd->pos_button[3]  = 24;  // Button X Blue
        rd->pos_button[4]  = 33;  // Start
        rd->pos_button[5]  = 36;  // Center button (select?)
        rd->pos_button[6]  = 35;  // Left joystick center depressed
        rd->pos_button[7]  = 34;  // Right joystick center depressed
        rd->pos_button[8]  = 46;  // Left top Button
        rd->pos_button[9]  = 30;  // Right top Button
        rd->pos_button[10] = 37;  // Turbo button
        rd->pos_button[11] = 47;  // Left Trigger
        rd->pos_button[12] = 31;  // Right Trigger
        rd->pos_button[13] = 32;  // Back
        rd->num_buttons    = 14;
        rd->pos_jpad[0]    = 41;  // Pad Up
        rd->pos_jpad[1]    = 40;  // Pad Down
        rd->pos_jpad[2]    = 43;  // Pad Left
        rd->pos_jpad[3]    = 42;  // Pad Right
        rd->offset_xy      = -2048; // Add to x / y / wheel / pan for center
        return (USBH_OK);  // End of sequence
    }
#undef DEBUG_NINTENDO
#ifdef DEBUG_NINTENDO
    printf("Nintendo pos %x  tx %x on pipe %x\n", npos, len, pipe_num);
#endif
    status = USBH_InterruptSendData(phost, (uint8_t *)nspro_init_data + npos + 1, len, pipe_num);
    if (status == USBH_OK)
        npos += len + 1;  // Go to next message to send
    return (USBH_BUSY);
}

/**
  * @brief  USBH_HID_GenericInit
  *         The function init the HID mouse.
  * @param  phost: Host handle
  * @retval USBH Status
  */
USBH_StatusTypeDef
USBH_HID_GenericInit(USBH_HandleTypeDef *phost, HID_HandleTypeDef *HID_Handle)
{
    uint16_t vendor  = phost->device.DevDesc.idVendor;
    uint16_t product = phost->device.DevDesc.idProduct;

    dprintf(DF_USB_DECODE_MISC, "USB%u.%u.%u genericinit\n",
            phost->id, phost->address, HID_Handle->interface);

    USBH_HID_PrepareFifo(phost, HID_Handle);

    if ((vendor == 0x057e) && (product == 0x2009)) {
        /* Nintendo Switch Pro Controller */
        dprintf(DF_USB_DECODE_MISC, "Vendor-specific Nintendo Switch Pro\n");
        HID_Handle->Vendor = USBH_HID_vendor_nintendo_switch_pro;
    }

    return (USBH_OK);
}
