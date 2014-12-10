/* --COPYRIGHT--,BSD
 * Copyright (c) 2014, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/

/*-----------------------------------------------------------------------------+
| Include files                                                                |
|-----------------------------------------------------------------------------*/
#include <USB_API/USB_Common/device.h>
#include <USB_API/USB_Common/defMSP430USB.h>
#include <USB_API/USB_Common/usb.h>              // USB-specific Data Structures
#include "descriptors.h"
#include <USB_API/USB_CDC_API/UsbCdc.h>
#include <USB_API/USB_HID_API/UsbHidReq.h>
#include "hidUsage.h"
#include "hal.h"

uint16_t const report_desc_size[HID_NUM_INTERFACES] =
{
80
};
uint8_t const report_len_input[HID_NUM_INTERFACES] =
{
20
};
/*-----------------------------------------------------------------------------+
| Device Descriptor                                                            |
|-----------------------------------------------------------------------------*/
uint8_t const abromDeviceDescriptor[SIZEOF_DEVICE_DESCRIPTOR] = {
    SIZEOF_DEVICE_DESCRIPTOR,               // Length of this descriptor
    DESC_TYPE_DEVICE,                       // Type code of this descriptor
    0x00, 0x02,                             // Release of USB spec
    0x00,                                   // Device's base class code
    0x00,                                   // Device's sub class code
    0x00,                                   // Device's protocol type code
    EP0_PACKET_SIZE,                        // End point 0's packet size
    USB_VID&0xFF, USB_VID>>8,               // Vendor ID for device, TI=0x0451
                                            // You can order your own VID at www.usb.org
    USB_PID&0xFF, USB_PID>>8,               // Product ID for device,
                                            // this ID is to only with this example
    VER_FW_L, VER_FW_H,                     // Revision level of device
    1,                                      // Index of manufacturer name string desc
    2,                                      // Index of product name string desc
    USB_STR_INDEX_SERNUM,                   // Index of serial number string desc
    1                                       //  Number of configurations supported
};

/*-----------------------------------------------------------------------------+
| Configuration Descriptor                                                     |
|-----------------------------------------------------------------------------*/
const struct abromConfigurationDescriptorGroup abromConfigurationDescriptorGroup=
{
    /* Generic part */
    {
        // CONFIGURATION DESCRIPTOR (9 bytes)
        SIZEOF_CONFIG_DESCRIPTOR,                          // bLength
        DESC_TYPE_CONFIG,                                  // bDescriptorType
        DESCRIPTOR_TOTAL_LENGTH, 0x00,                     // wTotalLength
        USB_NUM_INTERFACES,                                // bNumInterfaces
        USB_CONFIG_VALUE,                                  // bConfigurationvalue
        CONFIG_STRING_INDEX,                               // iConfiguration Description offset
        USB_SUPPORT_SELF_POWERED | USB_SUPPORT_REM_WAKE,   // bmAttributes, bus power, remote wakeup
        USB_MAX_POWER                                      // Max. Power Consumption
    },

    /******************************************************* start of HID*************************************/
    {
    /*start HID[0] Here */
        {
            //-------- Descriptor for HID class device -------------------------------------
            // INTERFACE DESCRIPTOR (9 bytes) 
            SIZEOF_INTERFACE_DESCRIPTOR,        // bLength 
            DESC_TYPE_INTERFACE,                // bDescriptorType: 4 
            HID0_REPORT_INTERFACE,              // bInterfaceNumber
            0x00,                               // bAlternateSetting
            2,                                  // bNumEndpoints
            0x03,                               // bInterfaceClass: 3 = HID Device
            1,                                  // bInterfaceSubClass:
            1,                                  // bInterfaceProtocol:
            INTF_STRING_INDEX + 0,              // iInterface:1

            // HID DESCRIPTOR (9 bytes)
            0x09,                                 // bLength of HID descriptor
            0x21,                                 // HID Descriptor Type: 0x21
            0x01,0x01,                            // HID Revision number 1.01
            0x00,                                // Target country, nothing specified (00h)
            0x01,                                // Number of HID classes to follow
            0x22,                                // Report descriptor type
             (report_desc_size_HID0 & 0x0ff),  // Total length of report descriptor
              (report_desc_size_HID0  >> 8),

            SIZEOF_ENDPOINT_DESCRIPTOR,         // bLength
            DESC_TYPE_ENDPOINT,                 // bDescriptorType
            HID0_INEP_ADDR,                     // bEndpointAddress; bit7=1 for IN, bits 3-0=1 for ep1
            EP_DESC_ATTR_TYPE_INT,              // bmAttributes, interrupt transfers
            0x40, 0x00,                         // wMaxPacketSize, 64 bytes
            10,                                  // bInterval, ms

            SIZEOF_ENDPOINT_DESCRIPTOR,         // bLength
            DESC_TYPE_ENDPOINT,                 // bDescriptorType
            HID0_OUTEP_ADDR,                    // bEndpointAddress; bit7=1 for IN, bits 3-0=1 for ep1
            EP_DESC_ATTR_TYPE_INT,              // bmAttributes, interrupt transfers
            0x40, 0x00,                         // wMaxPacketSize, 64 bytes
            10,                                  // bInterval, ms
             /* end of HID[0]*/
        }

    }
    /******************************************************* end of HID**************************************/

};
/*-----------------------------------------------------------------------------+
| String Descriptor                                                            |
|-----------------------------------------------------------------------------*/
uint8_t const abromStringDescriptor[] = {

    // String index0, language support
    4,        // Length of language descriptor ID
    3,        // LANGID tag
    0x09, 0x04,    // 0x0409 for English

    // String index1, Manufacturer
    36,        // Length of this string descriptor
    3,        // bDescriptorType
    'T',0x00,'e',0x00,'x',0x00,'a',0x00,'s',0x00,' ',0x00,
    'I',0x00,'n',0x00,'s',0x00,'t',0x00,'r',0x00,'u',0x00,
    'm',0x00,'e',0x00,'n',0x00,'t',0x00,'s',0x00,

    // String index2, Product
    38,        // Length of this string descriptor
    3,        // bDescriptorType
    'M',0x00,'S',0x00,'P',0x00,'4',0x00,'3',0x00,'0',0x00,
    '-',0x00,'U',0x00,'S',0x00,'B',0x00,' ',0x00,'G',0x00,
    'a',0x00,'m',0x00,'e',0x00,'p',0x00,'a',0x00,'d',0x00,

    // String index3, Serial Number
    4,        // Length of this string descriptor
    3,        // bDescriptorType
    '0',0x00,

    // String index4, Configuration String
    22,        // Length of this string descriptor
    3,        // bDescriptorType
    'M',0x00,'S',0x00,'P',0x00,'4',0x00,'3',0x00,'0',0x00,
    ' ',0x00,'U',0x00,'S',0x00,'B',0x00,

    // String index5, Interface String
    28,        // Length of this string descriptor
    3,        // bDescriptorType
    'H',0x00,'I',0x00,'D',0x00,' ',0x00,'I',0x00,'n',0x00,
    't',0x00,'e',0x00,'r',0x00,'f',0x00,'a',0x00,'c',0x00,
    'e',0x00
};

const uint8_t report_desc_HID0 [] = {
UsagePage(USB_HID_GENERIC_DESKTOP),
Usage(USB_HID_JOYSTICK),
Collection(USB_HID_APPLICATION),
    //
    // The axis for the controller.
    //
    UsagePage(USB_HID_GENERIC_DESKTOP),
    Usage (USB_HID_POINTER),
    Collection (USB_HID_PHYSICAL),

        //
        // The X, Y and Z values which are specified as 8-bit absolute
        // position values.
        //
        Usage (USB_HID_X),
        Usage (USB_HID_Y),
        Usage (USB_HID_Z),
        Usage (USB_HID_RX),
        Usage (USB_HID_RY),
        Usage (USB_HID_RZ),
        Usage (USB_HID_SLIDER),
        Usage (USB_HID_DIAL),
        //
        // 8 16-bit absolute values.
        //
        ReportSize(16),
        ReportCount(8),
        Input(USB_HID_INPUT_DATA | USB_HID_INPUT_VARIABLE |
              USB_HID_INPUT_ABS),

        //
        // Max 32 buttons.
        //
        UsagePage(USB_HID_BUTTONS),
        UsageMinimum(1),
        UsageMaximum(NUM_BUTTONS),
        LogicalMinimum(0),
        LogicalMaximum(1),
        PhysicalMinimum(0),
        PhysicalMaximum(1),

        //
        // 8 - 1 bit values for the buttons.
        //
        ReportSize(1),
        ReportCount(32),
        Input(USB_HID_INPUT_DATA | USB_HID_INPUT_VARIABLE |
              USB_HID_INPUT_ABS),

         //
         // Max 16 indicator bits
         //
         UsagePage(USB_HID_BUTTONS),
         UsageMinimum(1),
         UsageMaximum(NUM_INDICATORS),
         LogicalMinimum(0),
         LogicalMaximum(1),
         PhysicalMinimum(0),
         PhysicalMaximum(1),

         //
         // 8 - 1 bit values for the leds.
         //
         ReportSize(1),
         ReportCount(16),
         Output(USB_HID_INPUT_DATA | USB_HID_INPUT_VARIABLE |
               USB_HID_INPUT_ABS),

    EndCollection,
EndCollection
};

const uint8_t* report_desc[HID_NUM_INTERFACES] =
{
(uint8_t*)&report_desc_HID0
};
/**** Populating the endpoint information handle here ****/

const struct tUsbHandle stUsbHandle[]=
{
    {
        HID0_INEP_ADDR,
        HID0_OUTEP_ADDR,
        0, 
        HID_CLASS,
        0,
        0,
        OEP1_X_BUFFER_ADDRESS,
        OEP1_Y_BUFFER_ADDRESS,
        IEP1_X_BUFFER_ADDRESS,
        IEP1_Y_BUFFER_ADDRESS
    }
};
//-------------DEVICE REQUEST LIST---------------------------------------------

const tDEVICE_REQUEST_COMPARE tUsbRequestList[] = 
{

    //---- HID 0 Class Requests -----//
    USB_REQ_TYPE_INPUT | USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
    USB_REQ_GET_IDLE,
    0xff,0xff,
    HID0_REPORT_INTERFACE,0x00,
    0xff,0xff,
    0xcc,&usbGetIdle,
    USB_REQ_TYPE_OUTPUT | USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
    USB_REQ_SET_IDLE,
    0xff,0xFF,                          // bValueL is index and bValueH is type
    HID0_REPORT_INTERFACE,0x00,
    0xff,0xff,
    0xcc,&usbSetIdle,
    USB_REQ_TYPE_INPUT | USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
    USB_REQ_GET_PROTOCOL,
    0xff,0xff,
    HID0_REPORT_INTERFACE,0x00,
    0xff,0xff,
    0xcc,&usbGetProtocol,
    USB_REQ_TYPE_OUTPUT | USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
    USB_REQ_SET_PROTOCOL,
    0xff,0xFF,                          // bValueL is index and bValueH is type
    HID0_REPORT_INTERFACE,0x00,
    0xff,0xff,
    0xcc,&usbSetProtocol,
    USB_REQ_TYPE_INPUT | USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,

    USB_REQ_GET_REPORT,
    0xff,0xff,
    HID0_REPORT_INTERFACE,0x00,
    0xff,0xff,
    0xcc,&usbGetReport,
// SET REPORT
    USB_REQ_TYPE_OUTPUT | USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
    USB_REQ_SET_REPORT,
    0xff,0xFF,                          // bValueL is index and bValueH is type
    HID0_REPORT_INTERFACE,0x00,
    0xff,0xff,
    0xcc,&usbSetReport,
    // GET REPORT DESCRIPTOR
    USB_REQ_TYPE_INPUT | USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_INTERFACE,
    USB_REQ_GET_DESCRIPTOR,
    0xff,DESC_TYPE_REPORT,              // bValueL is index and bValueH is type
    HID0_REPORT_INTERFACE,0x00,
    0xff,0xff,
    0xdc,&usbGetReportDescriptor,
// GET HID DESCRIPTOR
    USB_REQ_TYPE_INPUT | USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_INTERFACE,
    USB_REQ_GET_DESCRIPTOR,
    0xff,DESC_TYPE_HID,                 // bValueL is index and bValueH is type
    HID0_REPORT_INTERFACE,0x00,
    0xff,0xff,

    0xdc,&usbGetHidDescriptor,

//---- USB Standard Requests -----//
    // clear device feature
    USB_REQ_TYPE_OUTPUT | USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_DEVICE,
    USB_REQ_CLEAR_FEATURE,
    FEATURE_REMOTE_WAKEUP,0x00,         // feature selector
    0x00,0x00,
    0x00,0x00,
    0xff,&usbClearDeviceFeature,

// clear endpoint feature
    USB_REQ_TYPE_OUTPUT | USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_ENDPOINT,
    USB_REQ_CLEAR_FEATURE,
FEATURE_ENDPOINT_STALL,0x00,
    0xff,0x00,
    0x00,0x00,
    0xf7,&usbClearEndpointFeature,
// get configuration
    USB_REQ_TYPE_INPUT | USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_DEVICE,
    USB_REQ_GET_CONFIGURATION,
    0x00,0x00, 
    0x00,0x00, 
    0x01,0x00,
    0xff,&usbGetConfiguration,
// get device descriptor
    USB_REQ_TYPE_INPUT | USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_DEVICE,
    USB_REQ_GET_DESCRIPTOR,
    0xff,DESC_TYPE_DEVICE,              // bValueL is index and bValueH is type
    0xff,0xff,
    0xff,0xff,
    0xd0,&usbGetDeviceDescriptor,
// get configuration descriptor
    USB_REQ_TYPE_INPUT | USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_DEVICE,
    USB_REQ_GET_DESCRIPTOR,
    0xff,DESC_TYPE_CONFIG,              // bValueL is index and bValueH is type
    0xff,0xff,
    0xff,0xff,
    0xd0,&usbGetConfigurationDescriptor,
// get string descriptor
    USB_REQ_TYPE_INPUT | USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_DEVICE,
    USB_REQ_GET_DESCRIPTOR,
    0xff,DESC_TYPE_STRING,              // bValueL is index and bValueH is type
    0xff,0xff,
    0xff,0xff,
    0xd0,&usbGetStringDescriptor,
// get interface
    USB_REQ_TYPE_INPUT | USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_INTERFACE,
    USB_REQ_GET_INTERFACE,
    0x00,0x00,
    0xff,0xff,
    0x01,0x00,
    0xf3,&usbGetInterface,
    // get device status
    USB_REQ_TYPE_INPUT | USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_DEVICE,
    USB_REQ_GET_STATUS,
    0x00,0x00,
    0x00,0x00,
    0x02,0x00,
    0xff,&usbGetDeviceStatus,     // get interface status
    USB_REQ_TYPE_INPUT | USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_INTERFACE,
    USB_REQ_GET_STATUS,
    0x00,0x00,
    0xff,0x00,
    0x02,0x00,
    0xf7,&usbGetInterfaceStatus,
    //     get endpoint status
    USB_REQ_TYPE_INPUT | USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_ENDPOINT,
    USB_REQ_GET_STATUS,
    0x00,0x00,
    0xff,0x00,
    0x02,0x00,
    0xf7,&usbGetEndpointStatus,
    // set address
    USB_REQ_TYPE_OUTPUT | USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_DEVICE,
    USB_REQ_SET_ADDRESS,
    0xff,0x00,
    0x00,0x00,
    0x00,0x00,
    0xdf,&usbSetAddress,
    // set configuration
    USB_REQ_TYPE_OUTPUT | USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_DEVICE,
    USB_REQ_SET_CONFIGURATION,
    0xff,0x00,
    0x00,0x00,
    0x00,0x00,
    0xdf,&usbSetConfiguration,
    // set device feature
    USB_REQ_TYPE_OUTPUT | USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_DEVICE,
    USB_REQ_SET_FEATURE,
    0xff,0x00,                      // feature selector
    0x00,0x00,
    0x00,0x00,
    0xdf,&usbSetDeviceFeature,
    // set endpoint feature
    USB_REQ_TYPE_OUTPUT | USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_ENDPOINT,
    USB_REQ_SET_FEATURE,
    0xff,0x00,                      // feature selector
    0xff,0x00,                      // endpoint number <= 127
    0x00,0x00,
    0xd7,&usbSetEndpointFeature,
    // set interface
    USB_REQ_TYPE_OUTPUT | USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_INTERFACE,
    USB_REQ_SET_INTERFACE,
    0xff,0x00,                      // feature selector
    0xff,0x00,                      // interface number
    0x00,0x00,
    0xd7,&usbSetInterface,

    // end of usb descriptor -- this one will be matched to any USB request
    // since bCompareMask is 0x00.
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff, 
    0x00,&usbInvalidRequest     // end of list
};

/*-----------------------------------------------------------------------------+
| END OF Descriptor.c FILE                                                     |
|-----------------------------------------------------------------------------*/
//Released_Version_4_10_02
