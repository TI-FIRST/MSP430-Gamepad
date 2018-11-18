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

#include <stdint.h>
#include "USB_API/USB_Common/usb.h"

#ifndef _DESCRIPTORS_H_
#define _DESCRIPTORS_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*-----------------------------------------------------------------------------+
| Include files                                                                |
|-----------------------------------------------------------------------------*/

//***********************************************************************************************
// CDC or HID - Define both for composite support
//***********************************************************************************************
#define _HID_          // Needed for HID interface
//***********************************************************************************************
// CONFIGURATION CONSTANTS
//***********************************************************************************************
// These constants configure the API stack and help define the USB descriptors.
// Refer to Sec. 6 of the MSP430 USB CDC API Programmer's Guide for descriptions of these constants.

// Configuration Constants that can change
// #define that relates to Device Descriptor
#define USB_VID               0x2047        // Vendor ID (VID)
#define USB_PID               0x0997        // Product ID (PID)
/*----------------------------------------------------------------------------+
| Firmware Version                                                            |
| How to detect version number of the FW running on MSP430?                   |
| on Windows Open ControlPanel->Systems->Hardware->DeviceManager->Ports->     |
|         Msp430->ApplicationUART->Details                                    |
+----------------------------------------------------------------------------*/
#define VER_FW_H              0x02          // Device release number, in binary-coded decimal
#define VER_FW_L              0x00          // Device release number, in binary-coded decimal
// If a serial number is to be reported, set this to the index within the string descriptor
//of the dummy serial number string.  It will then be automatically handled by the API.
// If no serial number is to be reported, set this to 0.
#define USB_STR_INDEX_SERNUM  3             
 #define PHDC_ENDPOINTS_NUMBER               2  // bulk in, bulk out


#define DESCRIPTOR_TOTAL_LENGTH             41    // wTotalLength, This is the sum of configuration descriptor length  + CDC descriptor length  + HID descriptor length
#define USB_NUM_INTERFACES                  1    // Number of implemented interfaces.

#define HID0_REPORT_INTERFACE              0              // Report interface number of HID0
#define HID0_OUTEP_ADDR                    0x01           // Output Endpoint number of HID0
#define HID0_INEP_ADDR                     0x81           // Input Endpoint number of HID0

#define CDC_NUM_INTERFACES                   0           //  Total Number of CDCs implemented. should set to 0 if there are no CDCs implemented.
#define HID_NUM_INTERFACES                   1           //  Total Number of HIDs implemented. should set to 0 if there are no HIDs implemented.
#define MSC_NUM_INTERFACES                   0           //  Total Number of MSCs implemented. should set to 0 if there are no MSCs implemented.
#define PHDC_NUM_INTERFACES                  0           //  Total Number of PHDCs implemented. should set to 0 if there are no PHDCs implemented.
// Interface numbers for the implemented CDSs and HIDs, This is to use in the Application(main.c) and in the interupt file(UsbIsr.c).
#define HID0_INTFNUM                0
#define MSC_MAX_LUN_NUMBER                   1           // Maximum number of LUNs supported

#define PUTWORD(x)      ((x)&0xFF),((x)>>8)

#define USB_OUTEP_INT_EN BIT0 | BIT1 
#define USB_INEP_INT_EN BIT0 | BIT1 

#define USB_USE_INTERNAL_3V3LDO TRUE
// MCLK frequency of MCU, in Hz
// For running higher frequencies the Vcore voltage adjustment may required.
// Please refer to Data Sheet of the MSP430 device you use
#define USB_PLL_XT        2                  // Defines which XT is used by the PLL (1=XT1, 2=XT2)
#define USB_XT_FREQ_VALUE       4.0   // Indicates the freq of the crystal on the oscillator indicated by USB_PLL_XT
#define USB_XT_FREQ       USBPLL_SETCLK_4_0  // Indicates the freq of the crystal on the oscillator indicated by USB_PLL_XT
#define USB_DISABLE_XT_SUSPEND 1             // If non-zero, then USB_suspend() will disable the oscillator
                                             // that is designated by USB_PLL_XT; if zero, USB_suspend won't
                                             // affect the oscillator
#define USB_DMA_CHAN             0x00        // Set to 0xFF if no DMA channel will be used 0..7 for selected DMA channel



// Controls whether the remote wakeup feature is supported by this device.
// A value of 0x20 indicates that is it supported (this value is the mask for
// the bmAttributes field in the configuration descriptor).
// A value of zero indicates remote wakeup is not supported.
// Other values are undefined, as they will interfere with bmAttributes.
#define USB_SUPPORT_REM_WAKE 0x00

// Controls whether the application is self-powered to any degree.  Should be
// set to 0x40, unless the USB device is fully supplied by the bus.
#define USB_SUPPORT_SELF_POWERED 0x80

// Controls what the device reports to the host regarding how much power it will
// consume from VBUS.  Expressed in 2mA units; that is, the number of mA
// communicated is twice the value of this field.
#define USB_MAX_POWER 0x32
//Configuration constants that can not change ( Fixed Values)
#define CDC_CLASS  2
#define HID_CLASS  3
#define MSC_CLASS  4
#define PHDC_CLASS 5

    #define MAX_PACKET_SIZE   0x40              // Max size of the USB packets.

//***********************************************************************************************
// DESCRIPTOR CONSTANTS
//***********************************************************************************************
#define SIZEOF_DEVICE_DESCRIPTOR  0x12
#define MAX_STRING_DESCRIPTOR_INDEX 5
#define report_desc_size_HID0 85
//#define SIZEOF_REPORT_DESCRIPTOR  36
//#define USBHID_REPORT_LENGTH      64  // length of whole HID report (including Report ID)
#define CONFIG_STRING_INDEX       4
#define INTF_STRING_INDEX         5
#define USB_CONFIG_VALUE          0x01
//***********************************************************************************************
// OUTWARD DECLARATIONS
//***********************************************************************************************

//Calculates the endpoint descriptor block number from given address
#define EDB(addr) ((addr&0x07)-1)

/* Structure for generic part of configuration descriptor */
struct abromConfigurationDescriptorGenric
{
    uint8_t sizeof_config_descriptor;            // bLength
     uint8_t desc_type_config;                    // bDescriptorType: 2
    uint8_t sizeof_configuration_descriptor1;    // wTotalLength
    uint8_t sizeof_configuration_descriptor2;
    uint8_t usb_num_configurations;              // bNumInterfaces
    uint8_t bconfigurationvalue;                 // bConfigurationValue
    uint8_t  config_string_index;                // iConfiguration Description offset
     uint8_t mattributes;                         // bmAttributes, bus power, remote wakeup
    uint8_t usb_max_power;                       // Max. Power Consumption at 2mA unit
};

/************************************************CDC Descriptor**************************/
struct abromConfigurationDescriptorCdc
{
// interface descriptor (9 bytes)
    uint8_t blength_intf;                          // blength: interface descriptor size
    uint8_t desc_type_interface;                  // bdescriptortype: interface
    uint8_t interface_number_cdc;                // binterfacenumber
    uint8_t balternatesetting;                   // balternatesetting: alternate setting
    uint8_t bnumendpoints;                       // bnumendpoints: three endpoints used
    uint8_t binterfaceclass;                     // binterfaceclass: communication interface class
    uint8_t binterfacesubclass;                  // binterfacesubclass: abstract control model
    uint8_t binterfaceprotocol;                  // binterfaceprotocol: common at commands 
    uint8_t intf_string_index;                      // interface:
//header functional descriptor
    uint8_t blength_header;                      // blength: endpoint descriptor size
    uint8_t bdescriptortype_header;              // bdescriptortype: cs_interface
    uint8_t bdescriptorsubtype_header;              // bdescriptorsubtype: header func desc
    uint8_t bcdcdc1;
    uint8_t bcdcdc2;                              // bcdcdc: spec release number

//call managment functional descriptor
    uint8_t bfunctionlength;                      // bfunctionlength
    uint8_t bdescriptortype_c;                      // bdescriptortype: cs_interface
    uint8_t bdescriptorsubtype_c;                  // bdescriptorsubtype: call management func desc
    uint8_t bmcapabilities;                      // bmcapabilities: d0+d1
    uint8_t intf_number_cdc;                     // bdatainterface: 0

//acm functional descriptor
    uint8_t bfunctionlength_acm;                  // bfunctionlength
    uint8_t bdescriptortype_acm;                  // bdescriptortype: cs_interface
    uint8_t bdescriptorsubtype_acm;              // bdescriptorsubtype: abstract control management desc
    uint8_t bmcapabilities_acm;                  // bmcapabilities

// Union Functional Descriptor
    uint8_t bLength_ufd;                         // Size, in bytes
    uint8_t bdescriptortype_ufd;                 // bDescriptorType: CS_INTERFACE
    uint8_t bdescriptorsubtype_ufd;              // bDescriptorSubtype: Union Functional Desc
    uint8_t bmasterinterface_ufd;                // bMasterInterface -- the controlling intf for the union
    uint8_t bslaveinterface_ufd;                 // bSlaveInterface -- the controlled intf for the union

//Interrupt end point related fields
    uint8_t sizeof_epintep_descriptor;           // blength: endpoint descriptor size
    uint8_t desc_type_epintep;                      // bdescriptortype: endpoint
    uint8_t cdc_intep_addr;                      // bendpointaddress: (in2)
    uint8_t epintep_desc_attr_type_int;          // bmattributes: interrupt
    uint8_t epintep_wmaxpacketsize1;
    uint8_t epintep_wmaxpacketsize;                 // wmaxpacketsize, 64 bytes
    uint8_t epintep_binterval;                   // binterval

// Data interface descriptor (9 bytes)
    uint8_t blength_slaveintf;                      // blength: interface descriptor size
    uint8_t desc_type_slaveinterface;              // bdescriptortype: interface
    uint8_t interface_number_slavecdc;           // binterfacenumber
    uint8_t balternatesetting_slave;             // balternatesetting: alternate setting
    uint8_t bnumendpoints_slave;                 // bnumendpoints: three endpoints used
    uint8_t binterfaceclass_slave;               // binterfaceclass: data interface class
    uint8_t binterfacesubclass_slave;            // binterfacesubclass: abstract control model
    uint8_t binterfaceprotocol_slave;            // binterfaceprotocol: common at commands
    uint8_t intf_string_index_slave;              // interface:

// Bulk out end point related fields
    uint8_t sizeof_outep_descriptor;             // blength: endpoint descriptor size
    uint8_t desc_type_outep;                      // bdescriptortype: endpoint
    uint8_t cdc_outep_addr;                      // bendpointaddress: (out3)
    uint8_t outep_desc_attr_type_bulk;              // bmattributes: bulk
    uint8_t outep_wmaxpacketsize1;
    uint8_t outep_wmaxpacketsize2;               // wmaxpacketsize, 64 bytes
    uint8_t outep_binterval;                       // binterval: ignored for bulk transfer

// Bulk in related fields
    uint8_t sizeof_inep_descriptor;              // blength: endpoint descriptor size
    uint8_t desc_type_inep;                      // bdescriptortype: endpoint
    uint8_t cdc_inep_addr;                          // bendpointaddress: (in3)
    uint8_t inep_desc_attr_type_bulk;              // bmattributes: bulk
    uint8_t inep_wmaxpacketsize1;
    uint8_t inep_wmaxpacketsize2;                // wmaxpacketsize, 64 bytes
    uint8_t inep_binterval;                      // binterval: ignored for bulk transfer
}    ;

/**************************************HID descriptor structure *************************/
struct abromConfigurationDescriptorHid
{
//INTERFACE DESCRIPTOR (9 bytes)
    uint8_t sizeof_interface_descriptor;        // Desc Length
    uint8_t desc_type_interface;                // DescriptorType
    uint8_t interface_number_hid;               // Interface number
    uint8_t balternatesetting;                  // Any alternate settings if supported
    uint8_t bnumendpoints;                      // Number of end points required
    uint8_t binterfaceclass;                    // Class ID
    uint8_t binterfacesubclass;                 // Sub class ID
    uint8_t binterfaceprotocol;                 // Protocol
    uint8_t intf_string_index;                  // String Index

//hid descriptor (9 bytes)
    uint8_t blength_hid_descriptor;             // HID Desc length
    uint8_t hid_descriptor_type;                // HID Desc Type
    uint8_t hidrevno1;                          // Rev no 
    uint8_t hidrevno2;                          // Rev no - 2nd part
    uint8_t tcountry;                              // Country code 
    uint8_t numhidclasses;                      // Number of HID classes to follow    
    uint8_t report_descriptor_type;             // Report desc type 
    uint8_t tlength;                            // Total length of report descriptor
    uint8_t size_rep_desc;

//input end point descriptor (7 bytes)
    uint8_t size_inp_endpoint_descriptor;       // End point desc size
    uint8_t desc_type_inp_endpoint;             // Desc type
    uint8_t hid_inep_addr;                      // Input end point address
    uint8_t ep_desc_attr_type_inp_int;          // Type of end point
    uint8_t  inp_wmaxpacketsize1;               // Max packet size
    uint8_t  inp_wmaxpacketsize2;
    uint8_t inp_binterval;                      // bInterval in ms

 // Output end point descriptor; (7 bytes)
    uint8_t size_out_endpoint_descriptor;       // Output endpoint desc size
    uint8_t desc_type_out_endpoint;             // Desc type
    uint8_t hid_outep_addr;                     // Output end point address
    uint8_t ep_desc_attr_type_out_int;          // End point type
    uint8_t out_wmaxpacketsize1;                // Max packet size
    uint8_t out_wmaxpacketsize2;
    uint8_t out_binterval;                      // bInterval in ms
};

/**************************************MSC descriptor structure *************************/
struct abromConfigurationDescriptorMsc
{
// INTERFACE DESCRIPTOR (9 bytes)
    uint8_t sizeof_interface_descriptor;         // Desc Length
    uint8_t desc_type_interface;                 // DescriptorType
    uint8_t interface_number_hid;                // Interface number
    uint8_t balternatesetting;                   // Any alternate settings if supported
    uint8_t bnumendpoints;                       // Number of end points required
    uint8_t binterfaceclass;                     // Class ID
    uint8_t binterfacesubclass;                  // Sub class ID
    uint8_t binterfaceprotocol;                  // Protocol
    uint8_t intf_string_index;                   // String Index

// input end point descriptor (7 bytes)
    uint8_t size_inp_endpoint_descriptor;        // End point desc size
    uint8_t desc_type_inp_endpoint;              // Desc type
    uint8_t hid_inep_addr;                       // Input end point address
    uint8_t ep_desc_attr_type_inp_int;           // Type of end point
    uint8_t  inp_wmaxpacketsize1;                // Max packet size
    uint8_t  inp_wmaxpacketsize2;
    uint8_t inp_binterval;                       // bInterval in ms

// Output end point descriptor; (7 bytes)
    uint8_t size_out_endpoint_descriptor;        // Output endpoint desc size
    uint8_t desc_type_out_endpoint;              // Desc type
    uint8_t hid_outep_addr;                      // Output end point address
    uint8_t ep_desc_attr_type_out_int;           // End point type
    uint8_t out_wmaxpacketsize1;                 // Max packet size
    uint8_t out_wmaxpacketsize2;
    uint8_t out_binterval;                       // bInterval in ms
};

/* Global structure having Generic,CDC,HID, MSC structures */
struct  abromConfigurationDescriptorGroup
{
    /* Generic part of config descriptor */
    const struct abromConfigurationDescriptorGenric abromConfigurationDescriptorGenric;
#ifdef _MSC_
    /* MSC descriptor structure */
    const struct abromConfigurationDescriptorMsc stMsc[MSC_NUM_INTERFACES];
#endif
#ifdef _CDC_ 
    /* CDC descriptor structure */
    const struct abromConfigurationDescriptorCdc stCdc[CDC_NUM_INTERFACES];
#endif
#ifdef _HID_
    /* HID descriptor structure */
    const struct abromConfigurationDescriptorHid stHid[HID_NUM_INTERFACES];
#endif
#ifdef _PHDC_
/* PDC descriptor structure */
    const struct abromConfigurationDescriptorPhdc stPhdc[PHDC_NUM_INTERFACES];
#endif
};

extern const struct  abromConfigurationDescriptorGroup abromConfigurationDescriptorGroup;
extern uint8_t const abromDeviceDescriptor[SIZEOF_DEVICE_DESCRIPTOR];
extern uint8_t const abromStringDescriptor[];
//extern uint8_t const abromReportDescriptor[SIZEOF_REPORT_DESCRIPTOR];

/* Handle Structure - Will be populated in descriptors.c based on number of CDC,HID interfaces */
struct tUsbHandle
{
    uint8_t ep_In_Addr;               // Input EP Addr 
    uint8_t ep_Out_Addr;              // Output EP Addr 
    uint8_t edb_Index;                // The EDB index 
    uint8_t dev_Class;                // Device Class- 2 for CDC, 3 for HID 
    uint16_t intepEP_X_Buffer;         // Interupt X Buffer Addr 
    uint16_t intepEP_Y_Buffer;         // Interupt Y Buffer Addr 
    uint16_t oep_X_Buffer;             // Output X buffer Addr 
    uint16_t oep_Y_Buffer;             // Output Y buffer Addr 
    uint16_t iep_X_Buffer;             // Input X Buffer Addr 
    uint16_t iep_Y_Buffer;             // Input  Y Buffer Addr 
};

extern const struct tUsbHandle stUsbHandle[CDC_NUM_INTERFACES + HID_NUM_INTERFACES + MSC_NUM_INTERFACES + PHDC_NUM_INTERFACES]; 
extern const tDEVICE_REQUEST_COMPARE tUsbRequestList[];

#ifdef __cplusplus
}
#endif

#endif

/*------------------------ Nothing Below This Line --------------------------*/

//Released_Version_4_10_02
