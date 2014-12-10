//*****************************************************************************
//
// usbdhid.h - Definitions used by HID class devices.
//
// Copyright (c) 2008-2014 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.0.12573 of the Tiva USB Library.
//
//*****************************************************************************

#ifndef __HIDUSAGE_H__
#define __HIDUSAGE_H__

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

#define USB_PID_MOUSE           0x0000
#define USB_PID_KEYBOARD        0x0001
#define USB_PID_SERIAL          0x0002
#define USB_PID_BULK            0x0003
#define USB_PID_SCOPE           0x0004
#define USB_PID_MSC             0x0005
#define USB_PID_AUDIO           0x0006
#define USB_PID_COMP_SERIAL     0x0007
#define USB_PID_COMP_AUDIO_HID  0x0008
#define USB_PID_COMP_HID_SER    0x0009
#define USB_PID_COMP_HID_DFU    0x000A
#define USB_PID_DATA_LOGGER     0x000B
#define USB_PID_COMP_HID_HID    0x000D
#define USB_PID_GAMEPAD         0x000F
#define USB_PID_LP_CGAMEPAD     0x0010
#define USB_PID_DFU             0x00FF

#define USB_HID_GENERIC_DESKTOP 0x01
#define USB_HID_BUTTONS         0x09
#define USB_HID_X               0x30
#define USB_HID_Y               0x31
#define USB_HID_Z               0x32
#define USB_HID_RX              0x33
#define USB_HID_RY              0x34
#define USB_HID_RZ              0x35
#define USB_HID_SLIDER			0x36
#define USB_HID_DIAL			0x37
#define USB_HID_VX              0x40
#define USB_HID_VY              0x41
#define USB_HID_VZ              0x42
#define USB_HID_VRX             0x43
#define USB_HID_VRY             0x44
#define USB_HID_VRZ             0x45

#define USB_HID_POINTER         0x01
#define USB_HID_MOUSE           0x02
#define USB_HID_JOYSTICK        0x04
#define USB_HID_GAME_PAD        0x05
#define USB_HID_KEYBOARD        0x06

#define USB_HID_PHYSICAL        0x00
#define USB_HID_APPLICATION     0x01
#define USB_HID_LOGICAL         0x02

#define USB_HID_USAGE_POINTER   0x0109
#define USB_HID_USAGE_BUTTONS   0x0509
#define USB_HID_USAGE_LEDS      0x0508
#define USB_HID_USAGE_KEYCODES  0x0507

#define USB_HID_INPUT_DATA      0x0000
#define USB_HID_INPUT_CONSTANT  0x0001
#define USB_HID_INPUT_ARRAY     0x0000
#define USB_HID_INPUT_VARIABLE  0x0002
#define USB_HID_INPUT_ABS       0x0000
#define USB_HID_INPUT_RELATIVE  0x0004
#define USB_HID_INPUT_NOWRAP    0x0000
#define USB_HID_INPUT_WRAP      0x0008
#define USB_HID_INPUT_LINEAR    0x0000
#define USB_HID_INPUT_NONLINEAR 0x0010
#define USB_HID_INPUT_PREFER    0x0000
#define USB_HID_INPUT_NONPREFER 0x0020
#define USB_HID_INPUT_NONULL    0x0000
#define USB_HID_INPUT_NULL      0x0040
#define USB_HID_INPUT_BITF      0x0100
#define USB_HID_INPUT_BYTES     0x0000

//*****************************************************************************
//
//! \addtogroup hid_device_class_api
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// This is the size of the g_pui8HIDInterface array in bytes.
//
//*****************************************************************************
#define HIDINTERFACE_SIZE       (9)

//*****************************************************************************
//
// This is the size of the g_pui8HIDInEndpoint array in bytes.
//
//*****************************************************************************
#define HIDINENDPOINT_SIZE      (7)

//*****************************************************************************
//
// This is the size of the g_pui8HIDOutEndpoint array in bytes.
//
//*****************************************************************************
#define HIDOUTENDPOINT_SIZE     (7)

//*****************************************************************************
//
// This is the size of the tHIDDescriptor in bytes.
//
//*****************************************************************************
#define HIDDESCRIPTOR_SIZE      (9)

//*****************************************************************************
//
//! The size of the memory that should be allocated to create a configuration
//! descriptor for a single instance of the USB HID Device.
//! This does not include the configuration descriptor which is automatically
//! ignored by the composite device class.
//
//*****************************************************************************
#define COMPOSITE_DHID_SIZE     (HIDINTERFACE_SIZE + HIDINENDPOINT_SIZE +     \
                                 HIDOUTENDPOINT_SIZE + HIDDESCRIPTOR_SIZE)

//*****************************************************************************
//
// Macros used to create the static Report Descriptors.
//
//*****************************************************************************

//*****************************************************************************
//
//! This is a macro to assist adding Usage Page entries in HID report
//! descriptors.
//!
//! \param ui8Value is the Usage Page value.
//!
//! This macro takes a value and prepares it to be placed as a Usage Page entry
//! into a HID report structure.  These are defined by the USB HID
//! specification.
//!
//! \return Not a function.
//
//*****************************************************************************
#define UsagePage(ui8Value)      0x05, ((ui8Value) & 0xff)

//*****************************************************************************
//
//! This is a macro to assist adding Usage Page entries in HID report
//! descriptors when a vendor-specific value is to be used.
//!
//! \param ui16Value is the Usage Page value.
//!
//! This macro takes a value and prepares it to be placed as a Usage Page entry
//! into a HID report structure.  These are defined by the USB HID
//! specification.  Vendor-specific values must lie in the range 0xFF00 to
//! 0xFFFF inclusive.
//!
//! \return Not a function.
//
//*****************************************************************************
#define UsagePageVendor(ui16Value)     0x06, ((ui16Value) & 0xFF),            \
                                       (((ui16Value) >> 8) & 0xFF)

//*****************************************************************************
//
//! This is a macro to assist adding Usage entries in HID report descriptors.
//!
//! \param ui8Value is the Usage value.
//!
//! This macro takes a value and prepares it to be placed as a Usage entry into
//! a HID report structure.  These are defined by the USB HID specification.
//!
//! \return Not a function.
//
//*****************************************************************************
#define Usage(ui8Value)          0x09, ((ui8Value) & 0xff)

//*****************************************************************************
//
//! This is a macro to assist adding vendor-specific Usage entries in HID
//! report descriptors.
//!
//! \param ui16Value is the vendor-specific Usage value in the range 0xFF00 to
//! 0xFFFF.
//!
//! This macro takes a value and prepares it to be placed as a Usage entry into
//! a HID report structure.  These are defined by the USB HID specification.
//!
//! \return Not a function.
//
//*****************************************************************************
#define UsageVendor(ui16Value)   0x0A, ((ui16Value) & 0xFF),            \
                                 (((ui16Value) >> 8) & 0xFF)

//*****************************************************************************
//
//! This is a macro to assist adding Usage Minimum entries in HID report
//! descriptors.
//!
//! \param ui8Value is the Usage Minimum value.
//!
//! This macro takes a value and prepares it to be placed as a Usage Minimum
//! entry into a HID report structure.  This is the first or minimum value
//! associated with a usage value.
//!
//! \return Not a function.
//
//*****************************************************************************
#define UsageMinimum(ui8Value)   0x19, ((ui8Value) & 0xff)

//*****************************************************************************
//
//! This is a macro to assist adding Usage Maximum entries in HID report
//! descriptors.
//!
//! \param ui8Value is the Usage Maximum value.
//!
//! This macro takes a value and prepares it to be placed as a Usage Maximum
//! entry into a HID report structure.  This is the last or maximum value
//! associated with a usage value.
//!
//! \return Not a function.
//
//*****************************************************************************
#define UsageMaximum(ui8Value)   0x29, ((ui8Value) & 0xff)

//*****************************************************************************
//
//! This is a macro to assist adding Logical Minimum entries in HID report
//! descriptors.
//!
//! \param i8Value is the Logical Minimum value.
//!
//! This macro takes a value and prepares it to be placed as a Logical Minimum
//! entry into a HID report structure.  This is the actual minimum value for a
//! range of values associated with a field.
//!
//! \return Not a function.
//
//*****************************************************************************
#define LogicalMinimum(i8Value)  0x15, ((i8Value) & 0xff)

//*****************************************************************************
//
//! This is a macro to assist adding Logical Maximum entries in HID report
//! descriptors.
//!
//! \param i8Value is the Logical Maximum value.
//!
//! This macro takes a value and prepares it to be placed as a Logical Maximum
//! entry into a HID report structure.  This is the actual maximum value for a
//! range of values associated with a field.
//!
//! \return Not a function.
//
//*****************************************************************************
#define LogicalMaximum(i8Value)  0x25, ((i8Value) & 0xff)

//*****************************************************************************
//
//! This is a macro to assist adding Physical Minimum entries in HID report
//! descriptors.
//!
//! \param i16Value is the Physical Minimum value.  It is a signed, 16 bit
//! number.
//!
//! This macro takes a value and prepares it to be placed as a Physical Minimum
//! entry into a HID report structure.  This is value is used in conversion of
//! the control logical value, as returned to the host in the relevant report,
//! to a physical measurement in the appropriate units.
//!
//! \return Not a function.
//
//*****************************************************************************
#define PhysicalMinimum(i16Value)                                           \
                                0x36, ((i16Value) & 0xFF),                    \
                                (((i16Value) >> 8) & 0xFF)

//*****************************************************************************
//
//! This is a macro to assist adding Physical Maximum entries in HID report
//! descriptors.
//!
//! \param i16Value is the Physical Maximum value.  It is a signed, 16 bit
//! number.
//!
//! This macro takes a value and prepares it to be placed as a Physical Maximum
//! entry into a HID report structure.  This is value is used in conversion of
//! the control logical value, as returned to the host in the relevant report,
//! to a physical measurement in the appropriate units.
//!
//! \return Not a function.
//
//*****************************************************************************
#define PhysicalMaximum(i16Value)                                           \
                                0x46, ((i16Value) & 0xFF),                    \
                                (((i16Value) >> 8) & 0xFF)

//*****************************************************************************
//
//! This is a macro to assist adding Collection entries in HID report
//! descriptors.
//!
//! \param ui8Value is the type of Collection.
//!
//! This macro takes a value and prepares it to be placed as a Collection
//! entry into a HID report structure.  This is the type of values that are
//! being grouped together, for instance input, output or features can be
//! grouped together as a collection.
//!
//! \return Not a function.
//
//*****************************************************************************
#define Collection(ui8Value)     0xa1, ((ui8Value) & 0xff)

//*****************************************************************************
//
//! This is a macro to assist adding End Collection entries in HID report
//! descriptors.
//!
//! This macro can be used to place an End Collection entry into a HID report
//! structure.  This is a tag to indicate that a collection of entries has
//! ended in the HID report structure.  This terminates a previous Collection()
//! entry.
//!
//! \return Not a function.
//
//*****************************************************************************
#define EndCollection           0xc0

//*****************************************************************************
//
//! This is a macro to assist adding Report Count entries in HID report
//! descriptors.
//!
//! \param ui8Value is the number of items in a report item.
//!
//! This macro takes a value and prepares it to be placed as a Report Count
//! entry into a HID report structure.  This is number of entries of Report
//! Size for a given item.
//!
//! \return Not a function.
//
//*****************************************************************************
#define ReportCount(ui8Value)    0x95, ((ui8Value) & 0xff)

//*****************************************************************************
//
//! This is a macro to assist adding Report ID entries in HID report
//! descriptors.
//!
//! \param ui8Value is the identifier prefix for the current report.
//!
//! This macro takes a value and prepares it to be placed as a Report ID
//! entry into a HID report structure.  This value is used as a 1 byte prefix
//! for the report it is contained within.
//!
//! \return Not a function.
//
//*****************************************************************************
#define ReportID(ui8Value)       0x85, ((ui8Value) & 0xff)

//*****************************************************************************
//
//! This is a macro to assist adding Report Size entries in HID report
//! descriptors.
//!
//! \param ui8Value is the size, in bits, of items in a report item.
//!
//! This macro takes a value and prepares it to be placed as a Report Size
//! entry into a HID report structure.  This is size in bits of the entries of
//! of a report entry.  The Report Count specifies how many entries of Report
//! Size are in a given item.  These can be individual bits or bit fields.
//!
//! \return Not a function.
//
//*****************************************************************************
#define ReportSize(ui8Value)     0x75, ((ui8Value) & 0xff)

//*****************************************************************************
//
//! This is a macro to assist adding Input entries in HID report descriptors.
//!
//! \param ui8Value is bit mask to specify the type of a set of input report
//! items.  Note that if the USB_HID_INPUT_BITF flag is required, the Input2
//! macro (which uses a 2 byte version of the Input item tag) must be used
//! instead of this macro.
//!
//! This macro takes a value and prepares it to be placed as an Input entry
//! into a HID report structure.  This specifies the type of an input item in
//! a report structure.  These refer to a bit mask of flags that indicate the
//! type of input for a set of items.
//!
//! \return Not a function.
//
//*****************************************************************************
#define Input(ui8Value)          0x81, ((ui8Value) & 0xff)

//*****************************************************************************
//
//! This is a macro to assist adding Input entries in HID report descriptors.
//!
//! \param ui16Value is bit mask to specify the type of a set of input report
//! items.  Note that this macro uses a version of the Input item tag with a
//! two byte payload and allows any of the 8 possible data bits for the tag to
//! be used.  If USB_HID_INPUT_BITF (bit 8) is not required, the Input macro
//! may be used instead.
//!
//! This macro takes a value and prepares it to be placed as an Input entry
//! into a HID report structure.  This specifies the type of an input item in
//! a report structure.  These refer to a bit mask of flags that indicate the
//! type of input for a set of items.
//!
//! \return Not a function.
//
//*****************************************************************************
#define Input2(ui16Value)       0x82, ((ui16Value) & 0xff),                   \
                                (((ui16Value) >> 8) & 0xFF)

//*****************************************************************************
//
//! This is a macro to assist adding Feature entries in HID report descriptors.
//!
//! \param ui8Value is bit mask to specify the type of a set of feature report
//! items.  Note that if the \b USB_HID_FEATURE_BITF flag is required, the
//! Feature2 macro (which uses a 2 byte version of the Feature item tag) must
//! be used instead of this macro.
//!
//! This macro takes a value and prepares it to be placed as a Feature entry
//! into a HID report structure.  This specifies the type of a feature item in
//! a report structure.  These refer to a bit mask of flags that indicate the
//! type of feature for a set of items.
//!
//! \return Not a function.
//
//*****************************************************************************
#define Feature(ui8Value)        0xB1, ((ui8Value) & 0xff)

//*****************************************************************************
//
//! This is a macro to assist adding Feature entries in HID report descriptors.
//!
//! \param ui16Value is bit mask to specify the type of a set of feature report
//! items.  Note that this macro uses a version of the Feature item tag with a
//! two byte payload and allows any of the 8 possible data bits for the tag to
//! be used.  If \b USB_HID_FEATURE_BITF (bit 8) is not required, the Feature
//! macro may be used instead.
//!
//! This macro takes a value and prepares it to be placed as a Feature entry
//! into a HID report structure.  This specifies the type of a feature item in
//! a report structure.  These refer to a bit mask of flags that indicate the
//! type of feature for a set of items.
//!
//! \return Not a function.
//
//*****************************************************************************
#define Feature2(ui16Value)     0xB2, ((ui16Value) & 0xff),                   \
                                (((ui16Value) >> 8) & 0xFF)

//*****************************************************************************
//
//! This is a macro to assist adding Output entries in HID report descriptors.
//!
//! \param ui8Value is bit mask to specify the type of a set of output report
//! items.  Note that if the \b USB_HID_OUTPUT_BITF flag is required, the
//! Output2 macro (which uses a 2 byte version of the Output item tag) must be
//! used instead of this macro.
//!
//! This macro takes a value and prepares it to be placed as an Output entry
//! into a HID report structure.  This specifies the type of an output item in
//! a report structure.  These refer to a bit mask of flags that indicate the
//! type of output for a set of items.
//!
//! \return Not a function.
//
//*****************************************************************************
#define Output(ui8Value)        0x91, ((ui8Value) & 0xff)

//*****************************************************************************
//
//! This is a macro to assist adding Output entries in HID report descriptors.
//!
//! \param ui16Value is bit mask to specify the type of a set of output report
//! items.  Note that this macro uses a version of the Output item tag with a
//! two byte payload and allows any of the 8 possible data bits for the tag to
//! be used.  If \b USB_HID_OUTPUT_BITF is not required, the Output macro
//! may be used instead.
//!
//! This macro takes a value and prepares it to be placed as an Output entry
//! into a HID report structure.  This specifies the type of an output item in
//! a report structure.  These refer to a bit mask of flags that indicate the
//! type of output for a set of items.
//!
//! \return Not a function.
//
//*****************************************************************************
#define Output2(ui16Value)      0x92, ((ui16Value) & 0xff),                   \
                                (((ui16Value) >> 8) & 0xFF)

//*****************************************************************************
//
//! This is a macro to assist adding Unit Exponent entries in HID report
//! descriptors.
//!
//! \param i8Value is the required exponent in the range [-8, 7].
//!
//! This macro takes a value and prepares it to be placed as a Unit Exponent
//! entry into a HID report structure.  This is the exponent applied to
//! PhysicalMinimum and PhysicalMaximum when scaling and converting control
//! values to "real" units.
//!
//! \return Not a function.
//
//*****************************************************************************
#define UnitExponent(i8Value)   0x55, ((i8Value) & 0x0f)

//*****************************************************************************
//
//! This is a macro to assist adding Unit entries for uncommon units in HID
//! report descriptors.
//!
//! \param ui32Value is the definition of the unit required as defined in
//! section 6.2.2.7 of the USB HID device class definition document.
//!
//! This macro takes a value and prepares it to be placed as a Unit entry into
//! a HID report structure.  Note that individual macros are defined for common
//! units and this macro is intended for use when a complex or uncommon unit
//! is needed.  It allows entry of a 5 nibble unit definition into the report
//! descriptor.
//!
//! \return Not a function.
//
//*****************************************************************************
#define Unit(ui32Value)         0x67, (ui32Value) & 0x0f),                    \
                                (((ui32Value) >> 8) & 0xFF),                  \
                                (((ui32Value) >> 16) & 0xFF),                 \
                                (((ui32Value) >> 24) & 0xFF)

//*****************************************************************************
//
//! This macro inserts a Unit entry for centimeters into a report descriptor.
//!
//*****************************************************************************
#define UnitDistance_cm         0x66, 0x11, 0x00

//*****************************************************************************
//
//! This macro inserts a Unit entry for inches into a report descriptor.
//!
//*****************************************************************************
#define UnitDistance_i          0x66, 0x13, 0x00

//*****************************************************************************
//
//! This macro inserts a Unit entry for degrees into a report descriptor.
//!
//*****************************************************************************
#define UnitRotation_deg        0x66, 0x14, 0x00

//*****************************************************************************
//
//! This macro inserts a Unit entry for radians into a report descriptor.
//!
//*****************************************************************************
#define UnitRotation_rad        0x66, 0x12, 0x00

//*****************************************************************************
//
//! This macro inserts a Unit entry for grams into a report descriptor.
//!
//*****************************************************************************
#define UnitMass_g              0x66, 0x01, 0x01

//*****************************************************************************
//
//! This macro inserts a Unit entry for seconds into a report descriptor.
//!
//*****************************************************************************
#define UnitTime_s              0x66, 0x01, 0x10

//*****************************************************************************
//
//! This macro inserts a Unit entry for temperature in Kelvin into a report
//! descriptor.
//!
//*****************************************************************************
#define UnitTemp_K              0x67, 0x01, 0x00, 0x01, 0x00

//*****************************************************************************
//
//! This macro inserts a Unit entry for temperature in Fahrenheit into a report
//! descriptor.
//!
//*****************************************************************************
#define UnitTemp_F              0x67, 0x03, 0x00, 0x01, 0x00

//*****************************************************************************
//
//! This macro inserts a Unit entry for velocity in cm/s into a report
//! descriptor.
//!
//*****************************************************************************
#define UnitVelocitySI          0x66, 0x11, 0xF0

//*****************************************************************************
//
//! This macro inserts a Unit entry for momentum in (grams * cm)/s into a
//! report descriptor.
//!
//*****************************************************************************
#define UnitMomentumSI          0x66, 0x11, 0xF1

//*****************************************************************************
//
//! This macro inserts a Unit entry for acceleration in cm/s**2 into a
//! report descriptor.
//!
//*****************************************************************************
#define UnitAccelerationSI      0x66, 0x11, 0xE0

//*****************************************************************************
//
//! This macro inserts a Unit entry for force in (cm * grams)/s**2 into a
//! report descriptor.
//!
//*****************************************************************************
#define UnitForceSI             0x66, 0x11, 0xE1

//*****************************************************************************
//
//! This macro inserts a Unit entry for energy in (grams * cm^2)/(s^2) into a
//! report descriptor.
//!
//*****************************************************************************
#define UnitEnergySI            0x66, 0x21, 0xE1

//*****************************************************************************
//
//! This macro inserts a Unit entry for angular acceleration in degrees/(s^2)
//! into a report descriptor.
//!
//*****************************************************************************
#define UnitAngAccelerationSI   0x66, 0x12, 0xE0

//*****************************************************************************
//
//! This macro inserts a Unit entry for voltage into a a report descriptor.
//!
//*****************************************************************************
#define UnitVoltage             0x67, 0x21, 0xD1, 0xF0, 0x00

//*****************************************************************************
//
//! This macro inserts a Unit entry for voltage into a a report descriptor.
//!
//*****************************************************************************
#define UnitCurrent_A           0x67, 0x01, 0x00, 0x10, 0x00

//*****************************************************************************
//
// PRIVATE
//
// The first few sections of this header are private defines that are used by
// the USB HID code and are here only to help with the application
// allocating the correct amount of memory for the HID device code.
//
//*****************************************************************************
#define USBDHID_MAX_PACKET      64

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __USBDHID_H__
