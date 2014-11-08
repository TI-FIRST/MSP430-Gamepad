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
/*  
 * ======== main.c ========
 * Gamepad Demo:
 *
 * This example functions as a gamepad on the host. The gamepad has a HID report as described in
 * report_desc_HID0 variable in descriptors.c. Please note that if this report structure is
 * changed then the following lengths need to be updated -
 * 1. #define report_desc_size_HID0 in descriptors.h needs to be updated with descriptor size
 * 2. report_desc_size and report_len_input need t be uopdated in descriptors.c
 * As is this demo will enumerate with 18 bytes of input report and 2 bytes of output report
 * The input and output report structures for the gamepad as described in USB_gamepad.h
 * The input reports are used to report ADC values and status of buttons (GPIO)
 * The output report is used to set/reset indicators (GPIO)
 */
#include <string.h>
#include "driverlib.h"
#include "USB_config/descriptors.h"
#include "USB_API/USB_Common/device.h"
#include "USB_API/USB_Common/usb.h"                  // USB-specific functions
#include "USB_API/USB_HID_API/UsbHid.h"
#include "USB_app/USB_gamepad.h"

/*
 * NOTE: Modify hal.h to select a specific device and board
 */
#include "hal.h"

/*********** Application specific globals **********************/
volatile uint8_t adcUpdateAvailable;
volatile USB_GamepadReportTx gamepadReportTx;
volatile USB_GamepadReportRx gamepadReportRx;
volatile uint8_t usbHidReportReceived;
volatile uint8_t usbHidReportSendComplete;

/*  
 * ======== main ========
 */
void main (void)
{
	uint32_t current;
	uint8_t update;
    
     WDT_A_hold(WDT_A_BASE); // Stop watchdog timer

    // Minumum Vcore setting required for the USB API is PMM_CORE_LEVEL_2 .
 #ifndef DRIVERLIB_LEGACY_MODE
    PMM_setVCore(PMM_CORE_LEVEL_2);
#else

    PMM_setVCore(PMM_BASE, PMM_CORE_LEVEL_2);
#endif

    // Init all GPIO ports as output to consume less current
    initPorts();
    // Init clocks for 8MHz operation
    initClocks(8000000);   // Config clocks. MCLK=SMCLK=FLL=8MHz; ACLK=REFO=32kHz
    //Init Timer to trigger ADC every 10ms
    initTimer();
    //Init ADC to capture all analog channels
    initADC();
    //Init buttons as inputs with pull-ups
    initButtons();
    //Init indicators as output low
    initIndicators();

    //Init USB & events; if a host is present, connect
    USB_setup(TRUE, TRUE);

    //Enable global interrupts
    __enable_interrupt();

    while (1)
    {
        
        switch(USB_connectionState())
        {
            // This case is executed while your device is enumerated on the
            // USB host
            case ST_ENUM_ACTIVE:

                update = 0;
            	current = buttonsPoll();
                if (current != gamepadReportTx.buttons) {
                	gamepadReportTx.buttons = current;
                	update = 1;
                }

                if (adcUpdateAvailable) {
                	adcUpdateAvailable = false;
                	update = 1;
                }

                if (update) {
                	usbHidReportSendComplete = FALSE;
                	USBHID_sendReport((const uint8_t *)&gamepadReportTx, HID0_INTFNUM);

                	while(!usbHidReportSendComplete);
                	usbHidReportSendComplete = FALSE;
                }

                if (usbHidReportReceived) {
                	usbHidReportReceived = FALSE;
                	USBHID_receiveReport((uint8_t *)&gamepadReportRx, HID0_INTFNUM);
                	setIndicators();
                }


                break;
            // These cases are executed while your device is disconnected from
            // the host (meaning, not enumerated); enumerated but suspended
            // by the host, or connected to a powered hub without a USB host
            // present.
            case ST_PHYS_DISCONNECTED:
            case ST_ENUM_SUSPENDED:
            case ST_PHYS_CONNECTED_NOENUM_SUSP:
            	__bis_SR_register(LPM3_bits + GIE);
                _NOP();
                break;

            // The default is executed for the momentary state
            // ST_ENUM_IN_PROGRESS.  Usually, this state only last a few
            // seconds.  Be sure not to enter LPM3 in this state; USB
            // communication is taking place here, and therefore the mode must
            // be LPM0 or active-CPU.
            case ST_ENUM_IN_PROGRESS:
            default:;
        }
    }  //while(1)
} //main()


/*  
 * ======== UNMI_ISR ========
 */
#if defined(__TI_COMPILER_VERSION__) || (__IAR_SYSTEMS_ICC__)
#pragma vector = UNMI_VECTOR
__interrupt void UNMI_ISR (void)
#elif defined(__GNUC__) && (__MSP430__)
void __attribute__ ((interrupt(UNMI_VECTOR))) UNMI_ISR (void)
#else
#error Compiler not found!
#endif
{
    switch (__even_in_range(SYSUNIV, SYSUNIV_BUSIFG))
    {
        case SYSUNIV_NONE:
            __no_operation();
            break;
        case SYSUNIV_NMIIFG:
            __no_operation();
            break;
        case SYSUNIV_OFIFG:
 #ifndef DRIVERLIB_LEGACY_MODE
            UCS_clearFaultFlag(UCS_XT2OFFG);
            UCS_clearFaultFlag(UCS_DCOFFG);
            SFR_clearInterrupt(SFR_OSCILLATOR_FAULT_INTERRUPT);
#else

            UCS_clearFaultFlag(UCS_BASE, UCS_XT2OFFG);
            UCS_clearFaultFlag(UCS_BASE, UCS_DCOFFG);
            SFR_clearInterrupt(SFR_BASE, SFR_OSCILLATOR_FAULT_INTERRUPT);
#endif
            break;
        case SYSUNIV_ACCVIFG:
            __no_operation();
            break;
        case SYSUNIV_BUSIFG:
            // If the CPU accesses USB memory while the USB module is
            // suspended, a "bus error" can occur.  This generates an NMI.  If
            // USB is automatically disconnecting in your software, set a
            // breakpoint here and see if execution hits it.  See the
            // Programmer's Guide for more information.
            SYSBERRIV = 0; //clear bus error flag
            USB_disable(); //Disable
    }
}

//Released_Version_4_10_02
