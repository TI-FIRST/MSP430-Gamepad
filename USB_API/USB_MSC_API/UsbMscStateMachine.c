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
/** @file UsbMscStateMachine.c
 *  @brief Contains APIs related to MSC task Management.
 */
//
//! \cond
//

/* 
 * ======== UsbMscStateMachine.c ========
 */
/*File includes */
#include "../USB_Common/device.h"
#include "../USB_Common/defMSP430USB.h"
#include "../USB_MSC_API/UsbMscScsi.h"
#include "../USB_MSC_API/UsbMsc.h"
#include "../USB_Common/usb.h"
#include <descriptors.h>
#include <string.h>

#ifdef _MSC_

/*Macros to indicate data direction */
#define DIRECTION_IN    0x80
#define DIRECTION_OUT   0x00

/*Buffer pointers passed by application */
extern __no_init tEDB __data16 tInputEndPointDescriptorBlock[];
extern struct _MscState MscState;

uint8_t Scsi_Verify_CBW ();

/*----------------------------------------------------------------------------+
 | Functions                                                                  |
 +----------------------------------------------------------------------------*/
void Msc_ResetStateMachine (void)
{
    MscState.bMscSendCsw = FALSE;
    MscState.Scsi_Residue = 0;
    MscState.Scsi_Status = SCSI_PASSED;             /*Variable to track command status */
    MscState.bMcsCommandSupported = TRUE;           /*Flag to indicate read/write command is recieved from host */
    MscState.bMscCbwReceived = 0;                   /*Flag to inidicate whether any CBW recieved from host*/
    MscState.bMscSendCsw = FALSE;
    MscState.isMSCConfigured = FALSE;
    MscState.bUnitAttention = FALSE;
    MscState.bMscCbwFailed = FALSE;
    MscState.bMscResetRequired = FALSE;
	MscState.stallEndpoint = FALSE;
	MscState.stallAtEndofTx = FALSE;
}

//----------------------------------------------------------------------------
/*This is the core function called by application to handle the MSC SCSI state
* machine */

//
//! \endcond
//

//*****************************************************************************
//
//! Checks to See if a SCSI Command has Been Received.
//!
//! Checks to see if a SCSI command has been received. If so, it handles it. If not, it returns
//! having taken no action.
//! The return values of this function are intended to be used with entry of low-power modes. If the
//! function returns \b kUSBMSC_okToSleep, then no further application action is required; that is,
//! either no SCSI command was received; one was received but immediately handled; or one was
//! received but the handling will be completed in the background by the API as it automatically
//! services USB interrupts.
//! If instead the function returns \b kUSBMSC_processBuffer, then the API is currently servicing a
//! SCSI READ or WRITE command, and the API requires the application to process a buffer. (See
//! Sec. 8.3.6 of \e "Programmer's Guide: MSP430 USB API Stack for CDC/PHDC/HID/MSC" for a discussion of buffer
//! processing.)
//! Note that even if the function returns these values, the values could potentially be outdated by
//! the time the application evaluates them. For this reason, it's important to disable interrupts prior
//! to calling this function. See Sec. 8.3.5 of \e "Programmer's Guide: MSP430 USB API Stack for CDC/PHDC/HID/MSC"
//! for more information.
//!
//! \return \b kUSBMSC_okToSleep or \b kUSBMSC_processBuffer
//
//*****************************************************************************

uint8_t USBMSC_poll ()
{
	uint16_t state;
    uint8_t edbIndex;
    uint8_t * pCT1;
    uint8_t * pCT2;

    edbIndex = stUsbHandle[MSC0_INTFNUM].edb_Index;
    pCT1 = &tInputEndPointDescriptorBlock[edbIndex].bEPBCTX;
    pCT2 = &tInputEndPointDescriptorBlock[edbIndex].bEPBCTY;

    //check if currently transmitting data..
    if (MscReadControl.bReadProcessing == TRUE){
    	state = usbDisableOutEndpointInterrupt(edbIndex);
        //atomic operation - disable interrupts
        if ((MscReadControl.dwBytesToSendLeft == 0) &&
            (MscReadControl.lbaCount == 0)){
            //data is no more processing - clear flags..
            MscReadControl.bReadProcessing = FALSE;
            usbRestoreOutEndpointInterrupt(state);
        } else {
            if (!(tInputEndPointDescriptorBlock[edbIndex].bEPCNF &
                  EPCNF_STALL)){                    //if it is not stalled - contiune communication
                USBIEPIFG |= 1 << (edbIndex + 1);   //trigger IN interrupt to finish data tranmition
            }
            usbRestoreOutEndpointInterrupt(state);
            return (kUSBMSC_processBuffer);
        }
    }

    if (MscState.isMSCConfigured == FALSE){
        return (kUSBMSC_okToSleep);
    }

    if (!MscState.bMscSendCsw){
        if (MscState.bMscCbwReceived){
            if (Scsi_Verify_CBW() == SUCCESS){
                //Successful reception of CBW
                //Parse the CBW opcode and invoke the right command handler function
                Scsi_Cmd_Parser(MSC0_INTFNUM);
                MscState.bMscSendCsw = TRUE;
            }
            MscState.bMscCbwReceived = FALSE;       //CBW is performed!
        } else {
            return (kUSBMSC_okToSleep);
        }
        //check if any of out pipes has pending data and trigger interrupt

        if ((MscWriteControl.pCT1 != NULL)   &&
            ((*MscWriteControl.pCT1 & EPBCNT_NAK ) ||
             (*MscWriteControl.pCT2 & EPBCNT_NAK ))){
            USBOEPIFG |= 1 << (edbIndex + 1);       //trigger OUT interrupt again
            return (kUSBMSC_processBuffer);            //do not asleep, as data is coming in
            //and follow up data perform will be required.
        }
    }

    if (MscState.bMscSendCsw){
        if (MscState.bMcsCommandSupported == TRUE){
            //watiting till transport is finished!
            if ((MscWriteControl.bWriteProcessing == FALSE) &&
                (MscReadControl.bReadProcessing == FALSE) &&
                (MscReadControl.lbaCount == 0)){
                //Send CSW
                if (MscState.stallAtEndofTx == TRUE) {
                	if ((*pCT1 & EPBCNT_NAK) && (*pCT2 & EPBCNT_NAK)) {
                		MscState.stallAtEndofTx = FALSE;
                		usbStallInEndpoint(MSC0_INTFNUM);
                	}
                }
                else if (SUCCESS == Scsi_Send_CSW(MSC0_INTFNUM)){
                    MscState.bMscSendCsw = FALSE;
                    return (kUSBMSC_okToSleep);
                }
            }		
            else {
            	MSCFromHostToBuffer();
            }
        }
    }

    return (kUSBMSC_processBuffer);                 //When MscState.bMcsCommandSupported = FALSE, bReadProcessing became true, and
                                                    //bWriteProcessing = true.
}

//
//! \cond
//

#endif //_MSC_

//
//! \endcond
//

/*----------------------------------------------------------------------------+
 | End of source file                                                          |
 +----------------------------------------------------------------------------*/
/*------------------------ Nothing Below This Line --------------------------*/
//Released_Version_4_10_02
