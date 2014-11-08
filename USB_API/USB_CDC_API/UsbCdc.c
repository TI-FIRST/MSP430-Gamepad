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
/** @file UsbCdc.c
 *  @brief Contains APIs related to CDC (Virtual COMport) device class
 */


//
//! \cond
//

/*
 * ======== UsbCdc.c ========
 */
#include <descriptors.h>

#ifdef _CDC_


#include "../USB_Common/device.h"
#include "../USB_Common/defMSP430USB.h"
#include "../USB_Common/usb.h"                  //USB-specific Data Structures
#include "../USB_CDC_API/UsbCdc.h"

#include <string.h>

//Local Macros
#define INTFNUM_OFFSET(X)   (X - CDC0_INTFNUM)  //Get the CDC offset

static struct _CdcControl {
    uint32_t lBaudrate;
    uint8_t bDataBits;
    uint8_t bStopBits;
    uint8_t bParity;
} CdcControl[CDC_NUM_INTERFACES];

static struct _CdcWrite {
    uint16_t nCdcBytesToSend;                       //holds counter of bytes to be sent
    uint16_t nCdcBytesToSendLeft;                   //holds counter how many bytes is still to be sent
    const uint8_t* pUsbBufferToSend;               //holds the buffer with data to be sent
    uint8_t bCurrentBufferXY;                      //is 0 if current buffer to write data is X, or 1 if current buffer is Y
    uint8_t bZeroPacketSent;                       //= FALSE;
    uint8_t last_ByteSend;
} CdcWriteCtrl[CDC_NUM_INTERFACES];

static struct _CdcRead {
    uint8_t *pUserBuffer;                          //holds the current position of user's receiving buffer. If NULL- no receiving
                                                //operation started
    uint8_t *pCurrentEpPos;                        //current positon to read of received data from curent EP
    uint16_t nBytesToReceive;                       //holds how many bytes was requested by receiveData() to receive
    uint16_t nBytesToReceiveLeft;                   //holds how many bytes is still requested by receiveData() to receive
    uint8_t * pCT1;                                //holds current EPBCTxx register
    uint8_t * pCT2;                                //holds next EPBCTxx register
    uint8_t * pEP2;                                //holds addr of the next EP buffer
    uint8_t nBytesInEp;                            //how many received bytes still available in current EP
    uint8_t bCurrentBufferXY;                      //indicates which buffer is used by host to transmit data via OUT endpoint3
} CdcReadCtrl[CDC_NUM_INTERFACES];

#ifdef BRIDGE_CDC_PRESENT

static struct _CdcBridgeCtrl {
    uint8_t *uartRx;
    uint8_t *uartTx;
    uint8_t *uartIFG;
    uint16_t *usbToUartDmaChSz;
    uint16_t *usbToUartDmaChCtl;
    uint8_t ctsState;
} CdcBridgeCtrl;

#endif

extern uint16_t wUsbEventMask;

//function pointers
extern void *(*USB_TX_memcpy)(void * dest, const void * source, size_t count);
extern void *(*USB_RX_memcpy)(void * dest, const void * source, size_t count);


/*----------------------------------------------------------------------------+
 | Global Variables                                                            |
 +----------------------------------------------------------------------------*/

extern __no_init tEDB __data16 tInputEndPointDescriptorBlock[];
extern __no_init tEDB __data16 tOutputEndPointDescriptorBlock[];


void CdcResetData ()
{
    int16_t i;

    //indicates which buffer is used by host to transmit data via OUT endpoint3 - X buffer is first
    //CdcReadCtrl[intfIndex].bCurrentBufferXY = X_BUFFER;

    memset(&CdcWriteCtrl, 0, sizeof(CdcWriteCtrl));
    memset(&CdcReadCtrl, 0, sizeof(CdcReadCtrl));
    memset(&CdcControl, 0, sizeof(CdcControl));

    for (i = 0; i < CDC_NUM_INTERFACES; i++){
        CdcControl[i].bDataBits = 8;
    }
}

//
//! \endcond
//

//*****************************************************************************
//
//! Begins a Send Operation to the USB Host.
//!
//! \param *data is an array of data to be sent.
//! \param size is the number of bytes to be sent, starting from address
//! 	\b data.
//! \param intfNum selects which data should be transmitted over.
//!
//! Initiates sending of a user buffer over CDC interface \b intfNum, of size
//! \b size and starting at address \b data. If \b size is larger than the
//! packet size, the function handles all packetization and buffer management.
//! \b size has no inherent upper limit (beyond being a 16-bit value).
//!
//! In most cases where a send operation is successfully started, the function
//! will return \b kUSBCDC_sendStarted. A send operation is said to be underway. At
//! some point, either before or after the function returns, the send operation
//! will complete, barring any events that would preclude it. (Even if the
//! operation completes before the function returns, the return code will still
//! be \b kUSBCDC_sendStarted.)
//! If the bus is not connected when the function is called, the function
//! returns \b kUSBCDC_busNotAvailable, and no operation is begun. If \b size is 0,
//! the function returns \b kUSBCDC_generalError. If a previous send operation is
//! already underway for this data interface, the function returns with
//! \b kUSBCDC_intfBusyError.
//!
//! USB includes low-level mechanisms that ensure valid transmission of data.
//!
//! See Sec. 7.2 of \e "Programmer's Guide: MSP430 USB API Stack for CDC/PHDC/HID/MSC" for a detailed discussion of
//! send operations.
//!
//! \return Any of the following:
//! 			- \b kUSBCDC_sendStarted: a send operation was successfully
//! 				started
//! 			- \b kUSBCDC_intfBusyError: a previous send operation is
//! 				underway
//! 			- \b kUSBCDC_busNotAvailable: the bus is either suspended or
//! 				disconnected
//! 			- \b kUSBCDC_generalError: \b size was zero, or other error
//
//*****************************************************************************

uint8_t USBCDC_sendData (const uint8_t* data, uint16_t size, uint8_t intfNum)
{
    uint8_t edbIndex;
    uint16_t state;

    edbIndex = stUsbHandle[intfNum].edb_Index;

    if (size == 0){
        return (kUSBCDC_generalError);
    }

    state = usbDisableInEndpointInterrupt(edbIndex);

    //do not access USB memory if suspended (PLL uce BUS_ERROR
    if ((bFunctionSuspended) ||
        (bEnumerationStatus != ENUMERATION_COMPLETE)){
        //data can not be read because of USB suspended
    	usbRestoreInEndpointInterrupt(state);                                            //restore interrupt status
        return (kUSBCDC_busNotAvailable);
    }

    if (CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].nCdcBytesToSendLeft != 0){
        //the USB still sends previous data, we have to wait
    	usbRestoreInEndpointInterrupt(state);                                           //restore interrupt status
        return (kUSBCDC_intfBusyError);
    }

    //This function generate the USB interrupt. The data will be sent out from interrupt

    CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].nCdcBytesToSend = size;
    CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].nCdcBytesToSendLeft = size;
    CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].pUsbBufferToSend = data;

    //trigger Endpoint Interrupt - to start send operation
    USBIEPIFG |= 1 << (edbIndex + 1);                                       //IEPIFGx;

    usbRestoreInEndpointInterrupt(state);

    return (kUSBCDC_sendStarted);
}

//
//! \cond
//

#define EP_MAX_PACKET_SIZE_CDC      0x40

//this function is used only by USB interrupt
int16_t CdcToHostFromBuffer (uint8_t intfNum)
{
    uint8_t byte_count, nTmp2;
    uint8_t * pEP1;
    uint8_t * pEP2;
    uint8_t * pCT1;
    uint8_t * pCT2;
    uint8_t bWakeUp = FALSE;                                                   //TRUE for wake up after interrupt
    uint8_t edbIndex;

    edbIndex = stUsbHandle[intfNum].edb_Index;

    if (CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].nCdcBytesToSendLeft == 0){    //do we have somtething to send?
        if (!CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].bZeroPacketSent){        //zero packet was not yet sent
            CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].bZeroPacketSent = TRUE;

            if (CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].last_ByteSend ==
                EP_MAX_PACKET_SIZE_CDC){
                if (CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY ==
                    X_BUFFER){
                    if (tInputEndPointDescriptorBlock[edbIndex].bEPBCTX &
                        EPBCNT_NAK){
                        tInputEndPointDescriptorBlock[edbIndex].bEPBCTX = 0;
                        CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY
                            = Y_BUFFER;                                     //switch buffer
                    }
                } else {
                    if (tInputEndPointDescriptorBlock[edbIndex].bEPBCTY &
                        EPBCNT_NAK){
                        tInputEndPointDescriptorBlock[edbIndex].bEPBCTY = 0;
                        CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY
                            = X_BUFFER;                                     //switch buffer
                    }
                }
            }

            CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].nCdcBytesToSend = 0;      //nothing to send

            //call event callback function
            if (wUsbEventMask & kUSB_sendCompletedEvent){
                bWakeUp = USBCDC_handleSendCompleted(intfNum);
            }
        } //if (!bSentZeroPacket)

        return (bWakeUp);
    }

    CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].bZeroPacketSent = FALSE;          //zero packet will be not sent: we have data

    if (CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY == X_BUFFER){
        //this is the active EP buffer
        pEP1 = (uint8_t*)stUsbHandle[intfNum].iep_X_Buffer;
        pCT1 = &tInputEndPointDescriptorBlock[edbIndex].bEPBCTX;

        //second EP buffer
        pEP2 = (uint8_t*)stUsbHandle[intfNum].iep_Y_Buffer;
        pCT2 = &tInputEndPointDescriptorBlock[edbIndex].bEPBCTY;
    } else {
        //this is the active EP buffer
        pEP1 = (uint8_t*)stUsbHandle[intfNum].iep_Y_Buffer;
        pCT1 = &tInputEndPointDescriptorBlock[edbIndex].bEPBCTY;

        //second EP buffer
        pEP2 = (uint8_t*)stUsbHandle[intfNum].iep_X_Buffer;
        pCT2 = &tInputEndPointDescriptorBlock[edbIndex].bEPBCTX;
    }

    //how many byte we can send over one endpoint buffer
    byte_count =
        (CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].nCdcBytesToSendLeft >
         EP_MAX_PACKET_SIZE_CDC) ? EP_MAX_PACKET_SIZE_CDC : CdcWriteCtrl[
            INTFNUM_OFFSET(intfNum)].nCdcBytesToSendLeft;
    nTmp2 = *pCT1;

    if (nTmp2 & EPBCNT_NAK){
        USB_TX_memcpy(pEP1, CdcWriteCtrl[INTFNUM_OFFSET(
                                             intfNum)].pUsbBufferToSend,
            byte_count);                                                            //copy data into IEP3 X or Y buffer
        *pCT1 = byte_count;                                                         //Set counter for usb In-Transaction
        CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY =
            (CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY + 1) & 0x01;    //switch buffer
        CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].nCdcBytesToSendLeft -= byte_count;
        CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].pUsbBufferToSend += byte_count;       //move buffer pointer
        CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].last_ByteSend = byte_count;

        //try to send data over second buffer
        nTmp2 = *pCT2;
        if ((CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].nCdcBytesToSendLeft > 0) &&      //do we have more data to send?
            (nTmp2 & EPBCNT_NAK)){                                                  //if the second buffer is free?
            //how many byte we can send over one endpoint buffer
            byte_count =
                (CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].nCdcBytesToSendLeft >
                 EP_MAX_PACKET_SIZE_CDC) ? EP_MAX_PACKET_SIZE_CDC :
                CdcWriteCtrl[
                    INTFNUM_OFFSET(intfNum)].nCdcBytesToSendLeft;

            USB_TX_memcpy(pEP2, CdcWriteCtrl[INTFNUM_OFFSET(
                                                 intfNum)].pUsbBufferToSend,
                byte_count);                                                        //copy data into IEP3 X or Y buffer
            *pCT2 = byte_count;                                                     //Set counter for usb In-Transaction
            CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY =
                (CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY +
                 1) & 0x01;                                                         //switch buffer
            CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].nCdcBytesToSendLeft -=
                byte_count;
            CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].pUsbBufferToSend +=
                byte_count;                                                         //move buffer pointer
            CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].last_ByteSend = byte_count;
        }
    }
    return (bWakeUp);
}

//
//! \endcond
//

//*****************************************************************************
//
//! Aborts an Active Send Operation.
//!
//! \param size is the number of bytes that were sent prior to the abort action.
//! \param intfNum is the data interface for which the send should be aborted.
//!
//! Aborts an active send operation on data interface \b intfNum. Returns the
//! number of bytes that were sent prior to the abort, in \b size.
//!
//! An application may choose to call this function if sending failed, due to
//! factors such as:
//! 		- a surprise removal of the bus
//! 		- a USB suspend event
//! 		- any send operation that extends longer than desired (perhaps due
//! 			to no open COM port on the host.)
//!
//! \return \b kUSB_succeed
//
//*****************************************************************************

uint8_t USBCDC_abortSend (uint16_t* size, uint8_t intfNum)
{
    uint8_t edbIndex;
    uint16_t state;

    edbIndex = stUsbHandle[intfNum].edb_Index;

    state = usbDisableInEndpointInterrupt(edbIndex);                                                         //disable interrupts - atomic operation

    *size =
        (CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].nCdcBytesToSend -
         CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].nCdcBytesToSendLeft);
    CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].nCdcBytesToSend = 0;
    CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].nCdcBytesToSendLeft = 0;

    usbRestoreInEndpointInterrupt(state);
    return (kUSB_succeed);
}

//
//! \cond
//

//This function copies data from OUT endpoint into user's buffer
//Arguments:
//pEP - pointer to EP to copy from
//pCT - pointer to pCT control reg
//
void CopyUsbToBuff (uint8_t* pEP, uint8_t* pCT, uint8_t intfNum)
{
    uint8_t nCount;

    //how many byte we can get from one endpoint buffer
    nCount =
        (CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft >
         CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp) ? CdcReadCtrl[
            INTFNUM_OFFSET(intfNum)].nBytesInEp : CdcReadCtrl[INTFNUM_OFFSET(
                                                                  intfNum)].
        nBytesToReceiveLeft;

    USB_RX_memcpy(CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer, pEP, nCount);   //copy data from OEP3 X or Y buffer
    CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft -= nCount;
    CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer += nCount;                     //move buffer pointer
    //to read rest of data next time from this place

    if (nCount == CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp){                 //all bytes are copied from receive buffer?
        //switch current buffer
        CdcReadCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY =
            (CdcReadCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY + 1) & 0x01;

        CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp = 0;

        //clear NAK, EP ready to receive data
        *pCT = 0x00;
    } else {
        CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp -= nCount;
        CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCurrentEpPos = pEP + nCount;
    }
}

//
//! \endcond
//

//*****************************************************************************
//
//! Begins a Receive Operation from the USB Host.
//!
//! \param *data is an array to contain the data received.
//! \param size is the number of bytes to be received.
//! \param intfNum is which data interface to receive from.
//!
//! Receives \b size bytes over CDC interface \b intfNum into memory starting at
//! address \b data. \b size has no inherent upper limit (beyond being a 16-bit
//! value).
//!
//! The function may return with \b kUSBCDC_receiveStarted, indicating that a
//! receive operation is underway. The operation completes when \b size bytes
//! are received. The application should ensure that the data memory buffer be
//! available during the whole of the receive operation.
//!
//! The function may also return with \b kUSBCDC_receiveCompleted. This means that
//! the receive operation was complete by the time the function returned.
//!
//! If the bus is not connected when the function is called, the function
//! returns \b kUSBCDC_busNotAvailable, and no operation is begun. If \b size is 0,
//! the function returns \b kUSBCDC_generalError. If a previous receive operation
//! is already underway for this data interface, the function returns
//! \b kUSBCDC_intfBusyError.
//!
//! USB includes low-level mechanisms that ensure valid transmission of data.
//!
//! See Sec. 7.2 of \e "Programmer's Guide: MSP430 USB API Stack for CDC/PHDC/HID/MSC" for a detailed discussion of
//! receive operations.
//!
//! \return Any of the following:
//! 		- \b kUSBCDC_receiveStarted: A receive operation has been
//! 			succesfully started.
//! 		- \b kUSBCDC_receiveCompleted: The receive operation is already
//! 			completed.
//! 		- \b kUSBCDC_intfBusyError: a previous receive operation is
//! 			underway.
//! 		- \b kUSBCDC_busNotAvailable: the bus is either suspended or
//! 			disconnected.
//! 		- \b kUSBCDC_generalError: \b size was zero, or other error.
//
//*****************************************************************************

uint8_t USBCDC_receiveData (uint8_t* data, uint16_t size, uint8_t intfNum)
{
    uint8_t nTmp1;
    uint8_t edbIndex;
    uint16_t state;

    edbIndex = stUsbHandle[intfNum].edb_Index;

    if ((size == 0) ||                                                      //read size is 0
        (data == NULL)){
        return (kUSBCDC_generalError);
    }

    state = usbDisableOutEndpointInterrupt(edbIndex);
    //atomic operation - disable interrupts

    //do not access USB memory if suspended (PLL off). It may produce BUS_ERROR
    if ((bFunctionSuspended) ||
        (bEnumerationStatus != ENUMERATION_COMPLETE)){
        //data can not be read because of USB suspended
    	usbRestoreOutEndpointInterrupt(state);
        return (kUSBCDC_busNotAvailable);
    }

    if (CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer != NULL){          //receive process already started
    	usbRestoreOutEndpointInterrupt(state);
        return (kUSBCDC_intfBusyError);
    }

    CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceive = size;            //bytes to receive
    CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft = size;        //left bytes to receive
    CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer = data;                //set user receive buffer

    //read rest of data from buffer, if any
    if (CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp > 0){
        //copy data from pEP-endpoint into User's buffer
        CopyUsbToBuff(CdcReadCtrl[INTFNUM_OFFSET(
                                      intfNum)].pCurrentEpPos,
            CdcReadCtrl[INTFNUM_OFFSET(
                            intfNum)
            ].pCT1, intfNum);

        if (CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft == 0){ //the Receive opereation is completed
            CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer = NULL;        //no more receiving pending
            if (wUsbEventMask & kUSB_receiveCompletedEvent){
                USBCDC_handleReceiveCompleted(intfNum);                     //call event handler in interrupt context
            }
            usbRestoreOutEndpointInterrupt(state);
            return (kUSBCDC_receiveCompleted);                              //receive completed
        }

        //check other EP buffer for data - exchange pCT1 with pCT2
        if (CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 ==
            &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX){
            CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 =
                &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY;
            CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCurrentEpPos =
                (uint8_t*)stUsbHandle[intfNum].oep_Y_Buffer;
        } else {
            CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 =
                &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX;
            CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCurrentEpPos =
                (uint8_t*)stUsbHandle[intfNum].oep_X_Buffer;
        }

        nTmp1 = *CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1;
        //try read data from second buffer
        if (nTmp1 & EPBCNT_NAK){                                            //if the second buffer has received data?
            nTmp1 = nTmp1 & 0x7f;                                           //clear NAK bit
            CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp = nTmp1;        //holds how many valid bytes in the EP buffer
            CopyUsbToBuff(CdcReadCtrl[INTFNUM_OFFSET(
                                          intfNum)].pCurrentEpPos,
                CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1, intfNum);
        }

        if (CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft == 0){ //the Receive opereation is completed
            CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer = NULL;        //no more receiving pending
            if (wUsbEventMask & kUSB_receiveCompletedEvent){
                USBCDC_handleReceiveCompleted(intfNum);                     //call event handler in interrupt context
            }
            usbRestoreOutEndpointInterrupt(state);
            return (kUSBCDC_receiveCompleted);                              //receive completed
        }
    } //read rest of data from buffer, if any

    //read 'fresh' data, if available
    nTmp1 = 0;
    if (CdcReadCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY == X_BUFFER){ //this is current buffer
        if (tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX & EPBCNT_NAK){ //this buffer has a valid data packet
            //this is the active EP buffer
            //pEP1
            CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCurrentEpPos =
                (uint8_t*)stUsbHandle[intfNum].oep_X_Buffer;
            CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 =
                &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX;

            //second EP buffer
            CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pEP2 =
                (uint8_t*)stUsbHandle[intfNum].oep_Y_Buffer;
            CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2 =
                &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY;
            nTmp1 = 1;                                                      //indicate that data is available
        }
    } else {                                                                //Y_BUFFER
        if (tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY & EPBCNT_NAK){
            //this is the active EP buffer
            CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCurrentEpPos =
                (uint8_t*)stUsbHandle[intfNum].oep_Y_Buffer;
            CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 =
                &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY;

            //second EP buffer
            CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pEP2 =
                (uint8_t*)stUsbHandle[intfNum].oep_X_Buffer;
            CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2 =
                &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX;
            nTmp1 = 1;                                                      //indicate that data is available
        }
    }

    if (nTmp1){
        //how many byte we can get from one endpoint buffer
        nTmp1 = *CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1;
        while (nTmp1 == 0)
        {
            nTmp1 = *CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1;
        }

        if (nTmp1 & EPBCNT_NAK){
            nTmp1 = nTmp1 & 0x7f;                                           //clear NAK bit
            CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp = nTmp1;        //holds how many valid bytes in the EP buffer

            CopyUsbToBuff(CdcReadCtrl[INTFNUM_OFFSET(
                                          intfNum)].pCurrentEpPos,
                CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1, intfNum);

            nTmp1 = *CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2;
            //try read data from second buffer
            if ((CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft >
                 0) &&                                                      //do we have more data to send?
                (nTmp1 & EPBCNT_NAK)){                                      //if the second buffer has received data?
                nTmp1 = nTmp1 & 0x7f;                                       //clear NAK bit
                CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp = nTmp1;    //holds how many valid bytes in the EP buffer
                CopyUsbToBuff(CdcReadCtrl[INTFNUM_OFFSET(
                                              intfNum)].pEP2,
                    CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2, intfNum);
                CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 =
                    CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2;
            }
        }
    }

    if (CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft == 0){     //the Receive opereation is completed
        CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer = NULL;            //no more receiving pending
        if (wUsbEventMask & kUSB_receiveCompletedEvent){
            USBCDC_handleReceiveCompleted(intfNum);                         //call event handler in interrupt context
        }
        usbRestoreOutEndpointInterrupt(state);
        return (kUSBCDC_receiveCompleted);
    }

    //interrupts enable
    usbRestoreOutEndpointInterrupt(state);
    return (kUSBCDC_receiveStarted);
}

//
//! \cond
//

//this function is used only by USB interrupt.
//It fills user receiving buffer with received data
int16_t CdcToBufferFromHost (uint8_t intfNum)
{
    uint8_t * pEP1;
    uint8_t nTmp1;
    uint8_t bWakeUp = FALSE;                                                   //per default we do not wake up after interrupt

    uint8_t edbIndex;

    edbIndex = stUsbHandle[intfNum].edb_Index;

    if (CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft == 0){     //do we have somtething to receive?
        CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer = NULL;            //no more receiving pending
        return (bWakeUp);
    }

    //No data to receive...
    if (!((tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX |
           tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY)
          & 0x80)){
        return (bWakeUp);
    }

    if (CdcReadCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY == X_BUFFER){ //X is current buffer
        //this is the active EP buffer
        pEP1 = (uint8_t*)stUsbHandle[intfNum].oep_X_Buffer;
        CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 =
            &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX;

        //second EP buffer
        CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pEP2 =
            (uint8_t*)stUsbHandle[intfNum].oep_Y_Buffer;
        CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2 =
            &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY;
    } else {
        //this is the active EP buffer
        pEP1 = (uint8_t*)stUsbHandle[intfNum].oep_Y_Buffer;
        CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 =
            &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY;

        //second EP buffer
        CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pEP2 =
            (uint8_t*)stUsbHandle[intfNum].oep_X_Buffer;
        CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2 =
            &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX;
    }

    //how many byte we can get from one endpoint buffer
    nTmp1 = *CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1;

    if (nTmp1 & EPBCNT_NAK){
        nTmp1 = nTmp1 & 0x7f;                                                   //clear NAK bit
        CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp = nTmp1;                //holds how many valid bytes in the EP buffer

        CopyUsbToBuff(pEP1, CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1, intfNum);

        nTmp1 = *CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2;
        //try read data from second buffer
        if ((CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft > 0) &&   //do we have more data to send?
            (nTmp1 & EPBCNT_NAK)){                                              //if the second buffer has received data?
            nTmp1 = nTmp1 & 0x7f;                                               //clear NAK bit
            CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp = nTmp1;            //holds how many valid bytes in the EP buffer
            CopyUsbToBuff(CdcReadCtrl[INTFNUM_OFFSET(
                                          intfNum)].pEP2,
                CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2, intfNum);
            CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 =
                CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2;
        }
    }

    if (CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft == 0){         //the Receive opereation is completed
        CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer = NULL;                //no more receiving pending
        if (wUsbEventMask & kUSB_receiveCompletedEvent){
            bWakeUp |= USBCDC_handleReceiveCompleted(intfNum);
        }

        if (CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp){                   //Is not read data still available in the EP?
            if (wUsbEventMask & kUSB_dataReceivedEvent){
                bWakeUp |= USBCDC_handleDataReceived(intfNum);
            }
        }
    }
    return (bWakeUp);
}

//helper for USB interrupt handler
int16_t CdcIsReceiveInProgress (uint8_t intfNum)
{
    return (CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer != NULL);
}

//
//! \endcond
//

//*****************************************************************************
//
//! Aborts an Active Receive Operation.
//!
//! \param *size is the number of bytes that were received and are waiting
//! 		at the assigned address.
//! \param intfNum is the data interface for which the send should be
//! 		aborted.
//!
//! Aborts an active receive operation on CDC interface \b intfNum. Returns the
//! number of bytes that were received and transferred to the data location
//! established for this receive operation. The data moved to the buffer up to
//! that time remains valid.
//!
//! An application may choose to call this function if it decides it no longer
//! wants to receive data from the USB host. It should be noted that if a
//! continuous stream of data is being received from the host, aborting the
//! operation is akin to pressing a "pause" button; the host will be NAK'ed
//! until another receive operation is opened.
//!
//! See Sec. 7.2 of \e "Programmer's Guide: MSP430 USB API Stack for CDC/PHDC/HID/MSC" for a detailed discussion of
//! receive operations.
//!
//! \return \b kUSB_succeed
//
//*****************************************************************************

uint8_t USBCDC_abortReceive (uint16_t* size, uint8_t intfNum)
{
    //interrupts disable
	uint8_t edbIndex;
	uint16_t state;

    edbIndex = stUsbHandle[intfNum].edb_Index;
    state = usbDisableOutEndpointInterrupt(edbIndex);

    *size = 0;                                                              //set received bytes count to 0

    //is receive operation underway?
    if (CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer){
        //how many bytes are already received?
        *size = CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceive -
                CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft;

        CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp = 0;
        CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer = NULL;
        CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft = 0;
    }

    //restore interrupt status
    usbRestoreOutEndpointInterrupt(state);
    return (kUSB_succeed);
}

//*****************************************************************************
//
//! Rejects the Data Received from the Host.
//!
//! This function rejects data that has been received from the host, for
//! interface inftNum, that does not have an active receive operation underway.
//! It resides in the USB endpoint buffer and blocks further data until a
//! receive operation is opened, or until rejected. When this function is
//! called, the buffer for this interface is purged, and the data lost. This
//! frees the USB path to resume communication.
//!
//! See Sec. 7.2 of \e "Programmer's Guide: MSP430 USB API Stack for CDC/PHDC/HID/MSC" for a detailed discussion of
//! receive operations.
//!
//! \return \b kUSB_succeed
//
//*****************************************************************************

uint8_t USBCDC_rejectData (uint8_t intfNum)
{
    uint8_t edbIndex;
    uint16_t state;

    edbIndex = stUsbHandle[intfNum].edb_Index;
    state = usbDisableOutEndpointInterrupt(edbIndex);

    //atomic operation - disable interrupts

    //do not access USB memory if suspended (PLL off). It may produce BUS_ERROR
    if (bFunctionSuspended){
    	usbRestoreOutEndpointInterrupt(state);
        return (kUSBCDC_busNotAvailable);
    }

    //Is receive operation underway?
    //- do not flush buffers if any operation still active.
    if (!CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer){
        uint8_t tmp1 = tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX &
                    EPBCNT_NAK;
        uint8_t tmp2 = tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY &
                    EPBCNT_NAK;

        if (tmp1 ^ tmp2){                                                   //switch current buffer if any and only ONE of buffers
                                                                            //is full
            //switch current buffer
            CdcReadCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY =
                (CdcReadCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY +
                 1) & 0x01;
        }

        tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX = 0;               //flush buffer X
        tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY = 0;               //flush buffer Y
        CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp = 0;                //indicates that no more data available in the EP
    }

    usbRestoreOutEndpointInterrupt(state);
    return (kUSB_succeed);
}

//*****************************************************************************
//
//! Indicates the Status of the CDC Interface.
//!
//! \param intfNum is the interface number for which status is being retrieved.
//! \param bytesSent If a send operation is underway, the number of bytes that
//! 	send have been transferred to the host is returned in this location. If
//! 	no operation is underway, this returns zero.
//! \param bytesReceived If a receive operation is underway, the number of bytes
//! 	that have been transferred to the assigned memory location is returned
//! 	in this location. If no receive operation is underway, this returns
//! 	zero.
//!
//! Indicates the status of the CDC interface \b intfNum. If a send operation is
//! active for this interface, the function also returns the number of bytes
//! that have been transmitted to the host. If a receive operation is active for
//! this interface, the function also returns the number of bytes that have been
//! received from the host and are waiting at the assigned address.
//!
//! Because multiple flags can be returned, the possible values can be masked
//! together - for example, \b kUSBCDC_waitingForSend + \b kUSBCDC_dataWaiting.
//!
//! \return Any combination of the following:
//! 		- \b kUSBCDC_waitingForSend: Indicates that a send operation is
//! 			open ont his interface
//! 		- \b kUSBCDC_waitingForReceive: Indicates that a receive operation
//! 			is open on this interface
//! 		- \b kUSBCDC_dataWaiting: Indicates that data has been received
//! 			from the host for this interface, waiting in the USB receive
//! 			buffers, lacking an open receive operation to accept it.
//! 		- \b kUSBCDC_busNotAvailable: Indicates that the bus is either
//! 			suspended or disconnected. Any operations that had previously
//! 			been underway are now aborted.
//
//*****************************************************************************

uint8_t USBCDC_intfStatus (uint8_t intfNum, uint16_t* bytesSent, uint16_t* bytesReceived)
{
    uint8_t ret = 0;
    uint16_t stateIn, stateOut;
    uint8_t edbIndex;

    *bytesSent = 0;
    *bytesReceived = 0;

    edbIndex = stUsbHandle[intfNum].edb_Index;

    stateIn = usbDisableInEndpointInterrupt(edbIndex);
    stateOut = usbDisableOutEndpointInterrupt(edbIndex);

    //Is send operation underway?
    if (CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].nCdcBytesToSendLeft != 0){
        ret |= kUSBCDC_waitingForSend;
        *bytesSent = CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].nCdcBytesToSend -
                     CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].nCdcBytesToSendLeft;
    }

    //Is receive operation underway?
    if (CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer != NULL){
        ret |= kUSBCDC_waitingForReceive;
        *bytesReceived = CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceive -
                         CdcReadCtrl[INTFNUM_OFFSET(intfNum)].
                         nBytesToReceiveLeft;
    } else {                                                                //receive operation not started
        //do not access USB memory if suspended (PLL off).
        //It may produce BUS_ERROR
        if (!bFunctionSuspended){
            if ((tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX &
                 EPBCNT_NAK)  |                                             //any of buffers has a valid data packet
                (tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY &
                 EPBCNT_NAK)){
                ret |= kUSBCDC_dataWaiting;
            }
        }
    }

    if ((bFunctionSuspended) ||
        (bEnumerationStatus != ENUMERATION_COMPLETE)){
        //if suspended or not enumerated - report no other tasks pending
        ret = kUSBCDC_busNotAvailable;
    }

    //restore interrupt status
    usbRestoreInEndpointInterrupt(stateIn);
    usbRestoreOutEndpointInterrupt(stateOut);

    __no_operation();
    return (ret);
}

//*****************************************************************************
//
//! Gives the Number of Bytes in the USB Endpoint Buffer.
//!
//! \param intfNum is the data interface whose buffer is to be checked.
//!
//! Returns the number of bytes waiting in the USB endpoint buffer for
//! \b intfNum. A non-zero value generally means that no receive operation is
//! open by which these bytes can be copied to a user buffer. If the value is
//! non-zero, the application should either open a receive operation so that the
//! data can be moved out of the endpoint buffer, or the data should be rejected
//! (USBCDC_rejectData()).
//!
//! \return The number of bytes waiting in this buffer.
//
//*****************************************************************************

uint8_t USBCDC_bytesInUSBBuffer (uint8_t intfNum)
{
    uint8_t bTmp1 = 0;
    uint16_t state;
    uint8_t edbIndex;

    edbIndex = stUsbHandle[intfNum].edb_Index;

    state = usbDisableOutEndpointInterrupt(edbIndex);
    //atomic operation - disable interrupts

    if ((bFunctionSuspended) ||
        (bEnumerationStatus != ENUMERATION_COMPLETE)){
    	usbRestoreOutEndpointInterrupt(state);
        //if suspended or not enumerated - report 0 bytes available
        return (0);
    }

    if (CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp > 0){               //If a RX operation is underway, part of data may
                                                                            //was read of the OEP buffer
        bTmp1 = CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp;
        if (*CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2 & EPBCNT_NAK){       //the next buffer has a valid data packet
            bTmp1 += *CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2 & 0x7F;
        }
    } else {
        if (tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX & EPBCNT_NAK){ //this buffer has a valid data packet
            bTmp1 = tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX & 0x7F;
        }
        if (tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY & EPBCNT_NAK){ //this buffer has a valid data packet
            bTmp1 += tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY & 0x7F;
        }
    }

    usbRestoreOutEndpointInterrupt(state);
    return (bTmp1);
}

//
//! \cond
//

//----------------------------------------------------------------------------
//Line Coding Structure
//dwDTERate     | 4 | Data terminal rate, in bits per second
//bCharFormat   | 1 | Stop bits, 0 = 1 Stop bit, 1 = 1,5 Stop bits, 2 = 2 Stop bits
//bParityType   | 1 | Parity, 0 = None, 1 = Odd, 2 = Even, 3= Mark, 4 = Space
//bDataBits     | 1 | Data bits (5,6,7,8,16)
//----------------------------------------------------------------------------
uint8_t usbGetLineCoding (void)
{
    uint8_t infIndex;

    if(tSetupPacket.wIndex % 2)
    {
        infIndex = (tSetupPacket.wIndex-1) / 2;
    }
    else
    {
        infIndex = (tSetupPacket.wIndex) / 2;
    }

    abUsbRequestReturnData[6] =
        CdcControl[infIndex].bDataBits;          //Data bits = 8
    abUsbRequestReturnData[5] =
        CdcControl[infIndex].bParity;            //No Parity
    abUsbRequestReturnData[4] =
        CdcControl[infIndex].bStopBits;          //Stop bits = 1

    abUsbRequestReturnData[3] =
        CdcControl[infIndex].lBaudrate >> 24;
    abUsbRequestReturnData[2] =
        CdcControl[infIndex].lBaudrate >> 16;
    abUsbRequestReturnData[1] =
        CdcControl[infIndex].lBaudrate >> 8;
    abUsbRequestReturnData[0] =
        CdcControl[infIndex].lBaudrate;

    wBytesRemainingOnIEP0 = 0x07;                                           //amount of data to be send over EP0 to host
    usbSendDataPacketOnEP0((uint8_t*)&abUsbRequestReturnData[0]);              //send data to host

    return (FALSE);
}

//----------------------------------------------------------------------------

uint8_t usbSetLineCoding (void)
{
    usbReceiveDataPacketOnEP0((uint8_t*)&abUsbRequestIncomingData);            //receive data over EP0 from Host

    return (FALSE);
}

//----------------------------------------------------------------------------

uint8_t usbSetControlLineState (void)
{
	USBCDC_handleSetControlLineState((uint8_t)tSetupPacket.wIndex,
            (uint8_t)tSetupPacket.wValue);
    usbSendZeroLengthPacketOnIEP0();                                        //Send ZLP for status stage

    return (FALSE);
}

//----------------------------------------------------------------------------

uint8_t Handler_SetLineCoding (void)
{
    uint8_t bWakeUp;
    volatile uint8_t infIndex;

    if(tSetupPacket.wIndex % 2)
    {
        infIndex = (tSetupPacket.wIndex-1) / 2;
    }
    else
    {
        infIndex = (tSetupPacket.wIndex) / 2;
    }

    //Baudrate Settings

    CdcControl[infIndex].lBaudrate =
        (uint32_t)abUsbRequestIncomingData[3] << 24 |
        (uint32_t)abUsbRequestIncomingData[2] << 16 |
        (uint32_t)
        abUsbRequestIncomingData[1] << 8 | abUsbRequestIncomingData[0];
    bWakeUp =
        USBCDC_handleSetLineCoding(tSetupPacket.wIndex,
            CdcControl[infIndex].lBaudrate);

    return (bWakeUp);
}

#endif  //ifdef _CDC_

//
//! \endcond
//

/*----------------------------------------------------------------------------+
 | End of source file                                                          |
 +----------------------------------------------------------------------------*/
/*------------------------ Nothing Below This Line --------------------------*/
//Released_Version_4_10_02
