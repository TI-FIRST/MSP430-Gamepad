/* --COPYRIGHT--,BSD
 * Copyright (c) 2013, Texas Instruments Incorporated
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
//*****************************************************************************
//
// tec.h - Driver for the TEC Module.
//
//*****************************************************************************

#ifndef __MSP430WARE_TEC_H__
#define __MSP430WARE_TEC_H__

#include "inc/hw_memmap.h"

#ifdef __MSP430_HAS_TEV0__

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

//*****************************************************************************
//
// The following are values that can be passed to the signalHold parameter for
// functions: TEC_configureExternalClearInput().
//
//*****************************************************************************
#define TEC_EXTERNAL_CLEAR_SIGNAL_NOT_HELD                                 0x00
#define TEC_EXTERNAL_CLEAR_SIGNAL_HELD                              TECEXCLRHLD

//*****************************************************************************
//
// The following are values that can be passed to the signalHold parameter for
// functions: TEC_configureExternalFaultInput().
//
//*****************************************************************************
#define TEC_EXTERNAL_FAULT_SIGNAL_NOT_HELD                                 0x00
#define TEC_EXTERNAL_FAULT_SIGNAL_HELD                              TECXFLTHLD0

//*****************************************************************************
//
// The following are values that can be passed to the polarityBit parameter for
// functions: TEC_configureExternalClearInput().
//
//*****************************************************************************
#define TEC_EXTERNAL_CLEAR_POLARITY_FALLING_EDGE_OR_LOW_LEVEL              0x00
#define TEC_EXTERNAL_CLEAR_POLARITY_RISING_EDGE_OR_HIGH_LEVEL       TECEXCLRPOL

//*****************************************************************************
//
// The following are values that can be passed to the polarityBit parameter for
// functions: TEC_configureExternalFaultInput().
//
//*****************************************************************************
#define TEC_EXTERNAL_FAULT_POLARITY_FALLING_EDGE_OR_LOW_LEVEL              0x00
#define TEC_EXTERNAL_FAULT_POLARITY_RISING_EDGE_OR_HIGH_LEVEL       TECXFLTPOL0

//*****************************************************************************
//
// The following are values that can be passed to the signalType parameter for
// functions: TEC_configureExternalClearInput().
//
//*****************************************************************************
#define TEC_EXTERNAL_CLEAR_SIGNALTYPE_EDGE_SENSITIVE                       0x00
#define TEC_EXTERNAL_CLEAR_SIGNALTYPE_LEVEL_SENSITIVE               TECEXCLRLVS

//*****************************************************************************
//
// The following are values that can be passed to the signalType parameter for
// functions: TEC_configureExternalFaultInput().
//
//*****************************************************************************
#define TEC_EXTERNAL_FAULT_SIGNALTYPE_EDGE_SENSITIVE                       0x00
#define TEC_EXTERNAL_FAULT_SIGNALTYPE_LEVEL_SENSITIVE               TECXFLTLVS0

//*****************************************************************************
//
// The following are values that can be passed to the selectedExternalFault
// parameter for functions: TEC_configureExternalFaultInput().
//
//*****************************************************************************
#define TEC_EXTERNAL_FAULT_0                                                  0
#define TEC_EXTERNAL_FAULT_1                                                  1
#define TEC_EXTERNAL_FAULT_2                                                  2
#define TEC_EXTERNAL_FAULT_3                                                  3
#define TEC_EXTERNAL_FAULT_4                                                  4
#define TEC_EXTERNAL_FAULT_5                                                  5
#define TEC_EXTERNAL_FAULT_6                                                  6

//*****************************************************************************
//
// The following are values that can be passed to the channelEventBlock
// parameter for functions: TEC_enableExternalFaultInput(), and
// TEC_disableExternalFaultInput(); the mask parameter for functions:
// TEC_clearExternalFaultStatus(), and TEC_getExternalFaultStatus() as well as
// returned by the TEC_getExternalFaultStatus() function.
//
//*****************************************************************************
#define TEC_CE0                                                     TECXFLT0STA
#define TEC_CE1                                                     TECXFLT1STA
#define TEC_CE2                                                     TECXFLT2STA
#define TEC_CE3                                                     TECXFLT3STA
#define TEC_CE4                                                     TECXFLT4STA
#define TEC_CE5                                                     TECXFLT5STA
#define TEC_CE6                                                     TECXFLT6STA

//*****************************************************************************
//
// The following are values that can be passed to the mask parameter for
// functions: TEC_clearInterruptFlag(), TEC_getInterruptStatus(),
// TEC_enableInterrupt(), and TEC_disableInterrupt() as well as returned by the
// TEC_getInterruptStatus() function.
//
//*****************************************************************************
#define TEC_EXTERNAL_FAULT_INTERRUPT                                 TECXFLTIFG
#define TEC_EXTERNAL_CLEAR_INTERRUPT                                TECEXCLRIFG
#define TEC_AUXILIARY_CLEAR_INTERRUPT                               TECAXCLRIFG

//*****************************************************************************
//
// The following are values that can be passed toThe following are values that
// can be returned by the TEC_getExternalClearStatus() function.
//
//*****************************************************************************
#define TEC_EXTERNAL_CLEAR_DETECTED                                  TECXCLRSTA
#define TEC_EXTERNAL_CLEAR_NOT_DETECTED                                    0x00

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
extern void TEC_configureExternalClearInput(uint32_t baseAddress,
                                            uint8_t signalType,
                                            uint8_t signalHold,
                                            uint8_t polarityBit);

extern void TEC_configureExternalFaultInput(uint32_t baseAddress,
                                            uint8_t selectedExternalFault,
                                            uint16_t signalType,
                                            uint8_t signalHold,
                                            uint8_t polarityBit);

extern void TEC_enableExternalFaultInput(uint32_t baseAddress,
                                         uint8_t channelEventBlock);

extern void TEC_disableExternalFaultInput(uint32_t baseAddress,
                                          uint8_t channelEventBlock);

extern void TEC_enableExternalClearInput(uint32_t baseAddress);

extern void TEC_disableExternalClearInput(uint32_t baseAddress);

extern void TEC_enableAuxiliaryClearSignal(uint32_t baseAddress);

extern void TEC_disableAuxiliaryClearSignal(uint32_t baseAddress);

extern void TEC_clearInterruptFlag(uint32_t baseAddress,
                                   uint8_t mask);

extern uint8_t TEC_getInterruptStatus(uint32_t baseAddress,
                                      uint8_t mask);

extern void TEC_enableInterrupt(uint32_t baseAddress,
                                uint8_t mask);

extern void TEC_disableInterrupt(uint32_t baseAddress,
                                 uint8_t mask);

extern uint8_t TEC_getExternalFaultStatus(uint32_t baseAddress,
                                          uint8_t mask);

extern void TEC_clearExternalFaultStatus(uint32_t baseAddress,
                                         uint8_t mask);

extern uint8_t TEC_getExternalClearStatus(uint32_t baseAddress);

extern void TEC_clearExternalClearStatus(uint32_t baseAddress);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif
#endif // __MSP430WARE_TEC_H__
//Released_Version_4_10_02
