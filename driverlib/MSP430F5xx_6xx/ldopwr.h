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
// ldopwr.h - Driver for the LDOPWR Module.
//
//*****************************************************************************

#ifndef __MSP430WARE_LDOPWR_H__
#define __MSP430WARE_LDOPWR_H__

#include "inc/hw_memmap.h"

#ifdef __MSP430_HAS_PU__

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
// The following are values that can be passed to the value parameter for
// functions: LDOPWR_setPort_U1_outputData(), and
// LDOPWR_setPort_U0_outputData() as well as returned by the
// LDOPWR_getPort_U1_inputData() function, the LDOPWR_getPort_U0_inputData()
// function, the LDOPWR_getPort_U1_outputData() function and the
// LDOPWR_getPort_U0_outputData() function.
//
//*****************************************************************************
#define LDOPWR_PORTU_PIN_HIGH                                            PUOUT0
#define LDOPWR_PORTU_PIN_LOW                                               0x00

//*****************************************************************************
//
// The following are values that can be passed toThe following are values that
// can be returned by the LDOPWR_isLDOInputValid() function.
//
//*****************************************************************************
#define LDOPWR_LDO_INPUT_VALID                                         LDOBGVBV
#define LDOPWR_LDO_INPUT_INVALID                                           0x00

//*****************************************************************************
//
// The following are values that can be passed toThe following are values that
// can be returned by the LDOPWR_getOverloadAutoOffStatus() function.
//
//*****************************************************************************
#define LDOPWR_AUTOOFF_ENABLED                                        OVLAOFF_H
#define LDOPWR_AUTOOFF_DISABLED                                            0x00

//*****************************************************************************
//
// The following are values that can be passed to the mask parameter for
// functions: LDOPWR_enableInterrupt(), LDOPWR_disableInterrupt(),
// LDOPWR_getInterruptStatus(), and LDOPWR_clearInterruptStatus() as well as
// returned by the LDOPWR_getInterruptStatus() function.
//
//*****************************************************************************
#define LDOPWR_LDOI_VOLTAGE_GOING_OFF_INTERRUPT                      LDOOFFIE_H
#define LDOPWR_LDOI_VOLTAGE_COMING_ON_INTERRUPT                       LDOONIE_H
#define LDOPWR_LDO_OVERLOAD_INDICATION_INTERRUPT                     LDOOVLIE_H

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
extern void LDOPWR_unLockConfiguration(uint32_t baseAddress);

extern void LDOPWR_lockConfiguration(uint32_t baseAddress);

extern void LDOPWR_enablePort_U_inputs(uint32_t baseAddress);

extern void LDOPWR_disablePort_U_inputs(uint32_t baseAddress);

extern void LDOPWR_enablePort_U_outputs(uint32_t baseAddress);

extern void LDOPWR_disablePort_U_outputs(uint32_t baseAddress);

extern uint8_t LDOPWR_getPort_U1_inputData(uint32_t baseAddress);

extern uint8_t LDOPWR_getPort_U0_inputData(uint32_t baseAddress);

extern uint8_t LDOPWR_getPort_U1_outputData(uint32_t baseAddress);

extern uint8_t LDOPWR_getPort_U0_outputData(uint32_t baseAddress);

extern void LDOPWR_setPort_U1_outputData(uint32_t baseAddress,
                                         uint8_t value);

extern void LDOPWR_setPort_U0_outputData(uint32_t baseAddress,
                                         uint8_t value);

extern void LDOPWR_togglePort_U1_outputData(uint32_t baseAddress);

extern void LDOPWR_togglePort_U0_outputData(uint32_t baseAddress);

extern void LDOPWR_enableInterrupt(uint32_t baseAddress,
                                   uint16_t mask);

extern void LDOPWR_disableInterrupt(uint32_t baseAddress,
                                    uint16_t mask);

extern void LDOPWR_enable(uint32_t baseAddress);

extern void LDOPWR_disable(uint32_t baseAddress);

extern uint8_t LDOPWR_getInterruptStatus(uint32_t baseAddress,
                                         uint16_t mask);

extern void LDOPWR_clearInterruptStatus(uint32_t baseAddress,
                                        uint16_t mask);

extern uint8_t LDOPWR_isLDOInputValid(uint32_t baseAddress);

extern void LDOPWR_enableOverloadAutoOff(uint32_t baseAddress);

extern void LDOPWR_disableOverloadAutoOff(uint32_t baseAddress);

extern uint8_t LDOPWR_getOverloadAutoOffStatus(uint32_t baseAddress);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif
#endif // __MSP430WARE_LDOPWR_H__
//Released_Version_4_10_02
