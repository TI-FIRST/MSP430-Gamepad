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
// pmm.h - Driver for the PMM Module.
//
//*****************************************************************************

#ifndef __MSP430WARE_PMM_H__
#define __MSP430WARE_PMM_H__

#include "inc/hw_memmap.h"

#ifdef __MSP430_HAS_PMM__

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
// The following are values that can be passed to the level parameter for
// functions: PMM_setVCoreUp(), PMM_setVCoreDown(), and PMM_setVCore().
//
//*****************************************************************************
#define PMM_CORE_LEVEL_0                                             PMMCOREV_0
#define PMM_CORE_LEVEL_1                                             PMMCOREV_1
#define PMM_CORE_LEVEL_2                                             PMMCOREV_2
#define PMM_CORE_LEVEL_3                                             PMMCOREV_3

//*****************************************************************************
//
// The following are values that can be passed to the mask parameter for
// functions: PMM_getInterruptStatus() as well as returned by the
// PMM_getInterruptStatus() function.
//
//*****************************************************************************
#define PMM_SVSMLDLYIFG                                             SVSMLDLYIFG
#define PMM_SVMLIFG                                                     SVMLIFG
#define PMM_SVMLVLRIFG                                               SVMLVLRIFG
#define PMM_SVSMHDLYIFG                                             SVSMHDLYIFG
#define PMM_SVMHIFG                                                     SVMHIFG
#define PMM_SVMHVLRIFG                                               SVMHVLRIFG
#define PMM_PMMBORIFG                                                 PMMBORIFG
#define PMM_PMMRSTIFG                                                 PMMRSTIFG
#define PMM_PMMPORIFG                                                 PMMPORIFG
#define PMM_SVSHIFG                                                     SVSHIFG
#define PMM_SVSLIFG                                                     SVSLIFG
#define PMM_PMMLPM5IFG                                               PMMLPM5IFG

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
extern void PMM_enableSvsL(uint32_t baseAddress);

extern void PMM_disableSvsL(uint32_t baseAddress);

extern void PMM_enableSvmL(uint32_t baseAddress);

extern void PMM_disableSvmL(uint32_t baseAddress);

extern void PMM_enableSvsH(uint32_t baseAddress);

extern void PMM_disableSvsH(uint32_t baseAddress);

extern void PMM_enableSvmH(uint32_t baseAddress);

extern void PMM_disableSvmH(uint32_t baseAddress);

extern void PMM_enableSvsLSvmL(uint32_t baseAddress);

extern void PMM_disableSvsLSvmL(uint32_t baseAddress);

extern void PMM_enableSvsHSvmH(uint32_t baseAddress);

extern void PMM_disableSvsHSvmH(uint32_t baseAddress);

extern void PMM_enableSvsLReset(uint32_t baseAddress);

extern void PMM_disableSvsLReset(uint32_t baseAddress);

extern void PMM_enableSvmLInterrupt(uint32_t baseAddress);

extern void PMM_disableSvmLInterrupt(uint32_t baseAddress);

extern void PMM_enableSvsHReset(uint32_t baseAddress);

extern void PMM_disableSvsHReset(uint32_t baseAddress);

extern void PMM_enableSvmHInterrupt(uint32_t baseAddress);

extern void PMM_disableSvmHInterrupt(uint32_t baseAddress);

extern void PMM_clearPMMIFGS(uint32_t baseAddress);

extern void PMM_SvsLEnabledInLPMFastWake(uint32_t baseAddress);

extern void PMM_SvsLEnabledInLPMSlowWake(uint32_t baseAddress);

extern void PMM_SvsLDisabledInLPMFastWake(uint32_t baseAddress);

extern void PMM_SvsLDisabledInLPMSlowWake(uint32_t baseAddress);

extern void PMM_SvsHEnabledInLPMNormPerf(uint32_t baseAddress);

extern void PMM_SvsHEnabledInLPMFullPerf(uint32_t baseAddress);

extern void PMM_SvsHDisabledInLPMNormPerf(uint32_t baseAddress);

extern void PMM_SvsHDisabledInLPMFullPerf(uint32_t baseAddress);

extern void PMM_SvsLOptimizedInLPMFastWake(uint32_t baseAddress);

extern void PMM_SvsHOptimizedInLPMFullPerf(uint32_t baseAddress);

extern uint16_t PMM_setVCoreUp(uint32_t baseAddress,
                               uint8_t level);

extern uint16_t PMM_setVCoreDown(uint32_t baseAddress,
                                 uint8_t level);

extern bool PMM_setVCore(uint32_t baseAddress,
                         uint8_t level);

extern uint16_t PMM_getInterruptStatus(uint32_t baseAddress,
                                       uint16_t mask);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif
#endif // __MSP430WARE_PMM_H__
//Released_Version_4_10_02
