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
// timer_d.h - Driver for the TIMER_D Module.
//
//*****************************************************************************

#ifndef __MSP430WARE_TIMER_D_H__
#define __MSP430WARE_TIMER_D_H__

#include "inc/hw_memmap.h"

#ifdef __MSP430_HAS_TxD7__

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
// The following is a parameter used for TIMER_D_getCounterValue that
// determines the maximum difference in counts of the TDxR register for a
// majority vote.
//
//*****************************************************************************
#define TIMER_D_THRESHOLD                                                    50

//*****************************************************************************
//
// The following are values that can be passed to the clockSourceDivider
// parameter for functions: TIMER_D_configureContinuousMode(),
// TIMER_D_configureUpMode(), TIMER_D_configureUpDownMode(),
// TIMER_D_generatePWM(), TIMER_D_configureHighResGeneratorInRegulatedMode(),
// and TIMER_D_combineTDCCRToGeneratePWM().
//
//*****************************************************************************
#define TIMER_D_CLOCKSOURCE_DIVIDER_1                                      0x01
#define TIMER_D_CLOCKSOURCE_DIVIDER_2                                      0x02
#define TIMER_D_CLOCKSOURCE_DIVIDER_4                                      0x04
#define TIMER_D_CLOCKSOURCE_DIVIDER_8                                      0x08
#define TIMER_D_CLOCKSOURCE_DIVIDER_3                                      0x03
#define TIMER_D_CLOCKSOURCE_DIVIDER_5                                      0x05
#define TIMER_D_CLOCKSOURCE_DIVIDER_6                                      0x06
#define TIMER_D_CLOCKSOURCE_DIVIDER_7                                      0x07
#define TIMER_D_CLOCKSOURCE_DIVIDER_10                                     0x0A
#define TIMER_D_CLOCKSOURCE_DIVIDER_12                                     0x0C
#define TIMER_D_CLOCKSOURCE_DIVIDER_14                                     0x0E
#define TIMER_D_CLOCKSOURCE_DIVIDER_16                                     0x10
#define TIMER_D_CLOCKSOURCE_DIVIDER_20                                     0x14
#define TIMER_D_CLOCKSOURCE_DIVIDER_24                                     0x18
#define TIMER_D_CLOCKSOURCE_DIVIDER_28                                     0x1C
#define TIMER_D_CLOCKSOURCE_DIVIDER_32                                     0x20
#define TIMER_D_CLOCKSOURCE_DIVIDER_40                                     0x28
#define TIMER_D_CLOCKSOURCE_DIVIDER_48                                     0x30
#define TIMER_D_CLOCKSOURCE_DIVIDER_56                                     0x38
#define TIMER_D_CLOCKSOURCE_DIVIDER_64                                     0x40

//*****************************************************************************
//
// The following are values that can be passed to the timerMode parameter for
// functions: TIMER_D_startCounter().
//
//*****************************************************************************
#define TIMER_D_STOP_MODE                                                  MC_0
#define TIMER_D_UP_MODE                                                    MC_1
#define TIMER_D_CONTINUOUS_MODE                                            MC_2
#define TIMER_D_UPDOWN_MODE                                                MC_3

//*****************************************************************************
//
// The following are values that can be passed to the timerClear parameter for
// functions: TIMER_D_configureContinuousMode(), TIMER_D_configureUpMode(), and
// TIMER_D_configureUpDownMode().
//
//*****************************************************************************
#define TIMER_D_DO_CLEAR                                                  TDCLR
#define TIMER_D_SKIP_CLEAR                                                 0x00

//*****************************************************************************
//
// The following are values that can be passed to the clockSource parameter for
// functions: TIMER_D_configureContinuousMode(), TIMER_D_configureUpMode(),
// TIMER_D_configureUpDownMode(), TIMER_D_generatePWM(),
// TIMER_D_configureHighResGeneratorInRegulatedMode(), and
// TIMER_D_combineTDCCRToGeneratePWM().
//
//*****************************************************************************
#define TIMER_D_CLOCKSOURCE_EXTERNAL_TDCLK                        TDSSEL__TACLK
#define TIMER_D_CLOCKSOURCE_ACLK                                   TDSSEL__ACLK
#define TIMER_D_CLOCKSOURCE_SMCLK                                 TDSSEL__SMCLK
#define TIMER_D_CLOCKSOURCE_INVERTED_EXTERNAL_TDCLK               TDSSEL__INCLK

//*****************************************************************************
//
// The following are values that can be passed to the clockingMode parameter
// for functions: TIMER_D_configureContinuousMode(), TIMER_D_configureUpMode(),
// TIMER_D_configureUpDownMode(), TIMER_D_generatePWM(),
// TIMER_D_configureHighResGeneratorInRegulatedMode(), and
// TIMER_D_combineTDCCRToGeneratePWM().
//
//*****************************************************************************
#define TIMER_D_CLOCKINGMODE_EXTERNAL_CLOCK                            TDCLKM_0
#define TIMER_D_CLOCKINGMODE_HIRES_LOCAL_CLOCK                         TDCLKM_1
#define TIMER_D_CLOCKINGMODE_AUXILIARY_CLK                             TDCLKM_2

//*****************************************************************************
//
// The following are values that can be passed to the timerInterruptEnable_TDIE
// parameter for functions: TIMER_D_configureContinuousMode(),
// TIMER_D_configureUpMode(), and TIMER_D_configureUpDownMode().
//
//*****************************************************************************
#define TIMER_D_TDIE_INTERRUPT_ENABLE                                      TDIE
#define TIMER_D_TDIE_INTERRUPT_DISABLE                                     0x00

//*****************************************************************************
//
// The following are values that can be passed to the
// captureCompareInterruptEnable_CCR0_CCIE parameter for functions:
// TIMER_D_configureUpMode(), and TIMER_D_configureUpDownMode().
//
//*****************************************************************************
#define TIMER_D_CCIE_CCR0_INTERRUPT_ENABLE                                 CCIE
#define TIMER_D_CCIE_CCR0_INTERRUPT_DISABLE                                0x00

//*****************************************************************************
//
// The following are values that can be passed to the channelCaptureMode
// parameter for functions: TIMER_D_initCapture().
//
//*****************************************************************************
#define TIMER_D_SINGLE_CAPTURE_MODE                                        0x00
#define TIMER_D_DUAL_CAPTURE_MODE                                          0x01

//*****************************************************************************
//
// The following are values that can be passed to the captureInputSelect
// parameter for functions: TIMER_D_initCapture().
//
//*****************************************************************************
#define TIMER_D_CAPTURE_INPUTSELECT_CCIxA                                CCIS_0
#define TIMER_D_CAPTURE_INPUTSELECT_CCIxB                                CCIS_1
#define TIMER_D_CAPTURE_INPUTSELECT_GND                                  CCIS_2
#define TIMER_D_CAPTURE_INPUTSELECT_Vcc                                  CCIS_3

//*****************************************************************************
//
// The following are values that can be passed to the compareOutputMode
// parameter for functions: TIMER_D_initCompare(), TIMER_D_generatePWM(), and
// TIMER_D_combineTDCCRToGeneratePWM(); the captureOutputMode parameter for
// functions: TIMER_D_initCapture().
//
//*****************************************************************************
#define TIMER_D_OUTPUTMODE_OUTBITVALUE                                 OUTMOD_0
#define TIMER_D_OUTPUTMODE_SET                                         OUTMOD_1
#define TIMER_D_OUTPUTMODE_TOGGLE_RESET                                OUTMOD_2
#define TIMER_D_OUTPUTMODE_SET_RESET                                   OUTMOD_3
#define TIMER_D_OUTPUTMODE_TOGGLE                                      OUTMOD_4
#define TIMER_D_OUTPUTMODE_RESET                                       OUTMOD_5
#define TIMER_D_OUTPUTMODE_TOGGLE_SET                                  OUTMOD_6
#define TIMER_D_OUTPUTMODE_RESET_SET                                   OUTMOD_7

//*****************************************************************************
//
// The following are values that can be passed to the compareRegister parameter
// for functions: TIMER_D_generatePWM(), TIMER_D_setCompareValue(),
// TIMER_D_initCompareLatchLoadEvent(), and TIMER_D_initCompare(); the
// captureCompareRegister parameter for functions:
// TIMER_D_enableCaptureCompareInterrupt(),
// TIMER_D_disableCaptureCompareInterrupt(),
// TIMER_D_getCaptureCompareInterruptStatus(),
// TIMER_D_getSynchronizedCaptureCompareInput(),
// TIMER_D_getOutputForOutputModeOutBitValue(),
// TIMER_D_getCaptureCompareCount(), TIMER_D_getCaptureCompareLatchCount(),
// TIMER_D_getCaptureCompareInputSignal(),
// TIMER_D_setOutputForOutputModeOutBitValue(), and
// TIMER_D_clearCaptureCompareInterruptFlag(); the captureRegister parameter
// for functions: TIMER_D_initCapture().
//
//*****************************************************************************
#define TIMER_D_CAPTURECOMPARE_REGISTER_0                                  0x08
#define TIMER_D_CAPTURECOMPARE_REGISTER_1                                  0x0E
#define TIMER_D_CAPTURECOMPARE_REGISTER_2                                  0x14
#define TIMER_D_CAPTURECOMPARE_REGISTER_3                                  0x1A
#define TIMER_D_CAPTURECOMPARE_REGISTER_4                                  0x20
#define TIMER_D_CAPTURECOMPARE_REGISTER_5                                  0x28
#define TIMER_D_CAPTURECOMPARE_REGISTER_6                                  0x2E

//*****************************************************************************
//
// The following are values that can be passed to the captureMode parameter for
// functions: TIMER_D_initCapture().
//
//*****************************************************************************
#define TIMER_D_CAPTUREMODE_NO_CAPTURE                                     CM_0
#define TIMER_D_CAPTUREMODE_RISING_EDGE                                    CM_1
#define TIMER_D_CAPTUREMODE_FALLING_EDGE                                   CM_2
#define TIMER_D_CAPTUREMODE_RISING_AND_FALLING_EDGE                        CM_3

//*****************************************************************************
//
// The following are values that can be passed to the synchronizeCaptureSource
// parameter for functions: TIMER_D_initCapture().
//
//*****************************************************************************
#define TIMER_D_CAPTURE_ASYNCHRONOUS                                       0x00
#define TIMER_D_CAPTURE_SYNCHRONOUS                                         SCS

//*****************************************************************************
//
// The following are values that can be passed to the compareInterruptEnable
// parameter for functions: TIMER_D_initCompare().
//
//*****************************************************************************
#define TIMER_D_CAPTURECOMPARE_INTERRUPT_ENABLE                            CCIE
#define TIMER_D_CAPTURECOMPARE_INTERRUPT_DISABLE                           0x00

//*****************************************************************************
//
// The following are values that can be passed to the mask parameter for
// functions: TIMER_D_enableHighResInterrupt(),
// TIMER_D_disableHighResInterrupt(), TIMER_D_getHighResInterruptStatus(), and
// TIMER_D_clearHighResInterruptStatus() as well as returned by the
// TIMER_D_getHighResInterruptStatus() function.
//
//*****************************************************************************
#define TIMER_D_HIGH_RES_FREQUENCY_UNLOCK                             TDHUNLKIE
#define TIMER_D_HIGH_RES_FREQUENCY_LOCK                                 TDHLKIE
#define TIMER_D_HIGH_RES_FAIL_HIGH                                      TDHFHIE
#define TIMER_D_HIGH_RES_FAIL_LOW                                       TDHFLIE

//*****************************************************************************
//
// The following are values that can be passed to the mask parameter for
// functions: TIMER_D_getCaptureCompareInterruptStatus() as well as returned by
// the TIMER_D_getCaptureCompareInterruptStatus() function.
//
//*****************************************************************************
#define TIMER_D_CAPTURE_OVERFLOW                                            COV
#define TIMER_D_CAPTURECOMPARE_INTERRUPT_FLAG                             CCIFG

//*****************************************************************************
//
// The following are values that can be passed to the synchronized parameter
// for functions: TIMER_D_getSynchronizedCaptureCompareInput().
//
//*****************************************************************************
#define TIMER_D_READ_SYNCHRONIZED_CAPTURECOMPAREINPUT                      SCCI
#define TIMER_D_READ_CAPTURE_COMPARE_INPUT                                  CCI

//*****************************************************************************
//
// The following are values that can be passed to the outputModeOutBitValue
// parameter for functions: TIMER_D_setOutputForOutputModeOutBitValue() as well
// as returned by the TIMER_D_getOutputForOutputModeOutBitValue() function.
//
//*****************************************************************************
#define TIMER_D_OUTPUTMODE_OUTBITVALUE_HIGH                                 OUT
#define TIMER_D_OUTPUTMODE_OUTBITVALUE_LOW                                 0x00

//*****************************************************************************
//
// The following are values that can be passed to the desiredHighResFrequency
// parameter for functions:
// TIMER_D_configureHighResGeneratorInFreeRunningMode().
//
//*****************************************************************************
#define TIMER_D_HIGHRES_64MHZ                                              0x00
#define TIMER_D_HIGHRES_128MHZ                                             0x01
#define TIMER_D_HIGHRES_200MHZ                                             0x02
#define TIMER_D_HIGHRES_256MHZ                                             0x03

//*****************************************************************************
//
// The following are values that can be passed to the highResClockDivider
// parameter for functions: TIMER_D_configureHighResGeneratorInRegulatedMode().
//
//*****************************************************************************
#define TIMER_D_HIGHRES_CLK_DIVIDER_1                                   TDHD__1
#define TIMER_D_HIGHRES_CLK_DIVIDER_2                                   TDHD__2
#define TIMER_D_HIGHRES_CLK_DIVIDER_4                                   TDHD__4
#define TIMER_D_HIGHRES_CLK_DIVIDER_8                                   TDHD__8

//*****************************************************************************
//
// The following are values that can be passed to the
// highResClockMultiplyFactor parameter for functions:
// TIMER_D_configureHighResGeneratorInRegulatedMode().
//
//*****************************************************************************
#define TIMER_D_HIGHRES_CLK_MULTIPLY_FACTOR_8x                           TDHM_0
#define TIMER_D_HIGHRES_CLK_MULTIPLY_FACTOR_16x                          TDHM_1

//*****************************************************************************
//
// The following are values that can be passed to the
// combineCCRRegistersCombination parameter for functions:
// TIMER_D_combineTDCCRToGeneratePWM().
//
//*****************************************************************************
#define TIMER_D_COMBINE_CCR1_CCR2                                             2
#define TIMER_D_COMBINE_CCR3_CCR4                                             4
#define TIMER_D_COMBINE_CCR5_CCR6                                             6

//*****************************************************************************
//
// The following are values that can be passed to the groupLatch parameter for
// functions: TIMER_D_selectLatchingGroup().
//
//*****************************************************************************
#define TIMER_D_GROUP_NONE                                            TDCLGRP_0
#define TIMER_D_GROUP_CL12_CL23_CL56                                  TDCLGRP_1
#define TIMER_D_GROUP_CL123_CL456                                     TDCLGRP_2
#define TIMER_D_GROUP_ALL                                             TDCLGRP_3

//*****************************************************************************
//
// The following are values that can be passed to the counterLength parameter
// for functions: TIMER_D_selectCounterLength().
//
//*****************************************************************************
#define TIMER_D_COUNTER_16BIT                                            CNTL_0
#define TIMER_D_COUNTER_12BIT                                            CNTL_1
#define TIMER_D_COUNTER_10BIT                                            CNTL_2
#define TIMER_D_COUNTER_8BIT                                             CNTL_3

//*****************************************************************************
//
// The following are values that can be passed to the compareLatchLoadEvent
// parameter for functions: TIMER_D_initCompareLatchLoadEvent().
//
//*****************************************************************************
#define TIMER_D_LATCH_ON_WRITE_TO_TDxCCRn_COMPARE_REGISTER               CLLD_0
#define TIMER_D_LATCH_WHEN_COUNTER_COUNTS_TO_0_IN_UP_OR_CONT_MODE        CLLD_1
#define TIMER_D_LATCH_WHEN_COUNTER_COUNTS_TO_0_IN_UPDOWN_MODE            CLLD_2
#define TIMER_D_LATCH_WHEN_COUNTER_COUNTS_TO_CURRENT_COMPARE_LATCH_VALUE CLLD_3

//*****************************************************************************
//
// The following are values that can be passed to the highResCoarseClockRange
// parameter for functions: TIMER_D_selectHighResCoarseClockRange().
//
//*****************************************************************************
#define TIMER_D_HIGHRES_BELOW_15MHz                                        0x00
#define TIMER_D_HIGHRES_ABOVE_15MHz                                    TDHCLKCR

//*****************************************************************************
//
// The following are values that can be passed to the highResClockRange
// parameter for functions: TIMER_D_selectHighResClockRange().
//
//*****************************************************************************
#define TIMER_D_CLOCK_RANGE0                                             0x0000
#define TIMER_D_CLOCK_RANGE1                                             0x2000
#define TIMER_D_CLOCK_RANGE2                                             0x4000

//*****************************************************************************
//
// The following are values that can be passed toThe following are values that
// can be returned by the TIMER_D_getSynchronizedCaptureCompareInput()
// function.
//
//*****************************************************************************
#define TIMER_D_CAPTURECOMPARE_INPUT_HIGH                                  0x01
#define TIMER_D_CAPTURECOMPARE_INPUT_LOW                                   0x00

//*****************************************************************************
//
// The following are values that can be passed toThe following are values that
// can be returned by the TIMER_D_getTimerInterruptStatus() function.
//
//*****************************************************************************
#define TIMER_D_INTERRUPT_NOT_PENDING                                      0x00
#define TIMER_D_INTERRUPT_PENDING                                          0x01

//*****************************************************************************
//
// The following are values that can be passed toThe following are values that
// can be returned by the TIMER_D_getCaptureCompareInputSignal() function.
//
//*****************************************************************************
#define TIMER_D_CAPTURECOMPARE_INPUT                                        CCI

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
extern void TIMER_D_startCounter(uint32_t baseAddress,
                                 uint16_t timerMode);

extern void TIMER_D_configureContinuousMode(uint32_t baseAddress,
                                            uint16_t clockSource,
                                            uint16_t clockSourceDivider,
                                            uint16_t clockingMode,
                                            uint16_t timerInterruptEnable_TDIE,
                                            uint16_t timerClear);

extern void TIMER_D_configureUpMode(uint32_t baseAddress,
                                    uint16_t clockSource,
                                    uint16_t clockSourceDivider,
                                    uint16_t clockingMode,
                                    uint16_t timerPeriod,
                                    uint16_t timerInterruptEnable_TDIE,
                                    uint16_t captureCompareInterruptEnable_CCR0_CCIE,
                                    uint16_t timerClear);

extern void TIMER_D_configureUpDownMode(uint32_t baseAddress,
                                        uint16_t clockSource,
                                        uint16_t clockSourceDivider,
                                        uint16_t clockingMode,
                                        uint16_t timerPeriod,
                                        uint16_t timerInterruptEnable_TDIE,
                                        uint16_t captureCompareInterruptEnable_CCR0_CCIE,
                                        uint16_t timerClear);

extern void TIMER_D_initCapture(uint32_t baseAddress,
                                uint16_t captureRegister,
                                uint16_t captureMode,
                                uint16_t captureInputSelect,
                                uint16_t synchronizeCaptureSource,
                                uint16_t captureInterruptEnable,
                                uint16_t captureOutputMode,
                                uint8_t channelCaptureMode);

extern void TIMER_D_initCompare(uint32_t baseAddress,
                                uint16_t compareRegister,
                                uint16_t compareInterruptEnable,
                                uint16_t compareOutputMode,
                                uint16_t compareValue);

extern void TIMER_D_enableTimerInterrupt(uint32_t baseAddress);

extern void TIMER_D_enableHighResInterrupt(uint32_t baseAddress,
                                           uint16_t mask);

extern void TIMER_D_disableTimerInterrupt(uint32_t baseAddress);

extern void TIMER_D_disableHighResInterrupt(uint32_t baseAddress,
                                            uint16_t mask);

extern uint32_t TIMER_D_getTimerInterruptStatus(uint32_t baseAddress);

extern void TIMER_D_enableCaptureCompareInterrupt(uint32_t baseAddress,
                                                  uint16_t captureCompareRegister);

extern void TIMER_D_disableCaptureCompareInterrupt(uint32_t baseAddress,
                                                   uint16_t captureCompareRegister);

extern uint32_t TIMER_D_getCaptureCompareInterruptStatus(uint32_t baseAddress,
                                                         uint16_t captureCompareRegister,
                                                         uint16_t mask);

extern uint16_t TIMER_D_getHighResInterruptStatus(uint32_t baseAddress,
                                                  uint16_t mask);

extern void TIMER_D_clear(uint32_t baseAddress);

extern void TIMER_D_clearHighResInterruptStatus(uint32_t baseAddress,
                                                uint16_t mask);

extern uint8_t TIMER_D_getSynchronizedCaptureCompareInput(uint32_t baseAddress,
                                                          uint16_t captureCompareRegister,
                                                          uint16_t synchronized);

extern uint8_t TIMER_D_getOutputForOutputModeOutBitValue(uint32_t baseAddress,
                                                         uint16_t captureCompareRegister);

extern uint16_t TIMER_D_getCaptureCompareCount(uint32_t baseAddress,
                                               uint16_t captureCompareRegister);

extern uint16_t TIMER_D_getCaptureCompareLatchCount(uint32_t baseAddress,
                                                    uint16_t captureCompareRegister);

extern uint8_t TIMER_D_getCaptureCompareInputSignal(uint32_t baseAddress,
                                                    uint16_t captureCompareRegister);

extern void TIMER_D_setOutputForOutputModeOutBitValue(uint32_t baseAddress,
                                                      uint16_t captureCompareRegister,
                                                      uint8_t outputModeOutBitValue);

extern void TIMER_D_generatePWM(uint32_t baseAddress,
                                uint16_t clockSource,
                                uint16_t clockSourceDivider,
                                uint16_t clockingMode,
                                uint16_t timerPeriod,
                                uint16_t compareRegister,
                                uint16_t compareOutputMode,
                                uint16_t dutyCycle);

extern void TIMER_D_stop(uint32_t baseAddress);

extern void TIMER_D_setCompareValue(uint32_t baseAddress,
                                    uint16_t compareRegister,
                                    uint16_t compareValue);

extern void TIMER_D_clearTimerInterruptFlag(uint32_t baseAddress);

extern void TIMER_D_clearCaptureCompareInterruptFlag(uint32_t baseAddress,
                                                     uint16_t captureCompareRegister);

extern uint8_t TIMER_D_configureHighResGeneratorInFreeRunningMode(uint32_t baseAddress,
                                                                  uint8_t desiredHighResFrequency);

extern void TIMER_D_configureHighResGeneratorInRegulatedMode(uint32_t baseAddress,
                                                             uint16_t clockSource,
                                                             uint16_t clockSourceDivider,
                                                             uint16_t clockingMode,
                                                             uint8_t highResClockMultiplyFactor,
                                                             uint8_t highResClockDivider);

extern void TIMER_D_combineTDCCRToGeneratePWM(uint32_t baseAddress,
                                              uint16_t clockSource,
                                              uint16_t clockSourceDivider,
                                              uint16_t clockingMode,
                                              uint16_t timerPeriod,
                                              uint16_t combineCCRRegistersCombination,
                                              uint16_t compareOutputMode,
                                              uint16_t dutyCycle1,
                                              uint16_t dutyCycle2);

extern void TIMER_D_selectLatchingGroup(uint16_t baseAddress,
                                        uint16_t groupLatch);

extern void TIMER_D_selectCounterLength(uint16_t baseAddress,
                                        uint16_t counterLength);

extern void TIMER_D_initCompareLatchLoadEvent(uint16_t baseAddress,
                                              uint16_t compareRegister,
                                              uint16_t compareLatchLoadEvent);

extern void TIMER_D_disableHighResFastWakeup(uint32_t baseAddress);

extern void TIMER_D_enableHighResFastWakeup(uint32_t baseAddress);

extern void TIMER_D_disableHighResClockEnhancedAccuracy(uint32_t baseAddress);

extern void TIMER_D_enableHighResClockEnhancedAccuracy(uint32_t baseAddress);

extern void TIMER_D_DisableHighResGeneratorForceON(uint32_t baseAddress);

extern void TIMER_D_EnableHighResGeneratorForceON(uint32_t baseAddress);

extern void TIMER_D_selectHighResCoarseClockRange(uint32_t baseAddress,
                                                  uint16_t highResCoarseClockRange);

extern void TIMER_D_selectHighResClockRange(uint32_t baseAddress,
                                            uint16_t highResClockRange);

extern uint16_t TIMER_D_getCounterValue(uint32_t baseAddress);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif
#endif // __MSP430WARE_TIMER_D_H__
//Released_Version_4_10_02
