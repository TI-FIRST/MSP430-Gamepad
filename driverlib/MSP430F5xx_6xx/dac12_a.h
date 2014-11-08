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
// dac12_a.h - Driver for the DAC12_A Module.
//
//*****************************************************************************

#ifndef __MSP430WARE_DAC12_A_H__
#define __MSP430WARE_DAC12_A_H__

#include "inc/hw_memmap.h"

#ifdef __MSP430_HAS_DAC12_2__

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
// The following are values that can be passed to the positiveReferenceVoltage
// parameter for functions: DAC12_A_init().
//
//*****************************************************************************
#define DAC12_A_VREF_AVCC                                         (DAC12SREF_1)
#define DAC12_A_VREF_INT                                          (DAC12SREF_0)
#define DAC12_A_VREF_EXT                                          (DAC12SREF_2)

//*****************************************************************************
//
// The following are values that can be passed to the amplifierSetting
// parameter for functions: DAC12_A_init(), and DAC12_A_setAmplifierSetting().
//
//*****************************************************************************
#define DAC12_A_AMP_OFF_PINOUTHIGHZ                                (DAC12AMP_0)
#define DAC12_A_AMP_OFF_PINOUTLOW                                  (DAC12AMP_1)
#define DAC12_A_AMP_LOWIN_LOWOUT                                   (DAC12AMP_2)
#define DAC12_A_AMP_LOWIN_MEDOUT                                   (DAC12AMP_3)
#define DAC12_A_AMP_LOWIN_HIGHOUT                                  (DAC12AMP_4)
#define DAC12_A_AMP_MEDIN_MEDOUT                                   (DAC12AMP_5)
#define DAC12_A_AMP_MEDIN_HIGHOUT                                  (DAC12AMP_6)
#define DAC12_A_AMP_HIGHIN_HIGHOUT                                 (DAC12AMP_7)

//*****************************************************************************
//
// The following are values that can be passed to the outputSelect parameter
// for functions: DAC12_A_init().
//
//*****************************************************************************
#define DAC12_A_OUTPUT_1                                          (!(DAC12OPS))
#define DAC12_A_OUTPUT_2                                             (DAC12OPS)

//*****************************************************************************
//
// The following are values that can be passed to the submoduleSelect parameter
// for functions: DAC12_A_init(), DAC12_A_setAmplifierSetting(),
// DAC12_A_disable(), DAC12_A_enableInterrupt(), DAC12_A_disableInterrupt(),
// DAC12_A_getInterruptStatus(), DAC12_A_clearInterrupt(),
// DAC12_A_calibrateOutput(), DAC12_A_getCalibrationData(),
// DAC12_A_setCalibrationOffset(), DAC12_A_enableConversions(),
// DAC12_A_setData(), DAC12_A_disableConversions(), DAC12_A_setResolution(),
// DAC12_A_setInputDataFormat(), and
// DAC12_A_getDataBufferMemoryAddressForDMA().
//
//*****************************************************************************
#define DAC12_A_SUBMODULE_0                                              (0x00)
#define DAC12_A_SUBMODULE_1                                              (0x10)

//*****************************************************************************
//
// The following are values that can be passed to the outputVoltageMultiplier
// parameter for functions: DAC12_A_init().
//
//*****************************************************************************
#define DAC12_A_VREFx1                                                (DAC12IR)
#define DAC12_A_VREFx2                                                (DAC12OG)
#define DAC12_A_VREFx3                                                    (0x0)

//*****************************************************************************
//
// The following are values that can be passed to the conversionTriggerSelect
// parameter for functions: DAC12_A_init().
//
//*****************************************************************************
#define DAC12_A_TRIGGER_ENCBYPASS                                 (DAC12LSEL_0)
#define DAC12_A_TRIGGER_ENC                                       (DAC12LSEL_1)
#define DAC12_A_TRIGGER_TA                                        (DAC12LSEL_2)
#define DAC12_A_TRIGGER_TB                                        (DAC12LSEL_3)

//*****************************************************************************
//
// The following are values that can be passed to the resolutionSelect
// parameter for functions: DAC12_A_setResolution().
//
//*****************************************************************************
#define DAC12_A_RESOLUTION_8BIT                                      (DAC12RES)
#define DAC12_A_RESOLUTION_12BIT                                  (!(DAC12RES))

//*****************************************************************************
//
// The following are values that can be passed to the inputJustification
// parameter for functions: DAC12_A_setInputDataFormat().
//
//*****************************************************************************
#define DAC12_A_JUSTIFICATION_RIGHT                               (!(DAC12DFJ))
#define DAC12_A_JUSTIFICATION_LEFT                                   (DAC12DFJ)

//*****************************************************************************
//
// The following are values that can be passed to the inputSign parameter for
// functions: DAC12_A_setInputDataFormat().
//
//*****************************************************************************
#define DAC12_A_UNSIGNED_BINARY                                    (!(DAC12DF))
#define DAC12_A_SIGNED_2SCOMPLEMENT                                   (DAC12DF)

//*****************************************************************************
//
// The following are values that can be passed toThe following are values that
// can be returned by the DAC12_A_getInterruptStatus() function.
//
//*****************************************************************************
#define DAC12_A_INT_ACTIVE                                           (DAC12IFG)
#define DAC12_A_INT_INACTIVE                                             (0x00)

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
extern bool DAC12_A_init(uint32_t baseAddress,
                         uint8_t submoduleSelect,
                         uint16_t outputSelect,
                         uint16_t positiveReferenceVoltage,
                         uint16_t outputVoltageMultiplier,
                         uint8_t amplifierSetting,
                         uint16_t conversionTriggerSelect);

extern void DAC12_A_setAmplifierSetting(uint32_t baseAddress,
                                        uint8_t submoduleSelect,
                                        uint8_t amplifierSetting);

extern void DAC12_A_disable(uint32_t baseAddress,
                            uint8_t submoduleSelect);

extern void DAC12_A_enableGrouping(uint32_t baseAddress);

extern void DAC12_A_disableGrouping(uint32_t baseAddress);

extern void DAC12_A_enableInterrupt(uint32_t baseAddress,
                                    uint8_t submoduleSelect);

extern void DAC12_A_disableInterrupt(uint32_t baseAddress,
                                     uint8_t submoduleSelect);

extern uint16_t DAC12_A_getInterruptStatus(uint32_t baseAddress,
                                           uint8_t submoduleSelect);

extern void DAC12_A_clearInterrupt(uint32_t baseAddress,
                                   uint8_t submoduleSelect);

extern void DAC12_A_calibrateOutput(uint32_t baseAddress,
                                    uint8_t submoduleSelect);

extern uint16_t DAC12_A_getCalibrationData(uint32_t baseAddress,
                                           uint8_t submoduleSelect);

extern void DAC12_A_setCalibrationOffset(uint32_t baseAddress,
                                         uint8_t submoduleSelect,
                                         uint16_t calibrationOffsetValue);

extern void DAC12_A_enableConversions(uint32_t baseAddress,
                                      uint8_t submoduleSelect);

extern void DAC12_A_setData(uint32_t baseAddress,
                            uint8_t submoduleSelect,
                            uint16_t data);

extern void DAC12_A_disableConversions(uint32_t baseAddress,
                                       uint8_t submoduleSelect);

extern void DAC12_A_setResolution(uint32_t baseAddress,
                                  uint8_t submoduleSelect,
                                  uint16_t resolutionSelect);

extern void DAC12_A_setInputDataFormat(uint32_t baseAddress,
                                       uint8_t submoduleSelect,
                                       uint8_t inputJustification,
                                       uint8_t inputSign);

extern uint32_t DAC12_A_getDataBufferMemoryAddressForDMA(uint32_t baseAddress,
                                                         uint8_t submoduleSelect);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif
#endif // __MSP430WARE_DAC12_A_H__
//Released_Version_4_10_02
