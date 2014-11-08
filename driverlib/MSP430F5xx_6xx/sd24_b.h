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
// sd24_b.h - Driver for the SD24_B Module.
//
//*****************************************************************************

#ifndef __MSP430WARE_SD24_B_H__
#define __MSP430WARE_SD24_B_H__

#include "inc/hw_memmap.h"

#ifdef __MSP430_HAS_SD24_B__

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
// The following are values that can be passed to the clockSourceSelect
// parameter for functions: SD24_B_init().
//
//*****************************************************************************
#define SD24_B_CLOCKSOURCE_MCLK                                (SD24SSEL__MCLK)
#define SD24_B_CLOCKSOURCE_SMCLK                              (SD24SSEL__SMCLK)
#define SD24_B_CLOCKSOURCE_ACLK                                (SD24SSEL__ACLK)
#define SD24_B_CLOCKSOURCE_SD24CLK                          (SD24SSEL__SD24CLK)

//*****************************************************************************
//
// The following are values that can be passed to the referenceSelect parameter
// for functions: SD24_B_init().
//
//*****************************************************************************
#define SD24_B_REF_EXTERNAL                                              (0x00)
#define SD24_B_REF_INTERNAL                                          (SD24REFS)

//*****************************************************************************
//
// The following are values that can be passed to the clockPreDivider parameter
// for functions: SD24_B_init().
//
//*****************************************************************************
#define SD24_B_PRECLOCKDIVIDER_1                                   (SD24PDIV_0)
#define SD24_B_PRECLOCKDIVIDER_2                                   (SD24PDIV_1)
#define SD24_B_PRECLOCKDIVIDER_4                                   (SD24PDIV_2)
#define SD24_B_PRECLOCKDIVIDER_8                                   (SD24PDIV_3)
#define SD24_B_PRECLOCKDIVIDER_16                                  (SD24PDIV_4)
#define SD24_B_PRECLOCKDIVIDER_32                                  (SD24PDIV_5)
#define SD24_B_PRECLOCKDIVIDER_64                                  (SD24PDIV_6)
#define SD24_B_PRECLOCKDIVIDER_128                                 (SD24PDIV_7)

//*****************************************************************************
//
// The following are values that can be passed to the clockDivider parameter
// for functions: SD24_B_init().
//
//*****************************************************************************
#define SD24_B_CLOCKDIVIDER_1                                            (0x00)
#define SD24_B_CLOCKDIVIDER_2                                        (SD24DIV0)
#define SD24_B_CLOCKDIVIDER_3                                        (SD24DIV1)
#define SD24_B_CLOCKDIVIDER_4                             (SD24DIV1 | SD24DIV0)
#define SD24_B_CLOCKDIVIDER_5                                        (SD24DIV2)
#define SD24_B_CLOCKDIVIDER_6                             (SD24DIV2 | SD24DIV0)
#define SD24_B_CLOCKDIVIDER_7                             (SD24DIV2 | SD24DIV1)
#define SD24_B_CLOCKDIVIDER_8                  (SD24DIV2 | SD24DIV1 | SD24DIV0)
#define SD24_B_CLOCKDIVIDER_9                                        (SD24DIV3)
#define SD24_B_CLOCKDIVIDER_10                            (SD24DIV3 | SD24DIV0)
#define SD24_B_CLOCKDIVIDER_11                            (SD24DIV3 | SD24DIV1)
#define SD24_B_CLOCKDIVIDER_12                 (SD24DIV3 | SD24DIV1 | SD24DIV0)
#define SD24_B_CLOCKDIVIDER_13                            (SD24DIV3 | SD24DIV2)
#define SD24_B_CLOCKDIVIDER_14                 (SD24DIV3 | SD24DIV2 | SD24DIV0)
#define SD24_B_CLOCKDIVIDER_15                 (SD24DIV3 | SD24DIV2 | SD24DIV1)
#define SD24_B_CLOCKDIVIDER_16      (SD24DIV3 | SD24DIV2 | SD24DIV1 | SD24DIV0)
#define SD24_B_CLOCKDIVIDER_17                                       (SD24DIV4)
#define SD24_B_CLOCKDIVIDER_18                            (SD24DIV4 | SD24DIV0)
#define SD24_B_CLOCKDIVIDER_19                            (SD24DIV4 | SD24DIV1)
#define SD24_B_CLOCKDIVIDER_20                 (SD24DIV4 | SD24DIV1 | SD24DIV0)
#define SD24_B_CLOCKDIVIDER_21                            (SD24DIV4 | SD24DIV2)
#define SD24_B_CLOCKDIVIDER_22                 (SD24DIV4 | SD24DIV2 | SD24DIV0)
#define SD24_B_CLOCKDIVIDER_23                 (SD24DIV4 | SD24DIV2 | SD24DIV1)
#define SD24_B_CLOCKDIVIDER_24      (SD24DIV4 | SD24DIV2 | SD24DIV1 | SD24DIV0)
#define SD24_B_CLOCKDIVIDER_25                            (SD24DIV4 | SD24DIV3)
#define SD24_B_CLOCKDIVIDER_26                 (SD24DIV4 | SD24DIV3 | SD24DIV0)
#define SD24_B_CLOCKDIVIDER_27                 (SD24DIV4 | SD24DIV3 | SD24DIV1)
#define SD24_B_CLOCKDIVIDER_28      (SD24DIV4 | SD24DIV3 | SD24DIV1 | SD24DIV0)
#define SD24_B_CLOCKDIVIDER_29                 (SD24DIV4 | SD24DIV3 | SD24DIV2)
#define SD24_B_CLOCKDIVIDER_30      (SD24DIV4 | SD24DIV3 | SD24DIV2 | SD24DIV0)
#define SD24_B_CLOCKDIVIDER_31      (SD24DIV4 | SD24DIV3 | SD24DIV2 | SD24DIV1)
#define SD24_B_CLOCKDIVIDER_32                                                \
        (SD24DIV4 | SD24DIV3 | SD24DIV2 | SD24DIV1 | SD24DIV0)

//*****************************************************************************
//
// The following are values that can be passed to the conversionMode parameter
// for functions: SD24_B_configureConverter(), and
// SD24_B_configureConverterAdvanced().
//
//*****************************************************************************
#define SD24_B_CONTINUOUS_MODE                                           (0x00)
#define SD24_B_SINGLE_MODE                                         (SD24SNGL_H)

//*****************************************************************************
//
// The following are values that can be passed to the converter parameter for
// functions: SD24_B_configureConverter(), SD24_B_configureConverterAdvanced(),
// SD24_B_setConverterDataFormat(), SD24_B_startConverterConversion(),
// SD24_B_stopConverterConversion(), SD24_B_setInterruptDelay(),
// SD24_B_setOversampling(), SD24_B_setGain(), SD24_B_getResults(),
// SD24_B_getHighWordResults(), SD24_B_enableInterrupt(),
// SD24_B_disableInterrupt(), SD24_B_clearInterrupt(), and
// SD24_B_getInterruptStatus().
//
//*****************************************************************************
#define SD24_B_CONVERTER_0                                                    0
#define SD24_B_CONVERTER_1                                                    1
#define SD24_B_CONVERTER_2                                                    2
#define SD24_B_CONVERTER_3                                                    3
#define SD24_B_CONVERTER_4                                                    4
#define SD24_B_CONVERTER_5                                                    5
#define SD24_B_CONVERTER_6                                                    6
#define SD24_B_CONVERTER_7                                                    7

//*****************************************************************************
//
// The following are values that can be passed to the alignment parameter for
// functions: SD24_B_configureConverter(), and
// SD24_B_configureConverterAdvanced().
//
//*****************************************************************************
#define SD24_B_ALIGN_RIGHT                                               (0x00)
#define SD24_B_ALIGN_LEFT                                            (SD24ALGN)

//*****************************************************************************
//
// The following are values that can be passed to the startSelect parameter for
// functions: SD24_B_configureConverter(), and
// SD24_B_configureConverterAdvanced().
//
//*****************************************************************************
#define SD24_B_CONVERSION_SELECT_SD24SC                       (SD24SCS__SD24SC)
#define SD24_B_CONVERSION_SELECT_EXT1                           (SD24SCS__EXT1)
#define SD24_B_CONVERSION_SELECT_EXT2                           (SD24SCS__EXT2)
#define SD24_B_CONVERSION_SELECT_EXT3                           (SD24SCS__EXT3)
#define SD24_B_CONVERSION_SELECT_GROUP0                       (SD24SCS__GROUP0)
#define SD24_B_CONVERSION_SELECT_GROUP1                       (SD24SCS__GROUP1)
#define SD24_B_CONVERSION_SELECT_GROUP2                       (SD24SCS__GROUP2)
#define SD24_B_CONVERSION_SELECT_GROUP3                       (SD24SCS__GROUP3)

//*****************************************************************************
//
// The following are values that can be passed to the oversampleRatio parameter
// for functions: SD24_B_configureConverterAdvanced(), and
// SD24_B_setOversampling().
//
//*****************************************************************************
#define SD24_B_OVERSAMPLE_32                                          (OSR__32)
#define SD24_B_OVERSAMPLE_64                                          (OSR__64)
#define SD24_B_OVERSAMPLE_128                                        (OSR__128)
#define SD24_B_OVERSAMPLE_256                                        (OSR__256)
#define SD24_B_OVERSAMPLE_512                                        (OSR__512)
#define SD24_B_OVERSAMPLE_1024                                      (OSR__1024)

//*****************************************************************************
//
// The following are values that can be passed to the dataFormat parameter for
// functions: SD24_B_configureConverterAdvanced(), and
// SD24_B_setConverterDataFormat().
//
//*****************************************************************************
#define SD24_B_DATA_FORMAT_BINARY                                    (SD24DF_0)
#define SD24_B_DATA_FORMAT_2COMPLEMENT                               (SD24DF_1)

//*****************************************************************************
//
// The following are values that can be passed to the gain parameter for
// functions: SD24_B_configureConverterAdvanced(), and SD24_B_setGain().
//
//*****************************************************************************
#define SD24_B_GAIN_1                                              (SD24GAIN_1)
#define SD24_B_GAIN_2                                              (SD24GAIN_2)
#define SD24_B_GAIN_4                                              (SD24GAIN_4)
#define SD24_B_GAIN_8                                              (SD24GAIN_8)
#define SD24_B_GAIN_16                                            (SD24GAIN_16)
#define SD24_B_GAIN_32                                            (SD24GAIN_32)
#define SD24_B_GAIN_64                                            (SD24GAIN_64)
#define SD24_B_GAIN_128                                          (SD24GAIN_128)

//*****************************************************************************
//
// The following are values that can be passed to the sampleDelay parameter for
// functions: SD24_B_configureConverterAdvanced(), and
// SD24_B_setInterruptDelay().
//
//*****************************************************************************
#define SD24_B_FOURTH_SAMPLE_INTERRUPT                           (SD24INTDLY_0)
#define SD24_B_THIRD_SAMPLE_INTERRUPT                            (SD24INTDLY_1)
#define SD24_B_SECOND_SAMPLE_INTERRUPT                           (SD24INTDLY_2)
#define SD24_B_FIRST_SAMPLE_INTERRUPT                            (SD24INTDLY_3)

//*****************************************************************************
//
// The following are values that can be passed to the group parameter for
// functions: SD24_B_startGroupConversion(), and SD24_B_stopGroupConversion().
//
//*****************************************************************************
#define SD24_B_GROUP0                                                         0
#define SD24_B_GROUP1                                                         1
#define SD24_B_GROUP2                                                         2
#define SD24_B_GROUP3                                                         3

//*****************************************************************************
//
// The following are values that can be passed to the interruptFlag parameter
// for functions: SD24_B_configureDMATrigger().
//
//*****************************************************************************
#define SD24_B_DMA_TRIGGER_IFG0                                     (SD24DMA_0)
#define SD24_B_DMA_TRIGGER_IFG1                                     (SD24DMA_1)
#define SD24_B_DMA_TRIGGER_IFG2                                     (SD24DMA_2)
#define SD24_B_DMA_TRIGGER_IFG3                                     (SD24DMA_3)
#define SD24_B_DMA_TRIGGER_IFG4                                     (SD24DMA_4)
#define SD24_B_DMA_TRIGGER_IFG5                                     (SD24DMA_5)
#define SD24_B_DMA_TRIGGER_IFG6                                     (SD24DMA_6)
#define SD24_B_DMA_TRIGGER_IFG7                                     (SD24DMA_7)
#define SD24_B_DMA_TRIGGER_TRGIFG                                   (SD24DMA_8)

//*****************************************************************************
//
// The following are values that can be passed to the mask parameter for
// functions: SD24_B_enableInterrupt(), SD24_B_disableInterrupt(),
// SD24_B_clearInterrupt(), and SD24_B_getInterruptStatus() as well as returned
// by the SD24_B_getInterruptStatus() function.
//
//*****************************************************************************
#define SD24_B_CONVERTER_INTERRUPT                                      SD24IE0
#define SD24_B_CONVERTER_OVERFLOW_INTERRUPT                           SD24OVIE0

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
extern void SD24_B_init(uint32_t baseAddress,
                        uint16_t clockSourceSelect,
                        uint16_t clockPreDivider,
                        uint16_t clockDivider,
                        uint16_t referenceSelect);

extern void SD24_B_configureConverter(uint32_t baseAddress,
                                      uint8_t converter,
                                      uint8_t alignment,
                                      uint8_t startSelect,
                                      uint8_t conversionMode);

extern void SD24_B_configureConverterAdvanced(uint32_t baseAddress,
                                              uint8_t converter,
                                              uint8_t alignment,
                                              uint8_t startSelect,
                                              uint8_t conversionMode,
                                              uint8_t dataFormat,
                                              uint8_t sampleDelay,
                                              uint16_t oversampleRatio,
                                              uint8_t gain);

extern void SD24_B_setConverterDataFormat(uint32_t baseAddress,
                                          uint8_t converter,
                                          uint8_t dataFormat);

extern void SD24_B_startGroupConversion(uint32_t baseAddress,
                                        uint8_t group);

extern void SD24_B_stopGroupConversion(uint32_t baseAddress,
                                       uint8_t group);

extern void SD24_B_startConverterConversion(uint32_t baseAddress,
                                            uint8_t converter);

extern void SD24_B_stopConverterConversion(uint32_t baseAddress,
                                           uint8_t converter);

extern void SD24_B_configureDMATrigger(uint32_t baseAddress,
                                       uint16_t interruptFlag);

extern void SD24_B_setInterruptDelay(uint32_t baseAddress,
                                     uint8_t converter,
                                     uint8_t sampleDelay);

extern void SD24_B_setOversampling(uint32_t baseAddress,
                                   uint8_t converter,
                                   uint16_t oversampleRatio);

extern void SD24_B_setGain(uint32_t baseAddress,
                           uint8_t converter,
                           uint8_t gain);

extern uint32_t SD24_B_getResults(uint32_t baseAddress,
                                  uint8_t converter);

extern uint16_t SD24_B_getHighWordResults(uint32_t baseAddress,
                                          uint8_t converter);

extern void SD24_B_enableInterrupt(uint32_t baseAddress,
                                   uint8_t converter,
                                   uint16_t mask);

extern void SD24_B_disableInterrupt(uint32_t baseAddress,
                                    uint8_t converter,
                                    uint16_t mask);

extern void SD24_B_clearInterrupt(uint32_t baseAddress,
                                  uint8_t converter,
                                  uint16_t mask);

extern uint16_t SD24_B_getInterruptStatus(uint32_t baseAddress,
                                          uint8_t converter,
                                          uint16_t mask);

//*****************************************************************************
//
// The following values are deprecated values. Please refer to the documenation
// for the correct values to use.
//
//*****************************************************************************
#define SD24_CONVERTER_INTERRUPT                     SD24_B_CONVERTER_INTERRUPT
#define SD24_CONVERTER_OVERFLOW_INTERRUPT   SD24_B_CONVERTER_OVERFLOW_INTERRUPT
#define SD24_DATA_FORMAT_BINARY                       SD24_B_DATA_FORMAT_BINARY
#define SD24_DATA_FORMAT_2COMPLEMENT             SD24_B_DATA_FORMAT_2COMPLEMENT
#define SD24_DMA_TRIGGER_IFG0                           SD24_B_DMA_TRIGGER_IFG0
#define SD24_DMA_TRIGGER_IFG1                           SD24_B_DMA_TRIGGER_IFG1
#define SD24_DMA_TRIGGER_IFG2                           SD24_B_DMA_TRIGGER_IFG2
#define SD24_DMA_TRIGGER_IFG3                           SD24_B_DMA_TRIGGER_IFG3
#define SD24_DMA_TRIGGER_IFG4                           SD24_B_DMA_TRIGGER_IFG4
#define SD24_DMA_TRIGGER_IFG5                           SD24_B_DMA_TRIGGER_IFG5
#define SD24_DMA_TRIGGER_IFG6                           SD24_B_DMA_TRIGGER_IFG6
#define SD24_DMA_TRIGGER_IFG7                           SD24_B_DMA_TRIGGER_IFG7
#define SD24_DMA_TRIGGER_TRGIFG                       SD24_B_DMA_TRIGGER_TRGIFG

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif
#endif // __MSP430WARE_SD24_B_H__
//Released_Version_4_10_02
