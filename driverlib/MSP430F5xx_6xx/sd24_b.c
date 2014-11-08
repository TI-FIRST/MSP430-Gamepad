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
// sd24_b.c - Driver for the sd24_b Module.
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup sd24_b_api
//! @{
//
//*****************************************************************************

#include "inc/hw_regaccess.h"
#include "inc/hw_memmap.h"

#ifdef __MSP430_HAS_SD24_B__
#include "sd24_b.h"

#include <assert.h>

//*****************************************************************************
//
//! \brief Initializes the SD24_B Module
//!
//! This function initializes the SD24_B module sigma-delta analog-to-digital
//! conversions. Specifically the function sets up the clock source for the
//! SD24_B core to use for conversions. Upon completion of the initialization
//! the SD24_B interrupt registers will be reset and the given parameters will
//! be set. The converter configuration settings are independent of this
//! function. The values you choose for the clock divider and predivider are
//! used to determine the effective clock frequency. The formula used is:
//! f_sd24 = f_clk /(divider * predivider)
//!
//! \param baseAddress is the base address of the SD24_B module.
//! \param clockSourceSelect selects the clock that will be used as the SD24_B
//!        core
//!        Valid values are:
//!        - \b SD24_B_CLOCKSOURCE_MCLK [Default]
//!        - \b SD24_B_CLOCKSOURCE_SMCLK
//!        - \b SD24_B_CLOCKSOURCE_ACLK
//!        - \b SD24_B_CLOCKSOURCE_SD24CLK
//!        \n Modified bits are \b SD24SSEL of \b SD24BCTL0 register.
//! \param clockPreDivider selects the amount that the clock will be predivided
//!        Valid values are:
//!        - \b SD24_B_PRECLOCKDIVIDER_1 [Default]
//!        - \b SD24_B_PRECLOCKDIVIDER_2
//!        - \b SD24_B_PRECLOCKDIVIDER_4
//!        - \b SD24_B_PRECLOCKDIVIDER_8
//!        - \b SD24_B_PRECLOCKDIVIDER_16
//!        - \b SD24_B_PRECLOCKDIVIDER_32
//!        - \b SD24_B_PRECLOCKDIVIDER_64
//!        - \b SD24_B_PRECLOCKDIVIDER_128
//!        \n Modified bits are \b SD24PDIVx of \b SD24BCTL0 register.
//! \param clockDivider selects the amount that the clock will be divided.
//!        Valid values are:
//!        - \b SD24_B_CLOCKDIVIDER_1 [Default]
//!        - \b SD24_B_CLOCKDIVIDER_2
//!        - \b SD24_B_CLOCKDIVIDER_3
//!        - \b SD24_B_CLOCKDIVIDER_4
//!        - \b SD24_B_CLOCKDIVIDER_5
//!        - \b SD24_B_CLOCKDIVIDER_6
//!        - \b SD24_B_CLOCKDIVIDER_7
//!        - \b SD24_B_CLOCKDIVIDER_8
//!        - \b SD24_B_CLOCKDIVIDER_9
//!        - \b SD24_B_CLOCKDIVIDER_10
//!        - \b SD24_B_CLOCKDIVIDER_11
//!        - \b SD24_B_CLOCKDIVIDER_12
//!        - \b SD24_B_CLOCKDIVIDER_13
//!        - \b SD24_B_CLOCKDIVIDER_14
//!        - \b SD24_B_CLOCKDIVIDER_15
//!        - \b SD24_B_CLOCKDIVIDER_16
//!        - \b SD24_B_CLOCKDIVIDER_17
//!        - \b SD24_B_CLOCKDIVIDER_18
//!        - \b SD24_B_CLOCKDIVIDER_19
//!        - \b SD24_B_CLOCKDIVIDER_20
//!        - \b SD24_B_CLOCKDIVIDER_21
//!        - \b SD24_B_CLOCKDIVIDER_22
//!        - \b SD24_B_CLOCKDIVIDER_23
//!        - \b SD24_B_CLOCKDIVIDER_24
//!        - \b SD24_B_CLOCKDIVIDER_25
//!        - \b SD24_B_CLOCKDIVIDER_26
//!        - \b SD24_B_CLOCKDIVIDER_27
//!        - \b SD24_B_CLOCKDIVIDER_28
//!        - \b SD24_B_CLOCKDIVIDER_29
//!        - \b SD24_B_CLOCKDIVIDER_30
//!        - \b SD24_B_CLOCKDIVIDER_31
//!        - \b SD24_B_CLOCKDIVIDER_32
//!        \n Modified bits are \b SD24DIVx of \b SD24BCTL0 register.
//! \param referenceSelect selects the reference source for the SD24_B core
//!        Valid values are:
//!        - \b SD24_B_REF_EXTERNAL [Default]
//!        - \b SD24_B_REF_INTERNAL
//!        \n Modified bits are \b SD24REFS of \b SD24BCTL0 register.
//!
//! \return None
//
//*****************************************************************************
void SD24_B_init(uint32_t baseAddress,
                 uint16_t clockSourceSelect,
                 uint16_t clockPreDivider,
                 uint16_t clockDivider,
                 uint16_t referenceSelect )
{
        assert(
                (SD24_B_CLOCKSOURCE_MCLK == clockSourceSelect) ||
                (SD24_B_CLOCKSOURCE_SMCLK == clockSourceSelect) ||
                (SD24_B_CLOCKSOURCE_ACLK == clockSourceSelect) ||
                (SD24_B_CLOCKSOURCE_SD24CLK == clockSourceSelect)
                );

        assert(
                (SD24_B_REF_EXTERNAL == referenceSelect) ||
                (SD24_B_REF_INTERNAL == referenceSelect)
                );

        // Reset all interrupts and flags
        HWREG16(baseAddress + OFS_SD24BIE)   &= 0x0000; //Reset ALL interrupt enables
        HWREG16(baseAddress + OFS_SD24BIFG)  &= 0x0000; //Reset ALL interrupt flags
        HWREG16(baseAddress + OFS_SD24BTRGCTL) &= ~(SD24TRGIE | SD24TRGIFG);

        // Turn off all group conversions
        HWREG16(baseAddress + OFS_SD24BCTL1) &= ~(SD24GRP0SC | SD24GRP1SC
                                                  | SD24GRP2SC | SD24GRP3SC);

        // Configure SD24_B
        HWREG16(baseAddress + OFS_SD24BCTL0) &= ~((SD24DIV4 | SD24DIV3 | SD24DIV2
                                                   | SD24DIV1 | SD24DIV0) | SD24PDIV_7 | SD24SSEL_3 | SD24REFS);
        HWREG16(baseAddress + OFS_SD24BCTL0) |= (clockSourceSelect | clockPreDivider
                                                 | clockDivider | referenceSelect);
        return;
}

//*****************************************************************************
//
//! \brief Configure SD24_B converter
//!
//! This function initializes a converter of the SD24_B module. Upon completion
//! the converter will be ready for a conversion and can be started with the
//! SD24_B_startGroupConversion() or SD24_B_startConverterConversion()
//! depending on the startSelect parameter. Additional configuration such as
//! data format can be configured in SD24_B_setConverterDataFormat().
//!
//! \param baseAddress is the base address of the SD24_B module.
//! \param converter selects the converter that will be configured. Check check
//!        datasheet for available converters on device.
//!        Valid values are:
//!        - \b SD24_B_CONVERTER_0
//!        - \b SD24_B_CONVERTER_1
//!        - \b SD24_B_CONVERTER_2
//!        - \b SD24_B_CONVERTER_3
//!        - \b SD24_B_CONVERTER_4
//!        - \b SD24_B_CONVERTER_5
//!        - \b SD24_B_CONVERTER_6
//!        - \b SD24_B_CONVERTER_7
//! \param alignment selects how the data will be aligned in result
//!        Valid values are:
//!        - \b SD24_B_ALIGN_RIGHT [Default]
//!        - \b SD24_B_ALIGN_LEFT
//!        \n Modified bits are \b SD24ALGN of \b SD24BCCTLx register.
//! \param startSelect selects what will trigger the start of the converter
//!        Valid values are:
//!        - \b SD24_B_CONVERSION_SELECT_SD24SC [Default]
//!        - \b SD24_B_CONVERSION_SELECT_EXT1
//!        - \b SD24_B_CONVERSION_SELECT_EXT2
//!        - \b SD24_B_CONVERSION_SELECT_EXT3
//!        - \b SD24_B_CONVERSION_SELECT_GROUP0
//!        - \b SD24_B_CONVERSION_SELECT_GROUP1
//!        - \b SD24_B_CONVERSION_SELECT_GROUP2
//!        - \b SD24_B_CONVERSION_SELECT_GROUP3
//!        \n Modified bits are \b SD24SCSx of \b SD24BCCTLx register.
//! \param conversionMode determines whether the converter will do continuous
//!        samples or a single sample
//!        Valid values are:
//!        - \b SD24_B_CONTINUOUS_MODE [Default]
//!        - \b SD24_B_SINGLE_MODE
//!        \n Modified bits are \b SD24SNGL of \b SD24BCCTLx register.
//!
//! \return None
//
//*****************************************************************************
void SD24_B_configureConverter(uint32_t baseAddress,
                               uint8_t converter,
                               uint8_t alignment,
                               uint8_t startSelect,
                               uint8_t conversionMode )
{

        assert(
                (SD24_B_CONVERTER_0 == converter) ||
                (SD24_B_CONVERTER_1 == converter) ||
                (SD24_B_CONVERTER_2 == converter) ||
                (SD24_B_CONVERTER_3 == converter) ||
                (SD24_B_CONVERTER_4 == converter) ||
                (SD24_B_CONVERTER_5 == converter) ||
                (SD24_B_CONVERTER_6 == converter) ||
                (SD24_B_CONVERTER_7 == converter)
                );

        uint16_t address = baseAddress + (OFS_SD24BCCTL0 + (converter * 0x08));

        // Clearing previous settings for configuration
        HWREG16(address) &= ~(SD24ALGN | SD24SNGL | SD24SCS__GROUP3);

        HWREG16(address) |= (alignment | startSelect |
                             (((uint16_t)conversionMode) << 8));

}

//*****************************************************************************
//
//! \brief Configure SD24_B converter - Advanced Configure
//!
//! This function initializes a converter of the SD24_B module. Upon completion
//! the converter will be ready for a conversion and can be started with the
//! SD24_B_startGroupConversion() or SD24_B_startConverterConversion()
//! depending on the startSelect parameter.
//!
//! \param baseAddress is the base address of the SD24_B module.
//! \param converter selects the converter that will be configured. Check check
//!        datasheet for available converters on device.
//!        Valid values are:
//!        - \b SD24_B_CONVERTER_0
//!        - \b SD24_B_CONVERTER_1
//!        - \b SD24_B_CONVERTER_2
//!        - \b SD24_B_CONVERTER_3
//!        - \b SD24_B_CONVERTER_4
//!        - \b SD24_B_CONVERTER_5
//!        - \b SD24_B_CONVERTER_6
//!        - \b SD24_B_CONVERTER_7
//! \param alignment selects how the data will be aligned in result
//!        Valid values are:
//!        - \b SD24_B_ALIGN_RIGHT [Default]
//!        - \b SD24_B_ALIGN_LEFT
//!        \n Modified bits are \b SD24ALGN of \b SD24BCCTLx register.
//! \param startSelect selects what will trigger the start of the converter
//!        Valid values are:
//!        - \b SD24_B_CONVERSION_SELECT_SD24SC [Default]
//!        - \b SD24_B_CONVERSION_SELECT_EXT1
//!        - \b SD24_B_CONVERSION_SELECT_EXT2
//!        - \b SD24_B_CONVERSION_SELECT_EXT3
//!        - \b SD24_B_CONVERSION_SELECT_GROUP0
//!        - \b SD24_B_CONVERSION_SELECT_GROUP1
//!        - \b SD24_B_CONVERSION_SELECT_GROUP2
//!        - \b SD24_B_CONVERSION_SELECT_GROUP3
//!        \n Modified bits are \b SD24SCSx of \b SD24BCCTLx register.
//! \param conversionMode determines whether the converter will do continuous
//!        samples or a single sample
//!        Valid values are:
//!        - \b SD24_B_CONTINUOUS_MODE [Default]
//!        - \b SD24_B_SINGLE_MODE
//!        \n Modified bits are \b SD24SNGL of \b SD24BCCTLx register.
//! \param dataFormat selects how the data format of the results
//!        Valid values are:
//!        - \b SD24_B_DATA_FORMAT_BINARY [Default]
//!        - \b SD24_B_DATA_FORMAT_2COMPLEMENT
//!        \n Modified bits are \b SD24DFx of \b SD24BCCTLx register.
//! \param sampleDelay selects the delay for the interrupt
//!        Valid values are:
//!        - \b SD24_B_FOURTH_SAMPLE_INTERRUPT [Default]
//!        - \b SD24_B_THIRD_SAMPLE_INTERRUPT
//!        - \b SD24_B_SECOND_SAMPLE_INTERRUPT
//!        - \b SD24_B_FIRST_SAMPLE_INTERRUPT
//!        \n Modified bits are \b SD24INTDLYx of \b SD24INCTLx register.
//! \param oversampleRatio selects oversampling ratio for the converter
//!        Valid values are:
//!        - \b SD24_B_OVERSAMPLE_32
//!        - \b SD24_B_OVERSAMPLE_64
//!        - \b SD24_B_OVERSAMPLE_128
//!        - \b SD24_B_OVERSAMPLE_256
//!        - \b SD24_B_OVERSAMPLE_512
//!        - \b SD24_B_OVERSAMPLE_1024
//!        \n Modified bits are \b SD24OSRx of \b SD24BOSRx register.
//! \param gain selects the gain for the converter
//!        Valid values are:
//!        - \b SD24_B_GAIN_1 [Default]
//!        - \b SD24_B_GAIN_2
//!        - \b SD24_B_GAIN_4
//!        - \b SD24_B_GAIN_8
//!        - \b SD24_B_GAIN_16
//!        - \b SD24_B_GAIN_32
//!        - \b SD24_B_GAIN_64
//!        - \b SD24_B_GAIN_128
//!        \n Modified bits are \b SD24GAINx of \b SD24BINCTLx register.
//!
//! \return None
//
//*****************************************************************************
void SD24_B_configureConverterAdvanced(uint32_t baseAddress,
                                       uint8_t converter,
                                       uint8_t alignment,
                                       uint8_t startSelect,
                                       uint8_t conversionMode,
                                       uint8_t dataFormat,
                                       uint8_t sampleDelay,
                                       uint16_t oversampleRatio,
                                       uint8_t gain
                                       )
{

        assert(
                (SD24_B_CONVERTER_0 == converter) ||
                (SD24_B_CONVERTER_1 == converter) ||
                (SD24_B_CONVERTER_2 == converter) ||
                (SD24_B_CONVERTER_3 == converter) ||
                (SD24_B_CONVERTER_4 == converter) ||
                (SD24_B_CONVERTER_5 == converter) ||
                (SD24_B_CONVERTER_6 == converter) ||
                (SD24_B_CONVERTER_7 == converter)
                );
        // Getting correct SD24BCCTLx register
        uint16_t address = baseAddress + (OFS_SD24BCCTL0 + (converter * 0x08));

        // Clearing previous settings for configuration
        HWREG16(address) &= ~(SD24ALGN | SD24SNGL | SD24DF_1 | SD24DF_0 | SD24SCS__GROUP3 );

        HWREG16(address) |= (alignment | startSelect | dataFormat |
                             (((uint16_t)conversionMode) << 8));

        // Getting correct SDBINTCTLx register
        address = baseAddress + (OFS_SD24BINCTL0 + (converter * 0x08));

        // Clearing previous settings for configuration
        HWREG16(address) &= ~(SD24GAIN_128 | SD24INTDLY_3);

        HWREG16(address) |= (gain | sampleDelay);

        // Getting correct SDBOSRx register
        address = baseAddress + (OFS_SD24BOSR0 + (converter * 0x08));

        // Clearing previous settings for configuration
        HWREG16(address) &= ~(OSR10 | OSR9 | OSR8 | OSR7 | OSR6 | OSR5 | OSR4 |
                              OSR3 | OSR2 | OSR1 | OSR0);

        HWREG16(address) |= oversampleRatio;
}

//*****************************************************************************
//
//! \brief Set SD24_B converter data format
//!
//! This function sets the converter format so that the resulting data can be
//! viewed in either binary or 2's complement.
//!
//! \param baseAddress is the base address of the SD24_B module.
//! \param converter selects the converter that will be configured. Check check
//!        datasheet for available converters on device.
//!        Valid values are:
//!        - \b SD24_B_CONVERTER_0
//!        - \b SD24_B_CONVERTER_1
//!        - \b SD24_B_CONVERTER_2
//!        - \b SD24_B_CONVERTER_3
//!        - \b SD24_B_CONVERTER_4
//!        - \b SD24_B_CONVERTER_5
//!        - \b SD24_B_CONVERTER_6
//!        - \b SD24_B_CONVERTER_7
//! \param dataFormat selects how the data format of the results
//!        Valid values are:
//!        - \b SD24_B_DATA_FORMAT_BINARY [Default]
//!        - \b SD24_B_DATA_FORMAT_2COMPLEMENT
//!        \n Modified bits are \b SD24DFx of \b SD24BCCTLx register.
//!
//! \return None
//
//*****************************************************************************
void SD24_B_setConverterDataFormat(uint32_t baseAddress,
                                   uint8_t converter,
                                   uint8_t dataFormat)
{

        uint16_t address = baseAddress + (OFS_SD24BCCTL0_L +
                                          (converter * 0x08));

        assert(
                (SD24_B_CONVERTER_0 == converter) ||
                (SD24_B_CONVERTER_1 == converter) ||
                (SD24_B_CONVERTER_2 == converter) ||
                (SD24_B_CONVERTER_3 == converter) ||
                (SD24_B_CONVERTER_4 == converter) ||
                (SD24_B_CONVERTER_5 == converter) ||
                (SD24_B_CONVERTER_6 == converter) ||
                (SD24_B_CONVERTER_7 == converter)
                );

        // Clearing previous settings for configuration
        HWREG8(address) &= ~(SD24DF0 | SD24DF1);

        HWREG8(address) |= dataFormat;
}

//*****************************************************************************
//
//! \brief Start Conversion Group
//!
//! This function starts all the converters that are associated with a group.
//! To set a converter to a group use the SD24_B_configureConverter() function.
//!
//! \param baseAddress is the base address of the SD24_B module.
//! \param group selects the group that will be started
//!        Valid values are:
//!        - \b SD24_B_GROUP0
//!        - \b SD24_B_GROUP1
//!        - \b SD24_B_GROUP2
//!        - \b SD24_B_GROUP3
//!        \n Modified bits are \b SD24DGRPxSC of \b SD24BCTL1 register.
//!
//! \return None
//
//*****************************************************************************
void SD24_B_startGroupConversion(uint32_t baseAddress,
                                 uint8_t group)
{
        switch (group) {
        case SD24_B_GROUP0:
                HWREG16(baseAddress + OFS_SD24BCTL1) |= SD24GRP0SC; break;
        case SD24_B_GROUP1:
                HWREG16(baseAddress + OFS_SD24BCTL1) |= SD24GRP1SC; break;
        case SD24_B_GROUP2:
                HWREG16(baseAddress + OFS_SD24BCTL1) |= SD24GRP2SC; break;
        case SD24_B_GROUP3:
                HWREG16(baseAddress + OFS_SD24BCTL1) |= SD24GRP3SC; break;
        }
}

//*****************************************************************************
//
//! \brief Stop Conversion Group
//!
//! This function stops all the converters that are associated with a group. To
//! set a converter to a group use the SD24_B_configureConverter() function.
//!
//! \param baseAddress is the base address of the SD24_B module.
//! \param group selects the group that will be stopped
//!        Valid values are:
//!        - \b SD24_B_GROUP0
//!        - \b SD24_B_GROUP1
//!        - \b SD24_B_GROUP2
//!        - \b SD24_B_GROUP3
//!        \n Modified bits are \b SD24DGRPxSC of \b SD24BCTL1 register.
//!
//! \return None
//
//*****************************************************************************
void SD24_B_stopGroupConversion(uint32_t baseAddress,
                                uint8_t group)
{
        switch (group) {
        case SD24_B_GROUP0:
                HWREG16(baseAddress + OFS_SD24BCTL1) &= ~(SD24GRP0SC); break;
        case SD24_B_GROUP1:
                HWREG16(baseAddress + OFS_SD24BCTL1) &= ~(SD24GRP1SC); break;
        case SD24_B_GROUP2:
                HWREG16(baseAddress + OFS_SD24BCTL1) &= ~(SD24GRP2SC); break;
        case SD24_B_GROUP3:
                HWREG16(baseAddress + OFS_SD24BCTL1) &= ~(SD24GRP3SC); break;
        }
}

//*****************************************************************************
//
//! \brief Start Conversion for Converter
//!
//! This function starts a single converter.
//!
//! \param baseAddress is the base address of the SD24_B module.
//! \param converter selects the converter that will be started
//!        Valid values are:
//!        - \b SD24_B_CONVERTER_0
//!        - \b SD24_B_CONVERTER_1
//!        - \b SD24_B_CONVERTER_2
//!        - \b SD24_B_CONVERTER_3
//!        - \b SD24_B_CONVERTER_4
//!        - \b SD24_B_CONVERTER_5
//!        - \b SD24_B_CONVERTER_6
//!        - \b SD24_B_CONVERTER_7
//!        \n Modified bits are \b SD24SC of \b SD24BCCTLx register.
//!
//! \return None
//
//*****************************************************************************
void SD24_B_startConverterConversion(uint32_t baseAddress,
                                     uint8_t converter)
{
        uint16_t address = baseAddress + (OFS_SD24BCCTL0 + (converter * 0x08));

        assert(
                (SD24_B_CONVERTER_0 == converter) ||
                (SD24_B_CONVERTER_1 == converter) ||
                (SD24_B_CONVERTER_2 == converter) ||
                (SD24_B_CONVERTER_3 == converter) ||
                (SD24_B_CONVERTER_4 == converter) ||
                (SD24_B_CONVERTER_5 == converter) ||
                (SD24_B_CONVERTER_6 == converter) ||
                (SD24_B_CONVERTER_7 == converter)
                );
        // Clearing trigger generation select
        HWREG16(address) &= ~(SD24SCS_7);

        // Setting SD24SC bit to start conversion
        HWREG16(address) |= SD24SC;
}

//*****************************************************************************
//
//! \brief Stop Conversion for Converter
//!
//! This function stops a single converter.
//!
//! \param baseAddress is the base address of the SD24_B module.
//! \param converter selects the converter that will be stopped
//!        Valid values are:
//!        - \b SD24_B_CONVERTER_0
//!        - \b SD24_B_CONVERTER_1
//!        - \b SD24_B_CONVERTER_2
//!        - \b SD24_B_CONVERTER_3
//!        - \b SD24_B_CONVERTER_4
//!        - \b SD24_B_CONVERTER_5
//!        - \b SD24_B_CONVERTER_6
//!        - \b SD24_B_CONVERTER_7
//!        \n Modified bits are \b SD24SC of \b SD24BCCTLx register.
//!
//! \return None
//
//*****************************************************************************
void SD24_B_stopConverterConversion(uint32_t baseAddress,
                                    uint8_t converter)
{
        assert(
                (SD24_B_CONVERTER_0 == converter) ||
                (SD24_B_CONVERTER_1 == converter) ||
                (SD24_B_CONVERTER_2 == converter) ||
                (SD24_B_CONVERTER_3 == converter) ||
                (SD24_B_CONVERTER_4 == converter) ||
                (SD24_B_CONVERTER_5 == converter) ||
                (SD24_B_CONVERTER_6 == converter) ||
                (SD24_B_CONVERTER_7 == converter)
                );

        uint16_t address = baseAddress + (OFS_SD24BCCTL0 + (converter * 0x08));

        // Clearing trigger generation select
        HWREG16(address) &= ~(SD24SCS_7);

        // Setting SD24SC bit to start conversion
        HWREG16(address) &= ~(SD24SC);
}

//*****************************************************************************
//
//! \brief Configures the converter that triggers a DMA transfer
//!
//! This function chooses which interrupt will trigger a DMA transfer.
//!
//! \param baseAddress is the base address of the SD24_B module.
//! \param interruptFlag selects the converter interrupt that triggers a DMA
//!        transfer.
//!        Valid values are:
//!        - \b SD24_B_DMA_TRIGGER_IFG0
//!        - \b SD24_B_DMA_TRIGGER_IFG1
//!        - \b SD24_B_DMA_TRIGGER_IFG2
//!        - \b SD24_B_DMA_TRIGGER_IFG3
//!        - \b SD24_B_DMA_TRIGGER_IFG4
//!        - \b SD24_B_DMA_TRIGGER_IFG5
//!        - \b SD24_B_DMA_TRIGGER_IFG6
//!        - \b SD24_B_DMA_TRIGGER_IFG7
//!        - \b SD24_B_DMA_TRIGGER_TRGIFG
//!        \n Modified bits are \b SD24DMAx of \b SD24BCTL1 register.
//!
//! \return None
//
//*****************************************************************************
void SD24_B_configureDMATrigger(uint32_t baseAddress,
                                uint16_t interruptFlag)
{
        // Clearing previous settings
        HWREG16(baseAddress + OFS_SD24BCTL1) &= ~(SD24DMA3_H | SD24DMA2_H |
                                                  SD24DMA1_H | SD24DMA0_H);

        HWREG16(baseAddress + OFS_SD24BCTL1) |= interruptFlag;
}

//*****************************************************************************
//
//! \brief Configures the delay for an interrupt to trigger
//!
//! This function configures the delay for the first interrupt service request
//! for the corresponding converter. This feature delays the interrupt request
//! for a completed conversion by up to four conversion cycles allowing the
//! digital filter to settle prior to generating an interrupt request.
//!
//! \param baseAddress is the base address of the SD24_B module.
//! \param converter selects the converter that will be stopped
//!        Valid values are:
//!        - \b SD24_B_CONVERTER_0
//!        - \b SD24_B_CONVERTER_1
//!        - \b SD24_B_CONVERTER_2
//!        - \b SD24_B_CONVERTER_3
//!        - \b SD24_B_CONVERTER_4
//!        - \b SD24_B_CONVERTER_5
//!        - \b SD24_B_CONVERTER_6
//!        - \b SD24_B_CONVERTER_7
//! \param sampleDelay selects the delay for the interrupt
//!        Valid values are:
//!        - \b SD24_B_FOURTH_SAMPLE_INTERRUPT [Default]
//!        - \b SD24_B_THIRD_SAMPLE_INTERRUPT
//!        - \b SD24_B_SECOND_SAMPLE_INTERRUPT
//!        - \b SD24_B_FIRST_SAMPLE_INTERRUPT
//!        \n Modified bits are \b SD24INTDLYx of \b SD24INCTLx register.
//!
//! \return None
//
//*****************************************************************************
void SD24_B_setInterruptDelay(uint32_t baseAddress,
                              uint8_t converter,
                              uint8_t sampleDelay)
{
        assert(
                (SD24_B_CONVERTER_0 == converter) ||
                (SD24_B_CONVERTER_1 == converter) ||
                (SD24_B_CONVERTER_2 == converter) ||
                (SD24_B_CONVERTER_3 == converter) ||
                (SD24_B_CONVERTER_4 == converter) ||
                (SD24_B_CONVERTER_5 == converter) ||
                (SD24_B_CONVERTER_6 == converter) ||
                (SD24_B_CONVERTER_7 == converter)
                );

        uint16_t address = baseAddress + (OFS_SD24BINCTL0 + (converter * 0x08));

        // Clear previous settings
        HWREG16(address) &= ~(SD24INTDLY_3);

        HWREG16(address) |= sampleDelay;

}

//*****************************************************************************
//
//! \brief Configures the oversampling ratio for a converter
//!
//! This function configures the oversampling ratio for a given converter.
//!
//! \param baseAddress is the base address of the SD24_B module.
//! \param converter selects the converter that will be configured
//!        Valid values are:
//!        - \b SD24_B_CONVERTER_0
//!        - \b SD24_B_CONVERTER_1
//!        - \b SD24_B_CONVERTER_2
//!        - \b SD24_B_CONVERTER_3
//!        - \b SD24_B_CONVERTER_4
//!        - \b SD24_B_CONVERTER_5
//!        - \b SD24_B_CONVERTER_6
//!        - \b SD24_B_CONVERTER_7
//! \param oversampleRatio selects oversampling ratio for the converter
//!        Valid values are:
//!        - \b SD24_B_OVERSAMPLE_32
//!        - \b SD24_B_OVERSAMPLE_64
//!        - \b SD24_B_OVERSAMPLE_128
//!        - \b SD24_B_OVERSAMPLE_256
//!        - \b SD24_B_OVERSAMPLE_512
//!        - \b SD24_B_OVERSAMPLE_1024
//!        \n Modified bits are \b SD24OSRx of \b SD24BOSRx register.
//!
//! \return None
//
//*****************************************************************************
void SD24_B_setOversampling(uint32_t baseAddress,
                            uint8_t converter,
                            uint16_t oversampleRatio)
{
        uint16_t address = baseAddress + (OFS_SD24BOSR0 + (converter * 0x08));

        assert(
                (SD24_B_CONVERTER_0 == converter) ||
                (SD24_B_CONVERTER_1 == converter) ||
                (SD24_B_CONVERTER_2 == converter) ||
                (SD24_B_CONVERTER_3 == converter) ||
                (SD24_B_CONVERTER_4 == converter) ||
                (SD24_B_CONVERTER_5 == converter) ||
                (SD24_B_CONVERTER_6 == converter) ||
                (SD24_B_CONVERTER_7 == converter)
                );
        // Clear previous settings
        HWREG16(address) &= ~(OSR10 | OSR9 | OSR8 | OSR7 | OSR6 | OSR5 | OSR4 |
                              OSR3 | OSR2 | OSR1 | OSR0);

        HWREG16(address) |= oversampleRatio;
}

//*****************************************************************************
//
//! \brief Configures the gain for the converter
//!
//! This function configures the gain for a single converter.
//!
//! \param baseAddress is the base address of the SD24_B module.
//! \param converter selects the converter that will be configured
//!        Valid values are:
//!        - \b SD24_B_CONVERTER_0
//!        - \b SD24_B_CONVERTER_1
//!        - \b SD24_B_CONVERTER_2
//!        - \b SD24_B_CONVERTER_3
//!        - \b SD24_B_CONVERTER_4
//!        - \b SD24_B_CONVERTER_5
//!        - \b SD24_B_CONVERTER_6
//!        - \b SD24_B_CONVERTER_7
//! \param gain selects the gain for the converter
//!        Valid values are:
//!        - \b SD24_B_GAIN_1 [Default]
//!        - \b SD24_B_GAIN_2
//!        - \b SD24_B_GAIN_4
//!        - \b SD24_B_GAIN_8
//!        - \b SD24_B_GAIN_16
//!        - \b SD24_B_GAIN_32
//!        - \b SD24_B_GAIN_64
//!        - \b SD24_B_GAIN_128
//!        \n Modified bits are \b SD24GAINx of \b SD24BINCTLx register.
//!
//! \return None
//
//*****************************************************************************
void SD24_B_setGain(uint32_t baseAddress,
                    uint8_t converter,
                    uint8_t gain)
{
        uint16_t address = baseAddress + (OFS_SD24BINCTL0 + (converter * 0x08));

        assert(
                (SD24_B_CONVERTER_0 == converter) ||
                (SD24_B_CONVERTER_1 == converter) ||
                (SD24_B_CONVERTER_2 == converter) ||
                (SD24_B_CONVERTER_3 == converter) ||
                (SD24_B_CONVERTER_4 == converter) ||
                (SD24_B_CONVERTER_5 == converter) ||
                (SD24_B_CONVERTER_6 == converter) ||
                (SD24_B_CONVERTER_7 == converter)
                );

        // Clear previous settings
        HWREG16(address) &= ~(SD24GAIN_128);

        HWREG16(address) |= gain;
}

//*****************************************************************************
//
//! \brief Returns the results for a converter
//!
//! This function gets the results from the SD24BMEMLx and SD24MEMHx registers
//! and concatenates them to form a long. The actual result is a maximum 24
//! bits.
//!
//! \param baseAddress is the base address of the SD24_B module.
//! \param converter selects the converter who's results will be returned
//!        Valid values are:
//!        - \b SD24_B_CONVERTER_0
//!        - \b SD24_B_CONVERTER_1
//!        - \b SD24_B_CONVERTER_2
//!        - \b SD24_B_CONVERTER_3
//!        - \b SD24_B_CONVERTER_4
//!        - \b SD24_B_CONVERTER_5
//!        - \b SD24_B_CONVERTER_6
//!        - \b SD24_B_CONVERTER_7
//!
//! \return Result of conversion
//
//*****************************************************************************
uint32_t SD24_B_getResults(uint32_t baseAddress,
                           uint8_t converter)
{
        assert(
                (SD24_B_CONVERTER_0 == converter) ||
                (SD24_B_CONVERTER_1 == converter) ||
                (SD24_B_CONVERTER_2 == converter) ||
                (SD24_B_CONVERTER_3 == converter) ||
                (SD24_B_CONVERTER_4 == converter) ||
                (SD24_B_CONVERTER_5 == converter) ||
                (SD24_B_CONVERTER_6 == converter) ||
                (SD24_B_CONVERTER_7 == converter)
                );

        // Calculating address to low word
        uint16_t address = baseAddress + (OFS_SD24BMEML0 + (converter * 0x04));

        // Getting low word result
        uint16_t lowResult = HWREG16(address);

        // Getting high word result and concatenate with low word
        uint32_t result = (((uint32_t)HWREG16(address + 0x02) ) << 16) + lowResult;

        return result;
}

//*****************************************************************************
//
//! \brief Returns the high word results for a converter
//!
//! This function gets the results from the SD24MEMHx register and returns it.
//!
//! \param baseAddress is the base address of the SD24_B module.
//! \param converter selects the converter who's results will be returned
//!        Valid values are:
//!        - \b SD24_B_CONVERTER_0
//!        - \b SD24_B_CONVERTER_1
//!        - \b SD24_B_CONVERTER_2
//!        - \b SD24_B_CONVERTER_3
//!        - \b SD24_B_CONVERTER_4
//!        - \b SD24_B_CONVERTER_5
//!        - \b SD24_B_CONVERTER_6
//!        - \b SD24_B_CONVERTER_7
//!
//! \return Result of conversion
//
//*****************************************************************************
uint16_t SD24_B_getHighWordResults(uint32_t baseAddress,
                                   uint8_t converter)
{
        assert(
                (SD24_B_CONVERTER_0 == converter) ||
                (SD24_B_CONVERTER_1 == converter) ||
                (SD24_B_CONVERTER_2 == converter) ||
                (SD24_B_CONVERTER_3 == converter) ||
                (SD24_B_CONVERTER_4 == converter) ||
                (SD24_B_CONVERTER_5 == converter) ||
                (SD24_B_CONVERTER_6 == converter) ||
                (SD24_B_CONVERTER_7 == converter)
                );

        // Calculating address
        uint16_t address = baseAddress + (OFS_SD24BMEMH0 + (converter * 0x04));

        // Getting high word result
        uint16_t result = HWREG16(address);

        return result;
}

//*****************************************************************************
//
//! \brief Enables interrupts for the SD24_B Module
//!
//! This function enables interrupts for the SD24_B module. Does not clear
//! interrupt flags.
//!
//! \param baseAddress is the base address of the SD24_B module.
//! \param converter is the selected converter.
//!        Valid values are:
//!        - \b SD24_B_CONVERTER_0
//!        - \b SD24_B_CONVERTER_1
//!        - \b SD24_B_CONVERTER_2
//!        - \b SD24_B_CONVERTER_3
//!        - \b SD24_B_CONVERTER_4
//!        - \b SD24_B_CONVERTER_5
//!        - \b SD24_B_CONVERTER_6
//!        - \b SD24_B_CONVERTER_7
//! \param mask is the bit mask of the converter interrupt sources to be
//!        enabled.
//!        Mask value is the logical OR of any of the following:
//!        - \b SD24_B_CONVERTER_INTERRUPT
//!        - \b SD24_B_CONVERTER_OVERFLOW_INTERRUPT
//!        \n Modified bits are \b SD24OVIEx of \b SD24BIE register.
//!
//! \return None
//
//*****************************************************************************
void SD24_B_enableInterrupt(uint32_t baseAddress,
                            uint8_t converter,
                            uint16_t mask)
{
        assert(
                (SD24_B_CONVERTER_0 == converter) ||
                (SD24_B_CONVERTER_1 == converter) ||
                (SD24_B_CONVERTER_2 == converter) ||
                (SD24_B_CONVERTER_3 == converter) ||
                (SD24_B_CONVERTER_4 == converter) ||
                (SD24_B_CONVERTER_5 == converter) ||
                (SD24_B_CONVERTER_6 == converter) ||
                (SD24_B_CONVERTER_7 == converter)
                );

        //Enable Interrupt
        HWREG16(baseAddress + OFS_SD24BIE) |= (mask << converter);

}

//*****************************************************************************
//
//! \brief Disables interrupts for the SD24_B Module
//!
//! This function disables interrupts for the SD24_B module.
//!
//! \param baseAddress is the base address of the SD24_B module.
//! \param converter is the selected converter.
//!        Valid values are:
//!        - \b SD24_B_CONVERTER_0
//!        - \b SD24_B_CONVERTER_1
//!        - \b SD24_B_CONVERTER_2
//!        - \b SD24_B_CONVERTER_3
//!        - \b SD24_B_CONVERTER_4
//!        - \b SD24_B_CONVERTER_5
//!        - \b SD24_B_CONVERTER_6
//!        - \b SD24_B_CONVERTER_7
//! \param mask is the bit mask of the converter interrupt sources to be
//!        disabled.
//!        Mask value is the logical OR of any of the following:
//!        - \b SD24_B_CONVERTER_INTERRUPT
//!        - \b SD24_B_CONVERTER_OVERFLOW_INTERRUPT
//!        \n Modified bits are \b SD24OVIEx of \b SD24BIE register.
//!
//! Modified bits of \b SD24BIE register.
//!
//! \return None
//
//*****************************************************************************
void SD24_B_disableInterrupt(uint32_t baseAddress,
                             uint8_t converter,
                             uint16_t mask)
{
        assert(
                (SD24_B_CONVERTER_0 == converter) ||
                (SD24_B_CONVERTER_1 == converter) ||
                (SD24_B_CONVERTER_2 == converter) ||
                (SD24_B_CONVERTER_3 == converter) ||
                (SD24_B_CONVERTER_4 == converter) ||
                (SD24_B_CONVERTER_5 == converter) ||
                (SD24_B_CONVERTER_6 == converter) ||
                (SD24_B_CONVERTER_7 == converter)
                );

        HWREG16(baseAddress + OFS_SD24BIE) &= ~(mask << converter);

}

//*****************************************************************************
//
//! \brief Clears interrupts for the SD24_B Module
//!
//! This function clears interrupt flags for the SD24_B module.
//!
//! \param baseAddress is the base address of the SD24_B module.
//! \param converter is the selected converter.
//!        Valid values are:
//!        - \b SD24_B_CONVERTER_0
//!        - \b SD24_B_CONVERTER_1
//!        - \b SD24_B_CONVERTER_2
//!        - \b SD24_B_CONVERTER_3
//!        - \b SD24_B_CONVERTER_4
//!        - \b SD24_B_CONVERTER_5
//!        - \b SD24_B_CONVERTER_6
//!        - \b SD24_B_CONVERTER_7
//! \param mask is the bit mask of the converter interrupt sources to clear.
//!        Mask value is the logical OR of any of the following:
//!        - \b SD24_B_CONVERTER_INTERRUPT
//!        - \b SD24_B_CONVERTER_OVERFLOW_INTERRUPT
//!        \n Modified bits are \b SD24OVIFGx of \b SD24BIFG register.
//!
//! \return None
//
//*****************************************************************************
void SD24_B_clearInterrupt(uint32_t baseAddress,
                           uint8_t converter,
                           uint16_t mask)
{
        assert(
                (SD24_B_CONVERTER_0 == converter) ||
                (SD24_B_CONVERTER_1 == converter) ||
                (SD24_B_CONVERTER_2 == converter) ||
                (SD24_B_CONVERTER_3 == converter) ||
                (SD24_B_CONVERTER_4 == converter) ||
                (SD24_B_CONVERTER_5 == converter) ||
                (SD24_B_CONVERTER_6 == converter) ||
                (SD24_B_CONVERTER_7 == converter)
                );
        HWREG16(baseAddress + OFS_SD24BIFG) &= ~(mask << converter);
}

//*****************************************************************************
//
//! \brief Returns the interrupt status for the SD24_B Module
//!
//! This function returns interrupt flag statuses for the SD24_B module.
//!
//! \param baseAddress is the base address of the SD24_B module.
//! \param converter is the selected converter.
//!        Valid values are:
//!        - \b SD24_B_CONVERTER_0
//!        - \b SD24_B_CONVERTER_1
//!        - \b SD24_B_CONVERTER_2
//!        - \b SD24_B_CONVERTER_3
//!        - \b SD24_B_CONVERTER_4
//!        - \b SD24_B_CONVERTER_5
//!        - \b SD24_B_CONVERTER_6
//!        - \b SD24_B_CONVERTER_7
//! \param mask is the bit mask of the converter interrupt sources to return.
//!        Mask value is the logical OR of any of the following:
//!        - \b SD24_B_CONVERTER_INTERRUPT
//!        - \b SD24_B_CONVERTER_OVERFLOW_INTERRUPT
//!
//! \return Logical OR of any of the following:
//!         - \b SD24_B_CONVERTER_INTERRUPT
//!         - \b SD24_B_CONVERTER_OVERFLOW_INTERRUPT
//!         \n indicating the status of the masked interrupts
//
//*****************************************************************************
uint16_t SD24_B_getInterruptStatus(uint32_t baseAddress,
                                   uint8_t converter,
                                   uint16_t mask)
{
        assert(
                (SD24_B_CONVERTER_0 == converter) ||
                (SD24_B_CONVERTER_1 == converter) ||
                (SD24_B_CONVERTER_2 == converter) ||
                (SD24_B_CONVERTER_3 == converter) ||
                (SD24_B_CONVERTER_4 == converter) ||
                (SD24_B_CONVERTER_5 == converter) ||
                (SD24_B_CONVERTER_6 == converter) ||
                (SD24_B_CONVERTER_7 == converter)
                );

        return HWREG16(baseAddress + OFS_SD24BIFG) & (mask << converter);
}

#endif
//*****************************************************************************
//
//! Close the doxygen group for sd24_b_api
//! @}
//
//*****************************************************************************
//Released_Version_4_10_02
