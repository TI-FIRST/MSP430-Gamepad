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
// tec.c - Driver for the tec Module.
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup tec_api
//! @{
//
//*****************************************************************************

#include "inc/hw_regaccess.h"
#include "inc/hw_memmap.h"

#ifdef __MSP430_HAS_TEV0__
#include "tec.h"

#include <assert.h>

//*****************************************************************************
//
//! \brief Configures the Timer Event Control External Clear Input
//!
//! \param baseAddress is the base address of the TEC module.
//! \param signalType is the selected signal type
//!        Valid values are:
//!        - \b TEC_EXTERNAL_CLEAR_SIGNALTYPE_EDGE_SENSITIVE [Default]
//!        - \b TEC_EXTERNAL_CLEAR_SIGNALTYPE_LEVEL_SENSITIVE
//! \param signalHold is the selected signal hold
//!        Valid values are:
//!        - \b TEC_EXTERNAL_CLEAR_SIGNAL_NOT_HELD [Default]
//!        - \b TEC_EXTERNAL_CLEAR_SIGNAL_HELD
//! \param polarityBit is the selected signal type
//!        Valid values are:
//!        - \b TEC_EXTERNAL_CLEAR_POLARITY_FALLING_EDGE_OR_LOW_LEVEL [Default]
//!        - \b TEC_EXTERNAL_CLEAR_POLARITY_RISING_EDGE_OR_HIGH_LEVEL
//!
//! Modified bits of \b TECxCTL2 register.
//!
//! \return None
//
//*****************************************************************************
void TEC_configureExternalClearInput(  uint32_t baseAddress,
                                       uint8_t signalType,
                                       uint8_t signalHold,
                                       uint8_t polarityBit
                                       )
{
        assert( signalType  == TEC_EXTERNAL_CLEAR_SIGNALTYPE_EDGE_SENSITIVE ||
                signalType  == TEC_EXTERNAL_CLEAR_SIGNALTYPE_LEVEL_SENSITIVE
                );
        assert( signalHold  == TEC_EXTERNAL_CLEAR_SIGNAL_NOT_HELD ||
                signalHold  == TEC_EXTERNAL_CLEAR_SIGNAL_HELD
                );
        assert( polarityBit  == TEC_EXTERNAL_CLEAR_POLARITY_FALLING_EDGE_OR_LOW_LEVEL ||
                polarityBit  == TEC_EXTERNAL_CLEAR_POLARITY_RISING_EDGE_OR_HIGH_LEVEL
                );

        HWREG8(baseAddress + OFS_TEC0XCTL2_L)  &= ~(TEC_EXTERNAL_CLEAR_SIGNALTYPE_LEVEL_SENSITIVE +
                                                    TEC_EXTERNAL_CLEAR_SIGNAL_HELD +
                                                    TEC_EXTERNAL_CLEAR_POLARITY_RISING_EDGE_OR_HIGH_LEVEL
                                                    );

        HWREG8(baseAddress + OFS_TEC0XCTL2_L)  |= (signalType +
                                                   signalHold +
                                                   polarityBit
                                                   );
}

//*****************************************************************************
//
//! \brief Configures the Timer Event Control External Fault Input
//!
//! \param baseAddress is the base address of the TEC module.
//! \param selectedExternalFault is the selected external fault
//!        Valid values are:
//!        - \b TEC_EXTERNAL_FAULT_0
//!        - \b TEC_EXTERNAL_FAULT_1
//!        - \b TEC_EXTERNAL_FAULT_2
//!        - \b TEC_EXTERNAL_FAULT_3
//!        - \b TEC_EXTERNAL_FAULT_4
//!        - \b TEC_EXTERNAL_FAULT_5
//!        - \b TEC_EXTERNAL_FAULT_6
//! \param signalType is the selected signal type
//!        Valid values are:
//!        - \b TEC_EXTERNAL_FAULT_SIGNALTYPE_EDGE_SENSITIVE [Default]
//!        - \b TEC_EXTERNAL_FAULT_SIGNALTYPE_LEVEL_SENSITIVE
//! \param signalHold is the selected signal hold
//!        Valid values are:
//!        - \b TEC_EXTERNAL_FAULT_SIGNAL_NOT_HELD [Default]
//!        - \b TEC_EXTERNAL_FAULT_SIGNAL_HELD
//! \param polarityBit is the selected signal type
//!        Valid values are:
//!        - \b TEC_EXTERNAL_FAULT_POLARITY_FALLING_EDGE_OR_LOW_LEVEL [Default]
//!        - \b TEC_EXTERNAL_FAULT_POLARITY_RISING_EDGE_OR_HIGH_LEVEL
//!
//! Modified bits of \b TECxCTL2 register.
//!
//! \return None
//
//*****************************************************************************
void TEC_configureExternalFaultInput(  uint32_t baseAddress,
                                       uint8_t selectedExternalFault,
                                       uint16_t signalType,
                                       uint8_t signalHold,
                                       uint8_t polarityBit
                                       )
{

        assert( selectedExternalFault  == TEC_EXTERNAL_FAULT_0 ||
                selectedExternalFault  == TEC_EXTERNAL_FAULT_1 ||
                selectedExternalFault  == TEC_EXTERNAL_FAULT_2 ||
                selectedExternalFault  == TEC_EXTERNAL_FAULT_3 ||
                selectedExternalFault  == TEC_EXTERNAL_FAULT_4 ||
                selectedExternalFault  == TEC_EXTERNAL_FAULT_5 ||
                selectedExternalFault  == TEC_EXTERNAL_FAULT_6
                );

        assert( signalType  == TEC_EXTERNAL_FAULT_SIGNALTYPE_EDGE_SENSITIVE ||
                signalType  == TEC_EXTERNAL_FAULT_SIGNALTYPE_LEVEL_SENSITIVE
                );

        assert( signalHold  == TEC_EXTERNAL_FAULT_SIGNAL_NOT_HELD ||
                signalHold  == TEC_EXTERNAL_FAULT_SIGNAL_HELD
                );
        assert( polarityBit  == TEC_EXTERNAL_FAULT_POLARITY_FALLING_EDGE_OR_LOW_LEVEL ||
                polarityBit  == TEC_EXTERNAL_FAULT_POLARITY_RISING_EDGE_OR_HIGH_LEVEL
                );

        HWREG8(baseAddress + OFS_TEC0XCTL2_L)  &= ~((TEC_EXTERNAL_FAULT_SIGNALTYPE_LEVEL_SENSITIVE << selectedExternalFault) +
                                                    (TEC_EXTERNAL_FAULT_POLARITY_RISING_EDGE_OR_HIGH_LEVEL << selectedExternalFault) +
                                                    (TEC_EXTERNAL_FAULT_SIGNAL_HELD << selectedExternalFault )
                                                    );

        HWREG8(baseAddress + OFS_TEC0XCTL2_L)  |= ((signalType << selectedExternalFault) +
                                                   (polarityBit << selectedExternalFault) +
                                                   (signalHold << selectedExternalFault )
                                                   );
}

//*****************************************************************************
//
//! \brief Enable the Timer Event Control External fault input
//!
//! \param baseAddress is the base address of the TEC module.
//! \param channelEventBlock selects the channel event block
//!        Valid values are:
//!        - \b TEC_CE0
//!        - \b TEC_CE1
//!        - \b TEC_CE2
//!        - \b TEC_CE3 - (available on TEC5 TEC7)
//!        - \b TEC_CE4 - (available on TEC5 TEC7)
//!        - \b TEC_CE5 - (only available on TEC7)
//!        - \b TEC_CE6 - (only available on TEC7)
//!
//! Modified bits of \b TECxCTL0 register.
//!
//! \return None
//
//*****************************************************************************
void TEC_enableExternalFaultInput(uint32_t baseAddress,
                                  uint8_t channelEventBlock
                                  )
{

        assert( channelEventBlock  == TEC_CE0 ||
                channelEventBlock  == TEC_CE1 ||
                channelEventBlock  == TEC_CE2 ||
                channelEventBlock  == TEC_CE3 ||
                channelEventBlock  == TEC_CE4 ||
                channelEventBlock  == TEC_CE5 ||
                channelEventBlock  == TEC_CE6
                );

        HWREG8(baseAddress + OFS_TEC0XCTL0_H)  |= (1 << channelEventBlock );
}

//*****************************************************************************
//
//! \brief Disable the Timer Event Control External fault input
//!
//! \param baseAddress is the base address of the TEC module.
//! \param channelEventBlock selects the channel event block
//!        Valid values are:
//!        - \b TEC_CE0
//!        - \b TEC_CE1
//!        - \b TEC_CE2
//!        - \b TEC_CE3 - (available on TEC5 TEC7)
//!        - \b TEC_CE4 - (available on TEC5 TEC7)
//!        - \b TEC_CE5 - (only available on TEC7)
//!        - \b TEC_CE6 - (only available on TEC7)
//!
//! Modified bits of \b TECxCTL0 register.
//!
//! \return None
//
//*****************************************************************************
void TEC_disableExternalFaultInput(uint32_t baseAddress,
                                   uint8_t channelEventBlock
                                   )
{

        assert( channelEventBlock  == TEC_CE0 ||
                channelEventBlock  == TEC_CE1 ||
                channelEventBlock  == TEC_CE2 ||
                channelEventBlock  == TEC_CE3 ||
                channelEventBlock  == TEC_CE4 ||
                channelEventBlock  == TEC_CE5 ||
                channelEventBlock  == TEC_CE6
                );

        HWREG8(baseAddress + OFS_TEC0XCTL0_H)  &= ~(1 << channelEventBlock );
}

//*****************************************************************************
//
//! \brief Enable the Timer Event Control External Clear Input
//!
//! \param baseAddress is the base address of the TEC module.
//!
//! Modified bits of \b TECxCTL2 register.
//!
//! \return None
//
//*****************************************************************************
void TEC_enableExternalClearInput(uint32_t baseAddress )
{
        HWREG8(baseAddress + OFS_TEC0XCTL2_L)  |= TECEXCLREN;
}

//*****************************************************************************
//
//! \brief Disable the Timer Event Control External Clear Input
//!
//! \param baseAddress is the base address of the TEC module.
//!
//! Modified bits of \b TECxCTL2 register.
//!
//! \return None
//
//*****************************************************************************
void TEC_disableExternalClearInput(uint32_t baseAddress )
{
        HWREG8(baseAddress + OFS_TEC0XCTL2_L)  &= ~TECEXCLREN;
}

//*****************************************************************************
//
//! \brief Enable the Timer Event Control Auxiliary Clear Signal
//!
//! \param baseAddress is the base address of the TEC module.
//!
//! Modified bits of \b TECxCTL2 register.
//!
//! \return None
//
//*****************************************************************************
void TEC_enableAuxiliaryClearSignal(uint32_t baseAddress )
{
        HWREG8(baseAddress + OFS_TEC0XCTL2_L)  |= TECAXCLREN;
}

//*****************************************************************************
//
//! \brief Disable the Timer Event Control Auxiliary Clear Signal
//!
//! \param baseAddress is the base address of the TEC module.
//!
//! Modified bits of \b TECxCTL2 register.
//!
//! \return None
//
//*****************************************************************************
void TEC_disableAuxiliaryClearSignal(uint32_t baseAddress )
{
        HWREG8(baseAddress + OFS_TEC0XCTL2_L)  &= ~TECAXCLREN;
}

//*****************************************************************************
//
//! \brief Clears the Timer Event Control Interrupt flag
//!
//! \param baseAddress is the base address of the TEC module.
//! \param mask is the masked interrupt flag to be cleared.
//!        Mask value is the logical OR of any of the following:
//!        - \b TEC_EXTERNAL_FAULT_INTERRUPT - External fault interrupt flag
//!        - \b TEC_EXTERNAL_CLEAR_INTERRUPT - External clear interrupt flag
//!        - \b TEC_AUXILIARY_CLEAR_INTERRUPT - Auxiliary clear interrupt flag
//!
//! Modified bits of \b TECxINT register.
//!
//! \return None
//
//*****************************************************************************
void TEC_clearInterruptFlag(uint32_t baseAddress,
                            uint8_t mask
                            )
{
        assert( 0x00 == ( mask & ~(TEC_EXTERNAL_FAULT_INTERRUPT +
                                   TEC_EXTERNAL_CLEAR_INTERRUPT +
                                   TEC_AUXILIARY_CLEAR_INTERRUPT
                                   ))
                );

        HWREG8(baseAddress + OFS_TEC0XINT_L)  &= ~mask;
}

//*****************************************************************************
//
//! \brief Gets the current Timer Event Control interrupt status.
//!
//! This returns the interrupt status for the module based on which flag is
//! passed.
//!
//! \param baseAddress is the base address of the TEC module.
//! \param mask is the masked interrupt flag status to be returned.
//!        Mask value is the logical OR of any of the following:
//!        - \b TEC_EXTERNAL_FAULT_INTERRUPT - External fault interrupt flag
//!        - \b TEC_EXTERNAL_CLEAR_INTERRUPT - External clear interrupt flag
//!        - \b TEC_AUXILIARY_CLEAR_INTERRUPT - Auxiliary clear interrupt flag
//!
//! \return Logical OR of any of the following:
//!         - \b TEC_EXTERNAL_FAULT_INTERRUPT External fault interrupt flag
//!         - \b TEC_EXTERNAL_CLEAR_INTERRUPT External clear interrupt flag
//!         - \b TEC_AUXILIARY_CLEAR_INTERRUPT Auxiliary clear interrupt flag
//!         \n indicating the status of the masked interrupts
//
//*****************************************************************************
uint8_t TEC_getInterruptStatus(uint32_t baseAddress,
                               uint8_t mask
                               )
{
        assert( 0x00 == ( mask & ~(TEC_EXTERNAL_FAULT_INTERRUPT +
                                   TEC_EXTERNAL_CLEAR_INTERRUPT +
                                   TEC_AUXILIARY_CLEAR_INTERRUPT
                                   ))
                );
        //Return the interrupt status of the request masked bit.
        return HWREG8(baseAddress + OFS_TEC0XINT_L) & mask;
}

//*****************************************************************************
//
//! \brief Enables individual Timer Event Control interrupt sources.
//!
//! Enables the indicated Timer Event Control interrupt sources. Only the
//! sources that are enabled can be reflected to the processor interrupt;
//! disabled sources have no effect on the processor. Does not clear interrupt
//! flags.
//!
//! \param baseAddress is the base address of the TEC module.
//! \param mask is the bit mask of the interrupt sources to be enabled.
//!        Mask value is the logical OR of any of the following:
//!        - \b TEC_EXTERNAL_FAULT_INTERRUPT - External fault interrupt flag
//!        - \b TEC_EXTERNAL_CLEAR_INTERRUPT - External clear interrupt flag
//!        - \b TEC_AUXILIARY_CLEAR_INTERRUPT - Auxiliary clear interrupt flag
//!
//! Modified bits of \b TECxINT register.
//!
//! \return None
//
//*****************************************************************************
void TEC_enableInterrupt(uint32_t baseAddress,
                         uint8_t mask
                         )
{
        assert( 0x00 == ( mask & ~(TEC_EXTERNAL_FAULT_INTERRUPT +
                                   TEC_EXTERNAL_CLEAR_INTERRUPT +
                                   TEC_AUXILIARY_CLEAR_INTERRUPT
                                   ))
                );

        //Enable the interrupt masked bit
        HWREG8(baseAddress + OFS_TEC0XINT_H) |= mask;
}

//*****************************************************************************
//
//! \brief Disables individual Timer Event Control interrupt sources.
//!
//! Disables the indicated Timer Event Control interrupt sources. Only the
//! sources that are enabled can be reflected to the processor interrupt;
//! disabled sources have no effect on the processor.
//!
//! \param baseAddress is the base address of the TEC module.
//! \param mask is the bit mask of the interrupt sources to be disabled.
//!        Mask value is the logical OR of any of the following:
//!        - \b TEC_EXTERNAL_FAULT_INTERRUPT - External fault interrupt flag
//!        - \b TEC_EXTERNAL_CLEAR_INTERRUPT - External clear interrupt flag
//!        - \b TEC_AUXILIARY_CLEAR_INTERRUPT - Auxiliary clear interrupt flag
//!
//! Modified bits of \b TECxINT register.
//!
//! \return None
//
//*****************************************************************************
void TEC_disableInterrupt(uint32_t baseAddress,
                          uint8_t mask
                          )
{
        assert( 0x00 == ( mask & ~(TEC_EXTERNAL_FAULT_INTERRUPT +
                                   TEC_EXTERNAL_CLEAR_INTERRUPT +
                                   TEC_AUXILIARY_CLEAR_INTERRUPT
                                   ))
                );

        //Disable the interrupt masked bit
        HWREG8(baseAddress + OFS_TEC0XINT_H) &= ~(mask);
}

//*****************************************************************************
//
//! \brief Gets the current Timer Event Control External Fault Status
//!
//! This returns the Timer Event Control fault status for the module.
//!
//! \param baseAddress is the base address of the TEC module.
//! \param mask is the masked interrupt flag status to be returned.
//!        Mask value is the logical OR of any of the following:
//!        - \b TEC_CE0
//!        - \b TEC_CE1
//!        - \b TEC_CE2
//!        - \b TEC_CE3 - (available on TEC5 TEC7)
//!        - \b TEC_CE4 - (available on TEC5 TEC7)
//!        - \b TEC_CE5 - (only available on TEC7)
//!        - \b TEC_CE6 - (only available on TEC7)
//!
//! \return Logical OR of any of the following:
//!         - \b TEC_CE0
//!         - \b TEC_CE1
//!         - \b TEC_CE2
//!         - \b TEC_CE3 (available on TEC5 TEC7)
//!         - \b TEC_CE4 (available on TEC5 TEC7)
//!         - \b TEC_CE5 (only available on TEC7)
//!         - \b TEC_CE6 (only available on TEC7)
//!         \n indicating the external fault status of the masked channel event
//!         blocks
//
//*****************************************************************************
uint8_t TEC_getExternalFaultStatus(uint32_t baseAddress,
                                   uint8_t mask
                                   )
{
        assert( 0x00 == ( mask & ~(TEC_CE0 +
                                   TEC_CE1 +
                                   TEC_CE2 +
                                   TEC_CE3 +
                                   TEC_CE4 +
                                   TEC_CE5 +
                                   TEC_CE6
                                   ))
                );
        //Return the interrupt status of the request masked bit.
        return HWREG8(baseAddress + OFS_TEC0STA_L) & mask;
}

//*****************************************************************************
//
//! \brief Clears the Timer Event Control External Fault Status
//!
//! \param baseAddress is the base address of the TEC module.
//! \param mask is the masked status flag be cleared
//!        Mask value is the logical OR of any of the following:
//!        - \b TEC_CE0
//!        - \b TEC_CE1
//!        - \b TEC_CE2
//!        - \b TEC_CE3 - (available on TEC5 TEC7)
//!        - \b TEC_CE4 - (available on TEC5 TEC7)
//!        - \b TEC_CE5 - (only available on TEC7)
//!        - \b TEC_CE6 - (only available on TEC7)
//!
//! Modified bits of \b TECxINT register.
//!
//! \return None
//
//*****************************************************************************
void TEC_clearExternalFaultStatus(uint32_t baseAddress,
                                  uint8_t mask
                                  )
{
        assert( 0x00 == ( mask & ~(TEC_CE0 +
                                   TEC_CE1 +
                                   TEC_CE2 +
                                   TEC_CE3 +
                                   TEC_CE4 +
                                   TEC_CE5 +
                                   TEC_CE6
                                   ))
                );

        HWREG8(baseAddress + OFS_TEC0STA_L)  &= ~mask;
}

//*****************************************************************************
//
//! \brief Gets the current Timer Event Control External Clear Status
//!
//! \param baseAddress is the base address of the TEC module.
//!
//! \return One of the following:
//!         - \b TEC_EXTERNAL_CLEAR_DETECTED
//!         - \b TEC_EXTERNAL_CLEAR_NOT_DETECTED
//!         \n indicating the status of the external clear
//
//*****************************************************************************
uint8_t TEC_getExternalClearStatus(uint32_t baseAddress)

{
        //Return the interrupt status of the request masked bit.
        return HWREG8(baseAddress + OFS_TEC0STA_L) & TECXCLRSTA;
}

//*****************************************************************************
//
//! \brief Clears the Timer Event Control External Clear Status
//!
//! \param baseAddress is the base address of the TEC module.
//!
//! Modified bits of \b TECxINT register.
//!
//! \return None
//
//*****************************************************************************
void TEC_clearExternalClearStatus(uint32_t baseAddress)
{
        HWREG8(baseAddress + OFS_TEC0STA_L)  &= ~TECXCLRSTA;
}

#endif
//*****************************************************************************
//
//! Close the doxygen group for tec_api
//! @}
//
//*****************************************************************************
//Released_Version_4_10_02
