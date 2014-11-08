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
// ldopwr.c - Driver for the ldopwr Module.
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup ldopwr_api
//! @{
//
//*****************************************************************************

#include "inc/hw_regaccess.h"
#include "inc/hw_memmap.h"

#ifdef __MSP430_HAS_PU__
#include "ldopwr.h"

#include <assert.h>

//*****************************************************************************
//
//! \brief Unlocks the configuration registers and enables write access
//!
//! \param baseAddress is the base address of the LDOPWR module.
//!
//! Modified bits of \b LDOKEYPID register.
//!
//! \return None
//
//*****************************************************************************
void LDOPWR_unLockConfiguration( uint32_t baseAddress )
{
        HWREG16(baseAddress + OFS_LDOKEYPID) = 0x9628;
}

//*****************************************************************************
//
//! \brief Locks the configuration registers and disables write access
//!
//! \param baseAddress is the base address of the LDOPWR module.
//!
//! Modified bits of \b LDOKEYPID register.
//!
//! \return None
//
//*****************************************************************************
void LDOPWR_lockConfiguration( uint32_t baseAddress )
{
        HWREG16(baseAddress + OFS_LDOKEYPID) = 0x0000;
}

//*****************************************************************************
//
//! \brief Enables Port U inputs
//!
//! \param baseAddress is the base address of the LDOPWR module.
//!
//! Modified bits of \b PUCTL register.
//!
//! \return None
//
//*****************************************************************************
void LDOPWR_enablePort_U_inputs(uint32_t baseAddress )
{
        HWREG8(baseAddress + OFS_PUCTL_H) |= PUIPE_H;
}

//*****************************************************************************
//
//! \brief Disables Port U inputs
//!
//! \param baseAddress is the base address of the LDOPWR module.
//!
//! Modified bits of \b PUCTL register.
//!
//! \return None
//
//*****************************************************************************
void LDOPWR_disablePort_U_inputs(uint32_t baseAddress )
{
        HWREG8(baseAddress + OFS_PUCTL_H) &= ~PUIPE_H;
}

//*****************************************************************************
//
//! \brief Enables Port U outputs
//!
//! \param baseAddress is the base address of the LDOPWR module.
//!
//! Modified bits of \b PUCTL register.
//!
//! \return None
//
//*****************************************************************************
void LDOPWR_enablePort_U_outputs(uint32_t baseAddress )
{
        HWREG8(baseAddress + OFS_PUCTL_L) |= PUOPE;
}

//*****************************************************************************
//
//! \brief Disables Port U inputs
//!
//! \param baseAddress is the base address of the LDOPWR module.
//!
//! Modified bits of \b PUCTL register.
//!
//! \return None
//
//*****************************************************************************
void LDOPWR_disablePort_U_outputs(uint32_t baseAddress )
{
        HWREG8(baseAddress + OFS_PUCTL_L) &= ~PUOPE;
}

//*****************************************************************************
//
//! \brief Returns PU.1 input data
//!
//! \param baseAddress is the base address of the LDOPWR module.
//!
//! \return One of the following:
//!         - \b LDOPWR_PORTU_PIN_HIGH
//!         - \b LDOPWR_PORTU_PIN_LOW
//
//*****************************************************************************
uint8_t LDOPWR_getPort_U1_inputData(uint32_t baseAddress )
{
        return (HWREG8(baseAddress + OFS_PUCTL_L) & PUIN1) >> 3;
}

//*****************************************************************************
//
//! \brief Returns PU.0 input data
//!
//! \param baseAddress is the base address of the LDOPWR module.
//!
//! \return One of the following:
//!         - \b LDOPWR_PORTU_PIN_HIGH
//!         - \b LDOPWR_PORTU_PIN_LOW
//
//*****************************************************************************
uint8_t LDOPWR_getPort_U0_inputData(uint32_t baseAddress )
{
        return (HWREG8(baseAddress + OFS_PUCTL_L) & PUIN0) >> 2;
}

//*****************************************************************************
//
//! \brief Returns PU.1 output data
//!
//! \param baseAddress is the base address of the LDOPWR module.
//!
//! \return One of the following:
//!         - \b LDOPWR_PORTU_PIN_HIGH
//!         - \b LDOPWR_PORTU_PIN_LOW
//
//*****************************************************************************
uint8_t LDOPWR_getPort_U1_outputData(uint32_t baseAddress )
{
        return (HWREG8(baseAddress + OFS_PUCTL_L) & PUOUT1) >> 1;
}

//*****************************************************************************
//
//! \brief Returns PU.0 output data
//!
//! \param baseAddress is the base address of the LDOPWR module.
//!
//! \return One of the following:
//!         - \b LDOPWR_PORTU_PIN_HIGH
//!         - \b LDOPWR_PORTU_PIN_LOW
//
//*****************************************************************************
uint8_t LDOPWR_getPort_U0_outputData(uint32_t baseAddress )
{
        return HWREG8(baseAddress + OFS_PUCTL_L) & PUOUT0;
}

//*****************************************************************************
//
//! \brief Sets PU.1 output data
//!
//! \param baseAddress is the base address of the LDOPWR module.
//! \param value
//!        Valid values are:
//!        - \b LDOPWR_PORTU_PIN_HIGH
//!        - \b LDOPWR_PORTU_PIN_LOW
//!
//! Modified bits of \b PUCTL register.
//!
//! \return None
//
//*****************************************************************************
void LDOPWR_setPort_U1_outputData(uint32_t baseAddress,
                                  uint8_t value
                                  )
{
        if (LDOPWR_PORTU_PIN_HIGH == value)
                HWREG8(baseAddress + OFS_PUCTL_L) |= PUOUT1;
        else
                HWREG8(baseAddress + OFS_PUCTL_L) &= ~PUOUT1;
}

//*****************************************************************************
//
//! \brief Sets PU.0 output data
//!
//! \param baseAddress is the base address of the LDOPWR module.
//! \param value
//!        Valid values are:
//!        - \b LDOPWR_PORTU_PIN_HIGH
//!        - \b LDOPWR_PORTU_PIN_LOW
//!
//! Modified bits of \b PUCTL register.
//!
//! \return None
//
//*****************************************************************************
void LDOPWR_setPort_U0_outputData(uint32_t baseAddress,
                                  uint8_t value
                                  )
{
        if (LDOPWR_PORTU_PIN_HIGH == value)
                HWREG8(baseAddress + OFS_PUCTL_L) |= PUOUT0;
        else
                HWREG8(baseAddress + OFS_PUCTL_L) &= ~PUOUT0;
}

//*****************************************************************************
//
//! \brief Toggles PU.1 output data
//!
//! \param baseAddress is the base address of the LDOPWR module.
//!
//! Modified bits of \b PUCTL register.
//!
//! \return None
//
//*****************************************************************************
void LDOPWR_togglePort_U1_outputData(uint32_t baseAddress)
{
        HWREG8(baseAddress + OFS_PUCTL_L) ^= PUOUT1;
}

//*****************************************************************************
//
//! \brief Toggles PU.0 output data
//!
//! \param baseAddress is the base address of the LDOPWR module.
//!
//! Modified bits of \b PUCTL register.
//!
//! \return None
//
//*****************************************************************************
void LDOPWR_togglePort_U0_outputData(uint32_t baseAddress)
{
        HWREG8(baseAddress + OFS_PUCTL_L) ^= PUOUT0;
}

//*****************************************************************************
//
//! \brief Enables LDO-PWR module interrupts
//!
//! Does not clear interrupt flags.
//!
//! \param baseAddress is the base address of the LDOPWR module.
//! \param mask mask of interrupts to enable
//!        Mask value is the logical OR of any of the following:
//!        - \b LDOPWR_LDOI_VOLTAGE_GOING_OFF_INTERRUPT
//!        - \b LDOPWR_LDOI_VOLTAGE_COMING_ON_INTERRUPT
//!        - \b LDOPWR_LDO_OVERLOAD_INDICATION_INTERRUPT
//!
//! Modified bits of \b LDOPWRCTL register.
//!
//! \return None
//
//*****************************************************************************
void LDOPWR_enableInterrupt(uint32_t baseAddress,
                            uint16_t mask
                            )
{
        assert(0x00 == (mask & ~(LDOPWR_LDOI_VOLTAGE_GOING_OFF_INTERRUPT +
                                 LDOPWR_LDOI_VOLTAGE_COMING_ON_INTERRUPT +
                                 LDOPWR_LDO_OVERLOAD_INDICATION_INTERRUPT
                                 )
                        )
               );
        HWREG8(baseAddress + OFS_LDOPWRCTL_H) |= mask;
}

//*****************************************************************************
//
//! \brief Disables LDO-PWR module interrupts
//!
//! \param baseAddress is the base address of the LDOPWR module.
//! \param mask mask of interrupts to disable
//!        Mask value is the logical OR of any of the following:
//!        - \b LDOPWR_LDOI_VOLTAGE_GOING_OFF_INTERRUPT
//!        - \b LDOPWR_LDOI_VOLTAGE_COMING_ON_INTERRUPT
//!        - \b LDOPWR_LDO_OVERLOAD_INDICATION_INTERRUPT
//!
//! Modified bits of \b LDOPWRCTL register.
//!
//! \return None
//
//*****************************************************************************
void LDOPWR_disableInterrupt(uint32_t baseAddress,
                             uint16_t mask
                             )
{
        assert(0x00 == (mask & ~(LDOPWR_LDOI_VOLTAGE_GOING_OFF_INTERRUPT +
                                 LDOPWR_LDOI_VOLTAGE_COMING_ON_INTERRUPT +
                                 LDOPWR_LDO_OVERLOAD_INDICATION_INTERRUPT
                                 )
                        )
               );
        HWREG8(baseAddress + OFS_LDOPWRCTL_H) &= ~mask;
}

//*****************************************************************************
//
//! \brief Enables LDO-PWR module
//!
//! \param baseAddress is the base address of the LDOPWR module.
//!
//! Modified bits of \b LDOPWRCTL register.
//!
//! \return None
//
//*****************************************************************************
void LDOPWR_enable(uint32_t baseAddress)
{
        HWREG8(baseAddress + OFS_LDOPWRCTL_H) |= LDOOEN_H;
}

//*****************************************************************************
//
//! \brief Disables LDO-PWR module
//!
//! \param baseAddress is the base address of the LDOPWR module.
//!
//! Modified bits of \b LDOPWRCTL register.
//!
//! \return None
//
//*****************************************************************************
void LDOPWR_disable(uint32_t baseAddress)
{
        HWREG8(baseAddress + OFS_LDOPWRCTL_H) &= ~LDOOEN_H;
}

//*****************************************************************************
//
//! \brief Returns the interrupt status of LDO-PWR module interrupts
//!
//! \param baseAddress is the base address of the LDOPWR module.
//! \param mask mask of interrupts to get the status of
//!        Mask value is the logical OR of any of the following:
//!        - \b LDOPWR_LDOI_VOLTAGE_GOING_OFF_INTERRUPT
//!        - \b LDOPWR_LDOI_VOLTAGE_COMING_ON_INTERRUPT
//!        - \b LDOPWR_LDO_OVERLOAD_INDICATION_INTERRUPT
//!
//! \return Logical OR of any of the following:
//!         - \b LDOPWR_LDOI_VOLTAGE_GOING_OFF_INTERRUPT
//!         - \b LDOPWR_LDOI_VOLTAGE_COMING_ON_INTERRUPT
//!         - \b LDOPWR_LDO_OVERLOAD_INDICATION_INTERRUPT
//!         \n indicating the status of the masked interrupts
//
//*****************************************************************************
uint8_t LDOPWR_getInterruptStatus(uint32_t baseAddress,
                                  uint16_t mask
                                  )
{
        assert(0x00 == (mask & ~(LDOPWR_LDOI_VOLTAGE_GOING_OFF_INTERRUPT +
                                 LDOPWR_LDOI_VOLTAGE_COMING_ON_INTERRUPT +
                                 LDOPWR_LDO_OVERLOAD_INDICATION_INTERRUPT
                                 )
                        )
               );
        return HWREG8(baseAddress + OFS_LDOPWRCTL_L) & mask;
}

//*****************************************************************************
//
//! \brief Clears the interrupt status of LDO-PWR module interrupts
//!
//! \param baseAddress is the base address of the LDOPWR module.
//! \param mask mask of interrupts to clear the status of
//!        Mask value is the logical OR of any of the following:
//!        - \b LDOPWR_LDOI_VOLTAGE_GOING_OFF_INTERRUPT
//!        - \b LDOPWR_LDOI_VOLTAGE_COMING_ON_INTERRUPT
//!        - \b LDOPWR_LDO_OVERLOAD_INDICATION_INTERRUPT
//!
//! Modified bits of \b LDOPWRCTL register.
//!
//! \return None
//
//*****************************************************************************
void LDOPWR_clearInterruptStatus(uint32_t baseAddress,
                                 uint16_t mask
                                 )
{
        assert(0x00 == (mask & ~(LDOPWR_LDOI_VOLTAGE_GOING_OFF_INTERRUPT +
                                 LDOPWR_LDOI_VOLTAGE_COMING_ON_INTERRUPT +
                                 LDOPWR_LDO_OVERLOAD_INDICATION_INTERRUPT
                                 )
                        )
               );
        HWREG8(baseAddress + OFS_LDOPWRCTL_L) &=  ~mask;
}

//*****************************************************************************
//
//! \brief Returns if the the LDOI is valid and within bounds
//!
//! \param baseAddress is the base address of the LDOPWR module.
//!
//! \return One of the following:
//!         - \b LDOPWR_LDO_INPUT_VALID
//!         - \b LDOPWR_LDO_INPUT_INVALID
//
//*****************************************************************************
uint8_t LDOPWR_isLDOInputValid(uint32_t baseAddress)
{
        return HWREG8(baseAddress + OFS_LDOPWRCTL_L) & LDOBGVBV;
}

//*****************************************************************************
//
//! \brief Enables the LDO overload auto-off
//!
//! \param baseAddress is the base address of the LDOPWR module.
//!
//! Modified bits of \b LDOPWRCTL register.
//!
//! \return None
//
//*****************************************************************************
void LDOPWR_enableOverloadAutoOff(uint32_t baseAddress)
{
        HWREG8(baseAddress + OFS_LDOPWRCTL_L) |= OVLAOFF_L;
}

//*****************************************************************************
//
//! \brief Disables the LDO overload auto-off
//!
//! \param baseAddress is the base address of the LDOPWR module.
//!
//! Modified bits of \b LDOPWRCTL register.
//!
//! \return None
//
//*****************************************************************************
void LDOPWR_disableOverloadAutoOff(uint32_t baseAddress)
{
        HWREG8(baseAddress + OFS_LDOPWRCTL_L) &= ~OVLAOFF_L;
}

//*****************************************************************************
//
//! \brief Returns if the LDOI overload auto-off is enabled or disabled
//!
//! \param baseAddress is the base address of the LDOPWR module.
//!
//! \return One of the following:
//!         - \b LDOPWR_AUTOOFF_ENABLED
//!         - \b LDOPWR_AUTOOFF_DISABLED
//
//*****************************************************************************
uint8_t LDOPWR_getOverloadAutoOffStatus(uint32_t baseAddress)
{
        return HWREG8(baseAddress + OFS_LDOPWRCTL_L) & OVLAOFF_L;
}

#endif
//*****************************************************************************
//
//! Close the doxygen group for ldopwr_api
//! @}
//
//*****************************************************************************
//Released_Version_4_10_02
