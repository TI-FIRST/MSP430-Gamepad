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
// bak_batt.c - Driver for the bak_batt Module.
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup bak_batt_api
//! @{
//
//*****************************************************************************

#include "inc/hw_regaccess.h"
#include "inc/hw_memmap.h"

#ifdef __MSP430_HAS_BATTERY_CHARGER__
#include "bak_batt.h"

#include <assert.h>

//*****************************************************************************
//
//! \brief Unlocks any pending backup input pins and RTC_B interrupts to be
//! serviced.
//!
//! This function unlocks the ability to view and service any pending backup
//! input pin interrupts, as well as pending RTC_B interrupts. The backup sub-
//! system can only be unlocked when the backup domain has settled, so this
//! function returns the status of the unlock bit after it has been to be
//! verified by user code. Please note, the backup sub-system should only be
//! unlocked after modifying the RTC_B registers.
//!
//! \param baseAddress is the base address of the BAK_BATT module.
//!
//! \return One of the following:
//!         - \b BAK_BATT_UNLOCKFAILURE backup system has not yet settled
//!         - \b BAK_BATT_UNLOCKSUCCESS successfully unlocked
//!         \n indicating if the backup system has been successfully unlocked
//
//*****************************************************************************
uint16_t BAK_BATT_unlockBackupSubSystem(uint32_t baseAddress)
{
        HWREG8(baseAddress + OFS_BAKCTL) &= ~(LOCKBAK);
        return HWREG8(baseAddress + OFS_BAKCTL) & LOCKBAK;
}

//*****************************************************************************
//
//! \brief Enables the backup supply to be measured by the ADC battery monitor
//! input.
//!
//! This function enables the backup supply signal to be monitored by the ADC
//! battery supply monitor input, to allow a measurement of the voltage from
//! the backup battery.
//!
//! \param baseAddress is the base address of the BAK_BATT module.
//!
//! \return None
//
//*****************************************************************************
void BAK_BATT_enableBackupSupplyToADC(uint32_t baseAddress)
{
        HWREG8(baseAddress + OFS_BAKCTL) |= BAKADC;
}

//*****************************************************************************
//
//! \brief Disables the backup supply input to the ADC module.
//!
//! This function disables the ability to monitor the backup supply voltage
//! from the ADC battery monitor input.
//!
//! \param baseAddress is the base address of the BAK_BATT module.
//!
//! \return None
//
//*****************************************************************************
void BAK_BATT_disableBackupSupplyToADC(uint32_t baseAddress)
{
        HWREG8(baseAddress + OFS_BAKCTL) &= ~(BAKADC);
}

//*****************************************************************************
//
//! \brief Manually switches to backup supply.
//!
//! This function uses software to manually switch to the backup battery
//! supply. Once this bit is set, it will be automatically reset by a POR and
//! the system returns to an automatic switch to backup supply.
//!
//! \param baseAddress is the base address of the BAK_BATT module.
//!
//! \return None
//
//*****************************************************************************
void BAK_BATT_manuallySwitchToBackupSupply(uint32_t baseAddress)
{
        HWREG8(baseAddress + OFS_BAKCTL) |= BAKSW;
}

//*****************************************************************************
//
//! \brief Disables backup battery system.
//!
//! This function disables the battery backup system from being used. The
//! battery backup system is re-enabled after a power cycle.
//!
//! \param baseAddress is the base address of the BAK_BATT module.
//!
//! \return None
//
//*****************************************************************************
void BAK_BATT_disable(uint32_t baseAddress)
{
        HWREG8(baseAddress + OFS_BAKCTL) |= BAKDIS;
}

//*****************************************************************************
//
//! \brief Initializes and enables the backup battery charger.
//!
//! This function initializes the backup battery charger with the selected
//! settings.
//!
//! \param baseAddress is the base address of the BAK_BATT module.
//! \param chargerEndVoltage is the maximum voltage to charge the backup
//!        battery to.
//!        Valid values are:
//!        - \b BAK_BATT_CHARGERENDVOLTAGE_VCC - charges backup battery up to
//!           Vcc
//!        - \b BAK_BATT_CHARGERENDVOLTAGE2_7V - charges backup battery up to
//!           2.7V OR up to Vcc if Vcc is less than 2.7V.
//!        \n Modified bits are \b BAKCHVx of \b BAKCHCTL register.
//! \param chargeCurrent is the maximum current to charge the backup battery
//!        at.
//!        Valid values are:
//!        - \b BAK_BATT_CHARGECURRENT_5KOHM
//!        - \b BAK_BATT_CHARGECURRENT_10KOHM
//!        - \b BAK_BATT_CHARGECURRENT_20KOHM
//!        \n Modified bits are \b BAKCHCx of \b BAKCHCTL register.
//!
//! \return None
//
//*****************************************************************************
void BAK_BATT_chargerInitAndEnable(uint32_t baseAddress,
                                   uint8_t chargerEndVoltage,
                                   uint8_t chargeCurrent)
{
        HWREG16(baseAddress +
                OFS_BAKCHCTL) = CHPWD + chargerEndVoltage + chargeCurrent + CHEN;
}

//*****************************************************************************
//
//! \brief Disables and resets backup battery charger settings.
//!
//! This function clears all backup battery charger settings and disables it.
//! To re-enable the charger, a call to BAK_BATT_chargerInitAndEnable() is
//! required.
//!
//! \param baseAddress is the base address of the BAK_BATT module.
//!
//! \return None
//
//*****************************************************************************
void BAK_BATT_disableCharger(uint32_t baseAddress)
{
        HWREG16(baseAddress + OFS_BAKCHCTL) = CHPWD;
}

//*****************************************************************************
//
//! \brief Sets data into the selected backup RAM space.
//!
//! This function sets the given 16-bit data into the selected backup RAM
//! space.
//!
//! \param baseAddress is the base address of the BAK_BATT module.
//! \param backupRAMSelect is the backup RAM space to set data into.
//!        Valid values are:
//!        - \b BAK_BATT_RAMSELECT_0
//!        - \b BAK_BATT_RAMSELECT_1
//!        - \b BAK_BATT_RAMSELECT_2
//!        - \b BAK_BATT_RAMSELECT_3
//! \param data is the data to set into the selected backup RAM space.
//!
//! \return None
//
//*****************************************************************************
void BAK_BATT_setBackupRAMData(uint32_t baseAddress,
                               uint8_t backupRAMSelect,
                               uint16_t data)
{
        HWREG16(baseAddress + backupRAMSelect) = data;
}

//*****************************************************************************
//
//! \brief Returns the data from the selected backup RAM space.
//!
//! This function returns the 16-bit data currently residing in the selected
//! backup RAM space.
//!
//! \param baseAddress is the base address of the BAK_BATT module.
//! \param backupRAMSelect is the backup RAM space to read out from.
//!        Valid values are:
//!        - \b BAK_BATT_RAMSELECT_0
//!        - \b BAK_BATT_RAMSELECT_1
//!        - \b BAK_BATT_RAMSELECT_2
//!        - \b BAK_BATT_RAMSELECT_3
//!
//! \return Data residing in the selected backup RAM space.
//
//*****************************************************************************
uint16_t BAK_BATT_getBackupRAMData(uint32_t baseAddress,
                                   uint8_t backupRAMSelect)
{
        return HWREG16(baseAddress + backupRAMSelect);
}

#endif
//*****************************************************************************
//
//! Close the doxygen group for bak_batt_api
//! @}
//
//*****************************************************************************
//Released_Version_4_10_02
