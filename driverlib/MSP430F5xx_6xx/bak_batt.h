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
// bak_batt.h - Driver for the BAK_BATT Module.
//
//*****************************************************************************

#ifndef __MSP430WARE_BAK_BATT_H__
#define __MSP430WARE_BAK_BATT_H__

#include "inc/hw_memmap.h"

#ifdef __MSP430_HAS_BATTERY_CHARGER__

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
// The following are values that can be passed toThe following are values that
// can be returned by the BAK_BATT_unlockBackupSubSystem() function.
//
//*****************************************************************************
#define BAK_BATT_UNLOCKFAILURE                                        (LOCKBAK)
#define BAK_BATT_UNLOCKSUCCESS                                            (0x0)

//*****************************************************************************
//
// The following are values that can be passed to the chargerEndVoltage
// parameter for functions: BAK_BATT_chargerInitAndEnable().
//
//*****************************************************************************
#define BAK_BATT_CHARGERENDVOLTAGE_VCC                                (BAKCHV0)
#define BAK_BATT_CHARGERENDVOLTAGE2_7V                                (BAKCHV1)

//*****************************************************************************
//
// The following are values that can be passed to the chargeCurrent parameter
// for functions: BAK_BATT_chargerInitAndEnable().
//
//*****************************************************************************
#define BAK_BATT_CHARGECURRENT_5KOHM                                  (BAKCHC0)
#define BAK_BATT_CHARGECURRENT_10KOHM                                 (BAKCHC1)
#define BAK_BATT_CHARGECURRENT_20KOHM                       (BAKCHC0 + BAKCHC1)

//*****************************************************************************
//
// The following are values that can be passed to the backupRAMSelect parameter
// for functions: BAK_BATT_setBackupRAMData(), and BAK_BATT_getBackupRAMData().
//
//*****************************************************************************
#define BAK_BATT_RAMSELECT_0                                           (0x0000)
#define BAK_BATT_RAMSELECT_1                                           (0x0002)
#define BAK_BATT_RAMSELECT_2                                           (0x0004)
#define BAK_BATT_RAMSELECT_3                                           (0x0006)

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
extern uint16_t BAK_BATT_unlockBackupSubSystem(uint32_t baseAddress);

extern void BAK_BATT_enableBackupSupplyToADC(uint32_t baseAddress);

extern void BAK_BATT_disableBackupSupplyToADC(uint32_t baseAddress);

extern void BAK_BATT_manuallySwitchToBackupSupply(uint32_t baseAddress);

extern void BAK_BATT_disable(uint32_t baseAddress);

extern void BAK_BATT_chargerInitAndEnable(uint32_t baseAddress,
                                          uint8_t chargerEndVoltage,
                                          uint8_t chargeCurrent);

extern void BAK_BATT_disableCharger(uint32_t baseAddress);

extern void BAK_BATT_setBackupRAMData(uint32_t baseAddress,
                                      uint8_t backupRAMSelect,
                                      uint16_t data);

extern uint16_t BAK_BATT_getBackupRAMData(uint32_t baseAddress,
                                          uint8_t backupRAMSelect);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif
#endif // __MSP430WARE_BAK_BATT_H__
//Released_Version_4_10_02
