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
// aes.h - Driver for the AES Module.
//
//*****************************************************************************

#ifndef __MSP430WARE_AES_H__
#define __MSP430WARE_AES_H__

#include "inc/hw_memmap.h"

#ifdef __MSP430_HAS_AES__

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
// can be returned by the AES_isBusy() function.
//
//*****************************************************************************
#define AES_BUSY                                                        AESBUSY
#define AES_NOT_BUSY                                                       0x00

//*****************************************************************************
//
// The following are values that can be passed toThe following are values that
// can be returned by the AES_getErrorFlagStatus() function.
//
//*****************************************************************************
#define AES_ERROR_OCCURRED                                             AESERRFG
#define AES_NO_ERROR                                                       0x00

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
extern uint8_t AES_setCipherKey(uint32_t baseAddress,
                                const uint8_t *CipherKey);

extern uint8_t AES_encryptData(uint32_t baseAddress,
                               const uint8_t *Data,
                               uint8_t *encryptedData);

extern uint8_t AES_decryptData(uint32_t baseAddress,
                               const uint8_t *Data,
                               uint8_t *decryptedData);

extern uint8_t AES_setDecipherKey(uint32_t baseAddress,
                                  const uint8_t *CipherKey);

extern void AES_clearInterruptFlag(uint32_t baseAddress);

extern uint32_t AES_getInterruptFlagStatus(uint32_t baseAddress);

extern void AES_enableInterrupt(uint32_t baseAddress);

extern void AES_disableInterrupt(uint32_t baseAddress);

extern void AES_reset(uint32_t baseAddress);

extern uint8_t AES_startEncryptData(uint32_t baseAddress,
                                    const uint8_t *Data,
                                    uint8_t *encryptedData);

extern uint8_t AES_startDecryptData(uint32_t baseAddress,
                                    const uint8_t *Data);

extern uint8_t AES_startSetDecipherKey(uint32_t baseAddress,
                                       const uint8_t *CipherKey);

extern uint8_t AES_getDataOut(uint32_t baseAddress,
                              uint8_t *OutputData);

extern uint8_t AES_isBusy(uint32_t baseAddress);

extern void AES_clearErrorFlag(uint32_t baseAddress);

extern uint32_t AES_getErrorFlagStatus(uint32_t baseAddress);

extern uint8_t AES_startDecryptDataUsingEncryptionKey(uint32_t baseAddress,
                                                      const uint8_t *Data);

extern uint8_t AES_decryptDataUsingEncryptionKey(uint32_t baseAddress,
                                                 const uint8_t *Data,
                                                 uint8_t *decryptedData);

//*****************************************************************************
//
// The following are deprecated APIs.
//
//*****************************************************************************
#define AES_startGenerateFirstRoundKey                  AES_startSetDecipherKey
#define AES_generateFirstRoundKey                            AES_setDecipherKey

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif
#endif // __MSP430WARE_AES_H__
//Released_Version_4_10_02
