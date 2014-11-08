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
// aes.c - Driver for the aes Module.
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup aes_api
//! @{
//
//*****************************************************************************

#include "inc/hw_regaccess.h"
#include "inc/hw_memmap.h"

#ifdef __MSP430_HAS_AES__
#include "aes.h"

#include <assert.h>

//*****************************************************************************
//
//! \brief Loads a 128 bit cipher key to AES module.
//!
//! \param baseAddress is the base address of the AES module.
//! \param CipherKey is a pointer to an uint8_t array with a length of 16 bytes
//!        that contains a 128 bit cipher key.
//!
//! \return STATUS_SUCCESS
//
//*****************************************************************************
uint8_t AES_setCipherKey(uint32_t baseAddress,
                         const uint8_t * CipherKey
                         )
{
        uint8_t i = 0;
        uint16_t tempVariable = 0;

        // Wait until AES accelerator is busy
        while (AESBUSY == (HWREG16(baseAddress + OFS_AESASTAT) & AESBUSY) ) ;

        for (i = 0; i < 16; i = i + 2) {
                tempVariable = (uint16_t)(CipherKey[i]);
                tempVariable = tempVariable | ((uint16_t)(CipherKey[i + 1]) << 8);
                HWREG16(baseAddress + OFS_AESAKEY) = tempVariable;
        }

        // Wait until key is written
        while (0x00 == (HWREG16(baseAddress + OFS_AESASTAT) & AESKEYWR )) ;

        return STATUS_SUCCESS;
}

//*****************************************************************************
//
//! \brief Encrypts a block of data using the AES module.
//!
//! The cipher key that is used for encryption should be loaded in advance by
//! using function \b AES_setCipherKey()
//!
//! \param baseAddress is the base address of the AES module.
//! \param Data is a pointer to an uint8_t array with a length of 16 bytes that
//!        contains data to be encrypted.
//! \param encryptedData is a pointer to an uint8_t array with a length of 16
//!        bytes in that the encrypted data will be written.
//!
//! \return STATUS_SUCCESS
//
//*****************************************************************************
uint8_t AES_encryptData(uint32_t baseAddress,
                        const uint8_t * Data,
                        uint8_t * encryptedData)
{
        uint8_t i;
        uint16_t tempData = 0;
        uint16_t tempVariable = 0;

        // Set module to encrypt mode
        HWREG16(baseAddress + OFS_AESACTL0) &= ~AESOP_3;


        // Write data to encrypt to module
        for (i = 0; i < 16; i = i + 2) {

                tempVariable = (uint16_t)(Data[i]);
                tempVariable = tempVariable | ((uint16_t)(Data[i + 1]) << 8);
                HWREG16(baseAddress + OFS_AESADIN) = tempVariable;
        }

        // Key that is already written shall be used
        // Encryption is initialized by setting AESKEYWR to 1
        HWREG16(baseAddress + OFS_AESASTAT) |= AESKEYWR;

        // Wait unit finished ~167 MCLK
        while (AESBUSY == (HWREG16(baseAddress + OFS_AESASTAT) & AESBUSY) ) ;

        // Write encrypted data back to variable
        for (i = 0; i < 16; i = i + 2) {
                tempData = HWREG16(baseAddress + OFS_AESADOUT);
                *(encryptedData + i) = (uint8_t)tempData;
                *(encryptedData + i + 1) = (uint8_t)(tempData >> 8);

        }

        return STATUS_SUCCESS;
}

//*****************************************************************************
//
//! \brief Decrypts a block of data using the AES module.
//!
//! This function requires a pre-generated decryption key. A key can be loaded
//! and pre-generated by using function \b AES_startSetDecipherKey() or \b
//! AES_setDecipherKey(). The decryption takes 167 MCLK.
//!
//! \param baseAddress is the base address of the AES module.
//! \param Data is a pointer to an uint8_t array with a length of 16 bytes that
//!        contains encrypted data to be decrypted.
//! \param decryptedData is a pointer to an uint8_t array with a length of 16
//!        bytes in that the decrypted data will be written.
//!
//! \return STATUS_SUCCESS
//
//*****************************************************************************
uint8_t AES_decryptData(uint32_t baseAddress,
                        const uint8_t * Data,
                        uint8_t * decryptedData)
{
        uint8_t i;
        uint16_t tempData = 0;
        uint16_t tempVariable = 0;

        // Set module to decrypt mode
        HWREG16(baseAddress + OFS_AESACTL0) |= (AESOP_3);

        // Write data to decrypt to module
        for (i = 0; i < 16; i = i + 2) {
                tempVariable = (uint16_t)(Data[i + 1] << 8);
                tempVariable = tempVariable | ((uint16_t)(Data[i]));
                HWREG16(baseAddress + OFS_AESADIN) = tempVariable;
        }

        // Key that is already written shall be used
        // Now decryption starts
        HWREG16(baseAddress + OFS_AESASTAT) |= AESKEYWR;

        // Wait unit finished ~167 MCLK
        while (AESBUSY == (HWREG16(baseAddress + OFS_AESASTAT) & AESBUSY )) ;

        // Write encrypted data back to variable
        for (i = 0; i < 16; i = i + 2) {
                tempData = HWREG16(baseAddress + OFS_AESADOUT);
                *(decryptedData + i ) = (uint8_t)tempData;
                *(decryptedData + i + 1) = (uint8_t)(tempData >> 8);
        }

        return STATUS_SUCCESS;
}

//*****************************************************************************
//
//! \brief Sets the decipher key The API
//!
//! The API \b AES_startSetDecipherKey() or \b AES_setDecipherKey() must be
//! invoked before invoking \b AES_setDecipherKey().
//!
//! \param baseAddress is the base address of the AES module.
//! \param CipherKey is a pointer to an uint8_t array with a length of 16 bytes
//!        that contains the initial AES key.
//!
//! \return STATUS_SUCCESS
//
//*****************************************************************************
uint8_t AES_setDecipherKey(uint32_t baseAddress,
                           const uint8_t * CipherKey)
{
        uint8_t i;
        uint16_t tempVariable = 0;

        // Set module to decrypt mode
        HWREG16(baseAddress + OFS_AESACTL0) &= ~(AESOP0);
        HWREG16(baseAddress + OFS_AESACTL0) |= AESOP1;

        // Write cipher key to key register
        for (i = 0; i < 16; i = i + 2) {
                tempVariable = (uint16_t)(CipherKey[i]);
                tempVariable = tempVariable | ((uint16_t)(CipherKey[i + 1]) << 8);
                HWREG16(baseAddress + OFS_AESAKEY) = tempVariable;
        }

        // Wait until key is processed ~52 MCLK
        while ((HWREG16(baseAddress + OFS_AESASTAT) & AESBUSY) == AESBUSY) ;

        return STATUS_SUCCESS;
}

//*****************************************************************************
//
//! \brief Clears the AES ready interrupt flag.
//!
//! \param baseAddress is the base address of the AES module.
//!
//! Modified bits are \b AESRDYIFG of \b AESACTL0 register.
//!
//! \return None
//
//*****************************************************************************
void AES_clearInterruptFlag(uint32_t baseAddress )
{
        HWREG8(baseAddress + OFS_AESACTL0) &=  ~AESRDYIFG;
}

//*****************************************************************************
//
//! \brief Gets the AES ready interrupt flag status.
//!
//! \param baseAddress is the base address of the AES module.
//!
//! \return uint32_t - AES_READY_INTERRUPT or 0x00.
//
//*****************************************************************************
uint32_t AES_getInterruptFlagStatus(uint32_t baseAddress)
{
        return (HWREG8(baseAddress + OFS_AESACTL0) & AESRDYIFG) << 0x04;
}

//*****************************************************************************
//
//! \brief Enables AES ready interrupt.
//!
//! Does not clear interrupt flags.
//!
//! \param baseAddress is the base address of the AES module.
//!
//! Modified bits are \b AESRDYIE of \b AESACTL0 register.
//!
//! \return None
//
//*****************************************************************************
void AES_enableInterrupt(uint32_t baseAddress)
{
        HWREG8(baseAddress + OFS_AESACTL0) |=  AESRDYIE;
}

//*****************************************************************************
//
//! \brief Disables AES ready interrupt.
//!
//! \param baseAddress is the base address of the AES module.
//!
//! Modified bits are \b AESRDYIE of \b AESACTL0 register.
//!
//! \return None
//
//*****************************************************************************
void AES_disableInterrupt(uint32_t baseAddress)
{
        HWREG8(baseAddress + OFS_AESACTL0) &=  ~AESRDYIE;
}

//*****************************************************************************
//
//! \brief Resets AES Module immediately.
//!
//! \param baseAddress is the base address of the AES module.
//!
//! Modified bits are \b AESSWRST of \b AESACTL0 register.
//!
//! \return None
//
//*****************************************************************************
void AES_reset(uint32_t baseAddress)
{
        HWREG8(baseAddress + OFS_AESACTL0) |=  AESSWRST;
}

//*****************************************************************************
//
//! \brief Starts an encryption process on the AES module.
//!
//! This is the non-blocking equivalent of AES_encryptData(). The cipher key
//! that is used for decryption should be loaded in advance by using function
//! \b AES_setCipherKey(). It is recommended to use interrupt to check for
//! procedure completion then using AES_getDataOut() API to retrieve the
//! encrypted data.
//!
//! \param baseAddress is the base address of the AES module.
//! \param Data is a pointer to an uint8_t array with a length of 16 bytes that
//!        contains data to be encrypted.
//! \param encryptedData is a pointer to an uint8_t array with a length of 16
//!        bytes in that the encrypted data will be written.
//!
//! \return STATUS_SUCCESS
//
//*****************************************************************************
uint8_t AES_startEncryptData(uint32_t baseAddress,
                             const uint8_t * Data,
                             uint8_t * encryptedData)
{
        uint8_t i;
        uint16_t tempVariable = 0;

        // Set module to encrypt mode
        HWREG16(baseAddress + OFS_AESACTL0) &= ~AESOP_3;


        // Write data to encrypt to module
        for (i = 0; i < 16; i = i + 2) {
                tempVariable = (uint16_t)(Data[i]);
                tempVariable = tempVariable | ((uint16_t)(Data[i + 1]) << 8);
                HWREG16(baseAddress + OFS_AESADIN) = tempVariable;
        }

        // Key that is already written shall be used
        // Encryption is initialized by setting AESKEYWR to 1
        HWREG16(baseAddress + OFS_AESASTAT) |= AESKEYWR;

        return STATUS_SUCCESS;
}

//*****************************************************************************
//
//! \brief Decrypts a block of data using the AES module.
//!
//! This is the non-blocking equivalent of AES_decryptData(). This function
//! requires a pre-generated decryption key. A key can be loaded and pre-
//! generated by using function \b AES_setDecipherKey() or \b
//! AES_startSetDecipherKey(). The decryption takes 167 MCLK. It is recommended
//! to use interrupt to check for procedure completion then using
//! AES_getDataOut() API to retrieve the decrypted data.
//!
//! \param baseAddress is the base address of the AES module.
//! \param Data is a pointer to an uint8_t array with a length of 16 bytes that
//!        contains encrypted data to be decrypted.
//!
//! \return STATUS_SUCCESS
//
//*****************************************************************************
uint8_t AES_startDecryptData(uint32_t baseAddress,
                             const uint8_t * Data)
{
        uint8_t i;
        uint16_t tempVariable = 0;

        // Set module to decrypt mode
        HWREG16(baseAddress + OFS_AESACTL0) |= (AESOP_3);

        // Write data to decrypt to module
        for (i = 0; i < 16; i = i + 2) {
                tempVariable = (uint16_t)(Data[i + 1] << 8);
                tempVariable = tempVariable | ((uint16_t)(Data[i]));
                HWREG16(baseAddress + OFS_AESADIN) = tempVariable;
        }

        // Key that is already written shall be used
        // Now decryption starts
        HWREG16(baseAddress + OFS_AESASTAT) |= AESKEYWR;

        return STATUS_SUCCESS;
}

//*****************************************************************************
//
//! \brief Loads the decipher key.
//!
//! This is the non-blocking equivalent of AES_setDecipherKey(). The API \b
//! AES_startSetDecipherKey() or \b AES_setDecipherKey() must be invoked before
//! invoking \b AES_startSetDecipherKey().
//!
//! \param baseAddress is the base address of the AES module.
//! \param CipherKey is a pointer to an uint8_t array with a length of 16 bytes
//!        that contains the initial AES key.
//!
//! \return STATUS_SUCCESS
//
//*****************************************************************************
uint8_t AES_startSetDecipherKey(uint32_t baseAddress,
                                const uint8_t * CipherKey)
{
        uint8_t i;
        uint16_t tempVariable = 0;

        HWREG16(baseAddress + OFS_AESACTL0) &= ~(AESOP0);
        HWREG16(baseAddress + OFS_AESACTL0) |= AESOP1;

        // Write cipher key to key register
        for (i = 0; i < 16; i = i + 2) {
                tempVariable = (uint16_t)(CipherKey[i]);
                tempVariable = tempVariable | ((uint16_t)(CipherKey[i + 1]) << 8);
                HWREG16(baseAddress + OFS_AESAKEY) = tempVariable;
        }

        return STATUS_SUCCESS;
}

//*****************************************************************************
//
//! \brief Reads back the output data from AES module.
//!
//! This function is meant to use after an encryption or decryption process
//! that was started and finished by initiating an interrupt by use of the \b
//! AES_startEncryptData() or \b AES_startDecryptData() functions.
//!
//! \param baseAddress is the base address of the AES module.
//! \param OutputData is a pointer to an uint8_t array with a length of 16
//!        bytes in which the output data of the AES module is available. If
//!        AES module is busy returns NULL.
//!
//! \return STATUS_SUCCESS if AES is not busy, STATUS_FAIL if it is busy
//
//*****************************************************************************
uint8_t  AES_getDataOut(uint32_t baseAddress,
                        uint8_t *OutputData
                        )
{
        uint8_t i;
        uint16_t tempData = 0;

        // If module is busy, exit and return failure
        if ( AESBUSY == (HWREG16(baseAddress + OFS_AESASTAT) & AESBUSY))
                return STATUS_FAIL;

        // Write encrypted data back to variable
        for (i = 0; i < 16; i = i + 2) {
                tempData = HWREG16(baseAddress + OFS_AESADOUT);
                *(OutputData + i) = (uint8_t)tempData;
                *(OutputData + i + 1) = (uint8_t)(tempData >> 8);
        }

        return STATUS_SUCCESS;
}

//*****************************************************************************
//
//! \brief Gets the AES module busy status.
//!
//! \param baseAddress is the base address of the AES module.
//!
//! \return One of the following:
//!         - \b AES_BUSY
//!         - \b AES_NOT_BUSY
//!         \n indicating if encryption/decryption/key generation is taking
//!         place
//
//*****************************************************************************
uint8_t AES_isBusy(uint32_t baseAddress)
{
        return HWREG16(baseAddress + OFS_AESASTAT) & AESBUSY;
}

//*****************************************************************************
//
//! \brief Clears the AES error flag.
//!
//! Modified bit is AESERRFG of AESACTL0 register.
//!
//! \param baseAddress is the base address of the AES module.
//!
//! Modified bits are \b AESERRFG of \b AESACTL0 register.
//!
//! \return None
//
//*****************************************************************************
void AES_clearErrorFlag(uint32_t baseAddress )
{
        HWREG8(baseAddress + OFS_AESACTL0) &=  ~AESERRFG;
}

//*****************************************************************************
//
//! \brief Gets the AES error flag status.
//!
//! \param baseAddress is the base address of the AES module.
//!
//! \return One of the following:
//!         - \b AES_ERROR_OCCURRED
//!         - \b AES_NO_ERROR
//!         \n indicating if AESAKEY or AESADIN were written while an AES
//!         operation was in progress
//
//*****************************************************************************
uint32_t AES_getErrorFlagStatus(uint32_t baseAddress)
{
        return HWREG8(baseAddress + OFS_AESACTL0) & AESERRFG;
}

//*****************************************************************************
//
//! \brief DEPRECATED Starts an decryption process on the AES module.
//!
//! This is the non-blocking equivalent of AES_decryptDataUsingEncryptionKey().
//! This function can be used to decrypt data by using the same key as used for
//! a previous performed encryption. The decryption takes 214 MCLK.
//!
//! \param baseAddress is the base address of the AES module.
//! \param Data is a pointer to an uint8_t array with a length of 16 bytes that
//!        contains encrypted data to be decrypted.
//!
//! \return STATUS_SUCCESS
//
//*****************************************************************************
uint8_t AES_startDecryptDataUsingEncryptionKey(
        uint32_t baseAddress,
        const uint8_t * Data)
{
        uint8_t i;
        uint16_t tempVariable = 0;

        // Set module to decrypt mode
        HWREG16(baseAddress + OFS_AESACTL0) &= ~(AESOP1);
        HWREG16(baseAddress + OFS_AESACTL0) |= AESOP0;

        // Write data to decrypt to module
        for (i = 0; i < 16; i = i + 2) {
                tempVariable = (uint16_t)(Data[i + 1] << 8);
                tempVariable = tempVariable | ((uint16_t)(Data[i]));
                HWREG16(baseAddress + OFS_AESADIN) = tempVariable;
        }

        // Key that is already written shall be used
        // Now decryption starts
        HWREG16(baseAddress + OFS_AESASTAT) |= AESKEYWR;

        return STATUS_SUCCESS;
}

//*****************************************************************************
//
//! \brief DEPRECATED Decrypts a block of data using the AES module.
//!
//! This function can be used to decrypt data by using the same key as used for
//! a previous performed encryption. The decryption takes 214 MCLK.
//!
//! \param baseAddress is the base address of the AES module.
//! \param Data is a pointer to an uint8_t array with a length of 16 bytes that
//!        contains encrypted data to be decrypted.
//! \param decryptedData is a pointer to an uint8_t array with a length of 16
//!        bytes in that the decrypted data will be written.
//!
//! \return STATUS_SUCCESS
//
//*****************************************************************************
uint8_t AES_decryptDataUsingEncryptionKey(uint32_t baseAddress,
                                          const uint8_t * Data,
                                          uint8_t * decryptedData)
{
        uint8_t i;
        uint16_t tempData = 0;
        uint16_t tempVariable = 0;

        // Set module to decrypt mode
        HWREG16(baseAddress + OFS_AESACTL0) &= ~(AESOP1);
        HWREG16(baseAddress + OFS_AESACTL0) |= AESOP0;

        // Write data to decrypt to module
        for (i = 0; i < 16; i = i + 2) {
                tempVariable = (uint16_t)(Data[i + 1] << 8);
                tempVariable = tempVariable | ((uint16_t)(Data[i]));
                HWREG16(baseAddress + OFS_AESADIN) = tempVariable;
        }

        // Key that is already written shall be used
        // Now decryption starts
        HWREG16(baseAddress + OFS_AESASTAT) |= AESKEYWR;

        // Wait unit finished ~214 MCLK
        while (AESBUSY == (HWREG16(baseAddress + OFS_AESASTAT) & AESBUSY) ) ;

        // Write encrypted data back to variable
        for (i = 0; i < 16; i = i + 2) {
                tempData = HWREG16(baseAddress + OFS_AESADOUT);
                *(decryptedData + i ) = (uint8_t)tempData;
                *(decryptedData + i + 1) = (uint8_t)(tempData >> 8);
        }

        return STATUS_SUCCESS;
}

#endif
//*****************************************************************************
//
//! Close the doxygen group for aes_api
//! @}
//
//*****************************************************************************
//Released_Version_4_10_02
