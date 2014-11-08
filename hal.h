/* --COPYRIGHT--,BSD
 * Copyright (c) 2014, Texas Instruments Incorporated
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
/*
 * ======== hal.h ========
 *
 * Device and board specific pins need to be configured here
 *
 */

#define ANALOG_AX_PORT					GPIO_PORT_P7
#define ANALOG_AX_PIN					GPIO_PIN0
#define ANALOG_AY_PORT					GPIO_PORT_P6
#define ANALOG_AY_PIN					GPIO_PIN4
#define ANALOG_AZ_PORT					GPIO_PORT_P6
#define ANALOG_AZ_PIN					GPIO_PIN3
#define ANALOG_AR_PORT					GPIO_PORT_P6
#define ANALOG_AR_PIN					GPIO_PIN2
#define ANALOG_BX_PORT					GPIO_PORT_P6
#define ANALOG_BX_PIN					GPIO_PIN1
#define ANALOG_BY_PORT					GPIO_PORT_P6
#define ANALOG_BY_PIN					GPIO_PIN0
#define ANALOG_BZ_PORT					GPIO_PORT_P6
#define ANALOG_BZ_PIN					GPIO_PIN6
#define ANALOG_BR_PORT					GPIO_PORT_P6
#define ANALOG_BR_PIN					GPIO_PIN5

typedef struct PortPinMap {
	uint8_t port;
	uint8_t pin;
}PortPinMap;

extern const PortPinMap IN_PORTPIN[11];
extern const PortPinMap OUT_PORTPIN[11];

#if defined(OPTION1)
#define NUM_BUTTONS	11
#define NUM_INDICATORS	11
#elif defined(OPTION2)
#define NUM_BUTTONS	16
#define NUM_INDICATORS	6
#elif defined(OPTION3)
#define NUM_BUTTONS	20
#define NUM_INDICATORS	2
#endif

#define OUT_S1_PORT						GPIO_PORT_P3
#define OUT_S1_PIN						GPIO_PIN5
#define OUT_S2_PORT						GPIO_PORT_P2
#define OUT_S2_PIN						GPIO_PIN5
#define OUT_S3_PORT						GPIO_PORT_P2
#define OUT_S3_PIN						GPIO_PIN4

#define OUT_B1DUP_PORT						GPIO_PORT_P1
#define OUT_B1DUP_PIN						GPIO_PIN0
#define OUT_B2DUP_PORT						GPIO_PORT_P4
#define OUT_B2DUP_PIN						GPIO_PIN7

void initPorts(void);
void initClocks(uint32_t mclkFreq);
void initADC(void);
void initButtons(void);
void initIndicators(void);
void initTimer(void);
void setIndicators(void);
uint32_t buttonsPoll(void);

//Released_Version_4_10_02
