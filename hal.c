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
 * ======== hal.c ========
 *
 */
#include "msp430.h"

#include "driverlib.h"

#include "USB_API/USB_Common/device.h"
#include "USB_config/descriptors.h"
#include "USB_app/USB_gamepad.h"

#include "hal.h"

#if defined(OPTION1)
#define NUM_BUTTONS	11
#define NUM_INDICATORS	11

const PortPinMap BUTTONS_PORTPIN[NUM_BUTTONS] = {
	{GPIO_PORT_P1, GPIO_PIN6},
	{GPIO_PORT_P3, GPIO_PIN2},
	{GPIO_PORT_P2, GPIO_PIN7},
	{GPIO_PORT_P4, GPIO_PIN2},
	{GPIO_PORT_P4, GPIO_PIN1},
	{GPIO_PORT_P3, GPIO_PIN6},
	{GPIO_PORT_P3, GPIO_PIN5},
	{GPIO_PORT_P2, GPIO_PIN2},
	{GPIO_PORT_P7, GPIO_PIN4},
	{GPIO_PORT_P1, GPIO_PIN5},
	{GPIO_PORT_P1, GPIO_PIN4}
};

const PortPinMap INDICATORS_PORTPIN[NUM_INDICATORS] = {
	{GPIO_PORT_P3, GPIO_PIN0},
	{GPIO_PORT_P3, GPIO_PIN1},
	{GPIO_PORT_P2, GPIO_PIN6},
	{GPIO_PORT_P2, GPIO_PIN3},
	{GPIO_PORT_P8, GPIO_PIN1},
	{GPIO_PORT_P1, GPIO_PIN3},
	{GPIO_PORT_P1, GPIO_PIN2},
	{GPIO_PORT_P4, GPIO_PIN3},
	{GPIO_PORT_P4, GPIO_PIN0},
	{GPIO_PORT_P3, GPIO_PIN7},
	{GPIO_PORT_P8, GPIO_PIN2},
};
#elif defined(OPTION2)
#define NUM_BUTTONS	16
#define NUM_INDICATORS	6

const PortPinMap BUTTONS_PORTPIN[NUM_BUTTONS] = {
	{GPIO_PORT_P1, GPIO_PIN6},
	{GPIO_PORT_P3, GPIO_PIN2},
	{GPIO_PORT_P2, GPIO_PIN7},
	{GPIO_PORT_P4, GPIO_PIN2},
	{GPIO_PORT_P4, GPIO_PIN1},
	{GPIO_PORT_P3, GPIO_PIN6},
	{GPIO_PORT_P3, GPIO_PIN5},
	{GPIO_PORT_P2, GPIO_PIN2},
	{GPIO_PORT_P7, GPIO_PIN4},
	{GPIO_PORT_P3, GPIO_PIN0},
	{GPIO_PORT_P3, GPIO_PIN1},
	{GPIO_PORT_P1, GPIO_PIN5},
	{GPIO_PORT_P1, GPIO_PIN4},
	{GPIO_PORT_P1, GPIO_PIN3},
	{GPIO_PORT_P1, GPIO_PIN2},
	{GPIO_PORT_P4, GPIO_PIN3},
};

const PortPinMap INDICATORS_PORTPIN[NUM_INDICATORS] = {

	{GPIO_PORT_P2, GPIO_PIN6},
	{GPIO_PORT_P2, GPIO_PIN3},
	{GPIO_PORT_P8, GPIO_PIN1},
	{GPIO_PORT_P4, GPIO_PIN0},
	{GPIO_PORT_P3, GPIO_PIN7},
	{GPIO_PORT_P8, GPIO_PIN2},
};

#elif defined(OPTION3)
#define NUM_BUTTONS	20
#define NUM_INDICATORS	2

const PortPinMap BUTTONS_PORTPIN[NUM_BUTTONS] = {
	{GPIO_PORT_P1, GPIO_PIN6},
	{GPIO_PORT_P3, GPIO_PIN2},
	{GPIO_PORT_P2, GPIO_PIN7},
	{GPIO_PORT_P4, GPIO_PIN2},
	{GPIO_PORT_P4, GPIO_PIN1},
	{GPIO_PORT_P3, GPIO_PIN6},
	{GPIO_PORT_P3, GPIO_PIN5},
	{GPIO_PORT_P2, GPIO_PIN2},
	{GPIO_PORT_P7, GPIO_PIN4},
	{GPIO_PORT_P3, GPIO_PIN0},
	{GPIO_PORT_P3, GPIO_PIN1},
	{GPIO_PORT_P2, GPIO_PIN6},
	{GPIO_PORT_P2, GPIO_PIN3},
	{GPIO_PORT_P1, GPIO_PIN5},
	{GPIO_PORT_P1, GPIO_PIN4},
	{GPIO_PORT_P1, GPIO_PIN3},
	{GPIO_PORT_P1, GPIO_PIN2},
	{GPIO_PORT_P4, GPIO_PIN3},
	{GPIO_PORT_P4, GPIO_PIN0},
	{GPIO_PORT_P3, GPIO_PIN7},
};

const PortPinMap INDICATORS_PORTPIN[NUM_INDICATORS] = {
	{GPIO_PORT_P8, GPIO_PIN1},
	{GPIO_PORT_P8, GPIO_PIN2},
};
#endif

#define GPIO_ALL	GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3| \
					GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7

/* Use this macro to convert 16bit ADC values to 8-bit if needed */
//#define Convert8Bit(ui32Value)  ((int8_t)((0x7ff - ui32Value) >> 4))
#define Convert8Bit(ui32Value)  (ui32Value)

extern volatile uint8_t adcUpdateAvailable;
volatile USB_GamepadReportTx gamepadReportTx;
volatile USB_GamepadReportRx gamepadReportRx;

/*
 * This function drives all the I/O's as output-low, to avoid floating inputs
 * (which cause extra power to be consumed).  This setting is compatible with  
 * TI FET target boards, the F5529 Launchpad, and F5529 Experimenters Board;  
 * but may not be compatible with custom hardware, which may have components  
 * attached to the I/Os that could be affected by these settings.  So if using
 * other boards, this function may need to be modified.
 */
void initPorts(void)
{
#ifdef __MSP430_HAS_PORT1_R__
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_ALL);
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_ALL);
#endif

#ifdef __MSP430_HAS_PORT2_R__
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_ALL);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_ALL);
#endif

#ifdef __MSP430_HAS_PORT3_R__
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_ALL);
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_ALL);
#endif

#ifdef __MSP430_HAS_PORT4_R__
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_ALL);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_ALL);
#endif

#ifdef __MSP430_HAS_PORT5_R__
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_ALL);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_ALL);
#endif

#ifdef __MSP430_HAS_PORT6_R__
    GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_ALL);
    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_ALL);
#endif

#ifdef __MSP430_HAS_PORT7_R__
    GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_ALL);
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_ALL);
#endif

#ifdef __MSP430_HAS_PORT8_R__
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_ALL);
    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_ALL);
#endif

#ifdef __MSP430_HAS_PORT9_R__
    GPIO_setOutputLowOnPin(GPIO_PORT_P9, GPIO_ALL);
    GPIO_setAsOutputPin(GPIO_PORT_P9, GPIO_ALL);
#endif

#ifdef __MSP430_HAS_PORTJ_R__
    GPIO_setOutputLowOnPin(GPIO_PORT_PJ, GPIO_ALL);
    GPIO_setAsOutputPin(GPIO_PORT_PJ, GPIO_ALL);
#endif
}

/* Configures the system clocks:
* MCLK = SMCLK = DCO/FLL = mclkFreq (expected to be expressed in Hz)
* ACLK = FLLref = REFO=32kHz
*
* XT2 is not configured here.  Instead, the USB API automatically starts XT2
* when beginning USB communication, and optionally disables it during USB
* suspend.  It's left running after the USB host is disconnected, at which
* point you're free to disable it.  You need to configure the XT2 frequency
* in the Descriptor Tool (currently set to 4MHz in this example).
* See the Programmer's Guide for more information.
*/
void initClocks(uint32_t mclkFreq)
{
#ifndef DRIVERLIB_LEGACY_MODE
	UCS_clockSignalInit(
	   UCS_FLLREF,
	   UCS_REFOCLK_SELECT,
	   UCS_CLOCK_DIVIDER_1);

	UCS_clockSignalInit(
	   UCS_ACLK,
	   UCS_REFOCLK_SELECT,
	   UCS_CLOCK_DIVIDER_1);

    UCS_initFLLSettle(
        mclkFreq/1000,
        mclkFreq/32768);
#else

	UCS_clockSignalInit(
       UCS_BASE,
	   UCS_FLLREF,
	   UCS_REFOCLK_SELECT,
	   UCS_CLOCK_DIVIDER_1);

	UCS_clockSignalInit(
       UCS_BASE,
	   UCS_ACLK,
	   UCS_REFOCLK_SELECT,
	   UCS_CLOCK_DIVIDER_1);

    UCS_initFLLSettle(
        UCS_BASE,
        mclkFreq/1000,
        mclkFreq/32768);
#endif

}

void initButtons(void)
{
	int8_t i;

	for (i = 0; i < NUM_BUTTONS; i++) {
		GPIO_setAsInputPinWithPullDownresistor(BUTTONS_PORTPIN[i].port, BUTTONS_PORTPIN[i].pin);
	}
}

void initIndicators(void)
{
	int8_t i;

	for (i = 0; i < NUM_INDICATORS; i++) {
		GPIO_setAsOutputPin(INDICATORS_PORTPIN[i].port, INDICATORS_PORTPIN[i].pin);
		GPIO_setDriveStrength(INDICATORS_PORTPIN[i].port, INDICATORS_PORTPIN[i].pin, GPIO_FULL_OUTPUT_DRIVE_STRENGTH);
		GPIO_setOutputLowOnPin(INDICATORS_PORTPIN[i].port, INDICATORS_PORTPIN[i].pin);
	}
	GPIO_setAsOutputPin(OUT_B1DUP_PORT, OUT_B1DUP_PIN);
	GPIO_setAsOutputPin(OUT_B2DUP_PORT, OUT_B2DUP_PIN);
	GPIO_setDriveStrength(OUT_B1DUP_PORT, OUT_B1DUP_PIN, GPIO_FULL_OUTPUT_DRIVE_STRENGTH);
	GPIO_setDriveStrength(OUT_B2DUP_PORT, OUT_B2DUP_PIN, GPIO_FULL_OUTPUT_DRIVE_STRENGTH);
	GPIO_setOutputLowOnPin(OUT_B1DUP_PORT, OUT_B1DUP_PIN);
	GPIO_setOutputLowOnPin(OUT_B2DUP_PORT, OUT_B2DUP_PIN);
}

uint32_t buttonsPoll(void)
{
	uint32_t temp = 0;
	uint8_t i;

	for (i = 0; i < NUM_BUTTONS; i++) {
	    if (GPIO_getInputPinValue(BUTTONS_PORTPIN[i].port, BUTTONS_PORTPIN[i].pin)){
	    	temp |= ((uint32_t)0x1 << i);
	    }
	}

    return (temp);
}

void setIndicators(void)
{
	int8_t i;

	for (i = 0; i < NUM_INDICATORS; i++) {
		if (gamepadReportRx.indicators & (0x01 << i)) {
			GPIO_setOutputHighOnPin(INDICATORS_PORTPIN[i].port, INDICATORS_PORTPIN[i].pin);
		}
		else {
			GPIO_setOutputLowOnPin(INDICATORS_PORTPIN[i].port, INDICATORS_PORTPIN[i].pin);
		}
	}
	if (gamepadReportRx.indicators & 0x01) {
		GPIO_setOutputHighOnPin(OUT_B1DUP_PORT, OUT_B1DUP_PIN);
	}
	else {
		GPIO_setOutputLowOnPin(OUT_B1DUP_PORT, OUT_B1DUP_PIN);
	}
	if (gamepadReportRx.indicators & 0x02) {
		GPIO_setOutputHighOnPin(OUT_B2DUP_PORT, OUT_B2DUP_PIN);
	}
	else {
		GPIO_setOutputLowOnPin(OUT_B2DUP_PORT, OUT_B2DUP_PIN);
	}
}

#define COMPARE_VALUE 328

void initTimer(void)
{

    //Set P1.0 to output direction
    GPIO_setAsOutputPin(
            GPIO_PORT_P1,
            GPIO_PIN0
            );

    //Start timer in continuous mode sourced by SMCLK
    TIMER_B_configureContinuousMode( TIMER_B0_BASE,
                                     TIMER_B_CLOCKSOURCE_ACLK,
                                     TIMER_B_CLOCKSOURCE_DIVIDER_1,
                                     TIMER_B_TBIE_INTERRUPT_DISABLE,
                                     TIMER_B_DO_CLEAR
                                     );

    //Initiaze compare mode
    TIMER_B_clearCaptureCompareInterruptFlag(TIMER_B0_BASE,
                                             TIMER_B_CAPTURECOMPARE_REGISTER_0);
    TIMER_B_initCompare(TIMER_B0_BASE,
                        TIMER_B_CAPTURECOMPARE_REGISTER_0,
                        TIMER_B_CAPTURECOMPARE_INTERRUPT_DISABLE,
                        TIMER_B_OUTPUTMODE_SET_RESET,
                        COMPARE_VALUE
                        );

    TIMER_B_startCounter( TIMER_B0_BASE,
    					  TIMER_B_UP_MODE
                          );
}

void initADC(void)
{
    GPIO_setAsPeripheralModuleFunctionInputPin(ANALOG_AX_PORT,
    										   ANALOG_AX_PIN
                                               );
    GPIO_setAsPeripheralModuleFunctionInputPin(ANALOG_AY_PORT,
    										   ANALOG_AY_PIN
                                               );
    GPIO_setAsPeripheralModuleFunctionInputPin(ANALOG_AZ_PORT,
    										   ANALOG_AZ_PIN
                                               );
    GPIO_setAsPeripheralModuleFunctionInputPin(ANALOG_AR_PORT,
    										   ANALOG_AR_PIN
                                               );
    GPIO_setAsPeripheralModuleFunctionInputPin(ANALOG_BX_PORT,
    										   ANALOG_BX_PIN
                                               );
    GPIO_setAsPeripheralModuleFunctionInputPin(ANALOG_BY_PORT,
    										   ANALOG_BY_PIN
                                               );
    GPIO_setAsPeripheralModuleFunctionInputPin(ANALOG_BZ_PORT,
    										   ANALOG_BZ_PIN
                                               );
    GPIO_setAsPeripheralModuleFunctionInputPin(ANALOG_BR_PORT,
    										   ANALOG_BR_PIN
                                               );

    //Initialize the ADC12_A Module
    /*
     * Base address of ADC12_A Module
     * Use internal ADC12_A bit as sample/hold signal to start conversion
     * USE MODOSC 5MHZ Digital Oscillator as clock source
     * Use default clock divider of 1
     */
    ADC12_A_init(ADC12_A_BASE,
    			 ADC12_A_SAMPLEHOLDSOURCE_2,
                 ADC12_A_CLOCKSOURCE_ADC12OSC,
                 ADC12_A_CLOCKDIVIDER_1
                 );

    ADC12_A_enable(ADC12_A_BASE);


    ADC12_A_setupSamplingTimer(ADC12_A_BASE,
                               ADC12_A_CYCLEHOLD_256_CYCLES,
                               ADC12_A_CYCLEHOLD_256_CYCLES,
                               ADC12_A_MULTIPLESAMPLESENABLE);

    ADC12_A_memoryConfigure(ADC12_A_BASE,
                            ADC12_A_MEMORY_0,
                            ADC12_A_INPUT_A0,
                            ADC12_A_VREFPOS_AVCC,
                            ADC12_A_VREFNEG_AVSS,
                            ADC12_A_NOTENDOFSEQUENCE);

    ADC12_A_memoryConfigure(ADC12_A_BASE,
                            ADC12_A_MEMORY_1,
                            ADC12_A_INPUT_A1,
                            ADC12_A_VREFPOS_AVCC,
                            ADC12_A_VREFNEG_AVSS,
                            ADC12_A_NOTENDOFSEQUENCE);

    ADC12_A_memoryConfigure(ADC12_A_BASE,
                            ADC12_A_MEMORY_2,
                            ADC12_A_INPUT_A2,
                            ADC12_A_VREFPOS_AVCC,
                            ADC12_A_VREFNEG_AVSS,
                            ADC12_A_NOTENDOFSEQUENCE);

    ADC12_A_memoryConfigure(ADC12_A_BASE,
                            ADC12_A_MEMORY_3,
                            ADC12_A_INPUT_A3,
                            ADC12_A_VREFPOS_AVCC,
                            ADC12_A_VREFNEG_AVSS,
                            ADC12_A_NOTENDOFSEQUENCE);

    ADC12_A_memoryConfigure(ADC12_A_BASE,
                            ADC12_A_MEMORY_4,
                            ADC12_A_INPUT_A4,
                            ADC12_A_VREFPOS_AVCC,
                            ADC12_A_VREFNEG_AVSS,
                            ADC12_A_NOTENDOFSEQUENCE);

    ADC12_A_memoryConfigure(ADC12_A_BASE,
                            ADC12_A_MEMORY_5,
                            ADC12_A_INPUT_A5,
                            ADC12_A_VREFPOS_AVCC,
                            ADC12_A_VREFNEG_AVSS,
                            ADC12_A_NOTENDOFSEQUENCE);

    ADC12_A_memoryConfigure(ADC12_A_BASE,
                            ADC12_A_MEMORY_6,
                            ADC12_A_INPUT_A6,
                            ADC12_A_VREFPOS_AVCC,
                            ADC12_A_VREFNEG_AVSS,
                            ADC12_A_NOTENDOFSEQUENCE);

    ADC12_A_memoryConfigure(ADC12_A_BASE,
                            ADC12_A_MEMORY_12,
                            ADC12_A_INPUT_A12,
                            ADC12_A_VREFPOS_AVCC,
                            ADC12_A_VREFNEG_AVSS,
                            ADC12_A_ENDOFSEQUENCE);

    ADC12_A_clearInterrupt(ADC12_A_BASE,
                           ADC12IFG12);
    ADC12_A_enableInterrupt(ADC12_A_BASE,
                            ADC12IE12);

    ADC12_A_startConversion(ADC12_A_BASE,
                            ADC12_A_MEMORY_0,
                            ADC12_A_REPEATED_SEQOFCHANNELS);
}

// Timer B0 interrupt service routine
//#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
//#pragma vector=TIMERB0_VECTOR
//__interrupt
//#elif defined(__GNUC__)
//__attribute__((interrupt(TIMERB0_VECTOR)))
//#endif
//void TIMERB0_ISR(void)
//{
//    ADC12_A_startConversion(ADC12_A_BASE,
//                            ADC12_A_MEMORY_0,
//                            ADC12_A_SEQOFCHANNELS);
//}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=ADC12_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(ADC12_VECTOR)))
#endif
void ADC12ISR(void)
{
    switch (__even_in_range(ADC12IV, 34)) {
    case  0: break;         //Vector  0:  No interrupt
    case  2: break;         //Vector  2:  ADC overflow
    case  4: break;         //Vector  4:  ADC timing overflow
    case  6: break;         //Vector  6:  ADC12IFG0
    case  8: break;         //Vector  8:  ADC12IFG1
    case 10: break;         //Vector 10:  ADC12IFG2
    case 12: break;         //Vector 12:  ADC12IFG3
    case 14: break;         //Vector 14:  ADC12IFG4
    case 16: break;         //Vector 16:  ADC12IFG5
    case 18: break;         //Vector 18:  ADC12IFG6
    case 20: break;         //Vector 20:  ADC12IFG7
    case 22: break;         //Vector 22:  ADC12IFG8
    case 24: break;         //Vector 24:  ADC12IFG9
    case 26: break;         //Vector 26:  ADC12IFG10
    case 28: break;         //Vector 28:  ADC12IFG11
    case 30:                //Vector 30:  ADC12IFG12
		gamepadReportTx.ax =  Convert8Bit(ADC12_A_getResults(ADC12_A_BASE, ADC12_A_MEMORY_12));
		gamepadReportTx.ay =  Convert8Bit(ADC12_A_getResults(ADC12_A_BASE, ADC12_A_MEMORY_4));
		gamepadReportTx.az =  Convert8Bit(ADC12_A_getResults(ADC12_A_BASE, ADC12_A_MEMORY_3));
		gamepadReportTx.ar =  Convert8Bit(ADC12_A_getResults(ADC12_A_BASE, ADC12_A_MEMORY_2));
		gamepadReportTx.bx =  Convert8Bit(ADC12_A_getResults(ADC12_A_BASE, ADC12_A_MEMORY_1));
		gamepadReportTx.by =  Convert8Bit(ADC12_A_getResults(ADC12_A_BASE, ADC12_A_MEMORY_0));
		gamepadReportTx.bz =  Convert8Bit(ADC12_A_getResults(ADC12_A_BASE, ADC12_A_MEMORY_6));
		gamepadReportTx.br =  Convert8Bit(ADC12_A_getResults(ADC12_A_BASE, ADC12_A_MEMORY_5));
		adcUpdateAvailable = TRUE;
    	__bic_SR_register_on_exit(LPM3_bits);   //Wake main from LPMx
		break;
    case 32: break;         //Vector 32:  ADC12IFG13
    case 34: break;         //Vector 34:  ADC12IFG14
    default: break;
    }
}

//Released_Version_4_10_02
