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
// rtc_c.c - Driver for the rtc_c Module.
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup rtc_c_api
//! @{
//
//*****************************************************************************

#include "inc/hw_regaccess.h"
#include "inc/hw_memmap.h"

#if defined(__MSP430_HAS_RTC_C__) || defined(__MSP430_HAS_RTC_CE__)
#include "rtc_c.h"

#include <assert.h>

//*****************************************************************************
//
//! \brief Starts the RTC.
//!
//! This function clears the RTC main hold bit to allow the RTC to function.
//!
//! \param baseAddress is the base address of the RTC_C module.
//!
//! \return None
//
//*****************************************************************************
void RTC_C_startClock(uint32_t baseAddress)
{
        HWREG8(baseAddress + OFS_RTCCTL0_H) = RTCKEY_H;
        HWREG8(baseAddress + OFS_RTCCTL13_L) &= ~(RTCHOLD);
        HWREG8(baseAddress + OFS_RTCCTL0_H) = 0x00;
}

//*****************************************************************************
//
//! \brief Holds the RTC.
//!
//! This function sets the RTC main hold bit to disable RTC functionality.
//!
//! \param baseAddress is the base address of the RTC_C module.
//!
//! \return None
//
//*****************************************************************************
void RTC_C_holdClock(uint32_t baseAddress)
{
        HWREG8(baseAddress + OFS_RTCCTL0_H) = RTCKEY_H;
        HWREG8(baseAddress + OFS_RTCCTL13_L) |= RTCHOLD;
        HWREG8(baseAddress + OFS_RTCCTL0_H) = 0x00;
}

//*****************************************************************************
//
//! \brief Allows and Sets the frequency output to RTCCLK pin for calibration
//! measurement.
//!
//! This function sets a frequency to measure at the RTCCLK output pin. After
//! testing the set frequency, the calibration could be set accordingly.
//!
//! \param baseAddress is the base address of the RTC_C module.
//! \param frequencySelect is the frequency output to RTCCLK.
//!        Valid values are:
//!        - \b RTC_C_CALIBRATIONFREQ_OFF [Default] - turn off calibration
//!           output
//!        - \b RTC_C_CALIBRATIONFREQ_512HZ - output signal at 512Hz for
//!           calibration
//!        - \b RTC_C_CALIBRATIONFREQ_256HZ - output signal at 256Hz for
//!           calibration
//!        - \b RTC_C_CALIBRATIONFREQ_1HZ - output signal at 1Hz for
//!           calibration
//!        \n Modified bits are \b RTCCALF of \b RTCCTL3 register.
//!
//! \return None
//
//*****************************************************************************
void RTC_C_setCalibrationFrequency(uint32_t baseAddress,
                                   uint16_t frequencySelect)
{
        HWREG8(baseAddress + OFS_RTCCTL0_H) = RTCKEY_H;
        HWREG16(baseAddress + OFS_RTCCTL13) &= ~(RTCCALF_3);
        HWREG16(baseAddress + OFS_RTCCTL13) |= frequencySelect;
        HWREG8(baseAddress + OFS_RTCCTL0_H) = 0x00;
}

//*****************************************************************************
//
//! \brief Sets the specified calibration for the RTC.
//!
//! This function sets the calibration offset to make the RTC as accurate as
//! possible. The offsetDirection can be either +4-ppm or -2-ppm, and the
//! offsetValue should be from 1-63 and is multiplied by the direction setting
//! (i.e. +4-ppm * 8 (offsetValue) = +32-ppm).
//!
//! \param baseAddress is the base address of the RTC_C module.
//! \param offsetDirection is the direction that the calibration offset will
//!        go.
//!        Valid values are:
//!        - \b RTC_C_CALIBRATION_DOWN1PPM - calibrate at steps of -1
//!        - \b RTC_C_CALIBRATION_UP1PPM - calibrate at steps of +1
//!        \n Modified bits are \b RTC0CALS of \b RTC0CAL register.
//! \param offsetValue is the value that the offset will be a factor of; a
//!        valid value is any integer from 1-240.
//!        \n Modified bits are \b RTC0CALx of \b RTC0CAL register.
//!
//! \return None
//
//*****************************************************************************
void RTC_C_setCalibrationData(uint32_t baseAddress,
                              uint8_t offsetDirection,
                              uint8_t offsetValue)
{
        HWREG8(baseAddress + OFS_RTCCTL0_H) = RTCKEY_H;
        HWREG16(baseAddress + OFS_RTCOCAL) = offsetValue + offsetDirection;
        HWREG8(baseAddress + OFS_RTCCTL0_H) = 0x00;
}

//*****************************************************************************
//
//! \brief Initializes the settings to operate the RTC in Counter mode.
//!
//! This function initializes the Counter mode of the RTC_C. Setting the clock
//! source and counter size will allow an interrupt from the RTCTEVIFG once an
//! overflow to the counter register occurs.
//!
//! \param baseAddress is the base address of the RTC_C module.
//! \param clockSelect is the selected clock for the counter mode to use.
//!        Valid values are:
//!        - \b RTC_C_CLOCKSELECT_32KHZ_OSC
//!        - \b RTC_C_CLOCKSELECT_RT1PS
//!        \n Modified bits are \b RTCSSEL of \b RTCCTL1 register.
//! \param counterSizeSelect is the size of the counter.
//!        Valid values are:
//!        - \b RTC_C_COUNTERSIZE_8BIT [Default]
//!        - \b RTC_C_COUNTERSIZE_16BIT
//!        - \b RTC_C_COUNTERSIZE_24BIT
//!        - \b RTC_C_COUNTERSIZE_32BIT
//!        \n Modified bits are \b RTCTEV of \b RTCCTL1 register.
//!
//! \return None
//
//*****************************************************************************
void RTC_C_initCounter(uint32_t baseAddress,
                       uint16_t clockSelect,
                       uint16_t counterSizeSelect)
{
        HWREG8(baseAddress + OFS_RTCCTL13) |= RTCHOLD;
        HWREG8(baseAddress + OFS_RTCCTL13) &= ~(RTCMODE);

        HWREG16(baseAddress + OFS_RTCCTL13) &= ~(RTCSSEL_3 | RTCTEV_3 );
        HWREG16(baseAddress + OFS_RTCCTL13) |= clockSelect + counterSizeSelect;
}

//*****************************************************************************
//
//! \brief Sets the specified temperature compensation for the RTC.
//!
//! This function sets the calibration offset to make the RTC as accurate as
//! possible. The offsetDirection can be either +1-ppm or -1-ppm, and the
//! offsetValue should be from 1-240 and is multiplied by the direction setting
//! (i.e. +1-ppm * 8 (offsetValue) = +8-ppm).
//!
//! \param baseAddress is the base address of the RTC_C module.
//! \param offsetDirection is the direction that the calibration offset wil go
//!        Valid values are:
//!        - \b RTC_C_COMPENSATION_DOWN1PPM
//!        - \b RTC_C_COMPENSATION_UP1PPM
//!        \n Modified bits are \b RTCTCMPS of \b RTCTCMP register.
//! \param offsetValue is the value that the offset will be a factor of; a
//!        valid value is any integer from 1-240.
//!        \n Modified bits are \b RTCTCMPx of \b RTCTCMP register.
//!
//! \return STATUS_SUCCESS or STATUS_FAILURE of setting the temperature
//!         compensation
//
//*****************************************************************************
bool RTC_C_setTemperatureCompensation(uint32_t baseAddress,
                                      uint8_t offsetDirection,
                                      uint8_t offsetValue)
{

        while (!(HWREG8(baseAddress + OFS_RTCTCMP_H) & RTCTCRDY_H)) ;

        HWREG16(baseAddress + OFS_RTCTCMP) = offsetValue + offsetDirection;

        if (HWREG8(baseAddress + OFS_RTCTCMP_H) & RTCTCOK_H)
                return STATUS_SUCCESS;
        else
                return STATUS_FAIL;
}

//*****************************************************************************
//
//! \brief Deprecated - Initializes the settings to operate the RTC in calendar
//! mode
//!
//! This function initializes the Calendar mode of the RTC module.
//!
//! \param baseAddress is the base address of the RTC_C module.
//! \param CalendarTime is the structure containing the values for the Calendar
//!        to be initialized to. Valid values should be of type Calendar and
//!        should contain the following members and corresponding values: \b
//!        Seconds between 0-59 \b Minutes between 0-59 \b Hours between 0-24
//!        \b DayOfWeek between 0-6 \b DayOfMonth between 0-31 \b Year between
//!        0-4095 NOTE: Values beyond the ones specified may result in erratic
//!        behavior.
//! \param formatSelect is the format for the Calendar registers to use.
//!        Valid values are:
//!        - \b RTC_C_FORMAT_BINARY [Default]
//!        - \b RTC_C_FORMAT_BCD
//!        \n Modified bits are \b RTCBCD of \b RTCCTL1 register.
//!
//! \return None
//
//*****************************************************************************
void RTC_C_calendarInit(uint32_t baseAddress,
                        Calendar CalendarTime,
                        uint16_t formatSelect)
{

        HWREG8(baseAddress + OFS_RTCCTL0_H) = RTCKEY_H;

        HWREG8(baseAddress + OFS_RTCCTL13_L) |= RTCHOLD;

        HWREG16(baseAddress + OFS_RTCCTL13_L) &= ~(RTCBCD);
        HWREG16(baseAddress + OFS_RTCCTL13_L) |= formatSelect;

        HWREG8(baseAddress + OFS_RTCTIM0_L) = CalendarTime.Seconds;
        HWREG8(baseAddress + OFS_RTCTIM0_H) = CalendarTime.Minutes;
        HWREG8(baseAddress + OFS_RTCTIM1_L) = CalendarTime.Hours;
        HWREG8(baseAddress + OFS_RTCTIM1_H) = CalendarTime.DayOfWeek;
        HWREG8(baseAddress + OFS_RTCDATE_L) = CalendarTime.DayOfMonth;
        HWREG8(baseAddress + OFS_RTCDATE_H) = CalendarTime.Month;
        HWREG16(baseAddress + OFS_RTCYEAR) = CalendarTime.Year;

        HWREG8(baseAddress + OFS_RTCCTL0_H) = 0x00;
} //*****************************************************************************
//
//! \brief Initializes the settings to operate the RTC in calendar mode
//!
//! This function initializes the Calendar mode of the RTC module.
//!
//! \param baseAddress is the base address of the RTC_C module.
//! \param CalendarTime is the pointer to the structure containing the values
//!        for the Calendar to be initialized to. Valid values should be of
//!        type pointer to Calendar and should contain the following members
//!        and corresponding values: \b Seconds between 0-59 \b Minutes between
//!        0-59 \b Hours between 0-24 \b DayOfWeek between 0-6 \b DayOfMonth
//!        between 0-31 \b Year between 0-4095 NOTE: Values beyond the ones
//!        specified may result in erratic behavior.
//! \param formatSelect is the format for the Calendar registers to use.
//!        Valid values are:
//!        - \b RTC_C_FORMAT_BINARY [Default]
//!        - \b RTC_C_FORMAT_BCD
//!        \n Modified bits are \b RTCBCD of \b RTCCTL1 register.
//!
//! \return None
//
//*****************************************************************************
void RTC_C_initCalendar(uint32_t baseAddress,
                        Calendar *CalendarTime,
                        uint16_t formatSelect)
{

        HWREG8(baseAddress + OFS_RTCCTL0_H) = RTCKEY_H;

        HWREG8(baseAddress + OFS_RTCCTL13_L) |= RTCHOLD;

        HWREG16(baseAddress + OFS_RTCCTL13_L) &= ~(RTCBCD);
        HWREG16(baseAddress + OFS_RTCCTL13_L) |= formatSelect;

        HWREG8(baseAddress + OFS_RTCTIM0_L) = CalendarTime->Seconds;
        HWREG8(baseAddress + OFS_RTCTIM0_H) = CalendarTime->Minutes;
        HWREG8(baseAddress + OFS_RTCTIM1_L) = CalendarTime->Hours;
        HWREG8(baseAddress + OFS_RTCTIM1_H) = CalendarTime->DayOfWeek;
        HWREG8(baseAddress + OFS_RTCDATE_L) = CalendarTime->DayOfMonth;
        HWREG8(baseAddress + OFS_RTCDATE_H) = CalendarTime->Month;
        HWREG16(baseAddress + OFS_RTCYEAR) = CalendarTime->Year;

        HWREG8(baseAddress + OFS_RTCCTL0_H) = 0x00;
}

//*****************************************************************************
//
//! \brief Returns the Calendar Time stored in the Calendar registers of the
//! RTC.
//!
//! This function returns the current Calendar time in the form of a Calendar
//! structure.
//!
//! \param baseAddress is the base address of the RTC_C module.
//!
//! \return A Calendar structure containing the current time.
//
//*****************************************************************************
Calendar RTC_C_getCalendarTime(uint32_t baseAddress)
{
        Calendar tempCal;

        while ( !(HWREG8(baseAddress + OFS_RTCCTL13_L) & RTCRDY) ) ;

        tempCal.Seconds    = HWREG8(baseAddress + OFS_RTCTIM0_L);
        tempCal.Minutes    = HWREG8(baseAddress + OFS_RTCTIM0_H);
        tempCal.Hours      = HWREG8(baseAddress + OFS_RTCTIM1_L);
        tempCal.DayOfWeek  = HWREG8(baseAddress + OFS_RTCTIM1_H);
        tempCal.DayOfMonth = HWREG8(baseAddress + OFS_RTCDATE_L);
        tempCal.Month      = HWREG8(baseAddress + OFS_RTCDATE_H);
        tempCal.Year       = HWREG16(baseAddress + OFS_RTCYEAR);

        return tempCal;
}

//*****************************************************************************
//
//! \brief Sets and Enables the desired Calendar Alarm settings.
//!
//! This function sets a Calendar interrupt condition to assert the RTCAIFG
//! interrupt flag. The condition is a logical and of all of the parameters.
//! For example if the minutes and hours alarm is set, then the interrupt will
//! only assert when the minutes AND the hours change to the specified setting.
//! Use the RTC_C_ALARM_OFF for any alarm settings that should not be apart of
//! the alarm condition.
//!
//! \param baseAddress is the base address of the RTC_C module.
//! \param minutesAlarm is the alarm condition for the minutes.
//!        Valid values are:
//!        - \b RTC_C_ALARMCONDITION_OFF [Default]
//!        - \b An integer between 0-59
//! \param hoursAlarm is the alarm condition for the hours.
//!        Valid values are:
//!        - \b RTC_C_ALARMCONDITION_OFF [Default]
//!        - \b An integer between 0-24
//! \param dayOfWeekAlarm is the alarm condition for the day of week.
//!        Valid values are:
//!        - \b RTC_C_ALARMCONDITION_OFF [Default]
//!        - \b An integer between 0-6
//! \param dayOfMonthAlarm is the alarm condition for the day of the month.
//!        Valid values are:
//!        - \b RTC_C_ALARMCONDITION_OFF [Default]
//!        - \b An integer between 0-31
//!
//! \return None
//
//*****************************************************************************
void RTC_C_setCalendarAlarm(uint32_t baseAddress,
                            uint8_t minutesAlarm,
                            uint8_t hoursAlarm,
                            uint8_t dayOfWeekAlarm,
                            uint8_t dayOfMonthAlarm)
{
        //Each of these is XORed with 0x80 to turn on if an integer is passed,
        //or turn OFF if RTC_C_ALARM_OFF (0x80) is passed.
        HWREG8(baseAddress + OFS_RTCAMINHR_L) = (minutesAlarm ^ 0x80);
        HWREG8(baseAddress + OFS_RTCAMINHR_H) = (hoursAlarm ^ 0x80);
        HWREG8(baseAddress + OFS_RTCADOWDAY_L) = (dayOfWeekAlarm ^ 0x80);
        HWREG8(baseAddress + OFS_RTCADOWDAY_H) = (dayOfMonthAlarm ^ 0x80);
}

//*****************************************************************************
//
//! \brief Sets a single specified Calendar interrupt condition
//!
//! This function sets a specified event to assert the RTCTEVIFG interrupt.
//! This interrupt is independent from the Calendar alarm interrupt.
//!
//! \param baseAddress is the base address of the RTC_C module.
//! \param eventSelect is the condition selected.
//!        Valid values are:
//!        - \b RTC_C_CALENDAREVENT_MINUTECHANGE - assert interrupt on every
//!           minute
//!        - \b RTC_C_CALENDAREVENT_HOURCHANGE - assert interrupt on every hour
//!        - \b RTC_C_CALENDAREVENT_NOON - assert interrupt when hour is 12
//!        - \b RTC_C_CALENDAREVENT_MIDNIGHT - assert interrupt when hour is 0
//!        \n Modified bits are \b RTCTEV of \b RTCCTL register.
//!
//! \return None
//
//*****************************************************************************
void RTC_C_setCalendarEvent(uint32_t baseAddress,
                            uint16_t eventSelect)
{
        HWREG8(baseAddress + OFS_RTCCTL0_H) = RTCKEY_H;
        HWREG8(baseAddress + OFS_RTCCTL13_L) &= ~(RTCTEV_3); //Reset bits
        HWREG8(baseAddress + OFS_RTCCTL13_L) |= eventSelect;
        HWREG8(baseAddress + OFS_RTCCTL0_H) = 0x00;
}

//*****************************************************************************
//
//! \brief Returns the value of the Counter register.
//!
//! This function returns the value of the counter register for the RTC_C
//! module. It will return the 32-bit value no matter the size set during
//! initialization. The RTC should be held before trying to use this function.
//!
//! \param baseAddress is the base address of the RTC_C module.
//!
//! \return The raw value of the full 32-bit Counter Register.
//
//*****************************************************************************
uint32_t RTC_C_getCounterValue(uint32_t baseAddress)
{
        if ( (HWREG8(baseAddress + OFS_RTCCTL13) & RTCHOLD)
             || (HWREG8(baseAddress + OFS_RTCPS1CTL) & RT1PSHOLD) )
                return 0;

        uint32_t counterValue_L = HWREG16(baseAddress + OFS_RTCTIM0);
        uint32_t counterValue_H = HWREG16(baseAddress + OFS_RTCTIM1);
        return (counterValue_H << 16) + counterValue_L;
}

//*****************************************************************************
//
//! \brief Sets the value of the Counter register
//!
//! This function sets the counter register of the RTC_C module.
//!
//! \param baseAddress is the base address of the RTC_C module.
//! \param counterValue is the value to set the Counter register to; a valid
//!        value may be any 32-bit integer.
//!
//! \return None
//
//*****************************************************************************
void RTC_C_setCounterValue(uint32_t baseAddress,
                           uint32_t counterValue)
{
        uint16_t mode = HWREG16(baseAddress + OFS_RTCCTL13) & RTCTEV_3;

        if (mode == RTC_C_COUNTERSIZE_8BIT && counterValue > 0xF)
                counterValue = 0xF;
        else if (mode == RTC_C_COUNTERSIZE_16BIT && counterValue > 0xFF)
                counterValue = 0xFF;
        else if (mode == RTC_C_COUNTERSIZE_24BIT && counterValue > 0xFFFFFF)
                counterValue = 0xFFFFFF;

        HWREG16(baseAddress + OFS_RTCTIM0) = counterValue;
        HWREG16(baseAddress + OFS_RTCTIM1) = ( counterValue >> 16 );
}

//*****************************************************************************
//
//! \brief Initializes the Prescaler for Counter mode.
//!
//! This function initializes the selected prescaler for the counter mode in
//! the RTC_C module. If the RTC is initialized in Calendar mode, then these
//! are automatically initialized. The Prescalers can be used to divide a clock
//! source additionally before it gets to the main RTC clock.
//!
//! \param baseAddress is the base address of the RTC_C module.
//! \param prescaleSelect is the prescaler to initialize.
//!        Valid values are:
//!        - \b RTC_C_PRESCALE_0
//!        - \b RTC_C_PRESCALE_1
//! \param prescaleClockSelect is the clock to drive the selected prescaler.
//!        Valid values are:
//!        - \b RTC_C_PSCLOCKSELECT_ACLK
//!        - \b RTC_C_PSCLOCKSELECT_SMCLK
//!        - \b RTC_C_PSCLOCKSELECT_RT0PS - use Prescaler 0 as source to
//!           Prescaler 1 (May only be used if prescaleSelect is
//!           RTC_C_PRESCALE_1)
//!        \n Modified bits are \b RTxSSEL of \b RTCPSxCTL register.
//! \param prescaleDivider is the divider for the selected clock source.
//!        Valid values are:
//!        - \b RTC_C_PSDIVIDER_2 [Default]
//!        - \b RTC_C_PSDIVIDER_4
//!        - \b RTC_C_PSDIVIDER_8
//!        - \b RTC_C_PSDIVIDER_16
//!        - \b RTC_C_PSDIVIDER_32
//!        - \b RTC_C_PSDIVIDER_64
//!        - \b RTC_C_PSDIVIDER_128
//!        - \b RTC_C_PSDIVIDER_256
//!        \n Modified bits are \b RTxPSDIV of \b RTCPSxCTL register.
//!
//! \return None
//
//*****************************************************************************
void RTC_C_counterPrescaleInit(uint32_t baseAddress,
                               uint8_t prescaleSelect,
                               uint16_t prescaleClockSelect,
                               uint16_t prescaleDivider)
{
        //Reset bits and set clock select
        HWREG16(baseAddress + OFS_RTCPS0CTL + prescaleSelect) =
                prescaleClockSelect + prescaleDivider;
}

//*****************************************************************************
//
//! \brief Holds the selected Prescaler.
//!
//! This function holds the prescale counter from continuing. This will only
//! work in counter mode, in Calendar mode, the RTC_C_holdClock() must be used.
//! In counter mode, if using both prescalers in conjunction with the main RTC
//! counter, then stopping RT0PS will stop RT1PS, but stopping RT1PS will not
//! stop RT0PS.
//!
//! \param baseAddress is the base address of the RTC_C module.
//! \param prescaleSelect is the prescaler to hold.
//!        Valid values are:
//!        - \b RTC_C_PRESCALE_0
//!        - \b RTC_C_PRESCALE_1
//!
//! \return None
//
//*****************************************************************************
void RTC_C_counterPrescaleHold(uint32_t baseAddress,
                               uint8_t prescaleSelect)
{
        HWREG8(baseAddress + OFS_RTCPS0CTL_H + prescaleSelect) |= RT0PSHOLD_H;
}

//*****************************************************************************
//
//! \brief Starts the selected Prescaler.
//!
//! This function starts the selected prescale counter. This function will only
//! work if the RTC is in counter mode.
//!
//! \param baseAddress is the base address of the RTC_C module.
//! \param prescaleSelect is the prescaler to start.
//!        Valid values are:
//!        - \b RTC_C_PRESCALE_0
//!        - \b RTC_C_PRESCALE_1
//!
//! \return None
//
//*****************************************************************************
void RTC_C_counterPrescaleStart(uint32_t baseAddress,
                                uint8_t prescaleSelect)
{
        HWREG8(baseAddress + OFS_RTCPS0CTL_H + prescaleSelect) &= ~(RT0PSHOLD_H);
}

//*****************************************************************************
//
//! \brief Sets up an interrupt condition for the selected Prescaler.
//!
//! This function sets the condition for an interrupt to assert based on the
//! individual prescalers.
//!
//! \param baseAddress is the base address of the RTC_C module.
//! \param prescaleSelect is the prescaler to define an interrupt for.
//!        Valid values are:
//!        - \b RTC_C_PRESCALE_0
//!        - \b RTC_C_PRESCALE_1
//! \param prescaleEventDivider is a divider to specify when an interrupt can
//!        occur based on the clock source of the selected prescaler. (Does not
//!        affect timer of the selected prescaler).
//!        Valid values are:
//!        - \b RTC_C_PSEVENTDIVIDER_2 [Default]
//!        - \b RTC_C_PSEVENTDIVIDER_4
//!        - \b RTC_C_PSEVENTDIVIDER_8
//!        - \b RTC_C_PSEVENTDIVIDER_16
//!        - \b RTC_C_PSEVENTDIVIDER_32
//!        - \b RTC_C_PSEVENTDIVIDER_64
//!        - \b RTC_C_PSEVENTDIVIDER_128
//!        - \b RTC_C_PSEVENTDIVIDER_256
//!        \n Modified bits are \b RTxIP of \b RTCPSxCTL register.
//!
//! \return None
//
//*****************************************************************************
void RTC_C_definePrescaleEvent(uint32_t baseAddress,
                               uint8_t prescaleSelect,
                               uint8_t prescaleEventDivider)
{
        HWREG8(baseAddress + OFS_RTCPS0CTL_L + prescaleSelect) &= ~(RT0IP_7);
        HWREG8(baseAddress + OFS_RTCPS0CTL_L +
               prescaleSelect) |= prescaleEventDivider;
}

//*****************************************************************************
//
//! \brief Returns the selected prescaler value.
//!
//! This function returns the value of the selected prescale counter register.
//! The counter should be held before reading. If in counter mode, the
//! individual prescaler can be held, while in Calendar mode the whole RTC must
//! be held.
//!
//! \param baseAddress is the base address of the RTC_C module.
//! \param prescaleSelect is the prescaler to obtain the value of.
//!        Valid values are:
//!        - \b RTC_C_PRESCALE_0
//!        - \b RTC_C_PRESCALE_1
//!
//! \return The value of the specified prescaler count register
//
//*****************************************************************************
uint8_t RTC_C_getPrescaleValue(uint32_t baseAddress,
                               uint8_t prescaleSelect)
{
        if (RTC_C_PRESCALE_0 == prescaleSelect)
                return HWREG8(baseAddress + OFS_RTCPS_L);
        else if (RTC_C_PRESCALE_1 == prescaleSelect)
                return HWREG8(baseAddress + OFS_RTCPS_H);
        else
                return 0;
}

//*****************************************************************************
//
//! \brief Sets the selected Prescaler value.
//!
//! This function sets the prescale counter value. Before setting the prescale
//! counter, it should be held.
//!
//! \param baseAddress is the base address of the RTC_C module.
//! \param prescaleSelect is the prescaler to set the value for.
//!        Valid values are:
//!        - \b RTC_C_PRESCALE_0
//!        - \b RTC_C_PRESCALE_1
//! \param prescaleCounterValue is the specified value to set the prescaler to.
//!        Valid values are any integer between 0-255
//!        \n Modified bits are \b RTxPS of \b RTxPS register.
//!
//! \return None
//
//*****************************************************************************
void RTC_C_setPrescaleValue(uint32_t baseAddress,
                            uint8_t prescaleSelect,
                            uint8_t prescaleCounterValue)
{
        HWREG8(baseAddress + OFS_RTCCTL0_H) = RTCKEY_H;
        if (RTC_C_PRESCALE_0 == prescaleSelect)
                HWREG8(baseAddress + OFS_RTCPS_L) = prescaleCounterValue;
        else if (RTC_C_PRESCALE_1 == prescaleSelect)
                HWREG8(baseAddress + OFS_RTCPS_H) = prescaleCounterValue;
        HWREG8(baseAddress + OFS_RTCCTL0_H) = 0x00;
}

//*****************************************************************************
//
//! \brief Enables selected RTC interrupt sources.
//!
//! This function enables the selected RTC interrupt source.  Only the sources
//! that are enabled can be reflected to the processor interrupt; disabled
//! sources have no effect on the processor. Does not clear interrupt flags.
//!
//! \param baseAddress is the base address of the RTC_C module.
//! \param interruptMask is a bit mask of the interrupts to enable.
//!        Mask value is the logical OR of any of the following:
//!        - \b RTC_C_TIME_EVENT_INTERRUPT - asserts when counter overflows in
//!           counter mode or when Calendar event condition defined by
//!           defineCalendarEvent() is met.
//!        - \b RTC_C_CLOCK_ALARM_INTERRUPT - asserts when alarm condition in
//!           Calendar mode is met.
//!        - \b RTC_C_CLOCK_READ_READY_INTERRUPT - asserts when Calendar
//!           registers are settled.
//!        - \b RTC_C_PRESCALE_TIMER0_INTERRUPT - asserts when Prescaler 0
//!           event condition is met.
//!        - \b RTC_C_PRESCALE_TIMER1_INTERRUPT - asserts when Prescaler 1
//!           event condition is met.
//!        - \b RTC_C_OSCILLATOR_FAULT_INTERRUPT - asserts if there is a
//!           problem with the 32kHz oscillator, while the RTC is running.
//!
//! \return None
//
//*****************************************************************************
void RTC_C_enableInterrupt(uint32_t baseAddress,
                           uint8_t interruptMask)
{
        if ( interruptMask & (RTCOFIE + RTCTEVIE + RTCAIE + RTCRDYIE) ) {
                HWREG8(baseAddress + OFS_RTCCTL0_H) = RTCKEY_H;
                HWREG8(baseAddress + OFS_RTCCTL0_L) |=
                        (interruptMask & (RTCOFIE + RTCTEVIE + RTCAIE + RTCRDYIE));
                HWREG8(baseAddress + OFS_RTCCTL0_H) = 0x00;
        }

        if (interruptMask & RTC_C_PRESCALE_TIMER0_INTERRUPT)
                HWREG8(baseAddress + OFS_RTCPS0CTL_L) |= RT0PSIE;

        if (interruptMask & RTC_C_PRESCALE_TIMER1_INTERRUPT)
                HWREG8(baseAddress + OFS_RTCPS1CTL_L) |= RT1PSIE;
}

//*****************************************************************************
//
//! \brief Disables selected RTC interrupt sources.
//!
//! This function disables the selected RTC interrupt source. Only the sources
//! that are enabled can be reflected to the processor interrupt; disabled
//! sources have no effect on the processor.
//!
//! \param baseAddress is the base address of the RTC_C module.
//! \param interruptMask is a bit mask of the interrupts to disable.
//!        Mask value is the logical OR of any of the following:
//!        - \b RTC_C_TIME_EVENT_INTERRUPT - asserts when counter overflows in
//!           counter mode or when Calendar event condition defined by
//!           defineCalendarEvent() is met.
//!        - \b RTC_C_CLOCK_ALARM_INTERRUPT - asserts when alarm condition in
//!           Calendar mode is met.
//!        - \b RTC_C_CLOCK_READ_READY_INTERRUPT - asserts when Calendar
//!           registers are settled.
//!        - \b RTC_C_PRESCALE_TIMER0_INTERRUPT - asserts when Prescaler 0
//!           event condition is met.
//!        - \b RTC_C_PRESCALE_TIMER1_INTERRUPT - asserts when Prescaler 1
//!           event condition is met.
//!        - \b RTC_C_OSCILLATOR_FAULT_INTERRUPT - asserts if there is a
//!           problem with the 32kHz oscillator, while the RTC is running.
//!
//! \return None
//
//*****************************************************************************
void RTC_C_disableInterrupt(uint32_t baseAddress,
                            uint8_t interruptMask)
{

        if ( interruptMask & (RTCOFIE + RTCTEVIE + RTCAIE + RTCRDYIE) ) {
                HWREG8(baseAddress + OFS_RTCCTL0_H) = RTCKEY_H;
                HWREG8(baseAddress + OFS_RTCCTL0_L) &=
                        ~(interruptMask & (RTCOFIE + RTCTEVIE + RTCAIE + RTCRDYIE));
                HWREG8(baseAddress + OFS_RTCCTL0_H) = 0x00;
        }

        if (interruptMask & RTC_C_PRESCALE_TIMER0_INTERRUPT)
                HWREG8(baseAddress + OFS_RTCPS0CTL_L) &= ~(RT0PSIE);

        if (interruptMask & RTC_C_PRESCALE_TIMER1_INTERRUPT)
                HWREG8(baseAddress + OFS_RTCPS1CTL_L) &= ~(RT1PSIE);
}

//*****************************************************************************
//
//! \brief Returns the status of the selected interrupts flags.
//!
//! This function returns the status of the interrupt flag for the selected
//! channel.
//!
//! \param baseAddress is the base address of the RTC_C module.
//! \param interruptFlagMask is a bit mask of the interrupt flags to return the
//!        status of.
//!        Mask value is the logical OR of any of the following:
//!        - \b RTC_C_TIME_EVENT_INTERRUPT - asserts when counter overflows in
//!           counter mode or when Calendar event condition defined by
//!           defineCalendarEvent() is met.
//!        - \b RTC_C_CLOCK_ALARM_INTERRUPT - asserts when alarm condition in
//!           Calendar mode is met.
//!        - \b RTC_C_CLOCK_READ_READY_INTERRUPT - asserts when Calendar
//!           registers are settled.
//!        - \b RTC_C_PRESCALE_TIMER0_INTERRUPT - asserts when Prescaler 0
//!           event condition is met.
//!        - \b RTC_C_PRESCALE_TIMER1_INTERRUPT - asserts when Prescaler 1
//!           event condition is met.
//!        - \b RTC_C_OSCILLATOR_FAULT_INTERRUPT - asserts if there is a
//!           problem with the 32kHz oscillator, while the RTC is running.
//!
//! \return Logical OR of any of the following:
//!         - \b RTC_C_TIME_EVENT_INTERRUPT asserts when counter overflows in
//!         counter mode or when Calendar event condition defined by
//!         defineCalendarEvent() is met.
//!         - \b RTC_C_CLOCK_ALARM_INTERRUPT asserts when alarm condition in
//!         Calendar mode is met.
//!         - \b RTC_C_CLOCK_READ_READY_INTERRUPT asserts when Calendar
//!         registers are settled.
//!         - \b RTC_C_PRESCALE_TIMER0_INTERRUPT asserts when Prescaler 0 event
//!         condition is met.
//!         - \b RTC_C_PRESCALE_TIMER1_INTERRUPT asserts when Prescaler 1 event
//!         condition is met.
//!         - \b RTC_C_OSCILLATOR_FAULT_INTERRUPT asserts if there is a problem
//!         with the 32kHz oscillator, while the RTC is running.
//!         \n indicating the status of the masked interrupts
//
//*****************************************************************************
uint8_t RTC_C_getInterruptStatus(uint32_t baseAddress,
                                 uint8_t interruptFlagMask)
{
        uint8_t tempInterruptFlagMask = 0x0000;

        tempInterruptFlagMask |= (HWREG8(baseAddress + OFS_RTCCTL0_L)
                                  & ((interruptFlagMask >> 4)
                                     & (RTCOFIFG +
                                        RTCTEVIFG +
                                        RTCAIFG +
                                        RTCRDYIFG)));

        tempInterruptFlagMask = tempInterruptFlagMask << 4;

        if (interruptFlagMask & RTC_C_PRESCALE_TIMER0_INTERRUPT)
                if ( HWREG8(baseAddress + OFS_RTCPS0CTL_L) & RT0PSIFG)
                        tempInterruptFlagMask |= RTC_C_PRESCALE_TIMER0_INTERRUPT;

        if (interruptFlagMask & RTC_C_PRESCALE_TIMER1_INTERRUPT)
                if ( HWREG8(baseAddress + OFS_RTCPS1CTL_L) & RT1PSIFG)
                        tempInterruptFlagMask |= RTC_C_PRESCALE_TIMER1_INTERRUPT;

        return tempInterruptFlagMask;
}

//*****************************************************************************
//
//! \brief Clears selected RTC interrupt flags.
//!
//! This function clears the RTC interrupt flag is cleared, so that it no
//! longer asserts.
//!
//! \param baseAddress is the base address of the RTC_C module.
//! \param interruptFlagMask is a bit mask of the interrupt flags to be
//!        cleared.
//!        Mask value is the logical OR of any of the following:
//!        - \b RTC_C_TIME_EVENT_INTERRUPT - asserts when counter overflows in
//!           counter mode or when Calendar event condition defined by
//!           defineCalendarEvent() is met.
//!        - \b RTC_C_CLOCK_ALARM_INTERRUPT - asserts when alarm condition in
//!           Calendar mode is met.
//!        - \b RTC_C_CLOCK_READ_READY_INTERRUPT - asserts when Calendar
//!           registers are settled.
//!        - \b RTC_C_PRESCALE_TIMER0_INTERRUPT - asserts when Prescaler 0
//!           event condition is met.
//!        - \b RTC_C_PRESCALE_TIMER1_INTERRUPT - asserts when Prescaler 1
//!           event condition is met.
//!        - \b RTC_C_OSCILLATOR_FAULT_INTERRUPT - asserts if there is a
//!           problem with the 32kHz oscillator, while the RTC is running.
//!
//! \return None
//
//*****************************************************************************
void RTC_C_clearInterrupt(uint32_t baseAddress,
                          uint8_t interruptFlagMask)
{

        if ( interruptFlagMask & (RTC_C_TIME_EVENT_INTERRUPT +
                                  RTC_C_CLOCK_ALARM_INTERRUPT +
                                  RTC_C_CLOCK_READ_READY_INTERRUPT +
                                  RTC_C_OSCILLATOR_FAULT_INTERRUPT) ) {
                HWREG8(baseAddress + OFS_RTCCTL0_H) = RTCKEY_H;
                HWREG8(baseAddress + OFS_RTCCTL0_L) &=
                        ~((interruptFlagMask >> 4) & (RTCOFIFG +
                                                      RTCTEVIFG +
                                                      RTCAIFG +
                                                      RTCRDYIFG));
                HWREG8(baseAddress + OFS_RTCCTL0_H) = 0x00;
        }

        if (interruptFlagMask & RTC_C_PRESCALE_TIMER0_INTERRUPT)
                HWREG8(baseAddress + OFS_RTCPS0CTL_L) &= ~(RT0PSIFG);

        if (interruptFlagMask & RTC_C_PRESCALE_TIMER1_INTERRUPT)
                HWREG8(baseAddress + OFS_RTCPS1CTL_L) &= ~(RT1PSIFG);
}

//*****************************************************************************
//
//! \brief Returns the given BCD value in Binary Format.
//!
//! This function converts BCD values to Binary format.
//!
//! \param baseAddress is the base address of the RTC_C module.
//! \param valueToConvert is the raw value in BCD format to convert to Binary.
//!        \n Modified bits are \b BCD2BIN of \b BCD2BIN register.
//!
//! \return The Binary version of the valueToConvert parameter.
//
//*****************************************************************************
uint16_t RTC_C_convertBCDToBinary(uint32_t baseAddress,
                                  uint16_t valueToConvert)
{
        HWREG16(baseAddress + OFS_BCD2BIN) = valueToConvert;
        return HWREG16(baseAddress + OFS_BCD2BIN);
}

//*****************************************************************************
//
//! \brief Returns the given Binary value in BCD Format.
//!
//! This function converts Binary values to BCD format.
//!
//! \param baseAddress is the base address of the RTC_C module.
//! \param valueToConvert is the raw value in Binary format to convert to BCD.
//!        \n Modified bits are \b BIN2BCD of \b BIN2BCD register.
//!
//! \return The BCD version of the valueToConvert parameter.
//
//*****************************************************************************
uint16_t RTC_C_convertBinaryToBCD(uint32_t baseAddress,
                                  uint16_t valueToConvert)
{
        HWREG16(baseAddress + OFS_BIN2BCD) = valueToConvert;
        return HWREG16(baseAddress + OFS_BIN2BCD);
}

#endif
//*****************************************************************************
//
//! Close the doxygen group for rtc_c_api
//! @}
//
//*****************************************************************************
//Released_Version_4_10_02
