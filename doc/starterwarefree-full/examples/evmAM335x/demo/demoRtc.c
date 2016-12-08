/**
 * \file    demoRtc.c
 *
 * \brief   This file contains RTC related functions.
 *
*/

/*
* Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
*/
/*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#include "soc_AM335x.h"
#include "interrupt.h"
#include "evmAM335x.h"
#include "consoleUtils.h"
#include "ascii.h"
#include "misc.h"
#include "rtc.h"
#include "demoRtc.h"

/*******************************************************************************
**                       INTERNAL MACRO DEFINITIONS
*******************************************************************************/
#define MASK_HOUR                        (0xFF000000u)
#define MASK_MIN                         (0x00FF0000u)
#define MASK_SEC                         (0x0000FF00u)
#define MASK_MERIDIEM                    (0x000000FFu)

#define SHIFT_HOUR                       (24u)
#define SHIFT_MIN                        (16u)
#define SHIFT_SEC                        (8u)

#define MASK_DAY                         (0xFF000000u)
#define MASK_MON                         (0x00FF0000u)
#define MASK_YEAR                        (0x0000FF00u)
#define MASK_DOTW                        (0x000000FFu)

#define SHIFT_DAY                        (24u)
#define SHIFT_MON                        (16u)
#define SHIFT_YEAR                       (8u)

#define RTC_INST_BASE                    (SOC_RTC_0_REGS)

/*******************************************************************************
**                      INTERNAL FUNCTION PROTOTYPES
*******************************************************************************/
static void RTCIsr(void);
static void RTCAlarmIsr(void);

/*******************************************************************************
**                      INTERNAL VARIABLE DEFINITIONS
*******************************************************************************/
unsigned int rtcSetFlag = FALSE;
unsigned int rtcSecUpdate = FALSE;

/*******************************************************************************
**                     FUNCTION DEFINITIONS
*******************************************************************************/
/*
** Registers RTC interrupts
*/
void RtcIntRegister(void)
{
    /* Register the ISR in the Interrupt Vector Table.*/
    IntRegister(SYS_INT_RTCINT, RTCIsr);
}

/*
** Enables RTC seconds interrupt
*/
void RtcSecIntEnable(void)
{
    /* Enable interrupts to be generated on every second.*/
    RTCIntTimerEnable(SOC_RTC_0_REGS, RTC_INT_EVERY_SECOND);
}

/*
** Initializes the RTC peripheral
*/
void RtcInit(void)
{
    /* Disabling Write Protection for RTC registers.*/
    RTCWriteProtectDisable(SOC_RTC_0_REGS);

    /* Selecting External Clock source for RTC. */
    RTC32KClkSourceSelect(SOC_RTC_0_REGS, RTC_EXTERNAL_CLK_SRC_SELECT);

    /* Enabling RTC to receive the Clock inputs. */
    RTC32KClkClockControl(SOC_RTC_0_REGS, RTC_32KCLK_ENABLE);

    RTCEnable(SOC_RTC_0_REGS);
}

/*
** Sets the Time and Calender in the RTC. This is a blocking call. 
** The time and date are entered through console.
*/
void RtcTimeCalSet(void)
{
    unsigned int time = 0; 
    unsigned int cal = 0;
    unsigned int temp = 0; 

    ConsoleUtilsPrintf("\n\rEnter Hours (0 to 23):");
    ConsoleUtilsScanf("%d", &temp);

    while(temp > 23)
    {
        ConsoleUtilsPrintf("\n\rValue entered is invalid. Enter value:");
        ConsoleUtilsScanf("%d", &temp);
    }

    time = (((temp / 10) << 4) << SHIFT_HOUR)
            | ((temp % 10) << SHIFT_HOUR);

    ConsoleUtilsPrintf("\n\rEnter Minutes (0 to 59):");
    ConsoleUtilsScanf("%d", &temp);

    while(temp > 59)
    {
        ConsoleUtilsPrintf("\n\rValue entered is invalid. Enter value:");
        ConsoleUtilsScanf("%d", &temp);
    }

    time |= (((temp / 10) << 4) << SHIFT_MIN)
            | ((temp % 10) << SHIFT_MIN);
 
    ConsoleUtilsPrintf("\n\rEnter Seconds (0 to 59):");
    ConsoleUtilsScanf("%d", &temp);

    while(temp > 59)
    {
        ConsoleUtilsPrintf("\n\rValue entered is invalid. Enter value:");
        ConsoleUtilsScanf("%d", &temp);
    }

    time |= (((temp / 10) << 4) << SHIFT_SEC)
             | ((temp % 10) << SHIFT_SEC);

    ConsoleUtilsPrintf("\n\rEnter the day of the month (1 to 31):");
    ConsoleUtilsScanf("%d", &temp);

    while((temp > 31) || (0 == temp))
    {
        ConsoleUtilsPrintf("\n\rValue entered is invalid. Enter value:");
        ConsoleUtilsScanf("%d", &temp);
    }

    cal = (((temp / 10) << 4) << SHIFT_DAY)
           | ((temp % 10) << SHIFT_DAY);

    ConsoleUtilsPrintf("\n\rEnter the month (1 to 12):");
    ConsoleUtilsScanf("%d", &temp);

    while((temp > 12) || (0 == temp))
    {
        ConsoleUtilsPrintf("\n\rValue entered is invalid. Enter value:");
        ConsoleUtilsScanf("%d", &temp);
    }

    cal |= (((temp / 10) << 4) << SHIFT_MON)
            | ((temp % 10) << SHIFT_MON);

    ConsoleUtilsPrintf("\n\rEnter the year (0 to 99):");
    ConsoleUtilsScanf("%d", &temp);
    while(temp > 99)
    {
        ConsoleUtilsPrintf("\n\rValue entered is invalid. Enter value:");
        ConsoleUtilsScanf("%d", &temp);
    }

    cal |= (((temp / 10) << 4) << SHIFT_YEAR)
            | ((temp % 10) << SHIFT_YEAR);

    ConsoleUtilsPrintf("\n\rEnter Day of the week (0 for Sunday...6 for "
                       "Saturday):");
    ConsoleUtilsScanf("%d", &temp);

    while(temp > 6)
    {
        ConsoleUtilsPrintf("\n\rValue entered is invalid. Enter value:");
        ConsoleUtilsScanf("%d", &temp);
    }

    cal |= (((temp / 10) << 4)) | ((temp % 10));

    /* Set the calendar registers of RTC with received calendar information.*/
    RTCCalendarSet(SOC_RTC_0_REGS, cal);

    /* Set the time registers of RTC with the received time information.*/
    RTCTimeSet(SOC_RTC_0_REGS, time);

    /* Run the RTC. The seconds tick from now on.*/
    RTCRun(SOC_RTC_0_REGS);
 
    ConsoleUtilsPrintf("\n\rThe Time and Date are set successfully! \n\n\r");

    rtcSetFlag = TRUE;
}

/*
** Displays the Time and Date on the UART console
*/
void RtcTimeCalDisplay(void)
{
    unsigned int time = 0;
    unsigned int cal = 0;
    unsigned char strTime[9] = {'\0'};
    unsigned char strDate[9] = {'\0'};

    time = RTCTimeGet(SOC_RTC_0_REGS);
    cal = RTCCalendarGet(SOC_RTC_0_REGS);
    TimeToStr(time, strTime);
    DateToStr(cal, strDate);
    ConsoleUtilsPrintf("\rCurrent Time And Date: %s, %s, ", strTime, strDate);

    switch(cal & MASK_DOTW)
    {
        case 0x00:
             ConsoleUtilsPrintf("Sun");
        break;

        case 0x01:
             ConsoleUtilsPrintf("Mon");
        break;

        case 0x02:
             ConsoleUtilsPrintf("Tue");
        break;

        case 0x03:
             ConsoleUtilsPrintf("Wed");
        break;

        case 0x04:
             ConsoleUtilsPrintf("Thu");
        break;

        case 0x05:
             ConsoleUtilsPrintf("Fri");
        break;

        case 0x06:
             ConsoleUtilsPrintf("Sat");

        default:
        break;

    }
}

/*
**Configure AINTC for enabling RTC alarm interrupt
*/
void RTCAlarmAINTCConfigure(void)
{
    /* Registering the Interrupt Service Routine(ISR). */
    IntRegister(SYS_INT_RTCALARMINT, RTCAlarmIsr);

    /* Setting the priority for the system interrupt in AINTC. */
    IntPrioritySet(SYS_INT_RTCALARMINT, 0, AINTC_HOSTINT_ROUTE_IRQ);

    /* Enabling the system interrupt in AINTC. */
    IntSystemEnable(SYS_INT_RTCALARMINT);
}

/*
** Configure wake up for  RTC alarm  interrupt
*/
void configWakeRTC(void)
{
    unsigned int currTime = 0;
    unsigned int alarmTime = 0x00002000; /* 20 Seconds for RTC Wake */
    unsigned int alarmDate = 0;
    unsigned char time[9]= {0};
    unsigned char date[9]= {0};

    /*	Force RTC into 24hr format	*/
    RTCHourModeSet(RTC_INST_BASE, RTC_24HOUR_MODE);

    /*	get current time	*/
    currTime = RTCTimeGet(RTC_INST_BASE);

    /*	get current date	*/
    alarmDate = RTCCalendarGet(RTC_INST_BASE);

    TimeToStr(currTime, time);
    DateToStr(alarmDate, date);
    ConsoleUtilsPrintf("\n\r Current Time & Date: %s, %s", time, date);

    /*	Add alarm time to current time	*/
    alarmTime = addTime(alarmTime, currTime, &alarmDate);

    /*	Set alarm Time	*/
    RTCAlarmTimeSet(RTC_INST_BASE, alarmTime);

    /*	Set alarm Date	*/
    RTCAlarmCalendarSet(RTC_INST_BASE, alarmDate);

    TimeToStr(alarmTime, time);
    DateToStr(alarmDate, date);
    ConsoleUtilsPrintf("\n\r RTC Alarm Wake Time & Date: %s, %s", time, date);

    /* Run the RTC. The seconds tick from now on.*/
    RTCRun(RTC_INST_BASE);
}

/*
** Eanble RTC wake alarm
*/
void  enableRTCAlarmWake(void)
{
    /* Disable timer interrupt */
    RTCIntTimerDisable(RTC_INST_BASE);

    RTCAlarmAINTCConfigure();

    /* Enable RTC smart-idle wakeup-capable mode       */
    RTCIdleModeConfigure(RTC_INST_BASE, RTC_IDLEMODE_SMART_IDLE_WAKEUP);

    /* Enable wakeup */
    RTCWakeUpAlarmEventControl(RTC_INST_BASE, RTC_ALARM_WAKEUP_ENABLE);

    /*     Clear IRQ Alarm wake up        */
    RTCAlarmIntStatusClear(RTC_INST_BASE);

    /* Enable alarm interrupt */
    RTCIntAlarmEnable(RTC_INST_BASE);

    /* Enable RTC_PMIC */
    RTCConfigPmicPowerEnable(RTC_INST_BASE, RTC_PMIC_PWR_ENABLE);
}

/*
** Eanble RTC alarm interrupt
*/
void  enableRTCAlarmIntr(void)
{
    /* Disable timer interrupt */
    RTCIntTimerDisable(RTC_INST_BASE);

    RTCAlarmAINTCConfigure();

    /* Clear IRQ Alarm wake up */
    RTCAlarmIntStatusClear(RTC_INST_BASE);

    /* Enable alarm interrupt */
    RTCIntAlarmEnable(RTC_INST_BASE);
}

/*
** Configure wake up for RTC Only
*/
void configRTCOnly(void)
{
    unsigned int currTime = 0;
    unsigned int alarm1Date = 0;
    unsigned int alarm1Time = 0x00002200; /* 22 Seconds for RTC Wake */
    unsigned int alarm2Date = 0;
    unsigned int alarm2Time = 0x00000200; /* 2 Seconds for RTC only */
    unsigned char time[9]= {0};
    unsigned char date[9]= {0};

    /*	Force RTC into 24hr format	*/
    RTCHourModeSet(RTC_INST_BASE, RTC_24HOUR_MODE);

    /*	get current time	*/
    currTime = RTCTimeGet(RTC_INST_BASE);

    /*	get current date	*/
    alarm1Date = RTCCalendarGet(RTC_INST_BASE);
    alarm2Date = alarm1Date;

    TimeToStr(currTime, time);
    DateToStr(alarm1Date, date);
    ConsoleUtilsPrintf("\n\r Current Time & Date: %s, %s", time, date);

    /*  Add alarm time to current time  */
    alarm2Time = addTime(alarm2Time, currTime, &alarm2Date);

    /*  Set alarm Time  */
    RTCAlarm2TimeSet(RTC_INST_BASE, alarm2Time);

    /*  Set alarm Date  */
    RTCAlarm2CalendarSet(RTC_INST_BASE, alarm2Date);

    TimeToStr(alarm2Time, time);
    DateToStr(alarm2Date, date);
    ConsoleUtilsPrintf("\n\r RTC Only Mode Sleep Time & Date: %s, %s", time,
                       date);

    /*	Add alarm time to current time	*/
    alarm1Time = addTime(alarm1Time, currTime, &alarm1Date);

    /*	Set alarm Time	*/
    RTCAlarmTimeSet(RTC_INST_BASE, alarm1Time);

    /*	Set alarm Date	*/
    RTCAlarmCalendarSet(RTC_INST_BASE, alarm1Date);

    TimeToStr(alarm1Time, time);
    DateToStr(alarm1Date, date);
    ConsoleUtilsPrintf("\n\r RTC Alarm Wake Time & Date: %s, %s", time, date);

    /* Set the polarity of RTC External Wake pin0 to Low */
    RTCConfigPmicExtWakePolarity(RTC_INST_BASE, 0,
                                 RTC_EXT_WAKEUP_POL_ACTIVE_LOW);

    /* Enable Wake through External Wake pin0 */
    RTCConfigPmicExtWake(RTC_INST_BASE, 0, RTC_EXT_WAKEUP_ENABLE);

    /* Disable the debounce on RTC External Wake pin0 */
    RTCConfigPmicExtWakeDebounce(RTC_INST_BASE, 0,
                                 RTC_PMIC_EXT_WAKEUP_DB_EN_DISABLE);

    /* Clear the external wake status of Wake pin0 */
    RTCPmicExtWakeStatusClear(RTC_INST_BASE, 0);

    /* Enable RTC_PMIC */
    RTCConfigPmicPowerEnable(RTC_INST_BASE, RTC_PMIC_PWR_ENABLE);

    /* Run the RTC. The seconds tick from now on.*/
    RTCRun(RTC_INST_BASE);
}

/*
** Eanble RTC Only
*/
void  enableRTCOnly(void)
{
    /* Disable timer interrupt */
    RTCIntTimerDisable(RTC_INST_BASE);

    /* Enable Write protect for RTC Registers */
    RTCWriteProtectDisable(RTC_INST_BASE);

    /* Enable RTC_PMIC */
    RTCConfigPmicPowerEnable(RTC_INST_BASE, RTC_PMIC_PWR_ENABLE);

    /*     Clear IRQ Alarm wake up        */
    RTCAlarmIntStatusClear(RTC_INST_BASE);

    /*     Clear IRQ Alarm2 wake up       */
    RTCAlarm2IntStatusClear(RTC_INST_BASE);

    /* Select External Feedback */
    RTCFeedbackResistanceSelect(RTC_INST_BASE, RTC_EXTERNAL_FEEDBACK_RES_SEL);

    /* Enable External OSC */
    RTC32KClkSourceSelect(RTC_INST_BASE, RTC_EXTERNAL_CLK_SRC_SELECT);

    /* Enable Alarm wakeup */
    RTCWakeUpAlarmEventControl(RTC_INST_BASE, RTC_ALARM_WAKEUP_ENABLE);

    /* Enable Timer wakeup */
    RTCWakeUpTimerEventControl(RTC_INST_BASE, RTC_TIMER_WAKEUP_DISABLE);

    /* Enable RTC smart-idle wakeup-capable mode       */
    RTCIdleModeConfigure(RTC_INST_BASE, RTC_IDLEMODE_SMART_IDLE_WAKEUP);

    /* Enable alarm interrupt */
    RTCIntAlarmEnable(RTC_INST_BASE);

    /* Enable alarm2 interrupt */
    RTCIntAlarm2Enable(RTC_INST_BASE);
}

/*
** Disable RTC wake alarm
*/
void disableRTCAlarm(void)
{
    /* Disable alarm interrupt */
    RTCIntAlarmDisable(RTC_INST_BASE);

    /* Disable alarm-wakeup */
    RTCWakeUpAlarmEventControl(RTC_INST_BASE, RTC_ALARM_WAKEUP_DISABLE);

    /* Configure IDle mode */
    RTCIdleModeConfigure(RTC_INST_BASE, RTC_IDLEMODE_FORCE_IDLE);
}

/*
** Disable RTC alarm interrupt
*/
void disableRTCIntr(void)
{
    /* Disable alarm interrupt */
    RTCIntAlarmDisable(RTC_INST_BASE);
}

/*
** Interrupt service routine for RTC
*/
static void RTCIsr(void)
{
    rtcSecUpdate = TRUE;
}

/*
** RTC alarm interrupt service routine
*/
static void RTCAlarmIsr(void)
{
    /* Clear Alarm interrupt */
    RTCAlarmIntStatusClear(RTC_INST_BASE);
}

/******************************** End of file **********************************/



