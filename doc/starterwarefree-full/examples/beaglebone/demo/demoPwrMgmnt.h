/**
 * \file     demoPwrMgmnt.h
 *
 * \brief    This file contains the function prototypes for PM RTC only mode
 *			 application
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

#ifndef      __PM_DEMO_H__
#define      __PM_DEMO_H__


#ifdef __cplusplus
extern "C" {
#endif

#include "hw_prm_mpu.h"
#include "hw_prm_per.h"
#include "cm3wkup_proxy.h"

extern unsigned int pmFlag;

/*****************************************************************************
**                   FUNCTION DECLARATIONS
*****************************************************************************/

void initializeAINTC(void);
void reInitPeripherals(void);
void clearTouchEvent(void);
void PowerSaveModeEnter(deepSleepData dsData, unsigned int slpMode);
extern void saveRestoreContext(unsigned int slpMode);
extern void configVddOpVoltage(void);
extern void enableWakeSource(unsigned int wakeSource);
extern void disableWakeSource(unsigned int wakeSource);
#ifdef __cplusplus
}
#endif

#endif