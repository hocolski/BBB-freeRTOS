/*
    FreeRTOS V7.0.1 - Copyright (C) 2011 Real Time Engineers Ltd.

    ***************************************************************************
    *                                                                         *
    * If you are:                                                             *
    *                                                                         *
    *    + New to FreeRTOS,                                                   *
    *    + Wanting to learn FreeRTOS or multitasking in general quickly       *
    *    + Looking for basic training,                                        *
    *    + Wanting to improve your FreeRTOS skills and productivity           *
    *                                                                         *
    * then take a look at the FreeRTOS books - available as PDF or paperback  *
    *                                                                         *
    *        "Using the FreeRTOS Real Time Kernel - a Practical Guide"        *
    *                  http://www.FreeRTOS.org/Documentation                  *
    *                                                                         *
    * A pdf reference manual is also available.  Both are usually delivered   *
    * to your inbox within 20 minutes to two hours when purchased between 8am *
    * and 8pm GMT (although please allow up to 24 hours in case of            *
    * exceptional circumstances).  Thank you for your support!                *
    *                                                                         *
    ***************************************************************************

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
    ***NOTE*** The exception to the GPL is included to allow you to distribute
    a combined work that includes FreeRTOS without being obliged to provide the
    source code for proprietary components outside of the FreeRTOS kernel.
    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT
    ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
    FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
    more details. You should have received a copy of the GNU General Public 
    License and the FreeRTOS license exception along with FreeRTOS; if not it 
    can be viewed here: http://www.freertos.org/a00114.html and also obtained 
    by writing to Richard Barry, contact details for whom are available on the
    FreeRTOS WEB site.

    1 tab == 4 spaces!

    http://www.FreeRTOS.org - Documentation, latest information, license and
    contact details.

    http://www.SafeRTOS.com - A version that is certified for use in safety
    critical systems.

    http://www.OpenRTOS.com - Commercial support, development, porting,
    licensing and training services.
*/


/*-----------------------------------------------------------
 * Implementation of functions defined in portable.h for the ARM7 port.
 *
 * Components that can be compiled to either ARM or THUMB mode are
 * contained in this file.  The ISR routines, which can only be compiled
 * to ARM mode are contained in portISR.c.
 *----------------------------------------------------------*/


/* Standard includes. */
#include <stdlib.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"

/* starterware */
#include "interrupt.h"
#include "dmtimer.h"
#include "cache.h"
#include "cpu.h"
#include "beaglebone.h"
#include "soc_AM335x.h"

/* Constants required to setup the task context. */
#define portINITIAL_SPSR			( ( portSTACK_TYPE ) 0x1F ) /* System mode, ARM mode, interrupts enabled. */
#define portINSTRUCTION_SIZE			( ( portSTACK_TYPE ) 4 )
#define TIMER_COUNT             		((0xFFFFFFFFu) - (25000000 / configTICK_RATE_HZ))

/* Setup the timer to generate the tick interrupts. */
static void prvSetupTimerInterrupt( void );
extern void vPortISRStartFirstTask( void );
extern void vTickISR( void );

/* 
 * Initialise the stack of a task to look exactly as if a call to 
 * portSAVE_CONTEXT had been called.
 *
 * See header file for description. 
 */
portSTACK_TYPE *pxPortInitialiseStack( portSTACK_TYPE *pxTopOfStack, pdTASK_CODE pxCode, void *pvParameters )
{
portSTACK_TYPE *pxOriginalTOS;

	pxOriginalTOS = pxTopOfStack;

	/* Setup the initial stack of the task.  The stack is set exactly as 
	expected by the portRESTORE_CONTEXT() macro. */

	/* First on the stack is the return address - which in this case is the
	start of the task.  The offset is added to make the return address appear
	as it would within an IRQ ISR. */
	*pxTopOfStack = ( portSTACK_TYPE ) pxCode + portINSTRUCTION_SIZE;		
	pxTopOfStack--;

	*pxTopOfStack = ( portSTACK_TYPE ) 0x14141414;	/* R14 */
	pxTopOfStack--;	
	*pxTopOfStack = ( portSTACK_TYPE ) pxOriginalTOS; /* Stack used when task starts goes in R13. */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0x12121212;	/* R12 */
	pxTopOfStack--;	
	*pxTopOfStack = ( portSTACK_TYPE ) 0x11111111;	/* R11 */
	pxTopOfStack--;	
	*pxTopOfStack = ( portSTACK_TYPE ) 0x10101010;	/* R10 */
	pxTopOfStack--;	
	*pxTopOfStack = ( portSTACK_TYPE ) 0x09090909;	/* R9 */
	pxTopOfStack--;	
	*pxTopOfStack = ( portSTACK_TYPE ) 0x08080808;	/* R8 */
	pxTopOfStack--;	
	*pxTopOfStack = ( portSTACK_TYPE ) 0x07070707;	/* R7 */
	pxTopOfStack--;	
	*pxTopOfStack = ( portSTACK_TYPE ) 0x06060606;	/* R6 */
	pxTopOfStack--;	
	*pxTopOfStack = ( portSTACK_TYPE ) 0x05050505;	/* R5 */
	pxTopOfStack--;	
	*pxTopOfStack = ( portSTACK_TYPE ) 0x04040404;	/* R4 */
	pxTopOfStack--;	
	*pxTopOfStack = ( portSTACK_TYPE ) 0x03030303;	/* R3 */
	pxTopOfStack--;	
	*pxTopOfStack = ( portSTACK_TYPE ) 0x02020202;	/* R2 */
	pxTopOfStack--;	
	*pxTopOfStack = ( portSTACK_TYPE ) 0x01010101;	/* R1 */
	pxTopOfStack--;	

	/* When the task starts is will expect to find the function parameter in
	R0. */
	*pxTopOfStack = ( portSTACK_TYPE ) pvParameters; /* R0 */
	pxTopOfStack--;

	/* First 8 64bit NEON registers */

	*pxTopOfStack = ( portSTACK_TYPE ) 0xD7D7D7D7;	/* D7 */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0xD7D7D7D7;	/* D7 */
	pxTopOfStack--;	
	*pxTopOfStack = ( portSTACK_TYPE ) 0xD6D6D6D6;	/* D6 */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0xD6D6D6D6;	/* D6 */
	pxTopOfStack--;		
	*pxTopOfStack = ( portSTACK_TYPE ) 0xD5D5D5D5;	/* D5 */
	pxTopOfStack--;	
	*pxTopOfStack = ( portSTACK_TYPE ) 0xD5D5D5D5;	/* D5 */
	pxTopOfStack--;	
	*pxTopOfStack = ( portSTACK_TYPE ) 0xD4D4D4D4;	/* D4 */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0xD4D4D4D4;	/* D4 */
	pxTopOfStack--;		
	*pxTopOfStack = ( portSTACK_TYPE ) 0xD3D3D3D3;	/* D3 */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0xD3D3D3D3;	/* D3 */
	pxTopOfStack--;		
	*pxTopOfStack = ( portSTACK_TYPE ) 0xD2D2D2D2;	/* D2 */
	pxTopOfStack--;	
	*pxTopOfStack = ( portSTACK_TYPE ) 0xD2D2D2D2;	/* D2 */
	pxTopOfStack--;	
	*pxTopOfStack = ( portSTACK_TYPE ) 0xD1D1D1D1;	/* D1 */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0xD1D1D1D1;	/* D1 */
	pxTopOfStack--;	
	*pxTopOfStack = ( portSTACK_TYPE ) 0xD0D0D0D0;	/* D0 */
	pxTopOfStack--;		
	*pxTopOfStack = ( portSTACK_TYPE ) 0xD0D0D0D0;	/* D0 */
	pxTopOfStack--;	

	/* The next thing onto the stack is the status register, which is set for
	system mode, with interrupts enabled. */
	*pxTopOfStack = ( portSTACK_TYPE ) portINITIAL_SPSR;
	pxTopOfStack--;	

	/* FPSCR is last on stack */
	*pxTopOfStack = ( portSTACK_TYPE ) 0x01010101;	/* FPSCR */


	return pxTopOfStack;
}

/*
 * Setup the timer 2 to generate the tick interrupts at the required frequency.
 */
static void prvSetupTimerInterrupt( void )
{
	/* Enable Instruction Cache */
	CacheEnable(CACHE_ALL);

	/* Initializing the ARM Interrupt Controller. */
	IntAINTCInit();

	/* Registering the Interrupt Service Routine(ISR). */
    	IntRegister(SYS_INT_TINT2, vTickISR);

    	/* Setting the priority for the system interrupt in AINTC. */
    	IntPrioritySet(SYS_INT_TINT2, 0, AINTC_HOSTINT_ROUTE_IRQ);

	/* Enable IRQ in CPSR */
    	IntMasterIRQEnable();

    	/* Enabling the system interrupt in AINTC. */
    	IntSystemEnable(SYS_INT_TINT2);   

    	/* This function will enable clocks for the DMTimer2 instance */
   	DMTimer2ModuleClkConfig();

	/* Load the counter with the initial count value */
    	DMTimerCounterSet(SOC_DMTIMER_2_REGS, TIMER_COUNT);

    	/* Load the load register with the reload count value */
    	DMTimerReloadSet(SOC_DMTIMER_2_REGS, TIMER_COUNT);

    	/* Configure the DMTimer for auto-reload mode */
    	DMTimerModeConfigure(SOC_DMTIMER_2_REGS, DMTIMER_AUTORLD_NOCMP_ENABLE);

    	/* Enable the DMTimer interrupts */
    	DMTimerIntEnable(SOC_DMTIMER_2_REGS, DMTIMER_INT_OVF_EN_FLAG);

	/* Start the DMTimer */
   	DMTimerEnable(SOC_DMTIMER_2_REGS);

}

/* 
* Start the timer and run tasks. 
*/
portBASE_TYPE xPortStartScheduler( void )
{
	/* Start timer */
	prvSetupTimerInterrupt();

	/* change to SVC mode */
	asm("SWI   21");

	/* Start the first task. */
	portRESTORE_CONTEXT();

	/* Should not get here! */
	return 0;
}

/* 
* It is unlikely that the ARM port will require this function as there
* is nothing to return to.  
*/
void vPortEndScheduler( void )
{
	for(;;);
}

