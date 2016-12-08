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

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"

#include "dmtimer.h"
#include "soc_AM335x.h"
#include "gpio_v2.h"
#include "interrupt.h"

#define GPIO_INSTANCE_ADDRESS           (SOC_GPIO_1_REGS)
#define GPIO_INSTANCE_PIN_NUMBER        (23)

/* Constants required to handle critical sections. */
#define portNO_CRITICAL_NESTING		( ( unsigned long ) 0 )
volatile unsigned long ulCriticalNesting = 0;

void IRQHandler ( void ) __attribute__((naked));
void SVC_Handler ( void ) __attribute__((naked));

/*
 * Called by portYIELD() or taskYIELD() to manually force a context switch.
 *
 * Called by prvSetupTimerInterrupt() to change into SVC mode
 *
 * When a context switch is performed from the task level the saved task 
 * context is made to look as if it occurred from within the tick ISR.  This
 * way the same restore context function can be used when restoring the context
 * saved from the ISR or that saved from a call to vPortYieldProcessor.
 */
void SVC_Handler( void )
{
	/* Within an IRQ ISR the link register has an offset from the true return 
	address, but an SWI ISR does not.  Add the offset manually so the same 
	ISR return code can be used in both cases. */

        asm volatile (	"STMDB	SP!, {R0}		\n\t"	// Store R0
			"LDR	r0, [r14, #-4]		\n\t"   // Load  SWI instruction to R0
        		"BIC	r0, r0, #0xFF000000	\n\t"	// Get the SWI number
			"CMP	r0, #21			\n\t"	// Check if yeld or switch to SVC
			"LDMIA  SP!, {R0}		\n\t"	// Restore R0	
			"MOVEQ	PC, LR			\n\t"	// Leave handler and return to execution within SVC mode 
			"ADD	LR, LR, #4		\n\t"	// LR correction
			);

	/* Perform the context switch.  First save the context of the current task. */
	portSAVE_CONTEXT();

	/* Find the highest priority task that is ready to run. */
	vTaskSwitchContext();

	/* Restore the context of the new task. */
	portRESTORE_CONTEXT();	
}

void IRQHandler( void )
{
	/* Perform the context switch.  First save the context of the current task. */
	portSAVE_CONTEXT();
	
	/* Jump to ISR */
 	(*fnRAMVectors[IntActiveIrqNumGet()])();	

	/* Enable new IRQ generation */
	HWREG(SOC_AINTC_REGS + INTC_CONTROL) = INTC_CONTROL_NEWIRQAGR;

	/* Restore the context of the new task. */
	portRESTORE_CONTEXT();
}

/* 
 * The ISR used for the scheduler tick.
 */

void vTickISR( void )
{
	/* Disable the DMTimer interrupts */
	DMTimerIntDisable(SOC_DMTIMER_2_REGS, DMTIMER_INT_OVF_EN_FLAG);

	/* Clear the status of the interrupt flags */
	DMTimerIntStatusClear(SOC_DMTIMER_2_REGS, DMTIMER_INT_OVF_IT_FLAG);

	/* Enable the DMTimer interrupts */
	DMTimerIntEnable(SOC_DMTIMER_2_REGS, DMTIMER_INT_OVF_EN_FLAG);

	/* Increment the RTOS tick count, then look for the highest priority 
	task that is ready to run. */
	if( xTaskIncrementTick() != pdFALSE )
	{
		vTaskSwitchContext();
	}

}

/* The code generated by the GCC compiler uses the stack in different ways at
different optimisation levels.  The interrupt flags can therefore not always
be saved to the stack.  Instead the critical section nesting level is stored
in a variable, which is then saved as part of the stack context. */
void vPortEnterCritical( void )
{
	/* Disable interrupts as per portDISABLE_INTERRUPTS(); */
	portDISABLE_INTERRUPTS();

	/* Now interrupts are disabled ulCriticalNesting can be accessed 
	directly.  Increment ulCriticalNesting to keep a count of how many times
	portENTER_CRITICAL() has been called. */
	ulCriticalNesting++;
}

void vPortExitCritical( void )
{
	if( ulCriticalNesting > portNO_CRITICAL_NESTING )
	{
		/* Decrement the nesting count as we are leaving a critical section. */
		ulCriticalNesting--;

		/* If the nesting level has reached zero then interrupts should be
		re-enabled. */
		if( ulCriticalNesting == portNO_CRITICAL_NESTING )
		{
			/* Enable interrupts as per portEXIT_CRITICAL() */
			portENABLE_INTERRUPTS();
		}
	}
}
