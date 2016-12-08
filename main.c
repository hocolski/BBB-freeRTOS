#include "soc_AM335x.h"
#include "beaglebone.h"
#include "gpio_v2.h"

#include "FreeRTOS.h"
#include "task.h"

/*****************************************************************************
**                INTERNAL MACRO DEFINITIONS
*****************************************************************************/


void init()
{
	/* Enabling functional clocks for GPIO1 instance. */
	GPIO1ModuleClkConfig();

	/* Enabling the GPIO module. */
	GPIOModuleEnable(SOC_GPIO_1_REGS);

	/* Resetting the GPIO module. */
	GPIOModuleReset(SOC_GPIO_1_REGS);

	/* Setting the GPIO pin as an output pin. */
	GPIODirModeSet(SOC_GPIO_1_REGS, 23, GPIO_DIR_OUTPUT);
	GPIODirModeSet(SOC_GPIO_1_REGS, 24, GPIO_DIR_OUTPUT);
}

void vLED_blink ( void *pvParameters )
{
	for(;;)
	{
		GPIOPinWrite(SOC_GPIO_1_REGS, 23, GPIO_PIN_HIGH);
		vTaskDelay( 500 );

		GPIOPinWrite(SOC_GPIO_1_REGS, 23, GPIO_PIN_LOW);
		vTaskDelay( 500 );
	}
}

int main()
{
	init();

	xTaskCreate( vLED_blink, "LED", 100, NULL, 1, NULL );

	vTaskStartScheduler();

	return 0;
} 


