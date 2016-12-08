# BBB-freeRTOS

This weekend project allows incorporation of Texas Instrument's [starterware](http://processors.wiki.ti.com/index.php/StarterWare) hardware API into [FreeRTOS](http://www.freertos.org/) projects.

Based on [steve-kim](https://github.com/steve-kim/BeagleBone) and [jerrinsg](https://github.com/jerrinsg/RTOS_Bone) ports. Updated to FreeRTOS V9.0.0 

### Directory structure

Standard FreeRTOS port directory structure was changed. You can find TI's and FreeRTOS code in respective directories, complete starterware code with examples in `./doc`

### Usage

Compile with `make`. Put `app` and `./doc/MLO` files on your external SD card formatted acording to [this](http://processors.wiki.ti.com/index.php/SD/MMC_format_for_OMAP3_boot) guide. Apply power to board while pressing boot button. `D4` should start flashing.

### Not working

- critical section nesting
- FIQ
- interrupt priorities and nesting
