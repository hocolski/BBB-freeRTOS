/**
 * \file  random.h
 *
 * \brief Protoytpes for the random number generator.
 */

/* Copyright (c) 2005-2010 Texas Instruments Incorporated.  All rights reserved.
 * Software License Agreement
 * 
 * Texas Instruments (TI) is supplying this software for use solely and
 * exclusively on TI's microcontroller products. The software is owned by
 * TI and/or its suppliers, and is protected under applicable copyright
 * laws. You may not combine this software with "viral" open-source
 * software in order to form a larger program.
 * 
 * THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
 * NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
 * NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
 * CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES, FOR ANY REASON WHATSOEVER.
 * 
 * This is part of revision 6288 of the EK-LM3S2965 Firmware Package. */

#ifndef __RANDOM_H__
#define __RANDOM_H__

/******************************************************************************
**                      FUNCTION DEFINITIONS
******************************************************************************/
extern void RandomAddEntropy(unsigned int ulEntropy);
extern void RandomSeed(void);
extern unsigned int RandomNumber(void);

#endif /* __RANDOM_H__ */
