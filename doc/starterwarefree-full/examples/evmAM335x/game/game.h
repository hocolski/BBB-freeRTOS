/**
 * \file  game.h
 *
 * \brief Prototypes for the game.
 */

/* Copyright (c) 2006-2010 Texas Instruments Incorporated.  All rights reserved.
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
 * This is part of revision 6288 of the EK-LM3S2965 Firmware Package.
 * This file is modified to make it work for StarterWare. */


#ifndef __GAME_H__
#define __GAME_H__

#include "hw_types.h"

/******************************************************************************
**                      VARIABLE DEFINITIONS
******************************************************************************/

extern unsigned int g_ulScore;
extern unsigned short g_usPlayerX;
extern unsigned short g_usPlayerY;
extern unsigned short g_pusMonsterX[100];
extern unsigned short g_pusMonsterY[100];
extern char g_ppcMaze[94][127];

/******************************************************************************
**                      FUNCTION DEFINITIONS
******************************************************************************/

extern tBoolean MainScreen(void);
extern void PlayGame(void);


#endif // __GAME_H__
