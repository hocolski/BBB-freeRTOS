Game Application

A game in which a blob-like character tries to find its way out of a maze.
The character starts in the middle of the maze and must find the exit,
which will always be located at one of the four corners of the maze.  Once
the exit to the maze is located, the character is placed into the middle of
a new maze and must find the exit to that maze; this repeats endlessly.

The game is started by pressing the select button on the right side
of the LCD.  During game play, the select button will fire a bullet
in the direction the character is currently facing, and the navigation push
buttons on the left side of the LCD will cause the character to walk in
the corresponding direction.

Populating the maze are a hundred spinning stars that mindlessly attack the
character.  Contact with one of these stars results in the game ending, but
the stars go away when shot.

Score is accumulated for shooting the stars and for finding the exit to the
maze.  The game lasts for only one character, and the score is displayed on
the virtual UART at 115,200, 8-N-1 during game play and will be displayed
on the screen at the end of the game.

The application also contains a screen saver.  The screen saver will only 
become active if two minutes have passed without the user action while 
waiting to start the game (that is, it will never come on during game play).
Qix-style bouncing lines are drawn on the display by the screen saver.

After two minutes of running the screen saver, the display will be turned
off and the user LED will blink.  Either mode of screen saver (bouncing
lines or blank display) will be exited by touching the touch screen on LCD.
The select button will then need to be pressed again to start the game.

-------------------------------------------------------------------------------

Copyright (c) 2006-2010 Texas Instruments Incorporated.  All rights reserved.
Software License Agreement

Texas Instruments (TI) is supplying this software for use solely and
exclusively on TI's microcontroller products. The software is owned by
TI and/or its suppliers, and is protected under applicable copyright
laws. You may not combine this software with "viral" open-source
software in order to form a larger program.

THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
DAMAGES, FOR ANY REASON WHATSOEVER.

This is part of revision 6288 of the Stellaris Firmware Development Package.
This has been modified to fit in StarterWare.
