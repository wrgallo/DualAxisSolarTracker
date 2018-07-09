#ifndef _myFUNCTIONS_H_
#define _myFUNCTIONS_H_

/*Arduino
  Purpose:      Needed in all Header Files
  License:      GNU Lesser General Public License
                Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#include "Arduino.h"


//--------------------------------------------------------------- 
//                      NRF24L01+ LIBRARY
//--------------------------------------------------------------- 
/*SPI
  Purpose:      Needed for SPI communication with nRF24L01+
  License:      GNU Lesser General Public License version 2 and version 2.1
                Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#include <SPI.h>

/*nRFL01
  Purpose:      Needed for SPI communication with nRF24L01+
  Source:       https://github.com/nRF24/RF24
  Author:       Copyright (c) 2007 Stefan Engelke <mbox@stefanengelke.de>
                Portions Copyright (C) 2011 Greg Copeland
                
  License:      Permission is hereby granted, free of charge, to any person
                obtaining a copy of this software and associated documentation
                files (the "Software"), to deal in the Software without
                restriction, including without limitation the rights to use, copy,
                modify, merge, publish, distribute, sublicense, and/or sell copies
                of the Software, and to permit persons to whom the Software is
                furnished to do so, subject to the following conditions:
                
                The above copyright notice and this permission notice shall be
                included in all copies or substantial portions of the Software.
*/

/*SdFat
  Purpose:      Needed for SPI communication with Micro SD Module
  Author:       Copyright (C) 2012 by William Greiman
  License:      GNU Lesser General Public License version 2
                Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#include <SdFat.h>


//COMMUNICATION PROTOCOL USED IN UART
#define START_CHAR    2                                          //DEC NUMBER OF ASCII TABLE
#define END_CHAR      3                                          //DEC NUMBER OF ASCII TABLE
//---------------------------------------------------------------



//---------------------------------------------------------------
//                           FUNCTIONS
//--------------------------------------------------------------
/**
 * Setup GPIO, UART, SPI, I2C, Start Display...
 */
void configThisUnit();

/**
 * Open/Create the file of the day.
 */
void createFile();

/**
 * Interrupt Timer Handler
 */
void timerHandler();

/**
 * Send information to external uC with SD Logger
 */
void handleData();

/**
 * True if not requested to stop yet
 */
bool checkStillLogging();
//---------------------------------------------------------------
#endif // _myFUNCTIONS_H_
