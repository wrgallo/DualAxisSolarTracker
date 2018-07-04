#ifndef _myFUNCTIONS_H_
#define _myFUNCTIONS_H_

/*Arduino
  Purpose:      Needed in all Header Files
  License:      GNU Lesser General Public License
                Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#include "Arduino.h"



//---------------------------------------------------------------
//                     LIBRARIES FOR HC05
//--------------------------------------------------------------- 
/*SoftwareSerial
  Purpose:      Needed to use more than one UART communication
  License:      GNU Lesser General Public License
                Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#include <SoftwareSerial.h>
//--------------------------------------------------------------- 


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
#include "nRF24L01.h"

/*RF24
  Purpose:      Needed for SPI communication with nRF24L01+
  Source:       https://github.com/nRF24/RF24
  License:      GNU Lesser General Public License version 2
                Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#include "RF24.h"
//--------------------------------------------------------------- 


//--------------------------------------------------------------- 
//                      I2C DEVICES LIBRARY
//--------------------------------------------------------------- 
/*Wire
  Purpose:      Needed for I2C Communication
  License:      GNU Lesser General Public License
                Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#include "Wire.h"

/*MPU9250
  Purpose:      Needed for I2C communication with MPU9250
  Source:       https://github.com/bolderflight/MPU9250
  Author:       Copyright (c) 2017 Bolder Flight Systems <brian.taylor@bolderflight.com>
                Brian R Taylor
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
//#include "MPU9250.h"

/*TSL2561
  Purpose:      Needed for I2C communication with TSL2561
  Source:       https://github.com/sparkfun/SparkFun_TSL2561_Arduino_Library
  Author:       SparkFun Electronics trademark
  License:      Beerware license
*/
#include <SparkFunTSL2561.h>

/*ADS1015
  Purpose:      Needed for I2C communication with ADS1015
  Source:       https://github.com/adafruit/Adafruit_ADS1X15
  Author:       Copyright (c) 2012, Adafruit Industries
                All rights reserved.
  License:      Software License Agreement (BSD License)
                Redistribution and use in source and binary forms, with or without
                modification, are permitted provided that the following conditions are met:
                1. Redistributions of source code must retain the above copyright
                notice, this list of conditions and the following disclaimer.
                2. Redistributions in binary form must reproduce the above copyright
                notice, this list of conditions and the following disclaimer in the
                documentation and/or other materials provided with the distribution.
                3. Neither the name of the copyright holders nor the
                names of its contributors may be used to endorse or promote products
                derived from this software without specific prior written permission.

                THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
                EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
                WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
                DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
                DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
                (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
                LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
                ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
                (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
                SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <Adafruit_ADS1015.h>

/*INA219
  Purpose:      Needed for I2C communication with INA219
  Source:       https://github.com/adafruit/Adafruit_INA219
  Author:       Copyright (c) 2012, Adafruit Industries
                All rights reserved.
  License:      Software License Agreement (BSD License)
                Redistribution and use in source and binary forms, with or without
                modification, are permitted provided that the following conditions are met:
                1. Redistributions of source code must retain the above copyright
                notice, this list of conditions and the following disclaimer.
                2. Redistributions in binary form must reproduce the above copyright
                notice, this list of conditions and the following disclaimer in the
                documentation and/or other materials provided with the distribution.
                3. Neither the name of the copyright holders nor the
                names of its contributors may be used to endorse or promote products
                derived from this software without specific prior written permission.

                THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
                EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
                WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
                DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
                DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
                (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
                LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
                ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
                (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
                SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <Adafruit_INA219.h>
//--------------------------------------------------------------- 


/*SdFat
  Purpose:      Needed for SPI communication with Micro SD Module
  Author:       Copyright (C) 2012 by William Greiman
  License:      GNU Lesser General Public License version 2
                Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
//#include <SdFat.h>



/*Stepper
  Purpose:      Needed for stepper motor control
  Source:       Included with Arduino IDE
  License:      Creative Commons Attribution-ShareAlike 3.0 License
*/
#include <Stepper.h>


//---------------------------------------------------------------
//        GPIO CONFIGURATION AND DEFINITIONS
//---------------------------------------------------------------
//PINOUT DEFINITIONS
#define BT_RX       6                                            //UART RX FOR BLUETOOTH MODULE
#define BT_TX       7                                            //UART TX FOR BLUETOOTH MODULE
#define LOGGER_RX   8                                            //UART RX COMMUNICATION FOR ANOTHER uC WITH CATALEX SD RECORDER/READER
#define LOGGER_TX   A7                                           //UART TX COMMUNICATION FOR ANOTHER uC WITH CATALEX SD RECORDER/READER
#define LM35_PIN    A6                                           //LM35 Vout Pin

//I2C ADDRESS DEFINITIONS
#define DS1307_ADDRESS 0x68                                      //REAL TIMER MODULE I2C ADDRESS
#define MPU9250_ADDRESS 0x69                                     //MPU 9-DOF

//CONSTANT OF PROJECT DEFINITIONS                                                             
#define SLAVE_IS_OFFLINE_TIMER 140                               //IF EACH SLAVE DONT ANSWER IN THIS TIME, IT IS OFFLINE
#define WAIT_FOR_AN_ANSWER 50                                    //WAIT FOR A RF24 ANSWER, MULTIPLE OF 20ms

//STEPPER MOTOR CONFIGURATION
#define STEPS_PER_REVOLUTION 8                                   //USING HALF-STEP FOR 28BYJ48
#define STEPPER_SPEED 2000                                       //SPEED CONTROL FOR STEPPER MOTORS
#define STEPS_PER_ROTATION 2048                                  //STEPS FOR A FULL ROTATION IN THIS CONFIGURATION
#define STEPS_PER_CHANGE_M1 15                                   //STEPS CHANGE WHEN SOLAR TRACKING FOR M1 (YAW CONTROL)
#define STEPS_PER_CHANGE_M2 30                                   //STEPS CHANGE WHEN SOLAR TRACKING FOR M2 (PITCH CONTROL)
#define STEPS_MAX_M1 9000                                        //SAFE LIMIT CONTROL FOR MOTOR1 (TO NOT DAMAGE THE STRUCTURE AND CABLES)
#define STEPS_MAX_M2 2000                                        //SAFE LIMIT CONTROL FOR MOTOR2 (TO NOT DAMAGE THE STRUCTURE AND CABLES)

//COMMUNICATION PROTOCOL BEETWEEN ESP8266, ARDUINO AND RF24
#define START_CHAR    2                                          //DEC NUMBER OF ASCII TABLE
#define END_CHAR      3                                          //DEC NUMBER OF ASCII TABLE

//DEBUGGING CONFIGURATION
#define DEBUG_PRINTING true                                      //[DEBUG ONLY] - Use Software UART for printing debug information
//---------------------------------------------------------------



//---------------------------------------------------------------
//                           FUNCTIONS
//---------------------------------------------------------------



//---------------------------------------
// FUNCTIONS: MASTER UNIT CONFIGURATION
//---------------------------------------
/**
 * Setup GPIO, UART, SPI, I2C, Start Display...
 */
void configThisUnit();

/**
 * RTC Gives the date and time, but this timer is used for internal control of periodic instructions
 */
void timerHandler();

/**
 * 1 Second Timer Handler
 * This function is called when DS1307 reports
 * a new second value.
 * serverHandler() is also called in this function.
 */
void oneSecondTimerHandler();

/**
  * 1 Minute Timer Handler
  * refreshServer() is also called in this function.
  */
void oneMinuteTimerHandler();



//---------------------------------------
// FUNCTIONS: WEB SERVER COMMUNICATION
//---------------------------------------
/**
 * Send updated information to ESP8266
 */
void refreshServer();

/**
 * Check if there is an incoming message from ESP8266
 */
void serverHandler();
//---------------------------------------



//---------------------------------------
// FUNCTIONS: RF24 COMMUNICATION
//---------------------------------------
/**
 * Check for incoming messages from nRF24L01+
 */
void handleRF24();

/**
 * Handle valid incoming messages from nRF24L01+
 */
void handleMessage();
//---------------------------------------



//---------------------------------------
// FUNCTIONS: REAL TIME MODULE
//---------------------------------------
/**
 * Convert Decimal to BCD
 * Used by DS1307
 */
byte    dec2bcd(byte    number);

/**
 * Convert BCD to Decimal
 * Used by DS1307
 */
uint8_t bcd2dec(uint8_t number);

/**
 * Update DS1307 time
 * timeWeekday = 0, means Sunday
 * timeWeekday = 6, means Saturday
 */
void setTimeDS1307(byte timeYear /*00 to 99*/, byte timeMonth /*01 to 12*/, byte timeDay /*01 to 31*/, byte timeWeekday /*0 to 6*/, byte timeHour /*00 to 23*/, byte timeMinute /*00 to 59*/);

/**
 * Get DS1307 time
 */
void getTimeDS1307();
//---------------------------------------

/**
 * Refresh data from all the sensors onboard
  * Power Monitor
  * Temperature
  * LUX
  * Angle (MPU)
 */
void refreshSensorData();

/**
 * Get new data from Phototransistors
 */
void refreshPhotoData();

/**
 * Try to point to the Sun
 */
void solarTracker();


/**
 * Send information to external uC with SD Logger
 */
void logData();

/**
 * Update Data to SD
 */
//void updateSDFile();

//---------------------------------------------------------------
#endif // _myFUNCTIONS_H_
