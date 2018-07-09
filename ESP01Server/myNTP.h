/*myNTP  
 * Purpose:      GET ONLINE TIME  
 * Author:       Wellington Rodrigo Gallo                
 * Inspired by the Michael Margolis' low level date functions
 * License:      GNU Lesser General Public License                
 *               Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _myNTPh_
#define _myNTPh_

//Arduino
//  Purpose:      Recommended in all headers of Arduino Sketch
//  License:      GNU Lesser General Public License
//                Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
#include <Arduino.h>

//ESP8266WiFi
//  Purpose:      ESSENTIAL FOR ESP8266
//  Version Used: 1.0 (Last Modified December 2014)
//  Source:       https://github.com/esp8266/Arduino/tree/master/libraries
//  License:      GNU Lesser General Public License
//                Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
#include <ESP8266WiFi.h>

//WiFiUdp
//  Purpose:      ESSENTIAL FOR UDP COMMUNICATION
//  Version Used: Last Modified by Modified by Ivan Grokhotkov, January 2015
//  Source:       https://github.com/esp8266/Arduino/tree/master/libraries
//  License:      GNU Lesser General Public License
//                Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
#include <WiFiUdp.h>

/**
 * Get UNIX Time
 * Return false if problem occured
 */
bool requestUTC(int8_t timeZone=0);

/**
 * Return Hour value 00 - 23
 */
uint8_t getHour();

/**
 * Returns Minute Value 00 - 59
 */
uint8_t getMinute();

/**
 * Returns Second Value 00 - 59
 */
uint8_t getSecond();

/**
 * Returns Weekday 0 == Sunday, ... , 6 == Saturday
 */
uint8_t getWeekday();

/**
 * Returns day of month, 01 - 31
 */
uint8_t getDay();

/**
 * Returns month 01 - 12
 */
uint8_t getMonth();

/**
 * Returns year (2 Digit) 00 - 99
 */
uint8_t getYear();

#endif
