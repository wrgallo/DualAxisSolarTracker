//  Purpose:      ENCODE STRING OR CHAR ARRAY TO BASE 64
//  Author:       Wellington Rodrigo Gallo
//                Inspired by the free softwares of 'soundstorm' (https://github.com/ArduinoHannove)
//  License:      GNU Lesser General Public License
//                Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#ifndef _my64Encoderh_
#define _my64Encoderh_

//Arduino
//  Purpose:      Recommended in all headers of Arduino Sketch
//  License:      GNU Lesser General Public License
//                Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
#include <Arduino.h>

/**
 * Returns the 64 Base Encoded String
 */
String base64_encode(String input);

/**
 * Modifies the char array and returns the new char array length
 */
int base64_encode(char *output, const char *input, int inputLen);

#endif
