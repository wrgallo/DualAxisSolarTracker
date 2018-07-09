//myEEPROM
//  Purpose:      SET DATA ON EEPROM MEMORY OR READ STRINGS FROM EEPROM MEMORY
//  Author:       Wellington Rodrigo Gallo
//  License:      GNU Lesser General Public License
//                Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
#ifndef _myEEPROMh_
#define _myEEPROMh_

/*Arduino
  Purpose:      Needed in all Header Files
  License:      GNU Lesser General Public License
                Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#include <Arduino.h>

/*EEPROM
  Purpose:      Needed for Read and Write EEPROM functions
  License:      GNU Lesser General Public License
                Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#include <EEPROM.h>

#ifndef EEPROM_LENGTH
#define EEPROM_LENGTH 1024
#endif

void setThisByteToAll( byte value = 255);
void readEEPROM(bool detailed=true);

void setDate(String filename);
String getDate(bool nextDay);

#endif
