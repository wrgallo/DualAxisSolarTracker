//myEEPROM
//  Purpose:      SET DATA ON EEPROM MEMORY OR READ STRINGS FROM EEPROM MEMORY
//  Author:       Wellington Rodrigo Gallo
//  License:      GNU Lesser General Public License
//                Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
#ifndef _myEEPROMh_
#define _myEEPROMh_

//Arduino
//  Purpose:      Recommended in all headers of Arduino Sketch
//  License:      GNU Lesser General Public License
//                Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
#include <Arduino.h>

//EEPROM
//  Purpose:      Write and Read EEPROM Address Functions
//  License:      GNU Lesser General Public License
//                Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
#include <EEPROM.h>

#ifndef EEPROM_LENGTH
#define EEPROM_LENGTH 512
#endif

/*
 * MEMORY MAP
 * -----------------------------------------------------------------------------------------------------------
 * | ADDRESS START                                                                   |      PURPOSE           |
 * |---------------------------------------------------------------------------------|------------------------|
 * | 0                                                                               |      EMAIL LOGIN       | 
 * |- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -|- - - - - - - - - - - - |
 * | 1*3 + MAX_EMAIL_LOGIN_LENGTH                                                    |      EMAIL PASSWORD    |
 * |- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -|- - - - - - - - - - - - |
 * | 2*3 + MAX_EMAIL_LOGIN_LENGTH + MAX_EMAIL_PSWD_LENGTH                            |      USER_LOGIN        |
 * |- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -|- - - - - - - - - - - - |
 * | 3*3 + MAX_EMAIL_LOGIN_LENGTH + MAX_EMAIL_PSWD_LENGTH                            |      USER_PASSWOD      |
 * |     + MAX_USER_CREDENTIALS_LOGIN_LENGTH                                         |                        |
 * |- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -|- - - - - - - - - - - - |
 * | 4*3 + MAX_EMAIL_LOGIN_LENGTH + MAX_EMAIL_PSWD_LENGTH                            |      EMAIL_RECIPIENT   |
 * |     + MAX_USER_CREDENTIALS_LOGIN_LENGTH + MAX_USER_CREDENTIALS_PSWD_LENGTH      |                        |
 * |- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -|- - - - - - - - - - - - |
 * | 5*3 + MAX_EMAIL_LOGIN_LENGTH + MAX_EMAIL_PSWD_LENGTH                            |      TIME_ZONE         |
 * |     + MAX_USER_CREDENTIALS_LOGIN_LENGTH + MAX_USER_CREDENTIALS_PSWD_LENGTH      |                        |
 * |     + MAX_EMAIL_LOGIN_LENGTH                                                    |                        |
 * -----------------------------------------------------------------------------------------------------------
 * 
 * ---------------------------------------------------
 * | Caution:                                         |
 * |    Changing the max_length of any variable       |
 * |    requires all the data stored to be rewritten! |
 * ---------------------------------------------------
 */
#define MAX_USER_CREDENTIALS_LOGIN_LENGTH 20
#define MAX_USER_CREDENTIALS_PSWD_LENGTH 20
#define MAX_EMAIL_LOGIN_LENGTH 50
#define MAX_EMAIL_PSWD_LENGTH 15
#define MAX_TIMEZONE_LEN 3

void setThisByteToAll( byte value = 255);

void readEEPROM(bool detailed=false);

String getEmailLogin(void);
String getEmailPassword(void);
String getUserCredentialsLogin(void);
String getUserCredentialsPassword(void);
String getEmailRecipient(void);
int8_t getTimezone(void);

void setEmailLogin(String login);
void setEmailPassword(String password);
void setUserCredentialsLogin(String user);
void setUserCredentialsPassword(String password);
void setEmailRecipient(String mail);
void setTimezone(int8_t timezone);

#endif
