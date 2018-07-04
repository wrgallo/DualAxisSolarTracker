/*
 serverFunc.h - HTML and XML Handler with UART Handler
 Created by:
              Wellington Rodrigo Gallo
               w.r.gallo@grad.ufsc.br
                    May, 2017
*/
#ifndef serverFunc_h
#define serverFunc_h

//Arduino
//  Purpose:      Needed in all headers of Arduino Sketch
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

//DNSServer
//  Purpose:      FOR AUTO REDIRECT TO ESP8266 IP
//  Version Used: 1.1.0
//  Source:       https://github.com/esp8266/Arduino/tree/master/libraries
//  License:      GNU Lesser General Public License
//                Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
#include <DNSServer.h>

//ESP8266WebServer
//  Purpose:      FOR HTML SERVER HANDLER FUNCTIONS
//  Version Used: 1.0 (Last Modified 8 May 2015)
//  Source:       https://github.com/esp8266/Arduino/tree/master/libraries
//  License:      GNU Lesser General Public License
//                Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
#include <ESP8266WebServer.h>

//WiFiManager
//  Purpose:      FOR WIFI CONNECTION MEMORY
//  Version Used: 0.12
//  Source:       https://github.com/tzapu/WiFiManager
//  License:      MIT License (permissive free software license)
#include <WiFiManager.h>          

//myEEPROM
//  Purpose:      SET DATA ON EEPROM MEMORY OR READ STRINGS FROM EEPROM MEMORY
//  Author:       Wellington Rodrigo Gallo
//  License:      GNU Lesser General Public License
//                Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
#include "myEEPROM.h"

//my64Encoder
//  Purpose:      ENCODE STRING OR CHAR ARRAY TO BASE 64
//  Author:       Wellington Rodrigo Gallo
//                Inspired by the free softwares of 'soundstorm' (https://github.com/ArduinoHannove)
//  License:      GNU Lesser General Public License
//                Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
#include "my64Encoder.h"

//myMailSender
//  Purpose:      Send SMTP E-Mails with HTML Body Content
//  Author:       Wellington Rodrigo Gallo
//                Inspired by:
//                  *the free softwares of 'soundstorm' (https://github.com/ArduinoHannove)
//                  *the free software  of 'Boris Shobat' (http://www.instructables.com/id/ESP8266-GMail-Sender/)
//  License:      GNU Lesser General Public License
//                Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
#include "myMailSender.h"

/*myNTP  
 * Purpose:      GET ONLINE TIME  
 * Author:       Wellington Rodrigo Gallo                
 * Inspired by the Michael Margolis' low level date functions
 * License:      GNU Lesser General Public License                
 *               Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#include "myNTP.h"

//---------------------------------------------------------------------------------------
//                                    DEFINES, PARAMETERS
//---------------------------------------------------------------------------------------
//DEBUG PARAMETERS
#define DEBUG_PRINTING          false

//ESP8266 ACESS POINT PARAMETERS
#define ESP_SSID                "SolarTrackerSystem"
#define ESP_PSWD                "12345678"

//HOME NETWORK LOGIN CREDENTIALS
#define ADMIN_NAME               "wrgallo"
#define ADMIN_PSWD               "12348765"

//COMMUNICATION BEETWEEN UNITS PROTOCOL
#define START_CHAR              2 //DEC NUMBER OF ASCII TABLE
#define END_CHAR                3 //DEC NUMBER OF ASCII TABLE
//---------------------------------------------------------------------------------------



//---------------------------------------------------------------------------------------
//                                          FUNCTIONS
//---------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------
/**
 * Build the parts of the Website that are used in all .html pages
 */
void buildOnce();

/**
 * Create the full .html Login Page
 */
void buildLoginPage(String msg);
 
/**
 * Create the full .html Webpage (the main page)
 */
void buildWebsite();

/**
 * Create the Javascript part of the HTML text
 */
void buildJavascript();

/**
 * Check if header is present and correct
 */
bool is_authentified();

/**
 * Handles the Login Webpage
 */
void handleLogin();

/**
 * Check USERNAME and PASSWORD
 */
bool checkCredentials(String user, String pswd);

/**
 * Handles the Webpage
 */
void handleRoot();

/**
 * Handles a Not Found Request
 */
void handleNotFound();

/**
 * Creates the .XML and Send it to the Server
 */
void handleXML();

/**
 * Creates the .HTML and Send it to the Server
 */
void handleWebsite();

/**
 * ESP8266 Setup
 */
void startESP(void);

/**
 * Server Setup
 */
void startServer();

/**
 * Handles Client Communication on Server;
 */
void handleServer(void);

/**
 * Handles Master Unit Communication
 */
void handleMaster(void);

/**
 * Send a email to the EMail Recipient Address saved in the EEPROM memory
 */
void reportMail();

/**
 * Request UTC and Send to Master on UART
 */
void sendOnlineTime();
//---------------------------------------------------------------------------------------


#endif 
