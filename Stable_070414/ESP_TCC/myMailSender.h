//myMailSender
//  Purpose:      SEND EMAILs
//  Author:       Wellington Rodrigo Gallo
//  License:      GNU Lesser General Public License
//                Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#ifndef _myMailSenderh_
#define _myMailSenderh_

//ESP8266WiFi
//  Purpose:      ESSENTIAL FOR ESP8266
//  Version Used: 1.0 (Last Modified December 2014)
//  Source:       https://github.com/esp8266/Arduino/tree/master/libraries
//  License:      GNU Lesser General Public License
//                Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
#include <ESP8266WiFi.h>

//my64Encoder
//  Purpose:      ENCODE STRING OR CHAR ARRAY TO BASE 64
//  Author:       Wellington Rodrigo Gallo
//                Inspired by the free softwares of 'soundstorm' (https://github.com/ArduinoHannove)
//  License:      GNU Lesser General Public License
//                Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
#include "my64Encoder.h"

#define MAIL_TIMEOUT 10000
#define PRINT_SMTP_RESPONSE false


/*
 * Specifies the SMTP server and port
 */
void mailConfig(String server, uint16_t port);

/*
 * sendEmail
 * 
 * Return meaning table:
 *  ----------------------------------------------
 * | value  | Meaning                             |
 *  ----------------------------------------------
 * | 0      | Email Sent                          |
 * | 1      | Couldn't Connect to SMTP Server     |
 * | 2      | SMTP Server Timeout                 |
 * | 3      | HELO Timeout                        |
 * | 4      | Auth Login Timeout                  |
 * | 5      | User ID Error                       |
 * | 6      | USER PW Error                       |
 * | 7      | MAIL FROM Timeout                   |
 * | 8      | RCPT TO Timeout                     |
 * | 9      | DATA Timeout                        |
 * | 10     | CONTENT Timeout                     |
 * | 11     | QUIT Timeout                        |
 * -----------------------------------------------
*/
uint8_t sendEmail(String MailFrom, String MailPswd, String MailRecipient, String MailSubject, String MailContent);

#endif
