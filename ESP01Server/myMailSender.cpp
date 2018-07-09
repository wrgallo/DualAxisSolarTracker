#include "myMailSender.h"

char serverAddr[100] = "mail.smtpcorp.com";
uint16_t serverPort = 2525;

WiFiClient client;

byte eRcv();

void mailConfig(String server, uint16_t port){
  serverPort = port;
  server.toCharArray(serverAddr, 100);
  serverAddr[ server.length()+1 ] = '\0';
}

/*
 * sendEmail
 * Return meaning
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
uint8_t sendEmail(String MailFrom, String MailPswd, String MailRecipient, String MailSubject, String *MailContent)
{
  
  if(!(client.connect(serverAddr, serverPort) == 1) )
  {
    Serial.println(F("connection failed"));
    return 1;
  }
  
  if (!eRcv())
    return 2;

  client.println("EHLO www.example.com");
  if (!eRcv())
    return 3;
  
  client.println("auth login");
  if (!eRcv())
    return 4;
  
  // Change to your base64, ASCII encoded user
  client.println( base64_encode( MailFrom ) ); // SMTP UserID
  if (!eRcv())
    return 5;
  
  // change to your base64, ASCII encoded password
  client.println( base64_encode( MailPswd ) ); //  SMTP Passw
  if (!eRcv())
    return 6;

  String temp = "MAIL From: " + MailFrom;
  client.println( temp );
  if (!eRcv())
    return 7;

  temp = "RCPT To: " + MailRecipient;
  client.println( temp );
  if (!eRcv())
    return 8;
  
  client.println(F("DATA"));
  if (!eRcv())
    return 9;

  temp = "To: " + MailRecipient;
  client.println(temp); // change to your address
  temp = "From: " + MailFrom;
  client.println( temp );
  temp = "Subject: [SolarTrackerSystem] " + MailSubject;
  client.println( temp );
  
  client.println("Mime-Version: 1.0;");
  client.println("Content-Type: text/html; charset=\"ISO-8859-1\";");
  client.println("Content-Transfer-Encoding: 7bit;");
  while( (*MailContent).length() > 45 )
  {
    client.print( (*MailContent).substring(0,45) );
    (*MailContent) = (*MailContent).substring(45);
    delay(5);
  }
  client.println( *MailContent );
  client.println(F("."));
  
  if (!eRcv())
    return 10;
  
  client.println(F("QUIT"));
  if (!eRcv())
    return 11;

  Serial.println("Email Sent");
  client.stop();
  return 0;
}

byte eRcv()
{
  byte respCode;
  byte thisByte;
  int loopCount = 0;
  while (!client.available())
  {
    delay(1);
    loopCount++; // if nothing received for 10 seconds, timeout
    if (loopCount > MAIL_TIMEOUT)
    {
      client.stop();
      Serial.println(F("\nTimeout"));
      return 0;
    }
  }

  respCode = client.peek();
  while (client.available())
  {
    thisByte = client.read();
    if(PRINT_SMTP_RESPONSE){Serial.write(thisByte);}
  }

  if (respCode >= '4')
  {
    return 0;
  }
  return 1;
}
