#include "Arduino.h"
#include "myFunctions.h"
//#include "myEEPROM.h"

void logThisLine(String message);
bool checkForInvalidChars(String *filename);
//---------------------------------------------------------------
//                  GLOBAL VARIABLES DEFINITIONS
//---------------------------------------------------------------
volatile bool timerFlag     = false;                             //PERIODIC TIMER INTERRUPT FLAG
const char CharactersInvalidForFileName[] = {'#','<','$','+','%','>','!','`','&','*','“','|','{','?','”','=','}','/',':','\\','@'};
const uint8_t CharactersInvalidForFileNameLEN = 21;

//---------------------------------------------------------------
//                     MICRO SD MODULE DEFINITION
//---------------------------------------------------------------
SdFat sdCard;
SdFile todayLog;


//---------------------------------------------------------------
//             FUNCTIONS: ARDUINO CONFIGURATION
//---------------------------------------------------------------
void configThisUnit()
{  
  
  //--------------------------------------------
  //       UART BAUD RATE CONFIGURATION
  //--------------------------------------------
  Serial.begin(115200);                             //RECEIVE DATA FROM UART
  //---------------------------------------------

  String filename = "12345678.123";                 //8.3 format = 8 chars for name + '.' + 3 chars for format extension.

  //Request for data
  Serial.println("LOG DATE?");                    //REQUEST FILENAME
  while( 1 )
  {
    while( !(Serial.available() > 0 ) ){}         //WAITING FOR MSG

    filename = Serial.readString();
    if( (filename.indexOf( START_CHAR ) >= 0) &&
        (filename.indexOf( END_CHAR ) > filename.indexOf( START_CHAR )) &&
        (((filename.indexOf( END_CHAR )) - filename.indexOf( START_CHAR )) <= 9) )
    {
      filename = filename.substring( filename.indexOf( START_CHAR ) + 1 , filename.indexOf( END_CHAR ) ) + ".txt";
      if( checkForInvalidChars( &filename ) ){ break; }
    }
    Serial.println("LOG DATE!");                   //REQUEST A VALID FILENAME
  }
  

  //--------------------------------------------
  //           CREATING FILE
  //--------------------------------------------
  //CREATE SD FILE MANAGER
  if( !sdCard.begin(8,SPI_HALF_SPEED) )
  {
    sdCard.initErrorHalt();
  }
  if( !todayLog.open( String(filename).c_str()  , O_RDWR | O_CREAT | O_AT_END) )
  {
    Serial.println("LOG ERR1");    
  }
  //--------------------------------------------

  Serial.println("LOG OK");
  delay( 500 );
}

void handleData()
{
  String data = "";
  if( Serial.available() > 0 )
  {
    data = Serial.readString();
    Serial.println("GOT["+data+"]");
    if( data.indexOf( END_CHAR ) >= 0 )
    {
      if( data.indexOf( END_CHAR ) == 0 )
      {
        todayLog.close();
      }
      else
      {
        data = data.substring(0,END_CHAR);
        logThisLine( data );
        todayLog.close();
      }
      Serial.println("LOG END");
    }
    else
    {
      logThisLine( data );
    }
  }
}

void logThisLine(String message)
{
  while( ( message.length() > 0 ) )
  {
    
    if( (message.indexOf( 13 ) == 0 ) or (message.indexOf( 10 ) == 0 ) )
    {
      message = message.substring( 1 );
    }
    else if(  message.indexOf( 13 ) > 0 )
    {
      String tempData = message.substring(0, message.indexOf( 13 ) );
      todayLog.println( tempData );
      Serial.println("["+tempData+"]");
      message = message.substring( message.indexOf( 13 ) ); 
    }
    else if(  message.indexOf( 10 ) > 0 )
    {
      String tempData = message.substring(0, message.indexOf( 10 ) );
      todayLog.println( tempData );
      message = message.substring( message.indexOf( 10 ) ); 
    }
    else
    {
      todayLog.print( message );
      Serial.println("["+message+"]");
      break;
    }
    delay( 50 );
  }
}

bool checkForInvalidChars(String *filename)
{
  bool valid = true;
  for( uint8_t i=0; i<CharactersInvalidForFileNameLEN; i++ )
  {
    if( (*filename).indexOf( CharactersInvalidForFileName[i] ) > -1 ){ valid = false; break; }
  }
  if( valid ){ return 1; }
  return 0;
}
