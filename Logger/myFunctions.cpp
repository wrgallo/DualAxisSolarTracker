#include "Arduino.h"
#include "myFunctions.h"
#include "myEEPROM.h"

void logThisLine(String message);
//---------------------------------------------------------------
//                  GLOBAL VARIABLES DEFINITIONS
//---------------------------------------------------------------
volatile bool timerFlag     = false;                             //PERIODIC TIMER INTERRUPT FLAG
//---------------------------------------------------------------
//                     MICRO SD MODULE DEFINITION
//---------------------------------------------------------------
SdFat sdCard;
SdFile todayLog;
String filename = "YY-MM-DD.txt";                                //8.3 format = 8 chars for name + '.' + 3 chars for format extension.
bool fileIsOpen = false;
bool stillLogging = false;

//---------------------------------------------------------------
//             FUNCTIONS: ARDUINO CONFIGURATION
//---------------------------------------------------------------
void configThisUnit()
{  
  
  //--------------------------------------------
  //       UART BAUD RATE CONFIGURATION
  //--------------------------------------------
  Serial.begin(9600);                             //RECEIVE DATA FROM UART
  //---------------------------------------------
  
  createFile();
 
}

void createFile()
{

  
  //--------------------------------------------
  //             ASKING FOR FILENAME
  //--------------------------------------------
  //Request for data
  Serial.println("LOG DATE?");                    //REQUEST FILENAME
  uint8_t k = 0;
  while( k < 255 )
  {
    k++;
    if( Serial.available() > 0 )
    {
      filename = Serial.readString();
      if( (filename.indexOf( START_CHAR )==0) && (filename.indexOf( END_CHAR ) == 9) )
      {
        filename = filename.substring( filename.indexOf( START_CHAR ) + 1 , filename.indexOf( END_CHAR ) ) + ".txt";
        Serial.println(  "[" + filename + "]" );
        if( (filename.charAt( 0 ) >= 49) && (filename.charAt( 0 ) <= 57) &&
            (filename.charAt( 1 ) >= 48) && (filename.charAt( 1 ) <= 57) &&
            (filename.charAt( 3 ) >= 48) && (filename.charAt( 3 ) <= 57) &&
            (filename.charAt( 4 ) >= 48) && (filename.charAt( 4 ) <= 57) &&
            (filename.charAt( 6 ) >= 48) && (filename.charAt( 6 ) <= 57) &&
            (filename.charAt( 7 ) >= 48) && (filename.charAt( 7 ) <= 57) )
        {
          setDate( filename );
          break;
        }
      }
    }
    Serial.println("LOG DATE!");                   //REQUEST A VALID FILENAME
    delay( 2000 );
  } 
  //--------------------------------------------
  if( k == 255 ){ filename = getDate(1); }

  
  
  //--------------------------------------------
  //           CREATING FILE
  //--------------------------------------------
  fileIsOpen = true;
  //CREATE SD FILE MANAGER
  if( !sdCard.begin(8,SPI_HALF_SPEED) )
  {
    fileIsOpen = false;
    sdCard.initErrorHalt();
  }
  if( !todayLog.open( String(filename).c_str()  , O_RDWR | O_CREAT | O_AT_END) )
  {
    fileIsOpen = false;
    Serial.println("LOG ERR1");    
  }
  //--------------------------------------------


  stillLogging = true;
  Serial.println("LOG OK");
  delay( 500 );
}


bool checkStillLogging(){
  return stillLogging;
}

void handleData()
{
  if( Serial.available() > 0 )
  {
    if( !fileIsOpen )
    {
      fileIsOpen = true;
      if( !todayLog.open( String(filename).c_str()  , O_RDWR | O_CREAT | O_AT_END) )
      {
        fileIsOpen = false;
      }
      delay( 10 );
    }

    if( fileIsOpen )
    {
      String data = "";
      while( Serial.available() > 0 )
      {
        data = Serial.readString();
        Serial.println("LRX["+data+"]");
        if( data.indexOf( END_CHAR ) >= 0 )
        {
          if( data.indexOf( END_CHAR ) == 0 )
          {
            todayLog.close();
            fileIsOpen = false;
            stillLogging = false;
          }
          else
          {
            data = data.substring(0,END_CHAR);
            logThisLine( data );
            todayLog.close();
            fileIsOpen = false;
            stillLogging = false;
          }
          Serial.println("LOG END");
        }
        else
        {
          logThisLine( data );
        }
      }
      todayLog.close();
      fileIsOpen = false;
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
      break;
    }
    delay( 50 );
  }
}
