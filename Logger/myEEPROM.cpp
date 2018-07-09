#include "myEEPROM.h"

#define START_CHAR 2 //DEC NUMBER OF ASCII TABLE
#define END_CHAR   3 //DEC NUMBER OF ASCII TABLE

#define WAIT_MS 30

//------------------------------------------------------------------------------------------
//                                     public functions
//------------------------------------------------------------------------------------------
void setThisByteToAll( byte value ){
  uint16_t address=0;
  
  while( (address < EEPROM_LENGTH) ){
    EEPROM.write(address++, value );delay(WAIT_MS);
  }
}

void readEEPROM(bool detailed){
  uint16_t address = 0;
  byte value;

  if( detailed ){
    Serial.println();
    Serial.println("------------------------------------");
    Serial.println("ADDR\tDEC\tCHAR");
  }
  while( address < EEPROM_LENGTH ){
    // read a byte from the current address of the EEPROM
    value = EEPROM.read(address);

    if( detailed ){
      delay( 100 );
      Serial.print(address);
      Serial.print("\t");
      Serial.print(value, DEC);
      Serial.print("\t");
    }else{
      delay( WAIT_MS );
    }
    
    Serial.write(value);
    
    if( detailed ){Serial.println();}
  
    // advance to the next address of the EEPROM
    address = address + 1;
  }
}

void setDate(String filename )
{
  String strYear, strMonth, strDay;
  strYear  = filename.substring(0 , filename.indexOf('-')     );
  strMonth = filename.substring(    filename.indexOf('-') + 1 );
  strMonth = strMonth.substring(0 , strMonth.indexOf('-')     );
  strDay   = filename.substring(    filename.indexOf('-') + 1 );
  strDay   =   strDay.substring(      strDay.indexOf('-') + 1 );
  strDay   =   strDay.substring(0 ,   strDay.indexOf('.')     );

  EEPROM.write( 0 ,  strYear.toInt() );delay( WAIT_MS );
  EEPROM.write( 1 , strMonth.toInt() );delay( WAIT_MS );
  EEPROM.write( 2 ,   strDay.toInt() );delay( WAIT_MS );
}

String getDate(bool nextDay)
{
  String tDay, tMon, tYea;
  uint8_t temp0, temp1, temp2;
  temp0 = EEPROM.read(0);delay( WAIT_MS );
  temp1 = EEPROM.read(1);delay( WAIT_MS );
  temp2 = EEPROM.read(2);delay( WAIT_MS );

  if( nextDay )
  {
    temp2++;
    if( temp2 > 31 ){
      Serial.println("INVALID DATE!");
      temp2 = 1;
      temp1++;
      if( temp1 > 12 ){
        temp1 = 1;
        temp0++;
      }
    }
  }
  
  if( temp0 < 10 ){ tYea = String("0") + String( temp0 ); }
  else{             tYea =               String( temp0 ); }
  if( temp1 < 10 ){ tMon = String("0") + String( temp1 ); }
  else{             tMon =               String( temp1 ); }
  if( temp2 < 10 ){ tDay = String("0") + String( temp2 ); }
  else{             tDay =               String( temp2 ); }
  return String( tYea + "-" + tMon + "-" + tDay + ".txt" );
}

//-------------------------------------------

//------------------------------------------------------------------------------------------
