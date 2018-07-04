#include "myEEPROM.h"

#define START_CHAR 2 //DEC NUMBER OF ASCII TABLE
#define END_CHAR   3 //DEC NUMBER OF ASCII TABLE

#define WAIT_MS 30
//byte* MSG = {START_CHAR, byteMeaning, msg_char0, ... , msg_charN, END_CHAR};

enum byteMeaning{
  byteEmailLogin = 0,
  byteEmailPswd,
  byteUserLogin,
  byteUserPswd,
  byteEmailRecipient,
  byteTimezone
};

//------------------------------------------------------------------------------------------
//                                     private functions
//------------------------------------------------------------------------------------------
String lookFor( byteMeaning byteMeaningValue );
void writeThis( byteMeaning byteMeaningValue, String stringToWrite   );
uint16_t getGoodPosition( byteMeaning byteMeaningValue  );

String lookFor( byteMeaning byteMeaningValue ){
  EEPROM.begin(EEPROM_LENGTH);delay(WAIT_MS);
  String output = "";
  uint16_t address = 0;
  byte value = 0;
  bool founded = false;
  
  //FINDING STRING
  while( (address < EEPROM_LENGTH) and (!founded) ){
    value = EEPROM.read(address++);delay(WAIT_MS);
    
    if( address == EEPROM_LENGTH ){
      output = "EEPROM Data Not Found";
      Serial.println();
      Serial.println(output);
      return output;
    }

    //FINDING START_CHAR
    if( value == START_CHAR ){
      value = EEPROM.read(address++);delay(WAIT_MS);
      
      //THIS IS THE CORRECT START_CHAR
      if( value == byteMeaningValue ){
        
        //FINDING THE REST OF THE MSG
        while( (address < EEPROM_LENGTH) ){
          value = EEPROM.read(address++);delay(WAIT_MS);
          if( value == END_CHAR ){
            founded = true;
            break;
          }
          else if( (value > 31) and (value < 127) ){
            output += (char)(value);
          }
        }
      }
    }
  }
  EEPROM.end();delay(WAIT_MS);
  return output;
}

void writeThis( byteMeaning byteMeaningValue, String stringToWrite   ){
  uint16_t address;
  address = getGoodPosition( byteMeaningValue  );
  
  if( address < EEPROM_LENGTH ){
    int maxLength = 0;
    
    if(       byteMeaningValue == byteEmailLogin     ){ maxLength = MAX_EMAIL_LOGIN_LENGTH; }
    else if(  byteMeaningValue == byteEmailPswd      ){ maxLength = MAX_EMAIL_PSWD_LENGTH; }
    else if(  byteMeaningValue == byteUserLogin      ){ maxLength = MAX_USER_CREDENTIALS_LOGIN_LENGTH; }
    else if(  byteMeaningValue == byteUserPswd       ){ maxLength = MAX_USER_CREDENTIALS_PSWD_LENGTH; }
    else if(  byteMeaningValue == byteEmailRecipient ){ maxLength = MAX_EMAIL_LOGIN_LENGTH; }
    else if(  byteMeaningValue == byteTimezone       ){ maxLength = MAX_TIMEZONE_LEN; }
    
    if( stringToWrite.length() <= maxLength ){
      
      EEPROM.begin( EEPROM_LENGTH );delay(WAIT_MS);
      
      EEPROM.write(address++, START_CHAR       );delay(WAIT_MS);
      EEPROM.write(address++, byteMeaningValue );delay(WAIT_MS);
      int i;
      for( i=0; i < stringToWrite.length(); i++ ){
        EEPROM.write( address++, stringToWrite.charAt(i) );delay(WAIT_MS);
      }
      EEPROM.write(address++, END_CHAR );delay(WAIT_MS);
      EEPROM.commit();delay(WAIT_MS);
      EEPROM.end();delay(WAIT_MS);
    }
    else{
      Serial.println();
      Serial.print("String too long");
    }
    
  }
}

uint16_t getGoodPosition( byteMeaning byteMeaningValue  ){
  /*
   * byteEmailLogin = 0,
   * byteEmailPswd,
   * byteUserLogin,
   * byteUserPswd,
   * byteTimezone
   */
  
  if(       byteMeaningValue == byteEmailLogin     ){ return 0; }
  else if(  byteMeaningValue == byteEmailPswd      ){ return (3 + MAX_EMAIL_LOGIN_LENGTH); }
  else if(  byteMeaningValue == byteUserLogin      ){ return (3*2 + MAX_EMAIL_LOGIN_LENGTH + MAX_EMAIL_PSWD_LENGTH); }
  else if(  byteMeaningValue == byteUserPswd       ){ return (3*3 + MAX_EMAIL_LOGIN_LENGTH + MAX_EMAIL_PSWD_LENGTH + MAX_USER_CREDENTIALS_LOGIN_LENGTH); }
  else if(  byteMeaningValue == byteEmailRecipient ){ return (3*4 + MAX_EMAIL_LOGIN_LENGTH + MAX_EMAIL_PSWD_LENGTH + MAX_USER_CREDENTIALS_LOGIN_LENGTH + MAX_USER_CREDENTIALS_PSWD_LENGTH); }
  else if(  byteMeaningValue == byteTimezone       ){ return (3*5 + MAX_EMAIL_LOGIN_LENGTH + MAX_EMAIL_PSWD_LENGTH + MAX_USER_CREDENTIALS_LOGIN_LENGTH + MAX_USER_CREDENTIALS_PSWD_LENGTH + MAX_EMAIL_LOGIN_LENGTH); }
  
  return EEPROM_LENGTH;
}
//------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------
//                                     public functions
//------------------------------------------------------------------------------------------
void setThisByteToAll( byte value ){
  EEPROM.begin( EEPROM_LENGTH );delay(WAIT_MS);
  uint16_t address=0;
  
  while( (address < EEPROM_LENGTH) ){
    EEPROM.write(address++, value );delay(WAIT_MS);
  }
  
  EEPROM.commit();delay(WAIT_MS);
  EEPROM.end();delay(WAIT_MS);
}

void readEEPROM(bool detailed){
  uint16_t address = 0;
  byte value;
  EEPROM.begin( EEPROM_LENGTH ); delay( WAIT_MS );

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
  
  EEPROM.end();delay(WAIT_MS);
}

//-------------------------------------------
//               get functions
//-------------------------------------------

String getEmailLogin(void){
  return lookFor( byteEmailLogin );
}

String getEmailPassword(void){
  return lookFor( byteEmailPswd );
}

String getUserCredentialsLogin(void){
  return lookFor( byteUserLogin );
}

String getUserCredentialsPassword(void){
  return lookFor( byteUserPswd );
}

String getEmailRecipient(void){
  return lookFor( byteEmailRecipient );
}

int8_t getTimezone(void){
  return ((lookFor( byteTimezone )).toInt()) - 12 - END_CHAR;
}
//-------------------------------------------


//-------------------------------------------
//               set functions
//-------------------------------------------
void setEmailLogin(String login){
  writeThis( byteEmailLogin , login   );
}

void setEmailPassword(String password){
  writeThis( byteEmailPswd , password   );
}

void setUserCredentialsLogin(String user){
  writeThis( byteUserLogin , user   );
}

void setUserCredentialsPassword(String password){
  writeThis( byteUserPswd , password   );
}

void setEmailRecipient(String mail){
  writeThis( byteEmailRecipient , mail );
}

void setTimezone(int8_t timezone){
  timezone += 12 + END_CHAR; //0 means "UTC-12", 24 means "UTC+12"
  if( (timezone>=0) and (timezone<=12) ){
    writeThis( byteTimezone , (String)(timezone) );
  }
}
//-------------------------------------------

//------------------------------------------------------------------------------------------
