#include "./serverFunc.h"

void timerConfig();
void timerHandler(void *tCall);
void data2Mail();


//---------------------------------------------------------------
//                  OPERATION MODE SELECT
//---------------------------------------------------------------
/*
 * DEFINE THE OPERATION MODE
 * operationMode = 0; //SHORT_CIRCUIT_TEST
 * operationMode = 1; //POWER_TEST
 * operationMode = 2; //OPEN_CIRCUIT_TEST
 * 
 * SHORT_CIRCUIT_TEST         
        Shunt Resistor of INA219 is the only Load on Solar Panel
        Data Useful: Current [mA], Current Generated [mAh]
        
 * POWER_TEST
        A specific load is applyed to the Solar Panel.
        Data Useful: Load Voltage [V], Current [mA], Power [mW], Power Generated [mWh]
          *Shunt Voltage == Shunt Resistor * Current
          *Load Voltage  == Bus Voltage - Shunt Voltage

 * OPEN_CIRCUIT_TEST
        No load is applyed to the Solar Panel, Voltage is measured with ADC with Common Ground.
        Data Useful: Bus Voltage [V]
*/
uint8_t operationMode = 1;
//---------------------------------------------------------------



//---------------------------------------------------------------------------------------
//                                      INTERNAL VARIABLES
//---------------------------------------------------------------------------------------
os_timer_t mTimer;
ESP8266WebServer myServer(80);        //Server object
volatile bool wifiConnected = false;  //indicates WiFi Connection Status
volatile uint8_t MCU_timeCounter = 90;//Counter of seconds without a MCU UART Answer
bool    stillBuffering = false;       //if there is a msg incomming
char    buf1[200];                    //to get the bytes out of UART buffer with only expected UART values
String  buffer_read = "";             //All msgs in current buffer
int     msgLen = 0;                   //Lenght of buffer_read 
uint8_t i = 0, j = 0, bufLen = 0;     //two counters and the Length of buf1
String  EEPROM_USER;                  //User Credential Username at EEPROM
String  EEPROM_PSWD;                  //User Credential Password at EEPROM
int8_t  EEPROM_TIMEZONE = 0;          //For Online Updating Time
//---------------------------------------------------------------------------------------


//---------------------------------------------------------------------------------------
//                            WEBSITE.H VARIABLES AND FUNCTIONS
//---------------------------------------------------------------------------------------
String websiteHTML,loginHtml,XML,Style,Footer, mailData;

/*
String var01_HTML              = "Unknown";
String var02_motionSensorState = "Unknown";
String var03_buzzerState       = "Unknown";
String var04_time              = "HH:MM";
String var05_date              = "DD/MM/YY";
String var06_weekday           = "Unknown";
String var07_alarmTime         = "HH:MM";
String var08_alarmDay          = "Unknown";
String var09_alarmSet          = "Unknown";
String var10_nightTimeStart    = "HH:MM";
String var11_nightTimeEnd      = "HH:MM";
String var12_nightTimeSet      = "Unknown";
String var13_mailRecipient     = "Unknown";
String var15_masterUnit        = "OFFLINE";
String var16_slave1            = "OFFLINE";
String var17_slave2            = "OFFLINE";
String var18_slave3            = "OFFLINE";
 */

//SOLAR TRACKER SYSTEMS INFORMATION - DEFAULT VALUES
String var01_HTML = "OFFLINE";  //MASTER UNIT STATUS
String var02_HTML = "OFFLINE";  //PROTOTYPE UNIT STATUS
String var03_HTML = "HH:MM";    //TIME
String var04_HTML = "DD/MM/YY"; //DATE
String var05_HTML = "Unknown";  //WEEKDAY
String var06_HTML = "Unknown";  //Email Recipient
//Solar Tracker Unit Data
String var07_HTML = "Unknown"; //Solar Tracker Unit - (Power Delivered Today)
String var08_HTML = "Unknown"; //Solar Tracker Unit - (Voltage [V])
String var09_HTML = "Unknown"; //Solar Tracker Unit - (Currnte [mA])
String var10_HTML = "Unknown"; //Solar Tracker Unit - (Power [mW])
String var11_HTML = "Unknown"; //Solar Tracker Unit - (Temperature [ºC])
String var12_HTML = "Unknown"; //Solar Tracker Unit - (Lux [lx])
String var13_HTML = "Unknown"; //Solar Tracker Unit - (Photo UL [mV])
String var14_HTML = "Unknown"; //Solar Tracker Unit - (Photo UR [mV])
String var15_HTML = "Unknown"; //Solar Tracker Unit - (Photo DL [mV])
String var16_HTML = "Unknown"; //Solar Tracker Unit - (Photo DR [mV])
String var17_HTML = "Unknown"; //Solar Tracker Unit - (Pitch Angle [º])
String var18_HTML = "Unknown"; //Solar Tracker Unit - (Yaw Angle [º])
//Control Unit Data
String var19_HTML = "Unknown"; //Control Unit - (Power Delivered Today)
String var20_HTML = "Unknown"; //Control Unit - (Voltage [V])
String var21_HTML = "Unknown"; //Control Unit - (Currnte [mA])
String var22_HTML = "Unknown"; //Control Unit - (Power [mW])
String var23_HTML = "Unknown"; //Control Unit - (Temperature [ºC])
String var24_HTML = "Unknown"; //Control Unit - (Lux [lx])
String var25_HTML = "Unknown"; //Control Unit - (Photo UL [mV])
String var26_HTML = "Unknown"; //Control Unit - (Photo UR [mV])
String var27_HTML = "Unknown"; //Control Unit - (Photo DL [mV])
String var28_HTML = "Unknown"; //Control Unit - (Photo DR [mV])
String var29_HTML = "Unknown"; //Control Unit - (Pitch Angle [º])
String var30_HTML = "Unknown"; //Control Unit - (Yaw Angle [º])

String var31_HTML = "Unknown"; //Solar Tracker milliampere Hour (mAh)
String var32_HTML = "Unknown"; //Control Unit  milliampere Hour (mAh).



//---------------------------------------------------------------------------------------
//                                     ESP Setup and Control
//---------------------------------------------------------------------------------------
void startESP(void){

  
  //-------------------------------------------------------------------------------------
  // WAIT 20 SECONDS BEFORE START, IF IT MAY RESET FOR ANY REASON, IT SHOULD HAPPEN NOW.
  delay(20000);
  //-------------------------------------------------------------------------------------


  //-------------------------------------------------------------------------------------
  //                               CONFIGURING UART CONNECTION
  Serial.begin(115200);  
  //-------------------------------------------------------------------------------------


  //TIMER TO AUTO RESET IF ESP8266 TOOK TOO LONG IN ACCESS POINT
  timerConfig();
  //-------------------------------------------------------------------------------------
  //                        CONNECTING TO WIFI, OR CREATING ACCESS POINT
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  
  //tries to connect to last known settings
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP" with password "password"
  //and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect( ESP_SSID , ESP_PSWD )) {
    //PROBLEM OCCURED DURING CONNECTION TO EXISTING WIFI
    Serial.write( START_CHAR );Serial.print("IP00");Serial.write( END_CHAR );
    delay(3000);
    ESP.reset();
    delay(5000);
  }
  //-------------------------------------------------------------------------------------


  //-------------------------------------------------------------------------------------
  //                            IF LOST CONNECTION, CONNECT AGAIN
  WiFi.setAutoReconnect(true);
  //-------------------------------------------------------------------------------------


  //-------------------------------------------------------------------------------------
  //                                    BUILDING STRINGS
  buildOnce();
  
  var06_HTML          = getEmailRecipient();
  EEPROM_USER         = getUserCredentialsLogin();
  EEPROM_PSWD         = getUserCredentialsPassword();
  EEPROM_TIMEZONE     = getTimezone();
  //-------------------------------------------------------------------------------------
  
  
  startServer();  

  //-------------------------------------------------------------------------------------
  //               WAIT 10 SECONDS BEFORE WE CONCLUDE IT STARTED PERFECTLY
  delay(10000);
  //-------------------------------------------------------------------------------------


  //-------------------------------------------------------------------------------------
  //                                    REPORT IP ADDRESS
  if(WiFi.status() == WL_CONNECTED){
    //SUCCESSFULLY CONNECTED TO EXISTING WIFI WITHOUT SELF-REBOOTING
    //RETURNING LOCAL IP ADDRESS
    Serial.write( START_CHAR );Serial.print("WEBOK");Serial.write( END_CHAR );
    Serial.write( START_CHAR );Serial.print("IP0");Serial.print( WiFi.localIP() );Serial.write( END_CHAR );
  }else{
    //PROBLEM OCCURED DURING CONNECTION TO EXISTING WIFI
    Serial.write( START_CHAR );Serial.print("IP00");Serial.write( END_CHAR );
    ESP.reset();
  }
  //-------------------------------------------------------------------------------------

  wifiConnected = true;

  sendOnlineTime();
  
}

void sendOnlineTime(){
  //--------------------------------------------------------------------------------------------------------------
  //                                    GET ONLINE TIME
  uint8_t i;
  for(i=0; i<10; i++){
    if( !requestUTC( EEPROM_TIMEZONE ) ){
      Serial.print("\nNTP Error");
    }else{ i = 0; break; }
    i++;
    delay(1000);
  }
  if( i == 0 ){
    Serial.write(START_CHAR);Serial.print( "WTH" + (String)( getHour()    ) );Serial.write(END_CHAR);//TIME HOUR
    Serial.write(START_CHAR);Serial.print( "WTM" + (String)( getMinute()  ) );Serial.write(END_CHAR);//TIME MINUTE
    Serial.write(START_CHAR);Serial.print( "WDW" + (String)( getWeekday() ) );Serial.write(END_CHAR);//WEEKDAY
    Serial.write(START_CHAR);Serial.print( "WDD" + (String)( getDay()     ) );Serial.write(END_CHAR);//DAY
    Serial.write(START_CHAR);Serial.print( "WDM" + (String)( getMonth()   ) );Serial.write(END_CHAR);//MONTH
    Serial.write(START_CHAR);Serial.print( "WDY" + (String)( getYear()    ) );Serial.write(END_CHAR);//YEAR

    if( getHour() == 19 ){ data2Mail(); }
  }
  //---------------------------------------------------------------------------------------------------------------
}

void startServer(){
  //-------------------------------------------------------------------------------------
  //                                  SERVER WEBPAGE HANDLERS
  myServer.onNotFound(handleNotFound);
  myServer.on("/xml"              , handleXML);
  myServer.on("/login"            , handleLogin);
  myServer.on("/"                 , handleRoot);
  //myServer.on("/setOPM"           , handleRoot);
  myServer.on("/setTime"          , handleRoot);
  myServer.on("/setNewCredential" , handleRoot);
  myServer.on("/setMailRecipient" , handleRoot);
  myServer.on("/setTimezone"      , handleRoot);
  //-------------------------------------------------------------------------------------


  //-------------------------------------------------------------------------------------
  //                                    COOKIE CONFIGURATION
  //The list of headers to be recorded
  const char * headerkeys[] = {"User-Agent","Cookie"} ;
  size_t headerkeyssize = sizeof(headerkeys)/sizeof(char*);
  //ask server to track these headers
  myServer.collectHeaders(headerkeys, headerkeyssize );
  myServer.begin();
  if(DEBUG_PRINTING){ Serial.println("[DEBUG] HTTP server started"); }
  //-------------------------------------------------------------------------------------
}

bool checkCredentials(String user, String pswd){
  if( ((user == ADMIN_NAME ) and (pswd == ADMIN_PSWD )) or
      ((user == EEPROM_USER) and (pswd == EEPROM_PSWD)) )
  {
      return true; 
  }
  return false;
}

void timerConfig(){
    os_timer_setfn(&mTimer, timerHandler, NULL);
    os_timer_arm(  &mTimer, 1000        , true);
}

void timerHandler(void *tCall){
  static uint16_t timeCounter = 0;
  
  if( !wifiConnected ){
    timeCounter++;
    
    if( timeCounter >= 600 ){
      timeCounter = 0;
      Serial.println("Timer Reset");
      ESP.restart();
    }
  }else{
    timeCounter = 0;
  }

  if( MCU_timeCounter < 70 ){
    MCU_timeCounter++;
    if( MCU_timeCounter == 70 ){
      var01_HTML = "OFFLINE";
    }
  }

}

bool is_authentified(){
  //Check if header is present and correct
  if(DEBUG_PRINTING){ Serial.println("[DEBUG] Enter is_authentified"); }
  if (myServer.hasHeader("Cookie")){
    String cookie = myServer.header("Cookie");
    if(DEBUG_PRINTING){ Serial.print("[DEBUG] Found cookie: "); }
    if(DEBUG_PRINTING){ Serial.println(cookie); }
    if (cookie.indexOf("ESPSESSIONID=1") != -1) {
      if(DEBUG_PRINTING){ Serial.println("[DEBUG] Authentification Successful"); }
      
      return true;
    }
  }
  if(DEBUG_PRINTING){ Serial.println("[DEBUG] Authentification Failed"); }
  return false;
}

void handleServer(void){
  if(WiFi.status() != WL_CONNECTED){ wifiConnected = false; }
  else{                              wifiConnected = true; }
  
  myServer.handleClient();
}


void handleMaster(void){

  if(Serial.available() > 0){
    stillBuffering = false;
    //------------------------------------------------------------
    //     READING UART VALUES AS VALID ASCII CHARACTERS ONLY
    //------------------------------------------------------------
    Serial.setTimeout(10);
    bufLen = Serial.readBytes(buf1 , 200);buf1[bufLen] = '\0';
    j=0;
    for(i=0; i<bufLen; i++){ 
      if( ( buf1[i] ==  START_CHAR ) or ( buf1[i] == END_CHAR ) or (( buf1[i]  > 32 ) && ( buf1[i]  < 127 )) ){ 
        buf1[j++] = buf1[i];
      } 
    }
    buf1[j] = '\0';

    buffer_read += buf1;
    msgLen = buffer_read.length();
    //------------------------------------------------------------
  }


  if( (msgLen > 0) and (stillBuffering == false) ){
    
    //FLUSHING CHARS UNTILL A VALID START CHAR IS FOUND
    while( (buffer_read.indexOf( START_CHAR ) != 0) and (msgLen > 0) ){
      buffer_read = buffer_read.substring(1);
      msgLen--;
    }
    if(DEBUG_PRINTING){ Serial.println( buffer_read ); }
    
    //WORD RECEIVED STARTS WITH A VALID START CHAR
    if( buffer_read.indexOf( START_CHAR ) == 0 ){
      stillBuffering = true;
      
      //WORD RECEIVED HAS THE VALID END CHAR
      int iterationCounter = 0;
      while( ( buffer_read.indexOf( END_CHAR ) >= 4 ) and (iterationCounter < 10) ) {
        iterationCounter++;
        stillBuffering = false;

        var01_HTML = "ONLINE";MCU_timeCounter = 0;
        
        //-----------------------------------------------------------------------------------------------
        //                                     CMDS THAT DO NOT NEED ANSWERS
        //-----------------------------------------------------------------------------------------------
        //GETTING V##_HTML DATA
        if( buffer_read.indexOf( 'V' ) == 1 ){
          if(      buffer_read.substring( 2,4 ) == "07" ){ var07_HTML = buffer_read.substring( 4, buffer_read.indexOf( END_CHAR ) ) + " [mWh]"; }
          else if( buffer_read.substring( 2,4 ) == "08" ){ var08_HTML = buffer_read.substring( 4, buffer_read.indexOf( END_CHAR ) ) + " [V]";  }
          else if( buffer_read.substring( 2,4 ) == "09" ){ var09_HTML = buffer_read.substring( 4, buffer_read.indexOf( END_CHAR ) ) + " [mA]"; }
          else if( buffer_read.substring( 2,4 ) == "10" ){ var10_HTML = buffer_read.substring( 4, buffer_read.indexOf( END_CHAR ) ) + " [mW]"; }
          else if( buffer_read.substring( 2,4 ) == "11" ){ var11_HTML = buffer_read.substring( 4, buffer_read.indexOf( END_CHAR ) ) + " [ºC]"; }
          else if( buffer_read.substring( 2,4 ) == "12" ){ var12_HTML = buffer_read.substring( 4, buffer_read.indexOf( END_CHAR ) ) + " [lx]"; }
          else if( buffer_read.substring( 2,4 ) == "13" ){ var13_HTML = buffer_read.substring( 4, buffer_read.indexOf( END_CHAR ) ) + " [mV]"; }
          else if( buffer_read.substring( 2,4 ) == "14" ){ var14_HTML = buffer_read.substring( 4, buffer_read.indexOf( END_CHAR ) ) + " [mV]"; }
          else if( buffer_read.substring( 2,4 ) == "15" ){ var15_HTML = buffer_read.substring( 4, buffer_read.indexOf( END_CHAR ) ) + " [mV]"; }
          else if( buffer_read.substring( 2,4 ) == "16" ){ var16_HTML = buffer_read.substring( 4, buffer_read.indexOf( END_CHAR ) ) + " [mV]"; }
          else if( buffer_read.substring( 2,4 ) == "17" ){ var17_HTML = buffer_read.substring( 4, buffer_read.indexOf( END_CHAR ) ) + " [DEG]";  }
          else if( buffer_read.substring( 2,4 ) == "18" ){ var18_HTML = buffer_read.substring( 4, buffer_read.indexOf( END_CHAR ) ) + " [DEG]";  }
          if(      buffer_read.substring( 2,4 ) == "19" ){ var19_HTML = buffer_read.substring( 4, buffer_read.indexOf( END_CHAR ) ) + " [mWh]"; }
          else if( buffer_read.substring( 2,4 ) == "20" ){ var20_HTML = buffer_read.substring( 4, buffer_read.indexOf( END_CHAR ) ) + " [V]";  }
          else if( buffer_read.substring( 2,4 ) == "21" ){ var21_HTML = buffer_read.substring( 4, buffer_read.indexOf( END_CHAR ) ) + " [mA]"; }
          else if( buffer_read.substring( 2,4 ) == "22" ){ var22_HTML = buffer_read.substring( 4, buffer_read.indexOf( END_CHAR ) ) + " [mW]"; }
          else if( buffer_read.substring( 2,4 ) == "23" ){ var23_HTML = buffer_read.substring( 4, buffer_read.indexOf( END_CHAR ) ) + " [C]"; }
          else if( buffer_read.substring( 2,4 ) == "24" ){ var24_HTML = buffer_read.substring( 4, buffer_read.indexOf( END_CHAR ) ) + " [lx]"; }
          else if( buffer_read.substring( 2,4 ) == "25" ){ var25_HTML = buffer_read.substring( 4, buffer_read.indexOf( END_CHAR ) ) + " [mV]"; }
          else if( buffer_read.substring( 2,4 ) == "26" ){ var26_HTML = buffer_read.substring( 4, buffer_read.indexOf( END_CHAR ) ) + " [mV]"; }
          else if( buffer_read.substring( 2,4 ) == "27" ){ var27_HTML = buffer_read.substring( 4, buffer_read.indexOf( END_CHAR ) ) + " [mV]"; }
          else if( buffer_read.substring( 2,4 ) == "28" ){ var28_HTML = buffer_read.substring( 4, buffer_read.indexOf( END_CHAR ) ) + " [mV]"; }
          else if( buffer_read.substring( 2,4 ) == "29" ){ var29_HTML = buffer_read.substring( 4, buffer_read.indexOf( END_CHAR ) ) + " [DEG]";  }
          else if( buffer_read.substring( 2,4 ) == "30" ){ var30_HTML = buffer_read.substring( 4, buffer_read.indexOf( END_CHAR ) ) + " [DEG]";  }
          else if( buffer_read.substring( 2,4 ) == "31" ){ var31_HTML = buffer_read.substring( 4, buffer_read.indexOf( END_CHAR ) ) + " [mAh]";  }
          else if( buffer_read.substring( 2,4 ) == "32" ){ var32_HTML = buffer_read.substring( 4, buffer_read.indexOf( END_CHAR ) ) + " [mAh]";  }
        }

        if( ( buffer_read.substring(1,4) == "OPM" ) && ( buffer_read.indexOf( END_CHAR ) == 5 ) ){
          operationMode = (int)( buffer_read.charAt(4) );
          if( operationMode > 47 ){ operationMode -= 48; } //JUST IN CASE IT COMES AS A ASCII INT, INSTEAD OF A DEC VALUE
        }
        
        //TIME
        else if( ( buffer_read.substring(1,4) == "MT1" ) && ( buffer_read.indexOf( END_CHAR ) >= 7 ) && ( buffer_read.indexOf( END_CHAR ) <= 9 ) ){
          if(      buffer_read.indexOf(":") == 5 ){ 
            var03_HTML  = "0" + buffer_read.substring( 4 , 6 );
            if(    buffer_read.indexOf( END_CHAR ) == 7 ){ var03_HTML  += "0" + buffer_read.substring( 6 , 7 ); }
            else{                                          var03_HTML  +=       buffer_read.substring( 6 , 8 ); }
          }
          else if( buffer_read.indexOf(":") == 6 ){
            var03_HTML  =       buffer_read.substring( 4 , 7 );
            if(    buffer_read.indexOf( END_CHAR ) == 8 ){ var03_HTML  += "0" + buffer_read.substring( 7 , 8 ); }
            else{                                          var03_HTML  +=       buffer_read.substring( 7 , 9 ); }
            if( buffer_read.substring( 4,7 ) == "19" ){
              data2Mail();
            }
          }
          
        }

        //DATE
        else if( ( buffer_read.substring(1,4) == "MT2" ) && ( buffer_read.indexOf( END_CHAR ) >= 9 ) && ( buffer_read.indexOf( END_CHAR ) <= 12 ) ){
          var04_HTML = buffer_read.substring( 4 , buffer_read.indexOf( END_CHAR ) );
        }

        //DAY OF THE WEEK
        else if( ( buffer_read.substring(1,4) == "MT3" ) && ( buffer_read.indexOf( END_CHAR ) == 5 ) ){
          if(      buffer_read.substring(4,5) == "0" ){ var05_HTML = "Sunday";   }
          else if( buffer_read.substring(4,5) == "1" ){ var05_HTML = "Monday";   }
          else if( buffer_read.substring(4,5) == "2" ){ var05_HTML = "Tuesday";  }
          else if( buffer_read.substring(4,5) == "3" ){ var05_HTML = "Wednesday";}
          else if( buffer_read.substring(4,5) == "4" ){ var05_HTML = "Thursday"; }
          else if( buffer_read.substring(4,5) == "5" ){ var05_HTML = "Friday";   }
          else if( buffer_read.substring(4,5) == "6" ){ var05_HTML = "Saturday"; }
        }

        //REQUEST SOFTWARE RESTART
        else if( ( buffer_read.substring(1,5) == "RST1" ) && ( buffer_read.indexOf( END_CHAR ) == 5 ) ){ 
          ESP.restart();
        }

        //REQUEST SOFTWARE RESET
        else if( ( buffer_read.substring(1,5) == "RST2" ) && ( buffer_read.indexOf( END_CHAR ) == 5 ) ){ 
          ESP.reset();
        }

        //REQUEST ONLINE TIME
        else if( ( buffer_read.substring(1,4) == "RTO" ) && ( buffer_read.indexOf( END_CHAR ) == 4 ) ){ 
          sendOnlineTime();
        }
        
        //REPORT SLAVE STATUS
        else if( (   buffer_read.substring(1,4) == "MSS" ) && ( buffer_read.indexOf( END_CHAR ) == 6 ) ){ 
          String temp = "";
          if(      buffer_read.substring(5,6) == "0" ){ temp = "OFFLINE"; }
          else if( buffer_read.substring(5,6) == "1" ){ temp = "ONLINE";  }
          
          if(      buffer_read.substring(4,5) == "1" ){ var02_HTML = temp; }
        }

        //REPORT ALERT
        else if( ( buffer_read.substring(1,4) == "MRP" ) && ( buffer_read.indexOf( END_CHAR ) == 5 ) ){ 
          if( buffer_read.substring(4,5) == "1" ){
            String MailContent = "<html>\n<body>\n<b>MCU Rebooted</b> at <i>" + var03_HTML + " of " + var04_HTML + "</i>\n</body>\n</html>";
            sendEmail( getEmailLogin(), getEmailPassword(), getEmailRecipient(), "!MCU Reset!" , &MailContent );
          }
        }

        //-----------------------------------------------------------------------------------------------
        //                                          CMDS THAT NEED ANSWERS
        //-----------------------------------------------------------------------------------------------
        //REQUEST IP ADDRESS
        else if( ( buffer_read.substring(1,4) == "RIP" ) && ( buffer_read.indexOf( END_CHAR ) == 5 ) ){
          
          //REQUEST WIFI IP ADDRESS
          if(      buffer_read.substring(4,5) == "0" ){
            if(WiFi.status() == WL_CONNECTED){
              //RETURNING WF IP ADDRESS
              Serial.write( START_CHAR );Serial.print("IP0");Serial.print( WiFi.localIP() );Serial.write( END_CHAR );
            }else{
              //PROBLEM OCCURED DURING CREATION OF AN ACCESS POINT
              Serial.write( START_CHAR );Serial.print("IP00");Serial.write( END_CHAR );
            }
          }
          
        }
        
      //DELETING THE PART OF THE MSG THAT HAS ALREADY BEEN READ
      if( buffer_read.indexOf( END_CHAR ) > 0 ){ buffer_read = buffer_read.substring( buffer_read.indexOf( END_CHAR ) + 1 );msgLen = buffer_read.length(); }
      
      }
    }
  }
}
//---------------------------------------------------------------------------------------






//---------------------------------------------------------------------------------------
//                                       PAGES HANDLERS
//---------------------------------------------------------------------------------------

void handleNotFound(){
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += myServer.uri();
  message += "\nMethod: ";
  message += (myServer.method() == HTTP_GET)?"GET":"POST";
  message += "\nArguments: ";
  message += myServer.args();
  message += "\n";
  for (uint8_t i=0; i<myServer.args(); i++){
    message += " " + myServer.argName(i) + ": " + myServer.arg(i) + "\n";
  }
  myServer.send(404, "text/plain", message);
}

void handleXML(){
  XML="<?xml version='1.0'?>";
  XML+="<response>\n";
  XML+="  <rvar01>" + var01_HTML + "</rvar01>\n";
  XML+="  <rvar02>" + var02_HTML + "</rvar02>\n";
  XML+="  <rvar03>" + var03_HTML + "</rvar03>\n";
  XML+="  <rvar04>" + var04_HTML + "</rvar04>\n";
  XML+="  <rvar05>" + var05_HTML + "</rvar05>\n";
  XML+="  <rvar06>" + var06_HTML + "</rvar06>\n";
  XML+="  <rvar07>" + var07_HTML + "</rvar07>\n";
  XML+="  <rvar08>" + var08_HTML + "</rvar08>\n";
  XML+="  <rvar09>" + var09_HTML + "</rvar09>\n";
  XML+="  <rvar10>" + var10_HTML + "</rvar10>\n";
  XML+="  <rvar11>" + var11_HTML + "</rvar11>\n";
  XML+="  <rvar12>" + var12_HTML + "</rvar12>\n";
  XML+="  <rvar13>" + var13_HTML + "</rvar13>\n";
  XML+="  <rvar14>" + var14_HTML + "</rvar14>\n";
  XML+="  <rvar15>" + var15_HTML + "</rvar15>\n";
  XML+="  <rvar16>" + var16_HTML + "</rvar16>\n";
  XML+="  <rvar17>" + var17_HTML + "</rvar17>\n";
  XML+="  <rvar18>" + var18_HTML + "</rvar18>\n";
  XML+="  <rvar19>" + var19_HTML + "</rvar19>\n";
  XML+="  <rvar20>" + var20_HTML + "</rvar20>\n";
  XML+="  <rvar21>" + var21_HTML + "</rvar21>\n";
  XML+="  <rvar22>" + var22_HTML + "</rvar22>\n";
  XML+="  <rvar23>" + var23_HTML + "</rvar23>\n";
  XML+="  <rvar24>" + var24_HTML + "</rvar24>\n";
  XML+="  <rvar25>" + var25_HTML + "</rvar25>\n";
  XML+="  <rvar26>" + var26_HTML + "</rvar26>\n";
  XML+="  <rvar27>" + var27_HTML + "</rvar27>\n";
  XML+="  <rvar28>" + var28_HTML + "</rvar28>\n";
  XML+="  <rvar29>" + var29_HTML + "</rvar29>\n";
  XML+="  <rvar30>" + var30_HTML + "</rvar30>\n";
  XML+="  <rvar31>" + var31_HTML + "</rvar31>\n";
  XML+="  <rvar32>" + var32_HTML + "</rvar32>\n";
  XML+="</response>\n";
  
  myServer.send(200,"text/xml",XML);
}

void handleLogin(){
  //login page, also called for disconnect
  String msg = "";
  if (myServer.hasHeader("Cookie")){
    String cookie = myServer.header("Cookie");
    if(DEBUG_PRINTING){ Serial.print("[DEBUG] Found cookie: ");Serial.println(cookie); }
  }
  if (myServer.hasArg("DISCONNECT")){
    if(DEBUG_PRINTING){ Serial.println("Disconnection"); }
    myServer.sendHeader("Location","/login");
    myServer.sendHeader("Cache-Control","no-cache");
    myServer.sendHeader("Set-Cookie","ESPSESSIONID=0");
    myServer.send(301);
    return;
  }
  
  if (myServer.hasArg("USERNAME") && myServer.hasArg("PASSWORD")){
    if( checkCredentials( myServer.arg("USERNAME"), myServer.arg("PASSWORD") ) ){
      myServer.sendHeader("Location","/");
      myServer.sendHeader("Cache-Control","no-cache");
      myServer.sendHeader("Set-Cookie","ESPSESSIONID=1");
      myServer.send(301);
      if(DEBUG_PRINTING){ Serial.println("[DEBUG] Log in Successful"); }
      return;
    }
  msg = "Invalid Credentials!";
  }
  
  buildLoginPage( msg );
  myServer.send(200, "text/html", loginHtml);
}

void handleRoot(){
  //root page can be accessed only if authentification is ok
  String header;
  
  if (!is_authentified()){
    myServer.sendHeader("Location","/login");
    myServer.sendHeader("Cache-Control","no-cache");
    myServer.send(301);
    return;
  }

  bool sendTheHeader = false;
  
  if (myServer.hasArg("buttonEmailTest")){
    if (myServer.arg("buttonEmailTest") == "YES"){
      sendTheHeader = true;
      String mailContent = "<html>\n<body>\n<i>Email Test <b>Button</b> Pressed</i>\n</body>\n</html>";
      sendEmail( getEmailLogin(), getEmailPassword(), getEmailRecipient(), "Test" , &mailContent );
      //sendEmail( getEmailLogin(), getEmailPassword(), getEmailRecipient(), "Test" , &XML );
    }
  }

  if (myServer.hasArg("getOPM")){
    //sendTheHeader = true;
    operationMode = (myServer.arg("getOPM")).toInt();
    Serial.write(START_CHAR);
    Serial.print( "OPM" + (String)( operationMode ) );
    Serial.write(END_CHAR);
    if( operationMode > 47 )
    {
      operationMode -= 48;
    }
  }
  
  if (myServer.hasArg("buttonTodayResults")){
    if (myServer.arg("buttonTodayResults") == "YES"){
      sendTheHeader = true;
      data2Mail();
    }
  }

  if (myServer.hasArg("buttonGetTime")){
    if (myServer.arg("buttonGetTime") == "YES"){
      sendTheHeader = true;

      sendOnlineTime();
    }
  }

  if (myServer.hasArg("timeHour") && myServer.hasArg("timeMin")){
    sendTheHeader = true;

    Serial.write(START_CHAR);Serial.print( "WTH" + (String)((myServer.arg("timeHour")).toInt()) );Serial.write(END_CHAR);;
    Serial.write(START_CHAR);Serial.print( "WTM" + (String)((myServer.arg("timeMin" )).toInt()) );Serial.write(END_CHAR);;
  }

  if (myServer.hasArg("dateDay") && myServer.hasArg("dateMon") && myServer.hasArg("dateYea")){
    sendTheHeader = true;

    Serial.write(START_CHAR);Serial.print( "WDD" + (String)((myServer.arg("dateDay")).toInt()) );Serial.write(END_CHAR);;
    Serial.write(START_CHAR);Serial.print( "WDM" + (String)((myServer.arg("dateMon")).toInt()) );Serial.write(END_CHAR);;
    Serial.write(START_CHAR);Serial.print( "WDY" + (String)((myServer.arg("dateYea")).toInt()) );Serial.write(END_CHAR);;
  }

  if (myServer.hasArg("weekday")){
    sendTheHeader = true;

    Serial.write(START_CHAR);Serial.print( "WDW" + (String)((myServer.arg("weekday")).toInt()) );Serial.write(END_CHAR);;
  }

  if (myServer.hasArg("mailRecipient")){
    sendTheHeader = true;

    var06_HTML = (String)(myServer.arg("mailRecipient")); 
    setEmailRecipient( var06_HTML );
  }

  if (myServer.hasArg("userCredentialName") && myServer.hasArg("userCredentialPswd1") && myServer.hasArg("userCredentialPswd2")){
    sendTheHeader = true;
    
    EEPROM_USER = (String)(myServer.arg("userCredentialName"));
    setUserCredentialsLogin( EEPROM_USER );

    EEPROM_PSWD = (String)(myServer.arg("userCredentialPswd1"));
    setUserCredentialsPassword( EEPROM_PSWD );
    
  }

  if (myServer.hasArg("timezoneR")){
    sendTheHeader = true;
    
    EEPROM_TIMEZONE = (myServer.arg("timezoneR")).toInt();    
    setTimezone( EEPROM_TIMEZONE );
  }
  
  
  if( sendTheHeader ){
    myServer.sendHeader("Location","/");
    myServer.sendHeader("Cache-Control","no-cache");
    myServer.send(301);
    return;
  }
  
  //CREATING AND SENDING THE .HTML FILE
  handleWebsite();
  
}

void handleWebsite(){
  /*
  myServer.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  myServer.sendHeader("Pragma", "no-cache");
  myServer.sendHeader("Expires", "-1");
  
  myServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
  myServer.send(200, "text/html", "");
  //...
  myServer.client().stop();
  */
  
  buildWebsite();
  myServer.send(200, "text/html", websiteHTML);
  
}

//---------------------------------------------------------------------------------------

void buildOnce(){
  Style   = "<style>\n";
  Style  += "  body {\n";
  Style  += "      height: 420px;\n";
  Style  += "      margin: 0 auto;\n";
  Style  += "      font-family: Courier New, Courier, monospace;\n";
  Style  += "      font-size: 18px;\n";
  Style  += "      text-align: center;\n";
  Style  += "      color: lightcyan;\n";
  Style  += "      background-color: darkslategray;\n";
  Style  += "      background-image: linear-gradient(to bottom, darkgray, darkslategray);\n";
  Style  += "      background-repeat: no-repeat;\n";
  Style  += "   }\n";
  Style  += "   p {\n";
  Style  += "      text-align: center;\n";
  Style  += "      font-size: 10px;\n";
  Style  += "      font-style: italic;\n";
  Style  += "      width: 100%;\n";
  Style  += "   }\n";
  Style  += "   h1{\n";
  Style  += "      text-align: center;\n";
  Style  += "      font-size: 24px;\n";
  Style  += "      width: 100%;\n";
  Style  += "   }\n";
  Style  += "   h2{\n";
  Style  += "      text-align: center;\n";
  Style  += "      font-size: 16px;\n";
  Style  += "      width: 100%;\n";
  Style  += "   }\n";
  Style  += "   sup{\n";
  Style  += "      text-align: center;\n";
  Style  += "      font-size: 16px;\n";
  Style  += "      color: lightpink;\n";
  Style  += "      font-style: italic;\n";
  Style  += "      width: 100%;\n";
  Style  += "   }\n";
  Style  += "</style>\n";
  
  Footer  = "  <table height='100px'></table>\n";
  Footer += "  <p><br>Wellington Rodrigo Gallo\n";
  Footer += "  <br><a href='mailto:w.r.gallo@grad.ufsc.br' style='color: lightcyan;'>w.r.gallo@grad.ufsc.br</a>\n";
  Footer += "  <br>2018</p>\n";
  Footer += "</BODY>\n";
  Footer += "</HTML>\n";
}

void buildLoginPage(String msg){
  loginHtml = "<!DOCTYPE html>\n";
  loginHtml +="<title>Enter Your Credentials</title>\n";
  loginHtml += "<html lang='en' >\n";

  loginHtml += Style;
  
  loginHtml += "    <body>\n";
  loginHtml += "    <h1> Solar Tracker System </h1>\n";
  loginHtml += "    <h2> Enter your Credentials </h2>\n";
  loginHtml += "    <table height='100px'></table>\n";
  loginHtml += "    <table align='center'>\n";
  loginHtml += "    <form action='/login' method='POST'>\n";
  loginHtml += "            <tr>\n";
  loginHtml += "                <td align='right'>\n";
  loginHtml += "                <label class='center' for='USERNAME' style='margin: 0px 0px 0px 0px;color: lightcyan;'>USERNAME:</label>\n";
  loginHtml += "                </td>\n";
  loginHtml += "                <td align='left'>\n";
  loginHtml += "                <input name='USERNAME' id='USERNAME' type='text' class='loginForm' style='width: 227px' /><br />\n";
  loginHtml += "                </td>\n";
  loginHtml += "            </tr>\n";
  loginHtml += "            <tr>\n";
  loginHtml += "                <td align='right'>\n";
  loginHtml += "                <label class='left' for='PASSWORD' style='margin: 0px 0px 0px 0px;color: lightcyan ;'>PASSWORD:</label>\n";
  loginHtml += "                </td>\n";
  loginHtml += "                <td align='left'>\n";
  loginHtml += "                <input name='PASSWORD' id='PASSWORD' type='PASSWORD' class='loginForm' style='width:159px;' />\n";
  loginHtml += "                <input name='submit' type='submit' class='button_2' value='Sign In' />\n";
  loginHtml += "                </td>\n";
  loginHtml += "            </tr>\n";
  loginHtml += "        </form>\n";
  loginHtml += "        </table>\n";
  loginHtml += "        <sup> " + msg + " </sup>\n";

  loginHtml += Footer;
  
  loginHtml += "       </body>";
  loginHtml += "</html>\n";
}

void buildWebsite(){

  buildJavascript();

  /* HTML PAGE, WITH THE HOME NETWORK AUTOMATION INFORMATION */
  websiteHTML += Style;
  
  websiteHTML +="<BODY onload='process()'>\n";
  websiteHTML +="<h1> Solar Tracker System </h1><br>\n";
  
  websiteHTML +="<br><table align=center width='600px' border='1' edgecolor='gray' bgcolor='darkslategray'>\n";
  websiteHTML +="<tr><td align=left width='220px'>Solar Tracker Unit</td>\n";
  websiteHTML +=    "<td align=center > <A id='var01'> </A></td></tr>\n";
  websiteHTML +="<tr><td align=left>Control Unit</td>\n";
  websiteHTML +=    "<td align=center> <A id='var02'></A> </td></tr>\n";
  websiteHTML +="<tr><td align=left width='220px'> Time </td>\n";
  websiteHTML +=    "<td align=center> <A id='var03'></A> </td></tr>\n";
  websiteHTML +="<tr><td align=left> Date (DD/MM/YY) </td>\n";
  websiteHTML +=    "<td align=center> <A id='var04'></A> </td></tr>\n";
  websiteHTML +="<tr><td align=left> Weekday </td>\n";
  websiteHTML +=    "<td align=center> <A id='var05'></A> </td></tr>\n";
  websiteHTML +="<tr><td align=left> Mail Recipient </td>\n";
  websiteHTML +=    "<td align=center> <A id='var06'></A> </td></tr></table>\n";


  websiteHTML += "<br><table width=600px align='center' border='1' edgecolor='green' bgcolor=darkslategray>\n";
  websiteHTML +=    "<tr><td align=center width='220px'><i>";
  if(      operationMode == 0 ){ websiteHTML += "Short Circuit Test"; }
  else if( operationMode == 1 ){ websiteHTML += "Power Test"; }
  else if( operationMode == 2 ){ websiteHTML += "Open Circuit Test"; }
  websiteHTML += "</i></td>\n";
  websiteHTML +=     "<td align=center width=190px><i> Solar Tracker </i></td>\n";
  websiteHTML +=     "<td align=center width=190px><i> Control </A> </i></td></tr>\n";

  if(      operationMode == 1 )
  {
    websiteHTML += "<tr><td align=left width='220px'> Power Delivered </td>\n";
    websiteHTML +=     "<td align=center> <A id='var07'></A> </td>\n";
    websiteHTML +=     "<td align=center> <A id='var19'></A> </td></tr>\n";
  }
  else if( operationMode == 0 )
  {
    websiteHTML += "<tr><td align=left width='220px'> Current Delivered </td>\n";
    websiteHTML +=     "<td align=center> <A id='var31'></A> </td>\n";
    websiteHTML +=     "<td align=center> <A id='var32'></A> </td></tr>\n";
  }
  if(      operationMode != 0 )
  {
    websiteHTML += "<tr><td align=left> Voltage </td>\n";
    websiteHTML +=     "<td align=center> <A id='var08'></A> </td>\n";
    websiteHTML +=     "<td align=center> <A id='var20'></A> </td></tr>\n";
  }
  if(      operationMode != 2 )
  {
    websiteHTML += "<tr><td align=left> Current </td>\n";
    websiteHTML +=     "<td align=center> <A id='var09'></A> </td>\n";
    websiteHTML +=     "<td align=center> <A id='var21'></A> </td></tr>\n";
  }
  if(      operationMode == 1 )
  {
    websiteHTML += "<tr><td align=left> Power </td>\n";
    websiteHTML +=     "<td align=center> <A id='var10'></A> </td>\n";
    websiteHTML +=     "<td align=center> <A id='var22'></A> </td></tr>\n";
  }
  websiteHTML += "<tr><td align=left> Temperature </td>\n";
  websiteHTML +=     "<td align=center> <A id='var11'></A> </td>\n";
  websiteHTML +=     "<td align=center> <A id='var23'></A> </td></tr>\n";
  websiteHTML += "<tr><td align=left> LUX </td>\n";
  websiteHTML +=     "<td align=center> <A id='var12'></A> </td>\n";
  websiteHTML +=     "<td align=center> <A id='var24'></A> </td></tr>\n";
  websiteHTML += "<tr><td align=left> Photo Up Left </td>\n";
  websiteHTML +=     "<td align=center> <A id='var13'></A> </td>\n";
  websiteHTML +=     "<td align=center> <A id='var25'></A> </td></tr>\n";
  websiteHTML += "<tr><td align=left> Photo Up Right </td>\n";
  websiteHTML +=     "<td align=center> <A id='var14'></A> </td>\n";
  websiteHTML +=     "<td align=center> <A id='var26'></A> </td></tr>\n";
  websiteHTML += "<tr><td align=left> Photo Down Left </td>\n";
  websiteHTML +=     "<td align=center> <A id='var15'></A> </td>\n";
  websiteHTML +=     "<td align=center> <A id='var27'></A> </td></tr>\n";
  websiteHTML += "<tr><td align=left> Photo Down Right </td>\n";
  websiteHTML +=     "<td align=center> <A id='var16'></A> </td>\n";
  websiteHTML +=     "<td align=center> <A id='var28'></A> </td></tr>\n";
  websiteHTML += "<tr><td align=left> Pitch Angle </td>\n";
  websiteHTML +=     "<td align=center> <A id='var17'></A> </td>\n";
  websiteHTML +=     "<td align=center> <A id='var29'></A> </td></tr>\n";
  websiteHTML += "<tr><td align=left> Yaw Angle </td>\n";
  websiteHTML +=     "<td align=center> <A id='var18'></A> </td>\n";
  websiteHTML +=     "<td align=center> <A id='var30'></A> </td></tr></table>\n";
  
  websiteHTML +="<br><table width=600px align='center' border='1' edgecolor='green' bgcolor=darkslategray>\n";
  websiteHTML +=    "<form action='/' method='POST'>\n";
  websiteHTML +=    "<td><table width=420px align='left'>\n";
  websiteHTML +=    "<tr><td align='left' width=210px>Select Test Mode: </td>\n";
  websiteHTML +=    "<td align='left'><select style='width: 200px;' name='getOPM'>\n";
  websiteHTML +=        "<option value=0>Short Circuit</option>\n";
  websiteHTML +=        "<option value=1>Power</option>\n";
  websiteHTML +=        "<option value=2>Open Circuit</option>\n";
  websiteHTML +=    "</select></td></tr>\n";
  websiteHTML +=    "</table></td>\n";
  websiteHTML +=    "<td align='center' width=160px><input name='buttonSetOPM' type='submit' class='button_2' value='Change Test Mode' /></td>\n";
  websiteHTML +="</form></table>\n";

  websiteHTML += "<br><table width=600px align='center'>\n";
  websiteHTML += "&nbsp<a href='/?buttonTodayResults=YES'><input name='buttonTodayResults' type='submit' class='button_2' onclick=\"alert('Sending an email may take a few seconds.')\" value='Email Results Table' ></a>\n";
  websiteHTML += "&nbsp<a href='/?buttonEmailTest=YES'><input name='buttonEmailTest' type='submit' class='button_2' onclick=\"alert('Sending an email may take a few seconds.')\" value='Test Email' ></a>\n";
  websiteHTML += "&nbsp<a href='/?buttonGetTime=YES'><input name='buttonGetTime' type='submit' class='button_2' value='Get Time Online' ></a></table>\n";
      
  websiteHTML += "<br><table width=600px align='center' border='1' edgecolor='green' bgcolor=darkslategray>\n";
  websiteHTML +=   "<form action='/setTimezone' method='POST'>\n";
  websiteHTML +=   "<td><table width=420px align='left'>\n";
  websiteHTML +=   "<tr><td align='left' width=210px>Time Zone: </td>\n";
  websiteHTML +=   "<td align='left' width=210px >\n";
  websiteHTML +=       "<input name='timezoneR' id='timezoneR' type='range' min='-12' max='12' value='0' step='1' onchange='showValue(this.value)' style='width:200px'/>\n";
  websiteHTML +=   "</td></tr>\n";
  websiteHTML +=   "<tr><td align='left' width=210px></td><td align='center' width=210px><span id='timezone'></span></td></tr></table></td>\n";
  websiteHTML +=  "<td align='center' width=160px><input name='buttonSetTimezone' type='submit' class='button_2' value='Update Time Zone' /></td>\n";
  websiteHTML += "</form></table>\n";
      
  websiteHTML += "<br><table width=600px align='center' border='1' edgecolor='green' bgcolor=darkslategray>\n";
  websiteHTML +=   "<form action='/setTime' method='POST'>\n";
  websiteHTML +=   "<td><table width=420px align='left'>\n";
  websiteHTML +=   "<tr><td align='left' width=210px>Time: </td>\n";
  websiteHTML +=   "<td align='left' width=210px >\n";
  websiteHTML +=       "<input name='timeHour' id='timeHour' type='number' min='0' max='23' class='setTimeForm' placeholder='Hour'   style='width:90px; text-align: center' required/>\n";
  websiteHTML +=       "<input name='timeMin'  id='timeMin'  type='number' min='0' max='59' class='setTimeForm' placeholder='Minute' style='width:90px; text-align: center' required/>\n";
  websiteHTML +=   "</td></tr>\n";
  websiteHTML +=   "<tr><td align='left' width=210px>Date: </td>\n";
  websiteHTML +=   "<td align='left' width=210px>\n";
  websiteHTML +=       "<input name='dateDay'  id='dateDay' type='number' min='1'  max='31' class='setTimeForm' placeholder='Day'   style='width:55px; text-align: center' required/>\n";
  websiteHTML +=       "<input name='dateMon'  id='dateMon' type='number' min='1'  max='12' class='setTimeForm' placeholder='Month' style='width:55px; text-align: center' required/>\n";
  websiteHTML +=       "<input name='dateYea'  id='dateYea' type='number' min='17' max='50' class='setTimeForm' placeholder='Year'  style='width:55px; text-align: center' required/>\n";
  websiteHTML +=   "</td></tr>\n";
  websiteHTML +=   "<tr><td align='left' width=210px>Weekday: </td>\n";
  websiteHTML +=   "<td align='left'><select style='width: 200px;' name='weekday'>\n";
  websiteHTML +=       "<option value=0>Sunday</option>\n";
  websiteHTML +=       "<option value=1>Monday</option>\n";
  websiteHTML +=       "<option value=2>Tuesday</option>\n";
  websiteHTML +=       "<option value=3>Wednesday</option>\n";
  websiteHTML +=       "<option value=4>Thursday</option>\n";
  websiteHTML +=       "<option value=5>Friday</option>\n";
  websiteHTML +=       "<option value=6>Saturday</option>\n";
  websiteHTML +=   "</select></td></tr></table></td>\n";
  websiteHTML +=    "<td align='center' width=160px><input name='buttonSetTime' type='submit' class='button_2' value='Update Time' /></td></form></table>\n";
  
  websiteHTML += "<br><table width=600px align='center' border='1' edgecolor='green' bgcolor=darkslategray>\n";
  websiteHTML +=     "<form action='/setMailRecipient' method='POST'>\n";
  websiteHTML +=     "<td><table width=420px align='left'>\n";
  websiteHTML +=     "<tr><td align='left' width=210px>Email Recipient: </td>\n";
  websiteHTML +=     "<td align='left' width=205px >\n";
  websiteHTML +=         "<input name='mailRecipient' id='mailRecipient' type='email' maxlength='50' class='setMailRecipientForm' placeholder='example@example.com'  style='width:195px; text-align: center' required/>\n";
  websiteHTML +=     "</td></tr></table></td>\n";
  websiteHTML +=     "<td align='center' width=160px><input name='buttonSetMailRecipient' type='submit' class='button_2' value='Change Mail Recipient' /></td>\n";
  websiteHTML +=   "</form></table>\n";
  websiteHTML +=   "<br><table width=600px align='center' border='1' edgecolor='green' bgcolor=darkslategray>\n";
  websiteHTML +=     "<form action='/setNewCredential' method='POST' onsubmit='return checkPassword(this);'>\n";
  websiteHTML +=     "<td><table width=420px align='left'>\n";
  websiteHTML +=     "<tr><td align='left' width=210px>User Credential: </td>\n";
  websiteHTML +=     "<td align='left' width=205px>\n";
  websiteHTML +=         "<input name='userCredentialName' id='userCredentialName' type='text'     minlength='5' maxlength='20' class='setCredentialFrom' placeholder='Username'  style='width:195px; text-align: center' required/></td></tr>\n";
  websiteHTML +=     "<tr><td align='left' width=210px>User Password: </td>\n";
  websiteHTML +=     "<td align='left' width=205px>\n";
  websiteHTML +=         "<input name='userCredentialPswd1' id='userCredentialPswd1' type='password' minlength='8' maxlength='20' class='setCredentialFrom' placeholder='Password'  style='width:195px; text-align: center' required/>\n";
  websiteHTML +=     "</td></tr>\n";
  websiteHTML +=     "<tr><td align='left' width=210px>Confirm Password: </td>\n";
  websiteHTML +=     "<td align='left' width=205px>\n";
  websiteHTML +=         "<input name='userCredentialPswd2' id='userCredentialPswd2' type='password' minlength='8' maxlength='20' class='setCredentialFrom' placeholder='Password'  style='width:195px; text-align: center' required/>\n";
  websiteHTML +=     "</td></tr>\n";
  websiteHTML +=     "</table></td>\n";
  websiteHTML +=     "<td align='center' width=160px><input name='buttonSetCredentials' type='submit' class='button_2' value='Change Credentials' /></td>\n";
  websiteHTML +=    "</form></table>\n";

  websiteHTML +="<br><br>You can access this page until you <a style='color: lightcyan;' href='/login?DISCONNECT=YES'>disconnect</a>\n";


  websiteHTML += Footer ;

}

void buildJavascript(){
  websiteHTML ="<!DOCTYPE HTML>\n";
  websiteHTML +="<title>Solar Tracker System</title>\n";
  
  websiteHTML +="<SCRIPT>\n";
  websiteHTML +="var xmlHttp=createXmlHttpObject();\n";

  websiteHTML +="function createXmlHttpObject(){\n";
  websiteHTML +=" if(window.XMLHttpRequest){\n";
  websiteHTML +="    xmlHttp=new XMLHttpRequest();\n";
  websiteHTML +=" }else{\n";
  websiteHTML +="    xmlHttp=new ActiveXObject('Microsoft.XMLHTTP');\n";
  websiteHTML +=" }\n";
  websiteHTML +=" return xmlHttp;\n";
  websiteHTML +="}\n";

  websiteHTML +="function process(){\n";
  websiteHTML +=" if(xmlHttp.readyState==0 || xmlHttp.readyState==4){\n";
  websiteHTML +="   xmlHttp.open('PUT','xml',true);\n";
  websiteHTML +="   xmlHttp.onreadystatechange=handleServerResponse;\n"; // no brackets?????
  websiteHTML +="   xmlHttp.send(null);\n";
  websiteHTML +=" }\n";
  websiteHTML +=" setTimeout('process()',1000);\n";
  websiteHTML +="}\n";
  
  websiteHTML +="function handleServerResponse(){\n";
  websiteHTML +=" if(xmlHttp.readyState==4 && xmlHttp.status==200){\n";
    
  websiteHTML +="   xmlResponse=xmlHttp.responseXML;\n";
  websiteHTML +="   xmldoc = xmlResponse.getElementsByTagName('response');\n";
  //UPDATING VARIABLES ON PAGE
  websiteHTML +="   message = xmldoc[0];\n";
  
  websiteHTML +="document.getElementById('var01').innerHTML = message.children[0].firstChild.data;\n";
  websiteHTML +="document.getElementById('var02').innerHTML = message.children[1].firstChild.data;\n";
  websiteHTML +="document.getElementById('var03').innerHTML = message.children[2].firstChild.data;\n";
  websiteHTML +="document.getElementById('var04').innerHTML = message.children[3].firstChild.data;\n";
  websiteHTML +="document.getElementById('var05').innerHTML = message.children[4].firstChild.data;\n";
  websiteHTML +="document.getElementById('var06').innerHTML = message.children[5].firstChild.data;\n";

  if( operationMode == 1 )
  {
    websiteHTML +="document.getElementById('var07').innerHTML = message.children[6].firstChild.data;\n";//TOTAL POWER DELIVERED
    websiteHTML +="document.getElementById('var19').innerHTML = message.children[18].firstChild.data;\n";
  }
  if( operationMode != 0 )
  {
    websiteHTML +="document.getElementById('var08').innerHTML = message.children[7].firstChild.data;\n";//VOLTAGE
    websiteHTML +="document.getElementById('var20').innerHTML = message.children[19].firstChild.data;\n";
  }
  if( operationMode != 2 )
  {
    websiteHTML +="document.getElementById('var09').innerHTML = message.children[8].firstChild.data;\n";//CURRENT
    websiteHTML +="document.getElementById('var21').innerHTML = message.children[20].firstChild.data;\n";
  }
  if( operationMode == 1 )
  {
    websiteHTML +="document.getElementById('var10').innerHTML = message.children[9].firstChild.data;\n";//Power
    websiteHTML +="document.getElementById('var22').innerHTML = message.children[21].firstChild.data;\n";
  }
  websiteHTML +="document.getElementById('var11').innerHTML = message.children[10].firstChild.data;\n";//Temp
  websiteHTML +="document.getElementById('var23').innerHTML = message.children[22].firstChild.data;\n";
  websiteHTML +="document.getElementById('var12').innerHTML = message.children[11].firstChild.data;\n";//Lux
  websiteHTML +="document.getElementById('var24').innerHTML = message.children[23].firstChild.data;\n";
  websiteHTML +="document.getElementById('var13').innerHTML = message.children[12].firstChild.data;\n";//PHOTO UL
  websiteHTML +="document.getElementById('var25').innerHTML = message.children[24].firstChild.data;\n";  
  websiteHTML +="document.getElementById('var14').innerHTML = message.children[13].firstChild.data;\n";//PHOTO UR
  websiteHTML +="document.getElementById('var26').innerHTML = message.children[25].firstChild.data;\n";  
  websiteHTML +="document.getElementById('var15').innerHTML = message.children[14].firstChild.data;\n";//PHOTO DL
  websiteHTML +="document.getElementById('var27').innerHTML = message.children[26].firstChild.data;\n";  
  websiteHTML +="document.getElementById('var16').innerHTML = message.children[15].firstChild.data;\n";//PHOTO DR
  websiteHTML +="document.getElementById('var28').innerHTML = message.children[27].firstChild.data;\n";
  websiteHTML +="document.getElementById('var17').innerHTML = message.children[16].firstChild.data;\n";//PITCH
  websiteHTML +="document.getElementById('var29').innerHTML = message.children[17].firstChild.data;\n";
  websiteHTML +="document.getElementById('var18').innerHTML = message.children[28].firstChild.data;\n";//ANGLE
  websiteHTML +="document.getElementById('var30').innerHTML = message.children[29].firstChild.data;\n";
  if( operationMode == 0 )
  {
    websiteHTML +="document.getElementById('var31').innerHTML = message.children[30].firstChild.data;\n";//TOTAL CURRENT DELIVIRED
    websiteHTML +="document.getElementById('var32').innerHTML = message.children[31].firstChild.data;\n";
  }
  /*
  websiteHTML +="document.getElementById('var01').innerHTML = message.children[0].firstChild.data;\n";
  websiteHTML +="document.getElementById('var02').innerHTML = message.children[1].firstChild.data;\n";
  websiteHTML +="document.getElementById('var03').innerHTML = message.children[2].firstChild.data;\n";
  websiteHTML +="document.getElementById('var04').innerHTML = message.children[3].firstChild.data;\n";
  websiteHTML +="document.getElementById('var05').innerHTML = message.children[4].firstChild.data;\n";
  websiteHTML +="document.getElementById('var06').innerHTML = message.children[5].firstChild.data;\n";
  websiteHTML +="document.getElementById('var07').innerHTML = message.children[6].firstChild.data;\n";
  websiteHTML +="document.getElementById('var08').innerHTML = message.children[7].firstChild.data;\n";
  websiteHTML +="document.getElementById('var09').innerHTML = message.children[8].firstChild.data;\n";
  websiteHTML +="document.getElementById('var10').innerHTML = message.children[9].firstChild.data;\n";
  websiteHTML +="document.getElementById('var11').innerHTML = message.children[10].firstChild.data;\n";
  websiteHTML +="document.getElementById('var12').innerHTML = message.children[11].firstChild.data;\n";
  websiteHTML +="document.getElementById('var13').innerHTML = message.children[12].firstChild.data;\n";
  websiteHTML +="document.getElementById('var14').innerHTML = message.children[13].firstChild.data;\n";
  websiteHTML +="document.getElementById('var15').innerHTML = message.children[14].firstChild.data;\n";
  websiteHTML +="document.getElementById('var16').innerHTML = message.children[15].firstChild.data;\n";
  websiteHTML +="document.getElementById('var17').innerHTML = message.children[16].firstChild.data;\n";
  websiteHTML +="document.getElementById('var18').innerHTML = message.children[17].firstChild.data;\n";
  websiteHTML +="document.getElementById('var19').innerHTML = message.children[18].firstChild.data;\n";
  websiteHTML +="document.getElementById('var20').innerHTML = message.children[19].firstChild.data;\n";
  websiteHTML +="document.getElementById('var21').innerHTML = message.children[20].firstChild.data;\n";
  websiteHTML +="document.getElementById('var22').innerHTML = message.children[21].firstChild.data;\n";
  websiteHTML +="document.getElementById('var23').innerHTML = message.children[22].firstChild.data;\n";
  websiteHTML +="document.getElementById('var24').innerHTML = message.children[23].firstChild.data;\n";
  websiteHTML +="document.getElementById('var25').innerHTML = message.children[24].firstChild.data;\n";
  websiteHTML +="document.getElementById('var26').innerHTML = message.children[25].firstChild.data;\n";
  websiteHTML +="document.getElementById('var27').innerHTML = message.children[26].firstChild.data;\n";
  websiteHTML +="document.getElementById('var28').innerHTML = message.children[27].firstChild.data;\n";
  websiteHTML +="document.getElementById('var29').innerHTML = message.children[28].firstChild.data;\n";
  websiteHTML +="document.getElementById('var30').innerHTML = message.children[29].firstChild.data;\n";
  websiteHTML +="document.getElementById('var31').innerHTML = message.children[30].firstChild.data;\n";
  websiteHTML +="document.getElementById('var32').innerHTML = message.children[31].firstChild.data;\n";
  */
  websiteHTML +=" }\n";
  websiteHTML +="}\n";

  websiteHTML +="function checkPassword(form){\n";
  websiteHTML +="  re = /^\\w+$/;\n";
  websiteHTML +="  if(!re.test(form.userCredentialName.value)) {\n";
  websiteHTML +="    alert('Error: Username must contain only letters, numbers and underscores!');\n";
  websiteHTML +="    form.userCredentialName.focus();\n";
  websiteHTML +="    return false;\n";
  websiteHTML +="  }\n";
  websiteHTML +="  if(form.userCredentialPswd1.value == form.userCredentialPswd2.value) {\n";
  websiteHTML +="    if(form.userCredentialPswd1.value == form.userCredentialName.value) {\n";
  websiteHTML +="      alert('Error: Password must be different from Username!');\n";
  websiteHTML +="      form.userCredentialPswd1.focus();\n";
  websiteHTML +="      return false;\n";
  websiteHTML +="    }\n";
  websiteHTML +="    re = /[0-9]/;\n";
  websiteHTML +="    if(!re.test(form.userCredentialPswd1.value)) {\n";
  websiteHTML +="      alert('Error: password must contain at least one number (0-9)!');\n";
  websiteHTML +="      form.userCredentialPswd1.focus();\n";
  websiteHTML +="      return false;\n";
  websiteHTML +="    }\n";
  websiteHTML +="    re = /[a-z]/;\n";
  websiteHTML +="    if(!re.test(form.userCredentialPswd1.value)) {\n";
  websiteHTML +="      alert('Error: password must contain at least one lowercase letter (a-z)!');\n";
  websiteHTML +="      form.userCredentialPswd1.focus();\n";
  websiteHTML +="      return false;\n";
  websiteHTML +="    }\n";
  websiteHTML +="    re = /[A-Z]/;\n";
  websiteHTML +="    if(!re.test(form.userCredentialPswd1.value)) {\n";
  websiteHTML +="      alert('Error: password must contain at least one uppercase letter (A-Z)!');\n";
  websiteHTML +="      form.userCredentialPswd1.focus();\n";
  websiteHTML +="      return false;\n";
  websiteHTML +="    }\n";
  websiteHTML +="  }\n";
  websiteHTML +="  return true;\n";
  websiteHTML +="}\n";

  websiteHTML +="function showValue(newValue)\n";
  websiteHTML +="{\n";
  websiteHTML +="  if( newValue > 0 ){\n";
  websiteHTML +="      document.getElementById('timezone').innerHTML='UTC +' + newValue;\n";
  websiteHTML +="  }else{\n";
  websiteHTML +="      document.getElementById('timezone').innerHTML='UTC ' + newValue;\n";
  websiteHTML +="  }\n";
  websiteHTML +="}\n";
  
  websiteHTML +="</SCRIPT>\n";
}

//---------------------------------------------------------------------------------------

void data2Mail()
{
  mailData  = "<HTML>\n";
  mailData += "<title>Solar Tracker System Results</title>\n";
  
  mailData +="<style>\n";
  mailData +="body {\n";
  mailData +=    "height: 420px;\n";
  mailData +=    "margin: 0 auto;\n";
  mailData +=    "font-family: Courier New, Courier, monospace;\n";
  mailData +=    "font-size: 18px;\n";
  mailData +=    "text-align: center;\n";
  mailData +=    "color: darkslategrey;\n";
  mailData += "}\n";
  mailData += "p {\n";
  mailData +=    "text-align: center;\n";
  mailData +=    "font-size: 10px;\n";
  mailData +=    "font-style: italic;\n";
  mailData +=    "width: 100%;\n";
  mailData += "}\n";
  mailData += "h1{\n";
  mailData +=    "text-align: center;\n";
  mailData +=    "font-size: 24px;\n";
  mailData +=    "width: 100%;\n";
  mailData += "}\n";
  mailData +="</style>\n";
  
  mailData += "<BODY>\n";
  mailData += "<h1> Solar Tracker System Results </h1><br>\n";

  mailData +="<br><table align=center width='600px' border='1' edgecolor='gray' bgcolor=lightblue>\n";
  mailData +=    "<tr><td align=left width='220px'>Solar Tracker Unit</td>\n";
  mailData +=        "<td align=center >" + var01_HTML + "</A></td></tr>\n";
  mailData +=    "<tr><td align=left>Control Unit</td>\n";
  mailData +=        "<td align=center>" + var02_HTML + "</td></tr>\n";
  mailData +=    "<tr><td align=left width='220px'> Time </td>\n";
  mailData +=        "<td align=center>" + var03_HTML + "</td></tr>\n";
  mailData +=    "<tr><td align=left> Date (DD/MM/YY) </td>\n";
  mailData +=        "<td align=center>" + var04_HTML + "</td></tr>\n";
  mailData +=    "<tr><td align=left> Weekday </td>\n";
  mailData +=        "<td align=center>" + var05_HTML + "</td></tr>\n";
  mailData +="</table>\n";
  
  mailData += "<br><table width=600px align='center' border='1' edgecolor='green' bgcolor=lightblue>\n";
  mailData +=    "<tr><td align=center width='220px'><i>";
  if(      operationMode == 0 ){ mailData += "Short Circuit Test"; }
  else if( operationMode == 1 ){ mailData += "Power Test"; }
  else if( operationMode == 2 ){ mailData += "Open Circuit Test"; }
  mailData += "</i></td>\n";
  mailData +=     "<td align=center width=190px><i> Solar Tracker </i></td>\n";
  mailData +=     "<td align=center width=190px><i> Control </A> </i></td></tr>\n";

  if(      operationMode == 1 )
  {
    mailData += "<tr><td align=left width='220px'> Power Delivered </td>\n";
    mailData +=     "<td align=center>" + var07_HTML + "</td>\n";
    mailData +=     "<td align=center>" + var19_HTML + "</td></tr>\n";
  }
  else if( operationMode == 0 )
  {
    mailData += "<tr><td align=left width='220px'> Current Delivered </td>\n";
    mailData +=     "<td align=center>" + var31_HTML + "</td>\n";
    mailData +=     "<td align=center>" + var32_HTML + "</td></tr>\n";
  }
  if(      operationMode != 0 )
  {
    mailData += "<tr><td align=left> Voltage </td>\n";
    mailData +=     "<td align=center>" + var08_HTML + "</td>\n";
    mailData +=     "<td align=center>" + var20_HTML + "</td></tr>\n";
  }
  if(      operationMode != 2 )
  {
    mailData += "<tr><td align=left> Current </td>\n";
    mailData +=     "<td align=center>" + var09_HTML + "</td>\n";
    mailData +=     "<td align=center>" + var21_HTML + "</td></tr>\n";
  }
  if(      operationMode == 1 )
  {
    mailData += "<tr><td align=left> Power </td>\n";
    mailData +=     "<td align=center>" + var10_HTML + "</td>\n";
    mailData +=     "<td align=center>" + var22_HTML + "</td></tr>\n";
  }

  mailData +=    "<tr><td align=left> Temperature </td>\n";
  mailData +=        "<td align=center>" + var11_HTML + "</td>\n";
  mailData +=        "<td align=center>" + var23_HTML + "</td></tr>\n";
  mailData +=    "<tr><td align=left> LUX </td>\n";
  mailData +=        "<td align=center>" + var12_HTML + "</td>\n";
  mailData +=        "<td align=center>" + var24_HTML + "</td></tr>\n";
  mailData +="</table>\n";

  mailData +="<p><br>Wellington Rodrigo Gallo\n";
  mailData +="<br><a href='mailto:w.r.gallo@grad.ufsc.br' style='color: darkslategray;'>w.r.gallo@grad.ufsc.br</a>\n";
  mailData +="<br>2018</p>\n";
  mailData +="</BODY>\n";
  mailData +="</HTML>\n";

  sendEmail( getEmailLogin(), getEmailPassword(), getEmailRecipient(), "Today Results" , &mailData );
}

