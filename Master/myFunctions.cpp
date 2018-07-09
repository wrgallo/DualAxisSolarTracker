#include "Arduino.h"
#include "myFunctions.h"
//#include "myEEPROM.h"

void loggerSetup();
void sendMessage2Logger(String message);
/*
  ARDUINO PRO MINI 5V/16MHZ - ATMEGA328P - PINOUT
  -----------------------------------------------
 | PIN |  PURPOSE                                |
 |-----|-----------------------------------------|
 | 0   |  UART RX (ESP8266 TX)                   |
 | 1   |  UART TX (ESP8266 RX)                   |
 | 2   |  STEPPER MOTOR 1 COIL                   |
 | 3   |  STEPPER MOTOR 1 COIL                   |
 | 4   |  STEPPER MOTOR 1 COIL                   |
 | 5   |  STEPPER MOTOR 1 COIL                   |
 | 6   |  SOFT UART RX (EXTERNAL LOGGER)         |
 | 7   |  SOFT UART TX (HC05 BT RX)              |
 | 8   |  SOFT UART TX (EXTERNAL LOGGER)         |
 | 9   |  CE   of  NRF24L01+                     |
 | 10  |  CSN  for NRF24L01+                     |
 | 11  |  MOSI for NRF24L01+                     |
 | 12  |  MISO for NRF24L01+                     |
 | 13  |  SCK  for NRF24L01+                     |
 | A0  |  STEPPER MOTOR 2 COIL                   |
 | A1  |  STEPPER MOTOR 2 COIL                   |
 | A2  |  STEPPER MOTOR 2 COIL                   |
 | A3  |  STEPPER MOTOR 2 COIL                   |
 | A4  |  SDA FOR I2C COMMUNICATION              |
 | A5  |  SCL FOR I2C COMMUNICATION              |
 | A6  |  ADC INPUT FOR LM35 Vout                |
 | A7  |  SOFT UART RX (HC05 BT TX)              | *Actually it is not connected, but one RX pin needed to be reserved for SoftwareSerial constructor.
  -----------------------------------------------   A7 is not even a compatible pin for SoftwareSerial
  

  ---------------------------------------------
 | Power Supply  | Peripheral                  |
 |---------------|-----------------------------|
 | +5V           |  HC05                       |
 | +5V           |  CATALEX MICRO SD MODULE    |
 | +5V           |  REAL TIME MODULE           |
 | +5V           |  INA219                     |
 | +5V           |  ADS1015                    |
 | +3v3          |  MPU9250                    |
 | +3v3          |  TSL2561                    |
 | +3v3          |  ESP8266                    |
 | +3v3          |  NRF24L01+                  |
  ---------------------------------------------
*/

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
uint8_t operationMode = 0;
//---------------------------------------------------------------
//                  GLOBAL VARIABLES DEFINITIONS
//---------------------------------------------------------------
volatile bool firstTime     = false;                             //THE MCU STARTED ONE MINUTE AGO (true or false)
volatile bool timerFlag     = false;                             //PERIODIC TIMER INTERRUPT FLAG
bool          stilFollowing = true;                              //DO NOT TRACK THE SUN BEETWENN 19h and 05h
//---------------------------------------------------------------
//                        REAL TIME MODULE
//---------------------------------------------------------------
uint8_t timeSecond;                                              //SECOND VALUE
uint8_t timeMinute;                                              //MINUTE VALUE
uint8_t timeHour;                                                //HOUR VALUE
uint8_t timeWeekday;                                             //DAY OF WEEK VALUE (0 = SUNDAY, 6 = SATURDAY)
uint8_t timeDay;                                                 //DAY VALUE
uint8_t timeMonth;                                               //MONTH VALUE
uint8_t timeYear;                                                //YEAR VALUE (2 DIGITS)
//---------------------------------------------------------------
//                           BLUETOOTH
//---------------------------------------------------------------
SoftwareSerial btSerial(BT_RX, BT_TX);                           //OBJECT btSerial FOR UART COMMUNICATION WITH HC-05
SoftwareSerial loggerSerial(LOGGER_RX, LOGGER_TX);               //OBJECT FOR UART COMMUNICATION WITH EXTERNAL LOGGER
//---------------------------------------------------------------
//                             WIFI
//---------------------------------------------------------------
bool     stillBuffering = false;                                 //SERVER IS STILL COMMUNICATING WITH THIS UNIT
char     buf1[10];                                               //BUFFER OF ONLY VALID UART VALUES
String   buffer_read = "";                                       //ALL MSGS IN CURRENT BUFFER
uint16_t msgLen = 0;                                             //LENGHT OF BUFFER READ
volatile uint8_t waitingForAnAnswer = WAIT_FOR_AN_ANSWER;        //Waiting Time Counter
volatile bool goAhead = false;                                   //STOP WAITING FOR A SLAVE MESSAGE, MAYBE IT IS OFFLINE
//---------------------------------------------------------------
//                  RF24 COMMUNICATION DEFINITIONS
//---------------------------------------------------------------
bool     prototypeStatus = false;                                //Control Prototype Online Status
uint8_t  prototypeStatusCounter = 0;                             //Seconds Counter without a answer from the other prototype
//Start NRF24L01+ with CE and CS on pins 9 and 10
RF24 radio(9,10);

// Radio pipe addresses for the nodes to communicate.
/*
 * MASTER UNIT PIPE  0xABCDA00
 * SLAVE 01 PIPE     0xABCDA01
 */
const uint64_t pipes[2] = { 0xABCDA00 , 0xABCDA01 }; 
String messageRF24 = "";
//---------------------------------------------------------------
//                   CONTROL PROTOTYPE DATA
//---------------------------------------------------------------
double   CTRLpowerGenerated = 0;                               //mWh from Control Unit
double   CTRLcurrentGenerated = 0;                              //mWh from Control Unit
float    CTRLtempLM35 = 0;                                      //LM35 CTRL Temperature
float    CTRLlux;                                               //LUX OBTAINED
float    CTRLcurrent_mA  = 0;                                   //Solar current
float    CTRLvoltage = 0;                                       //Solar load voltage
float    CTRLpower_mW    = 0;                                   //Solar load power
/*
uint16_t CTRLphotoUR, CTRLphotoDR, CTRLphotoUL, CTRLphotoDL;    //mV on Phototransistors
float    CTRLpitchAngle;                                      //PITCH ANGLE FOR REFERENCE
*/
//---------------------------------------------------------------
//                       STEPPER MOTOR DEFINITION
//---------------------------------------------------------------
Stepper myStepper1(STEPS_PER_REVOLUTION,  2,  4,  3,  5);         //STEPPER OBJECT (YAW  Control)
Stepper myStepper2(STEPS_PER_REVOLUTION, A0, A2, A1, A3);         //STEPPER OBJECT (PITCH Control)
uint16_t stepCounterM1 = STEPS_MAX_M1/2;                          //STEPPER COUNTER FOR LIMIT CONTROL
uint16_t stepCounterM2 = STEPS_MAX_M2/2;                          //STEPPER COUNTER FOR LIMIT CONTROL
//---------------------------------------------------------------
//                        LUXIMETER DEFINITION
//---------------------------------------------------------------
SFE_TSL2561 light;                                                //TSL2561 OBJECT
unsigned int ms = 400;                                            //INTEGRATION ("shutter") TIME IN MILLISECONDS
double lux;                                                       //LUX OBTAINED
bool isAtMax = false;                                             //TOO MUCH LUX
//---------------------------------------------------------------
//                      EXTERNAL ADC DEFINITION
//---------------------------------------------------------------
Adafruit_ADS1015 ads;                                             //12-bit ADC EXTERNAL ADC OBJECT
#define PHOTO_READINGS 5
uint16_t photoUR[PHOTO_READINGS+1],                               //mV on Phototransistors
         photoDR[PHOTO_READINGS+1],
         photoUL[PHOTO_READINGS+1],
         photoDL[PHOTO_READINGS+1];    
//---------------------------------------------------------------
//                      POWER MONITOR DEFINITION
//---------------------------------------------------------------
Adafruit_INA219 ina219;
double powerGenerated   = 0;                                     //mWh from Solar Tracker Unit
double currentGenerated = 0;                                     //mWh from Solar Tracker Unit
float current_mA        = 0;
float voltage           = 0;
float shuntVoltage      = 0;
float power_mW          = 0;
//---------------------------------------------------------------
//                         TEMPERATURE LM35
//---------------------------------------------------------------
double tempLM35         = 0;

/*
//---------------------------------------------------------------
//                          MPU DEFINITION
//---------------------------------------------------------------
MPU9250 IMU(Wire,MPU9250_ADDRESS);                                // MPU9250 sensor on I2C bus with address 0x69
double pitchAngle = 0;                                            // VARIABLES FOR REFERENCE
*/

/*
//---------------------------------------------------------------
//                     MICRO SD MODULE DEFINITION
//---------------------------------------------------------------
SdFat sd;
SdFile todayLog;
*/



//---------------------------------------------------------------
//             FUNCTIONS: ARDUINO CONFIGURATION
//---------------------------------------------------------------
void configThisUnit()
{  
  
  //--------------------------------------------
  //       UART BAUD RATE CONFIGURATION
  //--------------------------------------------
  Serial.begin(115200);                         //ESP8266 COMMUNICATION
  btSerial.begin(115200);                       //HC05 COMMUNICATION
  loggerSerial.begin(9600);                     //EXTERNAL uC WITH SD LOGGER
  //--------------------------------------------


  //--------------------------------------------
  //       STEPPER MOTOR CONFIGURATION
  //--------------------------------------------
  btSerial.println("SETUP STEPPERS");
  //STEPPER MOTOR DRIVER SETUP
  myStepper1.setSpeed(STEPPER_SPEED);
  myStepper2.setSpeed(STEPPER_SPEED);
  //--------------------------------------------


  //--------------------------------------------
  //               TSL2561 SETUP
  //--------------------------------------------
  btSerial.println("SETUP TSL2561");
  //LUXIMETER START SETUP
  light.begin();
  light.setTiming( 0 , 2 , ms );
  light.setPowerUp();
  //--------------------------------------------


  /*
  //--------------------------------------------
  //               MPU9250 SETUP
  //--------------------------------------------
  //IMU START SETUP
  btSerial.println("SETUP IMU");
  if (IMU.begin() < 0) {
    btSerial.println("IMU error");
    while(1) {}
  }
  IMU.setAccelRange(    MPU9250::ACCEL_RANGE_2G     );                          // setting the accelerometer full scale range to +/-8G 
  IMU.setGyroRange(     MPU9250::GYRO_RANGE_250DPS  );                          // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setDlpfBandwidth( MPU9250::DLPF_BANDWIDTH_92HZ);                          // setting DLPF bandwidth to 20 Hz
  IMU.setSrd(19); 
  //--------------------------------------------
  */

  
  //--------------------------------------------
  //            ADS1015 START SETUP
  //--------------------------------------------
  btSerial.println("SETUP ADS1015");
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  ads.begin();
  delay( 100 );
  //Moving Window Average with PHOTO_READINGS Calculations
  photoUR[0] = ads.readADC_SingleEnded(0) * 3; //mV
  photoDR[0] = ads.readADC_SingleEnded(1) * 3; //mV
  photoUL[0] = ads.readADC_SingleEnded(2) * 3; //mV
  photoDL[0] = ads.readADC_SingleEnded(3) * 3; //mV
  for(uint8_t i=0; i<PHOTO_READINGS; i++)
  {
    photoUL[ i+1 ] = photoUL[ 0 ];photoUR[ i+1 ] = photoUR[ 0 ];
    photoDL[ i+1 ] = photoDL[ 0 ];photoDR[ i+1 ] = photoDR[ 0 ];
  }
  //--------------------------------------------


  //--------------------------------------------
  //         POWER MONITOR START SETUP
  //--------------------------------------------
  btSerial.println("SETUP INA219");
  ina219.begin();
  ina219.setCalibration_16V_400mA();                                            // max precision on volts and amps
  //--------------------------------------------
  
  
  //--------------------------------------------
  //             RF24 CONFIGURATION
  //--------------------------------------------
  btSerial.println("SETUP NRF24L01P");
  //Start Communication
  radio.begin();

  radio.setPALevel( RF24_PA_LOW   );
  radio.setDataRate( RF24_250KBPS );
  
  //Select Writting and Reading Pipe Addresses
  radio.openReadingPipe( 1 , pipes[0] ); //The Pipe where Slaves Answer Master
  radio.startListening();                // Start listening
  //radio.openWritingPipe( pipes[1] ); //To Talk to Slave 1
  //radio.openWritingPipe( pipes[2] ); //To Talk to Slave 2
  //radio.openWritingPipe( pipes[3] ); //To Talk to Slave 3
  //--------------------------------------------

  /*
  btSerial.println("SETUP CATALEX SD");
  //CREATE SD FILE MANAGER
  sd.begin(8,SPI_HALF_SPEED);
  // Abre o arquivo LER_POT.TXT
  todayLog.open((String(timeMonth) + String(timeDay) + ".txt").c_str(), O_RDWR | O_CREAT | O_AT_END);
  todayLog.println(String(timeHour) + ":" + String(timeMinute));
  */

  //--------------------------------------------
  //             TIMER CONFIGURATION
  //--------------------------------------------
  btSerial.println("SETUP TIMER");
  //Timer is used for Safety Control of Time, just in case DS1307 isn't working properly.
  noInterrupts();
  TCCR1A = B00000000;//Register A all 0's since we're not toggling any pins
    // TCCR1B clock prescalers
    // 1 x x x CTC mode
    // x 0 0 1 clkI/O /1 (No prescaling)
    // x 0 1 0 clkI/O /8 (From prescaler)
    // x 0 1 1 clkI/O /64 (From prescaler)
    // x 1 0 0 clkI/O /256 (From prescaler)
    // x 1 0 1 clkI/O /1024 (From prescaler)
  TCCR1B = B00001101;//bit 3 set for CTC mode, bits 2,1,0 set to 1024 prescaler
  TIMSK1 = B00000010;//bit 1 set to call the interrupt on an OCR1A match
  OCR1A  = (unsigned long)(15625UL); //1 Second = 15625 cicles = 16M / 1024
  interrupts();
  //--------------------------------------------

  //--------------------------------------------
  //           REAL TIME FIRST READING
  //--------------------------------------------
  btSerial.println("SETUP RTC");
  //SETTING THE DATE AND TIME ON REAL TIME MODULE FOR THE FIRST TIME
  //setTimeDS1307( 17 , 5 , 27, 6, 15, 0 ); //year, month, day, weekday (0 = Sun, 6 = Sat), hour, minut
  //READING DATE AND TIME FROM REAL TIME MODULE
  getTimeDS1307();
  //---------------------------------------------

  loggerSetup();
}

ISR(TIMER1_COMPA_vect){
  timerFlag = true; 
}

void timerHandler()
{
  static uint8_t ISRCounter = 0;
  if( timerFlag )
  {
    timerFlag = false;
    ISRCounter++;
    
    oneSecondTimerHandler();
    
    if( ISRCounter == 60 )
    {
      ISRCounter = 0;
      oneMinuteTimerHandler();
    }
    
  }
  
}


void oneSecondTimerHandler()
{   
  //Update Slaves Status Information
  if (prototypeStatusCounter  <  SLAVE_IS_OFFLINE_TIMER){ prototypeStatusCounter++; }
  if (prototypeStatusCounter == (SLAVE_IS_OFFLINE_TIMER - 1)){ prototypeStatus = 0; }

  //Check for incoming messages from server
  serverHandler();

  solarTracker();

  if( loggerSerial.available() > 0 )
  {
    btSerial.println( loggerSerial.readString() );
    delay(200);
  }
}

void oneMinuteTimerHandler()
{
  getTimeDS1307();
  
  //Refresh Data
  refreshSensorData();
  powerGenerated       +=       power_mW * 1.66666e-2; //mW times (1/60 hours)
  currentGenerated     +=     current_mA * 1.66666e-2; //mA times (1/60 hours)
  //CTRLcurrentGenerated += CTRLcurrent_mA * 1.66666e-2; //mA times (1/60 hours)

  //Send Updated Data to Server
  refreshServer();
  /*
  if( !firstTime ){
    //Update SD File
    updateSDFile();
  }
  */
  logData();

  if( NIGHTTIME_DEBUG == 0 )
  {
    if( stilFollowing )
    {
      if( (timeHour < 5) || (timeHour > 19) )
      {
        stilFollowing = false;
        loggerSerial.write( END_CHAR );
        //noInterrupts();
        while( (timeHour < 5) || (timeHour > 19) )
        {
          delay( 1000 * 60 * 60);
          getTimeDS1307();
        }
        //interrupts();
        loggerSetup();
      }
    }
  }
  
}
//---------------------------------------------------------------



//---------------------------------------------------------------
//        FUNCTIONS: REAL TIME MODULE CONTROL
//---------------------------------------------------------------
byte dec2bcd(byte number) {
  return ((number / 10 * 16) + (number % 10));
}

uint8_t bcd2dec(uint8_t number) {
  return ((number >> 4) * 10 + (number & 0x0F));
}

void setTimeDS1307(byte timeYear, byte timeMonth, byte timeDay, byte timeWeekday, byte timeHour, byte timeMinute)
{
  Wire.beginTransmission(DS1307_ADDRESS);
  Wire.write(0x00);

  Wire.write(dec2bcd(0));
  Wire.write(dec2bcd(timeMinute));
  Wire.write(dec2bcd(timeHour));
  Wire.write(dec2bcd(timeWeekday));
  Wire.write(dec2bcd(timeDay));
  Wire.write(dec2bcd(timeMonth));
  Wire.write(dec2bcd(timeYear));

  Wire.write(0x00);
  Wire.endTransmission();
}

void getTimeDS1307()
{
  Wire.beginTransmission(DS1307_ADDRESS);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.requestFrom(DS1307_ADDRESS, 7);
  timeSecond  = bcd2dec(Wire.read());
  timeMinute  = bcd2dec(Wire.read());
  timeHour    = bcd2dec(Wire.read() & 0b111111);
  timeWeekday = bcd2dec(Wire.read());
  timeDay     = bcd2dec(Wire.read());
  timeMonth   = bcd2dec(Wire.read());
  timeYear    = bcd2dec(Wire.read());
  
  //Something is REALLY Wrong with DS1307
  if( ( timeSecond > 59 ) or ( timeMinute > 59 ) or (timeHour > 23) or (timeWeekday > 6) )
  {
    btSerial.println("DS1307 error");
    //Set Any Value for DS1307 Time
    setTimeDS1307( 17 , 7 , 3, 1, 14, 0 ); //year, month, day, day_week, (0 = Sun, 6 = Sat), hour, minute
    //Request Time Online
    Serial.write( START_CHAR );Serial.print("RTO");Serial.write( END_CHAR );
  }
  else
  {
    btSerial.println("RTC Checked");
  }

  btSerial.println( String(timeDay)  + "/" + String(timeMonth)  + "/" + String(timeYear) + " wday=" + String(timeWeekday)  );
  btSerial.println( String(timeHour) + ":" + String(timeMinute) + ":" + String(timeSecond) );
  
}
//---------------------------------------------------------------




//---------------------------------------------------------------
//               FUNCTIONS: RF24 COMMUNICATION
//---------------------------------------------------------------
void handleRF24(){
  //IS THERE RADIO SIGNAL INCOMING?
  while (radio.available())
  {
    char recebidos[60] = "";
    radio.read( recebidos , 59 );
    messageRF24 = recebidos;
    handleMessage();
    delay( 10 );
  }
}

void handleMessage(){
  //DUMPING INVALID CHARS
  while( (messageRF24[0] != START_CHAR) and (messageRF24.length() > 1) ){
    messageRF24 = messageRF24.substring(1);
  }

  btSerial.print("RF IN: ");
  while( (messageRF24[0] == START_CHAR) and (messageRF24.length() > 1) )
  {
    
    //HANDLING VALID MESSAGES
    if( (messageRF24[0] == START_CHAR) & (messageRF24[1] == 'V') )
    {
      prototypeStatus = true;
      prototypeStatusCounter = 0;
      
      int8_t endCharIndex = messageRF24.indexOf( END_CHAR );
      btSerial.println( messageRF24.substring(0,endCharIndex) );
      if(      messageRF24[2] == '0' ){ CTRLpowerGenerated   = (messageRF24.substring(3, endCharIndex)).toFloat(); }
      else if( messageRF24[2] == '1' ){ CTRLtempLM35         = (messageRF24.substring(3, endCharIndex)).toFloat(); }
      else if( messageRF24[2] == '2' ){ CTRLlux              = (messageRF24.substring(3, endCharIndex)).toFloat(); }
      else if( messageRF24[2] == '3' ){ CTRLcurrent_mA       = (messageRF24.substring(3, endCharIndex)).toFloat(); }
      else if( messageRF24[2] == '4' ){ CTRLvoltage          = (messageRF24.substring(3, endCharIndex)).toFloat(); }
      else if( messageRF24[2] == '5' ){ CTRLcurrentGenerated = (messageRF24.substring(3, endCharIndex)).toFloat(); }
      CTRLpower_mW = CTRLcurrent_mA * CTRLvoltage;
      /*
      else if( messageRF24[2] == '5' ){ CTRLpower_mW        = (messageRF24.substring(4, endCharIndex)).toFloat(); }
      else if( messageRF24[2] == '6' ){ CTRLphotoUR         = (messageRF24.substring(4, endCharIndex)).toInt(); }
      else if( messageRF24[2] == '7' ){ CTRLphotoDR         = (messageRF24.substring(4, endCharIndex)).toInt(); }
      else if( messageRF24[2] == '8' ){ CTRLphotoUL         = (messageRF24.substring(4, endCharIndex)).toInt(); }
      else if( messageRF24[2] == '9' ){ CTRLphotoDL         = (messageRF24.substring(4, endCharIndex)).toInt(); }
      */
    messageRF24 = messageRF24.substring(endCharIndex);
    if( messageRF24.length()>1 )
    {
      messageRF24 = messageRF24.substring(1);
    }
  }
    
  }
}
//---------------------------------------------------------------


//---------------------------------------------------------------
//             FUNCTIONS: WEB SERVER COMMUNICATION
//---------------------------------------------------------------
/**
 * Generic Send Message Used Inside refreshServer()
 */
void sendMessage2ESP(String *message)
{
  btSerial.println("WO: "+*message);
  while( Serial.availableForWrite() < (*message).length() + 2 ){}
  Serial.write( START_CHAR );
  Serial.print( *message );
  Serial.write( END_CHAR );
}

/**
 * Send updated information to ESP8266
 */
void refreshServer()
{
  String message = "";
  //--------------------------------------------------
  //            SOLAR TRACKER DATA TO WEB
  //--------------------------------------------------
  //Energy Generated
  message = "V07" + String( powerGenerated, 2 );
  sendMessage2ESP( &message );
  //Voltage
  message = "V08" + String( voltage, 2 );
  sendMessage2ESP( &message );
  //Current
  message = "V09" + String( current_mA, 2 );
  sendMessage2ESP( &message );
  //Power
  message = "V10" + String( power_mW, 2 );
  sendMessage2ESP( &message );
  //Temperature
  message = "V11" + String( tempLM35 , 2);
  sendMessage2ESP( &message );
  //LUX
  message = "V12" + String( lux, 2 );
  sendMessage2ESP( &message );
  //Photo UL
  message = "V13" + String(photoUL[0]);
  sendMessage2ESP( &message );
  //Photo UR
  message = "V14" + String(photoUR[0]);
  sendMessage2ESP( &message );
  //Photo DL
  message = "V15" + String(photoDL[0]);
  sendMessage2ESP( &message );
  //Photo DR
  message = "V16" + String(photoDR[0]);
  sendMessage2ESP( &message );
  //REPORT MILLIAMPERE HOUR
  message = "V31" + String( currentGenerated , 2 );
  sendMessage2ESP( &message );
  /*
  //Pitch Angle
  message = "V17" + String(pitchAngle,0);
  sendMessage2ESP( &message );
  //Yaw Angle
  message = "V18" + String( (int16_t)((stepCounterM1 - STEPS_MAX_M1/2) * 90/STEPS_MAX_M1) );
  sendMessage2ESP( &message );
  */
  //--------------------------------------------------


  if( prototypeStatus )
  {
    //--------------------------------------------------
    //            CONTROL UNIT TO WEB
    //--------------------------------------------------
    //Energy Generated
    message = "V19" + String( CTRLpowerGenerated, 2 );
    sendMessage2ESP( &message );
    //Voltage
    message = "V20" + String( CTRLvoltage, 2 );
    sendMessage2ESP( &message );
    //Current
    message = "V21" + String( CTRLcurrent_mA, 2 );
    sendMessage2ESP( &message );
    //Power
    message = "V22" + String( CTRLpower_mW, 2 );
    sendMessage2ESP( &message );
    //Temperature
    message = "V23" + String( CTRLtempLM35 , 2);
    sendMessage2ESP( &message );
    //LUX
    message = "V24" + String( CTRLlux, 2 );
    sendMessage2ESP( &message );

    //REPORT MILLIAMPERE HOUR
    message = "V32" + String( CTRLcurrentGenerated , 2 );
    sendMessage2ESP( &message );
  
    /*
    //Photo UL
    message = "V25" + String(CTRLphotoUL);
    sendMessage2ESP( &message );
    //Photo UR
    message = "V26" + String(CTRLphotoUR);
    sendMessage2ESP( &message );
    //Photo DL
    message = "V27" + String(CTRLphotoDL);
    sendMessage2ESP( &message );
    //Photo DR
    message = "V28" + String(CTRLphotoDR);
    sendMessage2ESP( &message );
    //Pitch Angle
    message = "V29" + String(CTRLpitchAngle,0);
    sendMessage2ESP( &message );
    //Yaw Angle
    message = "V300";
    sendMessage2ESP( &message );
    */
    //--------------------------------------------------
  }
  
  //HOUR TIME / 8 TO 10 BYTES
  message = "MT1" + (String)(timeHour) + ":" + (String)(timeMinute);
  sendMessage2ESP( &message );

  //DATE / 10 TO 13 BYTES
  message = "MT2" + (String)(timeDay) + "/" + (String)(timeMonth) + "/" + (String)(timeYear);
  sendMessage2ESP( &message );

  //WEEKDAY / 6 BYTES
  message = "MT3" + (String)(timeWeekday);
  sendMessage2ESP( &message );

  //REPORT PROTOTYPE STATUS
  message = "MSS1" + (String)( (int)(prototypeStatus) );
  sendMessage2ESP( &message );

  //REPORT OPERATION MODE
  message = "OPM" + (String)( (int)(operationMode) );
  sendMessage2ESP( &message );

  if( firstTime ){
    firstTime = false;
    //REPORT MCU STARTING
    message = "MRP1";
    sendMessage2ESP( &message );
  }
  
}

/**
 * Check if there is an incoming message from ESP8266
 */
void serverHandler()
{

  if (Serial.available() > 0)
  {
    stillBuffering = false;
    //------------------------------------------------------------
    //     READING UART VALUES AS VALID ASCII CHARACTERS ONLY
    //------------------------------------------------------------
    //Serial.setTimeout(10);
    uint8_t bufLen = Serial.readBytes(buf1, 10);
    buf1[bufLen] = '\0';
    uint8_t i=0, j = 0;
    for (i = 0; i < bufLen; i++)
    {
      if ((buf1[i] == START_CHAR) or (buf1[i] == END_CHAR) or ((buf1[i] > 32) && (buf1[i] < 127)))
      {
        buf1[j++] = buf1[i];
      }
    }
    buf1[j] = '\0';

    buffer_read += buf1;
    msgLen = buffer_read.length();
    //------------------------------------------------------------
  }

  if ((msgLen > 0) and (stillBuffering == false))
  {
    
    if (DEBUG_PRINTING)
    {
      btSerial.println(buffer_read);
    }
    
    //FLUSHING CHARS UNTILL A VALID START CHAR IS FOUND
    while ((buffer_read.indexOf(START_CHAR) != 0) and (msgLen > 0))
    {
      buffer_read = buffer_read.substring(1);
      msgLen--;
    }

    //WORD RECEIVED STARTS WITH A VALID START CHAR
    if (buffer_read.indexOf(START_CHAR) == 0)
    {
      stillBuffering = true;

      //WORD RECEIVED HAS THE VALID END CHAR
      if (buffer_read.indexOf(END_CHAR) >= 4)
      {
        stillBuffering = false;

        //ESP8266 CONNECTED
        if ((buffer_read.substring(1, 4) == "WEB") && (buffer_read.indexOf(END_CHAR) == 6))
        {
          refreshServer();
        }

        //GETTING EXISTING WIFI ESP8266 SERVER IP
        //if ((buffer_read.substring(1, 4) == "IP0") && (buffer_read.indexOf(END_CHAR) <= 19))

        
        //CHANGE OPERATION MODE
        if ( (buffer_read.substring(1, 4) == "OPM") ){
          operationMode = ( buffer_read.substring(4,5) ).toInt();
          if( operationMode > 47 ){ operationMode -= 48; }
        }
        
        //SET TIME AND DATE VALUES
        if ( (buffer_read.substring(1, 3) == "WT") or (buffer_read.substring(1, 3) == "WD") ){
          //  SET TIME HOUR
          if ((buffer_read.substring(1, 4) == "WTH") && (buffer_read.indexOf(END_CHAR) <= 6))
          {
            timeHour = (buffer_read.substring(4, buffer_read.indexOf(END_CHAR))).toInt();
          }
  
          //  SET TIME MINUTE
          if ((buffer_read.substring(1, 4) == "WTM") && (buffer_read.indexOf(END_CHAR) <= 6))
          {
            timeMinute = (buffer_read.substring(4, buffer_read.indexOf(END_CHAR))).toInt();
          }
  
          //  SET DATE DAY
          if ((buffer_read.substring(1, 4) == "WDD") && (buffer_read.indexOf(END_CHAR) <= 6))
          {
            timeDay = (buffer_read.substring(4, buffer_read.indexOf(END_CHAR))).toInt();
          }
  
          //  SET DATE MONTH
          if ((buffer_read.substring(1, 4) == "WDM") && (buffer_read.indexOf(END_CHAR) <= 6))
          {
            timeMonth = (buffer_read.substring(4, buffer_read.indexOf(END_CHAR))).toInt();
          }
  
          //  SET DATE YEAR
          if ((buffer_read.substring(1, 4) == "WDY") && (buffer_read.indexOf(END_CHAR) <= 6))
          {
            timeYear = (buffer_read.substring(4, buffer_read.indexOf(END_CHAR))).toInt();
          }
  
          //  SET DATE WEEKDAY
          if ((buffer_read.substring(1, 4) == "WDW") && (buffer_read.indexOf(END_CHAR) == 5))
          {
            timeWeekday = (buffer_read.substring(4, 5)).toInt();
          }

          setTimeDS1307(timeYear, timeMonth, timeDay, timeWeekday, timeHour, timeMinute);
          refreshServer();
        }
      }
      

      //DELETING THE PART OF THE MSG THAT HAS ALREADY BEEN READ
      if (buffer_read.indexOf(END_CHAR) > 0)
      {
        buffer_read = buffer_read.substring(buffer_read.indexOf(END_CHAR) + 1);
        msgLen = buffer_read.length();
      }
    }
  }
}
//---------------------------------------------------------------



//---------------------------------------------------------------
//                      SOLAR TRACKER FUNCTIONS
//---------------------------------------------------------------
void refreshSensorData()
{
  /*
    * Get new sensors data:
    * Power Monitor
    * Temperature
    * LUX
    * Angle (MPU)
    * Photo and Stepper data are updated somewhere else
  */
  btSerial.println("Refreshing data");
  
  //POWER MONITOR
  current_mA        = ina219.getCurrent_mA();
  voltage           = ina219.getBusVoltage_V() + (ina219.getShuntVoltage_mV() / 1000);
  power_mW          = ina219.getPower_mW();

  //TEMPERATURE
  tempLM35 = (double(analogRead(A6))*5/(1023))/0.01;

  //LUX
  unsigned int data0, data1;
  if (light.getData(data0,data1))
  {
    isAtMax = light.getLux(0,ms,data0,data1,lux);
  }
  else
  {
    btSerial.println("LUX ERROR");
  }

  /*
  //MPU PITCH
  IMU.readSensor();
  double accelX,accelY,accelZ;
  accelX      = IMU.getAccelX_mss();
  accelY      = IMU.getAccelY_mss();
  accelZ      = IMU.getAccelZ_mss();
  pitchAngle  = atan2(  accelY , sqrt( (accelX * accelX) + (accelZ * accelZ) ) ) * RAD_TO_DEG;
  */
}

void refreshPhotoData()
{
  static uint8_t movingWindowsIndex = 1;
  
  photoUR[ movingWindowsIndex ] = ads.readADC_SingleEnded(0) * 3; //mV
  photoDR[ movingWindowsIndex ] = ads.readADC_SingleEnded(1) * 3; //mV
  photoUL[ movingWindowsIndex ] = ads.readADC_SingleEnded(2) * 3; //mV
  photoDL[ movingWindowsIndex ] = ads.readADC_SingleEnded(3) * 3; //mV
  movingWindowsIndex++;
  if( movingWindowsIndex > PHOTO_READINGS )
  {
    movingWindowsIndex = 1;
    uint16_t sum;
    sum = 0;
    for( uint8_t j=0; j<PHOTO_READINGS; j++ ){ sum += photoUR[j+1]; }
    photoUR[0] = sum / PHOTO_READINGS;
    sum = 0;
    for( uint8_t j=0; j<PHOTO_READINGS; j++ ){ sum += photoDR[j+1]; }
    photoDR[0] = sum / PHOTO_READINGS;
    sum = 0;
    for( uint8_t j=0; j<PHOTO_READINGS; j++ ){ sum += photoUL[j+1]; }
    photoUL[0] = sum / PHOTO_READINGS;
    sum = 0;
    for( uint8_t j=0; j<PHOTO_READINGS; j++ ){ sum += photoDL[j+1]; }
    photoDL[0] = sum / PHOTO_READINGS;
  }
}

void solarTracker()
{ 
  if( stilFollowing || NIGHTTIME_DEBUG )
  {
    delay( 100 );                           //POWER PROTECTION
    int16_t mean0, mean1;
    uint8_t tries = 0;
    while( tries < 50)
    {
      tries++;
      bool correctionNeeded = false;
      //YAW CONTROL
      refreshPhotoData();
      mean0 = (photoUL[0] + photoDL[0])/2.0;
      mean1 = (photoUR[0] + photoDR[0])/2.0;
      int16_t checkAxis = ((int16_t)((100.0*( mean1 - mean0 ))/mean1)) / 10;
      if( checkAxis > 0  )      //GO RIGHT
      {
        correctionNeeded = true;
        stepCounterM1 -= STEPS_PER_CHANGE_M1;
        myStepper1.step( -STEPS_PER_CHANGE_M1 );
      }else if( checkAxis < 0  ) //GO LEFT
      {
        correctionNeeded = true;
        stepCounterM1 += STEPS_PER_CHANGE_M1;
        myStepper1.step( STEPS_PER_CHANGE_M1 );
      }
      
      delay( 5 );
  
      //PITCH CONTROL
      refreshPhotoData();
      mean0 = (photoDL[0] + photoDR[0])/2.0;
      mean1 = (photoUL[0] + photoUR[0])/2.0;
      checkAxis = ((int16_t)((100.0*( mean1 - mean0 ))/mean1)) / 10;
      if( checkAxis > 0  )      //GO UP
      {
        correctionNeeded = true;
        stepCounterM2 -= STEPS_PER_CHANGE_M2;
        myStepper2.step( -STEPS_PER_CHANGE_M2 );
      }
      else if( checkAxis < 0  ) //GO DOWN
      {
        correctionNeeded = true;
        stepCounterM2 += STEPS_PER_CHANGE_M2;
        myStepper2.step( STEPS_PER_CHANGE_M2 );
      }
      delay( 5 );
  
      //No motor was adjusted this time, so there is no need to keep in this loop for a while
      if( !correctionNeeded ){ break; }
      
        /*
        if( stepCounterM1 - STEPS_PER_CHANGE_M1 > STEPS_PER_CHANGE_M1)
        {
          stepCounterM1 -= STEPS_PER_CHANGE_M1;
          myStepper1.step( -STEPS_PER_CHANGE_M1 );
        }
        else
        {
          btSerial.println("M1 got min");
        }
      }
      else if( checkAxis < 0  ) //GO LEFT
      {
        if( stepCounterM1 + STEPS_PER_CHANGE_M1 < STEPS_MAX_M1 )
        {
          stepCounterM1 += STEPS_PER_CHANGE_M1;
          myStepper1.step( STEPS_PER_CHANGE_M1 );
        }
        else
        {
          btSerial.println("M1 got max");
        }
      }
      delay( 5 );
  
      //PITCH CONTROL
      refreshPhotoData();
      mean0 = (photoDL[0] + photoDR[0])/2.0;
      mean1 = (photoUL[0] + photoUR[0])/2.0;
      checkAxis = ((int16_t)((100.0*( mean1 - mean0 ))/mean1)) / 10;
      if( checkAxis > 0  )      //GO UP
      {
        if( stepCounterM2 - STEPS_PER_CHANGE_M2 > STEPS_PER_CHANGE_M2)
        {
          stepCounterM2 -= STEPS_PER_CHANGE_M2;
          myStepper2.step( -STEPS_PER_CHANGE_M2 );
        }
        else
        {
          btSerial.println("M2 got min");
        }
      }
      else if( checkAxis < 0  ) //GO DOWN
      {
        if( stepCounterM2 + STEPS_PER_CHANGE_M2 < STEPS_MAX_M2 )
        {
          stepCounterM2 += STEPS_PER_CHANGE_M2;
          myStepper2.step( STEPS_PER_CHANGE_M2 );
        }
        else
        {
          btSerial.println("M2 got max");
        }
      }
      delay( 5 );
      */
  
    }
    delay( 100 );
  }
}

//---------------------------------------------------------------
//                  EXTERNAL DATA LOGGER FUNCIONS
//---------------------------------------------------------------
void loggerSetup()
{
  //---------------------------------------------
  //             EXTERNAL LOGGER SETUP
  //---------------------------------------------
  btSerial.println("LOGGER SETUP");
  for( uint8_t k=0; k<10; k++ )
  {
    if( loggerSerial.available() > 0 )
    {
      btSerial.println( loggerSerial.readString() );
      //noInterrupts();
      loggerSerial.write( START_CHAR );
      String filename = "";
      if( timeYear < 10 ){ filename  = String("0") + String( timeYear); }
      else{                filename  = String(timeYear); }
      filename += "-";
      if( timeMonth < 10){ filename += String("0") + String( timeMonth ); }
      else{                filename += String(timeMonth); }
      filename += "-";
      if( timeDay   < 10){ filename += String("0") + String( timeDay ); }
      else{                filename += String( timeDay ); }
      loggerSerial.print( filename );
      loggerSerial.write( END_CHAR );
      btSerial.println( "FN = [" + filename + "]" );
      //interrupts();
      delay(100);
      if( loggerSerial.available() > 0 )
      {
        btSerial.println( loggerSerial.readString() );
      }
      break;
    }
    delay( 1000 );
  }
  //---------------------------------------------
}


void logData(){
  delay( 1000 );
  String data = "";
  data = "[" + String( timeHour ) + ":" + String( timeMinute ) + "]";
  sendMessage2Logger( data );
  sendMessage2Logger("Solar Tracker");
  if(      operationMode == 0 )
  {
    data = "\tTotal Current [mAh]: " + String(currentGenerated,5);
    sendMessage2Logger( data );
  }
  else if( operationMode == 1 )
  {
    data = "\tTotal Power   [mWh]: " + String(powerGenerated   ,5);
    sendMessage2Logger( data );
    data = "\tVoltage         [V]: " + String(voltage          ,5);
    sendMessage2Logger( data );
    data = "\tPower          [mW]: " + String(power_mW         ,5);
    sendMessage2Logger( data );
  }
  if( operationMode != 2 )
  {
    data = "\tCurrent        [mA]: " + String(current_mA      ,5);
    sendMessage2Logger( data );
  }
  data = "\tTemperature     [C]: " + String(tempLM35        ,2);
  sendMessage2Logger( data );

  if( !isAtMax )
  {
    data = "\tIlluminance    [lx]: " + String(lux             ,2);
  }
  else
  {
    data = "\tIlluminance    [lx]: max";
  }
  sendMessage2Logger( data );
  data = "\t Photo Up   Left  [mV]: " + String(photoUL[0]);sendMessage2Logger( data );
  data = "\t Photo Down Left  [mV]: " + String(photoDL[0]);sendMessage2Logger( data );
  data = "\t Photo Up   Right [mV]: " + String(photoUR[0]);sendMessage2Logger( data );
  data = "\t Photo Down Right [mV]: " + String(photoDR[0]);sendMessage2Logger( data );
  


  sendMessage2Logger("Control Unit");
  if(      operationMode == 0 )
  {
    sendMessage2Logger( "\tTotal Current [mAh]: " + String(CTRLcurrentGenerated,5) );
  }
  else if( operationMode == 1 )
  {
    sendMessage2Logger("\tTotal Power   [mWh]: " + String(CTRLpowerGenerated   ,5));
    sendMessage2Logger("\tVoltage         [V]: " + String(CTRLvoltage          ,5));
    sendMessage2Logger("\tPower          [mW]: " + String(CTRLpower_mW         ,5));
  }
  if( operationMode != 2 )
  {
    sendMessage2Logger("\tCurrent        [mA]: " + String(CTRLcurrent_mA      ,5));
  }
  sendMessage2Logger("\tTemperature    [ C]: " + String(CTRLtempLM35        ,2));
  if( ( isAtMax ) && (CTRLlux == 0) )
  {
    sendMessage2Logger("\tIlluminance    [lx]:  max");
  }
  else
  {
    sendMessage2Logger("\tIlluminance    [lx]: " + String(CTRLlux             ,2));
  }
  sendMessage2Logger("");
}

void sendMessage2Logger(String message)
{
  btSerial.println("LGTx: "+message);
  delay(300);
  loggerSerial.println( message );
  delay(300);
}
