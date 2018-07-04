#include "Arduino.h"
#include "myFunctions.h"
//#include "myEEPROM.h"

#define DEGUB_PRINTING 1

/*
  ARDUINO PRO MINI 5V/16MHZ - ATMEGA328P - PINOUT
  -----------------------------------------------
 | PIN |  PURPOSE                                |
 |-----|-----------------------------------------|
 | 9   |  CE   of NRF24L01+                      |
 | 10  |  CSN  for NRF24L01+                     |
 | 11  |  MOSI for NRF24L01+                     |
 | 12  |  MISO for NRF24L01+                     |
 | 13  |  SCK  for NRF24L01+                     |
 | A4  |  SDA FOR I2C COMMUNICATION              |
 | A5  |  SCL FOR I2C COMMUNICATION              |
 | A6  |  ADC INPUT FOR LM35 Vout                |
 | A7  |  NC                                     |
  -----------------------------------------------
  

  ---------------------------------------------
 | Power Supply  | Peripheral                  |
 |---------------|-----------------------------
 | +5V           |  INA219                     |
 | +3v3          |  TSL2561                    |
 | +3v3          |  NRF24L01+                  |
  ---------------------------------------------
*/

volatile bool timerFlag = false;
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
//                        LUXIMETER DEFINITION
//---------------------------------------------------------------
SFE_TSL2561 light;                                                //TSL2561 OBJECT
unsigned int ms = 400;                                            //INTEGRATION ("shutter") TIME IN MILLISECONDS
double lux;                                                       //LUX OBTAINED
//---------------------------------------------------------------
//                      POWER MONITOR DEFINITION
//---------------------------------------------------------------
Adafruit_INA219 ina219;
double energyGenerated = 0;                                      //mWh from This Unit
double currentGenerated = 0;                                     //mAh from This Unit
float current_mA  = 0;
float loadvoltage = 0;
float power_mW    = 0;
//---------------------------------------------------------------
//                         TEMPERATURE LM35
//---------------------------------------------------------------
double tempLM35 = 0;

//---------------------------------------------------------------
//             FUNCTIONS: ARDUINO CONFIGURATION
//---------------------------------------------------------------
void configThisUnit()
{
  //INPUT | OUTPUT CONFIGURATION
  Serial.begin(115200);

  if( DEBUG_PRINTING ){ Serial.println("Starting LUX"); }
  //LUXIMETER START SETUP
  light.begin();
  light.setTiming( 0 , 2 , ms );
  light.setPowerUp();

  if( DEBUG_PRINTING ){ Serial.println("Starting INA219"); }
  //POWER MONITOR START SETUP
  ina219.begin();
  ina219.setCalibration_16V_400mA();                                            // max precision on volts and amps

  if( DEBUG_PRINTING ){ Serial.println("Starting RF24"); }
  //--------------------------------------------
  //             RF24 CONFIGURATION
  //--------------------------------------------
  //Start Communication
  radio.begin();

  radio.setPALevel( RF24_PA_LOW   );
  radio.setDataRate( RF24_250KBPS );

  radio.openWritingPipe(     pipes[0] ); //When Answering the Master
  //radio.openReadingPipe( 1 , pipes[1] ); //The Pipe where Master Answers This Slave
  radio.startListening();                // Start listening
  //--------------------------------------------

  //--------------------------------------------
  //             TIMER CONFIGURATION
  //--------------------------------------------
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
}

//---------------------------------------------------------------


ISR(TIMER1_COMPA_vect){
  static uint8_t ISRCounter = 0;
  if( !timerFlag ){
    ISRCounter++;
    if( ISRCounter == 60 ){
      ISRCounter = 0;
      timerFlag = true; 
    }
  }
}

uint8_t checkTimerFlag()
{
  if( timerFlag )
  {
    timerFlag = false;
    return 1;
  }
  return 0;
}

//---------------------------------------------------------------
//               FUNCTIONS: RF24 COMMUNICATION
//---------------------------------------------------------------
void report2Master(){
  /*
  char message[10] = "#VxDDDDDD#";
  message[0] = START_CHAR;
  
  message[9] = END_CHAR;
  */
  String msg      = ((char)(START_CHAR)) + String("V0") + String( energyGenerated , 2 ) + ((char)(END_CHAR)) + 
                    ((char)(START_CHAR)) + String("V1") + String(        tempLM35 , 2 ) + ((char)(END_CHAR));
  
  char message[60];
  messageS.toCharArray(msg1 , 60);
  radio.stopListening();
  if( DEGUB_PRINTING ){ Serial.println("  Writing Message"); }
  radio.write( message, sizeof(message) );
  radio.startListening();
  if( DEGUB_PRINTING ){ 
    Serial.println(" ");
    Serial.print("Text Sent: [");
    Serial.print( message );
    Serial.println("]");
  }

  msg             = ((char)(START_CHAR)) + String("V2") + String(             lux , 2 ) + ((char)(END_CHAR)) + 
                    ((char)(START_CHAR)) + String("V3") + String(      current_mA , 2 ) + ((char)(END_CHAR));
  char message[60];
  messageS.toCharArray(msg1 , 60);
  radio.stopListening();
  if( DEGUB_PRINTING ){ Serial.println("  Writing Message"); }
  radio.write( message, sizeof(message) );
  radio.startListening();
  if( DEGUB_PRINTING ){ 
    Serial.println(" ");
    Serial.print("Text Sent: [");
    Serial.print( message );
    Serial.println("]");
  }

  msg             = ((char)(START_CHAR)) + String("V4") + String(     loadvoltage , 2 ) + ((char)(END_CHAR)) + 
                    ((char)(START_CHAR)) + String("V5") + String(currentGenerated , 2 ) + ((char)(END_CHAR));
  char message[60];
  messageS.toCharArray(msg1 , 60);
  radio.stopListening();
  if( DEGUB_PRINTING ){ Serial.println("  Writing Message"); }
  radio.write( message, sizeof(message) );
  radio.startListening();
  if( DEGUB_PRINTING ){ 
    Serial.println(" ");
    Serial.print("Text Sent: [");
    Serial.print( message );
    Serial.println("]");
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
  
  //POWER MONITOR
  current_mA       = ina219.getCurrent_mA();
  loadvoltage      = ina219.getBusVoltage_V() + (ina219.getShuntVoltage_mV() / 1000);
  power_mW         = ina219.getPower_mW();
  energyGenerated  += power_mW    * 1.66666e-2; //mW times (1/60 hours)
  currentGenerated += current_mA  * 1.66666e-2; //mA times (1/60 hours)

  //TEMPERATURE
  tempLM35 = (double(analogRead(LM35_PIN))*5/(1023))/0.01;

  //LUX
  unsigned int data0, data1;
  if (light.getData(data0,data1))
  {
    light.getLux(0,ms,data0,data1,lux);
  }

  if( DEGUB_PRINTING ){ 
    Serial.println("Energy Generated = " + String(energyGenerated ,2) + " mWh");
    Serial.println("Power            = " + String(power_mW        ,2) + " mW");
    Serial.println("loadvoltage      = " + String(loadvoltage     ,2) + " V");
    Serial.println("current_mA       = " + String(current_mA      ,2) + " mA");
    Serial.println("tempL35          = " + String(tempLM35        ,2) + " mA");
    Serial.println("LUX              = " + String(lux             ,2) + " lx");
  }
}
