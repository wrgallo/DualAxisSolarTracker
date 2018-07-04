#include "myNTP.h"

const char* NTPServer0 = "a.st1.ntp.br";
const char* NTPServer1 = "time.nist.gov";

bool usingMainServer = 1;
char NTPServer[30];

WiFiUDP UDP;                     // Create an instance of the WiFiUDP class to send and receive
IPAddress timeServerIP;          // time.nist.gov NTP server address
#define LEAP_YEAR(Y)     ( ((1970+Y)>0) && !((1970+Y)%4) && ( ((1970+Y)%100) || !((1970+Y)%400) ) ) // leap year calulator expects year argument as years offset from 1970
static  const uint8_t monthDays[]={31,28,31,30,31,30,31,31,30,31,30,31};
const uint8_t NTPPort = 123;
const int NTP_PACKET_SIZE = 48;  // NTP time stamp is in the first 48 bytes of the message
byte NTPBuffer[NTP_PACKET_SIZE]; // buffer to hold incoming and outgoing packets

uint8_t UNIXvalues[7] = {0,0,0,0,0,0,0}; //Hour, Minute, Second, Weekday, Day, Month, Year


//--------------------------------------------------------------------
//                        PRIVATE FUNCTIONS
//--------------------------------------------------------------------
uint32_t checkTime();
void sendNTPpacket(IPAddress& address);
void breakTime(uint32_t UNIXTime);

uint32_t checkTime() {
  if (UDP.parsePacket() == 0) { // If there's no response (yet)
    return 0;
  }
  UDP.read(NTPBuffer, NTP_PACKET_SIZE); // read the packet into the buffer
  // Combine the 4 timestamp bytes into one 32-bit number
  uint32_t NTPTime = (NTPBuffer[40] << 24) | (NTPBuffer[41] << 16) | (NTPBuffer[42] << 8) | NTPBuffer[43];
  // Convert NTP time to a UNIX timestamp:
  // Unix time starts on Jan 1 1970. That's 2208988800 seconds in NTP time:
  const uint32_t seventyYears = 2208988800UL;
  // subtract seventy years:
  uint32_t UNIXTime = NTPTime - seventyYears;
  return UNIXTime;
}

void sendNTPpacket(IPAddress& address) {
  // set all bytes in the buffer to 0
  memset(NTPBuffer, 0, NTP_PACKET_SIZE);
  
  // Initialize values needed to form NTP request
  NTPBuffer[0] = 0b11100011;

  // NTP Request
  UDP.beginPacket(address, NTPPort);
  UDP.write(NTPBuffer, NTP_PACKET_SIZE);
  UDP.endPacket();
}

void breakTime(uint32_t UNIXTime){
  //uint8_t UNIXvalues[7] = {0,0,0,0,0,0,0}; //Hour, Minute, Second, Weekday (0=Sun, 6=Sat), Day, Month, Year (00 to 99)
  uint8_t yearCounter, monthCounter, monthLength;
  unsigned long daysCounter;

  //
  UNIXvalues[0] = (UNIXTime / 3600) % 24;
  UNIXvalues[1] = (UNIXTime / 60) % 60;
  UNIXvalues[2] = (UNIXTime % 60);
  UNIXvalues[3] = (UNIXTime / 86400 + 4)%7;
  //
  UNIXTime /= 86400; // now it is days
  
  yearCounter = 0;  
  daysCounter = 0;
  while((unsigned)(daysCounter += (LEAP_YEAR(yearCounter) ? 366 : 365)) <= UNIXTime) {
    yearCounter++;
  }
  //00 to 99 year format
  UNIXvalues[6] = (1970 + yearCounter)%100;
  
  daysCounter -= LEAP_YEAR(yearCounter) ? 366 : 365;
  UNIXTime    -= daysCounter; // now it is days in this year, starting at 0
  
  monthLength=0;
  for (monthCounter=0; monthCounter<12; monthCounter++) {
    if (monthCounter==1) { // february
      if (LEAP_YEAR(yearCounter)) {
        monthLength=29;
      } else {
        monthLength=28;
      }
    } else {
      monthLength = monthDays[monthCounter];
    }
    
    if (UNIXTime >= monthLength) {
      UNIXTime -= monthLength;
    } else {
        break;
    }
  }
  
  UNIXvalues[4] = UNIXTime + 1;     // day of month
  UNIXvalues[5] = monthCounter + 1;  // jan is month 1
}
//--------------------------------------------------------------------



//--------------------------------------------------------------------
//                        PUBLIC FUNCTIONS
//--------------------------------------------------------------------
bool requestUTC(int8_t timeZone){
  static bool firstRun = true;
  static bool problemLastRun = false;
  
  if( firstRun ){
    firstRun = false;
    UDP.begin(NTPPort);

    if(   usingMainServer ){ strcpy(NTPServer,NTPServer0); }
    else{                    strcpy(NTPServer,NTPServer1); }
    
    if(!WiFi.hostByName(NTPServer, timeServerIP)){
      if(!problemLastRun){
        Serial.println("\nNTP failed, Trying Another");
        usingMainServer=!usingMainServer;
        problemLastRun=1;
        firstRun=1;
        requestUTC(timeZone);
      }
      else{return 0;}
    }
    
    sendNTPpacket(timeServerIP);
    delay(1000);
  }
  
  sendNTPpacket(timeServerIP);
  delay(1);
  uint32_t value = checkTime();
  if(!value){
    if(!problemLastRun){
      Serial.println("\nNTP Server failed, Trying Another");
      usingMainServer=!usingMainServer;
      problemLastRun=1;
      firstRun=1;
      requestUTC(timeZone);
    }
    else{return 0;}
  }
  else{
    value += timeZone * 3600;
    breakTime(value);  
    return 1;
  }
}

uint8_t getHour()   { return UNIXvalues[0]; }
uint8_t getMinute() { return UNIXvalues[1]; }
uint8_t getSecond() { return UNIXvalues[2]; }
uint8_t getWeekday(){ return UNIXvalues[3]; }
uint8_t getDay()    { return UNIXvalues[4]; }
uint8_t getMonth()  { return UNIXvalues[5]; }
uint8_t getYear()   { return UNIXvalues[6]; }


