// GPS to RTC time-sync by Ed Mallon
//====================================================================================================================
// This code works with an Arduino UNO driving the GPS at 5v, and the UNOs 3v & I2C connected to the 3v loggers I2C bus
// WITH the logger's ProMini asleep or running blink the entire time so it does not pay any attention to the I2C traffic
// Loggers should have perfectly synchronized LED blinks after this procedure.
// This code assumes that the GPS PPS is connected to D3 and the RTC SQW is connected to D2
// see the accompanying blog post at: 
//=====================================================================================================================
// This program supports an ongoing series of DIY 'Classroom Logger' tutorials from the Cave Pearl Project. 
// The goal is to provide a starting point for self-built student projects in environmental monitoring at Northwestern University

#include <Wire.h>

#include <SoftwareSerial.h>   // Include software serial library because UNO only has one serial line
static const int RXPin = 12, TXPin = 11; // the pin#s on the ARDUINO side
static const uint32_t GPSBaud = 9600; //default baud rate for the UART GPS NEO-6M is 9600
SoftwareSerial SerialGPS = SoftwareSerial(RXPin,TXPin);

#include <TinyGPS++.h>        // https://arduiniana.org/libraries/tinygpsplus/
TinyGPSPlus gps;

#include <fast_math.h>        // https://github.com/RobTillaart/fast_math/tree/master
uint32_t dmodQuotient;        // speeds up some of the calculations
uint8_t dmodRemainder;

//defines & variables for DS3231 RTC
//------------------------------------------------------------------------------
#define rtcAlarmInputPin 2                                // DS3231's SQW output is connected to interrupt0 pin D2 on the ProMini
#define DS3231_ADDRESS     0x68                           // this is the I2C bus address of our RTC chip
#define DS3231_STATUS_REG  0x0F                           // reflects status of internal operations
#define DS3231_CONTROL_REG 0x0E                           // enables or disables clock functions
char CycleTimeStamp[] = "0000-00-00,00:00:00";            // 16 character array to store human readble time (with seconds)
uint8_t t_second,t_minute,t_hour,t_day,t_month,t_dow;     // current time variables populated by calling RTC_DS3231_getTime()
uint16_t t_doy,t_year;                                    // current year //note: yOff = raw year to which you need to add 2000
bool rtcTimeHasBeenSet = false;

char gpsTime[]  = "GPSt:00:00:00";
char gpsDate[]  = "GPSd:2000-00-00";                      //International standard date notation order
uint8_t gpsMonth, gpsDay, gpsHour, gpsMinute, gpsSecond, gpsCentiSecond;
int16_t gpsYear;
uint32_t FixAge;

uint32_t adjustedGPS_unixFormat;
int32_t previousRTCtime,currentRTCsyncdTime,time_shift; // EEPROM(20-23); used to store the PREVIOUS time RTC was SYNC'd to GPS, which gets updated after setting

int32_t int32_Buffer;                  // 4-byte from -2,147,483,648 to 2,147,483,647
float float_Buffer = 9999.99;          // for float calculations

uint8_t inByte = 0.0;
boolean wait4input = true;
uint32_t startMenuStart = 0; 


volatile int32_t Tgps, Tclock;
volatile boolean gpsValid = false ;
volatile boolean rtcValid = false ;
int32_t diff; 
float diffMs; 
int8_t byteBuffer2,byteBuffer1;
char buff[10];   // String buffer for output of diffMs
volatile boolean state = false; 


//======================================================================
void setup(void) {
  pinMode(3,INPUT); pinMode(3,INPUT); pinMode(LED_BUILTIN, OUTPUT);

  SerialGPS.begin(GPSBaud);
  Wire.begin();  // Start the I2C bus // enables internal 30-50k pull-up resistors on SDA & SCL by default
  TWBR=2;        // Note: running I2C at 400khz may FAIL if your connections are flakey // comment this out if you have troubles
  i2c_setRegisterBit(DS3231_ADDRESS, DS3231_CONTROL_REG, 6, 1); // 0Eh Bit 6 BBSQW (Battery power ALARM Enable) - MUST be set to 1 for wake-up alarms when running from the coincell bkup battery
  i2c_setRegisterBit(DS3231_ADDRESS, DS3231_CONTROL_REG, 7, 0); // Enable Oscillator (EOSC).  This bit is clear (logic 0) when power is first applied.  EOSC = 0 IS REQUIRED with OUR LOGGER DESIGN:
                                                                // when EOSC (bit7) is 0, the RTC oscillator continues running during battery powered operation. Otherwise it would stop.
 
  Serial.begin(500000);
  Serial.println(F(" <- Ignore this old serial buffer cruft"));Serial.println(); // sometimes there are 'leftover' characters in the outgoing buffer
  while (Serial.available() != 0 ) {Serial.read();}   // clears the serial monitor input buffer
  Serial.println(F("GPS2RTC timesync: MOVE gpsPPS signal jumper to pin D3, RTC SQW on D2"));
  Serial.println(F("Wait until the PPS / LED blinks on the GPS module have started,"));
  Serial.println(F("Then enter any key to set the RTC time from the GPS"));Serial.println();
  
    while (Serial.available() != 0 ) {Serial.read();} // this just clears out any residual data in serial send buffer before starting our menu  
    //Serial.setTimeout(1000); // 1000 milliseconds is the default timeout for the Serial.read(); command
    inByte=0;
    wait4input = true;
    startMenuStart = millis();                          //Beginning of time-out period must be unsigned long variable
    
    while(wait4input){
    if (Serial.available()) { 
      inByte = Serial.parseInt();wait4input=false; 
      }
    if ((millis() - startMenuStart) > 480000) {           // 480000 = 8 minute timeout
      Serial.println(F("8 minute Time out with NO commands - will not proceed.")); 
      while(1);}
    }//while(wait4input)

//=========================================================== 

Serial.println(F("RTC will be set after latency falls below 200 msec:"));
  //wait here for beginning of PPS pulse - that way the NEMA update completes well before the FOLLOWING pulse
  //then WAIT for NEXT PPS pulse rising edge and then set the RTC time registers: // PINB is just a faster version of digitalRead  
  while(digitalRead(3));   // while(PIND & (1 << 3));         //while(digitalRead(3));  // wait for remainder of D3 pin HIGH pulse duration (if still high)
  while(!digitalRead(3));  // while(!(PIND & (1 << 3)));      //while(!digitalRead(3)); // wait for remainder of D3 LOW
  // if gps connected pin D8 use   while((PINB&(1<<0))); and while(!(PINB&(1<<0))); 

}  // end of setup

//==============================================================================

void loop() {

  while (SerialGPS.available()) {
    if (gps.encode(SerialGPS.read())) {     // process the gps NEMA messages

    if (( gps.date.year()>2000 )&&( gps.satellites.age()<200 )){  
    // .age =number of milliseconds since last update is typically 200-300 worst is ~ 600 for valid locations(?)   
    // could check other things:  if (gps.location.isValid() && gps.time.isValid() && (FixAge < 300)){
    
    //'temporarily' load the t_ variables so that unixTime can be calculated from current gps time
    t_second=gps.time.second();t_minute=gps.time.minute();t_hour=gps.time.hour();
    t_day=gps.date.day();t_month=gps.date.month();t_year=gps.date.year();
    adjustedGPS_unixFormat = RTC_DS3231_makeUnixtime();

    // NOTE: You could combine these two adjustments
    // but they are handled separately here for clarity

    // [OPTIONAL] Adjustment to convert UTC into your local timezone:
    adjustedGPS_unixFormat=adjustedGPS_unixFormat-18000;//adjustTime(-18000); // Using TimeLib to adjust the system time, adding or removing seconds.
    // Current DST is at my location = 5 hours [5*60*60=18000] of seconds for UTC-5
    // In non daylight saving time Central would Subtract 6 hours worth [6*60*60=21600] of seconds for UTC-6
 
    // FORWARD by ONE SECOND because we WAIT till the NEXT PPS rising edge before setting the RTC time registers
    adjustedGPS_unixFormat=adjustedGPS_unixFormat+1; //adjustTime(+1);

    // record the OLD RTC time for calculation of drift AFTER RTC time gets updated
    RTC_DS3231_getTime();  // populates t_variables with the current RTC time
    previousRTCtime = RTC_DS3231_makeUnixtime();   // store the current unix time on RTC before we update it

    // loads the 'adjusted' GPS time into the global t_ variables that will be used in RTC_DS3231_setTime
    RTC_DS3231_parseUnixTime(adjustedGPS_unixFormat);  // loads: t_second, t_minute, t_hour, t_day, t_month, t_year
    
    //then WAIT for NEXT PPS pulse rising edge and then set the RTC time registers:
    // PINB is a faster version of digitalRead which could also be used here  // while((PINB&(1<<0))); while(!(PINB&(1<<0))); if using D8
    while(PIND & (1 << 3));         // waits for remainder of D3 pin HIGH pulse duration (if still high)
    while(!(PIND & (1 << 3)));      // waits for remainder of D3 LOW, until next rising edge
     
    //At NEXT RISING EDGE on D8 (the PPS Pulse) we transfer our primed '+1sec' time into the RTC registers
    RTC_DS3231_setTime();           // delay(15);
    rtcTimeHasBeenSet=true;         // NOTE:  RTC's internal countdown chain restarts only when the seconds register is written

    // puting the numbers directly into the char array is a low memory alternative to using sprintf, + 48 converts each number into an ascii digit
      gpsDate[7]  =(gps.date.year() / 10) % 10 + 48; //regular modulo without divmod
      gpsDate[8]  = gps.date.year()       % 10 + 48;
    //OR separate the digits faster using divmod from Tillarts fast_math library:
      divmod10(gps.date.month(), &dmodQuotient, &dmodRemainder);
      gpsDate[10] = dmodQuotient + 48;        //month  / 10 + 48;
      gpsDate[11] = dmodRemainder + 48;       //month  % 10 + 48;
      divmod10(gps.date.day(), &dmodQuotient, &dmodRemainder);
      gpsDate[13] = dmodQuotient + 48;        //day    / 10 + 48;
      gpsDate[14] = dmodRemainder + 48;       //day    % 10 + 48;

      divmod10(gps.time.hour(), &dmodQuotient, &dmodRemainder);
      gpsTime[5] = dmodQuotient + 48;         //hour   / 10 + 48;
      gpsTime[6] = dmodRemainder + 48;        //hour   % 10 + 48;
      divmod10(gps.time.minute(), &dmodQuotient, &dmodRemainder);
      gpsTime[8] = dmodQuotient + 48;         //minute / 10 + 48;
      gpsTime[9] = dmodRemainder + 48;        //minute % 10 + 48;
      divmod10(gps.time.second(), &dmodQuotient, &dmodRemainder);
      gpsTime[11] = dmodQuotient + 48;        //second / 10 + 48;
      gpsTime[12] = dmodRemainder + 48;       //second % 10 + 48;
      
      Serial.print(gpsDate);Serial.print(" ");Serial.print(gpsTime);Serial.println(F("  [Fix age < 200msec]"));  
      } //terminates: if (FixAge < 300){
    } //terminates: if (gps.encode(SerialGPS.read()))
  } //terminates: while (SerialGPS.available())
    
if (rtcTimeHasBeenSet){ 

    // Enable Square wave output on RTC
    // When the INTCN (bit2) is set to logic 0, a square wave is output on SQW
    // Control Register (0Eh): setting RS2(bit4) and  RS1(bit3)  set both to logic 0 configures 1 hz frequency
    // the output pattern is low for 500msec and then high for 500msec
    // so the RTCs square wave falling edge coincides with the START of each new RTC second  //https://forum.arduino.cc/t/ds3231-1hz-output-timing-profile/1139730/4
    byteBuffer1=i2c_readRegisterByte(DS3231_ADDRESS,DS3231_CONTROL_REG);
    byteBuffer1 &= B11100011;    //'&=0' here changes only only our three target bits
    i2c_writeRegisterByte(DS3231_ADDRESS,DS3231_CONTROL_REG,byteBuffer1);

    RTC_DS3231_getTime();    // loads global t_ variables from RTC registers (to check that RTC has actually been set)
    currentRTCsyncdTime = RTC_DS3231_makeUnixtime();    // with values currently in global t_ variables
    sprintf(CycleTimeStamp, "%04d-%02d-%02d %02d:%02d:%02d", t_year, t_month, t_day, t_hour, t_minute,t_second);
    Serial.print(F("RTCset to ")); Serial.print(CycleTimeStamp);Serial.println(F("  [Local @ GPSsec+1]"));

    time_shift = currentRTCsyncdTime - previousRTCtime;  // this cacluation is increasing every time...why?
    Serial.print(F("RTCtime updated by "));Serial.print(time_shift-1);Serial.println(F(" seconds   [drift since last sync]"));
    Serial.println();Serial.flush();
    
    i2c_setRegisterBit(DS3231_ADDRESS,DS3231_STATUS_REG,7,0); //clear the OSF flag in the RTC after time is set 
    //delay(15);

    Serial.println(F("Lag:RTC 1Hz falling edge micros - GPS leading edge rise [SB <0.1 msec]"));
    attachInterrupt(0, clockIRQ, FALLING);      // RTC SQW alarm connected to Pin2 - square wave falling edge coincides with the START of each new RTC second 
    attachInterrupt(1, gpsIRQ, RISING);         // GPS PPS connected to Pin3  - leading edge pulses high
    
    // wait for SQW to stabilize
    // first cycle after can be short (?) https://forum.arduino.cc/t/ds3231-1hz-output-timing-profile/1139730/4
    noInterrupts(); gpsValid = rtcValid = false; interrupts();
    while (gpsValid == false){}
    while (rtcValid == false){}
    noInterrupts(); gpsValid = rtcValid = false; interrupts();

    byteBuffer1=0; 
    do{

      if (gpsValid && rtcValid)
        {
        diff = Tclock - Tgps;             // difference between the RTC and the GPS pulses. Note: breaks with overflow once every 70 minutes, sending 4294967.500ms 
        if(abs(diff)>500000){
          Serial.print(".");  // then we miss-timed one of the interrupts - try again
          noInterrupts(); gpsValid = rtcValid = false ; interrupts();
          }else{
          noInterrupts(); gpsValid = rtcValid = false ; interrupts();
          diffMs = diff / 1000.0;         // divide by 1000 to convert micros into millis
          dtostrf(diffMs, 7, 3, buff);    // Convert to a string with 7 digits and 3 decimal places. Negative output OK
          Serial.println();Serial.print(buff); Serial.print(F("ms")); // Display the time difference between the GPS PPS and the RTC 1Hz SQW alarm
          byteBuffer1++;
          }
        }
 
    }while(byteBuffer1<6);           // change value here to make the run longer, but remember to disable the SQW alarm or it will run forever.

    detachInterrupt(1);detachInterrupt(0); Serial.println();
    noInterrupts(); gpsValid = rtcValid = false; interrupts();  
    // DISABLE the RTC 1Hz SQW output by setting ICTN (bit2) to logic 1 (default) 
    byteBuffer1=i2c_readRegisterByte(DS3231_ADDRESS,DS3231_CONTROL_REG);
    byteBuffer1 |= B00000100;    // with '|=' only the 1's will set, others left unchanged //'&=0' here changes only only our three target bits
    i2c_writeRegisterByte(DS3231_ADDRESS,DS3231_CONTROL_REG,byteBuffer1);

    state = true; byteBuffer1 = 0;
    Serial.println();Serial.println(F("Run Again? y/n "));
    while (state) {
        if (Serial.available()) {byteBuffer1 = Serial.read();}  //.read captures only character at a time from the serial monitor window        
        switch (byteBuffer1) {
          case 'y': 
              Serial.println();
              rtcTimeHasBeenSet=false; state=false; 
              break;
          case 'n': 
            while(1);       // this TRAPs THE PROCESSOR
            break;
          default: break;
              }           // terminates switch case
      }                   // terminates while(state)  
    }                     // terminates: if(rtcTimeHasBeenSet) 
}                         // terminates: void loop()
//==============================================================================

// https://forum.arduino.cc/t/calculating-time-between-two-interrupts/193727/3

void gpsIRQ() {       // Process the GPS interrupt.
    Tgps = micros();
    gpsValid = true;
} 

void clockIRQ() {     // Process the clock interrupt.
  //if (gpsValid && !rtcValid)    // we could force the update order(?) but drift can make the RTC preceed the GPS
  //  {
    Tclock = micros();
    rtcValid = true;
  // }
}

uint32_t RTC_DS3231_makeUnixtime() {       // only call this function AFTER populating the global t_ variables
//----------------------------------
  uint32_t ut;
  uint16_t d2days = rtc_date2days(t_year, t_month, t_day);
  ut = rtc_time2long(d2days, t_hour, t_minute, t_second);
  ut += 946684800; // RTC epoch is y - 2000
                   // so adding seconds from 1970 (uixtime) to 2000  = 946684800
  return ut;
}

void RTC_DS3231_parseUnixTime(uint32_t intime){
//---------------------------------------------
    // Note: if converting from ntp instead of uTime add:   intime -= 2208988800;
    // here using divmod to speed the calculations          // https://github.com/RobTillaart/fast_math/tree/master
    divmod60(intime, &dmodQuotient, &dmodRemainder);        t_second = dmodRemainder;
    divmod60(dmodQuotient, &dmodQuotient, &dmodRemainder);  t_minute = dmodRemainder;
    divmod24(dmodQuotient, &dmodQuotient, &dmodRemainder);  t_hour = dmodRemainder;
    t_dow = ((dmodQuotient + 4) % 7) + 1;                   // Day of week
    ldiv_t d_loc = ldiv(dmodQuotient, 365);                 // Day of year //ldiv_t is a type that contains the quotient (td.quot) AND REMAINDER (td.rem) of a long division
    uint16_t doy_loc = d_loc.rem;
    uint16_t yr_loc = d_loc.quot + 1970;                    // Year (from UnixTime start)

// Adjust day of year for leap years:
    uint16_t ly_loc;                                        // Leap year // unsigned same as uint16_t
    for(ly_loc = 1972; ly_loc < yr_loc; ly_loc += 4) {      // Adjust year and day of year for leap years
        if(!(ly_loc % 100) && (ly_loc % 400)) continue;     // Skip years that are divisible by 100 and not by 400
        --doy_loc;
    }
    if(doy_loc < 0) doy_loc += 365, ++yr_loc;               // Handle underflow
    t_year =  yr_loc;                                       // Note: -2000 done in RTC_DS3231_setTime because DS3231 uses epoch from 2000 internally

// Find month and day of month from day of year
    static uint8_t const daysInMonth[2][12] = {             // Days in each month
        { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31},  // Not a leap year
        { 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}   // Leap year
    };
    int16_t dayCalc = doy_loc;                              // Init day of month
    t_month = 0;                                            // Init month
    ly_loc = (yr_loc == ly_loc) ? 1 : 0;                    // Make leap year index 0 = not a leap year, 1 = is a leap year
    while(dayCalc > daysInMonth[ly_loc][t_month]) dayCalc -= daysInMonth[ly_loc][t_month++]; // Calculate month and day of month
    t_doy = doy_loc + 1;
    t_day = dayCalc + 1;
    t_month = t_month + 1;
}  


void RTC_DS3231_getTime(){
//------------------------
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(0);
  Wire.endTransmission();

//LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF); // not sure this is needed?
//this 0 gets written to fast buffer memory not eeprom
  
  Wire.requestFrom(DS3231_ADDRESS, 7);
  t_second = rtc_bcd2bin(Wire.read() & 0x7F);
  t_minute = rtc_bcd2bin(Wire.read());
  t_hour = rtc_bcd2bin(Wire.read());
  Wire.read();                        // dayOfWeek not used:
  //DS3231 has a register (0x03) that tracks DoW as a number between 1 and 7. 
  //It increments at midnight and rolls back to 1 after 7. It's up to the user to decide 
  //which day corresponds to which value and set it correctly.
  t_day = rtc_bcd2bin(Wire.read());   // dayOfMonth
  t_month = rtc_bcd2bin(Wire.read());
  t_year = rtc_bcd2bin(Wire.read()) + 2000;
  return;
}

void RTC_DS3231_setTime(){
//------------------------
   Wire.beginTransmission(DS3231_ADDRESS);
   Wire.write((byte)0);
   Wire.write(rtc_bin2bcd(t_second));
   Wire.write(rtc_bin2bcd(t_minute));
   Wire.write(rtc_bin2bcd(t_hour));
   Wire.write(rtc_bin2bcd(0));  // dayOfWeek not used
   Wire.write(rtc_bin2bcd(t_day));
   Wire.write(rtc_bin2bcd(t_month));
   Wire.write(rtc_bin2bcd(t_year - 2000));
   //Wire.write(0);             // dom not used
   Wire.endTransmission();
  }

//Calculation / Conversion functions called by RTC_DS3231 functions
//-----------------------------------------------------------------
const uint8_t rtc_days_in_month [12] PROGMEM = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 }; //leap years handled below

static uint8_t rtc_bcd2bin (uint8_t val) {
  return val - 6 * (val >> 4);
}

static uint8_t rtc_bin2bcd (uint8_t val) {
  return val + 6 * (val / 10);
}

static long rtc_time2long(uint16_t indays, uint8_t h, uint8_t m, uint8_t s) {
  return ((indays * 24L + h) * 60 + m) * 60 + s;
}

// number of days since 2000/01/01, valid for 2001..2099
static uint16_t rtc_date2days(uint16_t y, uint8_t m, uint8_t d) {
  if (y >= 2000) y -= 2000;
  uint16_t indays = d;
  for (uint8_t i = 1; i < m; ++i)
  indays += pgm_read_byte(rtc_days_in_month + i - 1);
  if (m > 2 && (y%4 == 0)){++indays;} // (y % 4 == 0) modulo equivalent(?) (y&3) checks (if is LeapYear) add extra day
  //  A year is a leap year if it is divisible by 4 except if it's divisible by 100, in which case it's not a leap year, except if it's divisible by 400, in which case it is a leap year after all.
  return indays + 365 * y + (y + 3) / 4 - 1;
}

//==========================================================================================

byte i2c_setRegisterBit(uint8_t deviceAddress, uint8_t registerAddress, uint8_t bitPosition, bool state) 
//-----------------------------------------------------------------------------------------
{
  byte registerByte;
  byte result;
  registerByte = i2c_readRegisterByte(deviceAddress, registerAddress);  //load the existing register contents
  if (state) {    // when state = 1
    bitSet(registerByte,bitPosition);           // alt: registerByte |= (1 << bitPosition);  // bitPosition of registerByte now = 1
  }
  else {  // when state = 0
    bitClear(registerByte,bitPosition);         // alt: registerByte &= ~(1 << bitPosition); // bitPosition of registerByte now = 0
  }
  result = i2c_writeRegisterByte(deviceAddress, registerAddress, registerByte);
  return result;                                // result =0 if the writing the new data to the registry went ok
}

byte i2c_readRegisterByte(uint8_t deviceAddress, uint8_t registerAddress)
//-----------------------------------------------------------------------------------------
{
  byte registerData;
  Wire.beginTransmission(deviceAddress);        //set destination target
  Wire.write(registerAddress);                  //assumes register address is <255  - this is not the case for all sensors
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)deviceAddress, (uint8_t)1);
  registerData = Wire.read();
  return registerData;
}

byte i2c_writeRegisterByte(uint8_t deviceAddress, uint8_t registerAddress, uint8_t newRegisterByte){
//-----------------------------------------------------------------------------------------
  Wire.beginTransmission(deviceAddress);
  Wire.write(registerAddress);
  Wire.write(newRegisterByte);
  byte result = Wire.endTransmission();

  if (result > 0)   //error checking
  {
    //if(ECHO_TO_SERIAL){                           //NOTE: only call halt on error if in debug mode!
      Serial.print(F("FAIL in I2C register write! Result code: "));
      Serial.println(result); Serial.flush();
      while(1); // stops the processor
    //}
  }
  // LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);  // some sensors need this settling time after a register change?
  return result;
}
