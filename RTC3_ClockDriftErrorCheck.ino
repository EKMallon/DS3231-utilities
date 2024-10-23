// This is based on code from the Needle Nose Pliers blog [which requires google translation from Japanese]
// https://radiopench-blog96-fc2-com.translate.goog/blog-entry-943.html?_x_tr_sl=auto&_x_tr_tl=en&_x_tr_hl=en&_x_tr_pto=wapp&_x_tr_sch=http
// which assumes the GPS PPS signal is connected to pin D3, and RTC SQW on D2 - the code uses both hardware interrupt lines

/*  
====================================================================================================
I have made several CHANGES here because he was using 2 UNOs (one of which was controlling the clock)
while I run the test with only one UNO board that controls both the NEO6M gps and the DS3231 RTC module
I must enable BBSQW for our Cave Pearl Loggers which cut the VCC line to the RTC to save power
I'm using an UNO board powering a NEO6M at 5v while powering the RTC on the UNOs 3v line so the test 
is run with the RTC near the Cr2032 runtime voltage because this affects the RTC oscilator
I added input at the start so that the RTC aging register can be adjusted before the run  
All described in detail at:
https://thecavepearlproject.org/2024/10/22/setting-accurate-rtc-time-with-a-gps-the-ds3231-aging-offset-to-reduce-drift/
====================================================================================================
*/

#include <Wire.h>       // I2C bus coms library: RTC, EEprom & Sensors
volatile boolean state = false; 
volatile uint32_t Tgps, Tclock, oldTgps, oldTclock; // Time recording variable 
char buff[10];          // String manipulation buffer 
int32_t diff; 
float diffMs, aveSlope; 
float data[60];         // Past data storage buffer (used for calculating approximate straight line) 

//for DS3231 RTC
//--------------
#define rtcAlarmInputPin 2                    // DS3231's SQW output is connected to interrupt0 pin D2 on the ProMini
#define DS3231_ADDRESS     0x68               // this is the I2C bus address of our RTC chip
#define DS3231_STATUS_REG  0x0F               // reflects status of internal operations
#define DS3231_CONTROL_REG 0x0E               // enables or disables clock functions
#define DS3231_AGING_OFFSET_REG 0x10          // Aging offset register
//#define DS3231_TMP_UP_REG  0x11               // temperature registers (upper byte 0x11 & lower 0x12) gets updated every 64sec
char CycleTimeStamp[] = "0000-00-00,00:00:00";// 16 character array to store human readble time (with seconds)
uint8_t t_second,t_minute,t_hour,t_day,t_month,t_dow;    // current time variables populated by calling RTC_DS3231_getTime()
uint16_t t_doy,t_year;                                   // current year //note: yOff = raw year to which you need to add 2000
volatile boolean rtc_INT0_Flag = false;       // used in startup time sync delay //volatile because it's changed in an ISR // rtc_d2_alarm_ISR() sets this boolean flag=true when RTC alarm wakes the logger
int8_t RTCagingOffset = 0;
uint8_t byteBuffer1 = 9;                      // 1-byte (8 bit) type = unsigned number from 0 to 255
uint16_t CycleCounter=0;

void setup() { 
  pinMode(2,INPUT); pinMode(3,INPUT);         // assumes both devices have hardware pullup
  pinMode(LED_BUILTIN,OUTPUT);
  for (int n = 0; n <= 59; n++) {data[n] = 0.0;}     // clear the data array

// Configure the DS3231 Real Time Clock control registers
  Wire.begin();// Start the I2C bus // enables internal 30-50k pull-up resistors on SDA & SCL by default
  TWBR=2;    // Note: running I2C at 400khz may FAIL if your connections are flakey!
  i2c_setRegisterBit(DS3231_ADDRESS, DS3231_CONTROL_REG, 6, 1); // 0Eh Bit 6 (Battery power ALARM Enable) - MUST set to 1 for wake-up alarms when running from the coincell bkup battery
  i2c_setRegisterBit(DS3231_ADDRESS, DS3231_CONTROL_REG, 7, 0); // Enable Oscillator (EOSC).  This bit is clear (logic 0) when power is first applied.  EOSC = 0 IS REQUIRED with OUR LOGGER DESIGN:
                                                                // when EOSC (bit7) is 0, the RTC oscillator continues running during battery powered operation. Otherwise it would stop.
  RTC_DS3231_turnOffBothAlarms();                               // stops RTC from holding the D2 interrupt line low by writing logic 0 to the alarm flags A1F and A2F in the STATUS_REG 

  Serial.begin(500000);
  Serial.println(F(" <- Ignore this old serial buffer cruft"));Serial.println();// clears any leftover junk in the outgoing buffer
  while (Serial.available() != 0 ) {Serial.read();}  // clears serial input buffer
  
  Serial.println(F("DS3231 Drift Check: MOVE gpsPPS signal jumper to pin D3, RTC SQW to D2"));
  Serial.println(F("Wait until the PPS / LED blinks on the GPS module have started,"));Serial.println();
  Serial.println(F("Then input the RTC aging register setting to use for this test: -128 to +127"));
  Serial.println(F("(+)ive aging values slow the clock while (-)ive values increase clock speed"));
  Serial.println(F("At 25°C, one LSB typically provides about 0.1ppm change in RTC oscilator frequency"));
    
do { 
    Serial.setTimeout(100000);                              // parseInt will normally “time out” after default set point is 1 second (1000 milliseconds).
    while (Serial.available() != 0 ) {Serial.read();}       // clears the serial buffer  
    RTCagingOffset = Serial.parseInt();                     // parseInt() actually returns a long(?)
    } while((RTCagingOffset<-128) || (RTCagingOffset>127)); // if check fails & you have to re-enter the number til within range

   i2c_writeRegisterByte(DS3231_ADDRESS,DS3231_AGING_OFFSET_REG,RTCagingOffset);
   delay(15); RTCagingOffset = i2c_readRegisterByte(DS3231_ADDRESS,DS3231_AGING_OFFSET_REG);
   Serial.print(F("RTCage set to: ")); Serial.println(RTCagingOffset);Serial.println();

  //Interrupt on leading edges:
  attachInterrupt(0, clockIRQ, FALLING);      // RTC SQW alarm connected to D2
  attachInterrupt(1, gpsIRQ, RISING);         // GPS PPS connected to D3  

  // Control Register (0Eh) // pg13 https://www.analog.com/media/en/technical-documentation/data-sheets/DS3231.pdf
  // When the INTCN(bit2) is set to logic 0, a square wave is output on SQW
  // RS2(bit4) and RS1(bit3)- seting both to zero configures 1 Hz
  byteBuffer1=i2c_readRegisterByte(DS3231_ADDRESS,DS3231_CONTROL_REG);
  byteBuffer1 &= B11100011;     // with '&=' only 0's change the target bits, 1's have no effect
  i2c_writeRegisterByte(DS3231_ADDRESS,DS3231_CONTROL_REG,byteBuffer1);
  
  Serial.println(F("Note: Drift in ms will continue to increase over time"));
  Serial.println(F("ppm values ​​​​typically vary by ±0.02 (or more for -M chips)"));
  Serial.println(F("9.999 means still collecting the initial 60 readings"));
  Serial.println(); Serial.println(F("RTC Lag(msec), Drift Error(ppm)")); 

  while (state == false){}      // Waits until an interrupt from the clock is received
  state = false;                // Resets the state flag
} 
//===============================================================================

void loop() { 
  while (state == false){}      // WaitS until an interrupt from the clock is received
  state = false;                // state true when clockIRQ() executes - RTC usually LAGs the gps PPS pulse

  bitSet(PORTB,5);              // digitalWrite(13, HIGH); 
  
  diff = Tclock - Tgps; // Calculate the time difference between the clock and the GPS. Breaks (overflows ) once every 70 minutes. 
  diffMs = diff / 1000.0; 
  aveSlope = calcSlope(diffMs); 

  dtostrf(diffMs, 7, 3, buff); // Convert to a string with 7 digits and 3 decimal places. 
  Serial.print(buff); Serial.print(F(" ms")); // Display the time difference. 

  dtostrf(aveSlope, 6, 3, buff); // Convert to a string with 7 digits and 3 decimal places. 
  Serial.print(F(", ")); Serial.print(buff);Serial.println(F(" ppm"));
  //Serial.print(F(", After:")); Serial.print(CycleCounter++);Serial.println(F(" seconds"));
  
  bitClear(PORTB,5);            // digitalWrite(13, LOW); 
  // Loop processing time ~52ms 
}  //terminates loop()      // Loop processing time ~52ms 

//===============================================================================

// NOTE: see the ORIGINAL calcSlope function by Needle Nose Pliers [requires Google translation from Japanese]
// https://radiopench-blog96-fc2-com.translate.goog/blog-entry-943.html?_x_tr_sl=auto&_x_tr_tl=en&_x_tr_hl=en&_x_tr_pto=wapp&_x_tr_sch=http

float calcSlope(float d) { // Calculate the slope of the data and return it as a value 
  float sumXY = 0.0; 
  float sumX = 0.0; 
  float sumY = 0.0; 
  float sumX2 = 0.0; 
  float a; 
  static int dN = 0; 

  for (int n = 58; n >= 0; n--) { // Shift one data back and leave a space at the beginning
    data[n + 1] = data[n]; 
  } 
  data[0] = d; // Save the most recent data at the beginning 

  // Find the slope of the approximated line using the least squares method 
  for(int n = 0; n <= 59; n++) { 
    sumXY += n * (data[n] - data[0]); // ∑xy (The y value is a relative value from the first data to prevent overflow) 
    sumX += n; // ∑x 
    sumY += data[n] - data[0]; // ∑y (The y value is a relative value from the first data to prevent overflow) 
    sumX2 += n * n; // ∑x^2 
  } 
  a = (60.0 * sumXY - sumX * sumY) / (60.0 * sumX2 - pow(sumX, 2)); // Find the slope using the least squares method. 

  a = -1000.0 * a; // Invert the sign because the data is saved in the opposite order, and multiply by 1000 to convert from ms to ppm. 

  if (dN <= 59) { // If not all the data is available, 
    dN += 1; // Increment the counter. 
    a = 9.999; // Replace the value of a with the value to display uncalculated data 
    } 
  return a; 
} 

void gpsIRQ() {       // Process the GPS interrupt. 
  oldTgps = Tgps; 
  Tgps = micros(); 
} 

void clockIRQ() {     // Process the clock interrupt. 
  oldTclock = Tclock; 
  Tclock = micros(); 
  state = true; 
}

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

void RTC_DS3231_turnOffBothAlarms() {             // from http://forum.arduino.cc/index.php?topic=109062.0
//-----------------------------------
  // Each alarm event has to be acknowledged: After an alarm has been fired, the alarm flags (A1F and A2F) are logic 1 
  // and have to be actively reset to logic 0 to allow for the next alarm.
  // This is done by writing logic 0 to the alarm flags A1F and A2F in the STATUS_REG
  byteBuffer1=i2c_readRegisterByte(DS3231_ADDRESS,DS3231_STATUS_REG);
  byteBuffer1 &= B11111100;                       //change the target bits //with '&=' only the 0's affect the target byte // with '|=' only the 1's will set
  i2c_writeRegisterByte(DS3231_ADDRESS,DS3231_STATUS_REG,byteBuffer1);
  rtc_INT0_Flag = false;                          //clear the flag we use to indicate the RTC alarm occurred
}

//called by RTC_DS3231 functions
//----------------------------------------

static uint8_t rtc_bcd2bin (uint8_t val) {
  return val - 6 * (val >> 4);
}

static uint8_t rtc_bin2bcd (uint8_t val) {
  return val + 6 * (val / 10);
}
