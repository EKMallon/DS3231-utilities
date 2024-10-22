//This code uploaded to GITHUB 20241022

/*  
======================================================================================
This code is from an Arduino PlayGround post by ShermanP:
https://forum.arduino.cc/t/ds3231-setting-aging-register-to-optimum-value/1112826/13

I have made some TRIVIAL CHANGES here that were needed for the Cave Pearl Loggers which cut the VCC line to the RTC to save power
AND I use an UNO board powering a NEO6M at 5v while supplying the RTC module via the UNOs 3v output
so the test is done with the DS3231 near the Cr2032 runtime voltage (because this affects the RTC base frequency)
This testing setup described in detail on our blog at:

If you need DS3231 age register determination for your project then I recommend you go
to ShermanP's ORIGINAL GitHub repo at https://github.com/gbhug5a/DS3231-Aging-GPS
being sure to read the PDF there describing how this code works
======================================================================================
*/

#include <Wire.h>
#define flagsREG EIFR                     // ATMega328P interrupt flags register

const byte RTCpin = 2;                    // D2
volatile bool Capture = false;            // Capture interrupt has occurred - GPS
volatile bool Started = false;            // Square wave interrupt has occurred - RTC
volatile byte MSBtimer;                   // MS byte of 24-bit timer
volatile int Square = 0;                  // square wave clocks - for SN vs M
bool isSN = true, Pending = false;        // true if SN, false if M; forced conv pending
bool fineFlag = false;                    // switch to max deltaAging of +/- 1
unsigned long GPTlow, GPThi;              // timer counts on GPS interrupt
long Diff, prevDiff, oldDiff, deltaDiff;  // clocks from RTC to GPS
int ppm;                                  // Aging +/- 1:  count difference over 5 minutes
int SNdivisor = 264;                      // ppm for SN parts
int Mdivisor = 576;                       // ppm for M parts
int Restart = 32;                         // initial 32-sec run-in
int Period = 300;                         // number of seconds between adjustments
int finePeriod = 200;                     // period when close to end
int Count = Restart, k;                   // down counter - seconds until next calc
int8_t Aging, deltaAging;                 // contents of RTC Aging register
byte Seconds, Control, Status, j, i = 0;  // other RTC registers' contents
long batch[16];                           // last 16 readings
char buff[20];                            // serial input buffer
byte buffSize;                            // length of input string
char in;                                  // serial input character

void setup() {
  pinMode(8,INPUT);                       // Timer1 capture input - from GPS (ICP1)
  pinMode(RTCpin,INPUT_PULLUP);           // Hardware interrupt on D2 - from RTC
  
  Wire.begin();
  // BBSQW (Battery power ALARM Enable) must be set on RTCs running from Vbat when powered from the coincell bkup battery
  i2c_setRegisterBit(0x68,0x0E,6,1);  // CONTROL_REG 0Eh Bit 6 BBSQW (Battery powered ALARM Enable)
  delay(15);// I2C transaction also starts the RTC oscilator if only powered by Vbat line
  // i2c_setRegisterBit(0x68,0x0E,7,0); // Enable Oscillator (EOSC).
  // not needed because I2C transaction starts the osc. This bit is clear (logic 0) when power is first applied.  EOSC = 0 IS REQUIRED with OUR LOGGER DESIGN:
  
  Serial.begin(500000);
  Serial.println(F(" <- Ignore this old serial buffer cruft"));Serial.println();// clears any leftover junk in the outgoing buffer
  while (Serial.available() != 0 ) {Serial.read();}  // clears serial input buffer
  Serial.println(F("AgeReg: MOVE GPS/PPS signal wire to D8, +RTCSQW on D2"));
  Serial.println(F("Wait until the PPS / LED flashes on the GPS module have started,"));
  Serial.println(F("Then Enter any key to begin"));Serial.println();

  int key1 = 0;
  while ((key1 != 10) && (key1 != 13)) {
    key1 = Serial.read();
    delay (100);
  }
  key1 = Serial.read();                   // in case CR/LF

  // Clear /EOSC, CONV, RS2, INTCN, A2E, A1E.   Set BBSQW, RS1. (SQW freq = 1KHz)
  Control = 0b01001000;
  Status = 0;

  updateReg(0x0E);                        // update Control
  updateReg(0x0F);                        // update Status

  Wire.beginTransmission(0x68);           // read Aging and Seconds registers
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 4);

  Aging = Wire.read();                    // starting value of the Aging register
  Seconds = Wire.read();                  // read past temp registers
  Seconds = Wire.read();
  Seconds = Wire.read();                  // pointer wraps to zero

  cli();
  flagsREG = 3;                           // clear any flags on both pins
  attachInterrupt(digitalPinToInterrupt(RTCpin),SNvsM, FALLING);
  flagsREG = 3;
  sei();
  delay(2000);
  Control &= 0b11110111;                  // change squarewave to 1Hz
  updateReg(0x0E);
  detachInterrupt(digitalPinToInterrupt(RTCpin));  // will assign new ISR for D2
  flagsREG = 3;
  Serial.print (F("1KHz squarewave makes ")); Serial.print(Square);
  Serial.println (F(" cycles over 2 seconds."));
  if (Square < 500) {
    isSN = false;
    ppm = Mdivisor;                       // expected change from Aging +/- 1
    Serial.println (F("So this is a DS3231M"));
  }
  else {
    ppm = SNdivisor;                      // expected change from Aging +/- 1
    Serial.println (F("So this is a DS3231SN"));
  }
  Serial.println();
  if (F_CPU == 8000000) ppm /= 2;         // if 8MHz Pro Mini

  while(digitalRead(8));                  // wait for GPS low, then
  while(!digitalRead(8));                 // wait for GPS high - beginning of second
  delay(500);                             // wait half a second
  updateReg(0);                           // reinitialize RTC clock - now 1/2 sec apart

  cli();
  TIMSK0 = 0;                             // Disable Timer0 interrupts (millis)
  TCCR0A = 0;
  TCCR0B = 0;
  TIFR0  = 0xFF;

  TCCR1A = 0;                             // set up Timer1
  TCCR1B = 0;
  TCCR1C = 0;
  TCNT0  = 0;                             // clear Timer1
  TIFR1  = 0xFF;                          // clear flags
  TIMSK1 = 0b00100001;                    // enable input capture and T1 overflow interrupt (GPS)
  TIFR1  = 0xFF;                          // clear flags
  TCCR1A = 0b00000000;                    // Normal mode, no output, WGM #0
  TCCR1B = 0b01000001;                    // rising edge capture, timer1 on, no prescale

  flagsREG = 3;                           // new ISR for D2
  attachInterrupt(digitalPinToInterrupt(RTCpin),rtcISR, FALLING);
  flagsREG = 3;
  sei();
  Serial.println (F("Enter 'An' to change Aging to n (-128 to 127)"));
  Serial.println (F("Each Cycle takes 300 seconds = 5min per test > keep the Serial Monitor open"));
  Serial.println (F("Wait for at least 3 screen non-zero adjustments (takes ~20 minutes)"));
  Serial.println (F("Value does not usually change much after that"));
  Serial.println (F("Enter 'Q' to quit, or 'T' to enter new date/time")); Serial.println();
}

void loop() {
  if (Capture) {                          // GPS PPS has gone high
    Capture = false;
    GPTlow = ICR1;                        // read timer values
    GPThi = MSBtimer;
    cli();
    TCNT1 = 0;                            // clear timer1
    MSBtimer = 0;
    TIFR1 = 0xFF;                         // clear flags
    sei();

    Diff = (GPThi << 16) + GPTlow;        // combine timer counts to one long value
    if (abs(Diff - prevDiff) < 1000) {    // normal values only
      batch[i] = Diff;                    // collect last 16 values into array
      i = (i + 1) & 15;
    }
    prevDiff = Diff;

    Count--;
    if (!Count) {                         // do calculation every five minutes
      Diff = 0;
      for (j = 0; j < 16; j++) {
        Diff += batch[j];
      }
      Diff = (Diff + 8) / 16;             // average over last 16 seconds
      if (Restart == 32) oldDiff = Diff;
      deltaDiff = Diff - oldDiff;         // calculate new Aging
      deltaAging = deltaDiff / ppm;
      if ((Restart != 32) && (!deltaAging)) fineFlag = true;
      if (deltaAging) {                   // if any change
        if (fineFlag && (deltaAging > 1)) deltaAging = 1;
        if (fineFlag && (deltaAging < -1)) deltaAging = -1;
        Aging += deltaAging;
        updateReg(0x10);
        oldDiff = Diff;
        if (isSN) Pending = true;         // force conversion if SN
      }
      Serial.print ("Diff "); Serial.println(Diff);   // print results
      Serial.print ("deltaDiff ");
      if (deltaAging == 0) {
        Serial.print("[");
        Serial.print(deltaDiff);
        Serial.println("]");
      }
      else Serial.println(deltaDiff);
      Serial.print ("deltaAging "); Serial.println(deltaAging);
      Serial.print ("Aging "); Serial.println(Aging); Serial.println();

      if (Restart==32) Restart = Period;  // switch to 5 minutes after run-in
      if (fineFlag) Restart = finePeriod; // switch to 3.33 minutes in fine mode
      Count =  Restart;
    }
  }
  if (Started) {                          // beginning of second
    if (Pending) {
      Control |= 0b00100000;              // force conversion
      updateReg(0x0E);
      Control &= 0b11011111;
      Pending = false;
    }
    Started = false;
  }
  if(Serial.available()) {                // process input from Serial Monitor
    in = Serial.read();                   // set end-line option to Newline or CR
    if ((in == 13) || (in == 10)) {
      buff[buffSize] = 0;
      parse_cmd(buff, buffSize);
      buffSize = 0;
      buff[0] = 0;
    }
    else {
      buff[buffSize] = in;
      buffSize++;
    }
  }
}

void parse_cmd(char *cmd, byte cmdsize) {

  // YYYYMMDDWhhmmss
  if ((cmd[0]=='2')&&(cmdsize==15)) {     // "2" new date/time
    Wire.beginTransmission(0x68);
    Wire.write(0);
    Wire.write(inp2bcd(cmd,13));          // seconds
    Wire.write(inp2bcd(cmd,11));          // minutes
    Wire.write(inp2bcd(cmd,9));           // hours
    Wire.write(cmd[8] - 48);              // day of the week
    Wire.write(inp2bcd(cmd,6));           // date of the month
    Wire.write(inp2bcd(cmd,4) | 0x80);    // month & century
    Wire.write(inp2bcd(cmd,2));           // year
    Wire.endTransmission();
    Serial.println ("Data entered");
    shutdown();
  }

  else if ((cmd[0]&0xDF)=='T') {          // "T" Time set
    Serial.println ("Enter new date/time for RTC.  (w = day of week (1-7))");
    Serial.println ("YYYYMMDDwhhmmss");
  }

  else if ((cmd[0]&0xDF)=='A') {          // "A" Aging
    if (cmdsize > 1) {
      k = atoi(&cmd[1]);                  // get value of string
      if ((k < 128) && (k > -129)) {      // check for legit value
        Aging = k;                        // convert to signed byte
        updateReg(0x10);                  // write to Aging register
        Count = 32; Restart = 32; fineFlag = false;
        if (isSN) Pending = true;
      }
      else Serial.println ("Invalid Aging Value");
    }
    Wire.beginTransmission(0x68);         // "A" alone prints current value
    Wire.write(0x10);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 1);
    Aging = Wire.read();
    Serial.print("Aging = "); Serial.println(Aging); Serial.println();
  } 

  else if ((cmd[0]&0xDF)=='Q') {          // "Q" Quit
    shutdown();
  }
}

byte inp2bcd(char *inp, byte seek) {
  return (((inp[seek]-48)<<4) + (inp[seek+1] - 48));
}

void shutdown() {
  Control |= 0b00000100;                  // disable square wave
  updateReg(0x0E);
  cli();
  detachInterrupt(digitalPinToInterrupt(RTCpin));
  flagsREG = 3;
  TCCR1B = 0;
  TIMSK1 = 0;
  TIFR1 = 0;
  sei();
  Serial.println("Squarewave disabled");
  Serial.println("Shutting down");
  while (1);
}

void updateReg(byte addr) {
  Wire.beginTransmission(0x68);
  Wire.write(addr);
  if(addr == 0x0E) Wire.write(Control);
  else if(addr == 0x0F) Wire.write(Status);
  else if(addr == 0x10) Wire.write(Aging);
  else if(addr == 0) Wire.write(Seconds);
  Wire.endTransmission();
}

ISR(TIMER1_CAPT_vect) {
  TCCR1B &= 0xFE;                         // stop Timer1 clock
  Capture = true;
}

ISR(TIMER1_OVF_vect) {
  MSBtimer++;                             // increment MSB on overflow
}

void rtcISR() {
  TCCR1B |= 1;                            // start Timer1 clock
  Started = true;
}

void SNvsM() {                            // only used for SN vs M test
  Square++;
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
  return result;
}
