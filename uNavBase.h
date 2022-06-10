// uNavBase.h:  Base Framework for universal Navigator
// RSenser, 2021-12-03 
//
// used uNavMacros.h to set default macro values
// history:
//
//
/*
MIT License

Copyright (c) 2022 rwsenser

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 */

void Xdelay(unsigned long v) {
  delay(v * timerFixer);
}
// Fix millis() and delay() errors (needed with Adafruit M0....)
unsigned long Xmillis() { 
  return millis() / timerFixer;
}
// yield quick functions and friends
//
// some times you just have to blink!
//
void thread_LED () {
  if (Xmillis() > waitLEDTicks) {
    digitalWrite(13, !digitalRead(13));
    waitLEDTicks = Xmillis() + LEDDelay;
  }  
  return;
}

//
// watchdog timer, sets telloPanic
void thread_panic() {
  if (telloActive && (Xmillis() > watchEndTicks)) {
    // Tello flying and it's over time, so panic!
    telloPanic = true;
  }
  return;
}  

void Xyield() {
  thread_LED();     // blink LED 
  thread_panic();   // let the watch dog **bark**
  // thread_listen();  // check for UDP input from Tello
}

void setup() {
  bool setupError = false;
  Serial.begin(9600);
#if 0  
  while (1) {
      if (Serial.available() > 0) break;
  }
  // DEBUG
  Serial.println("DEBUG -- Serial.begin");
  // while (1);
  // end DEBUG
#endif  
  // put your setup code here, to run once:
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);  
  waitLEDTicks = Xmillis() + LEDDelay; 
#ifdef USE_WIRE
  Wire.begin();
#endif
#ifdef USE_I2C_BARO
  if (mySensor.beginI2C() == false) //Begin communication over I2C
  {
    Serial.println("The baro sensor did not respond. Please check I2C wiring.");
    setupError = true;
  }
#endif 
#ifdef USE_I2C_D251
  myD251.begin();
#endif
#ifdef USE_I2C_D271
  myD271.begin(14, 15); // pins 14 and 15 used to mark calibration status
#endif
  if (!setupError) {
    appSetup();   // can change setupError to true! 
  }
  if (setupError) {
    Serial.println("setup() error!");
    while (1); // Freeze
  }
}

void loop() {
#ifdef USE_GPS
  myGPS.loop();     // this can do 'slow' serial input!!
#endif 
#ifdef USE_I2C_D251
    myD251.loop(); 
#endif 
#ifdef USE_I2C_D271
    myD271.loop(); 
#endif      
  Xyield();  
  // app code called here, to run repeatedly:

  appLoop();
  
  // Xdelay(500);
  // Serial.print("#");
#if 0
#ifdef USE_GPS
  if (gpsActive) {
    char buffer[128];
    myGPS.ts = millis();
    sprintf(buffer, "%c%d,%ld,%f,%f,%.2f",myGPS.type, myGPS.cnt,
                    (myGPS.ts / 1000), myGPS.lat, myGPS.lng, myGPS.alt);
    Serial.println(buffer);      
  }
#endif  
#ifdef USE_I2C_BARO
  if (baroActive) {
    int alt = mySensor.readFloatAltitudeFeet();  
    Serial.print("Alt Ft: ");
    Serial.println(alt);    
  }
#endif 
#endif 
}
