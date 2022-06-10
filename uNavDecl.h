// uNavDecl.h:  declarations for Framework 
// RSenser, 2021-12-03 
//

// includes:
#include <Wire.h>

#ifdef USE_GPS
// #include "rwsGPS2.h"
// #include "rwsGPS3.h"
#include "rwsGPS4.h"

GPS myGPS;
#endif
#ifdef USE_I2C_BARO
#define USE_WIRE
#include "SparkFunBME280.h"
BME280 mySensor;
#endif
#ifdef USE_IBUS
#include "IBusBM.h"
IBusBM IBus;
IntervalTimer iBusTimer;
#endif
#ifdef USE_I2C_D251
#define USE_WIRE
#include "rwsD251.h"
D251 myD251;
#endif
#ifdef USE_I2C_D271
#define USE_WIRE
#include "rwsD271.h"
D271 myD271;
#endif
#ifdef USE_WIRE
#include <Wire.h>
#endif
// Constants:
const unsigned int timerFixer = 1;
const int LED_DELAY_OK = 50;
const int LED_DELAY_SCAN = 500;
//
// variables
int gpsActive =
#ifdef USE_GPS
true;
#else
false;
#endif
int baroActive = 
#ifdef USE_I2C_BARO
true;
#else
false;
#endif
int iBusActive =
#ifdef USE_IBUS
true;
#else
false;
#endif
int d251Active =
#ifdef USE_I2C_D251
true;
#else
false;
#endif
int d271Active =
#ifdef USE_I2C_D271
true;
#else
false;
#endif
//
int LEDDelay = LED_DELAY_SCAN; // LED_DELAY_OK;
bool telloActive = false;
bool telloPanic = false;
unsigned long int waitLEDTicks = 0; 
unsigned long int watchEndTicks = 0;

#ifdef KINETISK
#ifdef USE_IBUS
// this is teensy specific!!
void timerPop() {
  IBus.loop();
}
#endif
#endif
//
// end of moldule
