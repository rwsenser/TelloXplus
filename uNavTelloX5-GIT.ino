// uNavTelloX5-GIT from uNavTelloX5:  uNav for TelloX, for Adafruit M0 with WiFi
// RSenser, 2022-06-10
//
// build tools: (1) Adafruit Feather M0 and (2) Arduino ISP
//
//  Version uploaded to GIT, 2022-06-10
//  Follows MIT software license unless source contains previous license.
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
//

// BE SURE TO CORRECTLY SET THESE MACROS:
//  ** LOCAL_DEBUG         <-- If present, WiFi to Tello is off, simulates flight only
//  ** TELLOX_VERBOSE_ON   <-- If present, sends verbose output to Serial (some items are always send to Serial) 
//  ** leave use of GPS_ON present (it is used by uNav<xxx>)
//
// include macro defaults for preprocessor (conditional code)
//
// these should be OFF (not present) for normal flight!!
// NO WiFi to drive Tello
#undef LOCAL_DEBUG
// #define LOCAL_DEBUG
// LESS Serial Output
#undef TELLOX_VERBOSE_ON
#define TELLOX_VERBOSE_ON
//
// change history:
//
// 2022-05-27
// * Add hover in place for flight time measurement ...
//   (Turn GPS off)
//

// 2022-05-17
// * Ready for first straight vector test ... done, good results
//

// 2022-04-29/2022-05-04/2022-05-12:
// * cleanup code, misc. ... done
// * add code to follow a heading and distance
// * update FSM for added states ... done
// *     after startup and GPS check
// * lots of changes ... added CMD_VECTOR, etc. ... done
// * finally add GPS-based course correct to vector mode .. done 
// *     which means add new fsm step
//
// overall design:
// ** 2 modes: Vector and Waypoint
//    Waypoint 2 states set to Panic for now
//
//
// 2022-03-24/2022-04-07/2022-04-22:
// * fly 4 pre-steps, calibrate and decide, if "go" fly to "dest" and return to start!
// *   pre-steps: UP d, LEFT d, RIGHT 2*d, LEFT d, FORWARD d ... done
// *   wait after each step, record GPS coordinates ... done
// *   compute heading and distance ... done
// *   add led blink for GPS wait ... done
// *   verify GPS "saw" movements ... DONE!
// *   cleanup ActTelloStop and ..Panic code -- make one routine ... done
// * add distance-from-start-exceeded watchdog
// * add test mode with WiFi off (LOCAL_DEBUG??) ... already done 
// * fix land command to not then PANCI ... done
// * add GPS heading and distance from GPStiny ... done
// * fix signs on GPS LAT and LNG ... done
// * update to rwsGPS4.h (in uNavDecl.h) ... done
//
// Needs WTOK brought back...
//
#include "uNavMacros.h"
//
// enable "drivers" for selected devices
//
#undef USE_GPS
// #define USE_GPS
//
#include "uNavDecl.h"
#include "rwsTiny.h"


// app includes and variables
#include <SPI.h>
#include <WiFi101.h>
#include <WiFiUdp.h>
// #include <cctype>
//
// variables & constants
//
// key constants
//
// **Vector move**
const int moveDelta = 250;
const int maxCourseSlop = 5;
// **UDP**
const unsigned int localPort = 8888;      // local port to listen on
const char TelloIP[] = "192.168.10.1";
const unsigned int TelloPort = 8889;
const String TelloSDKon = "command";
// **TIMES**
const int gpsWaitTime = 500; // ms
const int watchDogSecs = 600; // 200; // 50; // 20
const int watchDogTime = watchDogSecs * 1000; // ms
const int blinkTimeStop = 1000;
const int blinkTimePanic = 50;
//
// FSM phases, used with fsmPhase
const int fsmStart = 0;
const int fsmPreFlight = 1;
const int fsmWaypoint1 = 2;
const int fsmWaypoint2 = 3;
const int fsmVector1 = 4;
const int fsmVector2 = 5;
const int fsmVector3 = 6;
const int fsmVector4 = 7;
const int fsmVector5 = 8;
const int fsmLogTrans = 9;
const int fsmSleep = 10;  
//
WiFiUDP Udp;
String TelloSSID = "";
int status = WL_IDLE_STATUS;
char packetBuffer[255];
char cmdBuffer[255];
int cnt;
unsigned long int endWaitTime;
unsigned long int endWatchDogTime;
unsigned long int endGPSWaitTime;
int cmdCnt;
int fsmPhase;
// Vector fields
unsigned long vfCourse = 0;   // in deg units
unsigned long vfDistance = 0;  // in cm units
int vfCorrection = 0;   // in +/- degrees
float vfPrevLat = 0.0;  // previous value
float vfPrevLng = 0.0; // previous value
//
// log stuff
// note TELLOX_VERBOSE_ON setting above....
const String logPrompt = "enter a 'g' to see tran log in Serial, enter 'X' to end log display";
const int LOG_REC_TEXT_SIZE = 64;
const int LOG_REC_ARRAY_SIZE = 256; // was 256;
struct logRec {
  int count;
  long millis;
  char mode;
  char text[LOG_REC_TEXT_SIZE];
};
int logDataUsed = 0;
bool logWrapped = false;
// changed to static
static logRec logData2[LOG_REC_ARRAY_SIZE];
// end log stuff
//
// GPS struct and array
//

struct pointRec {
  char cmd[16];
  int cnt;
  float alt;
  float lat;
  float lng;
  int sats;  
  unsigned long distance;  // units?
  unsigned long course;    // units?
};
#ifdef USE_GPS
// home sweat home :)
float homeLat = 0.0;
float homeLng = 0.0;
float homeSats = 0.0;
//
#endif
const int pointArraySize = 20;
int pointArrayAvail = 0;
pointRec pointArray[pointArraySize];
//
// ActTello() commands
//
enum cmds {
  CMD_RIGHT = 1,CMD_LEFT, CMD_HEADING, CMD_CW, CMD_CCW,
  CMD_TAKEOFF, CMD_LAND, CMD_UP, CMD_DOWN,
  CMD_FORWARD, CMD_BACK, CMD_SPEED,
  CMD_WAIT, CMD_WTOK, CMD_STOP,
  CMD_GET_SPEED, CMD_GET_BATTERY, CMD_GET_HEADING, CMD_GET_TIME,
  CMD_VECTOR
};
enum cmd_return {
  RET_OK = 1, RET_BAD_CMD, RET_PANIC_NOW, RET_UNKNOWN_ERROR
};
// Fix millis() and delay() errors (needed with Adafruit M0....)
unsigned long XmillisApp() {
  return millis() / 2;
}
void XdelayApp(unsigned long v) {
  delay(v * 2);
}
//
// storeGPSdata
//

void storeGPSdata(int _index, String _cmd) {
#ifdef USE_GPS  
  strcpy(pointArray[_index].cmd, _cmd.c_str());
  pointArray[_index].cnt = myGPS.getCnt();      
  pointArray[_index].alt = myGPS.getAlt();
  pointArray[_index].lat = myGPS.getLat();
  pointArray[_index].lng = myGPS.getLng();      
  pointArray[_index].sats = myGPS.getSats();
  if (_index == 0) {
    pointArray[_index].distance = 0;
    pointArray[_index].course = 0;  
  } else {
      int k = _index;
      pointArray[k].distance = 99; //(unsigned long)TinyGPS::distance_between(pointArray[k-1].lat, pointArray[k-1].lng,
                                     //              pointArray[k].lat, pointArray[k].lng);
      pointArray[k].course = 101; // (unsigned long)TinyGPS::course_to(pointArray[k-1].lat, pointArray[k-1].lng,
                                     //              pointArray[k].lat, pointArray[k].lng);                                 
  }
#endif    
  return;                                                   
}
//
// storeGPShome()
//
void storeGPShome() {
#ifdef USE_GPS
  homeLat = myGPS.getLat();
  homeLng = myGPS.getLng();  
  homeSats = myGPS.getSats();
#endif
return;  
}
//
// Display message and stop -- used during setup
//
void stopError(int _d, String _msg) {
  Serial.println(_msg);
  Serial.println("Stopped!");
  pinMode(13, OUTPUT);    
  while (true) {
    digitalWrite(13, !digitalRead(13));
    delay(13);
  }  
}
//
// sendCommand to Tello
//
void sendCommand(String _c) {
  // ZZDEBUG
  // return;
#ifndef LOCAL_DEBUG  
  Udp.beginPacket(TelloIP, TelloPort);
  // Udp.write(cmdBuffer, strlen(cmdBuffer))
  Udp.write(_c.c_str(), strlen(_c.c_str()) );
  Udp.endPacket(); 
#endif   
  return;
}
//
// formatSendCmd():
//
void formatSendCmd(String _str, int _arg = -1) {
  String cmdStr = _str; 
  if (_arg >= 0) {
    cmdStr += " ";
    cmdStr += String(_arg);
  } 
  String msg = "Exec: " + cmdStr; 
  logEvent(cmdCnt, "I", msg );   
  sendCommand(cmdStr);
}
//
// localCmdWait():
//
void localCmdWait(int _arg) {
  String msg = ">>>> wait " + String(_arg); 
  logEvent(cmdCnt,"I",msg);
  endWaitTime = XmillisApp() + (_arg * 1000);  
}
//
// ActTello(): 
//
cmd_return ActTello(cmds _cmd, int _arg = -1) {
  // String str; 
  cmd_return ret = RET_UNKNOWN_ERROR;
  /* **ACTIONS** */  
  switch (_cmd) {
    case (CMD_RIGHT): formatSendCmd("right", _arg); ret = RET_OK; break;
    case (CMD_LEFT): formatSendCmd("left", _arg); ret = RET_OK; break;
    case (CMD_HEADING): /*?*/formatSendCmd("????", _arg); ret = RET_OK; break;
    case (CMD_CW): formatSendCmd("cw", _arg); ret = RET_OK; break;
    case (CMD_CCW): formatSendCmd("ccw", _arg); ret = RET_OK; break;
    //
    case (CMD_TAKEOFF): formatSendCmd("takeoff"); ret = RET_OK; break;                    
    case (CMD_LAND): ActTelloLand("Land"); ret = RET_OK; break; 
    case (CMD_STOP): ActTelloLand("Stop"); ret = RET_OK; break;                    
    case (CMD_UP): formatSendCmd("up", _arg); ret = RET_OK; break;
    case (CMD_DOWN): formatSendCmd("down", _arg); ret = RET_OK; break;   
    //
    case (CMD_FORWARD): formatSendCmd("forward", _arg); ret = RET_OK; break;                    
    case (CMD_BACK): formatSendCmd("back", _arg); ret = RET_OK; break;                    
    case (CMD_SPEED): formatSendCmd("speed", _arg); ret = RET_OK; break;
    //
    case (CMD_WAIT): localCmdWait(_arg); ret = RET_OK; break;                    
    case (CMD_WTOK): /* ? */formatSendCmd("???", _arg); ret = RET_OK; break;        
    //                                       
    case (CMD_GET_BATTERY): formatSendCmd("battery?"); ret = RET_OK; break;                    
    case (CMD_GET_HEADING): /* ? */ formatSendCmd("???", _arg); ret = RET_OK; break;
    case (CMD_GET_SPEED):   formatSendCmd("speed?");         ret = RET_OK; break;
    case (CMD_GET_TIME):   formatSendCmd("time?");           ret = RET_OK; break; 
    //
    case (CMD_VECTOR): vector1(_arg); ret = RET_OK; break; // should never return...   
    //
    default: ret = RET_BAD_CMD;    
  }
  return ret;
}

//
// ActTelloLand()
//
void ActTelloLand(String _str) { 
  String msg = ">>>> " + _str + " ...";
  logEvent(cmdCnt, "I", msg ); 
#ifdef USE_GPS  
  gpsActive = false; // turn off GPS reading
#endif   
  formatSendCmd("land");
  if (_str[0] != 'P') {  // land and stop
    XdelayApp(5000); // 5.0 seconds to land
    LEDDelay = blinkTimeStop;  // slow blink normal stop    
  } else { // panic...
    XdelayApp(500);  
    formatSendCmd("land");
    XdelayApp(500);  
    LEDDelay = blinkTimePanic;  // fast blink panic stop      
  }     
  formatSendCmd("emergency"); // then motors OFF!
#ifdef TELLOX_VERBOSE_ON  
  Serial.println(logPrompt);
#endif     
  fsmPhase = fsmLogTrans;
}
#if 0
//
// ActTelloDead()
//
void ActTelloDead(int _arg) {
  // LEDDelay controls "hidden" LED blink timer
  if (_arg == 1) {
    LEDDelay = blinkTimeStop;  // slow blink normal stop
  } else {
    LEDDelay = blinkTimePanic;  // fast blink panic stop  
  }
}
#endif
//
// ActTelloSleep
//
void ActTelloSleep() {
  static bool msgDone = false;
#ifdef TELLOX_VERBOSE_ON
  if (!msgDone) {
    Serial.println("Going to sleep...");
    msgDone = true;
  }  
#endif    
  return; // currently noting to do // background "task" will run
}
//
// Find Tello SSID
//
String findTello() {
  // scan for nearby networks:
  String myTello = "";
  int myCnt = 0;
  int numSsid = WiFi.scanNetworks();
  if (numSsid == -1)
  {
    String msg = "Couldn't get a wifi connection";
    stopError(20, msg);
  }
  // print the network number and name for each network found:
  for (int thisNet = 0; thisNet < numSsid; thisNet++) {
    String SSIDname = WiFi.SSID(thisNet);
    if (SSIDname.startsWith("TELLO-")) {
      myTello = SSIDname;
      myCnt++;
    }
  }
  if (myCnt > 1) {
    myTello = ""; 
    stopError(20, "Too many Tellos!");
  }
  return myTello;
}

//
// Async read from Tello
//
long unsigned int thread_listen () {
  // if there's data available, read a packet
  // note that this thread is largely async and can get the result of
  // a Tello command that was processed earlier than the current Tello command!
  int len = 0;
  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    // read the packet into packetBufffer
    len = Udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = 0;  // make it a proper C string
      for (int j = 0; j < len; j++) {
        if (!isprint(packetBuffer[j])) packetBuffer[j] = ' ';
        packetBuffer[j] = toupper(packetBuffer[j]); // makes life simplier
      }
#if 0  
      // old WTOK code...
      // return processing    
      if (memcmp(packetBuffer, "OK", 2) == 0) {
        replyValue = replyOK;
      } else if (memcmp(packetBuffer, "ERR", 3) == 0) {
        replyValue = replyError;
      } else if (isdigit(packetBuffer[0])) {
        replyValue = atoi(packetBuffer);
      } else {
        replyValue = replyOther;
      }
      if (len < 16) {
        char lBuf[16];
        char tBuf[16];
        ltoa(replyValue, lBuf, 10);
        switch (replyValue) {
          case (replyOK): strcpy(tBuf,"OK"); break;
          case (replyError): strcpy(tBuf,"Error"); break;
          case (replyOther): strcpy(tBuf,"Other"); break;
          case (replyNone): strcpy(tBuf,"none"); break;
          default: strcpy(tBuf,"(num)");
        }
        logEvent(cnt, "T", String(packetBuffer) + " (" + String(packetSize) + ") decoded: " + String(lBuf) + " " + tBuf);
      }
#endif      
    } else {
      // logEvent(cnt, "T", String(" (0)"));
    }
  }
  return len;
}
//
// logEvent -- write to log
//
void logEvent(int cnt, String mode, String event) {
  logData2[logDataUsed].count = cnt;
  logData2[logDataUsed].millis = XmillisApp();
  logData2[logDataUsed].mode = mode[0];
  strncpy(logData2[logDataUsed].text, event.c_str(), LOG_REC_TEXT_SIZE-1);
  logData2[logDataUsed].text[LOG_REC_TEXT_SIZE-1] = '\0';  // insurance....
#ifdef TELLOX_VERBOSE_ON
  Serial.print(logData2[logDataUsed].count);
  Serial.print(",");
  Serial.print(logData2[logDataUsed].millis);
  Serial.print(",");
  Serial.print(logData2[logDataUsed].mode);
  Serial.print(",");
  Serial.println(logData2[logDataUsed].text);
#endif  
  logDataUsed++;
  if (logDataUsed >= LOG_REC_ARRAY_SIZE) {
    logDataUsed = 0;  // blah, wrapped the buffer!
    logWrapped = true;
  }
}
//
// logTrans -- transmit log
//
bool logTrans() {
  // only transmit if Serial is present and a non-X character is entered
  // return of true say log Trans is done
  //
  if (!Serial) return(false);  // try again
  // wait for input     
  if (Serial.available() > 0) {
    // read the incoming byte:
    int incomingByte = Serial.read();
    if (incomingByte == 'X') return(true);
  } else {
    return(false); // no input, try again   
  }
#ifdef USE_GPS
  Serial.println(">>> GPS Dump: <<<");
  for (int k=0; k < pointArrayAvail; k++) {
    char buffer[128];
#if 0    
    unsigned long d = 0.0;
    unsigned long h = 0.0;
    if (k > 0) {
      d = (unsigned long)TinyGPS::distance_between(pointArray[k-1].lat, pointArray[k-1].lng,
                                                   pointArray[k].lat, pointArray[k].lng);
      h = (unsigned long)TinyGPS::course_to(pointArray[k-1].lat, pointArray[k-1].lng,
                                                   pointArray[k].lat, pointArray[k].lng);
    }
#endif       
    sprintf(buffer, "%d,%s,%d,%f,%f,%.2f,%d,%u,%u",
      k,
      pointArray[k].cmd,
      pointArray[k].cnt,      
      pointArray[k].lat,
      pointArray[k].lng,
      pointArray[k].alt,            
      pointArray[k].sats,
      pointArray[k].distance,
      pointArray[k].course
      );
#ifdef TELLOX_VERBOSE_ON    
    Serial.println(buffer);
#endif                      
  }
    
  // DEBUG
  // return(true);                 
#endif  
  Serial.println(">>> LOG Dump: <<<");
  if (logWrapped) {
    logDataUsed =  LOG_REC_TEXT_SIZE;
    // say something
    String txt = "Warning: Buffer Wrapped"; 
#ifdef TELLOX_VERBOSE_ON    
    Serial.print(txt);
#endif        
  }
  int top = logDataUsed;
  for (int k = 0; k < top; k++) {
    String packet = String(logData2[k].count) + ',' + String(logData2[k].millis) + ',' +
                    logData2[k].mode + ',' + String(logData2[k].text) + '\n';
#ifdef TELLOX_VERBOSE_ON    
    Serial.print(packet);
#endif                 
  }  
  Serial.println(">>> End Log Dump <<<");  
#ifdef TELLOX_VERBOSE_ON  
  Serial.println(logPrompt);
#endif     
  return(false);
}

void showGPS() {
#ifdef USE_GPS
  if (gpsActive) {
    char buffer[128];
    float lat =  myGPS.getLat();
    float lng = myGPS.getLng();  
    // remove course and speed as has to be computed over time... 
    sprintf(buffer, "%d,%f,%f,%.2f,s:%d", myGPS.getCnt(), lat, lng, myGPS.getAlt(),
                     myGPS.getSats()); // , myGPS.course, myGPS.speed); // ,%.2f,%.2f
    logEvent(cmdCnt, "G", buffer );     
  }
#endif 
  return;
}

//
// appSetup()
//
void appSetup() {
  // 
  // pause and blink giving user a startup warning
  unsigned int freeAt = XmillisApp() + 5000;
  while (XmillisApp() < freeAt) {
    digitalWrite(13, !digitalRead(13));
    delay(200);    
  }
#ifdef LOCAL_DEBUG  
  Serial.println("in LOCAL_DEBUG Test Mode ..... "); 
#endif    
#ifdef USE_GPS
  // move GPS to Serial1 (was Serial3)
  Serial1.setTimeout(20);  // make serial reads not have much delay 
  Serial1.begin(9600);
  myGPS.begin(Serial1);
  // GPS check -- moved earlier in app start...
  logEvent(cmdCnt, "I", "GPS Stat Check Loop");
  bool done = false;
  while (1) {  
    myGPS.loop();   // this is a dirty way to start GPS ....
    if (XmillisApp() > endGPSWaitTime) {     
      showGPS(); 
      endGPSWaitTime = XmillisApp() + gpsWaitTime; 
      if (myGPS.sats >= 9) done = true;  // arbitrary number of active sats... 
      // simple LED wait blink
      digitalWrite(13, !digitalRead(13));               
    }
    if (done) break; // out of while 
  } 
  logEvent(cmdCnt, "I", "GPS Stat Check OK");  
  // load first GPS point array item
  storeGPSdata(pointArrayAvail,"*LOAD*");  
  pointArrayAvail++;
  // end load GPS  
#endif     
#ifdef USE_IBUS
  IBus.begin(Serial1,IBUSBM_NOTIMER);
  // this is teensy specific
#ifdef KINETISK  
  iBusTimer.begin(timerPop, 10000);  // IBus.loop to run every 0.01 second  
#endif    
#endif 
#ifdef KINETISK
  // this macro existing marks a Teensy 3!
  Serial.println("Teensy 3.X");
#endif
  // Start WiFi and Find to Tello (real flight -- not LOCAL_DEBUG)
#ifndef LOCAL_DEBUG
  WiFi.setPins(8, 7, 4, 2);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);  
  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    stopError(20, "WiFi shield not present");
  }  
  cnt = 0;
  while (true) {
    TelloSSID = findTello();
    if (TelloSSID.length() > 0) {
      Serial.print("TelloSSID: ");
      Serial.println(TelloSSID);
      break;
    }
    cnt++;
    Serial.print("No Tello found ");
    Serial.println(cnt); 
    for (int k = 0; k < 50; k++) { // flash 50 times then try again
      digitalWrite(13, !digitalRead(13));
      delay(100); // LED_NO_TELLO);
    }
  }
  // attempt to connect to Tello's WiFi network:
  status = WiFi.begin(TelloSSID); //, pass);
  while (status != WL_CONNECTED) {
    Serial.println("Tello connect wait..");
    delay(10000); // wait 10 seconds 
  }   
  // turn on the UDP access
  Udp.begin(localPort);
  // Send Tello command  to enter SDK mode
  // sendCommand(TelloSDKon);
  formatSendCmd(TelloSDKon);  
#endif  
#ifndef USE_GPS 
  logEvent(cmdCnt, "I", "GPS not in use!");
#endif
  cnt = 0;
  endWaitTime = 0;
  endGPSWaitTime = 0; 
  cmdCnt = 0; 
  fsmPhase = fsmStart; 
  endWatchDogTime = XmillisApp() + watchDogTime;     
  logEvent(cmdCnt, "I", "App setup completed!");
}

void processPreFlight() {
  // process watch dog timer, panic if needed
  if (XmillisApp() > endWatchDogTime) {
    // logEvent(cmdCnt, "I", ">> Watch Dog Timer Pop");  
    ActTelloLand("Panic -- Watch Dog Timer Pop");    
  } else {  
    // process wait
    // else
    // process command
    if (XmillisApp() > endWaitTime) {
      endWaitTime = 0;
      /* **FLY** **PATH** */ 
      // do next cmd 
      const int dist = 200; // 50; // 100; 
      const int dist2 = dist * 2;  
      const int waitTime = 4; // 3; // 6
      const int waitTime2 = waitTime * 2; 
      const int longTime = 10;
      ; 
      // **COMMANDS**     
      switch (cmdCnt) {   
        // near ground/inside  test  
        case 0: ActTello(CMD_GET_BATTERY); break;  
// just hover        
#if 0 
        case 1: ActTello(CMD_WAIT, waitTime);     break;      
        case 2: ActTello(CMD_TAKEOFF);            break;  
        case 3: ActTello(CMD_WAIT,waitTime);      break;              
        case 4: ActTello(CMD_GET_TIME);           break;                      
        case 5: ActTello(CMD_WAIT,waitTime);      break;
        case 6: ActTello(CMD_UP,25);              break;           
        case 7: ActTello(CMD_WAIT,waitTime);      break;     
        case 8: ActTello(CMD_RIGHT,25);              break; 
        case 9: ActTello(CMD_WAIT, longTime);     break;    
        case 10: ActTello(CMD_LEFT,25);           break; 
        case 11: ActTello(CMD_WAIT, longTime);    break;
        case 12: ActTello(CMD_RIGHT,25);             break; 
        case 13: ActTello(CMD_WAIT, longTime);    break;
        case 14: ActTello(CMD_LEFT,25);           break; 
        case 15: ActTello(CMD_WAIT, longTime);    break; 
        case 16: ActTello(CMD_GET_TIME);          break;                      
        case 17: ActTello(CMD_WAIT,waitTime);     break; 
        case 18: ActTello(CMD_GET_BATTERY);       break; 
        case 19: ActTello(CMD_WAIT,waitTime);     break;                                                             
        case 20: ActTello(CMD_LAND);              break;
// full run...        
#else
        case 1: ActTello(CMD_WAIT, waitTime  );  break;      
        case 2: ActTello(CMD_TAKEOFF);     break;           
        case 3: ActTello(CMD_WAIT,waitTime);      break;
        case 4: storeGPSdata(pointArrayAvail,"TAKEOFF");  pointArrayAvail++; break;
        case 5: ActTello(CMD_UP, 50);     break;           
        case 6: ActTello(CMD_WAIT,waitTime);      break;
        case 7: storeGPSdata(pointArrayAvail,"UP");  pointArrayAvail++; storeGPShome(); break;  // note setting of home        
        case 8: ActTello(CMD_LEFT,dist);     break;           
        case 9: ActTello(CMD_WAIT,waitTime);      break; 
        case 10: storeGPSdata(pointArrayAvail,"LEFT-1");  pointArrayAvail++; break;        
        case 11: ActTello(CMD_RIGHT,dist2);    break;           
        case 12: ActTello(CMD_WAIT,waitTime2);      break;
        case 13: storeGPSdata(pointArrayAvail,"RIGHT");  pointArrayAvail++; break;            
        case 14: ActTello(CMD_LEFT,dist);     break;           
        case 15: ActTello(CMD_WAIT,waitTime);      break;
        case 16: storeGPSdata(pointArrayAvail,"LEFT-2");  pointArrayAvail++; break;           
        case 17: ActTello(CMD_FORWARD,dist); break;           
        case 18: ActTello(CMD_WAIT,waitTime);      break;  
        case 19: storeGPSdata(pointArrayAvail,"FORWARD");  pointArrayAvail++; break;           
        case 20: ActTello(CMD_BACK,dist);   break;           
        case 21: ActTello(CMD_WAIT,waitTime);      break;    
        case 22: storeGPSdata(pointArrayAvail,"BACK");  pointArrayAvail++; break;                                           
        // case 23: ActTello(CMD_LAND);        break;
        case 23: ActTello(CMD_VECTOR, 999);   break;  // lands when done
#endif       
        // end ground                  
        default: ActTelloLand("Panic -- Commmand List Overflow");
      } // of 
      cmdCnt++;
    }
  }  
}

void waypoint1() {
  // not yet implimented
  ActTelloLand("Panic -- waypoint1() not implemented");  
}

void waypoint2() {
  // not yet implimented  
  ActTelloLand("Panic -- waypoint2() not implemented");  
}

//
// vector1(): Initialize vector move sequence
//
void vector1(int dist) {
  logEvent(cmdCnt, "I", "Vector mode!");  
  endWaitTime = 0; // is this good??? 
  vfDistance = dist;   
#ifdef USE_GPS  
  vfCourse = -9999; // special invalid value
  vfPrevLat = homeLat;
  vfPrevLng = homeLng;
#endif
  fsmPhase = fsmVector2; 
  return; 
}

//
// vector2(): Determine if rotate needed and skip vector3() if not rotate needed
//
void vector2() {
  if (XmillisApp() < endWaitTime) {
    return; // wait and do nothing
  }
  endWaitTime = 0;  
#ifdef USE_GPS  
  // get current distance and course
  float curLat = myGPS.getLat();
  float curLng = myGPS.getLng();
  unsigned long distance = (unsigned long)TinyGPS::distance_between(vfPrevLat, vfPrevLng, curLat, curLng);
  unsigned long course = (unsigned long)TinyGPS::course_to(vfPrevLat, vfPrevLng ,curLat, curLng); 

  // special init case
  if (vfCourse == -9999) {
    vfCourse = course;
  }
  // rotate as needed based upton vfCourse
  int dif = (int) vfCourse - course;
  char buffer[128];
  sprintf(buffer,"Vector course calc, dif: %d", dif);  
  logEvent(cmdCnt, "I", buffer);    
  // update prev...
  vfPrevLat = curLat;
  vfPrevLng = curLng;
  vfCourse = course;  
#if 0  
  // DEBUG
  Serial.print(vfCourse); Serial.print(" ");
  Serial.print(course); Serial.print(" ");
  Serial.println(dif);
  // end DEBUG
#endif  
  if (abs(dif) > maxCourseSlop) {
    // rotation correction needed
    vfCorrection = (dif + 720) % 360;      
    fsmPhase = fsmVector3;
  } else {
    // no rotation correction needed
    vfCorrection = 0;    
    fsmPhase = fsmVector4;     
  }
  return; 
#else
    // no rotation correction needed
    vfCorrection = 0;    
    fsmPhase = fsmVector4;    
#endif 
}

//
// vector3(): Perform optional rotate step
//
void vector3() {
  if (XmillisApp() < endWaitTime) {
    return; // wait and do nothing
  }
  endWaitTime = 0;
#ifdef USE_GPS   
  // pick direction and rotate
  int degs = abs(vfCorrection);
  if (vfCorrection >= 0) {
    ActTello(CMD_CW, degs);    
  } else if (vfCorrection <= 0) {
    ActTello(CMD_CCW, degs);     
  }
  if (degs != 0) {
    ActTello(CMD_WAIT, 3); // short wait  
  }
  fsmPhase = fsmVector4; // assume worked, on we go     
 #else
  ActTelloLand("Panic -- vector3() and no GPS"); 
#endif  
}

//
// vector4(): Perform move steps
//
void vector4() {

  if (XmillisApp() < endWaitTime) {
    return; // wait and do nothing
  }
  endWaitTime = 0; 
  // process a move cycle, uses vfDistance
  //          Nmove - Move
  //          if Distance less than Move, Nmove = Distance
  //          else Distant -= Move
  //          forward Nmove distance
  //          restart wait
  if (vfDistance > moveDelta) {
    ActTello(CMD_FORWARD, moveDelta);
    vfDistance -= moveDelta;
  } else {
    ActTello(CMD_FORWARD, vfDistance);    
    vfDistance = 0;
  }
  ActTello(CMD_WAIT, 6);
  if (vfDistance <= 0) {
    fsmPhase = fsmVector5;  // exit move forward/wait sequence
  }
  return;
}

//
// vector5(): wait while final move step completes
//
void vector5() {
  if (XmillisApp() < endWaitTime) {
    return; // wait and do nothing
  }
  endWaitTime = 0;  
  ActTello(CMD_LAND);
  return;
} 


void appLoop() {
  //
  // Tello "loop" code
  //  
  if (XmillisApp() > endGPSWaitTime) {
    showGPS();
    endGPSWaitTime = XmillisApp() + gpsWaitTime;
  } 
  // removed unused sensor code from uNav Template
  int len = thread_listen();
  if (len > 0) {
    // Serial.print(">> ");
    // Serial.println(packetBuffer);
    String msg = ">> " + String(packetBuffer);
    logEvent(cmdCnt, "R", msg ); 
  }
  delay(10);

  switch (fsmPhase) {
    case fsmStart: fsmPhase = fsmPreFlight; break;  // nothing for now
    case fsmPreFlight: processPreFlight();  break; // short flight to check GPS 
    case fsmWaypoint1: waypoint1(); break;
    case fsmWaypoint2: waypoint2(); break;
    // case fsmVector1: vector1(); break; // not used
    case fsmVector2: vector2(); break;
    case fsmVector3: vector3(); break;
    case fsmVector4: vector4(); break; 
    case fsmVector5: vector5(); break;   
    case fsmLogTrans: if (logTrans()) {fsmPhase = fsmSleep;} break;
    case fsmSleep: ActTelloSleep(); // just sleep
  }
  // delay(500); // basically for debugging
  cnt++;
}
//
#include "uNavBase.h"
// end of module
