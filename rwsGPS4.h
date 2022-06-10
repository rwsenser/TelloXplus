// rwsGPS3.h based on rwsGPS2.h
// Bob Senser, 2021-12-27

// 2021-12-27: add access to $GPRMC and add satellites from $GPGGA

// simple GPS output parser 
// needs GPS to be output standard lines, repeatedly
// tested with: 2?
//

//   
// for teensy and similar devices with several Serial<x> ports 
//

#undef GPS_PASSTHRU
// #define GPS_PASSTHRU

#include <Arduino.h>   // needed?

class GPS {
  public:
  const static int BUFFER_SIZE = 64;
  char buffer[BUFFER_SIZE + 4];
  char cntBuffer[32];
  // reading variables
  int fieldCnt;
  int len;
  int tLen;
  int cnt;
  unsigned long ts;
  char type;
  boolean skipLine;
  float alt;
  float lat;
  char latR;
  float lng;
  char lngR;
  int sats;
  int recType;
  float speed;
  float course;
  const static int TOKENSIZE = 16;
  char token[TOKENSIZE];
  Stream *gpsStream;

  GPS() { } // no ambition at all!
  
  void begin(HardwareSerial &_s) {  
    gpsStream =  &_s;
    fieldCnt = 0; 
    skipLine = false;
    strcpy(token,"");     
    type = 'G';
    cnt = 0;
  }

  // for lat/log:  DD = d + (min/60) + (sec/3600)
  // here we have only degs and mins (no secs, done as fractional minutes)
  float toDegs( float _v) {
    int d = ((int) _v) / 100;   // degrees
    float m = (_v - (d * 100)) / 60.0; // minutes expressed in degrees
#if 0  
    Serial.print("DEBUG:");
    Serial.print(" ");
    Serial.print(_v);
    Serial.print(" ");
    Serial.print(d); 
    Serial.print(" "); 
    Serial.println(m);        
    // delay(1000);
#endif  
    return (d + m); 
  }

  boolean processField(int fCnt, char * t, boolean skipLine) {
    boolean skip = skipLine;
    if (!skip && (strlen(t) > 0)) {
      // Serial.print("Field: "); Serial.print(fCnt); Serial.print(" :: "); Serial.println(t);
      // 2021-12-11: fix to allow GNGGA records too
      //if (fCnt == 0 && (strcmp(t,"$GPGGA") != 0)) {  // we only want to use GPGGA records
      // bool use = false;
      // if (strcmp(t,"$GPGGA") == 0) { use = true; recType = 1; }
      //  else if (strcmp(t,"$GNGGA") == 0) { use = true; recType = 1; }
      //else if (strcmp(t,"$GPRMC") == 0) { use = true; recType = 2; }
      // determine record type and decided if record is used
      if (fCnt == 0) {
        recType = 0; // skip
        if (strcmp(t,"$GPGGA") == 0) {
          // Serial.println("----found GPGGA!");
          recType = 1;
        } else if (strcmp(t,"$GNGGA") == 0) {
          // Serial.println("----found GNGGA!");
          recType = 1;
        } else if (strcmp(t,"$GPRMC") == 0) {
          // Serial.println("----found GPRMC!");
          recType = 2;
        }     
      }
      else if (recType == 1) {
        // process the field by its number
        switch (fCnt) {
          case (2): // lat
            ts = millis();          
            lat = atof(t);
            lat = toDegs(lat);
            // if (serialLevel == 2) { Serial.print(" lat: "); Serial.println(lat); }
            break;
          case (3): // lat region
            latR = t[0];
            // if (serialLevel == 2) { Serial.print(" lat r: "); Serial.println(t); }
            break;        
          case (4): // longitude
            lng = atof(t);
            lng = toDegs(lng);
            // if (serialLevel == 2) { Serial.print(" lng: "); Serial.println(lng); }
            break;       
          case (5): // longitude region
            lngR = t[0];
            // if (serialLevel == 2) { Serial.print(" lng r: "); Serial.println(t); }
            break; 
          case (7): // satellites
            sats = atoi(t);
            break;       
          case (9): // altitude
            alt = atof(t) * 3.28084; // in feet
            // if (serialLevel == 2) { Serial.print(" alt: "); Serial.println(alt); }
            break;        
        } /* of switch */
      } else if (recType == 2) {
        switch (fCnt) {
          case (7): // speed
              speed = atof(t);
            break;
          case (8): // course
              course = atof(t);
            break;
        } 
      }
    }
    return skip;
  }

  float getLng() {
    float lngCor = lng;
    if (lngR == 'W') lngCor = -lngCor; // correct for west    
    return lngCor;
  }
  float getLat() {
    float latCor =  lat;
    if (latR == 'S') latCor = -latCor; // correct for south    
    return latCor;
  }
  float getAlt() {
    return alt;
  }
  int getSats() {
    return sats;
  }
  int getCnt() {
    return cnt;
  }
  unsigned long getTs() {
    return ts;
  }

  void loop() {
    if (gpsStream -> available()>0) {
      // DEBUG
      // this is the slow part!!! fixed by setting Serial timer
      len = gpsStream -> readBytes(buffer, sizeof(buffer)-1);
      buffer[len] = 0;
      // end of slow...      
#ifdef GPS_PASSTHRU
      Serial.print(">> ");
      Serial.println(buffer);
#else    
      // for (int k=0; k < strlen(buffer); k++) {
      for (int k=0; k < len; k++) {
        
        char c = buffer[k];
        // Serial.print("c: "); Serial.print(c); Serial.print(" :: "); Serial.println(((int) c));
        if (c == ',') {
          skipLine = processField(fieldCnt, token, skipLine);
          fieldCnt++;  
          // strcpy(token,"");
          token[0] = '\0';          
        } else if (c == '\n' || c == '\r') {
          skipLine = processField(fieldCnt, token, skipLine);
          fieldCnt = 0;
          // strcpy(token,"");
          token[0] = '\0'; 
          skipLine = false;           
        } else {  // add char to token
          // make faster
#if 0          
          char c2[2];
          c2[0] = c;
          c2[1] = 0;
          
          if (strlen(token) < TOKENSIZE-1) { // prevent overflow
            strcat(token,c2);
          }
#else  
          tLen = strlen(token);
          if (tLen < TOKENSIZE-1) { // prevent overflow
            token[tLen] = c;
            token[tLen+1] = '\0';
          }
#endif             
        }
      }
#endif
       len = len;   
    }
    cnt++;
    return;
  }
};

/* end of code */
