//
// rwsTiny.h
// 2022-03-24

#include "TinyGPS.h"
#include <avr\pgmspace.h>

TinyGPS test_gps;

unsigned long tiny_distance (float lat1, float long1, float lat2, float long2){
  unsigned long d = (unsigned long)TinyGPS::distance_between(lat1, long1, lat2, long2);  
  return d;
}

unsigned long tiny_course (float lat1, float long1, float lat2, float long2) {
  unsigned long c = (unsigned long)TinyGPS::course_to(lat1, long1, lat2, long2);  
  return c;  
}

// end of rwsTiny.h
