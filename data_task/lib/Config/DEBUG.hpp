#ifndef DEBUG_HPP
#define DEBUG_HPP
#include <Arduino.h>
#define DEBUG(x, y) {Serial.print(x); Serial.println(y);} 
// #define DEBUG(x, y) {}

// #define DEBUG2(x) Serial.println(x);
#define DEBUG2(x) {}

#endif // DEBUG_HPP