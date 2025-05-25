#ifndef DEBUG_HPP
#define DEBUG_HPP
#include <Arduino.h>
#define ERROR_LOG(x, y) {Serial.print(x); Serial.println(y);} 
// #define ERROR_LOG(x, y) {}

#define PRINT_DEBUG(x) Serial.print(x);
//#define PRINT_DEBUG(x) {}

#define PRINT_DEBUG_NR(x, y) Serial.print(x, y);
//#define PRINT_DEBUG_NR(x, y) {}

#endif // DEBUG_HPP