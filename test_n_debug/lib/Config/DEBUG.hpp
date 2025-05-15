#ifndef DEBUG_HPP
#define DEBUG_HPP
#include <Arduino.h>
#define ERROR_LOG(x, y) {Serial.print(x); Serial.println(y);} 
// #define ERROR_LOG(x, y) {}

#define PRINT_DEBUG(x) Serial.print(x);
// #define PRINT_DEBUG(x) {}

#endif // DEBUG_HPP