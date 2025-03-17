#include <Arduino.h>
#include <Wire.h>
#include <MPU_I2C.hpp>
#define ClockSpeed 400000 // 400kHz


void setup() {
Serial.begin(115200);
while (!Serial){};
pinMode(SDA, OUTPUT);

Wire.begin(SDA, SCL);
Wire.setClock(ClockSpeed);
Magnometer_Bypass(true);

}

void loop() {
  // put your main code here, to run repeatedly:


}

