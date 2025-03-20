#include <Arduino.h>
#include <Wire.h>
#include <MPU_I2C.hpp>
#define ClockSpeed 400000 // 400kHz

#define SDA 18 // Signal Data Line pin
#define SCL 17 // Signal Clock Line pin

void setup() {
Serial.begin(115200);
while (!Serial){};

Wire.begin(SDA, SCL);
Wire.setClock(ClockSpeed);
Magnometer_Bypass(true);

}

void loop() {
  // put your main code here, to run repeatedly:


}

