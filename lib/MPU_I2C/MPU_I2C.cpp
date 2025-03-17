//#include <Arduino.h>
//#include <Wire.h>
#include <MPU_I2C.hpp>


void Magnometer_Bypass(bool state)
{
  Wire.beginTransmission(MPU9255_ADDRESS);
    Wire.requestFrom(Bypass_Enable_Config_add, 1, false);
    uint8_t ConfigByte = Wire.read();
    ConfigByte = (state = true) ? (ConfigByte | (1 << 1)) : (ConfigByte & ~(1 << 1));
    Wire.write(ConfigByte);
  Wire.endTransmission();
}
