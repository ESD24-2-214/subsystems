#include <Arduino.h>
#include <Wire.h>

#ifndef MPU_I2C_HPP
#define MPU_I2C_HPP
#endif

#define MPU9255_ADDRESS 0x68 // Device address when ADO = 0
// #define MPU9255_ADDRESS 0x69 // Device address when ADO = 1
#define AK8963_ADDRESS 0x0C //  Address of magnetometer

#define SDA 18 // Signal Data Line pin
#define SCL 17 // Signal Clock Line pin


#define Bypass_Enable_Config_add 0x37 // I2C_BYPASS_EN register


void Magnometer_Bypass(bool state);
