#include <Arduino.h>
#include <Wire.h>
#include <MPU_ADDRESS.hpp>

#ifndef MPU_I2C_HPP
#define MPU_I2C_HPP
#endif

#define MPU9255_ADDRESS 0x68 // Device address when ADO = 0
// #define MPU9255_ADDRESS 0x69 // Device address when ADO = 1
#define AK8963_ADDRESS 0x0C //  Address of magnetometer




#define Bypass_Enable_Config_add 0x37 // I2C_BYPASS_EN register

void set_bit_config (uint8_t unit_addr, uint8_t local_addr, bool state, uint8_t bit_pos);
void Magnometer_Bypass(bool state);
void Read(uint8_t unit_addr, uint8_t local_addr, uint8_t *data, uint8_t size);