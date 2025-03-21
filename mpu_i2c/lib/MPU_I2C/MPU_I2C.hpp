#include <Arduino.h>
#include <Wire.h>
#include <MPU_ADDRESS.hpp>
#include <AK8963_ADDRESS.hpp>

#ifndef MPU_I2C_HPP
#define MPU_I2C_HPP
#endif

#define MPU9255_ADDRESS 0x68 // Device address when ADO = 0
// #define MPU9255_ADDRESS 0x69 // Device address when ADO = 1
#define AK8963_ADDRESS 0x0C //  Address of magnetometer


#define Bypass_Enable_Config_add 0x37 // I2C_BYPASS_EN register

typedef struct {
    double xy;
    double zx;
    double yz;
} BiVector;
typedef struct {
    double x;
    double y;
    double z;
} Vector;

void I2Cbus_SCCAN(void);
void measuremode_config_togle(bool mode);
void set_bit_config (uint8_t unit_addr, uint8_t local_addr, bool state, uint8_t bit_pos);
void Magnometer_Bypass(bool state);
void read(uint8_t unit_addr, uint8_t local_addr, uint8_t *data, uint8_t size);
void write(uint8_t unit_addr, uint8_t local_addr, uint8_t data_byte);
void read_gryroscope(Vector *gyro_data);
void read_accelerometer(Vector *acc_data);
void read_magnetometer(Vector *mag_data);