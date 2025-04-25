#ifndef MPU_I2C_HPP
#define MPU_I2C_HPP
#include <I2C.hpp>
/*
#include <Arduino.h>
//#include <Wire.h>
#include <MPU_ADDRESS.hpp>
#include <AK8963_ADDRESS.hpp>
#include <DEBUG.hpp>
#include <semaph.hpp>
#include <Config.hpp>
*/

#define UNKNOWN 0
#define BYTE 1
#define MPU9255_ADDRESS 0x68 // Device address when AD0 = 0 e.i. to GND
// #define MPU9255_ADDRESS 0x69 // Device address when AD0 = 1 e.i. to VDD
#define AK8963_ADDRESS 0x0C //  Address of magnetometer
#define Bypass_Enable_Config_add 0x37 // I2C_BYPASS_EN register

extern MasterI2C MPU_I2C; // I2C object

typedef struct { // 3D bivector
    double xy;
    double zx;
    double yz;
} BiVector;
typedef struct { // 3D vector
    double x;
    double y;
    double z;
} Vector;

typedef enum { // Sensor types
    unknown = 0,
    MAGNOTOMETER = 1,
    ACCELEROMETER,
    GYROSCOPE
} Sensor;

typedef struct { // Sensor vector structure for reading data
    Sensor sensor;
    double scale_factor;
    uint8_t unit_addr;
    uint8_t local_addr;
//  uint8_t full_scale_range;
    Vector vector;
} SensorVector;
typedef enum { // Gyroscope full scale range
    GFS_250 = 1,
    GFS_500,
    GFS_1000,
    GFS_2000
} gyro_full_scale_range;

typedef enum { // Accelerometer full scale range
    AFS_2G = 1,
    AFS_4G,
    AFS_8G,
    AFS_16G
}acc_full_scale_range;
typedef enum { // Magnetometer resolution
    BIT_14 = 1,
    BIT_16
}mag_resolution;

typedef enum { // Magnetometer measurement mode
    POWER_DOWN = 0,
    SINGLE_MEASUREMENT,
    MEAS_MODE1,
    MEAS_MODE2,
    EXTERNAL_TRIGGER,
    SELF_TEST,
    FUSE_ROM_ACCESS,
}mag_meas_mode;

void magnotometer_selftest(void);
void magnotometer_softreset(void);
void hard_reset_MPU9255(void);
void I2Cbus_SCCAN(void);
void mag_resolution_config(mag_resolution mode);
uint8_t mag_meas_config(mag_meas_mode mode);
void set_bit_config (uint8_t unit_addr, uint8_t local_addr, bool state, uint8_t bit_pos);
void bypass_to_magnometer(bool state);
void read(uint8_t unit_addr, uint8_t local_addr, uint8_t *data, uint8_t size);
void write(uint8_t unit_addr, uint8_t local_addr, uint8_t data_byte);
void read_gryroscope(Vector *gyro_data);
void read_accelerometer(Vector *acc_data);
void read_magnetometer(Vector *mag_data);
void scale(Vector *Vec, double scale_factor);
int8_t factorScale(SensorVector *Vec);
int8_t read_data(SensorVector *Vec);
int8_t initSensorVector(SensorVector *Vec, Sensor sensor_t, double scale_factor);
void set_2bit(uint8_t unit_addr, uint8_t local_addr, uint8_t bit2, uint8_t bitpos);
void gyro_fs_sel(gyro_full_scale_range range);
void accel_fs_sel(acc_full_scale_range range);

void mag_debug_print(void);
void mpu_debug_print(void);
void dataPrint(Vector *Vec);


#endif // MPU_I2C_HPP