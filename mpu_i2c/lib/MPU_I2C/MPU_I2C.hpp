#include <Arduino.h>
//#include <Wire.h>
#include <I2C.hpp>
#include <MPU_ADDRESS.hpp>
#include <AK8963_ADDRESS.hpp>
#include <Config.hpp>

#ifndef MPU_I2C_HPP
#define MPU_I2C_HPP

#define UNKNOWN 0
#define BYTE 1
#define MPU9255_ADDRESS 0x68 // Device address when AD0 = 0 e.i. to GND
// #define MPU9255_ADDRESS 0x69 // Device address when AD0 = 1 e.i. to VDD
#define AK8963_ADDRESS 0x0C //  Address of magnetometer
#define Bypass_Enable_Config_add 0x37 // I2C_BYPASS_EN register

#define gyro_scale_factor250 131.0 // 250 degrees/sec
#define gyro_scale_factor500 65.5 // 500 degrees/sec
#define gyro_scale_factor1000 32.8 // 1000 degrees/sec
#define gyro_scale_factor2000 16.4 // 2000 degrees/sec

#define acc_scale_factor2g 16.384 // 2g
#define acc_scale_factor4g 8.192 // 4g
#define acc_scale_factor8g 4.096 // 8g	
#define acc_scale_factor16g 2.048 // 16g

#define mag_scale_factor1 0.15 // 0.15uT/LSB
#define mag_scale_factor2 0.6 // 0.6uT/LSB

extern MasterI2C MPU_I2C;

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

typedef enum {
    unknown = 0,
    MAGNOTOMETER = 1,
    ACCELEROMETER,
    GYROSCOPE
} Sensor;

typedef struct {
    Sensor sensor;
    double scale_factor;
    uint8_t unit_addr;
    uint8_t local_addr;
//  uint8_t full_scale_range;
    Vector vector;
} SensorVector;
typedef enum {
    GFS_250 = 1,
    GFS_500,
    GFS_1000,
    GFS_2000
} gyro_full_scale_range;

typedef enum {
    AFS_2G = 1,
    AFS_4G,
    AFS_8G,
    AFS_16G
}acc_full_scale_range;
typedef enum {
    BIT_14 = 1,
    BIT_16
}mag_resolution;

typedef enum {
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


#endif // MPU_I2C_HPP