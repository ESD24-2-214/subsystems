// defining all the addresses for the MPU9255 sensor

#ifndef MPU_ADDRESS_HPP
#define MPU_ADDRESS_HPP
#endif

// Config registers for Gyroscope and Accelerometer offsets
#define XG_OFFSET_H 0x0D
#define XG_OFFSET_L 0x0E
#define YG_OFFSET_H 0x0F
#define YG_OFFSET_L 0x10
#define ZG_OFFSET_H 0x11
#define ZG_OFFSET_L 0x12

#define XA_OFFSET_H 0x77
#define XA_OFFSET_L 0x78
#define YA_OFFSET_H 0x7A
#define YA_OFFSET_L 0x7B
#define ZA_OFFSET_H 0x7D
#define ZA_OFFSET_L 0x7E

#define INT_PIN_CFG 0x37 // Interrupt configuration, set Bit1 ti endable bypassmode

// Data registers for Gryoscope, Accelerometer and Temperature
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

