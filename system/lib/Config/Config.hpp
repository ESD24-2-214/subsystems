#ifndef CONFIG
#define CONFIG
#include <Arduino.h>

static const BaseType_t pro_cpu = 0;
static const BaseType_t app_cpu = 1;

#define gyro_scale_factor250 (1 / 131.0) // 250 degrees/sec LSB/(degrees/sec)
#define gyro_scale_factor500 (1 / 65.5)  // 500 degrees/sec LSB/(degrees/sec)
#define gyro_scale_factor1000 (1 / 32.8) // 1000 degrees/sec LSB/(degrees/sec)
#define gyro_scale_factor2000 (1 / 16.4) // 2000 degrees/sec LSB/(degrees/sec)

#define acc_scale_factor2g (1 / 16384.) // 2g LSB/(g)
#define acc_scale_factor4g (1 / 8192.)  // 4g LSB/(g)
#define acc_scale_factor8g (1 / 4096.)  // 8g LSB/(g)
#define acc_scale_factor16g (1 / 2048.) // 16g LSB/(g)

#define mag_scale_factor1 0.6  // 0.6uT/LSB
#define mag_scale_factor2 0.15 // 0.15uT/LSB

#define mag_hardiron_bias_e1 1.5  //Calibration of the magnetometer x-axis 
#define mag_hardiron_bias_e2 34.0 //Calibration of the magnetometer y-axis
#define mag_hardiron_bias_e3 0.0     //Calibration of the magnetometer z-axis

#define LDR_MAX 4095           // LDR max value
#define LDR_MIN 0              // LDR min value
#define LDR_CALIBRATION_E1 411 // to much in this direction
#define LDR_CALIBRATION_E2 397 // to much in this direction

#define ClockSpeed 400000 // I2C clockspeed: 400kHz

#define SDA 8 // Signal Data Line pin
#define SCL 9 // Signal Clock Line pin

// Control Loop
#define CONTROL_PERIODE 0.5 // seconds
#define K_P 0.4
#define K_I 0.0
#define K_D 2.0

// SensorRead Loop
const uint16_t SENSORREAD_PERIODE = (CONTROL_PERIODE * 1000.); // milisecond

// LDR
#define LDR_PIN_F 4            // LDR GPIO pin number
#define LDR_PIN_L 1            // LDR GPIO pin number
#define LDR_PIN_R 2            // LDR GPIO pin number
#define LDR_PIN_B 0            // LDR GPIO pin number
#define LDR_SAMPLES 30         // samples the ldr takes
const uint8_t LDR_PERIODE = 5; // LDR sample rate in ms
                               //// The MAX time the ldr can take in millisecond

// MAG
#define MAG1_EN 21               //
#define MAG2_EN 3                //
#define MAG1_CW 19               //
#define MAG1_CCW 18              //
#define MAG2_CW 20               //
#define MAG2_CCW 10              //
#define MAG_POWER_DOWN_TIME_MS 2 // Time in ms to power down the magnetorquer

// LED
#define LED1 5 //
#define LED2 6 //
#define LED3 7 //

#define PWM_RES 8       // in bits 2^n: n = 10
#define PWM_RES_MAX 255 // 2^n - 1: n = 10
#define PWM_FREQ 1e4    // in Hz

#endif
