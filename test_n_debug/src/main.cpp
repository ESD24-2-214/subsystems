#include "Arduino.h"
#include "loop.hpp"
#include "math.h"
#include "math_stuff.hpp"
#include <AK8963_ADDRESS.hpp>
#include <Config.hpp>
#include <Hbro.hpp>
#include <LDR.hpp>
#include <MPU_ADDRESS.hpp>
#include <MPU_I2C.hpp>
#include <cstdint>


SensorVector Mag_Data = {MAGNOTOMETER, mag_scale_factor2, AK8963_ADDRESS, MAGNO_XOUT_L, {0.0, 0.0, 0.0}};
Vector Mag_calibration_data = {0.0, 0.0, 0.0};
void calibrate_magnetometer(SensorVector *Vec, Vector *calibration_data) {
  Vec->vector.e1 -= Mag_calibration_data.e1;
  Vec->vector.e2 -= Mag_calibration_data.e2;
  Vec->vector.e3 -= Mag_calibration_data.e3;
}

void setup() {
  Serial.begin(115200); // Initialize serial monitor
  while (!Serial) {}
  MPU_I2C.begin(SDA, SCL, ClockSpeed); // Initialize I2C communication
  delay(2000); // Wait for serial monitor to open
  mag_resolution_config(BIT_16); // Set magnetometer resolution to 16 bit
  mag_meas_config(MEAS_MODE2); // Set magnetometer to power down mode
  bypass_to_magnometer(true); // Set the bypass mode to true
  delay(100); // Wait for the bypass mode to be set
  MPU_I2Cbus_SCCAN(); // Scan the I2C bus for devices
  write(AK8963_ADDRESS, ASTC, 0b00000000); // Set the magnetometer to power down mode

}

float a = 0; 

void loop() {

  read_magnetometer(&Mag_Data.vector); // Read magnetometer data
  // scale(&Mag_Data.vector, Mag_Data.scale_factor); // Scale the magnetometer data

  delay(500); // Wait for 1 second
  //calibrate_magnetometer(&Mag_Data, &Mag_calibration_data);
  dataPrint(&Mag_Data.vector);


  // a = ((uint16_t)(0b10111110 << 8 | 0b11100111));
  // Serial.println((uint16_t)a, BIN);
}

