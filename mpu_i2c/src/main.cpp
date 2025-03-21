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
set_bit_config(MPU9255_ADDRESS, PWR_MGMT_1, true, 7);
Magnometer_Bypass(true);
delay(2000);
I2Cbus_SCCAN();
measuremode_config_togle(true);
}

Vector gyro_data;
Vector acc_data;
Vector mag_data;
uint8_t data[6];
uint8_t data_byte;

void loop() {  // testing the magnometer
  read(AK8963_ADDRESS, CNTL1, &data_byte, 1);
  Serial.print("CNTL1 : ");
  Serial.println(data_byte, BIN);
  read(AK8963_ADDRESS, ASTC, &data_byte, 1);
  Serial.print("ASTC : ");
  Serial.println(data_byte, BIN);

  read(AK8963_ADDRESS, MAGNO_XOUT_L, data, 6);
  for (uint8_t i = 0; i < 6; i++)
  {
    Serial.print("Data ");
    Serial.print(i);
    Serial.print(" : ");
    Serial.println(data[i], BIN);
  }
  Serial.println();
  
  read_magnetometer(&mag_data);
  Serial.print("Magnetometer X : ");
  Serial.println(mag_data.x);
  Serial.print("Magnetometer Y : ");
  Serial.println(mag_data.y);
  Serial.print("Magnetometer Z : ");
  Serial.println(mag_data.z);
  Serial.println();
  delay(1000);
}

