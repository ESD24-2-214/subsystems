#include <Arduino.h>
#include <Wire.h>
#include <MPU_I2C.hpp>
#include <MPU_ADDRESS.hpp>
#include <AK8963_ADDRESS.hpp>

/* @brief This function is used to enable or disable the bypass mode of the magnometer
** @param state: true to enable the bypass mode, false to disable the bypass mode
*/
void Magnometer_Bypass(bool state)
{
  set_bit_config(MPU9255_ADDRESS, Bypass_Enable_Config_add, state, 1);
}

/* @brief This function is used to configure the measure mode of the magnometer
** @param mode: true sets measure mode 2, false sets measure mode 1
** @note This function standart is 16bit resolution
*/
void measuremode_config_togle(bool mode){
  write(AK8963_ADDRESS, CNTL1, B00010110);
  mode ? write(AK8963_ADDRESS, CNTL1, B00010110) : write(AK8963_ADDRESS, ASTC, B00010010);
}

/* @brief This function is used to set the bit of a register to a specific value
** @param unit_addr: the I2C address of the unit
** @param local_addr: the address of the register
** @param state: the value to set the bit to
** @param bit_pos: the position of the bit to set
*/
void set_bit_config (uint8_t unit_addr, uint8_t local_addr, bool state, uint8_t bit_pos){
  Wire.beginTransmission(unit_addr);
    Wire.write(local_addr);
  Wire.endTransmission(false);
    Wire.requestFrom(local_addr, 1, false);
    uint8_t ConfigByte = Wire.read();
    ConfigByte = (state = true) ? (ConfigByte | (1 << bit_pos)) : (ConfigByte & ~(1 << bit_pos));
    Wire.write(ConfigByte);
  Wire.endTransmission(true);
}

/* @brief This function is used to read data from a register to an array
** @param unit_addr: the I2C address of the unit
** @param local_addr: the address of the register
** @param data: the array to store the data
** @param size: the size of the data to read in bytes aka the size of the array
*/
void read(uint8_t unit_addr, uint8_t local_addr, uint8_t data[], uint8_t size)
{
  Wire.beginTransmission(unit_addr);
  Wire.write(local_addr);
  Wire.endTransmission(false);
  Wire.requestFrom(unit_addr, size);
  for (uint8_t i = 0; i < size; i++)
  {
    data[i] = Wire.read();
  }
}

/* @brief This function is used to write data to a register
** @param unit_addr: the I2C address of the unit
** @param local_addr: the address of the register
** @param data_byte: the data to write to the register
*/
void write(uint8_t unit_addr, uint8_t local_addr, uint8_t data_byte)
{
  Wire.beginTransmission(unit_addr);
    Wire.write(local_addr);
  Wire.endTransmission(false);
    Wire.write(data_byte);
  Wire.endTransmission(true);
}

/* @brief This function is used to read the accelerometer data
** @param acc_data: Must be Vector struct
*/
void read_accelerometer(Vector *acc_data)
{
  uint8_t data[6];
  read(MPU9255_ADDRESS, ACCEL_XOUT_H, data, 6);

  acc_data -> x = (int16_t)(data[0] << 8 | data[1]);
  acc_data -> y = (int16_t)(data[2] << 8 | data[3]);
  acc_data -> z = (int16_t)(data[4] << 8 | data[5]);
  
}

/* @brief This function is used to read the accelerometer data
** @param acc_data: Must be Vector struct
*/
void read_gryroscope(Vector *gyro_data)
{
  uint8_t data[6];
  read(MPU9255_ADDRESS, GYRO_XOUT_H, data, 6);

  gyro_data -> x = (int16_t)(data[0] << 8 | data[1]);
  gyro_data -> y = (int16_t)(data[2] << 8 | data[3]);
  gyro_data -> z = (int16_t)(data[4] << 8 | data[5]);
}
/* @brief This function is used to read the magnetometer data
** @param mag_data: Must be Vector struct
*/
void read_magnetometer(Vector *mag_data)
{
  uint8_t data[6];
  read(AK8963_ADDRESS, MAGNO_XOUT_L, data, 6);

  mag_data -> x = (int16_t)(data[1] << 8 | data[0]);
  mag_data -> y = (int16_t)(data[3] << 8 | data[2]);
  mag_data -> z = (int16_t)(data[5] << 8 | data[4]);
}

/* @brief This function is used to scan the I2C bus and print the addresses of the devices found
*/
void I2Cbus_SCCAN(void){
  Serial.println(">>>Scanning I2C bus<<<");
  for (int i = 0; i < 128; i++)
  {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (i < 16)
      {
        Serial.print("0");
      }
      Serial.print(i, HEX);
      Serial.println(" !");
    }
  }
  Serial.println(">>>Scanning I2C bus complete<<<");
}

