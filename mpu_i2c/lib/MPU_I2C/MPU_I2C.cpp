#include <Arduino.h>
#include <Wire.h>
#include <MPU_I2C.hpp>
#include <MPU_ADDRESS.hpp>

/* @brief This function is used to enable or disable the bypass mode of the magnometer
** @param state: true to enable the bypass mode, false to disable the bypass mode
*/
void Magnometer_Bypass(bool state)
{
  set_bit_config(MPU9255_ADDRESS, Bypass_Enable_Config_add, state, 1);
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
  Wire.requestFrom(unit_addr, size, true);
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
** @param acc_data: the array to store the accelerometer data in must be of size 3x int16_t
*/
void read_accelerometer(int16_t acc_data[])
{
  if (sizeof(acc_data) != 6){return;}// Error hadling needed

  uint8_t data[6];
  read(MPU9255_ADDRESS, ACCEL_XOUT_H, data, 6);

  for (uint8_t i = 0; i < 3; i++){
    acc_data[i] = (int16_t)(data[2 * i] << 8 | data[2 * i + 1]);
  }
}

/* @brief This function is used to read the accelerometer data
** @param acc_data: the array to store the accelerometer data in must be of size 3x int16_t
*/
void read_gryroscope(int16_t gyro_data[])
{
  if (sizeof(gyro_data) != 6){return;}// Error hadling needed

  uint8_t data[6];
  read(MPU9255_ADDRESS, GYRO_XOUT_H, data, 6);

  for (uint8_t i = 0; i < 3; i++){
    gyro_data[i] = (int16_t)(data[2 * i] << 8 | data[2 * i + 1]);
  }
}
