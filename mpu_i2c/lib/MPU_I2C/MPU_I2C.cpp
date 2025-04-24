#include <Arduino.h>
#include <I2C.hpp>
#include <MPU_I2C.hpp>
#include <MPU_ADDRESS.hpp>
#include <AK8963_ADDRESS.hpp>


MasterI2C MPU_I2C = MasterI2C(0);

/* @brief This function is used to hard reset the MPU9255
** @note Resets the internal registers and restores the default settings.
*/
void hard_reset_MPU9255(void)
{
  set_bit_config(MPU9255_ADDRESS, PWR_MGMT_1, true, 7);
}

/* @brief This function is used to enable or disable the bypass mode of the magnometer
** @param state: true to enable the bypass mode, false to disable the bypass mode
*/
void bypass_to_magnometer(bool state)
{
  set_bit_config(MPU9255_ADDRESS, Bypass_Enable_Config_add, state, 1);
}

/* @brief This function is used to configure the resolution of the magnometer
** @param mode: BIT_14 for 14 bit resolution, BIT_16 for 16 bit resolution
** @note This function standart is 16bit resolution if the mode is unknown
*/
void mag_resolution_config(mag_resolution mode){
  switch (mode)
  {
  case BIT_14:
    set_bit_config(AK8963_ADDRESS, CNTL1, false, 4); // 14 bit resolution
    break;
  case BIT_16:
    set_bit_config(AK8963_ADDRESS, CNTL1, true, 4); // 16 bit resolution
    break;
  case UNKNOWN:
  set_bit_config(AK8963_ADDRESS, CNTL1, true, 4); // 16 bit resolution
    DEBUG("Unknown resolution, defaulting to 16 bit", " ");
    break;
  default:
    break;
  }

  //mode ? write(AK8963_ADDRESS, CNTL1, B00010110) : write(AK8963_ADDRESS, ASTC, B00010010);
}

uint8_t mag_meas_config(mag_meas_mode mode){
  uint8_t bits = 0b0000;
  switch (mode)
  {
  case MEAS_MODE1:
    bits = 0b0010;
    break;
  case MEAS_MODE2:
    bits = 0b0110;
    break;
  case SINGLE_MEASUREMENT:
    bits = 0b0001;
    break;
  case POWER_DOWN:
    break;
  case EXTERNAL_TRIGGER:
    bits = 0b0100;
    break;
  case SELF_TEST:
    bits = 0b1000;
    break;
  default:
    return 1;
  }    
  MPU_I2C.beginTransmission(AK8963_ADDRESS);
    MPU_I2C.write(CNTL1);
    MPU_I2C.requestData((size_t)BYTE, true);
    uint8_t ConfigByte = MPU_I2C.read();
    ConfigByte = (ConfigByte & (0b11110000));
    ConfigByte = (ConfigByte | (bits));
    MPU_I2C.write(CNTL1);
    MPU_I2C.write(ConfigByte);
    MPU_I2C.transmitWrite();
  MPU_I2C.endTransmission();
  return 0;
}


/* @brief This function is used to set the bit of a register to a specific value
** @param unit_addr: the I2C address of the unit
** @param local_addr: the address of the register
** @param state: the value to set the bit to
** @param bit_pos: the position of the bit to set
*/
void set_bit_config (uint8_t unit_addr, uint8_t local_addr, bool state, uint8_t bit_pos){
  MPU_I2C.beginTransmission(unit_addr);
    MPU_I2C.write((uint8_t)local_addr);
    MPU_I2C.requestData((size_t)BYTE, true);
    uint8_t ConfigByte = MPU_I2C.read();
    ConfigByte = (state = true) ? (ConfigByte | (1 << bit_pos)) : (ConfigByte & ~(1 << bit_pos));
    MPU_I2C.write(local_addr);
    MPU_I2C.write(ConfigByte);
    MPU_I2C.transmitWrite();
  MPU_I2C.endTransmission();
}

/* @brief This function is used to read data from a register to an array
** @param unit_addr: the I2C address of the unit
** @param local_addr: the address of the register
** @param data: the array to store the data
** @param size: the size of the data to read in bytes aka the size of the array
*/
void read(uint8_t unit_addr, uint8_t local_addr, uint8_t data[], uint8_t size)
{
  MPU_I2C.beginTransmission(unit_addr);
    MPU_I2C.write((uint8_t)local_addr);
    MPU_I2C.requestData(size, true);
  for (uint8_t i = 0; i < size; i++)
  {
    data[i] = MPU_I2C.read();
  }
  MPU_I2C.endTransmission();
}

/* @brief This function is used to write data to a register
** @param unit_addr: the I2C address of the unit
** @param local_addr: the address of the register
** @param data_byte: the data to write to the register
*/
void write(uint8_t unit_addr, uint8_t local_addr, uint8_t data_byte)
{
  MPU_I2C.beginTransmission(unit_addr);
    MPU_I2C.write(local_addr);
    MPU_I2C.write(data_byte);
    MPU_I2C.transmitWrite();
  MPU_I2C.endTransmission();
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

int8_t read_data(SensorVector *Vec){
  if(Vec == NULL){
    DEBUG("NULL pointer passed to read_data", " ");
    return -1;
  }
  if(Vec -> sensor == UNKNOWN){
    DEBUG("Sensor is unknown", " ");
    return 1;
  }
  if(Vec -> scale_factor == UNKNOWN){
    DEBUG("Scale factor is UNKNOWN", " ");
    return 2;
  }
  uint8_t data[6];
  switch (Vec -> sensor)
  {
  case MAGNOTOMETER:
    read_magnetometer(&Vec -> vector);
    break;
  case ACCELEROMETER:
    read_accelerometer(&Vec -> vector);
    break;
  case GYROSCOPE:
    read_gryroscope(&Vec -> vector);
    break;
  default:
    break;
  }

  scale(&Vec -> vector, Vec -> scale_factor);

  return 0;
}

/* @brief This function is used to scan the I2C bus and print the addresses of the devices found
*/
void I2Cbus_SCCAN(void){
  Serial.println(">>>Scanning I2C bus<<<");
  for (int i = 0; i < 128; i++)
  {
    MPU_I2C.beginTransmission(i);
    if (MPU_I2C.transmitWrite() == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (i < 16)
      {
        Serial.print("0");
      }
      Serial.print(i, HEX);
      Serial.println(" !");
    }
    MPU_I2C.endTransmission();
  }
  Serial.println(">>>Scanning I2C bus complete<<<");
}
/* @brief This function is used to set the AK8963 to self-test mode
*/
void magnotometer_selftest(void){
write(AK8963_ADDRESS, CNTL1, B00011000);
}

/* @brief This function is used to soft reset the AK8963
*/
void magnotometer_softreset(void){
  set_bit_config(AK8963_ADDRESS, CNTL2, true, 0); // Soft reset the AK8963
}

/*
* @brief This function is used to scale the vector by a factor
* @param Vec: the vector to scale
* @param scale_factor: the factor to scale the vector by
*/
void scale(Vector *Vec, double scale_factor){
  Vec -> x = Vec -> x * scale_factor;
  Vec -> y = Vec -> y * scale_factor;
  Vec -> z = Vec -> z * scale_factor;
}

/*
* @brief This function is used to scale the vector by a factor
* @param Vec: the vector to scale
* @note return: 0 if success, -1 if NULL pointer passed, 1 if sensor is unknown, 2 if scale factor is unknown 
*/
int8_t factorScale(SensorVector *Vec){
  if(Vec == NULL){
    DEBUG("NULL pointer passed to factorScale", " ");
    return -1;
  }
  if(Vec -> sensor == UNKNOWN){
    DEBUG("Sensor is unknown", " ");
    return 1;
  }
  if(Vec -> scale_factor == UNKNOWN){
    DEBUG("Scale factor is UNKNOWN", " ");
    return 2;
  }

  switch (Vec -> sensor)
  {
  case MAGNOTOMETER:
    scale(&Vec -> vector, Vec -> scale_factor);
    break;
  case ACCELEROMETER:
    scale(&Vec -> vector, Vec -> scale_factor);
    break;
  case GYROSCOPE:
    scale(&Vec -> vector, Vec -> scale_factor);
    break;
  default:
    break;
  }
  return 0;
}

int8_t initSensorVector(SensorVector *Vec, Sensor sensor_t, double scale_factor){
  if(Vec == NULL){
    DEBUG("NULL pointer passed to initSensorVector", " ");
    return -1;
  }
  switch (sensor_t)
  {
  case MAGNOTOMETER:
    Vec -> unit_addr = AK8963_ADDRESS;
    Vec -> local_addr = MAGNO_XOUT_L;
    break;
  case ACCELEROMETER:
    Vec -> unit_addr = MPU9255_ADDRESS;
    Vec -> local_addr = ACCEL_XOUT_H;
    break;
  case GYROSCOPE:
    Vec -> unit_addr = MPU9255_ADDRESS;
    Vec -> local_addr = GYRO_XOUT_H;
    break;
  default:
    DEBUG("Sensor is unknown", " ");
    return 1;
  }
  if(scale_factor == UNKNOWN){
    DEBUG("Scale factor is UNKNOWN", " ");
    return 2;
  }

  Vec -> vector.x = 0;
  Vec -> vector.y = 0;
  Vec -> vector.z = 0;
  Vec -> scale_factor = scale_factor;
  Vec -> sensor = sensor_t;
  return 0;
}

void set_2bit(uint8_t unit_addr, uint8_t local_addr, uint8_t bit2, uint8_t bitpos){
  MPU_I2C.beginTransmission(unit_addr);
  MPU_I2C.write(local_addr);
  MPU_I2C.requestData((size_t)BYTE, true);
  uint8_t ConfigByte = MPU_I2C.read();
  ConfigByte = (ConfigByte & ~(0b11 << bitpos));
  ConfigByte = (ConfigByte | (bit2 << bitpos));
  MPU_I2C.write(local_addr);
  MPU_I2C.write(ConfigByte);
  MPU_I2C.transmitWrite();
MPU_I2C.endTransmission();
}

void gyro_fs_sel(gyro_full_scale_range range){
  uint8_t bit2 = 0b00;
  switch (range)
  {
  case GFS_250:
    break;
  case GFS_500:
    bit2 = 0b01;
    break;
  case GFS_1000:
    bit2 = 0b10;
    break;
  case GFS_2000:
    bit2 = 0b11;
    break;
  default:
    DEBUG("Unknown range", " ");
    return;
  }
  set_2bit(MPU9255_ADDRESS, GYRO_CONFIG, bit2, 3);
}

void accel_fs_sel(acc_full_scale_range range){
  uint8_t bit2 = 0b00;
  switch (range)
  {
  case AFS_2G:
    break;
  case AFS_4G:
    bit2 = 0b01;
    break;
  case AFS_8G:
    bit2 = 0b10;
    break;
  case AFS_16G:
    bit2 = 0b11;
    break;
  default:
    DEBUG("Unknown range", " ");
    return;
  }
  set_2bit(MPU9255_ADDRESS, ACCEL_CONFIG, bit2, 3);
}


