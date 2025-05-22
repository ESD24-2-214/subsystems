#include <Arduino.h>
//#include <I2C.hpp>
#include <MPU_I2C.hpp>
#include <MPU_ADDRESS.hpp>
#include <AK8963_ADDRESS.hpp>
#include <DEBUG.hpp>
#include <Config.hpp>

MasterI2C MPU_I2C = MasterI2C(0); // I2C object

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
    ERROR_LOG("Unknown resolution, defaulting to 16 bit", " ");
    break;
  default:
    break;
  }
}

/* @brief This function is used to configure the measurement mode of the magnometer
** @param mode: the measurement mode to set
** @note param: MEAS_MODE1, MEAS_MODE2, SINGLE_MEASUREMENT, POWER_DOWN, EXTERNAL_TRIGGER, SELF_TEST
** @note This function is used to set bits 0, 1, 2 and 3 (0000xxxx) of the CNTL1 register in AK8963 
*/
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
  acc_data -> e1 = (int16_t)(data[0] << 8 | data[1]);
  acc_data -> e2 = (int16_t)(data[2] << 8 | data[3]);
  acc_data -> e3 = (int16_t)(data[4] << 8 | data[5]);
}

/* @brief This function is used to read the accelerometer data
** @param acc_data: Must be Vector struct
*/
void read_gryroscope(Vector *gyro_data)
{
  uint8_t data[6];
  read(MPU9255_ADDRESS, GYRO_XOUT_H, data, 6);
  gyro_data -> e1 = (int16_t)(data[0] << 8 | data[1]);
  gyro_data -> e2 = (int16_t)(data[2] << 8 | data[3]);
  gyro_data -> e3 = (int16_t)(data[4] << 8 | data[5]);
}

/* @brief This function is used to read the magnetometer data
** @param mag_data: Must be Vector struct
** @note If the magnetic sensor overflow the data is invalid and we keep the last data
*/
void read_magnetometer(Vector *mag_data)
{
  uint8_t stat = 0;
  uint8_t data[6] = {0};
  read(AK8963_ADDRESS, ST2, &stat, 1);   // Read ST2 register to clear the data
  vTaskDelay(pdMS_TO_TICKS(7.2)); // Wait for 7.5ms to make sure the data is ready
  do
  {
    read(AK8963_ADDRESS, ST1, &stat, 1); // Read ST1 to cheak bit 0 for the data bit ready
  } while (!(stat & 0b01)); // May be agresive use of CPU time but we use to cores Max wait 9ms typ 7.2ms

  read(AK8963_ADDRESS, MAGNO_XOUT_L, data, 6);
  read(AK8963_ADDRESS, ST2, &stat, 1); // Read ST2 register to clear the data ready bit
  if (stat & 0b00001000) // Check for magnetic sensor overflow i.e. the data is invalid
  {
    ERROR_LOG("Magnetic sensor overflow", " ");  
    return; // If invalid we keep the last data
  }
  mag_data -> e1 = ((int16_t)(data[1] << 8 | data[0])) - mag_hardiron_bias_e1;
  mag_data -> e2 = ((int16_t)(data[3] << 8 | data[2])) - mag_hardiron_bias_e2;
  mag_data -> e3 = ((int16_t)(data[5] << 8 | data[4])) - mag_hardiron_bias_e3;
}

/* @brief This function is used to read the data from the sensor and scale it
** @param Vec: the sensorvector to read data to
** @note remember ti initialize the sensorvector before using this function
** @note return: 0 if success, -1 if NULL pointer passed, 1 if sensor is unknown, 2 if scale factor is unknown
*/
int8_t read_data(SensorVector *Vec){
  if(Vec == NULL){
    ERROR_LOG("NULL pointer passed to read_data", " ");
    return -1;
  }
  if(Vec -> sensor == UNKNOWN){
    ERROR_LOG("Sensor is unknown", " ");
    return 1;
  }
  if(Vec -> scale_factor == UNKNOWN){
    ERROR_LOG("Scale factor is UNKNOWN", " ");
    return 2;
  }
  uint8_t data[6];
  switch (Vec -> sensor)
  {
  case MAGNOTOMETER:
    read_magnetometer_single16(&Vec -> vector);
    //scale(&Vec -> vector, Vec -> scale_factor);
    break;
  case ACCELEROMETER:
    read_accelerometer(&Vec -> vector);
    scale(&Vec -> vector, Vec -> scale_factor);
    break;
  case GYROSCOPE:
    read_gryroscope(&Vec -> vector);
    scale(&Vec -> vector, Vec -> scale_factor);
    break;
  default:
    break;
  }

  return 0;
}

/* @brief This function is used to scan the I2C bus and print the addresses of the devices found
*/
void I2Cbus_SCCAN(void){
  PRINT_DEBUG(">>>Scanning I2C bus<<<\n");
  for (int i = 0; i < 128; i++)
  {
    MPU_I2C.beginTransmission(i);
    if (MPU_I2C.transmitWrite() == 0)
    {
      PRINT_DEBUG("I2C device found at address 0x");
      if (i < 16)
      {
        PRINT_DEBUG("0");
      }
      PRINT_DEBUG_NR(i, HEX);
      PRINT_DEBUG(" !\n");
    }
    MPU_I2C.endTransmission();
  }
  PRINT_DEBUG(">>>Scanning I2C bus complete<<<\n");
}

/* @brief This function is used to scan the I2C bus fir MPU units and print the addresses of the devices found
** @returns Number of units that didn't return an acknowledge bit. 
** If all were found retuns 0.
** @note Unit addesses is defined localy in an array hard coded in the function
*/
uint8_t MPU_I2Cbus_SCCAN(void){
  uint8_t no_ack = 0; // "No acknowledge bit" - counter
  uint8_t units = 2; // Number of units
  uint8_t unit_add_arr[units] = {MPU9255_ADDRESS, AK8963_ADDRESS}; // Unit addesses
  PRINT_DEBUG(">>>Scanning for MPU units<<<\n");
  for (int i = 0; i < units; i++)
  {
    MPU_I2C.beginTransmission(unit_add_arr[i]);
    if (MPU_I2C.transmitWrite() == 0)
    {
      PRINT_DEBUG("I2C device found at address 0x");

      if (i < 16) {PRINT_DEBUG("0");}
      PRINT_DEBUG_NR(unit_add_arr[i], HEX);
      PRINT_DEBUG(" !\n");
    } else { no_ack++; }
    MPU_I2C.endTransmission();
  }
  PRINT_DEBUG(">>>Scanning I2C bus complete<<<\n");
  return no_ack;
}

/* @brief This function is used to make a self-test on the AK8963
*/
void magnotometer_selftest(void){
SensorVector mag_data;
mag_meas_config(POWER_DOWN);
set_bit_config(AK8963_ADDRESS, ASTC, true, 6); // Soft reset the AK8963
mag_meas_config(SELF_TEST);
delay(1000);
read_data(&mag_data);
PRINT_DEBUG("mag: \n");
dataPrint(&mag_data.vector);
set_bit_config(AK8963_ADDRESS, ASTC, false, 6);
mag_meas_config(POWER_DOWN);
}

/* @brief This function is used to soft reset the AK8963
*/
void magnotometer_softreset(void){
  set_bit_config(AK8963_ADDRESS, CNTL2, true, 0); // Soft reset the AK8963
}

/*@brief This function is used to scale the vector by a factor
* @param Vec: the vector to scale
* @param scale_factor: the factor to scale the vector by
*/
void scale(Vector *Vec, double scale_factor){
  Vec -> e1 = ((Vec -> e1) * scale_factor);
  Vec -> e2 = ((Vec -> e2) * scale_factor);
  Vec -> e3 = ((Vec -> e3) * scale_factor);
}

/*
* @brief This function is used to scale the vector by a factor
* @param Vec: the vector to scale
* @note return: 0 if success, -1 if NULL pointer passed, 1 if sensor is unknown, 2 if scale factor is unknown 
*/
int8_t factorScale(SensorVector *Vec){
  if(Vec == NULL){
    ERROR_LOG("NULL pointer passed to factorScale", " ");
    return -1;
  }
  if(Vec -> sensor == UNKNOWN){
    ERROR_LOG("Sensor is unknown", " ");
    return 1;
  }
  if(Vec -> scale_factor == UNKNOWN){
    ERROR_LOG("Scale factor is UNKNOWN", " ");
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

/* @brief This function is used to initialize the sensor vector
** @param Vec: the vector to initialize
** @param sensor_t: the type of sensor to initialize
** @param scale_factor: the scale factor to set
** @note return: 0 if success, -1 if NULL pointer passed, 1 if sensor is unknown, 2 if scale factor is unknown 
*/
int8_t initSensorVector(SensorVector *Vec, Sensor sensor_t, double scale_factor){
  if(Vec == NULL){
    ERROR_LOG("NULL pointer passed to initSensorVector", " ");
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
    ERROR_LOG("Sensor is unknown", " ");
    return 1;
  }
  if(scale_factor == UNKNOWN){
    ERROR_LOG("Scale factor is UNKNOWN", " ");
    return 2;
  }

  Vec -> vector.e1 = 0;
  Vec -> vector.e2 = 0;
  Vec -> vector.e3 = 0;
  Vec -> scale_factor = scale_factor;
  Vec -> sensor = sensor_t;
  return 0;
}

/* @brief This function is used to set the 2 bits of a register to a specific value
** @param unit_addr: the I2C address of the unit
** @param local_addr: the address of the register
** @param bit2: the value to set the bits to
** @param bitpos: the position of the bits to set. If bitpos = 0, => (000000xx)
*/
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

/* @brief This function is used to set the full scale range of the gyroscope
** @param range: the full scale range to set
** @note GFS_250: 250 degrees/sec, GFS_500: 500 degrees/sec, GFS_1000: 1000 degrees/sec, GFS_2000: 2000 degrees/sec
** @note This function is used to set bits 3 and 4 of the GYRO_CONFIG register (000xx000)
*/
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
    ERROR_LOG("Unknown range", " ");
    return;
  }
  set_2bit(MPU9255_ADDRESS, GYRO_CONFIG, bit2, 3);
}

/* @brief This function is used to set the full scale range of the accelerometer
** @param range: the full scale range to set
** @note AFS_2G: 2g, AFS_4G: 4g, AFS_8G: 8g, AFS_16G: 16g
** @note This function is used to set bits 3 and 4 of the ACCEL_CONFIG register (000xx000)
*/
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
    ERROR_LOG("Unknown range", " ");
    return;
  }
  set_2bit(MPU9255_ADDRESS, ACCEL_CONFIG, bit2, 3);
}

/* @brief This function is used to print the data of the vector
** @param Vec: the vector to print
*/
void dataPrint(Vector *Vec){
  PRINT_DEBUG("e1, e2, e3,\n");
  PRINT_DEBUG((Vec -> e1));
  PRINT_DEBUG(", ");
  PRINT_DEBUG((Vec -> e2));
  PRINT_DEBUG(", ");
  PRINT_DEBUG((Vec -> e3));
  PRINT_DEBUG("\n");
}

/* @brief This function is used to print the data of important config registers of the AK8963
** @note This function is used for debugging purposes only should be removed in final code
** @note registers are CNTL1, CNTL2, ASTC, ST1, ST2
*/ 
void mag_debug_print(void){
  uint8_t data_byte = 0;
  read(AK8963_ADDRESS, CNTL1, &data_byte, 1);
  PRINT_DEBUG("CNTL1 : ");
  PRINT_DEBUG((data_byte, BIN));
  read(AK8963_ADDRESS, CNTL2, &data_byte, 1);
  PRINT_DEBUG("\nCNTL2 : ");
  PRINT_DEBUG((data_byte, BIN));
  read(AK8963_ADDRESS, ASTC, &data_byte, 1);
  PRINT_DEBUG("\nASTC : ");
  PRINT_DEBUG((data_byte, BIN));
  read(AK8963_ADDRESS, ST1, &data_byte, 1);
  PRINT_DEBUG("\nST1 : ");
  PRINT_DEBUG((data_byte, BIN));
  read(AK8963_ADDRESS, ST2, &data_byte, 1);
  PRINT_DEBUG("\nST2 : ");
  PRINT_DEBUG((data_byte, BIN));
  PRINT_DEBUG("\n");
}

/* @brief This function is used to print the data of important config registers of the MPU9255
** @note This function is used for debugging purposes only should be removed in final code
** @note registers are PWR_MGMT_1, PWR_MGMT_2, CONFIG, ACCEL_CONFIG, GYRO_CONFIG
*/
void mpu_debug_print(void){
  uint8_t data_byte = 0;
  read(MPU9255_ADDRESS, PWR_MGMT_1, &data_byte, 1);
  PRINT_DEBUG("PWR_MGMT_1 : ");
  PRINT_DEBUG((data_byte, BIN));
  read(MPU9255_ADDRESS, PWR_MGMT_2, &data_byte, 1);
  PRINT_DEBUG("\nPWR_MGMT_2 : ");
  PRINT_DEBUG((data_byte, BIN));
  read(MPU9255_ADDRESS, MPU_CONFIG, &data_byte, 1);
  PRINT_DEBUG("\nCONFIG : ");
  PRINT_DEBUG((data_byte, BIN));
  read(MPU9255_ADDRESS, ACCEL_CONFIG, &data_byte, 1);
  PRINT_DEBUG("\nACCEL_CONFIG : ");
  PRINT_DEBUG((data_byte, BIN));
  read(MPU9255_ADDRESS, GYRO_CONFIG, &data_byte, 1);
  PRINT_DEBUG("\nGYRO_CONFIG : ");
  PRINT_DEBUG((data_byte, BIN));
  PRINT_DEBUG("\n");
}

/* @brief This function is used to read the magnetometer data in single measurement mode
** @param mag_data: Must be Vector struct
** @note If the magnetic sensor overflow the data is invalid and we keep the last data
*/
void read_magnetometer_single16(Vector *mag_data)
{
  uint8_t stat = 0;
  uint8_t data[6] = {0};
  read(AK8963_ADDRESS, ST2, &stat, 1);   // Read ST2 register to clear the data
  write(AK8963_ADDRESS, CNTL1, 0b00010001); // Set to single measurement mode 16 bit
  vTaskDelay(pdMS_TO_TICKS(7.2)); // Wait for 7.2ms to make sure the data is ready
  do
  {
    read(AK8963_ADDRESS, ST1, &stat, 1); // Read ST1 to cheak bit 0 for the data bit ready
  } while (!(stat & 0b01)); // May be agresive use of CPU time but we use two cores Max wait 9ms typ 7.2ms

  read(AK8963_ADDRESS, MAGNO_XOUT_L, data, 6);
  read(AK8963_ADDRESS, ST2, &stat, 1); // Read ST2 register to clear the data ready bit
  if (stat & 0b00001000) // Check for magnetic sensor overflow i.e. the data is invalid
  {
    ERROR_LOG("Magnetic sensor overflow", " ");  
    return; // If invalid we keep the last data
  }
  mag_data -> e1 = (float)((int16_t)data[1] << 8 | (int16_t)data[0]);
  mag_data -> e2 = (float)((int16_t)data[3] << 8 | (int16_t)data[2]);
  mag_data -> e3 = (float)((int16_t)data[5] << 8 | (int16_t)data[4]);
}