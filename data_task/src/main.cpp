#include <Arduino.h>
#include <MPU_I2C.hpp>
#include <Config.hpp>
#include <AK8963_ADDRESS.hpp>
#include <MPU_ADDRESS.hpp>
#include <LDR.hpp>



const mag_resolution mag_res = BIT_16; // mag resolution
const mag_meas_mode mag_mode = MEAS_MODE1; // mag mode
const gyro_full_scale_range gyro_fs = GFS_500; // gyro full scale range
const acc_full_scale_range accel_fs = AFS_2G; // accel full scale range
const double gyro_scale = gyro_scale_factor500; // gyro scale factor
const double accel_scale = acc_scale_factor2g; // accel scale factor
const double mag_scale = mag_scale_factor2; // mag scale factor
SensorVector gyro_data = {unknown, 0, 0, 0, 0};
SensorVector acc_data = {unknown, 0, 0, 0, 0}; 
SensorVector mag_data = {unknown, 0, 0, 0, 0};
LDRData_t ldr_data = {0,0,0,0}; // LDR data structure

void read_sens_vector(SensorVector *Vec);

void setup() {
Serial.begin(115200);
while (!Serial){};
delay(5000);
if(MPU_I2C.begin(SDA, SCL, ClockSpeed) == false){
  Serial.println("I2C begin Failed!");
}

bypass_to_magnometer(true);
I2Cbus_SCCAN();
delay(2000);

mag_resolution_config(mag_res);
mag_meas_config(mag_mode);
gyro_fs_sel(gyro_fs);
accel_fs_sel(accel_fs);
write(AK8963_ADDRESS, ASTC , 0x00); // Set clock source to PLL with X axis gyroscope reference

initSensorVector(&gyro_data, GYROSCOPE, gyro_scale);
initSensorVector(&acc_data, ACCELEROMETER, accel_scale);
initSensorVector(&mag_data, MAGNOTOMETER, mag_scale);
read_sens_vector(&gyro_data);
read_sens_vector(&acc_data);
read_sens_vector(&mag_data);
delay(5000);
}


void loop() {
  read_data(&gyro_data);
  read_data(&acc_data);
  read_data(&mag_data);
  ldr_read_data(&ldr_data, 100, 10);

  Serial.println("mag: ");
  dataPrint(&mag_data.vector);
  Serial.println("acc: ");
  dataPrint(&acc_data.vector);
  Serial.println("gyro: ");
  dataPrint(&gyro_data.vector);
  Serial.println("LDR: ");
  ldr_data_print(&ldr_data);
  //mpu_debug_print();
  //mag_debug_print();

  delay(1000);
}

void read_sens_vector(SensorVector *Vec){
  Serial.print(Vec->sensor);
  Serial.print(", ");
  Serial.print(Vec->unit_addr, HEX);
  Serial.print(", ");
  Serial.print(Vec->local_addr, HEX);
  Serial.print(", ");
  Serial.print(Vec->vector.e1);
  Serial.print(", ");
  Serial.print(Vec->vector.e2);
  Serial.print(", ");
  Serial.print(Vec->vector.e3);
  Serial.print(", ");
  Serial.println(Vec->scale_factor, 6);
}

