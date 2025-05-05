#include <Arduino.h>
#include <MPU_I2C.hpp>
#include <Config.hpp>
#include <AK8963_ADDRESS.hpp>
#include <MPU_ADDRESS.hpp>
#define ClockSpeed 400000 // 400kHz

// #define SDA 18 // Signal Data Line pin
// #define SCL 17 // Signal Clock Line pin

#define SDA 8 // Signal Data Line pin
#define SCL 9 // Signal Clock Line pin

const mag_resolution mag_res = BIT_16; // mag resolution
const mag_meas_mode mag_mode = MEAS_MODE1; // mag mode
const gyro_full_scale_range gyro_fs = GFS_500; // gyro full scale range
const acc_full_scale_range accel_fs = AFS_2G; // accel full scale range
const double gyro_scale = gyro_scale_factor500; // gyro scale factor

const double accel_scale = acc_scale_factor2g; // accel scale factor
const double mag_scale = mag_scale_factor2; // mag scale factor
SensorVector gyro_data;
SensorVector acc_data;
SensorVector mag_data;

void read_sens_vector(SensorVector *Vec);

void setup() {
Serial.begin(115200);
while (!Serial){};
delay(5000); Serial.println("YO!!");
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
  
  Serial.println("mag: ");
  dataPrint(&mag_data.vector);
  Serial.println("acc: ");
  dataPrint(&acc_data.vector);
  Serial.println("gyro: ");
  dataPrint(&gyro_data.vector);
  
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
  Serial.print(Vec->vector.x);
  Serial.print(", ");
  Serial.print(Vec->vector.y);
  Serial.print(", ");
  Serial.print(Vec->vector.z);
  Serial.print(", ");
  Serial.println(Vec->scale_factor, 6);
}

