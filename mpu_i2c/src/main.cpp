#include <Arduino.h>
#include <MPU_I2C.hpp>
#include <Config.hpp>
#define ClockSpeed 400000 // 400kHz

#define SDA 18 // Signal Data Line pin
#define SCL 17 // Signal Clock Line pin

const uint8_t mag_res = BIT_16; // mag resolution
const uint8_t mag_mode = MEAS_MODE2; // mag mode
const uint8_t gyro_fs = GFS_250; // gyro full scale range
const uint8_t accel_fs = AFS_4G; // accel full scale range
const double gyro_scale = gyro_scale_factor250; // gyro scale factor
const double accel_scale = acc_scale_factor4g; // accel scale factor
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
  Serial.println("HERE!!");
}

delay(2000);
bypass_to_magnometer(true);
I2Cbus_SCCAN();
delay(2000);
mag_resolution_config(BIT_16);
mag_meas_config(MEAS_MODE2);
gyro_fs_sel(GFS_250);
accel_fs_sel(AFS_4G);


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
  
  mpu_debug_print();
  mag_debug_print();
  //read_magnetometer(&mag_data.vector);
  //read_accelerometer(&acc_data.vector);
  //read_gryroscope(&gyro_data.vector);

  Serial.println("mag: ");
  dataPrint(&mag_data.vector);
  Serial.println("acc: ");
  dataPrint(&acc_data.vector);
  Serial.println("gyro: ");
  dataPrint(&gyro_data.vector);
  
  delay(500);

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