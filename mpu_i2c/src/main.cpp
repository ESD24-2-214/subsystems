#include <Arduino.h>
#include <I2C.hpp>
#include <MPU_I2C.hpp>
#define ClockSpeed 400000 // 400kHz

#define SDA 18 // Signal Data Line pin
#define SCL 17 // Signal Clock Line pin
void mag_debug_print(void);
void dataPrint(Vector *Vec);
const uint8_t mag_res = BIT_16; // mag resolution
const uint8_t mag_mode = MEAS_MODE2; // mag mode
const uint8_t gyro_fs = GFS_250; // gyro full scale range
const uint8_t accel_fs = AFS_4G; // accel full scale range
const double gyro_scale = gyro_scale_factor250; // gyro scale factor
const double accel_scale = acc_scale_factor4g; // accel scale factor

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
}

Vector gyro_data;
Vector acc_data;
Vector mag_data;
uint8_t data[6];
uint8_t data_byte = 0;

void loop() { 
  mag_debug_print();
  read_magnetometer(&mag_data);
  read_accelerometer(&acc_data);
  read_gryroscope(&gyro_data);



  Serial.println("mag: ");
  dataPrint(&mag_data);
  Serial.println("acc: ");
  dataPrint(&acc_data);
  Serial.println("gyro: ");
  dataPrint(&gyro_data);
  
  delay(500);




}


void dataPrint(Vector *Vec){

  Serial.print("x: ");
  Serial.println((Vec -> x));
  Serial.print("y: ");
  Serial.println((Vec -> y));
  Serial.print("z: ");
  Serial.println((Vec -> z));
}


void mag_debug_print(void){
  read(AK8963_ADDRESS, CNTL1, &data_byte, 1);
  Serial.print("CNTL1 : ");
  Serial.println(data_byte, BIN);
  read(AK8963_ADDRESS, CNTL2, &data_byte, 1);
  Serial.print("CNTL2 : ");
  Serial.println(data_byte, BIN);
  read(AK8963_ADDRESS, ASTC, &data_byte, 1);
  Serial.print("ASTC : ");
  Serial.println(data_byte, BIN);
  read(AK8963_ADDRESS, ST1, &data_byte, 1);
  Serial.print("ST1 : ");
  Serial.println(data_byte, BIN);
  read(AK8963_ADDRESS, ST2, &data_byte, 1);
  Serial.print("ST2 : ");
  Serial.println(data_byte, BIN);
}


