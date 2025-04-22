#include <Arduino.h>
#include <I2C.hpp>
#include <MPU_I2C.hpp>
#define ClockSpeed 400000 // 400kHz

#define SDA 18 // Signal Data Line pin
#define SCL 17 // Signal Clock Line pin
void debug_print(void);


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
resolution_config(FULL);
}

Vector gyro_data;
Vector acc_data;
Vector mag_data;
uint8_t data[6];
uint8_t data_byte = 0;

void loop() {  // testing the magnometer
  debug_print();
  read(MPU9255_ADDRESS, ACCEL_XOUT_H, data, 6);
  Serial.print("ACCEL : ");
  for (int i = 0; i < 6; i++){
    Serial.print(data[i]); Serial.print(" ");
  }
  Serial.println();

  delay(500);




}





void debug_print(void){
  read(AK8963_ADDRESS, CNTL1, &data_byte, 1);
  Serial.print("CNTL1 : ");
  Serial.println(data_byte, BIN);
  read(AK8963_ADDRESS, CNTL2, &data_byte, 1);
  Serial.print("CNTL2 : ");
  Serial.println(data_byte, BIN);
  read(AK8963_ADDRESS, ASTC, &data_byte, 1);
  Serial.print("ASTC : ");
  Serial.println(data_byte, BIN);
}


