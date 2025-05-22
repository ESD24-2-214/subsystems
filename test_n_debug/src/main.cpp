#include "Arduino.h"
#include "loop.hpp"
#include "math.h"
#include "math_stuff.hpp"
#include <AK8963_ADDRESS.hpp>
#include <Config.hpp>
#include <Hbro.hpp>
#include <LDR.hpp>
#include <MPU_ADDRESS.hpp>
#include <MPU_I2C.hpp>
#include <cstdint>




void SensorRead(void *par);
struct SensorData {
  uint32_t time_stamp_msec;
  Vector magno_sat;
  Vector sun_sat;
  Vector gyro_sat;
  Vector accl_sat;
};
struct MagnetorquerScalarData {
  uint32_t time_stamp_msec;
  float scalar_for_e12;
  float scalar_for_e31;
  float scalar_for_e23;
};
// Queues
QueueHandle_t xQueueSensorData = NULL;
QueueHandle_t xQueueMagnetorquerScalarData = NULL;
// Task handles
TaskHandle_t *SensorRead_th;

void setup() {
  Serial.begin(115200); // Initialize serial monitor
  while (!Serial) {}
  delay(2000); // Wait for serial monitor to open

  


  // Queues
  xQueueSensorData = xQueueCreate(1, sizeof(SensorData));
  xQueueMagnetorquerScalarData =
      xQueueCreate(1, sizeof(MagnetorquerScalarData));
  if (xQueueSensorData == NULL) {/* Queue was not created and must not be used. */}

  // Create tasks
  xTaskCreatePinnedToCore(SensorRead,   // Function to call
                          "SensorRead", // Name of the task
                          1024,         // Stack size in bytes
                          NULL,         // Task input parameter
                          1,    // Task priority (0 to configMAX_PRIORITIES - 1)
                          SensorRead_th, // Task handle
                          pro_cpu); // Create SendToQueue
}

void SensorRead(void *par) {
  // xSemaphoreTake(binarykey1, portMAX_DELAY);
  if (MPU_I2C.begin(SDA, SCL, ClockSpeed) == false) {
    Serial.println("I2C begin Failed!");
  }
  // Define param of task
  const mag_resolution mag_res = BIT_16;          // mag resolution
  const mag_meas_mode mag_mode = MEAS_MODE1;      // mag mode
  const gyro_full_scale_range gyro_fs = GFS_500;  // gyro full scale range
  const acc_full_scale_range accel_fs = AFS_2G;   // accel full scale range
  const double gyro_scale = gyro_scale_factor500; // gyro scale factor
  const double accel_scale = acc_scale_factor2g;  // accel scale factor
  const double mag_scale = mag_scale_factor2;     // mag scale factor
  SensorVector gyro_data = {unknown, 0, 0, 0, 0};
  SensorVector accl_data = {unknown, 0, 0, 0, 0};
  SensorVector mag_data = {unknown, 0, 0, 0, 0};
  Vector sun_data = {0, 0, 0}; // LDR data structure
  // Config thing
  bypass_to_magnometer(true);
  if (MPU_I2Cbus_SCCAN()) {
    // ERROR
  }

  mag_resolution_config(mag_res);
  mag_meas_config(mag_mode);
  gyro_fs_sel(gyro_fs);
  accel_fs_sel(accel_fs);
  write(AK8963_ADDRESS, ASTC, 0x00); // Making sure that selftest mode i off

  // Init of sensorvectors
  initSensorVector(&gyro_data, GYROSCOPE, gyro_scale);
  initSensorVector(&accl_data, ACCELEROMETER, accel_scale);
  initSensorVector(&mag_data, MAGNOTOMETER, mag_scale);
  SensorData data = {.time_stamp_msec = (xTaskGetTickCount()),
                     .magno_sat = mag_data.vector,
                     .sun_sat = sun_data,
                     .gyro_sat = gyro_data.vector,
                     .accl_sat = accl_data.vector};
  
  // Task loop
  while (1) {
    Serial.print("Start time: ");
    Serial.println(pdTICKS_TO_MS(xTaskGetTickCount()));
    read_data(&gyro_data);
    read_data(&accl_data);
    
    // xSemaphoreTake(binarykey3, portMAX_DELAY);
    //   xSemaphoreGive(binarykey1); // Signal to ActuatorControl to stop magnotorqer
    //     xSemaphoreTake(binarykey2, portMAX_DELAY); // wait for signal

        read_data(&mag_data); // measure magneticfield

    //     xSemaphoreGive(binarykey2); // return key
    //   xSemaphoreTake(binarykey1, portMAX_DELAY); // Retrive key
    // xSemaphoreGive(binarykey3);
   

    sun_read_data(&sun_data, LDR_PERIODE, LDR_SAMPLES);

    data.time_stamp_msec = pdTICKS_TO_MS(xTaskGetTickCount()); // millisecond
    data.sun_sat = sun_data;
    data.magno_sat = mag_data.vector;
    data.gyro_sat = gyro_data.vector;
    data.accl_sat = accl_data.vector;
    xQueueOverwrite(xQueueSensorData, &data);

  }
}


void loop() {}

