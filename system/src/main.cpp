#include "Arduino.h"
#include "loop.hpp"
#include "math_stuff.hpp"
#include <AK8963_ADDRESS.hpp>
#include <Config.hpp>
#include <Hbro.hpp>
#include <LDR.hpp>
#include <MPU_ADDRESS.hpp>
#include <MPU_I2C.hpp>
#include <cstdint>

// Queue declaration:
struct SensorData {
  uint32_t time_stamp_msec;
  Vector magno_sat;
  Vector sun_sat;
};
struct MagnetorquerScalarData {
  uint32_t time_stamp_msec;
  float scalar_for_e12;
  float scalar_for_e31;
  float scalar_for_e23;
};
QueueHandle_t xQueueSensorData = NULL;
QueueHandle_t xQueueMagnetorquerScalarData = NULL;

// Mutex
Vector reference_world = Vector{.e1 = 0, .e2 = 1, .e3 = 0};
Vector current_world;

SemaphoreHandle_t mutexRefernceVector = NULL;
SemaphoreHandle_t mutexCurrentVector = NULL;
SemaphoreHandle_t mutexSerial = NULL;

void overide_reference_vector(Vector new_reference_world);
void read_reference_vector(Vector &local_reference_world);
void overide_current_vector(Vector new_current_world);
void read_current_vector(Vector &local_current_world);

// Task declaration:
void control_loop(void *pvParameters);
void SensorRead(void *par);
void ActuatorControl(void *par);

void setup() {
  Serial.begin(115200); // Initialize serial monitor
  while (!Serial) {
  }

  // Enable H-Bridges
  digitalWrite(HIGH, MAG1_EN);
  digitalWrite(HIGH, MAG2_EN);

  // Queues
  xQueueSensorData = xQueueCreate(1, sizeof(SensorData));
  xQueueMagnetorquerScalarData =
      xQueueCreate(1, sizeof(MagnetorquerScalarData));

  if (xQueueSensorData == NULL) {

    /* Queue was not created and must not be used. */
  }

  // Mutex
  mutexRefernceVector = xSemaphoreCreateMutex();
  mutexCurrentVector = xSemaphoreCreateMutex();
  mutexSerial = xSemaphoreCreateMutex();

  // Tasks
  xTaskCreatePinnedToCore(control_loop,   // Function to call
                          "Control Loop", // Name of the task
                          2048,           // Stack size in bytes
                          NULL,           // Task input parameter
                          1,    // Task priority (0 to configMAX_PRIORITIES - 1)
                          NULL, // Task handle
                          app_cpu); // Create Task1

  xTaskCreatePinnedToCore(SensorRead,   // Function to call
                          "SensorRead", // Name of the task
                          1024,         // Stack size in bytes
                          NULL,         // Task input parameter
                          1,    // Task priority (0 to configMAX_PRIORITIES - 1)
                          NULL, // Task handle
                          pro_cpu); // Create SendToQueue

  xTaskCreatePinnedToCore(ActuatorControl,
                          "ActuatorControl", // Name of the task
                          1024,              // Stack size in bytes
                          NULL,              // Task input parameter
                          1,    // Task priority (0 to configMAX_PRIORITIES - 1)
                          NULL, // Task handle
                          pro_cpu); // Create SendToQueue
}

void loop() {}

void control_loop(void *pvParameters) {

  // pid
  Bivector last_int_world{
      // The last derivetive variable in WORLD
      .e12 = 0.0f,
      .e31 = 0.0f,
      .e23 = 0.0f,
  };

  Bivector last_err_world{
      // The last derivetive variable in world
      .e12 = 0.0f,
      .e31 = 0.0f,
      .e23 = 0.0f,
  };

  // inertia matrix in SAT
  // This is modelled as a plate
  // 1/12 * m * (a^2 + b^2)
  Matrix3x3 inertia_sat{
      .m11 = 1.0f / 6.0f * 0.01f * 0.01f, // kilogram * meter^2
      .m12 = 0.0f,
      .m13 = 0.0f,

      .m21 = 0.0f,
      .m22 = 1.0f,
      .m23 = 0.0f,

      .m31 = 0.0f,
      .m32 = 0.0f,
      .m33 = 1.0f,
  };
  // This need to be in WORLD coordinats
  Matrix3x3 inertia_world;

  // Magnetorquers
  float magnetorquer_dipole = 0.1458f; // ampere meter^2
  MagnetorquerScalarData magnetorquer_scalars;
  Bivector mag1_sat = Bivector{
      // This does not exist
      .e12 = 1.0f,
      .e31 = 0.0f,
      .e23 = 0.0f,
  };
  Bivector mag2_sat = Bivector{
      .e12 = 0.0f,
      .e31 = magnetorquer_dipole,
      .e23 = 0.0f,
  };
  Bivector mag3_sat = Bivector{
      .e12 = 0.0f,
      .e31 = 0.0f,
      .e23 = magnetorquer_dipole,
  };

  /// Sensors
  SensorData sensor_data;

  Bivector magnetric_flux_density_sat;

  // The change of basis matrices
  Matrix3x3 cob_vec_from_world_to_sat;
  Matrix3x3 cob_vec_from_sat_to_world;
  Matrix3x3 cob_bivec_from_world_to_sat;
  Matrix3x3 cob_bivec_from_sat_to_world;

  // Angular Velocity in WORLD
  Bivector angular_velocity_world; // radian / second

  // This is a test with no real messurements
  // This is a rotation angle the "sensors" are rotated by
  // This must be done in SAT
  Bivector sensor_rotation_angle_sat; // radian

  // test the first run
  int first_run = 1; // 1 for yes

  // Time
  TickType_t xLastWakeTime;
  float control_loop_periode = 0.500; // second
  const TickType_t xFrequency = pdMS_TO_TICKS(int(control_loop_periode * 1000));
  xLastWakeTime = xTaskGetTickCount();

  // loop
  for (;;) {
    // Receive Data from Queue
    sensor_data.time_stamp_msec =
        pdTICKS_TO_MS(xTaskGetTickCount()); // millisecond
    if (xQueueReceive(xQueueSensorData, &(sensor_data), (TickType_t)10) ==
        pdPASS) {
          if(xSemaphoreTake(mutexSerial, 0) == pdTRUE){
            // Write got data to serial
            Serial.println("Resived Sensor Vector");
            Serial.print("\tTime Stamp (msec): ");
            Serial.println(sensor_data.time_stamp_msec);
            Serial.print("\tMagno Vector { ");
            Serial.print(sensor_data.magno_sat.e1);
            Serial.print("e1, ");
            Serial.print(sensor_data.magno_sat.e2);
            Serial.print("e2 ");
            Serial.print(sensor_data.magno_sat.e3);
            Serial.println("e3 }");
            Serial.print("\tSun Vector { ");
            Serial.print(sensor_data.sun_sat.e1);
            Serial.print("e1, ");
            Serial.print(sensor_data.sun_sat.e2);
            Serial.print("e2 ");
            Serial.print(sensor_data.sun_sat.e3);
            Serial.println("e3 }");

            xSemaphoreGive(mutexSerial);
          }
     
    } else {
        if(xSemaphoreTake(mutexSerial, 0) == pdTRUE){
          // Write running with old data
          Serial.println("Using Old Sensor Vector");
          Serial.print("\tOld Time Stamp (msec): ");
          Serial.println(sensor_data.time_stamp_msec);
          Serial.print("\tMagno Vector { ");
          Serial.print(sensor_data.magno_sat.e1);
          Serial.print("e1, ");
          Serial.print(sensor_data.magno_sat.e2);
          Serial.print("e2 ");
          Serial.print(sensor_data.magno_sat.e3);
          Serial.println("e3 }");
          Serial.print("\tSun Vector { ");
          Serial.print(sensor_data.sun_sat.e1);
          Serial.print("e1, ");
          Serial.print(sensor_data.sun_sat.e2);
          Serial.print("e2 ");
          Serial.print(sensor_data.sun_sat.e3);
          Serial.println("e3 }");

          xSemaphoreGive(mutexSerial);
        }

    }

    // Get magnotometer output as bivector
    magnetric_flux_density_sat = dual_vector_to_bivector(sensor_data.magno_sat);

    // make change of basis matrix from messurements
    cob_from_measure(cob_vec_from_world_to_sat, cob_vec_from_sat_to_world,
                     cob_bivec_from_world_to_sat, cob_bivec_from_sat_to_world,
                     sensor_data.sun_sat, sensor_data.magno_sat);

    // The inerita is defined in SAT
    // It is for bivectors
    inertia_world = matrix_mul(cob_bivec_from_sat_to_world, inertia_sat);

    // The reference is WORLD frame
    Vector reference_world;
    read_reference_vector(reference_world);

    // the current postion is also fixed in SAT frame
    Vector current_world =
        matrix_vector_mul(cob_vec_from_sat_to_world, Vector{1, 0, 0});

    // mix the last calcutated point dir
    if (!first_run) {
      // The calculated pointing direction from last iteration
      // n-1
      Vector last_calc_world;
      read_current_vector(last_calc_world);

      current_world = Vector{
          .e1 = (current_world.e1 + last_calc_world.e1) * 0.5f,
          .e2 = (current_world.e2 + last_calc_world.e2) * 0.5f,
          .e3 = (current_world.e3 + last_calc_world.e3) * 0.5f,
      };
    } else {
      first_run = 0;
    }

    Bivector angle_err_world =
        angle_difference_bivector(current_world, reference_world);

    Bivector pid_res_world =
        pid(0.4f, 0.0f, 1.0f, angle_err_world, last_int_world, last_err_world,
            control_loop_periode);

    Bivector torque_1_world = matrix_bivector_mul(inertia_world, pid_res_world);

    Bivector torque_2_world = actuator_test(
        torque_1_world, cob_bivec_from_sat_to_world, magnetric_flux_density_sat,
        mag1_sat, mag2_sat, mag3_sat, magnetorquer_scalars.scalar_for_e12,
        magnetorquer_scalars.scalar_for_e31,
        magnetorquer_scalars.scalar_for_e23);

    // send scalars to queue
    magnetorquer_scalars.time_stamp_msec =
        pdTICKS_TO_MS(xTaskGetTickCount()); // millisecond
    if (xQueueOverwrite(xQueueMagnetorquerScalarData, &magnetorquer_scalars) ==
        pdPASS) {
          if(xSemaphoreTake(mutexSerial, 0) == pdTRUE){
            Serial.println("Send Scalar to Actuator Controller");
            Serial.print("\tTime Stamp (msec): ");
            Serial.println(magnetorquer_scalars.time_stamp_msec);
            Serial.print("\tScalar for e12: ");
            Serial.println(magnetorquer_scalars.scalar_for_e12);
            Serial.print("\tScalar for e31: ");
            Serial.println(magnetorquer_scalars.scalar_for_e31);
            Serial.print("\tScalar for e23: ");
            Serial.println(magnetorquer_scalars.scalar_for_e23);

            xSemaphoreGive(mutexSerial);
          }

    }

    Bivector rotation_angle_world =
        angle_from_torque(torque_2_world, inertia_world, angular_velocity_world,
                          control_loop_periode);

    // find the next point direction
    Rotor rotor = rotor_form_halv_angle_bivector(
        scale_bivector(rotation_angle_world, 0.5));

    // Write the calculated next current vector
    overide_current_vector(rotate_vector(current_world, rotor));

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void SensorRead(void *par) {
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
  SensorVector acc_data = {unknown, 0, 0, 0, 0};
  SensorVector mag_data = {unknown, 0, 0, 0, 0};
  Vector sun_data = {0, 0, 0}; // LDR data structure
  uint16_t period = 10;        // millisecond
  uint16_t samples = 100;
  // Config thing
  bypass_to_magnometer(true);
  I2Cbus_SCCAN();
  delay(2000);

  mag_resolution_config(mag_res);
  mag_meas_config(mag_mode);
  gyro_fs_sel(gyro_fs);
  accel_fs_sel(accel_fs);
  write(AK8963_ADDRESS, ASTC,
        0x00); // Set clock source to PLL with X axis gyroscope reference

  // Init of sensorvectors
  initSensorVector(&gyro_data, GYROSCOPE, gyro_scale);
  initSensorVector(&acc_data, ACCELEROMETER, accel_scale);
  initSensorVector(&mag_data, MAGNOTOMETER, mag_scale);
  SensorData data = {.time_stamp_msec = (xTaskGetTickCount()),
                     .magno_sat = mag_data.vector,
                     .sun_sat = sun_data};

  // Task loop
  while (1) {

    read_data(&gyro_data);
    read_data(&acc_data);
    read_data(&mag_data);
    sun_read_data(&sun_data, period, samples);

    data.time_stamp_msec = pdTICKS_TO_MS(xTaskGetTickCount()); // millisecond
    data.sun_sat = sun_data;
    data.magno_sat = mag_data.vector;
    xQueueOverwrite(xQueueSensorData, &data);
  }
}

void ActuatorControl(void *par) {
  struct MagnetorquerScalarData receivedScalarData;

  while (1) {
    if (xQueueReceive(xQueueMagnetorquerScalarData, &receivedScalarData, 10) ==
        pdPASS) {
          if(xSemaphoreTake(mutexSerial, 0) == pdTRUE){
      Serial.println("Received the following data: ");
      Serial.print("\te12: ");
      Serial.println(receivedScalarData.scalar_for_e12, 6);
      Serial.print("\te31: ");
      Serial.println(receivedScalarData.scalar_for_e31, 6);
      Serial.print("\te23: ");
      Serial.println(receivedScalarData.scalar_for_e23, 6);
      Serial.print("\tTimeStamp(msec): ");
      Serial.println(receivedScalarData.time_stamp_msec);

            xSemaphoreGive(mutexSerial);
          }

    }
    PulseMag(receivedScalarData.scalar_for_e31, MAG2_CW, MAG2_CCW);
    PulseMag(receivedScalarData.scalar_for_e23, MAG1_CW, MAG2_CCW);
  }
}

// Mutex functions
void overide_reference_vector(Vector new_reference_world) {
  if (xSemaphoreTake(mutexRefernceVector, (TickType_t)10) == pdTRUE) {
    reference_world = new_reference_world;
    xSemaphoreGive(mutexRefernceVector);
  } else {
    Serial.println("Could not take mutexReferenceVector");
    Serial.println("No change happend");
    Serial.print("\tTime Stamp: ");
    Serial.println(pdTICKS_TO_MS(xTaskGetTickCount())); // millisecond
  }
}
void read_reference_vector(Vector &local_reference_world) {
  if (xSemaphoreTake(mutexRefernceVector, (TickType_t)10) == pdTRUE) {
    local_reference_world = reference_world;
    xSemaphoreGive(mutexRefernceVector);
  } else {
    Serial.println("Could not take mutexReferenceVector");
    Serial.println("No vector was copied");
    Serial.print("\tTime Stamp: ");
    Serial.println(pdTICKS_TO_MS(xTaskGetTickCount())); // millisecond
  }
}

void overide_current_vector(Vector new_current_world) {
  if (xSemaphoreTake(mutexCurrentVector, (TickType_t)10) == pdTRUE) {
    current_world = new_current_world;
    xSemaphoreGive(mutexCurrentVector);
  } else {
    Serial.println("Could not take mutexCurrentVector");
    Serial.println("No change happend");
    Serial.print("\tTime Stamp: ");
    Serial.println(pdTICKS_TO_MS(xTaskGetTickCount())); // millisecond
  }
}
void read_current_vector(Vector &local_current_world) {
  if (xSemaphoreTake(mutexCurrentVector, (TickType_t)10) == pdTRUE) {
    local_current_world = current_world;
    xSemaphoreGive(mutexCurrentVector);
  } else {
    Serial.println("Could not take mutexCurrentVector");
    Serial.println("No vector was copied");
    Serial.print("\tTime Stamp: ");
    Serial.println(pdTICKS_TO_MS(xTaskGetTickCount())); // millisecond
  }
}
