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

// Queue declaration:
struct SensorData {
  uint32_t time_stamp_msec;
  Vector magno_sat;
  Vector sun_sat;
  Vector gyro_sat;
  Vector accl_sat;
};
typedef struct {
  uint32_t time_stamp_msec;
  float scalar_for_e12;
  float scalar_for_e31;
  float scalar_for_e23;
}MagnetorquerScalarData;

QueueHandle_t xQueueSensorData = NULL;
QueueHandle_t xQueueMagnetorquerScalarData = NULL;

// Mutex
Vector reference_world = Vector{.e1 = 0, .e2 = 1, .e3 = 0};
Vector current_world = {0, 0, 0};

SemaphoreHandle_t mutexRefernceVector = NULL;
SemaphoreHandle_t mutexCurrentVector = NULL;
SemaphoreHandle_t mutexSerial = NULL;
SemaphoreHandle_t binarykey1 = NULL;
SemaphoreHandle_t binarykey2 = NULL;
SemaphoreHandle_t binarykey3 = NULL;

void overide_reference_vector(Vector new_reference_world);
void read_reference_vector(Vector &local_reference_world);
void overide_current_vector(Vector new_current_world);
void read_current_vector(Vector &local_current_world);

// Task declaration:
void control_loop(void *pvParameters);
void SensorRead(void *par);
void ActuatorControl(void *par);
// #define DEBUG_CONTROL
void setup() {
#if defined(DEBUG_CONTROL) // Release mode the Serial TX pin is
                  // used for the magnetorquer
                  // MAG1_EN

  Serial.begin(115200); // Initialize serial monitor
  while (!Serial) {
  }
#else
  pinMode(MAG1_EN, OUTPUT);
  pinMode(MAG2_EN, OUTPUT);
#endif
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
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
  // Binary
  binarykey1 = xSemaphoreCreateMutex();
  binarykey2 = xSemaphoreCreateMutex();
  binarykey3 = xSemaphoreCreateMutex();

  // Tasks
  xTaskCreatePinnedToCore(SensorRead,   // Function to call
                          "SensorRead", // Name of the task
                          1124,         // Stack size in bytes
                          NULL,         // Task input parameter
                          1,    // Task priority (0 to configMAX_PRIORITIES - 1)
                          NULL, // Task handle
                          pro_cpu); // Create SendToQueue

  xTaskCreatePinnedToCore(control_loop,   // Function to call
                          "Control Loop", // Name of the task
                          3048,           // Stack size in bytes
                          NULL,           // Task input parameter
                          1,    // Task priority (0 to configMAX_PRIORITIES - 1)
                          NULL, // Task handle
                          app_cpu); // Create Task1

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
  float magnetorquer_dipole = 0.1318f; // ampere meter^2
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

  // The change of basis matrices (cob)
  Matrix3x3 cob_vec_from_world_to_sat;
  Matrix3x3 cob_vec_from_sat_to_world;
  Matrix3x3 cob_bivec_from_world_to_sat;
  Matrix3x3 cob_bivec_from_sat_to_world;

  // Angular Velocity in WORLD
  Bivector angular_velocity_world; // radian / second

  // test the first run
  int first_run = 1; // 1 for yes

  // Time
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  // Serial print
  char buffer0[80];
  char buffer1[80];
  char buffer2[80];
  int i = 0;

  // loop
  for (;;) {
    i++;
    // Receive Data from Queue
    sensor_data.time_stamp_msec =
        pdTICKS_TO_MS(xTaskGetTickCount()); // millisecond
    if (xQueuePeek(xQueueSensorData, &(sensor_data), (TickType_t)10) ==
        pdPASS) {
      snprintf(buffer0, sizeof(buffer0), "\tTime Stamp (msec): %i\n\0",
               sensor_data.time_stamp_msec);
      snprintf(buffer1, sizeof(buffer1),
               "\tMagno Vector { %f, %f, %f } \n\0",
               sensor_data.magno_sat.e1, sensor_data.magno_sat.e2,
               sensor_data.magno_sat.e3);
      snprintf(buffer2, sizeof(buffer2),
               "\tSun Vector { %f, %f, %f } \n\n\0",
               sensor_data.sun_sat.e1, sensor_data.sun_sat.e2,
               sensor_data.sun_sat.e3);
      if (xSemaphoreTake(mutexSerial, 10) == pdTRUE && mutexSerial != NULL &&
          Serial) {
        // Write got data to serial
        Serial.print("Resived Sensor Vector: ");
        Serial.println(i);
        Serial.print(buffer0);
        Serial.print(buffer1);
        Serial.print(buffer2);

        xSemaphoreGive(mutexSerial);
      }
    } else {
      snprintf(buffer0, sizeof(buffer0), "\tTime Stamp (msec): %i\n\0",
               sensor_data.time_stamp_msec);
      snprintf(buffer1, sizeof(buffer1),
               "\tMagno Vector { %f, %f, %f } \n\0",
               sensor_data.magno_sat.e1, sensor_data.magno_sat.e2,
               sensor_data.magno_sat.e3);
      snprintf(buffer2, sizeof(buffer2),
               "\tSun Vector { %f, %f, %f } \n\n\0",
               sensor_data.sun_sat.e1, sensor_data.sun_sat.e2,
               sensor_data.sun_sat.e3);
      if (xSemaphoreTake(mutexSerial, 10) == pdTRUE && mutexSerial != NULL &&
          Serial) {
        // Write running with old data
        Serial.print("Resived Sensor Vector: ");
        Serial.println(i);
        Serial.print(buffer0);
        Serial.print(buffer1);
        Serial.print(buffer2);
        xSemaphoreGive(mutexSerial);
      }
    }

    // Do not run if both vectors are the zero vector
    if ((fabsf(sensor_data.magno_sat.e1) < 1.0e-12f &&
         fabsf(sensor_data.magno_sat.e2) < 1.0e-12f &&
         fabsf(sensor_data.magno_sat.e3) < 1.0e-12f) ||
        (fabsf(sensor_data.sun_sat.e1) < 1.0e-12f &&
         fabsf(sensor_data.sun_sat.e2) < 1.0e-12f &&
         fabsf(sensor_data.sun_sat.e3) < 1.0e-12f)) {

      magnetorquer_scalars.scalar_for_e12 = 0.0f;
      magnetorquer_scalars.scalar_for_e31 = 0.0f;
      magnetorquer_scalars.scalar_for_e23 = 0.0f;

      magnetorquer_scalars.time_stamp_msec =
          pdTICKS_TO_MS(xTaskGetTickCount()); // millisecond
      if (xQueueOverwrite(xQueueMagnetorquerScalarData,
                          &magnetorquer_scalars) == pdPASS) {
        snprintf(buffer0, sizeof(buffer0), "\tTime Stamp (msec): %i \n\0",
                 magnetorquer_scalars.time_stamp_msec);
        if (xSemaphoreTake(mutexSerial, 10) == pdTRUE && mutexSerial != NULL &&
            Serial) {
          Serial.println("One of the sensor vectors is the zero vector");
          Serial.print("Send Scalar to Actuator Controller: ");
          Serial.println(i);
          Serial.print(buffer0);
          Serial.print("\tScalar for e12: ");
          Serial.println(magnetorquer_scalars.scalar_for_e12);
          Serial.print("\tScalar for e31: ");
          Serial.println(magnetorquer_scalars.scalar_for_e31);
          Serial.print("\tScalar for e23: ");
          Serial.println(magnetorquer_scalars.scalar_for_e23);
          Serial.println("");

          xSemaphoreGive(mutexSerial);
        }
      }

    } else {

      // Get magnotometer output as bivector
      magnetric_flux_density_sat =
         scale_bivector(     // scale from micro Tesla to Tesla
      dual_vector_to_bivector(sensor_data.magno_sat),1e-6f); 

#if defined(DEBUG_CONTROL)
      snprintf(buffer0, sizeof(buffer0),
               "\tMag Bivector { %.2ee12, %.2ee31, %.2ee23 }: %i \n\n\0",
               magnetric_flux_density_sat.e12, magnetric_flux_density_sat.e31,
               magnetric_flux_density_sat.e23, i);
      if (xSemaphoreTake(mutexSerial, 10) == pdTRUE && mutexSerial != NULL &&
          Serial) {
        Serial.print(buffer0);
        xSemaphoreGive(mutexSerial);
      }
#endif

      // make change of basis matrix from messurements
      cob_from_measure(cob_vec_from_world_to_sat, cob_vec_from_sat_to_world,
                       cob_bivec_from_world_to_sat, cob_bivec_from_sat_to_world,
                       sensor_data.sun_sat, sensor_data.magno_sat);

#if defined(DEBUG_CONTROL)
      snprintf(buffer0, sizeof(buffer0),
               "\tCOB Matrix\n"
               "\t{ %.2e, %.2e, %.2e\n",
               cob_vec_from_world_to_sat.m11, cob_vec_from_world_to_sat.m12,
               cob_vec_from_world_to_sat.m13);
      snprintf(buffer1, sizeof(buffer1),
               "\t  %.2e, %.2e, %.2e\n"
               "\t  %.2e, %.2e, %.2e }: %i\n\n",
               cob_vec_from_world_to_sat.m21, cob_vec_from_world_to_sat.m22,
               cob_vec_from_world_to_sat.m23, cob_vec_from_world_to_sat.m31,
               cob_vec_from_world_to_sat.m32, cob_vec_from_world_to_sat.m33, i);
      if (xSemaphoreTake(mutexSerial, 10) == pdTRUE && mutexSerial != NULL &&
          Serial) {
        Serial.print(buffer0);
        Serial.print(buffer1);
        xSemaphoreGive(mutexSerial);
      }
#endif

      // The inerita is defined in SAT
      // It is for bivectors
      inertia_world = matrix_mul(cob_bivec_from_sat_to_world, inertia_sat);

#if defined(DEBUG_CONTROL)
      snprintf(buffer0, sizeof(buffer0),
               "Inertia WORLD Matrix\n"
               "\t{ %.2e, %.2e, %.2e\n",
               inertia_world.m11, inertia_world.m12, inertia_world.m13);
      snprintf(buffer1, sizeof(buffer1),
               "\t  %.2e, %.2e, %.2e\n"
               "\t  %.2e, %.2e, %.2e }: %i\n\n",
               inertia_world.m21, inertia_world.m22, inertia_world.m23,
               inertia_world.m31, inertia_world.m32, inertia_world.m33, i);
      if (xSemaphoreTake(mutexSerial, 10) == pdTRUE && mutexSerial != NULL &&
          Serial) {
        Serial.print(buffer0);
        Serial.print(buffer1);
        xSemaphoreGive(mutexSerial);
      }
#endif

      // The reference is WORLD frame
      Vector reference_world;
      read_reference_vector(reference_world);

      // the current postion is also fixed in SAT frame
      Vector current_world =
          matrix_vector_mul(cob_vec_from_sat_to_world, Vector{1, 0, 0});

#if defined(DEBUG_CONTROL)
      snprintf(buffer0, sizeof(buffer0),
               "Reference WORLD Vector "
               "{ %f, %f, %f }: %i\n",
               reference_world.e1, reference_world.e2, reference_world.e3, i);
      snprintf(buffer1, sizeof(buffer1),
               "Current WORLD Vector "
               "{ %f, %f, %f }: %i\n\n",
               current_world.e1, current_world.e2, current_world.e3, i);
      if (xSemaphoreTake(mutexSerial, 10) == pdTRUE && mutexSerial != NULL &&
          Serial) {
        Serial.print(buffer0);
        Serial.print(buffer1);
        xSemaphoreGive(mutexSerial);
      }
#endif

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

#if defined(DEBUG_CONTROL)
      snprintf(buffer0, sizeof(buffer0),
               "Angle Error WORLD Bivector "
               "{ %f, %f, %f }: %i\n\n",
               angle_err_world.e12, angle_err_world.e31, angle_err_world.e23,
               i);
      if (xSemaphoreTake(mutexSerial, 10) == pdTRUE && mutexSerial != NULL &&
          Serial) {
        Serial.print(buffer0);
        xSemaphoreGive(mutexSerial);
      }
#endif

      Bivector pid_res_world =
          pid(K_P, K_I, K_D, angle_err_world, last_int_world, last_err_world,
              CONTROL_PERIODE);

#if defined(DEBUG_CONTROL)
      snprintf(buffer0, sizeof(buffer0),
               "PID Res WORLD Bivector "
               "{ %f, %f, %f }: %i\n\n",
               pid_res_world.e12, pid_res_world.e31, pid_res_world.e23, i);
      if (xSemaphoreTake(mutexSerial, 10) == pdTRUE && mutexSerial != NULL &&
          Serial) {
        Serial.print(buffer0);
        xSemaphoreGive(mutexSerial);
      }
#endif

      Bivector torque_1_world =
          matrix_bivector_mul(inertia_world, pid_res_world);

#if defined(DEBUG_CONTROL)
      snprintf(buffer0, sizeof(buffer0),
               "Torque 1 WORLD Bivector "
               "{ %f, %f, %f }: %i\n\n",
               torque_1_world.e12, torque_1_world.e31, torque_1_world.e23, i);
      if (xSemaphoreTake(mutexSerial, 10) == pdTRUE && mutexSerial != NULL &&
          Serial) {
        Serial.print(buffer0);
        xSemaphoreGive(mutexSerial);
      }
#endif


      Bivector torque_2_world =
          actuator_test(torque_1_world, cob_bivec_from_sat_to_world,
                        magnetric_flux_density_sat, mag1_sat, mag2_sat,
                        mag3_sat, magnetorquer_scalars.scalar_for_e12,
                        magnetorquer_scalars.scalar_for_e31,
                        magnetorquer_scalars.scalar_for_e23);

#if defined(DEBUG_CONTROL)
      snprintf(buffer0, sizeof(buffer0),
               "Torque 2 WORLD Bivector "
               "{ %f, %f, %f }: %i\n\n",
               torque_2_world.e12, torque_2_world.e31, torque_2_world.e23, i);
      if (xSemaphoreTake(mutexSerial, 10) == pdTRUE && mutexSerial != NULL &&
          Serial) {
        Serial.print(buffer0);
        xSemaphoreGive(mutexSerial);
      }
#endif

      // send scalars to queue
      magnetorquer_scalars.time_stamp_msec =
          pdTICKS_TO_MS(xTaskGetTickCount()); // millisecond
      if (xQueueOverwrite(xQueueMagnetorquerScalarData,
                          &magnetorquer_scalars) == pdPASS) {
        snprintf(buffer0, sizeof(buffer0), "\tTime Stamp (msec): %i \n\0",
                 magnetorquer_scalars.time_stamp_msec);
        if (xSemaphoreTake(mutexSerial, 10) == pdTRUE && mutexSerial != NULL &&
            Serial) {
          Serial.print("Send Scalar to Actuator Controller: ");
          Serial.println(i);
          Serial.print(buffer0);
          Serial.print("\tScalar for e12: ");
          Serial.println(magnetorquer_scalars.scalar_for_e12);
          Serial.print("\tScalar for e31: ");
          Serial.println(magnetorquer_scalars.scalar_for_e31);
          Serial.print("\tScalar for e23: ");
          Serial.println(magnetorquer_scalars.scalar_for_e23);
          Serial.println("");

          xSemaphoreGive(mutexSerial);
        }
      }

      Bivector rotation_angle_world =
          angle_from_torque(torque_2_world, inertia_world,
                            angular_velocity_world, CONTROL_PERIODE);

      // find the next point direction
      Rotor rotor = rotor_form_halv_angle_bivector(
          scale_bivector(rotation_angle_world, 0.5));

      // Write the calculated next current vector
      overide_current_vector(rotate_vector(current_world, rotor));
    } // end of zero vector test
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(int(CONTROL_PERIODE * 1000)));
  }
}

void SensorRead(void *par) {
  xSemaphoreTake(binarykey1, portMAX_DELAY);
  if (MPU_I2C.begin(SDA, SCL, ClockSpeed) == false) {
    if (xSemaphoreTake(mutexSerial, pdMS_TO_TICKS(5000)) == pdTRUE && mutexSerial != NULL &&
        Serial) {
      Serial.println("I2C begin Failed!");
      xSemaphoreGive(mutexSerial);
    }
  }
  // Define param of task
  const mag_resolution mag_res = BIT_16;          // mag resolution
  const mag_meas_mode mag_mode = MEAS_MODE2;      // mag mode
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

  TickType_t xLastWakeTime = 0;
  xLastWakeTime = xTaskGetTickCount(); // Get the current tick count

  // Task loop
#if defined(DEBUG_CONTROL)
  while (1) {
    data.time_stamp_msec = pdTICKS_TO_MS(xTaskGetTickCount()); // millisecond
    data.sun_sat = Vector{
        .e1 = 0.7071f,
        .e2 = -0.7071f,
        .e3 = 0.0f,
    };
    float magnetric_flux_density_norm = 0.00002f; // 20 * 10^(-6) tesla
    data.magno_sat = Vector{
        .e1 = 0.7071f * magnetric_flux_density_norm,
        .e2 = 0.7071f * magnetric_flux_density_norm,
        .e3 = 0.0f,
    };
    xQueueOverwrite(xQueueSensorData, &data);
  }
#else
  while (1) {
    read_data(&gyro_data);
    read_data(&accl_data);

    xSemaphoreTake(binarykey3, portMAX_DELAY);
    xSemaphoreGive(binarykey1); // Signal to ActuatorControl to stop magnotorqer
    xSemaphoreTake(binarykey2, portMAX_DELAY); // wait for signal

    read_data(&mag_data); // measure magneticfield

    xSemaphoreGive(binarykey2);                // return key
    xSemaphoreTake(binarykey1, portMAX_DELAY); // Retrive key
    xSemaphoreGive(binarykey3);
    
    sun_read_data(&sun_data, LDR_PERIODE, LDR_SAMPLES);

    // scaling and calibrating the mag_data after to speed up the reading time
    scale(&mag_data.vector, mag_data.scale_factor); 
    mag_data.vector.e1 -= mag_hardiron_bias_e1;
    mag_data.vector.e2 -= mag_hardiron_bias_e2;
    mag_data.vector.e3 -= mag_hardiron_bias_e3;

    data.time_stamp_msec = pdTICKS_TO_MS(xTaskGetTickCount()); // millisecond
    data.sun_sat = sun_data;
    data.magno_sat = mag_data.vector;
    data.gyro_sat = gyro_data.vector;
    data.accl_sat = accl_data.vector;
    xQueueOverwrite(xQueueSensorData, &data);

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(SENSORREAD_PERIODE)); // Delay
  }
#endif
}

void ActuatorControl(void *par) {
  xSemaphoreTake(binarykey2, portMAX_DELAY);
  MagnetorquerScalarData receivedScalarData;
  // Enable H-Bridges
  enable_mag(true);
  // Set PWM frequncy
  analogWriteResolution((uint8_t)(PWM_RES));
  analogWriteFrequency((uint32_t)(PWM_FREQ));
  while (1) {

#if defined(DEBUG_PWM)
    receivedScalarData.scalar_for_e31 = -0.2;
    // receivedScalarData.scalar_for_e23 = 0.2;
    pulse_mag(receivedScalarData.scalar_for_e31, MAG2_CW, MAG2_CCW);
    // pulse_mag(receivedScalarData.scalar_for_e23, MAG1_CW, MAG1_CCW);

#else
    if (xQueueReceive(xQueueMagnetorquerScalarData, &receivedScalarData, 10) ==
        pdPASS) {
      if (xSemaphoreTake(mutexSerial, 100) == pdTRUE && mutexSerial != NULL &&
          Serial) {
        Serial.println("Received the following data: ");
        Serial.print("\tTimeStamp(msec): ");
        Serial.println(receivedScalarData.time_stamp_msec);
        Serial.print("\te12: ");
        Serial.println(receivedScalarData.scalar_for_e12, 6);
        Serial.print("\te31: ");
        Serial.println(receivedScalarData.scalar_for_e31, 6);
        Serial.print("\te23: ");
        Serial.println(receivedScalarData.scalar_for_e23, 6);

        xSemaphoreGive(mutexSerial);
      }
    }

    if (xSemaphoreTake(binarykey1, pdMS_TO_TICKS(1)) ==
        pdTRUE) { // Check for signal from SensorRead
      enable_mag(false); // Turn off Magnotorqer
      vTaskDelay(pdMS_TO_TICKS(MAG_POWER_DOWN_TIME_MS)); // Wait for magnototorer too power down
      xSemaphoreGive(binarykey1); // signal to SensorRead to measure magneticfield
      xSemaphoreGive(binarykey2); // give back the first key
      xSemaphoreTake(binarykey3, portMAX_DELAY); // wait for SensorRead to be done
      enable_mag(true);
      xSemaphoreGive(binarykey3);
      xSemaphoreTake(binarykey2, portMAX_DELAY);
    }
    // Set the PWM signal to the H-Bridge
    pulse_mag(receivedScalarData.scalar_for_e31, MAG2_CW, MAG2_CCW);
    pulse_mag(receivedScalarData.scalar_for_e23, MAG1_CW, MAG1_CCW);
#endif
  }
}

// Mutex functions
void overide_reference_vector(Vector new_reference_world) {
  if (xSemaphoreTake(mutexRefernceVector, (TickType_t)10) == pdTRUE) {
    reference_world = new_reference_world;
    xSemaphoreGive(mutexRefernceVector);
  } else {
    // Serial.println("Could not take mutexReferenceVector");
    // Serial.println("No change happend");
    // Serial.print("\tTime Stamp: ");
    // Serial.println(pdTICKS_TO_MS(xTaskGetTickCount())); // millisecond
  }
}
void read_reference_vector(Vector &local_reference_world) {
  if (xSemaphoreTake(mutexRefernceVector, (TickType_t)10) == pdTRUE) {
    local_reference_world = reference_world;
    xSemaphoreGive(mutexRefernceVector);
  } else {
    // Serial.println("Could not take mutexReferenceVector");
    // Serial.println("No vector was copied");
    // Serial.print("\tTime Stamp: ");
    // Serial.println(pdTICKS_TO_MS(xTaskGetTickCount())); // millisecond
  }
}

void overide_current_vector(Vector new_current_world) {
  if (xSemaphoreTake(mutexCurrentVector, (TickType_t)10) == pdTRUE) {
    current_world = new_current_world;
    xSemaphoreGive(mutexCurrentVector);
  } else {
    // Serial.println("Could not take mutexCurrentVector");
    // Serial.println("No change happend");
    // Serial.print("\tTime Stamp: ");
    // Serial.println(pdTICKS_TO_MS(xTaskGetTickCount())); // millisecond
  }
}
void read_current_vector(Vector &local_current_world) {
  if (xSemaphoreTake(mutexCurrentVector, (TickType_t)10) == pdTRUE) {
    local_current_world = current_world;
    xSemaphoreGive(mutexCurrentVector);
  } else {
    // Serial.println("Could not take mutexCurrentVector");
    // Serial.println("No vector was copied");
    // Serial.print("\tTime Stamp: ");
    // Serial.println(pdTICKS_TO_MS(xTaskGetTickCount())); // millisecond
  }
}
