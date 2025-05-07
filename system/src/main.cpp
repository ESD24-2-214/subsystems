#include "Arduino.h"
#include "loop.hpp"
#include "math_stuff.hpp"
#include <AK8963_ADDRESS.hpp>
#include <Config.hpp>
#include <LDR.hpp>
#include <MPU_ADDRESS.hpp>
#include <MPU_I2C.hpp>
#include <cstdint>

// Queue declaration:
struct SensorData {
  uint32_t time_stamp;
  Vector magno_sat;
  Vector sun_sat;
};
struct MagnetorquerScalarData {
  uint32_t time_stamp;
  float scalar_1;
  float scalar_2;
  float scalar_3;
};
QueueHandle_t xQueueSensorData = NULL;
QueueHandle_t xQueueMagnetorquerScalarData = NULL;

// Task declaration:
void control_loop(void *pvParameters);
void SensorRead(void *par);

void setup() {
  Serial.begin(115200); // Initialize serial monitor
  while (!Serial) {
  }

  // Queues
  xQueueSensorData = xQueueCreate(1, sizeof(SensorData));
  xQueueMagnetorquerScalarData =
      xQueueCreate(1, sizeof(MagnetorquerScalarData));

  if (xQueueSensorData == NULL) {

    /* Queue was not created and must not be used. */
  }

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
}

void communication() {
  Serial.write(0x1B);
  Serial.print("[2J"); // clear screen
  Serial.write(0x1B);
  Serial.print("[1;1H"); // place cursor in 1 1
  Serial.println("Hello, World!");
}

void loop() { communication(); }

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

  // The calculated next pointing direction
  // n+1
  Vector current_calc_world;

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
  float mag_s1 = 0.0f;
  float mag_s2 = 0.0f;
  float mag_s3 = 0.0f;
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

  // sun sensor
  // This is messured
  // Used to create WORLD
  Vector sun_sensor_sat{
      .e1 = 0.7071f,
      .e2 = -0.7071f,
      .e3 = 0.0f,
  };

  // Magnetic Flux Density
  // Messured in MAG frame
  // Translated to SAT frame
  // Used to create WORLD
  float magnetric_flux_density_norm = 0.00002f; // 20 * 10^(-6) tesla
  Vector mag_sensor_sat{
      .e1 = 0.7071f * magnetric_flux_density_norm,
      .e2 = 0.7071f * magnetric_flux_density_norm,
      .e3 = 0.0f,
  };

  Bivector magnetric_flux_density_sat = dual_vector_to_bivector(mag_sensor_sat);

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
  float control_loop_periode = 0.200; // second
  const TickType_t xFrequency = pdMS_TO_TICKS(int(control_loop_periode * 1000));
  xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    // Receive Data from Queue
    if (xQueueReceive(xQueueSensorData, &(sensor_data), (TickType_t)10) ==
        pdPASS) {

      /* xRxedStructure now contains a copy of xMessage. */
    }

    // "messure the sensors"
    measure(sensor_rotation_angle_sat, sun_sensor_sat, mag_sensor_sat);
    // make change of basis matrix from messurements
    cob_from_measure(cob_vec_from_world_to_sat, cob_vec_from_sat_to_world,
                     cob_bivec_from_world_to_sat, cob_bivec_from_sat_to_world,
                     sun_sensor_sat, mag_sensor_sat);

    // The inerita is defined in SAT
    // It is for bivectors
    inertia_world = matrix_mul(cob_bivec_from_sat_to_world, inertia_sat);

    // The reference is WORLD frame
    Vector reference_world = Vector{0, 1, 0};
    // the current postion is also fixed in SAT frame
    Vector current_world =
        matrix_vector_mul(cob_vec_from_sat_to_world, Vector{1, 0, 0});

    // mix the last calcutated point dir
    if (first_run) {
      current_world = Vector{
          .e1 = (current_world.e1 + current_calc_world.e1) * 0.5f,
          .e2 = (current_world.e2 + current_calc_world.e2) * 0.5f,
          .e3 = (current_world.e3 + current_calc_world.e3) * 0.5f,
      };
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
        mag1_sat, mag2_sat, mag3_sat, mag_s1, mag_s2, mag_s3);

    Bivector rotation_angle_world =
        angle_from_torque(torque_2_world, inertia_world, angular_velocity_world,
                          control_loop_periode);

    // Set rotation angle for sensor
    // It is in SAT
    // the sensors must rotated the oppiste direction of the sat
    sensor_rotation_angle_sat = scale_bivector(
        matrix_bivector_mul(cob_bivec_from_world_to_sat, rotation_angle_world),
        -1.0);

    // find the next point direction
    Rotor rotor = rotor_form_halv_angle_bivector(
        scale_bivector(rotation_angle_world, 0.5));
    current_calc_world = rotate_vector(current_world, rotor);

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
  LDRData_t ldr_data = {0, 0, 0, 0}; // LDR data structure

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

  // Task loop
  while (1) {
    read_data(&gyro_data);
    read_data(&acc_data);
    read_data(&mag_data);
    ldr_read_data(&ldr_data, 100, 10);
  }
}
