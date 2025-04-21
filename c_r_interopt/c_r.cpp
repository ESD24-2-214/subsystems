#include "math_stuff.hpp"
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <stdlib.h>

// Macros
#define FNPRINT(...)                                                           \
  printf("  ");                                                                \
  printf(__VA_ARGS__)

// Global Variables
float sampling_time = 0.100f; // seconds
float run_time = 15.0f;       // second

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
// sun sensor
// This is messured
// Used to create WORLD
Vector sun_sensor_sat{
    .e1 = 0.7071f,
    .e2 = -0.7071f,
    .e3 = 0.0f,
};
Vector sun_sensor_normed_sat;

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
Vector mag_sensor_normed_sat;

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

// Functions
void measure();
void normilize_measure();
Bivector pid(float K_p, float K_i, float K_d, Bivector error);
Bivector actuator_test(Bivector torque);
Bivector angle_from_torque(Bivector torque);

int main() {

#ifdef DEBUG
  printf("Debug Mode\n");
#else
  // printf("\x1b[?25l");
  printf("\x1b[2J");   // clear screen
  printf("\x1b[1;1H"); // place cursor in 1 1
#endif

#ifdef TEST
  test();
  return 0;
#endif // do not run the program when testing

  // Write date to csv file
  std::ofstream file("angle_err_over_time.csv");

  if (!file.is_open()) {
    std::cerr << "Failed to open file!" << std::endl;
    return 1;
  }

  file << "Time, Angle Error Norm, Angle Error e1e2,Angle Error e3e1, Angle "
          "Error e2e3,\n";

#ifdef DEBUG
  int samples = 1;
#else
  int samples = (int)(run_time / sampling_time);
#endif

  for (int i = 0; i < samples; i++) {
    float time = (float)i * sampling_time;
    printf("\nIteration: %d, Time: %f \n", i, (double)time);

    // "messure the sensors"
    measure();
    // normilize messurements to make the basis have approximently the same size
    normilize_measure();

    // Change of basis
    Vector third_basis_sat =
        vector_cross(sun_sensor_normed_sat, mag_sensor_normed_sat);
    // get change of basis matrix for vectors from WORLD to SAT
    cob_vec_from_world_to_sat = matrix_from_vectors(
        sun_sensor_normed_sat, mag_sensor_normed_sat, third_basis_sat);
    // get change of basis matrix for vectors from SAT to WORLD
    cob_vec_from_sat_to_world = matrix_inv(cob_vec_from_world_to_sat);

    // get the dual of the change of basis matrices
    cob_bivec_from_world_to_sat = matrix_dual(cob_vec_from_world_to_sat);
    cob_bivec_from_sat_to_world = matrix_dual(cob_vec_from_sat_to_world);

    // The inerita is defined in SAT
    // It is for bivectors
    inertia_world = matrix_mul(cob_bivec_from_sat_to_world, inertia_sat);

    // The reference is WORLD frame
    Vector reference_world = Vector{0, 1, 0};
    // the current postion is also fixed in SAT frame
    Vector current_world =
        matrix_vector_mul(cob_vec_from_sat_to_world, Vector{1, 0, 0});

    // mix the last calcutated point dir
    if (i != 0) {
      current_world = Vector{
          .e1 = (current_world.e1 + current_calc_world.e1) * 0.5f,
          .e2 = (current_world.e2 + current_calc_world.e2) * 0.5f,
          .e3 = (current_world.e3 + current_calc_world.e3) * 0.5f,
      };
    }

    printf("Current Vector WORLD:");
    printf("{ e1: %f,e2: %f,e3 %f }\n", (double)current_world.e1,
           (double)current_world.e2, (double)current_world.e3);

    Bivector angle_err_world =
        angle_difference_bivector(current_world, reference_world);

    printf("Angle Error WORLD: ");
    printf("%fe12, %fe31, %fe23, ", (double)angle_err_world.e12,
           (double)angle_err_world.e31, (double)angle_err_world.e23);
    float norm = sqrtf((angle_err_world.e12 * angle_err_world.e12)   //
                       + (angle_err_world.e31 * angle_err_world.e31) //
                       + (angle_err_world.e23 * angle_err_world.e23));
    printf("norm: %f\n\n", (double)norm);

    file << time;
    file << ", ";
    file << norm;
    file << ", ";
    file << angle_err_world.e12;
    file << ", ";
    file << angle_err_world.e31;
    file << ", ";
    file << angle_err_world.e23;
    file << "\n";

    Bivector pid_res_world = pid(0.4f, 0.0f, 1.0f, angle_err_world);
    Bivector torque_1_world = matrix_bivector_mul(inertia_world, pid_res_world);
#ifdef DEBUG
    printf("Torque Before Actuator: ");
    printf("%fe12, %fe31, %fe23, ", (double)torque_1_world.e12,
           (double)torque_1_world.e31, (double)torque_1_world.e23);
#endif

    Bivector torque_2_world =
        actuator_test(torque_1_world); // TODO something wrong

#ifdef DEBUG
    printf("Torque After actuator: ");
    printf("%fe12, %fe31, %fe23, ", (double)torque_2_world.e12,
           (double)torque_2_world.e31, (double)torque_2_world.e23);
#endif

    Bivector rotation_angle_world = angle_from_torque(torque_2_world);

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
  }

  file.close();
  std::cout << "CSV file created successfully." << std::endl;

  return 0;
}

void measure() {
  // rotate sensors in the opperise direction of the satellite
  Rotor rotor = rotor_form_halv_angle_bivector(
      scale_bivector(sensor_rotation_angle_sat, 0.5));
  sun_sensor_sat = rotate_vector(sun_sensor_sat, rotor);
  mag_sensor_sat = rotate_vector(mag_sensor_sat, rotor);
}

void normilize_measure() {
#ifdef DEBUG
  printf("\nNormilize Measurements:\n");
  printf("Sun Sensor: ");
  printf("{ e1: %f, e2: %f, e3: %f }\n", (double)sun_sensor_sat.e1,
         (double)sun_sensor_sat.e2, (double)sun_sensor_sat.e3);
#endif
  float sun_norm = sqrtf((sun_sensor_sat.e1 * sun_sensor_sat.e1) +
                         (sun_sensor_sat.e2 * sun_sensor_sat.e2) +
                         (sun_sensor_sat.e3 * sun_sensor_sat.e3));
#ifdef DEBUG
  printf("Sun Sensor Norm: %f\n", (double)sun_norm);
#endif
  sun_sensor_normed_sat = scale_vector(sun_sensor_sat, 1.0f / sun_norm);
#ifdef DEBUG
  printf("Sun Sensor Normilized:");
  printf("{ e1: %f,e2: %f,e3: %f }\n", (double)sun_sensor_normed_sat.e1,
         (double)sun_sensor_normed_sat.e2, (double)sun_sensor_normed_sat.e3);
#endif

#ifdef DEBUG
  printf("Mag Sensor: ");
  printf(" { e1: %f, e2: %f, e3: %fe23 }\n", (double)mag_sensor_sat.e1,
         (double)mag_sensor_sat.e2, (double)mag_sensor_sat.e3);
#endif
  float mag_norm = sqrtf((mag_sensor_sat.e1 * mag_sensor_sat.e1) +
                         (mag_sensor_sat.e2 * mag_sensor_sat.e2) +
                         (mag_sensor_sat.e3 * mag_sensor_sat.e3));
#ifdef DEBUG
  printf("Mag Sensor Norm: %f\n", (double)mag_norm);
#endif
  mag_sensor_normed_sat = scale_vector(mag_sensor_sat, 1.0f / mag_norm);
#ifdef DEBUG
  printf("Mag Sensor Normilized:");
  printf("{ e1: %f, e2: %f, e3: %f }\n", (double)mag_sensor_normed_sat.e1,
         (double)mag_sensor_normed_sat.e2, (double)mag_sensor_normed_sat.e3);
#endif
}

Bivector pid(float K_p, float K_i, float K_d, Bivector error) {

#ifdef DEBUG
  printf("pid consts:");
  printf(" k_p: %f", (double)K_p);
  printf(" k_i: %f", (double)K_i);
  printf(" k_d: %f\n", (double)K_d);
#endif

  // The PID controller is three part
  //
  // P is propertional
  // $$ K_p e$$
  //
  // I is intergral
  // $$ K_i \int e \mathrm{d}\! t$$
  // https://en.wikipedia.org/wiki/Numerical_integration
  //
  // d is derivitive
  // $$ K_p \frac{\mathrm{d}}{\mathrm{d}\!t} e$$
  // https://en.wikipedia.org/wiki/Numerical_differentiation
  //
  // The result is the sum of these three
  //
  // To implement these function there need to be memery between runs

  struct Bivector p = Bivector{
      K_p * error.e12,
      K_p * error.e31,
      K_p * error.e23,
  };

  // Intergration is adding up squares
  // The squares are time multiplied by function height

  last_int_world.e12 = last_int_world.e12 +
                       ((error.e12 + last_err_world.e12) / 2 * sampling_time);
  last_int_world.e31 = last_int_world.e31 +
                       ((error.e31 + last_err_world.e31) / 2 * sampling_time);
  last_int_world.e23 = last_int_world.e23 +
                       ((error.e23 + last_err_world.e23) / 2 * sampling_time);

  struct Bivector i = Bivector{
      K_i * last_int_world.e12,
      K_i * last_int_world.e31,
      K_i * last_int_world.e23,
  };

  // To calculate this remember
  // rise over run
  float delta_err_e12 = error.e12 - last_err_world.e12;
  float delta_err_e31 = error.e31 - last_err_world.e31;
  float delta_err_e23 = error.e23 - last_err_world.e23;

#ifdef DEBUG
  printf("delta error:\n");
  printf("  e12: %f\n", (double)delta_err_e12);
  printf("  e31: %f\n", (double)delta_err_e31);
  printf("  e23: %f\n", (double)delta_err_e23);
#endif

  struct Bivector d = Bivector{
      (K_d * delta_err_e12) / sampling_time,
      (K_d * delta_err_e31) / sampling_time,
      (K_d * delta_err_e23) / sampling_time,
  };

#ifdef DEBUG
  printf("pid d-part:\n");
  printf("  e12: %f\n", (double)d.e12);
  printf("  e31: %f\n", (double)d.e31);
  printf("  e23: %f\n", (double)d.e23);
#endif

  // reset last error
  last_err_world = error;

  struct Bivector result = Bivector{
      p.e12 + i.e12 + d.e12,
      p.e31 + i.e31 + d.e31,
      p.e23 + i.e23 + d.e23,
  };

#ifdef DEBUG
  printf("pid res\n");
  printf("  e12: %f\n", (double)result.e12);
  printf("  e31: %f\n", (double)result.e31);
  printf("  e23: %f\n", (double)result.e23);
#endif

  return result;
}

int compare_float(const void *a, const void *b) {
  float fa = *(const float *)a;
  float fb = *(const float *)b;
  return (fa > fb) - (fa < fb);
}

Bivector actuator_test(Bivector torque) {
#ifdef DEBUG
  FNPRINT("\nActuator Model:\n");
#endif

  // The magnetic dipole moment need to be calculated
  // torque = - mag_dipole cross mag_flux
  //
  // There infinetly many corrent magnetic dipole moment.
  // The magnetic dipole moment and magnetic flux density are always
  // orthogonal Then there is only one.
  //
  // mag_dipole = mag_flux cross torque / |mag_flux|^2

  // $$A A^\dag$$
  Bivector magnetric_flux_density_world = matrix_bivector_mul(
      cob_bivec_from_sat_to_world, magnetric_flux_density_sat);

  float mag_flux_norm_square =
      (magnetric_flux_density_world.e12 * magnetric_flux_density_world.e12) +
      (magnetric_flux_density_world.e31 * magnetric_flux_density_world.e31) +
      (magnetric_flux_density_world.e23 * magnetric_flux_density_world.e23);

  Bivector torque_scaled = Bivector{
      .e12 = torque.e12 / mag_flux_norm_square,
      .e31 = torque.e31 / mag_flux_norm_square,
      .e23 = torque.e23 / mag_flux_norm_square,
  };

  // The negation can be removed by switching the places
  Bivector mag_dipole =
      bivector_cross(torque_scaled, magnetric_flux_density_world);

  // translate magnetorquers to WORLD
  Bivector mag1_world =
      matrix_bivector_mul(cob_bivec_from_sat_to_world, mag1_sat);
  Bivector mag2_world =
      matrix_bivector_mul(cob_bivec_from_sat_to_world, mag2_sat);
  Bivector mag3_world =
      matrix_bivector_mul(cob_bivec_from_sat_to_world, mag3_sat);

  Matrix3x3 mag_dipol_matrix_world =
      matrix_from_bivectors(mag1_sat, mag2_sat, mag3_sat);

  // get scalaer.
  // The scalers are not part of a bivector, this is just the easiest impl
  Bivector scalers =
      matrix_bivector_mul(matrix_inv(mag_dipol_matrix_world), mag_dipole);

  // test if scalars are between -1 and 1
  // square them, sort, find the biggest,
  // if bigger 1 then scale all down by sqrt
  float scalar_list[] = {
      (scalers.e12 * scalers.e12),
      (scalers.e31 * scalers.e31),
      (scalers.e23 * scalers.e23),
  };
#ifdef DEBUG
  FNPRINT("Mag scalar before sort:\n");
  FNPRINT("  %f, %f, %f\n", (double)scalar_list[0], (double)scalar_list[1],
          (double)scalar_list[2]);
#endif
  qsort(scalar_list, sizeof(scalar_list) / sizeof(scalar_list[0]),
        sizeof(scalar_list[0]), compare_float);
#ifdef DEBUG
  FNPRINT("Mag scalar after sort:\n");
  FNPRINT("  %f, %f, %f\n", (double)scalar_list[0], (double)scalar_list[1],
          (double)scalar_list[2]);
#endif

  if (scalar_list[2] > 1.0f) {
#ifdef DEBUG
    FNPRINT("scalars are too big\n");
#endif
    float max_scaler_sqrt = sqrtf(scalar_list[2]);
    mag_s1 = scalers.e12 / max_scaler_sqrt;
    mag_s2 = scalers.e31 / max_scaler_sqrt;
    mag_s3 = scalers.e23 / max_scaler_sqrt;

    Bivector new_mag_world = Bivector{
        .e12 = (mag_s1 * mag1_world.e12) + (mag_s2 * mag2_world.e12) +
               (mag_s3 * mag3_world.e12),
        .e31 = (mag_s1 * mag1_world.e31) + (mag_s2 * mag2_world.e31) +
               (mag_s3 * mag3_world.e31),
        .e23 = (mag_s1 * mag1_world.e23) + (mag_s2 * mag2_world.e23) +
               (mag_s3 * mag3_world.e23),
    };

    // torque = - mag_dipole cross mag_flux
    // switch place to get negate
    return bivector_cross(magnetric_flux_density_world, new_mag_world);

  } else {
#ifdef DEBUG
    FNPRINT("scalars are just fine\n");
#endif
    mag_s1 = scalers.e12;
    mag_s2 = scalers.e31;
    mag_s3 = scalers.e23;
    return torque;
  }
}

Bivector angle_from_torque(Bivector torque) {

#ifdef DEBUG
  FNPRINT("\nGet the Rotation angle:\n");
#endif
  // kinematic equations
  // theta = 0.5 * inerita^-1 torque * time^2 + angular_vel * time + 0

  Bivector angular_acc_world =
      matrix_bivector_mul(matrix_inv(inertia_world), torque);
#ifdef DEBUG
  FNPRINT("  angular acceleration:\n");
  FNPRINT("  e12: %f, e31: %f, e23: %f\n", (double)angular_acc_world.e12,
          (double)angular_acc_world.e31, (double)angular_acc_world.e23);
#endif

  // ang_vel = ang_acc * delta_time + ang_vel_0
  angular_velocity_world.e12 =
      angular_acc_world.e12 * sampling_time + angular_velocity_world.e12;
  angular_velocity_world.e31 =
      angular_acc_world.e31 * sampling_time + angular_velocity_world.e31;
  angular_velocity_world.e23 =
      angular_acc_world.e23 * sampling_time + angular_velocity_world.e23;

#ifdef DEBUG
  FNPRINT("  angular velocity:\n");
  FNPRINT("  e12: %f, e31: %f, e23: %f\n", (double)angular_velocity_world.e12,
          (double)angular_velocity_world.e31,
          (double)angular_velocity_world.e23);
#endif

  float samp_square = sampling_time * sampling_time;

#ifdef DEBUG
  FNPRINT("  Sample time squared: %f", (double)samp_square);
#endif

  Bivector rotation_angle_world = Bivector{
      .e12 = (0.5f * angular_acc_world.e12 * samp_square) +
             (angular_velocity_world.e12 * sampling_time),
      .e31 = (0.5f * angular_acc_world.e31 * samp_square) +
             (angular_velocity_world.e31 * sampling_time),
      .e23 = (0.5f * angular_acc_world.e23 * samp_square) +
             (angular_velocity_world.e23 * sampling_time),
  };

#ifdef DEBUG
  FNPRINT("  rotation angle:\n");
  FNPRINT("  e12: %f, e31: %f, e23: %f\n", (double)rotation_angle_world.e12,
          (double)rotation_angle_world.e31, (double)rotation_angle_world.e23);
#endif

  return rotation_angle_world;
}
