#include "loop.hpp"
#include "math_stuff.hpp"
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <string>
// Macros
#define FNPRINT(...)                                                           \
  printf("  ");                                                                \
  printf(__VA_ARGS__)

// Global Variables
float sampling_time = 0.100f; // seconds
float run_time = 15.0f;       // second

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

int main() {

  for (int j = 0; j < 8; j++) {

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

    float mag_s1 = 0.0f;
    float mag_s2 = 0.0f;
    float mag_s3 = 0.0f;

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
    std::stringstream filename;
    filename << "control_loop-" << j << ".csv";
    std::ofstream file(filename.str());

    if (!file.is_open()) {
      std::cerr << "Failed to open file!" << std::endl;
      return 1;
    }

    file << "Time, Angle Error Norm,"
            "Angle Error e1e2,Angle Error e3e1, Angle Error e2e3,"
            "ref e1 world, ref e2 world, ref e3 world,"
            "cur e1 world, cur e2 world, cur e3 world\n";

    // #ifdef DEBUG
    //     int samples = 1;
    // #else
    int samples = (int)(run_time / sampling_time);
    // #endif

    /// Sensors
    // sun sensor
    // This is messured
    // Used to create WORLD
    Vector sun_sensor_sat{
        .e1 = 0.7071f,
        .e2 = 0.7071f,
        .e3 = 0.0f,
    };

    // Magnetic Flux Density
    // Messured in MAG frame
    // Translated to SAT frame
    // Used to create WORLD
    float magnetric_flux_density_norm = 0.00002f; // 20 * 10^(-6) tesla
    Vector mag_sensor_sat{
        .e1 = 0.7071f * magnetric_flux_density_norm,
        .e2 = -0.7071f * magnetric_flux_density_norm,
        .e3 = 0.0f,
    };

    Bivector magnetric_flux_density_sat =
        dual_vector_to_bivector(mag_sensor_sat);

    // The reference is WORLD frame
    Vector reference_world = Vector{cosf(1.0f / 8.0f * (float)TAU * j),
                                    sinf(1.0f / 8.0f * (float)TAU * j), 0.0f};

    for (int i = 0; i < samples; i++) {
      float time = (float)i * sampling_time;
      printf("\nRun: %d, Iteration: %d, Time: %f \n", j, i, (double)time);

      // "messure the sensors"
      measure(sensor_rotation_angle_sat, sun_sensor_sat, mag_sensor_sat);
      // make change of basis matrix from messurements
      cob_from_measure(cob_vec_from_world_to_sat, cob_vec_from_sat_to_world,
                       cob_bivec_from_world_to_sat, cob_bivec_from_sat_to_world,
                       sun_sensor_sat, mag_sensor_sat);

      // The inerita is defined in SAT
      // It is for bivectors
      inertia_world = matrix_mul(cob_bivec_from_sat_to_world, inertia_sat);

      // the current postion is also fixed in SAT frame
      Vector current_world =
          matrix_vector_mul(cob_vec_from_sat_to_world, Vector{1, 0, 0});

      // mix the last calcutated point dir
      // if (i != 0) {
      //   current_world = Vector{
      //       .e1 = (current_world.e1 + current_calc_world.e1) * 0.5f,
      //       .e2 = (current_world.e2 + current_calc_world.e2) * 0.5f,
      //       .e3 = (current_world.e3 + current_calc_world.e3) * 0.5f,
      //   };
      // }

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
      file << ", ";
      file << reference_world.e1;
      file << ", ";
      file << reference_world.e2;
      file << ", ";
      file << reference_world.e3;
      file << ", ";
      file << current_world.e1;
      file << ", ";
      file << current_world.e2;
      file << ", ";
      file << current_world.e3;
      file << "\n";

      Bivector pid_res_world =
          pid(0.4f, 0.0f, 1.1f, angle_err_world, last_int_world, last_err_world,
              sampling_time);

      Bivector torque_1_world =
          matrix_bivector_mul(inertia_world, pid_res_world);
#ifdef DEBUG
      printf("Torque Before Actuator: ");
      printf("%fe12, %fe31, %fe23, ", (double)torque_1_world.e12,
             (double)torque_1_world.e31, (double)torque_1_world.e23);
#endif

      Bivector torque_2_world =
          actuator_test(torque_1_world, cob_bivec_from_sat_to_world,
                        magnetric_flux_density_sat, mag1_sat, mag2_sat,
                        mag3_sat, mag_s1, mag_s2, mag_s3);

#ifdef DEBUG
      printf("Torque After actuator: ");
      printf("%fe12, %fe31, %fe23, ", (double)torque_2_world.e12,
             (double)torque_2_world.e31, (double)torque_2_world.e23);
#endif

      Bivector rotation_angle_world = angle_from_torque(
          torque_2_world, inertia_world, angular_velocity_world, sampling_time);

      // Set rotation angle for sensor
      // It is in SAT
      // the sensors must rotated the oppiste direction of the sat
      sensor_rotation_angle_sat =
          scale_bivector(matrix_bivector_mul(cob_bivec_from_world_to_sat,
                                             rotation_angle_world),
                         -1.0);

      // find the next point direction
      Rotor rotor = rotor_form_halv_angle_bivector(
          scale_bivector(rotation_angle_world, 0.5));
      current_calc_world = rotate_vector(current_world, rotor);
    }

    file.close();
    std::cout << "CSV file created successfully." << std::endl;
  }
  return 0;
}
