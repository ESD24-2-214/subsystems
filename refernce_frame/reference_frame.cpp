#include "math_stuff.hpp"
#include <cmath>
#include <cstdio>

// Macros
#define FNPRINT(...)                                                           \
  printf("  ");                                                                \
  printf(__VA_ARGS__)

// WHAT ARE WE DOING?
//
// In our sat there are two reference frames.
// SAT and WORLD
// When working with vector or bivector they all need to be in the in the same
// frame (WORLD) These frames are at the same point however they are rotated.
//
// PLAN:
//
// To define a refernce Frame three linear independent basis are needed.
// In our test there will be a fixed sun(lamp) and a fixed magnetic field.
// If these to direcetions are linear independented, then a thrid basis
// can be found using the cross product.
//
// 1. messure sundirection in SAT
// 2. messure magnetic field direction in SAT
// 3. use these two to create a change-of-basis matrix
// from WORLD to SAT
// 4. Inverse to get a change-of-basis matrix
// from SAT to WORLD
//
// PS. This change-of-basis matrix is for vectors.
// The change-of-basis matrix for bivectors is found by
// mirroring the matrix about the secondary diagonal
//
// 5. translate everything into WORLD
//

// Global Constants

// Sunsensor
// The Sunsensors are in Sat frame
// North = +e1, South = -e1, West = +e2, East = -e2, Up = +e3, Down = -e3
// enum SunSensor { Forward, Back, Left, Right, Up, Down };
// float SunSensorArray[] = {Forward, Back, Left, Right, Up, Down};
Vector sun_direction_sat{
    .e1 = 0.7071f,
    .e2 = -0.7071f,
    .e3 = 0.0f,
};

// Magnetometer
// This is the e2 is WORLD
Vector magnet_direction_sat{
    .e1 = 0.7071f,
    .e2 = 0.7071f,
    .e3 = 0.0f,
};

// Rotation angle from iteration
Bivector rotation_angle{
    .e12 = 0.0f,
    .e31 = 0.0f,
    .e23 = 0.0f,
};

// Functions
void set_sun_sensor();
void set_mag_sensor();
Vector get_sun_direction();
Vector get_mag_direction();
Bivector angle_difference_bivector(Vector a, Vector b);

int main() {

  // Set the sensors for next iteration
  // In the real test this would be measurred
  set_sun_sensor();
  set_mag_sensor();

  // get the directions
  Vector sun_direction_sat = get_sun_direction();
  printf("Sun Direction: Vector { ");
  printf("e1: %f ", (double)sun_direction_sat.e1);
  printf("e2: %f ", (double)sun_direction_sat.e2);
  printf("e3: %f ", (double)sun_direction_sat.e3);
  printf("}\n");

  Vector mag_direction_sat = get_mag_direction();
  printf("Mag Direction: Vector { ");
  printf("e1: %f ,e2: %f, e3: %f }\n", (double)mag_direction_sat.e1,
         (double)mag_direction_sat.e2, (double)mag_direction_sat.e3);

  // get the third basis
  Vector third_basis_sat = vector_cross(sun_direction_sat, mag_direction_sat);
  printf("Third Basis direction: Vector { ");
  printf("e1: %f ,e2: %f, e3: %f }\n", (double)third_basis_sat.e1,
         (double)third_basis_sat.e2, (double)third_basis_sat.e3);

  // get change of basis matrix for vectors from WORLD to SAT
  Matrix3x3 cob_vec_from_world_to_sat = matrix_from_vectors(
      sun_direction_sat, mag_direction_sat, third_basis_sat);
  // get change of basis matrix for vectors from SAT to WORLD
  Matrix3x3 cob_vec_from_sat_to_world = matrix_inv(cob_vec_from_world_to_sat);

  // Translate sat pointing dir to WORLD
  Vector current_dir_world = matrix_vector_mul(
      cob_vec_from_sat_to_world, Vector{.e1 = 1.0f, .e2 = 0.0f, .e3 = 0.0f});
  printf("Current dir world: Vector{ e1: %f, e2: %f, e3: %f }\n",
         (double)current_dir_world.e1, (double)current_dir_world.e2,
         (double)current_dir_world.e3);

  // dual the change of basis from SAT to WORLD to work for bivectors
  Matrix3x3 cob_bivec_from_sat_to_world =
      matrix_dual(cob_vec_from_sat_to_world);

  // translate magnetorquer SAT to WORLD
  Bivector magnetorquer1_world = matrix_bivector_mul(
      cob_bivec_from_sat_to_world, Bivector{0.0f, 0.02f, 0.0f});
  printf("Magnetorquer 1 world: Bivector { e12: %f, e31: %f, e23: %f }\n",
         (double)magnetorquer1_world.e12, (double)magnetorquer1_world.e31,
         (double)magnetorquer1_world.e23);

  return 0;
}

void set_sun_sensor() {
  // Rotate in the oppisite direction
  Rotor rotor =
      rotor_form_halv_angle_bivector(scale_bivector(rotation_angle, -0.5));
  Vector sun_sensor_new = rotate_vector(sun_direction_sat, rotor);
  sun_direction_sat.e1 = sun_sensor_new.e1;
  sun_direction_sat.e2 = sun_sensor_new.e2;
  sun_direction_sat.e3 = sun_sensor_new.e3;
}

void set_mag_sensor() {
  // Rotate in the oppisite direction
  Rotor rotor =
      rotor_form_halv_angle_bivector(scale_bivector(rotation_angle, -0.5));
  Vector mag_sensor_new = rotate_vector(magnet_direction_sat, rotor);
  magnet_direction_sat.e1 = mag_sensor_new.e1;
  magnet_direction_sat.e2 = mag_sensor_new.e2;
  magnet_direction_sat.e3 = mag_sensor_new.e3;
}

Vector get_sun_direction() { return sun_direction_sat; }
Vector get_mag_direction() { return magnet_direction_sat; }
