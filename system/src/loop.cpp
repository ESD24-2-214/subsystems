#include "loop.hpp"
#include "math_stuff.hpp"
#include <cmath>

void measure(Bivector sensor_rotation_angle_sat, Vector &sun_sensor_sat,
             Vector &mag_sensor_sat) {
  // Measure the sensors

  // rotate sensors in the opperise direction of the satellite
  Rotor rotor = rotor_form_halv_angle_bivector(
      scale_bivector(sensor_rotation_angle_sat, 0.5));
  sun_sensor_sat = rotate_vector(sun_sensor_sat, rotor);
  mag_sensor_sat = rotate_vector(mag_sensor_sat, rotor);
}

void cob_from_measure(Matrix3x3 &cob_vec_from_world_to_sat,
                      Matrix3x3 &cob_vec_from_sat_to_world,
                      Matrix3x3 &cob_bivec_from_world_to_sat,
                      Matrix3x3 &cob_bivec_from_sat_to_world,
                      Vector sun_sensor_sat, Vector mag_sensor_sat) {

#ifdef DEBUG
  printf("\nChange of Basis Matrix from Measurements:\n");
  printf("Sun Sensor: ");
  printf("{ e1: %f, e2: %f, e3: %f }\n", (double)sun_sensor_sat.e1,
         (double)sun_sensor_sat.e2, (double)sun_sensor_sat.e3);
  printf("Mag Sensor: ");
  printf(" { e1: %f, e2: %f, e3: %fe23 }\n", (double)mag_sensor_sat.e1,
         (double)mag_sensor_sat.e2, (double)mag_sensor_sat.e3);

#endif

  // Get orthononormal basis set from the sunsensor and magnetometer
  // The sunsensor is the e1 basis the rest is defined from there
  //
  // Plan:
  // 1. normize sun
  // 2. get e3 from the cross product of sun and mag
  // 3. normilize e3
  // 4. get e2 from cross product between e3 and sun
  // (e2 should be normal)
  // 6. make the change of basis matrix

  float sun_norm = sqrtf((sun_sensor_sat.e1 * sun_sensor_sat.e1) +
                         (sun_sensor_sat.e2 * sun_sensor_sat.e2) +
                         (sun_sensor_sat.e3 * sun_sensor_sat.e3));
#ifdef DEBUG
  printf("Sun Sensor Norm: %f\n", (double)sun_norm);
#endif
  Vector e1_basis_sat = scale_vector(sun_sensor_sat, 1.0f / sun_norm);
#ifdef DEBUG
  printf("Sun Sensor Normilized:");
  printf("{ e1: %f,e2: %f,e3: %f }\n", (double)e1_basis_sat.e1,
         (double)e1_basis_sat.e2, (double)e1_basis_sat.e3);
#endif

  // e3
  Vector e3_basis_not_normed_sat = vector_cross(e1_basis_sat, mag_sensor_sat);

  float e3_basis_norm_sat =
      sqrtf((e3_basis_not_normed_sat.e1 * e3_basis_not_normed_sat.e1) +
            (e3_basis_not_normed_sat.e2 * e3_basis_not_normed_sat.e2) +
            (e3_basis_not_normed_sat.e3 * e3_basis_not_normed_sat.e3));
#ifdef DEBUG
  printf("World e3 basis Norm: %f\n", (double)e3_basis_norm_sat);
#endif

  Vector e3_basis_sat =
      scale_vector(e3_basis_not_normed_sat, 1.0f / e3_basis_norm_sat);

#ifdef DEBUG
  printf("World e3 Normilized in SAT:");
  printf("{ e1: %f,e2: %f,e3: %f }\n", (double)e3_basis_sat.e1,
         (double)e3_basis_sat.e2, (double)e3_basis_sat.e3);
#endif

  Vector e2_basis_sat = vector_cross(e3_basis_sat, e1_basis_sat);

#ifdef DEBUG
  float e2_basis_norm_sat = sqrtf((e2_basis_sat.e1 * e2_basis_sat.e1) +
                                  (e2_basis_sat.e2 * e2_basis_sat.e2) +
                                  (e2_basis_sat.e3 * e2_basis_sat.e3));
  printf("World e2 basis Norm: %f\n", (double)e2_basis_norm_sat);
#endif

  // get change of basis matrix for vectors from WORLD to SAT
  cob_vec_from_world_to_sat =
      matrix_from_vectors(e1_basis_sat, e2_basis_sat, e3_basis_sat);
  // get change of basis matrix for vectors from SAT to WORLD
  cob_vec_from_sat_to_world = matrix_inv(cob_vec_from_world_to_sat);

  // get the dual of the change of basis matrices
  cob_bivec_from_world_to_sat = matrix_dual(cob_vec_from_world_to_sat);
  cob_bivec_from_sat_to_world = matrix_dual(cob_vec_from_sat_to_world);
}

Bivector pid(float K_p, float K_i, float K_d, Bivector error,
             Bivector &last_int_world, Bivector &last_err_world,
             float delta_time) {

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

  last_int_world.e12 =
      last_int_world.e12 + ((error.e12 + last_err_world.e12) / 2 * delta_time);
  last_int_world.e31 =
      last_int_world.e31 + ((error.e31 + last_err_world.e31) / 2 * delta_time);
  last_int_world.e23 =
      last_int_world.e23 + ((error.e23 + last_err_world.e23) / 2 * delta_time);

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
      (K_d * delta_err_e12) / delta_time,
      (K_d * delta_err_e31) / delta_time,
      (K_d * delta_err_e23) / delta_time,
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

Bivector actuator_test(Bivector torque, Matrix3x3 cob_bivec_from_sat_to_world,
                       Bivector magnetric_flux_density_sat, Bivector mag1_sat,
                       Bivector mag2_sat, Bivector mag3_sat,
                       float &mag_s_for_e12, float &mag_s_for_e31,
                       float &mag_s_for_e23) {

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
    mag_s_for_e12 = scalers.e12 / max_scaler_sqrt;
    mag_s_for_e31 = scalers.e31 / max_scaler_sqrt;
    mag_s_for_e23 = scalers.e23 / max_scaler_sqrt;

    Bivector new_mag_world = Bivector{
        .e12 = (mag_s_for_e12 * mag1_world.e12) +
               (mag_s_for_e31 * mag2_world.e12) +
               (mag_s_for_e23 * mag3_world.e12),
        .e31 = (mag_s_for_e12 * mag1_world.e31) +
               (mag_s_for_e31 * mag2_world.e31) +
               (mag_s_for_e23 * mag3_world.e31),
        .e23 = (mag_s_for_e12 * mag1_world.e23) +
               (mag_s_for_e31 * mag2_world.e23) +
               (mag_s_for_e23 * mag3_world.e23),
    };

    // torque = - mag_dipole cross mag_flux
    // switch place to get negate
    return bivector_cross(magnetric_flux_density_world, new_mag_world);

  } else {
#ifdef DEBUG
    FNPRINT("scalars are just fine\n");
#endif
    mag_s_for_e12 = scalers.e12;
    mag_s_for_e31 = scalers.e31;
    mag_s_for_e23 = scalers.e23;
    return torque;
  }
}

Bivector angle_from_torque(Bivector torque, Matrix3x3 inertia_frame1,
                           Bivector &angular_velocity_frame1,
                           float delta_time) {

#ifdef DEBUG
  FNPRINT("\nGet the Rotation angle:\n");
#endif
  // kinematic equations
  // theta = 0.5 * inerita^-1 torque * time^2 + angular_vel * time + 0

  Bivector angular_acc_frame1 =
      matrix_bivector_mul(matrix_inv(inertia_frame1), torque);
#ifdef DEBUG
  FNPRINT("  angular acceleration:\n");
  FNPRINT("  e12: %f, e31: %f, e23: %f\n", (double)angular_acc_frame1.e12,
          (double)angular_acc_frame1.e31, (double)angular_acc_frame1.e23);
#endif

  // ang_vel = ang_acc * delta_time + ang_vel_0
  angular_velocity_frame1.e12 =
      angular_acc_frame1.e12 * delta_time + angular_velocity_frame1.e12;
  angular_velocity_frame1.e31 =
      angular_acc_frame1.e31 * delta_time + angular_velocity_frame1.e31;
  angular_velocity_frame1.e23 =
      angular_acc_frame1.e23 * delta_time + angular_velocity_frame1.e23;

#ifdef DEBUG
  FNPRINT("  angular velocity:\n");
  FNPRINT("  e12: %f, e31: %f, e23: %f\n", (double)angular_velocity_frame1.e12,
          (double)angular_velocity_frame1.e31,
          (double)angular_velocity_frame1.e23);
#endif

  float samp_square = delta_time * delta_time;

#ifdef DEBUG
  FNPRINT("  Sample time squared: %f", (double)samp_square);
#endif

  Bivector rotation_angle_frame1 = Bivector{
      .e12 = (0.5f * angular_acc_frame1.e12 * samp_square) +
             (angular_velocity_frame1.e12 * delta_time),
      .e31 = (0.5f * angular_acc_frame1.e31 * samp_square) +
             (angular_velocity_frame1.e31 * delta_time),
      .e23 = (0.5f * angular_acc_frame1.e23 * samp_square) +
             (angular_velocity_frame1.e23 * delta_time),
  };

#ifdef DEBUG
  FNPRINT("  rotation angle:\n");
  FNPRINT("  e12: %f, e31: %f, e23: %f\n", (double)rotation_angle_frame1.e12,
          (double)rotation_angle_frame1.e31, (double)rotation_angle_frame1.e23);
#endif

  return rotation_angle_frame1;
}
