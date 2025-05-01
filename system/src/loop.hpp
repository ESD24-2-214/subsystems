#ifndef LOOP_H_
#define LOOP_H_
#include "math_stuff.hpp"

// Functions
void measure(Bivector sensor_rotation_angle_sat, Vector &sun_sensor_sat,
             Vector &mag_sensor_sat);

void cob_from_measure(Matrix3x3 &cob_vec_from_world_to_sat,
                      Matrix3x3 &cob_vec_from_sat_to_world,
                      Matrix3x3 &cob_bivec_from_world_to_sat,
                      Matrix3x3 &cob_bivec_from_sat_to_world,
                      Vector sun_sensor_sat, Vector mag_sensor_sat);

Bivector pid(float K_p, float K_i, float K_d, Bivector error,
             Bivector &last_int_world, Bivector &last_err_world,
             float delta_time);

Bivector actuator_test(Bivector torque, Matrix3x3 cob_bivec_from_sat_to_world,
                       Bivector magnetric_flux_density_sat, Bivector mag1_sat,
                       Bivector mag2_sat, Bivector mag3_sat, float &mag_s1,
                       float &mag_s2, float &mag_s3);

Bivector angle_from_torque(Bivector torque, Matrix3x3 inertia_frame1,
                           Bivector &angular_velocity_frame1, float delta_time);

#endif // LOOP_H_
