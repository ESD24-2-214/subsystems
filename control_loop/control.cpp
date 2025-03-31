#include "math_stuff.hpp"
#include <cmath>
#include <cstdio>
#include <stdlib.h>

// Macros
#define FNPRINT(...)                                                           \
  printf("  ");                                                                \
  printf(__VA_ARGS__)

// Global Variables
float sampling_time = 0.200f; // seconds

// pid
float last_int_e12 = 0; // The last derivetive variable
float last_int_e31 = 0; // The last derivetive variable
float last_int_e23 = 0; // The last derivetive variable

float last_err_e12 = 0; // The last derivetive variable
float last_err_e31 = 0; // The last derivetive variable
float last_err_e23 = 0; // The last derivetive variable

// inertia matrix
Matrix3x3 inertia{
    .m11 = 1.0f,
    .m12 = 0.0f,
    .m13 = 0.0f,

    .m21 = 0.0f,
    .m22 = 1.0f,
    .m23 = 0.0f,

    .m31 = 0.0f,
    .m32 = 0.0f,
    .m33 = 1.0f,
};
Matrix3x3 inertia_inv = matrix_inv(inertia);

// Magnetorquers
float magnetorquer_dipole = 4; // ampere meter^2
float mag_s1 = 0.0f;
float mag_s2 = 0.0f;
float mag_s3 = 0.0f;
Bivector mag1 = Bivector{
    .e12 = magnetorquer_dipole,
    .e31 = 0.0f,
    .e23 = 0.0f,
};
Bivector mag2 = Bivector{
    .e12 = 0.0f,
    .e31 = magnetorquer_dipole,
    .e23 = 0.0f,
};
Bivector mag3 = Bivector{
    .e12 = 0.0f,
    .e31 = 0.0f,
    .e23 = 0.0f,
};

// Magnetic Flux Density
Bivector magnetrix_flux_density{
    .e12 = 2.0f, // tesla
    .e31 = 0.0f, // tesla
    .e23 = 0.0f, // tesla
};

// Functions
Bivector angle_err_bivector(Vector current, Vector reference);
Bivector pid(float K_p, float K_i, float K_d, Bivector error);
Bivector actuator_test(Bivector torque, Bivector magnetrix_flux_density);

void test();

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
#endif

  Vector reference = Vector{1, 0, 0};
  Vector current = Vector{0, 1, 0};

  Bivector angle_err = angle_err_bivector(current, reference);
  Bivector pid_res = pid(0.2f, 0.0f, 0.2f, angle_err);
  Bivector torque_1 = matrix_bivector_mul(inertia, pid_res);

  while (1) {
  }

  return 0;
}

Bivector angle_err_bivector(Vector current, Vector reference) {

  // Find the angle error bivector betweem the current vector and refernce
  // $$ \overset\Rightarrow{\theta}_\text{err} =
  // \frac{\vec{y}\wedge \vec{r}}{|\vec{y}\wedge \vec{r}|}
  // \arccos \left( \frac{|\vec{y}\cdot \vec{r}|}{|\vec{y}| |\vec{r}|}  \right)
  // $$

  // There are three cases
  // 1 They are parallel and the same direction. There the error angle is 0 with
  // no plane 2 They are anti parallet in the opesite direction. The error angle
  // is pi with no plane ( give a plane) 3 anything in between. Has both plane
  // and error

  // $$ \vec{y}\cdot \vec{r} $$
  float inner_product = (current.e1 * reference.e1)   //
                        + (current.e2 * reference.e2) //
                        + (current.e3 * reference.e3);

  // Norm of inner product
  // $$A A^{\dag}$$
  float current_norm = sqrtf(float((current.e1 * current.e1)   //
                                   + (current.e2 * current.e2) //
                                   + (current.e3 * current.e3)));
  float reference_norm = sqrtf(float((reference.e1 * reference.e1)   //
                                     + (reference.e2 * reference.e2) //
                                     + (reference.e3 * reference.e3)));
  float inner_norm = inner_product / (current_norm * reference_norm);

#ifdef DEBUG
  FNPRINT("inner norm: %f\n", (double)inner_norm);
#endif

  // if (inner_product > 0.90000000f) { // parallel
  if (inner_norm >= 1.00f && inner_norm <= 1.01f) { // parallel
#ifdef DEBUG
    FNPRINT("parallel path\n");
#endif
    FNPRINT("inner norm: %f\n", (double)inner_norm);

    return Bivector{0.0, 0.0, 0.0};
  } else if (inner_norm >= -1.01f && inner_norm <= -1.0f) { // anti
                                                            // parallel
#ifdef DEBUG
    FNPRINT("anti parallel path\n");
#endif

    return Bivector{3.14159274f, 0.0, 0.0};
  } else { // everything else
#ifdef DEBUG
    FNPRINT("general path\n");
#endif

    // $$ \vec{y}\wedge \vec{r} $$
    Bivector plane = {
        .e12 = current.e1 * reference.e2 - current.e2 * reference.e1,
        .e31 = current.e3 * reference.e1 - current.e1 * reference.e3,
        .e23 = current.e2 * reference.e3 - current.e3 * reference.e2,
    };

    // Norm of exterior product
    // $$A A^{\dag}$$
    float plane_area = sqrtf(float((plane.e12 * plane.e12)   //
                                   + (plane.e31 * plane.e31) //
                                   + (plane.e23 * plane.e23)));

    // $$\arccos\left(\frac{|\vec{y}\cdot\vec{r}|}{|\vec{y}||\vec{r}|}\right)$$
    float angle_scalar = acosf(float(inner_norm));

#ifdef DEBUG
    FNPRINT("angle scalar: %f, degree: %f\n", (double)angle_scalar,
            (double)(angle_scalar * 180.0f / 3.14159274f));
#endif

    // scales plane area to 1 and set the angle
    float scalar = angle_scalar / plane_area;

    Bivector angle = Bivector{
        plane.e12 * scalar,
        plane.e31 * scalar,
        plane.e23 * scalar,
    };

#ifdef DEBUG
    FNPRINT("angle err\n");
    printf("  e12: %f\n", (double)angle.e12);
    printf("  e31: %f\n", (double)angle.e31);
    printf("  e23: %f\n", (double)angle.e23);
#endif
    return angle;
  }
}

Bivector pid(float K_p, float K_i, float K_d, Bivector error) {

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

  last_int_e12 =
      last_int_e12 + ((error.e12 + last_err_e12) / 2 * sampling_time);
  last_int_e31 =
      last_int_e31 + ((error.e31 + last_err_e12) / 2 * sampling_time);
  last_int_e23 =
      last_int_e23 + ((error.e23 + last_err_e12) / 2 * sampling_time);

  struct Bivector i = Bivector{
      K_i * last_int_e12,
      K_i * last_int_e31,
      K_i * last_int_e23,
  };

  // To calculate this remember
  // rise over run
  float delta_err_e12 = last_err_e12 - error.e12;
  float delta_err_e31 = last_err_e31 - error.e31;
  float delta_err_e23 = last_err_e23 - error.e23;

  struct Bivector d = Bivector{
      K_d * delta_err_e12 / sampling_time,
      K_d * delta_err_e31 / sampling_time,
      K_d * delta_err_e23 / sampling_time,
  };

  // reset last error
  last_err_e12 = error.e12;
  last_err_e31 = error.e31;
  last_err_e23 = error.e23;

  struct Bivector result = Bivector{
      p.e12 + i.e31 + d.e31,
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

Bivector actuator_test(Bivector torque, Bivector magnetrix_flux_density) {

  // The magnetic dipole moment need to be calculated
  // torque = - mag_dipole cross mag_flux
  //
  // There infinetly many corrent magnetic dipole moment.
  // The magnetic dipole moment and magnetic flux density are always orthogonal
  // Then there is only one.
  //
  // mag_dipole = - mag_flux cross torque / |mag_flux|^2

  // $$A A^\dag$$
  float mag_flux_norm_square =
      (magnetrix_flux_density.e12 * magnetrix_flux_density.e12) +
      (magnetrix_flux_density.e31 * magnetrix_flux_density.e31) +
      (magnetrix_flux_density.e23 * magnetrix_flux_density.e23);

  Bivector torque_scaled = Bivector{
      .e12 = torque.e12 / mag_flux_norm_square,
      .e31 = torque.e31 / mag_flux_norm_square,
      .e23 = torque.e23 / mag_flux_norm_square,
  };

  // The negation can be removed by switching the places
  Bivector mag_dipole = bivector_cross(torque_scaled, magnetrix_flux_density);

  Matrix3x3 mag_dipol_matrix = Matrix3x3{
      .m11 = mag1.e12,
      .m12 = mag2.e12,
      .m13 = mag3.e12,

      .m21 = mag1.e31,
      .m22 = mag2.e31,
      .m23 = mag3.e31,

      .m31 = mag1.e23,
      .m32 = mag2.e23,
      .m33 = mag3.e23,
  };

  // get scalaer.
  // The scalers are not part of a bivector, this is just the easiest impl
  Bivector scalers =
      matrix_bivector_mul(matrix_inv(mag_dipol_matrix), mag_dipole);

  // test if scalars are between -1 and 1
  // square them, sort, find the biggest,
  // if bigger 1 then scale all down by sqrt
  float scalar_list[] = {
      (scalers.e12 * scalers.e12),
      (scalers.e31 * scalers.e31),
      (scalers.e23 * scalers.e23),
  };

  qsort(scalar_list, sizeof(scalar_list) / sizeof(scalar_list[0]),
        sizeof(scalar_list[0]), compare_float);

  if (scalar_list[2] > 1.0f) {
    float max_scaler_sqrt = sqrtf(scalar_list[2]);
    mag_s1 = scalers.e12 * max_scaler_sqrt;
    mag_s2 = scalers.e31 * max_scaler_sqrt;
    mag_s3 = scalers.e23 * max_scaler_sqrt;

    Bivector new_mag = Bivector{
        .e12 = (mag_s1 * mag1.e12) + (mag_s2 * mag2.e12) + (mag_s3 * mag3.e12),
        .e31 = (mag_s1 * mag1.e31) + (mag_s2 * mag2.e31) + (mag_s3 * mag3.e31),
        .e23 = (mag_s1 * mag1.e23) + (mag_s2 * mag2.e23) + (mag_s3 * mag3.e23),
    };

    // torque = - mag_dipole cross mag_flux
    // switch place to get negate
    return bivector_cross(magnetrix_flux_density, new_mag);

  } else {
    mag_s1 = scalers.e12;
    mag_s2 = scalers.e31;
    mag_s3 = scalers.e23;
    return torque_scaled;
  }
}

void test() {

  // parallel
  printf("\nparallel\n");
  struct Vector y = Vector{1, 1, 0};
  struct Vector r = Vector{1, 1, 0};
  struct Bivector angle_err = angle_err_bivector(y, r);
  printf("e12: %f, e31: %f, e23: %f\n", (double)angle_err.e12,
         (double)angle_err.e31, (double)angle_err.e23);

  // anti parallel
  printf("\nantiparallel\n");
  y = Vector{1, 1, 0};
  r = Vector{-1, -1, 0};
  angle_err = angle_err_bivector(y, r);

  // general
  printf("\ngeneral\n");
  y = Vector{1.0, 0.0, 0.0};
  r = Vector{0.0, 1.0, 0.0};
  angle_err = angle_err_bivector(y, r);

  printf("\ngeneral\n");
  y = Vector{3.0, 1.0, -6.0};
  r = Vector{-3.0, 1.0, 4.0};
  angle_err = angle_err_bivector(y, r);

  printf("\nfn inverse matrix:\n");
  Matrix3x3 matrix{
      .m11 = 1.0f,
      .m12 = 0.0f,
      .m13 = 0.0f,

      .m21 = 3.0f,
      .m22 = 3.0f,
      .m23 = 0.0f,

      .m31 = 5.0f,
      .m32 = 2.0f,
      .m33 = -1.0f,
  };
  Matrix3x3 a = matrix_inv(matrix);
  //  1.0000        0        0
  // -1.0000   0.3333        0
  //  3.0000   0.6667  -1.0000
  a.m11 = 1.0f; // do something with the matrix
}
