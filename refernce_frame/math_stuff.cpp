#include "math_stuff.hpp"
#include "iostream"
#include <cmath>

// Macros
#define FNPRINT(...)                                                           \
  printf("  ");                                                                \
  printf(__VA_ARGS__)

Matrix3x3 matrix_inv(Matrix3x3 matrix) {

#ifdef DEBUG
  FNPRINT("start matrix:\n");
  FNPRINT("  %f, %f, %f\n", (double)matrix.m11, (double)matrix.m12,
          (double)matrix.m13);
  FNPRINT("  %f, %f, %f\n", (double)matrix.m21, (double)matrix.m22,
          (double)matrix.m23);
  FNPRINT("  %f, %f, %f\n", (double)matrix.m31, (double)matrix.m32,
          (double)matrix.m33);
#endif

  // The inverse is one over the determite multiplied by the adjoint of the
  // matrix

  // determinate of 2x2
  // |A| = [ad - bc]
  //

  // reuse the calcs
  float m11_det =
      matrix.m22 * matrix.m33 - matrix.m23 * matrix.m32; // col 1 row 1
  float m12_det =
      matrix.m21 * matrix.m33 - matrix.m23 * matrix.m31; // col 2 row 1
  float m13_det =
      matrix.m21 * matrix.m32 - matrix.m22 * matrix.m31; // col 3 row 1

  float determinate =
      matrix.m11 * m11_det - matrix.m12 * m12_det + matrix.m13 * m13_det;

  if (determinate >= 0.00f && determinate <= 0.00001f) {
    exit(1);
  }
  float scalar = 1 / determinate;

  Matrix3x3 res = Matrix3x3{
      .m11 = scalar * m11_det, // pos
      .m12 =
          scalar * (matrix.m13 * matrix.m32 - matrix.m12 * matrix.m33), // neg
      .m13 =
          scalar * (matrix.m12 * matrix.m23 - matrix.m13 * matrix.m22), // pos

      .m21 =
          scalar * (matrix.m23 * matrix.m31 - matrix.m21 * matrix.m33), // neg
      .m22 =
          scalar * (matrix.m11 * matrix.m33 - matrix.m13 * matrix.m31), // pos
      .m23 =
          scalar * (matrix.m13 * matrix.m21 - matrix.m11 * matrix.m23), // neg

      .m31 =
          scalar * (matrix.m21 * matrix.m32 - matrix.m22 * matrix.m31), // pos
      .m32 =
          scalar * (matrix.m12 * matrix.m31 - matrix.m11 * matrix.m32), // neg
      .m33 =
          scalar * (matrix.m11 * matrix.m22 - matrix.m12 * matrix.m21), // pos
  };

#ifdef DEBUG
  FNPRINT("inverse matrix:\n");
  FNPRINT("  %f, %f, %f\n", (double)res.m11, (double)res.m12, (double)res.m13);
  FNPRINT("  %f, %f, %f\n", (double)res.m21, (double)res.m22, (double)res.m23);
  FNPRINT("  %f, %f, %f\n\n", (double)res.m31, (double)res.m32,
          (double)res.m33);
#endif

  return res;
};

Bivector matrix_bivector_mul(Matrix3x3 matrix, Bivector bivector) {

  Bivector res = Bivector{
      .e12 = (matrix.m11 * bivector.e12) + (matrix.m12 * bivector.e12) +
             (matrix.m13 * bivector.e12),

      .e31 = (matrix.m21 * bivector.e31) + (matrix.m22 * bivector.e31) +
             (matrix.m23 * bivector.e31),

      .e23 = (matrix.m31 * bivector.e23) + (matrix.m32 * bivector.e23) +
             (matrix.m33 * bivector.e23),
  };

  return res;
}

Bivector bivector_cross(Bivector a, Bivector b) {

#ifdef DEBUG
  FNPRINT("\nBivector Cross:\n");
  FNPRINT("  bivector a:\n");
  FNPRINT("  %f, %f, %f\n", (double)a.e12, (double)a.e31, (double)a.e23);
  FNPRINT("  %f, %f, %f\n", (double)b.e12, (double)b.e31, (double)b.e23);
#endif

  // The anti symmetric product of bivectors
  Bivector res = Bivector{
      .e12 = a.e31 * b.e23 - a.e23 * b.e31,
      .e31 = a.e23 * b.e12 - a.e12 * b.e23,
      .e23 = a.e12 * b.e31 - a.e31 * b.e12,
  };

#ifdef DEBUG
  FNPRINT("  bivector res:\n");
  FNPRINT("  %f, %f, %f\n", (double)res.e12, (double)res.e31, (double)res.e23);
#endif
  return res;
}

Bivector scale_bivector(Bivector bivector, float scalar) {
  Bivector bivector_scaled{
      .e12 = bivector.e12 * scalar,
      .e31 = bivector.e31 * scalar,
      .e23 = bivector.e23 * scalar,
  };
  return bivector_scaled;
}

Rotor rotor_form_halv_angle_bivector(Bivector half_angle) {
  // spilt up the angle bivector into angle and plane
  float rot_half_angle_norm = sqrtf((half_angle.e12 * half_angle.e12) +
                                    (half_angle.e31 * half_angle.e31) +
                                    (half_angle.e23 * half_angle.e23));

  if (rot_half_angle_norm == 0.0f) {
    // There is no plane
    // cos(0.0) = 1.0
    // sin(0.0) = 0.0
    Rotor rotor{
        .scalar = 1.0,
        .bivector = Bivector{.e12 = 0.0, .e31 = 0.0, .e23 = 0.0},
    };
    return rotor;
  }

  // rotor is made from half the angle
  float rotor_scalar = cosf(rot_half_angle_norm);
  float sin = sinf(rot_half_angle_norm);

  Rotor rotor{
      .scalar = rotor_scalar,
      .bivector =
          Bivector{
              .e12 = half_angle.e12 * sin / rot_half_angle_norm,
              .e31 = half_angle.e31 * sin / rot_half_angle_norm,
              .e23 = half_angle.e23 * sin / rot_half_angle_norm,
          },
  };
  return rotor;
}

Vector rotate_vector(Vector vector, Rotor rotor) {
  // Plan
  // 1. get rotor
  // 2. rotate
#ifdef DEBUG
  FNPRINT("\nRotate:\n");
#endif

  // matrix version of the vector rotor formula
  // \[\mathcal{R}(R) =
  // \begin{bmatrix}
  // r_0^{2}-r_4^{2}-r_5^{2}+r_6^{2 }& 2(r_{6}r_{5} -r_{0} r_{4} ) &
  // 2(r_{0}r_{5} + r_{4}r_{6}) \\
// 2(r_{0}r_{4} +r_{5}r_{6}) & r_0^{2}-r_4^{2}+r_5^{2}-r_6^{2} &
  // 2(r_{4}r_{5}-r_{0}r_{6}) \\
// 2(r_{4}r_{6}-r_{0}r_{5}) & 2(r_{0}r_{6}+r_{4}r_{5}) &
  // r_0^{2}+r_4^{2}-r_5^{2}-r_6^{2} \\
// \end{bmatrix}\]

  Vector res = Vector{
      .e1 =
          // r_0^{2}-r_4^{2}-r_5^{2}+r_6^{2 } &
      // 2(r_{6}r_{5} -r_{0} r_{4} ) &
      // 2(r_{0}r_{5} + r_{4}r_{6})
      (((rotor.scalar * rotor.scalar) -
        (rotor.bivector.e12 * rotor.bivector.e12) -
        (rotor.bivector.e31 * rotor.bivector.e31) +
        (rotor.bivector.e23 * rotor.bivector.e23)) *
       vector.e1) +
      (2 *
       (rotor.bivector.e23 * rotor.bivector.e31 -
        rotor.scalar * rotor.bivector.e12) *
       vector.e2) +
      (2 *
       (rotor.scalar * rotor.bivector.e31 +
        rotor.bivector.e12 * rotor.bivector.e23) *
       vector.e3),
      .e2 =
          // 2(r_{0}r_{4} + r_{5}r_{6}) &
          // r_0^{2}-r_4^{2}+r_5^{2}-r_6^{2} &
          // 2(r_{4}r_{5}-r_{0}r_{6})
      (2 *
       (rotor.scalar * rotor.bivector.e12 +
        rotor.bivector.e31 * rotor.bivector.e23) *
       vector.e1) +
      (((rotor.scalar * rotor.scalar) -
        (rotor.bivector.e12 * rotor.bivector.e12) +
        (rotor.bivector.e31 * rotor.bivector.e31) -
        (rotor.bivector.e23 * rotor.bivector.e23)) *
       vector.e2) +
      (2 *
       (rotor.bivector.e12 * rotor.bivector.e31 -
        rotor.scalar * rotor.bivector.e23) *
       vector.e3),
      .e3 =
          // 2(r_{4}r_{6}-r_{0}r_{5}) &
          // 2(r_{0}r_{6}+r_{4}r_{5}) &
          // r_0^{2}+r_4^{2}-r_5^{2}-r_6^{2}
      (2 *
       (rotor.bivector.e12 * rotor.bivector.e23 -
        rotor.scalar * rotor.bivector.e31) *
       vector.e1) +
      (2 *
       (rotor.scalar * rotor.bivector.e23 +
        rotor.bivector.e12 * rotor.bivector.e31) *
       vector.e2) +
      (((rotor.scalar * rotor.scalar) +
        (rotor.bivector.e12 * rotor.bivector.e12) -
        (rotor.bivector.e31 * rotor.bivector.e31) -
        (rotor.bivector.e23 * rotor.bivector.e23)) *
       vector.e3),
  };

#ifdef DEBUG
  FNPRINT("  rotated vector:\n");
  FNPRINT("  e1: %f, e2: %f, e3: %f\n", (double)res.e1, (double)res.e2,
          (double)res.e3);
#endif

  return res;
}

Bivector rotate_bivector(Bivector bivector, Rotor rotor) {
  // TODO

  float rotor_scalar_squared = rotor.scalar * rotor.scalar;
  float rotor_e12_squared = rotor.bivector.e12 * rotor.bivector.e12;
  float rotor_e31_squared = rotor.bivector.e31 * rotor.bivector.e31;
  float rotor_e23_squared = rotor.bivector.e23 * rotor.bivector.e23;

  Bivector res{
      // ((r_0^{2} + r_4^{2}  - r_5^{2} - r_6^{2} ) a_{4}
      // +(r_5r_4 + r_4r_5 + r_6r_0 + r_0r_6) a_{5}
      // +(r_6r_4 + r_4r_6 - r_5r_0 - r_0r_5) a_{6} )\einheit{1,2}
      (rotor_scalar_squared //
       + rotor_e12_squared  //
       - rotor_e31_squared  //
       - rotor_e23_squared) *
              bivector.e12 //
          + 2.0f *
                (rotor.bivector.e12 * rotor.bivector.e31 +
                 rotor.scalar * rotor.bivector.e23) *
                bivector.e31 //
          + 2.0f *
                (rotor.bivector.e12 * rotor.bivector.e23 -
                 rotor.scalar * rotor.bivector.e31) *
                bivector.e23,
      // ((r_4r_5 + r_5r_4 - r_6r_0- r_0r_6 ) a_{4}
      // +( r_0^{2} - r_4^{2} + r_5^{2} -r_6^{2}) a_{5}
      // +( r_6r_5 + r_5r_6 + r_4r_0 + r_0r_4 ) a_{6 })\einheit{3,1}
      2.0f *
              (rotor.bivector.e12 * rotor.bivector.e31 -
               rotor.scalar * rotor.bivector.e23) *
              bivector.e12 +
          (rotor_scalar_squared //
           - rotor_e12_squared  //
           + rotor_e31_squared  //
           - rotor_e23_squared) *
              bivector.e31 +
          2.0f *
              (rotor.scalar * rotor.bivector.e12 +
               rotor.bivector.e31 * rotor.bivector.e23) *
              bivector.e23,
      //  ((r_4r_6 + r_6r_4 + r_5r_0 + r_0r_5 ) a_{4}
      // +(r_5r_6 + r_6r_5 - r_4r_0 - r_0r_4 ) a_{5 }
      // +(r_0^{2} - r_4^{2} - r_5^{2} + r_6^{2}  ) a_{6} )\einheit{2,3}
      2.0f *
              (rotor.bivector.e12 * rotor.bivector.e23 +
               rotor.scalar * rotor.bivector.e31) *
              bivector.e12 +
          2.0f *
              (rotor.bivector.e31 * rotor.bivector.e23 -
               rotor.scalar * rotor.bivector.e12) *
              bivector.e31 +
          (rotor_scalar_squared //
           - rotor_e12_squared  //
           - rotor_e31_squared  //
           + rotor_e23_squared) *
              bivector.e23,
  };
  return res;
}
