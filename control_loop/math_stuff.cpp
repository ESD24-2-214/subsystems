#include "math_stuff.hpp"
#include "iostream"

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
