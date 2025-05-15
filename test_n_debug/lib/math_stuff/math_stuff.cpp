#include "math_stuff.hpp"
#include "iostream"
#include <cmath>

// Macros
#define FNPRINT(...)                                                           \
  printf("  ");                                                                \
  printf(__VA_ARGS__)

Matrix3x3 matrix_inv(Matrix3x3 matrix) {

#ifdef DEBUG
  FNPRINT("Invserse Matrix:\n");
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
#ifdef DEBUG
  FNPRINT("Determinate: %.9g\n", (double)determinate);
#endif

  if (determinate >= 0.0000000000000f && determinate <= 0.00000000000001f) {
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

Matrix3x3 matrix_dual(Matrix3x3 matrix) {
  // Takes matrix ment for vectors and turns matrix ment for bivectors
  // The dual of a vector:
  // Vector {e1, e2, e3}
  // becomes:
  // Bivector {e12: e3, e31: e2, e23: e1}
  // Therefor the dual of a matrix must be the flip in the secondary diaginal
#ifdef DEBUG
  FNPRINT("\nMatrix Dual\n");
  FNPRINT("Input Matrix {\n");
  FNPRINT("\t%f, %f, %f\n", (double)matrix.m11, (double)matrix.m12,
          (double)matrix.m13);
  FNPRINT("\t%f, %f, %f\n", (double)matrix.m21, (double)matrix.m22,
          (double)matrix.m23);
  FNPRINT("\t%f, %f, %f }\n", (double)matrix.m31, (double)matrix.m32,
          (double)matrix.m33);
#endif

  Matrix3x3 matrix_for_bivectors = Matrix3x3{
      .m11 = matrix.m33,
      .m12 = matrix.m23,
      .m13 = matrix.m13,
      //
      .m21 = matrix.m32,
      .m22 = matrix.m22,
      .m23 = matrix.m12,
      //
      .m31 = matrix.m31,
      .m32 = matrix.m21,
      .m33 = matrix.m11,
  };

#ifdef DEBUG
  FNPRINT("Matrix_For_Bivectors {\n");
  FNPRINT("\t%f, %f, %f\n", (double)matrix_for_bivectors.m11,
          (double)matrix_for_bivectors.m12, (double)matrix_for_bivectors.m13);
  FNPRINT("\t%f, %f, %f\n", (double)matrix_for_bivectors.m21,
          (double)matrix_for_bivectors.m22, (double)matrix_for_bivectors.m23);
  FNPRINT("\t%f, %f, %f }\n", (double)matrix_for_bivectors.m31,
          (double)matrix_for_bivectors.m32, (double)matrix_for_bivectors.m33);
#endif
  return matrix_for_bivectors;
}

Matrix3x3 matrix_from_vectors(Vector vector1, Vector vector2, Vector vector3) {
  Matrix3x3 res = Matrix3x3{
      .m11 = vector1.e1,
      .m12 = vector2.e1,
      .m13 = vector3.e1,

      .m21 = vector1.e2,
      .m22 = vector2.e2,
      .m23 = vector3.e2,

      .m31 = vector1.e3,
      .m32 = vector2.e3,
      .m33 = vector3.e3,
  };

#ifdef DEBUG
  FNPRINT("Matrix from Vectors:\n");
  FNPRINT("  %f, %f, %f\n", (double)res.m11, (double)res.m12, (double)res.m13);
  FNPRINT("  %f, %f, %f\n", (double)res.m21, (double)res.m22, (double)res.m23);
  FNPRINT("  %f, %f, %f\n\n", (double)res.m31, (double)res.m32,
          (double)res.m33);
#endif

  return res;
};

Matrix3x3 matrix_from_bivectors(Bivector a, Bivector b, Bivector c) {

  Matrix3x3 res = Matrix3x3{
      .m11 = a.e12,
      .m12 = b.e12,
      .m13 = c.e12,

      .m21 = a.e31,
      .m22 = b.e31,
      .m23 = c.e31,

      .m31 = a.e23,
      .m32 = b.e23,
      .m33 = c.e23,
  };

#ifdef DEBUG
  FNPRINT("Matrix from Bivectors:\n");
  FNPRINT("  %f, %f, %f\n", (double)res.m11, (double)res.m12, (double)res.m13);
  FNPRINT("  %f, %f, %f\n", (double)res.m21, (double)res.m22, (double)res.m23);
  FNPRINT("  %f, %f, %f\n\n", (double)res.m31, (double)res.m32,
          (double)res.m33);
#endif

  return res;
}

Vector matrix_vector_mul(Matrix3x3 matrix, Vector vector) {
#ifdef DEBUG
  FNPRINT("\nVector matrix mul input:\n");
  FNPRINT("Vector {");
  FNPRINT("%f, %f, %f }\n", (double)vector.e1, (double)vector.e2,
          (double)vector.e3);
  FNPRINT("Matrix {\n");
  FNPRINT("\t%f, %f, %f\n", (double)matrix.m11, (double)matrix.m12,
          (double)matrix.m13);
  FNPRINT("\t%f, %f, %f\n", (double)matrix.m21, (double)matrix.m22,
          (double)matrix.m23);
  FNPRINT("\t%f, %f, %f }\n", (double)matrix.m31, (double)matrix.m32,
          (double)matrix.m33);
#endif

  Vector res = Vector{
      .e1 = (matrix.m11 * vector.e1) + (matrix.m12 * vector.e2) +
            (matrix.m13 * vector.e2),

      .e2 = (matrix.m21 * vector.e1) + (matrix.m22 * vector.e2) +
            (matrix.m23 * vector.e3),

      .e3 = (matrix.m31 * vector.e1) + (matrix.m32 * vector.e2) +
            (matrix.m33 * vector.e3),
  };

#ifdef DEBUG
  FNPRINT("Vector matrix mul res:\n");
  FNPRINT("\tVector { %f, %f, %f }\n", (double)res.e1, (double)res.e2,
          (double)res.e3);
#endif
  return res;
}

Bivector matrix_bivector_mul(Matrix3x3 matrix, Bivector bivector) {
#ifdef DEBUG
  FNPRINT("\nBivector matrix mul input:\n");
  FNPRINT("Bivector {");
  FNPRINT("%f, %f, %f }\n", (double)bivector.e12, (double)bivector.e31,
          (double)bivector.e23);
  FNPRINT("Matrix {\n");
  FNPRINT("\t%f, %f, %f\n", (double)matrix.m11, (double)matrix.m12,
          (double)matrix.m13);
  FNPRINT("\t%f, %f, %f\n", (double)matrix.m21, (double)matrix.m22,
          (double)matrix.m23);
  FNPRINT("\t%f, %f, %f }\n", (double)matrix.m31, (double)matrix.m32,
          (double)matrix.m33);

#endif

  Bivector res = Bivector{
      .e12 = (matrix.m11 * bivector.e12) + (matrix.m12 * bivector.e31) +
             (matrix.m13 * bivector.e23),

      .e31 = (matrix.m21 * bivector.e12) + (matrix.m22 * bivector.e31) +
             (matrix.m23 * bivector.e23),

      .e23 = (matrix.m31 * bivector.e12) + (matrix.m32 * bivector.e31) +
             (matrix.m33 * bivector.e23),
  };
#ifdef DEBUG
  FNPRINT("Bivector matrix mul res:\n");
  FNPRINT("\tBivector {");
  FNPRINT("%f, %f, %f }\n", (double)res.e12, (double)res.e31, (double)res.e23);
#endif

  return res;
}

Matrix3x3 matrix_mul(Matrix3x3 a, Matrix3x3 b) {
#ifdef DEBUG
  FNPRINT("\nMatrix mul:\n");
  FNPRINT("Matrix1 {\n");
  FNPRINT("\t%f, %f, %f\n", (double)a.m11, (double)a.m12, (double)a.m13);
  FNPRINT("\t%f, %f, %f\n", (double)a.m21, (double)a.m22, (double)a.m23);
  FNPRINT("\t%f, %f, %f }\n", (double)a.m31, (double)a.m32, (double)a.m33);
  FNPRINT("Matrix2 {\n");
  FNPRINT("\t%f, %f, %f\n", (double)b.m11, (double)b.m12, (double)b.m13);
  FNPRINT("\t%f, %f, %f\n", (double)b.m21, (double)b.m22, (double)b.m23);
  FNPRINT("\t%f, %f, %f }\n", (double)b.m31, (double)b.m32, (double)b.m33);
#endif

  Matrix3x3 res = Matrix3x3{
      .m11 = (a.m11 * b.m11) + (a.m12 * b.m21) + (a.m13 * b.m31),
      .m12 = (a.m11 * b.m12) + (a.m12 * b.m22) + (a.m13 * b.m32),
      .m13 = (a.m11 * b.m13) + (a.m12 * b.m23) + (a.m13 * b.m33),
      //
      .m21 = (a.m21 * b.m11) + (a.m22 * b.m21) + (a.m23 * b.m31),
      .m22 = (a.m21 * b.m12) + (a.m22 * b.m22) + (a.m23 * b.m32),
      .m23 = (a.m21 * b.m13) + (a.m22 * b.m23) + (a.m23 * b.m33),
      //
      .m31 = (a.m31 * b.m11) + (a.m32 * b.m21) + (a.m33 * b.m31),
      .m32 = (a.m31 * b.m12) + (a.m32 * b.m22) + (a.m33 * b.m32),
      .m33 = (a.m31 * b.m13) + (a.m32 * b.m23) + (a.m33 * b.m33),

  };
#ifdef DEBUG
  FNPRINT("Matrix Res {\n");
  FNPRINT("\t%f, %f, %f\n", (double)res.m11, (double)res.m12, (double)res.m13);
  FNPRINT("\t%f, %f, %f\n", (double)res.m21, (double)res.m22, (double)res.m23);
  FNPRINT("\t%f, %f, %f }\n", (double)res.m31, (double)res.m32,
          (double)res.m33);
#endif
  return res;
}

Bivector dual_vector_to_bivector(Vector a) {
  Bivector res = Bivector{
      .e12 = a.e3,
      .e31 = a.e2,
      .e23 = a.e1,
  };
  return res;
}

Vector vector_cross(Vector a, Vector b) {
#ifdef DEBUG
  FNPRINT("\nVector Cross:\n");
  FNPRINT("\tvector a:\n");
  FNPRINT("\t%f, %f, %f\n", (double)a.e1, (double)a.e2, (double)a.e3);
  FNPRINT("\tvector b:\n");
  FNPRINT("\t%f, %f, %f\n", (double)b.e1, (double)b.e2, (double)b.e3);
#endif

  // The anti symmetric product of bivectors
  Vector res = Vector{
      .e1 = a.e2 * b.e3 - a.e3 * b.e2,
      .e2 = a.e3 * b.e1 - a.e1 * b.e3,
      .e3 = a.e1 * b.e2 - a.e2 * b.e1,
  };

#ifdef DEBUG
  FNPRINT("\tVector res: {");
  FNPRINT(" e1: %f,e2: %f,e3: %f } \n", (double)res.e1, (double)res.e2,
          (double)res.e3);
#endif
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
Bivector angle_difference_bivector(Vector a, Vector b) {

#ifdef DEBUG
  FNPRINT("\nAngle Difference Bivector\n");
  FNPRINT("Vector A: { e1: %f, e2: %f, e3: %f }\n", (double)a.e1, (double)a.e2,
          (double)a.e3);
  FNPRINT("Vector B: { e1: %f, e2: %f, e3: %f }\n", (double)b.e1, (double)b.e2,
          (double)b.e3);
#endif

  // Find the angle error bivector betweem the current vector and refernce
  // $$ \overset\Rightarrow{\theta}_\text{err} =
  // \frac{\vec{y}\wedge \vec{r}}{|\vec{y}\wedge \vec{r}|}
  // \arccos \left( \frac{|\vec{y}\cdot \vec{r}|}{|\vec{y}| |\vec{r}|} \right)
  // $$

  // There are three cases
  // 1 They are parallel and the same direction. There the error angle is 0
  // with no plane 2 They are anti parallet in the opesite direction. The
  // error angle is pi with no plane ( give a plane) 3 anything in between.
  // Has both plane and error

  // $$ \vec{y}\cdot \vec{r} $$
  float inner_product = (a.e1 * b.e1)   //
                        + (a.e2 * b.e2) //
                        + (a.e3 * b.e3);

  // Norm of inner product
  // $$A A^{\dag}$$
  float a_norm = sqrtf(float((a.e1 * a.e1)   //
                             + (a.e2 * a.e2) //
                             + (a.e3 * a.e3)));
  float b_norm = sqrtf(float((b.e1 * b.e1)   //
                             + (b.e2 * b.e2) //
                             + (b.e3 * b.e3)));
  float inner_norm = inner_product / (a_norm * b_norm);

#ifdef DEBUG
  FNPRINT("inner norm: %f\n", (double)inner_norm);
#endif

  // if (inner_product > 0.90000000f) { // parallel
  if (inner_norm >= 1.0f && inner_norm <= 1.0000001f) { // parallel
#ifdef DEBUG
    FNPRINT("parallel path\n");
#endif

    return Bivector{0.0, 0.0, 0.0};
  } else if (inner_norm >= -1.01f && inner_norm <= -1.00f) { // anti
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
        .e12 = a.e1 * b.e2 - a.e2 * b.e1,
        .e31 = a.e3 * b.e1 - a.e1 * b.e3,
        .e23 = a.e2 * b.e3 - a.e3 * b.e2,
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

Vector scale_vector(Vector vector, float scalar) {
  Vector vector_scaled{
      .e1 = vector.e1 * scalar,
      .e2 = vector.e2 * scalar,
      .e3 = vector.e3 * scalar,
  };
  return vector_scaled;
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

int compare_float(const void *a, const void *b) {
  float fa = *(const float *)a;
  float fb = *(const float *)b;
  return (fa > fb) - (fa < fb);
}
