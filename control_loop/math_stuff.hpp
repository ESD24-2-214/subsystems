#ifndef MATH_STUFF_H_
#define MATH_STUFF_H_

// Structs
typedef struct Vector {
  float e1;
  float e2;
  float e3;
} Vector;

typedef struct Bivector {
  float e12;
  float e31;
  float e23;
} Bivector;

typedef struct Rotor {
  float scalar;
  Bivector bivector;
} Rotor;

typedef struct Matrix3x3 { // row first, colomn second
  float m11, m12, m13;
  float m21, m22, m23;
  float m31, m32, m33;
} Matrix3x3;

Matrix3x3 matrix_inv(Matrix3x3 matrix);
Bivector matrix_bivector_mul(Matrix3x3 matrix, Bivector bivector);
Bivector bivector_cross(Bivector a, Bivector b);
Bivector scale_bivector(Bivector bivector, float scalar);
Bivector angle_difference_bivector(Vector a, Vector b);
Rotor rotor_form_halv_angle_bivector(Bivector half_angle);
Vector rotate_vector(Vector vector, Rotor rotor);
Bivector rotate_bivector(Bivector bivector, Rotor rotor);

#endif // MATH_STUFF_H_
