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
// Sun sensors on all faces of the sat.
// Linear combination gives a vector pointing at the sun.
//
// The North sunsensor is defined to be in the e1 direction of SAT.
// This means that the angle between the sun sensor vector and North
// is the angle differnce between SAT and WORLD.
//
// Rotate all mathematical obejects to be in WORLD frame.

// Global Constants

// Sunsensor
// The Sunsensors are in Sat frame
// North = +e1, South = -e1, West = +e2, East = -e2, Up = +e3, Down = -e3
Vector base_sat = {
    .e1 = 1.0,
    .e2 = 0.0,
    .e3 = 0.0,
};
enum SunSensor { North, South, West, East, Up, Down };
float SunSensorArray[] = {North, South, West, East, Up, Down};

// Functions
void set_sun_sensor();
Vector get_sun_direction();
Bivector angle_difference_bivector(Vector a, Vector b);

int main() {

  set_sun_sensor();

  Vector sun_direction = get_sun_direction();
  printf("Sun Direction: Vector {\n");
  printf("\te1: %f\n", (double)sun_direction.e1);
  printf("\te2: %f\n", (double)sun_direction.e2);
  printf("\te3: %f\n", (double)sun_direction.e3);
  printf("}\n");

  // Vector to rotate from SAT to WORLD
  Bivector angle_frame_diff =
      angle_difference_bivector(base_sat, sun_direction);
  printf("Angle Frame Difference: Bivector {\n");
  printf("\te12: %f\n", (double)angle_frame_diff.e12);
  printf("\te31: %f\n", (double)angle_frame_diff.e31);
  printf("\te23: %f\n", (double)angle_frame_diff.e23);
  printf("}\n");

  // // Test bivector rotate
  // Bivector bivector = Bivector{.e12 = 2.0f, .e31 = 1.0f, .e23 = 1.0f};
  // Rotor rotor = Rotor{.scalar = 0.6234898018587336f,
  //                     .bivector = Bivector{
  //                         .e12 = 0.2089532529706375f,
  //                         .e31 = 0.6268597589119125f,
  //                         .e23 = -0.417906505941275f,
  //                     }};
  // Bivector res = rotate_bivector(bivector, rotor);
  // // rotate_bivector(bivector, rotor);
  // printf("res: Bivector { e12: %f, e31: %f, e23: %f}\n", (double)res.e12,
  //        (double)res.e31, (double)res.e23);
  // printf("{e13: -1.4858753192745673, e31: 1.866187603653894, e23: "
  //        "0.556343745843557}\n");
  return 0;
}

void set_sun_sensor() {
  // Set sun direction in Sat frame
  SunSensorArray[North] = 1.0f;
  SunSensorArray[East] = 0.0f;
  SunSensorArray[South] = 0.0f;
  SunSensorArray[West] = 0.0f;
  SunSensorArray[Up] = 0.0f;
  SunSensorArray[Down] = 0.0f;

#ifdef DEBUG
  FNPRINT("Sun North: %f\n", (double)SunSensorArray[North]);
  FNPRINT("Sun South: %f\n", (double)SunSensorArray[South]);
  FNPRINT("Sun West: %f\n", (double)SunSensorArray[West]);
  FNPRINT("Sun East: %f\n", (double)SunSensorArray[East]);
  FNPRINT("Sun Up: %f\n", (double)SunSensorArray[Up]);
  FNPRINT("Sun Down: %f\n", (double)SunSensorArray[Down]);
#endif
}

Vector get_sun_direction() {
  Vector sun_direction{
      .e1 = SunSensorArray[North] - SunSensorArray[South],
      .e2 = SunSensorArray[West] - SunSensorArray[East],
      .e3 = SunSensorArray[Up] - SunSensorArray[Down],
  };
  return sun_direction;
}

Bivector angle_difference_bivector(Vector a, Vector b) {

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
