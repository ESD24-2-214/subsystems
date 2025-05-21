#include "Config.hpp"
#include <Hbro.hpp>
#include <math.h>

void pulse_mag(float scalar, int pinCW, int pinCCW) {
  int pwm_val = (int)(floorf(PWM_RES_MAX * abs(scalar)));

#if defined(DEBUG_PWM)
  Serial.print("Scalar:");
  Serial.println(scalar);
  Serial.print("PWM Value:");
  Serial.println(pwm_val);
#endif

  if (scalar <= 0) {
    analogWrite(pinCCW, LOW);
    analogWrite(pinCW, pwm_val);
  } else {
    analogWrite(pinCW, LOW);
    analogWrite(pinCCW, pwm_val);
  }
}

void enable_mag(bool state) {
  if (state) {
    digitalWrite(MAG1_EN, HIGH);
    digitalWrite(MAG2_EN, HIGH);
  } else {
    digitalWrite(MAG1_EN, LOW);
    digitalWrite(MAG2_EN, LOW);
  }
}
