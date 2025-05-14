#include "Config.hpp"
#include <Hbro.hpp>
#include <math.h>

void pulse_mag(float scalar, int pinCW, int pinCCW) {
  if (scalar <= 0) {
    analogWrite(pinCCW, LOW);
    analogWrite(pinCW, (int)(floorf(PWM_RES_MAX * abs(scalar))));
  } else {
    analogWrite(pinCW, LOW);
    analogWrite(pinCCW, (int)(floorf(PWM_RES_MAX * abs(scalar))));
  }
}

void enable_mag(bool state){
  if(state){
    digitalWrite(MAG1_EN, HIGH);
    digitalWrite(MAG2_EN, HIGH);
  } else {
    digitalWrite(MAG1_EN, LOW);
    digitalWrite(MAG2_EN, LOW);
  }
}