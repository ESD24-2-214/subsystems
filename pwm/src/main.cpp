#include "Arduino.h"
#include "stdint.h"

uint8_t pwm_pin = 7;
uint16_t pwm_max_bit = 8;
uint32_t pwm_freq = 10e4;
// uint8_t pwm_val = 150;

void setup() {
  Serial.begin(115200);
  analogWriteResolution(pwm_max_bit);
  analogWriteFrequency(pwm_freq);
}

void loop() {
  // Serial.println("Hello, World!");
  // void analogWrite(pwm_pin, powf(2, pwm_max_bit) - 1);
  for (int pwm_val = 0; pwm_val < powf(2, pwm_max_bit); pwm_val++) {
    analogWrite(pwm_pin, pwm_val);
    Serial.println(pwm_val);
    delay(20);
  }
}
