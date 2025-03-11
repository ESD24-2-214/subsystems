#include <Arduino.h>
#include <cstdint>
#include <stdint.h>

struct SunVector {
  int x;
  int y;
};

int16_t ADC1 = 0;
int8_t loop_count = 10;
struct SunVector a = {0, 0};

enum LDRPin {
  North = 0,
  East = 1,
  South = 2,
  West = 3,
};

void setup() { Serial.begin(115200); }

void loop() {

  Serial.write(0x1b);
  Serial.print("[?25l");
  Serial.write(0x1b);
  Serial.print("[2J");
  Serial.write(0x1b);
  Serial.print("[1;1H");

  if (loop_count < 100) {
    loop_count++;
    for (int i = North; i < West; i++) {
      switch (i) {
      case North:
        a.y += 4095 - analogRead(North);
        break;
      case East:
        a.x += 4095 - analogRead(East);
        break;
      // case South:
      //   a.y -= 4095 - analogRead(South);
      //   break;
      // case West:
      //   a.x -= 4095 - analogRead(West);
      //   break;
      default:
        break;
      }
    }
  } else {
    Serial.print("The x part:");
    Serial.println(a.x / 100);
    Serial.print("The y part:");
    Serial.println(a.y / 100);
    a = {0, 0};
    loop_count = 0;
    delay(500);
  }
}
