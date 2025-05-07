#ifndef LDR_HPP
#define LDR_HPP

#include <Arduino.h>
#include <Config.hpp>
#include <cstdint>
#include <math_stuff.hpp>

typedef struct {
  float F;
  float L;
  float R;
  float B;
} LDRData_t;

void ldr_read(LDRData_t *ldr_data);
void ldr_read_data(LDRData_t *ldr_data, uint16_t t_ms, uint16_t samples);
void sun_read_data(Vector *sun_data, uint16_t t_ms, uint16_t samples);
void ldr_data_print(LDRData_t *ldr_data);

#endif // LDR_HPP
