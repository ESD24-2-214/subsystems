#ifndef LDR_HPP
#define LDR_HPP

#include <Arduino.h>
#include <Config.hpp>
#include <cstdint>
#include <math_stuff.hpp>

/* @brief Struct used to calibrate sensors in software
** @param Vector e1, e2 ,e3. (float)
*/
typedef struct {
  float e1;
  float e2;
  float e3;
} LDRCalibration_t;

/* @brief struct to store LDR sensor data.
** @param Directions F, L, R, B. (float)
*/
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
LDRCalibration_t ldr_calibrate_test(void);

#endif // LDR_HPP
