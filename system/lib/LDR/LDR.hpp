#ifndef LDR_HPP
#define LDR_HPP

#include <Arduino.h>
#include <Config.hpp>

typedef struct {
    double F;
    double L;
    double R;
    double B;
} LDRData_t;

void ldr_read(LDRData_t *ldr_data);
void ldr_read_data(LDRData_t *ldr_data, uint16_t t_ms, uint16_t samples);
void ldr_data_print(LDRData_t *ldr_data);

#endif //LDR_HPP