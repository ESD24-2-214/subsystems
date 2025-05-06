#include <LDR.hpp>
#include <Config.hpp>

void init_LDR() {
    pinMode(LDR_PIN_F, ANALOG); // Set the LDR pin as input
    pinMode(LDR_PIN_L, ANALOG); // Set the LDR pin as input
    pinMode(LDR_PIN_R, ANALOG); // Set the LDR pin as input
    pinMode(LDR_PIN_B, ANALOG); // Set the LDR pin as input
}

int ldr_data_read(LDRData_t *ldr_data) {
    ldr_data->F = analogRead(LDR_PIN_F); // Read the LDR value
    ldr_data->L = analogRead(LDR_PIN_L); // Read the LDR value
    ldr_data->R = analogRead(LDR_PIN_R); // Read the LDR value
    ldr_data->B = analogRead(LDR_PIN_B); // Read the LDR value
}
