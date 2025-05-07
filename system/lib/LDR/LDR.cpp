#include <LDR.hpp>
#include <Config.hpp>

/* @brief This function reads the analog LDR values from the GPIO pins set in the Config.hpp file
** @param ldr_data: pointer to the LDRData_t structure
*/
void ldr_read(LDRData_t *ldr_data) {
    ldr_data->F = analogRead(LDR_PIN_F); // Read the LDR value
    ldr_data->L = analogRead(LDR_PIN_L); // Read the LDR value
    ldr_data->R = analogRead(LDR_PIN_R); // Read the LDR value
    ldr_data->B = analogRead(LDR_PIN_B); // Read the LDR value
}

/* @brief This function sample reads the analog LDR values from the GPIO pins set in the Config.hpp file
** @param ldr_data: pointer to the LDRData_t structure
** @param t_ms: time in milliseconds to wait between samples
** @param samples: number of samples to take
*/
void ldr_read_data(LDRData_t *ldr_data, uint16_t t_ms, uint16_t samples) {
    LDRData_t temp_data = {0}; // Initialize temp data structure
    LDRData_t sum_data = {0}; // Initialize temp data structure
    // For FREERTOS delay
    TickType_t xLastWakeTime = 0;
    xLastWakeTime = xTaskGetTickCount(); // Get the current tick count
    
    for(int i = 0; i < samples; i++) {
        ldr_read(&temp_data); 
        sum_data.F += temp_data.F; // Sum the LDR values
        sum_data.L += temp_data.L; 
        sum_data.R += temp_data.R; 
        sum_data.B += temp_data.B; 
        //delay(t_ms); // if not FREERTOS
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(t_ms)); // Delay should be FREERTOS wait until delay
    }
    ldr_data->F = sum_data.F / samples; // Average the LDR values
    ldr_data->L = sum_data.L / samples; 
    ldr_data->R = sum_data.R / samples; 
    ldr_data->B = sum_data.B / samples; 
}

/* @brief This function prints the LDR data to the serial monitor
** @param ldr_data: pointer to the LDRData_t structure
*/
void ldr_data_print(LDRData_t *ldr_data) {
    Serial.println("F, B, L, R,");
    Serial.print(ldr_data->F);
    Serial.print(", ");
    Serial.print(ldr_data->L);
    Serial.print(", ");
    Serial.print(ldr_data->R);
    Serial.print(", ");
    Serial.println(ldr_data->B);
}

void sun_data_read(Vector *sun_data, uint16_t t_ms, uint16_t samples){
    LDRData_t ldr_data = {0,0,0,0};
    ldr_read_data(&ldr_data, t_ms, samples);
    
    sun_data -> e1 = (ldr_data.F - ldr_data.B);
    sun_data -> e2 = (ldr_data.L - ldr_data.R);
    sun_data -> e3 = 0;
}