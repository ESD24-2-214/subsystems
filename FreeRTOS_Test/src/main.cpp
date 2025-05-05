#include <Arduino.h>

#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0; // Unicore mode
#else                                // BaseType_t is defined in FreeRTOS e.g. int
static const BaseType_t app_cpu = 1;
#endif
SemaphoreHandle_t DataShare; // Declare a semaphore handle
QueueHandle_t Buffer; // Declare a queue handle


void Task1(void *par){
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount(); // Get the current tick count
  const TickType_t timePeriod = 100; // Set the time period
  int n;
  while(1){
  vTaskDelayUntil(&xLastWakeTime, timePeriod/portTICK_PERIOD_MS);
  
  if (xSemaphoreTake(DataShare, 0) == pdTRUE) { // Try to take the semaphore
    xQueueReceive(Buffer, &n, 0); // Receive the value from the queue
    xSemaphoreGive(DataShare); // Release the semaphore
  }
  else {
    Serial.println("Semaphore not available"); // Semaphore not available
  } 
  Serial.println(n);
  }
}

void Task2(void *par) {
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount(); // Get the current tick count
  const TickType_t timePeriod = 2000; // Set the time period
  int v;
  while(1){
    vTaskDelayUntil(&xLastWakeTime, timePeriod/portTICK_PERIOD_MS);

    if (xSemaphoreTake(DataShare, 0) == pdTRUE) { // Try to take the semaphore
      v++; // Access the shared variable
      xQueueSend(Buffer, &v, 0); // Send the value to the queue
      xSemaphoreGive(DataShare); // Release the semaphore
    }
    else {
      Serial.println("Semaphore not available"); // Semaphore not available
    }
  }
}

void setup() {
Serial.begin(115200);
while(!Serial) {} // wait for serial port to connect. Needed for native USB port only

  xTaskCreatePinnedToCore(
    Task1,    // Function to call
    "Task1",  // Name of the task
    1024,     // Stack size in bytes
    NULL,     // Task input parameter
    24,       // Task priority (0 to configMAX_PRIORITIES - 1)
    NULL,     // Task handle
    app_cpu); // Create Task1

  xTaskCreatePinnedToCore(
    Task2, 
    "Task2", 
    1024, 
    NULL, 
    10, 
    NULL,
    app_cpu); // Create Task2

    DataShare = xSemaphoreCreateMutex();
    Buffer = xQueueCreate(10, sizeof(int)); // Create a queue to hold 10 integers


  // if this was vanilla FreeRTOS, you would run vTaskStartScheduler(); 
  // but this is ESP32 in arduino and it has already been started
}

void loop() {}
