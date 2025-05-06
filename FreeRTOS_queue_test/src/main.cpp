#include <Arduino.h>

#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0; // Unicore mode
#else // BaseType_t is defined in FreeRTOS e.g. int
static const BaseType_t app_cpu = 1;
#endif

QueueHandle_t Queue1, SerialQueue  = NULL; // Declare a queue handle

struct Message {
  int messageval;
};

struct StringMessage {
  char message[20];
};

void SendToQueue(void *par) {

  struct Message sendValue {
    .messageval = 0
  }; 

  for (;;) {
    sendValue.messageval++;

    if (xQueueOverwrite(Queue1, &sendValue) == pdPASS) {
      String s = "Send item";
      xQueueSend(SerialQueue, &s, 0);
    } else {
      String f = "Failed to send item";
      xQueueSend(SerialQueue, &f, 0);
    }

    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void ReceiveFromQueue(void *par) {
  struct Message receivedValue;
  for (;;) {
    if (xQueueReceive(Queue1, &receivedValue, 0) == pdPASS) {
      String m = "Received Item";
      xQueueSend(SerialQueue, &m, 0);

      struct StringMessage msg2;
      snprintf(msg2.message, sizeof(msg2.message), "%d", receivedValue.messageval);
      if(xQueueSend(SerialQueue, &msg2, 0) != pdPASS){
        Serial.println("Failed to print value");
      }
    }
    }
    vTaskDelay(1100 / portTICK_PERIOD_MS);
  }

void SerialService(void *par){
  struct StringMessage outboundMessage;

  for(;;){
    if (xQueueReceive(SerialQueue, &outboundMessage, 0) == pdPASS) {
      Serial.println(outboundMessage.message);
    }}
  }


void setup() {
  Serial.begin(115200);
  while (!Serial) {
  } // wait for serial port to connect. Needed for native USB port only

  delay(1000);
  // Create queue BEFORE creating tasks
  Queue1 = xQueueCreate(
      1, sizeof(Message)); // Create a queue to hold 1 Message struct
  SerialQueue = xQueueCreate(
      2, sizeof(StringMessage));

  if (Queue1 == NULL) {
    Serial.println("Queue creation failed");
    return;
  } else {
    Serial.println("Queue creation Succeded");
  }

  // Now create tasks
  xTaskCreatePinnedToCore(SendToQueue,   // Function to call
                          "SendToQueue", // Name of the task
                          1024,          // Stack size in bytes
                          NULL,          // Task input parameter
                          1,    // Task priority (0 to configMAX_PRIORITIES - 1)
                          NULL, // Task handle
                          app_cpu); // Create SendToQueue

  xTaskCreatePinnedToCore(ReceiveFromQueue, "ReceiveFromQueue", 1024,
                          NULL, // Task input parameter
                          1,    // Task priority (0 to configMAX_PRIORITIES - 1)
                          NULL, // Task handle
                          app_cpu); // Create ReceiveFromQueue

  xTaskCreatePinnedToCore(SerialService,
                          "SerialService",
                          1024,
                          NULL, // Task input parameter
                          1,    // Task priority (0 to configMAX_PRIORITIES - 1)
                          NULL, // Task handle
                          app_cpu); // Create ReceiveFromQueue
}

void loop() {
  // Nothing to do here
  delay(1000);
}
