#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"

// Define the GPIO pin for signal output
#define SIGNAL_PIN 2

void setup() {
  // Initialize the serial port for debugging
  Serial.begin(115200);
  while (!Serial) { ; }  // Wait for port to connect

  // Initialize the digital pin as an output
  pinMode(SIGNAL_PIN, OUTPUT);

  // Create a FreeRTOS task for outputting the digital signal
  xTaskCreate(
    digitalSignalTask,        // Task function
    "Digital Signal Output",  // Name of the task
    10000,                    // Stack size (bytes)
    NULL,                     // Task input parameter
    1,                        // Priority
    NULL                      // Task handle
  );
}

void loop() {
  // The loop function is intentionally left empty
}

// FreeRTOS task for outputting the digital signal
void digitalSignalTask(void *parameter) {
  for (;;) {  // Infinite loop
    digitalWrite(SIGNAL_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(180));  // 180μs HIGH
    digitalWrite(SIGNAL_PIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(40));  // 40μs LOW
    digitalWrite(SIGNAL_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(530));  // 530μs HIGH
    digitalWrite(SIGNAL_PIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(3250));  // 3.25ms LOW
  }
}
