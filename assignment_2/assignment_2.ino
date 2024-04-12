extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp32/rom/ets_sys.h"  // For ets_delay_us()
}

#define SIGNAL_PIN 2

void setup() {
  Serial.begin(115200);
  pinMode(SIGNAL_PIN, OUTPUT);
  xTaskCreate(
    digitalSignalTask,
    "Digital Signal Output",
    10000,
    NULL,
    1,
    NULL);
}

void loop() {
  // Intentionally empty.
}

void digitalSignalTask(void *parameter) {
  for (;;) {
    digitalWrite(SIGNAL_PIN, HIGH);
    ets_delay_us(180);  // 180μs HIGH
    digitalWrite(SIGNAL_PIN, LOW);
    ets_delay_us(40);  // 40μs LOW
    digitalWrite(SIGNAL_PIN, HIGH);
    ets_delay_us(530);  // 530μs HIGH
    digitalWrite(SIGNAL_PIN, LOW);
    ets_delay_us(3250);  // 3.25ms LOW

    vTaskDelay(1);
  }
}
