extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp32/rom/ets_sys.h"  // For ets_delay_us()
}

#define SIGNAL_PIN 2
#define SQUARE_WAVE_INPUT_PIN 34  // Use a GPIO pin that supports interrupts

volatile unsigned long lastRiseTime = 0;
volatile unsigned long currentRiseTime = 0;
volatile boolean newFrequencyReady = false;
volatile float measuredFrequency = 0.0;

void IRAM_ATTR onRisingEdge() {
  lastRiseTime = currentRiseTime;
  currentRiseTime = micros();
  unsigned long period = currentRiseTime - lastRiseTime;
  if (period > 0) {                          // Avoid division by zero
    measuredFrequency = 1000000.0 / period;  // Convert period in microseconds to frequency in Hz
    newFrequencyReady = true;
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(SIGNAL_PIN, OUTPUT);
  xTaskCreate(digitalSignalTask, "Digital Signal Output", 10000, NULL, 1, NULL);
  xTaskCreate(frequencyMeasurementTask, "Frequency Measurement", 2048, NULL, 2, NULL);
  pinMode(SQUARE_WAVE_INPUT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(SQUARE_WAVE_INPUT_PIN), onRisingEdge, RISING);
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

void frequencyMeasurementTask(void *parameter) {
  for (;;) {
    if (newFrequencyReady) {
      newFrequencyReady = false;  // Reset the flag
      // Process the measured frequency. For example, print it:
      Serial.print("Measured Frequency: ");
      Serial.println(measuredFrequency);
    }
    vTaskDelay(pdMS_TO_TICKS(20));  // Wait for 20ms (task requirement)
  }
}