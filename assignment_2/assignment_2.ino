extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp32/rom/ets_sys.h"  // For ets_delay_us()
}

#define SIGNAL_PIN 2
#define SQUARE_WAVE_INPUT_PIN 34   // Use a GPIO pin that supports interrupts
#define SQUARE_WAVE_INPUT_PIN2 35  // Example pin, choose one that fits your setup

volatile unsigned long lastRiseTime = 0;
volatile unsigned long currentRiseTime = 0;
volatile boolean newFrequencyReady = false;
volatile float measuredFrequency = 0.0;

volatile unsigned long lastRiseTime2 = 0;
volatile unsigned long currentRiseTime2 = 0;
volatile boolean newFrequencyReady2 = false;
volatile float measuredFrequency2 = 0.0;

void IRAM_ATTR onRisingEdge() {
  lastRiseTime = currentRiseTime;
  currentRiseTime = micros();
  unsigned long period = currentRiseTime - lastRiseTime;
  if (period > 0) {                          // Avoid division by zero
    measuredFrequency = 1000000.0 / period;  // Convert period in microseconds to frequency in Hz
    newFrequencyReady = true;
  }
}

void IRAM_ATTR onRisingEdge2() {
  lastRiseTime2 = currentRiseTime2;
  currentRiseTime2 = micros();
  unsigned long period = currentRiseTime2 - lastRiseTime2;
  if (period > 0) {                           // Avoid division by zero
    measuredFrequency2 = 1000000.0 / period;  // Convert period in microseconds to frequency in Hz
    newFrequencyReady2 = true;
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(SIGNAL_PIN, OUTPUT);
  xTaskCreate(digitalSignalTask, "Digital Signal Output", 10000, NULL, 1, NULL);
  xTaskCreate(frequencyMeasurementTask, "Frequency Measurement", 2048, NULL, 2, NULL);
  pinMode(SQUARE_WAVE_INPUT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(SQUARE_WAVE_INPUT_PIN), onRisingEdge, RISING);
  pinMode(SQUARE_WAVE_INPUT_PIN2, INPUT);
  attachInterrupt(digitalPinToInterrupt(SQUARE_WAVE_INPUT_PIN2), onRisingEdge2, RISING);
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
      Serial.print("Measured Frequency 1: ");
      Serial.println(measuredFrequency);
    }
    if (newFrequencyReady2) {
      newFrequencyReady2 = false;  // Reset the flag
      Serial.print("Measured Frequency 2: ");
      Serial.println(measuredFrequency2);
    }
    vTaskDelay(pdMS_TO_TICKS(8));
  }
}
