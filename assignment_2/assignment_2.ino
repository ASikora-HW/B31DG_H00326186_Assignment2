extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp32/rom/ets_sys.h"  // For ets_delay_us()
}

#define SIGNAL_PIN 2
#define SQUARE_WAVE_INPUT_PIN 34   // Use a GPIO pin that supports interrupts
#define SQUARE_WAVE_INPUT_PIN2 33  // Example pin, choose one that fits your setup
#define ANALOG_PIN 34              // Example analog pin, adjust based on your ESP32 board
#define ERROR_LED_PIN 3            // LED pin to indicate error
#define MAX_ANALOG_READING 4095    // Maximum reading for a 12-bit ADC, adjust if different
#define NUM_READINGS 10

volatile unsigned long lastRiseTime = 0;
volatile unsigned long currentRiseTime = 0;
volatile boolean newFrequencyReady = false;
volatile float measuredFrequency = 0.0;

volatile unsigned long lastRiseTime2 = 0;
volatile unsigned long currentRiseTime2 = 0;
volatile boolean newFrequencyReady2 = false;
volatile float measuredFrequency2 = 0.0;

int readings[NUM_READINGS];  // Circular buffer for readings
int readIndex = 0;           // Current position in the buffer
long total = 0;              // Sum of the readings
float average = 0;           // Running average

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
  xTaskCreate(sampleAnalogTask, "Sample Analog Input", 2048, NULL, 1, NULL);
  xTaskCreate(logFrequencyTask, "Log Frequency", 2048, NULL, 3, NULL);

  pinMode(SQUARE_WAVE_INPUT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(SQUARE_WAVE_INPUT_PIN), onRisingEdge, RISING);
  pinMode(SQUARE_WAVE_INPUT_PIN2, INPUT);
  attachInterrupt(digitalPinToInterrupt(SQUARE_WAVE_INPUT_PIN2), onRisingEdge2, RISING);
  pinMode(ERROR_LED_PIN, OUTPUT);
  pinMode(ANALOG_PIN, INPUT);
  for (int thisReading = 0; thisReading < NUM_READINGS; thisReading++) {
    readings[thisReading] = 0;
  }
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
      //Serial.print("Measured Frequency 1: ");
      //Serial.println(measuredFrequency);
    }
    if (newFrequencyReady2) {
      newFrequencyReady2 = false;  // Reset the flag
      //Serial.print("Measured Frequency 2: ");
      //Serial.println(measuredFrequency2);
    }
    vTaskDelay(pdMS_TO_TICKS(8));
  }
}

void sampleAnalogTask(void *parameter) {
  for (;;) {
    // Subtract the last reading:
    total = total - readings[readIndex];
    // Read from the sensor:
    readings[readIndex] = analogRead(ANALOG_PIN);
    // Add the reading to the total:
    total = total + readings[readIndex];
    // Advance to the next position in the array:
    readIndex = readIndex + 1;

    // If we're at the end of the array...
    if (readIndex >= NUM_READINGS) {
      // ...wrap around to the beginning:
      readIndex = 0;
    }

    // Calculate the average:
    average = total / NUM_READINGS;
    // Check if the average exceeds half the maximum range:
    if (average > MAX_ANALOG_READING / 2) {
      digitalWrite(ERROR_LED_PIN, HIGH);  // Turn on the error LED
    } else {
      digitalWrite(ERROR_LED_PIN, LOW);  // Turn off the error LED
    }

    // Debug output to serial (optional):
    //Serial.print("Average: ");
    //Serial.println(average);

    // Wait for a bit before sampling again:
    vTaskDelay(pdMS_TO_TICKS(20));  // 20ms delay for 50Hz rate
  }
}

void logFrequencyTask(void *parameter) {
  for (;;) {
    // Scale the frequencies. Assuming the full scale is 333Hz to 1000Hz.
    int scaledFreq1 = map(constrain((int)measuredFrequency, 333, 1000), 333, 1000, 0, 99);
    int scaledFreq2 = map(constrain((int)measuredFrequency2, 500, 1000), 500, 1000, 0, 99);

    // Log the scaled frequencies to the serial port
    Serial.print(scaledFreq1);
    Serial.print(",");
    Serial.println(scaledFreq2);

    // Wait 200ms before the next log
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}