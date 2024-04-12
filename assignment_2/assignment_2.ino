extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp32/rom/ets_sys.h"  // For ets_delay_us()
#include "freertos/queue.h"     // Include for the Queue
#include "freertos/semphr.h"    // Include for the Semaphore
}

#define SIGNAL_PIN 2               // Digital output pin
#define SQUARE_WAVE_INPUT_PIN 34   // Measure pin 1
#define SQUARE_WAVE_INPUT_PIN2 33  // Measure pin 2
#define ANALOG_PIN 34              // Analogue input pin
#define ERROR_LED_PIN 3            // LED error pin
#define MAX_ANALOG_READING 4095    // Maximum reading for a 12-bit ADC
#define NUM_READINGS 10            // Number of readings to average
#define LED_PIN 13                 // Toggle LED pin
#define BUTTON_PIN 22              // Button pin

typedef struct {
  float frequency1;
  float frequency2;
} FrequencyMeasurements;

FrequencyMeasurements freqMeasurements;
SemaphoreHandle_t freqMeasurementsMutex;
QueueHandle_t buttonEventQueue;

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

volatile bool ledState = false;      // Track the LED state
unsigned long lastDebounceTime = 0;  // Last time the output pin was toggled
unsigned long debounceDelay = 50;    // Debounce time

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

void CPU_work(int time) {
  unsigned long endTime = millis() + time;
  while (millis() < endTime) {
    volatile int dummy = 0;
    dummy++;
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(SIGNAL_PIN, OUTPUT);

  // Initialise semaphore and queue
  freqMeasurementsMutex = xSemaphoreCreateMutex();
  buttonEventQueue = xQueueCreate(10, sizeof(bool));

  // Task creation
  xTaskCreate(digitalSignalTask, "Digital Signal Output", 10000, NULL, 2, NULL);
  xTaskCreate(frequencyMeasurementTask, "Frequency Measurement", 2048, NULL, 3, NULL);
  xTaskCreate(sampleAnalogTask, "Sample Analog Input", 2048, NULL, 2, NULL);
  xTaskCreate(logFrequencyTask, "Log Frequency", 2048, NULL, 4, NULL);
  xTaskCreate(handleButtonTask, "Handle Button", 2048, NULL, 1, NULL);
  xTaskCreate(controlLedTask, "Control LED", 2048, NULL, 1, NULL);
  xTaskCreate(CPUWorkTask, "CPU Work", 2048, NULL, 1, NULL);

  pinMode(SQUARE_WAVE_INPUT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(SQUARE_WAVE_INPUT_PIN), onRisingEdge, RISING);
  pinMode(SQUARE_WAVE_INPUT_PIN2, INPUT);
  attachInterrupt(digitalPinToInterrupt(SQUARE_WAVE_INPUT_PIN2), onRisingEdge2, RISING);
  pinMode(ERROR_LED_PIN, OUTPUT);
  pinMode(ANALOG_PIN, INPUT);
  for (int thisReading = 0; thisReading < NUM_READINGS; thisReading++) {
    readings[thisReading] = 0;
  }
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
}

void loop() {
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
      newFrequencyReady = false;
      if (xSemaphoreTake(freqMeasurementsMutex, portMAX_DELAY) == pdTRUE) {
        freqMeasurements.frequency1 = measuredFrequency;
        xSemaphoreGive(freqMeasurementsMutex);
      }
    }
    if (newFrequencyReady2) {
      newFrequencyReady2 = false;
      if (xSemaphoreTake(freqMeasurementsMutex, portMAX_DELAY) == pdTRUE) {
        freqMeasurements.frequency2 = measuredFrequency2;
        xSemaphoreGive(freqMeasurementsMutex);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(8));
  }
}


void sampleAnalogTask(void *parameter) {
  for (;;) {
    total = total - readings[readIndex];
    readings[readIndex] = analogRead(ANALOG_PIN);
    total = total + readings[readIndex];
    readIndex = readIndex + 1;

    // Iterate back through array by wrapping around
    if (readIndex >= NUM_READINGS) {
      readIndex = 0;
    }

    average = total / NUM_READINGS;
    if (average > MAX_ANALOG_READING / 2) {
      digitalWrite(ERROR_LED_PIN, HIGH);  // Turn on the error LED
    } else {
      digitalWrite(ERROR_LED_PIN, LOW);  // Turn off the error LED
    }

    vTaskDelay(pdMS_TO_TICKS(20));  // 20ms delay for 50Hz rate
  }
}

void logFrequencyTask(void *parameter) {
  for (;;) {
    FrequencyMeasurements localFreqMeasurements;

    if (xSemaphoreTake(freqMeasurementsMutex, portMAX_DELAY) == pdTRUE) {
      localFreqMeasurements = freqMeasurements;  // Copy the structure locally
      xSemaphoreGive(freqMeasurementsMutex);
    }

    int scaledFreq1 = map(constrain((int)localFreqMeasurements.frequency1, 333, 1000), 333, 1000, 0, 99);
    int scaledFreq2 = map(constrain((int)localFreqMeasurements.frequency2, 500, 1000), 500, 1000, 0, 99);

    Serial.print(scaledFreq1);
    Serial.print(",");
    Serial.println(scaledFreq2);

    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void handleButtonTask(void *parameter) {
  bool buttonPressed = false;
  for (;;) {
    if (digitalRead(BUTTON_PIN) == LOW) {  // Button press detected
      if (!buttonPressed) {
        buttonPressed = true;
        xQueueSend(buttonEventQueue, &buttonPressed, portMAX_DELAY);
      }
    } else {
      buttonPressed = false;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void controlLedTask(void *parameter) {
  bool buttonPressed;
  for (;;) {
    if (xQueueReceive(buttonEventQueue, &buttonPressed, portMAX_DELAY) == pdPASS) {
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState ? HIGH : LOW);
    }
  }
}

void CPUWorkTask(void *parameter) {
  for (;;) {
    CPU_work(2);  // Simulate CPU work for about 2ms

    vTaskDelay(pdMS_TO_TICKS(20) - 2);  // Adjust the delay to account for work time
  }
}
