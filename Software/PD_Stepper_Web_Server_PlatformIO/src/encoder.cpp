#include "encoder.h"
#include <Wire.h>

namespace encoder {

#define AS5600_ADDRESS 0x36

static signed long total_encoder_counts = 0;
static int prev_raw_counts = 0;
static signed long revolutions = 0;
static SemaphoreHandle_t encoderMutex = NULL;
static TaskHandle_t encoderTaskHandle = NULL;

void EncoderTask(void *pvParameters);

void init() {
  Wire.begin();
  Wire.setClock(400000); // Fast mode I2C
  if (encoderMutex == NULL) {
    encoderMutex = xSemaphoreCreateMutex();
  }
}

void startTask(int priority, int interval_ms) {
  if (encoderTaskHandle == NULL) {
    xTaskCreatePinnedToCore(EncoderTask, "EncoderTask", 2048,
                            (void *)interval_ms, priority, &encoderTaskHandle,
                            1);
  }
}

void read() {
  int raw_counts = 0;

  Wire.beginTransmission(AS5600_ADDRESS);
  Wire.write(0x0C);
  Wire.endTransmission(false);
  Wire.requestFrom(AS5600_ADDRESS, 2);

  if (Wire.available() >= 2) {
    raw_counts = Wire.read() << 8 | Wire.read();
  } else {
    return; // Fast fail if I2C busy
  }

  if (xSemaphoreTake(encoderMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    // Handle full revolutions clockwise and anti clockwise
    if (prev_raw_counts > 3000 && raw_counts < 1000) {
      revolutions++;
    } else if (prev_raw_counts < 1000 && raw_counts > 3000) {
      revolutions--;
    }

    prev_raw_counts = raw_counts;
    total_encoder_counts = raw_counts + (4096 * revolutions);
    xSemaphoreGive(encoderMutex);
  }
}

void EncoderTask(void *pvParameters) {
  int interval = (int)pvParameters;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(interval);

  while (true) {
    read();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

signed long getTotalCounts() {
  signed long counts = 0;
  if (xSemaphoreTake(encoderMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    counts = total_encoder_counts;
    xSemaphoreGive(encoderMutex);
  }
  return counts;
}

} // namespace encoder
