#include "debug_log.h"

#include <cstdarg>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static constexpr size_t RING_SIZE = 8192;

static char ring[RING_SIZE];
static size_t ringHead = 0; // next write position
static bool ringWrapped = false;
static SemaphoreHandle_t ringMutex = nullptr;
static portMUX_TYPE initLock = portMUX_INITIALIZER_UNLOCKED;

static SemaphoreHandle_t mutex() {
  if (ringMutex == nullptr) {
    SemaphoreHandle_t created = xSemaphoreCreateMutex();
    portENTER_CRITICAL(&initLock);
    if (ringMutex == nullptr) {
      ringMutex = created;
      created = nullptr;
    }
    portEXIT_CRITICAL(&initLock);
    if (created != nullptr)
      vSemaphoreDelete(created);
  }
  return ringMutex;
}

void debug_log(const char *format, ...) {
  char line[256];
  const int prefix = snprintf(line, sizeof(line), "[%8.1f] ", millis() / 1000.0);
  va_list args;
  va_start(args, format);
  vsnprintf(line + prefix, sizeof(line) - prefix - 1, format, args);
  va_end(args);
  size_t length = strlen(line);
  line[length++] = '\n';

  Serial.write(reinterpret_cast<const uint8_t *>(line), length);

  if (xSemaphoreTake(mutex(), pdMS_TO_TICKS(50)) != pdTRUE)
    return; // never block a time-critical task on the log
  for (size_t i = 0; i < length; i++) {
    ring[ringHead++] = line[i];
    if (ringHead == RING_SIZE) {
      ringHead = 0;
      ringWrapped = true;
    }
  }
  xSemaphoreGive(mutex());
}

void debug_log_dump(String &out) {
  if (xSemaphoreTake(mutex(), pdMS_TO_TICKS(200)) != pdTRUE) {
    out += "(log busy)\n";
    return;
  }
  if (ringWrapped) {
    // Skip the partial line the write head has overwritten into.
    size_t start = ringHead;
    while (start < RING_SIZE && ring[start] != '\n')
      start++;
    for (size_t i = start + 1; i < RING_SIZE; i++)
      out += ring[i];
  }
  for (size_t i = 0; i < ringHead; i++)
    out += ring[i];
  xSemaphoreGive(mutex());
}
