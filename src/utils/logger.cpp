#include "utils/logger.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <stdarg.h>
#include <stdio.h>

static SemaphoreHandle_t s_mutex = nullptr;
static LogLevel s_level = LogLevel::INFO;

void Logger::begin(unsigned long baud) {
  Serial.begin(baud);
  if (!s_mutex) s_mutex = xSemaphoreCreateMutex();
}

void Logger::setLevel(LogLevel lvl) { s_level = lvl; }

void Logger::log(LogLevel lvl, const char* tag, const char* fmt, ...) {
  if (lvl < s_level) return;
  char msg[192];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(msg, sizeof(msg), fmt, ap);
  va_end(ap);
  static const char kLetter[] = {'D', 'I', 'W', 'E'};
  if (s_mutex) xSemaphoreTake(s_mutex, portMAX_DELAY);
  Serial.printf("[%lu][%c][%s] %s\n", (unsigned long)millis(), kLetter[(uint8_t)lvl], tag, msg);
  if (s_mutex) xSemaphoreGive(s_mutex);
}
