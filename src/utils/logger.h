#pragma once
#include <stdint.h>

enum class LogLevel : uint8_t { DEBUG = 0, INFO = 1, WARN = 2, ERROR = 3 };

// Thread-safe printf-style logger over Serial (mutex protected, usable from any task).
class Logger {
 public:
  static void begin(unsigned long baud = 115200);
  static void setLevel(LogLevel lvl);
  static void log(LogLevel lvl, const char* tag, const char* fmt, ...) __attribute__((format(printf, 3, 4)));
};

#define LOGD(tag, fmt, ...) Logger::log(LogLevel::DEBUG, tag, fmt, ##__VA_ARGS__)
#define LOGI(tag, fmt, ...) Logger::log(LogLevel::INFO, tag, fmt, ##__VA_ARGS__)
#define LOGW(tag, fmt, ...) Logger::log(LogLevel::WARN, tag, fmt, ##__VA_ARGS__)
#define LOGE(tag, fmt, ...) Logger::log(LogLevel::ERROR, tag, fmt, ##__VA_ARGS__)
