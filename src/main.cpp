// =============================================================================
//  Line-maze solver  -  ESP32-C6 / FreeRTOS
//
//   Sensor task (single-core compatible)   (prio 5, 200 Hz)  QTR-8RC + encoders + BNO055 -> EKF -> RobotState
//   Mission task  (prio 3)          calibrate / dry run / actual run (blocking logic)
//   Arduino loop()        (prio 1)          serial commands, button, LED, telemetry
//
//  Serial commands:  c calibrate | d dry run | r actual run | m print map | z erase map
//                    x abort | t toggle telemetry | h help
//  BOOT button: idle -> next step (calibrate -> dry run -> actual run); running -> abort.
// =============================================================================
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <Preferences.h>
#include "config.h"
#include "sensors/qtr8rc.h"
#include "sensors/encoders.h"
#include "sensors/bno055.h"
#include "motors/motor_driver.h"
#include "localization/ekf.h"
#include "control/drive.h"
#include "mapping/maze.h"
#include "runs/dry_run.h"
#include "runs/actual_run.h"
#include "utils/logger.h"

static const char* TAG = "main";

static QTR8RC qtr;
static Encoders enc;
static BNO055 imu;
static MotorDriver motors;
static EKF ekf;
static Drive drive(qtr, enc, imu, motors, ekf);
static Maze maze;

enum class Cmd : uint8_t { CALIBRATE, DRY_RUN, ACTUAL_RUN, PRINT_MAZE, CLEAR_MAZE };
static QueueHandle_t g_cmdQueue = nullptr;
static bool g_busy = false;
static portMUX_TYPE g_cmdMux = portMUX_INITIALIZER_UNLOCKED;
static bool g_telemetry = false;

static bool busyGet() {
  portENTER_CRITICAL(&g_cmdMux);
  const bool value = g_busy;
  portEXIT_CRITICAL(&g_cmdMux);
  return value;
}

static void busySet(bool value) {
  portENTER_CRITICAL(&g_cmdMux);
  g_busy = value;
  portEXIT_CRITICAL(&g_cmdMux);
}

static void missionTask(void*) {
  Cmd c;
  for (;;) {
    if (xQueueReceive(g_cmdQueue, &c, portMAX_DELAY) != pdTRUE) continue;
    switch (c) {
      case Cmd::CALIBRATE:
        LOGI(TAG, "calibrating: keep the array over the line");
        vTaskDelay(pdMS_TO_TICKS(cfg::START_DELAY_MS));
        drive.calibrateSensors();
        break;
      case Cmd::DRY_RUN: {
        LOGI(TAG, "dry run starts in %lu ms", (unsigned long)cfg::START_DELAY_MS);
        vTaskDelay(pdMS_TO_TICKS(cfg::START_DELAY_MS));
        DryRun run(drive, maze);
        run.run();
        break;
      }
      case Cmd::ACTUAL_RUN: {
        LOGI(TAG, "actual run starts in %lu ms", (unsigned long)cfg::START_DELAY_MS);
        vTaskDelay(pdMS_TO_TICKS(cfg::START_DELAY_MS));
        ActualRun run(drive, maze);
        run.run();
        break;
      }
      case Cmd::PRINT_MAZE:
        if (maze.size() == 0) maze.load();
        maze.print();
        break;
      case Cmd::CLEAR_MAZE: {
        maze.clear();
        Preferences p;
        if (!p.begin("maze", false)) {
          LOGE(TAG, "could not open NVS namespace; map erase failed");
          break;
        }
        const bool erased = p.clear();
        p.end();
        if (erased) LOGI(TAG, "map erased");
        else LOGE(TAG, "NVS clear failed; map erase not confirmed");
        break;
      }
    }
    drive.stop();
    busySet(false);
  }
}

static bool post(Cmd c) {
  if (g_cmdQueue == nullptr) { LOGE(TAG, "command queue unavailable"); return false; }

  // Reserve the single mission slot atomically across loop() and missionTask.
  portENTER_CRITICAL(&g_cmdMux);
  if (g_busy) {
    portEXIT_CRITICAL(&g_cmdMux);
    LOGW(TAG, "busy: send 'x' to abort first");
    return false;
  }
  g_busy = true;
  portEXIT_CRITICAL(&g_cmdMux);

  // Clear an old abort only after this command has exclusively reserved the slot.
  // Do not clear it in missionTask: an abort received during the start delay must persist.
  if (c == Cmd::CALIBRATE || c == Cmd::DRY_RUN || c == Cmd::ACTUAL_RUN) drive.clearAbort();
  if (xQueueSend(g_cmdQueue, &c, 0) != pdTRUE) {
    busySet(false);
    LOGE(TAG, "command queue full");
    return false;
  }
  return true;
}

static void handleSerial() {
  while (Serial.available()) {
    const int ch = Serial.read();
    switch (ch) {
      case 'c': post(Cmd::CALIBRATE); break;
      case 'd': post(Cmd::DRY_RUN); break;
      case 'r': post(Cmd::ACTUAL_RUN); break;
      case 'm': post(Cmd::PRINT_MAZE); break;
      case 'z': post(Cmd::CLEAR_MAZE); break;
      case 'x': drive.abort(); motors.brake(); break;
      case 't': g_telemetry = !g_telemetry; break;
      case 'h':
        Serial.println("c calibrate | d dry run | r actual run | m map | z erase map | x abort | t telemetry");
        break;
      default: break;
    }
  }
}

void setup() {
  Logger::begin(115200);
  delay(300);
  LOGI(TAG, "maze solver booting");

  if (!cfg::HARDWARE_PINMAP_VERIFIED) {
    LOGE(TAG, "SAFE LOCKOUT: ESP32-C6 carrier pin map is unverified. Update config.h from the actual board schematic before enabling hardware.");
    return;
  }
  motors.begin();
  motors.brake();
  qtr.begin();
  if (qtr.loadCalibration()) LOGI(TAG, "QTR calibration loaded from NVS");
  else LOGW(TAG, "QTR not calibrated: press 'c' with the array over the line");
  enc.begin();
  const bool imuOk = imu.begin();
  if (!imuOk) {
    LOGE(TAG, "BNO055 unavailable: fail-closed; motors disabled until IMU is healthy");
    motors.enable(false);
    return;
  }
  LOGI(TAG, "BNO055 ready (mode 0x%02X)", cfg::BNO_MODE);

  pinMode(cfg::PIN_LED, OUTPUT);
  pinMode(cfg::PIN_BUTTON, INPUT_PULLUP);

  if (!drive.start(imuOk)) { LOGE(TAG, "sensor task failed to start; motors remain disabled"); motors.enable(false); return; }
  if (maze.load()) LOGI(TAG, "stored map loaded: %u nodes", maze.size());

  g_cmdQueue = xQueueCreate(1, sizeof(Cmd));
  if (g_cmdQueue == nullptr) { LOGE(TAG, "command queue creation failed"); motors.enable(false); return; }
  if (xTaskCreate(missionTask, "mission", 10240, nullptr, cfg::MISSION_TASK_PRIO, nullptr) != pdPASS) {
    LOGE(TAG, "mission task creation failed"); motors.enable(false); return;
  }
  LOGI(TAG, "ready: send 'h' for commands");
}

void loop() {
  if (!cfg::HARDWARE_PINMAP_VERIFIED) { delay(1000); return; }
  handleSerial();

  static bool lastBtn = false;
  static uint32_t lastBtnMs = 0;
  static uint8_t step = 0;
  const bool pressed = digitalRead(cfg::PIN_BUTTON) == LOW;
  if (pressed && !lastBtn && millis() - lastBtnMs > 250) {
    lastBtnMs = millis();
    if (busyGet()) {
      drive.abort();
      motors.brake();
    } else {
      static const Cmd seq[3] = {Cmd::CALIBRATE, Cmd::DRY_RUN, Cmd::ACTUAL_RUN};
      if (post(seq[step]) && step < 2) step++;
    }
  }
  lastBtn = pressed;

  const uint32_t period = busyGet() ? 100 : 500;
  digitalWrite(cfg::PIN_LED, (millis() / period) & 1);

  static uint32_t lastTel = 0;
  if (g_telemetry && millis() - lastTel > 50) {
    lastTel = millis();
    const RobotState s = drive.state();
    Serial.printf("T,%lu,%.1f,%.1f,%.1f,%u,0x%02X,%.0f,%.0f\n", (unsigned long)millis(), s.x, s.y,
                  s.theta * 57.2957795f, s.linePos, s.darkMask, s.speedL, s.speedR);
  }
  vTaskDelay(pdMS_TO_TICKS(10));
}
