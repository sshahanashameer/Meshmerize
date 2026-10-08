# Meshmerize firmware audit report

Date: 2026-10-08

## Status

**Not competition-ready.** The code has received a targeted repair pass and a standalone host regression test passes, but a full PlatformIO target build was not possible because `pio`/`platformio` is not installed in the execution environment. No physical robot, carrier schematic, BOM, competition rulebook, or hardware measurements were supplied. The firmware intentionally boots into a hardware lockout by default.

## Critical findings and changes

1. **Wrong target and pin assumptions** — `platformio.ini`, `include/config.h`, `src/sensors/encoders.cpp`, `src/main.cpp`. Changed the default environment to `esp32-c6-devkitc-1`, removed direct reads of original-ESP32 GPIO register structs in favour of the ESP-IDF GPIO API, removed core-affinity task creation, and added a default-off pin-map interlock. The legacy pin numbers are explicitly not validated or safe for use on the C6. The exact carrier remains unknown.
2. **Dry-run lifecycle contradicted the required run** — `src/runs/dry_run.cpp`, `src/runs/dry_run.h`, `include/config.h`. Removed return-to-start from the dry-run path. Exploration stops when End is detected, stops the motors, and only saves after End plus graph validation. An abort, error, step-limit, or missing End does not commit a completed map.
3. **Actual run could trust a stale in-RAM map** — `src/runs/actual_run.cpp`. It now reloads persisted map data on every actual-run request and validates it before route planning.
4. **Map graph consistency was unchecked** — `src/mapping/maze.cpp`, `src/mapping/maze.h`, `src/navigation/exploration.cpp`. Added validation for node counts/IDs, start/finish flags, exit-state values, neighbor bounds, positive edge lengths, and reciprocal edges. `connect()` refuses conflicting occupied links and exploration reports a failed connection.
5. **Encoder implementation was original-ESP32-specific** — `src/sensors/encoders.cpp`. Replaced register access with `gpio_get_level()` and added interrupt-masked snapshots for coherent tick reads. The transition table still needs physical direction and missed-edge validation on the target board.
6. **QTR polarity was configured for a black line** — `include/config.h`, `src/sensors/qtr8rc.cpp`. Set line polarity for white-on-dark. The code's RC timing produces larger raw values for dark surfaces; calibration normalization followed by inversion makes the calibrated white-line response high. This remains conditional on successful calibration and physical readings.
7. **IMU setup silently ignored important write failures and forced an external crystal** — `src/sensors/bno055.cpp`, `src/main.cpp`, `src/control/drive.cpp`. Configuration writes are checked, the driver no longer forces an unverified external crystal, I2C timeout is bounded, sensor read failure invalidates the IMU, and startup fails closed if the IMU is unavailable. Mounting/orientation and heading sign still require physical tests.
8. **Task and command startup could fail silently or accept queued conflicts** — `src/main.cpp`, `src/control/drive.cpp`. Added checks for task/queue creation, changed command queue capacity to one, reserves the busy state when a command is queued, and makes abort issue a brake command and blocks subsequent wheel commands.
9. **Documentation referred to the original ESP32 and return-to-start behavior** — `README.md`, test comments, task comments. Updated target, lifecycle, lockout, commands, and known limitations.

## Files changed

- `platformio.ini`: C6 environment, consistent native environment naming, and test build isolation to avoid duplicate source definitions.
- `include/config.h`: target note, hardware lockout, white-line polarity, stop-at-End exploration setting.
- `src/sensors/encoders.cpp`: ESP-IDF GPIO API and coherent count snapshots.
- `src/sensors/bno055.cpp`: checked setup writes, internal-clock default, bounded I2C timeout, invalidate on read failure.
- `src/control/drive.cpp`, `src/control/drive.h`: single-core-compatible task creation, fail-closed stale IMU handling, abort brake gate, updated task comments.
- `src/main.cpp`: pin-map lockout, fail-closed IMU/task startup, checked command queue/task creation, safer command queueing.
- `src/mapping/maze.cpp`, `src/mapping/maze.h`: graph validation and conflict-safe links.
- `src/navigation/exploration.cpp`: reject failed/conflicting graph connection.
- `src/runs/dry_run.cpp`, `src/runs/dry_run.h`: Start-to-End lifecycle and commit only after confirmed finish plus validation.
- `src/runs/actual_run.cpp`: always reload and validate persisted map.
- `test/test_pid/test_pid.cpp`, `test/test_ekf/test_ekf.cpp`: target command comments.
- `test/test_qtr8rc/test_qtr8rc.cpp`: corrected manual test prompts to white line on dark floor.
- `test/host_regression.cpp`, `test/host_stubs/Arduino.h`, `test/host_stubs/Preferences.h`: standalone host regression harness for PID bounds, EKF cardinal motion/covariance, graph integrity/capacity, and route planning.
- `README.md`: target, safe lockout, run behavior, test commands, and explicit limits.

## Verification performed

- Standalone C++11 build with `g++ -Wall -Wextra -Werror` of PID, EKF, maze graph, node, and path solver plus test harness: **PASS**.
- Host regression assertions: **PASS** — PID output bounds, EKF cardinal movement and finite covariance, graph reciprocal-link validation, conflicting-link rejection, node capacity, and route reconstruction/length.
- Source-level regression checks: **PASS** — C6 default environment, no original ESP32 GPIO register reads, default-off hardware lockout, no dry-run return-to-start call, save-after-finish/validation path, actual-run reload and validation, no pinned-to-core task creation, IMU fail-closed startup, no stale `esp32dev` test command comments.
- `pio run -e esp32-c6-devkitc-1`: **NOT RUN** — command failed because PlatformIO is not installed (`pio: command not found`).
- PlatformIO Unity test suites: **NOT RUN** — PlatformIO unavailable.
- Hardware-in-the-loop and competition runs: **NOT RUN**.

## Important limitations / remaining risks

- The exact board is only known as ESP32-C6 WROOM-1, not the carrier/development-board model. `esp32-c6-devkitc-1` is a valid PlatformIO board ID for that specific Espressif DevKitC-1, but must be changed if the physical carrier differs.
- `HARDWARE_PINMAP_VERIFIED` remains `false`; firmware intentionally refuses hardware initialization until the complete pin map is verified from the actual schematic. Legacy pins are placeholders and must not be used.
- No official rulebook was supplied. Stopping at first End honours the stated Start-to-End lifecycle, but does not guarantee a globally optimal route if the scoring/rules require more exploration and permit it. Under a strict one-way run, unseen branches cannot be fully mapped without additional information.
- The current map format still serializes raw `Node` structs. It has a version field and validates loaded graph invariants, but ABI padding/layout dependence, CRC, and dual-slot power-loss-safe commits remain outstanding.
- The route cost is still distance plus a configurable turn penalty, not a proven physical-time optimizer. It needs measured turn/straight speeds and repeatability data.
- Existing driver/algorithm test coverage is incomplete: QTR white/black/noise/finish cases, quadrature transition table, motor truth table/brake/coast, BNO055 register decoding and heading wrapping, EKF invalid-input robustness, abort timing, persistence corruption, and state-machine failure cases still need dedicated tests.
- The encoder count snapshots mask interrupts briefly but do not establish a complete end-to-end real-time guarantee. Measure worst-case QTR RC timing, I2C latency, sensor-loop period, missed encoder transitions, and watchdog behavior on the actual board.
- The configured 840 ticks/revolution, wheel geometry, motor inversion, speed constants, line thresholds, finish-box distance, node matching tolerance, and all PID gains remain provisional.

## Safe bring-up order

1. Keep `HARDWARE_PINMAP_VERIFIED = false`.
2. Supply/inspect the exact carrier schematic, BOM, official rulebook, and wiring drawing. Select a pin map avoiding unavailable GPIOs, boot straps, USB/JTAG, flash connections, and conflicts; verify TB6612 STBY has a safe hardware pull-down.
3. Install PlatformIO and run `pio run -e esp32-c6-devkitc-1`; fix all build failures before connecting motors.
4. Verify sensor power/logic levels and I2C wiring with motor power disconnected. Check BNO055 identity, mode, calibration, mounting, and heading direction.
5. With the robot lifted and motor supply disconnected, inspect raw QTR RC timings over both white line and dark floor; calibrate over the real track and confirm normalized white readings dominate.
6. Verify each encoder phase transition and forward sign at low speed; measure ticks per wheel revolution and wheel circumference.
7. Verify TB6612 IN1/IN2, PWM, STBY, brake, coast, and motor inversion at low duty on a stand; confirm emergency stop.
8. Only after all pin and driver checks, set the pin-map flag true, start with conservative speed limits, and test straight motion/90-degree turns on a short test track.
9. Run a small synthetic/physical maze, verify End recognition and persisted map, power-cycle, confirm the same map reloads, then test actual-run rejection for missing/corrupt/incomplete maps.
10. Increase speed only after measured stopping distance, finish detection, line loss recovery, abort response, and repeated route timing are reliable.


## Follow-up hardening pass

- `src/main.cpp`: replaced the unsynchronized `volatile g_busy` flag with a critical-section-protected reservation/read/write; removed `clearAbort()` from the mission task so an abort during the start delay is not erased when the command is dequeued; button sequencing now advances only when posting succeeds; map erase checks Preferences open/clear results.
- `HARDWARE_CONFIG.md`: lists confirmed component/track requirements separately from the missing carrier wiring and required sign-off.
- `VERIFICATION_STATUS.md`: records exact checks and the remaining test gaps.
- `.gitignore`: excludes generated PlatformIO and IDE artifacts.
