# Verification status

Status date: 2026-10-08

## Verified in this environment

- ZIP archive extracted and inspected.
- Host regression target compiled with `g++ -std=c++11 -Wall -Wextra -Werror`.
- Host regression test passed: PID output bounds, EKF cardinal motion and covariance finiteness, graph reciprocal consistency/conflicting-link rejection/capacity, and route reconstruction/length.
- Source checks confirmed the ESP32-C6 default PlatformIO environment, no original-ESP32 GPIO register reads, default-off hardware pin-map lockout, dry run does not call `returnToStart()`, actual run reloads and validates persisted map, and no pinned-to-core task creation is used in the sensor task.

## Not verified

- PlatformIO firmware build: not run; `pio`/`platformio` is unavailable in the environment.
- PlatformIO Unity tests: not run for the same reason.
- ESP32-C6 hardware, exact carrier board, pin assignments and electrical levels.
- QTR polarity/thresholds on the physical track; finish-box false-positive rate.
- Encoder direction, quadrature edge loss, ticks/revolution, wheel geometry and motor inversion.
- TB6612 truth table, braking/coasting, standby behavior and emergency-stop latency on the physical robot.
- BNO055 address, mounting axes, calibration and heading signs on the physical robot.
- Sensor/control loop deadlines, watchdog behavior and worst-case RC sampling time.
- Power-cycle persistence and interrupted-write behavior on device.
- Full dry-run/actual-run integration and competition scoring.

## Known remaining software risks

- Maze persistence stores raw `Node` structs. Version and graph checks exist, but explicit field serialization, CRC, and dual-slot power-loss-safe commit are not implemented.
- Mission reservation now uses a FreeRTOS critical section and abort is no longer cleared after the mission task dequeues a command. This reduces the identified command-state race, but concurrent abort timing still needs hardware/integration testing.
- The finish detector is based on a sustained all-sensor activation distance and has not been tested against the physical finish box or wide intersections.
- The route objective uses distance plus configurable turn penalty; it is not a measured traversal-time optimizer.
- The exploration strategy stops at first detected End. It does not guarantee complete mapping or global optimality if unvisited branches remain and the rules allow/require additional exploration.
- Sensor, motor, encoder, BNO055 register, persistence-corruption and state-machine failure test coverage is incomplete.

A successful host test is not evidence of ESP32-C6 compilation or safe physical motion. Keep the hardware lockout enabled until the board pinout is verified and the target build succeeds.
