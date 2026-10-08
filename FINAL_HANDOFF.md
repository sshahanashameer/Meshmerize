# Meshmerize Maze Solver — Engineering Handoff

## What this package is

This is the most complete maze-solver codebase in the supplied files, and it is the recommended development baseline over the uploaded `Meshmerize_project.zip` bring-up project. It contains the maze graph, exploration controller, route planner, dry-run/actual-run flow, sensor/drive modules, and host regression tests.

**It is not certified as working firmware and is not ready to flash and drive without the checks below.** Do not represent this archive as a verified competition release.

## Verified here

The host regression executable compiled with:

```sh
g++ -std=c++11 -Wall -Wextra -Werror -I test/host_stubs -I include -I src test/host_regression.cpp src/control/pid.cpp src/localization/ekf.cpp src/mapping/node.cpp src/mapping/maze.cpp src/navigation/path_solver.cpp -o host_test
./host_test
```

Result: `host_regression: PASS (PID bounds, EKF cardinal motion/covariance, graph integrity, route planning)`.

## Not verified here

- ESP32-C6 PlatformIO firmware build (PlatformIO was unavailable in the validation environment).
- Exact board/carrier pinout, voltage levels, electrical safety, or wiring.
- Physical QTR polarity/calibration, encoder direction/counts, motor polarity, BNO055 mounting/address, finish-box detection, or closed-loop driving.
- Full dry-run/actual-run integration on the robot, persistent storage across power loss, or competition performance.

## Required before motor operation

1. Confirm the exact ESP32-C6 board/carrier and map every signal in `include/config.h` against the actual schematic. Do not copy legacy ESP32 pin values blindly.
2. Keep `cfg::HARDWARE_PINMAP_VERIFIED = false` until all pins are verified and conflict-checked. Only then may the team set it to `true`.
3. Build the target with `pio run -e esp32-c6-devkitc-1`. If the actual board is not that target, select the correct PlatformIO board ID first.
4. Run host tests and review `VERIFICATION_STATUS.md` and `HARDWARE_CONFIG.md`.
5. Bring up sensors with motors disabled, then test motor outputs with wheels lifted at low duty, verify abort/braking, and only then test at low speed on the track.
6. Validate the dry-run map and actual-run replay after a power cycle before any competition run.

## Important behavior / limitations

- Dry run is intended to go Start → detected End and stop; it does not return to Start.
- The current exploration policy stops at the first detected End and does not guarantee every branch is mapped or the globally fastest path is known.
- Route cost is distance plus a configured turn penalty, not a measured time-optimal model.
- Map persistence currently stores raw `Node` structs; explicit serialization, CRC, and dual-slot power-loss-safe commits remain recommended work.

## Build and test commands

```sh
pio run -e esp32-c6-devkitc-1
pio test -e native
pio test -e esp32-c6-devkitc-1
```

A successful compile or host test does not prove safe physical motion. Follow staged hardware validation.
