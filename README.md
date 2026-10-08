# Meshmerize maze solver — ESP32-C6 firmware (engineering audit build)

This is an audit/repair build, **not yet competition-ready**. The exact ESP32-C6 carrier-board pinout, BOM/wiring sheet, official rulebook, physical sensor polarity measurements, and wheel/encoder calibration were not present in the supplied repository. For safety, the firmware starts in a hardware lockout until the pin map is verified.

## Target

PlatformIO environment: `esp32-c6-devkitc-1` (appropriate only if the carrier is the Espressif ESP32-C6-DevKitC-1). Platform: `espressif32`; framework: Arduino. If the actual carrier differs, select its correct board ID after checking the board documentation.

```sh
pio run -e esp32-c6-devkitc-1
pio test -e native
pio test -e esp32-c6-devkitc-1
```

The tests compile their selected implementation units directly (`test_build_src = no`) to avoid duplicate definitions from including driver `.cpp` files in Unity tests. These commands have **not been executed in the current environment** because PlatformIO is not installed here. A successful compile also would not validate physical wiring or motion.

## Critical hardware lockout

`include/config.h` deliberately sets `HARDWARE_PINMAP_VERIFIED = false`. The pin constants inherited from the original ESP32 project are legacy placeholders, not a valid ESP32-C6 wiring plan. The program logs a safe-lockout message and does not initialize motor/sensor GPIOs or accept motion commands. Do not set the flag to `true` until every QTR, TB6612, encoder, BNO055, button, and LED pin is selected from the exact carrier schematic and checked for boot-strapping, USB/JTAG, flash, and peripheral conflicts. The BOM and wiring documentation are still needed to finish this safely.

## Run lifecycle

- Dry run starts at Start and explores using the currently implemented graph-based DFS policy.
- It stops at detected End; it does **not** return to Start.
- A map is committed only if End was found and graph validation succeeds. Abort, step-limit, navigation error, or missing End does not save a complete map.
- Actual run reloads the stored map, validates node IDs and reciprocal links, requires Start and Finish, then plans and replays a route. It stops on unexpected events.
- The strategy stops at first detected End. Whether this is optimal depends on the official competition rules; no rulebook was supplied, so a complete map cannot be guaranteed under all rule interpretations.

## Sensor polarity

The QTR-8RC RC timing routine measures discharge time: dark surfaces generally decay more slowly (larger raw values), white surfaces more quickly (smaller raw values). Calibration maps each sensor's observed range to 0–1000; `QTR_LINE_IS_BLACK = false` inverts this so white-line readings should be high. This polarity must still be checked on the actual white-line/dark-floor surface after calibration. Do not drive until the pin map is verified and raw/normalized readings are inspected.

## Serial commands (after hardware lockout is cleared)

- `c` — calibrate QTR sensors
- `d` — dry run, Start to End
- `r` — actual run using validated persisted map
- `m` — print map
- `z` — erase stored map
- `x` — abort and brake
- `t` — toggle telemetry
- `h` — help

The BOOT-button step sequence remains calibration → dry run → actual run. Use serial commands during initial bring-up to avoid accidental step sequencing.

## Known unverified / incomplete items

- The repository contained no official rulebook, BOM, carrier schematic, or exact board model beyond ESP32-C6 WROOM-1.
- The legacy pin constants remain intentionally untrusted and the lockout must remain enabled until verified.
- Encoder ticks/revolution, wheel diameter/base, motor speed, motor polarity, BNO055 mounting/orientation and crystal, QTR thresholds, and all control gains require physical measurement/tuning.
- QTR, motor, encoder, IMU, and navigation code still requires hardware-in-the-loop validation. The current test suite does not cover all requested failure modes.
- Raw `Node` structure persistence remains ABI/layout dependent despite version and graph validation; a future storage format should serialize fields explicitly and use dual-slot/CRC commit for stronger power-loss resilience.
- EKF and mission concurrency, finish-box geometry, edge-length uncertainty, and real-time timing need further targeted tests before any high-speed trial.

## Git repository

The repository is intended to be committed from the `maze_solver/` directory. Generated PlatformIO output under `.pio/` and local IDE files are excluded by `.gitignore`.

Before the first hardware commit, review `HARDWARE_CONFIG.md` and `VERIFICATION_STATUS.md`. The project is a repair/audit baseline, not a certified competition build. Do not disable the hardware lockout or increase speed limits until target compilation, wiring review, and staged physical validation are complete.
