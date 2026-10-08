# Hardware configuration and bring-up gate

## Confirmed from the project requirements

- MCU module: ESP32-C6-WROOM-1
- Reflectance array: QTR-8RC, eight channels
- Motor driver: TB6612FNG dual H-bridge
- Motors: two N20 6 V, 200 RPM geared DC motors with quadrature encoders
- IMU: BNO055
- Chassis: differential drive
- Track: white line on dark floor, approximately 30 mm line width, 90-degree connections
- Finish area: approximately 400 x 400 mm solid box

## Not established by the source ZIP

The uploaded ZIP contains firmware source/configuration, but no carrier-board schematic, pin-to-pin wiring diagram, BOM with regulator/logic-level details, or official rulebook. The MCU module and peripheral model names do not uniquely identify the carrier pinout or wiring.

The GPIO constants in `include/config.h` are inherited legacy values and are **not an approved ESP32-C6 pin map**. Do not wire or flash the robot based on those values. `HARDWARE_PINMAP_VERIFIED` must remain `false` until the actual carrier schematic and wiring are checked.

## Required sign-off before enabling hardware

- [ ] Exact ESP32-C6 carrier/development-board model and revision recorded.
- [ ] Every QTR-8RC output and LEDON connection mapped to a valid, available GPIO.
- [ ] TB6612FNG PWMA/PWMB, AIN1/AIN2, BIN1/BIN2, STBY and motor supply wiring verified.
- [ ] STBY has a safe hardware pull-down so the bridge remains disabled during reset/boot.
- [ ] Encoder output voltage is safe for ESP32-C6 GPIO and encoder pull-ups are suitable for 3.3 V.
- [ ] BNO055 SDA/SCL, address strap, supply and logic levels verified.
- [ ] GPIO conflicts with boot strapping, USB/JTAG, flash, and carrier peripherals checked.
- [ ] Wheel diameter, wheelbase, encoder counts per wheel revolution and motor polarity measured.
- [ ] QTR calibration ranges recorded on the actual white line and dark floor.

Only after all checks pass should the pin map be entered in `include/config.h`, reviewed, and the interlock deliberately enabled.
