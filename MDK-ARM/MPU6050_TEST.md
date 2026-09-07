# MPU6050 startup test

This document records the earlier initialization-only stage. Continuous sampling,
shared-bus locking and Cray visualization are now described in
[ATTITUDE_TELEMETRY.md](ATTITUDE_TELEMETRY.md); its settings supersede this stage.

Branch: `test/mpu6050-bringup` (based on `Basic`).

## Hardware and reference checks

- Schematic: `E:\SRC\ctrl_board\SRC_CtrlBoard.pdf`, page 1, U3.
- Legacy code: `E:\SRC\Embedded_code\Embedded_Code_2026`:
  `src/i2c.c`, `src/MPU6050_driver.c`, `src/init.c` and `inc/cfg.h`.
- U3 SCL pin 23 connects to PB10; SDA pin 24 connects to PB11.
  These are the current OLED I2C2 pins, not SPI pins.
- AD0 is grounded: 7-bit address `0x68`, shifted HAL address `0xD0`.
  The EEPROM and OLED have different addresses on the same bus.
  The schematic provides 3.3 kOhm pull-ups R1/R2 to 3.3 V.
- INT connects to PC14 and FSYNC to PC15. This startup test uses neither;
  sensor interrupts and external synchronization are disabled.
- The old firmware enables `MPU6050_GYRO_USED` and calls `InitMPU6050`
  during initialization. Subsequent hardware I2C setup and offset calibration
  are limited to joystick/Bluetooth branches; do not copy these mode paths
  into the HAL/FreeRTOS firmware.
- Register reference: [TDK MPU-6000/MPU-6050 register map, revision 4.2](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map.pdf).

## Startup behavior

1. Play the existing `mode + 1` beeps (100 ms on / 100 ms off).
2. Keep the buzzer off for an additional 1000 ms.
3. Check WHO_AM_I, reset the device, wait 100 ms, wake and configure it,
   wait another 100 ms, then verify all written configuration registers.
4. Success: two 150 ms beeps, with a 150 ms gap.
   Failure: one 1000 ms beep.
5. Leave at least 500 ms silence before runtime indications. Initialize
   runtime timers and start the motors, then release communication and display.

All delays above use `osDelay` in `RobotTask`; scheduler interrupts remain
enabled. I2C register transfers use a 20 ms timeout (the HAL bus-busy check
has its own finite 25 ms timeout). Missing hardware does not prevent startup.
The OLED initialization now runs in `Do_Display` after startup releases I2C2.
Communication waits too, so incoming commands cannot overwrite the startup
buzzer pattern or accumulate motion commands while startup is delayed.

The sensor is configured for +/-500 dps, +/-4 g, DLPF setting 4 and a sample
divider of 1, following the old driver ranges/filter. No samples, calibration,
angle integration or motion correction are connected to the control loop.

Inspect `g_mpu6050_status` in the debugger:

| Value | Meaning |
| --- | --- |
| 0 | Not tested |
| 1 | Identity and configuration verified |
| 2 | I2C communication failed / busy / timeout |
| 3 | Unexpected WHO_AM_I |
| 4 | Configuration readback mismatch |

## Validation and board acceptance

- This change passed a full Keil rebuild: 0 errors, 6 reported warnings
  from existing code/tooling. No warnings were reported for the new driver.
- Host tests passed: successful startup, 48 injected bus failures, wrong ID,
  7 configuration readback faults, and a successful retry after failure.
- Test firmware: `output/mpu6050/SRC_Firmware_mpu6050.hex`; matching debug
  image: `output/mpu6050/SRC_Firmware_mpu6050.axf`. Build log is in the same folder.
- Build `SRC_Firmware.uvprojx`, target `SRC_Firmware`, with Keil ARMCC 5.
- Run `python tests/mpu6050/test_init.py` from MDK-ARM for host fault injection.
- On the old board, verify the mode pattern, silence, then two short beeps.
- On a board without MPU6050, verify the mode pattern, silence, then one long
  beep, followed by normal startup. No connection should be changed while powered.
- Repeat in CRAY and another mode to check that differing mode counts do not
  overlap the result. If an OLED is connected, check that it updates afterward.
- Confirm remote control works after the startup sequence on both boards.

Host tests and compilation do not verify physical I2C signals, sound timing,
sensor accuracy or straight-line motion. Hardware acceptance remains required.
