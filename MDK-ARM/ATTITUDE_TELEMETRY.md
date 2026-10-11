# MPU6050 attitude telemetry

## 2026-10-10: this board's Z accelerometer correction

Author: COEX, Xuanting Liu. User measurements while stationary:

- Face up: `(0.0604248047, 0.0278320312, 0.808837891)` g.
- Face down: `(-0.0615234375, -0.00219726562, -1.22680664)` g.

The Z midpoint is -0.208984375 g and the half-span is 1.017822266 g.
`cfg.h` now sets `MPU6050_ACCEL_Z_BIAS_G=-0.208984375f` and
`MPU6050_ACCEL_Z_SCALE=0.982489806f`. The driver applies
`z_corrected = (z_measured - bias) * scale` before the attitude filter.
The existing stationary-calibration and gravity-rejection thresholds stay intact.
The production driver/filter host regression completes calibration with both
measured input vectors after correction.

These are approximate two-face coefficients for this board, not a six-face
calibration or universal MPU6050 parameters. X/Y are unchanged. A replacement
sensor needs its own measurements; bias=0 and scale=1 disable the correction.
Watch `g_robot.attitude.accel_g` now shows corrected values. Flash the freshly
built `SRC_Firmware/SRC_Firmware.hex` under MDK-ARM, not an older exported image,
and verify stationary calibration on the board.

Firmware branch: `test/mpu6050-bringup`.
Cray source: `E:\SRC\Develop_Cray\Cray`, branch `test/mpu6050-attitude`.

## What is measured

MPU6050 provides three-axis acceleration, three-axis angular velocity and die
temperature. It does not directly measure Euler angles, position or magnetic
heading. This implementation fuses acceleration and angular velocity into a
quaternion, then derives roll, pitch and relative yaw.

- Roll/pitch use gravity as a long-term reference. Vehicle acceleration and
  vibration can affect that estimate; samples outside 0.85..1.15 g are excluded
  from gravity correction and flagged in the UI.
- Yaw has no absolute reference because MPU6050 has no magnetometer. It starts
  at zero after stationary calibration and drifts over time. It is not compass
  heading, vision heading or a motion-correction signal.
- Keep the robot stationary until calibration completes (at least 200 samples
  over 2 seconds). Movement resets/rejects the calibration window. Do not use a
  slow continuous turn as a calibration pose: inertial sensors cannot distinguish
  constant slow rotation from gyro bias without an external reference.
- Loss of sensor communication or an integration gap over 100 ms invalidates
  attitude and requires another stationary calibration. Relative yaw then has
  a new zero reference. A missing sensor at boot remains failed until restart.

Register reference: [TDK MPU-6000/MPU-6050 register map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map.pdf).
Quaternion complementary-filter reference: [AHRS Mahony filter documentation](https://ahrs.readthedocs.io/en/latest/filters/mahony.html).

## Firmware implementation

- `Core/Src/MPU6050_driver.c`: existing startup check and coherent 14-byte runtime
  burst reads. 100 Hz sensor output (divider 9), +/-4 g, +/-500 deg/s, DLPF 4.
  Conversions: 8192 LSB/g, 65.5 LSB/(deg/s), temperature = raw/340 + 36.53 C.
- `Core/Src/imu_attitude.c`: stationary bias calibration and normalized quaternion
  complementary filter. Uses actual elapsed time, not an assumed exact interval.
- `Core/Src/freertos.c`: `Update_Imu`, nominal 10 ms task period, below the robot
  and communication task priorities. Starts after the existing startup sounds.
- `Core/Inc/robot.h`: `g_robot.attitude` stores `accel_g[3]`, `gyro_dps[3]`,
  `temperature_c`, `quaternion[4]` (w,x,y,z), `roll_deg`, `pitch_deg`, `yaw_deg`,
  `flags`, `sequence`, and `sample_tick`. Measurements are in sensor axes;
  `gyro_dps` retains the measured value before bias subtraction.
- `Core/Src/i2c.c`: an RTOS mutex with priority inheritance serializes IMU/OLED
  access. OLED transactions have finite timeouts, and screen updates are throttled.
- `Core/Src/comm.c`: atomically snapshots attitude and extends the existing
  `build_game_telemetry_packet`. Does not overwrite an in-flight RF payload.
- `FreeRTOSConfig.h` and `.ioc`: enable `INCLUDE_vTaskDelayUntil`.

No motor PID, movement command or automatic heading correction uses the estimate.
No additional IMU startup beep is introduced beyond the already tested pattern.

## Wire format: both endpoints

Existing framing remains `AA 55 10`, 24 bytes total. Bytes 0..14 are unchanged.

| Byte offset | Field | Encoding |
| --- | --- | --- |
| 0..2 | Header/type | AA 55 10 |
| 3 | Robot ID | `(g_robot.num - 1) & 0x0f`, same as Cray's selected 0-based ID |
| 4..14 | Existing infrared, voltage, wheel speeds | Unchanged |
| 15..16 | Roll | Signed 16-bit little-endian, 0.01 degree |
| 17..18 | Pitch | Signed 16-bit little-endian, 0.01 degree |
| 19..20 | Relative yaw | Signed 16-bit little-endian, 0.01 degree |
| 21 high nibble | Attitude extension version | 0xA |
| 21 bit 0 | Valid estimate | 1 = valid |
| 21 bit 1 | Calibrating | 1 = keep stationary |
| 21 bit 2 | Sensor read failure | 1 = failed |
| 21 bit 3 | Acceleration rejected | 1 = gyro-only update during acceleration disturbance |
| 22 | Sample sequence | Increments for each fresh IMU sample, modulo 256 |
| 23 | Existing CRC8 | Covers bytes 0..22, unchanged algorithm |

Roll/yaw range: -180..180 degrees. Pitch: -90..90 degrees. Cray rejects out-of-range
attitude. Zero-filled reserved bytes from old firmware mean **unsupported**, not a
valid level orientation. Old Cray continues to read existing fields from new packets.
Both copies of `attitude_protocol.h` must stay identical when the format changes.

The existing 13 x 20 ms RF slots are retained: approximately 3.85 packets/s per
robot, subject to other radio traffic/loss. UI smoothing does not increase sensor
or radio measurement rate. Simultaneous robots should have distinct slots (the
existing slot assignment is ID modulo 13); IDs 13..15 are supported for individual
manual tests but share slots with IDs 0..2.

## Cray display

- `src/serialobject.cpp` now decodes these packets in manual and game modes.
- Manual view follows the control panel's robot ID. Game mode has a separate
  attitude robot selector. The 3D view is independent of historical wheel-speed logs.
- `src/qml/AttitudePanel.qml` displays a perspective-projected 3D robot mesh,
  fixed ground grid, sensor axes, angles and status. It uses quaternion shortest-path
  interpolation, including transitions across +/-180 degrees.
- Repeated sample sequence numbers do not refresh sensor age. At 1000 ms without
  a new sample the model dims, angle readings are hidden and a timeout is shown.
  Closing/reopening the serial port clears attitude; unplug errors also invalidate it.
- "显示航向归零" changes only the displayed yaw reference, not firmware state or
  any control command. Changing the selected robot clears that display reference.

The frame is right-handed: the quaternion rotates MPU6050 sensor vectors into a
Z-up world; Euler rotation is `Rz(yaw) * Ry(pitch) * Rx(roll)`. The yellow arrow is
sensor +X. PCB mounting direction relative to robot forward has not been verified.
Before interpreting the model as vehicle-forward coordinates, physically identify
the sensor axes; apply a proper right-handed axis mapping to both acceleration and
gyro vectors in the driver if required. Euler pitch near +/-90 degrees is singular,
but the internal filter and rendered rotation use quaternions.

## Verification completed

- Keil ARMCC 5 firmware build: 0 errors, 5 reported existing-code/tool warnings.
- Cray Qt 5.15.2 / MSVC release build completed; Qt runtime files deployed alongside
  the test executable.
- HAL stub tests: initialization and 48 injected transfer failures; coherent sensor
  bursts, signed conversions, temperature, missing data and mutex release on errors.
- Filter tests: bias removal, known tilts, yaw rotation/wrap, gravity convergence,
  acceleration rejection, failed reads, long sample gaps and tick wrap.
- The real firmware packet builder generates a golden binary frame. Cray's real
  stream parser accepts that same frame and CRC; tests also cover fragmented/mixed
  frames, bad CRC/type, legacy packets, IDs, invalid angles, duplicates, sequence
  wrap, both modes and disconnect clearing.
- The actual main QML window was rendered at 1100x860 and 760x680 with synthetic
  telemetry. Live/stale/disconnected UI states were checked. Screenshots are test
  inputs, not evidence of a hardware attitude measurement.

Re-run firmware host tests from MDK-ARM:

```powershell
python tests/mpu6050/test_init.py
python tests/mpu6050/test_telemetry.py
gcc -std=c99 -Wall -Wextra -Werror -I ../Core/Inc tests/mpu6050/test_attitude.c ../Core/Src/imu_attitude.c -o tests/mpu6050/build/test_attitude.exe
./tests/mpu6050/build/test_attitude.exe
```

Cray test project: `tests/attitude/attitude_integration.pro`. Build using Qt 5.15
and MSVC, then run with the firmware-generated `attitude_golden.bin` and an existing
screenshot directory as arguments. For headless rendering set `QT_QPA_PLATFORM=offscreen`
and `QT_QUICK_BACKEND=software`. `test_math.js` runs with Node.js.

## Hardware test steps

1. Flash `output/attitude/SRC_Firmware_attitude.hex` (or rebuild the Keil target).
2. Run `output/attitude/cray/Cray.exe` from the complete deployed directory.
   Use the same transmitter, serial port, frequency and robot ID as before.
3. Keep the robot stationary through startup and until Cray shows valid attitude.
   In manual mode start the usual command transmission to receive periodic telemetry.
4. Tilt about each sensor axis and rotate slowly: verify angle sign, axes and the
   model direction. Return to level and compare roll/pitch with the original pose.
5. Rotate across +/-180 degrees to verify smooth display. Observe yaw drift during
   a longer stationary period; this stage does not promise absolute heading accuracy.
6. Stop telemetry/disconnect and verify the timeout/disconnected display. Power up
   a board without MPU6050 to verify explicit failure and continued normal control.
7. Check remote control, wheel feedback and optional OLED still operate together.

Only startup connection was hardware-confirmed by the user in the preceding stage.
Continuous sampling, mounting-axis mapping, RF throughput and dynamic attitude
accuracy still require this new firmware/Cray pair to be tested on the board.
