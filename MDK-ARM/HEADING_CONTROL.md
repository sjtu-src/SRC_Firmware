# IMU heading control

The controller is active in `NORMAL_MODE` and `CRAY_MODE` when the robot uses
`SPEED_PID` and the MPU6050 attitude sample is valid and newer than 100 ms.

## Command path

`do_acc_handle_move()` keeps the existing X/Y acceleration limiter, snapshots
the attitude published by the IMU task, and passes the operator rotation command
through `HeadingControl_Update()` before calling `do_move()`.

The corrected rotation command remains floating point through the wheel
kinematics. It is quantized only when each encoder setpoint is written, so a
correction smaller than one protocol rotation-command unit is not discarded.

The operator command remains the angular-rate feedforward. The controller
integrates that rate into a target yaw, so translation and rotation can be
commanded at the same time:

```
target_rate = speed_rot * 0.025 rad/s * MOTION_COMMAND_GAIN
target_yaw += target_rate * dt
desired_rate = target_rate + HEADING_ANGLE_KP * yaw_error
correction = HEADING_RATE_KP * rate_error
           + HEADING_RATE_KI * integrated_rate_error
output = speed_rot + correction
```

`MOTION_COMMAND_GAIN` is also used by the final wheel setpoint conversion in
`do_move()`. It is 8 for `NEUMANN_S02` and 3 for the other existing variants.
This avoids silently using the pre-gain angular rate as the yaw reference.

The rate loop uses `gyro_corrected_dps[2]`, which is the MPU6050 Z-axis rate
after the stationary startup bias has been removed. Raw rate and learned bias
remain available in `g_robot.attitude` for debugging.

When the operator rotation command is zero, a dedicated yaw-hold controller is
used instead of angular-rate tracking:

```
correction = HEADING_HOLD_KP * yaw_error
           + HEADING_HOLD_KI * integrated_yaw_error
           - HEADING_HOLD_KD * measured_yaw_rate
```

The gyro damping reacts to wheel-startup asymmetry. The yaw integral supplies a
continuous correction for direction-dependent wheel or floor imbalance, so the
robot should return to the captured heading instead of accepting a new heading.

## Fail-safe behavior

- Invalid or stale attitude data bypasses correction and preserves the received
  rotation command.
- A full zero command clears the controller and aligns its next target to the
  measured yaw.
- Position PID and non-driving test modes bypass and reset heading control.
- Target lag, correction, integrator, and final rotation command are bounded.
- A long or invalid update interval re-aligns the target instead of integrating
  across missing command packets.

## Initial tuning

The initial gains in `Core/Inc/cfg.h` are deliberately conservative. The tested
robot turns clockwise for a positive rotation command, while the MPU6050 uses
right-handed Z-up yaw and reports clockwise motion as negative. Therefore
`HEADING_SENSOR_YAW_SIGN` is `-1.0f`. This conversion is applied inside the
controller; raw attitude telemetry keeps the sensor convention.

The IMU task runs at 100 Hz with `osPriorityAboveNormal`. Heading control runs
when a command packet arrives (nominally every 15 ms in Cray).

Then test on the floor in this order:

1. Translate with `speed_rot = 0` and confirm the robot resists yaw disturbance.
2. Command translation and a slow constant turn together; target yaw should
   advance continuously without fighting the requested turn.
3. Increase `HEADING_ANGLE_KP` if heading recovery is too slow.
4. Increase `HEADING_RATE_KP` in small steps if angular-rate response is soft.
5. Keep `HEADING_RATE_KI` small; reduce it if direction reversal has a delayed
   response or oscillation.

Live controller state is available at `g_robot.heading_control`, including
target yaw/rate, yaw and rate errors, correction, input/output commands, and
status.
