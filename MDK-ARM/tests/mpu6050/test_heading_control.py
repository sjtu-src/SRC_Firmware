"""Build and exercise the real heading controller with extracted firmware config."""

import re
import subprocess
from pathlib import Path


root = Path(__file__).resolve().parents[2]
core = root.parent / "Core"
build = Path(__file__).resolve().parent / "build" / "heading"
include = build / "include"
include.mkdir(parents=True, exist_ok=True)

cfg_text = (core / "Inc" / "cfg.h").read_text(encoding="utf-8")
robot_version = re.search(r"^#define\s+ROBOT_VERSION\s+(\w+)", cfg_text, re.M).group(1)
gain = "8.0f" if robot_version == "NEUMANN_S02" else "3.0f"
heading_defines = re.findall(r"^#define\s+(HEADING_[A-Z0-9_]+)\s+(.+)$", cfg_text, re.M)
(include / "cfg.h").write_text(
    "#define SPEED_PID 1\n#define POSITION_PID 0\n"
    f"#define MOTION_COMMAND_GAIN {gain}\n"
    + "".join(f"#define {name} {value}\n" for name, value in heading_defines),
    encoding="ascii",
)
(include / "imu_attitude.h").write_text("#define IMU_VALID 0x01U\n", encoding="ascii")
(include / "pid.h").write_text("/* Test stub. */\n", encoding="ascii")

source = r'''
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "heading_control.h"
#include "cfg.h"
#include "imu_attitude.h"

#define RAD_TO_DEG 57.29577951308232f

static float update(heading_control_t *c, int x, int y, float rot, int pid,
                    float yaw, float rate, unsigned flags,
                    unsigned sample_tick, unsigned now)
{
    return HeadingControl_Update(c, x, y, rot, pid, yaw, rate,
                                 (unsigned char)flags, sample_tick, now);
}

int main(void)
{
    heading_control_t c;
    float expected_rate = 10.0f * 0.025f * MOTION_COMMAND_GAIN * RAD_TO_DEG;
    float output;

    HeadingControl_Reset(&c);
    output = update(&c, 100, 50, 10, SPEED_PID, 0, 0, 0, 100, 100);
    assert(output == 10.0f && c.status == HEADING_CONTROL_NO_ATTITUDE);

    output = update(&c, 100, 50, 10, SPEED_PID, 0,
                    expected_rate / HEADING_SENSOR_YAW_SIGN,
                    IMU_VALID, 100, 100);
    assert(output == 10.0f && c.status == HEADING_CONTROL_ACTIVE);
    assert(fabsf(c.target_yaw_rate_dps - expected_rate) < 0.01f);

    /* Simultaneous translation and rotation advances the reference by the
     * command's 0.025 rad/s base unit and the final wheel gain.
     */
    output = update(&c, 100, 50, 10, SPEED_PID,
                    expected_rate * 0.1f / HEADING_SENSOR_YAW_SIGN,
                    expected_rate / HEADING_SENSOR_YAW_SIGN,
                    IMU_VALID, 200, 200);
    assert(fabsf(output - 10.0f) < 0.001f);
    assert(fabsf(c.target_yaw_deg - expected_rate * 0.1f) < 0.01f);

    /* A robot lag produces correction in the commanded turn direction. */
    output = update(&c, 100, 50, 10, SPEED_PID, 0, 0,
                    IMU_VALID, 300, 300);
    assert(output > 10.0f && c.correction_command > 0.0f);

    /* Releasing rotation holds the current target while translating. */
    output = update(&c, 100, 50, 0, SPEED_PID, 0, 0,
                    IMU_VALID, 400, 400);
    assert(output > 0.0f && fabsf(c.target_yaw_rate_dps) < 0.001f);

    output = update(&c, 0, 0, 0, SPEED_PID, 7, 0,
                    IMU_VALID, 500, 500);
    assert(output == 0.0f && !c.initialized &&
           fabsf(c.target_yaw_deg - 7 * HEADING_SENSOR_YAW_SIGN) < 0.001f);

    output = update(&c, 10, 0, 3, POSITION_PID, 7, 0,
                    IMU_VALID, 600, 600);
    assert(output == 3.0f && c.status == HEADING_CONTROL_UNSUPPORTED_MODE);

    output = update(&c, 10, 0, -4, SPEED_PID, 7, 0,
                    IMU_VALID, 400, 600);
    assert(output == -4.0f && c.status == HEADING_CONTROL_STALE_ATTITUDE);

    /* Preserve a correction below one command unit. The old integer return
     * rounded this useful 0.16-unit correction to zero.
     */
    HeadingControl_Reset(&c);
    update(&c, 100, 0, 0.0f, SPEED_PID, 0, 0,
           IMU_VALID, 700, 700);
    output = update(&c, 100, 0, 0.0f, SPEED_PID,
                    -1.0f / HEADING_SENSOR_YAW_SIGN, 0,
                    IMU_VALID, 710, 710);
    assert(output > 0.1f && output < 1.0f);

    /* Persistent yaw error increases the hold correction without changing the
     * captured target. This applies identically to reverse and lateral motion.
     */
    {
        float first = output;
        unsigned i;
        for (i = 1; i <= 100; ++i)
            output = update(&c, -100, 70, 0.0f, SPEED_PID,
                            -1.0f / HEADING_SENSOR_YAW_SIGN, 0,
                            IMU_VALID, 710 + i * 10, 710 + i * 10);
        assert(output > first + 0.02f);
        assert(fabsf(c.target_yaw_deg) < 0.001f);
    }

    /* Gyro damping opposes an unwanted turn before yaw error accumulates. */
    HeadingControl_Reset(&c);
    update(&c, 0, 100, 0.0f, SPEED_PID, 0, 0,
           IMU_VALID, 1800, 1800);
    output = update(&c, 0, 100, 0.0f, SPEED_PID, 0,
                    5.0f / HEADING_SENSOR_YAW_SIGN,
                    IMU_VALID, 1810, 1810);
    assert(output < -0.3f);

    puts("PASS: gain-aware reference, mixed translation/rotation, hold, bypass, stale data");
    return 0;
}
'''
test_c = build / "test_heading_control.c"
test_c.write_text(source, encoding="ascii")
exe = build / "test_heading_control.exe"
subprocess.run(
    [
        "gcc", "-std=c99", "-Wall", "-Wextra", "-Werror",
        "-I", str(include), "-I", str(core / "Inc"),
        str(test_c), str(core / "Src" / "heading_control.c"),
        "-o", str(exe),
    ],
    check=True,
)
subprocess.run([str(exe)], check=True)
