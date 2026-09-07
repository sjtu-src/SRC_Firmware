"""Compile the actual firmware packet builder with fake robot/ADC inputs.

Writes a binary golden frame for the Cray parser integration test.
"""
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[3]
source = (ROOT/'Core/Src/comm.c').read_text(encoding='utf-8')

def function(name):
    start = source.index('static ', source.index(name)-30)
    opening = source.index('{', start)
    level = 1
    end = opening+1
    while level:
        level += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]

defines = '\n'.join(re.findall(r'^#define GAME_TELEMETRY_.*', source, re.M))
table = re.search(r'static const uint8_t game_crc8_table\[.*?\};',source,re.S).group()
stub = r'''
#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>
#include "imu_attitude.h"
#include "attitude_protocol.h"
#define PACKET_LEN_UP 24
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
struct { int is_ball_detected; struct {int cur_speed;} wheels[4]; imu_attitude_t attitude; } g_robot;
static uint8_t get_bat_v(void) { return 100; }
static uint8_t get_cap_v(void) { return 120; }
static uint32_t HAL_GetTick(void) { return 1010; }
'''
main = r'''
int main(int argc, char **argv) {
    uint8_t frame[24]; FILE *file;
    assert(argc == 2);
    g_robot.is_ball_detected=1;
    g_robot.wheels[0].cur_speed=-1230;
    g_robot.wheels[1].cur_speed=4560;
    g_robot.attitude.roll_deg=-12.34f;
    g_robot.attitude.pitch_deg=45.67f;
    g_robot.attitude.yaw_deg=-179.99f;
    g_robot.attitude.flags=IMU_VALID;
    g_robot.attitude.sequence=254;
    g_robot.attitude.sample_tick=1000;
    build_game_telemetry_packet(frame,3);
    assert(frame[0]==0xaa && frame[1]==0x55 && frame[2]==0x10 && frame[3]==3);
    assert(frame[4]==1 && frame[5]==100 && frame[6]==120);
    assert(frame[7]==0x85 && frame[8]==0xff && frame[9]==0xc8 && frame[10]==1);
    assert(frame[15]==0x2e && frame[16]==0xfb && frame[17]==0xd7 && frame[18]==0x11);
    assert(frame[19]==0xb1 && frame[20]==0xb9 && frame[21]==0xa1 && frame[22]==254);
    file=fopen(argv[1],"wb"); assert(file); assert(fwrite(frame,1,24,file)==24); fclose(file);
    g_robot.attitude.sample_tick=0;
    build_game_telemetry_packet(frame,3);
    assert(frame[21]==0xa0);
    puts("PASS: real firmware packet builder, legacy fields, LE angles, version/sequence, stale validity");
    return 0;
}
'''
build = Path(__file__).resolve().parent/'build'
build.mkdir(exist_ok=True)
test = build/'telemetry.c'
test.write_text(stub+defines+'\n'+table+'\n'+function('game_crc8_calc')+'\n'+function('build_game_telemetry_packet')+'\n'+main)
exe = build/'telemetry.exe'
subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror','-I',str(ROOT/'Core/Inc'),str(test),'-o',str(exe)],check=True)
subprocess.run([str(exe),str(build/'attitude_golden.bin')],check=True)
