"""Exercise the actual legacy packet builder for every DIP ID and radio mode.

Author: COEX, Xuanting Liu
"""
from pathlib import Path
import re
import subprocess

root = Path(__file__).resolve().parents[3]
source = (root / 'Core/Src/packet.c').read_text(encoding='utf-8')
builder = re.search(r'void packet\(char \*q\)\n\{.*?^\}', source, re.S | re.M).group()
stub = r'''
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
typedef unsigned char u8;
#define NORMAL_MODE 0
#define CRAY_MODE 7
#define DIS_INT() ((void)0)
static struct {
    int is_ball_detected, num, mode;
    u8 bat_v, cap_v;
    struct {int cur_speed;} wheels[4];
} g_robot;
static int shooter, packet_flag, frequency;
static u8 get_bat_v(void) {return 100;}
static u8 get_cap_v(void) {return 150;}
'''
main = r'''
int main(void)
{
    char q[24];
    const int modes[] = {NORMAL_MODE, CRAY_MODE};
    g_robot.is_ball_detected = 1;
    frequency = 8;
    for (unsigned mode = 0; mode < 2; ++mode) {
        g_robot.mode = modes[mode];
        for (int dip = 0; dip < 16; ++dip) {
            g_robot.num = dip + 1;
            memset(q, 0, sizeof q);
            packet(q);
            assert((u8)q[0] == 0xff && q[1] == 2);
            assert(q[2] == dip);
            assert(q[3] == 0x40 && q[4] == 100 && (u8)q[5] == 150);
            assert((u8)q[6] == 0xf0 && (u8)q[23] == 0x87);
            assert(packet_flag == 1);
        }
    }
    puts("PASS: actual FF02 builder, DIP 0..15 in both modes, infrared/voltage/frequency preserved");
}
'''
build = Path(__file__).parent / 'build'
build.mkdir(exist_ok=True)
c = build / 'test_feedback_id.c'
exe = build / 'test_feedback_id.exe'
c.write_text(stub + builder + main, encoding='utf-8')
subprocess.run(['gcc', '-std=c99', '-Wall', '-Wextra', '-Werror', str(c), '-o', str(exe)], check=True)
subprocess.run([str(exe)], check=True)
