"""Check actual player with a fixed reference score, independent of song selection."""
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[2]
BUILD = ROOT / 'MDK-ARM/tmp/startup_music'
BUILD.mkdir(parents=True, exist_ok=True)
source = (ROOT / 'Core/Src/freertos.c').read_text(encoding='utf-8')
player = re.search(r'static void Play_Mode3Music\(void\)\n\{.*?^\}', source, re.M | re.S).group()
harness = r'''
#include <stdint.h>
#include <assert.h>
#include <stdio.h>
#include "music.h"
/* Playback fixture: keep user-edited song data and selection independent. */
#undef MODE3_MUSIC
static const music_note_t reference_music[] = {{587,2},{587,2},{0,2},{587,2},{784,4}};
#define MODE3_MUSIC reference_music
struct timer { uint32_t PSC; } regs = {167};
struct { struct timer *Instance; } htim8 = {&regs};
struct { uint32_t CFGR; } rcc = {0x8000};
#define RCC (&rcc)
#define RCC_CFGR_PPRE2 0xe000U
static uint32_t arr=499, elapsed, silent=1, count, stop_at;
static uint32_t starts[4], periods[4], sounded[4];
#define __HAL_TIM_GET_AUTORELOAD(t) arr
#define __HAL_TIM_SET_AUTORELOAD(t,v) (arr=(v))
#define __HAL_TIM_SET_COUNTER(t,v) ((void)0)
#define BEEP_OFF() (silent=1)
static void beep_on(void) {
    assert(count<4); starts[count]=elapsed; periods[count]=arr+1;
    ++count; silent=0;
}
#define BEEP_ON() beep_on()
static uint32_t HAL_RCC_GetPCLK2Freq(void) { return 84000000; }
static int board_actuators_disabled(void) { return stop_at && elapsed>=stop_at; }
static void osDelay(uint32_t ms) {
    assert(ms>0);
    if(!silent) sounded[count-1]+=ms;
    /* The complete underscore interval must remain silent. */
    if(elapsed>=624 && elapsed<936 && !stop_at) assert(silent);
    elapsed+=ms;
}
@PLAYER@
int main(void) {
    assert(MODE3_MUSIC_COUNT==5);
    assert(MODE3_MUSIC[2].frequency_hz==0 && MODE3_MUSIC[2].sixteenths==2);
    Play_Mode3Music();
    assert(count==4 && elapsed==1873 && silent && arr==499);
    assert(starts[0]==0 && starts[1]==312 && starts[2]==936 && starts[3]==1248);
    assert(periods[0]==1704 && periods[1]==1704 && periods[2]==1704 && periods[3]==1276);
    assert(sounded[3]==625); /* One continuous full quarter-note beat. */
    puts("PASS: 2 2 rest 2 5; half-beat rest; final tone continuous 625 ms; total 1873 ms");
    elapsed=count=0; stop_at=500;
    Play_Mode3Music();
    assert(elapsed==624 && count==2 && silent && arr==499);
    puts("PASS: shutdown stops playback and restores original pitch");
}
'''
(BUILD / 'test.c').write_text(harness.replace('@PLAYER@', player), encoding='utf-8')
exe = BUILD / 'test.exe'
subprocess.run(['gcc', '-std=c99', '-Wall', '-Wextra', '-Werror',
                '-I'+str(ROOT/'Core/Inc'), str(BUILD/'test.c'), '-o', str(exe)], check=True)
subprocess.run([str(exe)], check=True)
