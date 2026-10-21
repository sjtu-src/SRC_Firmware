"""Exercise the actual OLED charging strip with a recording display stub."""
from pathlib import Path
import subprocess
r=Path(__file__).resolve().parents[2]
b=r/'MDK-ARM/tmp/charge_oled';b.mkdir(parents=True,exist_ok=True)
s=(r/'Core/Src/misc.c').read_text(encoding='utf-8')
block=s.split('/* The 43..50 strip is reserved in every normal display mode. */')[1].split('\n\tOLED_Update();')[0]
c=r'''
#include "board_monitor.h"
#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#define OLED_6X8 1
static board_status_t state;
static unsigned tick;
static char text[128];
unsigned HAL_GetTick(void) {return tick;}
void board_monitor_get(board_status_t *s) {*s=state;}
void OLED_ClearArea(int x,int y,int w,int h) {assert(x==0 && y==43 && w==128 && h==8);text[0]=0;}
void OLED_Printf(int x,int y,int font,const char *fmt,...) {
    assert(x==0 && y==43 && font==OLED_6X8);
    va_list ap;va_start(ap,fmt);vsnprintf(text,sizeof(text),fmt,ap);va_end(ap);
    assert(strlen(text)<=21); /* 21 six-pixel glyphs fit 128 columns. */
}
static void draw(void) {
'''+block+r'''
}
int main(void) {
    state.charge_enabled=1;draw();assert(!strcmp(text,"CHG:ON"));
    state.charge_enabled=0;
    for(unsigned i=0;i<12;i++) {state.charge_inhibit=1U<<i;state.charge_retry_ms=30000;draw();assert(strstr(text,"CHG:"));}
    state.charge_inhibit=BOARD_CHG_PROGRESS;draw();assert(!strcmp(text,"CHG:NO RISE 30s"));
    state.charge_retry_ms=1;draw();assert(!strcmp(text,"CHG:NO RISE 1s"));
    state.charge_inhibit=BOARD_CHG_ADC|BOARD_CHG_QUIET;
    tick=0;draw();assert(!strcmp(text,"CHG:CAP ADC"));
    tick=1500;draw();assert(!strcmp(text,"CHG:QUIET"));
    tick=3000;draw();assert(!strcmp(text,"CHG:CAP ADC"));
    state.charge_inhibit=0;draw();assert(!strcmp(text,"CHG:WAIT"));
    puts("PASS: OLED labels, countdown, rotation and 128x8 strip bounds");
}
'''
f=b/'test.c';f.write_text(c,encoding='utf-8');exe=b/'test.exe'
subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror','-I'+str(r/'Core/Inc'),str(f),'-o',str(exe)],check=True)
subprocess.run([str(exe)],check=True)
assert '10+i*8' in s  # Last wheel ends at y=41; strip begins at y=43.
