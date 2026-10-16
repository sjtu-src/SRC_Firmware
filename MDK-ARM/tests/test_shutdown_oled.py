"""Render the production shutdown UI using the actual OLED font and drawing code.

Author: COEX, Xuanting Liu
"""
from pathlib import Path
import re
import subprocess
from PIL import Image, ImageDraw

root = Path(__file__).resolve().parents[2]
core = root / 'Core'
build = root / 'MDK-ARM/tmp/shutdown_oled'
build.mkdir(exist_ok=True)
def extract(path, name):
    source = path.read_text(encoding='utf-8')
    result = re.search(r'(?m)^[\w ]+\b' + name + r'\([^\n]*\)\s*\n\{.*?^\}', source, re.S | re.M)
    assert result, name
    return result.group()

stub = r'''
#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>
#include "board_monitor.h"
#include "oledfont.h"
#define OLED_6X8 6
#define OLED_8X16 8
uint8_t OLED_DisplayBuf[8][128];
static uint32_t tick;
static unsigned updates;
static board_status_t status;
uint32_t HAL_GetTick(void) {return tick;}
void board_monitor_get(board_status_t *out) {*out=status;}
void OLED_Update(void) {++updates;}
void OLED_ClearArea(uint8_t,uint8_t,uint8_t,uint8_t);
void OLED_ShowImage(uint8_t,uint8_t,uint8_t,uint8_t,const uint8_t*);
void OLED_ShowChar(uint8_t,uint8_t,char,uint8_t);
void OLED_ShowString(uint8_t,uint8_t,char*,uint8_t);
void OLED_Clear(void);
static void save(const char *name) {
    FILE *out=fopen(name,"wb");assert(out);
    fprintf(out,"P5\n128 64\n255\n");
    for(unsigned y=0;y<64;y++)for(unsigned x=0;x<128;x++)
        fputc((OLED_DisplayBuf[y/8][x]&(1U<<(y%8)))?255:0,out);
    fclose(out);
}
'''
main = r'''
int main(void) {
    status.shutdown_state=BOARD_SHUTDOWN_DISCHARGING;
    for(unsigned phase=0;phase<4;phase++) {
        tick=phase*400;shutdown_display();assert(updates==phase+1);
        char name[32];snprintf(name,sizeof(name),"discharging_%u.pgm",phase);save(name);
        for(unsigned i=0;i<3;i++) {tick+=100;shutdown_display();assert(updates==phase+1);}
    }
    status.shutdown_state=BOARD_SHUTDOWN_FAILED;
    shutdown_display();assert(updates==5);save("failure.pgm");
    tick+=4000;shutdown_display();assert(updates==5);
    status.shutdown_state=BOARD_SHUTDOWN_KILL;
    shutdown_display();assert(updates==6);save("kill.pgm");
    tick+=4000;shutdown_display();assert(updates==6);
    puts("PASS: actual OLED rendering, four dots phases, failure/kill screen, 400ms refresh and static-screen caching");
}
'''
draw = '\n'.join(extract(core / 'Src/oled.c', n) for n in
                 ('OLED_Clear', 'OLED_ClearArea', 'OLED_ShowImage', 'OLED_ShowChar', 'OLED_ShowString'))
c = build / 'oled_test.c'
c.write_text(stub + draw + extract(core / 'Src/misc.c', 'shutdown_display') + main, encoding='utf-8')
exe = build / 'oled_test.exe'
subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror','-Wno-missing-braces','-I',str(core/'Inc'),
                str(c),str(core/'Src/oledfont.c'),'-o',str(exe)],check=True)
subprocess.run([str(exe)],cwd=build,check=True)
names = ['discharging_0','discharging_1','discharging_2','discharging_3','failure','kill']
sheet = Image.new('RGB',(544, (64*2+24)*3),(30,30,30))
for i,name in enumerate(names):
    pic = Image.open(build / (name+'.pgm')).resize((256,128),Image.Resampling.NEAREST).convert('RGB')
    x,y=(i%2)*272,(i//2)*152
    sheet.paste(pic,(x,y+24))
    ImageDraw.Draw(sheet).text((x+4,y+4),name,fill='white')
out = root / 'MDK-ARM/output/shutdown'
out.mkdir(parents=True,exist_ok=True)
sheet.save(out/'oled_shutdown.png')
