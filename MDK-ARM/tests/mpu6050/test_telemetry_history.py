"""Compare current on-wire bytes with the pre-board-migration packet builder."""
from pathlib import Path
import re
import subprocess
import sys

here=Path(__file__).resolve().parent
root=here.parents[2]
subprocess.run([sys.executable,str(here/'test_telemetry.py')],check=True)
old=subprocess.check_output(['git','show','a96fdef:Core/Src/comm.c'],cwd=root).decode('utf-8')
current=(root/'Core/Src/comm.c').read_text(encoding='utf-8')
table=r'static const uint8_t game_crc8_table\[.*?\};'
assert re.search(table,old,re.S).group()==re.search(table,current,re.S).group()
builder=re.search(r'static void build_game_telemetry_packet\(.*?^\}',old,re.S|re.M).group()
builder=builder.replace('build_game_telemetry_packet','historical_packet')
source=(here/'build/telemetry.c').read_text()
source=source.replace('build_game_telemetry_packet(', 'checked_packet(')
source=source.replace('static void checked_packet(', 'static void build_game_telemetry_packet(',1)
wrapper='''
static void checked_packet(uint8_t *out,uint8_t id) {
    uint8_t expected[PACKET_LEN_UP];
    historical_packet(expected,id);
    build_game_telemetry_packet(out,id);
    assert(memcmp(out,expected,PACKET_LEN_UP)==0);
}
'''
source=source.replace('void do_game_telemetry_up(void)',builder+'\n'+wrapper+'\nvoid do_game_telemetry_up(void)',1)
test=here/'build/telemetry_history.c'
test.write_text(source)
exe=here/'build/telemetry_history.exe'
subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror','-I',str(root/'Core/Inc'),str(test),'-o',str(exe)],check=True)
subprocess.run([str(exe),str(here/'build/attitude_history.bin')],check=True)
print('PASS: all 25 payload bytes match a96fdef for identical inputs, including all calibration diagnostic combinations')
