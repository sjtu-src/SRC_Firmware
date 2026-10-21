"""Charging inhibit/cooldown regression against production board_monitor.c."""
from pathlib import Path
import subprocess

BASE = Path(__file__).with_name('test_board_migration.py')
# Reuse its production-function extraction and fault-injecting HAL, without the
# unrelated fixed-55%-dribbler assertions (production currently clamps to 15%).
ns = {'__file__': str(BASE), '__name__': 'charge_test_setup'}
exec(compile(BASE.read_text(encoding='utf-8').split('# Exercise both reference')[0], str(BASE), 'exec'), ns)
b, core = ns['BUILD'], ns['CORE']
source = (b/'test.c').read_text(encoding='utf-8')
start = source.index('int main(int argc,char **argv)')
main = r'''
int main(int argc,char **argv) {
    assert(argc==2);
    cap=100;warmup();
    if(!strcmp(argv[1],"normal")) {
        assert(status().charge_enabled && status().charge_inhibit==0);
    } else if(!strcmp(argv[1],"done")) {
        pins[KICK_ERR_Pin]=0;update(10);update(20);
        assert(!status().charge_fault && status().charge_enabled);
        assert(status().charge_inhibit==0);
    } else if(!strcmp(argv[1],"progress")) {
        for(int i=0;i<301;i++) update(10);
        assert(status().charge_inhibit & BOARD_CHG_PROGRESS);

    } else if(!strcmp(argv[1],"hot")) {
        temperature=62;update(10);assert(status().charge_inhibit & BOARD_CHG_HOT);
        temperature=57;update(10);assert(status().charge_inhibit & BOARD_CHG_HOT);
        temperature=54;update(10);assert(status().charge_enabled);
    } else if(!strcmp(argv[1],"multi")) {
        spi_error=1;i2c_error=1;update(10);
        assert((status().charge_inhibit & (BOARD_CHG_ADC|BOARD_CHG_BAT_DATA)) ==
            (BOARD_CHG_ADC|BOARD_CHG_BAT_DATA));
    } else if(!strcmp(argv[1],"full")) {
        cap=181;update(10);assert(status().charge_inhibit & BOARD_CHG_FULL);
        cap=177;update(10);assert(status().charge_inhibit & BOARD_CHG_FULL);
        cap=174;update(10);assert(status().charge_enabled);
    } else if(!strcmp(argv[1],"quiet")) {
        i2c_error=1;update(10);update(5000);i2c_error=0;
        update(1000);update(10);
        assert(status().battery_valid && (status().charge_inhibit & BOARD_CHG_QUIET));
    } else if(!strcmp(argv[1],"cooldown") || !strcmp(argv[1],"short") || !strcmp(argv[1],"wrap")) {
        /* Real kick entry point; low CAP is applied after a ready pulse. */
        cap=181;update(10);cap=180;update(10);
        if(!strcmp(argv[1],"wrap")) {tick=0xfffffff0U;update(0);update(10);}
        shoot_on(1);assert(event_count==1 && !pins[CHG_EN_Pin]);
        cap=100;
        if(!strcmp(argv[1],"cooldown")) {
            update(1);assert(status().charge_inhibit & BOARD_CHG_KICK);
        }
        update(20); /* Short pulse already finished, even if never sampled high. */
        assert(status().charge_inhibit & BOARD_CHG_COOLDOWN);
        assert(status().charge_cooldown_ms==KICKER_CHARGE_COOLDOWN_MS);
        assert(!pins[CHG_EN_Pin]);
        update(79);assert(!pins[CHG_EN_Pin] && status().charge_cooldown_ms==1);
        update(1);assert(pins[CHG_EN_Pin] && !status().charge_cooldown_ms);
        assert(!status().charge_inhibit);
    } else assert(0);
    printf("PASS inhibit: %s\n",argv[1]);return 0;
}
'''
(b/'charge_inhibit.c').write_text(source[:start]+main,encoding='utf-8')
for cells in (4,6):
    exe=b/f'charge_inhibit_{cells}s.exe'
    subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror','-Wno-unused-function',f'-DBATTERY_CELLS={cells}',
                    '-I',str(b),'-I',str(core/'Inc'),str(b/'charge_inhibit.c'),
                    str(core/'Src/board_monitor.c'),str(core/'Src/dribbler_control.c'),'-lm','-o',str(exe)],check=True)
    for case in ('normal','done','progress','hot','multi','full','quiet','cooldown','short','wrap'):
        subprocess.run([str(exe),case],check=True)
    old=b/f'charge_existing_{cells}s.exe'
    subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror','-Wno-unused-function',f'-DBATTERY_CELLS={cells}',
                    '-I',str(b),'-I',str(core/'Inc'),str(b/'test.c'),
                    str(core/'Src/board_monitor.c'),str(core/'Src/dribbler_control.c'),'-lm','-o',str(old)],check=True)
    for case in ('power','charge','pulse','shutdown','discharge','fallback','failure','sensor','output','timeout','quiet','missing'):
        subprocess.run([str(old),case],check=True)




