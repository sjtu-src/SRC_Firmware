"""TIGERs CHG bias identification and ADC-based charging integration."""
from pathlib import Path
import subprocess
base=Path(__file__).with_name('test_board_migration.py')
ns={'__file__':str(base),'__name__':'identify_setup'}
exec(compile(base.read_text(encoding='utf-8').split('# Exercise both reference')[0],str(base),'exec'),ns)
b,core=ns['BUILD'],ns['CORE']
s=(b/'test.c').read_text(encoding='utf-8')
s=s[:s.index('int main(int argc,char **argv)')]+r'''
int main(int argc,char **argv) {
    assert(argc==2);
    cap=100;
    if(!strcmp(argv[1],"absent")) charge_bias=0;
    if(!strcmp(argv[1],"pullup")) charge_bias=2;
    pins[KICK_ERR_Pin]=0; /* Intentionally low throughout identification/start. */
    warmup();
    assert(delay_ms==20 && bias_reads>=2);
    if(charge_bias!=1) {
        assert(!status().kicker_installed && !board_kicker_installed());
        assert((status().charge_inhibit & BOARD_CHG_NO_BOARD) && !pins[CHG_EN_Pin]);
        assert(charge_mode==GPIO_MODE_INPUT && !board_kicker_ready());
        cap=180;update(10);shoot_on(127);assert(event_count==0);
        update(3000);assert(!status().charge_fault);
    } else {
        assert(status().kicker_installed && board_kicker_installed());
        assert(charge_mode==GPIO_MODE_OUTPUT_PP && charge_pull==GPIO_NOPULL);
        assert(status().charge_enabled && !status().charge_fault);
        if(!strcmp(argv[1],"done_unused")) {
            for(unsigned i=0;i<10;i++) {pins[KICK_ERR_Pin]=i&1;update(10);assert(pins[CHG_EN_Pin] && !status().charge_fault);}
        } else if(!strcmp(argv[1],"progress")) {
            update(3000);assert(status().charge_fault_reason==BOARD_CHG_PROGRESS && !pins[CHG_EN_Pin]);
            update(30000);assert(pins[CHG_EN_Pin] && !status().charge_fault);
        } else if(!strcmp(argv[1],"adc")) {
            spi_error=1;update(10);assert(!pins[CHG_EN_Pin] && (status().charge_inhibit&BOARD_CHG_ADC));
        }
        update(10);assert(delay_ms==20); /* Identify once, not on each charging retry. */
    }
    printf("PASS TIGERs charge: %s\n",argv[1]);return 0;
}
'''
(b/'charge_identify.c').write_text(s,encoding='utf-8')
for cells in (4,6):
    exe=b/f'charge_identify_{cells}s.exe'
    subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror','-Wno-unused-function',f'-DBATTERY_CELLS={cells}',
                    '-I',str(b),'-I',str(core/'Inc'),str(b/'charge_identify.c'),
                    str(core/'Src/board_monitor.c'),str(core/'Src/dribbler_control.c'),'-lm','-o',str(exe)],check=True)
    for case in ('present','absent','pullup','done_unused','progress','adc'):
        subprocess.run([str(exe),case],check=True)
board=(core/'Src/board_monitor.c').read_text(encoding='utf-8')
assert 'KICK_ERR_Pin' not in board and 'KICKER_DONE_SETTLE_MS' not in board
print('PASS TIGERs charge: no DONE reads or added DONE settle window')
