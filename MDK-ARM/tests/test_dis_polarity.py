"""Both DIS polarities: real retained timer setup, pulses and shutdown exits."""
from pathlib import Path
import re
import subprocess
base=Path(__file__).with_name('test_board_migration.py')
ns={'__file__':str(base),'__name__':'dis_setup'}
exec(compile(base.read_text(encoding='utf-8').split('# Exercise both reference')[0],str(base),'exec'),ns)
b,core=ns['BUILD'],ns['CORE']
s=(b/'test.c').read_text(encoding='utf-8')
tim=(core/'Src/tim.c').read_text(encoding='utf-8')
init=tim.split('void MX_TIM9_Init(void)')[1].split('/* TIM10 init function */')[0]
assert 'sConfigOC.OCMode = TIM_OCMODE_PWM2;' in init
assert 'sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;' in init
final=init.split('/* USER CODE BEGIN TIM9_Init 2 */')[1].split('/* USER CODE END TIM9_Init 2 */')[0]
body=tim.split('else if(timHandle->Instance==TIM9)')[1].split('else if(timHandle->Instance==TIM10)')[0]
setup=r'''
typedef struct {unsigned OCPolarity;} TIM_OC_InitTypeDef;
#define TIM_OCPOLARITY_HIGH 0U
#define TIM_OCPOLARITY_LOW TIM_CCER_CC1P
static unsigned preload;
static HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *h,TIM_OC_InitTypeDef *c,unsigned channel) {
    unsigned bit=channel ? TIM_CCER_CC2P : TIM_CCER_CC1P;
    h->Instance->CCER &= ~bit;
    if(c->OCPolarity==TIM_OCPOLARITY_LOW) h->Instance->CCER |= bit;
    preload |= 1U<<channel; /* Real HAL enables preload again. */
    return HAL_OK;
}
#define __HAL_TIM_DISABLE_OCxPRELOAD(h,c) (preload &= ~(1U<<(c)))
static void Error_Handler(void) {assert(0);}
static void actual_tim_init(void) {
    TIM_OC_InitTypeDef sConfigOC={TIM_OCPOLARITY_LOW};(void)sConfigOC;
    timer9.CCER=TIM_CCER_CC1P|TIM_CCER_CC2P;preload=3;
'''+final+r'''
    assert(preload==0);
    assert((timer9.CCER&(TIM_CCER_CC1P|TIM_CCER_CC2P))==EXPECTED_KICK_CCER);
}
static void actual_msp_post(void) {
    GPIO_InitTypeDef GPIO_InitStruct={0};(void)GPIO_InitStruct;if(1)
'''+body+r'''
}
static void idle(void) {
    assert(dis_mode[0]==GPIO_MODE_OUTPUT_PP && dis_mode[1]==GPIO_MODE_OUTPUT_PP);
    assert(dis_latch[0]==EXPECTED_KICK_IDLE && dis_latch[1]==EXPECTED_KICK_IDLE);
    assert(!enabled_channels && !(timer9.CR1&TIM_CR1_CEN));
}
static unsigned pwm_level(unsigned ch,unsigned cnt) {
    unsigned inverted=(timer9.CCER&(ch ? TIM_CCER_CC2P : TIM_CCER_CC1P))!=0;
    return (cnt>=timer9.CCR[ch])^inverted;
}
static void pulse_shape(unsigned ch,unsigned ticks) {
    unsigned active=0;
    for(unsigned cnt=0;cnt<=timer9.ARR;cnt++) active+=pwm_level(ch,cnt)==KICK_POLARITY;
    assert(active==ticks && pwm_level(ch,0)==EXPECTED_KICK_IDLE);
    assert(dis_mode[1-ch]==GPIO_MODE_OUTPUT_PP && dis_latch[1-ch]==EXPECTED_KICK_IDLE);
}
int main(int argc,char **argv) {
    assert(argc==2);actual_tim_init();cap=180;
    /* Legacy shutdown cases perform their own warmup; never initialize twice. */
    if(strcmp(argv[1],"discharge") && strcmp(argv[1],"fallback") &&
       strcmp(argv[1],"failure") && strcmp(argv[1],"sensor") &&
       strcmp(argv[1],"output") && strcmp(argv[1],"timeout")) warmup();
    if(!strcmp(argv[1],"msp_startup")) {
        dis_latch[0]=dis_latch[1]=KICK_POLARITY;dis_mode[0]=dis_mode[1]=0;
        actual_msp_post();idle();
    } else if(!strcmp(argv[1],"start_stop")) {
        kicker_hold_outputs();idle();assert(board_kicker_ready());
        for(unsigned device=0;device<2;device++) for(unsigned strength=1;strength<=127;strength+=126) {
            if(device) chip_on(strength);else shoot_on(strength);
            assert(enabled_channels==(1U<<device));pulse_shape(device,strength*5U);
            timer9.CR1=0;timer9.CNT=0;assert(pwm_level(device,0)==EXPECTED_KICK_IDLE);
            if(device) chip_off();else shoot_off();idle();
        }
    } else if(!strcmp(argv[1],"hal_failure")) {
        for(unsigned device=0;device<2;device++) {
            pwm_error=1;if(device)chip_on(127);else shoot_on(127);idle();
        }
    } else if(!strcmp(argv[1],"irq")) {
        irq_mask=1;shoot_off();idle();assert(irq_mask==1);
        irq_mask=0;chip_off();idle();assert(irq_mask==0);
    } else if(!strcmp(argv[1],"wrong_polarity")) {
        kicker_hold_outputs();timer9.CCER=EXPECTED_KICK_CCER^(TIM_CCER_CC1P|TIM_CCER_CC2P);
        assert(!board_kicker_ready());shoot_on(127);chip_on(127);idle();assert(event_count==0);
        enter_shutdown();assert(status().shutdown_state==BOARD_SHUTDOWN_FAILED);
        assert(status().discharge_fault==BOARD_DISCHARGE_OUTPUT);idle();
    } else if(!strcmp(argv[1],"shutdown_channels")) {
        enter_shutdown();assert(board_is_discharging() && enabled_channels==1);
        pulse_shape(0,timer9.ARR+1-timer9.CCR[0]);
        shooter_off();assert(discharge_on(1,1000));assert(enabled_channels==2);
        pulse_shape(1,50);assert(!pins[CHG_EN_Pin]);shooter_off();idle();
    } else if(!strcmp(argv[1],"discharge")) {discharge_test(0);idle();}
    else if(!strcmp(argv[1],"fallback")) {discharge_test(1);idle();}
    else if(!strcmp(argv[1],"failure")) {failure_test();idle();}
    else if(!strcmp(argv[1],"sensor")) {sensor_test();idle();}
    else if(!strcmp(argv[1],"output")) {output_test();idle();}
    else if(!strcmp(argv[1],"timeout")) {timeout_test();idle();}
    else assert(0);
    printf("PASS DIS polarity=%d: %s\n",KICK_POLARITY,argv[1]);return 0;
}
'''
(b/'dis_polarity.c').write_text(s[:s.index('int main(int argc,char **argv)')]+setup,encoding='utf-8')
production_cfg=(core/'Inc/cfg.h').read_text(encoding='utf-8')
try:
    for polarity in (0,1):
        cfg,count=re.subn(r'(?m)^#define KICK_POLARITY\s+[^\n]+',f'#define KICK_POLARITY {polarity}',production_cfg)
        assert count==1
        (b/'cfg.h').write_text(cfg,encoding='utf-8')
        for cells in (4,6):
            exe=b/f'dis_polarity_{polarity}_{cells}s.exe'
            subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror','-Wno-unused-function',f'-DBATTERY_CELLS={cells}',
                            '-I',str(b),'-I',str(core/'Inc'),str(b/'dis_polarity.c'),
                            str(core/'Src/board_monitor.c'),str(core/'Src/dribbler_control.c'),'-lm','-o',str(exe)],check=True)
            for case in ('msp_startup','start_stop','hal_failure','irq','wrong_polarity','shutdown_channels',
                         'discharge','fallback','failure','sensor','output','timeout'):
                subprocess.run([str(exe),case],check=True)
finally:
    (b/'cfg.h').write_text(production_cfg,encoding='utf-8')
guard=body.split('/* USER CODE BEGIN TIM9_MspPostInit 0 */')[1].split('/* USER CODE END TIM9_MspPostInit 0 */')[0]
assert guard.index('kicker_hold_outputs();')<guard.index('#if 0')
assert body.index('#if 0')<body.index('GPIO_MODE_AF_PP')<body.index('#endif')
assert '#endif' in body.split('/* USER CODE BEGIN TIM9_MspPostInit 1 */')[1].split('/* USER CODE END TIM9_MspPostInit 1 */')[0]
main=(core/'Src/main.c').read_text(encoding='utf-8')
assert 'kicker_hold_outputs();' in main.split('/* USER CODE BEGIN Init */')[1].split('/* USER CODE END Init */')[0]
assert main.index('kicker_hold_outputs();')<main.index('  SystemClock_Config();')<main.index('  MX_GPIO_Init();')
ioc=(core.parent/'SRC_Firmware.ioc').read_text(encoding='utf-8')
assert all(f'TIM9.OCPolarity_{ch}=TIM_OCPOLARITY_LOW' in ioc for ch in (1,2))
print('PASS DIS: actual final timer config, preload, early startup and deferred AF')
