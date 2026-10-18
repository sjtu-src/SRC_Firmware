"""Exercise production motor startup against A4931 polarity and HAL states."""
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[2]
BUILD = ROOT / 'MDK-ARM/tmp/motor_startup'
BUILD.mkdir(parents=True, exist_ok=True)
source = (ROOT / 'Core/Src/motor.c').read_text(encoding='utf-8')
names = ('set_motor_dir', 'set_motor_break', 'set_motor_pwm',
         'motor_hold_startup', 'motor_prepare_pwm', 'init_motor', 'start_motor')
functions = []
for name in names:
    match = re.search(r'(?m)^void ' + name + r'\([^\n]*\)\s*\n\{.*?^\}', source, re.S | re.M)
    assert match, name
    functions.append(match.group())
tim = (ROOT / 'Core/Src/tim.c').read_text(encoding='utf-8')
robot = (ROOT / 'Core/Src/robot.c').read_text(encoding='utf-8')
freertos = (ROOT / 'Core/Src/freertos.c').read_text(encoding='utf-8')
assert 'motor_prepare_pwm();' not in tim
init = tim[tim.index('void MX_TIM1_Init(void)'):tim.index('void MX_TIM2_Init(void)')]
assert 'HAL_TIM_MspPostInit(&htim1);' not in init
hold = init.split('/* USER CODE BEGIN TIM1_Init 2 */')[1].split('/* USER CODE END TIM1_Init 2 */')[0]
post = tim.split('if(timHandle->Instance==TIM1)')[1].split('else if(timHandle->Instance==TIM8)')[0]
post = post[:post.rfind('}')].lstrip().removeprefix('{')
functions.insert(0, 'void hold_pwm_gpio(void) {\n' + hold + '\n}')
functions.insert(1, 'void HAL_TIM_MspPostInit(TIM_HandleTypeDef *timHandle) {\n GPIO_InitTypeDef GPIO_InitStruct = {0}; (void)timHandle;\n' + post + '\n}')
gpio = (ROOT / 'Core/Src/gpio.c').read_text(encoding='utf-8')
levels = re.findall(r'HAL_GPIO_WritePin\(GPIOE, (.*?), (GPIO_PIN_\w+)\);', gpio, re.S)
for number in (1,2,3,4):
    assert any('MOTOR_BRK%d_Pin' % number in pins and value == 'GPIO_PIN_SET' for pins, value in levels)
    assert not any('MOTOR_BRK%d_Pin' % number in pins and value == 'GPIO_PIN_RESET' for pins, value in levels)

assert 'init_gpio();\n\tmotor_hold_startup();' in robot
boot = freertos[freertos.index('void RobotTask(void const * argument)\n{'):]
assert boot.index('Play_Mode3Music();') < boot.index('start_motor();')

harness = r'''
#include <stdint.h>
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
typedef uint8_t u8;
typedef uint16_t u16;
typedef struct { uint32_t CCR[4], active[4]; } TIM_TypeDef;
typedef struct { TIM_TypeDef *Instance; } TIM_HandleTypeDef;
typedef struct { uint32_t Pin, Mode, Pull, Speed, Alternate; } GPIO_InitTypeDef;
static TIM_TypeDef regs;
static TIM_HandleTypeDef htim1={&regs};
static struct { struct { int pid; } wheels[4]; } g_robot;
static u8 is_motor_run;
static uint32_t primask, pins, channels, mapped, starts, updates, cleared, calls;
static int disabled, fail_at, encoder_starts, pid_resets, base_starts;
static jmp_buf fatal;
#define HAL_OK 0
#define HAL_ERROR 1
#define CHANNEL_NUM 4
#define TIM_CHANNEL_1 0
#define TIM_CHANNEL_2 1
#define TIM_CHANNEL_3 2
#define TIM_CHANNEL_4 3
#define TIM_EVENTSOURCE_UPDATE 1
#define TIM_FLAG_UPDATE 1
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1
#define GPIOE 0
#define MOTOR_DIR1_Pin (1U<<0)
#define MOTOR_DIR2_Pin (1U<<1)
#define MOTOR_DIR3_Pin (1U<<2)
#define MOTOR_DIR4_Pin (1U<<3)
#define MOTOR_BRK1_Pin (1U<<4)
#define MOTOR_BRK2_Pin (1U<<5)
#define MOTOR_BRK3_Pin (1U<<6)
#define MOTOR_BRK4_Pin (1U<<7)
#define ALL_BRAKES 0xf0U
#define MOTOR_PWM1_Pin (1U<<8)
#define MOTOR_PWM2_Pin (1U<<9)
#define MOTOR_PWM3_Pin (1U<<10)
#define MOTOR_PWM4_Pin (1U<<11)
#define ALL_PWM 0xf00U
#define GPIO_MODE_OUTPUT_PP 1
#define GPIO_MODE_AF_PP 2
#define GPIO_NOPULL 0
#define GPIO_SPEED_FREQ_LOW 0
#define GPIO_AF1_TIM1 1
#define __HAL_RCC_GPIOE_CLK_ENABLE() ((void)0)
#define __get_PRIMASK() primask
#define __disable_irq() (primask=1)
#define __set_PRIMASK(v) (primask=(v))
#define DIS_INT() __disable_irq()
#define EN_INT() __set_PRIMASK(0)
#define __HAL_TIM_SET_COMPARE(t,c,v) ((t)->Instance->CCR[c]=(v))
#define __HAL_TIM_CLEAR_FLAG(t,f) (++cleared)
static int board_actuators_disabled(void) { return disabled; }
static void Error_Handler(void) { longjmp(fatal,1); }
static void HAL_GPIO_WritePin(int port,uint32_t mask,int value) {
    (void)port;
    /* Startup must never actively brake. */
    if(mask & ALL_BRAKES) assert(value);
    if(value) pins|=mask; else pins&=~mask;
}
static void HAL_GPIO_Init(int port, GPIO_InitTypeDef *gpio) {
    (void)port;
    assert(gpio->Pin==ALL_PWM);
    if(gpio->Mode==GPIO_MODE_OUTPUT_PP) {
        assert((pins & ALL_PWM)==ALL_PWM && !channels); mapped=0;
    } else {
        assert(gpio->Mode==GPIO_MODE_AF_PP && gpio->Alternate==GPIO_AF1_TIM1);
        assert((pins & ALL_PWM)==ALL_PWM && channels==15 && updates && cleared);
        for(int i=0;i<4;i++) assert(regs.active[i]==0);
        mapped=1;
    }
}
static int HAL_TIM_GenerateEvent(TIM_HandleTypeDef *t,int event) {
    assert(t==&htim1 && event==TIM_EVENTSOURCE_UPDATE);
    assert((pins & ALL_BRAKES)==ALL_BRAKES && !mapped);
    if(++calls==(uint32_t)fail_at) return HAL_ERROR;
    memcpy(regs.active,regs.CCR,sizeof(regs.active)); ++updates;
    return HAL_OK;
}
static int HAL_TIM_PWM_Start(TIM_HandleTypeDef *t,int c) {
    assert(t==&htim1 && updates && cleared);
    assert((pins & ALL_BRAKES)==ALL_BRAKES && !mapped && !(channels & (1U<<c)));
    for(int i=0;i<4;i++) assert(regs.active[i]==0);
    if(++calls==(uint32_t)fail_at) return HAL_ERROR;
    channels|=1U<<c; ++starts; return HAL_OK;
}
static void start_encoder(void) { ++encoder_starts; }
static void pid_reinit(int *pid) { *pid=0; ++pid_resets; }
static void HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *t) {
    assert(t==&htim1 && is_motor_run && channels==15); ++base_starts;
}
@FUNCTIONS@
int main(void) {
    /* Inject stale nonzero shadow values to require the explicit zero update. */
    for(int mask=0;mask<2;mask++) {
        memset(&regs,0xff,sizeof(regs));
        pins=channels=mapped=starts=updates=cleared=calls=is_motor_run=0;
        primask=(uint32_t)mask;
        hold_pwm_gpio(); /* TIM1 init keeps pins GPIO high, with no PWM start. */
        motor_hold_startup();
        assert(primask==(uint32_t)mask && !is_motor_run && !starts && !updates);
        assert((pins & ALL_BRAKES)==ALL_BRAKES && !mapped);
        /* Coast throughout music/IMU; only hand over GPIO after zero PWM starts. */
        disabled=1; start_motor(); assert(!is_motor_run && !starts);
        disabled=0; start_motor();
        assert(primask==(uint32_t)mask && is_motor_run);
        assert(mapped && channels==15 && starts==4 && (pins & ALL_BRAKES)==ALL_BRAKES);
        for(int i=0;i<4;i++) assert(regs.CCR[i]==0 && regs.active[i]==0);
    }
    assert(encoder_starts==2 && pid_resets==8 && base_starts==2);
    puts("PASS: GPIO high and brakes released through startup; AF connected only after zero PWM established; IRQ mask preserved");
    for(fail_at=1;fail_at<=5;fail_at++) {
        pins=channels=updates=cleared=calls=is_motor_run=0; mapped=0;
        hold_pwm_gpio(); motor_hold_startup();
        if(setjmp(fatal)==0) { start_motor(); assert(0); }
        assert(!is_motor_run && !mapped && (pins & ALL_PWM)==ALL_PWM && (pins & ALL_BRAKES)==ALL_BRAKES);
    }
    puts("PASS: update/channel-start failures keep PWM pins GPIO high without braking");
}
'''
(BUILD / 'test.c').write_text(harness.replace('@FUNCTIONS@', '\n\n'.join(functions)), encoding='utf-8')
exe = BUILD / 'test.exe'
subprocess.run(['gcc', '-std=c99', '-Wall', '-Wextra', '-Werror',
                str(BUILD/'test.c'), '-o', str(exe)], check=True)
subprocess.run([str(exe)], check=True)
