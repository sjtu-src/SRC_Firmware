"""Check production MOTOR5 zero-duty startup and HAL failure ordering."""
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[2]
BUILD = ROOT / 'MDK-ARM/tmp/dribbler_startup'
BUILD.mkdir(parents=True, exist_ok=True)
tim = (ROOT / 'Core/Src/tim.c').read_text(encoding='utf-8')
misc = (ROOT / 'Core/Src/misc.c').read_text(encoding='utf-8')
boot = (ROOT / 'Core/Src/freertos.c').read_text(encoding='utf-8')
init = tim.split('void MX_TIM11_Init(void)')[1].split('/* TIM12 init function */')[0]
assert 'HAL_TIM_PWM_Start(' not in init
assert 'HAL_TIM_MspPostInit(&htim11);' not in init
hold = init.split('/* USER CODE BEGIN TIM11_Init 2 */')[1].split('/* USER CODE END TIM11_Init 2 */')[0]
post = tim.split('else if(timHandle->Instance==TIM11)')[1]
post = post.split('/* USER CODE END TIM11_MspPostInit 1 */')[0]
post = post.lstrip().removeprefix('{')
functions = ['void hold_pwm_gpio(void) {\n' + hold + '\n}',
             'void HAL_TIM_MspPostInit(TIM_HandleTypeDef *timHandle) {\n'
             'GPIO_InitTypeDef GPIO_InitStruct = {0}; (void)timHandle;\n' + post + '\n}']
for name in ('init_dribbler', 'start_dribbler'):
    match = re.search(r'(?m)^void ' + name + r'\([^\n]*\)\s*\n\{.*?^\}', misc, re.S | re.M)
    assert match, name
    functions.append(match.group())
assert boot.index('Play_Mode3Music();') < boot.index('start_motor();') < boot.index('start_dribbler();')

harness = r'''
#include <stdint.h>
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
typedef struct { uint32_t CCR[1], active; } TIM_TypeDef;
typedef struct { TIM_TypeDef *Instance; } TIM_HandleTypeDef;
typedef struct { uint32_t Pin, Mode, Pull, Speed, Alternate; } GPIO_InitTypeDef;
static TIM_TypeDef regs;
static TIM_HandleTypeDef htim11={&regs};
#define TIM11 (&regs)
#define GPIOB 0
#define GPIOE 1
#define MOTOR_PWM5_Pin (1U<<9)
#define MOTOR_PWM5_GPIO_Port GPIOB
#define MOTOR_BRK5_Pin (1U<<8)
#define MOTOR_BRK5_GPIO_Port GPIOE
#define MOTOR_DIR5_Pin (1U<<10)
#define MOTOR_DIR5_GPIO_Port GPIOE
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1
#define GPIO_MODE_OUTPUT_PP 1
#define GPIO_MODE_AF_PP 2
#define GPIO_NOPULL 0
#define GPIO_SPEED_FREQ_LOW 0
#define GPIO_AF3_TIM11 3
#define TIM_CHANNEL_1 0
#define TIM_EVENTSOURCE_UPDATE 1
#define TIM_FLAG_UPDATE 1
#define HAL_OK 0
#define HAL_ERROR 1
static uint32_t pins[2], irq_mask, mapped, started, updated, cleared, calls;
static int disabled, fail_at;
static jmp_buf fatal;
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define __get_PRIMASK() irq_mask
#define __disable_irq() (irq_mask=1)
#define __set_PRIMASK(v) (irq_mask=(v))
#define __HAL_TIM_SET_COMPARE(t,c,v) ((t)->Instance->CCR[c]=(v))
#define __HAL_TIM_CLEAR_FLAG(t,f) (++cleared)
static int board_actuators_disabled(void) {return disabled;}
static void Error_Handler(void) {longjmp(fatal,1);}
static void HAL_GPIO_WritePin(int port,uint32_t mask,int value) {
    if(port==GPIOE && (mask & MOTOR_BRK5_Pin)) assert(value);
    if(value) pins[port]|=mask; else pins[port]&=~mask;
}
static void HAL_GPIO_Init(int port,GPIO_InitTypeDef *gpio) {
    assert(port==GPIOB && gpio->Pin==MOTOR_PWM5_Pin);
    assert(pins[GPIOB] & MOTOR_PWM5_Pin);
    if(gpio->Mode==GPIO_MODE_OUTPUT_PP) {assert(!started);mapped=0;}
    else {
        assert(gpio->Mode==GPIO_MODE_AF_PP && gpio->Alternate==GPIO_AF3_TIM11);
        assert(started && updated && cleared && regs.active==0);
        mapped=1;
    }
}
static int HAL_TIM_GenerateEvent(TIM_HandleTypeDef *t,int event) {
    assert(t==&htim11 && event==TIM_EVENTSOURCE_UPDATE && irq_mask);
    assert(!mapped && !started && regs.CCR[0]==0);
    assert(pins[GPIOE] & MOTOR_BRK5_Pin);
    if(++calls==(uint32_t)fail_at) return HAL_ERROR;
    regs.active=regs.CCR[0];updated=1;return HAL_OK;
}
static int HAL_TIM_PWM_Start(TIM_HandleTypeDef *t,int channel) {
    assert(t==&htim11 && channel==0 && updated && cleared && irq_mask);
    assert(!mapped && regs.active==0);
    if(++calls==(uint32_t)fail_at) return HAL_ERROR;
    started=1;return HAL_OK;
}
@FUNCTIONS@
static void reset(void) {
    regs.CCR[0]=regs.active=1234;
    pins[0]=pins[1]=irq_mask=mapped=started=updated=cleared=calls=0;
    disabled=0;fail_at=0;
}
int main(void) {
    for(unsigned mask=0;mask<2;mask++) {
        reset();irq_mask=mask;
        hold_pwm_gpio();init_dribbler();
        assert(!started && !mapped && regs.CCR[0]==0);
        assert((pins[GPIOE]&MOTOR_BRK5_Pin) && !(pins[GPIOE]&MOTOR_DIR5_Pin));
        disabled=1;start_dribbler();
        assert(!started && !mapped && irq_mask==mask);
        disabled=0;start_dribbler();
        assert(started && mapped && irq_mask==mask && regs.active==0);
    }
    for(int failure=1;failure<=2;failure++) {
        reset();hold_pwm_gpio();init_dribbler();fail_at=failure;
        if(setjmp(fatal)==0) {start_dribbler();assert(0);}
        assert(!mapped && (pins[GPIOB]&MOTOR_PWM5_Pin));
        assert(pins[GPIOE]&MOTOR_BRK5_Pin);
    }
    puts("PASS: MOTOR5 coast/zero startup, deferred AF, interlock, PRIMASK and HAL failure paths");
}
'''
(BUILD / 'test.c').write_text(harness.replace('@FUNCTIONS@', '\n\n'.join(functions)), encoding='utf-8')
exe = BUILD / 'test.exe'
subprocess.run(['gcc', '-std=c99', '-Wall', '-Wextra', '-Werror', str(BUILD/'test.c'), '-o', str(exe)], check=True)
subprocess.run([str(exe)], check=True)
