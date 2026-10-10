"""Exercise production board drivers and control code against a fault-injecting HAL.

Author: COEX, Xuanting Liu. Run with Python 3 and host GCC; no hardware required.
"""
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[2]
CORE = ROOT / "Core"
BUILD = ROOT / "MDK-ARM/tmp/board_migration/tests"
BUILD.mkdir(parents=True, exist_ok=True)

HAL = r'''
#ifndef TEST_HAL_H
#define TEST_HAL_H
#include <stdint.h>
#include <stddef.h>
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef int HAL_StatusTypeDef;
typedef int GPIO_PinState;
typedef int I2C_HandleTypeDef;
typedef int SPI_HandleTypeDef;
typedef struct {uint32_t ARR, PSC, CNT, CR1, CCER, CCR[4];} TIM_TypeDef;
typedef struct {TIM_TypeDef *Instance;} TIM_HandleTypeDef;
extern TIM_TypeDef timer1, timer8, timer9, timer11;
extern TIM_HandleTypeDef htim1, htim8, htim9, htim11;
extern I2C_HandleTypeDef hi2c3;
extern SPI_HandleTypeDef hspi2;
#define TIM9 (&timer9)
#define HAL_OK 0
#define HAL_ERROR 1
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1
#define I2C_MEMADD_SIZE_8BIT 1
#define TIM_CHANNEL_1 0
#define TIM_CHANNEL_2 1
#define TIM_CHANNEL_3 2
#define TIM_CHANNEL_4 3
#define TIM_CCER_CC1P 2U
#define TIM_CCER_CC2P 32U
#define TIM_CR1_CEN 1U
#define TIM_EVENTSOURCE_UPDATE 1U
#define TIM_FLAG_UPDATE 1U
#define __HAL_TIM_GET_AUTORELOAD(h) ((h)->Instance->ARR)
#define __HAL_TIM_SET_COMPARE(h,c,v) ((h)->Instance->CCR[c] = (v))
#define __HAL_TIM_CLEAR_FLAG(h,f) ((void)0)
#define KICK_CS_GPIO_Port 0
#define CHG_EN_GPIO_Port 0
#define KILL_GPIO_Port 0
#define KICK_ERR_GPIO_Port 0
#define PDN_GPIO_Port 0
#define KICK_CS_Pin 0
#define CHG_EN_Pin 1
#define KILL_Pin 2
#define KICK_ERR_Pin 3
#define PDN_Pin 4
#define SMB_ALERT_Pin 5
extern int critical_depth;
extern uint32_t irq_mask;
#define __get_PRIMASK() irq_mask
#define __disable_irq() (irq_mask=1U)
#define __set_PRIMASK(value) (irq_mask=(value))
#define taskENTER_CRITICAL() (++critical_depth)
#define taskEXIT_CRITICAL() (--critical_depth)
uint32_t HAL_GetTick(void);
void HAL_GPIO_WritePin(int, uint16_t, GPIO_PinState);
GPIO_PinState HAL_GPIO_ReadPin(int, uint16_t);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef*,uint8_t*,uint8_t*,uint16_t,uint32_t);
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef*,uint16_t,uint16_t,uint16_t,uint8_t*,uint16_t,uint32_t);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef*,uint16_t,uint16_t,uint16_t,uint8_t*,uint16_t,uint32_t);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef*,uint32_t);
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef*,uint32_t);
HAL_StatusTypeDef HAL_TIM_GenerateEvent(TIM_HandleTypeDef*,uint32_t);
void init_motor(void);
void do_dribbler(int);
int shooter_off(void);
void set_dribbler(u16,u8);
void shoot_on(u32);
void chip_on(u32);
void set_buzzer(uint8_t);
#define BEEP_OFF() set_buzzer(0)
#define MOTOR_BALL_DIR_HI() ((void)0)
#define MOTOR_BALL_DIR_LOW() ((void)0)
typedef struct {float cap_v_f, bat_v_f; int dribbler, is_cap_low; uint8_t bat_v;} robot_t;
extern robot_t g_robot;
typedef struct {int strength, count_down;} shooter_t;
extern shooter_t g_shooter[2];
extern char shooter;
#define SHOOT 0
#define CHIP 1
#endif
'''

HARNESS = r'''
#include "main.h"
#include "cfg.h"
#include "board_monitor.h"
#include "dribbler_control.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

TIM_TypeDef timer1, timer8 = {.ARR=499}, timer9 = {.ARR=635}, timer11 = {.ARR=6719};
TIM_HandleTypeDef htim1={&timer1}, htim8={&timer8}, htim9={&timer9}, htim11={&timer11};
I2C_HandleTypeDef hi2c3;
SPI_HandleTypeDef hspi2;
robot_t g_robot;
shooter_t g_shooter[2];
char shooter;
int critical_depth;
uint32_t irq_mask;
static uint32_t tick;
static int pins[6] = {1,0,1,1,1,1};
static int spi_error, i2c_error, bad_adc, no_ready;
static int init_motor_calls, stops;
static float cap=170.0f, temperature=25.0f;
static int16_t shunt_raw=-800;
static uint16_t bus_raw=12800;
static int event_count;
static uint32_t enabled_channels;

uint32_t HAL_GetTick(void) {return tick;}
void HAL_GPIO_WritePin(int port,uint16_t pin,GPIO_PinState value) {(void)port;pins[pin]=value;}
GPIO_PinState HAL_GPIO_ReadPin(int port,uint16_t pin) {(void)port;return pins[pin];}
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *h,uint8_t *tx,uint8_t *rx,uint16_t n,uint32_t timeout)
{
    assert(h==&hspi2 && n==64 && timeout<=3 && !pins[KICK_CS_Pin]);
    assert(critical_depth==0);
    for(unsigned i=0;i<32;i++) {
        assert(tx[i*2]==(i<16?0:8) && tx[i*2+1]==0);
        uint16_t sample = i<16 ? (uint16_t)((800.0f+temperature*39.0f)*4096.0f/6600.0f) :
                                (uint16_t)(cap*4096.0f/KICKER_CAP_FULL_SCALE_V);
        if(i%16<2 || bad_adc) sample=0xffff; /* Pipeline/settling words must be ignored. */
        rx[i*2]=sample>>8;rx[i*2+1]=(uint8_t)sample;
    }
    return spi_error?HAL_ERROR:HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *h,uint16_t addr,uint16_t reg,uint16_t size,uint8_t *data,uint16_t n,uint32_t timeout)
{
    assert(h==&hi2c3 && addr==0x80 && size==1 && n==2 && timeout<=3);
    assert(critical_depth==0);
    if(i2c_error) return HAL_ERROR;
    uint16_t value;
    switch(reg) {
        case 0xfe:value=0x5449;break;
        case 0xff:value=0x2260;break;
        case 6:value=no_ready?0:8;break;
        case 1:value=(uint16_t)shunt_raw;break;
        case 2:value=bus_raw;break;
        default:assert(0);value=0;
    }
    data[0]=value>>8; data[1]=(uint8_t)value;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *h,uint16_t addr,uint16_t reg,uint16_t size,uint8_t *data,uint16_t n,uint32_t timeout)
{
    assert(h==&hi2c3 && addr==0x80 && size==1 && n==2 && timeout<=3);
    assert(critical_depth==0);
    uint16_t value=((uint16_t)data[0]<<8)|data[1];
    assert((reg==0 && value==0x42df)||(reg==6 && value==0x400));
    return i2c_error?HAL_ERROR:HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *h,uint32_t channel)
{
    if(h==&htim9) enabled_channels|=1U<<channel;
    h->Instance->CR1|=TIM_CR1_CEN;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *h,uint32_t channel)
{
    if(h==&htim9) enabled_channels&=~(1U<<channel);
    h->Instance->CR1&=~TIM_CR1_CEN;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_GenerateEvent(TIM_HandleTypeDef *h,uint32_t event)
{
    assert(h==&htim9 && event==TIM_EVENTSOURCE_UPDATE && enabled_channels==0);
    h->Instance->CNT=0;
    h->Instance->CR1&=~TIM_CR1_CEN;
    event_count++;
    return HAL_OK;
}
void init_motor(void) {init_motor_calls++;}
int shooter_off(void) {stops++;timer9.CR1=0;enabled_channels=0;return 0;}

/* These functions are extracted verbatim from production files below. */
@FUNCTIONS@

static void update(unsigned elapsed) {tick+=elapsed;board_monitor_update();assert(critical_depth==0);}
static board_status_t status(void) {board_status_t s;board_monitor_get(&s);return s;}
static void warmup(void) {board_monitor_init();update(10);update(10);assert(status().battery_valid);}
static void pulses(unsigned n) {while(n--) dribbler_fg_edge();}

static void power_test(void)
{
    warmup();
    board_status_t s=status();
    assert(s.kicker_valid && s.battery_valid && s.current_valid);
    assert(fabsf(s.cap_v-170)<0.1f && fabsf(s.temperature-25)<0.1f);
    assert(fabsf(s.bat_v-16)<0.001f && fabsf(s.current_a+1)<0.001f);
    assert(get_bat_v()==(uint8_t)(16.0f/BAT_V_ADC_GAIN+0.5f));
    assert(get_cap_v()==(uint8_t)(s.cap_v/CAP_V_ADC_GAIN+0.5f));
    assert(s.charge_enabled && pins[CHG_EN_Pin] && pins[KICK_CS_Pin]);
    cap=181;update(10);assert(!status().charge_enabled);
    cap=177;update(10);assert(!status().charge_enabled);
    cap=174;update(10);assert(status().charge_enabled);
    temperature=62;update(10);assert(!status().charge_enabled);
    temperature=57;update(10);assert(!status().charge_enabled);
    temperature=54;update(10);assert(status().charge_enabled);
    temperature=25;cap=175;update(10);assert(board_kicker_ready());
    tick+=101;assert(!board_kicker_ready() && !status().battery_valid);
    update(10);assert(board_kicker_ready());
    spi_error=1;update(10);assert(!status().kicker_valid && !pins[CHG_EN_Pin] && pins[KICK_CS_Pin]);
    assert(get_cap_v()==0);
    spi_error=0;bad_adc=1;update(10);assert(!status().kicker_valid);
    bad_adc=0;i2c_error=1;update(10);assert(!status().battery_valid && !board_kicker_ready());
    assert(get_bat_v()==0 && is_power_low()==2);
    i2c_error=0;update(1000);update(10);assert(status().battery_valid);
    no_ready=1;update(101);assert(!status().battery_valid);
}
static void charge_test(void)
{
    cap=100;warmup();
    for(unsigned i=0;i<301;i++) update(10);
    assert(status().charge_fault && !pins[CHG_EN_Pin]);
    update(30000);assert(!status().charge_fault && pins[CHG_EN_Pin]);
    pins[KICK_ERR_Pin]=0;update(10);assert(status().charge_fault && !pins[CHG_EN_Pin]);
    update(30000); /* DONE may remain latched while CHARGE was low. Retry must be possible. */
    assert(!status().charge_fault && pins[CHG_EN_Pin]);
    pins[KICK_ERR_Pin]=1;update(10);assert(!status().charge_fault);
    /* A kick can stop CHARGE between two monitor iterations. A latched DONE
     * must not be mistaken for a new charging fault when CHARGE is already low. */
    pins[CHG_EN_Pin]=0; pins[KICK_ERR_Pin]=0; timer9.CR1=TIM_CR1_CEN;
    update(10);assert(!status().charge_fault && !pins[CHG_EN_Pin]);
}
static void pulse_test(void)
{
    cap=180;warmup();
    timer9.CNT=413;
    shoot_on(1);
    assert(event_count==1 && enabled_channels==1 && timer9.CNT==0 && timer9.CCR[0]==631);
    assert(!pins[CHG_EN_Pin]);
    chip_on(127);assert(event_count==1); /* Never overlap the two coils. */
    timer9.CR1=0;timer9.CNT=0;
    chip_on(127);
    assert(event_count==2 && enabled_channels==2 && timer9.CCR[1]==1);
    /* Model PWM2/HIGH for all 636 counts: exactly one pulse, then idle low. */
    for(unsigned channel=0;channel<2;channel++) {
        unsigned high=0;
        for(unsigned count=0;count<=635;count++) high+=(count>=timer9.CCR[channel]);
        assert(high==(channel==0?5:635));
        assert(0<timer9.CCR[channel]);
    }
    timer9.CR1=0;
    timer9.CCER=TIM_CCER_CC1P;
    shoot_on(50);assert(event_count==2); /* Reject an unsafe regenerated polarity. */
    timer9.CCER=0;
    tick+=101;shoot_on(50);assert(event_count==2); /* Reject stale sensors. */
    update(10);shoot_on(0);assert(event_count==2);
    shoot_on(255);assert(timer9.CCR[0]==1 && event_count==3);
    timer9.CR1=0;
    assert(set_shooter(2,50)==-1);
    assert(set_shooter(0,50)==0 && g_shooter[0].count_down==SHOOTER_INTERVAL && shooter==2);
    assert(set_shooter(1,50)==-1);
    set_buzzer(1);assert(timer8.CCR[0]==250);
    set_buzzer(0);assert(timer8.CCR[0]==0);
    assert(irq_mask==0);
    irq_mask=1;set_buzzer(1);assert(irq_mask==1);
    set_motor_pwm(1,2,3,4);assert(irq_mask==1);
    irq_mask=0;set_motor_pwm(0,0,0,0);assert(irq_mask==0);
}
static void fg_test(void)
{
    dribbler_status_t s;
    dribbler_set_target(1);
    tick=100;dribbler_update();dribbler_get(&s);
    assert(s.target_rpm==5000 && s.duty>0 && s.duty<=0.05001f);
    for(unsigned i=0;i<50;i++) {
        pulses(25);tick+=100;dribbler_update();dribbler_get(&s);
        assert(fabsf(s.rpm-5000)<0.01f && !s.stalled && s.duty<=DRIBBLER_MAX_DUTY);
    }
    float duty=s.duty;
    dribbler_set_target(2);dribbler_get(&s);assert(s.target_rpm==5000 && s.duty==duty);
    for(unsigned i=0;i<10;i++) {
        pulses(10);tick+=100;dribbler_update();dribbler_get(&s);
        assert(s.duty<=DRIBBLER_MAX_DUTY);
    }
    assert(s.duty>duty); /* Added load must increase drive. */
    for(unsigned i=0;i<10;i++) {tick+=100;dribbler_update();}
    dribbler_get(&s);assert(s.stalled && s.duty==0 && timer11.CCR[0]==0);
    dribbler_set_target(3);tick+=100;dribbler_update();dribbler_get(&s);assert(s.stalled);
    dribbler_set_target(0);dribbler_get(&s);assert(!s.stalled && s.target_rpm==0);
    dribbler_set_target(-1);pulses(25);tick+=100;dribbler_update();dribbler_get(&s);
    assert(!s.stalled && s.target_rpm==5000);
    dribbler_set_target(7);assert(timer11.CCR[0]==0);
}
static void shutdown_test(void)
{
    cap=180;warmup();
    pins[PDN_Pin]=0;update(10);update(40);assert(!board_is_shutdown());
    pins[PDN_Pin]=1;update(10);assert(!board_is_shutdown());
    pins[PDN_Pin]=0;update(10);update(50);
    assert(board_is_shutdown() && !pins[KILL_Pin] && !pins[CHG_EN_Pin]);
    assert(init_motor_calls==1 && stops==1);
    set_dribbler(1000,1);assert(timer11.CCR[0]==0);
    set_motor_pwm(1000,1000,1000,1000);assert(timer1.CCR[0]==0 && timer1.CCR[3]==0);
    dribbler_set_target(1);tick+=100;dribbler_update();assert(timer11.CCR[0]==0);
    shoot_on(127);assert(event_count==0);
    set_buzzer(1);assert(timer8.CCR[0]==0);
    update(0);assert(pins[KILL_Pin]);
    update(100);assert(!pins[KILL_Pin]);
    assert(critical_depth==0);
}
int main(int argc,char **argv)
{
    assert(argc==2);
    if(!strcmp(argv[1],"power"))power_test();
    else if(!strcmp(argv[1],"charge"))charge_test();
    else if(!strcmp(argv[1],"pulse"))pulse_test();
    else if(!strcmp(argv[1],"fg"))fg_test();
    else if(!strcmp(argv[1],"shutdown"))shutdown_test();
    else assert(0);
    printf("PASS: %s\n",argv[1]);
    return 0;
}
'''

(BUILD / 'main.h').write_text(HAL)
for name in ('spi', 'i2c', 'tim', 'gpio', 'misc', 'motor', 'action', 'robot', 'FreeRTOS', 'task'):
    (BUILD / f'{name}.h').write_text('#include "main.h"\n')
(BUILD / 'nrf24l01_drv.h').write_text('')
(BUILD / 'cfg.h').write_text((CORE / 'Inc/cfg.h').read_text(encoding='utf-8'), encoding='utf-8')

functions = []
for filename, names in {
    'misc.c': ['start_kick', 'shoot_on', 'chip_on', 'get_bat_v', 'get_cap_v', 'is_power_low'],
    'action.c': ['do_dribbler', 'set_dribbler', 'set_shooter'],
    'motor.c': ['set_motor_pwm'],
    'gpio.c': ['set_buzzer'],
}.items():
    source = (CORE / 'Src' / filename).read_text(encoding='utf-8')
    for name in names:
        match = re.search(r'(?m)^[\w ]+\b' + name + r'\([^\n]*\)\s*\n\{.*?^\}', source, re.S | re.M)
        assert match, name
        functions.append(match.group(0))
(BUILD / 'test.c').write_text(HARNESS.replace('@FUNCTIONS@', '\n\n'.join(functions)), encoding='utf-8')
exe = BUILD / 'test.exe'
subprocess.run(['gcc', '-std=c99', '-Wall', '-Wextra', '-Werror', '-I', str(BUILD), '-I', str(CORE/'Inc'),
                str(BUILD/'test.c'), str(CORE/'Src/board_monitor.c'), str(CORE/'Src/dribbler_control.c'), '-lm', '-o', str(exe)], check=True)
for case in ('power', 'charge', 'pulse', 'fg', 'shutdown'):
    subprocess.run([str(exe), case], check=True)
