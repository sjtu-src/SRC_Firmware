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
#define __HAL_TIM_GET_COMPARE(h,c) ((h)->Instance->CCR[c])
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
#define SMB_SCL_GPIO_Port 0
#define SMB_SDA_GPIO_Port 0
#define SMB_SCL_Pin 6
#define SMB_SDA_Pin 7
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
uint32_t HAL_I2C_GetError(I2C_HandleTypeDef*);
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
int discharge_on(uint8_t,uint32_t);
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
static int pins[8] = {1,0,1,1,1,1,1,1};
static uint32_t mock_i2c_error=0x220;
static int write_error_reg=-1;
static int spi_error, i2c_error, bad_adc, no_ready;
static int init_motor_calls, stops;
static float cap=170.0f, temperature=25.0f;
static int16_t shunt_raw=-800;
static uint16_t bus_raw=BATTERY_CELLS*3200;
static int event_count;
static uint32_t enabled_channels;
static uint32_t pulse_end, pulse_starts[1024];
static float coil_drop[2];
static int pwm_error;

uint32_t HAL_GetTick(void) {return tick;}
uint32_t HAL_I2C_GetError(I2C_HandleTypeDef *h) {assert(h==&hi2c3);return mock_i2c_error;}
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
    return (i2c_error || reg==write_error_reg)?HAL_ERROR:HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *h,uint32_t channel)
{
    if(h==&htim9) {
        if(pwm_error) return HAL_ERROR;
        enabled_channels|=1U<<channel;
        pulse_end=tick+13;
        assert(event_count<1024);
        pulse_starts[event_count]=tick;
        cap-=coil_drop[channel];
        if(cap<1)cap=1;
    }
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

static void update(unsigned elapsed) {
    tick+=elapsed;
    if(enabled_channels && (int32_t)(tick-pulse_end)>=0) timer9.CR1=0; /* OPM rollover: output idle low. */
    board_monitor_update();assert(critical_depth==0);
}
static board_status_t status(void) {board_status_t s;board_monitor_get(&s);return s;}
static void warmup(void) {board_monitor_init();update(10);update(10);assert(status().battery_valid);}
static void pulses(unsigned n) {while(n--) dribbler_fg_edge();}

static void power_test(void)
{
    warmup();
    board_status_t s=status();
    assert(s.kicker_valid && s.battery_valid && s.current_valid);
    assert(fabsf(s.cap_v-170)<0.1f && fabsf(s.temperature-25)<0.1f);
    assert(fabsf(s.bat_v-4.0f*BATTERY_CELLS)<0.001f && fabsf(s.current_a+1)<0.001f);
    assert(get_bat_v()==(uint8_t)(4.0f*BATTERY_CELLS/BAT_V_ADC_GAIN+0.5f));
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
static void battery_diag_test(int write)
{
    if(write) write_error_reg=0; else i2c_error=1;
    board_monitor_init();update(10);
    assert(g_battery_diag.failures==1 && !g_battery_diag.battery_valid);
    assert(g_battery_diag.first_error==0x220 && g_battery_diag.first_hal_status==HAL_ERROR);
    assert(g_battery_diag.first_register==(write?0:0xfe) && g_battery_diag.first_write==write);
    assert(g_battery_diag.scl_high && g_battery_diag.sda_high);
    uint32_t first=g_battery_diag.first_failure_ms;
    mock_i2c_error=4;pins[SMB_SCL_Pin]=pins[SMB_SDA_Pin]=0;
    update(1000);
    assert(g_battery_diag.failures==2 && g_battery_diag.first_error==0x220);
    assert(g_battery_diag.first_failure_ms==first && g_battery_diag.scl_high && g_battery_diag.sda_high);
    i2c_error=0;write_error_reg=-1;update(1000);update(10);
    assert(g_battery_diag.battery_valid && g_battery_diag.ina226_ready);
    assert(g_battery_diag.manufacturer_id==0x5449 && g_battery_diag.device_id==0x2260);
    assert(g_battery_diag.bat_v>0 && g_battery_diag.failures==2);
    /* A missing capacitor board does not prevent independent battery reads. */
    spi_error=1;update(10);assert(g_battery_diag.battery_valid && !status().kicker_valid);
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
static void drib_diag_test(void)
{
    timer11.CR1=1; timer11.CCER=3;
    dribbler_set_target(1);
    assert(g_dribbler_diag.run_requests==1 && g_dribbler_diag.command==1);
    assert(g_dribbler_diag.pwm_ccr==0 && g_dribbler_diag.updates==0);
    tick=100; dribbler_update();
    assert(g_dribbler_diag.updates==1 && g_dribbler_diag.nonzero_outputs==1);
    assert(g_dribbler_diag.last_nonzero_ccr==timer11.CCR[0]);
    assert(g_dribbler_diag.last_nonzero_cr1==1 && g_dribbler_diag.last_nonzero_ccer==3);
    assert(g_dribbler_diag.last_nonzero_ms==100 && !g_dribbler_diag.disabled);
    tick=1000; dribbler_update();
    assert(g_dribbler_diag.no_fg_stops==1 && g_dribbler_diag.stalled);
    assert(g_dribbler_diag.pwm_ccr==0 && g_dribbler_diag.fg_edges==0);
    dribbler_set_target(0);
    assert(!g_dribbler_diag.stalled && !g_dribbler_diag.command);
    assert(g_dribbler_diag.no_fg_stops==1 && g_dribbler_diag.nonzero_outputs==1);
    cap=180; warmup();
    pins[PDN_Pin]=0; update(10); update(50);
    unsigned before=g_dribbler_diag.rejected_requests;
    dribbler_set_target(1);
    assert(g_dribbler_diag.rejected_requests==before+1);
    assert(g_dribbler_diag.disabled && !g_dribbler_diag.command && !timer11.CCR[0]);
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
    assert(board_is_shutdown() && pins[KILL_Pin] && !pins[CHG_EN_Pin]);
    assert(status().shutdown_state==BOARD_SHUTDOWN_DISCHARGING && event_count==1);
    assert(init_motor_calls==1 && stops==1);
    set_dribbler(1000,1);assert(timer11.CCR[0]==0);
    set_motor_pwm(1000,1000,1000,1000);assert(timer1.CCR[0]==0 && timer1.CCR[3]==0);
    dribbler_set_target(1);dribbler_update();assert(timer11.CCR[0]==0);
    shoot_on(127);assert(event_count==1);
    set_buzzer(1);assert(timer8.CCR[0]==0);
    cap=1;
    update(20);update(10);update(10);assert(pins[KILL_Pin]);
    update(460);assert(!pins[KILL_Pin] && status().shutdown_state==BOARD_SHUTDOWN_KILL);
    update(99);assert(!pins[KILL_Pin]);
    update(1);assert(pins[KILL_Pin]);
    update(100);assert(pins[KILL_Pin]); /* USB alive: a single pulse, no restart. */
    assert(critical_depth==0);
}
static void enter_shutdown(void) {
    pins[PDN_Pin]=0;update(10);update(50);
    assert(board_is_shutdown() && pins[KILL_Pin]);
}
static void discharge_test(int fallback) {
    cap=180;coil_drop[0]=fallback?0:5;coil_drop[1]=5;
    warmup();enter_shutdown();
    assert(timer9.CCR[0]==611); /* 500 us, rounded to 25 timer counts. */
    for(unsigned i=0;i<1200 && status().shutdown_state==BOARD_SHUTDOWN_DISCHARGING;i++)update(10);
    assert(status().shutdown_state==BOARD_SHUTDOWN_KILL && cap<2);
    assert(status().discharge_fault==BOARD_DISCHARGE_OK);
    assert(status().discharge_failed_coils==(fallback?1:0));
    assert(!pins[CHG_EN_Pin]);
    for(int i=2;i<=event_count;i++) assert(pulse_starts[i]-pulse_starts[i-1]>=90);
    assert(critical_depth==0);
}
static void failure_test(void) {
    cap=180;warmup();enter_shutdown();
    for(unsigned i=0;i<600 && status().shutdown_state==BOARD_SHUTDOWN_DISCHARGING;i++)update(10);
    assert(status().shutdown_state==BOARD_SHUTDOWN_FAILED && status().discharge_failed_coils==3);
    assert(status().discharge_fault==BOARD_DISCHARGE_COILS && pins[KILL_Pin] && !enabled_channels);
    int pulses=event_count;
    update(3100);assert(pins[KILL_Pin] && event_count==pulses); /* Original held button is not force-off. */
    pins[PDN_Pin]=1;update(10);pins[PDN_Pin]=0;update(10);update(2900);
    assert(pins[KILL_Pin]);
    pins[PDN_Pin]=1;update(10);pins[PDN_Pin]=0;update(10);update(2999);
    assert(pins[KILL_Pin]);update(1);assert(!pins[KILL_Pin]);
    assert(status().shutdown_state==BOARD_SHUTDOWN_KILL && event_count==pulses && cap>100);
}
static void sensor_test(void) {
    warmup();enter_shutdown();spi_error=1;int pulses=event_count;
    update(499);assert(status().shutdown_state==BOARD_SHUTDOWN_DISCHARGING && pins[KILL_Pin]);
    update(1);assert(status().shutdown_state==BOARD_SHUTDOWN_FAILED && pins[KILL_Pin]);
    assert(status().discharge_fault==BOARD_DISCHARGE_SENSOR && event_count==pulses);
}
static void output_test(void) {
    warmup();pwm_error=1;enter_shutdown();
    assert(status().shutdown_state==BOARD_SHUTDOWN_FAILED && pins[KILL_Pin] && !enabled_channels);
    assert(status().discharge_fault==BOARD_DISCHARGE_OUTPUT);
}
static void timeout_test(void) {
    cap=240;coil_drop[0]=0.05f;warmup();enter_shutdown();
    for(unsigned i=0;i<6000 && status().shutdown_state==BOARD_SHUTDOWN_DISCHARGING;i++)update(10);
    assert(status().shutdown_state==BOARD_SHUTDOWN_FAILED && pins[KILL_Pin]);
    assert(status().discharge_fault==BOARD_DISCHARGE_TIMEOUT && status().discharge_failed_coils==0);
}
static void quiet_test(void) {
    warmup();bus_raw=(uint16_t)(BATTERY_CELLS*3.2f/0.00125f);update(10);
    update(4999);assert(!board_actuators_disabled());
    update(1);assert(board_actuators_disabled() && !board_is_shutdown() && status().quiet);
    assert(init_motor_calls==1 && !pins[CHG_EN_Pin]);
    set_motor_pwm(1000,1000,1000,1000);set_dribbler(1000,1);shoot_on(127);set_buzzer(1);
    assert(timer1.CCR[0]==0 && timer11.CCR[0]==0 && timer8.CCR[0]==0 && event_count==0);
    bus_raw=BATTERY_CELLS*3200;update(10);assert(status().quiet && !pins[CHG_EN_Pin]);
    /* Continue sensing in quiet; severe undervoltage takes the same discharge route. */
    bus_raw=(uint16_t)(BATTERY_CELLS*2.9f/0.00125f);update(10);update(999);
    assert(!board_is_shutdown());update(1);assert(board_is_discharging() && pins[KILL_Pin]);
    update(10);assert(event_count==1);
}
static void missing_power_test(void) {
    i2c_error=1;spi_error=1;board_monitor_init();
    update(10);update(5000);assert(status().quiet && !board_is_shutdown() && pins[KILL_Pin]);
}
int main(int argc,char **argv)
{
    assert(argc==2);
    if(!strcmp(argv[1],"diag_read"))battery_diag_test(0);
    else if(!strcmp(argv[1],"diag_write"))battery_diag_test(1);
    else if(!strcmp(argv[1],"power"))power_test();
    else if(!strcmp(argv[1],"charge"))charge_test();
    else if(!strcmp(argv[1],"pulse"))pulse_test();
    else if(!strcmp(argv[1],"drib_diag"))drib_diag_test();
    else if(!strcmp(argv[1],"fg"))fg_test();
    else if(!strcmp(argv[1],"shutdown"))shutdown_test();
    else if(!strcmp(argv[1],"discharge"))discharge_test(0);
    else if(!strcmp(argv[1],"fallback"))discharge_test(1);
    else if(!strcmp(argv[1],"failure"))failure_test();
    else if(!strcmp(argv[1],"sensor"))sensor_test();
    else if(!strcmp(argv[1],"output"))output_test();
    else if(!strcmp(argv[1],"timeout"))timeout_test();
    else if(!strcmp(argv[1],"quiet"))quiet_test();
    else if(!strcmp(argv[1],"missing"))missing_power_test();
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
    'misc.c': ['kick_pulse', 'start_kick', 'discharge_on', 'shoot_on', 'chip_on', 'get_bat_v', 'get_cap_v', 'is_power_low'],
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
for cells in (4,6):
    exe = BUILD / f'test_{cells}s.exe'
    subprocess.run(['gcc', '-std=c99', '-Wall', '-Wextra', '-Werror', f'-DBATTERY_CELLS={cells}', '-I', str(BUILD), '-I', str(CORE/'Inc'),
                    str(BUILD/'test.c'), str(CORE/'Src/board_monitor.c'), str(CORE/'Src/dribbler_control.c'), '-lm', '-o', str(exe)], check=True)
    for case in ('diag_read', 'diag_write', 'drib_diag', 'power', 'charge', 'pulse', 'fg', 'shutdown','discharge','fallback','failure','sensor','output','timeout','quiet','missing'):
        subprocess.run([str(exe), case], check=True)
