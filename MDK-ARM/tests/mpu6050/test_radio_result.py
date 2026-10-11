"""Exercise actual radio send/RX-switch code with simulated STATUS results."""
from pathlib import Path
import re
import subprocess

root = Path(__file__).resolve().parents[3]
source = (root/'Core/Src/NRF24L01.c').read_text(encoding='utf-8')
declarations = re.search(r'volatile struct\n\{.*?static uint8_t telemetry_tx_pending;', source, re.S).group()

def function(name):
    return re.search(r'(?:int|void) '+name+r'\([^\n]*\)\n\{.*?^\}', source, re.S | re.M).group()

stub = r'''
#include <stdint.h>
#include <assert.h>
#include <stdio.h>
#define TX_DS 5
#define MAX_RT 4
#define IRQ_ALL 0x70
#define TX_MODE 0
#define RX_MODE 1
typedef struct {struct {int pos; uint8_t buf[25];} buf;} nRF24L01;
static int receive_flag;
static uint8_t simulated_status;
static unsigned reads;
#define L01_CE_LOW() ((void)0)
#define L01_CE_HIGH() ((void)0)
#define L01_SetTRMode(mode) ((void)(mode))
#define L01_FlushTX() ((void)0)
#define L01_FlushRX() ((void)0)
#define L01_WriteTXPayload_NoAck(data,len) ((void)(data),(void)(len))
#define wait_us(us) ((void)(us))
static void clr_receive_flag(void) { receive_flag=0; }
static uint8_t L01_ReadStatusReg(void) { reads++; return simulated_status; }
static void L01_ClearIRQ(unsigned irq) { simulated_status &= ~irq; }
'''
main = r'''
int main(void) {
    nRF24L01 dev={{25,{0xaa,0x55,0x10}}};
    uint8_t outcomes[]={0x2e,0x1e,0x0e,0xff};
    for (unsigned i=0;i<4;i++) {
        assert(send_nRF24L01_packet(&dev)==25);
        simulated_status=outcomes[i];
        start_nRF24L01_RX();
        assert(g_telemetry_radio.last_status==outcomes[i]);
        assert(receive_flag==1 && telemetry_tx_pending==0);
        start_nRF24L01_RX();
        assert(reads==i+1);
    }
    assert(g_telemetry_radio.completed==1);
    assert(g_telemetry_radio.failed==2);
    assert(g_telemetry_radio.unconfirmed==1);
    dev.buf.buf[0]=0xff; dev.buf.buf[1]=2;
    send_nRF24L01_packet(&dev);
    simulated_status=0x2e;
    start_nRF24L01_RX();
    assert(reads==4 && g_telemetry_radio.completed==1);
    puts("PASS: actual send/RX transition, result before IRQ clear, completion/failure/unknown, legacy exclusion");
}
'''
build = Path(__file__).parent/'build'
build.mkdir(exist_ok=True)
cfile = build/'radio_result.c'
cfile.write_text(stub+declarations+function('send_nRF24L01_packet')+function('start_nRF24L01_RX')+main)
exe = build/'radio_result.exe'
subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror',str(cfile),'-o',str(exe)],check=True)
subprocess.run([str(exe)],check=True)
