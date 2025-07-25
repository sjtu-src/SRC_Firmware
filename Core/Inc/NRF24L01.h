#ifndef NRF24L01_H_
#define NRF24L01_H_

#include "main.h"

typedef struct _nRF24L01_buf
{
	int len;
	int pos;   //nrf接收或者发送数据包长度
	unsigned char buf[256];
}nRF24L01_buf;

typedef struct nRF24L01
{
	void (*init_dev)(void);
	int (*get_packet)(struct nRF24L01 *);
	int (*send_packet)(struct nRF24L01 *);
	nRF24L01_buf buf;
	unsigned int packet_error;
}nRF24L01;

extern nRF24L01 nRF24L01_dev;

void init_nrf24l01(void);
int get_nRF24L01_packet(nRF24L01 *dev);
int send_nRF24L01_packet(nRF24L01 *dev);
void nrf24l01_init(void);
void start_nRF24L01_RX(void);
void set_receive_flag(void);
void clr_receive_flag(void);


#endif /* NRF24L01_H_ */

