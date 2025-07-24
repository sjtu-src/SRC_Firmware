
#ifndef __COMMM_H__
#define __COMMM_H__

#include "stm32f4xx.h"
#include "typedef.h"
#include "cfg.h"
#include "nrf24l01.h"

#define MAX_BUFFER_LEN 25
#define nRF2401_BUFFER_LEN 256
#define ROBOT_HIGH_SPEED_INCREMENT 0x80

typedef struct _comm_
{
	unsigned char buffer[MAX_BUFFER_LEN];
	unsigned char buffer_pos;

	unsigned char status;

	int packet_error; //count number of bad packet received

}rf_comm_t;

int init_comm(void);
int do_comm(void);
int do_comm_up(void);

extern rf_comm_t g_rf_comm;

#endif
