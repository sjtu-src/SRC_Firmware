#ifndef _ERROR_H
#define _ERROR_H
#include "stm32f4xx.h"
#include "typedef.h"

/***控制器硬件状态***/
typedef union
{
    u8 all;
    struct
    {
       u8 mpu6050_flag:1;        //mpu6050陀螺仪错误标志 1 表示出问题 0 表示ok
       u8 ble_hm13_flag:1;       //hm13蓝牙模块错误标志 1表示出问题 0 表示ok
	   u8 nrf24lc01_flag:1;      //nrf24lc01无线模块标志
	   u8 eeprom_24lc64_flag:1;  //存储器模块标志
	   u8 reserved:4;
 	
    }bit;
}ERROR_EVENT_UNION;

extern ERROR_EVENT_UNION error_flag;


#endif