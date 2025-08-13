#include "simulate_i2c.h"
#include "stm32f4xx.h"
#include "typedef.h"
#include "tim.h"
#include "MPU6050_driver.h"
void i2c_sta(void);
void i2c_stop(void);
u8 is_i2c_ack(void);
u8 i2c_send_byte(u8 dat);
u8 i2c_rec_byte(void);
void i2c_ack(u8 val);


/******************************************************************************
 * init_i2c: Initial I2C
 *
 * Parameter:
 * Input: 
 * Output: 
 * Returns: 
 * 
 * modification history
 * --------------------
 *    2011/12/14, Shouxian Chen create this function
 * 
 ******************************************************************************/
void init_i2c(void)
{
	i2c_stop();
}

/******************************************************************************
 * i2c_sta: Generate a I2C start signal.
 *
 * Parameter:
 * Input: 
 * Output: 
 * Returns: 
 * 
 * modification history
 * --------------------
 *    2011/12/14, Shouxian Chen create this function
 * 
 ******************************************************************************/
void i2c_sta(void) 
{ 
	SDA_H;          //1
	i2c_delay();
	SCL_H;       //1
	i2c_delay();
	SDA_L;          
	i2c_delay();
	SCL_L;      
	i2c_delay();
}

/******************************************************************************
 * i2c_stop: Generate a I2C stop signal.
 *
 * Parameter:
 * Input: 
 * Output: 
 * Returns: 
 * 
 * modification history
 * --------------------
 *    2011/12/14, Shouxian Chen create this function
 * 
 ******************************************************************************/
void i2c_stop(void) 
{  
	SDA_L; 
	i2c_delay(); 
	SCL_H; 
	i2c_delay(); 
	SDA_H; 
	i2c_delay(); 
} 

/******************************************************************************
 * is_i2c_ack: Check if a ack signal is received
 *
 * Parameter:
 * Input: 
 * Output: 
 * Returns: =0, NACK. =1 ACK
 * 
 * modification history
 * --------------------
 *    2011/12/14, Shouxian Chen create this function
 * 
 ******************************************************************************/
u8 is_i2c_ack(void) 
{ 
	u32 val;
	
	SDA_H;
	i2c_delay(); 
	SCL_H; 
	i2c_delay(); 

	SDA_RD(val);
	
	if(val)
	{
		/* there is no ack */
		i2c_stop();
		return 0; //失败
	}
	else
	{
		/* there is a ack */
		SCL_L;
		i2c_delay(); 
		return 1; //成功
	}
}

/******************************************************************************
 * i2c_send_byte: Send a byte on i2c bus
 *
 * Parameter:
 * Input: - dat: the data want to send
 * Output: 
 * Returns: =0, no ack. =1, ack.
 * 
 * modification history
 * --------------------
 *    2011/12/14, Shouxian Chen create this function
 * 
 ******************************************************************************/
u8 i2c_send_byte(u8 dat)
{ 
    u8 i;
	u8 is_ack;

	DIS_INT();
	
    for(i = 0;i < 8; i++)
    {
		if((dat << i) & 0x80) 
		{
			SDA_H;
		}
		else 
		{
			SDA_L;
		}

		i2c_delay();
		SCL_H; 
		i2c_delay(); 
		SCL_L;      
    } 
    i2c_delay();
	
    is_ack = is_i2c_ack();

	EN_INT();
	return is_ack;
} 

/******************************************************************************
 * i2c_rec_byte: Receive a byte from i2c bus
 *
 * Parameter:
 * Input: 
 * Output: 
 * Returns: received data
 * 
 * modification history
 * --------------------
 *    2011/12/14, Shouxian Chen create this function
 * 
 ******************************************************************************/
u8 i2c_rec_byte(void)
{  
    u8 i = 8; 
    u8 dat = 0; 
	u32 val;

	DIS_INT();
	
    SDA_H; 
	
    while(i--) 
    { 
		i2c_delay(); 
		SCL_L; 
		i2c_delay(); 
		SCL_H; 
		i2c_delay();
		dat = dat << 1;

		SDA_RD(val);
		if(val) dat = dat | 0x1; 
		i2c_delay();
    }
	
    SCL_L; 
    i2c_delay();

	EN_INT();
	
    return dat; 
} 

/******************************************************************************
 * i2c_ack: Set I2C master ACK or NACK signal.
 *
 * Parameter:
 * Input: - val: 0, give a ACK signal; 1, give a NACK signal.
 * Output: 
 * Returns: 
 * 
 * modification history
 * --------------------
 *    2011/12/14, Shouxian Chen create this function
 * 
 ******************************************************************************/
void i2c_ack(u8 val)
{
	if(val == 0)
	{
		SDA_L; 
	}
	else 
	{
		SDA_H;
	}
	
	i2c_delay(); 
	SCL_H;
	i2c_delay(); 
	SCL_L;
	i2c_delay(); 
}


/******************************************************************************
 * i2c_m_send_dat: i2c master send data.
 *
 * Parameter:
 * Input: 
 *	- dev_addr: i2c device address, 8 bits
 *	- buf: send data buf
 *	- cnt: send data count
 * Output: 
 * Returns: Actually have been sent data count
 * 
 * modification history
 * --------------------
 *    2011/12/14, Shouxian Chen create this function
 * 
 ******************************************************************************/
u8 i2c_m_send_dat(u8 dev_addr, u8 *buf, u8 cnt)
{
	u8 ret;
	u8 sent_cnt;
	
	/* i2c bus start */
	i2c_sta();

	/* send i2c device address first */
	ret = i2c_send_byte(dev_addr & 0xfe);
	if(!ret)
	{
		/* this i2c deivce is not ack */
		return 0; //失败
	}

	sent_cnt = 0;
	while(cnt--)
	{
		ret = i2c_send_byte(*(buf + sent_cnt));
		if(!ret)
		{
			/* this i2c deivce is not ack */
			return sent_cnt;
		}
		sent_cnt++;
	}
	
	i2c_stop();
	i2c_delay();
	
	return sent_cnt;
}


/******************************************************************************
 * i2c_m_rec_dat: i2c master read data 
 *
 * Parameter:
 * Input: 
 *	- dev_addr: i2c device address, 8 bits
 *	- buf: read data buf
 *	- cnt: read data count
 * Output: 
 * Returns: Actually have been read data acnt
 * 
 * modification history
 * --------------------
 *    2011/12/14, Shouxian Chen create this function
 * 
 ******************************************************************************/
u8 i2c_m_read_dat(u8 dev_addr, u8 *buf, u8 cnt)
{  
	u8 read_cnt;
	u8 ret;

	/* i2c bus start */
	i2c_sta();

	/* send i2c device address first */
	ret = i2c_send_byte(dev_addr | 0x01);
	if(!ret)
	{
		/* this i2c deivce is not ack */
		return 0;
	}

	read_cnt = 0;
	while(cnt > 1)
	{
		*(buf + read_cnt) = i2c_rec_byte();
		/* send ack */
		i2c_ack(0);
		i2c_delay();
		
		cnt--;
		read_cnt++;
	}

	/* read the last data */
	*(buf + read_cnt) = i2c_rec_byte();
	/* send nack */
	i2c_ack(1);
	i2c_delay();

	/* i2c bus stop */
	i2c_stop();
	i2c_delay();
	
	read_cnt++;
	
	return read_cnt;
}


/******************************************************************************
 * m_probe_i2c_dev: I2c master probe if there is a i2c dev at certain i2c dev 
 *	addr. I also can use this function to poll if last EEPROM write is finished.
 *
 * Parameter:
 * Input: 
 * Output: 
 * Returns: 0, i2c device is not ack. 1, i2c device is ack.
 * 
 * modification history
 * --------------------
 *    2011/12/14, Shouxian Chen create this function
 * 
 ******************************************************************************/
u8 m_probe_i2c_dev(u8 dev_addr)
{
	u8 ret;
	
	/* i2c bus start */
	i2c_sta();

	/* send i2c device address first */
	ret = i2c_send_byte(dev_addr & 0xfe);

	if(!ret)
	{
		/* this i2c deivce is not ack */
		return 0;
	}
	else
	{
		/* there is a ack. I must stop i2c */
		i2c_stop();
		i2c_delay();
		return 1;
	}
}


/******************************************************************************
 * low_size_eeprom_wr: low size i2c eeprom AT24C01 - AT24C16 write.
 *
 * Parameter:
 * Input: 
 * Output: 
 * Returns: 
 * 
 * modification history
 * --------------------
 *    2012/07/27, Shouxian Chen create this function
 * 
 ******************************************************************************/
void low_size_eeprom_wr(void *dat_buf, u16 addr, u32 count)
{
	u32 has_done;
	u8 j,n;
	u8 time_out;

	if(count == 0) return;
 
	DIS_INT(); /* disable global interrupt */

	n = PAGE_WR_SIZE - (addr % PAGE_WR_SIZE); //first page count
	if(n > count) n = count;

	has_done = 0;
	while(1) /* write by page mode , PAGE_WR_SIZE byte in a page */
	{
		/* 24C01 - 24C16 used */
		i2c_sta();
		i2c_send_byte( EEPROM_DEV_ADDR + ((addr >> 8) << 1) );  
		i2c_send_byte( addr % 256 );
		
		while(n--)
		{
			i2c_send_byte( *(unsigned char*)dat_buf );
			dat_buf = (u8*)dat_buf + 1;
		}

		i2c_stop();

		/* waiting for write cycle to be completed by polling the ack bit of 
			24c08 , if ack bit = 1, busy*/
		j = 0; 
		time_out = 0;
		while(time_out < 10)
		{
			wait_ms_with_dis_int(10); /* waiting for write cycle to be completed */
			time_out++;
			
			i2c_sta();
			j = i2c_send_byte( EEPROM_DEV_ADDR + ((addr >> 8) << 1) );  
			i2c_stop();

			if(j > 0) break;
		}
		
		if(time_out == 10)
		{
			/* write time out */
			EN_INT();
			return ;
		}

		has_done = has_done + n;
		addr = addr + n;
		if(has_done >= count)
		{
			/* finish */
			break;
		}

		if((count - has_done) > PAGE_WR_SIZE)
		{
			n = PAGE_WR_SIZE;
		}
		else
		{
			n = count - has_done;
		}
	}

	EN_INT(); /* enable global interrupt */
}

/******************************************************************************
 * low_size_eeprom_rd: low size i2c eeprom AT24C01 - AT24C16 read.
 *
 * Parameter:
 * Input: 
 * Output: 
 * Returns: 
 * 
 * modification history
 * --------------------
 *    2012/07/27, Shouxian Chen create this function
 * 
 ******************************************************************************/
void low_size_eeprom_rd(void *dat_buf, u16 addr, u32 count)
{
	int i = 0;
	
	DIS_INT(); /* disable global interrupt */
	
	i2c_sta();
	i2c_send_byte( EEPROM_DEV_ADDR + ((addr >> 8) << 1) );
	i2c_send_byte( addr % 256 );
	
	i2c_sta();
	i2c_send_byte( EEPROM_DEV_ADDR + 0x1 + ((addr >> 8) << 1) );

	for( i = 0; i < count - 1; i++ )
	{
		*((u8 *)dat_buf) = i2c_rec_byte();
		i2c_ack(0);
		dat_buf = (u8*)dat_buf + 1;
	}
	
	/* read last byte */
	*(u8*)dat_buf = i2c_rec_byte();
	i2c_ack(1);
	i2c_stop();
	
	EN_INT(); /* enable global interrupt */
}


/******************************************************************************
 * i2c_m_rd_byte: 
 *
 * Parameter:
 * Input: 
 * Output: 
 * Returns: 
 * 
 * modification history
 * --------------------
 *    2015/08/08, Shouxian Chen create this function
 * 
 ******************************************************************************/
u8 i2c_m_rd_byte(u8 dev_addr, u8 reg_addr, u8 *dat)
{
	u8 ret;
	
	DIS_INT(); /* disable global interrupt */
		
	i2c_sta();
	ret = i2c_send_byte( dev_addr & 0xfe );
	if(!ret)
	{
		/* this i2c deivce is not ack */
		goto err;
	}

	ret = i2c_send_byte( reg_addr );
	if(!ret)
	{
		/* this i2c deivce is not ack */
		goto err;

	}
	
	i2c_sta();
	ret = i2c_send_byte( dev_addr | 0x1 );
	if(!ret)
	{
		/* this i2c deivce is not ack */
		goto err;
	}
	
	*dat = i2c_rec_byte();
	i2c_ack(1);
	i2c_stop();
	i2c_delay();

	EN_INT(); 
	return 1;

err:
	i2c_stop();
	i2c_delay();
	EN_INT();
	return 0;
}

/******************************************************************************
 * i2c_m_wr_byte: 
 *
 * Parameter:
 * Input: 
 * Output: 
 * Returns: 
 * 
 * modification history
 * --------------------
 *    2015/08/08, Shouxian Chen create this function
 *  返回1成功0失败
 ******************************************************************************/
u8 i2c_m_wr_byte(u8 dev_addr, u8 reg_addr, u8 dat)
{
	u8 buf[2];
	u8 ret;

	buf[0] = reg_addr;
	buf[1] = dat;

	DIS_INT();
	ret = i2c_m_send_dat(dev_addr, buf, 2);
	EN_INT();

	if(ret > 0) return 1;
	else return 0;
}

/******************************************************************************
 * high_size_eeprom_wr: high size eeprom wirte, such as 24LC32 ~ 24LC64
 *
 * Parameter:
 * Input: 
 * Output: 
 * Returns: 
 * 
 * modification history
 * --------------------
 *    2015/09/15, Shouxian Chen create this function
 * 
 ******************************************************************************/
void high_size_eeprom_wr(void *dat_buf, u16 addr, u32 count)
{
	u32 has_done;
	u8 j,n;
	u8 time_out;
	u8 wr_cnt;

	if(count == 0) return;
 
	DIS_INT(); /* disable global interrupt */

	n = HIGH_SZ_PAGE_WR_SIZE - (addr % HIGH_SZ_PAGE_WR_SIZE); //first page count
	if(n > count) n = count;
	
	has_done = 0;
	wr_cnt = n;
	while(1) /* write by page mode , PAGE_WR_SIZE byte in a page */
	{
		/* 24C32 - 24C64 used */
		i2c_sta();
		i2c_send_byte( EEPROM_DEV_ADDR );  
		i2c_send_byte( addr / 256 );
		i2c_send_byte( addr % 256 );
		
		while(n--)
		{
			i2c_send_byte( *(unsigned char*)dat_buf );
			dat_buf = (u8*)dat_buf + 1;
		}

		i2c_stop();
		
		/* waiting for write cycle to be completed by polling the ack bit of 
			24c08 , if ack bit = 1, busy*/
		j = 0; 
		time_out = 0;
		while(time_out < 10)
		{
			wait_ms_with_dis_int(10); /* waiting for write cycle to be completed */
			time_out++;
			
			i2c_sta();
			j = i2c_send_byte( EEPROM_DEV_ADDR );  
			i2c_stop();

			if(j > 0) break;
		}

		if(time_out == 10)
		{
			/* write time out */
			EN_INT();
			return ;
		}

		has_done = has_done + wr_cnt;
		addr = addr + wr_cnt;
		if(has_done >= count)
		{
			/* finish */
			break;
		}

		if((count - has_done) > HIGH_SZ_PAGE_WR_SIZE)
		{
			n = HIGH_SZ_PAGE_WR_SIZE;
		}
		else
		{
			n = count - has_done;
		}
		wr_cnt = n;
	}

	EN_INT(); /* enable global interrupt */
}


/******************************************************************************
 * high_size_eeprom_rd: high size i2c eeprom AT24C32 - AT24C64 read.
 *
 * Parameter:
 * Input: 
 * Output: 
 * Returns: 
 * 
 * modification history
 * --------------------
 *    2015/09/15, Shouxian Chen create this function
 * 
 ******************************************************************************/
void high_size_eeprom_rd(void *dat_buf, u16 addr, u32 count)
{
	int i = 0;
	
	DIS_INT(); /* disable global interrupt */
	
	i2c_sta();
	i2c_send_byte( EEPROM_DEV_ADDR );
	i2c_send_byte( addr / 256 );
	i2c_send_byte( addr % 256 );
	
	i2c_sta();
	i2c_send_byte( EEPROM_DEV_ADDR + 0x1 );

	for( i = 0; i < count - 1; i++ )
	{
		*((u8 *)dat_buf) = i2c_rec_byte();
		i2c_ack(0);
		dat_buf = (u8*)dat_buf + 1;
	}
	
	/* read last byte */
	*(u8*)dat_buf = i2c_rec_byte();
	i2c_ack(1);
	i2c_stop();
	
	EN_INT(); /* enable global interrupt */
}

