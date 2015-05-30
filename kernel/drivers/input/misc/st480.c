/* 
 * Copyright (C) 2012 Senodia Corporation.
 *
 * Author: Tori Xu <tori.xz.xu@gmail.com,xuezhi_xu@senodia.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include <linux/earlysuspend.h>
#include "st480.h"

#ifndef ST480_BURST_MODE
#ifdef SENSOR_AUTO_TEST
#include <linux/kthread.h>
#endif
#endif

#ifdef ACCELEROMETER_CONTROLL
#define SENSOR_DATA_SIZE 9
#else
#define SENSOR_DATA_SIZE 6
#endif	

#ifdef ST480_BURST_MODE
#define BURST_MODE 0x1F
#define BURST_RATE (0x01<<2)
#endif

#define SINGLE_MEASUREMENT_MODE 0x3F
#define READ_MEASUREMENT 0x4F
#define WRITE_REGISTER 0x60
#define CALIBRATION_REG (0x02<<2)

#define SENODIA_DEBUG_MSG	0
#define SENODIA_DEBUG_FUNC	0
#define SENODIA_DEBUG_DATA	0
#define MAX_FAILURE_COUNT	3
#define SENODIA_RETRY_COUNT	10
#define SENODIA_DEFAULT_DELAY	50

#define SENODIA_SMBUS_READ_WRITE_BYTE_BLOCK
#define SENODIA_SMBUS_READ_BYTE

#if SENODIA_DEBUG_MSG
#define SENODIADBG(format, ...)	printk(KERN_INFO "SENODIA " format "\n", ## __VA_ARGS__)
#else
#define SENODIADBG(format, ...)
#endif

#if SENODIA_DEBUG_FUNC
#define SENODIAFUNC(func) printk(KERN_INFO "SENODIA " func " is called\n")
#else
#define SENODIAFUNC(func)
#endif

struct senodia_data {
	struct i2c_client *client; 
	struct input_dev *input_dev;
	struct delayed_work work;
#ifdef SENODIA_EARLY_SUSPEND
	struct early_suspend senodia_early_suspend;
#endif
};

static struct senodia_data *senodia;
#ifdef ST480_BURST_MODE
static struct work_struct burst_work;
#endif
/* Addresses to scan -- protected by sense_data_mutex */
volatile static int sense_data[SENSOR_DATA_SIZE];
static struct mutex sense_data_mutex;

static atomic_t m_flag;
static atomic_t a_flag;
static atomic_t mv_flag;

static atomic_t open_count;
static atomic_t open_flag;
static atomic_t reserve_open_flag;

volatile static short open_flag_mag = 0;

volatile static short senodiad_delay = SENODIA_DEFAULT_DELAY;

struct mag_3{
	s16  mag_x,
	mag_y,
	mag_z;
};
volatile static struct mag_3 mag;

#ifndef ACCELEROMETER_CONTROLL
struct mag_6{
        s16   acc_x,
	      acc_y,
	      acc_z,
	      mag_x,
              mag_y,
              mag_z;
};
volatile static struct mag_6 mag_acc;
static s16  acc[3];
#endif

static short acc_sensor;
static short compass_sensor;

#ifdef SENODIA_SMBUS_READ_WRITE_WORD
static int magnetic_smbus_read_word_data(struct i2c_client *client,
	unsigned char reg_addr, unsigned char *data, unsigned char length)
{
	s32 dummy;
	dummy = i2c_smbus_read_word_data(client, reg_addr);
	if (dummy < 0)
                return -EPERM;

	*data = dummy & 0x00ff;
	*(data+1) = (dummy & 0xff00) >> 8;
	
	return 0;		
}

static int magnetic_smbus_write_word_data(struct i2c_client *client,
	unsigned char reg_addr, unsigned char *data, unsigned char length)
{
	s32 dummy;
	u16 value = (*(data+1) << 8) | (*(data));
	dummy = i2c_smbus_write_word_data(client, reg_addr, value);
	if (dummy < 0)
	{
		printk("magnetic write word data error!\n");
                return -EPERM;
	}
        return 0;
}
#endif

#ifdef SENODIA_SMBUS_READ_BYTE
static int magnetic_smbus_read_byte(struct i2c_client *client,
                        unsigned char reg_addr, unsigned char *data)
{
        s32 dummy;
        dummy = i2c_smbus_read_byte_data(client, reg_addr);
        if (dummy < 0)
                return -EPERM;
        *data = dummy & 0x000000ff;

        return 0;
}
#endif

#ifdef SENODIA_SMBUS_WRITE_BYTE
static int magnetic_smbus_write_byte(struct i2c_client *client,
                        unsigned char reg_addr, unsigned char *data)
{
        s32 dummy;
        dummy = i2c_smbus_write_byte_data(client, reg_addr, *data);
        if (dummy < 0)
                return -EPERM;
        return 0;
}
#endif

#ifdef SENODIA_SMBUS_READ_WRITE_BYTE_BLOCK
static int magnetic_smbus_read_byte_block(struct i2c_client *client,
                unsigned char reg_addr, unsigned char *data, unsigned char len)
{
        s32 dummy;
        dummy = i2c_smbus_read_i2c_block_data(client, reg_addr, len, data);
        if (dummy < 0)
                return -EPERM;
        return 0;
}


static int magnetic_smbus_write_byte_block(struct i2c_client *client,
                unsigned char reg_addr, unsigned char *data, unsigned char len)
{
        s32 dummy;
        dummy = i2c_smbus_write_i2c_block_data(client, reg_addr, len, data);
        if (dummy < 0)
                return -EPERM;
        return 0;
}
#endif

#ifdef SENODIA_I2C_READ
static int magnetic_i2c_read_data(struct i2c_client *client, char *buf, int length)
{
        struct i2c_msg msgs[] = {
                {
                        .addr  =  client->addr,
                        .flags  =  0,
                        .len  =  1,
                        .buf  =  buf,
                },
                {
                        .addr  =  client->addr,
                        .flags  = I2C_M_RD,
                        .len  =  length,
                        .buf  =  buf,
                },
        };

        if(i2c_transfer(client->adapter, msgs, 2) < 0){
                pr_err("megnetic_i2c_read_data: transfer error\n");
                return EIO;
        }
        else
		return 0;
}
#endif

#ifdef SENODIA_I2C_WRITE
static int magnetic_i2c_write_data(struct i2c_client *client, char *buf, int length)
{
        struct i2c_msg msgs[] = {
                {
                        .addr = client->addr,
                        .flags = 0,
                        .len = length,
                        .buf = buf,
                },
        };

        if (i2c_transfer(client->adapter, msgs, 1) < 0) {
        #ifdef SENODIA_DEBUG      
		pr_err("megnetic_i2c_write_data: transfer error\n");
	#endif
                return -EIO;
        } else
                return 0;
}
#endif


int st480_init(struct i2c_client *client)
{
	int i;
	char buf[3];
//init register
	buf[0] = 0x00;
	buf[1] = 0x7C;
	buf[2] = 0x00;	
	i=0;
	while(magnetic_smbus_write_byte_block(client, WRITE_REGISTER, buf, 3)!=0)
	{
		i++;
		msleep(1);
		if(magnetic_smbus_write_byte_block(client, WRITE_REGISTER, buf, 3)==0)
		{
			break;
		}
		if(i>4)
		{
			return -EIO;
		}
	}

	buf[0] = 0x00;
        buf[1] = 0x00;
        buf[2] = 0x08;

	i=0;
	while(magnetic_smbus_write_byte_block(client, WRITE_REGISTER, buf, 3)!=0)
        {
                i++;
		msleep(1);
                if(magnetic_smbus_write_byte_block(client, WRITE_REGISTER, buf, 3)==0)
                {
                        break;
                }
                if(i>4)
		{
			return -EIO;
		}
        }

//set calibration register
	buf[0] = 0x00;
        buf[1] = 0x1c; //0x18; //0x1A; /*freq = 100hz*/ //0x1F /*freq = 5hz*/
        buf[2] = CALIBRATION_REG;
        i=0;
        while(magnetic_smbus_write_byte_block(client, WRITE_REGISTER, buf, 3)!=0)
        {
                i++;
                msleep(1);
                if(magnetic_smbus_write_byte_block(client, WRITE_REGISTER, buf, 3)==0)
                {
                        break;
                }
                if(i>4)
		{
			return -EIO;
		}
        }
	
#ifdef ST480_BURST_MODE
	buf[0] = 0x00;
        buf[1] = 0x01; 
        buf[2] = BURST_RATE;
        i=0;
        while(magnetic_smbus_write_byte_block(client, WRITE_REGISTER, buf, 3)!=0)
        {
                i++;
                msleep(1);
                if(magnetic_smbus_write_byte_block(client, WRITE_REGISTER, buf, 3)==0)
                {
                        break;
                }
                if(i>4)
		{
			return -EIO;
		}
        }
#endif

	return 0;
}

#ifdef ST480_BURST_MODE
static void senodia_work_func(struct work_struct *work)
#else
static void senodia_work_func(void)
#endif
{
	char buffer[9];
	int ret;

#ifndef ST480_BURST_MODE
	char buf[1];
	char data[1];
	buf[0] = SINGLE_MEASUREMENT_MODE;
	memset(data, 0, 1);	
	ret=0;

	while((magnetic_smbus_read_byte(senodia->client, buf[0], data)!=0))
        {
                ret++;
                if(magnetic_smbus_read_byte(senodia->client, buf[0], data)==0)
			break;
                if(ret>3)
                {
                        break;
                }
        }
	
	if((!(data[0]>>5) & 0X01))
	{
		printk("SM_MODE flag fail!\n");
	}

#endif

	memset(buffer, 0, 9);
	ret=0;
	while((magnetic_smbus_read_byte_block(senodia->client, READ_MEASUREMENT, buffer, 9)!=0))
	{
		ret++;

		if(magnetic_smbus_read_byte_block(senodia->client, READ_MEASUREMENT, buffer, 9)==0)
		{
			break;
		}
		if(ret>=3)
		{
			break;
		}
	}

	if(!((buffer[0]>>4) & 0X01))
	{
		if(SENSOR_SIZE == 30)
		{
		#if defined (CONFIG_ST480_BOARD_LOCATION_FRONT)
			#if defined (CONFIG_ST480_BOARD_LOCATION_FRONT_DEGREE_0)
				mag.mag_x = (-1)*((buffer[3]<<8)|buffer[4]);
                		mag.mag_y = (-1)*((buffer[5]<<8)|buffer[6]);
                		mag.mag_z = (buffer[7]<<8)|buffer[8];
			#elif defined (CONFIG_ST480_BOARD_LOCATION_FRONT_DEGREE_90)
				mag.mag_x = (buffer[5]<<8)|buffer[6];
               			mag.mag_y = (-1)((buffer[3]<<8)|buffer[4]);
                		mag.mag_z = (buffer[7]<<8)|buffer[8];
			#elif defined (CONFIG_ST480_BOARD_LOCATION_FRONT_DEGREE_180)
				mag.mag_x = (buffer[3]<<8)|buffer[4];
                		mag.mag_y = (buffer[5]<<8)|buffer[6];
                		mag.mag_z = (buffer[7]<<8)|buffer[8];
			#elif defined (CONFIG_ST480_BOARD_LOCATION_FRONT_DEGREE_270)
				mag.mag_x = (-1)*((buffer[5]<<8)|buffer[6]);
                		mag.mag_y = (buffer[3]<<8)|buffer[4];
                		mag.mag_z = (buffer[7]<<8)|buffer[8];
			#endif
		#elif defined (CONFIG_ST480_BOARD_LOCATION_BACK)
			#if defined (CONFIG_ST480_BOARD_LOCATION_BACK_DEGREE_0)
				mag.mag_x = (buffer[3]<<8)|buffer[4];
                		mag.mag_y = (-1)*((buffer[5]<<8)|buffer[6]);
                		mag.mag_z = (-1)*((buffer[7]<<8)|buffer[8]);
        		#elif defined (CONFIG_ST480_BOARD_LOCATION_BACK_DEGREE_90)
				mag.mag_x = (buffer[5]<<8)|buffer[6];
                		mag.mag_y = (buffer[3]<<8)|buffer[4];
                		mag.mag_z = (-1)*((buffer[7]<<8)|buffer[8]);
        		#elif defined (CONFIG_ST480_BOARD_LOCATION_BACK_DEGREE_180)
				mag.mag_x = (-1)*((buffer[3]<<8)|buffer[4]);
                		mag.mag_y = (buffer[5]<<8)|buffer[6];
                		mag.mag_z = (-1)*((buffer[7]<<8)|buffer[8]);
        		#elif defined (CONFIG_ST480_BOARD_LOCATION_BACK_DEGREE_270)
				mag.mag_x = (-1)*((buffer[5]<<8)|buffer[6]);
                		mag.mag_y = (-1)*((buffer[3]<<8)|buffer[4]);
                		mag.mag_z = (-1)*((buffer[7]<<8)|buffer[8]);
        		#endif
		#endif	
		}
/*
//old
		else if (SENSOR_SIZE == 20)
		{
                #if defined (CONFIG_ST480_BOARD_LOCATION_FRONT)
                        #if defined (CONFIG_ST480_BOARD_LOCATION_FRONT_DEGREE_0)
                                mag.mag_x = (-1)*((buffer[5]<<8)|buffer[6]);
                                mag.mag_y = (buffer[3]<<8)|buffer[4];
                                mag.mag_z = (buffer[7]<<8)|buffer[8];
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_FRONT_DEGREE_90)
                                mag.mag_x = (-1)*((buffer[3]<<8)|buffer[4]);
                                mag.mag_y = (-1)*((buffer[5]<<8)|buffer[6]);
                                mag.mag_z = (buffer[7]<<8)|buffer[8];
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_FRONT_DEGREE_180)
                                mag.mag_x = (buffer[5]<<8)|buffer[6];
                                mag.mag_y = (-1)*((buffer[3]<<8)|buffer[4]);
                                mag.mag_z = (buffer[7]<<8)|buffer[8];
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_FRONT_DEGREE_270)
                                mag.mag_x = (buffer[3]<<8)|buffer[4];
                                mag.mag_y = (buffer[5]<<8)|buffer[6];
                                mag.mag_z = (buffer[7]<<8)|buffer[8];
                        #endif
                #elif defined (CONFIG_ST480_BOARD_LOCATION_BACK)
                        #if defined (CONFIG_ST480_BOARD_LOCATION_BACK_DEGREE_0)
				mag.mag_x = (-1)*((buffer[5]<<8)|buffer[6]);
                                mag.mag_y = (-1)*((buffer[3]<<8)|buffer[4]);
                                mag.mag_z = (-1)*((buffer[7]<<8)|buffer[8]);
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_BACK_DEGREE_90)
				mag.mag_x = (buffer[3]<<8)|buffer[4];
                                mag.mag_y = (-1)*((buffer[5]<<8)|buffer[6]);
                                mag.mag_z = (-1)*((buffer[7]<<8)|buffer[8]);
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_BACK_DEGREE_180)
				mag.mag_x = (buffer[5]<<8)|buffer[6];
                                mag.mag_y = (buffer[3]<<8)|buffer[4];
                                mag.mag_z = (-1)*((buffer[7]<<8)|buffer[8]);
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_BACK_DEGREE_270)
				mag.mag_x = (-1)*((buffer[3]<<8)|buffer[4]);
                                mag.mag_y = (buffer[5]<<8)|buffer[6];
                                mag.mag_z = (-1)*((buffer[7]<<8)|buffer[8]);
                        #endif
                #endif
		}
*/
//2m1 2m2
         else if (SENSOR_SIZE == 20)
         {
                #if defined (CONFIG_ST480_BOARD_LOCATION_FRONT)
                        #if defined (CONFIG_ST480_BOARD_LOCATION_FRONT_DEGREE_0)
                                mag.mag_x = (buffer[5]<<8)|buffer[6];
                                mag.mag_y = (-1)*((buffer[3]<<8)|buffer[4]);
                                mag.mag_z = (buffer[7]<<8)|buffer[8];
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_FRONT_DEGREE_90)
                                mag.mag_x = (-1)*((buffer[3]<<8)|buffer[4]);
                                mag.mag_y = (-1)*((buffer[5]<<8)|buffer[6]);
                                mag.mag_z = (buffer[7]<<8)|buffer[8];
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_FRONT_DEGREE_180)
                                mag.mag_x = (-1)*((buffer[5]<<8)|buffer[6]);
                                mag.mag_y = (buffer[3]<<8)|buffer[4];
                                mag.mag_z = (buffer[7]<<8)|buffer[8];
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_FRONT_DEGREE_270)
                                mag.mag_x = (buffer[3]<<8)|buffer[4];
                                mag.mag_y = (buffer[5]<<8)|buffer[6];
                                mag.mag_z = (buffer[7]<<8)|buffer[8];
                        #endif
                #elif defined (CONFIG_ST480_BOARD_LOCATION_BACK)
                        #if defined (CONFIG_ST480_BOARD_LOCATION_BACK_DEGREE_0)
                                mag.mag_x = (-1)*((buffer[5]<<8)|buffer[6]);
                                mag.mag_y = (-1)*((buffer[3]<<8)|buffer[4]);
                                mag.mag_z = (-1)*((buffer[7]<<8)|buffer[8]);
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_BACK_DEGREE_90)
                                mag.mag_x = (buffer[3]<<8)|buffer[4];
                                mag.mag_y = (-1)*((buffer[5]<<8)|buffer[6]);
                                mag.mag_z = (-1)*((buffer[7]<<8)|buffer[8]);
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_BACK_DEGREE_180)
                                mag.mag_x = (buffer[5]<<8)|buffer[6];
                                mag.mag_y = (buffer[3]<<8)|buffer[4];
                                mag.mag_z = (-1)*((buffer[7]<<8)|buffer[8]);
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_BACK_DEGREE_270)
                                mag.mag_x = (-1)*((buffer[3]<<8)|buffer[4]);
                                mag.mag_y = (buffer[5]<<8)|buffer[6];
                                mag.mag_z = (-1)*((buffer[7]<<8)|buffer[8]);
                        #endif
                #endif
         }
/*
//2x1
         else if (SENSOR_SIZE == 20)
         {
                #if defined (CONFIG_ST480_BOARD_LOCATION_FRONT)
                        #if defined (CONFIG_ST480_BOARD_LOCATION_FRONT_DEGREE_0)
                                mag.mag_x = (buffer[3]<<8)|buffer[4];
                                mag.mag_y = (buffer[5]<<8)|buffer[6];
                                mag.mag_z = (buffer[7]<<8)|buffer[8];
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_FRONT_DEGREE_90)
                                mag.mag_x = (buffer[5]<<8)|buffer[6];
                                mag.mag_y = (-1)*((buffer[3]<<8)|buffer[4]);
                                mag.mag_z = (buffer[7]<<8)|buffer[8];
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_FRONT_DEGREE_180)
                                mag.mag_x = (-1)*((buffer[3]<<8)|buffer[4]);
                                mag.mag_y = (-1)*((buffer[5]<<8)|buffer[6]);
                                mag.mag_z = (buffer[7]<<8)|buffer[8];
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_FRONT_DEGREE_270)
                                mag.mag_x = (-1)*((buffer[5]<<8)|buffer[6]);
                                mag.mag_y = (buffer[3]<<8)|buffer[4];
                                mag.mag_z = (buffer[7]<<8)|buffer[8];
                        #endif
                #elif defined (CONFIG_ST480_BOARD_LOCATION_BACK)
                        #if defined (CONFIG_ST480_BOARD_LOCATION_BACK_DEGREE_0)
                                mag.mag_x = (-1)*((buffer[3]<<8)|buffer[4]);
                                mag.mag_y = (buffer[5]<<8)|buffer[6];
                                mag.mag_z = (-1)*((buffer[7]<<8)|buffer[8]);
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_BACK_DEGREE_90)
                                     mag.mag_x = (-1)*((buffer[5]<<8)|buffer[6]);
                                mag.mag_y = (-1)*((buffer[3]<<8)|buffer[4]);
                                mag.mag_z = (-1)*((buffer[7]<<8)|buffer[8]);
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_BACK_DEGREE_180)
                                     mag.mag_x = (buffer[3]<<8)|buffer[4];
                                mag.mag_y = (-1)*((buffer[5]<<8)|buffer[6]);
                                mag.mag_z = (-1)*((buffer[7]<<8)|buffer[8]);
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_BACK_DEGREE_270)
                                     mag.mag_x = (buffer[5]<<8)|buffer[6];
                                mag.mag_y = (buffer[3]<<8)|buffer[4];
                                mag.mag_z = (-1)*((buffer[7]<<8)|buffer[8]);
                        #endif
                #endif
         }
*/
		else if (SENSOR_SIZE == 16)
                {
                #if defined (CONFIG_ST480_BOARD_LOCATION_FRONT)
                        #if defined (CONFIG_ST480_BOARD_LOCATION_FRONT_DEGREE_0)
                                mag.mag_x = (buffer[5]<<8)|buffer[6];
                                mag.mag_y = (buffer[3]<<8)|buffer[4];
                                mag.mag_z = (-1)*((buffer[7]<<8)|buffer[8]);
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_FRONT_DEGREE_90)
                                mag.mag_x = (buffer[3]<<8)|buffer[4];
                                mag.mag_y = (-1)*((buffer[5]<<8)|buffer[6]);
                                mag.mag_z = (-1)*((buffer[7]<<8)|buffer[8]);
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_FRONT_DEGREE_180)
                                mag.mag_x = (-1)*((buffer[5]<<8)|buffer[6]);
                                mag.mag_y = (-1)*((buffer[3]<<8)|buffer[4]);
                                mag.mag_z = (-1)*((buffer[7]<<8)|buffer[8]);
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_FRONT_DEGREE_270)
                                mag.mag_x = (-1)*((buffer[3]<<8)|buffer[4]);
                                mag.mag_y = (buffer[5]<<8)|buffer[6];
                                mag.mag_z = (-1)*((buffer[7]<<8)|buffer[8]);
                        #endif
                #elif defined (CONFIG_ST480_BOARD_LOCATION_BACK)
                        #if defined (CONFIG_ST480_BOARD_LOCATION_BACK_DEGREE_0)
                                mag.mag_x = (buffer[5]<<8)|buffer[6];
                                mag.mag_y = (-1)*((buffer[3]<<8)|buffer[4]);
                                mag.mag_z = (buffer[7]<<8)|buffer[8];
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_BACK_DEGREE_90)
                                mag.mag_x = (-1)*((buffer[3]<<8)|buffer[4]);
                                mag.mag_y = (-1)*((buffer[5]<<8)|buffer[6]);
                                mag.mag_z = (buffer[7]<<8)|buffer[8];
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_BACK_DEGREE_180)
                                mag.mag_x = (-1)*((buffer[5]<<8)|buffer[6]);
                                mag.mag_y = (buffer[3]<<8)|buffer[4];
                                mag.mag_z = (buffer[7]<<8)|buffer[8];
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_BACK_DEGREE_270)
                                mag.mag_x = (buffer[3]<<8)|buffer[4];
                                mag.mag_y = (buffer[5]<<8)|buffer[6];
                                mag.mag_z = (buffer[7]<<8)|buffer[8];
                        #endif
                #endif
                }	
		if( ((buffer[1]<<8)|(buffer[2])) > 46244)
		{
			mag.mag_x = mag.mag_x * (1 + (70/128/4096) * (((buffer[1]<<8)|(buffer[2])) - 46244));
			mag.mag_y = mag.mag_y * (1 + (70/128/4096) * (((buffer[1]<<8)|(buffer[2])) - 46244));
			mag.mag_z = mag.mag_z * (1 + (70/128/4096) * (((buffer[1]<<8)|(buffer[2])) - 46244));
		} 
		else if( ((buffer[1]<<8)|(buffer[2])) < 46244)
		{
			mag.mag_x = mag.mag_x * (1 + (60/128/4096) * (((buffer[1]<<8)|(buffer[2])) - 46244));
			mag.mag_y = mag.mag_y * (1 + (60/128/4096) * (((buffer[1]<<8)|(buffer[2])) - 46244));
			mag.mag_z = mag.mag_z * (1 + (60/128/4096) * (((buffer[1]<<8)|(buffer[2])) - 46244));
		}

#ifdef SENODIA_DEBUG
	printk("mag_x = %d, mag_y = %d, mag_z = %d\n",mag.mag_x,mag.mag_y,mag.mag_z);
#endif	
	}

#ifndef ACCELEROMETER_CONTROLL
	mag_acc.acc_x = acc[0];
	mag_acc.acc_y = acc[1];
	mag_acc.acc_z = acc[2];
	mag_acc.mag_x = mag.mag_x;
	mag_acc.mag_y = mag.mag_y;
	mag_acc.mag_z = mag.mag_z;
#endif

#ifdef ST480_BURST_MODE
	enable_irq(senodia->client->irq);
#endif
}

static void sensor_input_func(struct work_struct *work)
{
	/* Report magnetic sensor information */
	if (atomic_read(&mv_flag)) {
#ifdef SENODIA_DEBUG
	#ifdef ACCELEROMETER_CONTROLL
		printk(KERN_INFO "  yaw =%6d, pitch =%6d, roll =%6d\n",
                   sense_data[6], sense_data[7], sense_data[8]);
	#else
		printk(KERN_INFO "  yaw =%9d, pitch =%9d, roll =%9d\n",
                   sense_data[3], sense_data[4], sense_data[5]);
	#endif
#endif
#ifdef ACCELEROMETER_CONTROLL
		input_report_abs(senodia->input_dev, ABS_RX, sense_data[6]);
		input_report_abs(senodia->input_dev, ABS_RY, sense_data[7]);
		input_report_abs(senodia->input_dev, ABS_RZ, sense_data[8]);
#else
		input_report_abs(senodia->input_dev, ABS_RX, sense_data[3]);
		input_report_abs(senodia->input_dev, ABS_RY, sense_data[4]);
		input_report_abs(senodia->input_dev, ABS_RZ, sense_data[5]);
#endif	
	}
	
	/* Report acceleration sensor information */
	if (atomic_read(&a_flag)) {
#ifdef ACCELEROMETER_CONTROLL
#ifdef SENODIA_DEBUG
		printk(KERN_INFO "  Acceleration[LSB]: %6d,%6d,%6d\n",
               sense_data[3], sense_data[4], sense_data[5]);
#endif
		input_report_abs(senodia->input_dev, ABS_X, sense_data[3]);
		input_report_abs(senodia->input_dev, ABS_Y, sense_data[4]);
		input_report_abs(senodia->input_dev, ABS_Z, sense_data[5]);
#endif
	}
	
	/* Report magnetic vector information */
	if (atomic_read(&m_flag)) {
#ifdef SENODIA_DEBUG
		printk(KERN_INFO "  Magnetic: %6d,%6d,%6d\n",
               sense_data[0], sense_data[1], sense_data[2]);
#endif
		input_report_abs(senodia->input_dev, ABS_HAT0X, sense_data[0]);
		input_report_abs(senodia->input_dev, ABS_HAT0Y, sense_data[1]);
		input_report_abs(senodia->input_dev, ABS_BRAKE, sense_data[2]);
	}
	
	input_sync(senodia->input_dev);

	schedule_delayed_work(&senodia->work,msecs_to_jiffies(senodiad_delay));
}

static void ecs_closedone(void)
{
	SENODIADBG("enter %s\n", __func__);
	atomic_set(&m_flag, 0);
	atomic_set(&a_flag, 0);
	atomic_set(&mv_flag, 0);
}

/***** senodia_aot functions ***************************************/
static int senodia_aot_open(struct inode *inode, struct file *file)
{
#ifdef ST480_BURST_MODE
	int flag;
#endif
	int ret = -1;
	
#ifdef ST480_BURST_MODE
	char buf[1];
	char data[1];
#endif

	SENODIAFUNC("senodia_aot_open");

	open_flag_mag++;

	if (atomic_cmpxchg(&open_count, 0, 1) == 0) {
		if (atomic_cmpxchg(&open_flag, 0, 1) == 0) {
			atomic_set(&reserve_open_flag, 1);
		#ifdef ST480_BURST_MODE
			buf[0] = BURST_MODE;
			memset(data, 0, 1);	
			flag = 0;
			
			while((magnetic_smbus_read_byte(senodia->client, buf[0], data)!=0))
        		{
                		flag++;
                		if(magnetic_smbus_read_byte(senodia->client, buf[0], data)==0)
					break;
                		if(flag>3)
                		{
                        		break;
                		}
        		}

			if((!(data[0]>>7) & 0X01))
			{
				printk("BURST_MODE flag fail!\n");
			}
			else
			{
				printk("Burst mode set success!\n");
			}
				
		#endif
			ret = 0;
		}
	}
	schedule_delayed_work(&senodia->work,msecs_to_jiffies(senodiad_delay));
	return ret;
}

static int senodia_aot_release(struct inode *inode, struct file *file)
{
#ifdef ST480_BURST_MODE
	int ret;
#endif
	SENODIAFUNC("senodia_aot_release");

	open_flag_mag--;	

#ifdef ST480_BURST_MODE
	if (0 == open_flag_mag)
	{
		char buf[1];
		char data[1];
		memset(data, 0, 1);
		buf[0] = SINGLE_MEASUREMENT_MODE;	
		ret=0;
		while((magnetic_smbus_read_byte(senodia->client, buf[0], data)!=0))
		{
			ret++;
			if(magnetic_smbus_read_byte(senodia->client, buf[0], data)==0)
				break;
			if(ret>3)
			{
				break;
			}
		}
        	
		if((!(data[0]>>5) & 0X01))
        	{
                	printk("SM_MODE flag fail!\n");
        	}	
	}
#endif
	atomic_set(&reserve_open_flag, 0);
	atomic_set(&open_flag, 0);
	atomic_set(&open_count, 0);

	cancel_delayed_work(&senodia->work);
	return 0;
}

#ifdef OLD_KERNEL_VERSION
static int
senodia_aot_ioctl(struct inode *inode, struct file *file,
			  unsigned int cmd, unsigned long arg)
#else
static long
senodia_aot_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
#endif
{
	void __user *argp = (void __user *)arg;
	short flag;

#ifndef ACCELEROMETER_CONTROLL
	int i;
	s16 acc_data[3];
#endif

	SENODIADBG("enter %s\n", __func__);
	
	switch (cmd) {
		case ECS_IOCTL_APP_SET_MFLAG:
		case ECS_IOCTL_APP_SET_AFLAG:
		case ECS_IOCTL_APP_SET_MVFLAG:
			if (copy_from_user(&flag, argp, sizeof(flag))) {
				return -EFAULT;
			}
			if (flag < 0 || flag > 1) {
				return -EINVAL;
			}
			break;
		case ECS_IOCTL_APP_SET_DELAY:
			if (copy_from_user(&flag, argp, sizeof(flag))) {
				return -EFAULT;
			}
			break;
#ifndef ACCELEROMETER_CONTROLL
		case ECS_IOCTL_WRITE_ACC_DATA:   //for hal
			if(copy_from_user(&acc_data, argp, sizeof(acc_data)))
                        {
                                printk("copy from user error.\n");
                                return -EPERM;
			}

			break;
#endif
		default:
			break;
	}
	
	switch (cmd) {
		case ECS_IOCTL_APP_SET_MFLAG:
			atomic_set(&m_flag, flag);
			SENODIADBG("MFLAG is set to %d", flag);
			break;
		case ECS_IOCTL_APP_GET_MFLAG:
			flag = atomic_read(&m_flag);
		#ifdef SENODIA_DEBUG
			printk("Mflag = %d\n",flag);	
		#endif
			break;
		case ECS_IOCTL_APP_SET_AFLAG:
			atomic_set(&a_flag, flag);
			SENODIADBG("AFLAG is set to %d", flag);
			break;
		case ECS_IOCTL_APP_GET_AFLAG:
			flag = atomic_read(&a_flag);
		#ifdef SENODIA_DEBUG
			printk("Aflag = %d\n",flag);
		#endif
			break;
		case ECS_IOCTL_APP_SET_MVFLAG:
			atomic_set(&mv_flag, flag);
			SENODIADBG("MVFLAG is set to %d", flag);
			break;
		case ECS_IOCTL_APP_GET_MVFLAG:
			flag = atomic_read(&mv_flag);
		#ifdef SENODIA_DEBUG
			printk("MVflag = %d\n",flag);		
		#endif
			break;
		case ECS_IOCTL_APP_SET_DELAY:
			senodiad_delay = flag;
			SENODIADBG("Delay is set to %d", flag);
			break;
		case ECS_IOCTL_APP_GET_DELAY:
			flag = senodiad_delay;
			break;
#ifndef ACCELEROMETER_CONTROLL
		case ECS_IOCTL_WRITE_ACC_DATA:
			mutex_lock(&sense_data_mutex);
			for (i=0; i<3; i++)
			{
				acc[i] = acc_data[i];
			}
			mutex_unlock(&sense_data_mutex);
			break;
#endif
		default:
			return -ENOTTY;
	}
	
	switch (cmd) {
		case ECS_IOCTL_APP_GET_MFLAG:
		case ECS_IOCTL_APP_GET_AFLAG:
		case ECS_IOCTL_APP_GET_MVFLAG:
		case ECS_IOCTL_APP_GET_DELAY:
			if (copy_to_user(argp, &flag, sizeof(flag))) {
				return -EFAULT;
			}
			break;
		default:
			break;
	}
	
	return 0;
}

/***** senodiad functions ********************************************/
static int senodiad_open(struct inode *inode, struct file *file)
{
	SENODIAFUNC("senodiad_open");
	return nonseekable_open(inode, file);
}

static int senodiad_release(struct inode *inode, struct file *file)
{
	SENODIAFUNC("senodiad_release");
	ecs_closedone();
	return 0;
}

#ifdef OLD_KERNEL_VERSION
static int
senodiad_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
		   unsigned long arg)
#else
static long
senodiad_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
#endif
{
	void __user *argp = (void __user *)arg;
	int senser_data[SENSOR_DATA_SIZE];
	int i;

	memset(senser_data, 0, SENSOR_DATA_SIZE);
	
	switch (cmd) {
		case IOCTL_SENSOR_GET_DATA_MAG:
			
#ifndef ST480_BURST_MODE
			senodia_work_func();
#endif
#ifndef ACCELEROMETER_CONTROLL
                        if(copy_to_user(argp, (void *)&mag_acc,sizeof(mag_acc))!=0)
                        {
                                printk("copy to user error.\n");
                                return -EPERM;
                        }
#else
			if(copy_to_user(argp, (void *)&mag,sizeof(mag))!=0)
                        {
                                printk("copy to user error.\n");
                                return -EPERM;
                        }
#endif
                        break;
	
		case IOCTL_SENSOR_WRITE_DATA_COMPASS:
#ifdef SENODIA_DEBUG
			printk("sensor write compass data!\n");
#endif
#ifdef ACCELEROMETER_CONTROLL
			if(copy_from_user((void *)&senser_data, argp, (sizeof(int)*9))!=0)
                        {
                                printk("copy from user error.\n");
                                return -EPERM;
			}
#else		
			if(copy_from_user((void *)&senser_data, argp, (sizeof(int)*6))!=0)
                        {
                                printk("copy from user error.\n");
                                return -EPERM;
			}
#endif			
			mutex_lock(&sense_data_mutex);
			 
			for (i=0; i<SENSOR_DATA_SIZE; i++)
			{
				sense_data[i] = senser_data[i];
			}
			mutex_unlock(&sense_data_mutex);

                        break;
		case IOCTL_SENSOR_GET_ACC_FLAG:	
			if(atomic_read(&a_flag))
				acc_sensor = 1;
			else acc_sensor = 0;

			if(copy_to_user(argp, (void *)&acc_sensor,sizeof(acc_sensor))!=0)
                        {
                                printk("copy to user error.\n");
                                return -EPERM;
                        }
			break;
		case IOCTL_SENSOR_GET_COMPASS_FLAG:
			
			if((atomic_read(&m_flag)) || (atomic_read(&mv_flag)))
				compass_sensor = 1;
			else compass_sensor = 0;

			if(copy_to_user(argp, (void *)&compass_sensor,sizeof(compass_sensor))!=0)
                        {
                                printk("copy to user error.\n");
                                return -EPERM;
                        }
			break;
		}
	return 0;
}

#ifdef ST480_BURST_MODE
static irqreturn_t senodia_interrupt(int irq,void *dev_id)
{
	disable_irq_nosync(senodia->client->irq);
	schedule_work(&burst_work);

	return IRQ_HANDLED;
}
#endif

#ifdef SENODIA_EARLY_SUSPEND
static void senodia_early_suspend(struct early_suspend *handler)
{
	SENODIAFUNC("senodia_early_suspend");
	atomic_set(&reserve_open_flag, atomic_read(&open_flag));
	atomic_set(&open_flag, 0);
#ifdef ST480_BURST_MODE
	disable_irq(senodia->client->irq);
#endif
	cancel_delayed_work(&senodia->work);
	SENODIADBG("suspended with flag=%d", 
	       atomic_read(&reserve_open_flag));
}

static void senodia_early_resume(struct early_suspend *handler)
{
	SENODIAFUNC("senodia_early_resume");
	atomic_set(&open_flag, atomic_read(&reserve_open_flag));
#ifdef ST480_BURST_MODE
	enable_irq(senodia->client->irq);
#endif
	schedule_delayed_work(&senodia->work,msecs_to_jiffies(senodiad_delay));
	SENODIADBG("resumed with flag=%d", 
	       atomic_read(&reserve_open_flag));
}
#endif

#ifdef SENODIA_SUSPEND
static int senodia_suspend(struct i2c_client *client, pm_message_t mesg)
{
	SENODIAFUNC("SENODIA_early_suspend");
	atomic_set(&reserve_open_flag, atomic_read(&open_flag));
	atomic_set(&open_flag, 0);
#ifdef ST480_BURST_MODE
	disable_irq(senodia->client->irq);
#endif
	cancel_delayed_work(&senodia->work);
	SENODIADBG("suspended with flag=%d", 
	       atomic_read(&reserve_open_flag));
	return 0;
}

static int senodia_resume(struct i2c_client *client)
{
	SENODIAFUNC("SENODIA_early_resume");
	atomic_set(&open_flag, atomic_read(&reserve_open_flag));
#ifdef ST480_BURST_MODE
	enable_irq(senodia->client->irq);
#endif
	schedule_delayed_work(&senodia->work,msecs_to_jiffies(senodiad_delay)); 
	SENODIADBG("resumed with flag=%d", 
	       atomic_read(&reserve_open_flag));
	return 0;
}
#endif

/*********************************************/
static struct file_operations senodiad_fops = {
	.owner = THIS_MODULE,
	.open = senodiad_open,
	.release = senodiad_release,
#ifdef OLD_KERNEL_VERSION
	.ioctl = senodiad_ioctl,
#else
	.unlocked_ioctl = senodiad_ioctl,
#endif
};

static struct file_operations senodia_aot_fops = {
	.owner = THIS_MODULE,
	.open = senodia_aot_open,
	.release = senodia_aot_release,
#ifdef OLD_KERNEL_VERSION
	.ioctl = senodia_aot_ioctl,
#else
	.unlocked_ioctl = senodia_aot_ioctl,
#endif
};

static struct miscdevice senodiad_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "senodia_dev",
	.fops = &senodiad_fops,
};

static struct miscdevice senodia_aot_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "senodia_aot",
	.fops = &senodia_aot_fops,
};

/*********************************************/
#ifndef ST480_BURST_MODE
#ifdef SENSOR_AUTO_TEST
static int sensor_test_read(void)

{
        senodia_work_func();
        return 0;
}

static int auto_test_read(void *unused)
{
        while(1){
                sensor_test_read();
                msleep(200);
        }
        return 0;
}
#endif
#endif


int senodia_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;

#ifndef ST480_BURST_MODE
#ifdef SENSOR_AUTO_TEST
        struct task_struct *thread;
#endif
#endif

	SENODIAFUNC("senodia_probe");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "SENODIA senodia_probe: check_functionality failed.\n");
		err = -ENODEV;
		goto exit0;
	}
	
	/* Allocate memory for driver data */
	senodia = kzalloc(sizeof(struct senodia_data), GFP_KERNEL);
	if (!senodia) {
		printk(KERN_ERR "SENODIA senodia_probe: memory allocation failed.\n");
		err = -ENOMEM;
		goto exit1;
	}
	
	senodia->client = client;
	
	i2c_set_clientdata(client, senodia);
	
	INIT_DELAYED_WORK(&senodia->work, sensor_input_func);

//IRQ
#ifdef ST480_BURST_MODE
	INIT_WORK(&burst_work, senodia_work_func);
	
	err = gpio_request(client->irq, "st480_irq");
	if (err) 
	{
		printk( "Failed to request st480 irq GPIO!\n");
		goto exit4;
	}
    	err = gpio_direction_input(client->irq);
   	if (err) 
	{
        	printk("failed to set st480 irq gpio input\n");
		goto exit4;
    	}
	gpio_pull_updown(client->irq, GPIOPullUp);

	err = request_irq(client->irq, senodia_interrupt, IRQ_TYPE_EDGE_RISING, "senodia_st480", senodia);
	if(err < 0)
	{
		printk(KERN_ERR "ST480: request irq failed\n");
		goto exit4;
	}
	
#endif

	if(st480_init(senodia->client) != 0)
        {
                printk("st480 init error!\n");
		goto exit5;
        }
	
	/* Declare input device */
	senodia->input_dev = input_allocate_device();
	if (!senodia->input_dev) {
		err = -ENOMEM;
		printk(KERN_ERR
		       "SENODIA senodia_probe: Failed to allocate input device\n");
		goto exit5;
	}
	/* Setup input device */
	set_bit(EV_ABS, senodia->input_dev->evbit);
	/* yaw (0, 360) */
	input_set_abs_params(senodia->input_dev, ABS_RX, ABSMIN_YAW, ABSMAX_YAW, 0, 0);
	/* pitch (-90, 90) */
	input_set_abs_params(senodia->input_dev, ABS_RY, ABSMIN_PITCH, ABSMAX_PITCH, 0, 0);
	/* roll (-180, 180) */
	input_set_abs_params(senodia->input_dev, ABS_RZ, ABSMIN_ROLL, ABSMAX_ROLL, 0, 0);
#ifdef ACCELEROMETER_CONTROLL
	/* x-axis lis3de acceleration (0, 255) */
	input_set_abs_params(senodia->input_dev, ABS_X, ABSMIN_ACC, ABSMAX_ACC, 0, 0);
	/* y-axis lis33de acceleration (0, 255) */
	input_set_abs_params(senodia->input_dev, ABS_Y, ABSMIN_ACC, ABSMAX_ACC, 0, 0);
	/* z-axis lis33de acceleration (0, 255) */
	input_set_abs_params(senodia->input_dev, ABS_Z, ABSMIN_ACC, ABSMAX_ACC, 0, 0);
#endif
	/* x-axis of raw magnetic vector (-32768, 32767) */
	input_set_abs_params(senodia->input_dev, ABS_HAT0X, ABSMIN_MAG, ABSMAX_MAG, 0, 0);
	/* y-axis of raw magnetic vector (-32768, 32767) */
	input_set_abs_params(senodia->input_dev, ABS_HAT0Y, ABSMIN_MAG, ABSMAX_MAG, 0, 0);
	/* z-axis of raw magnetic vector (-32768, 32767) */
	input_set_abs_params(senodia->input_dev, ABS_BRAKE, ABSMIN_MAG, ABSMAX_MAG, 0, 0);
	/* Set name */
	senodia->input_dev->name = "compass";
	
	/* Register */
	err = input_register_device(senodia->input_dev);
	if (err) {
		printk(KERN_ERR
		       "SENODIA senodia_probe: Unable to register input device\n");
		goto exit6;
	}

	err = misc_register(&senodiad_device);
	if (err) {
		printk(KERN_ERR
			   "SENODIA senodia_probe: senodiad_device register failed\n");
		goto exit7;
	}
	
	err = misc_register(&senodia_aot_device);
	if (err) {
		printk(KERN_ERR
		       "SENODIA senodia_probe: senodia_aot_device register failed\n");
		goto exit8;
	}
		

	mutex_init(&sense_data_mutex);	
	
	/* As default, report all information */
	atomic_set(&m_flag, 1);
	atomic_set(&a_flag, 1);
	atomic_set(&mv_flag, 1);

#ifdef SENODIA_EARLY_SUSPEND	
	senodia->senodia_early_suspend.suspend = senodia_early_suspend;
	senodia->senodia_early_suspend.resume = senodia_early_resume;
	register_early_suspend(&senodia->senodia_early_suspend);
#endif

#ifndef ST480_BURST_MODE
#ifdef SENSOR_AUTO_TEST
	thread=kthread_run(auto_test_read,NULL,"st480_read_test");
#endif
#endif	

	printk("Senodia compass successfully probed.");
	return 0;
	
exit8:
	misc_deregister(&senodiad_device);
	cancel_delayed_work(&senodia->work);
exit7:
	input_unregister_device(senodia->input_dev);
	cancel_delayed_work(&senodia->work);
exit6:
	input_free_device(senodia->input_dev);
exit5:
#ifdef ST480_BURST_MODE
exit4:
	free_irq(client->irq, senodia);
#endif
exit3:
	kfree(senodia);
exit1:
exit0:
	return err;
	
}

static int senodia_remove(struct i2c_client *client)
{
	SENODIAFUNC("SENODIA_remove");
#ifdef SENODIA_EARLY_SUSPEND
	unregister_early_suspend(&senodia->senodia_early_suspend);
#endif
	misc_deregister(&senodia_aot_device);
	misc_deregister(&senodiad_device);
	input_unregister_device(senodia->input_dev);
	cancel_delayed_work(&senodia->work);
#ifdef ST480_BURST_MODE
    free_irq(client->irq, senodia);
#endif
	kfree(senodia);
	SENODIADBG("successfully removed.");
	return 0;
}

static const struct i2c_device_id senodia_id[] = {
	{SENODIA_I2C_NAME, 0 },
	{ }
};

static struct i2c_driver senodia_driver = {
	.probe		= senodia_probe,
	.remove 	= senodia_remove,
#ifdef SENODIA_SUSPEND
	.suspend = senodia_suspend,
	.resume = senodia_resume,
#endif
	.id_table	= senodia_id,
	.driver = {
		.name = SENODIA_I2C_NAME,
	},
};

static int __init senodia_init(void)
{
	printk(KERN_INFO "senodia compass driver: initialize\n");
	return i2c_add_driver(&senodia_driver);
}

static void __exit senodia_exit(void)
{
	printk(KERN_INFO "senodia compass driver: release\n");
	i2c_del_driver(&senodia_driver);
}

module_init(senodia_init);
module_exit(senodia_exit);

MODULE_AUTHOR("Tori Xu <xuezhi_xu@senodia.com,tori.xz.xu@gmail.com>");
MODULE_DESCRIPTION("senodia compass driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("6.0");
