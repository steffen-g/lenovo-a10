/*
 * BlueBerry tp i2c/smbus read/write driver based on EC
 *
 * Copyright (C) 2013-2013 Paul Ma <magf@bitland.com.cn>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * Trademarks are the property of their respective owners.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <asm/unaligned.h>
#include <linux/idr.h>
#include <asm/atomic.h>
#include <linux/io.h>
#include <linux/ioctl.h>
#include <mach/board.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/cdev.h>
#include <linux/vmalloc.h>
#include "blueberry_tp_i2c.h"

int blueberry_tp_smbus_read_byte(struct i2c_client *client,
                        unsigned char reg_addr, unsigned char *data)
{
    s32 dummy;

    dummy = i2c_smbus_read_byte_data(client, reg_addr);
    if (dummy < 0) {
		mdelay(1);
        return -EPERM;
	}
    *data = dummy & 0x000000ff;

	mdelay(1);
    return 0;
}

int blueberry_tp_smbus_write_byte(struct i2c_client *client,
                        unsigned char reg_addr, unsigned char *data)
{
    s32 dummy;

    dummy = i2c_smbus_write_byte_data(client, reg_addr, *data);
    if (dummy < 0) {
		mdelay(1);
        return -EPERM;
	}

	mdelay(1);
    return 0;
}

int blueberry_tp_smbus_write_byte_no_data(struct i2c_client *client,
						unsigned char reg_addr)
{
	s32 dummy;
	
	dummy = i2c_smbus_write_byte(client, reg_addr);
	if(dummy < 0) {
		mdelay(1);
		return -EPERM;
	}

	mdelay(1);
	return 0;
}

int blueberry_tp_smbus_read_word_data(struct i2c_client *client,
    unsigned char reg_addr, unsigned char *data, unsigned char length)
{
    s32 dummy;

    dummy = i2c_smbus_read_word_data(client, reg_addr);
    if (dummy < 0) {
		mdelay(1);
        return -EPERM;
	}

    *data = dummy & 0x00ff;
    *(data+1) = (dummy & 0xff00) >> 8;

	mdelay(1);
    return 0;
}

int blueberry_tp_smbus_write_word_data(struct i2c_client *client,
    unsigned char reg_addr, unsigned char *data, unsigned char length)
{
    s32 dummy;
	u16 value;

    value = (*(data+1) << 8) | (*(data));
    dummy = i2c_smbus_write_word_data(client, reg_addr, value);
    if (dummy < 0)
    {
        pr_err("blueberry_tp write word data error!\n");
		mdelay(1);
        return -EPERM;
    }

	mdelay(1);
    return 0;
}

int blueberry_tp_smbus_read_byte_block(struct i2c_client *client,
                unsigned char reg_addr, unsigned char *data, unsigned char len)
{
    s32 dummy;

    dummy = i2c_smbus_read_i2c_block_data(client, reg_addr, len, data);
    if (dummy < 0) {
		mdelay(1);
        return -EPERM;
	}

	mdelay(1);
    return 0;
}

int blueberry_tp_smbus_write_byte_block(struct i2c_client *client,
                unsigned char reg_addr, unsigned char *data, unsigned char len)
{
    s32 dummy;

    dummy = i2c_smbus_write_i2c_block_data(client, reg_addr, len, data);
    if (dummy < 0) {
		mdelay(1);
        return -EPERM;
	}

	mdelay(1);
    return 0;
}

int blueberry_tp_i2c_read_data(struct i2c_client *client, char *buf, int length)
{
    struct i2c_msg msgs[] = {
        {
            .addr  =  client->addr,
            .flags  =  0,
            .len  =  1,
            .buf  =  buf,
            .scl_rate = 400 * 1000,
        },
        {
            .addr  =  client->addr,
            .flags  = I2C_M_RD,
            .len  =  length,
            .buf  =  buf,
            .scl_rate = 400 * 1000,
        },
    };

    if(i2c_transfer(client->adapter, msgs, 2) < 0){
        pr_err("blueberry_tp_i2c_read_data: transfer error\n");
		mdelay(1);
        return -EIO;
    }else{
		mdelay(1);
    	return 0;
	}
}

int blueberry_tp_i2c_write_data(struct i2c_client *client, char *buf, int length)
{
    struct i2c_msg msgs[] = {
        {
            .addr = client->addr,
            .flags = 0,
            .len = length,
            .buf = buf,
            .scl_rate = 400 * 1000,
        },
    };

    if (i2c_transfer(client->adapter, msgs, 1) < 0) {
        pr_err("blueberry_tp_i2c_write_data: transfer error\n");
		mdelay(1);
        return -EIO;
    } else {
		mdelay(1);
        return 0;
	}
}

