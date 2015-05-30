/* drivers/misc/ec/modules/ec-tp.c
 *
 * Copyright (C) 2012-2015 ROCKCHIP.
 * Author: luowei <lw@rock-chips.com>
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
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include <linux/proc_fs.h>
#include <linux/input/mt.h>
#include <linux/power_supply.h>
#include <mach/gpio.h>
#include <mach/board.h> 	 
#include <linux/ec-dev.h>

static int ec_active(struct ec_private_data *ec, int enable)
{	
	int result = 0;

	if(enable)
	{
		
	}
	else
	{
		
	}
		

	
	DBG_EC("%s,enable=%d, name=%s\n", __func__,enable, ec->ops[EC_ID_TP]->name);
	return result;
}


static int ec_init(struct ec_private_data *ec)
{	
	int result = 0;
	struct input_dev *input_dev = ec->tp.input_dev;
	
	input_dev = input_allocate_device();
	if (!input_dev) {
		result = -ENOMEM;
		dev_err(ec->dev,
			"Failed to allocate input device %s\n", input_dev->name);
		goto error;
	}	
	
	input_dev->dev.parent = ec->dev;
	input_dev->name = ec->ops[EC_ID_TP]->name;
	
	result = input_register_device(input_dev);
	if (result) {
		dev_err(ec->dev,
			"Unable to register input device %s\n", input_dev->name);
		goto out_input_register_device_failed;
	}

	input_set_capability(input_dev, EV_REL, REL_X);
	input_set_capability(input_dev, EV_REL, REL_Y);
	//input_set_capability(input_dev, EV_REL, REL_WHEEL);
	input_set_capability(input_dev, EV_KEY, BTN_LEFT);
	input_set_capability(input_dev, EV_KEY, BTN_RIGHT);
	//input_set_capability(input_dev, EV_KEY, BTN_MIDDLE);
	
out_input_register_device_failed:
	input_free_device(input_dev);
error:	
	return result;
}



static int ec_deinit(struct ec_private_data *ec)
{	
	int result = 0;

	
	DBG_EC("%s,name=%s\n", __func__,ec->ops[EC_ID_TP]->name);
	return result;
}



static int ec_handle_data(struct ec_private_data *ec)
{
	//struct ec_platform_data *pdata = ec->pdata;	
	int result = 0;

	//to handle data and report value

	
	DBG_EC("%s,name=%s\n", __func__,ec->ops[EC_ID_TP]->name);
	return result;
}

static int ec_suspend(struct ec_private_data *ec)
{
	if(ec->ops[EC_ID_TP]->active)
		ec->ops[EC_ID_TP]->active(ec, 0);

	
	DBG_EC("%s,name=%s\n", __func__,ec->ops[EC_ID_TP]->name);
	return 0;
}


static int ec_resume(struct ec_private_data *ec)
{	
	if(ec->ops[EC_ID_TP]->active)
		ec->ops[EC_ID_TP]->active(ec, 1);

	
	DBG_EC("%s,name=%s\n", __func__,ec->ops[EC_ID_TP]->name);
	return 0;
}


struct ec_operate ec_tp_ops = {
	.name				= "ec_tp",
	.ec_id				= EC_ID_TP,			//i2c id number
	.slave_addr			= 0x61,
	.bus_type			= EC_BUS_TYPE_I2C,
	
	.active				= ec_active,
	.init				= ec_init,	
	.deinit				= ec_deinit,	
	.handle				= ec_handle_data,
	.suspend			= ec_suspend,
	.resume				= ec_resume,
	.misc_dev			= NULL,
};

/****************operate according to ec chip:end************/

//function name should not be changed
static struct ec_operate *ec_get_ops(void)
{
	return &ec_tp_ops;
}


static int __init ec_tp_init(void)
{
	struct ec_operate *ops = ec_get_ops();
	int result = 0;
	result = ec_register_slave(NULL, NULL, ec_get_ops);	
	return result;
}

static void __exit ec_tp_exit(void)
{
	struct ec_operate *ops = ec_get_ops();
	ec_unregister_slave(NULL, NULL, ec_get_ops);
}


subsys_initcall_sync(ec_tp_init);
module_exit(ec_tp_exit);

