/* drivers/misc/ec/modules/ec-key.c
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

#include "ec-key.h"

static int ec_active(struct ec_private_data *ec, int enable)
{	
	int result = 0;

	if(enable)
	{
		
	}
	else
	{
		
	}
		
	
	DBG_EC("%s,name=%s\n", __func__,ec->ops[EC_ID_KEY]->name);
	return result;
}

static int ec_init(struct ec_private_data *ec)
{	
	int result = 0;
	int i = 0;
	struct input_dev *input_dev = ec->key.input_dev;
	
	input_dev = input_allocate_device();
	if (!input_dev) {
		result = -ENOMEM;
		dev_err(ec->dev,
			"Failed to allocate input device %s\n", input_dev->name);
		goto error;
	}	
	
	input_dev->dev.parent = ec->dev;
	input_dev->name = ec->ops[EC_ID_KEY]->name;
	
	result = input_register_device(input_dev);
	if (result) {
		dev_err(ec->dev,
			"Unable to register input device %s\n", input_dev->name);
		goto out_input_register_device_failed;
	}

	set_bit(EV_KEY, input_dev->evbit);
	for ( i = 0; i < 246; i++)
	set_bit(i, input_dev->keybit);

	
out_input_register_device_failed:
	input_free_device(input_dev);
error:	
	return result;
}



static int ec_deinit(struct ec_private_data *ec)
{	
	int result = 0;

	
	DBG_EC("%s,name=%s\n", __func__,ec->ops[EC_ID_KEY]->name);
	return result;
}


static int ec_get_key_code(struct ec_private_data *ec, char *keycode)
{
	int result = 0;
	
	result = ec_bulk_read(ec, EC_REG_KEY_CODE, 2, &keycode);
	if(result)
	{
		printk("%s:fail to get ec id,result=%d\n",__func__, result);
		result = -1;
	}

	return result;
}



static int ec_code_to_key(struct ec_private_data *ec, int keycode, int keyvalue)
{
	int result = 0;

	switch(keycode)
	{
		case ASUSDEC_KEYPAD_ESC:
		keyvalue = KEY_BACK;
		break;

		case ASUSDEC_KEYPAD_KEY_WAVE:
		keyvalue = KEY_GRAVE;	    
		break;

		case ASUSDEC_KEYPAD_KEY_1:
		keyvalue = KEY_1;	    
		break;

		case ASUSDEC_KEYPAD_KEY_2:
		keyvalue = KEY_2;	    
		break;

		case ASUSDEC_KEYPAD_KEY_3:
		keyvalue = KEY_3;    
		break;

		case ASUSDEC_KEYPAD_KEY_4:
		keyvalue = KEY_4;    
		break;

		case ASUSDEC_KEYPAD_KEY_5:
		keyvalue = KEY_5;    
		break;

		case ASUSDEC_KEYPAD_KEY_6:
		keyvalue = KEY_6;	    
		break;

		case ASUSDEC_KEYPAD_KEY_7:
		keyvalue = KEY_7;    
		break;

		case ASUSDEC_KEYPAD_KEY_8:
		keyvalue = KEY_8;    
		break;

		case ASUSDEC_KEYPAD_KEY_9:
		keyvalue = KEY_9;	    
		break;

		case ASUSDEC_KEYPAD_KEY_0:
		keyvalue = KEY_0;	    
		break;

		case ASUSDEC_KEYPAD_KEY_MINUS:
		keyvalue = KEY_MINUS;
		break;

		case ASUSDEC_KEYPAD_KEY_EQUAL:
		keyvalue = KEY_EQUAL;
		break;

		case ASUSDEC_KEYPAD_KEY_BACKSPACE:
		keyvalue = KEY_BACKSPACE;
		break;

		case ASUSDEC_KEYPAD_KEY_TAB:
		keyvalue = KEY_TAB;
		break;

		case ASUSDEC_KEYPAD_KEY_Q:
		keyvalue = KEY_Q;
		break;

		case ASUSDEC_KEYPAD_KEY_W:
		keyvalue = KEY_W;
		break;

		case ASUSDEC_KEYPAD_KEY_E:
		keyvalue = KEY_E;
		break;

		case ASUSDEC_KEYPAD_KEY_R:
		keyvalue = KEY_R;
		break;

		case ASUSDEC_KEYPAD_KEY_T:
		keyvalue = KEY_T;
		break;

		case ASUSDEC_KEYPAD_KEY_Y:
		keyvalue = KEY_Y;
		break;

		case ASUSDEC_KEYPAD_KEY_U:
		keyvalue = KEY_U;
		break;

		case ASUSDEC_KEYPAD_KEY_I:
		keyvalue = KEY_I;
		break;

		case ASUSDEC_KEYPAD_KEY_O:
		keyvalue = KEY_O;
		break;

		case ASUSDEC_KEYPAD_KEY_P:
		keyvalue = KEY_P;
		break;

		case ASUSDEC_KEYPAD_KEY_LEFTBRACE:
		keyvalue = KEY_LEFTBRACE;
		break;

		case ASUSDEC_KEYPAD_KEY_RIGHTBRACE:
		keyvalue = KEY_RIGHTBRACE;
		break;

		case ASUSDEC_KEYPAD_KEY_BACKSLASH:
		keyvalue = KEY_BACKSLASH;
		break;

		case ASUSDEC_KEYPAD_KEY_CAPSLOCK:
		keyvalue = KEY_CAPSLOCK;
		break;

		case ASUSDEC_KEYPAD_KEY_A:
		keyvalue = KEY_A;
		break;

		case ASUSDEC_KEYPAD_KEY_S:
		keyvalue = KEY_S;
		break;

		case ASUSDEC_KEYPAD_KEY_D:
		keyvalue = KEY_D;
		break;

		case ASUSDEC_KEYPAD_KEY_F:
		keyvalue = KEY_F;
		break;

		case ASUSDEC_KEYPAD_KEY_G:
		keyvalue = KEY_G;
		break;

		case ASUSDEC_KEYPAD_KEY_H:
		keyvalue = KEY_H;
		break;

		case ASUSDEC_KEYPAD_KEY_J:
		keyvalue = KEY_J;
		break;

		case ASUSDEC_KEYPAD_KEY_K:
		keyvalue = KEY_K;
		break;

		case ASUSDEC_KEYPAD_KEY_L:
		keyvalue = KEY_L;
		break;

		case ASUSDEC_KEYPAD_KEY_SEMICOLON:
		keyvalue = KEY_SEMICOLON;
		break;

		case ASUSDEC_KEYPAD_KEY_APOSTROPHE:
		keyvalue = KEY_APOSTROPHE;
		break;

		case ASUSDEC_KEYPAD_KEY_ENTER:
		keyvalue = KEY_ENTER;
		break;

		case ASUSDEC_KEYPAD_KEY_LEFTSHIFT:
		keyvalue = KEY_LEFTSHIFT;
		break;

		case ASUSDEC_KEYPAD_KEY_Z:
		keyvalue = KEY_Z;
		break;

		case ASUSDEC_KEYPAD_KEY_X:
		keyvalue = KEY_X;
		break;

		case ASUSDEC_KEYPAD_KEY_C:
		keyvalue = KEY_C;
		break;

		case ASUSDEC_KEYPAD_KEY_V:
		keyvalue = KEY_V;
		break;

		case ASUSDEC_KEYPAD_KEY_B:
		keyvalue = KEY_B;
		break;

		case ASUSDEC_KEYPAD_KEY_N:
		keyvalue = KEY_N;
		break;

		case ASUSDEC_KEYPAD_KEY_M:
		keyvalue = KEY_M;
		break;

		case ASUSDEC_KEYPAD_KEY_COMMA:
		keyvalue = KEY_COMMA;
		break;

		case ASUSDEC_KEYPAD_KEY_DOT:
		keyvalue = KEY_DOT;
		break;

		case ASUSDEC_KEYPAD_KEY_SLASH:
		keyvalue = KEY_SLASH;
		break;

		case ASUSDEC_KEYPAD_KEY_RIGHTSHIFT:
		keyvalue = KEY_RIGHTSHIFT;
		break;

		case ASUSDEC_KEYPAD_KEY_LEFT:
		keyvalue = KEY_LEFT;
		break;

		case ASUSDEC_KEYPAD_KEY_RIGHT:
		keyvalue = KEY_RIGHT;
		break;

		case ASUSDEC_KEYPAD_KEY_UP:
		keyvalue = KEY_UP;
		break;

		case ASUSDEC_KEYPAD_KEY_DOWN:
		keyvalue = KEY_DOWN;
		break;

		case ASUSDEC_KEYPAD_RIGHTWIN:
		keyvalue = KEY_SEARCH;
		break;

		case ASUSDEC_KEYPAD_LEFTCTRL:
		keyvalue = KEY_LEFTCTRL;

		case ASUSDEC_KEYPAD_LEFTWIN:
		keyvalue = KEY_HOMEPAGE;
		break;

		case ASUSDEC_KEYPAD_LEFTALT:
		keyvalue = KEY_LEFTALT;
		break;

		case ASUSDEC_KEYPAD_KEY_SPACE:
		keyvalue = KEY_SPACE;
		break;

		case ASUSDEC_KEYPAD_RIGHTALT:
		keyvalue = KEY_RIGHTALT;
		break;

		case ASUSDEC_KEYPAD_WINAPP:
		keyvalue = KEY_MENU;
		break;

		case ASUSDEC_KEYPAD_RIGHTCTRL:
		keyvalue = KEY_RIGHTCTRL;
		break;

		case ASUSDEC_KEYPAD_HOME:
		keyvalue = KEY_HOME;
		break;

		case ASUSDEC_KEYPAD_PAGEUP:
		keyvalue = KEY_PAGEUP;
		break;

		case ASUSDEC_KEYPAD_PAGEDOWN:
		keyvalue = KEY_PAGEDOWN;
		break;

		case ASUSDEC_KEYPAD_END:
		keyvalue = KEY_END;
		break;

		case ASUSDEC_KEYPAD_F1:
		keyvalue = KEY_HOME;
		break;

		case ASUSDEC_KEYPAD_F2:
		keyvalue = KEY_LAUNCHER;//250	
		break;

		case ASUSDEC_KEYPAD_F3:
		keyvalue = KEY_SETTING;//251
		break;

		case ASUSDEC_KEYPAD_F4:
		keyvalue = KEY_MUTE;
		break;

		case ASUSDEC_KEYPAD_F5:
		keyvalue = KEY_VOLUMEDOWN;
		break;

		case ASUSDEC_KEYPAD_F6:
		keyvalue = KEY_VOLUMEUP;
		break;

		case ASUSDEC_KEYPAD_F7:
		keyvalue = KEY_BRIGHTNESSDOWN;
		break;

		case ASUSDEC_KEYPAD_F8:
		keyvalue = KEY_BRIGHTNESSUP;
		break;

		case ASUSDEC_KEYPAD_F9:
		keyvalue = KEY_WLAN;
		break;

		case ASUSDEC_KEYPAD_F10:
		keyvalue = KEY_BLUETOOTH;
		break;

		case ASUSDEC_KEYPAD_F11:
		keyvalue = KEY_TOUCHPAD;//252
		break;

		case ASUSDEC_KEYPAD_F12:
		keyvalue = KEY_FORCE_ROTATION;//253
		break;

		case ASUSDEC_KEYPAD_INSERT:
		keyvalue = KEY_WWW;
		break;

		case ASUSDEC_KEYPAD_PRINTSCREEN:
		keyvalue = KEY_SCREENSHOT;//254
		break;

		case ASUSDEC_KEYPAD_DELETE:
		keyvalue = KEY_DELETE;
		break;

		default:
		keyvalue = -1;
		break;
	}
	
	return result;
}


static int ec_handle_data(struct ec_private_data *ec)
{
	//struct ec_platform_data *pdata = ec->pdata;	
	int result = 0;
	int keycode = 0 , keystate = 0, keyvalue = 0;
	struct input_dev *input_dev = ec->key.input_dev;
	char code[2] = {0, 0};

	result = ec_get_key_code(ec, code);
	if(result)
	{
		printk("%s:fail to get keycode,ret=%d\n",__func__, result);
		goto error;
	}


	keycode = code[0];
	keystate = code[1];
		
	//to handle data and report value
	result = ec_code_to_key(ec, keycode, keyvalue);
	if(result)
	{
		printk("%s:fail to get keyvalue,ret=%d\n",__func__, result);
		goto error;
	}
	
	if(keyvalue)
	{
		if (keystate)
			input_report_key(input_dev, keyvalue, keystate);		
		else 
			input_report_key(input_dev, keyvalue, !!keystate);

		input_sync(input_dev);
	}
	
	
	DBG_EC("%s,name=%s\n", __func__,ec->ops[EC_ID_KEY]->name);
	
error:
	return result;
}

static int ec_suspend(struct ec_private_data *ec)
{
	if(ec->ops[EC_ID_KEY]->active)
		ec->ops[EC_ID_KEY]->active(ec, 0);

	
	DBG_EC("%s,name=%s\n", __func__,ec->ops[EC_ID_KEY]->name);
	return 0;
}


static int ec_resume(struct ec_private_data *ec)
{	
	if(ec->ops[EC_ID_KEY]->active)
		ec->ops[EC_ID_KEY]->active(ec, 1);

	
	DBG_EC("%s,name=%s\n", __func__,ec->ops[EC_ID_KEY]->name);
	
	return 0;
}


struct ec_operate ec_key_ops = {
	.name				= "ec_key",
	.ec_id				= EC_ID_KEY,			//i2c id number
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
	return &ec_key_ops;
}


static int __init ec_key_init(void)
{
	struct ec_operate *ops = ec_get_ops();
	int result = 0;
	result = ec_register_slave(NULL, NULL, ec_get_ops);	
	return result;
}

static void __exit ec_key_exit(void)
{
	struct ec_operate *ops = ec_get_ops();
	ec_unregister_slave(NULL, NULL, ec_get_ops);
}


subsys_initcall_sync(ec_key_init);
module_exit(ec_key_exit);

