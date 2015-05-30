/*
 *  smsc.c - Linux kernel module for
 *
 *  Copyright (c) 2009 Daniel Mack <daniel@caiaq.de>
 *
 *  Based on code written by
 *  	Rodolfo Giometti <giometti@linux.it>
 *  	Eurotech S.p.A. <info@eurotech.it>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <mach/board.h>

#define USB4604_DRV_NAME	"usb4604"
#define DRIVER_VERSION		"1.0"

struct smsc_data {
	struct i2c_client *client;
	struct mutex lock;
};

static struct smsc_data *psmsc_data;
static int smsc_i2c_read_interface(struct i2c_client *client,unsigned char address,unsigned char reg,
												u8* read_buf, int read_len) 
{
	int ret;
	struct i2c_msg msgs[2];

	msgs[0].addr = address;
	msgs[0].flags = 0;	/* write */
	msgs[0].buf = &reg;
	msgs[0].len = 1;
	msgs[0].scl_rate = 200*1000;
	
	msgs[1].addr = address;
	msgs[1].flags = I2C_M_RD;
	msgs[1].buf = read_buf;
	msgs[1].len = read_len;
	msgs[1].scl_rate = 200*1000;	

	ret = i2c_transfer(client->adapter, msgs, 2);
	if (ret == 2)
		return 0;
	else if(ret == 0)
		return -EBUSY;
	else
		return ret;

	return 0;
}

static bool  smsc_i2c_write_interface(struct i2c_client *client,unsigned char addr,u8* write_buf, int write_len)
{
	int ret = 0;
	
	struct i2c_msg msgs[2];
	
	msgs[0].addr = addr;
	msgs[0].flags = 0;	/* write */
	msgs[0].buf = (unsigned char *)write_buf;
	msgs[0].len = write_len;
	msgs[0].scl_rate = 200*1000;

	ret = i2c_transfer(client->adapter, msgs, 1);
	if (ret == 1)
		return 0;
	else if(ret == 0)
		return -EBUSY;
	else
		return ret;

}

static int smsc_start(struct i2c_client *client)
{
	int ret = -1;
	unsigned char buf[9];
	//disable internal mcu
	buf[0] = 0x00;
	buf[1] = 0x00;
	buf[2] = 0x05;
	buf[3] = 0x00;
	buf[4] = 0x01;
	buf[5] = 0x41;
	buf[6] = 0x30;
	buf[7] = 0x02;
	buf[8] = 0x00;

	ret = smsc_i2c_write_interface(client , client->addr, buf,9);
	printk("disable internal mcu of usb4604 [ step 1 ] ,ret is %d\n",ret);
	
	buf[0] = 0x99;
	buf[1] = 0x37;
	buf[2] = 0x00;
	ret = smsc_i2c_write_interface(client , client->addr, buf,3);
	
	printk("disable internal mcu of usb4604 [ step 2 ],ret is %d\n",ret);

#if 0
	/*****************configure USB Port 1 VariSense Register*************************/
	buf[0] = 0x00;
	buf[1] = 0x00;//memory address 0x0000H
	buf[2] = 0x05;//count,number of bytes to write to memory
	buf[3] = 0x00;//write configuration register
	buf[4] = 0x01;//writing one data byte
	buf[5] = 0x64;
	buf[6] = 0xCC;//configuration address
	buf[7] = 0x05;//the data to write to the configuration address
	buf[8] = 0x00;
	ret = smsc_i2c_write_interface(client , client->addr, buf,9);
	
	buf[0] = 0x99;
	buf[1] = 0x37;
	buf[2] = 0x00;
	ret = smsc_i2c_write_interface(client , client->addr, buf,3);
	/*****************configure USB Port 1 VariSense Register*************************/
#endif
	buf[0] = 0xAA;
	buf[1] = 0x55;
	buf[2] = 0x00;
	ret = smsc_i2c_write_interface(client , client->addr, buf,3);
	printk("enable  communication of usb4604 ,ret is %d\n",ret);

	return ret;
}
static int __devinit smsc_probe(struct i2c_client *client,
				    const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct smsc_data *data;
	struct usb4604_platform_data *pdata = client->dev.platform_data;

	int err = 0;

	if(pdata->usb4604_platform_hw_init)
		pdata->usb4604_platform_hw_init();
	
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		return -EIO;

	data = kzalloc(sizeof(struct smsc_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	psmsc_data = data;
	data->client = client;
	i2c_set_clientdata(client, data);
	mutex_init(&data->lock);
	
#if defined (CONFIG_MACH_RK3188_FLEX10) || defined (CONFIG_MACH_RK3188_A10)
	//CONFIG_MACH_RK3188_S115A use usb3503,usb3503 dont need to write register.
	//usb4604 need to write register 0xAA.
	smsc_start(client);
#endif
	dev_info(&client->dev, "driver version %s enabled\n", DRIVER_VERSION);
	return 0;

exit_kfree:
	kfree(data);
	return err;
}
int smsc_reinit(void)
{
	struct usb4604_platform_data *pdata = psmsc_data->client->dev.platform_data;

	if(pdata->usb4604_platform_hw_init)
		pdata->usb4604_platform_hw_init();
	
#if defined (CONFIG_MACH_RK3188_FLEX10) || defined (CONFIG_MACH_RK3188_A10)
	//CONFIG_MACH_RK3188_S115A use usb3503,usb3503 dont need to write register.
	//usb4604 need to write register 0xAA.
	smsc_start(psmsc_data->client);
#endif
	dev_info(&psmsc_data->client->dev, "driver version %s enabled\n", DRIVER_VERSION);
	return 0;
}
EXPORT_SYMBOL(smsc_reinit);
static int __devexit smsc_remove(struct i2c_client *client)
{
	kfree(i2c_get_clientdata(client));
	return 0;
}

static const struct i2c_device_id smsc_id[] = {
	{ "usb4604", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, smsc_id);

static struct i2c_driver smsc_driver = {
	.driver = {
		.name	= USB4604_DRV_NAME,
		.owner	= THIS_MODULE,
	},
	.probe	= smsc_probe,
	.remove	= __devexit_p(smsc_remove),
	.id_table = smsc_id,
};

static int __init smsc_init(void)
{
	return i2c_add_driver(&smsc_driver);
}

static void __exit smsc_exit(void)
{
	i2c_del_driver(&smsc_driver);
}

MODULE_DESCRIPTION("USB4604 hub driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRIVER_VERSION);

module_init(smsc_init);
module_exit(smsc_exit);

