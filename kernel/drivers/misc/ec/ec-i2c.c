/* drivers/input/ec/ec-i2c.c - ec i2c handle
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
#include <linux/input/mt.h>
#include <linux/power_supply.h>
#include <mach/gpio.h>
#include <mach/board.h> 
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
	 
#include <linux/ec-dev.h>


#define TS_I2C_RATE 200*1000

static int ec_i2c_read_device(struct ec_private_data *ec, unsigned short reg,
				  int bytes, void *dest, int reg_size)
{
	struct i2c_client *client = ec->control_data;
	struct i2c_adapter *i2c_adap = client->adapter;
	struct i2c_msg msgs[2];
	int i,res;	
	char *buf = dest;
	
	if (!dest || !i2c_adap) {
		printk("%s:line=%d,error\n",__func__,__LINE__);
		return -EINVAL;
	}
	
	//client->addr = ec->ops->slave_addr;

	msgs[0].addr = client->addr;
	msgs[0].flags = 0;	/* write */
	msgs[0].buf = (unsigned char *)&reg;
	if(reg_size == 2)		
	msgs[0].len = 2;
	else	
	msgs[0].len = 1;
	msgs[0].scl_rate = TS_I2C_RATE;
	
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].buf = dest;
	msgs[1].len = bytes;
	msgs[1].scl_rate = TS_I2C_RATE; 

	res = i2c_transfer(i2c_adap, msgs, 2);
	if (res == 2)
		return 0;
	else if(res == 0)
		return -EBUSY;
	else
		return res;
	
	DBG_EC("%s:reg=0x%x,len=%d,rxdata:",__func__, reg, bytes);
	for(i=0; i<bytes; i++)
		DBG_EC("0x%x,",buf[i]);
	DBG_EC("\n");

}

/* Currently we allocate the write buffer on the stack; this is OK for
 * small writes - if we need to do large writes this will need to be
 * revised.
 */
static int ec_i2c_write_device(struct ec_private_data *ec, unsigned short reg,
				   int bytes, void *src, int reg_size)
{
	struct i2c_client *client = ec->control_data;
	struct i2c_adapter *i2c_adap = client->adapter;
	struct i2c_msg msgs[1];
	int res;
	unsigned char buf[bytes + 2];	
	int i = 0;
	
	if (!src || !i2c_adap) {
		printk("%s:line=%d,error\n",__func__,__LINE__);
		return -EINVAL;
	}

	
	//client->addr = ec->ops->slave_addr;
	
	if(reg_size == 2)
	{
		buf[0] = (reg & 0xff00) >> 8;
		buf[1] = (reg & 0x00ff) & 0xff;
		memcpy(&buf[2], src, bytes);
	}
	else
	{
		buf[0] = reg & 0xff;
		memcpy(&buf[1], src, bytes);
	}
	
	DBG_EC("%s:reg=0x%x,len=%d,txdata:",__func__, reg, bytes);
	for(i=0; i<bytes; i++)
		DBG_EC("0x%x,",buf[i]);
	DBG_EC("\n");

	if (!src || !i2c_adap) {
		printk("%s:line=%d,error\n",__func__,__LINE__);
		return -EINVAL;
	}

	msgs[0].addr = client->addr;
	msgs[0].flags = 0;	/* write */
	msgs[0].buf = buf;
	if(reg_size  == 2)		
	msgs[0].len = bytes+2;
	else	
	msgs[0].len = bytes+1;	
	msgs[0].scl_rate = TS_I2C_RATE;

	res = i2c_transfer(i2c_adap, msgs, 1);
	if (res == 1)
		return 0;
	else if(res == 0)
		return -EBUSY;
	else
		return res;
			
}

int ec_bulk_read_normal(struct ec_private_data *ec,
		     int count, unsigned char *buf, int rate)
{
	int ret;
	unsigned short reg;
	struct i2c_client *client = ec->control_data;
	//client->addr = ec->ops->slave_addr;
	
	mutex_lock(&ec->io_lock);
	ret = i2c_master_normal_recv(client, buf, count, rate);
	if(ret == 1)
		ret = 0;
	mutex_unlock(&ec->io_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(ec_bulk_read_normal);


int ec_bulk_write_normal(struct ec_private_data *ec, int count, unsigned char *buf, int rate)
{
	int ret;
	unsigned short reg;
	struct i2c_client *client = ec->control_data;
	//client->addr = ec->ops->slave_addr;
	
	mutex_lock(&ec->io_lock);
	ret = i2c_master_normal_send(client, buf, count, rate);
	if(ret == 1)
		ret = 0;
	mutex_unlock(&ec->io_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(ec_bulk_write_normal);



static int ec_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct ec_private_data *ec;
	int ret,gpio,irq;
	int type = EC_BUS_TYPE_I2C;

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		dev_err(&i2c->adapter->dev, "%s failed\n", __func__);
		return -ENODEV;
	}
	
	ec = kzalloc(sizeof(struct ec_private_data), GFP_KERNEL);
	if (ec == NULL)
		return -ENOMEM;

	i2c_set_clientdata(i2c, ec);
	
	ec->irq = i2c->irq;
	ec->dev = &i2c->dev;
	ec->control_data = i2c;
	ec->read_dev = ec_i2c_read_device;
	ec->write_dev = ec_i2c_write_device;

	ret = ec_device_init(ec, type, ec->irq);
	if(ret)
	{
		printk("%s:fail to regist touch, type is %d\n",__func__, type);
		return -1;
	}
	

	return 0;
}

static int ec_i2c_remove(struct i2c_client *i2c)
{
	struct ec_private_data *ec = i2c_get_clientdata(i2c);

	ec_device_exit(ec);

	return 0;
}

static int ec_suspend(struct i2c_client *i2c, pm_message_t mesg)
{
	struct ec_private_data *ec = i2c_get_clientdata(i2c);
	
	return ec_device_suspend(ec);
}


static int ec_resume(struct i2c_client *i2c)
{
	struct ec_private_data *ec = i2c_get_clientdata(i2c);
	
	return ec_device_resume(ec);
}


static const struct i2c_device_id ec_i2c_id[] = {
	{"ec_dev_i2c", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, ec_i2c_id);

static struct i2c_driver ec_i2c_driver = {
	.driver = {
		.name = "ec_dev_i2c",
		.owner = THIS_MODULE,
	},
	.probe = ec_i2c_probe,
	.remove = ec_i2c_remove,
	.suspend = ec_suspend,
	.resume	= ec_resume,
	.id_table = ec_i2c_id,
};

static int __init ec_i2c_init(void)
{
	int ret;

	printk("%s\n", __FUNCTION__);
	ret = i2c_add_driver(&ec_i2c_driver);
	if (ret != 0)
		pr_err("Failed to register ec I2C driver: %d\n", ret);

	return ret;
}
module_init(ec_i2c_init);

static void __exit ec_i2c_exit(void)
{
	i2c_del_driver(&ec_i2c_driver);
}
module_exit(ec_i2c_exit);

