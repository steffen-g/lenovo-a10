/* drivers/misc/ec/ec-dev.c - handle all ec in this file
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

struct ec_private_data *g_ec;
static struct class *g_ec_class;
static struct ec_operate *g_ec_ops[EC_NUM_ID]; 


static ssize_t ec_proc_write(struct file *file, const char __user *buffer,
			   size_t count, loff_t *data)
{
	char c;
	int rc;
	int i = 0, num = 0;
	
	rc = get_user(c, buffer);
	if (rc)
	{
		atomic_set(&g_ec->flags.debug_flag, EC_ID_NULL);
		return rc; 
	}

	
	num = c - '0';

	printk("%s command list:close:0,enable:1\n",__func__);

	if(g_ec)
	atomic_set(&g_ec->flags.debug_flag, num);
	
	
	return count; 
}

static const struct file_operations ec_proc_fops = {
	.owner		= THIS_MODULE, 
	.write		= ec_proc_write,
};




/**
 * ec_reg_read: Read a single ec register.
 *
 * @ec: Device to read from.
 * @reg: Register to read.
 */
int ec_reg_read(struct ec_private_data *ec, unsigned short reg)
{
	unsigned short val;
	int ret;

	mutex_lock(&ec->io_lock);

	ret = ec->read_dev(ec, reg, ec->pdata->reg_size, &val, ec->pdata->reg_size);

	mutex_unlock(&ec->io_lock);

	if (ret < 0)
		return ret;
	else
		return val;
}
EXPORT_SYMBOL_GPL(ec_reg_read);

/**
 * ec_bulk_read: Read multiple ec registers
 *
 * @ec: Device to read from
 * @reg: First register
 * @count: Number of registers
 * @buf: Buffer to fill.
 */
int ec_bulk_read(struct ec_private_data *ec, unsigned short reg,
		     int count, unsigned char *buf)
{
	int ret;

	mutex_lock(&ec->io_lock);

	ret = ec->read_dev(ec, reg, count, buf, ec->pdata->reg_size);

	mutex_unlock(&ec->io_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(ec_bulk_read);


/**
 * ec_reg_write: Write a single ec register.
 *
 * @ec: Device to write to.
 * @reg: Register to write to.
 * @val: Value to write.
 */
int ec_reg_write(struct ec_private_data *ec, unsigned short reg,
		     unsigned short val)
{
	int ret;

	mutex_lock(&ec->io_lock);

	ret = ec->write_dev(ec, reg, ec->pdata->reg_size, &val, ec->pdata->reg_size);

	mutex_unlock(&ec->io_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(ec_reg_write);


int ec_bulk_write(struct ec_private_data *ec, unsigned short reg,
		     int count, unsigned char *buf)
{
	int ret;

	mutex_lock(&ec->io_lock);

	ret = ec->write_dev(ec, reg, count, buf, ec->pdata->reg_size);

	mutex_unlock(&ec->io_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(ec_bulk_write);



/**
 * ec_set_bits: Set the value of a bitfield in a ec register
 *
 * @ec: Device to write to.
 * @reg: Register to write to.
 * @mask: Mask of biec to set.
 * @val: Value to set (unshifted)
 */
int ec_set_bits(struct ec_private_data *ec, unsigned short reg,
		    unsigned short mask, unsigned short val)
{
	int ret;
	u16 r;

	mutex_lock(&ec->io_lock);

	ret = ec->read_dev(ec, reg, ec->pdata->reg_size, &r, ec->pdata->reg_size);
	if (ret < 0)
		goto out;

	r &= ~mask;
	r |= val;

	ret = ec->write_dev(ec, reg, ec->pdata->reg_size, &r, ec->pdata->reg_size);

out:
	mutex_unlock(&ec->io_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(ec_set_bits);

static int ec_get_ec_id(struct ec_private_data *ec, char ec_id)
{
	int result = 0;
#if 0	
	result = ec_bulk_read(ec, EC_REG_TOP_STATUS, 1, &ec_id);
	if(result)
	{
		printk("%s:fail to get ec id,result=%d\n",__func__, result);
		result = -1;
	}

	

#else
	ec_id = EC_ID_KEY;
#endif
	return result;
}


static int ec_chip_init(struct ec_private_data *ec, int type)
{
	int result = 0;
	int i = 0;

	if((type <= EC_BUS_TYPE_INVALID) || (type >= EC_BUS_TYPE_NUM_ID))
	{
		printk("%s:type=%d is error\n",__func__,type);
		return -1;	
	}
	
	if(ec->pdata && ec->pdata->init_platform_hw)
		ec->pdata->init_platform_hw(ec->pdata);
		
	for(i=EC_ID_NULL+1; i<EC_NUM_ID; i++)
	{
		if(g_ec_ops[i])
		{
			ec->ops[i] = g_ec_ops[i];
			printk("%s:ec->ops[%d]->name=%s\n",__FUNCTION__,i,ec->ops[i]->name);
		}
	}

	for(i=EC_ID_NULL+1; i<EC_NUM_ID; i++)
	{
		if(ec->ops[i] && ec->ops[i]->init)
		result = ec->ops[i]->init(ec);
		if(result)
		printk("%s:fail to init ec module %s\n",__FUNCTION__, ec->ops[i]->name);
	}
	
	
	return 0;

}


static int ec_ec_wakeup_ap(struct ec_private_data *ec)
{		
	int result = 0;

	
	wake_lock_timeout(&ec->ec_wakelock, 10 * HZ);
	
	DBG_EC("%s:ec wake up ap,wake_pin=%d\n",__FUNCTION__, ec->pdata->ec_wakeup_ap);

	return result;	
}



static int ec_ap_wakeup_ec(struct ec_private_data *ec)
{		
	int result = 0;
	
	
	
	DBG_EC("%s\n", __func__);
	return result;	
}



static int ec_handle_data(struct ec_private_data *ec)
{		
	int i = 0;
	int result = 0;
	char ec_id = 0;

	
	DBG_EC("%s\n", __func__);
	
	result = ec_get_ec_id(ec, ec_id);
	if(result)
	{
		printk("%s:error:result=%d\n",__func__, result);
		result = -1;
		goto error;
	}
	
	for(i=EC_ID_NULL+1; i<EC_NUM_ID; i++)
	{
		if(ec->ops[i] && ec->ops[i]->handle && (ec_id == i))
		{
			result = ec->ops[i]->handle(ec);
			if(result)
			printk("%s:fail to handle ec module %s\n",__FUNCTION__, ec->ops[i]->name);
		}
	}
	
error:
	return result;	
}

static void  ec_interrupt_delaywork_func(struct work_struct *work)
{
	struct delayed_work *delaywork = container_of(work, struct delayed_work, work);
	struct ec_private_data *ec = container_of(delaywork, struct ec_private_data, delaywork);

	mutex_lock(&ec->ec_lock);	
	if (ec_handle_data(ec) < 0) 
		DBG_EC(KERN_ERR "%s: Handle data failed\n",__func__);
	
	if(!ec->pdata->irq_enable)//restart work while polling
	schedule_delayed_work(&ec->delaywork, msecs_to_jiffies(ec->pdata->poll_delay_ms));
	//else
	//{
		//if((ec->pdata->irq_trig == IRQF_TRIGGER_LOW) || (ec->pdata->irq_trig == IRQF_TRIGGER_HIGH))
		//enable_irq(ec->client->irq);
	//}
	mutex_unlock(&ec->ec_lock);	
}

/*
 * This is a threaded IRQ handler so can access I2C/SPI.  Since all
 * interrupts are clear on read the IRQ line will be reasserted and
 * the physical IRQ will be handled again if another interrupt is
 * asserted while we run - in the normal course of events this is a
 * rare occurrence so we save I2C/SPI reads.  We're also assuming that
 * it's rare to get lots of interrupts firing simultaneously so try to
 * minimise I/O.
 */
static irqreturn_t ec_interrupt(int irq, void *dev_id)
{
	struct ec_private_data *ec = (struct ec_private_data *)dev_id;

	//use threaded IRQ
	if (ec_handle_data(ec) < 0) 
		DBG_EC(KERN_ERR "%s: Get data failed\n",__func__);
	//msleep(ec->pdata->poll_delay_ms);

	
	//if((ec->pdata->irq_trig == IRQF_TRIGGER_LOW) || (ec->pdata->irq_trig == IRQF_TRIGGER_HIGH))
	//disable_irq_nosync(irq);
	//schedule_delayed_work(&ec->delaywork, msecs_to_jiffies(ec->pdata->poll_delay_ms));
	DBG_EC("%s:irq=%d\n",__func__,irq);
	return IRQ_HANDLED;
}


static void  ec_wakeup_delaywork_func(struct work_struct *work)
{
	struct delayed_work *delaywork = container_of(work, struct delayed_work, work);
	struct ec_private_data *ec = container_of(delaywork, struct ec_private_data, delaywork);

	mutex_lock(&ec->wakeup_lock);	
	if (ec_ec_wakeup_ap(ec) < 0) 
		DBG_EC(KERN_ERR "%s: Handle data failed\n",__func__);
	
	mutex_unlock(&ec->wakeup_lock);	
}

/*
 * This is a threaded IRQ handler so can access I2C/SPI.  Since all
 * interrupts are clear on read the IRQ line will be reasserted and
 * the physical IRQ will be handled again if another interrupt is
 * asserted while we run - in the normal course of events this is a
 * rare occurrence so we save I2C/SPI reads.  We're also assuming that
 * it's rare to get lots of interrupts firing simultaneously so try to
 * minimise I/O.
 */
static irqreturn_t ec_wakeup(int irq, void *dev_id)
{
	struct ec_private_data *ec = (struct ec_private_data *)dev_id;

	//use threaded IRQ
	if (ec_ec_wakeup_ap(ec) < 0) 
		DBG_EC(KERN_ERR "%s: Get data failed\n",__func__);
	
	//if((ec->pdata->wake_trig == IRQF_TRIGGER_LOW) || (ec->pdata->wake_trig == IRQF_TRIGGER_HIGH))
	//disable_irq_nosync(irq);
	//schedule_delayed_work(&ec->delaywork, msecs_to_jiffies(ec->pdata->poll_delay_ms));
	DBG_EC("%s:irq=%d\n",__func__,irq);
	return IRQ_HANDLED;
}


static int ec_irq_init(struct ec_private_data *ec)
{
	int result = 0;
	int irq;
	if((ec->pdata->irq_enable)&&(ec->pdata->irq_trig != EC_UNKNOW_DATA))
	{
		INIT_DELAYED_WORK(&ec->delaywork, ec_interrupt_delaywork_func);
		if(ec->pdata->poll_delay_ms < 0)
			ec->pdata->poll_delay_ms = 30;
		
		result = gpio_request(ec->pdata->irq, ec->pdata->name);
		if (result)
		{
			printk("%s:fail to request gpio :%d\n",__func__,ec->pdata->irq);
		}
		
		gpio_pull_updown(ec->pdata->irq, PullEnable);
		irq = gpio_to_irq(ec->pdata->irq);
		//result = request_irq(irq, ec_interrupt, ec->pdata->irq_trig, ec->pdata->name, ec);
		result = request_threaded_irq(irq, NULL, ec_interrupt, ec->pdata->irq_trig, ec->pdata->name, ec);
		if (result) {
			printk(KERN_ERR "%s:fail to request irq = %d, ret = 0x%x\n",__func__, irq, result);	       
			goto error;	       
		}
		ec->irq = irq;
		enable_irq_wake(irq);
		printk("%s:use irq=%d\n",__func__,irq);
	}
	else if(!ec->pdata->irq_enable)
	{		
		INIT_DELAYED_WORK(&ec->delaywork, ec_interrupt_delaywork_func);
		if(ec->pdata->poll_delay_ms < 0)
			ec->pdata->poll_delay_ms = 30;
		
		schedule_delayed_work(&ec->delaywork, msecs_to_jiffies(ec->pdata->poll_delay_ms));
		printk("%s:use polling,delay=%d ms\n",__func__,ec->pdata->poll_delay_ms);
	}

	if(ec->pdata->ec_wakeup_ap)
	{
		INIT_DELAYED_WORK(&ec->delaywork, ec_wakeup_delaywork_func);
	
		gpio_pull_updown(ec->pdata->ec_wakeup_ap, PullEnable);
		irq = gpio_to_irq(ec->pdata->ec_wakeup_ap);
		//result = request_irq(irq, ec_wakeup, ec->pdata->wake_trig, ec->pdata->name, ec);
		result = request_threaded_irq(irq, NULL, ec_wakeup, ec->pdata->wake_trig, "ec_wakeup_ap", ec);
		if (result) {
			printk(KERN_ERR "%s:fail to request irq = %d, ret = 0x%x\n",__func__, irq, result);	       
			goto error;	       
		}
	
		enable_irq_wake(irq);
		printk("%s:ec_wakeup_ap=%d\n",__func__,irq);
	};

error:	
	return result;
}


#if 1
static int ec_dev_open(struct inode *inode, struct file *file)
{
	struct ec_private_data *ec = g_ec;	

	int result = 0;

	
	DBG_EC("%s\n", __func__);
	return result;
}


static int ec_dev_release(struct inode *inode, struct file *file)
{	
	struct ec_private_data *ec = g_ec;	

	int result = 0;

	
	DBG_EC("%s\n", __func__);
	return result;
}

/* ioctl - I/O control */
static long ec_dev_ioctl(struct file *file,
			  unsigned int cmd, unsigned long arg)
{
	struct ec_private_data *ec = g_ec;
	void __user *argp = (void __user *)arg;
	int key_ctrl[2] = {0, 0};
	int result = 0;


	switch (cmd) {
	case EC_IOCTL_KEYBOARD:	
	        mutex_lock(&ec->operation_mutex);		
		if (copy_from_user(key_ctrl, argp, sizeof(key_ctrl)))
		{
			result = -EFAULT;
			goto error;
		}

		if(ec->flags.key_flag != key_ctrl[0])
		{
			ec->flags.key_flag = key_ctrl[0];
			
			if(ec->ops[EC_ID_KEY] && ec->ops[EC_ID_KEY]->active)
			ec->ops[EC_ID_KEY]->active(ec, key_ctrl[0]);		
		}
		
	        mutex_unlock(&ec->operation_mutex);
		
		DBG_EC("%s:EC_IOCTL_KEYBOARD ok,key_en=%d,angle=%d\n", __func__,key_ctrl[0],key_ctrl[1]);	        
	        break;

	case EC_IOCTL_TEST:				
	        DBG_EC("%s:EC_IOCTL_TEST start\n", __func__);
	        mutex_lock(&ec->operation_mutex);		
		printk("%s:EC_IOCTL_TEST\n",__func__);
	        mutex_unlock(&ec->operation_mutex);		
	        DBG_EC("%s:EC_IOCTL_TEST ok\n", __func__);
	        break;

	default:
		result = -ENOTTY;
		goto error;
	}
	
error:
	return result;
}


static int ec_misc_init(struct ec_private_data *ec)
{	
	int result = 0;

	ec->fops.owner = THIS_MODULE;
	ec->fops.unlocked_ioctl = ec_dev_ioctl;
	ec->fops.open = ec_dev_open;
	ec->fops.release = ec_dev_release;

	ec->miscdev.minor = MISC_DYNAMIC_MINOR;
	ec->miscdev.name = "ec";
	ec->miscdev.fops = &ec->fops;

	ec->miscdev.parent = ec->dev;
	result = misc_register(&ec->miscdev);
	if (result < 0) {
		dev_err(ec->dev,
			"fail to register misc device %s\n", ec->miscdev.name);
		goto error;
	}

error:	
	return result;

}

#endif

int ec_device_init(struct ec_private_data *ec, int type, int irq)
{
	struct ec_platform_data *pdata = ec->dev->platform_data;
	struct proc_dir_entry *ec_proc_entry;
	int result = -1, i;

	if(!pdata)
		return -1;
		
	mutex_init(&ec->io_lock);
	mutex_init(&ec->ec_lock);
	mutex_init(&ec->wakeup_lock);	
	mutex_init(&ec->operation_mutex);	
	dev_set_drvdata(ec->dev, ec);
	
	ec->pdata = pdata;
	result = ec_chip_init(ec, type);
	if(result < 0)
	{
		printk("%s:ec with bus type %d is not exist\n",__func__,type);
		goto out_free_memory;
	}


	result = ec_irq_init(ec);
	if (result) {
		dev_err(ec->dev, "fail to init ec irq,ret=%d\n",result);
		goto out_free_memory;
	}

	
	result = ec_misc_init(ec);
	if (result) {
		dev_err(ec->dev, "fail to regist misc,ret=%d\n",result);
		goto out_free_memory;
	}

	wake_lock_init(&ec->ec_wakelock, WAKE_LOCK_SUSPEND, "ec_wakelock");

	ec_proc_entry = proc_create("driver/ec_dbg", 0660, NULL, &ec_proc_fops); 
	
	g_ec = ec;
	
	printk("%s:initialized ok,ec name:%s,irq=%d,miscname=/dev/%s\n\n",__func__,ec->pdata->name,ec->irq,ec->miscdev.name);

	return result;
		
out_free_memory:	
	kfree(ec);
	
	printk("%s:line=%d,error\n",__func__,__LINE__);
	return result;
}


void ec_device_exit(struct ec_private_data *ec)
{
	int i = 0; 
	int result = 0;
	
	if(!ec->pdata->irq_enable)	
	cancel_delayed_work_sync(&ec->delaywork);
	
	for(i=EC_ID_NULL+1; i<EC_NUM_ID; i++)
	{
		if(ec->ops[i] && ec->ops[i]->deinit)
		result = ec->ops[i]->deinit(ec);
		if(result)
		printk("%s:fail to deinit ec modules %s\n",__FUNCTION__, ec->ops[i]->name);
	}
	
	kfree(ec);	

	
	DBG_EC("%s\n", __func__);
}


int ec_device_suspend(struct ec_private_data *ec)
{
	int i = 0;
	int result = 0;
	
	for(i=EC_ID_NULL+1; i<EC_NUM_ID; i++)
	{
		if(ec->ops[i] && ec->ops[i]->suspend)
		result = ec->ops[i]->suspend(ec);
		if(result)
		printk("%s:fail to suspend ec modules %s\n",__FUNCTION__, ec->ops[i]->name);
	}

	if(ec->pdata->irq_enable)	
		disable_irq_nosync(ec->irq);
	else
		cancel_delayed_work_sync(&ec->delaywork);

	DBG_EC("%s\n", __func__);

	return 0;
}


int ec_device_resume(struct ec_private_data *ec)
{
	int i = 0;
	int ret = 0;
	
	for(i=EC_ID_NULL+1; i<EC_NUM_ID; i++)
	{
		if(ec->ops[i] && ec->ops[i]->resume)
		ret = ec->ops[i]->resume(ec);
		if(ret)
		printk("%s:fail to resume ec modules %s\n",__FUNCTION__, ec->ops[i]->name);
	}

	if(ec->pdata->irq_enable)	
		enable_irq(ec->irq);
	else
	{
		PREPARE_DELAYED_WORK(&ec->delaywork, ec_interrupt_delaywork_func);
		schedule_delayed_work(&ec->delaywork, msecs_to_jiffies(ec->pdata->poll_delay_ms));
	}

	ec_ap_wakeup_ec(ec);

	
	DBG_EC("%s\n", __func__);
	return 0;
}


int ec_register_slave(struct ec_private_data *ec,
			struct ec_platform_data *slave_pdata,
			struct ec_operate *(*get_ec_ops)(void))
{
	int result = 0;
	struct ec_operate *ops = get_ec_ops();
	if((ops->ec_id >= EC_NUM_ID) || (ops->ec_id <= EC_ID_NULL))
	{	
		printk("%s:%s id is error %d\n", __func__, ops->name, ops->ec_id);
		return -1;	
	}
	g_ec_ops[ops->ec_id] = ops;
	printk("%s:%s,id=%d\n",__func__,g_ec_ops[ops->ec_id]->name, ops->ec_id);
	return result;
}


int ec_unregister_slave(struct ec_private_data *ec,
			struct ec_platform_data *slave_pdata,
			struct ec_operate *(*get_ec_ops)(void))
{
	int result = 0;
	struct ec_operate *ops = get_ec_ops();
	if((ops->ec_id >= EC_NUM_ID) || (ops->ec_id <= EC_ID_NULL))
	{	
		printk("%s:%s id is error %d\n", __func__, ops->name, ops->ec_id);
		return -1;	
	}
	printk("%s:%s,id=%d\n",__func__,g_ec_ops[ops->ec_id]->name, ops->ec_id);
	g_ec_ops[ops->ec_id] = NULL;	
	return result;
}


MODULE_AUTHOR("ROCKCHIP Corporation:lw@rock-chips.com");
MODULE_DESCRIPTION("device interface for ec chip");
MODULE_LICENSE("GPL");

