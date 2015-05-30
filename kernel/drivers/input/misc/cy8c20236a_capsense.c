 /****************************************************************************************
 * File:            driver/input/misc/cy8c20236a_capsense.c
 * Copyright:       Copyright (C) BITLAND.
 * Author:      	liuwei <liuwei3@bitland.com.cn>
 * Date:            2013.02.17
 * Description: 	This driver use for cy8c20236a capsense key. I2C addr = 0x64 (7 bits)??
 *****************************************************************************************/
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
#include <mach/gpio.h>
#include <mach/board.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#define DEBUG   0

#define CY8C20236_IRQ       "cy8c20236-irq"
#define CY8C20236_INT_PIN   RK30_PIN1_PB2
#define THRESHOLD			6

#if DEBUG
#define DBG(X...)   printk(KERN_NOTICE X)
#else
#define DBG(X...)
#endif

struct cy8c20236_data {
    struct delayed_work	cy_work;
	struct workqueue_struct *cy_workqueue;
	struct timer_list   timer;
    struct input_dev    *input;
    struct i2c_client	*client;
	int irq;
	int flag_down;
    int first_value;
	int trig_status;
};
/*
#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend cy8c20236_early_suspend;
#endif
*/
static struct cy8c20236_data *capsense;


static int cy8c20236_read_reg(struct i2c_client *client, char reg, char *value)
{
    int ret = 0;
    struct i2c_msg msg[2];
    struct i2c_adapter *adap = client->adapter;

    msg[0].addr  = client->addr;
    msg[0].flags = client->flags;
    msg[0].len = 1;
    msg[0].buf = (char *)&reg;
    msg[0].scl_rate = 400 * 1000;

    msg[1].addr  = client->addr;
    msg[1].flags = client->flags | I2C_M_RD;
    msg[1].len = 1;
    msg[1].buf = (char *)value;
    msg[1].scl_rate = 400 * 1000;

    if ((ret = i2c_transfer(adap, (struct i2c_msg *)&msg, 2)) < 2) {
        DBG("%s: read cy8c20236 register  %#x failure\n", __FUNCTION__, reg);
        return -EIO;
    }

    return 1;
}

static int cy8c20236_write_reg(struct i2c_client *client, char reg, char value)
{
    int ret = 0;
    char buf[2];
    struct i2c_msg msg;
    struct i2c_adapter *adap = client->adapter;

    buf[0] = reg;
    buf[1] = value;

    msg.addr  = client->addr;
    msg.flags = client->flags;
    msg.len = 2;
    msg.buf = (char *)&buf;
    msg.scl_rate = 400 * 1000;


    if ((ret = i2c_transfer(adap, (struct i2c_msg *)&msg, 1)) < 1) {
        DBG("%s: write cy8c20236 register  %#x failure\n", __FUNCTION__, reg);
        return -EIO;
    }

    return 1;
}

static int cy8c20236_write_cmd(struct i2c_client *client, char reg)
{
    int ret = 0;
    struct i2c_msg msg;
    struct i2c_adapter *adap = client->adapter;

    msg.addr  = client->addr;
    msg.flags = client->flags;
    msg.len = 1;
    msg.buf = (char *)&reg;
    msg.scl_rate = 400 * 1000;


    if ((ret = i2c_transfer(adap, (struct i2c_msg *)&msg, 1)) < 1) {
        DBG("%s: write cy8c20236 cmd  %#x failure\n", __FUNCTION__, reg);
        return -EIO;
    }

    return 1;
}

static void cy8c20236_timer_work(unsigned long data)
{
	struct cy8c20236_data *cy8c20236 = (struct cy8c20236_data *)data;
	char value;
	int i,step;

	DBG("%s.....%d...\n",__FUNCTION__,__LINE__);

	if(!gpio_get_value(CY8C20236_INT_PIN))
	{
		cy8c20236_read_reg(cy8c20236->client, 0x00, &value);
		if(cy8c20236->first_value == 255)
		{
    		DBG("cy8c20236 report first_value = %d\n", value);
			cy8c20236->first_value = value;
		}
		else
		{
			DBG("cy8c20236 report second_val(%d) - first_val(%d) = %d\n", value, cy8c20236->first_value, value-cy8c20236->first_value);

			if(THRESHOLD <= value - cy8c20236->first_value)
			{
				step = (value-cy8c20236->first_value)/THRESHOLD;
				for(i=0;i<step;i++)
				{
				input_event(cy8c20236->input, EV_KEY, KEY_VOLUMEUP, 1);
        		input_sync(cy8c20236->input);
				input_event(cy8c20236->input, EV_KEY, KEY_VOLUMEUP, 0);
        		input_sync(cy8c20236->input);
				}
				cy8c20236->first_value = value;
			}
			else if(THRESHOLD <= cy8c20236->first_value - value)
			{
				step = (cy8c20236->first_value - value)/THRESHOLD;
				for(i=0;i<step;i++)
				{
                input_event(cy8c20236->input, EV_KEY, KEY_VOLUMEDOWN, 1);
                input_sync(cy8c20236->input);
                input_event(cy8c20236->input, EV_KEY, KEY_VOLUMEDOWN, 0);
                input_sync(cy8c20236->input);
				}
				cy8c20236->first_value = value;
			}

		}
		mod_timer(&cy8c20236->timer, jiffies + msecs_to_jiffies(10));
	}
	else
	{
		DBG("%s.....%d...	up\n",__FUNCTION__,__LINE__);
		cy8c20236->first_value = 255;
	}
	return;
}

static irqreturn_t cy8c20236_interrupt(int irq, void *dev_id)
{
    struct cy8c20236_data *cy8c20236 = dev_id;
	int err;

    //disable_irq_nosync(cy8c20236->irq);
	DBG("%s.....%d...\n",__FUNCTION__,__LINE__);

	mod_timer(&cy8c20236->timer, jiffies + 1);

    return IRQ_HANDLED;
}



static int cy8c20236_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
    struct cy8c20236_data *cy8c20236;
    struct cy8c20236_platform_data *pdata = pdata = client->dev.platform_data;
    int err = 0;

    DBG("============= cy8c20236 probe enter ==============\n");
    cy8c20236 = kmalloc(sizeof(struct cy8c20236_data), GFP_KERNEL);
    if(!cy8c20236){
        printk("cy8c20236 alloc memory err !!!\n");
        err = -ENOMEM;
        goto alloc_memory_fail;
    }
    capsense = cy8c20236;
    cy8c20236->client = client;
	i2c_set_clientdata(client, cy8c20236);

    cy8c20236->input = input_allocate_device();
    if (!cy8c20236->input) {
        err = -ENOMEM;
        printk(KERN_ERR"cy8c20236: Failed to allocate input device\n");
        goto exit_input_allocate_device_failed;
    }

	input_set_capability(cy8c20236->input, EV_KEY, KEY_VOLUMEUP);
	input_set_capability(cy8c20236->input, EV_KEY, KEY_VOLUMEDOWN);

    cy8c20236->input->name = "cy8c20236";
    err = input_register_device(cy8c20236->input);
    if (err < 0) {
        printk(KERN_ERR"cy8c20236: Unable to register input device: %s\n",cy8c20236->input->name);
        goto exit_input_register_device_failed;
    }

	setup_timer(&cy8c20236->timer, cy8c20236_timer_work, (unsigned long)cy8c20236);
	cy8c20236->first_value = 255;
	cy8c20236->trig_status = IRQF_TRIGGER_FALLING;

	err = request_threaded_irq(client->irq, NULL, cy8c20236_interrupt, \
                       cy8c20236->trig_status, \
                       CY8C20236_IRQ, cy8c20236);

    if (err < 0) {
        dev_err(&client->dev, "irq %d busy?\n", cy8c20236->irq);
        goto exit_irq_request_fail;
    }
	enable_irq(cy8c20236->irq);

/*
#ifdef CONFIG_HAS_EARLYSUSPEND
    cy8c20236_early_suspend.suspend = cy8c20236_suspend;
    cy8c20236_early_suspend.resume = cy8c20236_resume;
    cy8c20236_early_suspend.level = 0x2;
    register_early_suspend(&cy8c20236_early_suspend);
#endif
*/
    printk("capsense-key cy8c20236 driver created !\n");
    return 0;
    free_irq(cy8c20236->irq,cy8c20236);
exit_irq_request_fail:
    cancel_work_sync(&cy8c20236->cy_work);
    destroy_workqueue(cy8c20236->cy_workqueue);
//exit_misc_register_fail:
    input_unregister_device(cy8c20236->input);
exit_input_register_device_failed:
    input_free_device(cy8c20236->input);
exit_input_allocate_device_failed:
    kfree(cy8c20236);
alloc_memory_fail:
    printk("%s error\n",__FUNCTION__);
    return err;
}

static __devexit int  cy8c20236_remove(struct i2c_client *client)
{
    struct cy8c20236_data *cy8c20236 = capsense;

    free_irq(cy8c20236->irq, cy8c20236);
    cancel_work_sync(&cy8c20236->cy_work);
    destroy_workqueue(cy8c20236->cy_workqueue);
	input_unregister_device(cy8c20236->input);
	input_free_device(cy8c20236->input);
    kfree(cy8c20236);
    return 0;
}

static const struct i2c_device_id cy8c20236_i2c_id[] = {
    { "cy8c20236", 0 },
    { }
};

static struct i2c_driver cy8c20236_driver = {
    .probe = cy8c20236_probe,
    .remove = __devexit_p(cy8c20236_remove),
    .driver = {
        .owner = THIS_MODULE,
        .name = "cy8c20236",
    },
    .id_table = cy8c20236_i2c_id,
};

static int __init cy8c20236_i2c_init(void)
{
    return i2c_add_driver(&cy8c20236_driver);
}

static void __exit cy8c20236_i2c_exit(void)
{
    i2c_del_driver(&cy8c20236_driver);
}

module_init(cy8c20236_i2c_init);
module_exit(cy8c20236_i2c_exit);
