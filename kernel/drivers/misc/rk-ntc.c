/* 
 *
 * Copyright (C) 2012-2015 ROCKCHIP.
 * Author:  ?<?@rock-chips.com>
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

#include <linux/module.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/adc.h>
#include <linux/wakelock.h>

#include <asm/gpio.h>
#include <mach/board.h>

#include <linux/suspend.h>

#include "rk-ntc.h"

struct rk_ntc_data{
	int adcvalue ;
	int oldadcvalue ;
	struct adc_client *adc_client ;	
	struct timer_list timer;
	int in_suspend;
	struct ntc_res_temp_tbl *ntc_res_temp_tbl;
	int table_size;
	struct adc_client       *client; 
};
struct rk_ntc_data * ntc_data;

static int voltage_to_temperature(int vol,int *temperature)
{
	int i;
	int resistance;
	struct ntc_res_temp_tbl *table;
	table = ntc_data->ntc_res_temp_tbl;

	resistance = 1000*vol/(ADC_REF_VOLTAGE - vol);
	for(i=0;i<ntc_data->table_size;i++){
		if(table[i].resistance == TAIL_OF_TABLE){
			printk("[error]%s,can't find match resistance[%d] in the table\n",__FUNCTION__,resistance);
			return -1;
		}
//		printk("table[%d].resistance=%d,table[%d].resistance=%d,resistance=%d\n",i,table[i].resistance,i+1,table[i+1].resistance,resistance);
		if(resistance <= table[i].resistance && resistance > table[i+1].resistance){
			*temperature = table[i].temperature;
			break;
		}
	}
	return 0;
}

static void ntc_adc_timer(unsigned long _data)
{
	struct rk_ntc_data *ddata = (struct rk_ntc_data *)_data;

	if (!ddata->in_suspend)
		adc_async_read(ddata->client);
	mod_timer(&ddata->timer, jiffies + msecs_to_jiffies(ADC_SAMPLE_TIME));
	return;

}
static void ntc_adc_callback(struct adc_client *client, void *client_param, int result)
{
	struct rk_ntc_data *ddata = (struct rk_ntc_data *)client_param;
	int temperature,ret;
	int vol;
	vol = result*ADC_REF_VOLTAGE/1024;
	ret  = voltage_to_temperature(vol,&temperature);
	if(ret !=0){
		temperature = 60;
		printk("[error]%s,read a invalid result[%d] voltage[%d] ,we set temperature to 60 degree\n",__FUNCTION__,result,vol);
	}
//	printk("ntc async read voltage is [%d],temperature is [%d]\n]",vol,temperature);
	
	return;
}
int get_ntc_temp()
{
	int vol,ret;
	int temperature;
	vol = adc_sync_read(ntc_data->adc_client);
	ret  = voltage_to_temperature(vol , &temperature);
	if(ret != 0){
		temperature = 60;
		printk("[error]%s,read a invalid voltage[%d] ,we set temperature to 60 degree\n",__FUNCTION__,vol);
	}
	printk("ntc sync read voltage is [%d],temperature is [%d]\n]",vol,temperature);

	return temperature;
}
static int rk_ntc_probe(struct platform_device *pdev)
{
	struct rk_ntc_platform_data *pdata = pdata = pdev->dev.platform_data;
	struct rk_ntc_data *ddata;
	int error = 0;
	
	printk("%s start\n",__FUNCTION__);

	ddata = kmalloc(sizeof(struct rk_ntc_data), GFP_KERNEL);
	if(!ddata){
		printk("rk ntc alloc memory err !!!\n");
		error = -ENOMEM;
		goto alloc_memory_fail;
	}	
	platform_set_drvdata(pdev, ddata);
	ddata->in_suspend = 0;
	if (pdata->adc_chn>= 0) {
		setup_timer(&ddata->timer, ntc_adc_timer, (unsigned long)ddata);

		ddata->client = adc_register(pdata->adc_chn, ntc_adc_callback, NULL);
		if (!ddata->client) {
			error = -EINVAL;
			goto adc_register_fail;
		}
		mod_timer(&ddata->timer, jiffies + msecs_to_jiffies(ADC_SAMPLE_TIME));
	}
	ddata->ntc_res_temp_tbl = pdata->res_to_temp_tbl;
	ddata->table_size = sizeof(pdata->res_to_temp_tbl)/sizeof(pdata->res_to_temp_tbl[0]);
	ntc_data = ddata;
	printk("%s end\n",__FUNCTION__);
	return error;
	
adc_register_fail:
        del_timer_sync(&ddata->timer);

alloc_memory_fail:
 	platform_set_drvdata(pdev, NULL);
	kfree(ddata);
	return error;
}
static int __devexit rk_ntc_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rk_ntc_platform_data *pdata = pdata = pdev->dev.platform_data;
	struct rk_ntc_data *ddata = dev_get_drvdata(dev);
	del_timer_sync(&ddata->timer);
	adc_unregister(ddata->client);
	
	platform_set_drvdata(pdev, NULL);
	kfree(ddata);

	return 0;
}


#ifdef CONFIG_PM
static int rk_ntc_suspend(struct device *dev)
{
	struct rk_ntc_platform_data *pdata = dev->platform_data;
	struct rk_ntc_data *ddata = dev_get_drvdata(dev);

	ddata->in_suspend = 1;
	return 0;
}

static int rk_ntc_resume(struct device *dev)
{
	struct rk_ntc_platform_data *pdata = dev->platform_data;
	struct rk_ntc_data *ddata = dev_get_drvdata(dev);

	ddata->in_suspend = 0;
	return 0;
}

static const struct dev_pm_ops rk_ntc_pm_ops = {
	.suspend	= rk_ntc_suspend,
	.resume		= rk_ntc_resume,
};
#endif

static struct platform_driver rk_ntc_device_driver = {
	.probe		= rk_ntc_probe,
	.remove		= __devexit_p(rk_ntc_remove),
	.driver		= {
		.name	= "rk-ntc",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &rk_ntc_pm_ops,
#endif
	}
};

static int __init rk_ntc_init(void)
{
	return platform_driver_register(&rk_ntc_device_driver);
}

static void __exit rk_ntc_exit(void)
{
	platform_driver_unregister(&rk_ntc_device_driver);
}


module_init(rk_ntc_init);
module_exit(rk_ntc_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("zwp <zwp@rock-chips.com>");
MODULE_DESCRIPTION("ntc driver for rock-chips platform");
MODULE_ALIAS("platform:rk-ntc");


