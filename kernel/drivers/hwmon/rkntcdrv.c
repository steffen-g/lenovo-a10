/*
 * HWMON interface for rk_ntc from rockchip. Tested with RK3188 on
 * lenovo A10 running linux by Steffen Graf - S.Graf@GSG-Elektronik.de
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include "../../../drivers/misc/rk-ntc.h"

struct device 		*hwmon_dev;
#ifdef CONFIG_PM
static int ntc_suspend(struct device *dev)
{
	return 0;
}

static int ntc_resume(struct device *dev)
{
	return 0;
}


static const struct dev_pm_ops ntc_pm_ops = {
	.suspend	= ntc_suspend,
	.resume		= ntc_resume,
};
#endif

static ssize_t show_temp(struct device *dev, struct device_attribute *da,
			 char *buf)
{
	return sprintf(buf, "%d\n", get_ntc_temp());
}

static ssize_t show_temp_raw(struct device *dev, struct device_attribute *da,
			 char *buf)
{
	return sprintf(buf, "%d\n", get_ntc_temp_raw());
}

static SENSOR_DEVICE_ATTR(temp1_input, S_IRUGO, show_temp, NULL, 0);
static SENSOR_DEVICE_ATTR(in1_input, S_IRUGO, show_temp_raw, NULL, 0);

static struct attribute *ntc_attributes[] = {
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	&sensor_dev_attr_in1_input.dev_attr.attr,
	NULL
};

static const struct attribute_group ntc_group = {
	.attrs = ntc_attributes,
};

/*-----------------------------------------------------------------------*/

/* device probe and removal */

static int __devinit ntc_probe(struct platform_device *dev)
{
	int status;
	printk(KERN_INFO "rkntcdrv %s called\n", __FUNCTION__);
	//struct device *dev = &pdev->dev;
	
	/* Register sysfs hooks */
	status = sysfs_create_group(&dev->dev.kobj, &ntc_group);
	if (status)
		return status;

	hwmon_dev = hwmon_device_register(&dev->dev);
	if (IS_ERR(hwmon_dev)) {
		status = PTR_ERR(hwmon_dev);
		goto exit_remove;
	}
	
	return 0;

exit_remove:
	sysfs_remove_group(&dev->dev.kobj, &ntc_group);

	return status;
}

static int __devexit ntc_remove(struct platform_device *dev)
{
	printk(KERN_INFO "rkntcdrv %s called\n", __FUNCTION__);
	hwmon_device_unregister(hwmon_dev);
	sysfs_remove_group(&dev->dev.kobj, &ntc_group);
	return 0;
}


static struct platform_driver ntc_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "rkntcdrv",
#ifdef CONFIG_PM
		.pm	= &ntc_pm_ops,
#endif
	},
	.probe	= ntc_probe,
	.remove	= __devexit_p(ntc_remove),
};

static int __init ntc_init(void)
{
	printk(KERN_INFO "rkntcdrv %s called\n", __FUNCTION__);
	return platform_driver_register(&ntc_driver);
}

static void __exit ntc_exit(void)
{
	printk(KERN_INFO "rkntcdrv %s called\n", __FUNCTION__);
	platform_driver_unregister(&ntc_driver);
}

MODULE_AUTHOR("Steffen Graf S.Graf@gsg-elektronik.de");
MODULE_DESCRIPTION("RK NTC HWmon driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rkntcdrv");

module_init(ntc_init);
module_exit(ntc_exit);
