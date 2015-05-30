/* ehci-msm.c - HSUSB Host Controller Driver Implementation
 *
 * Copyright (c) 2008-2011, Code Aurora Forum. All rights reserved.
 *
 * Partly derived from ehci-fsl.c and ehci-hcd.c
 * Copyright (c) 2000-2004 by David Brownell
 * Copyright (c) 2005 MontaVista Software
 *
 * All source code in this file is licensed under the following license except
 * where indicated.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org
 */

#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/device.h>

#include <linux/wakelock.h>
#include <mach/gpio.h>
#include "ehci.h"
#include "../dwc_otg/usbdev_rk.h"

static int rkehci_status = 1;
static int smsc_reinit_status = 0;
extern int smsc_reinit(void);
extern int dpm_resume_status;
struct delayed_work hsic_reset_work;
static struct wake_lock ehci_wakelock;

#define EHCI_DEVICE_FILE        "/sys/devices/platform/rk_hsusb_host/ehci_power"
#define HSIC_HUB_DEVICE_FILE    "/sys/devices/platform/rk_hsusb_host/usb4604_reinit"
DEFINE_MUTEX(hsic_mutex);

static struct hc_driver rk_hc_driver = {
	.description		= hcd_name,
	.product_desc		= "Rockchip On-Chip EHCI Host Controller",
	.hcd_priv_size		= sizeof(struct ehci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq			= ehci_irq,
	.flags			= HCD_USB2 | HCD_MEMORY,

	.reset			= ehci_init,
	.start			= ehci_run,

	.stop			= ehci_stop,
	.shutdown		= ehci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue		= ehci_urb_enqueue,
	.urb_dequeue		= ehci_urb_dequeue,
	.endpoint_disable	= ehci_endpoint_disable,
	.endpoint_reset		= ehci_endpoint_reset,
	.clear_tt_buffer_complete = ehci_clear_tt_buffer_complete,

	/*
	 * scheduling support
	 */
	.get_frame_number	= ehci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data	= ehci_hub_status_data,
	.hub_control		= ehci_hub_control,
	.relinquish_port	= ehci_relinquish_port,
	.port_handed_over	= ehci_port_handed_over,

	/*
	 * PM support
	 */
	.bus_suspend		= ehci_bus_suspend,
	.bus_resume		= ehci_bus_resume,
};

static ssize_t smsc_reinit_enable_show( struct device *_dev, 
					struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", smsc_reinit_status);
}

static ssize_t smsc_reinit_enable_store( struct device *_dev,
					struct device_attribute *attr, 
					const char *buf, size_t count )
{
	uint32_t val = simple_strtoul(buf, NULL, 16);

	if(val == 1){
		printk("reinit usb4604!!\n");
		smsc_reinit();
		smsc_reinit_status++;
	}
	return count;
}
static DEVICE_ATTR(usb4604_reinit, S_IRUGO|S_IWUSR, smsc_reinit_enable_show, smsc_reinit_enable_store);
static ssize_t ehci_power_show( struct device *_dev, 
					struct device_attribute *attr, char *buf) 
{
	return sprintf(buf, "%d\n", rkehci_status);
}
static ssize_t ehci_power_store( struct device *_dev,
					struct device_attribute *attr, 
					const char *buf, size_t count ) 
{
	uint32_t val = simple_strtoul(buf, NULL, 16);
	struct usb_hcd *hcd = dev_get_drvdata(_dev);
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	struct rkehci_platform_data *pldata = _dev->platform_data;

	printk("%s: %d setting to: %d\n", __func__, rkehci_status, val);
	if(val == rkehci_status)
		goto out;
	
	rkehci_status = val;
	switch(val){
		case 0: //power down
			ehci_port_power(ehci, 0);
			writel_relaxed(0 ,hcd->regs +0xb0);
			dsb();
			msleep(5);
			usb_remove_hcd(hcd);
            		break;
		case 1: // power on
			pldata->soft_reset();
          		usb_add_hcd(hcd, hcd->irq, IRQF_DISABLED | IRQF_SHARED);
        
    			ehci_port_power(ehci, 1);
    			writel_relaxed(1 ,hcd->regs +0xb0);
    			writel_relaxed(0x1d4d ,hcd->regs +0x90);
			writel_relaxed(0x4 ,hcd->regs +0xa0);
			dsb();
            		break;
		default:
            		break;
	}
out:
	return count;
}
DEVICE_ATTR(ehci_power, S_IRUGO|S_IWUSR, ehci_power_show, ehci_power_store);

static void ehci_wake_lock(void)
{
	unsigned long	flags;

	local_irq_save(flags);
	wake_lock(&ehci_wakelock);
	local_irq_restore(flags);

}

static void ehci_wake_unlock(void)
{
	unsigned long	flags;
	local_irq_save(flags);
	wake_unlock(&ehci_wakelock);
	local_irq_restore(flags);
}

static void ehci_hsic_reset_work(struct work_struct *work)
{
	struct file *filp;
	struct file *filp1;
	mm_segment_t oldfs;

	if(dpm_resume_status){
		printk("In dpm resume status, reset hsic later!\n");
		schedule_delayed_work(&hsic_reset_work, HZ * 5);
		return; 
	}
	printk("%s:**************************************\n", __func__);
	/*get wake lock,inorder to avoid entering suspend */
	ehci_wake_lock();
	oldfs = get_fs();
	set_fs(KERNEL_DS);

	filp = filp_open(EHCI_DEVICE_FILE, O_RDWR, 0);
	if (!filp || IS_ERR(filp)) {
		printk("Invalid EHCI filp=%ld\n", PTR_ERR(filp));
		set_fs(oldfs);
		return;
	}else{
		filp->f_op->write(filp, "0", 1, &filp->f_pos);/*disable ehci*/
		filp->f_op->write(filp, "1", 1, &filp->f_pos);/*re-enable ehci*/
		filp_close(filp, NULL);
	}

	filp1 = filp_open(HSIC_HUB_DEVICE_FILE, O_RDWR, 0); 
	if (!filp1 || IS_ERR(filp1)) {
		printk("Invalid HSIC HUB filp=%ld\n", PTR_ERR(filp1));
		set_fs(oldfs);
		return;
	}else{
		filp1->f_op->write(filp1, "1", 1, &filp1->f_pos);
		filp_close(filp1, NULL);
	}

	set_fs(oldfs);
	/*release wake lock*/
	ehci_wake_unlock();
	mutex_unlock(&hsic_mutex);
	printk("%s: release mutex!\n", __func__);
}

static int ehci_rk_probe(struct platform_device *pdev)
{
	struct usb_hcd *hcd;
	struct ehci_hcd *ehci;
	struct resource *res;
	struct device *dev = &pdev->dev;
	struct rkehci_platform_data *pldata = dev->platform_data;
	int ret;
	int retval = 0;
	static u64 usb_dmamask = 0xffffffffUL;

	dev_dbg(&pdev->dev, "ehci_rk proble\n");
	
	dev->dma_mask = &usb_dmamask;

	retval = device_create_file(dev, &dev_attr_usb4604_reinit);
	retval = device_create_file(dev, &dev_attr_ehci_power);
	hcd = usb_create_hcd(&rk_hc_driver, &pdev->dev, dev_name(&pdev->dev));
	if (!hcd) {
		dev_err(&pdev->dev, "Unable to create HCD\n");
		return  -ENOMEM;
	}

	pldata->hw_init();
	pldata->clock_init(pldata);
	pldata->clock_enable(pldata, 1);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Unable to get memory resource\n");
		ret = -ENODEV;
		goto put_hcd;
	}
	hcd->rsrc_start = res->start;
	hcd->rsrc_len = resource_size(res);
	hcd->regs = ioremap(hcd->rsrc_start, hcd->rsrc_len);
	if (!hcd->regs) {
		dev_err(&pdev->dev, "ioremap failed\n");
		ret = -ENOMEM;
		goto put_hcd;
	}
	
	hcd->irq = platform_get_irq(pdev, 0);
	if (hcd->irq < 0) {
		dev_err(&pdev->dev, "Unable to get IRQ resource\n");
		ret = hcd->irq;
		goto put_hcd;
	}
	
    ehci = hcd_to_ehci(hcd);
    ehci->caps = hcd->regs;
    ehci->regs = hcd->regs + 0x10;
    printk("%s %p %p\n", __func__, ehci->caps, ehci->regs);
    
    dbg_hcs_params(ehci, "reset");
    dbg_hcc_params(ehci, "reset");

    ehci->hcs_params = readl(&ehci->caps->hcs_params);

	ret = usb_add_hcd(hcd, hcd->irq, IRQF_DISABLED | IRQF_SHARED);
    if (ret) {
        dev_err(&pdev->dev, "Failed to add USB HCD\n");
          goto unmap;
    }

	ehci_port_power(ehci, 1);
	writel_relaxed(1 ,hcd->regs +0xb0);
	writel_relaxed(0x1d4d ,hcd->regs +0x90);
	writel_relaxed(0x4 ,hcd->regs +0xa0);
	dsb();

	INIT_DELAYED_WORK(&hsic_reset_work, ehci_hsic_reset_work);
	wake_lock_init(&ehci_wakelock, WAKE_LOCK_SUSPEND,"ehci_reset");

	printk("%s ok\n", __func__);

	return 0;

unmap:
	iounmap(hcd->regs);
put_hcd:
	usb_put_hcd(hcd);

	return ret;
}

static int __devexit ehci_rk_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	usb_put_hcd(hcd);

	return 0;
}

#ifdef CONFIG_PM
static int ehci_rk_pm_suspend(struct device *dev)
{
	struct usb_hcd *hcd = dev_get_drvdata(dev);
	bool wakeup = device_may_wakeup(dev);

	dev_dbg(dev, "ehci-rk PM suspend\n");

	/*
	 * EHCI helper function has also the same check before manipulating
	 * port wakeup flags.  We do check here the same condition before
	 * calling the same helper function to avoid bringing hardware
	 * from Low power mode when there is no need for adjusting port
	 * wakeup flags.
	 */
	if (hcd->self.root_hub->do_remote_wakeup && !wakeup) {
		pm_runtime_resume(dev);
		ehci_prepare_ports_for_controller_suspend(hcd_to_ehci(hcd),
				wakeup);
	}

	return 0;
}

static int ehci_rk_pm_resume(struct device *dev)
{
	struct usb_hcd *hcd = dev_get_drvdata(dev);

	dev_dbg(dev, "ehci-rk PM resume\n");
	ehci_prepare_ports_for_controller_resume(hcd_to_ehci(hcd));

	return 0;
}
#else
#define ehci_rk_pm_suspend	NULL
#define ehci_rk_pm_resume	NULL
#endif

static const struct dev_pm_ops ehci_rk_dev_pm_ops = {
	.suspend         = ehci_rk_pm_suspend,
	.resume          = ehci_rk_pm_resume,
};

static struct platform_driver ehci_rk_driver = {
	.probe	= ehci_rk_probe,
	.remove	= __devexit_p(ehci_rk_remove),
	.driver = {
		   .name = "rk_hsusb_host",
		   .pm = &ehci_rk_dev_pm_ops,
	},
};

