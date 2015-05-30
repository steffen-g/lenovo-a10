/*
 * BlueBerry battery driver based on EC
 *
 * Copyright (C) 2013-2013 Paul Ma <magf@bitland.com.cn>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * Trademarks are the property of their respective owners.
 */

#define pr_fmt(fmt) KBUILD_BASENAME ": " fmt

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
#include <linux/adc.h>
#include <linux/reboot.h>
#include "blueberry_batfwi2c.h"
#include "blueberry_firmware.h"
#include "blueberry_battery.h"

static enum power_supply_property blueberry_bat_props[] = {
    POWER_SUPPLY_PROP_STATUS,
    POWER_SUPPLY_PROP_PRESENT,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
    POWER_SUPPLY_PROP_CURRENT_NOW,
    POWER_SUPPLY_PROP_CAPACITY,
    POWER_SUPPLY_PROP_TEMP,
#ifdef CONFIG_REPORT_FULL_DATA_ITEM
    POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
    POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
    POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
#endif
    POWER_SUPPLY_PROP_TECHNOLOGY,
#ifdef CONFIG_REPORT_FULL_DATA_ITEM
    POWER_SUPPLY_PROP_CHARGE_FULL,
    POWER_SUPPLY_PROP_CHARGE_NOW,
    POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
    POWER_SUPPLY_PROP_CYCLE_COUNT,
    POWER_SUPPLY_PROP_ENERGY_NOW,
#endif
	POWER_SUPPLY_PROP_HEALTH,										
};

static enum power_supply_property blueberry_ac_props[] = {
    POWER_SUPPLY_PROP_ONLINE,
};

#define blueberry_bat_debug(fmt, ...)                        \
    do {                                    	             \
        if (bbbat->debug)                                    \
            pr_info(pr_fmt(fmt), ##__VA_ARGS__);             \
    } while (0)

/*
 * read generic registers (do not need convertion as time)
 * return < 0 if failed.
 */
static int blueberry_bat_read_register(struct blueberry_bat *bbbat, 
					unsigned char reg_addr, int *value, unsigned char *prompt)
{
	int ret;
	unsigned char buf[2];
	
	ret = blueberry_bat_smbus_read_byte_block(bbbat->client, regBatBase + reg_addr, buf, 2);
	if(ret) {
		pr_err("read %s failed\n", prompt);
		*value = -1;
		return ret;
	}
	*value = (buf[0] << 8) | buf[1];
    blueberry_bat_debug("read buf[0]=0x%x, buf[1]=0x%x, %s=0x%x\n", buf[0], buf[1], prompt, *value);

	return ret;
}

#ifdef CONFIG_REPORT_FULL_DATA_ITEM
/*	
 * read time, need convert from minute to second.
 */
static int blueberry_bat_read_time(struct blueberry_bat *bbbat, 
						unsigned char reg_addr, int *time, unsigned char *prompt)
{
    int ret;
    unsigned char buf[2];

    ret = blueberry_bat_smbus_read_byte_block(bbbat->client, regBatBase + reg_addr, buf, 2);
    if(ret) {
		pr_err("read %s failed.\n", prompt);
		*time = -1;
        return ret;
    }

	*time = (buf[0] << 8) | buf[1];
    if(*time == 65535) {
        blueberry_bat_debug("read %s is 65535.\n", prompt);
		*time = -1; 	/* or *time = -ENODATA; */
        return -ENODATA;
    }

    blueberry_bat_debug("read buf[0]=0x%x, buf[1]=0x%x, %s=0x%x\n", buf[0], buf[1], prompt, *time);
    *time = (*time) * 60;
    
	return ret;
}
#endif

static void blueberry_bat_update(struct blueberry_bat *bbbat)
{
	power_supply_changed(&bbbat->bat);
}

static void blueberry_bat_work(struct work_struct *work)
{
	struct blueberry_bat *bbbat = container_of(work, struct blueberry_bat, work.work);

	blueberry_bat_debug("blueberry_bat_poll called.\n");

	blueberry_bat_update(bbbat);
	schedule_delayed_work(&bbbat->work, bbbat->interval);
}

#ifdef CONFIG_CHARGER_INPUT_SELECT
extern int dwc_otg_check_dpdm(void);
extern int dwc_vbus_status(void);

static int blueberry_bat_charger_input_select(struct blueberry_bat *bbbat, int command)
{
	int retry = 3;
	int ret;

	if(atomic_read(&bbbat->working_mode) != NORMAL_MODE) {
		pr_err("Now in firmware update mode, do not report change until it finish.\n");
		return -1;
	}

	do{
    	mutex_lock(&bbbat->xfer_lock);
    	ret = blueberry_bat_smbus_write_byte_no_data(bbbat->client, command);	
    	if(ret) {
			pr_err("Tell ec input changed failed, retry = %d\n", retry);
        	mutex_unlock(&bbbat->xfer_lock);
        	continue;
    	}else{
			pr_err("Tell ec input changed succ. retry = %d\n", retry);
			mutex_unlock(&bbbat->xfer_lock);
			break;
		}		
	}while(retry--);

	return ret;	
}

static void blueberry_bat_usb_ac_detect_work(struct work_struct *work)
{
	int ret;
	int pre;
    struct blueberry_bat *bbbat = container_of(work, struct blueberry_bat, usb_ac_detect_work.work);

    if(2 == /*dwc_otg_check_dpdm()) { */dwc_vbus_status()) {
        /* standard adapter has connected, so set big current */
		pre = atomic_read(&bbbat->connect);
		if(atomic_read(&bbbat->connect) != CONNECT_ADAPTER) {
			/* tell ec that ADAPTER is connected */
			ret = blueberry_bat_charger_input_select(bbbat, CONNECT_ADAPTER_COMMAND);
			if(!ret) {
				atomic_set(&bbbat->connect, CONNECT_ADAPTER);
			}else{
				pr_err("tell ec charger input change failed.\n");
			}
			pr_err("pre = %d, now = %d\n", pre, atomic_read(&bbbat->connect));
		}
    }else if(1 == /*dwc_otg_check_dpdm()) { */dwc_vbus_status()) {
        /* none standard adapter or usb host has connected, set small current */
		pre = atomic_read(&bbbat->connect);
		if(atomic_read(&bbbat->connect) != CONNECT_USB) {
			ret = blueberry_bat_charger_input_select(bbbat, CONNECT_USB_COMMAND);
			if(!ret) {
				atomic_set(&bbbat->connect, CONNECT_USB);
			}else{
				pr_err("tell ec charger input change failed.\n");
			}
			pr_err("pre = %d, now = %d\n", pre, atomic_read(&bbbat->connect));
		}
    }else if(0 == /*dwc_otg_check_dpdm()) { */dwc_vbus_status()) {
        /* no adapter or usb host connected, set small current */
		//pr_err("only for test, dwc_vbus_status return 0.\n");
		pre = atomic_read(&bbbat->connect);
		if(atomic_read(&bbbat->connect) != CONNECT_NONE) {
			atomic_set(&bbbat->connect, CONNECT_NONE);
			pr_err("pre = %d, now = %d\n", pre, atomic_read(&bbbat->connect));
		}
    }

    queue_delayed_work(bbbat->usb_ac_detect_workqueue, &bbbat->usb_ac_detect_work, bbbat->usb_ac_detect_interval);
}
#endif

static irqreturn_t blueberry_bat_dcdet_wakeup(int irq, void *dev_id)
{
	struct blueberry_bat *bbbat = (struct blueberry_bat *)dev_id;

	/* when enter isr, this irq has been disabled by system */
	schedule_work(&bbbat->dcwakeup_work);

	return IRQ_HANDLED;
}

static void blueberry_bat_dcdet_work(struct work_struct *work)
{
	int ret;
	struct blueberry_bat *bbbat;
	int irq;
	int irq_flag;

	bbbat = (struct blueberry_bat *)container_of(work, struct blueberry_bat, dcwakeup_work);

	irq = gpio_to_irq(bbbat->dc_det_pin);
	irq_flag = gpio_get_value(bbbat->dc_det_pin) ? IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING;

	blueberry_bat_debug("now call rk28_send_wakeup_key, may be wrong.\n");
	rk28_send_wakeup_key(); 	/* wake up the system, do we need this? */
	
	free_irq(irq, bbbat);
	ret = request_irq(irq, blueberry_bat_dcdet_wakeup, irq_flag, "dc_det_irq", bbbat);
	if(ret) {
		blueberry_bat_debug("can't request irq in dcdet_work.\n");
	}

#ifdef CONFIG_CHARGER_INPUT_SELECT
    /* when ac plug/unplug, we do a detect immediately */
    cancel_delayed_work(&bbbat->usb_ac_detect_work);
    queue_delayed_work(bbbat->usb_ac_detect_workqueue, &bbbat->usb_ac_detect_work, 0);
#endif

	power_supply_changed(&bbbat->ac);

	/* when ac state changed, bat should also be notified to make it update quickly. */
	/* power_supply_changed(&bbbat->bat); */

	return;
}	

static irqreturn_t blueberry_bat_lowpower_wakeup(int irq, void *dev_id)
{
	struct blueberry_bat *bbbat = (struct blueberry_bat *)dev_id;

	/* when enter isr, interrupt has been disabled by system. */
	schedule_work(&bbbat->lowpower_work);

	return IRQ_HANDLED;
}

static void blueberry_bat_lowpower_work(struct work_struct *work)
{
	struct blueberry_bat *bbbat;
	int irq;
	
	bbbat = (struct blueberry_bat *)container_of(work, struct blueberry_bat, lowpower_work);

	blueberry_bat_debug("now call rk28_send_wakeup_key, maybe wrong.\n");
	rk28_send_wakeup_key();     /* wake up the system, do we need this? */

	power_supply_changed(&bbbat->bat);

    irq = gpio_to_irq(bbbat->bat_low_det_pin);
    enable_irq(irq);

	return;
}

#ifdef CONFIG_PM
static int blueberry_bat_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct blueberry_bat *bbbat;
	
	bbbat = i2c_get_clientdata(client);

#ifdef CONFIG_CHARGER_INPUT_SELECT
    cancel_delayed_work(&bbbat->usb_ac_detect_work);
#endif

	blueberry_bat_debug("enter blueberry_bat_suspend.\n");
	cancel_delayed_work(&bbbat->work);

	if((bbbat->dc_det_pin != INVALID_GPIO) && device_may_wakeup(&client->dev)) {
		enable_irq_wake(gpio_to_irq(bbbat->dc_det_pin));
	}

	if((bbbat->bat_low_det_pin != INVALID_GPIO) && device_may_wakeup(&client->dev)) {
		enable_irq_wake(gpio_to_irq(bbbat->bat_low_det_pin));
	}

	return 0;
}

static int blueberry_bat_resume(struct i2c_client *client)
{
    struct blueberry_bat *bbbat;

    bbbat = i2c_get_clientdata(client);

	blueberry_bat_debug("enter blueberry_bat_resume.\n");
	schedule_delayed_work(&bbbat->work, bbbat->interval);

    if((bbbat->dc_det_pin != INVALID_GPIO) && device_may_wakeup(&client->dev)) {
        disable_irq_wake(gpio_to_irq(bbbat->dc_det_pin));
    }

    if((bbbat->bat_low_det_pin != INVALID_GPIO) && device_may_wakeup(&client->dev)) {
        disable_irq_wake(gpio_to_irq(bbbat->bat_low_det_pin));
    }

#ifdef CONFIG_CHARGER_INPUT_SELECT
    /* when power up, we do a detect immediately */
    queue_delayed_work(bbbat->usb_ac_detect_workqueue, &bbbat->usb_ac_detect_work, 0);
#endif
	
	return 0;
}
#endif

/*
 * Return the battery temperature in tenths of degree Celsius
 * Or < 0 if something fails.
 */
static int blueberry_bat_temperature(struct blueberry_bat *bbbat,
    union power_supply_propval *val)
{
	int ret;
	int temperature;

	ret = blueberry_bat_read_register(bbbat, regBatTemperature, &temperature, "temperature");
	if(ret) 
		return ret;
	
	val->intval = temperature - 2731;
	blueberry_bat_debug("get property return temperature=0x%x\n", val->intval);

	if(val->intval > 680) {
		pr_err("read back battery temperature(%d) too high, it is out of range.\n", val->intval);
	}

	bbbat->cache.temperature = val->intval;

    return 0;
}

/*
 * Return the battery average current in µA
 * Note that current can be negative signed as well
 * Or 0 if something fails.
 */
static int blueberry_bat_current(struct blueberry_bat *bbbat,
    union power_supply_propval *val)
{
    int curr;
	int ret;

	ret = blueberry_bat_read_register(bbbat, regBatAverageCurrent, &curr, "average current");
	if(ret < 0)
		return ret;

	val->intval = (int)((s16)curr) * 1000;

	if((val->intval > 2000000) ||(val->intval < -2000000)) {
		pr_err("read back current %d (uA), it's an out of range value.\n", val->intval);
	}

	bbbat->cache.current_now = val->intval; 

	return 0;
}

static int blueberry_bat_status(struct blueberry_bat *bbbat,
    union power_supply_propval *val)
{
	int ret;
    int status;
	int flags;
	int rsoc;

	ret = blueberry_bat_read_register(bbbat, regBatFlags, &flags, "flags");
	if(ret) {
		pr_err("blueberry_bat: read flags in battery_bat_status failed.\n");
		val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		return ret;
	}

	ret = blueberry_bat_read_register(bbbat, regBatStateOfCharge, &rsoc, "rsoc");
	if(ret) {
		pr_err("blueberry_bat: can't read rsoc in blueberry_bat_status.\n");
	}

	if((flags & BLUEBERRY_BAT_FLAG_FC) && (rsoc == 0x64) && !gpio_get_value(bbbat->dc_det_pin)) {
		status = POWER_SUPPLY_STATUS_FULL;
	}else if((flags & BLUEBERRY_BAT_FLAG_FC) && (rsoc == 0x64) && gpio_get_value(bbbat->dc_det_pin)) {
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	}else if((flags & BLUEBERRY_BAT_FLAG_FC) && (rsoc < 0x64) && !gpio_get_value(bbbat->dc_det_pin)) {
		status = POWER_SUPPLY_STATUS_CHARGING;
	}else if(!(flags & BLUEBERRY_BAT_FLAG_FC) && !gpio_get_value(bbbat->dc_det_pin)) {
		status = POWER_SUPPLY_STATUS_CHARGING;
	}else{
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	}

	/* rules:
		Blueberry battery ask for not being charged if FC set and not below 95% capacity.
		so there are the following:
		1. if FC set and dc is present, then display fully-charged, whether its cpacity is over 95% or not.
		2. if FC not set and dc is present, then report charging.
		3. if DSG set and dc is not present, then report not charging.
		4. dc is not present, then report not charging.
	*/
#if 0	
	if((flags & BLUEBERRY_BAT_FLAG_FC) && !gpio_get_value(bbbat->dc_det_pin)) {				/* rules 1. */
		status = POWER_SUPPLY_STATUS_FULL;													
	}else if((!(flags & BLUEBERRY_BAT_FLAG_FC)) && !gpio_get_value(bbbat->dc_det_pin)) {	/* rules 2. */
		status = POWER_SUPPLY_STATUS_CHARGING;					
	}else if(gpio_get_value(bbbat->dc_det_pin) || (flags & BLUEBERRY_BAT_FLAG_DSC)) {		/* rules 3. and rules 4. */
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	}else
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;
#endif

    val->intval = status;

	bbbat->cache.status = val->intval;

    return 0;
}

/*
 * Return the battery Voltage in uV
 * Or < 0 if something fails.
 */
static int blueberry_bat_voltage(struct blueberry_bat *bbbat,
    union power_supply_propval *val)
{
    int volt;
	int ret;

	ret = blueberry_bat_read_register(bbbat, regBatVoltage, &volt, "voltage");
	if(ret)
		return ret;

	volt = volt * 1000;
	val->intval = volt;

	bbbat->cache.voltage = val->intval;
	
	return 0;
}

static int blueberry_bat_health(struct blueberry_bat *bbbat,
                  union power_supply_propval *val)
{
    int ret;
    int health;
    int flags;

    ret = blueberry_bat_read_register(bbbat, regBatFlags, &flags, "flags");
    if(ret)
        return ret;

	if((flags & BLUEBERRY_FLAG_OTD) || (flags & BLUEBERRY_FLAG_OTC)) {
		blueberry_bat_debug("battery is over temperature in flags.\n");
		health = POWER_SUPPLY_HEALTH_OVERHEAT;
	}else{
		blueberry_bat_debug("battery health is good in flags.\n");
		health = POWER_SUPPLY_HEALTH_GOOD;
	}

    val->intval = health;

	bbbat->cache.health = val->intval;

    return 0;
}

static int blueberry_bat_present(struct blueberry_bat *bbbat,
				union power_supply_propval *val)
{
#ifdef CONFIG_BAT_ADC_DET
    int present;
    int adc_value;

    adc_value = adc_sync_read(bbbat->adclient);
    blueberry_bat_debug("blueberry_bat_present: adc_value = %d\n", adc_value);

    if(adc_value < 400)
        present = 0;
    else
        present = 1;

    blueberry_bat_debug("blueberry_bat_present: present = %d\n", present);
#else
	int ret;
	int state = 0;
	int present;

	ret = blueberry_bat_read_register(bbbat, regBatACDCState, &state, "ACDCState");
	if(ret) {
		blueberry_bat_debug("can't read ACDCState, default is not present.\n");	
		val->intval = 0;
		return 0;	
	}

	if(state & BLUEBERRY_DC_PRESENT) {
		blueberry_bat_debug("DC is present.\n");
		present = 1;
	}else{
		blueberry_bat_debug("DC is not present.\n");
		present = 0;
	}
#endif
	val->intval = present;

	bbbat->cache.present = val->intval;

	return 0;
}

static int blueberry_bat_capacity(struct blueberry_bat *bbbat,
				union power_supply_propval *val)
{
	int ret;
	int rsoc;

	ret = blueberry_bat_read_register(bbbat, regBatStateOfCharge, &rsoc, "rsoc");
	if(ret)
		return ret;
	
	val->intval = rsoc;

	bbbat->cache.capacity = val->intval;

	return ret;
}

#ifdef CONFIG_REPORT_FULL_DATA_ITEM
/*
 * Return the battery Available energy in µWh
 * Or < 0 if something fails.
 */
static int blueberry_bat_energy(struct blueberry_bat *bbbat,
    union power_supply_propval *val)
{
    int ae;
    int ret;
    ret = blueberry_bat_read_register(bbbat, regBatAvailableEnergy, &ae, "available energy");
    if(ret)
        return ret;

    ae = ae * 1000;
    val->intval = ae;

    bbbat->cache.energy = val->intval;

    return 0;
}

static int blueberry_bat_tte(struct blueberry_bat *bbbat,
				union power_supply_propval *val)
{
	int ret;
	int tte;
	
	ret = blueberry_bat_read_time(bbbat, regBatTimeToEmpty, &tte, "tte");
	if(ret)
		return ret;
	
	val->intval = tte;

	bbbat->cache.tte = val->intval;
	
	return ret;
}

static int blueberry_bat_ttecp(struct blueberry_bat *bbbat,
                union power_supply_propval *val)
{
    int ret;
    int ttecp;

    ret = blueberry_bat_read_time(bbbat, regBatTTEatConstantPower, &ttecp, "ttecp");
    if(ret)
        return ret;

    val->intval = ttecp;

	bbbat->cache.ttecp = val->intval;

    return ret;
}

static int blueberry_bat_ttf(struct blueberry_bat *bbbat,
                union power_supply_propval *val)
{
    int ret;
    int ttf;

    ret = blueberry_bat_read_time(bbbat, regBatTimeToFull, &ttf, "ttf");
    if(ret)
        return ret;

    val->intval = ttf;

	bbbat->cache.ttf = val->intval;

    return ret;
}

static int blueberry_bat_nac(struct blueberry_bat *bbbat,
				union power_supply_propval *val)
{
	int ret;
	int nac;
	
	ret = blueberry_bat_read_register(bbbat, regBatNominalAvailableCapacity, &nac, "nac");
	if(ret) 
		return ret;

	val->intval = nac;

	bbbat->cache.nac = val->intval;
	
	return ret;
}

static int blueberry_bat_lmd(struct blueberry_bat *bbbat,
					union power_supply_propval *val)
{
	int ret;
	int lmd;

	ret = blueberry_bat_read_register(bbbat, regBatFullChargeCapacity, &lmd, "lmd");
	if(ret)
		return ret;

	val->intval = lmd;

	bbbat->cache.lmd = val->intval;

	return ret;
}

static int blueberry_bat_ilmd(struct blueberry_bat *bbbat,
					union power_supply_propval *val)
{
	int ret;
	int ilmd;
	
	ret = blueberry_bat_read_register(bbbat, regBatDesignCapacity, &ilmd, "ilmd");
	if(ret)
		return ret;

	val->intval = ilmd;

	bbbat->cache.ilmd = val->intval;

	return ret;
}

static int blueberry_bat_cyct(struct blueberry_bat *bbbat,
					union power_supply_propval *val)
{
	int ret;
	int cyct;

	ret = blueberry_bat_read_register(bbbat, regBatCycleCount, &cyct, "cycle count");
	if(ret)
		return ret;
	
	val->intval = cyct;

	bbbat->cache.cyct = val->intval;

	return ret;
}
#endif

/* 0: not present, 1: present */
static int blueberry_bat_connected(struct blueberry_bat *bbbat)
{
#ifdef CONFIG_BAT_ADC_DET
	int connected;
	int adc_value;

	adc_value = adc_sync_read(bbbat->adclient);
	blueberry_bat_debug("blueberry_bat_adc_det: adc_value = %d\n", adc_value);

	if(adc_value < 400)
		connected = 0;
	else
		connected = 1;

	blueberry_bat_debug("blueberry_bat_adc_det: connected = %d\n", connected);
#else
    int ret;
    int state;
    int connected;

    ret = blueberry_bat_read_register(bbbat, regBatACDCState, &state, "ACDCState");
    if(ret) {
		blueberry_bat_debug("read ACDCState failed, return not connected.\n");
		return 0;
    }

    if(state & BLUEBERRY_DC_PRESENT) {
        blueberry_bat_debug("DC is present.\n");
        connected = 1;
    }else{
        blueberry_bat_debug("DC is not present.\n");
        connected = 0;
    }
#endif
    return connected;
}

static int blueberry_bat_virt_report(struct blueberry_bat *bbbat, 
						enum power_supply_property psp, union power_supply_propval *val)
{
    switch (psp) {
    case POWER_SUPPLY_PROP_STATUS:
        val->intval = bbbat->cache.status; 
        break;
    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
        val->intval = bbbat->cache.voltage;
        break;
    case POWER_SUPPLY_PROP_PRESENT:
        val->intval = bbbat->cache.present;
        break;
    case POWER_SUPPLY_PROP_CURRENT_NOW:
        val->intval = bbbat->cache.current_now;
        break;
    case POWER_SUPPLY_PROP_CAPACITY:                       
        val->intval = bbbat->cache.capacity;
        break;
    case POWER_SUPPLY_PROP_TEMP:
        val->intval = bbbat->cache.temperature;
        break;
#ifdef CONFIG_REPORT_FULL_DATA_ITEM
    case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
        val->intval = bbbat->cache.tte;
        break;
    case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
        val->intval = bbbat->cache.ttecp;
        break;
    case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
        val->intval = bbbat->cache.ttf;
        break;
#endif
    case POWER_SUPPLY_PROP_TECHNOLOGY:
        val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
        break;
#ifdef CONFIG_REPORT_FULL_DATA_ITEM
    case POWER_SUPPLY_PROP_CHARGE_NOW:
        val->intval = bbbat->cache.nac;
        break;
    case POWER_SUPPLY_PROP_CHARGE_FULL:
        val->intval = bbbat->cache.lmd;
        break;
    case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
        val->intval = bbbat->cache.ilmd;
        break;
    case POWER_SUPPLY_PROP_CYCLE_COUNT:
        val->intval = bbbat->cache.cyct;
        break;
    case POWER_SUPPLY_PROP_ENERGY_NOW:
        val->intval = bbbat->cache.energy;
        break;
#endif
    case POWER_SUPPLY_PROP_HEALTH:
        val->intval = bbbat->cache.health;
        break;
    default:
        return -EINVAL;
    }

	return 0;
}

#define to_blueberry_bat_by_bat(x) container_of((x), \
                struct blueberry_bat, bat);

static int blueberry_bat_get_property(struct power_supply *psy,
                    enum power_supply_property psp,
                    union power_supply_propval *val)
{
    int ret = 0;
	int bat_present;
    struct blueberry_bat *bbbat = to_blueberry_bat_by_bat(psy);

	/* we need place mutex_lock here rather than next one 
	   because if we have got mutex, but working_mode has
	   been changed, then i2c read will fail one time, to
	   avoid that, put mutex_lock here 
	*/
	mutex_lock(&bbbat->xfer_lock);

	if(atomic_read(&bbbat->working_mode) == FLASH_MODE) {
		ret = blueberry_bat_virt_report(bbbat, psp, val);
		mutex_unlock(&bbbat->xfer_lock);
		return ret;
	}

    /* mutex_lock(&bbbat->xfer_lock); */

	bat_present = blueberry_bat_connected(bbbat);
	if((!bat_present) && (psp != POWER_SUPPLY_PROP_PRESENT)) {
		mutex_unlock(&bbbat->xfer_lock);
		return -ENODEV;
	}

    switch (psp) {
    case POWER_SUPPLY_PROP_STATUS:
        ret = blueberry_bat_status(bbbat, val);
        break;
    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
        ret = blueberry_bat_voltage(bbbat, val);
        break;
    case POWER_SUPPLY_PROP_PRESENT:
        ret = blueberry_bat_present(bbbat, val);
        break;
    case POWER_SUPPLY_PROP_CURRENT_NOW:
        ret = blueberry_bat_current(bbbat, val);
        break;
    case POWER_SUPPLY_PROP_CAPACITY:						/* do we need use voltage < 3.4 to get a 0 capacity, it seems not */
        ret = blueberry_bat_capacity(bbbat, val);
        break;
    case POWER_SUPPLY_PROP_TEMP:
        ret = blueberry_bat_temperature(bbbat, val);
        break;
#ifdef CONFIG_REPORT_FULL_DATA_ITEM
    case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
        ret = blueberry_bat_tte(bbbat, val);
        break;
    case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
        ret = blueberry_bat_ttecp(bbbat, val);
        break;
    case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
        ret = blueberry_bat_ttf(bbbat, val);
        break;
#endif
    case POWER_SUPPLY_PROP_TECHNOLOGY:
        val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
        break;
#ifdef CONFIG_REPORT_FULL_DATA_ITEM
    case POWER_SUPPLY_PROP_CHARGE_NOW:
		ret = blueberry_bat_nac(bbbat, val);
        break;
    case POWER_SUPPLY_PROP_CHARGE_FULL:
        ret = blueberry_bat_lmd(bbbat, val);
        break;
    case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
        ret = blueberry_bat_ilmd(bbbat, val);
        break;
    case POWER_SUPPLY_PROP_CYCLE_COUNT:
        ret = blueberry_bat_cyct(bbbat, val);
        break;
    case POWER_SUPPLY_PROP_ENERGY_NOW:
        ret = blueberry_bat_energy(bbbat, val);
        break;
#endif
	case POWER_SUPPLY_PROP_HEALTH:
		ret = blueberry_bat_health(bbbat, val);
		break;
    default:
        ret = -EINVAL;
		break;
    }

	mutex_unlock(&bbbat->xfer_lock);

    return ret;
}

#define to_blueberry_bat_by_ac(x) container_of((x), \
                struct blueberry_bat, ac);

static int blueberry_ac_get_property(struct power_supply *psy,
            enum power_supply_property psp,
            union power_supply_propval *val)
{
    int ret = 0;
    struct blueberry_bat *bbbat = to_blueberry_bat_by_ac(psy);

	/* only for debug */
	/* pr_info("blueberry_bat, ac_get_property, bbbat->debug=%d, bbbat->dc_det_pin=%d.\n",
							bbbat->debug, bbbat->dc_det_pin);
	*/

    switch (psp) {
    case POWER_SUPPLY_PROP_ONLINE:
        if (psy->type == POWER_SUPPLY_TYPE_MAINS){
            if(gpio_get_value(bbbat->dc_det_pin))
                val->intval = 0;    /*discharging*/
            else
                val->intval = 1;    /*charging*/
        }
		blueberry_bat_debug("blueberry_ac_get_property, val->intval=%d\n", val->intval);
		/* only for debug */
		/* pr_info("blueberry_bat, ac_get_property, val->intval=%d\n", val->intval); */
        break;

    default:
        ret = -EINVAL;
        break;
    }
    return ret;
}

static void blueberry_bat_external_power_changed(struct power_supply *psy)
{
    struct blueberry_bat *bbbat = to_blueberry_bat_by_bat(psy);

    cancel_delayed_work_sync(&bbbat->work);
    schedule_delayed_work(&bbbat->work, 0);
}

static int blueberry_bat_powersupply_init(struct blueberry_bat *bbbat)
{
    int ret;

    bbbat->bat.type = POWER_SUPPLY_TYPE_BATTERY;
    bbbat->bat.properties = blueberry_bat_props;
    bbbat->bat.num_properties = ARRAY_SIZE(blueberry_bat_props);
    bbbat->bat.get_property = blueberry_bat_get_property;
    bbbat->bat.external_power_changed = blueberry_bat_external_power_changed;

    INIT_DELAYED_WORK(&bbbat->work, blueberry_bat_work);

    ret = power_supply_register(bbbat->dev, &bbbat->bat);
    if (ret) {
        dev_err(bbbat->dev, "failed to register battery: %d\n", ret);
        return ret;
    }
	pr_err("register blueberry_bat succ.\n");

    blueberry_bat_update(bbbat);

    return 0;
}

static int blueberry_ac_powersupply_init(struct blueberry_bat *bbbat)
{
	int ret;

    bbbat->ac.type = POWER_SUPPLY_TYPE_MAINS;
    bbbat->ac.properties = blueberry_ac_props;
    bbbat->ac.num_properties = ARRAY_SIZE(blueberry_ac_props);
    bbbat->ac.get_property = blueberry_ac_get_property;

	ret = power_supply_register(bbbat->dev, &bbbat->ac);
	if(ret) {
		blueberry_bat_debug("failed to register ac: %d\n", ret);
		return ret;
	}
	
	return 0;
}

static void blueberry_bat_powersupply_unregister(struct blueberry_bat *bbbat)
{
    cancel_delayed_work_sync(&bbbat->work);

    power_supply_unregister(&bbbat->bat);
}

static ssize_t blueberry_bat_debug_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t len)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct blueberry_bat *bbbat = i2c_get_clientdata(client);

    pr_info("enter %s\n",__FUNCTION__);

    if (*buf == 'E') {

        pr_info("user enable debug battery debug by sysfs.\n");
		bbbat->debug = 1;

    }else if(*buf == 'D') {

        pr_info("user disable battery debug by sysfs.\n");
		bbbat->debug = 0;

    }

    return len;
}

static ssize_t blueberry_bat_debug_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct blueberry_bat *bbbat = i2c_get_clientdata(client);

    pr_info("enter %s\n",__FUNCTION__);

    return sprintf(buf, "%d\n", bbbat->debug > 0 ? 1 : 0);
}

static ssize_t blueberry_fw_debug_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t len)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct blueberry_bat *bbbat = i2c_get_clientdata(client);

    pr_info("enter %s\n",__FUNCTION__);

    if (*buf == 'E') {

        pr_info("user enable fw debug by sysfs.\n");
        bbbat->fw_debug = 1;

    }else if(*buf == 'D') {

        pr_info("user disable fw debug by sysfs.\n");
        bbbat->fw_debug = 0;

    }

    return len;
};

static ssize_t blueberry_fw_debug_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct blueberry_bat *bbbat = i2c_get_clientdata(client);

    pr_info("enter %s\n",__FUNCTION__);

    return sprintf(buf, "%d\n", bbbat->fw_debug > 0 ? 1 : 0);
}

void blueberry_fw_hexdump(unsigned char *fw,int size)
{
    int i;
    for (i=0; i<size; i++)
    {
        printk(" %02x", fw[i]);
        if ((i+1)%32 ==0)
        {
            printk("\n");
        }
    }
    printk("\n");
}

static ssize_t blueberry_fw_dump_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t len)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct blueberry_bat *bbbat = i2c_get_clientdata(client);

	int size = 64 * 1024;
	unsigned char *fw;
	int ret;

    pr_info("enter %s\n",__FUNCTION__);

    if (*buf == 'D') {

        pr_info("user enable ec firmware dump by sysfs.\n");
		fw = vmalloc(size);
		if(!fw) {
			if(bbbat->fw_debug) {
				pr_err("err while alloc ec firmware memory.\n");
			}
			return len;
		}
		memset(fw, 0, size);

		ret = blueberry_fw_read_firmware(bbbat, fw, size);
		if(ret) {
			if(bbbat->fw_debug) {
				pr_err("err while read ec firmware.\n");
			}
			vfree(fw);
			return len;
		}

		blueberry_fw_hexdump(fw, size);
		vfree(fw);		
    }

    return len;
};

static ssize_t blueberry_fw_dump_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct blueberry_bat *bbbat = i2c_get_clientdata(client);
	int ret;
	int ver;

    pr_info("enter %s\n",__FUNCTION__);
	
	ret = blueberry_fw_get_version(bbbat, &ver);
	if(ret) {
		if(bbbat->fw_debug) {
			pr_err("get version error.\n");
		}
		ver = 0;
	}

    return sprintf(buf, "0x%x\n", ver);
}

int blueberry_bat_get_vendor(struct blueberry_bat *bbbat, unsigned char *vendor)
{
    int ret;
    unsigned char buf[2];
	int ven;

    if(atomic_read(&bbbat->working_mode) != NORMAL_MODE) {
		pr_err("not NORMAL_MODE, exit.\n");
		return -ENODEV;
	}	

    /* to prevent to use i2c bus at the same time */
    mutex_lock(&bbbat->xfer_lock);
    ret = blueberry_bat_smbus_read_word_data(bbbat->client, 0xf3, buf, 2);
    mutex_unlock(&bbbat->xfer_lock);
    if(ret){
        blueberry_bat_debug("read battery vendor failed.\n");
        return ret;
    }

    ven = (buf[0] << 8) | buf[1];
    blueberry_bat_debug("battery vendor read, buf[0]=0x%x, buf[1]=0x%x, ver=0x%x\n", buf[0], buf[1], ven);
	*vendor = (unsigned char)ven;

    return 0;
}

/* 1: simpo, 2: LG */
static ssize_t blueberry_bat_vendor_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct blueberry_bat *bbbat = i2c_get_clientdata(client);
    int ret;
    unsigned char ven;

    pr_info("enter %s\n",__FUNCTION__);

    ret = blueberry_bat_get_vendor(bbbat, &ven);
    if(ret) {
        if(bbbat->fw_debug) {
            pr_err("get battery vendor error.\n");
        }
        ven = 0;
    }

    return sprintf(buf, "0x%x\n", ven);
}

static DEVICE_ATTR(debug, S_IRUGO|S_IWUSR, blueberry_bat_debug_show, blueberry_bat_debug_store);
static DEVICE_ATTR(fw_debug, S_IRUGO|S_IWUSR, blueberry_fw_debug_show, blueberry_fw_debug_store);
static DEVICE_ATTR(dump, S_IRUGO|S_IWUSR, blueberry_fw_dump_show, blueberry_fw_dump_store);
static DEVICE_ATTR(vendor, S_IRUGO|S_IWUSR, blueberry_bat_vendor_show, NULL);

static struct attribute *blueberry_bat_attributes[] = {
    &dev_attr_debug.attr,
	&dev_attr_fw_debug.attr,
	&dev_attr_dump.attr,
	&dev_attr_vendor.attr,
    NULL
};

static struct attribute_group blueberry_bat_attribute_group = {
        .attrs = blueberry_bat_attributes
};

#ifdef CONFIG_BAT_ADC_DET
static void blueberry_bat_adc_det_callback(struct adc_client *client, void *client_param, int result)
{
	struct blueberry_bat *bbbat = (struct blueberry_bat *)client_param;
	
	blueberry_bat_debug("bat_adc_det_callback, result=%d\n", result);
	bbbat->adcval = result;

	return;
}
#endif
	
/* If the system has several batteries we need a different name for each
 * of them...
 */
static DEFINE_IDR(battery_id);
static DEFINE_MUTEX(battery_mutex);
static struct blueberry_bat *g_bbbat;
//#ifdef CONFIG_LOGO_LOWERPOWER_WARNING
#define VOLTAGE_ALLOW_POWER_ON	3300000 //uv
int get_battery_status(void)
{
	int volt = 0;
	int ret = 0;
	int err1 = 0;
	int err2 = 0;
	int status = 0;
	//int flags = 0;
	int rsoc = 0;
	//int curr = 0;
	int i;
	
	struct blueberry_bat *bbbat = g_bbbat;

	if(!bbbat)
	{
		printk("%s:bbbat is null\n",__func__);
		return 0;
	}

	for(i=0;i<3;i++)
	{
		err1 = blueberry_bat_read_register(bbbat, regBatVoltage, &volt, "voltage");
		if(err1)
		{	
			printk("%s:blueberry_bat_read_register voltage err, ret=%d\n",__func__, err1);
		}

		err2 = blueberry_bat_read_register(bbbat, regBatStateOfCharge, &rsoc, "rsoc");
		if(err2)
		{
			printk("%s:blueberry_bat_read_register rsoc err, ret=%d\n",__func__, err2);
		}
/*
		err = blueberry_bat_read_register(bbbat, regBatAverageCurrent, &curr, "average current");
		if(err)
		{
			printk("%s:blueberry_bat_read_register average current err, ret=%d\n",__func__, err);
		}
*/
		if(!err1 && !err2)
		{
			volt = volt * 1000;
	
			if((volt < VOLTAGE_ALLOW_POWER_ON) || (rsoc < 1))
			{
				printk("%s:battery too low\n", __func__);
				ret = 1;
			}
			else
			{
				ret = 0;
				break;
			}
		}
	}
/*
	if(!ret)
		goto exit;
	
	err = blueberry_bat_read_register(bbbat, regBatFlags, &flags, "flags");
	if(err) {
		status = POWER_SUPPLY_STATUS_UNKNOWN;	
		pr_err("blueberry_bat: read flags in battery_bat_status failed.status=%d\n",status);
		ret = 0;		
		goto exit;
	}

	if((flags & BLUEBERRY_BAT_FLAG_FC) && (rsoc == 0x64) && !gpio_get_value(bbbat->dc_det_pin)) {
		status = POWER_SUPPLY_STATUS_FULL;
	}else if((flags & BLUEBERRY_BAT_FLAG_FC) && (rsoc == 0x64) && gpio_get_value(bbbat->dc_det_pin)) {
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	}else if((flags & BLUEBERRY_BAT_FLAG_FC) && (rsoc < 0x64) && !gpio_get_value(bbbat->dc_det_pin)) {
		status = POWER_SUPPLY_STATUS_CHARGING;
	}else if(!(flags & BLUEBERRY_BAT_FLAG_FC) && !gpio_get_value(bbbat->dc_det_pin)) {
		status = POWER_SUPPLY_STATUS_CHARGING;
	}else{
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	}

	if(status == POWER_SUPPLY_STATUS_NOT_CHARGING)
		ret = 1;
exit:
*/
	printk("%s:volt=%d,charge=%d,rsoc=%d,ret=%d\n",__func__, volt, status, rsoc, ret);

	return ret;
}

EXPORT_SYMBOL(get_battery_status);

//#endif

static int blueberry_bat_probe(struct i2c_client *client,
                 const struct i2c_device_id *id)
{
    char *name;
	char *ac_name;
    struct blueberry_bat *bbbat;
    int num = -1;
    int retval = 0;
	int charge_type;
	int irq, irq_flag, bat_irq;
	struct blueberry_bat_platform_data *pdata = client->dev.platform_data;

    /* Get new ID for the new battery device */
    retval = idr_pre_get(&battery_id, GFP_KERNEL);
    if (retval == 0)
        return -ENOMEM;

    mutex_lock(&battery_mutex);
    retval = idr_get_new(&battery_id, client, &num);
    mutex_unlock(&battery_mutex);
	pr_info("blueberry_bat, idr_get_new:num=0x%d\n", num);
    if (retval < 0)
        return retval;

    name = kasprintf(GFP_KERNEL, "%s-%d", id->name, num);
    if (!name) {
        dev_err(&client->dev, "failed to allocate device name\n");
        retval = -ENOMEM;
        goto batt_failed_1;
    }

	ac_name = kasprintf(GFP_KERNEL, "%s", "ac");

    bbbat = kzalloc(sizeof(*bbbat), GFP_KERNEL);
    if (!bbbat) {
        dev_err(&client->dev, "failed to allocate device info data\n");
        retval = -ENOMEM;
        goto batt_failed_2;
    }
	
	atomic_set(&bbbat->working_mode, NORMAL_MODE);						/* if upgrade firmware, then set it to FLASH_MODE */

	bbbat->client = client;
    bbbat->id = num;
    bbbat->dev = &client->dev;
    bbbat->bat.name = name;
	bbbat->interval = msecs_to_jiffies(4 * 1000);
	bbbat->debug = 0;										/* for debug message output */
	bbbat->fw_debug = 0;									/* for fw_debug message output */

	bbbat->dc_det_pin = pdata->dc_det_pin;					/* interrupt pin, can wakeup the machine */
	bbbat->bat_det_adc_channel = pdata->bat_adc_channel;	/* bat insert detection, using adc channel 0 */
	bbbat->bat_low_det_pin = pdata->bat_low_det_pin;		/* interrupt pin, can wakeup the machine */

#ifdef CONFIG_CHARGER_INPUT_SELECT
	atomic_set(&bbbat->connect, CONNECT_NONE);
    bbbat->usb_ac_detect_interval = msecs_to_jiffies(3000); /* three seconds interval */
#endif

	g_bbbat = bbbat;

    i2c_set_clientdata(client, bbbat);

	/* need prevent multi-get-property at the same time */
	mutex_init(&bbbat->xfer_lock);

	if((1 == get_battery_status()) && gpio_get_value(bbbat->dc_det_pin))
	{
		charge_type = dwc_otg_check_dpdm();
		printk("-----------%s: charge_type = %d\n", __func__, charge_type);
		switch(charge_type)
		{
			case 2:
				retval = blueberry_bat_charger_input_select(bbbat, CONNECT_ADAPTER_COMMAND);
				if(retval)
					pr_err("tell ec charger input change failed.\n");
				break;
			case 1:
				retval = blueberry_bat_charger_input_select(bbbat, CONNECT_USB_COMMAND);
				if(retval)
					pr_err("tell ec charger input change failed.\n");
				break;
			default:
				break;
		}
		printk("-----------%s: battery low and no charging, so power off !!!!!\n", __func__);
		kernel_power_off();
	}

	/*notify EC: ARM power on*/
	mutex_lock(&bbbat->xfer_lock);
	retval = blueberry_bat_smbus_write_byte_no_data(bbbat->client, 0xf2);
	mutex_unlock(&bbbat->xfer_lock);

#ifdef CONFIG_BAT_ADC_DET
    /* for adc battery plug in/out detect */
    bbbat->adclient = adc_register(0, blueberry_bat_adc_det_callback, bbbat);
    if(!bbbat->adclient) {
        pr_err("%s: adc_register for bat_det failed.\n", __func__);
    }
#endif

    if (blueberry_bat_powersupply_init(bbbat))
        goto batt_failed_3;

	ac_name = kasprintf(GFP_KERNEL, "%s", "ac");
	bbbat->ac.name = ac_name;

	if(blueberry_ac_powersupply_init(bbbat))
		goto batt_failed_4;

	INIT_DELAYED_WORK(&bbbat->work, blueberry_bat_work);
	schedule_delayed_work(&bbbat->work, bbbat->interval);

	if(bbbat->dc_det_pin != INVALID_GPIO) {
		retval = gpio_request(bbbat->dc_det_pin, "dc_det_pin");
		if(retval) {
			blueberry_bat_debug("failed to rquest dc_det_pin\n");
			goto batt_failed_5;
		}
		gpio_pull_updown(bbbat->dc_det_pin, GPIOPullUp);
		retval = gpio_direction_input(bbbat->dc_det_pin);
		if(retval) {
			blueberry_bat_debug("failed set dc_det_pin to input.\n");
			goto batt_failed_6;
		}
		INIT_WORK(&bbbat->dcwakeup_work, blueberry_bat_dcdet_work);
		irq = gpio_to_irq(bbbat->dc_det_pin);
		irq_flag = gpio_get_value(bbbat->dc_det_pin) ? IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING;
		retval = request_irq(irq, blueberry_bat_dcdet_wakeup, irq_flag, "dc_det_irq", bbbat);
		if(retval) {
			blueberry_bat_debug("failed to request dc det irq\n");
			goto batt_failed_6;
		}
		/* according to the rule, we should only enable irq wake when suspend. */
		/* enable_irq_wake(irq); */			/* BUGBUG: when ac in, should we wakeup the machine? need test further */
	}

	if(bbbat->bat_low_det_pin != INVALID_GPIO) {
		retval = gpio_request(bbbat->bat_low_det_pin, "bat_low_det_pin");
		if(retval) {
			blueberry_bat_debug("failed to request bat_low_det_pin\n");
			goto batt_failed_7;
		}
		gpio_pull_updown(bbbat->bat_low_det_pin, GPIOPullUp);
		retval = gpio_direction_input(bbbat->bat_low_det_pin);
		if(retval) {
			blueberry_bat_debug("failed to set bat_low_det_pin to input.\n");
			goto batt_failed_8;
		}
		INIT_WORK(&bbbat->lowpower_work, blueberry_bat_lowpower_work);
		bat_irq = gpio_to_irq(bbbat->bat_low_det_pin);
		retval = request_irq(bat_irq, blueberry_bat_lowpower_wakeup, IRQF_TRIGGER_LOW, "bat_low_det_irq", bbbat);
		if(retval) {
			blueberry_bat_debug("failed to request bat_low_det irq.\n");
			goto batt_failed_8;
		}
		/* according to the rule, we should only enable irq wake when suspend. */
		/* enable_irq_wake(bat_irq); */		/* BUGBUG: when battery low, should we wakeup the machine? need test further */
	}	

	retval = blueberry_fw_driver_init(bbbat);
	if(retval) {
		blueberry_bat_debug("failed to init fw device.\n");
		goto batt_failed_9;
	}
	/*delete from here because we didn't need upgrade in kernel*/
/*
#if defined(CONFIG_BLUEBERRY_FW_KERNEL_UPGRADE)
	retval = blueberry_fw_kernel_upgrade(bbbat);	
	if(retval) {
		blueberry_bat_debug("failed to upgrade ec fw in kernel.\n");
	}
#endif
*/

#ifdef CONFIG_CHARGER_INPUT_SELECT
    bbbat->usb_ac_detect_workqueue = create_singlethread_workqueue("usb_ac_detectd");
    INIT_DELAYED_WORK(&bbbat->usb_ac_detect_work, blueberry_bat_usb_ac_detect_work);
    /* when driver up, do a detect immediately */
    queue_delayed_work(bbbat->usb_ac_detect_workqueue, &bbbat->usb_ac_detect_work, 0);  //bbbat->usb_ac_detect_interval);
#endif

 	retval = sysfs_create_group(&client->dev.kobj, &blueberry_bat_attribute_group);
    if (retval) {
        pr_err("%s: sysfs_create_group returned err = %d. Abort.\n", __func__, retval);
    }
    blueberry_bat_debug("sysfs_create_group OK \n");

	device_init_wakeup(&client->dev, 1);

	pr_info("blueberry_bat proble completed successfully.\n");

    return 0;

batt_failed_9:
	if(bbbat->bat_low_det_pin != INVALID_GPIO)
		free_irq(gpio_to_irq(bbbat->bat_low_det_pin), bbbat);
batt_failed_8:
	if(bbbat->bat_low_det_pin != INVALID_GPIO)
		gpio_free(bbbat->bat_low_det_pin);
batt_failed_7:
	if(bbbat->dc_det_pin != INVALID_GPIO)
		free_irq(gpio_to_irq(bbbat->dc_det_pin), bbbat);
batt_failed_6:
	if(bbbat->dc_det_pin != INVALID_GPIO)
		gpio_free(bbbat->dc_det_pin);
batt_failed_5:
	power_supply_unregister(&bbbat->ac);
batt_failed_4:
	blueberry_bat_powersupply_unregister(bbbat);
	kfree(ac_name);
batt_failed_3:
    kfree(bbbat);
batt_failed_2:
    kfree(name);
batt_failed_1:
    mutex_lock(&battery_mutex);
    idr_remove(&battery_id, num);
    mutex_unlock(&battery_mutex);

    return retval;
}

static int blueberry_bat_remove(struct i2c_client *client)
{
	struct blueberry_bat *bbbat = i2c_get_clientdata(client);

	power_supply_unregister(&bbbat->ac);
	kfree(bbbat->ac.name);

    blueberry_bat_powersupply_unregister(bbbat);
    kfree(bbbat->bat.name);

    mutex_lock(&battery_mutex);
    idr_remove(&battery_id, bbbat->id);
    mutex_unlock(&battery_mutex);

	if(bbbat->dc_det_pin != INVALID_GPIO) {
		free_irq(gpio_to_irq(bbbat->dc_det_pin), bbbat);
		gpio_free(bbbat->dc_det_pin);
	}

	if(bbbat->bat_low_det_pin != INVALID_GPIO) {
		free_irq(gpio_to_irq(bbbat->bat_low_det_pin), bbbat);
		gpio_free(bbbat->bat_low_det_pin);
	}

	device_init_wakeup(&client->dev, 0);

	mutex_destroy(&bbbat->xfer_lock);

	i2c_set_clientdata(client, NULL);

	blueberry_fw_driver_uninit(bbbat);			/* do uninit for fw upgrade device */

#ifdef CONFIG_BAT_ADC_DET
	if(bbbat->adclient)
		adc_unregister(bbbat->adclient);		/* unregister adc for bat plug in/out detect */
#endif

#ifdef CONFIG_CHARGER_INPUT_SELECT
    cancel_delayed_work(&bbbat->usb_ac_detect_work);
#endif

    kfree(bbbat);
    return 0;
}

static const struct i2c_device_id blueberry_bat_id[] = {
    { "blueberry_bat", 0 },
};
MODULE_DEVICE_TABLE(i2c, blueberry_bat_id);

static struct i2c_driver blueberry_bat_driver = {
    .driver = {
        .name = "blueberry_bat",
    },
    .probe = blueberry_bat_probe,
    .remove = blueberry_bat_remove,
#ifdef CONFIG_PM
	.suspend = blueberry_bat_suspend,
	.resume = blueberry_bat_resume,
#endif
    .id_table = blueberry_bat_id,
};

static int __init blueberry_bat_init(void)
{
    int ret;

    ret = i2c_add_driver(&blueberry_bat_driver);
    if (ret)
        pr_err("blueberry_bat: unable to register blueberry_bat driver\n");

    return ret;
}
subsys_initcall_sync(blueberry_bat_init);   /* module_init(), fs_initcall_sync(), arch_initcall() */

static void __exit blueberry_bat_exit(void)
{
    i2c_del_driver(&blueberry_bat_driver);
}
module_exit(blueberry_bat_exit);

MODULE_AUTHOR("Paul Ma");
MODULE_DESCRIPTION("BlueBerry battery monitor driver");
MODULE_LICENSE("GPL");

