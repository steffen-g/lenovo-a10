 /*
 * bq24193 battery driver
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <mach/gpio.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <mach/board.h>
#include <linux/interrupt.h>

#include <linux/power/bq24195_charger.h>

#define BQ24193_SPEED 			100 * 1000
#define MAX_REG_INDEX           10

#define REG_IN_SRC         0x0
#define REG_PWR_ON_CFG     0x1
#define REG_CHRG_C         0x2
#define REG_P_CHRG_TRM_C   0x3
#define REG_CHRG_V         0x4
#define REG_CHRG_TRM_TMR   0x5
#define REG_IR_CMP_T_REG   0x6
#define REG_MISC_OP        0x7
#define REG_SYS_STATUS     0x8
#define REG_FAULT          0x9
#define REG_V_P_REV        0xA
#define NO_CHARGER_MODE    0
#define USB_HOST_MODE      1
#define DC_MODE            2
#define OTG_MODE           3
#define CHARGE_LOW_TEMP        4
#define CHARGE_OVER_TEMP       5

#if 0
#define BQ24193_DG(x...) printk(KERN_INFO x)
#else
#define BQ24193_DG(x...) do { } while (0)
#endif



struct bq24193_device
{
    struct i2c_client *client;
    struct delayed_work power_work;
    struct delayed_work irq_work;
    struct power_supply ac;
    struct power_supply usb;

    int wake_irq;
    int otg_power_en;
    int charge_mode;
    int over_temp;
    int charge_en;
    unsigned int charger_en_pin;
    unsigned int charger_det_pin;
    int now_current;
};

struct bq24193_device *g_bq24193_dev=NULL;

static int bq24193_read(struct i2c_client *client, const char reg, char *buf, int len)
{
    int ret;
    ret = i2c_master_reg8_recv(client, reg, buf, len, BQ24193_SPEED);
    return ret;
}

static int bq24193_write(struct i2c_client *client,const char reg, char *buf, int len)
{
    int ret;
    BQ24193_DG("%s: reg 0x%x, value 0x%x \n",__func__,reg, *buf);
    ret = i2c_master_reg8_send(client, reg, buf, len, BQ24193_SPEED);
    return ret;
}
static int dump_bq24193_reg(struct bq24193_device *dev)
{
    int ret = 0;
    char buf = 0;
    int reg = 0;
    if(!dev)
    {
        printk("dev is null");
        return -1;
    }
    for(reg = 0; reg <= MAX_REG_INDEX; reg++)
    {
    	ret = i2c_master_reg8_recv(dev->client, reg, &buf, 1, BQ24193_SPEED);

    	if(ret < 0)
    	{
        	printk("read smb137 reg error:%d\n",ret);
    	}
    	else
    	{
        	printk("reg 0x%x:0x%x\n",reg,buf);
    	}
    }

	return 0;
}

static ssize_t bq24193_debug_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t _count)
{
    int temp;
    u8 reg;
    u8 val;
    struct bq24193_device *bq24193_dev = dev_get_drvdata(dev);
    if (sscanf(buf, "%x", &temp) != 1)
        return -EINVAL;
    val = temp & 0x00ff;
    reg = temp >> 8;
    bq24193_write(bq24193_dev->client, reg, &val,1);
    return _count;
}

static ssize_t bq24193_debug_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
    struct bq24193_device *bq24193_dev = dev_get_drvdata(dev);
    dump_bq24193_reg(bq24193_dev);
    return 0;
}


static ssize_t charger_charge_enable_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
    if(g_bq24193_dev == NULL)return sprintf(buf, "%d\n", 0);
    return sprintf(buf, "%d\n", ((g_bq24193_dev->charge_mode==USB_HOST_MODE)||(g_bq24193_dev->charge_mode==DC_MODE)));
}

static ssize_t bq24193_charge_enable_battery_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t _count)
{
    int temp;
    u8 reg;
    u8 val;
    struct bq24193_device *bq24193_dev = dev_get_drvdata(dev);
    if (sscanf(buf, "%x", &temp) != 1)
        return -EINVAL;
    if(g_bq24193_dev == NULL)return _count;
    
    val = temp & 0xff;
    
    bq24193_dev->charge_en = val ? 1 : 0;

    bq24193_read(bq24193_dev->client, REG_IN_SRC, &reg, 1);
    reg &= ~0x80;
    reg |= val ? 0 : 0x80;
    bq24193_write(bq24193_dev->client, REG_IN_SRC, &reg,1);
    msleep(10);
    bq24193_read(bq24193_dev->client, REG_SYS_STATUS, &reg, 1);
    bq24193_read(bq24193_dev->client, REG_SYS_STATUS, &reg, 1);
    g_bq24193_dev->charge_mode = val ? ((reg & 0xc0) >> 6) : NO_CHARGER_MODE;
    return _count;
}

static ssize_t bq24193_charge_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t _count)
{
    int temp;
    u8 reg;
    u8 val;
    struct i2c_client *client = NULL;
    struct bq24195_info *info = NULL;

    if (sscanf(buf, "%x", &temp) != 1)
        return -EINVAL;
    val = temp & 0xff;
    if(g_bq24193_dev == NULL)return _count;
    
    client = g_bq24193_dev->client;
    info = (struct bq24195_info*)client->dev.platform_data;
    g_bq24193_dev->charge_en = val ? 1 : 0;

    gpio_set_value(info->chg_en_pin, val ? GPIO_LOW : GPIO_HIGH);
    bq24193_read(g_bq24193_dev->client, REG_SYS_STATUS, &reg, 1);
    bq24193_read(g_bq24193_dev->client, REG_SYS_STATUS, &reg, 1);
    g_bq24193_dev->charge_mode = val ? ((reg & 0xc0) >> 6) : NO_CHARGER_MODE;
    return _count;
}

static struct device_attribute  bq24193_attrs[] = {
    __ATTR(charge_debug, S_IRUGO | S_IWUSR, bq24193_debug_show, bq24193_debug_store),
    __ATTR(charge_enable_battery, S_IRUGO| S_IWUSR, charger_charge_enable_show, bq24193_charge_enable_battery_store),
    __ATTR(charge_enable_input, S_IRUGO| S_IWUSR, charger_charge_enable_show, bq24193_charge_enable_store),
};


static int  bq24193_create_sysfs(struct device *dev)
{
    int r;
    int t;
    for (t = 0; t < ARRAY_SIZE(bq24193_attrs); t++)
    {
        r = device_create_file(dev,&bq24193_attrs[t]);
        if (r)
        {
            dev_err(dev, "failed to create sysfs "
					"file\n");
            return r;
        }
    }
    return 0;
}

__weak int bq27541_battery_get_temp(void)
{
	return 0;
}

//0: No charging , 1: USB, 2: DC, 3:otg, 4: low temp ;5:over temp;
int rk30_get_charger_mode(void)
{
    struct i2c_client *client = NULL;
    char reg =0;
    char status_reg = 0;
    char fault_reg = 0;
    int temp = 0;
    struct bq24195_info *info = NULL;
   // BQ24193_DG("%s \n",__func__);
    if(g_bq24193_dev == NULL)
    {
        printk("have no charger detct");
        return 0;
    }
    client = g_bq24193_dev->client;
    info = (struct bq24195_info*)client->dev.platform_data;

    bq24193_read(client, REG_SYS_STATUS, &status_reg, 1);
    BQ24193_DG("%s status = %x \n",__func__,status_reg);
    
    bq24193_read(client, REG_FAULT, &fault_reg, 1);
    temp = bq27541_battery_get_temp();
    BQ24193_DG("%s fault_reg = %x, temp = %d \n",__func__,fault_reg,temp);
    if((g_bq24193_dev->charge_en == 0)||(gpio_get_value(RK30_PIN0_PB2)==GPIO_HIGH))
    {
        g_bq24193_dev->charge_mode = NO_CHARGER_MODE;
        return NO_CHARGER_MODE;
    }

    if(((fault_reg&0x7)!=0) || (temp < 30) || (temp > 470))    //over-temp Protection or NTC fault
    {
        g_bq24193_dev->over_temp = 1;
        g_bq24193_dev->charge_mode = NO_CHARGER_MODE;
        gpio_set_value(info->chg_en_pin, GPIO_HIGH);
    }else if(((temp > 40)&&(temp < 420)) && (g_bq24193_dev->charge_mode==NO_CHARGER_MODE))
    {
        g_bq24193_dev->over_temp = 0;
        if((status_reg & 0xc0)==0x40)
        {
            reg = 0x52;
            bq24193_write(client,REG_IN_SRC,&reg,1);
            gpio_set_value(info->chg_en_pin, GPIO_LOW);
        }else if((status_reg & 0xc0)==0x80)
        {
            reg = 0x55;
            bq24193_write(client,REG_IN_SRC,&reg,1);
            gpio_set_value(info->chg_en_pin, GPIO_LOW);
        }
    }

    if(g_bq24193_dev->over_temp == 0)
    {
        g_bq24193_dev->charge_mode = ((status_reg & 0xc0) >> 6);
        return g_bq24193_dev->charge_mode;
    }
    else return CHARGE_OVER_TEMP;
}
EXPORT_SYMBOL(rk30_get_charger_mode);

static int bq24193_init(struct i2c_client *client)
{
    int ret = 0;
    u8 reg;
    u8 fault_reg=0;
    struct bq24195_info *info = (struct bq24195_info*)client->dev.platform_data;
    BQ24193_DG("%s !!!\n",__func__);
    reg = 0x80;     //reset register
    bq24193_write(client,REG_PWR_ON_CFG,&reg,1);

    msleep(1);
    bq24193_read(client, REG_CHRG_TRM_TMR, &reg, 1);
    reg &= ~0x30;  // disable whactdog, enable Charging Termination
    ret = bq24193_write(client,REG_CHRG_TRM_TMR,&reg,1);

    reg = 0x1b;     //enable charge
    bq24193_write(client,REG_PWR_ON_CFG,&reg,1);
    reg = 0x10;  // set Charging Termination current 128MA
    ret = bq24193_write(client,REG_P_CHRG_TRM_C,&reg,1);

    bq24193_read(client, REG_CHRG_C, &reg, 1);
    reg = 0x60;     // Fast Charge Current Limit 2.0A
    ret = bq24193_write(client,REG_CHRG_C,&reg,1);

    bq24193_read(client, REG_CHRG_V, &reg, 1);
    reg = 0xb2;     // Charge Voltage Limit 4.208v
    ret = bq24193_write(client,REG_CHRG_V,&reg,1);
    bq24193_read(client, REG_SYS_STATUS, &reg, 1);
    //bq24193_read(client, REG_SYS_STATUS, &reg, 1);
    bq24193_read(client, REG_FAULT, &fault_reg, 1);
    if((((reg&0xc0) == 0x80) || ((reg&0xc0) == 0x40))  //adapter port or usb host
        && ((fault_reg&0x7)==0))                         //NTC not over temp
    {
        g_bq24193_dev->over_temp = 0;
        //gpio_set_value(info->chg_en_pin, GPIO_LOW);
        g_bq24193_dev->charge_mode = ((reg & 0xc0) >> 6);
        if(g_bq24193_dev->charge_mode == DC_MODE)
        {
            reg = 0x55;            // 1.5A
        }   else  {
            reg = 0x55;            // 0.5A
        }
    }  else {
        if((fault_reg&0x7)!=0)g_bq24193_dev->over_temp = 1;
        else g_bq24193_dev->over_temp = 0;
        //gpio_set_value(info->chg_en_pin, GPIO_HIGH);
        g_bq24193_dev->charge_mode = NO_CHARGER_MODE;
        reg = 0x55;
    }
    bq24193_write(client,REG_IN_SRC,&reg,1);
    return 0;
}

#ifdef CONFIG_OTG_POWER_PROVIDE_BY_CHARGER
static void bq24193_host_power_work(struct work_struct *work)
{
    char reg = 0;
    BQ24193_DG(" %s \n",__func__);
    if(g_bq24193_dev == NULL)
    {
        printk(" have no charger detct");
        return;
    }
    if(g_bq24193_dev->otg_power_en)
    {
        bq24193_read(g_bq24193_dev->client, REG_PWR_ON_CFG, &reg, 1);
        reg &= ~0x30;
        reg |= 0x20;   //enable otg
    	bq24193_write(g_bq24193_dev->client,REG_PWR_ON_CFG,&reg,1);
    }
    else
    {
        bq24193_read(g_bq24193_dev->client, REG_PWR_ON_CFG, &reg, 1);
        reg &= ~0x30;
        reg |= 0x10;   //enable charge
    	bq24193_write(g_bq24193_dev->client,REG_PWR_ON_CFG,&reg,1);
    }
}


void usb_host_power_ctrl(int power_en)
{
    BQ24193_DG("\n\n\n  %s; power = [%d] \n\n\n",__func__, power_en);
    if(g_bq24193_dev == NULL)
    {
        printk(" have no charger detct");
        return;
    }
    g_bq24193_dev->otg_power_en = power_en;
    schedule_delayed_work(&g_bq24193_dev->power_work, msecs_to_jiffies(0));
}
EXPORT_SYMBOL(usb_host_power_ctrl);
#endif

extern ssize_t dwc_otg_conn_en_store(struct device_driver *_drv, const char *_buf,
				     size_t _count);

static void bq24193_get_status_work(struct work_struct *work)
{
    struct i2c_client *client = NULL;
    char status_reg = 0;
    char fault_reg = 0;
    char reg = 0;
    char usb_en[2]={0,0};
    int temp = 0;
    
    struct bq24195_info *info = NULL;
    if(g_bq24193_dev == NULL)
    {
        printk(" have no charger detct");
        return;
    }

    client = g_bq24193_dev->client;
    info = (struct bq24195_info*)client->dev.platform_data;

    bq24193_read(client, REG_SYS_STATUS, &status_reg, 1);
    BQ24193_DG("%s status 1 = %x \n",__func__,status_reg);
    msleep(10);
    bq24193_read(client, REG_SYS_STATUS, &status_reg, 1);
    BQ24193_DG("%s status 2 = %x \n",__func__,status_reg);

    bq24193_read(client, REG_FAULT, &fault_reg, 1);
    BQ24193_DG("%s fault = %x ,g_bq24193_dev->charge_en = %d\n",__func__,fault_reg,g_bq24193_dev->charge_en);

    if(g_bq24193_dev->charge_en == 0)
        return;
    temp = bq27541_battery_get_temp();

    if((gpio_get_value(RK30_PIN0_PB2)==GPIO_LOW)                                //adapter port or usb host
         && ((fault_reg&0x7)==0) &&(temp>40 && temp<420 ))   //over-temp Protection
    {
        switch(status_reg & 0xc0){ //charge done
        case 0:
            gpio_set_value(info->chg_en_pin, GPIO_HIGH);
            g_bq24193_dev->charge_mode = NO_CHARGER_MODE;
            reg = 0x50;     // set input current limit 0.1A
            break;
        case 0x40:
            gpio_set_value(info->chg_en_pin, GPIO_LOW);     //enable charger
            g_bq24193_dev->charge_mode = USB_HOST_MODE;
            reg = 0x52;     // usb host charging, set input current limit 0.5A
            break;
        case 0x80:
            gpio_set_value(info->chg_en_pin, GPIO_LOW);     //enable charger
            g_bq24193_dev->charge_mode = DC_MODE;
            reg = 0x55;     // set input current limit 1.5A
            break;
        case 0xc0:
            gpio_set_value(info->chg_en_pin, GPIO_HIGH);
            g_bq24193_dev->charge_mode = OTG_MODE;
            reg = 0x50;     // default input current limit 0.1A
            break;
        }
        bq24193_write(client,REG_IN_SRC,&reg,1);
        g_bq24193_dev->over_temp = 0;
    }
    else {
        gpio_set_value(info->chg_en_pin, GPIO_HIGH);
        g_bq24193_dev->charge_mode = NO_CHARGER_MODE; 
        if(((fault_reg&0x7)!=0) ||(temp<=40) || (temp>=420 ))
            g_bq24193_dev->over_temp = 1;
        else g_bq24193_dev->over_temp = 0;
    }
   
    power_supply_changed(&g_bq24193_dev->ac);
    power_supply_changed(&g_bq24193_dev->usb);

    if((status_reg & 0xc0)==0x40)
    {
        usb_en[0] = '1';
        //dwc_otg_conn_en_store(NULL,usb_en,2);
    }    else if(g_bq24193_dev->charge_mode == NO_CHARGER_MODE){
        usb_en[0] = '0';
        //dwc_otg_conn_en_store(NULL,usb_en,2);
    }
   

}

static irqreturn_t bq24193_irq(int irq, void *dev_id)
{
	struct bq24193_device *dev = (struct bq24193_device *)dev_id;

	printk("!!! %s !!!\n\n\n",__func__);

    schedule_delayed_work(&dev->irq_work, msecs_to_jiffies(0));

	return IRQ_HANDLED;
}

static enum power_supply_property bq24193_charger_props[] = {
//    POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
};

static int bq24193_dc_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	//struct power_supply *bci = dev_get_drvdata(psy->dev->parent);
	switch (psp) {	
    case POWER_SUPPLY_PROP_STATUS:
        if(g_bq24193_dev->charge_mode == DC_MODE)         
            val->intval=POWER_SUPPLY_STATUS_CHARGING;
        else val->intval=POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = (g_bq24193_dev->charge_mode == DC_MODE);		
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
static int bq24193_usb_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	//struct power_supply *bci = dev_get_drvdata(psy->dev->parent);
	switch (psp) {		
    case POWER_SUPPLY_PROP_STATUS:
        if((g_bq24193_dev->charge_mode == DC_MODE)||(g_bq24193_dev->charge_mode == USB_HOST_MODE))
            val->intval=POWER_SUPPLY_STATUS_CHARGING;
        else val->intval=POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval=(g_bq24193_dev->charge_mode == USB_HOST_MODE);		
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int bq24193_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
    int ret = 0;
    struct bq24193_device *bq24193_dev;
    struct bq24195_info *info;
    printk(KERN_INFO "charger %s start\n",__func__);

    bq24193_dev = kzalloc(sizeof(struct bq24193_device), GFP_KERNEL);
    if (!bq24193_dev) {
        dev_err(&client->dev, "failed to allocate device info data\n");
        ret = -ENOMEM;
        return ret;
    }
    
    i2c_set_clientdata(client, bq24193_dev);
	dev_set_drvdata(&client->dev,bq24193_dev);
    bq24193_dev->client = client;
	info = (struct bq24195_info*)client->dev.platform_data;
    bq24193_dev->charger_en_pin = info->chg_en_pin;
    bq24193_dev->charger_det_pin = info->chg_det_pin;
    bq24193_dev->charge_mode = NO_CHARGER_MODE;
    bq24193_dev->charge_en = 1;
    bq24193_dev->over_temp = 0;
    g_bq24193_dev = bq24193_dev;

//--------zyw add for debug --------

//    dump_bq24193_reg(bq24193_dev);

//----------------
	if(info->chg_en_pin != INVALID_GPIO)
	{
		ret = gpio_request(info->chg_en_pin, "chg en pin");
		if (ret != 0) {
			gpio_free(info->chg_en_pin);
			printk("bq24193 gpio_request chg_en_pin error\n");
			return -EIO;
		}
		gpio_direction_output(info->chg_en_pin, GPIO_HIGH);
		gpio_set_value(info->chg_en_pin, GPIO_HIGH);
	}
    if(info->chg_det_pin != INVALID_GPIO)
    {
		ret = gpio_request(info->chg_det_pin, "chg det pin");
		if (ret != 0) {
			gpio_free(info->chg_det_pin);
			printk("bq24193 gpio_request chg_det_pin error\n");
			return -EIO;
		}
		gpio_direction_input(info->chg_det_pin);
	}
	bq24193_init(client);
	ret = bq24193_create_sysfs(&client->dev);
	if(ret)
	{
		dev_err(&client->dev, "failed to create sysfs file\n");
		return ret;
	}

    INIT_DELAYED_WORK(&bq24193_dev->irq_work, bq24193_get_status_work);

#ifdef CONFIG_OTG_POWER_PROVIDE_BY_CHARGER
    INIT_DELAYED_WORK(&bq24193_dev->power_work, bq24193_host_power_work);
#endif
	bq24193_dev->ac.name = "Adapter";
	bq24193_dev->ac.type = POWER_SUPPLY_TYPE_MAINS;
	bq24193_dev->ac.properties = bq24193_charger_props;
	bq24193_dev->ac.num_properties = ARRAY_SIZE(bq24193_charger_props);
	bq24193_dev->ac.get_property = bq24193_dc_get_property;
	bq24193_dev->ac.external_power_changed = NULL;

	bq24193_dev->usb.name = "USB";
	bq24193_dev->usb.type = POWER_SUPPLY_TYPE_USB;
	bq24193_dev->usb.properties = bq24193_charger_props;
	bq24193_dev->usb.num_properties = ARRAY_SIZE(bq24193_charger_props);
	bq24193_dev->usb.get_property = bq24193_usb_get_property;
	bq24193_dev->usb.external_power_changed = NULL;
    
    ret = power_supply_register(&client->dev, &bq24193_dev->ac);
	if (ret){
		printk("power_supply_register bq24160_info.ac  fail\n");
		return 1;
	}
    
    ret = power_supply_register(&client->dev, &bq24193_dev->usb);
	if (ret){
		printk("power_supply_register bq24160_info.ac  fail\n");
		return 1;
	}

    //irq
	bq24193_dev->wake_irq = gpio_to_irq(client->irq);
	ret = request_irq(bq24193_dev->wake_irq, bq24193_irq, IRQF_TRIGGER_FALLING, "bq24193_charger", bq24193_dev);
	if (ret) {
		printk("failed to request bat det irq\n");
	}
    enable_irq_wake(bq24193_dev->wake_irq);

	printk(KERN_INFO "charger %s end\n",__func__);

	return 0;
}

static int bq24193_remove(struct i2c_client *client)
{
    printk("%s !!!\n",__func__);
    bq24193_init(client);
	return 0;
}

static void bq24193_shutdown(struct i2c_client *client)
{
    printk("%s !!!\n",__func__);
    bq24193_init(client);
    disable_irq(g_bq24193_dev->wake_irq);
    free_irq(g_bq24193_dev->wake_irq,g_bq24193_dev);
}

static const struct i2c_device_id bq24193_id[] = {
	{ "bq24193", 0 },
	{}
};

static struct i2c_driver bq24193_battery_driver = {
	.driver = {
		.name = "bq24193",
	},
	.probe = bq24193_probe,
	.remove =  bq24193_remove,
	.shutdown =  bq24193_shutdown,
	.id_table = bq24193_id,
};

static int __init bq24193_battery_init(void)
{
	int ret;

	ret = i2c_add_driver(&bq24193_battery_driver);
	if (ret)
		printk(KERN_ERR "Unable to register bq24193 driver\n");

	return ret;
}
subsys_initcall(bq24193_battery_init);

static void __exit bq24193_battery_exit(void)
{
	i2c_del_driver(&bq24193_battery_driver);
}
module_exit(bq24193_battery_exit);

MODULE_AUTHOR("Jyj");
MODULE_DESCRIPTION("bq24193 battery monitor driver");
MODULE_LICENSE("GPL");
