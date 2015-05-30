
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/jiffies.h>
#include <linux/fs.h>
#include <linux/power_supply.h>
#include <mach/gpio.h>
#include <mach/iomux.h>
#include <mach/board.h>

//#include <linux/power/ns115-battery.h>
#include <linux/string.h>
#include <asm/irq.h>

struct ns115_battery_gauge  {
	int (*get_battery_mvolts)(void);
	int (*get_battery_capacity)(int, int);
};

/*----- driver defines -----*/
#define MYDRIVER "cw2015"


#define REG_VERSION     0x0
#define REG_VCELL       0x2
#define REG_SOC         0x4
#define REG_RRT_ALERT   0x6
#define REG_CONFIG      0x8
#define REG_MODE        0xA
#define REG_BATINFO     0x10

#define SIZE_BATINFO    64 

#define MODE_SLEEP_MASK (0x3<<6)
#define MODE_SLEEP      (0x3<<6)
#define MODE_NORMAL     (0x0<<6)
#define MODE_QUICK_START (0x3<<4)
#define MODE_RESTART    (0xf<<0)

#define CONFIG_UPDATE_FLG (0x1<<1)
#define ATHD (0xa<<3)   //ATHD =10%

/*----- config part for battery information -----*/
#if 0
#undef dev_info
#define dev_info dev_err
#endif

#define FORCE_WAKEUP_CHIP 1

#define BATT_MAX_VOL_VALUE		  4200				//满电时的电池电压	 
#define BATT_ZERO_VOL_VALUE 	  3400				//关机时的电池电压 

static int iiii=0;

static int sitemp =0xFF00;

extern void kernel_power_off(void);
#if 0
/* battery info: two 3650 battery 7300mah add by ben for tianzhi */
static char cw2015_bat_config_info[SIZE_BATINFO] = {
0x15  ,0x4C  ,0x5D  ,0x5D  ,0x5A  ,0x59  ,0x55  ,
0x51  ,0x4E  ,0x48  ,0x46  ,0x41  ,0x3C  ,0x39  ,
0x33  ,0x2D  ,0x25  ,0x1E  ,0x19  ,0x19  ,0x1A  ,
0x2C  ,0x44  ,0x4A  ,0x43  ,0x40  ,0x0C  ,0xCD  ,
0x22  ,0x43  ,0x56  ,0x82  ,0x78  ,0x6F  ,0x62  ,
0x60  ,0x42  ,0x19  ,0x37  ,0x31  ,0x00  ,0x1D  ,
0x59  ,0x85  ,0x8F  ,0x91  ,0x91  ,0x18  ,0x58  ,
0x82  ,0x94  ,0xA5  ,0xFF  ,0xAF  ,0xE8  ,0xCB  ,
0x2F  ,0x7D  ,0x72  ,0xA5  ,0xB5  ,0xC1  ,0x46  ,
0xAE
};
#endif
//V80_BLD102_XWD4030_ProfileV3_20130313.txt
static char cw2015_bat_config_info[SIZE_BATINFO] = {
    0x15, 0x79, 0x6a, 0x68, 0x64,
    0x60, 0x59, 0x55, 0x52, 0x4E,
    0x4A, 0x44, 0x3F, 0x34, 0x23,
    0x1B, 0x18, 0x1E, 0x29, 0x42,
    0x56, 0x6F, 0x6E, 0x68, 0x63,
    0x76, 0x0B, 0x85, 0x0D, 0x1A,
    0x2F, 0x3D, 0x42, 0x43, 0x45,
    0x47, 0x3F, 0x1B, 0x47, 0x45,
    0x09, 0x2C, 0x28, 0x59, 0x88,
    0x94, 0x97, 0x18, 0x58, 0x82,
    0x94, 0xA5, 0x78, 0x9F, 0xFF,
    0xCB, 0x2F, 0x7D, 0x72, 0xA5,
    0xB5, 0xC1, 0x46, 0xAE
};

struct cw2015_device_info {
	struct device 		*dev;
	struct power_supply	bat;
	struct power_supply	ac;
	struct power_supply	usb;
	struct delayed_work work;
	unsigned int interval;
	struct i2c_client	*client;
	struct cw2015_platform_data *info;
	int bat_status;
	int bat_voltage;
	int bat_capacity;
	int bat_rrt;
};


static enum power_supply_property rk29_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static struct cw2015_device_info *g_cw2015;

static int cw2015_verify_update_battery_info(void)
{
	int ret = 0;
	int i;
	char value = 0;
	char buffer[SIZE_BATINFO*2];
	struct i2c_client *client = NULL;
	char reg_mode_value = 0;


	if(NULL == g_cw2015)
		return -1;

	client = to_i2c_client(g_cw2015->dev);

	/* make sure not in sleep mode */
	ret = i2c_smbus_read_byte_data(client, REG_MODE);
	if(ret < 0) {
		dev_err(&client->dev, "Error read mode\n");
		return ret;
	}

	value = ret;
	reg_mode_value = value; /* save MODE value for later use */
	if((value & MODE_SLEEP_MASK) == MODE_SLEEP) {
		dev_err(&client->dev, "Error, device in sleep mode, cannot update battery info\n");
		return -1;
	}

	/* update new battery info */
	for(i=0; i<SIZE_BATINFO; i++) {
		ret = i2c_smbus_write_byte_data(client, REG_BATINFO+i, cw2015_bat_config_info[i]);
		if(ret < 0) {
			dev_err(&client->dev, "Error update battery info @ offset %d, ret = 0x%x\n", i, ret);
			return ret;
		}
	}

	/* readback & check */
	for(i=0; i<SIZE_BATINFO; i++) {
		ret = i2c_smbus_read_byte_data(client, REG_BATINFO+i);
		if(ret < 0) {
			dev_err(&client->dev, "Error read origin battery info @ offset %d, ret = 0x%x\n", i, ret);
			return ret;
		}

		buffer[i] = ret;
	}

	if(0 != memcmp(buffer, cw2015_bat_config_info, SIZE_BATINFO)) {
		dev_info(&client->dev, "battery info NOT matched, after readback.\n");
		return -1;
	} else {
		dev_info(&client->dev, "battery info matched, after readback.\n");
	}

	/* set 2015 to use new battery info */
	ret = i2c_smbus_read_byte_data(client, REG_CONFIG);
	if(ret < 0) {
		dev_err(&client->dev, "Error to read CONFIG\n");
		return ret;
	}
	value = ret;
	  
	value |= CONFIG_UPDATE_FLG;/* set UPDATE_FLAG */
	value &= 0x7;  /* clear ATHD */
	value |= ATHD; /* set ATHD */
	
	ret = i2c_smbus_write_byte_data(client, REG_CONFIG, value);
	if(ret < 0) {
		dev_err(&client->dev, "Error to update flag for new battery info\n");
		return ret;
	}
	
	/* check 2015 for ATHD&update_flag */
	ret = i2c_smbus_read_byte_data(client, REG_CONFIG);
	if(ret < 0) {
		dev_err(&client->dev, "Error to read CONFIG\n");
		return ret;
	}
	value = ret;
	
	if (!(value & CONFIG_UPDATE_FLG)) {
	  dev_info(&client->dev, "update flag for new battery info have not set\n");
	}
	if ((value & 0xf8) != ATHD) {
	  dev_info(&client->dev, "the new ATHD have not set\n");
	}	  

	reg_mode_value &= ~(MODE_RESTART);  /* RSTART */
	ret = i2c_smbus_write_byte_data(client, REG_MODE, reg_mode_value|MODE_RESTART);
	if(ret < 0) {
		dev_err(&client->dev, "Error to restart battery info1\n");
		return ret;
	}
	ret = i2c_smbus_write_byte_data(client, REG_MODE, reg_mode_value|0);
	if(ret < 0) {
		dev_err(&client->dev, "Error to restart battery info2\n");
		return ret;
	}
	return 0;
}

static int cw2015_init_charger(void)
{
        int cnt = 0;
        int i = 0;
	int ret = 0;
	char value = 0;
	char buffer[SIZE_BATINFO*2];
	short value16 = 0;
	struct i2c_client *client = NULL;

	if(NULL == g_cw2015)
		return -1;

	client = to_i2c_client(g_cw2015->dev);

#if FORCE_WAKEUP_CHIP
	value = MODE_SLEEP;
#else

	/* check if sleep mode, bring up */
	ret = i2c_smbus_read_byte_data(client, REG_MODE);
	if(ret < 0) {
		dev_err(&client->dev, "read mode\n");
		return ret;
	}

	value = ret;
#endif

	if((value & MODE_SLEEP_MASK) == MODE_SLEEP) {
                /* do wakeup cw2015 */
                ret = i2c_smbus_write_byte_data(client, REG_MODE, MODE_NORMAL);
                if(ret < 0) {
                        dev_err(&client->dev, "Error update mode\n");
                        return ret;
                }
                /* check 2015 if not set ATHD */
                ret = i2c_smbus_read_byte_data(client, REG_CONFIG);
                if(ret < 0) {
                        dev_err(&client->dev, "Error to read CONFIG\n");
                        return ret;
                }
                value = ret;
                
                if ((value & 0xf8) != ATHD) {
                        dev_info(&client->dev, "the new ATHD have not set\n");
                        value &= 0x7;  /* clear ATHD */
                        value |= ATHD; 
                        /* set ATHD */
                        ret = i2c_smbus_write_byte_data(client, REG_CONFIG, value);
                        if(ret < 0) {
                                dev_err(&client->dev, "Error to set new ATHD\n");
                                return ret;
                        }
                }
            

                /* check 2015 for update_flag */
                ret = i2c_smbus_read_byte_data(client, REG_CONFIG);
                if(ret < 0) {
                        dev_err(&client->dev, "Error to read CONFIG\n");
                        return ret;
                }
                value = ret;  	    	 
                /* not set UPDATE_FLAG,do update_battery_info  */
                if (!(value & CONFIG_UPDATE_FLG)) {
                        dev_info(&client->dev, "update flag for new battery info have not set\n");
                        cw2015_verify_update_battery_info();
                }

  	        /* read origin info */
                for(i=0; i<SIZE_BATINFO; i++) {
                        ret = i2c_smbus_read_byte_data(client, REG_BATINFO+i);
                        if(ret < 0) {
                                dev_err(&client->dev, "Error read origin battery info @ offset %d, ret = 0x%x\n", i, ret);
                                return ret;
                        }
                        buffer[i] = ret;
                }

                if(0 != memcmp(buffer, cw2015_bat_config_info, SIZE_BATINFO)) {
                        //dev_info(&client->dev, "battery info NOT matched.\n");
                        /* battery info not matched,do update_battery_info  */
                        cw2015_verify_update_battery_info();
                } else {
                        //dev_info(&client->dev, "battery info matched.\n");
                }
                
		/* do wait valide SOC, if the first time wakeup after poweron */
		ret = i2c_smbus_read_word_data(client, REG_SOC);
		if(ret < 0) {
			dev_err(&client->dev, "Error read init SOC\n");
			return ret;
		}   
		value16 = ret;		    
		
		while ((value16 == 0xff)&&(cnt < 10000)) {     //SOC value is not valide or time is not over 3 seconds
                        ret = i2c_smbus_read_word_data(client, REG_SOC);		  
                        if(ret < 0) {
                                dev_err(&client->dev, "Error read init SOC\n");
                                return ret;
                        }   
                        value16 = ret;
                        cnt++;
                }
                
        }

	return 0;
}

int cw2015_gasgauge_quickstart(void)
{
        int ret = 0;
        struct i2c_client *client = NULL;
        
        if(NULL == g_cw2015)
                return -1;
        
        client = to_i2c_client(g_cw2015->dev);
        
        ret = i2c_smbus_write_byte_data(client, REG_MODE, MODE_QUICK_START|MODE_NORMAL);
        if(ret < 0) {
                dev_err(&client->dev, "Error quick start1\n");
                return ret;
        }
        
        ret = i2c_smbus_write_byte_data(client, REG_MODE, MODE_NORMAL);
        if(ret < 0) {
                dev_err(&client->dev, "Error quick start2\n");
                return ret;
        }
        return 1;
}

int cw2015_gasgauge_get_mvolts(void)
{
	int ret = 0;
	short ustemp =0,ustemp1 =0,ustemp2 =0,ustemp3 =0;
	int voltage;
	struct i2c_client *client = NULL;
	//printk("%s\n", __FUNCTION__);
	if(NULL == g_cw2015)
		return -1;

	client = to_i2c_client(g_cw2015->dev);

	ret = i2c_smbus_read_word_data(client, REG_VCELL);
	if(ret < 0) {
		dev_err(&client->dev, "Error read VCELL\n");
		return -1;
	}
	ustemp = ret;
	ustemp = cpu_to_be16(ustemp);
	
	ret = i2c_smbus_read_word_data(client, REG_VCELL);
	if(ret < 0) {
		dev_err(&client->dev, "Error read VCELL\n");
		return -1;
	}
	ustemp1 = ret;
	ustemp1 = cpu_to_be16(ustemp1);
	
	ret = i2c_smbus_read_word_data(client, REG_VCELL);
	if(ret < 0) {
		dev_err(&client->dev, "Error read VCELL\n");
		return -1;
	}
	ustemp2 = ret;
	ustemp2 = cpu_to_be16(ustemp2);
	
	if(ustemp >ustemp1)
	{	 
	   ustemp3 =ustemp;
		  ustemp =ustemp1;
		 ustemp1 =ustemp3;
        }
	if(ustemp1 >ustemp2)
	{
	   ustemp3 =ustemp1;
		 ustemp1 =ustemp2;
		 ustemp2 =ustemp3;
	}	
	if(ustemp >ustemp1)
	{	 
	   ustemp3 =ustemp;
		  ustemp =ustemp1;
		 ustemp1 =ustemp3;
        }			

	/* 1 voltage LSB is 305uV, ~312/1024mV */
	// voltage = value16 * 305 / 1000;
	voltage = ustemp1 * 312 / 1024; 
	return voltage;
}

int cw2015_gasgauge_get_capacity(void)
{
	int ret = 0;
	short value16 = 0;
	int soc;
	struct i2c_client *client = NULL;
	//printk("%s\n", __FUNCTION__);
	if(NULL == g_cw2015)
		return -1;

	client = to_i2c_client(g_cw2015->dev);
	ret = i2c_smbus_read_word_data(client, REG_SOC);
	if(ret < 0) {
		dev_err(&client->dev, "Error read SOC\n");
		return -1;
	}
	
	
	
	value16 = ret;
	value16 = cpu_to_be16(value16);
	
	if (sitemp >=0xFF00) {
                sitemp =value16;
	} else {
                if ((abs(sitemp -value16) <=0xFF) && (value16 >0xFF) && (value16 <0x6300)) {
                        value16 =sitemp;  //99%以下1%以上单位差值处理，防止soc来回跳
                } 
                else 
                {
                        if ((value16 == 0)&&(sitemp >= 0x0200)) {       //返回值为0%时的处理
                             value16 =sitemp;
                        } 
                        else 
                        {
                          
                           sitemp =value16;
                        
                           if ((value16 <= 0xFF)&&(value16 >0)) {         //1%到1%以下同时大于0%时，SOC四舍五入
                                   sitemp =0;
                                   value16 =0x0100;
                           }
                        
                           if (value16 >= 0x6300) {
                                   sitemp =0x6380;
                           }
                        }
                        
      
                }
        }
 
  soc =  value16 >> 8; 
  
	//dev_info(&client->dev, "read SOC %d%%. value16 0x%x\n", soc, value16);
#if 0	
	if((soc == 0)&&(cw2015_gasgauge_get_mvolts() >= 3850))
			{		  
			   iiii++;
			   if(iiii==60*30)//30分钟
			   	{
			   		  cw2015_gasgauge_quickstart();      //如果端压在20%的电压值时SOC还报0，QSTRT一次
			   			iiii=0;
			   	}	
		} else{
		  iiii =0;
		}
	if((soc == 99) ||(soc == 98))
		{
				//启动定时器，每间隔10分钟读一次soc，读三次如果都是99，直接启动QSTRT
				//cw2015_gasgauge_quickstart跳变到100%，如果这个时候拔掉电池按道理不会
			  //立马直接跳变到99，因为cw2015芯片内部有算法会控制时间。
			  
			   iiii++;
			   if(iiii==60*10)//30分钟自动变为100%
			   	{
			   		  cw2015_gasgauge_quickstart();
			   			iiii=0;
			   	}	
		} else{
		  iiii=0;
		}
#endif		

	return soc;
}

int cw2015_gasgauge_get_rrt(void)
{
	int ret = 0;
	short value16 = 0;
	int rrt;
	struct i2c_client *client = NULL;
	if(NULL == g_cw2015)
		return -1;

	client = to_i2c_client(g_cw2015->dev);

	ret = i2c_smbus_read_word_data(client, REG_RRT_ALERT);
	if(ret < 0) {
		dev_err(&client->dev, "Error read RRT\n");
		return -1;
	}
	value16 = ret;
	value16 = cpu_to_be16(value16);
	rrt = value16 & 0x1fff;	
	//dev_info(&client->dev, "read RRT %d%%. value16 0x%x\n", rrt, value16);
	return rrt;
}

int cw2015_gasgauge_get_alt(void)
{
	int ret = 0;
	short value16 = 0;
	int alt;
	char value=0;
	struct i2c_client *client = NULL;

	if(NULL == g_cw2015)
		return -1;

	client = to_i2c_client(g_cw2015->dev);

	ret = i2c_smbus_read_word_data(client, REG_RRT_ALERT);
	if(ret < 0) {
		dev_err(&client->dev, "Error read ALT\n");
		return -1;
	}
	value16 = ret;
	value16 = cpu_to_be16(value16);
	alt = value16 >>15;

	//dev_info(&client->dev, "read RRT %d%%. value16 0x%x\n", alt, value16);
	
	value = (char)(value16 >>8);
	value = value&0x7f;
	ret = i2c_smbus_write_byte_data(client, REG_RRT_ALERT, value);
	if(ret < 0) {
		 dev_err(&client->dev, "Error to clear ALT\n");
                 return ret;
	}	
		
	return alt;
}


static enum power_supply_property axp_battery_props[] = {

	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
        POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
};

int cw2015_battery_status(void)
{
	struct i2c_client *client = NULL;
	int status;
        struct cw2015_platform_data *pdata = g_cw2015->info;
	if(NULL == g_cw2015)
		return -1;

	client = to_i2c_client(g_cw2015->dev);
        if(gpio_get_value(pdata->dc_det_pin)== pdata->dc_det_level) {
                if(g_cw2015-> bat_capacity >=100) {
                        status=POWER_SUPPLY_STATUS_FULL;
                } else {
                        status=POWER_SUPPLY_STATUS_CHARGING; 
                }
                
        } else	{
                status = POWER_SUPPLY_STATUS_NOT_CHARGING;
        }
        return status;	
}			

static int cw2015_battery_get_property(struct power_supply *psy,
           enum power_supply_property psp,
           union power_supply_propval *val)
{
        int ret = 0;
        switch (psp) {
                case POWER_SUPPLY_PROP_STATUS:
                        val->intval = g_cw2015 -> bat_status;
                        break;

                case POWER_SUPPLY_PROP_HEALTH:
                        val->intval= POWER_SUPPLY_HEALTH_GOOD;
                        break;
                case POWER_SUPPLY_PROP_PRESENT:
                        val->intval = g_cw2015->bat_voltage <= 0 ? 0 : 1;
                        break;

		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
                        val->intval = g_cw2015->bat_voltage;
                        break;
        
                case POWER_SUPPLY_PROP_CAPACITY:
                        val->intval = g_cw2015->bat_capacity;
                        break;  
    
		case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
                        val->intval = g_cw2015->bat_rrt;			
                        break;
    
 		case POWER_SUPPLY_PROP_TECHNOLOGY:
                        val->intval = POWER_SUPPLY_TECHNOLOGY_LION;	
                        break;
#if 1
		case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
						val->intval = BATT_MAX_VOL_VALUE;
                        break;

		case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
                        val->intval = BATT_ZERO_VOL_VALUE;
                        break;
#endif
                default:
                        ret = -EINVAL;
                        break;
        }
        return ret;
}

#if 0
static irqreturn_t cw2015_interrupt(int irq, void *dev_id)
{
        return IRQ_HANDLED;
}
#endif

static int rk29_ac_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	int ret = 0; 
	struct cw2015_platform_data *pdata = g_cw2015->info;
	switch (psp) {	
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_MAINS){
			if(gpio_get_value(pdata->dc_det_pin)== pdata->dc_det_level)
				val->intval = 1;
			else
				val->intval = 0;	
		}	
		break;
		
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}


static void cw2015_powersupply_init(struct cw2015_device_info *data)
{
	data->bat.name = "battery";
	data->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	data->bat.properties = axp_battery_props;
	data->bat.num_properties = ARRAY_SIZE(axp_battery_props);
	data->bat.get_property = cw2015_battery_get_property;

	data->ac.name = "ac";
	data->ac.type = POWER_SUPPLY_TYPE_MAINS;
	data->ac.properties = rk29_ac_props;
	data->ac.num_properties = ARRAY_SIZE(rk29_ac_props);
	data->ac.get_property = rk29_ac_get_property;
}

		

static void cw2015_battery_update_status(struct cw2015_device_info *data)
{
        int ret;
	if(NULL == g_cw2015) {
	        return;
        }
        ret = cw2015_battery_status();
        if (ret >= 0) {
	       g_cw2015->bat_status = ret;
        }

        ret = cw2015_gasgauge_get_mvolts();
        if (ret >= 0) {
                 g_cw2015->bat_voltage=ret * 1000;
        }

        ret = cw2015_gasgauge_get_capacity();
        if (ret >= 0) {
	       g_cw2015->bat_capacity = ret;
        }
         
        ret = cw2015_gasgauge_get_rrt();
        if (ret >= 0) {
                g_cw2015->bat_rrt = ret;
        }
	power_supply_changed(&data->bat);

	//printk("g_cw2015->bat_status=%d,g_cw2015->bat_voltage=%d mV,g_cw2015->bat_capacity=%d%%,g_cw2015->bat_rrt=%d Minute\n",g_cw2015->bat_status,g_cw2015->bat_voltage,g_cw2015->bat_capacity,g_cw2015->bat_rrt);
	
}

static void cw2015_battery_work(struct work_struct *work)
{
	struct cw2015_device_info *data = container_of(work, struct cw2015_device_info, work.work); 
	cw2015_battery_update_status(data);
	/* reschedule for the next time */
	schedule_delayed_work(&data->work, data->interval);
}

   
static int cw2015_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int ret;
	struct cw2015_device_info *data;
	struct cw2015_platform_data *pdata;
	pdata = client->dev.platform_data;
	
	//printk("%s %d +++\n",__FUNCTION__,__LINE__);

	if (!(data = kzalloc(sizeof(*data), GFP_KERNEL))){
		printk("failed to allocate device info data\n");
		ret = -ENOMEM;
		goto batt_failed_2;
	}

	// Init real i2c_client
	i2c_set_clientdata(client, data);
	data->dev = &client->dev;
	data->client = client;
	/* 4 seconds between monotor runs interval */
	data->interval = msecs_to_jiffies(1 * 1000);
        data->bat_capacity = 50;
	data->info = pdata;
	g_cw2015 = data;
	
	ret = cw2015_init_charger();
	if(ret < 0) {
		return ret;	
        }
	cw2015_powersupply_init(data);
        ret = power_supply_register(&client->dev, &data->bat);
	if (ret) {
		dev_err(&client->dev, "failed to register battery\n");
		goto batt_failed_4;
	}
	ret = power_supply_register(&client->dev, &data->ac);
	if (ret) {
			dev_err(&client->dev, "failed to register ac\n");
			goto batt_failed_4;
	}	
	INIT_DELAYED_WORK(&data->work, cw2015_battery_work);
	schedule_delayed_work(&data->work, data->interval);
	
	g_cw2015 = data;

	if(pdata->dc_det_pin) {
		ret = gpio_request(pdata->dc_det_pin, NULL);
		if (ret != 0) {
			gpio_free(pdata->dc_det_pin);
			
			return -EIO;
		}
		gpio_direction_input(pdata->dc_det_pin);
	}		
	if(pdata->batt_low_pin) {
	        ret = gpio_request(pdata->batt_low_pin, NULL);
        	if (ret != 0) {
	        	gpio_free(pdata->batt_low_pin);
		        return -EIO;
	        }
        	gpio_direction_input(pdata->batt_low_pin);
	}		

	printk("%s ok. %d ---\n",__FUNCTION__,__LINE__);
		return 0;					//return Ok

	batt_failed_4:
	        kfree(data);
	batt_failed_2:
		return ret;
}

static int cw2015_remove(struct i2c_client *client)
{
	struct cw2015_device_info *data = i2c_get_clientdata(client);
	kfree(data);
	g_cw2015 = NULL;
	return 0;
}

static int cw2015_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct cw2015_device_info *data = i2c_get_clientdata(client);
        cancel_delayed_work(&data->work);
	return 0;
}

static int cw2015_resume(struct i2c_client *client)
{
	struct cw2015_device_info *data = i2c_get_clientdata(client);
        schedule_delayed_work(&data->work, data->interval);
	return 0;
}

static const struct i2c_device_id cw2015_id[] = {
	{ MYDRIVER, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, cw2015_id);

static struct i2c_driver cw2015_driver = {
	.driver = {
		.name	= MYDRIVER,
	},
	.probe			= cw2015_probe,
	.remove			= cw2015_remove,
	.suspend		= cw2015_suspend,
	.resume			= cw2015_resume,
	.id_table		= cw2015_id,
};

static int __init cw2015_init(void)
{
	return i2c_add_driver(&cw2015_driver);
}

static void __exit cw2015_exit(void)
{
	i2c_del_driver(&cw2015_driver);
}

MODULE_DESCRIPTION("cw2015 i2c battery gasgauge driver");
MODULE_LICENSE("public");

module_init(cw2015_init);
module_exit(cw2015_exit);


