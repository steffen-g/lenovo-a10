#ifndef __EC_DEV_H
#define __EC_DEV_H

#include <linux/miscdevice.h>
#include <linux/power_supply.h>

struct ec_private_data;

enum ec_bus_type{
	EC_BUS_TYPE_INVALID = 0,		
	EC_BUS_TYPE_I2C,
	EC_BUS_TYPE_SPI,
	EC_BUS_TYPE_SDIO,	
	EC_BUS_TYPE_NUM_ID,
};


enum ec_id {
	EC_ID_NULL,		
	EC_ID_FIRMWARE,
	EC_ID_KEY,
	EC_ID_TP,
	EC_ID_PS2,	
	EC_ID_BATTERY,	
	EC_ID_LED,	
	EC_ID_CHARGE,	
	EC_ID_GPIO,
	EC_ID_ADC,
	EC_NUM_ID
};

struct ec_firmware{
	char *fw_buf;
	char fw_version[10];	
	int fw_size;
	int fw_status;
};

struct ec_key{	
	struct input_dev *input_dev;
	int key_code;
	int key_value;
	int key_status;
};

struct ec_tp{	
	struct input_dev *input_dev;
	int tp_code;
	int tp_x;
	int tp_y;
	int tp_pressure;
	int tp_status;
	int tp_last_status;
	int range[2];	
};

struct ec_ps2{	
	struct input_dev *input_dev;
	int ps2_code;
	int ps2_value;
	int ps2_status;
};

struct ec_battery{	
	struct power_supply bat;
	struct power_supply usb;
	struct power_supply ac;
	struct power_supply bk_bat;
	int bat_online;
	int bat_status;
	int bat_health;
	int bat_level;
	int bat_temp;
	int bat_voltage;
};


struct ec_led{	
	struct input_dev *input_dev;
	int led_code;
	int color;
	int delay;
};


struct ec_charge{
	int chg_en;
	int chg_current;
	int chg_status;	
};

struct ec_gpio{
	struct gpio_chip chip;
};


struct ec_adc{	
	struct adc_host	*adc;
	int adc_chn;
	int adc_code;
};


struct ec_flag{	
	atomic_t debug_flag;	
	int key_flag;
};

struct ec_platform_data {
	char *name;
	int irq;
	int irq_enable;         //if irq_enable=1 then use irq else use polling  
	int poll_delay_ms;      //polling	
	int reg_size;	
	int ap_wakeup_ec;
	int ec_wakeup_ap;	
	int irq_trig;		//intterupt trigger
	int wake_trig;		//wakeup trigger
	int (*init_platform_hw)(struct ec_platform_data *pdata);
	void (*exit_platform_hw)(struct ec_platform_data *pdata);
	int (*power_on)(void);
	int (*power_off)(void);
};

struct ec_operate {
	char *name;
	int ec_id;
	char slave_addr;
	int bus_type;
	
	int (*active)(struct ec_private_data *ec, int enable);
	int (*init)(struct ec_private_data *ec);
	int (*deinit)(struct ec_private_data *ec);
	int (*handle)(struct ec_private_data *ec);
	int (*suspend)(struct ec_private_data *ec);
	int (*resume)(struct ec_private_data *ec);	
	struct miscdevice *misc_dev;
};

struct ec_private_data {
	struct device *dev;	
	int (*read_dev)(struct ec_private_data *ec, unsigned short reg,
			int bytes, void *dest, int reg_size);
	int (*write_dev)(struct ec_private_data *ec, unsigned short reg,
			 int bytes, void *src, int reg_size);
	void *control_data;
	int irq;
	
	struct ec_flag flags;
	struct input_dev *input_dev;
	struct ec_firmware fw;
	struct ec_key key;
	struct ec_tp tp;
	struct ec_ps2 ps2;
	struct ec_led led;	
	struct ec_battery bat;
	struct ec_charge chg;
	struct ec_gpio gpio;
	struct ec_adc adc;
	
	struct work_struct work;
	struct delayed_work delaywork;
	struct mutex ec_lock;
	struct mutex io_lock;	
	struct mutex wakeup_lock;
	struct mutex operation_mutex;
	struct mutex data_mutex;
	struct wake_lock ec_wakelock;
	struct ec_platform_data *pdata;
	struct ec_operate *ops[EC_NUM_ID]; 
	struct file_operations fops;
	struct miscdevice miscdev;

};


#if 1
#define DBG_EC(x...) if(!!atomic_read(&ec->flags.debug_flag)) printk(x)
#else
#define DBG_EC(x...)
#endif

//top reg
#define EC_REG_CHIPID			0x00
#define EC_REG_FW_VER			0x01
#define EC_REG_TOP_STATUS		0x02
#define EC_REG_TOP_CTRL			0x03


//key reg
#define EC_REG_KEY_CODE			0x10
#define EC_REG_KEY_STATUS		0x11
#define EC_REG_KEY_CTRL			0x12

//tp reg
#define EC_REG_TP_CODE			0x20
#define EC_REG_TP_STATUS		0x21
#define EC_REG_TP_CTRL			0x22

//ps2 reg
#define EC_REG_PS2_CODE			0x30
#define EC_REG_PS2_STATUS		0x31
#define EC_REG_PS2_CTRL			0x32


//bat reg
#define EC_REG_BAT_LEVEL		0x40
#define EC_REG_BAT_STATUS		0x41
#define EC_REG_BAT_ONLINE		0x42
#define EC_REG_BAT_VOL			0x43
#define EC_REG_BAT_TEMP			0x44
#define EC_REG_BAT_HEALTH		0x45
#define EC_REG_BAT_CTRL			0x46


//led reg
#define EC_REG_LED_CODE			0x50
#define EC_REG_LED_STATUS		0x51
#define EC_REG_LED_TIME			0x52
#define EC_REG_LED_CTRL			0x53



//charge reg





#define EC_ENABLE	1
#define	EC_DISABLE	0
#define EC_UNKNOW_DATA	-1


#define GSENSOR_IOCTL_MAGIC			'a'
#define GSENSOR_IOCTL_KEYBOARD			_IOW(GSENSOR_IOCTL_MAGIC, 0x11, int[2])


#define EC_IOCTL_MAGIC				'e'
#define EC_IOCTL_KEYBOARD			GSENSOR_IOCTL_KEYBOARD
#define EC_IOCTL_TEST				_IOW(EC_IOCTL_MAGIC, 0x01, int)



extern int ec_device_init(struct ec_private_data *ec, int type, int irq);
extern void ec_device_exit(struct ec_private_data *ec);
extern int ec_register_slave(struct ec_private_data *ec,
			struct ec_platform_data *slave_pdata,
			struct ec_operate *(*get_ec_ops)(void));
extern int ec_unregister_slave(struct ec_private_data *ec,
			struct ec_platform_data *slave_pdata,
			struct ec_operate *(*get_ec_ops)(void));
extern int ec_reg_read(struct ec_private_data *ec, unsigned short reg);
extern int ec_reg_write(struct ec_private_data *ec, unsigned short reg,
		     unsigned short val);
extern int ec_bulk_read(struct ec_private_data *ec, unsigned short reg,
		     int count, unsigned char *buf);
extern int ec_bulk_read_normal(struct ec_private_data *ec, int count, unsigned char *buf, int rate);
extern int ec_bulk_write(struct ec_private_data *ec, unsigned short reg,
		     int count, unsigned char *buf);
extern int ec_bulk_write_normal(struct ec_private_data *ec, int count, unsigned char *buf, int rate);
extern int ec_set_biec(struct ec_private_data *ec, unsigned short reg,
		    unsigned short mask, unsigned short val);
extern int ec_device_suspend(struct ec_private_data *ec);

extern int ec_device_resume(struct ec_private_data *ec);


#endif
