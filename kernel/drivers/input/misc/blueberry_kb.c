/* 
 * EC base keyboard driver.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/err.h>
#include <linux/leds.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/gpio_event.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <linux/gpio.h>
#include <linux/ioctl.h>
#include <linux/uaccess.h>
#include <linux/earlysuspend.h>
#include <linux/miscdevice.h>
#include <mach/board.h>
#include "blueberry_kb.h"

#define arm_notify_ec_pin	RK30_PIN1_PA7
#define REPORT_ID_KB	0x01
#define KB_INPUT_DEV_NAME	"A10 KEYBOARD"

#define DEBUG		1
#if 1
#define DBG_EC(x...) if(!!atomic_read(&bbkb->debug_flag)) printk(x)
#else
#define DBG_EC(x...)
#endif


//#define BLUEBERRY_KB_I2C_READ_DATA
#define BLUEBERRY_KB_I2C_WRITE_DATA
#define BLUEBERRY_KB_SMBUS_READ_BYTE_BLOCK

static struct miscdevice blueberry_ec_device;

static struct blueberry_kb *g_bbkb;     

extern void rk29_send_power_key(int state);

static int pwrkey_send_once = 0;

#ifdef BLUEBERRY_KB_I2C_READ_DATA
static int blueberry_kb_i2c_read_data(struct i2c_client *client, char *buf, int length)
{
        struct i2c_msg msgs[] = {
                {
                        .addr  =  client->addr,
                        .flags  =  0,
                        .len  =  1,
                        .buf  =  buf,
                                                .scl_rate = 400 * 1000,
                },
                {
                        .addr  =  client->addr,
                        .flags  = I2C_M_RD,
                        .len  =  length,
                        .buf  =  buf,
                                                .scl_rate = 400 * 1000,
                },
        };

        if(i2c_transfer(client->adapter, msgs, 2) < 0){
                pr_err("blueberry_kb_i2c_read_data: transfer error\n");
                return EIO;
        }
        else
        return 0;
}
#endif

#ifdef BLUEBERRY_KB_I2C_WRITE_DATA
static int blueberry_kb_i2c_write_data(struct i2c_client *client, char *buf, int length)
{
        struct i2c_msg msgs[] = {
                {
                        .addr = client->addr,
                        .flags = 0,
                        .len = length,
                        .buf = buf,
                                                .scl_rate = 400 * 1000,
                },
        };

        if (i2c_transfer(client->adapter, msgs, 1) < 0) {
                        pr_err("blueberry_kb_i2c_write_data: transfer error\n");
                return -EIO;
        } else
                return 0;
}
#endif

#ifdef BLUEBERRY_KB_SMBUS_READ_BYTE_BLOCK
static int blueberry_kb_smbus_read_byte_block(struct i2c_client *client,
                unsigned char reg_addr, unsigned char *data, unsigned char len)
{
        s32 dummy;
        dummy = i2c_smbus_read_i2c_block_data(client, reg_addr, len, data);
        if (dummy < 0)
                return -EPERM;
        return 0;
}
#endif


int blueberry_bat_shipping_mode_ctrl(void)  
{
    struct blueberry_kb *bbkb  = g_bbkb;
    unsigned char ec_cmd[3] = {0,0,0};
    int ret = 0;
    
    if(!bbkb)
        { 
               printk("%s:bbkb is null\n",__func__); 
               return 0; 
       } 
 
     if(!bbkb->ship_mode) 
             return 0; 

       if(bbkb->ship_mode)      
            ec_cmd[2] = SHIP_MODE; 
      // else     
         //   ec_cmd[2] = AMBER_ON; 
        
       ec_cmd[0] = 0x12; 
       ec_cmd[1] = 0x00; 
       ret = blueberry_kb_i2c_write_data(bbkb->client, ec_cmd, 3); 
       if(ret) { 
               pr_err("blueberry_kb: EC_IOCTL_FORCE_CHARGE failed.\n"); 
               return ret; 
       } 

       printk("%s excute OK :shipping mode %s\n",__func__, (bbkb->ship_mode)?"enable":"disable"); 
 
       return 0; 
 
} 
 





static void blueberry_kb_early_suspend(struct early_suspend *handler)
{
	struct blueberry_kb *bbkb = (struct blueberry_kb *) container_of(handler, struct blueberry_kb, early_drv);
	unsigned char ec_cmd[3];
	int ret;
	pr_err("enter blueberry_kb_early_suspend.\n");
	bbkb->arm_early_suspend = 1;
	pwrkey_send_once = 0;
	printk("suspend capskey status = %d\n", bbkb->capskey_status);
	ec_cmd[0] = 0x10;
	ec_cmd[1] = 0x00;
	ec_cmd[2] = CMD_CAPSKEY_LED_OFF;
	ret = blueberry_kb_i2c_write_data(bbkb->client, ec_cmd, 3);
	if(ret) {
		pr_err("blueberry_kb: set capskey led failed.\n");
		return;
	}
}

static void blueberry_kb_early_resume(struct early_suspend *handler)
{
	struct blueberry_kb *bbkb = (struct blueberry_kb *) container_of(handler, struct blueberry_kb, early_drv);
	unsigned char ec_cmd[3];
	int ret;
	pr_err("enter blueberry_kb_early_resume.\n");
	bbkb->arm_early_suspend = 0;
    printk("resume capskey status = %d\n", bbkb->capskey_status);
	ec_cmd[0] = 0x10;
    ec_cmd[1] = 0x00;
    ec_cmd[2] = ((bbkb->capskey_status) ? CMD_CAPSKEY_LED_ON : CMD_CAPSKEY_LED_OFF);
    msleep(2);
    ret = blueberry_kb_i2c_write_data(bbkb->client, ec_cmd, 3);
    if(ret) {
        pr_err("blueberry_kb: set capskey led failed.\n");
        return;
    }
}

static int blueberry_kb_kp_key_mapping(int x)
{
    switch (x){
        case ASUSDEC_KEYPAD_ESC:
            return KEY_BACK;

        case ASUSDEC_KEYPAD_KEY_WAVE:
            return KEY_GRAVE;

        case ASUSDEC_KEYPAD_KEY_1:
            return KEY_1;

        case ASUSDEC_KEYPAD_KEY_2:
            return KEY_2;

        case ASUSDEC_KEYPAD_KEY_3:
            return KEY_3;

        case ASUSDEC_KEYPAD_KEY_4:
            return KEY_4;

        case ASUSDEC_KEYPAD_KEY_5:
            return KEY_5;

        case ASUSDEC_KEYPAD_KEY_6:
            return KEY_6;

        case ASUSDEC_KEYPAD_KEY_7:
            return KEY_7;

        case ASUSDEC_KEYPAD_KEY_8:
            return KEY_8;

        case ASUSDEC_KEYPAD_KEY_9:
            return KEY_9;

        case ASUSDEC_KEYPAD_KEY_0:
            return KEY_0;

        case ASUSDEC_KEYPAD_KEY_MINUS:
            return KEY_MINUS;

        case ASUSDEC_KEYPAD_KEY_EQUAL:
            return KEY_EQUAL;

        case ASUSDEC_KEYPAD_KEY_BACKSPACE:
            return KEY_BACKSPACE;

        case ASUSDEC_KEYPAD_KEY_TAB:
            return KEY_TAB;

        case ASUSDEC_KEYPAD_KEY_Q:
            return KEY_Q;

        case ASUSDEC_KEYPAD_KEY_W:
            return KEY_W;

        case ASUSDEC_KEYPAD_KEY_E:
            return KEY_E;

        case ASUSDEC_KEYPAD_KEY_R:
            return KEY_R;

        case ASUSDEC_KEYPAD_KEY_T:
            return KEY_T;

        case ASUSDEC_KEYPAD_KEY_Y:
            return KEY_Y;

        case ASUSDEC_KEYPAD_KEY_U:
            return KEY_U;

        case ASUSDEC_KEYPAD_KEY_I:
            return KEY_I;

        case ASUSDEC_KEYPAD_KEY_O:
            return KEY_O;

        case ASUSDEC_KEYPAD_KEY_P:
            return KEY_P;

        case ASUSDEC_KEYPAD_KEY_LEFTBRACE:
            return KEY_LEFTBRACE;

        case ASUSDEC_KEYPAD_KEY_RIGHTBRACE:
            return KEY_RIGHTBRACE;

        case ASUSDEC_KEYPAD_KEY_BACKSLASH:
            return KEY_BACKSLASH;

        case ASUSDEC_KEYPAD_KEY_CAPSLOCK:
            return KEY_CAPSLOCK;

        case ASUSDEC_KEYPAD_KEY_A:
            return KEY_A;

        case ASUSDEC_KEYPAD_KEY_S:
            return KEY_S;

        case ASUSDEC_KEYPAD_KEY_D:
            return KEY_D;

        case ASUSDEC_KEYPAD_KEY_F:
            return KEY_F;

        case ASUSDEC_KEYPAD_KEY_G:
            return KEY_G;

        case ASUSDEC_KEYPAD_KEY_H:
            return KEY_H;

        case ASUSDEC_KEYPAD_KEY_J:
            return KEY_J;

        case ASUSDEC_KEYPAD_KEY_K:
            return KEY_K;

        case ASUSDEC_KEYPAD_KEY_L:
            return KEY_L;

        case ASUSDEC_KEYPAD_KEY_SEMICOLON:
            return KEY_SEMICOLON;

        case ASUSDEC_KEYPAD_KEY_APOSTROPHE:
            return KEY_APOSTROPHE;

        case ASUSDEC_KEYPAD_KEY_ENTER:
            return KEY_ENTER;

        case ASUSDEC_KEYPAD_KEY_LEFTSHIFT:
            return KEY_LEFTSHIFT;

        case ASUSDEC_KEYPAD_KEY_Z:
            return KEY_Z;

        case ASUSDEC_KEYPAD_KEY_X:
            return KEY_X;

        case ASUSDEC_KEYPAD_KEY_C:
            return KEY_C;

        case ASUSDEC_KEYPAD_KEY_V:
            return KEY_V;

        case ASUSDEC_KEYPAD_KEY_B:
            return KEY_B;

        case ASUSDEC_KEYPAD_KEY_N:
            return KEY_N;

        case ASUSDEC_KEYPAD_KEY_M:
            return KEY_M;

        case ASUSDEC_KEYPAD_KEY_COMMA:
            return KEY_COMMA;

        case ASUSDEC_KEYPAD_KEY_DOT:
            return KEY_DOT;

        case ASUSDEC_KEYPAD_KEY_SLASH:
            return KEY_SLASH;

        case ASUSDEC_KEYPAD_KEY_RIGHTSHIFT:
            return KEY_RIGHTSHIFT;

        case ASUSDEC_KEYPAD_KEY_LEFT:
            return KEY_LEFT;

        case ASUSDEC_KEYPAD_KEY_RIGHT:
            return KEY_RIGHT;

        case ASUSDEC_KEYPAD_KEY_UP:
            return KEY_UP;

        case ASUSDEC_KEYPAD_KEY_DOWN:
            return KEY_DOWN;

        case ASUSDEC_KEYPAD_RIGHTWIN:
            return KEY_SEARCH;

        case ASUSDEC_KEYPAD_LEFTCTRL:
            return KEY_LEFTCTRL;

        case ASUSDEC_KEYPAD_LEFTWIN:
            return 249;//KEY_HOMEPAGE;

        case ASUSDEC_KEYPAD_LEFTALT:
            return KEY_LEFTALT;

        case ASUSDEC_KEYPAD_KEY_SPACE:
            return KEY_SPACE;

        case ASUSDEC_KEYPAD_RIGHTALT:
            return KEY_RIGHTALT;

        case ASUSDEC_KEYPAD_WINAPP:
            return KEY_MENU;

        case ASUSDEC_KEYPAD_RIGHTCTRL:
            return KEY_RIGHTCTRL;

        case ASUSDEC_KEYPAD_HOME:
            return KEY_HOME;

        case ASUSDEC_KEYPAD_PAGEUP:
            return KEY_PAGEUP;

        case ASUSDEC_KEYPAD_PAGEDOWN:
            return KEY_PAGEDOWN;

        case ASUSDEC_KEYPAD_END:
            return KEY_END;

//////////////////////////////////////////////////////////////////

	case ASUSDEC_KEYPAD_F1:
		return 255;//KEY_HOME;

	case ASUSDEC_KEYPAD_F2:
		return KEY_LAUNCHER;//250

        case ASUSDEC_KEYPAD_F3:
        	return KEY_SETTING;//251

	case ASUSDEC_KEYPAD_F4:
        	return KEY_MUTE;

        case ASUSDEC_KEYPAD_F5:
	        return KEY_VOLUMEDOWN;

        case ASUSDEC_KEYPAD_F6:
        	return KEY_VOLUMEUP;

        case ASUSDEC_KEYPAD_F7:
        	return KEY_BRIGHTNESSDOWN;

        case ASUSDEC_KEYPAD_F8:
	        return KEY_BRIGHTNESSUP;

        case ASUSDEC_KEYPAD_F9:
	        return KEY_WLAN;

        case ASUSDEC_KEYPAD_F10:
        	return KEY_BLUETOOTH;

        case ASUSDEC_KEYPAD_F11:
        	return KEY_TOUCHPAD;//252

        case ASUSDEC_KEYPAD_F12:
	        return KEY_FORCE_ROTATION;//253

	case ASUSDEC_KEYPAD_INSERT:
		return KEY_WWW;

	case ASUSDEC_KEYPAD_PRINTSCREEN:
		return KEY_SCREENSHOT;//254

	case ASUSDEC_KEYPAD_DELETE:
		return KEY_DELETE;

//////////////////////////////////////////////////////////////////


        //--- JP keys
        case ASUSDEC_YEN:
            return KEY_YEN;

        case ASUSDEC_RO:
            return KEY_RO;

        case ASUSDEC_MUHENKAN:
            return KEY_MUHENKAN;

        case ASUSDEC_HENKAN:
            return KEY_HENKAN;

        case ASUSDEC_HIRAGANA_KATAKANA:
            return KEY_KATAKANAHIRAGANA;

        //--- UK keys
        case ASUSDEC_EUROPE_2:
            return KEY_102ND;

        default:
            return -1;
    }
}

static void blueberry_kb_keypad_processing(struct blueberry_kb *bbkb)
{

	unsigned char ec_cmd[3];
	int ret;

	bbkb->value = bbkb->i2c_data[2];
    bbkb->input_keycode = blueberry_kb_kp_key_mapping(bbkb->i2c_data[3]);

	if(bbkb->input_keycode > 0)
	{
            if((bbkb->input_keycode == KEY_CAPSLOCK) && (bbkb->value == 1)&&(!(bbkb->test_mode == 1)))
            {
                bbkb->capskey_status ^= 0x01;
                ec_cmd[0] = 0x10;
                ec_cmd[1] = 0x00;
                ec_cmd[2] = ((bbkb->capskey_status) ? CMD_CAPSKEY_LED_ON : CMD_CAPSKEY_LED_OFF);
                DBG_EC("blueberry_kb: bbkb->capskey_status = %d, delay 5ms to send ec cmd\n", bbkb->capskey_status);
				msleep(2);//add delay
                ret = blueberry_kb_i2c_write_data(bbkb->client, ec_cmd, 3);
                if(ret) {
                    pr_err("blueberry_kb: set capskey led failed.\n");
                    return;
                }
            }
			if((bbkb->test_mode == 1)&&(0x3a==bbkb->input_keycode)){
				return;
			}
        	DBG_EC("blueberry_kb: input_keycode = 0x%x, input_value = %d\n", bbkb->input_keycode, bbkb->value);
        	input_report_key(bbkb->input_dev, bbkb->input_keycode, bbkb->value);
	        input_sync(bbkb->input_dev);
	}else{
		pr_err("blueberry_kb: unknown keycode\n");
	}
}

static void blueberry_kb_input_work(struct work_struct *work)
{
        struct blueberry_kb *bbkb = (struct blueberry_kb *)container_of(work, struct blueberry_kb, input_work);
        int ret;

	ret = blueberry_kb_smbus_read_byte_block(bbkb->client, 0x03, bbkb->i2c_data, 4);
	if(ret)
	{
		pr_err("blueberry_kb: read keyboard i2c data failed.\n");
                goto enable_irq;
	}
	//printk("blueberry_kb: 0x%02x 0x%02x 0x%02x 0x%02x\n",bbkb->i2c_data[0],bbkb->i2c_data[1],bbkb->i2c_data[2],bbkb->i2c_data[3]);
	if(bbkb->i2c_data[1] == REPORT_ID_KB)
	{
	    if((pwrkey_send_once == 0) && (bbkb->arm_early_suspend == 1) && (bbkb->i2c_data[2] == 1))
    	{
        	rk29_send_power_key(1);
        	rk29_send_power_key(0);
			printk("blueberry_kb: rk29_send_power_key-----------\n");
			pwrkey_send_once = 1;
        	goto enable_irq;
    	}

		if(bbkb->flag_register)
		{
			blueberry_kb_keypad_processing(bbkb);
		}
		else
		{
			printk("no input device, do not report input event\n");
		}
	}
	else
		pr_err("blueberry_kb: data is not from keyboard\n");
enable_irq:
	enable_irq_wake(bbkb->client->irq);
	enable_irq(bbkb->client->irq);
	return;
}

static irqreturn_t blueberry_kb_interrupt(int irq, void *dev_id)
{
	struct blueberry_kb *bbkb = dev_id;

	disable_irq_wake(bbkb->client->irq);
    disable_irq_nosync(bbkb->client->irq);
	if(bbkb->arm_suspend_status == 1)
	{
		//rk28_send_wakeup_key();
		//printk("%s:	rk28_send_wakeup_key\n", __func__);
	}
	queue_work(bbkb->keyboard_wq, &bbkb->input_work);
    //schedule_work(&bbkb->input_work);

    return IRQ_HANDLED;
}

static void blueberry_kb_keypad_set_input_params(struct input_dev *dev)
{
    int i = 0;
    set_bit(EV_KEY, dev->evbit);
    for ( i = 0; i < 256; i++)
        set_bit(i,dev->keybit);

    //input_set_capability(dev, EV_LED, LED_CAPSL);
}


static ssize_t blueberry_kb_status_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t len)
{
	printk("blueberry_kb: blueberry_kb_status_store enter\n");
	return 0;
}

static ssize_t blueberry_kb_status_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct blueberry_kb *bbkb = i2c_get_clientdata(client);

	DBG_EC("blueberry_kb: blueberry_kb_status_show enter\n");
	DBG_EC("bbkb->kb_enable = %d\n", bbkb->kb_enable);
	DBG_EC("bbkb->tp_enable = %d\n", bbkb->tp_enable);
	DBG_EC("bbkb->tp_mode = %d\n", bbkb->tp_mode);
	DBG_EC("bbkb->test_mode = %d\n", bbkb->test_mode);
	DBG_EC("bbkb->led_flash = %d\n", bbkb->led_flash);
	DBG_EC("bbkb->amber_led_status = %d\n", bbkb->amber_led_status);
	DBG_EC("bbkb->blue_led_status = %d\n", bbkb->blue_led_status);
	DBG_EC("bbkb->bat_status = %d\n", bbkb->bat_status);
	DBG_EC("bbkb->ship_mode = %d\n", bbkb->ship_mode);
	return 0;
}

static ssize_t blueberry_kb_kb_enable_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct blueberry_kb *bbkb = i2c_get_clientdata(client);
	return sprintf(buf, "%d\n", bbkb->kb_enable > 0 ? 1 : 0);
}

static ssize_t blueberry_kb_kb_enable_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t len)
{
	struct i2c_client *client = to_i2c_client(dev);
    struct blueberry_kb *bbkb = i2c_get_clientdata(client);
	unsigned char ec_cmd[3];
	int ret;
    ec_cmd[0] = 0x10;
    ec_cmd[1] = 0x00;	

    printk("blueberry_kb: %s() enter\n", __func__);
    if (*buf == 'E') {
		ec_cmd[2] = 0x01;
		ret = blueberry_kb_i2c_write_data(bbkb->client, ec_cmd, 3);
		bbkb->kb_enable = 1;
    }else if(*buf == 'D') {
		ec_cmd[2] = 0x00;
		ret = blueberry_kb_i2c_write_data(bbkb->client, ec_cmd, 3);
		bbkb->kb_enable = 0;
    }

    return len;
}

static ssize_t blueberry_kb_debug_flag_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct blueberry_kb *bbkb = i2c_get_clientdata(client);
    return sprintf(buf, "%d\n", atomic_read(&bbkb->debug_flag) > 0 ? 1 : 0);
}

static ssize_t blueberry_kb_debug_flag_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t len)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct blueberry_kb *bbkb = i2c_get_clientdata(client);

    printk("blueberry_kb: %s() enter\n", __func__);
    if (*buf == '1') {
        atomic_set(&bbkb->debug_flag, 1);
    }else if(*buf == '0') {
        atomic_set(&bbkb->debug_flag, 0);
    }

    return len;
}

static DEVICE_ATTR(status, S_IRUGO|S_IWUSR, blueberry_kb_status_show, blueberry_kb_status_store);
static DEVICE_ATTR(kb_enable, S_IRUGO|S_IWUSR, blueberry_kb_kb_enable_show, blueberry_kb_kb_enable_store);
static DEVICE_ATTR(debug_flag, S_IRUGO|S_IWUSR, blueberry_kb_debug_flag_show, blueberry_kb_debug_flag_store);

static struct attribute *blueberry_kb_attributes[] = {
	&dev_attr_debug_flag.attr,
    &dev_attr_status.attr,
    &dev_attr_kb_enable.attr,
    NULL
};

static struct attribute_group blueberry_kb_attribute_group = {
        .attrs = blueberry_kb_attributes
};

static int blueberry_ec_open(struct inode *inode, struct file *file)
{
        int result = 0;

        pr_info("%s\n", __func__);
        return result;
}

static int blueberry_ec_release(struct inode *inode, struct file *file)
{
        int result = 0;

        pr_info("%s\n", __func__);
        return result;
}

/* ioctl - I/O control */
static long blueberry_ec_ioctl(struct file *file,
                          unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client = container_of(blueberry_ec_device.parent, struct i2c_client, dev);
	struct blueberry_kb* bbkb = (struct blueberry_kb *)i2c_get_clientdata(client);

    void __user *argp = (void __user *)arg;
    int key_ctrl[2] = {0, 0};
	int key_ctrl_capslock[2] = {0, 0};
	int ctrl[1] = {0};
    int ret = 0;
	unsigned char ec_cmd[3];
	int i = 0;

        switch (cmd) {
		case EC_IOCTL_ENABLE_TRACKING_ANGLE:
                printk("%s:EC_IOCTL_ENABLE_TRACKING_ANGLE start\n", __func__);
                mutex_lock(&bbkb->operation_mutex);
                if (copy_from_user(ctrl, argp, sizeof(ctrl)))
                {
                        ret = -EFAULT;
                        mutex_unlock(&(bbkb->operation_mutex));
                        goto error;
                }
				printk("EC_IOCTL_ENABLE_TRACKING_ANGLE: enable=%d\n", ctrl[0]);
				bbkb->tracking_angle = ctrl[0];

				if((bbkb->kb_enable != bbkb->kb_enable1) && bbkb->tracking_angle)
				{
					printk("%s: change kb status after boot complete\n", __func__);
					ec_cmd[0] = 0x10;
					ec_cmd[1] = 0x00;
					ec_cmd[2] = ((bbkb->kb_enable1) ? 1 : 0);
                    ret = blueberry_kb_i2c_write_data(bbkb->client, ec_cmd, 3);
                    if(ret) {
                        pr_err("blueberry_kb: EC_IOCTL_KEYBOARD failed.\n");
                        mutex_unlock(&(bbkb->operation_mutex));
                        return ret;
                    }
					if(bbkb->kb_enable1 == 0)
                    {
                        if(bbkb->flag_register)
                        {
                            disable_irq_nosync(bbkb->client->irq);
                            cancel_work_sync(&bbkb->input_work);
                            input_unregister_device(bbkb->input_dev);
                            bbkb->flag_register = 0;
                            bbkb->capskey_status = 0;
                            enable_irq(bbkb->client->irq);
                            printk("%s:input_unregister_device\n",__func__);
                        }
                    }
                    else
                    {
                        if(!bbkb->flag_register)
                        {
                            disable_irq_nosync(bbkb->client->irq);
                            bbkb->input_dev = input_allocate_device();
                            bbkb->input_dev->name = KB_INPUT_DEV_NAME;
                            bbkb->input_dev->phys = bbkb->client->adapter->name;
                            bbkb->input_dev->id.bustype = BUS_I2C;
                            bbkb->input_dev->dev.parent = &client->dev;

                            blueberry_kb_keypad_set_input_params(bbkb->input_dev);

                            for(i=0; i<5; i++)
                            {
                                ret = input_register_device(bbkb->input_dev);
                                if(!ret)
                                break;
                            }

                            if(i>5)
                            {
                                pr_err("blueberry_kb input_register_device failed.\n");
                                mutex_unlock(&(bbkb->operation_mutex));
                                enable_irq(bbkb->client->irq);
                                return ret;
                            }

                            bbkb->flag_register = 1;
                            enable_irq(bbkb->client->irq);

                            printk("%s:input_register_device %s\n",__func__,bbkb->input_dev->name);

                        }
                    }
					bbkb->kb_enable = bbkb->kb_enable1;
				}
                mutex_unlock(&bbkb->operation_mutex);
                printk("%s:EC_IOCTL_TEST ok\n", __func__);
                break;

        case EC_IOCTL_KB_ENABLE:
                mutex_lock(&(bbkb->operation_mutex));
                if (copy_from_user(key_ctrl, argp, sizeof(key_ctrl)))
                {
                        ret = -EFAULT;
						mutex_unlock(&(bbkb->operation_mutex));
                        goto error;
                }

				bbkb->kb_enable1 = key_ctrl[0];
				printk("EC_IOCTL_KEYBOARD key_ctrl = %d, %d, tracking_angle=%d\n", key_ctrl[0], key_ctrl[1], bbkb->tracking_angle);
                if((bbkb->kb_enable != key_ctrl[0]) && bbkb->tracking_angle)
                {
                    bbkb->kb_enable = key_ctrl[0];

					printk("EC_IOCTL_KEYBOARD bbkb->kb_enable = %d ,angle = %d\n", bbkb->kb_enable, key_ctrl[1]);

					ec_cmd[0] = 0x10;
                    ec_cmd[1] = 0x00;
					ec_cmd[2] = ((bbkb->kb_enable) ? 1 : 0);		
		        	ret = blueberry_kb_i2c_write_data(bbkb->client, ec_cmd, 3);
        			if(ret) {			
                		pr_err("blueberry_kb: EC_IOCTL_KEYBOARD failed.\n");
						mutex_unlock(&(bbkb->operation_mutex));
                		return ret;
        			}
	
					if(key_ctrl[0] == 0)
					{
						if(bbkb->flag_register)
						{
							disable_irq_nosync(bbkb->client->irq);
							cancel_work_sync(&bbkb->input_work);
							input_unregister_device(bbkb->input_dev);				
							bbkb->flag_register = 0;
							bbkb->capskey_status = 0;					
							enable_irq(bbkb->client->irq);
							printk("%s:input_unregister_device\n",__func__);
						}
						
					}
					else
					{
						if(!bbkb->flag_register)
						{		
							disable_irq_nosync(bbkb->client->irq);
							bbkb->input_dev = input_allocate_device();
							bbkb->input_dev->name = KB_INPUT_DEV_NAME;
							bbkb->input_dev->phys = bbkb->client->adapter->name;
							bbkb->input_dev->id.bustype = BUS_I2C;
							bbkb->input_dev->dev.parent = &client->dev;

							blueberry_kb_keypad_set_input_params(bbkb->input_dev);
							
							for(i=0; i<5; i++)
							{
								ret = input_register_device(bbkb->input_dev);
								if(!ret)
								break;
							}	

							if(i>5)
							{
								pr_err("blueberry_kb input_register_device failed.\n");
								mutex_unlock(&(bbkb->operation_mutex));					
								enable_irq(bbkb->client->irq);
								return ret;
							}
							
							bbkb->flag_register = 1;		
							enable_irq(bbkb->client->irq);
						
							printk("%s:input_register_device %s\n",__func__,bbkb->input_dev->name);
							
						}
					}
					
                }

                mutex_unlock(&(bbkb->operation_mutex));
                break;

        case EC_IOCTL_TEST_MODE:
                printk("%s:EC_IOCTL_TEST start\n", __func__);
				mutex_lock(&bbkb->operation_mutex);
                if (copy_from_user(ctrl, argp, sizeof(ctrl)))
                {
                        ret = -EFAULT;
						mutex_unlock(&(bbkb->operation_mutex));
                        goto error;
                }
				if(bbkb->test_mode != ctrl[0])
				{
					bbkb->test_mode = ctrl[0];
            		ec_cmd[0] = 0x12;
	        		ec_cmd[1] = 0x00;
					ec_cmd[2] = ((bbkb->test_mode) ? TEST_MODE : 0x00);
                	ret = blueberry_kb_i2c_write_data(bbkb->client, ec_cmd, 3);
                        if(ret) {
                                pr_err("blueberry_kb: EC_IOCTL_TEST failed.\n");
								mutex_unlock(&(bbkb->operation_mutex));
                                return ret;
                        }
				}
                mutex_unlock(&bbkb->operation_mutex);
                printk("%s:EC_IOCTL_TEST ok\n", __func__);
                break;

		case EC_IOCTL_FLASH_LED:
			mutex_lock(&bbkb->operation_mutex);
			if (copy_from_user(ctrl, argp, sizeof(ctrl)))
            {
            	ret = -EFAULT;
				mutex_unlock(&(bbkb->operation_mutex));
                goto error;
            }
			if(bbkb->led_flash != ctrl[0])
			{
				bbkb->led_flash = ctrl[0];
            	ec_cmd[0] = 0x10;
            	ec_cmd[1] = 0x00;
				ec_cmd[2] = ((bbkb->led_flash) ? CMD_LED_FLASH_START : CMD_LED_FLASH_END);
                ret = blueberry_kb_i2c_write_data(bbkb->client, ec_cmd, 3);
                if(ret) {
                	pr_err("blueberry_kb: EC_IOCTL_LED_FLASH failed.\n");
					mutex_unlock(&(bbkb->operation_mutex));
                    return ret;
              	}
			}
			mutex_unlock(&bbkb->operation_mutex);
			break;

	case EC_IOCTL_AMBER_LED:
                mutex_lock(&bbkb->operation_mutex);
                if (copy_from_user(ctrl, argp, sizeof(ctrl)))
                {
                        ret = -EFAULT;
						mutex_unlock(&(bbkb->operation_mutex));
                        goto error;
                }
                if((bbkb->test_mode == 1) && (bbkb->amber_led_status != ctrl[0]))
                {
                        bbkb->amber_led_status = ctrl[0];
                        ec_cmd[0] = 0x12;
                        ec_cmd[1] = 0x00;
						ec_cmd[2] = ((bbkb->amber_led_status) ? AMBER_ON : TEST_MODE);
                        ret = blueberry_kb_i2c_write_data(bbkb->client, ec_cmd, 3);
                        if(ret) {
                                pr_err("blueberry_kb: EC_IOCTL_LED_AMBER failed.\n");
								mutex_unlock(&(bbkb->operation_mutex));
                                return ret;
                        }
                }
                mutex_unlock(&bbkb->operation_mutex);
		break;

	case EC_IOCTL_BLUE_LED:
                mutex_lock(&bbkb->operation_mutex);
                if (copy_from_user(ctrl, argp, sizeof(ctrl)))
                {
                        ret = -EFAULT;
						mutex_unlock(&(bbkb->operation_mutex));
                        goto error;
                }
                if((bbkb->test_mode == 1) && (bbkb->blue_led_status != ctrl[0]))
                {
                        bbkb->blue_led_status = ctrl[0];
                        ec_cmd[0] = 0x12;
                        ec_cmd[1] = 0x00;
						ec_cmd[2] = ((bbkb->blue_led_status) ? BLUE_ON : TEST_MODE);
                        ret = blueberry_kb_i2c_write_data(bbkb->client, ec_cmd, 3);
                        if(ret) {
                                pr_err("blueberry_kb: EC_IOCTL_LED_BLUE failed.\n");
								mutex_unlock(&(bbkb->operation_mutex));
                                return ret;
                        }
                }
                mutex_unlock(&bbkb->operation_mutex);
		break;

	case EC_IOCTL_FORCE_CHARGE_MODE:
                mutex_lock(&bbkb->operation_mutex);
                if (copy_from_user(ctrl, argp, sizeof(ctrl)))
                {
                        ret = -EFAULT;
						mutex_unlock(&(bbkb->operation_mutex));
                        goto error;
                }
                if((bbkb->test_mode == 1) && (bbkb->bat_status != ctrl[0]))
                {
                        bbkb->bat_status = ctrl[0];
                        ec_cmd[0] = 0x12;
                        ec_cmd[1] = 0x00;
						ec_cmd[2] = ((bbkb->bat_status) ? CHARGE : DISCHARGE);
                        ret = blueberry_kb_i2c_write_data(bbkb->client, ec_cmd, 3);
                        if(ret) {
                                pr_err("blueberry_kb: EC_IOCTL_FORCE_CHARGE failed.\n");
								mutex_unlock(&(bbkb->operation_mutex));
                                return ret;
                        }
                }
                mutex_unlock(&bbkb->operation_mutex);
                break;

	case EC_IOCTL_SHIP_MODE:
                mutex_lock(&bbkb->operation_mutex);
                if (copy_from_user(ctrl, argp, sizeof(ctrl)))
                {
                        ret = -EFAULT;
						mutex_unlock(&(bbkb->operation_mutex));
                        goto error;
                }
                if((bbkb->test_mode == 1) && (bbkb->ship_mode != ctrl[0]))
                {
                        printk("EC_IOCTL_SHIP_MODE bbkb->test_mode = %d, ctrl[0] = %d\n", bbkb->test_mode, ctrl[0]);
                        bbkb->ship_mode = ctrl[0];
                  #if 0 
                        ec_cmd[0] = 0x12;
                        ec_cmd[1] = 0x00;
                        ec_cmd[2] = SHIP_MODE;
                        ret = blueberry_kb_i2c_write_data(bbkb->client, ec_cmd, 3);
                        if(ret) {
                                pr_err("blueberry_kb: EC_IOCTL_FORCE_CHARGE failed.\n");
								mutex_unlock(&(bbkb->operation_mutex));
                                return ret;
                        }
                    #endif  

                  
                }
                mutex_unlock(&bbkb->operation_mutex);
                break;
// leave tp control in tp driver
/*
	case EC_IOCTL_TP_ENABLE:
                mutex_lock(&bbkb->operation_mutex);
                if (copy_from_user(ctrl, argp, sizeof(ctrl)))
                {
                        ret = -EFAULT;
						mutex_unlock(&(bbkb->operation_mutex));
                        goto error;
                }
                if(bbkb->tp_enable != ctrl[0])
                {
                        bbkb->tp_enable = ctrl[0];
						ec_cmd[0] = 0x0d;
						ec_cmd[1] = 0x00;
                        ec_cmd[2] = bbkb->tp_enable;
                        ret = blueberry_kb_i2c_write_data(bbkb->client, ec_cmd, 3);
                        if(ret) {
                                pr_err("blueberry_kb: EC_IOCTL_TP_ENABLE failed.\n");
								mutex_unlock(&(bbkb->operation_mutex));
                                return ret;
                        }
                }
                mutex_unlock(&bbkb->operation_mutex);
                break;

    case EC_IOCTL_TP_MODE:
                mutex_lock(&bbkb->operation_mutex);
                if (copy_from_user(ctrl, argp, sizeof(ctrl)))
                {
                        ret = -EFAULT;
						mutex_unlock(&(bbkb->operation_mutex));
                        goto error;
                }
                if(bbkb->tp_mode != ctrl[0])
                {
                        bbkb->tp_mode = ctrl[0];
                        ec_cmd[0] = 0x07;
                        ec_cmd[1] = 0x00;
                        ec_cmd[2] = bbkb->tp_mode;
                        ret = blueberry_kb_i2c_write_data(bbkb->client, ec_cmd, 3);
                        if(ret) {
                                pr_err("blueberry_kb: EC_IOCTL_TP_MODE failed.\n");
								mutex_unlock(&(bbkb->operation_mutex));
                                return ret;
                        }
                }
                mutex_unlock(&bbkb->operation_mutex);
                break;
*/

		case EC_IOCTL_GET_KB_STATUS:
			mutex_lock(&bbkb->operation_mutex);
            if (copy_to_user(argp, &bbkb->kb_enable, sizeof(bbkb->kb_enable))) {
				printk("%s():	copy to user error.\n", __func__);
				mutex_unlock(&(bbkb->operation_mutex));
                return -EFAULT;
            }
			mutex_unlock(&bbkb->operation_mutex);
            break;

		case EC_IOCTL_GET_TEST_MODE_STATUS:
            mutex_lock(&bbkb->operation_mutex);
            if (copy_to_user(argp, &bbkb->test_mode, sizeof(bbkb->test_mode))) {
                printk("%s():   copy to user error.\n", __func__);
				mutex_unlock(&(bbkb->operation_mutex));
                return -EFAULT;
            }
            mutex_unlock(&bbkb->operation_mutex);
            break;

		case EC_IOCTL_GET_FLASH_LED_STATUS:
            mutex_lock(&bbkb->operation_mutex);
            if (copy_to_user(argp, &bbkb->led_flash, sizeof(bbkb->led_flash))) {
                printk("%s():   copy to user error.\n", __func__);
				mutex_unlock(&(bbkb->operation_mutex));
                return -EFAULT;
            }
            mutex_unlock(&bbkb->operation_mutex);
            break;

		case GET_CAPSLOCK_STATUS:
			mutex_lock(&(bbkb->operation_mutex));
			if (copy_to_user(argp, &bbkb->capskey_status, sizeof(bbkb->capskey_status))) {
            	printk("%s():   copy to user error.\n", __func__);
				mutex_unlock(&(bbkb->operation_mutex));
              	return -EFAULT;
            }
			mutex_unlock(&(bbkb->operation_mutex));
			break;
		case KB_CAPSLOCK_TEST:
			mutex_lock(&(bbkb->operation_mutex));
             if (copy_from_user(key_ctrl_capslock, argp, sizeof(key_ctrl_capslock)))
             {
                   ret = -EFAULT;
				   mutex_unlock(&(bbkb->operation_mutex));
                   goto error;
             }
			 
		if((bbkb->test_mode == 1)){
		  ec_cmd[0] = 0x10;
      	  ec_cmd[1] = 0x00;
		  if(1 == key_ctrl_capslock[0]){
          	ec_cmd[2] =CMD_CAPSKEY_LED_ON;
		  }
		  else{
			ec_cmd[2] =CMD_CAPSKEY_LED_OFF;
		  }
                printk("blueberry_kb: key_ctrl_capslock[0] = %d, bbkb->capskey_status = %d\n", key_ctrl_capslock[0],bbkb->capskey_status);
				msleep(2);//add delay
                ret = blueberry_kb_i2c_write_data(bbkb->client, ec_cmd, 3);
                if(ret) {
                    pr_err("blueberry_kb: set capskey led failed.\n");
					mutex_unlock(&(bbkb->operation_mutex));	
                    return;
                }  
		}
        mutex_unlock(&(bbkb->operation_mutex));	 
				break;

        default:
                ret = -ENOTTY;
                goto error;
        }

error:
        return ret;
}

static struct file_operations blueberry_ec_fops = {
        .owner = THIS_MODULE,
        .open = blueberry_ec_open,
        .release = blueberry_ec_release,
        .unlocked_ioctl = blueberry_ec_ioctl,
};

static struct miscdevice blueberry_ec_device = {
        .minor = MISC_DYNAMIC_MINOR,
        .name = "ec",
        //.name = "blueberry_ec_dev",
        .fops = &blueberry_ec_fops,
};

static int blueberry_kb_resume(struct i2c_client *client)
{
    struct blueberry_kb *bbkb = i2c_get_clientdata(client);
	unsigned char ec_cmd[1] = {0x00};

	printk("%s():	notify ec arm resume\n", __func__);
	blueberry_kb_i2c_write_data(bbkb->client, ec_cmd, 1);//wake ec from idle	
	gpio_set_value(arm_notify_ec_pin, GPIO_HIGH);
	bbkb->arm_suspend_status = 0;
	cancel_work_sync(&bbkb->input_work);
    return 0;
}

static int blueberry_kb_suspend(struct i2c_client *client, pm_message_t mesg)
{
    struct blueberry_kb *bbkb = i2c_get_clientdata(client);
	printk("%s():	notify ec arm suspend\n", __func__);
	gpio_set_value(arm_notify_ec_pin, GPIO_LOW);
	bbkb->arm_suspend_status = 1;
    return 0;
}

static void blueberry_kb_shutdown(struct i2c_client *client)
{
	struct blueberry_kb *bbkb = i2c_get_clientdata(client);
	unsigned char ec_cmd[3];
	int ret;
	gpio_set_value(arm_notify_ec_pin, GPIO_LOW);
	printk("%s(): turn off capslock led\n", __func__);
    ec_cmd[0] = 0x10;
    ec_cmd[1] = 0x00;
    ec_cmd[2] = CMD_CAPSKEY_LED_OFF;
    ret = blueberry_kb_i2c_write_data(bbkb->client, ec_cmd, 3);
    if(ret) {
        pr_err("blueberry_kb: set capskey led failed.\n");
        return;
    }
	bbkb->capskey_status = 0;	
	return;
}

static void rk29_system_led_set(struct led_classdev *led_cdev,
					  enum led_brightness brightness)
{
    struct i2c_client *client = container_of(blueberry_ec_device.parent, struct i2c_client, dev);
	struct blueberry_kb* bbkb = (struct blueberry_kb *)i2c_get_clientdata(client);
    unsigned char ec_cmd[3];
    int ret = 0;
    
    if (LED_OFF == brightness) // LED_OFF  indicate led flash stop
    {
        mutex_lock(&bbkb->operation_mutex);

    	ec_cmd[0] = 0x10;
    	ec_cmd[1] = 0x00;
		ec_cmd[2] = CMD_LED_FLASH_END;
        ret = blueberry_kb_i2c_write_data(bbkb->client, ec_cmd, 3);
        if(ret) {
        	pr_err("blueberry_kb: rk29_system_led_set failed.\n");
			mutex_unlock(&(bbkb->operation_mutex));
            return ;
      	}
        
		mutex_unlock(&bbkb->operation_mutex);
    }
    else if (LED_FULL == brightness) // LED_FULL  indicate led flash open
    {
        mutex_lock(&bbkb->operation_mutex);

    	ec_cmd[0] = 0x10;
    	ec_cmd[1] = 0x00;
		ec_cmd[2] = CMD_LED_FLASH_START;
        ret = blueberry_kb_i2c_write_data(bbkb->client, ec_cmd, 3);
        if(ret) {
        	pr_err("blueberry_kb: rk29_system_led_set failed.\n");
			mutex_unlock(&(bbkb->operation_mutex));
            return ;
      	}
        
		mutex_unlock(&bbkb->operation_mutex);
    }
}

static struct led_classdev rk29_system_led = {
    .name           = "system_led",
    .brightness_set = rk29_system_led_set,
    .flags          = 0,
};

static int __devinit blueberry_kb_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{
	struct blueberry_kb *bbkb;
	unsigned char ec_cmd[3];
	int ret =0;

    pr_info("enter blueberry_kb_probe\n");

	bbkb = NULL;
	bbkb = kzalloc(sizeof(*bbkb), GFP_KERNEL);
    if(!bbkb) {
    	pr_err("blueberry_kb: can't alloc bbkb, no memory.\n");
        ret = -ENOMEM;
        goto kfree_dev;
	}
    bbkb->client = client;
	bbkb->tracking_angle = 0; /*default not tracking angle to change kb enable and disable */

	bbkb->keyboard_wq = create_singlethread_workqueue("keyboard_wq");
	if(!(bbkb->keyboard_wq))
	{
		ret = -ENOMEM;
		goto kfree_dev;
	}

    INIT_WORK(&bbkb->input_work, blueberry_kb_input_work);
	mutex_init(&bbkb->operation_mutex);

	i2c_set_clientdata(client, bbkb);

//	alloc input dev
	bbkb->input_dev = input_allocate_device();
	bbkb->input_dev->name = KB_INPUT_DEV_NAME;
    bbkb->input_dev->phys = bbkb->client->adapter->name;
    bbkb->input_dev->id.bustype = BUS_I2C;
    bbkb->input_dev->dev.parent = &client->dev;

	blueberry_kb_keypad_set_input_params(bbkb->input_dev);

	ret = input_register_device(bbkb->input_dev);
    if(ret){
    	pr_err("blueberry_kb input_register_device failed.\n");
        goto err_register_input_free;
	}	
//	request irq

	bbkb->flag_register = 1;

    ret = gpio_request(client->irq, "bbkb_irq");
    if(ret){
        pr_err("Failed to request blueberry_kb irq GPIO!\n");
        goto kfree_dev;
    }
    ret = gpio_direction_input(client->irq);
    if(ret){
        pr_err("failed to set blueberry_kb irq gpio input\n");
        goto kfree_dev;
    }
    gpio_pull_updown(client->irq, GPIOPullUp);

	enable_irq_wake(client->irq);
    ret = request_irq(client->irq, blueberry_kb_interrupt, IRQ_TYPE_LEVEL_LOW/*IRQ_TYPE_EDGE_FALLING*/, "bbkb_irq", bbkb);
    if(ret){
        pr_err("blueberry_kb: request irq failed\n");
        goto kfree_dev;
    }

	ret = gpio_request(arm_notify_ec_pin,"arm_notify_ec");
    if(ret){
        pr_err("Failed to request blueberry_kb ap_notify_ec GPIO!\n");
        goto kfree_dev;
    }
	ret = gpio_direction_output(arm_notify_ec_pin, GPIO_HIGH);

#if defined(CONFIG_HAS_EARLYSUSPEND)
    bbkb->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
    bbkb->early_drv.suspend  = blueberry_kb_early_suspend,
    bbkb->early_drv.resume   = blueberry_kb_early_resume,
    register_early_suspend(&bbkb->early_drv);
#endif	
    ret = sysfs_create_group(&client->dev.kobj, &blueberry_kb_attribute_group);
    if (ret) {
        pr_err("%s: sysfs_create_group returned err = %d. Abort.\n", __func__, ret);
    }

	bbkb->kb_enable = 1;
	bbkb->kb_enable1 = 1;

	ec_cmd[0] = 0x10;
	ec_cmd[1] = 0x00;
	ec_cmd[2] = ((bbkb->kb_enable) ? 1 : 0);
	printk("defalte enable keyboard\n");
	ret = blueberry_kb_i2c_write_data(bbkb->client, ec_cmd, 3);
	if(ret){
		pr_err("%s: defalte enable keyboard err = %d\n", __func__, ret);
	}

	bbkb->tp_enable = 1;
	bbkb->tp_mode = 1;//default abs
	bbkb->test_mode = 0;
	bbkb->led_flash = 0;
	bbkb->amber_led_status = 0;
	bbkb->blue_led_status = 0;
	bbkb->bat_status = 0;
	bbkb->ship_mode = 0;
	bbkb->capskey_status = 0;
	atomic_set(&bbkb->debug_flag, 0);
	bbkb->arm_suspend_status = 0;
	bbkb->arm_early_suspend = 0;

	blueberry_ec_device.parent = &client->dev;
	ret = misc_register(&blueberry_ec_device);
        if (ret) {
        	pr_err("blueberry_kb: misc register failed\n");
                //goto err_register_input_free;
        }
	else
		printk("blueberry_kb: misc register OK\n");

    ret = led_classdev_register(/*&client->dev*/NULL, &rk29_system_led);
    if (ret < 0)
    {
        pr_err("blueberry_kb: led class dev register fail\n");
    }
        g_bbkb = bbkb;  
        
	pr_info("blueberry_kb_probe OK\n");
	return 0;

err_register_input_free:
        input_unregister_device(bbkb->input_dev);
        if(bbkb->input_dev) input_free_device(bbkb->input_dev);
kfree_dev:
        free_irq(client->irq, bbkb);
        gpio_free(client->irq);
		gpio_free(arm_notify_ec_pin);
        if(bbkb) kfree(bbkb);
        return ret;
}

static int __devexit blueberry_kb_remove(struct i2c_client *client)
{
	struct blueberry_kb *bbkb = i2c_get_clientdata(client);

        led_classdev_unregister(&rk29_system_led);
        
    	dev_dbg(&client->dev, "%s()\n", __func__);
#ifdef CONFIG_HAS_EARLYSUSPEND
    	unregister_early_suspend(&bbkb->early_drv);
#endif
	sysfs_remove_group(&client->dev.kobj, &blueberry_kb_attribute_group);

	flush_workqueue(bbkb->keyboard_wq);
	destroy_workqueue(bbkb->keyboard_wq);
    	input_unregister_device(bbkb->input_dev);
    	kfree(bbkb);
    	return 0;
}

static const struct i2c_device_id blueberry_kb_id[] = {
    {"blueberry_kb", 0},
    {}
};

static struct i2c_driver blueberry_kb_driver = {
    .class  = I2C_CLASS_HWMON,
    .driver  = {
        .name = "blueberry_kb",
        .owner = THIS_MODULE,
    },
    .probe   = blueberry_kb_probe,
    .remove  = __devexit_p(blueberry_kb_remove),
	.suspend = blueberry_kb_suspend,
	.resume	 = blueberry_kb_resume,
	.shutdown = blueberry_kb_shutdown,
    .id_table = blueberry_kb_id,
};

static int __init blueberry_kb_init(void)
{
    pr_info("enter blueberry_kb_init.\n");
    return i2c_add_driver(&blueberry_kb_driver);
}

static void __exit blueberry_kb_exit(void)
{
    pr_info("enter blueberry_kb_exit.\n");
    i2c_del_driver(&blueberry_kb_driver);
}

module_init(blueberry_kb_init);
module_exit(blueberry_kb_exit);
