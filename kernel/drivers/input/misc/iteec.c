/* 
 * ASUS Dock EC driver.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/cdev.h>
#include <linux/gpio_event.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <linux/gpio.h>
#include <linux/ioctl.h>
#include <linux/uaccess.h>
#include <linux/power_supply.h>
#include <linux/earlysuspend.h>

#include "iteec.h"

static int __devinit asusdec_probe(struct i2c_client *client,
        const struct i2c_device_id *id);
static int __devexit asusdec_remove(struct i2c_client *client);
static int asusdec_open(struct inode *inode, struct file *flip);
static int asusdec_release(struct inode *inode, struct file *flip);
static long asusdec_ioctl(struct file *flip, unsigned int cmd, unsigned long arg);
static ssize_t ec_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos);
static ssize_t ec_read(struct file *file, char __user *buf, size_t count, loff_t *ppos);

#define asusdec_apwake_gpio_in          RK30_PIN0_PD7      //H:N/A, L:Interrupt (Docking EC)
#define REPORT_ID_KB	0x01
#define REPORT_ID_TP	0x02

#define READ_INPUT_CMD	0X03


static struct class *asusdec_class;
static struct device *asusdec_device;
static struct i2c_client dockram_client;
static struct asusdec_chip *ec_chip;

static char host_to_ec_buffer[EC_BUFF_LEN];
static char ec_to_host_buffer[EC_BUFF_LEN];
static int h2ec_count;
static int buff_in_ptr;   // point to the next free place
static int buff_out_ptr;      // points to the first data

struct cdev *asusdec_cdev ;
static dev_t asusdec_dev ;
static int asusdec_major = 0 ;
static int asusdec_minor = 0 ;

static struct workqueue_struct *asusdec_wq;

static int asusdec_kp_sci_table[]={0};

static const struct i2c_device_id asusdec_id[] = {
    {"asusdec", 0},
    {}
};
MODULE_DEVICE_TABLE(i2c, asusdec_id);

struct file_operations asusdec_fops = {
    .owner = THIS_MODULE,
    .unlocked_ioctl = asusdec_ioctl,
    .open = asusdec_open,
    .write = ec_write,
    .read = ec_read,
    .release = asusdec_release,
};

static struct i2c_driver asusdec_driver = {
    .class  = I2C_CLASS_HWMON,
    .driver  = {
        .name = "asusdec",
        .owner = THIS_MODULE,
    },
    .probe   = asusdec_probe,
    .remove  = __devexit_p(asusdec_remove),
    //.suspend = asusdec_suspend,
    //.resume = asusdec_resume,
    .id_table = asusdec_id,
};

static struct attribute *asusdec_smbus_attributes[] = {
/*
    &dev_attr_ec_status.attr,
    &dev_attr_ec_tp_status.attr,
    &dev_attr_ec_info.attr,
    &dev_attr_ec_dock.attr,
    &dev_attr_ec_dock_led.attr,
    &dev_attr_ec_charging_led.attr,
    &dev_attr_ec_wakeup.attr,
    &dev_attr_ec_dock_discharge.attr,
    &dev_attr_ec_dock_battery_info.attr,
    &dev_attr_ec_dock_battery.attr,
    &dev_attr_ec_dock_battery_all.attr,
    &dev_attr_ec_dock_control_flag.attr,
    &dev_attr_hallsensor_status.attr,
    &dev_attr_dock_ec_org_info.attr,
*/
NULL
};
static const struct attribute_group asusdec_smbus_group = {
    .attrs = asusdec_smbus_attributes,
};

static void asusdec_dockram_init(struct i2c_client *client){
    dockram_client.adapter = client->adapter;
    dockram_client.addr = 0x62;
    dockram_client.detected = client->detected;
    dockram_client.dev = client->dev;
    dockram_client.driver = client->driver;
    dockram_client.flags = client->flags;
    strcpy(dockram_client.name,client->name);
}

static int asusdec_open(struct inode *inode, struct file *flip){
    ASUSDEC_NOTICE(" ");
    return 0;
}
static int asusdec_release(struct inode *inode, struct file *flip){
    ASUSDEC_NOTICE(" ");
    return 0;
}

static int asusdec_tp_control(int arg)
{
/*
    int ret_val = 0;

    if(arg == ASUSDEC_TP_ON){
        if (ec_chip->tp_enable == 0){
            ec_chip->tp_wait_ack = 1;
            ec_chip->tp_enable = 1;
            asusdec_i2c_write_data(ec_chip->client, 0xF4D4);
            ec_chip->d_index = 0;
        }
        if (ec_chip->touchpad_member == -1){
            ec_chip->susb_on = 1;
            ec_chip->init_success = -1;
            asusdec_reset_dock();
        }
        ret_val = 0;
    } else if (arg == ASUSDEC_TP_OFF){
        ec_chip->tp_wait_ack = 1;
        ec_chip->tp_enable = 0;
        asusdec_i2c_write_data(ec_chip->client, 0xF5D4);
        ec_chip->d_index = 0;
        ret_val = 0;
    } else
        ret_val = -ENOTTY;

    return ret_val;
*/
}


static long asusdec_ioctl(struct file *flip,
                    unsigned int cmd, unsigned long arg){
/*
    int err = 1;
    char *envp[3];
    char name_buf[64];
    int env_offset = 0;
    int length = 0;

    if (_IOC_TYPE(cmd) != ASUSDEC_IOC_MAGIC)
     return -ENOTTY;
    if (_IOC_NR(cmd) > ASUSDEC_IOC_MAXNR)
    return -ENOTTY;

    if (_IOC_DIR(cmd) & _IOC_READ)
        err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
    else if (_IOC_DIR(cmd) & _IOC_WRITE)
        err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
    if (err) return -EFAULT;

     switch (cmd) {
        //case ASUSDEC_POLLING_DATA:
        //  if (arg == ASUSDEC_IOCTL_HEAVY){
        //      ASUSDEC_NOTICE("heavy polling\n");
        //      ec_chip->polling_rate = 80;
        //      queue_delayed_work(asusdec_wq, &asusdec_stress_work, HZ/ec_chip->polling_rate);
        //  }
        //  else if (arg == ASUSDEC_IOCTL_NORMAL){
        //      ASUSDEC_NOTICE("normal polling\n");
        //      ec_chip->polling_rate = 10;
        //      queue_delayed_work(asusdec_wq, &asusdec_stress_work, HZ/ec_chip->polling_rate);
        //  }
        //  else if  (arg == ASUSDEC_IOCTL_END){
        //      ASUSDEC_NOTICE("polling end\n");
        //      cancel_delayed_work_sync(&asusdec_stress_work) ;
        //  }
        //  else
        //      return -ENOTTY;
        //  break;
        case ASUSDEC_FW_UPDATE:
            if (ec_chip->dock_in){
                ASUSDEC_NOTICE("ASUSDEC_FW_UPDATE\n");
                buff_in_ptr = 0;
                buff_out_ptr = 0;
                h2ec_count = 0;
                ec_chip->suspend_state = 0;
                ec_chip->status = 0;
                //////////asusdec_reset_dock();
                wake_lock_timeout(&ec_chip->wake_lock, 3*60*HZ);
                msleep(3000);
                ec_chip->op_mode = 1;
                ec_chip->i2c_dm_data[0] = 0x02;
                ec_chip->i2c_dm_data[1] = 0x55;
                ec_chip->i2c_dm_data[2] = 0xAA;
                i2c_smbus_write_i2c_block_data(&dockram_client, 0x40, 3, ec_chip->i2c_dm_data);
                ec_chip->init_success = 0;
                ec_chip->dock_behavior = 0;
                ec_chip->tf201_dock = 0;
                msleep(1000);
            } else {
                ASUSDEC_NOTICE("No dock detected\n");
                return -1;
            }
            break;

        case ASUSDEC_INIT:
            msleep(500);
            ec_chip->status = 0;
            ec_chip->op_mode = 0;
            queue_delayed_work(asusdec_wq, &ec_chip->asusdec_dock_init_work, 0);
            msleep(2500);
            ASUSDEC_NOTICE("ASUSDEC_INIT - EC version: %s\n", ec_chip->ec_version);
            length = strlen(ec_chip->ec_version);
            //ec_chip->ec_version[length] = NULL;
            //memset(ec_chip->ec_version, 0, 32);
            snprintf(name_buf, sizeof(name_buf), "SWITCH_NAME=%s", ec_chip->ec_version);
            envp[env_offset++] = name_buf;
            envp[env_offset] = NULL;
            kobject_uevent_env(&ec_chip->dock_sdev.dev->kobj, KOBJ_CHANGE, envp);
            break;

        case ASUSDEC_TP_CONTROL:
            ASUSDEC_NOTICE("ASUSDEC_TP_CONTROL\n");
            if ((ec_chip->op_mode == 0) && ec_chip->dock_in){
                err = asusdec_tp_control(arg);
                return err;
            }
            else
                return -ENOTTY;

        case ASUSDEC_EC_WAKEUP:
            msleep(500);
        //  ASUSDEC_NOTICE("ASUSDEC_EC_WAKEUP, arg = %d\n", arg);
            if (arg == ASUSDEC_EC_OFF){
                ec_chip->ec_wakeup = 0;
                ASUSDEC_NOTICE("Set EC shutdown when PAD in LP0\n");
                return asusdec_set_wakeup_cmd();
            }
            else if (arg == ASUSDEC_EC_ON){
                ec_chip->ec_wakeup = 1;
                ASUSDEC_NOTICE("Keep EC active when PAD in LP0\n");
                return asusdec_set_wakeup_cmd();
            }
            else {
                ASUSDEC_ERR("Unknown argument");
                return -ENOTTY;
            }
        case ASUSDEC_FW_DUMMY:
            ASUSDEC_NOTICE("ASUSDEC_FW_DUMMY\n");
            ec_chip->i2c_dm_data[0] = 0x02;
            ec_chip->i2c_dm_data[1] = 0x55;
            ec_chip->i2c_dm_data[2] = 0xAA;
            i2c_smbus_write_i2c_block_data(&dockram_client, 0x40, 3, ec_chip->i2c_dm_data);
            return 0;

        default:
            return -ENOTTY;
    }
    return 0;
*/
}
/*
static int BuffDataSize(void)
{
    int in = buff_in_ptr;
    int out = buff_out_ptr;

    if (in >= out)
    {
        return (in - out);
    }
    else
    {
        return ((EC_BUFF_LEN - out) + in);
    }
}

static char BuffGet(void)
{
    char c = (char)0;

    if (BuffDataSize() != 0)
    {
        c = (char) ec_to_host_buffer[buff_out_ptr];
        buff_out_ptr++;
         if (buff_out_ptr >= EC_BUFF_LEN)
         {
             buff_out_ptr = 0;
         }
    }
    return c;
}
*/

static ssize_t ec_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
/*
    int err;
    int i;

    if (h2ec_count > 0)
    {                   // There is still data in the buffer that
        return -EBUSY;  // was not sent to the EC
    }
    if (count > EC_BUFF_LEN)
    {
        return -EINVAL; // data size is too big
    }

    err = copy_from_user(host_to_ec_buffer, buf, count);
    if (err)
    {
        ASUSDEC_ERR("ec_write copy error\n");
        return err;
    }

    h2ec_count = count;
    for (i = 0; i < count ; i++)
    {
        i2c_smbus_write_byte_data(&dockram_client, host_to_ec_buffer[i],0);
    }
    h2ec_count = 0;
    return count;
*/
}

static ssize_t ec_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
/*
    int i = 0;
    int ret;
    char tmp_buf[EC_BUFF_LEN];
    static int f_counter = 0;
    static int total_buf = 0;

    mutex_lock(&ec_chip->lock);
    while ((BuffDataSize() > 0) && count)
    {
        tmp_buf[i] = BuffGet();
        count--;
        i++;
        f_counter = 0;
        total_buf++;
    }

    ret = copy_to_user(buf, tmp_buf, i);
    if (ret == 0)
    {
        ret = i; // No error. Return the number of byte read.
    }
    mutex_unlock(&ec_chip->lock);
    return ret;
*/
}

static int asusdec_i2c_read_data(struct i2c_client *client)
{
    int ret = 0;

    ret = i2c_smbus_read_i2c_block_data(client, READ_INPUT_CMD, 8, ec_chip->i2c_data);
    if (ret < 0) {
        ASUSDEC_ERR("Fail to read data, status %d\n", ret);
    }
    return ret;
}

static void asusdec_keypad_set_input_params(struct input_dev *dev)
{
    int i = 0;
    set_bit(EV_KEY, dev->evbit);
    for ( i = 0; i < 246; i++)
        set_bit(i,dev->keybit);

    input_set_capability(dev, EV_LED, LED_CAPSL);
}

static void asusdec_touchpad_set_input_params(struct input_dev *dev)
{
    input_set_capability(dev, EV_REL, REL_X);
    input_set_capability(dev, EV_REL, REL_Y);
    //input_set_capability(dev, EV_REL, REL_WHEEL);
	input_set_capability(dev, EV_KEY, BTN_LEFT);
	input_set_capability(dev, EV_KEY, BTN_RIGHT);
	//input_set_capability(dev, EV_KEY, BTN_MIDDLE);
}

static int asusdec_input_device_create(struct i2c_client *client){
    int err = 0;

    if (ec_chip->indev){
        return 0;
    }
    ec_chip->indev = input_allocate_device();
    if (!ec_chip->indev) {
        ASUSDEC_ERR("input_dev allocation fails\n");
        err = -ENOMEM;
        goto exit;
    }

    ec_chip->indev->name = "asusdec";
    ec_chip->indev->phys = "/dev/input/asusdec";
    ec_chip->indev->dev.parent = &client->dev;
    //ec_chip->indev->event = asusdec_event;
    
    /*zwp
     add vendor/product/version to keyboard input device for android load keyboard map.
	*/
    ec_chip->indev->id.vendor= 0x0001;
    ec_chip->indev->id.product = 0x0002;
    ec_chip->indev->id.version = 0x0001;

    asusdec_keypad_set_input_params(ec_chip->indev);
	asusdec_touchpad_set_input_params(ec_chip->indev);

    err = input_register_device(ec_chip->indev);
    if (err) {
        ASUSDEC_ERR("input registration fails\n");
        goto exit_input_free;
    }
    return 0;

exit_input_free:
    input_free_device(ec_chip->indev);
    ec_chip->indev = NULL;
exit:
    return err;

}

#if (!TOUCHPAD_MODE)
static void asusdec_tp_rel(void)
{
    ec_chip->touchpad_data.x_sign = (ec_chip->ec_data[0] & X_SIGN_MASK) ? 1:0;
    ec_chip->touchpad_data.y_sign = (ec_chip->ec_data[0] & Y_SIGN_MASK) ? 1:0;//should mofify ec code

	if(ec_chip->ec_data[2] == 0)
	{
		ec_chip->touchpad_data.y_sign = 0;
	}

    ec_chip->touchpad_data.left_btn = (ec_chip->ec_data[0] & LEFT_BTN_MASK) ? 1:0;
    ec_chip->touchpad_data.right_btn = (ec_chip->ec_data[0] & RIGHT_BTN_MASK) ? 1:0;
    ec_chip->touchpad_data.delta_x =
        (ec_chip->touchpad_data.x_sign) ? (ec_chip->ec_data[1] - 0xff):ec_chip->ec_data[1];
    ec_chip->touchpad_data.delta_y =
        (ec_chip->touchpad_data.y_sign) ? (ec_chip->ec_data[2] - 0xff):ec_chip->ec_data[2];

	printk("ec_data[1] = %d, x_sign = %d, delta_x = %d\n", ec_chip->ec_data[1], ec_chip->touchpad_data.x_sign, ec_chip->touchpad_data.delta_x);
	printk("ec_data[2] = %d, y_sign = %d, delta_y = %d\n", ec_chip->ec_data[2], ec_chip->touchpad_data.y_sign, ec_chip->touchpad_data.delta_y);

	input_report_rel(ec_chip->indev, REL_X, ec_chip->touchpad_data.delta_x);
    input_report_rel(ec_chip->indev, REL_Y, (-1) * ec_chip->touchpad_data.delta_y);
    input_report_key(ec_chip->indev, BTN_LEFT, ec_chip->touchpad_data.left_btn);
    input_report_key(ec_chip->indev, KEY_BACK, ec_chip->touchpad_data.right_btn);
    input_sync(ec_chip->indev);
}
#endif

#if TOUCHPAD_MODE
static void asusdec_tp_abs(void)
{
/*
    unsigned char SA1,A1,B1,SB1,C1,D1;
    static unsigned char SA1_O=0,A1_O=0,B1_O=0,SB1_O=0,C1_O=0,D1_O=0;
    static int Null_data_times = 0;

    if ((ec_chip->tp_enable) && (ec_chip->touchpad_member == ELANTOUCHPAD)){
        SA1= ec_chip->ec_data[0];
        A1 = ec_chip->ec_data[1];
        B1 = ec_chip->ec_data[2];
        SB1= ec_chip->ec_data[3];
        C1 = ec_chip->ec_data[4];
        D1 = ec_chip->ec_data[5];
        ASUSDEC_INFO("SA1=0x%x A1=0x%x B1=0x%x SB1=0x%x C1=0x%x D1=0x%x \n",SA1,A1,B1,SB1,C1,D1);
        if ( (SA1 == 0xC4) && (A1 == 0xFF) && (B1 == 0xFF) &&
             (SB1 == 0x02) && (C1 == 0xFF) && (D1 == 0xFF)){
            Null_data_times ++;
            goto asusdec_tp_abs_end;
        }

        if(!(SA1 == SA1_O && A1 == A1_O && B1 == B1_O &&
           SB1 == SB1_O && C1 == C1_O && D1 == D1_O)) {
            elantech_report_absolute_to_related(ec_chip, &Null_data_times);
        }

asusdec_tp_abs_end:
        SA1_O = SA1;
        A1_O = A1;
        B1_O = B1;
        SB1_O = SB1;
        C1_O = C1;
        D1_O = D1;
    } else if (ec_chip->touchpad_member == -1){
        ec_chip->susb_on = 1;
        ec_chip->init_success = -1;
        asusdec_reset_dock();
    }
*/
}
#endif

static void asusdec_touchpad_processing(void){
    int i;
    int length = 0;
    int tp_start = 0;
    ASUSDEC_I2C_DATA(ec_chip->i2c_data,ec_chip->index);

#if TOUCHPAD_MODE
    length = ec_chip->i2c_data[0];
    if (ec_chip->tp_wait_ack){
        ec_chip->tp_wait_ack = 0;
        tp_start = 1;
        ec_chip->d_index = 0;
    } else {
        tp_start = 0;
    }

    for( i = tp_start; i < length - 1 ; i++){
        ec_chip->ec_data[ec_chip->d_index] = ec_chip->i2c_data[i+2];
        ec_chip->d_index++;
        if (ec_chip->d_index == 6){
            asusdec_tp_abs();
            ec_chip->d_index = 0;
        }
    }

    if (ec_chip->d_index)
        mod_timer(&ec_chip->asusdec_timer,jiffies+(HZ * 1/20));
#else
    length = ec_chip->i2c_data[0];
/*
    for( i = 0; i < length -1 ; i++)
	{
        ec_chip->ec_data[ec_chip->d_index] = ec_chip->i2c_data[i+3];
        ec_chip->d_index++;
		printk("ec_chip->d_index= %d, ec_chip->ec_data[%d] = %d\n", ec_chip->d_index, ec_chip->d_index, ec_chip->ec_data[ec_chip->d_index]);
        if (ec_chip->d_index == 3){
            asusdec_tp_rel();
            ec_chip->d_index = 0;
        }
    }
*/

	for(i=0; i<3; i++)
	{
		ec_chip->ec_data[i] = ec_chip->i2c_data[i+2];
	}
	asusdec_tp_rel();
#endif
}

static int asusdec_kp_key_mapping(int x)
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
            return KEY_HOMEPAGE;

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
		return KEY_HOME;

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

/*
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
*/
        default:
            return -1;
    }
}

/*
static void asusdec_kp_sci(void){
    int ec_signal = ec_chip->i2c_data[3];

    ec_chip->keypad_data.input_keycode = asusdec_kp_sci_table[ec_signal];
    if(ec_chip->keypad_data.input_keycode > 0){
        ASUSDEC_INFO("input_keycode = 0x%x\n", ec_chip->keypad_data.input_keycode);

        input_report_key(ec_chip->indev, ec_chip->keypad_data.input_keycode, 1);
        input_sync(ec_chip->indev);
        input_report_key(ec_chip->indev, ec_chip->keypad_data.input_keycode, 0);
        input_sync(ec_chip->indev);

    }else{
        ASUSDEC_INFO("Unknown ec_signal = 0x%x\n", ec_signal);
    }
}
*/

static void asusdec_kp_key(void)
{
    int scancode = 0;
/*
    if (ec_chip->i2c_data[2] == ASUSDEC_KEYPAD_KEY_EXTEND){     // data is an extended data
        ec_chip->keypad_data.extend = 1;
        ec_chip->bc = 3;
    }else{
        ec_chip->keypad_data.extend = 0;
        ec_chip->bc = 2;
    }
    if(ec_chip->i2c_data[ec_chip->bc] == ASUSDEC_KEYPAD_KEY_BREAK){ // the data is a break signal
        ec_chip->keypad_data.value = 0;
        ec_chip->bc++;
    }else{
        ec_chip->keypad_data.value = 1;
    }

    if (ec_chip->keypad_data.extend == 1){
        scancode = ((ASUSDEC_KEYPAD_KEY_EXTEND << 8) | ec_chip->i2c_data[ec_chip->bc]);
    } else {
        scancode = ec_chip->i2c_data[ec_chip->bc];
    }
    if (ec_chip->i2c_data[0] == 6){                             // left shift DOWN + note2 keys
        if ((ec_chip->i2c_data[2] == 0xE0) &&
            (ec_chip->i2c_data[3] == 0xF0) &&
            (ec_chip->i2c_data[4] == 0x12)){
            scancode = ec_chip->i2c_data[5] << 8 | ec_chip->i2c_data[6];
            ec_chip->keypad_data.value = 1;
        }
        else if ((ec_chip->i2c_data[2] == 0xE0) &&              // right shift DOWN + note2 keys
            (ec_chip->i2c_data[3] == 0xF0) &&
            (ec_chip->i2c_data[4] == 0x59)){
            scancode = ec_chip->i2c_data[5] << 8 | ec_chip->i2c_data[6];
            ec_chip->keypad_data.value = 1;
        }
    }
*/
	if(ec_chip->i2c_data[1] == REPORT_ID_KB)
{

	scancode = ec_chip->i2c_data[3];
	ec_chip->keypad_data.value = ec_chip->i2c_data[2];
    //printk("hid code = 0x%x, value = %d\n", scancode, ec_chip->keypad_data.value);
    ASUSDEC_INFO("scancode = 0x%x\n", scancode);
    ec_chip->keypad_data.input_keycode = asusdec_kp_key_mapping(scancode);

    if(ec_chip->keypad_data.input_keycode > 0)
	{
        //printk("input_keycode = 0x%x, input_value = %d\n", ec_chip->keypad_data.input_keycode, ec_chip->keypad_data.value);

        input_report_key(ec_chip->indev, ec_chip->keypad_data.input_keycode, ec_chip->keypad_data.value);
        input_sync(ec_chip->indev);

    }else{
        ASUSDEC_INFO("Unknown scancode = 0x%x\n", scancode);
    }
}
}

static void asusdec_keypad_processing(void)
{
	ASUSDEC_I2C_DATA(ec_chip->i2c_data,ec_chip->index);

	asusdec_kp_key();
/*
    if (ec_chip->i2c_data[1] & ASUSDEC_KBC_MASK)
        asusdec_kp_kbc();
    else if (ec_chip->i2c_data[1] & ASUSDEC_SCI_MASK)
        asusdec_kp_sci();
    else
        asusdec_kp_key();
*/
}

static void asusdec_work_function(struct work_struct *dat)
{
    int gpio = asusdec_apwake_gpio_in;
    int irq = gpio_to_irq(gpio);
    int ret_val = 0;

    //if(MY_DBG==1) printk("DOCKEC: asusdec_work_function ++ \n");
/*
    ec_chip->dock_in = gpio_get_value(asusdec_dock_in_gpio_in) ? 0 : 1;

    if (ec_chip->wakeup_lcd)
    {
        if (gpio_get_value(asusdec_hall_sensor_gpio_in))
        {
            ec_chip->wakeup_lcd = 0;
            wake_lock_timeout(&ec_chip->wake_lock_timeout, 3*HZ);
            msleep(500);
        }
    }
*/
    ret_val = asusdec_i2c_read_data(ec_chip->client);

    printk("DOCKEC: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x ret_val=%x \n",ec_chip->i2c_data[0],ec_chip->i2c_data[1],ec_chip->i2c_data[2],ec_chip->i2c_data[3],ec_chip->i2c_data[4],ec_chip->i2c_data[5],ec_chip->i2c_data[6],ec_chip->i2c_data[7],ret_val);

    enable_irq(irq);

    if (ret_val < 0){
        return ;
    }

/////////////////////////
	mutex_lock(&ec_chip->input_lock);
	switch(ec_chip->i2c_data[1])
	{
		case REPORT_ID_KB:
			asusdec_keypad_processing();
			break;
		case REPORT_ID_TP:
			//if (ec_chip->private->abs_dev)
				asusdec_touchpad_processing();
			break;
		default:
			break;
	}
	mutex_unlock(&ec_chip->input_lock);
/////////////////////////
/*
    if (ec_chip->i2c_data[1] & ASUSDEC_OBF_MASK){       // ec data is valid
        if (ec_chip->i2c_data[1] & ASUSDEC_SMI_MASK){   // ec data is from touchpad
            //asusdec_kp_smi();
            return ;
        }
    }

    mutex_lock(&ec_chip->input_lock);
    if (ec_chip->indev == NULL){
        mutex_unlock(&ec_chip->input_lock);
        return;
    }
    if (ec_chip->i2c_data[1] & ASUSDEC_OBF_MASK){       // ec data is valid
        if (ec_chip->i2c_data[1] & ASUSDEC_AUX_MASK){   // ec data is from touchpad
            if (ec_chip->private->abs_dev)
                asusdec_touchpad_processing();
        }else{      // ec data is from keyboard
            asusdec_keypad_processing();
        }
    }
    mutex_unlock(&ec_chip->input_lock);
*/
    //if(MY_DBG==1) printk("DOCKEC: asusdec_work_function -- \n");
}

static irqreturn_t asusdec_apwake_interrupt_handler(int irq, void *dev_id)
{
    disable_irq_nosync(irq);
	queue_delayed_work(asusdec_wq, &ec_chip->asusdec_work, 0);
/*
    if (ec_chip->op_mode){
        queue_delayed_work(asusdec_wq, &ec_chip->asusdec_fw_update_work, 0);
    }
    else{
        if (ec_chip->suspend_state){
            ec_chip->wakeup_lcd = 1;
        }
        if(gpio_get_value(asusdec_core_pwr_req_gpio_out))
        {
            wake_lock_timeout(&ec_chip->wake_lock_timeout, 1.6*HZ);
            queue_delayed_work(asusdec_wq, &ec_chip->asusdec_work, 1.5*HZ);
        }
        else
        {
            queue_delayed_work(asusdec_wq, &ec_chip->asusdec_work, 0);
        }
        ec_chip->ap_wake_wakeup = 1;
    }
*/
    return IRQ_HANDLED;
}

static int asusdec_irq_apwake(struct i2c_client *client)
{
    int rc = 0 ;
    unsigned gpio = asusdec_apwake_gpio_in;
    unsigned int irq;
    const char* label = "asusdec_input";

    rc = gpio_request(gpio, label);
    if (rc) {
        ASUSDEC_ERR("gpio_request failed for input %d\n", gpio);
        goto err_request_input_gpio_failed;
    }
    rc = gpio_direction_input(gpio) ;
    if (rc) {
        ASUSDEC_ERR("gpio_direction_input failed for input %d\n", gpio);
        goto err_gpio_direction_input_failed;
    }

    irq = gpio_to_irq(gpio);
    rc = request_irq(irq, asusdec_apwake_interrupt_handler,/*IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING|IRQF_TRIGGER_HIGH|*/IRQF_TRIGGER_LOW, label, client);
    if (rc < 0) {
        ASUSDEC_ERR("Could not register for %s interrupt, irq = %d, rc = %d\n", label, irq, rc);
        rc = -EIO;
        goto err_gpio_request_irq_fail ;
    }
    enable_irq_wake(irq);
    return 0 ;

err_gpio_request_irq_fail :
    gpio_free(gpio);
err_gpio_direction_input_failed:
err_request_input_gpio_failed :
    return rc;
}


static int __devinit asusdec_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{
    int err = 0;

    if(MY_DBG==1) printk("DOCKEC: asusdec_probe ++ \n");

    err = sysfs_create_group(&client->dev.kobj, &asusdec_smbus_group);
    if (err) {
        ASUSDEC_ERR("Unable to create the sysfs\n");
        goto exit;
    }

    ec_chip = kzalloc(sizeof (struct asusdec_chip), GFP_KERNEL);
    if (!ec_chip) {
        ASUSDEC_ERR("Memory allocation fails\n");
        err = -ENOMEM;
        goto exit;
    }
    ec_chip->private = kzalloc(sizeof(struct elantech_data), GFP_KERNEL);
    if (!ec_chip->private) {
        ASUSDEC_ERR("Memory allocation (elantech_data) fails\n");
        err = -ENOMEM;
        goto exit;
    }

    i2c_set_clientdata(client, ec_chip);
    ec_chip->client = client;
    ec_chip->client->driver = &asusdec_driver;

	asusdec_dockram_init(client);

    mutex_init(&ec_chip->lock);
    mutex_init(&ec_chip->input_lock);

	cdev_add(asusdec_cdev,asusdec_dev,1);

	if(asusdec_input_device_create(client)){
    	goto fail_to_access_ec;
	}
    //TODO register power supply
	
	asusdec_wq = create_singlethread_workqueue("asusdec_wq");
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->asusdec_work, asusdec_work_function);

	//asusdec_irq_hall_sensor(client);
	asusdec_irq_apwake(client);

fail_to_access_ec:
exit:
	return err;
}

static int __devexit asusdec_remove(struct i2c_client *client)
{
    struct asusdec_chip *chip = i2c_get_clientdata(client);

    dev_dbg(&client->dev, "%s()\n", __func__);
    input_unregister_device(chip->indev);
    kfree(chip);
    return 0;
}

static int __init asusdec_init(void)
{
    int err = 0;

    if (asusdec_major) {
        asusdec_dev = MKDEV(asusdec_major, asusdec_minor);
        err = register_chrdev_region(asusdec_dev, 1, "asusdec");
    } else {
        err = alloc_chrdev_region(&asusdec_dev, asusdec_minor, 1,"asusdec");
        asusdec_major = MAJOR(asusdec_dev);
    }

    asusdec_cdev = cdev_alloc();
    asusdec_cdev->owner = THIS_MODULE;
    asusdec_cdev->ops = &asusdec_fops;

    err = i2c_add_driver(&asusdec_driver);
    if(err)
    {
        ASUSDEC_ERR("i2c_add_driver fail\n");
    }

    asusdec_class = class_create(THIS_MODULE, "asusdec");
    if(asusdec_class <= 0){
        ASUSDEC_ERR("asusdec_class create fail\n");
        err = -1;
        goto class_create_fail ;
    }
    asusdec_device = device_create(asusdec_class, NULL, MKDEV(asusdec_major, asusdec_minor), NULL, "asusdec" );
    if(asusdec_device <= 0){
        ASUSDEC_ERR("asusdec_device create fail\n");
        err = -1;
        goto device_create_fail ;
    }
	return 0;

device_create_fail :
    class_destroy(asusdec_class) ;
class_create_fail :
    i2c_del_driver(&asusdec_driver);
i2c_add_driver_fail :

    return err;
}

static void __exit asusdec_exit(void)
{
    device_destroy(asusdec_class,MKDEV(asusdec_major, asusdec_minor)) ;
    class_destroy(asusdec_class) ;
    i2c_del_driver(&asusdec_driver);
    unregister_chrdev_region(asusdec_dev, 1);
    //switch_dev_unregister(&ec_chip->dock_sdev);
}

module_init(asusdec_init);
module_exit(asusdec_exit);
