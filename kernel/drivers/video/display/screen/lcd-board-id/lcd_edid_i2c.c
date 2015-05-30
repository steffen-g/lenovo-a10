#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <linux/delay.h>
#include <linux/freezer.h>
#include <mach/gpio.h>
#include <mach/board.h> 
#include <linux/fb.h>
#include <linux/delay.h>
#include <mach/gpio.h>
#include <mach/iomux.h>
#include <mach/board.h>
#include <linux/miscdevice.h>

#include "../screen.h"
#include "../../../rk29_fb.h"

#include <linux/board-id.h> 
#include "../../../edid.h"

#define EDID_I2C_RATE 200*1000

extern struct board_id_private_data *g_board_id;

 
#if 1
#define DBG(x...) printk(x)
#else
#define DBG(x...)
#endif

#define OUT_TYPE		SCREEN_RGB
#define OUT_FACE		OUT_D888_P666
#define OUT_CLK			64000000
#define LCDC_ACLK       500000000

/* Timing */
#define H_PW			100
#define H_BP			100
#define H_VD			1368
#define H_FP			120

#define V_PW			10
#define V_BP			10
#define V_VD			768
#define V_FP			15


#define LCD_WIDTH		222
#define LCD_HEIGHT		125

#define DCLK_POL		1//0
#define SWAP_RB			0   


struct edid_private_data {
	struct i2c_client *client;		
	struct mutex i2c_mutex;	
	unsigned char edid_data[128];
	int lcd_id;
	struct file_operations fops;
	struct miscdevice miscdev;
};

static struct edid_private_data *g_edid_data;

#define EDID_IOCTL_MAGIC 'd'
#define EDID_IOCTL_GET_ALL_DATA 			_IOR(EDID_IOCTL_MAGIC, 1, int *)
#define EDID_IOCTL_GET_VID_PID				_IOR(EDID_IOCTL_MAGIC, 2, int *)

static int edid_dev_open(struct inode *inode, struct file *file)
{		
	int result = 0;
	
	return result;
}


static int edid_dev_release(struct inode *inode, struct file *file)
{
	int result = 0;

	return result;
}

/* ioctl - I/O control */
static long edid_dev_ioctl(struct file *file,
			  unsigned int cmd, unsigned long arg)

{	
	struct edid_private_data *edid = g_edid_data;
	unsigned int *argp = (unsigned int *)arg;	
	int result = 0;

	if(!edid)
		return -1;

	switch(cmd)
	{
		case EDID_IOCTL_GET_ALL_DATA:
			if ( copy_to_user(argp, edid->edid_data, sizeof(edid->edid_data)) ) 
			{
				printk("failed to copy edid->edid_data to user space.");
				return  -EFAULT;			
			}	
			break;

		case EDID_IOCTL_GET_VID_PID:
			if ( copy_to_user(argp, &edid->edid_data[ID_MANUFACTURER_NAME], 4) )
			{
				printk("failed to copy ID_MANUFACTURER_NAME to user space.");
				return  -EFAULT;			
			}		
			break;

		default:
			printk("%s:unknow cmd\n",__func__);
			break;

	}
	
	return 0;
}


void set_lcd_info_edid_i2c(struct rk29fb_screen *screen, struct rk29lcd_info *lcd_info )
{
	struct edid_private_data *edid = NULL;	
	unsigned char *block;
	
	edid = g_edid_data;
	if(!edid)
	{
		printk("%s:edid is null,use default value\n",__func__);
		/* screen type & face */
		screen->type = OUT_TYPE;
		screen->face = OUT_FACE;

		/* Screen size */
		screen->x_res = H_VD;
		screen->y_res = V_VD;

		screen->width = LCD_WIDTH;
		screen->height = LCD_HEIGHT;

		/* Timing */
		screen->lcdc_aclk = LCDC_ACLK;
		screen->pixclock = OUT_CLK;
		screen->left_margin = H_BP;
		screen->right_margin = H_FP;
		screen->hsync_len = H_PW;
		screen->upper_margin = V_BP;
		screen->lower_margin = V_FP;
		screen->vsync_len = V_PW;
		return;
	}
	
	block = edid->edid_data + DETAILED_TIMING_DESCRIPTIONS_START;

	/* screen type & face */
	screen->type = OUT_TYPE;
	screen->face = OUT_FACE;

	/* Screen size */
	screen->x_res = (H_ACTIVE%8)? ((H_ACTIVE/8+1)*8): H_ACTIVE;
	screen->y_res = (V_ACTIVE%8)? ((V_ACTIVE/8+1)*8): V_ACTIVE;
	if(!H_SIZE_LO || !V_SIZE_LO)	//why 0?
	{
		screen->width = LCD_WIDTH;
		screen->height = LCD_HEIGHT;
	}
	else
	{
		screen->width = H_SIZE_LO;
		screen->height = V_SIZE_LO;
	}

	/* Timing */
	screen->lcdc_aclk = LCDC_ACLK;
	screen->pixclock = OUT_CLK;//PIXEL_CLOCK
	screen->left_margin = (H_ACTIVE + H_BLANKING) -
		(H_ACTIVE + H_SYNC_OFFSET + H_SYNC_WIDTH);
	screen->right_margin = H_SYNC_OFFSET;
	screen->hsync_len = H_SYNC_WIDTH;
	screen->upper_margin = V_BLANKING - V_SYNC_OFFSET -
		V_SYNC_WIDTH;
	screen->lower_margin = V_SYNC_OFFSET;
	screen->vsync_len = V_SYNC_WIDTH;

	/* Pin polarity */
	screen->pin_hsync = HSYNC_POSITIVE;//0
	screen->pin_vsync = VSYNC_POSITIVE;//0
	screen->pin_den = 0;
	screen->pin_dclk = 0;//DCLK_POL;

	/* Swap rule */
	screen->swap_rb = SWAP_RB;
	screen->swap_rg = 0;
	screen->swap_gb = 0;
	screen->swap_delta = 0;
	screen->swap_dumy = 0;

	/* Operation function*/
	screen->init = NULL;
	screen->standby = NULL;
	//screen->dsp_lut = dsp_lut;

	if (INTERLACED) {
		screen->y_res *= 2;
		screen->upper_margin *= 2;
		screen->lower_margin *= 2;
		screen->vsync_len *= 2;
	}


	DBG("%s:type = %d\n face = %d\n x_res = %d\n y_res = %d\n width = %d\n height = %d\n lcdc_aclk = %d\n pixclock = %d\n left_margin = %d\n right_margin = %d\n hsync_len = %d\n upper_margin = %d\n lower_margin = %d\n vsync_len = %d\n pin_hsync = %d\n pin_vsync = %d\n pin_den = %d\n pin_dclk = %d\n ",
	__func__,
	screen->type,
	screen->face,
	screen->x_res,
	screen->y_res,
	screen->width,
	screen->height,
	screen->lcdc_aclk,
	screen->pixclock,
	screen->left_margin,
	screen->right_margin,
	screen->hsync_len,
	screen->upper_margin,
	screen->lower_margin,
	screen->vsync_len,
	screen->pin_hsync,
	screen->pin_vsync,
	screen->pin_den,
	screen->pin_dclk

	);



	printk("%s\n",__func__);

	
}


static int edid_i2c_write(struct i2c_adapter *i2c_adap,
			    unsigned char address,
			    unsigned int len, unsigned char const *data)
{
	struct i2c_msg msgs[1];
	int res;

	if (!data || !i2c_adap) {
		printk("%s:line=%d,error\n",__func__,__LINE__);
		return -EINVAL;
	}

	msgs[0].addr = address;
	msgs[0].flags = 0;	/* write */
	msgs[0].buf = (unsigned char *)data;
	msgs[0].len = len;
	msgs[0].scl_rate = EDID_I2C_RATE;

	res = i2c_transfer(i2c_adap, msgs, 1);
	if (res == 1)
		return 0;
	else if(res == 0)
		return -EBUSY;
	else
		return res;

}

static int senosr_i2c_read(struct i2c_adapter *i2c_adap,
			   unsigned char address, unsigned char reg,
			   unsigned int len, unsigned char *data)
{
	struct i2c_msg msgs[2];
	int res;

	if (!data || !i2c_adap) {
		printk("%s:line=%d,error\n",__func__,__LINE__);
		return -EINVAL;
	}

	msgs[0].addr = address;
	msgs[0].flags = 0;	/* write */
	msgs[0].buf = &reg;
	msgs[0].len = 1;
	msgs[0].scl_rate = EDID_I2C_RATE;
	
	msgs[1].addr = address;
	msgs[1].flags = I2C_M_RD;
	msgs[1].buf = data;
	msgs[1].len = len;
	msgs[1].scl_rate = EDID_I2C_RATE;	

	res = i2c_transfer(i2c_adap, msgs, 2);
	if (res == 2)
		return 0;
	else if(res == 0)
		return -EBUSY;
	else
		return res;

}


static int edid_rx_data(struct i2c_client *client, char *rxData, int length)
{
	struct edid_private_data* edid = 
		(struct edid_private_data *)i2c_get_clientdata(client);
	int i = 0;
	int ret = 0;
	char reg = rxData[0];
	ret = senosr_i2c_read(client->adapter, client->addr, reg, length, rxData);
	
	DBG("addr=0x%x,len=%d,rxdata:",reg,length);
	for(i=0; i<length; i++)
		DBG("0x%x,",rxData[i]);
	DBG("\n");
	return ret;
}

static int edid_tx_data(struct i2c_client *client, char *txData, int length)
{
	struct edid_private_data* edid = 
		(struct edid_private_data *)i2c_get_clientdata(client);
	int i = 0;
	int ret = 0;

	DBG("addr=0x%x,len=%d,txdata:",txData[0],length);
	for(i=1; i<length; i++)
		DBG("0x%x,",txData[i]);
	DBG("\n");
	ret = edid_i2c_write(client->adapter, client->addr, length, txData);
	return ret;

}

static int edid_write_reg(struct i2c_client *client, int addr, int value)
{
	char buffer[2];
	int ret = 0;
	struct edid_private_data* edid = 
		(struct edid_private_data *)i2c_get_clientdata(client);
	
	mutex_lock(&edid->i2c_mutex);	
	buffer[0] = addr;
	buffer[1] = value;
	ret = edid_tx_data(client, &buffer[0], 2);	
	mutex_unlock(&edid->i2c_mutex);	
	return ret;
}

static int edid_read_reg(struct i2c_client *client, int addr)
{
	char tmp[1] = {0};
	int ret = 0;	
	struct edid_private_data* edid = 
		(struct edid_private_data *)i2c_get_clientdata(client);
	
	mutex_lock(&edid->i2c_mutex);	
	tmp[0] = addr;
	ret = edid_rx_data(client, tmp, 1);
	mutex_unlock(&edid->i2c_mutex);
	
	return tmp[0];
}



static int edid_tx_data_normal(struct i2c_client *client, char *buf, int num)
{
	int ret = 0;
	ret = i2c_master_normal_send(client, buf, num, EDID_I2C_RATE);
	
	return (ret == num) ? 0 : ret;
}


static int edid_rx_data_normal(struct i2c_client *client, char *buf, int num)
{
	int ret = 0;
	ret = i2c_master_normal_recv(client, buf, num, EDID_I2C_RATE);
	
	return (ret == num) ? 0 : ret;
}



static int edid_write_reg_normal(struct i2c_client *client, char value)
{
	char buffer[2];
	int ret = 0;
	struct edid_private_data* edid = 
		(struct edid_private_data *)i2c_get_clientdata(client);
	
	mutex_lock(&edid->i2c_mutex);	
	buffer[0] = value;
	ret = edid_tx_data_normal(client, &buffer[0], 1);	
	mutex_unlock(&edid->i2c_mutex);	
	return ret;
}

static int edid_read_reg_normal(struct i2c_client *client)
{
	char tmp[1] = {0};
	int ret = 0;	
	struct edid_private_data* edid = 
		(struct edid_private_data *)i2c_get_clientdata(client);
	
	mutex_lock(&edid->i2c_mutex);	
	ret = edid_rx_data_normal(client, tmp, 1);
	mutex_unlock(&edid->i2c_mutex);
	
	return tmp[0];
}


static int edid_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct edid_private_data *edid;
	struct board_id_private_data *board_id  = g_board_id;
	int result = 0;
	
	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		dev_err(&i2c->adapter->dev, "%s failed\n", __func__);
		return -ENODEV;
	}
	
	edid = kzalloc(sizeof(struct edid_private_data), GFP_KERNEL);
	if (edid == NULL)
		return -ENOMEM;

	i2c_set_clientdata(i2c, edid);

	edid->client = i2c;	
	mutex_init(&edid->i2c_mutex);
	
	result = edid_rx_data(i2c, edid->edid_data, 128);
	if(result)
	{
		printk("%s:fail to read edid data\n",__func__);
		return -1;
	}
	
	board_id->lcd_id_name[LCD_ID_EDID_I2C].id = LCD_ID_EDID_I2C;
	board_id->device_selected[DEVICE_TYPE_LCD].id = LCD_ID_EDID_I2C;


	edid->fops.owner = THIS_MODULE;
	edid->fops.unlocked_ioctl = edid_dev_ioctl;
	edid->fops.open = edid_dev_open;
	edid->fops.release = edid_dev_release;

	edid->miscdev.minor = MISC_DYNAMIC_MINOR;
	edid->miscdev.name = "edid_misc";
	edid->miscdev.fops = &edid->fops;

	edid->miscdev.parent = &i2c->dev;
	result = misc_register(&edid->miscdev);
	if (result < 0) {
		dev_err(&i2c->dev,
			"fail to register misc device %s\n", edid->miscdev.name);
		return result;
	}

	g_edid_data = edid;



	printk("%s:ok\n",__func__);
	
	return 0;
}

static int edid_i2c_remove(struct i2c_client *i2c)
{
	//struct edid_private_data *edid = i2c_get_clientdata(i2c);
	

	return 0;
}

static int edid_suspend(struct i2c_client *i2c, pm_message_t mesg)
{
	//struct edid_private_data *edid = i2c_get_clientdata(i2c);
	
	return 0;
}


static int edid_resume(struct i2c_client *i2c)
{
	//struct edid_private_data *edid = i2c_get_clientdata(i2c);
	
	return 0;
}


static const struct i2c_device_id edid_i2c_id[] = {
	{"lcd_edid_i2c", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, edid_i2c_id);

static struct i2c_driver edid_i2c_driver = {
	.driver = {
		.name = "lcd_edid_i2c",
		.owner = THIS_MODULE,
	},
	.probe = edid_i2c_probe,
	.remove = edid_i2c_remove,
	.suspend = edid_suspend,
	.resume	= edid_resume,
	.id_table = edid_i2c_id,
};

static int __init edid_i2c_init(void)
{
	int ret;

	printk("%s\n", __FUNCTION__);
	ret = i2c_add_driver(&edid_i2c_driver);
	if (ret != 0)
		pr_err("Failed to register edid I2C driver: %d\n", ret);

	return ret;
}
fs_initcall(edid_i2c_init);

static void __exit edid_i2c_exit(void)
{
	i2c_del_driver(&edid_i2c_driver);
}
module_exit(edid_i2c_exit);

