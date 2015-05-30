/* drivers/input/touchscreen/ektf2k.c - ELAN EKTF2K verions of driver
 *
 * Copyright (C) 2011 Elan Microelectronics Corporation.
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
 * 2011/12/06: The first release, version 0x0001
 * 2012/2/15:  The second release, version 0x0002 for new bootcode
 * 2012/5/8:   Release version 0x0003 for china market
 *             Integrated 2 and 5 fingers driver code together and
 *             auto-mapping resolution.
 * 2012/12/1:	 Release version 0x0005: support up to 10 fingers but no buffer mode.
 *             Please change following parameters
 *                 1. For 5 fingers protocol, please enable ELAN_PROTOCOL.
                      The packet size is 18 or 24 bytes.
 *                 2. For 10 fingers, please enable both ELAN_PROTOCOL and ELAN_TEN_FINGERS.
                      The packet size is 40 or 4+40+40+40 (Buffer mode) bytes.
 *                 3. Please enable the ELAN_BUTTON configuraton to support button.
 *								 4. For ektf3k serial, Add Re-Calibration Machanism 
 *                    So, please enable the define of RE_CALIBRATION.
 *                   
 *								 
 */

/* The ELAN_PROTOCOL support normanl packet format */	
#define ELAN_PROTOCOL		

#define ELAN_TEN_FINGERS   /* james check: Can not be use to auto-resolution mapping */
//#define ELAN_BUFFER_MODE
//#define ELAN_BUTTON
#define RE_CALIBRATION		/* The Re-Calibration was designed for ektf3k serial. */
//#define ELAN_2WIREICE
//#define ELAN_POWER_SOURCE

#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/miscdevice.h>

// for linux 2.6.36.3
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <asm/ioctl.h>
#include <linux/switch.h>
#include <linux/proc_fs.h>
#include <linux/wakelock.h>
#include <mach/board.h>
#include "ektf2k.h"

#ifdef ELAN_TEN_FINGERS
#define PACKET_SIZE		40		/* support 10 fingers packet for nexus7 55 */
#else
//#define PACKET_SIZE		8 		/* support 2 fingers packet  */
#define PACKET_SIZE		24			/* support 5 fingers packet  */
#endif

#define PWR_STATE_DEEP_SLEEP	0
#define PWR_STATE_NORMAL		1
#define PWR_STATE_MASK			BIT(3)

#define CMD_S_PKT		0x52
#define CMD_R_PKT		0x53
#define CMD_W_PKT		0x54

#define HELLO_PKT		0x55

#define TWO_FINGERS_PKT		0x5A
#define FIVE_FINGERS_PKT	0x5D
#define MTK_FINGERS_PKT		0x6D
#define TEN_FINGERS_PKT		0x62
#define BUFFER_PKT		0x63
#define BUFFER55_PKT		0x66

#define RESET_PKT		0x77
#define CALIB_PKT		0x66


// modify
#define SYSTEM_RESET_PIN_SR 62	// nexus7 TEGRA_GPIO_PH6	62	

//Add these Define
#define IAP_PORTION            	
#define PAGERETRY  30
#define IAPRESTART 5


// For Firmware Update 
#define ELAN_IOCTLID	0xD0
#define IOCTL_I2C_SLAVE	_IOW(ELAN_IOCTLID,  1, int)
#define IOCTL_MAJOR_FW_VER  _IOR(ELAN_IOCTLID, 2, int)
#define IOCTL_MINOR_FW_VER  _IOR(ELAN_IOCTLID, 3, int)
#define IOCTL_RESET  _IOR(ELAN_IOCTLID, 4, int)
#define IOCTL_IAP_MODE_LOCK  _IOR(ELAN_IOCTLID, 5, int)
#define IOCTL_CHECK_RECOVERY_MODE  _IOR(ELAN_IOCTLID, 6, int)
#define IOCTL_FW_VER  _IOR(ELAN_IOCTLID, 7, int)
#define IOCTL_X_RESOLUTION  _IOR(ELAN_IOCTLID, 8, int)
#define IOCTL_Y_RESOLUTION  _IOR(ELAN_IOCTLID, 9, int)
#define IOCTL_FW_ID  _IOR(ELAN_IOCTLID, 10, int)
#define IOCTL_ROUGH_CALIBRATE  _IOR(ELAN_IOCTLID, 11, int)
#define IOCTL_IAP_MODE_UNLOCK  _IOR(ELAN_IOCTLID, 12, int)
#define IOCTL_I2C_INT  _IOR(ELAN_IOCTLID, 13, int)
#define IOCTL_RESUME  _IOR(ELAN_IOCTLID, 14, int)
#define IOCTL_POWER_LOCK  _IOR(ELAN_IOCTLID, 15, int)
#define IOCTL_POWER_UNLOCK  _IOR(ELAN_IOCTLID, 16, int)
#define IOCTL_FW_UPDATE  _IOR(ELAN_IOCTLID, 17, int)
#define IOCTL_BC_VER  _IOR(ELAN_IOCTLID, 18, int)
#define IOCTL_2WIREICE  _IOR(ELAN_IOCTLID, 19, int)

#define CUSTOMER_IOCTLID	0xA0
#define IOCTL_CIRCUIT_CHECK  _IOR(CUSTOMER_IOCTLID, 1, int)
#define IOCTL_GET_UPDATE_PROGREE	_IOR(CUSTOMER_IOCTLID,  2, int)

#if 0
#define DBG(x...)   printk(x)
#else
#define DBG(x...)
#endif

uint8_t RECOVERY=0x00;
int FW_VERSION=0x00;

int X_RESOLUTION=2496;
int Y_RESOLUTION=1472;

int FW_ID=0x00;
int work_lock=0x00;
int power_lock=0x00;
int circuit_ver=0x01;
/*++++i2c transfer start+++++++*/
int file_fops_addr=0x15;
/*++++i2c transfer end+++++++*/

int button_state = 0;

#ifdef IAP_PORTION
uint8_t ic_status=0x00;	//0:OK 1:master fail 2:slave fail
int update_progree=0;
uint8_t I2C_DATA[3] = {0x10, 0x20, 0x21};/*I2C devices address*/  
int is_OldBootCode = 0; // 0:new 1:old


/*The newest firmware, if update must be changed here*/
static uint8_t file_fw_data[] = {
#include "ektf2k_fw_data.i"
};


enum
{
	PageSize		= 132,
	PageNum		        = 249,
	ACK_Fail		= 0x00,
	ACK_OK			= 0xAA,
	ACK_REWRITE		= 0x55,
};

enum
{
	E_FD			= -1,
};
#endif
struct elan_ktf2k_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct workqueue_struct *elan_wq;
	struct work_struct work;
	struct early_suspend early_suspend;
	int intr_gpio;
	int reset_gpio;
// Firmware Information
	int fw_ver;
	int fw_id;
	int bc_ver;
	int x_resolution;
	int y_resolution;
// For Firmare Update 
	struct miscdevice firmware;
};

static struct elan_ktf2k_ts_data *private_ts;
static int __fw_packet_handler(struct i2c_client *client);
static int elan_ktf2k_ts_rough_calibrate(struct i2c_client *client);
static int elan_ktf2k_ts_resume(struct i2c_client *client);

#ifdef IAP_PORTION
int Update_FW_One(/*struct file *filp,*/ struct i2c_client *client, int recovery);
static int __hello_packet_handler(struct i2c_client *client);
#endif

#ifdef ELAN_2WIREICE
int elan_TWO_WIRE_ICE( struct i2c_client *client);
#endif

// For Firmware Update 
int elan_iap_open(struct inode *inode, struct file *filp){ 
	struct elan_ktf2k_ts_data *ts = container_of(filp->private_data, struct elan_ktf2k_ts_data, firmware);
	pr_info("elan_iap_open, ts=0x%x, intr_pin=%d\n", (unsigned int)ts, ts->intr_gpio);

    DBG("[ELAN]into elan_iap_open\n");
        if (private_ts == NULL)  DBG("private_ts is NULL~~~");
		
	return 0;
}

int elan_iap_release(struct inode *inode, struct file *filp){    
	struct elan_ktf2k_ts_data *ts = container_of(filp->private_data, struct elan_ktf2k_ts_data, firmware);
	pr_info("elan_iap_release, ts=0x%x, intr_pin=%d\n", (unsigned int)ts, ts->intr_gpio);
	return 0;
}

static ssize_t elan_iap_write(struct file *filp, const char *buff, size_t count, loff_t *offp){  
	struct elan_ktf2k_ts_data *ts = container_of(filp->private_data, struct elan_ktf2k_ts_data, firmware);
    int ret;
    char *tmp;
	
	pr_info("elan_iap_write, ts=0x%x, intr_pin=%d\n", (unsigned int)ts, ts->intr_gpio);

    DBG("[ELAN]into elan_iap_write\n");
#if 0		/* get rid of extra warning. */
    /*++++i2c transfer start+++++++*/    	
    struct i2c_adapter *adap = private_ts->client->adapter;    	
    struct i2c_msg msg;
    /*++++i2c transfer end+++++++*/	
#endif
    if (count > 8192)
        count = 8192;

    tmp = kmalloc(count, GFP_KERNEL);
    
    if (tmp == NULL)
        return -ENOMEM;

    if (copy_from_user(tmp, buff, count)) {
        return -EFAULT;
    }

/*++++i2c transfer start+++++++*/
#if 0
	//down(&worklock);
	msg.addr = file_fops_addr;
	msg.flags = 0x00;// 0x00
	msg.len = count;
	msg.buf = (char *)tmp;
	//up(&worklock);
	ret = i2c_transfer(adap, &msg, 1);
#else
	
    ret = i2c_master_send(private_ts->client, tmp, count);
#endif	
/*++++i2c transfer end+++++++*/

    //if (ret != count) DBG("ELAN i2c_master_send fail, ret=%d \n", ret);
    kfree(tmp);
    //return ret;
    return (ret == 1) ? count : ret;

}

ssize_t elan_iap_read(struct file *filp, char *buff, size_t count, loff_t *offp){    
    struct elan_ktf2k_ts_data *ts = container_of(filp->private_data, struct elan_ktf2k_ts_data, firmware);
    char *tmp;
    int ret;  
    long rc;
    DBG("[ELAN]into elan_iap_read\n");
	pr_info("elan_iap_read, ts=0x%x, intr_pin=%d\n", (unsigned int)ts, ts->intr_gpio);
#if 0 	/* get rid of extra warning. */
   	/*++++i2c transfer start+++++++*/
    	struct i2c_adapter *adap = private_ts->client->adapter;
    	struct i2c_msg msg;
	/*++++i2c transfer end+++++++*/
#endif
    if (count > 8192)
        count = 8192;

    tmp = kmalloc(count, GFP_KERNEL);

    if (tmp == NULL)
        return -ENOMEM;
/*++++i2c transfer start+++++++*/
#if 0
	//down(&worklock);
	msg.addr = file_fops_addr;
	//msg.flags |= I2C_M_RD;
	msg.flags = 0x00;
	msg.flags |= I2C_M_RD;
	msg.len = count;
	msg.buf = tmp;
	//up(&worklock);
	ret = i2c_transfer(adap, &msg, 1);
#else
    ret = i2c_master_recv(private_ts->client, tmp, count);
#endif
/*++++i2c transfer end+++++++*/
    if (ret >= 0)
        rc = copy_to_user(buff, tmp, count);
    
    kfree(tmp);

    //return ret;
    return (ret == 1) ? count : ret;
	
}

static long elan_iap_ioctl( struct file *filp,    unsigned int cmd, unsigned long arg){
    struct elan_ktf2k_ts_data *ts = container_of(filp->private_data, struct elan_ktf2k_ts_data, firmware);
	int __user *ip = (int __user *)arg;
	DBG("[ELAN]into elan_iap_ioctl\n");
	DBG("cmd value %x\n",cmd);

	pr_info("elan_iap_ioctl, ts=0x%x, intr_pin=%d\n", (unsigned int)ts, ts->intr_gpio);
	
	switch (cmd) {        
		case IOCTL_I2C_SLAVE: 
			private_ts->client->addr = (int __user)arg;
			//file_fops_addr = 0x15;
			break;   
		case IOCTL_MAJOR_FW_VER:            
			break;        
		case IOCTL_MINOR_FW_VER:            
			break;        
		case IOCTL_RESET:
// modify
				gpio_set_value(ts->reset_gpio, 0);
				msleep(20);
				gpio_set_value(ts->reset_gpio, 1);
				msleep(5);
/* should be removed
		        gpio_set_value(SYSTEM_RESET_PIN_SR, 0);
		        msleep(20);
		        gpio_set_value(SYSTEM_RESET_PIN_SR, 1);
		        msleep(5);
*/
			break;
		case IOCTL_IAP_MODE_LOCK:
			if(work_lock==0)
			{
				work_lock=1;
				disable_irq(private_ts->client->irq);
				cancel_work_sync(&private_ts->work);
			}
			break;
		case IOCTL_IAP_MODE_UNLOCK:
			if(work_lock==1)
			{			
				work_lock=0;
				enable_irq(private_ts->client->irq);
			}
			break;
		case IOCTL_CHECK_RECOVERY_MODE:
			return RECOVERY;
			break;
		case IOCTL_FW_VER:
			__fw_packet_handler(private_ts->client);
			return FW_VERSION;
			break;
		case IOCTL_X_RESOLUTION:
			__fw_packet_handler(private_ts->client);
			return X_RESOLUTION;
			break;
		case IOCTL_Y_RESOLUTION:
			__fw_packet_handler(private_ts->client);
			return Y_RESOLUTION;
			break;
		case IOCTL_FW_ID:
			__fw_packet_handler(private_ts->client);
			return FW_ID;
			break;
		case IOCTL_ROUGH_CALIBRATE:
			return elan_ktf2k_ts_rough_calibrate(private_ts->client);
		case IOCTL_I2C_INT:
			put_user(gpio_get_value(private_ts->intr_gpio), ip);
			break;	
		case IOCTL_RESUME:
			elan_ktf2k_ts_resume(private_ts->client);
			break;	
		case IOCTL_POWER_LOCK:
			power_lock=1;
			break;
		case IOCTL_POWER_UNLOCK:
			power_lock=0;
			break;
#ifdef IAP_PORTION		
		case IOCTL_GET_UPDATE_PROGREE:
			update_progree=(int __user)arg;
			break; 
		case IOCTL_FW_UPDATE:
			Update_FW_One(private_ts->client, 0);
			break;
#endif
#ifdef ELAN_2WIREICE
		case IOCTL_2WIREICE:
			elan_TWO_WIRE_ICE(private_ts->client);
			break;		
#endif
		case IOCTL_CIRCUIT_CHECK:
			return circuit_ver;
			break;
		default:      
			DBG("[elan] Un-known IOCTL Command %d\n", cmd);      
			break;   
	}       
	return 0;
}

struct file_operations elan_touch_fops = {    
        .open =         elan_iap_open,    
        .write =        elan_iap_write,    
        .read = 	elan_iap_read,    
        .release =	elan_iap_release,    
	.unlocked_ioctl=elan_iap_ioctl, 
 };
#ifdef IAP_PORTION
int EnterISPMode(struct i2c_client *client, uint8_t  *isp_cmd)
{
	char buff[4] = {0};
	int len = 0;
	
	len = i2c_master_send(private_ts->client, isp_cmd,  sizeof(isp_cmd));
	if (len != sizeof(buff)) {
		DBG("[ELAN] ERROR: EnterISPMode fail! len=%d\r\n", len);
		return -1;
	}
	else
		DBG("[ELAN] IAPMode write data successfully! cmd = [%2x, %2x, %2x, %2x]\n", isp_cmd[0], isp_cmd[1], isp_cmd[2], isp_cmd[3]);
	return 0;
}

int ExtractPage(struct file *filp, uint8_t * szPage, int byte)
{
	int len = 0;

	len = filp->f_op->read(filp, szPage,byte, &filp->f_pos);
	if (len != byte) 
	{
		DBG("[ELAN] ExtractPage ERROR: read page error, read error. len=%d\r\n", len);
		return -1;
	}

	return 0;
}

int WritePage(uint8_t * szPage, int byte)
{
	int len = 0;

	len = i2c_master_send(private_ts->client, szPage,  byte);
	if (len != byte) 
	{
		DBG("[ELAN] ERROR: write page error, write error. len=%d\r\n", len);
		return -1;
	}

	return 0;
}

int GetAckData(struct i2c_client *client)
{
	int len = 0;

	char buff[2] = {0};
	
	len=i2c_master_recv(private_ts->client, buff, sizeof(buff));
	if (len != sizeof(buff)) {
		DBG("[ELAN] ERROR: read data error, write 50 times error. len=%d\r\n", len);
		return -1;
	}

	pr_info("[ELAN] GetAckData:%x,%x",buff[0],buff[1]);
	if (buff[0] == 0xaa/* && buff[1] == 0xaa*/) 
		return ACK_OK;
	else if (buff[0] == 0x55 && buff[1] == 0x55)
		return ACK_REWRITE;
	else
		return ACK_Fail;

	return 0;
}

void print_progress(int page, int ic_num, int j)
{
	int i, percent,page_tatol,percent_tatol;
	char str[256];
	str[0] = '\0';
	for (i=0; i<((page)/10); i++) {
		str[i] = '#';
		str[i+1] = '\0';
	}
	
	page_tatol=page+249*(ic_num-j);
	percent = ((100*page)/(249));
	percent_tatol = ((100*page_tatol)/(249*ic_num));

	if ((page) == (249))
		percent = 100;

	if ((page_tatol) == (249*ic_num))
		percent_tatol = 100;		

	DBG("\rprogress %s| %d%%", str, percent);
	
	if (page == (249))
		DBG("\n");
}
/* 
* Restet and (Send normal_command ?)
* Get Hello Packet
*/
void elan_ktf2k_ts_hw_reset(struct i2c_client *client)
{
	/* get rid of extra warning. */
	/* int res; */
	/* reset */
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	pr_info("elan_ktf2k_ts_hw_reset, ts=0x%x, intr_gpio=%d\n", (unsigned int)ts, ts->intr_gpio);
	gpio_set_value(ts->reset_gpio, 0);
	msleep(20);
	gpio_set_value(ts->reset_gpio, 1);
	msleep(5);
/*
	gpio_set_value(SYSTEM_RESET_PIN_SR, 0);
	msleep(20);
	gpio_set_value(SYSTEM_RESET_PIN_SR, 1);
	msleep(5);
*/
}


int Update_FW_One(struct i2c_client *client, int recovery)
{
	int res = 0,ic_num = 1;
	int iPage = 0, rewriteCnt = 0; //rewriteCnt for PAGE_REWRITE
	int i = 0;
	uint8_t data;
	//struct timeval tv1, tv2;
	int restartCnt = 0; // For IAP_RESTART

	/* get rid of extra warning */	
	/* uint8_t recovery_buffer[4] = {0}; */
	int byte_count;
	uint8_t *szBuff = NULL;
	int curIndex = 0;
	uint8_t isp_cmd[] = {0x45, 0x49, 0x41, 0x50}; //{0x45, 0x49, 0x41, 0x50};
	  																						// 0x54, 0x00, 0x12, 0x34
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	pr_info("Update_FW_One, ts=0x%x, intr_pin=%d\n", (unsigned int)ts, ts->intr_gpio);

	dev_dbg(&client->dev, "[ELAN] %s:  ic_num=%d\n", __func__, ic_num);
IAP_RESTART:	
	//reset
// modify    


	data=I2C_DATA[0];//Master
	dev_dbg(&client->dev, "[ELAN] %s: address data=0x%x \r\n", __func__, data);

	if(recovery != 0x80)
	{
        DBG("[ELAN] Firmware upgrade normal mode !\n");
		gpio_set_value(ts->reset_gpio, 0);
		mdelay(20);
		gpio_set_value(ts->reset_gpio, 1);
		mdelay(5);
/*
		gpio_set_value(SYSTEM_RESET_PIN_SR,0);
		mdelay(20);
		gpio_set_value(SYSTEM_RESET_PIN_SR,1);
		mdelay(5);
*/
		res = EnterISPMode(private_ts->client, isp_cmd);	 //enter ISP mode
	} else
        DBG("[ELAN] Firmware upgrade recovery mode !\n");
	//res = i2c_master_recv(private_ts->client, recovery_buffer, 4);   //55 aa 33 cc 
	//DBG("[ELAN] recovery byte data:%x,%x,%x,%x \n",recovery_buffer[0],recovery_buffer[1],recovery_buffer[2],recovery_buffer[3]);		

	// Send Dummy Byte	
	DBG("[ELAN] send one byte data:%x,%x",private_ts->client->addr,data);
	res = i2c_master_send(private_ts->client, &data,  sizeof(data));
	if(res!=sizeof(data))
	{
		DBG("[ELAN] dummy error code = %d\n",res);
	}	
	mdelay(10);


	// Start IAP
	for( iPage = 1; iPage <= PageNum; iPage++ ) 
	{
PAGE_REWRITE:
#if 1 // 8byte mode
		// 8 bytes
		//szBuff = fw_data + ((iPage-1) * PageSize); 
		for(byte_count=1;byte_count<=17;byte_count++)
		{
			if(byte_count!=17)
			{		
	//			DBG("[ELAN] byte %d\n",byte_count);	
	//			DBG("curIndex =%d\n",curIndex);
				szBuff = file_fw_data + curIndex;
				curIndex =  curIndex + 8;

				//ioctl(fd, IOCTL_IAP_MODE_LOCK, data);
				res = WritePage(szBuff, 8);
			}
			else
			{
	//			DBG("byte %d\n",byte_count);
	//			DBG("curIndex =%d\n",curIndex);
				szBuff = file_fw_data + curIndex;
				curIndex =  curIndex + 4;
				//ioctl(fd, IOCTL_IAP_MODE_LOCK, data);
				res = WritePage(szBuff, 4); 
			}
		} // end of for(byte_count=1;byte_count<=17;byte_count++)
#endif 
#if 0 // 132byte mode		
		szBuff = file_fw_data + curIndex;
		curIndex =  curIndex + PageSize;
		res = WritePage(szBuff, PageSize);
#endif
//#if 0
		if(iPage==249 || iPage==1)
		{
			mdelay(600); 			 
		}
		else
		{
			mdelay(50); 			 
		}	
		res = GetAckData(private_ts->client);

		if (ACK_OK != res) 
		{
			mdelay(50); 
			DBG("[ELAN] ERROR: GetAckData fail! res=%d\r\n", res);
			if ( res == ACK_REWRITE ) 
			{
				rewriteCnt = rewriteCnt + 1;
				if (rewriteCnt == PAGERETRY)
				{
					DBG("[ELAN] ID 0x%02x %dth page ReWrite %d times fails!\n", data, iPage, PAGERETRY);
					return E_FD;
				}
				else
				{
					DBG("[ELAN] ---%d--- page ReWrite %d times!\n",  iPage, rewriteCnt);
					goto PAGE_REWRITE;
				}
			}
			else
			{
				restartCnt = restartCnt + 1;
				if (restartCnt >= 5)
				{
					DBG("[ELAN] ID 0x%02x ReStart %d times fails!\n", data, IAPRESTART);
					return E_FD;
				}
				else
				{
					DBG("[ELAN] ===%d=== page ReStart %d times!\n",  iPage, restartCnt);
					goto IAP_RESTART;
				}
			}
		}
		else
		{       DBG("  data : 0x%02x ",  data);  
			rewriteCnt=0;
			print_progress(iPage,ic_num,i);
		}

		mdelay(10);
	} // end of for(iPage = 1; iPage <= PageNum; iPage++)

	DBG("[ELAN] read Hello packet data!\n"); 	  
	res= __hello_packet_handler(client);
	if (res > 0)
		DBG("[ELAN] Update ALL Firmware successfully!\n");

	return 0;
}

#endif
// End Firmware Update

// Star 2wireIAP which used I2C to simulate JTAG function
#ifdef ELAN_2WIREICE
static uint8_t file_bin_data[] = {
#include "ektf2k_2wireice.i"
};

int write_ice_status=0;
int shift_out_16(struct i2c_client *client){
	int res;
        uint8_t buff[] = {0xbb,0xbb,0xbb,0xbb,0xbb,0xbb,0xbb,0xbb,0xbf,0xff};
	res = i2c_master_send(client, buff,  sizeof(buff));
	return res;
}
int tms_reset(struct i2c_client *client){
	int res;
	uint8_t buff[] = {0xee,0xee,0xea,0xe0};
	res = i2c_master_send(client, buff,  sizeof(buff));
	return res;
}

int mode_gen(struct i2c_client *client){
	int res;
	//uint8_t buff[] = {0xee,0xee,0xee,0x2a,0x6a,0x66,0xaa,0x66,0xa6,0xaa,0x66,0xae,0x2a,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xae};
	uint8_t buff[] = {0xee,0xee,0xee,0x20,0xa6,0xa6,0x6a,0xa6,0x6a,0x6a,0xa6,0x6a,0xe2,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xe0};
	uint8_t buff_1[] = {0x2a,0x6a,0xa6,0xa6,0x6e};
	char mode_buff[2]={0};
	res = i2c_master_send(client, buff,  sizeof(buff));
        res = i2c_master_recv(client, mode_buff, sizeof(mode_buff));
	DBG("[elan] mode_gen read: %x %x\n", mode_buff[0], mode_buff[1]);
	
	res = i2c_master_send(client, buff_1,  sizeof(buff_1));
	return res;
}

int word_scan_out(struct i2c_client *client){
	//DBG("[elan] fun = %s\n", __func__);
	int res;
	uint8_t buff[] = {0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x26,0x66};
	res = i2c_master_send(client, buff,  sizeof(buff));
	return res;
}

int long_word_scan_out(struct i2c_client *client){
	//DBG("[elan] fun = %s\n", __func__);
	int res;
	uint8_t buff[] = {0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x26,0x66};
	res = i2c_master_send(client, buff,  sizeof(buff));
	return res;
}


int bit_manipulation(int TDI, int TMS, int TCK, int TDO,int TDI_1, int TMS_1, int TCK_1, int TDO_1){
        int res; 
	res= ((TDI<<3 |TMS<<2 |TCK |TDO)<<4) |(TDI_1<<3 |TMS_1<<2 |TCK_1 |TDO_1);
	//DBG("[elan] func=%s, res=%x\n", __func__, res);
	return res;
}

int ins_write(struct i2c_client *client, uint8_t buf){
	int res=0;
	int length=13;
	//int write_buf[7]={0};
	uint8_t write_buf[7]={0};
	int TDI_bit[13]={0};
	int TMS_bit[13]={0};
	int i=0;
	uint8_t buf_rev=0;
        int TDI=0, TMS=0, TCK=0,TDO=0;
	int bit_tdi, bit_tms;
	int len;
		
	for(i=0;i<8;i++) 
	{
	     buf_rev = buf_rev | (((buf >> i) & 0x01) << (7-i));
	}
		
	//DBG( "[elan ]bit = %x, buf_rev = %x \n", buf, buf_rev); 
	
        TDI = (0x7<<10) | buf_rev <<2 |0x00;
        TMS = 0x1007;
	TCK=0x2;
	TDO=0;

	

	//DBG( "[elan ]TDI = %p\n", TDI); //6F -> 111F600 (1FD8)
	//DBG( "[elan ]TMS = %p\n", TMS); 
	
        for ( len=0; len<=length-1; len++){
		bit_tdi = TDI & 0x1;
		bit_tms = TMS & 0x1;
		//DBG( "[elan ]bit_tdi = %d, bit_tms = %d\n", bit_tdi, bit_tms );
		TDI_bit[length-1-len] =bit_tdi;
		TMS_bit[length-1-len] = bit_tms;
                TDI = TDI >>1;
		TMS = TMS >>1;
	}


       /*for (len=0;len<=12;len++){
	   	DBG("[elan] TDI[%d]=%d,  TMS[%d]= %d TCK=%d, TDO=%d, write[buf]=%x\n", len, TDI_bit[len], len, TMS_bit[len], TCK, TDO, (TDI_bit[len]<<3 |TMS_bit[len]<<2 |TCK |TDO));
		DBG("[elan] %d, %d, %d, %d\n", TDI_bit[len] << 3 ,TMS_bit[len]<<2, TCK, TDO);
	}
        */
       /*
        for (len=0; len<=12;len=len+2){
		if (len !=12){
	            write_buf[len/2] =((TDI_bit[len]<<3 |TMS_bit[len]<<2|TCK |TDO)<<4) |((TDI_bit[len+1]<<3 |TMS_bit[len+1]<<2 |TCK |TDO));
 		    DBG("[elan] write_buf[%d]=%x\n", len/2, write_buf[len/2]) ;
		} else {
		    write_buf[len/2] = ((TDI_bit[len]<<3 |TMS_bit[len]<<2 |TCK |TDO)<<4) |0x0000;	
	 	    DBG("[elan] write_buf[%d]=%x\n", len/2, write_buf[len/2]) ;
		}
	}
        */
        for (len=0;len<=length-1;len=len+2){
	     if (len == length-1 && len%2 ==0)
		 res = bit_manipulation(TDI_bit[len], TMS_bit[len], TCK, TDO, 0, 0, 0, 0); 	
	     else
		 res = bit_manipulation(TDI_bit[len], TMS_bit[len], TCK, TDO, TDI_bit[len+1], TMS_bit[len+1], TCK, TDO); 	
	     write_buf[len/2] = res;
        }

/* for debug msg
       for(len=0;len<=(length-1)/2;len++){
		DBG("[elan] write_buf[%d]=%x\n", len, write_buf[len]);
	}
*/
        res = i2c_master_send(client, write_buf,  sizeof(write_buf));
	return res;
}


int word_scan_in(struct i2c_client *client, uint16_t buf){
	int res=0;
	uint8_t write_buf[10]={0};
	int TDI_bit[20]={0};
	int TMS_bit[20]={0};
	
	
        int TDI =  buf <<2 |0x00;
        int  TMS = 0x7;
	int  TCK=0x2;
	int TDO=0;
	
	int bit_tdi, bit_tms;
	int len;
	//DBG( "[elan] fun =%s,   %x\n", __func__,buf); 
	
	//DBG("[elan] work_scan_in, buf=%x\n", buf);
	
	//DBG( "[elan]TDI = %p\n", TDI); //0302 ->  (c08)
	//DBG( "[elan]TMS = %p\n", TMS); //7
	
        for ( len=0; len<=19; len++){    //length =20
		bit_tdi = TDI & 0x1;
		bit_tms = TMS & 0x1;
		//DBG( "[elan ]bit_tdi = %d, bit_tms = %d\n", bit_tdi, bit_tms );
		
		TDI_bit[19-len] =bit_tdi;
		TMS_bit[19-len] = bit_tms;
                TDI = TDI >>1;
		TMS = TMS >>1;
	}

/* for debug msg
       for (len=0;len<=19;len++){
	   	DBG("[elan] TDI[%d]=%d,  TMS[%d]= %d TCK=%d, TDO=%d, write[buf]=%x\n", len, TDI_bit[len], len, TMS_bit[len], TCK, TDO, (TDI_bit[len]<<3 |TMS_bit[len]<<2 |TCK |TDO));
		DBG("[elan] %d, %d, %d, %d\n", TDI_bit[len] << 3 ,TMS_bit[len]<<2, TCK, TDO);
	}
        
    
        for (len=0; len<=19;len=len+2){
	            write_buf[len/2] =((TDI_bit[len]<<3 |TMS_bit[len]<<2|TCK |TDO)<<4) |((TDI_bit[len+1]<<3 |TMS_bit[len+1]<<2 |TCK |TDO));
 		    DBG("[elan] write_buf[%d]=%x\n", len/2, write_buf[len/2]) ;
	}
*/
        for (len=0;len<=19;len=len+2){
	     if (len == 19 && len%2 ==0)
	 	res = bit_manipulation(TDI_bit[len], TMS_bit[len], TCK, TDO, 0,0,0,0); 
	     else
                res = bit_manipulation(TDI_bit[len], TMS_bit[len], TCK, TDO, TDI_bit[len+1], TMS_bit[len+1], TCK, TDO); 
	        write_buf[len/2] = res;
        }

 /*/for debug msg
        for(len=0;len<=9;len++){
		DBG("[elan] write_buf[%d]=%x\n", len, write_buf[len]);
	}
*/
        
	res = i2c_master_send(client, write_buf,  sizeof(write_buf));
	return res;
}

int long_word_scan_in(struct i2c_client *client, int buf_1, int buf_2){
       	uint8_t write_buf[18]={0};
	int TDI_bit[36]={0};
	int TMS_bit[36]={0};
	//DBG( "[elan] fun =%s, %x,   %x\n", __func__,buf_1, buf_2); 
	
        int TDI =  buf_1 <<18|buf_2<<2 |0x00;
        int  TMS = 0x7;
	int  TCK=0x2;
	int TDO=0;
	
	int bit_tdi, bit_tms;
	int len;
	int res=0;
	
	//DBG( "[elan]TDI = %p\n", TDI); //007e,0020 ->  (1f80080)
	//DBG( "[elan]TMS = %p\n", TMS); //7

        for ( len=0; len<=35; len++){    //length =36
		bit_tdi = TDI & 0x1;
		bit_tms = TMS & 0x1;
		
		TDI_bit[35-len] =bit_tdi;
		TMS_bit[35-len] = bit_tms;
                TDI = TDI >>1;
		TMS = TMS >>1;
	}


        for (len=0;len<=35;len=len+2){
	     if (len == 35 && len%2 ==0)
	 	res = bit_manipulation(TDI_bit[len], TMS_bit[len], TCK, TDO, 0,0,0,0); 
	     else
                res = bit_manipulation(TDI_bit[len], TMS_bit[len], TCK, TDO, TDI_bit[len+1], TMS_bit[len+1], TCK, TDO); 
	     write_buf[len/2] = res;
        }
/* for debug msg
        for(len=0;len<=17;len++){
		DBG("[elan] write_buf[%d]=%x\n", len, write_buf[len]);
	}
*/		
        res = i2c_master_send(client, write_buf,  sizeof(write_buf));
	return res;
}

uint16_t trimtable[8]={0};

int Read_SFR(struct i2c_client *client, int open){
        uint8_t voltage_recv[2]={0};
	
	int count, ret;
	//uint16_t address_1[8]={0x0000,0x0001,0x0002,0x0003,0x0004,0x0005,0x0006,0x0007};
	        
	ins_write(client, 0x6f); // IO write
	long_word_scan_in(client, 0x007e, 0x0020);  
	long_word_scan_in(client, 0x007f, 0x4000);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);

        //  0
	ins_write(client, 0x6f);  //IO Write
	long_word_scan_in(client, 0x007f, 0x9002);  //TM=2h
	ins_write(client, 0x68);  //Program Memory Sequential Read
	word_scan_in(client, 0x0000);  //set Address 0x0000
        shift_out_16(client);   //move data to I2C buf
		
	mdelay(10);
	count = 0;
        ret = i2c_master_recv(client, voltage_recv, sizeof(voltage_recv)); 
        trimtable[count]=voltage_recv[0]<<8 | voltage_recv[1];
	//DBG("[elan] Open_High_Voltage recv -1 0 word =%x %x, trimtable[%d]=%x \n", voltage_recv[0],voltage_recv[1], count, trimtable[count]); 

        //  1
	ins_write(client, 0x6f); // IO write
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9002);
	ins_write(client, 0x68);
	word_scan_in(client, 0x0001);
        shift_out_16(client); 

	mdelay(10);
	count=1;
        ret = i2c_master_recv(client, voltage_recv, sizeof(voltage_recv)); 
        trimtable[count]=voltage_recv[0]<<8 | voltage_recv[1];
	//DBG("[elan] Open_High_Voltage recv -1 1word =%x %x, trimtable[%d]=%x \n", voltage_recv[0],voltage_recv[1], count, trimtable[count]); 
	

        //  2
	ins_write(client, 0x6f); // IO write
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9002);
	ins_write(client, 0x68);
	word_scan_in(client, 0x0002);
        shift_out_16(client); 

	mdelay(10);
	count=2;
        ret = i2c_master_recv(client, voltage_recv, sizeof(voltage_recv)); 
        trimtable[count]=voltage_recv[0]<<8 | voltage_recv[1];
	//DBG("[elan] Open_High_Voltage recv -1 1word =%x %x, trimtable[%d]=%x \n", voltage_recv[0],voltage_recv[1], count, trimtable[count]); 
	

        //  3
	ins_write(client, 0x6f); // IO write
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9002);
	ins_write(client, 0x68);
	word_scan_in(client, 0x0003);
        shift_out_16(client); 

	mdelay(10);
	count=3;
        ret = i2c_master_recv(client, voltage_recv, sizeof(voltage_recv)); 
        trimtable[count]=voltage_recv[0]<<8 | voltage_recv[1];
	//DBG("[elan] Open_High_Voltage recv -1 1word =%x %x, trimtable[%d]=%x \n", voltage_recv[0],voltage_recv[1], count, trimtable[count]); 

        //  4
	ins_write(client, 0x6f); // IO write
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9002);
	ins_write(client, 0x68);
	word_scan_in(client, 0x0004);
        shift_out_16(client); 

	mdelay(10);
	count=4;
        ret = i2c_master_recv(client, voltage_recv, sizeof(voltage_recv)); 
        trimtable[count]=voltage_recv[0]<<8 | voltage_recv[1];
	//DBG("[elan] Open_High_Voltage recv -1 1word =%x %x, trimtable[%d]=%x \n", voltage_recv[0],voltage_recv[1], count, trimtable[count]); 
	


        //  5
	ins_write(client, 0x6f); // IO write
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9002);
	ins_write(client, 0x68);
	word_scan_in(client, 0x0005);
        shift_out_16(client); 

	mdelay(10);
	count=5;
        ret = i2c_master_recv(client, voltage_recv, sizeof(voltage_recv)); 
        trimtable[count]=voltage_recv[0]<<8 | voltage_recv[1];
	//DBG("[elan] Open_High_Voltage recv -1 1word =%x %x, trimtable[%d]=%x \n", voltage_recv[0],voltage_recv[1], count, trimtable[count]); 
	
	
        //  6
	ins_write(client, 0x6f); // IO write
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9002);
	ins_write(client, 0x68);
	word_scan_in(client, 0x0006);
        shift_out_16(client); 

	mdelay(10);
	count=6;
        ret = i2c_master_recv(client, voltage_recv, sizeof(voltage_recv)); 
        trimtable[count]=voltage_recv[0]<<8 | voltage_recv[1];
	//DBG("[elan] Open_High_Voltage recv -1 1word =%x %x, trimtable[%d]=%x \n", voltage_recv[0],voltage_recv[1], count, trimtable[count]); 
	

	//  7
	ins_write(client, 0x6f); // IO write
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9002);
	ins_write(client, 0x68);
	word_scan_in(client, 0x0007);
        shift_out_16(client); 

	mdelay(10);
	count=7;
        ret = i2c_master_recv(client, voltage_recv, sizeof(voltage_recv)); 
	DBG("open= %d\n", open);
	if (open == 1)
            trimtable[count]=voltage_recv[0]<<8 |  (voltage_recv[1] & 0xbf);
	else
            trimtable[count]=voltage_recv[0]<<8 | (voltage_recv[1] | 0x40);
	DBG("[elan] Open_High_Voltage recv -1 1word =%x %x, trimtable[%d]=%x \n", voltage_recv[0],voltage_recv[1], count, trimtable[count]); 


        ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x8000);


	 
/*	
	for (count =0; count <8; count++){

	ins_write(client, 0x6f); // IO write
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9002);
	ins_write(client, 0x68);
	word_scan_in(client, address_1[count]);
        shift_out_16(client); 

	mdelay(10);
	//count=6;
        ret = i2c_master_recv(client, voltage_recv, sizeof(voltage_recv)); 
        trimtable[count]=voltage_recv[0]<<8 | voltage_recv[1];
	DBG("[elan] Open_High_Voltage recv -1 1word =%x %x, trimtable[%d]=%x \n", voltage_recv[0],voltage_recv[1], count, trimtable[count]); 

	}
	
	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x8000);

*/


	
	return 0;
}

int Write_SFR(struct i2c_client *client){

       ins_write(client, 0x6f);
       long_word_scan_in(client, 0x007f, 0x9001);


       ins_write(client, 0x66);  // Program Memory Write
       long_word_scan_in(client, 0x0000, trimtable[0]);
       ins_write(client, 0xfd);  //Set up the initial addr for sequential access
       word_scan_in(client,0x7f);
	
       ins_write(client, 0x66);
       long_word_scan_in(client, 0x0001, trimtable[1]);
       ins_write(client, 0xfd);
       word_scan_in(client,0x7f);

       ins_write(client, 0x66);
       long_word_scan_in(client, 0x0002, trimtable[2]);
       ins_write(client, 0xfd);
       word_scan_in(client,0x7f);

       ins_write(client, 0x66);
       long_word_scan_in(client, 0x0003, trimtable[3]);
       ins_write(client, 0xfd);
       word_scan_in(client,0x7f);

       ins_write(client, 0x66);
       long_word_scan_in(client, 0x0004, trimtable[4]);
       ins_write(client, 0xfd);
       word_scan_in(client,0x7f);

       ins_write(client, 0x66);
       long_word_scan_in(client, 0x0005, trimtable[5]);
       ins_write(client, 0xfd);
       word_scan_in(client,0x7f);
	   
       ins_write(client, 0x66);
       long_word_scan_in(client, 0x0006, trimtable[6]);	
       ins_write(client, 0xfd);
       word_scan_in(client,0x7f);

       ins_write(client, 0x66);
       long_word_scan_in(client, 0x0007, trimtable[7]);
       ins_write(client, 0xfd);
       word_scan_in(client,0x7f);


       ins_write(client, 0x6f);
       long_word_scan_in(client, 0x7f, 0x8000);	   
       /*
       for (count=0;count<8;count++){
              ins_write(client, 0x66);
	      long_word_scan_in(client, 0x0000+count, trimtable[count]);
		
       }
	*/

	return 0;
}

int Enter_Mode(struct i2c_client *client){
	mode_gen(client);
	tms_reset(client);
	ins_write(client,0xfc); //system reset
	tms_reset(client);
	return 0;
}
int Open_High_Voltage(struct i2c_client *client, int open){
	Read_SFR(client, open);
	Write_SFR(client);
        Read_SFR(client, open);
	return 0;
}

int Mass_Erase(struct i2c_client *client){
	char mass_buff[4]={0};
	char mass_buff_1[2]={0};
	int ret, finish=0, i=0;
	DBG("[Elan] Mass_Erase!!!!\n");
        
	ins_write(client,0x01); //id code read
	mdelay(2);
	long_word_scan_out(client);

	ret = i2c_master_recv(client, mass_buff, sizeof(mass_buff));
	DBG("[elan] Mass_Erase mass_buff=%x %x %x %x(c0 08 01 00)\n", mass_buff[0],mass_buff[1],mass_buff[2],mass_buff[3]);  //id: c0 08 01 00
	
/* / add for test
	ins_write(client, 0xf3);
        word_scan_out(client);
        ret = i2c_master_recv(client, mass_buff_1, sizeof(mass_buff_1));
	DBG("[elan] Mass_Erase mass_buff_1=%x %x(a0 00)\n", mass_buff_1[0],mass_buff_1[1]);  // a0 00 : stop
//add for test

	//read low->high 5th bit
	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);

// add for test
	ins_write(client, 0xf3);
        word_scan_out(client);
        ret = i2c_master_recv(client, mass_buff_1, sizeof(mass_buff_1));
	DBG("[elan] Mass_Erase (II) mass_buff_1=%x %x(40 00)\n", mass_buff_1[0],mass_buff_1[1]);  // 40 00
//add for test
	mdelay(10); //for malata
	*/
	
	ins_write(client,0x6f);  //IO Write
	long_word_scan_in(client,0x007e,0x0023);
	long_word_scan_in(client,0x007f,0x8000);
	long_word_scan_in(client,0x007f,0x9040);
	ins_write(client,0x66); //Program data Write
	long_word_scan_in(client, 0x0008,0x8765);  //set ALE for flash
	ins_write(client,0x6f);  //IO Write
	long_word_scan_in(client, 0x007f,0x8000);	//clear flash control PROG

  ins_write(client,0xf3);
        
	while (finish==0){
	    word_scan_out(client);
	    ret=i2c_master_recv(client, mass_buff_1, sizeof(mass_buff_1));
	    finish = (mass_buff_1[1] >> 4 ) & 0x01;

	    DBG("[elan] mass_buff_1[0]=%x, mass_buff_1[1]=%x (80 10)!!!!!!!!!! finish=%d \n", mass_buff_1[0], mass_buff_1[1], finish); //80 10: OK, 80 00: fail
	    if (mass_buff_1[1]!= 0x10 && finish!=1 && i<100) {  
			mdelay(100);
			//DBG("[elan] mass_buff_1[1] >>4  !=1\n");
			i++;
			if (i == 50) {
                                DBG("[elan] Mass_Erase fail ! \n");
				//return -1;  //for test
			}
	    }
	    
	}

	return 0;
}

int Reset_ICE(struct i2c_client *client){
        //struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
        int res;
	DBG("[Elan] Reset ICE!!!!\n");
	ins_write(client, 0x94);
	ins_write(client, 0xd4);
	ins_write(client, 0x20);
client->addr = 0x10;////Modify address before 2-wire
DBG("[Elan] Modify address = %x\n ", client->addr);
	elan_ktf2k_ts_hw_reset(private_ts->client);  //???
	mdelay(250);

/*	gpio_set_value(SYSTEM_RESET_PIN_SR,0);
	mdelay(20);
	gpio_set_value(SYSTEM_RESET_PIN_SR,1);
	mdelay(100);*/
	 res = __hello_packet_handler(client);
	
	return 0;
}

int normal_write_func(struct i2c_client *client, int j, uint8_t *szBuff){
	//char buff_check=0;
	uint16_t szbuff=0, szbuff_1=0;
	uint16_t sendbuff=0;
        int write_byte, iw;
	
	ins_write(client,0xfd);
        word_scan_in(client, j*64); 
        
	ins_write(client,0x65);  //Program data sequential write

        write_byte =64;

        for(iw=0;iw<write_byte;iw++){ 
		szbuff = *szBuff;
		szbuff_1 = *(szBuff+1);
		sendbuff = szbuff_1 <<8 |szbuff;
		DBG("[elan]  Write Page sendbuff=0x%04x @@@\n", sendbuff);
		//mdelay(1);
		word_scan_in(client, sendbuff); //data????   buff_read_data
		szBuff+=2;
		
	}
        return 0;
}

int fastmode_write_func(struct i2c_client *client, int j, uint8_t *szBuff){
	 uint8_t szfwbuff=0, szfwbuff_1=0;
	 uint8_t sendfwbuff[130]={0};
	 uint16_t tmpbuff;
	 int i=0, len=0;
	 private_ts->client->addr = 0x76;

	 sendfwbuff[0] = (j*64)>>8;
	 tmpbuff = ((j*64)<< 8) >> 8;
	 sendfwbuff[1] = tmpbuff;
	 //DBG("fastmode_write_func, sendfwbuff[0]=0x%x, sendfwbuff[1]=0x%x\n", sendfwbuff[0], sendfwbuff[1]);
	 
	 for (i=2;i < 129; i=i+2) {      //  1 Page = 64 word, 1 word=2Byte
	 
	     szfwbuff = *szBuff;
	     szfwbuff_1 = *(szBuff+1);
	     sendfwbuff[i] = szfwbuff_1;
	     sendfwbuff[i+1] = szfwbuff;
	     szBuff+=2;
	     //DBG("[elan] sendfwbuff[%d]=0x%x, sendfwbuff[%d]=0x%x\n", i, sendfwbuff[i], i+1, sendfwbuff[i+1]);
	 }

	 
	 len = i2c_master_send(private_ts->client, sendfwbuff,  sizeof(sendfwbuff));
	 //DBG("fastmode_write_func, send len=%d (130), Page %d --\n", len, j);

	  private_ts->client->addr = 0x77;
	  
	return 0;
}


int ektSize;
int lastpage_byte;
int lastpage_flag=0;
int Write_Page(struct i2c_client *client, int j, uint8_t *szBuff){
	int len, finish=0;
	char buff_read_data[2];
	int i=0;
	
	ins_write(client,0x6f);   //IO Write
	long_word_scan_in(client,0x007e,0x0023);
	long_word_scan_in(client,0x007f,0x9400);

	ins_write(client,0x66);    //Program Data Write
	//long_word_scan_in(client,0x0000,0x5a5a);
	//DBG("[elan] j*64=0x%x @@ \n", j*64);

	long_word_scan_in(client, j*64,0x5a5a);  //set ALE
	
        //normal_write_func(client, j, szBuff); ////////////choose one : normal / fast mode
        fastmode_write_func(client, j, szBuff); //////////
        
	ins_write(client,0x6f);
	long_word_scan_in(client,0x007f,0x9000);

	//ins_write(client,0x6f);
	long_word_scan_in(client,0x007f,0x8000);

	ins_write(client, 0xf3);  //Debug Reg Read
	
	while (finish==0){
	    word_scan_out(client);
	    len=i2c_master_recv(client, buff_read_data, sizeof(buff_read_data));
	    finish = (buff_read_data[1] >> 4 ) & 0x01;
	    //DBG("[elan] Write_Page:buff_read_data[0]=%x, buff_read_data[1]=%x !!!!!!!!!! finish=%d \n", buff_read_data[0], buff_read_data[1], finish); //80 10: ok
	    if (finish!=1) {  
			mdelay(10);
			DBG("[elan] Write_Page finish !=1\n");
                        i++;
			if (i==50){ 
                                write_ice_status=1;
				return -1;
			}
	    }

	}
	return 0;
}

int fastmode_read_func(struct i2c_client *client, int j, uint8_t *szBuff){
	 uint8_t szfrbuff=0, szfrbuff_1=0;
	 uint8_t sendfrbuff[2]={0};
	 uint8_t recvfrbuff[130]={0};
	 uint16_t tmpbuff;
	 int i=0, len=0;
	 
	 ins_write(client,0x67);
	 
	 private_ts->client->addr = 0x76;

	 sendfrbuff[0] = (j*64)>>8;
	 tmpbuff = ((j*64)<< 8) >> 8;
	 sendfrbuff[1] = tmpbuff;
	 //DBG("fastmode_write_func, sendfrbuff[0]=0x%x, sendfrbuff[1]=0x%x\n", sendfrbuff[0], sendfrbuff[1]);
	 len = i2c_master_send(private_ts->client, sendfrbuff,  sizeof(sendfrbuff));

	 len = i2c_master_recv(private_ts->client, recvfrbuff,  sizeof(recvfrbuff));
	 //DBG("fastmode_read_func, recv len=%d (128)\n", len);
		 
         for (i=2;i < 129;i=i+2){ 
		szfrbuff=*szBuff;
	        szfrbuff_1=*(szBuff+1);
	        szBuff+=2;
		if (recvfrbuff[i] != szfrbuff_1 || recvfrbuff[i+1] != szfrbuff)  
	        {
		  	 DBG("[elan] @@@@Read Page Compare Fail. recvfrbuff[%d]=%x, recvfrbuff[i+1]=%x, szfrbuff_1=%x, szfrbuff=%x, ,j =%d@@@@@@@@@@@@@@@@\n\n", i,recvfrbuff[i], recvfrbuff[i+1], szfrbuff_1, szfrbuff, j);
		  	 write_ice_status=1;
		}
		break;//for test
         }

	  private_ts->client->addr = 0x77;
	  
	return 0;
}


int normal_read_func(struct i2c_client *client, int j,  uint8_t *szBuff){
        char read_buff[2];
        int m, len, read_byte;
	uint16_t szbuff=0, szbuff_1=0;
	
	ins_write(client,0xfd);
	
	//DBG("[elan] Read_Page, j*64=0x%x\n", j*64);
	word_scan_in(client, j*64);
	ins_write(client,0x67);

	word_scan_out(client);

        read_byte=64;
	//for(m=0;m<64;m++){
	for(m=0;m<read_byte;m++){
            // compare......
                word_scan_out(client);
	        len=i2c_master_recv(client, read_buff, sizeof(read_buff));

		szbuff=*szBuff;
	        szbuff_1=*(szBuff+1);
	        szBuff+=2;
	        DBG("[elan] Read Page: byte=%x%x, szbuff=%x%x \n", read_buff[0], read_buff[1],szbuff, szbuff_1);
	        if (read_buff[0] != szbuff_1 || read_buff[1] != szbuff) 
	        {
		  	 DBG("[elan] @@@@@@@@@@Read Page Compare Fail. j =%d. m=%d.@@@@@@@@@@@@@@@@\n\n", j, m);
		  	 write_ice_status=1;
		}
     }
     return 0;
}


int Read_Page(struct i2c_client *client, int j,  uint8_t *szBuff){
	
	ins_write(client,0x6f);
	long_word_scan_in(client,0x007e,0x0023);
	long_word_scan_in(client,0x007f,0x9000);

        //mdelay(10); //for malata
	//normal_read_func(client, j,  szBuff); ////////////////choose one: normal / fastmode
	fastmode_read_func(client, j,  szBuff);

	//Clear Flashce
	ins_write(client,0x6f);
	long_word_scan_in(client,0x007f,0x8000);
	return 0;
        
}

int TWO_WIRE_ICE(struct i2c_client *client){
     int i;
     
     //test	 
     uint8_t *szBuff = NULL;
     int curIndex = 0;
     int PageSize=128;
     int res;
     //int ektSize;
     //test
     write_ice_status=0;
     ektSize = sizeof(file_bin_data) /PageSize;

client->addr = 0x77;////Modify address before 2-wire
     
     DBG("[Elan] ektSize=%d ,modify address = %x\n ", ektSize, client->addr);
     //test	
     i = Enter_Mode(client);
     i = Open_High_Voltage(client, 1);     
     if (i == -1)
     {
	 DBG("[Elan] Open High Voltage fail\n");
	return -1;
     }
    //return 0;
	
     i = Mass_Erase(client);  //mark temp
     if (i == -1)  {
	 DBG("[Elan] Mass Erase fail\n");
	return -1;
     }

    
     //for fastmode
     ins_write(client,0x6f);
     long_word_scan_in(client, 0x7e, 0x36);
     long_word_scan_in(client, 0x7f, 0x8000);	
     long_word_scan_in(client, 0x7e, 0x37);	 
     long_word_scan_in(client, 0x7f, 0x76);

	

// client->addr = 0x76;////Modify address before 2-wire 
 DBG("[Elan-test] client->addr =%2x\n", client->addr);    
     //for fastmode
     for (i =0 ; i<ektSize; i++)
     {
	szBuff = file_bin_data + curIndex; 
        curIndex =  curIndex + PageSize; 	
//	DBG("[Elan] Write_Page %d........................wait\n ", i);	

        res=Write_Page(client, i, szBuff);
	if (res == -1) 
	{
		DBG("[Elan] Write_Page %d fail\n ", i);
		break;
	}
	      //DBG("[Elan] Read_Page %d........................wait\n ", i);
	mdelay(1);
        Read_Page(client,i, szBuff);
	//DBG("[Elan] Finish  %d  Page!!!!!!!.........wait\n ", i);	
     }
//client->addr = 0x77;////Modify address before 2-wire
DBG("[Elan-test] client->addr =%2x\n", client->addr); 

     if(write_ice_status==0)
     {
     	DBG("[elan] Update_FW_Boot Finish!!! \n");
     }
     else
     {
     	DBG("[elan] Update_FW_Boot fail!!! \n");
     }
 DBG("[Elan-test] close High Voltage \n");
     i = Open_High_Voltage(client, 0);     
     if (i == -1) return -1; //test

     Reset_ICE(client);

     return 0;	
}
int elan_TWO_WIRE_ICE( struct i2c_client *client) // for driver internal 2-wire ice
{
work_lock=1;
disable_irq(private_ts->client->irq);
//wake_lock(&private_ts->wakelock);
	TWO_WIRE_ICE(client);
work_lock=0;
enable_irq(private_ts->client->irq);
//wake_unlock(&private_ts->wakelock);
	return 0;
}
// End 2WireICE
#endif


// Start sysfs
static ssize_t elan_ktf2k_gpio_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct elan_ktf2k_ts_data *ts = private_ts;

	ret = gpio_get_value(ts->intr_gpio);
	DBG(KERN_DEBUG "GPIO_TP_INT_N=%d\n", ts->intr_gpio);
	sprintf(buf, "GPIO_TP_INT_N=%d\n", ret);
	ret = strlen(buf) + 1;
	return ret;
}

static DEVICE_ATTR(gpio, S_IRUGO, elan_ktf2k_gpio_show, NULL);

static ssize_t elan_ktf2k_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct elan_ktf2k_ts_data *ts = private_ts;

	sprintf(buf, "%s_x%4.4x\n", "ELAN_KTF2K", ts->fw_ver);
	ret = strlen(buf) + 1;
	return ret;
}

static DEVICE_ATTR(vendor, S_IRUGO, elan_ktf2k_vendor_show, NULL);

static struct kobject *android_touch_kobj;

static int elan_ktf2k_touch_sysfs_init(void)
{
	int ret ;

	android_touch_kobj = kobject_create_and_add("android_touch", NULL) ;
	if (android_touch_kobj == NULL) {
		DBG(KERN_ERR "[elan]%s: subsystem_register failed\n", __func__);
		ret = -ENOMEM;
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_gpio.attr);
	if (ret) {
		DBG(KERN_ERR "[elan]%s: sysfs_create_file failed\n", __func__);
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_vendor.attr);
	if (ret) {
		DBG(KERN_ERR "[elan]%s: sysfs_create_group failed\n", __func__);
		return ret;
	}
	return 0 ;
}

static void elan_touch_sysfs_deinit(void)
{
	sysfs_remove_file(android_touch_kobj, &dev_attr_vendor.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_gpio.attr);
	kobject_del(android_touch_kobj);
}	

// end sysfs

static int __elan_ktf2k_ts_poll(struct i2c_client *client)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	int status = 0, retry = 10;

	do {
		status = gpio_get_value(ts->intr_gpio);
		DBG("%s: status = %d\n", __func__, status);
		retry--;
		mdelay(50);
	} while (status == 1 && retry > 0);

	DBG( "[elan]%s: poll interrupt status %s\n",
			__func__, status == 1 ? "high" : "low");
	return (status == 0 ? 0 : -ETIMEDOUT);
}

static int elan_ktf2k_ts_poll(struct i2c_client *client)
{
	return __elan_ktf2k_ts_poll(client);
}

static int elan_ktf2k_ts_get_data(struct i2c_client *client, uint8_t *cmd,
			uint8_t *buf, size_t size)
{
	int rc;
#ifdef RE_CALIBRATION
    uint8_t buf_recv[8] = { 0 };
#endif


	dev_dbg(&client->dev, "[elan]%s: enter\n", __func__);

	if (buf == NULL)
		return -EINVAL;

	if ((i2c_master_send(client, cmd, 4)) != 4) {
		dev_err(&client->dev,
			"[elan]%s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

	rc = elan_ktf2k_ts_poll(client);
	if (rc < 0)
		return -EINVAL;
	else {
		if (i2c_master_recv(client, buf, size) != size ||
		    buf[0] != CMD_S_PKT)
			return -EINVAL;
	}
#ifdef RE_CALIBRATION	
	mdelay(200);
	rc = i2c_master_recv(client, buf_recv, 8);
	DBG("[elan] %s: Re-Calibration Packet %2x:%2x:%2x:%2x\n", __func__, buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3]);
	if (buf_recv[0] != 0x66) {
		mdelay(200);
		rc = i2c_master_recv(client, buf_recv, 8);
		DBG("[elan] %s: Re-Calibration Packet, re-try again %2x:%2x:%2x:%2x\n", __func__, buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3]);
	}
#endif

	return 0;
}

static int __hello_packet_handler(struct i2c_client *client)
{
	int rc;
	uint8_t buf_recv[8] = { 0 };

	rc = elan_ktf2k_ts_poll(client);
	if (rc < 0) {
		DBG( "[elan] %s: Int poll failed!\n", __func__);
		RECOVERY=0x80;
		return RECOVERY;
		//return -EINVAL;
	}

	rc = i2c_master_recv(client, buf_recv, 8);
	DBG("[elan] %s: hello packet %2x:%2X:%2x:%2x:%2x:%2x:%2x:%2x\n", __func__, buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3] , buf_recv[4], buf_recv[5], buf_recv[6], buf_recv[7]);

	if(buf_recv[0]==0x55 && buf_recv[1]==0x55 && buf_recv[2]==0x80 && buf_recv[3]==0x80)
	{
             RECOVERY=0x80;
	     return RECOVERY; 
	}
	return 0;
}

static int __fw_packet_handler(struct i2c_client *client)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	int rc;
	int major, minor;
	uint8_t cmd[] = {CMD_R_PKT, 0x00, 0x00, 0x01};/* Get Firmware Version*/
	uint8_t cmd_x[] = {0x53, 0x60, 0x00, 0x00}; /*Get x resolution*/
	uint8_t cmd_y[] = {0x53, 0x63, 0x00, 0x00}; /*Get y resolution*/
	uint8_t cmd_id[] = {0x53, 0xf0, 0x00, 0x01}; /*Get firmware ID*/
    uint8_t cmd_bc[] = {CMD_R_PKT, 0x10, 0x00, 0x01};/* Get BootCode Version*/
	uint8_t buf_recv[4] = {0};
// Firmware version
	rc = elan_ktf2k_ts_get_data(client, cmd, buf_recv, 4);
	if (rc < 0)
		return rc;
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	ts->fw_ver = major << 8 | minor;
	FW_VERSION = ts->fw_ver;
// Firmware ID
	rc = elan_ktf2k_ts_get_data(client, cmd_id, buf_recv, 4);
	if (rc < 0)
		return rc;
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	ts->fw_id = major << 8 | minor;
	FW_ID = ts->fw_id;
// Bootcode version
        rc = elan_ktf2k_ts_get_data(client, cmd_bc, buf_recv, 4);
        if (rc < 0)
                return rc;
        major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
        minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
        ts->bc_ver = major << 8 | minor;

// X Resolution
	rc = elan_ktf2k_ts_get_data(client, cmd_x, buf_recv, 4);
	if (rc < 0)
		return rc;
	minor = ((buf_recv[2])) | ((buf_recv[3] & 0xf0) << 4);
	ts->x_resolution =minor;
//#ifndef ELAN_TEN_FINGERS
	X_RESOLUTION = ts->x_resolution;
	DBG(KERN_INFO "[elan] %s: X resolution: %d\n", __func__, X_RESOLUTION);//Kenny
	X_RESOLUTION = 2496;//Kenny
//#endif
	
// Y Resolution	
	rc = elan_ktf2k_ts_get_data(client, cmd_y, buf_recv, 4);
	if (rc < 0)
		return rc;
	minor = ((buf_recv[2])) | ((buf_recv[3] & 0xf0) << 4);
	ts->y_resolution =minor;
//#ifndef ELAN_TEN_FINGERS
	Y_RESOLUTION = ts->y_resolution;
	DBG(KERN_INFO "[elan] %s: Y resolution: %d\n", __func__, Y_RESOLUTION);//Kenny
	Y_RESOLUTION = 1472;//Kenny
//#endif
	
	DBG(KERN_INFO "[elan] %s: Firmware version: 0x%4.4x\n",
			__func__, ts->fw_ver);
	DBG(KERN_INFO "[elan] %s: Firmware ID: 0x%4.4x\n",
			__func__, ts->fw_id);
	DBG(KERN_INFO "[elan] %s: Bootcode Version: 0x%4.4x\n",
			__func__, ts->bc_ver);
	DBG(KERN_INFO "[elan] %s: x resolution: %d, y resolution: %d\n",
			__func__, X_RESOLUTION, Y_RESOLUTION);
	
	return 0;
}

static inline int elan_ktf2k_ts_parse_xy(uint8_t *data,
			uint16_t *x, uint16_t *y)
{
	*x = *y = 0;

	*x = (data[0] & 0xf0);
	*x <<= 4;
	*x |= data[1];

	*y = (data[0] & 0x0f);
	*y <<= 8;
	*y |= data[2];

	return 0;
}

static int elan_ktf2k_ts_setup(struct i2c_client *client)
{
	int rc;
	
	rc = __hello_packet_handler(client);
	DBG("[elan] hellopacket's rc = %d\n",rc);

	mdelay(10);
	if (rc != 0x80){
	    rc = __fw_packet_handler(client);
	    if (rc < 0)
		    DBG("[elan] %s, fw_packet_handler fail, rc = %d", __func__, rc);
	    dev_dbg(&client->dev, "[elan] %s: firmware checking done.\n", __func__);
//Check for FW_VERSION, if 0x0000 means FW update fail!
	    if ( FW_VERSION == 0x00)
	    {
		rc = 0x80;
		DBG("[elan] FW_VERSION = %d, last FW update fail\n", FW_VERSION);
	    }
      }
	return rc;
}

static int elan_ktf2k_ts_rough_calibrate(struct i2c_client *client){
      uint8_t cmd[] = {CMD_W_PKT, 0x29, 0x00, 0x01};

	//dev_info(&client->dev, "[elan] %s: enter\n", __func__);
	DBG("[elan] %s: enter\n", __func__);
	dev_info(&client->dev,
		"[elan] dump cmd: %02x, %02x, %02x, %02x\n",
		cmd[0], cmd[1], cmd[2], cmd[3]);

	if ((i2c_master_send(client, cmd, sizeof(cmd))) != sizeof(cmd)) {
		dev_err(&client->dev,
			"[elan] %s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static int elan_ktf2k_ts_set_power_state(struct i2c_client *client, int state)
{
	uint8_t cmd[] = {CMD_W_PKT, 0x50, 0x00, 0x01};

	dev_dbg(&client->dev, "[elan] %s: enter\n", __func__);

	cmd[1] |= (state << 3);

	dev_dbg(&client->dev,
		"[elan] dump cmd: %02x, %02x, %02x, %02x\n",
		cmd[0], cmd[1], cmd[2], cmd[3]);

	if ((i2c_master_send(client, cmd, sizeof(cmd))) != sizeof(cmd)) {
		dev_err(&client->dev,
			"[elan] %s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static int elan_ktf2k_ts_get_power_state(struct i2c_client *client)
{
	int rc = 0;
	uint8_t cmd[] = {CMD_R_PKT, 0x50, 0x00, 0x01};
	uint8_t buf[4], power_state;

	rc = elan_ktf2k_ts_get_data(client, cmd, buf, 4);
	if (rc)
		return rc;

	power_state = buf[1];
	dev_dbg(&client->dev, "[elan] dump repsponse: %0x\n", power_state);
	power_state = (power_state & PWR_STATE_MASK) >> 3;
	dev_dbg(&client->dev, "[elan] power state = %s\n",
		power_state == PWR_STATE_DEEP_SLEEP ?
		"Deep Sleep" : "Normal/Idle");

	return power_state;
}

#ifdef ELAN_POWER_SOURCE
static unsigned now_usb_cable_status=0;

#if 0
static int elan_ktf2k_ts_hw_reset(struct i2c_client *client)
{
      struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
      touch_debug(DEBUG_INFO, "[ELAN] Start HW reset!\n");
      gpio_direction_output(ts->rst_gpio, 0);
        usleep_range(1000,1500);
        gpio_direction_output(ts->rst_gpio, 1);
      msleep(5);
        return 0;
}
static int elan_ktf2k_ts_set_power_source(struct i2c_client *client, u8 state)
{
        uint8_t cmd[] = {CMD_W_PKT, 0x40, 0x00, 0x01};
        int length = 0;

        dev_dbg(&client->dev, "[elan] %s: enter\n", __func__);
    /*0x52 0x40 0x00 0x01  =>    Battery Mode
       0x52 0x41 0x00 0x01  =>   AC Adapter Mode
       0x52 0x42 0x00 0x01 =>    USB Mode */
        cmd[1] |= state & 0x0F;

        dev_dbg(&client->dev,
                "[elan] dump cmd: %02x, %02x, %02x, %02x\n",
                cmd[0], cmd[1], cmd[2], cmd[3]);
        
      down(&pSem);
      length = i2c_master_send(client, cmd, sizeof(cmd));
      up(&pSem);
        if (length != sizeof(cmd)) {
                dev_err(&client->dev,
                        "[elan] %s: i2c_master_send failed\n", __func__);
                return -EINVAL;
        }

        return 0;
}



static void update_power_source(){
      unsigned power_source = now_usb_cable_status;
        if(private_ts == NULL || work_lock) return;

        if(private_ts->abs_x_max == ELAN_X_MAX) //TF 700T device
            return; // do nothing for TF700T;
            
      touch_debug(DEBUG_INFO, "Update power source to %d\n", power_source);
      switch(power_source){
        case USB_NO_Cable:
            elan_ktf2k_ts_set_power_source(private_ts->client, 0);
            break;
        case USB_Cable:
          elan_ktf2k_ts_set_power_source(private_ts->client, 1);
            break;
        case USB_AC_Adapter:
          elan_ktf2k_ts_set_power_source(private_ts->client, 2);
      }
}
#endif

void touch_callback(unsigned cable_status){ 
      now_usb_cable_status = cable_status;
      //update_power_source();
}
#endif

static int elan_ktf2k_ts_recv_data(struct i2c_client *client, uint8_t *buf, int bytes_to_recv)
{

	int rc;
	if (buf == NULL)
		return -EINVAL;

	memset(buf, 0, bytes_to_recv);

/* The ELAN_PROTOCOL support normanl packet format */	
#ifdef ELAN_PROTOCOL		
	rc = i2c_master_recv(client, buf, bytes_to_recv);
DBG("[elan] Elan protocol rc = %d \n", rc);
	if (rc != bytes_to_recv) {
		dev_err(&client->dev, "[elan] %s: i2c_master_recv error?! \n", __func__);
		return -1;
	}

#else 
	rc = i2c_master_recv(client, buf, 8);
	if (rc != 8)
	{
		 DBG("[elan] Read the first package error.\n");
		 mdelay(30);
		 return -1;
  }
  DBG("[elan_debug] %x %x %x %x %x %x %x %x\n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
	mdelay(1);
	
  if (buf[0] == 0x6D){    //for five finger
	  rc = i2c_master_recv(client, buf+ 8, 8);	
	  if (rc != 8)
	  {
		      DBG("[elan] Read the second package error.\n");
		      mdelay(30);
		      return -1;
		}		      
    DBG("[elan_debug] %x %x %x %x %x %x %x %x\n", buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14], buf[15]);
	  rc = i2c_master_recv(client, buf+ 16, 2);
    if (rc != 2)
    {
		      DBG("[elan] Read the third package error.\n");
		      mdelay(30);
		      return -1;
		}		      
	  mdelay(1);
    DBG("[elan_debug] %x %x \n", buf[16], buf[17]);
  }
#endif
//DBG("[elan_debug] end ts_work\n");
	return rc;
}

static void elan_ktf2k_ts_report_data(struct i2c_client *client, uint8_t *buf)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	struct input_dev *idev = ts->input_dev;
	uint16_t x, y;
	uint16_t fbits=0;
	uint8_t i, num, reported = 0;
	uint8_t idx, btn_idx;
	int finger_num;
	
/* for 10 fingers	*/
	if (buf[0] == TEN_FINGERS_PKT){
	    	finger_num = 10;
	    	num = buf[2] & 0x0f; 
	    	fbits = buf[2] & 0x30;	
		fbits = (fbits << 4) | buf[1]; 
	    	idx=3;
		btn_idx=33;
      }
/* for 5 fingers	*/
   		else if ((buf[0] == MTK_FINGERS_PKT) || (buf[0] == FIVE_FINGERS_PKT)){
	    	finger_num = 5;
	    	num = buf[1] & 0x07; 
        fbits = buf[1] >>3;
	    	idx=2;
		btn_idx=17;
      }else{
/* for 2 fingers */    
      	finger_num = 2;
	    	num = buf[7] & 0x03;		// for elan old 5A protocol the finger ID is 0x06
	    	fbits = buf[7] & 0x03;
//        fbits = (buf[7] & 0x03) >> 1;	// for elan old 5A protocol the finger ID is 0x06
	    	idx=1;
		btn_idx=7;
			}
		
    switch (buf[0]) {
    	case MTK_FINGERS_PKT:
    	case TWO_FINGERS_PKT:
    	case FIVE_FINGERS_PKT:	
	case TEN_FINGERS_PKT:
//qichaomin modify form here
		input_report_key(idev, BTN_TOUCH, 1);
			if (num == 0) {
#ifdef ELAN_BUTTON
				if (buf[btn_idx] == 0x21) 
        {
						button_state = 0x21;
						input_report_key(idev, KEY_BACK, 1);
						input_report_key(idev, KEY_BACK, 0);
DBG("[elan_debug] button %x \n", buf[btn_idx]);
				} 
				else if (buf[btn_idx] == 0x41)
				{
						button_state = 0x41;
						input_report_key(idev, KEY_HOME, 1);
				} 
				else if (buf[btn_idx] == 0x81)
				{
						button_state = 0x81;
						input_report_key(idev, KEY_MENU, 1);
				} 
				else if (button_state == 0x21) 
				{
						button_state=0;
						input_report_key(idev, KEY_BACK, 0);
		    } 
				else if (button_state == 0x41) 
				{
						button_state=0;
						input_report_key(idev, KEY_HOME, 0);
				} 
				else if (button_state == 0x81) 
				{
						button_state=0;
						input_report_key(idev, KEY_MENU, 0);
				}
				else
				{
				dev_dbg(&client->dev, "no press\n");
				input_mt_sync(idev);

				}
				
#endif	
			} else {			
				dev_dbg(&client->dev, "[elan] %d fingers\n", num);                        
				input_report_key(idev, BTN_TOUCH, 1);
				for (i = 0; i < finger_num; i++) {	
			  	if ((fbits & 0x01)) {
				    	//elan_ktf2k_ts_parse_xy(&buf[idx], &x, &y);  
						elan_ktf2k_ts_parse_xy(&buf[idx], &y, &x);  
			     	DBG("[elan_debug] %s, x=%d, y=%d\n",__func__, x , y);
			    //x = X_RESOLUTION-x;	 
			    y = Y_RESOLUTION-y;			     
			    	x = x*LCM_X_RESOLUTION/X_RESOLUTION;		//Kenny
						y = y*LCM_Y_RESOLUTION/Y_RESOLUTION;		//Kenny
						DBG("[elan_debug] %s, Mapping x=%d, y=%d\n",__func__, x , y);
						if (!((x<=0) || (y<=0) || (x>=LCM_X_RESOLUTION) || (y>=LCM_Y_RESOLUTION))) {   //Kenny
    					input_report_abs(idev, ABS_MT_TRACKING_ID, i);
							input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 8);
							input_report_abs(idev, ABS_MT_PRESSURE, 80);
							input_report_abs(idev, ABS_MT_POSITION_X, x);
							input_report_abs(idev, ABS_MT_POSITION_Y, y);
                            //input_report_abs(idev, ABS_MT_WIDTH_MAJOR,8);
							input_mt_sync(idev);
							reported++;
			  		} // end if border
					} // end if finger status
			  	fbits = fbits >> 1;
			  	idx += 3;
				} // end for
			}
			if (reported)
				input_sync(idev);
			else {
				input_mt_sync(idev);
				input_sync(idev);
			}
			break;
	   	default:
				dev_err(&client->dev,
								"[elan] %s: unknown packet type: %0x\n", __func__, buf[0]);
				break;
		} // end switch

	return;
}

static void elan_ktf2k_ts_work_func(struct work_struct *work)
{
	int rc;
	struct elan_ktf2k_ts_data *ts =
	container_of(work, struct elan_ktf2k_ts_data, work);
	uint8_t buf[4+PACKET_SIZE] = { 0 };
#ifdef ELAN_BUFFER_MODE	/* get rid of extra warning */
	uint8_t buf1[PACKET_SIZE] = { 0 };
#endif
		if (gpio_get_value(ts->intr_gpio))
		{
			DBG("[elan] Detected the jitter on INT pin");
			enable_irq(ts->client->irq);
			return;
		}
	
		rc = elan_ktf2k_ts_recv_data(ts->client, buf,4+PACKET_SIZE);
 
		if (rc < 0)
		{
			DBG("[elan] Received the packet Error.\n");
			enable_irq(ts->client->irq);
			return;
		}

		DBG("[elan_debug] %2x,%2x,%2x,%2x,%2x,%2x,%2x,%2x ....., %2x\n",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7],buf[17]);

#ifndef ELAN_BUFFER_MODE
		elan_ktf2k_ts_report_data(ts->client, buf);
#else
		elan_ktf2k_ts_report_data(ts->client, buf+4);

 	// Second package
	if (((buf[0] == 0x63) || (buf[0] == 0x66)) && ((buf[1] == 2) || (buf[1] == 3))) {
		rc = elan_ktf2k_ts_recv_data(ts->client, buf1, PACKET_SIZE);
		if (rc < 0){
			enable_irq(ts->client->irq);
                                return;
		}
		elan_ktf2k_ts_report_data(ts->client, buf1);
	// Final package
		if (buf[1] == 3) {
			rc = elan_ktf2k_ts_recv_data(ts->client, buf1, PACKET_SIZE);
			if (rc < 0){
				enable_irq(ts->client->irq);
				return;
			}
			elan_ktf2k_ts_report_data(ts->client, buf1);
		}
	}
#endif

		enable_irq(ts->client->irq);

	return;
}

static irqreturn_t elan_ktf2k_ts_irq_handler(int irq, void *dev_id)
{
	struct elan_ktf2k_ts_data *ts = dev_id;
	struct i2c_client *client = ts->client;

	dev_dbg(&client->dev, "[elan] %s\n", __func__);
	disable_irq_nosync(ts->client->irq);
	queue_work(ts->elan_wq, &ts->work);

	return IRQ_HANDLED;
}

static int elan_ktf2k_ts_register_interrupt(struct i2c_client *client)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	int err = 0;

	err = request_irq(client->irq, elan_ktf2k_ts_irq_handler,
											IRQF_TRIGGER_LOW, client->name, ts);
	if (err)
		dev_err(&client->dev, "[elan] %s: request_irq %d failed\n",
				__func__, client->irq);

	return err;
}

static int elan_ktf2k_ts_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err = 0;
	int fw_err = 0;
	struct elan_ktf2k_i2c_platform_data *pdata;
	struct elan_ktf2k_ts_data *ts;
	int New_FW_ID;	
	int New_FW_VER;	
		
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		DBG(KERN_ERR "[elan] %s: i2c check functionality error\n", __func__);
		err = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(struct elan_ktf2k_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		DBG(KERN_ERR "[elan] %s: allocate elan_ktf2k_ts_data failed\n", __func__);
		err = -ENOMEM;
		goto err_alloc_data_failed;
	}

	ts->elan_wq = create_singlethread_workqueue("elan_wq");
	if (!ts->elan_wq) {
		DBG(KERN_ERR "[elan] %s: create workqueue failed\n", __func__);
		err = -ENOMEM;
		goto err_create_wq_failed;
	}

	INIT_WORK(&ts->work, elan_ktf2k_ts_work_func);
	ts->client = client;
	i2c_set_clientdata(client, ts);
// james: maybe remove	
	pdata = client->dev.platform_data;
	if (likely(pdata != NULL)) {
		ts->intr_gpio = pdata->intr_gpio;
		ts->reset_gpio = pdata->reset_gpio;
	}

	/* setup gpio intr_gpio and reset gpio */
	err = gpio_request(ts->reset_gpio, "ektf2k_rst_pin");
	if(err) {
		pr_err("request ektf2k reset gpio pin failed.\n");
	}
	gpio_pull_updown(ts->reset_gpio, GPIOPullUp);
	pr_info("ektf2k begin reset touch panel.\n");
	gpio_direction_output(ts->reset_gpio, GPIO_LOW);
	msleep(20);
	gpio_set_value(ts->reset_gpio, GPIO_HIGH);
	msleep(5);
	pr_info("ektf2k end reset touch panel.\n");

	err = gpio_request(ts->intr_gpio, "ektf2k_int_pin");
	if(err) {
		pr_err("request ektf2k interrupt pin failed.\n");
	}
	gpio_pull_updown(ts->intr_gpio, GPIOPullUp);
	gpio_direction_input(ts->intr_gpio);
	pr_info("setup ektf2k touch panel interrupt pin done.\n");
	/* setup gpio intr_gpio and reset pin done */

	fw_err = elan_ktf2k_ts_setup(client);
	if (fw_err < 0) {
		DBG(KERN_INFO "No Elan chip inside\n");
//		fw_err = -ENODEV;  
	}

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev, "[elan] Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	ts->input_dev->name = "elan-touchscreen";     

	set_bit(BTN_TOUCH, ts->input_dev->keybit);
#ifdef ELAN_BUTTON
	set_bit(KEY_BACK, ts->input_dev->keybit);
	set_bit(KEY_MENU, ts->input_dev->keybit);
	set_bit(KEY_HOME, ts->input_dev->keybit);
	set_bit(KEY_SEARCH, ts->input_dev->keybit);
#endif

	input_set_abs_params(ts->input_dev, ABS_X, 0,  LCM_X_RESOLUTION, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, 0,  LCM_Y_RESOLUTION, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_TOOL_WIDTH, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, LCM_X_RESOLUTION, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, LCM_Y_RESOLUTION, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, 5, 0, 0);	

	__set_bit(EV_ABS, ts->input_dev->evbit);
	__set_bit(EV_SYN, ts->input_dev->evbit);
	__set_bit(EV_KEY, ts->input_dev->evbit);
/*
	set_bit(ABS_MT_POSITION_X, ts->input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, ts->input_dev->absbit);
	set_bit(ABS_MT_TOUCH_MAJOR, ts->input_dev->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR, ts->input_dev->absbit);
*/
	__set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);

	err = input_register_device(ts->input_dev);
	if (err) {
		dev_err(&client->dev,
			"[elan]%s: unable to register %s input device\n",
			__func__, ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	elan_ktf2k_ts_register_interrupt(ts->client);

	if (gpio_get_value(ts->intr_gpio) == 0) {
		DBG(KERN_INFO "[elan]%s: handle missed interrupt\n", __func__);
		elan_ktf2k_ts_irq_handler(client->irq, ts);
	}

	private_ts = ts;

	elan_ktf2k_touch_sysfs_init();

	dev_info(&client->dev, "[elan] Start touchscreen %s in interrupt mode\n",
		ts->input_dev->name);

// Firmware Update
	ts->firmware.minor = MISC_DYNAMIC_MINOR;
	ts->firmware.name = "elan-iap";
	ts->firmware.fops = &elan_touch_fops;
	ts->firmware.mode = S_IFREG|S_IRWXUGO; 

	if (misc_register(&ts->firmware) < 0)
  		DBG("[ELAN]misc_register failed!!");
  	else
		DBG("[ELAN]misc_register finished!!");

	dev_set_drvdata(ts->firmware.this_device, ts);

// End Firmware Update	
#ifdef IAP_PORTION
	if(0)
	{
    DBG("[ELAN]misc_register finished!!");
		work_lock=1;
		disable_irq(ts->client->irq);
		cancel_work_sync(&ts->work);
	
		power_lock = 1;
/* FW ID & FW VER*/
#if 0  /* For ektf21xx and ektf20xx  */
    DBG("[ELAN]  [7bd0]=0x%02x,  [7bd1]=0x%02x, [7bd2]=0x%02x, [7bd3]=0x%02x\n",  file_fw_data[31696],file_fw_data[31697],file_fw_data[31698],file_fw_data[31699]);
		New_FW_ID = file_fw_data[31699]<<8  | file_fw_data[31698] ;	       
		New_FW_VER = file_fw_data[31697]<<8  | file_fw_data[31696] ;
#endif
		
#if 0   /* for ektf31xx 2 wire ice ex: 2wireice -b xx.bin */
    DBG(" [7c16]=0x%02x,  [7c17]=0x%02x, [7c18]=0x%02x, [7c19]=0x%02x\n",  file_fw_data[31766],file_fw_data[31767],file_fw_data[31768],file_fw_data[31769]);
		New_FW_ID = file_fw_data[31769]<<8  | file_fw_data[31768] ;	       
		New_FW_VER = file_fw_data[31767]<<8  | file_fw_data[31766] ;
#endif	
    /* for ektf31xx iap ekt file   */	
    DBG(" [7bd8]=0x%02x,  [7bd9]=0x%02x, [7bda]=0x%02x, [7bdb]=0x%02x\n",  file_fw_data[31704],file_fw_data[31705],file_fw_data[31706],file_fw_data[31707]);
		New_FW_ID = file_fw_data[31707]<<8  | file_fw_data[31708] ;	       
		New_FW_VER = file_fw_data[31705]<<8  | file_fw_data[31704] ;
	  DBG(" FW_ID=0x%x,   New_FW_ID=0x%x \n",  FW_ID, New_FW_ID);   	       
		DBG(" FW_VERSION=0x%x,   New_FW_VER=0x%x \n",  FW_VERSION  , New_FW_VER);  

/* for firmware auto-upgrade            
	  if (New_FW_ID   ==  FW_ID){		      
		   	if (New_FW_VER > (FW_VERSION)) 
		                Update_FW_One(client, RECOVERY);
		} else {                        
		                DBG("FW_ID is different!");		
		}
*/         	
		if (FW_ID == 0)  RECOVERY=0x80;
			Update_FW_One(client, RECOVERY);
		power_lock = 0;

		work_lock=0;
		enable_irq(ts->client->irq);

	}
#endif	
	return 0;

err_input_register_device_failed:
	if (ts->input_dev)
		input_free_device(ts->input_dev);

err_input_dev_alloc_failed: 
	if (ts->elan_wq)
		destroy_workqueue(ts->elan_wq);

err_create_wq_failed:
	kfree(ts);

err_alloc_data_failed:
err_check_functionality_failed:

	return err;
}

static int elan_ktf2k_ts_remove(struct i2c_client *client)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);

	elan_touch_sysfs_deinit();

	unregister_early_suspend(&ts->early_suspend);
	free_irq(client->irq, ts);

	if (ts->elan_wq)
		destroy_workqueue(ts->elan_wq);
	input_unregister_device(ts->input_dev);
	kfree(ts);

	return 0;
}

static int elan_ktf2k_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	int rc = 0;
	if(power_lock==0) /* The power_lock can be removed when firmware upgrade procedure will not be enter into suspend mode.  */
	{
		DBG(KERN_INFO "[elan] %s: enter\n", __func__);

		disable_irq(client->irq);

		rc = cancel_work_sync(&ts->work);
		if (rc)
			enable_irq(client->irq);

		rc = elan_ktf2k_ts_set_power_state(client, PWR_STATE_DEEP_SLEEP);
	}
	return 0;
}

static int elan_ktf2k_ts_resume(struct i2c_client *client)
{

	int rc = 0, retry = 3;
#ifdef RE_CALIBRATION
	uint8_t buf_recv[4] = { 0 };
#endif
	if(power_lock==0)   /* The power_lock can be removed when firmware upgrade procedure will not be enter into suspend mode.  */
	{
		DBG(KERN_INFO "[elan] %s: enter\n", __func__);

		do {
			rc = elan_ktf2k_ts_set_power_state(client, PWR_STATE_NORMAL);
			mdelay(200);
#ifdef RE_CALIBRATION
			rc = i2c_master_recv(client, buf_recv, 4);
			DBG("[elan] %s: Re-Calibration Packet %2x:%2x:%2x:%2x\n", __func__, buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3]);
			if (buf_recv[0] != 0x66) {
				mdelay(200);
				rc = i2c_master_recv(client, buf_recv, 4);
				DBG("[elan] %s: Re-Calibration Packet, re-try again %2x:%2x:%2x:%2x\n", __func__, buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3]);
			}
#endif
			rc = elan_ktf2k_ts_get_power_state(client);
			if (rc != PWR_STATE_NORMAL)
				DBG(KERN_ERR "[elan] %s: wake up tp failed! err = %d\n",
					__func__, rc);
			else
				break;
		} while (--retry);

		enable_irq(client->irq);
	}
	return 0;
}

static const struct i2c_device_id elan_ktf2k_ts_id[] = {
	{ ELAN_KTF2K_NAME, 0 },
	{ }
};

static struct i2c_driver ektf2k_ts_driver = {
	.probe		= elan_ktf2k_ts_probe,
	.remove		= elan_ktf2k_ts_remove,
	.suspend	= elan_ktf2k_ts_suspend,
	.resume		= elan_ktf2k_ts_resume,
	.id_table	= elan_ktf2k_ts_id,
	.driver		= {
		.name = ELAN_KTF2K_NAME,
	},
};

static int __devinit elan_ktf2k_ts_init(void)
{
	DBG(KERN_INFO "[elan] %s driver version 0x0005: Integrated 2, 5, and 10 fingers together and auto-mapping resolution\n", __func__);
	return i2c_add_driver(&ektf2k_ts_driver);
}

static void __exit elan_ktf2k_ts_exit(void)
{
	i2c_del_driver(&ektf2k_ts_driver);
	return;
}

module_init(elan_ktf2k_ts_init);
module_exit(elan_ktf2k_ts_exit);

MODULE_DESCRIPTION("ELAN KTF2K Touchscreen Driver");
MODULE_LICENSE("GPL");



