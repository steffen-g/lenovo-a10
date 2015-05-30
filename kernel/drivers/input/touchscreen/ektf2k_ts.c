#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/wakelock.h>

#include "ektf2k_ts.h"

#define FW_ID_OFILM 0x0448
#define FW_ID_MUTTO 0x0482

#define EKTF2K_NAME "elan-ktf2k"
#define PACKET_SIZE 30

#define POWER_STATE_SLEEP  0
#define POWER_STATE_NORMAL 1
#define POWER_STATE_MASK   BIT(3)

#define PAGERETRY 30

#define UPDATE_FIRMWARE_ADB
#define UPDATE_FIRMWARE_KERNEL

#define CMD_S_PKT 0x52
#define CMD_R_PKT 0x53
#define CMD_W_PKT 0x54

// For Firmware Update
#define ELAN_IOCTLID    0xD0
#define IOCTL_I2C_SLAVE _IOW(ELAN_IOCTLID,  1, int)
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

#define CUSTOMER_IOCTLID    0xA0
#define IOCTL_CIRCUIT_CHECK  _IOR(CUSTOMER_IOCTLID, 1, int)
#define IOCTL_GET_UPDATE_PROGREE    _IOR(CUSTOMER_IOCTLID,  2, int)

#define EKTF2K_DBG(fmt, ...)	\
	do {	\
		pr_info(pr_fmt(fmt), ##__VA_ARGS__);	\
	} while (0)

enum
{
	PageSize    = 132,
	//PageNum     = 377,
	ACK_Fail    = 0x00,
	ACK_OK      = 0xAA,
	ACK_REWRITE = 0x55,
};

int RECOVERY = 0x00;
int FW_ID = 0x00;
int FW_VERSION = 0x00;
int work_lock = 0x00;
int power_lock = 0x00;
int X_RESOLUTION = 2496;
int Y_RESOLUTION = 1472;
int update_progree = 0;
int circuit_ver = 0x01;

static char file_fw_mutto_data[] = {
#include "fw_mutto_data.i"
};

static char file_fw_ofilm_data[] = {
#include "fw_ofilm_data.i"
};

#ifdef UPDATE_FIRMWARE_KERNEL
static struct wake_lock fw_update_lock;
#endif
static void ektf2k_ts_early_suspend(struct early_suspend *es);
static void ektf2k_ts_late_resume(struct early_suspend *es);
static int ektf2k_ts_poll(struct i2c_client *client);
static int ektf2k_ts_resume(struct i2c_client *client);

static int ektf2k_ts_i2c_send(struct i2c_client *client, const char *buf, int count)
{
	return i2c_master_normal_send(client, buf, count, 400*1000);
}

static int ektf2k_ts_i2c_recv(struct i2c_client *client, char *buf, int count)
{
	return i2c_master_normal_recv(client, buf, count, 400*1000);
}

static int ektf2k_ts_rough_calibrate(struct i2c_client *client)
{
	uint8_t cmd[] = {CMD_W_PKT, 0x29, 0x00, 0x01};
	if ((ektf2k_ts_i2c_send(client, cmd, sizeof(cmd))) !=
		sizeof(cmd)){
		return -EINVAL;
	}
	return 0;
}

static int ektf2k_ts_get_data(struct i2c_client *client, uint8_t *cmd, uint8_t *buf, size_t size)
{

	if (buf == NULL)
		return -EINVAL;

	if ((ektf2k_ts_i2c_send(client, cmd, 4)) != 4 ) {
		return -EINVAL;
	}

	if (ektf2k_ts_i2c_recv(client, buf, size) != size ||
		buf[0] != CMD_S_PKT)
		return -EINVAL;
	
	return 0;
}

static int ektf2k_ts_get_power_state(struct i2c_client *client)
{
	int ret = 0;
	uint8_t cmd[] = {CMD_R_PKT, 0x50, 0x00, 0x01};
	uint8_t buf[4];
	uint8_t power_state;

	ret = ektf2k_ts_get_data(client, cmd, buf, 4);
	if (ret)
		return ret;

	power_state = buf[1];
	power_state = (power_state & POWER_STATE_MASK) >> 3;

	return power_state;
}

static int ektf2k_ts_set_power_state(struct i2c_client *client, int state)
{
	uint8_t cmd[] = {CMD_W_PKT, 0x50, 0x00, 0x01};
	cmd[1] |= (state << 3);
	if (ektf2k_ts_i2c_send(client, cmd, sizeof(cmd)) != sizeof(cmd)){
		return -EINVAL;
	}
	return 0;
}

static int ektf2k_ts_fw_get_information(struct i2c_client *client)
{
	int rc;
	int major, minor;
	int bootcode_version;
	uint8_t cmd[] = {CMD_R_PKT, 0x00, 0x00, 0x01};/* Get Firmware Version*/
	//uint8_t cmd_x[] = {0x53, 0x60, 0x00, 0x00}; /*Get x resolution*/
	//uint8_t cmd_y[] = {0x53, 0x63, 0x00, 0x00}; /*Get y resolution*/
	uint8_t cmd_id[] = {0x53, 0xf0, 0x00, 0x01}; /*Get firmware ID*/
	uint8_t cmd_bc[] = {CMD_R_PKT, 0x10, 0x00, 0x01};/* Get BootCode Version*/
	uint8_t buf_recv[4] = {0};
// Firmware version
	rc = ektf2k_ts_get_data(client, cmd, buf_recv, 4);
	if (rc < 0)
		return rc;
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	FW_VERSION = major << 8 | minor;
    
// Firmware ID
	rc = ektf2k_ts_get_data(client, cmd_id, buf_recv, 4);
	if (rc < 0)
		return rc;
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	FW_ID = major << 8 | minor;

// Bootcode version
	rc = ektf2k_ts_get_data(client, cmd_bc, buf_recv, 4);
	if (rc < 0)
		return rc;
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	bootcode_version = major << 8 | minor;

// X Resolution
    X_RESOLUTION = 2496;

// Y Resolution
    Y_RESOLUTION = 1472;

	return 0;

}

static int ektf2k_ts_poll(struct i2c_client *client)
{
	struct ektf2k_ts_data *ts = i2c_get_clientdata(client);
	int status = 0;
	int retry = 3;
	
	do {
		status = gpio_get_value(ts->irq_pin);
		retry--;
		msleep(50);
	} while (status == 1 && retry > 0);
	
	return (status == 0 ? 0 : -ETIMEDOUT);
}

static int ektf2k_ts_check_recovery(struct i2c_client *client)
{
	int rc;
	uint8_t buf_recv[8] = { 0 };
	rc = ektf2k_ts_poll(client);
	if (rc < 0) {
		RECOVERY = 0x80;
		return RECOVERY;
	}

	rc = ektf2k_ts_i2c_recv(client, buf_recv, 8);
	printk("hello packet %2x:%2x:%2x:%2x:%2x:%2x:%2x:%2x\n", buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3], buf_recv[4], buf_recv[5], buf_recv[6], buf_recv[7]);
	/* bug bug need check here*/
	if(buf_recv[0] != 0x55 && buf_recv[1] != 0x55 && 
		buf_recv[2] != 0x55 && buf_recv[3] != 0x55)
	{
		RECOVERY = 0x80;
	} else {
		RECOVERY = 0x00;
	}
	return RECOVERY;
}

static int ektf2k_ts_re_calibration(struct i2c_client *client)
{
	int rc;
	uint8_t buf_recv[8] = { 0 };
	msleep(200);
	rc = ektf2k_ts_i2c_recv(client, buf_recv, 8);
	if (buf_recv[0] != 0x66) {
		msleep(200);
		rc = ektf2k_ts_i2c_recv(client, buf_recv, 8);
	}
	return 0;
}

int ektf2k_ts_EnterISPMode(struct i2c_client *client, uint8_t *isp_cmd)
{
	char buff[4] = {0};
	int len = 0;

	len = ektf2k_ts_i2c_send(client, isp_cmd, sizeof(isp_cmd));
	if (len != sizeof(buff)){
		return -1;
	}else{
		return 0;
	}
}

int ektf2k_ts_WritePage(struct i2c_client *client, char *szPage, int byte)
{
	int len = 0;
	struct ektf2k_ts_data *ts = i2c_get_clientdata(client);

	len = ektf2k_ts_i2c_send(ts->client, szPage, byte);
	if (len != byte) {
		return -1;
	}
	return 0;
}

int ektf2k_ts_GetAckData(struct i2c_client *client)
{
	int len = 0;
	char buff[2] = {0};
	struct ektf2k_ts_data *ts = i2c_get_clientdata(client);
	
	len = ektf2k_ts_i2c_recv(ts->client, buff, sizeof(buff));
	if (len != sizeof(buff)) {
		return -1;
	}

	if (buff[0] == 0xaa)
		return ACK_OK;
	else if (buff[0] == 0x55 && buff[1] == 0x55)
		return ACK_REWRITE;
	else
		return ACK_Fail;

	return 0;
}

void ektf2k_ts_print_progress(int page, int ic_num, int j)
{
	int i, percent, page_tatol, percent_tatol;
	char str[256];
	str[0] = '\0';
	for (i = 0; i < ((page)/10); i++) {
		str[i] = '#';
		str[i + 1] = '\0';
	}

	page_tatol = page + 249*(ic_num - j);
	percent = ((100 * page) / (249));
	percent_tatol = ((100*page_tatol)/(249*ic_num));

	if ((page) == (249))
		percent = 100;

	if ((page_tatol) == (249*ic_num))
		percent_tatol = 100;

	printk("\rprogress %s| %d%%", str, percent);
	
	if (page == (249))
		printk("\n");

	return;
}

int ektf2k_ts_update_fw(struct i2c_client *client, int fw_id)
{
	int res = 0;
	int ic_num = 1;
	int iPage = 0;
	int rewriteCnt = 0;
	int i = 0;
	uint8_t data = 0x10;
	int restartCnt = 0;
	char *szBuff = NULL;
	int PageNum = 0;
	uint8_t isp_cmd[] = {0x45, 0x49, 0x41, 0x50};
	int retry = 3;

	struct ektf2k_ts_data *ts = i2c_get_clientdata(client);

IAP_RESTART:
	
	gpio_set_value(ts->reset_pin, GPIO_LOW);
	msleep(20);
	gpio_set_value(ts->reset_pin, GPIO_HIGH);
	msleep(20); //15-25

	res = ektf2k_ts_EnterISPMode(ts->client, isp_cmd);
	msleep(40); //10-50
	
	res = ektf2k_ts_i2c_send(ts->client, &data, sizeof(data));
	if ( res != sizeof(data) ){
		;//error do something
	}
	msleep(10);

	if (fw_id == FW_ID_MUTTO) {
		PageNum = sizeof(file_fw_mutto_data) / 132;
	} else if (fw_id == FW_ID_OFILM) {
		PageNum = sizeof(file_fw_ofilm_data) / 132;
	} else {
		return -1;
	}

	printk("ektf2k update firmware start\n");

	for ( iPage = 1; iPage <= PageNum; iPage++ )
	{
		printk("ektf2k update firmware iPage = %d\n", iPage);
PAGE_REWRITE:
		if (fw_id == FW_ID_MUTTO) {
			szBuff = file_fw_mutto_data + (iPage - 1) * 132;
		} else if (fw_id == FW_ID_OFILM) {
			szBuff = file_fw_ofilm_data + (iPage - 1) * 132;
		}
		res = ektf2k_ts_WritePage(ts->client, szBuff, 132);
		
		if( iPage == 249 || iPage == 1 ){
			msleep(300);
		}else{
			msleep(50);
		}
		res = ektf2k_ts_GetAckData(ts->client);
		
		if (ACK_OK != res) {
			//msleep(50);
			if ( res == ACK_REWRITE ) {
				rewriteCnt = rewriteCnt + 1;
				if (rewriteCnt == PAGERETRY){
					return -1;
				} else {
					goto PAGE_REWRITE;
				}
			} else {
				restartCnt = restartCnt + 1;
				if (restartCnt >= 5){
					return -1;
				} else {
					goto IAP_RESTART;
				}
			}
		} else {
			rewriteCnt = 0;
			ektf2k_ts_print_progress(iPage, ic_num, i);
		}

		msleep(10);
	}

	gpio_set_value(ts->reset_pin, GPIO_LOW);
	msleep(20);
	gpio_set_value(ts->reset_pin, GPIO_HIGH);
	msleep(5);

	msleep(150);
	do {
		res = ektf2k_ts_check_recovery(ts->client);
	} while(res == 0x80 && retry--);

	/*Re Calibration start*/
	ektf2k_ts_re_calibration(ts->client);
	

	return 0;
}

int ektf2k_ts_iap_open(struct inode *inode, struct file *filp)
{
	return 0;
}

int ektf2k_ts_iap_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static ssize_t ektf2k_ts_iap_write(struct file *filp, const char *buff, size_t count, loff_t *offp)
{
	struct ektf2k_ts_data *ts = container_of(filp->private_data, struct ektf2k_ts_data, firmware);
	int ret;
	char *tmp;
	
	if (count > 8192)
		count = 8192;

	tmp = kmalloc(count, GFP_KERNEL);

	if (tmp == NULL)
		return -ENOMEM;

	if (copy_from_user(tmp, buff, count))
		return -EFAULT;

	ret = ektf2k_ts_i2c_send(ts->client, tmp, count);

	kfree(tmp);

	return (ret == 1) ? count : ret;
}

static ssize_t ektf2k_ts_iap_read(struct file *filp, char *buff, size_t count, loff_t *offp)
{
	struct ektf2k_ts_data *ts = container_of(filp->private_data, struct ektf2k_ts_data, firmware);
	int ret;
	char *tmp;
	long rc;
	
	if (count > 8192)
		count = 8192;

	tmp = kmalloc(count, GFP_KERNEL);

	if (tmp == NULL)
		return -ENOMEM;

	ret = ektf2k_ts_i2c_recv(ts->client, tmp, count);

	if (ret >= 0)
		rc = copy_to_user(buff, tmp, count);

	kfree(tmp);

	return (ret == 1) ? count : ret;
}

static long ektf2k_ts_iap_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct ektf2k_ts_data *ts = container_of(filp->private_data, struct ektf2k_ts_data, firmware);
	int __user *ip = (int __user*)arg;

    switch (cmd) {
        case IOCTL_I2C_SLAVE:
            ts->client->addr = (int __user)arg;
            //file_fops_addr = 0x15;
            break;
        case IOCTL_MAJOR_FW_VER:
            break;
        case IOCTL_MINOR_FW_VER:
            break;
        case IOCTL_RESET:
			gpio_set_value(ts->reset_pin, GPIO_LOW);
			msleep(20);
			gpio_set_value(ts->reset_pin, GPIO_HIGH);
            msleep(5);
            break;
        case IOCTL_IAP_MODE_LOCK:
            if(work_lock==0)
            {
                work_lock=1;
                disable_irq(ts->irq);
                cancel_work_sync(&ts->work);
            }
            break;
        case IOCTL_IAP_MODE_UNLOCK:
            if(work_lock==1)
            {
                work_lock=0;
                enable_irq(ts->irq);
            }
            break;
        case IOCTL_CHECK_RECOVERY_MODE:
			ektf2k_ts_check_recovery(ts->client);
            return RECOVERY;
            break;
        case IOCTL_FW_VER:
            ektf2k_ts_fw_get_information(ts->client);
            return FW_VERSION;
            break;
        case IOCTL_X_RESOLUTION:
            ektf2k_ts_fw_get_information(ts->client);
            return X_RESOLUTION;
            break;
        case IOCTL_Y_RESOLUTION:
            ektf2k_ts_fw_get_information(ts->client);
            return Y_RESOLUTION;
            break;
        case IOCTL_FW_ID:
            ektf2k_ts_fw_get_information(ts->client);
            return FW_ID;
            break;
        case IOCTL_ROUGH_CALIBRATE:
            return ektf2k_ts_rough_calibrate(ts->client);
        case IOCTL_I2C_INT:
            put_user(gpio_get_value(ts->irq_pin), ip);
            break;
        case IOCTL_RESUME:
			ektf2k_ts_resume(ts->client);
            break;
        case IOCTL_POWER_LOCK:
            power_lock=1;
            break;
        case IOCTL_POWER_UNLOCK:
            power_lock=0;
            break;
        case IOCTL_GET_UPDATE_PROGREE:
            update_progree=(int __user)arg;
            break;
        case IOCTL_FW_UPDATE:
            ektf2k_ts_update_fw(ts->client, 0);
            break;
        case IOCTL_CIRCUIT_CHECK:
            return circuit_ver;
            break;
        default:
            break;
    }
    return 0;
}

struct file_operations ektf2k_ts_firmware_fops = {
	.open           = ektf2k_ts_iap_open,
	.write          = ektf2k_ts_iap_write,
	.read           = ektf2k_ts_iap_read,
	.release        = ektf2k_ts_iap_release,
	.unlocked_ioctl = ektf2k_ts_iap_ioctl,
};

static inline int ektf2k_ts_parse_xy(uint8_t *data,
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

static uint16_t last_status[10];

static void ektf2k_ts_report_data(struct i2c_client *client, uint8_t *buf)
{
	struct ektf2k_ts_data *ts = i2c_get_clientdata(client);
	struct input_dev *idev = ts->input_dev;
	uint16_t x, y;
	uint16_t fbits = 0;
	uint8_t i, num, reported = 0;
	/*uint8_t idx, btn_idx;*/
	uint8_t idx;
	int finger_num;

	if (buf[0] == EKTF2K_TEN_FINGER) {
		finger_num = 10;
		num = buf[2] & 0x0f;
		fbits = buf[2] & 0x30;
		fbits = (fbits << 4) | buf[1];
		idx = 3;
		/*btn_idx = 33;*/
		input_report_key(idev, BTN_TOUCH, 1);
		for (i = 0; i < finger_num; i++) {
			if((fbits & 0x01)){
				ektf2k_ts_parse_xy(&buf[idx], &y, &x);
				y = TOUCH_MAX_WIDTH - y;
				x = x * SCREEN_MAX_HEIGHT / TOUCH_MAX_HEIGHT;
				y = y * SCREEN_MAX_WIDTH / TOUCH_MAX_WIDTH;
				if (!( (x < 0) || (y < 0) || (x > SCREEN_MAX_HEIGHT) || ( y > SCREEN_MAX_WIDTH))){
					input_report_abs(idev, ABS_MT_TRACKING_ID, i);
					input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 8);
					input_report_abs(idev, ABS_MT_WIDTH_MAJOR, 8);
					input_report_abs(idev, ABS_MT_POSITION_X, x);
					input_report_abs(idev, ABS_MT_POSITION_Y, y);
					last_status[i] = 1;
					input_mt_sync(idev);
					reported++;
				}
			} else {
				if(last_status[i] != 0){
					input_report_abs(idev, ABS_MT_TRACKING_ID, -1);
					input_mt_sync(idev);
				}
				last_status[i] = 0;
			}
			fbits = fbits >> 1;
			idx += 3;
		}

		input_sync(idev);

	} else {
		return;
	}

}

static int ektf2k_ts_recv_data(struct i2c_client *client, uint8_t *buf, int bytes_to_recv)
{
	int ret = 0;
	if (buf == NULL)
		return -EINVAL;

	memset(buf, 0, bytes_to_recv);
	
	ret = ektf2k_ts_i2c_recv(client, buf, bytes_to_recv);
	if (ret != bytes_to_recv) {
		return -1;
	}
	
	return ret;
}

static void ektf2k_ts_work_func(struct work_struct *fwork)
{
	int ret = 0;
	struct ektf2k_ts_data *ts = container_of(fwork, struct ektf2k_ts_data, work);
	uint8_t buf[4+PACKET_SIZE] = { 0 };
	if (gpio_get_value(ts->irq_pin)){
		enable_irq(ts->irq);
		EKTF2K_DBG("Detected the jitter on INT pin.\n");
		return;
	}

	ret = ektf2k_ts_recv_data(ts->client, buf, 4+PACKET_SIZE);
	if (ret < 0) {
		enable_irq(ts->irq);
		EKTF2K_DBG("Received the packet Error.\n");
		return;
	}

	ektf2k_ts_report_data(ts->client, buf);
	enable_irq(ts->irq);
	return;
}


static irqreturn_t ektf2k_ts_irq_handler(int irq, void *ts_data)
{
	struct ektf2k_ts_data *ts = ts_data;
	disable_irq_nosync(ts->irq);
	queue_work(ts->ektf2k_wq, &ts->work);
	return IRQ_HANDLED;
}


static int ektf2k_ts_setup(struct ektf2k_ts_data *ts)
{
    int ret = 0;
	int retry = 3;
    ret = gpio_request(ts->reset_pin, "TS_RESET");
    if (ret < 0){
        return ret;
    }
    gpio_pull_updown(ts->reset_pin, GPIOPullUp);
    gpio_direction_output(ts->reset_pin, GPIO_LOW);
    msleep(20);
    gpio_set_value(ts->reset_pin, GPIO_HIGH);
    msleep(5);

	msleep(150);
	do {
		ret = ektf2k_ts_check_recovery(ts->client);
	} while(ret == 0x80 && retry--);

	/*Re Calibration start*/
	ektf2k_ts_re_calibration(ts->client);

    return ret;
}

static int get_fw_mutto_version(void)
{
	return (file_fw_mutto_data[32101]<<8  | file_fw_mutto_data[32100]);
}

static int get_fw_ofilm_version(void)
{
	return (file_fw_ofilm_data[32101]<<8  | file_fw_ofilm_data[32100]);
}

#ifdef UPDATE_FIRMWARE_KERNEL
static int ektf2k_ts_update_proc(void *data)
{
	struct ektf2k_ts_data *ts = data;
	int retry = 5;
	do {
		ektf2k_ts_fw_get_information(ts->client);
		msleep(100);
	}while(FW_ID == 0x00 && retry-- > 1);
	printk("ektf2k FW_ID = %x, FW_VERSION = %x, get_fw_mutto_version = %x, get_fw_ofilm_version = %x\n", FW_ID, FW_VERSION, get_fw_mutto_version(), get_fw_ofilm_version());
	if (FW_ID == FW_ID_MUTTO) {
		if (get_fw_mutto_version() > FW_VERSION){
			ektf2k_ts_update_fw(ts->client, FW_ID_MUTTO);
		}
	} else if (FW_ID == FW_ID_OFILM) {
		if (get_fw_ofilm_version() > FW_VERSION){
			ektf2k_ts_update_fw(ts->client, FW_ID_OFILM);
		}
	} else {
		ektf2k_ts_update_fw(ts->client, FW_ID_OFILM);
	}
	power_lock = 0;
	work_lock = 0;
	enable_irq(ts->irq);
	wake_unlock(&fw_update_lock);
	return 0;
}

static int ektf2k_ts_update_thread(struct ektf2k_ts_data *ts)
{
	struct task_struct *thread = NULL;
	thread = kthread_run(ektf2k_ts_update_proc, (void *)ts, "ektf2k_update");
	if (IS_ERR(thread))
	{
		return -1;
	}
	return 0;
}
#endif

static int ektf2k_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct ektf2k_ts_data *ts;
    int ret = 0;


    struct ektf2k_i2c_platform_data *pdata;

	printk("ektf2k_ts probe start\n");
     
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)){
        ret = -ENODEV;
        goto err_check_functionality_failed;
    }

    ts = kzalloc(sizeof(*ts), GFP_KERNEL);
    if (ts == NULL){
        ret = -ENOMEM;
        goto err_alloc_data_failed;
    }

	ts->ektf2k_wq = create_singlethread_workqueue("ektf2k_wq");
	if (!(ts->ektf2k_wq)){
		ret = -ENOMEM;
        goto err_alloc_data_failed;
	}

    pdata = client->dev.platform_data;
    
    if (pdata && pdata->reset_pin) {
        ts->reset_pin = pdata->reset_pin;
        ts->irq_pin = pdata->irq_pin;
    } else {
        ts->reset_pin = RK30_PIN0_PB6;
        ts->irq_pin = RK30_PIN1_PB7;
    }

    msleep(16);
    
    INIT_WORK(&ts->work, ektf2k_ts_work_func);
    ts->client = client;
    i2c_set_clientdata(client, ts);
    ts->input_dev = input_allocate_device();
    if (ts->input_dev == NULL) {
        ret = -ENOMEM;
        goto err_input_dev_alloc_failed;
    }


    ts->input_dev->name = EKTF2K_NAME;
    __set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);
    __set_bit(EV_SYN, ts->input_dev->evbit);
    __set_bit(EV_ABS, ts->input_dev->evbit);
    input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 10, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 1, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, SCREEN_MAX_HEIGHT, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, SCREEN_MAX_WIDTH, 0, 0);

    ret = input_register_device(ts->input_dev);
    if (ret) {
        goto err_input_register_device_failed;
    }
   
//setup reset gpio start
    ret = ektf2k_ts_setup(ts);
    if (ret < 0){
        goto err_reset_gpio_request;
    }
//setup reset gpio end

	/*For Firmware Update*/
#ifdef UPDATE_FIRMWARE_ADB
	ts->firmware.minor = MISC_DYNAMIC_MINOR;
	ts->firmware.name = "elan-iap";
	ts->firmware.fops = &ektf2k_ts_firmware_fops;
	ts->firmware.mode = S_IFREG|S_IRWXUGO;

	if (misc_register(&ts->firmware) < 0){
		EKTF2K_DBG("[EKTF2k]misc_register failed.\n");
	}
	dev_set_drvdata(ts->firmware.this_device, ts);
#endif
	
//setup int irq gpio start
    ret = gpio_request(ts->irq_pin, "TS_IRQ");
    if (ret < 0){
        goto err_irq_gpio_request;
    }
    gpio_pull_updown(ts->irq_pin, GPIOPullUp);
    gpio_direction_input(ts->irq_pin);
//setup int irq gpio end

    ts->irq = gpio_to_irq(ts->irq_pin);
	if (ts->irq)
	{
		ret = request_irq(ts->irq, ektf2k_ts_irq_handler, IRQ_TYPE_LEVEL_LOW, "ektf2k_ts_irq", ts);
		if (ret != 0) {
			goto err_irq_request_irq;
		}
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	ts->early_suspend.suspend = ektf2k_ts_early_suspend;
	ts->early_suspend.resume = ektf2k_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

#ifdef UPDATE_FIRMWARE_KERNEL
    wake_lock_init(&fw_update_lock, WAKE_LOCK_SUSPEND, "fw_up_lock");
	wake_lock(&fw_update_lock);
	work_lock = 1;
	disable_irq(ts->irq);
	cancel_work_sync(&ts->work);
	power_lock = 1;
	ektf2k_ts_update_thread(ts);
#endif

	printk("ektf2k_ts probe end\n");

	return 0;

err_irq_request_irq:
	free_irq(ts->irq, ts);
	gpio_direction_input(ts->irq_pin);
	gpio_free(ts->irq_pin);
err_irq_gpio_request:
	gpio_free(ts->reset_pin);
err_reset_gpio_request:
	input_free_device(ts->input_dev);
err_input_register_device_failed:
err_input_dev_alloc_failed:
    i2c_set_clientdata(client, NULL);
	kfree(ts);
err_alloc_data_failed:
err_check_functionality_failed:
	
	return ret;
}

static int ektf2k_ts_remove(struct i2c_client *client)
{
	struct ektf2k_ts_data *ts = i2c_get_clientdata(client);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif
	if (ts){
		free_irq(ts->irq, ts);
		gpio_direction_input(ts->irq_pin);
		gpio_free(ts->irq_pin);
		gpio_free(ts->reset_pin);
		flush_workqueue(ts->ektf2k_wq);
		destroy_workqueue(ts->ektf2k_wq);
	}
	i2c_set_clientdata(ts->client, NULL);
	kfree(ts);
	return 0;
}

static int ektf2k_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret = 0;
	struct ektf2k_ts_data *ts = i2c_get_clientdata(client);
	if (power_lock == 0) {
		flush_workqueue(ts->ektf2k_wq);
		disable_irq(ts->irq);
		ektf2k_ts_set_power_state(client, POWER_STATE_SLEEP);
		if (ts->power) {
			ret = ts->power(ts, 0);
		}
	}
	return ret;
}

static void ektf2k_ts_resume_work_func(struct work_struct *work)
{
	struct ektf2k_ts_data *ts = container_of(work, struct ektf2k_ts_data, work);
	int retry = 2;
	int ret = 0;
	do {
		ret = ektf2k_ts_set_power_state(ts->client, POWER_STATE_NORMAL);

		ektf2k_ts_re_calibration(ts->client);

		ret = ektf2k_ts_get_power_state(ts->client);

		if (ret == POWER_STATE_NORMAL)
			break;
	}while (--retry);
	enable_irq(ts->irq);
	PREPARE_WORK(&ts->work, ektf2k_ts_work_func);
}

static int ektf2k_ts_resume(struct i2c_client *client)
{
	int ret = 0;
	struct ektf2k_ts_data *ts = i2c_get_clientdata(client);
	printk("ektf2k_ts resume start\n");
	if (power_lock == 0) {
		if (ts->power) {
			ret = ts->power(ts, 1);
		}
		if (!work_pending(&ts->work)) {
			PREPARE_WORK(&ts->work, ektf2k_ts_resume_work_func);
			queue_work(ts->ektf2k_wq, &ts->work);
		}
	}
	printk("ektf2k_ts resume end\n");
	return ret;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ektf2k_ts_early_suspend(struct early_suspend *es)
{
	struct ektf2k_ts_data *ts = container_of(es, struct ektf2k_ts_data, early_suspend);
	ektf2k_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void ektf2k_ts_late_resume(struct early_suspend *es)
{
	struct ektf2k_ts_data *ts = container_of(es, struct ektf2k_ts_data, early_suspend);
	ektf2k_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id ektf2k_ts_id[] = {
    {EKTF2K_NAME, 0},
    {}
};

static struct i2c_driver ektf2k_ts_driver = {
    .probe    = ektf2k_ts_probe,
    .remove   = ektf2k_ts_remove,
#ifndef CONFIG_HAS_EARLY_SUSPEND
    .suspend  = ektf2k_ts_suspend,
    .resume   = ektf2k_ts_resume,
#endif
    .id_table = ektf2k_ts_id,
    .driver = {
        .owner = THIS_MODULE,
        .name  = EKTF2K_NAME,
    },
};

static int __devinit ektf2k_ts_init(void)
{
    return i2c_add_driver(&ektf2k_ts_driver);
}

static void __exit ektf2k_ts_exit(void)
{
    i2c_del_driver(&ektf2k_ts_driver);
    return;
}
module_init(ektf2k_ts_init);
module_exit(ektf2k_ts_exit);

MODULE_DESCRIPTION("ELAN KTF2k Touchscreen Driver");
MODULE_LICENSE("GPL");
