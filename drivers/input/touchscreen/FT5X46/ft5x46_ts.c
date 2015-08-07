/*
 *
 * FocalTech focaltech TouchScreen driver.
 *
 * Copyright (c) 2010  Focal tech Ltd.
 * Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
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
 */
 
#include <linux/of_gpio.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/hrtimer.h>
#include <linux/proc_fs.h>
#include <linux/interrupt.h>

#include <asm/uaccess.h>
#include <mach/device_info.h>
#include <mach/oppo_boot_mode.h>
#include <mach/oppo_project.h>

#include <linux/regulator/consumer.h>
#include <linux/firmware.h>

#include <linux/rtc.h>
#include <linux/syscalls.h>

#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/mutex.h>
#include "ft5x46_ts.h"


#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
/* Early-suspend level */
#define FT_SUSPEND_LEVEL 1
#endif

#define IC_FT5X46	5
#define TP_CHIPER_ID			0x54	//FT5x46 ic
#define DEVICE_IC_TYPE	IC_FT5X46
#define FT5X46_UPGRADE_AA_DELAY 		10
#define FT5X46_UPGRADE_55_DELAY 		10
#define FT5X46_UPGRADE_ID_1			    0x54
#define FT5X46_UPGRADE_ID_2			    0x2c
#define FT5X46_UPGRADE_READID_DELAY 	10
#define FTS_UPGRADE_LOOP	30

#ifdef FTS_APK_DEBUG
	/*create apk debug channel*/
	#define PROC_UPGRADE			0
	#define PROC_READ_REGISTER		1
	#define PROC_WRITE_REGISTER		2
	#define PROC_AUTOCLB			4
	#define PROC_UPGRADE_INFO		5
	#define PROC_WRITE_DATA			6
	#define PROC_READ_DATA			7
	#define PROC_NAME	"ft5x0x-debug"
#endif

static bool irq_enable = true;
#ifdef SUPPORT_GESTURE
	#include "ft_gesture_lib.h"

	#define GESTURE_LEFT		0x20
	#define GESTURE_RIGHT		0x21
	#define GESTURE_UP		    0x22
	#define GESTURE_DOWN		0x23
	#define GESTURE_DOUBLECLICK	0x24
	#define GESTURE_DOUBLELINE	0x25

	#define GESTURE_LEFT_V		0x51
	#define GESTURE_RIGHT_V		0x52
	#define GESTURE_UP_V		0x53
	#define GESTURE_DOWN_V		0x54

	#define GESTURE_O    	    0x57
	#define GESTURE_O_Anti	    0x30
	#define GESTURE_W		    0x31
	#define GESTURE_M		    0x32

	#define FTS_GESTRUE_POINTS 255
	#define FTS_GESTRUE_POINTS_ONETIME  62
	#define FTS_GESTRUE_POINTS_HEADER 8
	#define FTS_GESTURE_OUTPUT_ADRESS 0xD3
	#define FTS_GESTURE_OUTPUT_UNIT_LENGTH 4


	#define UnkownGestrue       0
	#define DouTap              1   // double tap
	#define UpVee               2   // V
	#define DownVee             3   // ^
	#define LeftVee             4   // >
	#define RightVee            5   // <
	#define Circle              6   // O
	#define DouSwip             7   // ||
	#define Left2RightSwip      8   // -->
	#define Right2LeftSwip      9   // <--
	#define Up2DownSwip         10  // |v
	#define Down2UpSwip         11  // |^
	#define Mgestrue            12  // M
	#define Wgestrue            13  // W

	unsigned short coordinate_x[150] = {0};
	unsigned short coordinate_y[150] = {0};
	unsigned short coordinate_doblelion_1_x[150] = {0};
	unsigned short coordinate_doblelion_2_x[150] = {0};
	unsigned short coordinate_doblelion_1_y[150] = {0};
	unsigned short coordinate_doblelion_2_y[150] = {0};
	unsigned short coordinate_report[14] = {0};
	static uint32_t gesture;
	static bool gesture_enable;
	static bool gesture_mode;
	static bool i2c_device_test;
#endif

#ifdef SUPPORT_CHG_MODE
	#define  CHG_MODE_ADDR  0x8b
	static bool CHG_Mode = false;
#endif 

#ifdef SUPPORT_GLOVES_MODE
	#define GLOVES_MODE_ADDR   0xc0
	static bool gloves_enable = false;
#endif


#define FT_DRIVER_VERSION	0x02

#define FT_META_REGS		3
#define FT_ONE_TCH_LEN		6
#define FT_TCH_LEN(x)		(FT_META_REGS + FT_ONE_TCH_LEN * x)

#define FT_PRESS		0x7F
#define FT_MAX_ID		0x0F
#define FT_TOUCH_X_H_POS	3
#define FT_TOUCH_X_L_POS	4
#define FT_TOUCH_Y_H_POS	5
#define FT_TOUCH_Y_L_POS	6
#define FT_TD_STATUS		2
#define FT_TOUCH_EVENT_POS	3
#define FT_TOUCH_ID_POS		5
#define FT_TOUCH_DOWN		0
#define FT_TOUCH_CONTACT	2

/*register address*/
#define FT_REG_DEV_MODE		0x00
#define FT_DEV_MODE_REG_CAL	0x02
#define FT_REG_ID		0xA3
#define FT_REG_PMODE		0xA5
#define FT_REG_FW_VER		0xA6
#define FT_REG_FW_VENDOR_ID	0xA8
#define FT_REG_POINT_RATE	0x88
#define FT_REG_THGROUP		0x80
#define FT_REG_ECC		0xCC
#define FT_REG_RESET_FW		0x07
#define FT_REG_FW_MIN_VER	0xB2
#define FT_REG_FW_SUB_MIN_VER	0xB3

/* power register bits*/
#define FT_PMODE_ACTIVE		0x00
#define FT_PMODE_MONITOR	0x01
#define FT_PMODE_STANDBY	0x02
#define FT_PMODE_HIBERNATE	0x03
#define FT_FACTORYMODE_VALUE	0x40
#define FT_WORKMODE_VALUE	0x00
#define FT_RST_CMD_REG1		0xFC
#define FT_RST_CMD_REG2		0xBC
#define FT_READ_ID_REG		0x90
#define FT_ERASE_APP_REG	0x61
#define FT_ERASE_PANEL_REG	0x63
#define FT_FW_START_REG		0xBF

#define FT_STATUS_NUM_TP_MASK	0x0F

#define FT_VTG_MIN_UV		2600000
#define FT_VTG_MAX_UV		3300000
#define FT_I2C_VTG_MIN_UV	1800000
#define FT_I2C_VTG_MAX_UV	1800000

#define FT_COORDS_ARR_SIZE	4
#define MAX_BUTTONS		4

#define FT_8BIT_SHIFT		8
#define FT_4BIT_SHIFT		4
#define FT_FW_NAME_MAX_LEN	50

#define FT_UPGRADE_AA		0xAA
#define FT_UPGRADE_55		0x55

/**
* Application data verification will be run before upgrade flow.
* Firmware image stores some flags with negative and positive value
* in corresponding addresses, we need pick them out do some check to
* make sure the application data is valid.
*/

#define FT_MAX_TRIES		5
#define FT_RETRY_DLY		20

#define FT_MAX_WR_BUF		10
#define FT_MAX_RD_BUF		2
#define FT_FW_PKT_LEN		128
#define FT_FW_PKT_META_LEN	6
#define FT_FW_PKT_DLY_MS	20
#define FT_FW_LAST_PKT		0x6ffa
#define FT_EARSE_DLY_MS		100
#define FT_55_AA_DLY_NS		5000

#define FT_UPGRADE_LOOP		5
#define FT_CAL_START		0x04
#define FT_CAL_FIN		0x00
#define FT_CAL_STORE		0x05
#define FT_CAL_RETRY		100
#define FT_REG_CAL		0x00
#define FT_CAL_MASK		0x70

#define FT_INFO_MAX_LEN		512

#define FT_BLOADER_SIZE_OFF	12
#define FT_BLOADER_NEW_SIZE	30
#define FT_DATA_LEN_OFF_OLD_FW	8
#define FT_DATA_LEN_OFF_NEW_FW	14
#define FT_FINISHING_PKT_LEN_OLD_FW	6
#define FT_FINISHING_PKT_LEN_NEW_FW	12
#define FT_MAGIC_BLOADER_Z7	0x7bfa
#define FT_MAGIC_BLOADER_LZ4	0x6ffa
#define FT_MAGIC_BLOADER_GZF_30	0x7ff4
#define FT_MAGIC_BLOADER_GZF	0x7bf4

#define PAGESIZE 512

#define TPD_DEVICE "focaltech"

static int LCD_WIDTH ;
static int LCD_HEIGHT;
u32 button_map[MAX_BUTTONS];
static int report_key_point_y = 0;

static int tp_fw_ok = 0;
u8 current_fw_ver[3] = {0};
static int tp_probe_ok = 0;

enum {
	FT_BLOADER_VERSION_LZ4 = 0,
	FT_BLOADER_VERSION_Z7 = 1,
	FT_BLOADER_VERSION_GZF = 2,
};

enum {
	FT_FT5336_FAMILY_ID_0x11 = 0x11,  
	FT_FT5336_FAMILY_ID_0x12 = 0x12,
	FT_FT5336_FAMILY_ID_0x13 = 0x13,
	FT_FT5336_FAMILY_ID_0x14 = 0x14,
};

#define FT_STORE_TS_INFO(buf, id, name, max_tch, group_id, fw_vkey_support, \
			fw_name, fw_maj, fw_min, fw_sub_min) \
			snprintf(buf, FT_INFO_MAX_LEN, \
				"controller\t= focaltech\n" \
				"model\t\t= 0x%x\n" \
				"name\t\t= %s\n" \
				"max_touches\t= %d\n" \
				"drv_ver\t\t= 0x%x\n" \
				"group_id\t= 0x%x\n" \
				"fw_vkey_support\t= %s\n" \
				"fw_name\t\t= %s\n" \
				"fw_ver\t\t= %d.%d.%d\n", id, name, \
				max_tch, FT_DRIVER_VERSION, group_id, \
				fw_vkey_support, fw_name, fw_maj, fw_min, \
				fw_sub_min)

#define FT_DEBUG_DIR_NAME	"ts_debug"

struct focaltech_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct input_dev *kpd;
	const struct focaltech_ts_platform_data *pdata;	
	char fw_name[FT_FW_NAME_MAX_LEN];
	bool loading_fw;
	u8 family_id;
	struct dentry *dir;
	u16 addr;
	bool suspended;
	char *ts_info;
	u8 *tch_data;
	u32 tch_data_len;
	u8 fw_ver[3];
	u8 fw_vendor_id;
	u8 fw_manufacture;
	char fw_version[12]; 
	struct manufacture_info tp_info;
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif	
	
	struct work_struct speed_up_work;
	struct work_struct suspend_wk;

#if defined(FTS_SCAP_TEST)
	struct mutex selftest_lock;
#endif
};

static bool Is_IRQ_Wake = false;

struct focaltech_ts_data *data_g = NULL;

extern int is_tp_driver_loaded;

static DEFINE_MUTEX(i2c_rw_access);

static struct workqueue_struct *speedup_resume_wq = NULL;

static void speedup_focaltech_resume(struct work_struct *work);
static void speedup_focaltech_suspend(struct work_struct *work);
static ssize_t cap_vk_show(struct kobject *kobj, struct kobj_attribute *attr,char *buf);

static unsigned char *CTPM_FW = NULL; 
static u16 FW_len= 0;
static unsigned char CTPM_FW_Truly[] = {
#include "FT_Upgrade_App_Truly.i"
};
static unsigned char CTPM_FW_Ofilm[] = {
#include "FT_Upgrade_App_Ofilm.i"
};
static int tp_dev = TP_OFILM;


#define MSM_AUTO_DOWNLOAD

struct Upgrade_Info{
	u16		delay_aa;		/*delay of write FT_UPGRADE_AA*/
	u16		delay_55;		/*delay of write FT_UPGRADE_55*/
	u8		upgrade_id_1;	/*upgrade id 1*/
	u8		upgrade_id_2;	/*upgrade id 2*/
	u16		delay_readid;	/*delay of read id*/
	#ifdef MSM_AUTO_DOWNLOAD
	u8  download_id_1;	    /*download id 1 */
	u8  download_id_2;	    /*download id 2 */
	#endif
};

struct Upgrade_Info upgradeinfo;

#ifdef MSM_AUTO_DOWNLOAD
static unsigned char aucFW_PRAM_BOOT[] =
{
	#include "bootloader.i"
};

static unsigned char aucFW_ALL_AUO[] =
{
	#include "AUO_all.i"
};
#endif // TPD_AUTO_DOWNLOAD

//static struct mutex g_device_mutex;


static int focaltech_i2c_Read(struct i2c_client *client, char *writebuf,int writelen, char *readbuf, int readlen)
{
	int ret;
	mutex_lock(&i2c_rw_access);
	
	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
				 .addr = client->addr,
				 .flags = 0,
				 .len = writelen,
				 .buf = writebuf,
			 },
			{
				 .addr = client->addr,
				 .flags = I2C_M_RD,
				 .len = readlen,
				 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&client->dev, "%s: i2c read error.\n",
				__func__);
	} else {
		struct i2c_msg msgs[] = {
			{
				 .addr = client->addr,
				 .flags = I2C_M_RD,
				 .len = readlen,
				 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s:i2c read error.\n", __func__);
		}

	mutex_unlock(&i2c_rw_access);
	
	return ret;
}

static int focaltech_i2c_Write(struct i2c_client *client, char *writebuf,int writelen)
{
	int ret;
	struct i2c_msg msgs[] = {
		{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
		 },
	};

	mutex_lock(&i2c_rw_access);
	
	ret = i2c_transfer(client->adapter, msgs, 1);
	if (ret < 0)
		dev_err(&client->dev, "%s: i2c write error.\n", __func__);

	mutex_unlock(&i2c_rw_access);
	
	return ret;
}

static int focaltech_write_reg(struct i2c_client *client, u8 addr, const u8 val)
{
	u8 buf[2] = {0};

	buf[0] = addr;
	buf[1] = val;
	return focaltech_i2c_Write(client, buf, sizeof(buf));
}

static int focaltech_read_reg(struct i2c_client *client, u8 addr, u8 *val)
{
	return focaltech_i2c_Read(client, &addr, 1, val, 1);
}

static int i2c_device_test_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE];
	
	ret = sprintf(page, "%d\n", i2c_device_test);
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page)); 	
	return ret;
}

static const struct file_operations i2c_device_test_fops = {	
	.read =  i2c_device_test_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};


#ifdef SUPPORT_CHG_MODE
void focaltech_chg_mode_enble(bool enable)
{
	int ret = 0;	
	if(enable)
	{
		printk("focaltech chg mode will be enable\n");
		ret = focaltech_write_reg(data_g->client, CHG_MODE_ADDR, 0x01);
		if(ret < 0) {
			pr_err("%s: focaltech_write_reg failed,reg = 0xd0,ret = %d \n",__func__,ret);	
		}
	}else{
		printk("focaltech chg mode will be disable\n");
		ret = focaltech_write_reg(data_g->client, CHG_MODE_ADDR, 0x00);
		if(ret < 0) {
			pr_err("%s: focaltech_write_reg failed,reg = 0xd0,ret = %d \n",__func__,ret);	
		}	
	}	
	CHG_Mode = enable;
}
#endif


#ifdef SUPPORT_TP_SLEEP_MODE
static bool sleep_mode = false;

static void focaltech_sleep_mode_enable(struct focaltech_ts_data *data,int mode)
{
	int ret = 0;
	u8 state;	
	
	switch (mode){
	
	case FT_PMODE_ACTIVE:
	
		focaltech_read_reg(data->client, FT_REG_PMODE, &state);
		ret = focaltech_write_reg(data->client, FT_REG_PMODE, FT_PMODE_ACTIVE);
		if(ret < 0) {
			pr_err("%s: focaltech_write_reg failed,reg = %x,ret = %d \n",__func__,FT_REG_PMODE,ret);					
		}
		
		break;
	case FT_PMODE_MONITOR:
	
		focaltech_read_reg(data->client, FT_REG_PMODE, &state);
		ret = focaltech_write_reg(data->client, FT_REG_PMODE, FT_PMODE_MONITOR);
		if(ret < 0) {
			pr_err("%s: focaltech_write_reg failed,reg = %x,ret = %d \n",__func__,FT_REG_PMODE,ret);					
		}			
		break;
	case FT_PMODE_STANDBY:		
		
		ret = focaltech_write_reg(data->client, FT_REG_PMODE, FT_PMODE_STANDBY);
		if(ret < 0) {
			pr_err("%s: focaltech_write_reg failed,reg = %x,ret = %d \n",__func__,FT_REG_PMODE,ret);
		}
		break;
	case FT_PMODE_HIBERNATE:
	
		ret = focaltech_write_reg(data->client, FT_REG_PMODE, FT_PMODE_HIBERNATE);
		if(ret < 0) {
			pr_err("%s: focaltech_write_reg failed,reg = %x,ret = %d \n",__func__,FT_REG_PMODE,ret);					
		}	
		break;	
	}
}
static int tp_sleep_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
    int ret = 0;
	char page[PAGESIZE];
	printk("sleep mode enable is: %d\n", sleep_mode);
	ret = sprintf(page, "%d\n", sleep_mode);
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static int tp_sleep_write_func(struct file *file, const char *buffer, size_t count, loff_t *ppos)
{
    
    int ret = 0 ;
	char buf[10];
	if( count > 10 ) 
		return count;	
	if( copy_from_user( buf, buffer, count) ){
		printk(KERN_INFO "%s: read proc input error.\n", __func__);
		return count;
	}
	sscanf(buf, "%d", &ret); 
	
	printk("tp_sleep_write_func:buf = %d,ret = %d\n", *buf, ret);
	if( (ret == 0 ) || (ret == 1) ){
		sleep_mode = ret?true:false;		
	}
	switch(ret){
		case 0:
			printk("tp_sleep_func will be disable\n");
			focaltech_sleep_mode_enable(data_g,FT_PMODE_MONITOR);
			break;
		case 1:
			printk("tp_sleep_func will be enable\n");
			focaltech_sleep_mode_enable(data_g,FT_PMODE_STANDBY);
			break;
		default:
			printk("Please enter 0 or 1 to open or close the sleep function\n");
	}
	return count;
}

static const struct file_operations sleep_mode_enable_proc_fops = {
	.write = tp_sleep_write_func,
	.read =  tp_sleep_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};
#endif

#ifdef SUPPORT_GESTURE
static int focaltech_gesture_mode_enable(struct focaltech_ts_data *data, bool enable)
{
	int ret = 0;
	gesture_mode = enable;
	if(gesture_mode){
		printk("focaltech gesture mode will be enable\n");
		focaltech_write_reg(data->client, 0xd0, 0x01);
		if(ret < 0) {
			pr_err("%s: focaltech_write_reg failed,reg = 0xd0,ret = %d \n",__func__,ret);	
			return ret;
		}
		ret = focaltech_write_reg(data->client, 0xd1, 0x3f);
		if(ret < 0) {
			pr_err("%s: focaltech_write_reg failed,reg = 0xd1,ret = %d \n",__func__,ret);	
			return ret;
		}
		ret = focaltech_write_reg(data->client, 0xd2, 0x07);
		if(ret < 0) {
			pr_err("%s: focaltech_write_reg failed,reg = 0xd2,ret = %d \n",__func__,ret);	
			return ret;
		}
		ret = focaltech_write_reg(data->client, 0xd6, 0x1e);
		if(ret < 0) {
			pr_err("%s: focaltech_write_reg failed,reg = 0xd6,ret = %d \n",__func__,ret);	
			return ret;
		}		
	}else{
		printk("focaltech gesture mode will be disable\n");
		focaltech_write_reg(data->client, 0xd0, 0x00);
		if(ret < 0) {
			pr_err("%s: focaltech_write_reg failed,reg = 0xd0,ret = %d \n",__func__,ret);	
			return ret;
		}
		ret = focaltech_write_reg(data->client, 0xd1, 0x00);
		if(ret < 0) {
			pr_err("%s: focaltech_write_reg failed,reg = 0xd1,ret = %d \n",__func__,ret);	
			return ret;
		}
		ret = focaltech_write_reg(data->client, 0xd2, 0x00);
		if(ret < 0) {
			pr_err("%s: focaltech_write_reg failed,reg = 0xd2,ret = %d \n",__func__,ret);	
			return ret;
		}
		ret = focaltech_write_reg(data->client, 0xd6, 0x00);
		if(ret < 0) {
			pr_err("%s: focaltech_write_reg failed,reg = 0xd6,ret = %d \n",__func__,ret);	
			return ret;
		}
		
	}
	return 0;
}

static void focaltech_gesture_judge(struct focaltech_ts_data *data)
{
	unsigned char buf[FTS_GESTRUE_POINTS * 3] = { 0 }; 
	u8 reg_addr = 0xd3;
    int i = 0; 
    int ret = 0;	
    int gestrue_id = 0;
	int clk;
    short pointnum = 0;	
	struct i2c_client *client = data->client;
   
    memset(coordinate_report, 0, sizeof(coordinate_report));
	printk("%s  is call\n",__func__);	
    ret = focaltech_i2c_Read(client, &reg_addr, 1, buf, FTS_GESTRUE_POINTS_HEADER);	
    if (ret < 0){
        pr_err( "[focaltech]:%s read touchdata failed.\n", __func__);
        return ;
    }
    /* FW */
    if (data->family_id == 0x54){
		gestrue_id = buf[0];
		printk("[focaltech]:%s::gestrue_id =0x%x\n", __func__,gestrue_id);
		pointnum = (short)(buf[1]) & 0xff;	 
		if(gestrue_id != GESTURE_DOUBLELINE){	 
			if((pointnum * 4 + 2)<255){
				ret = focaltech_i2c_Read(client, &reg_addr, 1, buf, (pointnum * 4 + 2));
			}else{
				ret = focaltech_i2c_Read(client, &reg_addr, 1, buf, 255);
				ret = focaltech_i2c_Read(client, &reg_addr, 0, buf+255, (pointnum * 4 + 2) -255);
			}
			if (ret < 0){
				pr_err( "[focaltech]:%s read touchdata failed.\n", __func__);
				return ;
			}			
			for(i = 0;i < pointnum;i++){
				coordinate_x[i] =  (((s16) buf[2 + (4 * i)]) & 0x0F) <<
					8 | (((s16) buf[3 + (4 * i)])& 0xFF);
				coordinate_y[i] = (((s16) buf[4 + (4 * i)]) & 0x0F) <<
					8 | (((s16) buf[5 + (4 * i)]) & 0xFF);
				//printk("[focaltech]:pointx[%d] = %d,pointy[%d] = %d\n",i,coordinate_x[i],i,coordinate_y[i]);
			}
			if(gestrue_id != GESTURE_O && gestrue_id !=GESTURE_O_Anti){	 
				coordinate_report[1] = coordinate_x[0];
				coordinate_report[2] = coordinate_y[0];
				coordinate_report[3] = coordinate_x[pointnum-1];
				coordinate_report[4] = coordinate_y[pointnum-1];
				coordinate_report[5] = coordinate_x[1];
				coordinate_report[6] = coordinate_y[1];
				coordinate_report[7] = coordinate_x[2];
				coordinate_report[8] = coordinate_y[2];
				coordinate_report[9] = coordinate_x[3];
				coordinate_report[10] = coordinate_y[3];
				coordinate_report[11] = coordinate_x[4];
				coordinate_report[12] = coordinate_y[4];
			}else{				
				coordinate_report[1] = coordinate_x[0];
				coordinate_report[2] = coordinate_y[0];
				coordinate_report[3] = coordinate_x[pointnum-2];
				coordinate_report[4] = coordinate_y[pointnum-2];
				coordinate_report[5] = coordinate_x[1];
				coordinate_report[6] = coordinate_y[1];
				coordinate_report[7] = coordinate_x[2];
				coordinate_report[8] = coordinate_y[2];
				coordinate_report[9] = coordinate_x[3];
				coordinate_report[10] = coordinate_y[3];
				coordinate_report[11] = coordinate_x[4];
				coordinate_report[12] = coordinate_y[4];				
				clk = gestrue_id == GESTURE_O?1:0;
			}
     	}else{ // gesture DOUBLELINE
			if((pointnum * 4 + 4)<255){
				ret = focaltech_i2c_Read(client, &reg_addr, 1, buf, (pointnum * 4 + 4));
			}else{
				ret = focaltech_i2c_Read(client, &reg_addr, 1, buf, 255);
				ret = focaltech_i2c_Read(client, &reg_addr, 0, buf+255, (pointnum * 4 + 4) -255);
			}
			for(i = 0;i < pointnum;i++){
				coordinate_doblelion_1_x[i] =  (((s16) buf[2 + (4 * i)]) & 0x0F) <<
					8 | (((s16) buf[3 + (4 * i)])& 0xFF);
				coordinate_doblelion_1_y[i] = (((s16) buf[4 + (4 * i)]) & 0x0F) <<
					8 | (((s16) buf[5 + (4 * i)]) & 0xFF);
				//printk("pointx[%d] = %d,pointy[%d] = %d\n",i,coordinate_doblelion_1_x[i],i,coordinate_doblelion_1_y[i]);
			}
			coordinate_report[5] = coordinate_doblelion_1_x[0];
			coordinate_report[6] = coordinate_doblelion_1_y[0];
			coordinate_report[7] = coordinate_doblelion_1_x[pointnum-1];
			coordinate_report[8] = coordinate_doblelion_1_y[pointnum-1];
			pointnum = buf[pointnum * 4 + 2]<<8 |buf[pointnum * 4 + 3];
			 //ret = focaltech_i2c_Read(client, buf, 0, buf, (pointnum * 4));
			if((pointnum * 4 )<255){
				ret = focaltech_i2c_Read(client, &reg_addr, 0, buf, (pointnum * 4));
			}else{
				ret = focaltech_i2c_Read(client, &reg_addr, 0, buf, 255);
				ret = focaltech_i2c_Read(client, &reg_addr, 0, buf+255, (pointnum * 4) -255);
			}
			for(i = 0;i < pointnum;i++){
				coordinate_doblelion_2_x[i] =  (((s16) buf[0 + (4 * i)]) & 0x0F) <<
					8 | (((s16) buf[1 + (4 * i)])& 0xFF);
				coordinate_doblelion_2_y[i] = (((s16) buf[2 + (4 * i)]) & 0x0F) <<
					8 | (((s16) buf[3 + (4 * i)]) & 0xFF);
				//printk("pointx[%d] = %d,pointy[%d] = %d\n",i,coordinate_doblelion_2_x[i],i,coordinate_doblelion_2_y[i]);
			}
			if (ret < 0)			{
				pr_err( "[focaltech]:%s read touchdata failed.\n", __func__);
				return ;
			}			
			coordinate_report[1] = coordinate_doblelion_2_x[0];
			coordinate_report[2] = coordinate_doblelion_2_y[0];
			coordinate_report[3] = coordinate_doblelion_2_x[pointnum-1];
			coordinate_report[4] = coordinate_doblelion_2_y[pointnum-1];
			
		}		
		gesture = (gestrue_id == GESTURE_LEFT)        ? Right2LeftSwip :
				  (gestrue_id == GESTURE_RIGHT)       ? Left2RightSwip :
				  (gestrue_id == GESTURE_UP)          ? Down2UpSwip    :
				  (gestrue_id == GESTURE_DOWN)        ? Up2DownSwip    :
				  (gestrue_id == GESTURE_DOUBLECLICK) ? DouTap         :
				  (gestrue_id == GESTURE_DOUBLELINE)  ? DouSwip        :
				  (gestrue_id == GESTURE_LEFT_V)      ? RightVee       :
				  (gestrue_id == GESTURE_RIGHT_V)     ? LeftVee        :
				  (gestrue_id == GESTURE_UP_V)        ? DownVee        :
				  (gestrue_id == GESTURE_DOWN_V)      ? UpVee	       :
				  (gestrue_id == GESTURE_O)           ? Circle         :
				  (gestrue_id == GESTURE_W)           ? Wgestrue       :
                  (gestrue_id == GESTURE_M)           ? Mgestrue       :
				  (gestrue_id == GESTURE_O_Anti)      ? Circle         :
                  UnkownGestrue;
				  
		if(gesture != UnkownGestrue ){
			input_report_key(data->input_dev, KEY_F4, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_F4, 0);
			input_sync(data->input_dev);	
		}
		coordinate_report[0] = gesture;
		coordinate_report[13] = (gesture == Circle)?clk:2;
		
    }  
}

static int tp_gesture_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE];
	printk("gesture_enable is: %d\n", gesture_enable);
	ret = sprintf(page, "%d\n", gesture_enable);
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page)); 
	return ret;
}

static int tp_gesture_write_func(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{ 
	int ret = 0;
	char buf[10];
	if( count > 2) 
		return count;	
	if( copy_from_user(buf, buffer, count) ){
		printk(KERN_INFO "%s: read proc input error.\n", __func__);
		return count;
	}	
	sscanf(buf, "%d", &ret);
	if( (ret == 0 )||(ret == 1) ){
		gesture_enable = ret?true:false;	
		printk("gesture_enable = %d\n",gesture_enable);
	}
	if(data_g->suspended){
		switch(ret){
			case 0:				
				focaltech_gesture_mode_enable(data_g,false);
				focaltech_sleep_mode_enable(data_g,FT_PMODE_STANDBY);	
				break;
			case 1:				
				focaltech_sleep_mode_enable(data_g,FT_PMODE_MONITOR);	
				focaltech_gesture_mode_enable(data_g,true);
				break;
			default:
				pr_err("Please enter 0 or 1 to open or close the double-tap function\n");
		}
	}
	return count;
}
static const struct file_operations tp_double_proc_fops = {
	.write = tp_gesture_write_func,
	.read =  tp_gesture_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};


static int coordinate_proc_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{	
	int ret = 0;
	char page[PAGESIZE];
	ret = sprintf(page, "%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d\n", coordinate_report[0],
                   coordinate_report[1], coordinate_report[2], coordinate_report[3], coordinate_report[4],
                   coordinate_report[5], coordinate_report[6], coordinate_report[7], coordinate_report[8],
                   coordinate_report[9], coordinate_report[10], coordinate_report[11], coordinate_report[12],
                   coordinate_report[13]);
				   
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
    return ret;	
}
static const struct file_operations coordinate_proc_fops = {
	.read =  coordinate_proc_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};

#endif

#ifdef SUPPORT_GLOVES_MODE

static void focaltech_gloves_mode_enable(struct focaltech_ts_data *data,bool enable)
{
	
	int ret = 0;
	
	if(enable)
	{
		printk("focaltech gloves mode will be enable\n");	
		ret = focaltech_write_reg(data->client, GLOVES_MODE_ADDR, 0x01);
		if(ret < 0) {
			pr_err("%s: focaltech_write_reg failed,reg = 0xd0,ret = %d \n",__func__,ret);	
		}
	}else{
		printk("focaltech gloves mode will be disable\n");
		ret = focaltech_write_reg(data->client, GLOVES_MODE_ADDR, 0x00);
		if(ret < 0) {
			pr_err("%s: focaltech_write_reg failed,reg = 0xd0,ret = %d \n",__func__,ret);	
		}	
	}
	
}
static int tp_glove_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE];
	printk("glove mode enable is: %d\n", gloves_enable);
	ret = sprintf(page, "%d\n", gloves_enable);
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static int tp_glove_write_func(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	
	int ret = 0 ;
	char buf[10];
	
	//	down(&work_sem);
	if( count > 10 )
		goto GLOVE_ENABLE_END;
	if( copy_from_user( buf, buffer, count) ){
		printk(KERN_INFO "%s: read proc input error.\n", __func__);	
		goto GLOVE_ENABLE_END;
	}
	sscanf(buf, "%d", &ret);
	
	printk("tp_glove_write_func:buf = %d,ret = %d\n", *buf, ret);
	if( (ret == 0 ) || (ret == 1) ){
		gloves_enable = ret?true:false;
		focaltech_gloves_mode_enable(data_g,gloves_enable);
	}	
	switch(ret){	
		case 0:	
			//printk("tp_glove_func will be disable\n");			
			break;
		case 1:	
			//printk("tp_glove_func will be enable\n");			
			break;		
		default:
			printk("Please enter 0 or 1 to open or close the glove function\n");
	}
GLOVE_ENABLE_END:
//	up(&work_sem);
	return count;
}

static const struct file_operations glove_mode_enable_proc_fops = {
	.write = tp_glove_write_func,
	.read =  tp_glove_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};

#endif


#ifdef RESET_ONESECOND
static void focaltech_tp_reset(struct focaltech_ts_data *data)
{
	int i;
	u8 reg_addr = FT_REG_ID;
	u8	reg_value;
	
	if(data == NULL){
		pr_err("%s:focaltech_ts_data is NULL \n", __func__);
		return;
	}
	if (gpio_is_valid(data->pdata->reset_gpio)) {
		gpio_set_value_cansleep(data->pdata->reset_gpio, 0);
		msleep(data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
	}
	//msleep(data->pdata->soft_rst_dly);
	msleep(50);		
	for(i = 0; i < 15; i++){
		focaltech_i2c_Read(data->client, &reg_addr, 1, &reg_value, 1);
		if(reg_value == 0x54){
			break;
		}
		msleep(10);
	}

#ifdef SUPPORT_CHG_MODE
	focaltech_chg_mode_enble(CHG_Mode);
#endif	

#ifdef SUPPORT_GLOVES_MODE
	if(!data->suspended){
		focaltech_gloves_mode_enable(data,gloves_enable);
	}
#endif

#ifdef SUPPORT_GESTURE
	if(data->suspended){
		focaltech_gesture_mode_enable(data,gesture_mode);
	}
#endif	
	
}

static int tp_reset_write_func (struct file *file, const char *buffer, size_t count, loff_t *ppos)
{
	int ret = 0 ;
	char buf[10];
	if( count > 10 ) 
		return count;	
	if( copy_from_user( buf, buffer, count) ){
		printk(KERN_INFO "%s: read proc input error.\n", __func__);
		return count;
	}
	sscanf(buf, "%d", &ret);		
	printk("tp_reset_write_func:buf = %d,ret = %d\n", *buf, ret);	
	switch(ret){
		case 0:
			printk("tp_sleep_func will be disable\n");
			break;
		case 1:
			printk("tp_sleep_func will be enable\n");
			focaltech_tp_reset(data_g);
			break;
		default:
			printk("Please enter 0 or 1 to open or close the sleep function\n");
	}
	return count;
}

static const struct file_operations tp_reset_proc_fops = {
	.write = tp_reset_write_func,
	//.read =  tp_sleep_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};
#endif

static struct kobject *fts_properties_kobj;
static struct kobj_attribute qrd_virtual_keys_attr = {
    .attr = {
        .name = "virtualkeys."TPD_DEVICE,
        .mode = S_IRUGO,
    },
    .show = &cap_vk_show,
};

static struct attribute *qrd_properties_attrs[] = {
    &qrd_virtual_keys_attr.attr,
    NULL
};

static struct attribute_group qrd_properties_attr_group = {
    .attrs = qrd_properties_attrs,
};


static ssize_t cap_vk_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf){
      /* LEFT: search: CENTER: menu ,home:search 412, RIGHT: BACK */
	 if(is_project(OPPO_15005)){
		return sprintf(buf,
				__stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":%d:%d:%d:%d"
			":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE)   ":%d:%d:%d:%d"
			":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)   ":%d:%d:%d:%d"
			"\n",65,button_map[2],button_map[0],button_map[1],242,button_map[2],button_map[0],button_map[1],420,button_map[2],button_map[0],button_map[1]);
	 } else {
		return sprintf(buf,
				__stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":%d:%d:%d:%d"
			":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE)   ":%d:%d:%d:%d"
			":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)   ":%d:%d:%d:%d"
			"\n",LCD_WIDTH/6,button_map[2],button_map[0],button_map[1],LCD_WIDTH/2,button_map[2],button_map[0],button_map[1],LCD_WIDTH*5/6,button_map[2],button_map[0],button_map[1]);
	}
}




static int focaltech_tpd_button_init(struct focaltech_ts_data *data)
{
	int ret = 0;
	data->kpd = input_allocate_device();
    if( data->kpd == NULL ){
		ret = -ENOMEM;
		printk(KERN_ERR "focaltech_tpd_button_init: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	data->kpd->name = TPD_DEVICE "-kpd";
    set_bit(EV_KEY, data->kpd->evbit);
	__set_bit(KEY_MENU, data->kpd->keybit);
	__set_bit(KEY_HOMEPAGE, data->kpd->keybit);
	__set_bit(KEY_BACK, data->kpd->keybit);
	data->kpd->id.bustype = BUS_HOST;
    data->kpd->id.vendor  = 0x0001;
    data->kpd->id.product = 0x0001;
    data->kpd->id.version = 0x0100;
	
	if(input_register_device(data->kpd)){
        printk("input_register_device failed.(kpd)\n");	
		input_unregister_device(data->kpd);
		input_free_device(data->kpd);
	}
	
	report_key_point_y = data->pdata->y_max*button_map[2]/LCD_HEIGHT;
    fts_properties_kobj = kobject_create_and_add("board_properties", NULL);
    if( fts_properties_kobj )
        ret = sysfs_create_group(fts_properties_kobj, &qrd_properties_attr_group);
    if( !fts_properties_kobj || ret )
		printk("failed to create board_properties\n");	

	err_input_dev_alloc_failed:		
		return ret;
}


static int Dot_report_conut = 0;
static irqreturn_t focaltech_ts_interrupt(int irq, void *dev_id)
{
	struct focaltech_ts_data *data = dev_id;
	struct input_dev *ip_dev;
	int rc, i;
	u32 id, x, y, status, num_touches;
	u8 reg = 0x00, *buf;
	bool update_input = false;
	
	if (!data) {
		pr_err("%s: Invalid data\n", __func__);
		return IRQ_HANDLED;
	}
	
	ip_dev = data->input_dev;
	buf = data->tch_data;
	reg = 0x00;
	rc = focaltech_i2c_Read(data->client, &reg, 1,buf, data->tch_data_len);
	if (rc < 0) {
		dev_err(&data->client->dev, "%s: read data fail\n", __func__);
		return IRQ_HANDLED;
	}

	for (i = 0; i < data->pdata->num_max_touches; i++) {
		id = (buf[FT_TOUCH_ID_POS + FT_ONE_TCH_LEN * i]) >> 4;
		if (id >= FT_MAX_ID)
			break;

		update_input = true;

		x = (buf[FT_TOUCH_X_H_POS + FT_ONE_TCH_LEN * i] & 0x0F) << 8 |
			(buf[FT_TOUCH_X_L_POS + FT_ONE_TCH_LEN * i]);
		y = (buf[FT_TOUCH_Y_H_POS + FT_ONE_TCH_LEN * i] & 0x0F) << 8 |
			(buf[FT_TOUCH_Y_L_POS + FT_ONE_TCH_LEN * i]);

		status = buf[FT_TOUCH_EVENT_POS + FT_ONE_TCH_LEN * i] >> 6;
		num_touches = buf[FT_TD_STATUS] & FT_STATUS_NUM_TP_MASK;

		/* invalid combination */
		if (!num_touches && !status && !id)
			break;

		input_mt_slot(ip_dev, id);
		if (status == FT_TOUCH_DOWN || status == FT_TOUCH_CONTACT) {
			input_mt_report_slot_state(ip_dev, MT_TOOL_FINGER, 1);
			input_report_abs(ip_dev, ABS_MT_POSITION_X, x);
			input_report_abs(ip_dev, ABS_MT_POSITION_Y, y);
			if( Dot_report_conut == 50 ){
				pr_err(" [focaltech]: %s, x=%d, y=%d\n", __func__, x, y);
				Dot_report_conut = 0;
			}else{
				Dot_report_conut++;
			}				
		} else {
			input_mt_report_slot_state(ip_dev, MT_TOOL_FINGER, 0);
		}
	}

	if (update_input) {
		input_mt_report_pointer_emulation(ip_dev, false);
		input_sync(ip_dev);
	}
#ifdef SUPPORT_GESTURE	
	if(data->suspended){
		if(gesture_enable){	
			focaltech_gesture_judge(data);		
		}
	}
#endif

	return IRQ_HANDLED;
}

static int focaltech_power_on(struct focaltech_ts_data *data, bool on)
{
	int rc = 0;

	if (on)
	{		
		if (gpio_is_valid(data->pdata->reset_gpio)) {		
			gpio_set_value_cansleep(data->pdata->reset_gpio, 0);			
		}else{
			dev_err(&data->client->dev,"reseting @ TP power on failed\n");
		}		
		if (regulator_count_voltages(data->pdata->vdd) > 0) {
			rc = regulator_set_voltage(data->pdata->vdd, FT_VTG_MIN_UV,FT_VTG_MAX_UV);
			if (rc) {
				dev_err(&data->client->dev,"Regulator set_vtg failed vdd rc=%d\n", rc);
				regulator_put(data->pdata->vdd);
				return rc;
			}
		}		
		if (regulator_count_voltages(data->pdata->vcc_i2c) > 0) {
			rc = regulator_set_voltage(data->pdata->vcc_i2c, FT_I2C_VTG_MIN_UV,FT_I2C_VTG_MAX_UV);
			if (rc) {
				dev_err(&data->client->dev,"Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
				return rc;
			}
		}
    	rc = regulator_enable(data->pdata->vdd);
		if (rc) {
			dev_err(&data->client->dev,"Regulator vdd enable failed rc=%d\n", rc);
			return rc;
		}
		msleep(20);
		rc = regulator_enable(data->pdata->vcc_i2c);
		if (rc) {
			dev_err(&data->client->dev,"Regulator vcc_i2c enable failed rc=%d\n", rc);
			regulator_disable(data->pdata->vdd);
		}		
		if (gpio_is_valid(data->pdata->irq_gpio)) {		
			rc = gpio_direction_input(data->pdata->irq_gpio);		
		}				
		if (gpio_is_valid(data->pdata->reset_gpio)) {					
			gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
			dev_err(&data->client->dev,"reseting @ TP power on OK.\n");
		}	
		msleep(200);
		enable_irq(data->client->irq);		
	}else{	
		
		disable_irq_nosync(data->client->irq);	
		if (gpio_is_valid(data->pdata->reset_gpio)) {			
			gpio_set_value_cansleep(data->pdata->reset_gpio, 0);
		}
		mdelay(1);		
		if (regulator_count_voltages(data->pdata->vdd) > 0){
			regulator_set_voltage(data->pdata->vdd, 0, FT_VTG_MAX_UV);
		}

		if (regulator_count_voltages(data->pdata->vcc_i2c) > 0){
			regulator_set_voltage(data->pdata->vcc_i2c, 0, FT_I2C_VTG_MAX_UV);
		}
		
		rc = regulator_disable(data->pdata->vdd);
		if (rc) {
			dev_err(&data->client->dev,"Regulator vdd disable failed rc=%d\n", rc);
			return rc;
		}
		rc = regulator_disable(data->pdata->vcc_i2c);
		if (rc) {
			dev_err(&data->client->dev,"Regulator vcc_i2c disable failed rc=%d\n", rc);
			rc = regulator_enable(data->pdata->vdd);
			if (rc) {
				dev_err(&data->client->dev,"Regulator vdd enable failed rc=%d\n", rc);
			}
		}
	}
	return rc;
}

#ifdef CONFIG_PM

static void speedup_focaltech_suspend(struct work_struct *work)
{
	
	struct focaltech_ts_data *data = container_of(work, struct focaltech_ts_data, suspend_wk);
	
	pr_err("focaltech call %s \n", __func__);
	
	if(cancel_work_sync(&data->speed_up_work)){
		pr_err("focaltech call %s error: speed_up_work queue is not run\n", __func__);		
		return;	
	}	
	if (data->suspended) {		
		pr_err("focaltech call %s error: Already in suspend state\n", __func__);
		return;
	}
	
	data->suspended = true;
	input_set_drvdata(data->input_dev, data);
	i2c_set_clientdata(data->client, data);
	
	
#ifdef SUPPORT_GLOVES_MODE	
	focaltech_gloves_mode_enable(data,false);
#endif	

#ifdef SUPPORT_GESTURE	
	if( gesture_enable ){
		enable_irq_wake(data->client->irq);
		Is_IRQ_Wake = true;
		focaltech_gesture_mode_enable(data, true);		
		return;
	}	
#endif	
	Is_IRQ_Wake = false;
	if(irq_enable){
		disable_irq_nosync(data->client->irq);	
		irq_enable = false;		
	}
	if (gpio_is_valid(data->pdata->reset_gpio)) {		
		focaltech_sleep_mode_enable(data,FT_PMODE_HIBERNATE);
	}
}
static int focaltech_ts_suspend(struct device *dev)
{
	struct focaltech_ts_data *data = dev_get_drvdata(dev);	

	pr_err("focaltech call %s \n", __func__);
	if(data->input_dev == NULL)
	{
		pr_err("input_dev  registration is not complete\n");
		return -ENOMEM;
	}
	if (data->loading_fw) {
		dev_info(dev, "Firmware loading in process...\n");
		return 0;
	}
	queue_work(speedup_resume_wq, &(data->suspend_wk));
	return 0;
}

static void speedup_focaltech_resume(struct work_struct *work)
{
	int  i;
	struct focaltech_ts_data *data  = data_g;	
	
	if(cancel_work_sync(&data->suspend_wk)){
		pr_err("focaltech call %s error: suspend_wk queue is not run\n", __func__);		
		return;	
	}	
	
	data->suspended = false;	
	input_set_drvdata(data->input_dev, data);
	i2c_set_clientdata(data->client, data);
	
	focaltech_tp_reset(data);
	
#ifdef SUPPORT_GESTURE	
	if(Is_IRQ_Wake){
		disable_irq_wake(data->client->irq);
	}
	focaltech_gesture_mode_enable(data,false);	
#endif	
	if(!irq_enable){
		enable_irq(data->client->irq);
		irq_enable = true;
	}
	// release all touches 
	for (i = 0; i < data->pdata->num_max_touches; i++) {
		input_mt_slot(data->input_dev, i);
		input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 0);		
	}
	input_mt_report_pointer_emulation(data->input_dev, false);
	input_sync(data->input_dev);
	
	data_g = data;
	printk("speedup_focaltech_resume done\n");
}

static int focaltech_ts_resume(struct device *dev)
{
	struct focaltech_ts_data *data = dev_get_drvdata(dev);

	printk("focaltech call %s \n", __func__);
	if (data->loading_fw) {
		dev_info(dev, "Firmware loading in process...\n");
		return 0;
	}
	if (!data->suspended) {		
		pr_err("focaltech call %s error: Already in awake state\n", __func__);
		return 0;
	}
	queue_work(speedup_resume_wq, &data->speed_up_work);	

	return 0;
}

static const struct dev_pm_ops focaltech_ts_pm_ops = {
#if (!defined(CONFIG_FB) && !defined(CONFIG_HAS_EARLYSUSPEND))
	.suspend = focaltech_ts_suspend,
	.resume = focaltech_ts_resume,
#endif
};

#else
static int focaltech_ts_suspend(struct device *dev)
{
	return 0;
}

static int focaltech_ts_resume(struct device *dev)
{
	return 0;
}

#endif

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct focaltech_ts_data *focaltech_data =
		container_of(self, struct focaltech_ts_data, fb_notif);

	if (evdata && evdata->data && event == FB_EVENT_BLANK &&
			focaltech_data && focaltech_data->client) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK)
			focaltech_ts_resume(&focaltech_data->client->dev);
		else if (*blank == FB_BLANK_POWERDOWN){			
			focaltech_ts_suspend(&focaltech_data->client->dev);
			}
	}

	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void focaltech_ts_early_suspend(struct early_suspend *handler)
{
	struct focaltech_ts_data *data = container_of(handler,
						   struct focaltech_ts_data,
						   early_suspend);

	focaltech_ts_suspend(&data->client->dev);
}

static void focaltech_ts_late_resume(struct early_suspend *handler)
{
	struct focaltech_ts_data *data = container_of(handler,
						   struct focaltech_ts_data,
						   early_suspend);

	focaltech_ts_resume(&data->client->dev);
}
#endif

int hid_to_i2c(struct i2c_client * client)
{
	u8 auc_i2c_write_buf[5] = {0};
	int bRet = 0;

	auc_i2c_write_buf[0] = 0xeb;
	auc_i2c_write_buf[1] = 0xaa;
	auc_i2c_write_buf[2] = 0x09;

	focaltech_i2c_Write(client, auc_i2c_write_buf, 3);

	msleep(10);

	auc_i2c_write_buf[0] = auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = 0;

	focaltech_i2c_Read(client, auc_i2c_write_buf, 0, auc_i2c_write_buf, 3);
	printk("%s, auc_i2c_write_buf=0x%x, 0x%x, 0x%x\n", __func__, 
		auc_i2c_write_buf[0], auc_i2c_write_buf[1], auc_i2c_write_buf[2]);
	if(0xeb==auc_i2c_write_buf[0] && 0xaa==auc_i2c_write_buf[1] && 0x08==auc_i2c_write_buf[2])
		bRet = 1;		
	else
		bRet = 0;

	return bRet;
	
}

/**************************************fts fw upgrade and download begin*******************/

static void fts_get_upgrade_info(struct Upgrade_Info * upgrade_info)
{
	switch(DEVICE_IC_TYPE)
	{
		
	case IC_FT5X46:
		upgrade_info->delay_55 = FT5X46_UPGRADE_55_DELAY;
		upgrade_info->delay_aa = FT5X46_UPGRADE_AA_DELAY;
		upgrade_info->upgrade_id_1 = FT5X46_UPGRADE_ID_1;
		upgrade_info->upgrade_id_2 = FT5X46_UPGRADE_ID_2;
		upgrade_info->delay_readid = FT5X46_UPGRADE_READID_DELAY;
		#ifdef MSM_AUTO_DOWNLOAD
		upgrade_info->download_id_1 = 0x54;	    /*download id 1 */
		upgrade_info->download_id_2 = 0x2b;	    /*download id 2 */
		#endif
		break;
	default:
		break;
	}
}



static void focaltech_get_fw_info(struct focaltech_ts_data *data)
{
	struct i2c_client *client = data->client;
	u8 reg_addr;
	int err;

	reg_addr = FT_REG_FW_VER;
	err = focaltech_i2c_Read(client, &reg_addr, 1, &data->fw_ver[0], 1);
	if (err < 0){
		dev_err(&client->dev, "fw major version read failed");
	}
	reg_addr = FT_REG_FW_MIN_VER;
	err = focaltech_i2c_Read(client, &reg_addr, 1, &data->fw_ver[1], 1);
	if (err < 0){
		dev_err(&client->dev, "fw minor version read failed");
	}
	reg_addr = FT_REG_FW_SUB_MIN_VER;
	err = focaltech_i2c_Read(client, &reg_addr, 1, &data->fw_ver[2], 1);
	if (err < 0){
		dev_err(&client->dev, "fw sub minor version read failed");
	}
	reg_addr = FT_REG_FW_VENDOR_ID;
	err = focaltech_i2c_Read(client, &reg_addr, 1, &data->fw_vendor_id, 1);
	if (err < 0){
		dev_err(&client->dev, "fw vendor id read failed");
	}
	if(data->fw_vendor_id == 0x5a){
		data->fw_manufacture = TP_TRULY;
	}else if(data->fw_vendor_id == 0x51){
		data->fw_manufacture = TP_OFILM;
	}else{
		data->fw_manufacture = TP_UNKNOWN;
	}
	pr_err("%s, Firmware version = 0x%02x.%d.%d,\n", __func__, data->fw_ver[0], data->fw_ver[1], data->fw_ver[2]);
}


static int focaltech_fw_check(struct focaltech_ts_data *data)
{
	struct i2c_client *client = data->client;	
	int err;
	u8 reg_value;
	u8 reg_addr;	
	if(!data){
		pr_err("%s data is NULL\n",__func__);	
		return -1;
	}
	focaltech_get_fw_info(data);		
#ifdef SUPPORT_READ_TP_VERSION
	memset(fw_version, 0, sizeof(fw_version));
	sprintf(fw_version, "vid:%d, fw:0x%x, ic:%s\n",data->fw_vendor_id, data->fw_ver[0], "FT5346");
	init_tp_fm_info(data->fw_ver[0], fw_version, "FocalTech");
#endif

	// check the controller id 
	reg_addr = FT_REG_ID;	
	err = focaltech_i2c_Read(client, &reg_addr, 1, &reg_value, 1);
	if (err < 0) {
		dev_err(&client->dev, "version read failed");
		i2c_device_test = false;
		return -1;
	}
	pr_err( "focaltech_i2c_Read ok, Device ID = 0x%x\n", reg_value);		
	i2c_device_test = true;
	if (((data->family_id != reg_value) && (!data->pdata->ignore_id_check)) || (data->fw_manufacture != tp_dev)) {
		
		pr_err( "[focaltech]%s:Firmware is not ok,family_id =0x%x,ignore_id_check = 0x%x ,fw_manufacture = %d,tp_dev = %d\n ", __func__,data->family_id,data->pdata->ignore_id_check,data->fw_manufacture,tp_dev);
		return -1;
	}	
	return 0;
}

int  fts_ctpm_fw_upgrade(struct i2c_client * client, u8* pbt_buf, u32 dw_lenth)
{
	u8 reg_val[4] = {0};
	u32 i = 0;
	u32 packet_number;
	u32 j;
	u32 temp;
	u32 lenght;
	u8 packet_buf[FTS_PACKET_LENGTH + 6];
	u8 auc_i2c_write_buf[10];
	u8 bt_ecc;
	int i_ret;	
	
	i_ret = hid_to_i2c(client);

	if(i_ret == 0)
	{
		pr_err("hid_to_i2c fail ! \n");
	}
	fts_get_upgrade_info(&upgradeinfo);
	for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
		/*********Step 1:Reset  CTPM *****/
		focaltech_write_reg(client, 0xfc, FT_UPGRADE_AA);
		msleep(upgradeinfo.delay_aa);
		focaltech_write_reg(client, 0xfc, FT_UPGRADE_55);
		msleep(200);
		/*********Step 2:Enter upgrade mode *****/
		i_ret = hid_to_i2c(client);

		if(i_ret == 0)
		{
			pr_err("hid_to_i2c fail ! \n");			
		}
		msleep(10);
		auc_i2c_write_buf[0] = FT_UPGRADE_55;
		auc_i2c_write_buf[1] = FT_UPGRADE_AA;
		i_ret = focaltech_i2c_Write(client, auc_i2c_write_buf, 2);
		if(i_ret < 0)
		{
			pr_err("failed writing  0x55 and 0xaa ! \n");
			continue;
		}

		/*********Step 3:check READ-ID***********************/
		msleep(1);
		auc_i2c_write_buf[0] = 0x90;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] = 0x00;

		reg_val[0] = reg_val[1] = 0x00;

		focaltech_i2c_Read(client, auc_i2c_write_buf, 4, reg_val, 2);

		if (reg_val[0] == upgradeinfo.upgrade_id_1
			&& reg_val[1] == upgradeinfo.upgrade_id_2) {
				pr_err("[FTS] Step 3: READ OK CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0], reg_val[1]);
				break;
		} else {
			dev_err(&client->dev, "[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);

			continue;
		}
	}
	if (i >= FTS_UPGRADE_LOOP )
		return -EIO;

	/*Step 4:erase app and panel paramenter area*/
	pr_err("Step 4:erase app and panel paramenter area\n");
	auc_i2c_write_buf[0] = 0x61;
	focaltech_i2c_Write(client, auc_i2c_write_buf, 1);     //erase app area 
	msleep(1650);

	for(i = 0;i < 25;i++)
	{
		auc_i2c_write_buf[0] = 0x6a;
		reg_val[0] = reg_val[1] = 0x00;
		focaltech_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 2);

		if(0xF0==reg_val[0] && 0xAA==reg_val[1])
		{
			break;
		}
		msleep(50);
	}

       //write bin file length to FW bootloader.
	auc_i2c_write_buf[0] = 0xB0;
	auc_i2c_write_buf[1] = (u8) ((dw_lenth >> 16) & 0xFF);
	auc_i2c_write_buf[2] = (u8) ((dw_lenth >> 8) & 0xFF);
	auc_i2c_write_buf[3] = (u8) (dw_lenth & 0xFF);

	focaltech_i2c_Write(client, auc_i2c_write_buf, 4);

	/*********Step 5:write firmware(FW) to ctpm flash*********/
	bt_ecc = 0;
	pr_err("Step 5:write firmware(FW) to ctpm flash\n");

	//dw_lenth = dw_lenth - 8;
	temp = 0;
	//packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
	packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
	packet_buf[0] = 0xbf;
	packet_buf[1] = 0x00;

	pr_err("focaltech dw_lenth:%d,FTS_PACKET_LENGTH:%d,packet_number:%d\n",dw_lenth,FTS_PACKET_LENGTH,packet_number);
	for (j = 0; j < packet_number; j++) {
		temp = j * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		lenght = FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (lenght >> 8);
		packet_buf[5] = (u8) lenght;
	
		for (i = 0; i < FTS_PACKET_LENGTH; i++) {
			packet_buf[6 + i] = pbt_buf[j * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}
		i_ret = focaltech_i2c_Write(client, packet_buf, FTS_PACKET_LENGTH + 6);
		//msleep(20);
		//pr_err("focaltech_i2c_Write result %x\n",i_ret);
		
		for(i = 0;i < 30;i++)
		{
			auc_i2c_write_buf[0] = 0x6a;
			reg_val[0] = reg_val[1] = 0x00;
			i_ret =focaltech_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 2);
			//pr_err("focaltech_i2c_Write result %x\n",i_ret);
			if ((j + 0x1000) == (((reg_val[0]) << 8) | reg_val[1]))
			{
				break;
			}
			msleep(2);
		}
		//pr_err("focaltech j= %d,i=%d \n",j,i);
		if (i == 30)
		{
			pr_err("focaltech_packet_number j + 0x1000 = %d,reg_val=%d \n",(j + 0x1000),((reg_val[0]) << 8) | reg_val[1]);
			break;
		}
		
	}
	//pr_err("focaltech_packet_number %x\n",j);
	if ((dw_lenth) % FTS_PACKET_LENGTH > 0) {
		temp = packet_number * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		temp = (dw_lenth) % FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (temp >> 8);
		packet_buf[5] = (u8) temp;

		for (i = 0; i < temp; i++) {
			packet_buf[6 + i] = pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}        
		focaltech_i2c_Write(client, packet_buf, temp + 6);

		for(i = 0;i < 30;i++)
		{
			auc_i2c_write_buf[0] = 0x6a;
			reg_val[0] = reg_val[1] = 0x00;
			focaltech_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 2);

			if ((j + 0x1000) == (((reg_val[0]) << 8) | reg_val[1]))
			{
				break;
			}
			msleep(1);
		}
	}

	msleep(50);

	/*********Step 6: read out checksum***********************/
	/*send the opration head */
	pr_err("Step 6: read out checksum\n");
	auc_i2c_write_buf[0] = 0x64;
	focaltech_i2c_Write(client, auc_i2c_write_buf, 1); 
	msleep(300);

	temp = 0;
	auc_i2c_write_buf[0] = 0x65;
	auc_i2c_write_buf[1] = (u8)(temp >> 16);
	auc_i2c_write_buf[2] = (u8)(temp >> 8);
	auc_i2c_write_buf[3] = (u8)(temp);
	temp = dw_lenth;
	auc_i2c_write_buf[4] = (u8)(temp >> 8);
	auc_i2c_write_buf[5] = (u8)(temp);
	i_ret = focaltech_i2c_Write(client, auc_i2c_write_buf, 6); 
	msleep(dw_lenth/256);

	for(i = 0;i < 100;i++)
	{
		auc_i2c_write_buf[0] = 0x6a;
		reg_val[0] = reg_val[1] = 0x00;
		focaltech_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 2);

		if (0xF0==reg_val[0] && 0x55==reg_val[1])
		{
			break;
		}
		msleep(1);

	}

	auc_i2c_write_buf[0] = 0x66;
	focaltech_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 1);
	if (reg_val[0] != bt_ecc) 
	{
		dev_err(&client->dev, "[FTS]--ecc error! FW=%02x bt_ecc=%02x\n",
			reg_val[0],
			bt_ecc);

		return -EIO;
	}
	printk(KERN_WARNING "checksum %X %X \n",reg_val[0],bt_ecc);       
	/*********Step 7: reset the new FW***********************/
	pr_err("Step 7: reset the new FW\n");
	auc_i2c_write_buf[0] = 0x07;
	focaltech_i2c_Write(client, auc_i2c_write_buf, 1);
	msleep(200);   //make sure CTP startup normally 
	i_ret = hid_to_i2c(client);//Android to Std i2c.
	if(i_ret == 0)
	{
		pr_err("hid_to_i2c fail ! \n");
	}
	return 0;
}

int fts_ctpm_auto_clb(struct i2c_client * client)
{
	unsigned char uc_temp;
	unsigned char i ;

	/*start auto CLB*/
	msleep(200);
	focaltech_write_reg(client, 0, 0x40);  
	msleep(100);   /*make sure already enter factory mode*/
	focaltech_write_reg(client, 2, 0x4);  /*write command to start calibration*/
	msleep(300);

	for(i=0;i<100;i++)
	{
		focaltech_read_reg(client, 0, &uc_temp);
		if (0x0 == ((uc_temp&0x70)>>4))  /*return to normal mode, calibration finish*/
		{
			break;
		}
		msleep(20);	    
	}

	/*calibration OK*/
	focaltech_write_reg(client, 0, 0x40);  /*goto factory mode for store*/
	msleep(200);   /*make sure already enter factory mode*/
	focaltech_write_reg(client, 2, 0x5);  /*store CLB result*/
	msleep(300);
	focaltech_write_reg(client, 0, 0x0); /*return to normal mode*/ 
	msleep(300);
	/*store CLB result OK*/

	return 0;
}

int fts_ctpm_fw_upgrade_with_i_file(struct i2c_client * client)
{
	u8 * pbt_buf = NULL;
	int i_ret;

	/*judge the fw that will be upgraded
	* if illegal, then stop upgrade and return.
	*/
	if(FW_len<8 || FW_len>54*1024)
	{
		pr_err("FW length error\n");
		return -EIO;
	}	
	pr_err("FW upgrade");
	/*FW upgrade*/
	pbt_buf = CTPM_FW;
	/*call the upgrade function*/
	i_ret =  fts_ctpm_fw_upgrade(client, pbt_buf, FW_len);
	if (i_ret != 0)
	{
		dev_err(&client->dev, "[FTS] upgrade failed. err=%d.\n", i_ret);
	}
	else
	{
#ifdef AUTO_CLB
		fts_ctpm_auto_clb(client);  /*start auto CLB*/
#endif
	}
	return i_ret;
}

u8 fts_ctpm_get_i_file_ver(void)
{
	
	if (FW_len > 2)
	{
		return CTPM_FW[FW_len - 2];		
	}
	else
	{
		return 0x00; /*default value*/
	}
}

static ssize_t focaltech_update_fw_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct focaltech_ts_data *data = dev_get_drvdata(dev);
	return snprintf(buf, 2, "%d\n", data->loading_fw);
}

static ssize_t focaltech_update_fw_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	struct focaltech_ts_data *data = NULL;
	//struct focaltech_ts_data *data = dev_get_drvdata(dev);
	u8 uc_host_fm_ver;
	int i_ret;
	unsigned long fileval;	
	bool fw_upgrade_ok = false;
	u8   upgrade_times = 2;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	data = (struct focaltech_ts_data *) i2c_get_clientdata( client );	
	
	if (count > 2){
		return -EINVAL;
	}
	i_ret = kstrtoul(buf, 10, &fileval);
	if (i_ret != 0){		
		return i_ret;
	}
	if(!dev){
		dev_info(dev, "dev is NULL return!!!!!\n");
        return count;		
	}
	if (tp_probe_ok==0) {
		dev_info(dev, "probe not finished return!!!!\n");
		return count;
	}	
	uc_host_fm_ver = fts_ctpm_get_i_file_ver();
	
	sprintf(data->fw_version,"150050%x",uc_host_fm_ver);
	
	pr_err("focaltech_update_fw,old fw_ver = %x,new fw_ver = %x,fileval = %ld,tp_fw_ok = %d\n",data->fw_ver[0],uc_host_fm_ver,fileval,tp_fw_ok);
	if((fileval == 1)||(tp_fw_ok !=1)||(uc_host_fm_ver != data->fw_ver[0]))	{		
		pr_err("focaltech_update_fw,fw begin upgrade!\n");
		mutex_lock(&data->input_dev->mutex);
		pr_err("mutex_lock");
		disable_irq_nosync(client->irq);		
		if (!data->loading_fw) {
			data->loading_fw = true;
			while((upgrade_times--) && !fw_upgrade_ok){
				i_ret = fts_ctpm_fw_upgrade_with_i_file(client);    
				if (i_ret == 0)
				{
					msleep(300);
					fw_upgrade_ok = true;
					break;					
				}else{
					fw_upgrade_ok = false;
					pr_err("focaltech_update_fw fail,try again\n");
				}
			}
			if(fw_upgrade_ok){				
				pr_err("focaltech_update_fw ok\n");
					
			}else{
				pr_err("focaltech_update_fw failed\n");
			}
			data->loading_fw = false;
		}		
		enable_irq(client->irq);		
		mutex_unlock(&data->input_dev->mutex);
	}else{
		pr_err("focaltech_update_fw,fw will not be upgrade!\n");		
	}
	
	return count;
}

#ifdef MSM_AUTO_DOWNLOAD
void fts_ctpm_hw_reset(void)
{
	if (gpio_is_valid(data_g->pdata->reset_gpio)) 
	{
		gpio_set_value_cansleep(data_g->pdata->reset_gpio, 0);
		msleep(20);
		gpio_set_value_cansleep(data_g->pdata->reset_gpio, 1);
	}
	else
	{
		pr_err("data_g->pdata->reset_gpio is invalid ! \n");
	}
}
#define		MAX_R_FLASH_SIZE	256//32
#define		FT5508_CMD_READ_SRAM			0x04

int COMM_FLASH_FT5422_ReadSRAM(struct i2c_client * client, u16 StartAddr, u8 *pData, int ByetsNumToRead)
{	
	
	//u8 packet_buf[MAX_R_FLASH_SIZE] = {0};	
	int i_ret;	
	
	u8 cmd[6];
	
	cmd[0] = FT5508_CMD_READ_SRAM;
	cmd[1] = ~FT5508_CMD_READ_SRAM;
	cmd[2] = ((StartAddr>>9)&0x7f);
	cmd[3] = (u8)(StartAddr>>1);
	
	cmd[4] = (((ByetsNumToRead >> 1)-1)>>8);//FT5926¶ÁÐ´Êý¾ÝÊ±£¬Á½¸ö×Ö½ÚËãÒ»¸öÊý¾Ý£¬ByetsNumToRead»òByetsNumToWriteÒª¼õ°ë
	cmd[5] = (u8 )((ByetsNumToRead >> 1) -1);	
		
	i_ret = focaltech_i2c_Read(client, cmd, 6, pData, ByetsNumToRead);

	return i_ret;
}

int  fts_ctpm_Read_Pram(struct i2c_client * client, u8* pbt_buf, u32 dw_lenth)
{
	
	u8 packet_buf[MAX_R_FLASH_SIZE]={0};
	u8 auc_i2c_write_buf[10];	
	int i_ret;	
	int RamAddr = 0;
	int iReadLen=dw_lenth;
	auc_i2c_write_buf[0] = FT_UPGRADE_55;		
	auc_i2c_write_buf[1] = FT_UPGRADE_AA;
	
	
	
	//Step 1:Reset  CTPM 
	fts_ctpm_hw_reset();
	//Step 2:Enter debug mode 
	i_ret = focaltech_i2c_Write(client, auc_i2c_write_buf, 2);
	if(i_ret < 0)
	{
		
		pr_err("[FTS] failed writing  0x55 0xaa ! \n");
		i_ret = focaltech_i2c_Write(client, auc_i2c_write_buf, 2);
		if(i_ret < 0)
		{
			pr_err("[focaltech] fts_ctpm_Read_Pram:failed writing  0x55 0xaa ! \n");
			return i_ret;
		}
	}
	//Step 3:Read data from pram 		
	while(1)
	{
		if(RamAddr == iReadLen)
		{
			break;
		}
		else if(RamAddr+MAX_R_FLASH_SIZE > iReadLen)
		{
			i_ret = COMM_FLASH_FT5422_ReadSRAM(client, RamAddr, packet_buf, iReadLen-RamAddr);
			memcpy(pbt_buf+RamAddr, packet_buf, iReadLen-RamAddr);
			
			RamAddr=iReadLen;
			
			break;
		}
		else
		{
			i_ret = COMM_FLASH_FT5422_ReadSRAM(client, RamAddr, packet_buf, MAX_R_FLASH_SIZE);			
			memcpy(pbt_buf+RamAddr, packet_buf, MAX_R_FLASH_SIZE);
			RamAddr += MAX_R_FLASH_SIZE;
		}

		if(i_ret < 0)
		{			
			break;
		}			
	}	
	
	//*********Step 4: reset mcu***********************/
	pr_warn("[focaltech] fts_ctpm_Read_Pram:Step 4: start app\n");
	auc_i2c_write_buf[0] = 0x0a;
	auc_i2c_write_buf[1] = ~0x0a;
	focaltech_i2c_Write(client, auc_i2c_write_buf, 2);	
	return 0;
}


#define Max_PRAM_LENGH  (54*1024)
void fts_read_pram_data(struct i2c_client * client)
{

	u8 *buf1 = NULL;	
	int fd = -1;
    struct timespec   now_time;
    struct rtc_time   rtc_now_time;
    uint8_t  data_buf[64];
    mm_segment_t old_fs;
	
	buf1 = kmalloc(Max_PRAM_LENGH+1, GFP_ATOMIC);
		
	fts_ctpm_Read_Pram(client, buf1, Max_PRAM_LENGH);
	pr_err("magic number = %x,crc = %x %x %x %x %x %x %x %x \n",buf1[0],buf1[Max_PRAM_LENGH-8],buf1[Max_PRAM_LENGH-7],
			buf1[Max_PRAM_LENGH-6],buf1[Max_PRAM_LENGH-5],buf1[Max_PRAM_LENGH-4],buf1[Max_PRAM_LENGH-3],buf1[Max_PRAM_LENGH-2],buf1[Max_PRAM_LENGH-1]);	
	
	
	getnstimeofday(&now_time);
    rtc_time_to_tm(now_time.tv_sec, &rtc_now_time);
    sprintf(data_buf, "/sdcard/pramdata%02d%02d%02d-%02d%02d%02d.bin", 
                (rtc_now_time.tm_year+1900)%100, rtc_now_time.tm_mon+1, rtc_now_time.tm_mday,
                rtc_now_time.tm_hour, rtc_now_time.tm_min, rtc_now_time.tm_sec);
    old_fs = get_fs();
    set_fs(KERNEL_DS);
    fd = sys_open(data_buf, O_WRONLY | O_CREAT | O_TRUNC, 0);
    if (fd < 0) {            
		pr_err("Open log file '%s' failed.\n", data_buf);
        set_fs(old_fs);
     }	

	if (fd >= 0){       
        sys_write(fd, buf1, Max_PRAM_LENGH);
    }		
	if (fd >= 0) {
        sys_close(fd);
        set_fs(old_fs);
    }	
	
}
static int tp_read_pram_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	fts_read_pram_data(data_g->client);
	return ret;
}
static const struct file_operations tp_read_pram_proc_fops = {
	//.write = tp_write_pram_func,
	.read =  tp_read_pram_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};


static int init_proc_touchpanel_file_node(void)
{
	int ret = 0;
	struct proc_dir_entry *prEntry_tp = proc_mkdir("touchpanel", NULL);
	struct proc_dir_entry *prResult_tp = NULL;
	
	if( prEntry_tp == NULL ){
		ret = -ENOMEM;
		printk(KERN_INFO"init_focaltech_proc: Couldn't create TP proc entry\n");
		pr_err(KERN_INFO"init_focaltech_proc: Couldn't create TP proc entry\n");
	}
    	
#ifdef SUPPORT_GESTURE	
	prResult_tp = proc_create( "double_tap_enable", 0666, prEntry_tp, &tp_double_proc_fops);
	if(prResult_tp == NULL){
		ret = -ENOMEM;
		printk(KERN_INFO"init_focaltech_proc: Couldn't create proc entry\n");
	}	
	
	prResult_tp = proc_create("coordinate", 0444, prEntry_tp, &coordinate_proc_fops);
	if(prResult_tp == NULL){	   
		ret = -ENOMEM;	   
		printk(KERN_INFO"init_focaltech_proc: Couldn't create proc entry\n");
	}
	
	prResult_tp = proc_create("read_pramdata", 0444, prEntry_tp, &tp_read_pram_proc_fops);
	if(prResult_tp == NULL){	   
		ret = -ENOMEM;	   
		printk(KERN_INFO"init_focaltech_proc: Couldn't create proc entry\n");
	}
#endif	 

#ifdef SUPPORT_GLOVES_MODE
	
	prResult_tp = proc_create( "glove_mode_enable", 0666, prEntry_tp,&glove_mode_enable_proc_fops);
	if(prResult_tp == NULL) {
		ret = -ENOMEM;
		printk(KERN_INFO"init_focaltech_proc: Couldn't create proc entry\n");
	}	
#endif	
	
#ifdef SUPPORT_TP_SLEEP_MODE
	prResult_tp = proc_create("sleep_mode_enable", 0666, prEntry_tp, &sleep_mode_enable_proc_fops);
	if( prResult_tp == NULL ){	   
		ret = -ENOMEM;	   
		printk(KERN_INFO"init_focaltech_proc: Couldn't create proc entry\n");
	}
#endif	
		
#ifdef RESET_ONESECOND
	prResult_tp = proc_create( "tp_reset", 0666, prEntry_tp, &tp_reset_proc_fops);
	if( prResult_tp == NULL ){
		ret = -ENOMEM;
		printk(KERN_INFO"init_focaltech_proc: Couldn't create tp reset proc entry\n");
	}
#endif	
	prResult_tp = proc_create( "i2c_device_test", 0666, prEntry_tp, &i2c_device_test_fops);
	if(prResult_tp == NULL){
		ret = -ENOMEM;
		printk(KERN_INFO"init_focaltech_proc: Couldn't create proc entry\n");
	}
	
	return ret;
}

static DEVICE_ATTR(oppo_tp_fw_update, 0664, focaltech_update_fw_show, focaltech_update_fw_store);

int  fts_ctpm_fw_preupgrade_hwreset(struct i2c_client * client, u8* pbt_buf, u32 dw_lenth)
{
	u8 reg_val[4] = {0};
	u32 i = 0;
	u32 packet_number;
	u32 j;
	u32 temp;
	u32 lenght;
	u8 packet_buf[FTS_PACKET_LENGTH + 6];
	u8 auc_i2c_write_buf[10];
	u8 bt_ecc;
	int i_ret;
	u8 state;

	fts_get_upgrade_info(&upgradeinfo);
	
	for (i = 0; i < FTS_UPGRADE_LOOP; i++) 
	{
		/*********Step 1:Reset  CTPM *****/
		fts_ctpm_hw_reset();
		
		if (i<=15)
		{
			msleep(upgradeinfo.delay_55+i*3);
		}
		else
		{
			msleep(upgradeinfo.delay_55-(i-15)*2);
		}
		
		/*********Step 2:Enter upgrade mode *****/
		auc_i2c_write_buf[0] = FT_UPGRADE_55;
		i_ret = focaltech_i2c_Write(client, auc_i2c_write_buf, 1);
		if(i_ret < 0)
		{
			pr_err("[FTS] failed writing  0x55 ! \n");
			continue;
		}
		
		auc_i2c_write_buf[0] = FT_UPGRADE_AA;
		i_ret = focaltech_i2c_Write(client, auc_i2c_write_buf, 1);
		if(i_ret < 0)
		{
			pr_err("[FTS] failed writing  0xaa ! \n");
			continue;
		}

		/*********Step 3:check READ-ID***********************/
		msleep(1);
		auc_i2c_write_buf[0] = 0x90;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] =
			0x00;
		reg_val[0] = reg_val[1] = 0x00;
		
		focaltech_i2c_Read(client, auc_i2c_write_buf, 4, reg_val, 2);
		if (reg_val[0] == 0x54
			&& reg_val[1] == 0x22) 
		{
			focaltech_read_reg(client, 0xd0, &state);
			if(state == 0)
			{
				pr_err("[FTS] Step 3: READ State fail \n");
				continue;
			}

			pr_warn("[FTS] Step 3: i_ret = %d \n", i_ret);			
			pr_warn("[FTS] Step 3: READ CTPM ID OK,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0], reg_val[1]);			
			break;
		} 
		else 
		{
			dev_err(&client->dev, "[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0], reg_val[1]);
			continue;
		}
	}

	if (i >= FTS_UPGRADE_LOOP )
		return -EIO;

	/*********Step 4:write firmware(FW) to ctpm flash*********/
	bt_ecc = 0;
	pr_warn("Step 5:write firmware(FW) to ctpm flash\n");

	dw_lenth = dw_lenth - 8;
	temp = 0;
	packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
	packet_buf[0] = 0xae;
	packet_buf[1] = 0x00;

	pr_warn("packet_number: %d\n", packet_number);
	for (j = 0; j < packet_number; j++) {
		temp = j * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		lenght = FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (lenght >> 8);
		packet_buf[5] = (u8) lenght;

		for (i = 0; i < FTS_PACKET_LENGTH; i++) {
			packet_buf[6 + i] = pbt_buf[j * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}
		focaltech_i2c_Write(client, packet_buf, FTS_PACKET_LENGTH + 6);
	}

	if ((dw_lenth) % FTS_PACKET_LENGTH > 0) {
		temp = packet_number * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		temp = (dw_lenth) % FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (temp >> 8);
		packet_buf[5] = (u8) temp;

		for (i = 0; i < temp; i++) {
			packet_buf[6 + i] = pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}		
		focaltech_i2c_Write(client, packet_buf, temp + 6);
	}

	temp = FT_APP_INFO_ADDR;
	packet_buf[2] = (u8) (temp >> 8);
	packet_buf[3] = (u8) temp;
	temp = 8;
	packet_buf[4] = (u8) (temp >> 8);
	packet_buf[5] = (u8) temp;
	for (i = 0; i < 8; i++) 
	{
		packet_buf[6+i] = pbt_buf[dw_lenth + i];
		bt_ecc ^= packet_buf[6+i];
	}	
	focaltech_i2c_Write(client, packet_buf, 6+8);
	
	/*********Step 5: read out checksum***********************/
	/*send the opration head */
	pr_warn("Step 6: read out checksum\n");
	auc_i2c_write_buf[0] = 0xcc;
	//msleep(2);
	focaltech_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 1);
	if (reg_val[0] != bt_ecc) {
		dev_err(&client->dev, "[FTS]--ecc error! FW=%02x bt_ecc=%02x\n",
					reg_val[0],
					bt_ecc);	
		return -EIO;
	}
	pr_warn("checksum %X %X \n",reg_val[0],bt_ecc);
	pr_warn("Read flash and compare\n");
		
	msleep(50);

	/*********Step 6: start app***********************/
	pr_warn("Step 6: start app\n");
	auc_i2c_write_buf[0] = 0x08;
	focaltech_i2c_Write(client, auc_i2c_write_buf, 1);
	msleep(20);

	return 0;
}

int fts_ctpm_fw_download(struct i2c_client *client, u8 *pbt_buf, u32 dw_lenth)
{
	u8 reg_val[4] = {0};
	u32 i = 0;
	//u8 is_5336_new_bootloader = 0;
	//u8 is_5336_fwsize_30 = 0;
	u32 packet_number;
	u32 j=0;
	u32 temp;
	u32 lenght;
	u8 packet_buf[FTS_PACKET_LENGTH + 6];
	u8 auc_i2c_write_buf[10];
	u8 bt_ecc;
	int i_ret;

	fts_get_upgrade_info(&upgradeinfo);

	for (i = 0; i < FTS_UPGRADE_LOOP; i++)
	{
		msleep(100);

		pr_warn("[FTS] Download Step 1:Reset  CTPM\n");
		/*********Step 1:Reset  CTPM *****/

		/*write 0xaa to register 0xfc */
		focaltech_write_reg(client, 0xfc, FT_UPGRADE_AA);
		msleep(upgradeinfo.delay_aa);

		/*write 0x55 to register 0xfc */
		focaltech_write_reg(client, 0xfc, FT_UPGRADE_55);

		if (i<=15)
		{
			msleep(upgradeinfo.delay_55+i*3);
		}
		else
		{
			msleep(upgradeinfo.delay_55-(i-15)*2);
		}

		pr_warn("[FTS] Download Step 2:Enter upgrade mode \n");
		/*********Step 2:Enter upgrade mode *****/

		auc_i2c_write_buf[0] = FT_UPGRADE_55;
		focaltech_i2c_Write(client, auc_i2c_write_buf, 1);
		msleep(5);
		auc_i2c_write_buf[0] = FT_UPGRADE_AA;
		focaltech_i2c_Write(client, auc_i2c_write_buf, 1);

		/*********Step 3:check READ-ID***********************/
		msleep(upgradeinfo.delay_readid);
		auc_i2c_write_buf[0] = 0x90;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] =0x00;
		focaltech_i2c_Read(client, auc_i2c_write_buf, 4, reg_val, 2);
		
		pr_warn("[FTS] Download Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0], reg_val[1]);
		if (reg_val[0] == upgradeinfo.download_id_1 && reg_val[1] == upgradeinfo.download_id_2)
		{
			printk("[FTS] Download Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0], reg_val[1]);
			break;
		}
		else
		{
			pr_err("[FTS] Download Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0], reg_val[1]);
			continue;
		}
	}

	if (i >= FTS_UPGRADE_LOOP)
	{
		if (reg_val[0] != 0x00 || reg_val[1] != 0x00)
		{
			fts_ctpm_hw_reset();
			msleep(300);
		}
		return -EIO;
	}
 
	pr_warn("[FTS] Download Step 4:change to write flash mode\n");
	focaltech_write_reg(client, 0x09, 0x0a);

	/*Step 4:erase app and panel paramenter area*/
	pr_warn("[FTS] Download Step 4:erase app and panel paramenter area\n");
	auc_i2c_write_buf[0] = 0x61;
	focaltech_i2c_Write(client, auc_i2c_write_buf, 1);	/*erase app area */
	//msleep(upgradeinfo.delay_earse_flash);
	/*erase panel parameter area */

	for(i = 0; i < 15; i++)
	{
		auc_i2c_write_buf[0] = 0x6a;
		reg_val[0] = reg_val[1] = 0x00;
		focaltech_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 2);

		if(0xF0==reg_val[0] && 0xAA==reg_val[1])
		{
			break;
		}
		msleep(50);
	}

	pr_warn("[FTS] Download Step 5:write firmware(FW) to ctpm flash\n");
	/*********Step 5:write firmware(FW) to ctpm flash*********/
	bt_ecc = 0;

	//dw_lenth = dw_lenth - 8;
	temp = 0;
	packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
	packet_buf[0] = 0xbf;
	packet_buf[1] = 0x00;

	pr_warn("[FTS] Download Step 5:packet_number: %d\n", packet_number);
	for (j = 0; j < packet_number; j++)
	{
		temp = j * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		lenght = FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (lenght >> 8);
		packet_buf[5] = (u8) lenght;

		for (i = 0; i < FTS_PACKET_LENGTH; i++)
		{
			packet_buf[6 + i] = pbt_buf[j * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}
		focaltech_i2c_Write(client, packet_buf, FTS_PACKET_LENGTH + 6);
		//msleep(10);

		for(i = 0; i < 30; i++)
		{
			auc_i2c_write_buf[0] = 0x6a;
			reg_val[0] = reg_val[1] = 0x00;
			focaltech_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 2);

			if ((j + 0x1000) == (((reg_val[0]) << 8) | reg_val[1]))
			{
				break;
			}
			//msleep(1);

		}
	}

	if ((dw_lenth) % FTS_PACKET_LENGTH > 0)
	{
		temp = packet_number * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		temp = (dw_lenth) % FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (temp >> 8);
		packet_buf[5] = (u8) temp;

		for (i = 0; i < temp; i++)
		{
			packet_buf[6 + i] = pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}
		focaltech_i2c_Write(client, packet_buf, temp + 6);

		for(i = 0; i < 30; i++)
		{
			auc_i2c_write_buf[0] = 0x6a;
			reg_val[0] = reg_val[1] = 0x00;
			focaltech_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 2);

			if ((j + 0x1000) == (((reg_val[0]) << 8) | reg_val[1]))
			{
				break;
			}
			msleep(1);

		}
	}

	msleep(50);


	pr_warn("[FTS] Download Step 6: read out checksum\n");
	/*********Step 6: read out checksum***********************/
	auc_i2c_write_buf[0] = 0x64;
	focaltech_i2c_Write(client, auc_i2c_write_buf, 1);
	msleep(300);

	temp = 0;
	auc_i2c_write_buf[0] = 0x65;
	auc_i2c_write_buf[1] = (u8)(temp >> 16);
	auc_i2c_write_buf[2] = (u8)(temp >> 8);
	auc_i2c_write_buf[3] = (u8)(temp);
	temp = dw_lenth;
	auc_i2c_write_buf[4] = (u8)(temp >> 8);
	auc_i2c_write_buf[5] = (u8)(temp);
	i_ret = focaltech_i2c_Write(client, auc_i2c_write_buf, 6);
	msleep(dw_lenth/256);

	for(i = 0; i < 100; i++)
	{
		auc_i2c_write_buf[0] = 0x6a;
		reg_val[0] = reg_val[1] = 0x00;
		focaltech_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 2);

		if (0xF0==reg_val[0] && 0x55==reg_val[1])
		{
			break;
		}
		msleep(1);

	}

	auc_i2c_write_buf[0] = 0x66;
	focaltech_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 1);
	if (reg_val[0] != bt_ecc)
	{
		pr_err("[FTS] Download Step 6: ecc error! FW=%02x bt_ecc=%02x\n",
		        reg_val[0],
		        bt_ecc);

		return -EIO;
	}


	pr_warn("[FTS] Download Step 7: reset the new FW\n");
	/*********Step 7: reset the new FW***********************/
#if 1 //sw reset
	auc_i2c_write_buf[0] = 0x07;
	focaltech_i2c_Write(client, auc_i2c_write_buf, 1);
#else //hw reset
	fts_ctpm_hw_reset();
#endif
	msleep(300);	/*make sure CTP startup normally */

//	i_ret = fts_hidi2c_to_stdi2c(client);//Android to Std i2c.
	i_ret = hid_to_i2c(client);
	
	if(i_ret == 0)
	{
		pr_err("HidI2c change to StdI2c i_ret = %d ! \n", i_ret);
	}

	return 0;
}

int fts_ctpm_fw_download_with_i_file(struct i2c_client *client,
                                    u8 *fw_data, u32 fw_len)
{
	int i_ret;

	/*judge the fw that will be upgraded
	* if illegal, then stop upgrade and return.
	*/
	do
	{
		if (fw_data == NULL)
		{
			pr_err( "%s:FW data = NULL error\n", __func__);
			i_ret = -EIO;
			break;
		}

		if (fw_len < FTS_MIN_FW_LENGTH || fw_len > FTS_ALL_FW_LENGTH)
		{
			pr_err( "%s:FW length error\n", __func__);
			i_ret = -EIO;
			break;
		}

	    i_ret = fts_ctpm_fw_preupgrade_hwreset(client, aucFW_PRAM_BOOT, sizeof(aucFW_PRAM_BOOT));
		if (i_ret != 0)
		{
			pr_err( "%s:write pram failed. err.\n",
			         __func__);
			
			break;
		}
		
		/*call the upgrade function */
		i_ret = fts_ctpm_fw_download(client, fw_data, fw_len);

		if (i_ret != 0)
		{
			pr_err( "%s:upgrade failed. err.\n",
			         __func__);
		}
		
	} while(0);

	return i_ret;
}

static ssize_t focaltech_download_fw_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct focaltech_ts_data *data = dev_get_drvdata(dev);
	return snprintf(buf, 2, "%d\n", data->loading_fw);
}

static ssize_t focaltech_download_fw_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	struct focaltech_ts_data *data = NULL;
	//struct focaltech_ts_data *data = dev_get_drvdata(dev);
	//u8 uc_host_fm_ver;
	int i_ret;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	data = (struct focaltech_ts_data *) i2c_get_clientdata( client );
	
	
	pr_err("focaltech_download_fw_store");
	//mutex_lock(&g_device_mutex);
	//mutex_lock(&data->input_dev->mutex);
	pr_err("mutex_lock");
	disable_irq_nosync(client->irq);
	i_ret = fts_ctpm_fw_download_with_i_file(client, aucFW_ALL_AUO, sizeof(aucFW_ALL_AUO));
	if (i_ret != 0)
	{
		dev_err(dev, "%s ERROR:[FTS] upgrade failed ret=%d.\n", __FUNCTION__, i_ret);
	}
	enable_irq(client->irq);

	//mutex_unlock(&g_device_mutex);
	//mutex_unlock(&data->input_dev->mutex);
	
	return count;
}
static DEVICE_ATTR(oppo_tp_fw_download, 0664, focaltech_download_fw_show, focaltech_download_fw_store);
#endif
/**************************************fts fw upgrade and download end*******************/


/**************************************fts app debug begin*******************/
/*sysfs debug*/

/*
*get firmware size

@firmware_name:firmware name
*note:the firmware default path is /storage/sdcard0.
	if you want to change the dir, please modify by yourself.
*/
#ifdef FTS_APK_DEBUG
static int fts_GetFirmwareSize(char *firmware_name)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize = 0;
	char filepath[128];
	memset(filepath, 0, sizeof(filepath));
 
	sprintf(filepath, "/storage/sdcard0/%s", firmware_name);

	if (NULL == pfile)
		pfile = filp_open(filepath, O_RDONLY, 0);

	if (IS_ERR(pfile)) {
		pr_err("error occured while opening file %s.\n", filepath);
		return -EIO;
	}

	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	filp_close(pfile, NULL);
	return fsize;
}



/*
*read firmware buf for .bin file.

@firmware_name: fireware name
@firmware_buf: data buf of fireware

note:the firmware default path is /storage/sdcard0.
	if you want to change the dir, please modify by yourself.
*/
static int fts_ReadFirmware(char *firmware_name,
			       unsigned char *firmware_buf)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize;
	char filepath[128];
	loff_t pos;
	mm_segment_t old_fs;

	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "/storage/sdcard0/%s", firmware_name);
	if (NULL == pfile)
		pfile = filp_open(filepath, O_RDONLY, 0);
	if (IS_ERR(pfile)) {
		pr_err("error occured while opening file %s.\n", filepath);
		return -EIO;
	}

	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_read(pfile, firmware_buf, fsize, &pos);
	filp_close(pfile, NULL);
	set_fs(old_fs);

	return 0;
}



/*
upgrade with *.bin file
*/

int fts_ctpm_fw_upgrade_with_app_file(struct i2c_client *client,
				       char *firmware_name)
{
	u8 *pbt_buf = NULL;
	int i_ret;
	int fwsize = fts_GetFirmwareSize(firmware_name);

	if (fwsize <= 0) {
		dev_err(&client->dev, "%s ERROR:Get firmware size failed\n",
					__func__);
		return -EIO;
	}

	if (fwsize < 8 || fwsize > 54 * 1024) {
		dev_dbg(&client->dev, "%s:FW length error\n", __func__);
		return -EIO;
	}

	/*=========FW upgrade========================*/
	pbt_buf = kmalloc(fwsize + 1, GFP_ATOMIC);

	if (fts_ReadFirmware(firmware_name, pbt_buf)) {
		dev_err(&client->dev, "%s() - ERROR: request_firmware failed\n",
					__func__);
		kfree(pbt_buf);
		return -EIO;
	}

	i_ret = fts_ctpm_fw_upgrade(client, pbt_buf, fwsize);
	
	if (i_ret != 0)
		dev_err(&client->dev, "%s() - ERROR:[FTS] upgrade failed..\n",
					__func__);
	
	kfree(pbt_buf);

	return i_ret;
}

static ssize_t fts_tpfwver_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	ssize_t num_read_chars = 0;
	u8 fwver = 0;
	//struct ft5x06_ts_data *data = NULL;
	
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	//data = (struct ft5x06_ts_data *) i2c_get_clientdata( client );

	//mutex_lock(&g_device_mutex);

	if (focaltech_read_reg(client, FT_REG_FW_VER, &fwver) < 0)
		num_read_chars = snprintf(buf, 128,
					"get tp fw version fail!\n");
	else
		num_read_chars = snprintf(buf, 128, "%02X\n", fwver);

	//mutex_unlock(&g_device_mutex);

	return num_read_chars;
}

static ssize_t fts_tpfwver_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	/*place holder for future use*/
	return -EPERM;
}



static ssize_t fts_tprwreg_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	/*place holder for future use*/
	return -EPERM;
}

static ssize_t fts_tprwreg_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	ssize_t num_read_chars = 0;
	int retval;
	long unsigned int wmreg = 0;
	u8 regaddr = 0xff, regvalue = 0xff;
	u8 valbuf[5] = {0};

	memset(valbuf, 0, sizeof(valbuf));
	//mutex_lock(&g_device_mutex);
	num_read_chars = count - 1;

	if (num_read_chars != 2) {
		if (num_read_chars != 4) {
			pr_info("please input 2 or 4 character\n");
			goto error_return;
		}
	}

	memcpy(valbuf, buf, num_read_chars);
	retval = strict_strtoul(valbuf, 16, &wmreg);

	if (0 != retval) {
		dev_err(&client->dev, "%s() - ERROR: Could not convert the "\
						"given input to a number." \
						"The given input was: \"%s\"\n",
						__func__, buf);
		goto error_return;
	}

	if (2 == num_read_chars) {
		/*read register*/
		regaddr = wmreg;
		if (focaltech_read_reg(client, regaddr, &regvalue) < 0)
			dev_err(&client->dev, "Could not read the register(0x%02x)\n",
						regaddr);
		else
			pr_info("the register(0x%02x) is 0x%02x\n",
					regaddr, regvalue);
	} else {
		regaddr = wmreg >> 8;
		regvalue = wmreg;
		if (focaltech_write_reg(client, regaddr, regvalue) < 0)
			dev_err(&client->dev, "Could not write the register(0x%02x)\n",
							regaddr);
		else
			dev_err(&client->dev, "Write 0x%02x into register(0x%02x) successful\n",
							regvalue, regaddr);
	}

error_return:
	//mutex_unlock(&g_device_mutex);

	return count;
}

static ssize_t fts_fwupgradeapp_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	/*place holder for future use*/
	return -EPERM;
}


/*upgrade from app.bin*/
static ssize_t fts_fwupgradeapp_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	char fwname[128];
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	memset(fwname, 0, sizeof(fwname));
	sprintf(fwname, "%s", buf);
	fwname[count - 1] = '\0';

	//mutex_lock(&g_device_mutex);
	disable_irq_nosync(client->irq);

	fts_ctpm_fw_upgrade_with_app_file(client, fwname);

	enable_irq(client->irq);
	//mutex_unlock(&g_device_mutex);

	return count;
}



/*get the fw version *example:cat ftstpfwver */
static DEVICE_ATTR(ftstpfwver, 0664, fts_tpfwver_show,fts_tpfwver_store);

/*read and write register
*read example: echo 88 > ftstprwreg ---read register 0x88
*write example:echo 8807 > ftstprwreg ---write 0x07 into register 0x88
*
*note:the number of input must be 2 or 4.if it not enough,please fill in the 0.
*/
static DEVICE_ATTR(ftstprwreg, 0664, fts_tprwreg_show,fts_tprwreg_store);

/*upgrade from app.bin
*example:echo "*_app.bin" > ftsfwupgradeapp
*/
static DEVICE_ATTR(ftsfwupgradeapp, 0664, fts_fwupgradeapp_show,fts_fwupgradeapp_store);


static unsigned char proc_operate_mode = PROC_READ_REGISTER;
static struct proc_dir_entry *fts_proc_entry;

/*interface of write proc*/

//static ssize_t fts_debug_write(struct file *filp,const char __user *buff, unsigned long count, void *data)

static ssize_t fts_debug_write(struct file *file, const char __user *buff, size_t count, loff_t *ppos)
{
	//struct i2c_client *client = (struct i2c_client *)fts_proc_entry->data;
	unsigned char writebuf[128];
	int buflen = count;
	int writelen = 0;
	int ret = 0;
	
	if (copy_from_user(&writebuf, buff, buflen)) {
		dev_err(&data_g->client->dev, "%s:copy from user error\n", __func__);
		return -EFAULT;
	}
	proc_operate_mode = writebuf[0];
	switch (proc_operate_mode) {
	case PROC_UPGRADE:
		{
			char upgrade_file_path[128];
			memset(upgrade_file_path, 0, sizeof(upgrade_file_path));
			sprintf(upgrade_file_path, "%s", writebuf + 1);
			upgrade_file_path[buflen-1] = '\0';
			printk("%s\n", upgrade_file_path);
			disable_irq_nosync(data_g->client->irq);

			ret = fts_ctpm_fw_upgrade_with_app_file(data_g->client, upgrade_file_path);

			enable_irq(data_g->client->irq);
			if (ret < 0) {
				dev_err(&data_g->client->dev, "%s:upgrade failed.\n", __func__);
				return ret;
			}
		}
		break;
	case PROC_READ_REGISTER:
		writelen = 1;
		ret = focaltech_i2c_Write(data_g->client, writebuf + 1, writelen);
		if (ret < 0) {
			dev_err(&data_g->client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_WRITE_REGISTER:
		writelen = 2;
		ret = focaltech_i2c_Write(data_g->client, writebuf + 1, writelen);
		if (ret < 0) {
			dev_err(&data_g->client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_AUTOCLB:
		printk("%s: autoclb\n", __func__);
		fts_ctpm_auto_clb(data_g->client);
		break;
	case PROC_READ_DATA:
	case PROC_WRITE_DATA:
		writelen = count - 1;
		ret = focaltech_i2c_Write(data_g->client, writebuf + 1, writelen);
		if (ret < 0) {
			dev_err(&data_g->client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	default:
		break;
	}	

	return count;
}

/*interface of read proc*/
//static int fts_debug_read(struct file *filp,/const char __user *buff, unsigned long count, void *data)
static ssize_t fts_debug_read(struct file *file, char __user *buff, size_t count, loff_t *ppos)
{
	//struct i2c_client *client = (struct i2c_client *)fts_proc_entry->data;
	int ret = 0;
	char buf[1016];		
	int num_read_chars = 0;
	int readlen = 0;
	u8 regvalue = 0x00, regaddr = 0x00;
	
	switch (proc_operate_mode){
	case PROC_UPGRADE:
		//after calling ft5x0x_debug_write to upgrade
		regaddr = 0xA6;
		ret = focaltech_read_reg(data_g->client, regaddr, &regvalue);
		if (ret < 0)
			num_read_chars = sprintf(buf, "%s", "get fw version failed.\n");
		else
			num_read_chars = sprintf(buf, "current fw version:0x%02x\n", regvalue);
		break;
	case PROC_READ_REGISTER:
		readlen = 1;
		ret = focaltech_i2c_Read(data_g->client, NULL, 0, buf, readlen);
		if (ret < 0) {
			dev_err(&data_g->client->dev, "%s:read iic error\n", __func__);
			return ret;
		} 
		num_read_chars = 1;
		break;
	case PROC_READ_DATA:
		readlen = count;
		ret = focaltech_i2c_Read(data_g->client, NULL, 0, buf, readlen);
		if (ret < 0) {
			dev_err(&data_g->client->dev, "%s:read iic error\n", __func__);
			return ret;
		}
		num_read_chars = readlen;
		break;
	case PROC_WRITE_DATA:
		break;
	default:
		break;
	}	
	
	if (copy_to_user(buff, buf, num_read_chars)) {
		dev_err(&data_g->client->dev, "%s:copy to user error\n", __func__);
		return -EFAULT;
	}
	return num_read_chars;
}
static const struct file_operations fts_proc_fops = {
	.write = fts_debug_write,
	.read = fts_debug_read,
//	.open = simple_open,
//	.owner = THIS_MODULE,
};
int fts_create_apk_debug_channel(struct i2c_client * client)
{
	fts_proc_entry = proc_create(PROC_NAME, 0777, NULL, &fts_proc_fops);
	if (NULL == fts_proc_entry) {
		dev_err(&client->dev, "Couldn't create proc entry!\n");
		return -ENOMEM;
	} else {
		dev_info(&client->dev, "Create proc entry success!\n");
		//fts_proc_entry->data = client;
	}
	return 0;
}

void fts_release_apk_debug_channel(void)
{
	if (fts_proc_entry)
		remove_proc_entry(PROC_NAME, NULL);
}
#endif
/**************************************fts app debug end*******************/
static bool focaltech_debug_addr_is_valid(int addr)
{
	if (addr < 0 || addr > 0xFF) {
		pr_err("FT reg address is invalid: 0x%x\n", addr);
		return false;
	}

	return true;
}

static int focaltech_debug_data_set(void *_data, u64 val)
{
	struct focaltech_ts_data *data = _data;

	//mutex_lock(&data->input_dev->mutex);

	if (focaltech_debug_addr_is_valid(data->addr))
		dev_info(&data->client->dev,
			"Writing into FT registers not supported\n");

	//mutex_unlock(&data->input_dev->mutex);
	return 0;
}

static int focaltech_debug_data_get(void *_data, u64 *val)
{
	struct focaltech_ts_data *data = _data;
	int rc;
	u8 reg;

	//mutex_lock(&data->input_dev->mutex);

	if (focaltech_debug_addr_is_valid(data->addr)) {
		rc = focaltech_read_reg(data->client, data->addr, &reg);
		if (rc < 0)
			dev_err(&data->client->dev,
				"FT read register 0x%x failed (%d)\n",
				data->addr, rc);
		else
			*val = reg;
	}

	//mutex_unlock(&data->input_dev->mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_data_fops, focaltech_debug_data_get,
			focaltech_debug_data_set, "0x%02llX\n");

static int focaltech_debug_addr_set(void *_data, u64 val)
{
	struct focaltech_ts_data *data = _data;

	if (focaltech_debug_addr_is_valid(val)) {
		//mutex_lock(&data->input_dev->mutex);
		data->addr = val;
		//mutex_unlock(&data->input_dev->mutex);
	}

	return 0;
}

static int focaltech_debug_addr_get(void *_data, u64 *val)
{
	struct focaltech_ts_data *data = _data;

	//mutex_lock(&data->input_dev->mutex);

	if (focaltech_debug_addr_is_valid(data->addr))
		*val = data->addr;

	//mutex_unlock(&data->input_dev->mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_addr_fops, focaltech_debug_addr_get,
			focaltech_debug_addr_set, "0x%02llX\n");

static int focaltech_debug_suspend_set(void *_data, u64 val)
{
	struct focaltech_ts_data *data = _data;

	//mutex_lock(&data->input_dev->mutex);

	if (val)
		focaltech_ts_suspend(&data->client->dev);
	else
		focaltech_ts_resume(&data->client->dev);

	//mutex_unlock(&data->input_dev->mutex);

	return 0;
}

static int focaltech_debug_suspend_get(void *_data, u64 *val)
{
	struct focaltech_ts_data *data = _data;

	//mutex_lock(&data->input_dev->mutex);
	*val = data->suspended;
	//mutex_unlock(&data->input_dev->mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_suspend_fops, focaltech_debug_suspend_get,
			focaltech_debug_suspend_set, "%lld\n");

static int focaltech_debug_dump_info(struct seq_file *m, void *v)
{
	struct focaltech_ts_data *data = m->private;

	seq_printf(m, "%s\n", data->ts_info);

	return 0;
}

static int debugfs_dump_info_open(struct inode *inode, struct file *file)
{
	return single_open(file, focaltech_debug_dump_info, inode->i_private);
}

static const struct file_operations debug_dump_info_fops = {
	.owner		= THIS_MODULE,
	.open		= debugfs_dump_info_open,
	.read		= seq_read,
	.release	= single_release,
};


#ifdef CONFIG_OF
static int focaltech_get_dt_coords(struct device *dev, char *name,
				struct focaltech_ts_platform_data *pdata)
{
	u32 coords[FT_COORDS_ARR_SIZE];
	struct property *prop;
	struct device_node *np = dev->of_node;
	int coords_size, rc;

	prop = of_find_property(np, name, NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;

	coords_size = prop->length / sizeof(u32);
	if (coords_size != FT_COORDS_ARR_SIZE) {
		pr_err("focaltech_get_dt_coords,invalid %s\n",name);
		return -EINVAL;
	}

	rc = of_property_read_u32_array(np, name, coords, coords_size);
	if (rc && (rc != -EINVAL)) {
		pr_err("focaltech_get_dt_coords,Unable to read %s\n", name);
		return rc;
	}

	if (!strcmp(name, "focaltech,panel-coords")) {
		pdata->panel_minx = coords[0];
		pdata->panel_miny = coords[1];
		pdata->panel_maxx = coords[2];
		pdata->panel_maxy = coords[3];
	} else if (!strcmp(name, "focaltech,display-coords")) {
		pdata->x_min = coords[0];
		pdata->y_min = coords[1];
		pdata->x_max = coords[2];
		pdata->y_max = coords[3];
	} else {
		pr_err("focaltech_get_dt_coords,unsupported property %s\n", name);
		return -EINVAL;
	}

	LCD_WIDTH = pdata->panel_maxx;
	LCD_HEIGHT = pdata->panel_maxy;
	return 0;
}

static int focaltech_parse_dt(struct device *dev,struct focaltech_ts_platform_data *pdata)
{
	int rc = 0;
	struct device_node *np = dev->of_node;
	struct property *prop;
	u32 temp_val, num_buttons;	

	printk("%s begin\n", __func__);
	pdata->name = "focaltech";
	rc = of_property_read_string(np, "focaltech,name", &pdata->name);
	if (rc && (rc != -EINVAL)) {
		pr_err("focaltech_parse_dt,Unable to read name\n");
		return rc;
	}	rc = focaltech_get_dt_coords(dev, "focaltech,panel-coords", pdata);
	if (rc && (rc != -EINVAL)){
		return rc;
	}
	rc = focaltech_get_dt_coords(dev, "focaltech,display-coords", pdata);
	if (rc){
		return rc;
	}

	pdata->i2c_pull_up = of_property_read_bool(np,"focaltech,i2c-pull-up");

	pdata->no_force_update = of_property_read_bool(np,"focaltech,no-force-update");
	
	pdata->fw_auto_update = of_property_read_bool(np,"focaltech,fw-auto-update");
	
	
	/* reset, irq gpio info */
	pdata->reset_gpio = of_get_named_gpio_flags(np, "focaltech,reset-gpio",0, &pdata->reset_gpio_flags);
	if (pdata->reset_gpio < 0){
		pr_err("%s, can not get reset_gpio\n", __func__);
	}else{
		printk("%s, reset_gpio=%d\n", __func__, pdata->reset_gpio);
	}
	
	pdata->id1_gpio = of_get_named_gpio(np, "focaltech,id1-gpio",0);
	if (pdata->id1_gpio < 0){
		pr_err("%s, can not get id1_gpio\n", __func__);
	}else{
		printk("%s, id1_gpio=%d\n", __func__, pdata->id1_gpio);
	}
	pdata->id2_gpio = of_get_named_gpio(np, "focaltech,id2-gpio",0);
	if (pdata->id2_gpio < 0){
		pr_err("%s, can not get id2_gpio\n", __func__);
	}else{
		printk("%s, id2_gpio=%d\n", __func__, pdata->id2_gpio);
	}
	pdata->id3_gpio = of_get_named_gpio(np, "focaltech,id3-gpio",0);
	if (pdata->id3_gpio < 0){
		pr_err("%s, can not get id3_gpio\n", __func__);
	}else{
		printk("%s, id3_gpio=%d\n", __func__, pdata->id3_gpio);
	}
	
	pdata->irq_gpio = of_get_named_gpio_flags(np, "focaltech,irq-gpio",0, &pdata->irq_gpio_flags);
	if (pdata->irq_gpio < 0){
		pr_err("%s, can not get irq_gpio\n", __func__);
	}else{
		printk("%s, irq_gpio=%d\n", __func__, pdata->irq_gpio);
	}
		
	pdata->fw_name = "ft_fw.bin";
	rc = of_property_read_string(np, "focaltech,fw-name", &pdata->fw_name);
	if (rc && (rc != -EINVAL)) {
		pr_err("%s,Unable to read fw name\n", __func__);		
		return rc;
	}else{
		printk("%s, fw name=%s\n", __func__, pdata->fw_name);
	}

	rc = of_property_read_u32(np, "focaltech,group-id", &temp_val);
	if (rc && (rc != -EINVAL)) {	
		pr_err("%s,Unable to read group-id\n", __func__);	
		return rc;		
	}else{
		pdata->group_id = temp_val;
		printk("%s, group-id=%d\n", __func__, pdata->group_id);
	}

	rc = of_property_read_u32(np, "focaltech,hard-reset-delay-ms",&temp_val);
	if (rc && (rc != -EINVAL)) {		
		pr_err("%s,Unable to read hard-reset-delay-ms\n", __func__);	
		return rc;
	}else{
		pdata->hard_rst_dly = temp_val;
		printk("%s, hard_rst_dly=%d\n", __func__, pdata->hard_rst_dly);		
	}

	rc = of_property_read_u32(np, "focaltech,soft-reset-delay-ms",&temp_val);
	if (rc && (rc != -EINVAL)) {
		pr_err("%s,Unable to read soft-reset-delay-ms\n", __func__);	
		return rc;		
	}else{
		pdata->soft_rst_dly = temp_val;
		printk("%s, soft-reset-delay-ms=%d\n", __func__, pdata->soft_rst_dly);
	}
	
	rc = of_property_read_u32(np, "focaltech,num-max-touches", &temp_val);
	if (rc && (rc != -EINVAL)) {
		pr_err("%s,Unable to read num-max-touches\n", __func__);	
		return rc;
	}else{	
		pdata->num_max_touches = temp_val;
		printk("%s, snum-max-touches=%d\n", __func__, pdata->num_max_touches);
	}
		
	rc = of_property_read_u32(np, "focaltech,fw-delay-aa-ms", &temp_val);
	if (rc && (rc != -EINVAL)) {		
		pr_err("%s,Unable to read Unable to read fw delay aa\n", __func__);	
		return rc;
	} else if (rc != -EINVAL){
		pdata->info.delay_aa =  temp_val;
		printk("%s, focaltech,fw-delay-aa-ms=%d\n", __func__, pdata->info.delay_aa);
	}

	rc = of_property_read_u32(np, "focaltech,fw-delay-55-ms", &temp_val);
	if (rc && (rc != -EINVAL)) {		
		pr_err("%s,Unable to read Unable to read fw delay 55\n", __func__);	
		return rc;
	} else if (rc != -EINVAL){
		pdata->info.delay_55 =  temp_val;
		printk("%s, focaltech,fw-delay-55-ms=%d\n", __func__, pdata->info.delay_55);
	}

	rc = of_property_read_u32(np, "focaltech,fw-upgrade-id1", &temp_val);
	if (rc && (rc != -EINVAL)) {		
		pr_err("%s,Unable to read Unable to read fw upgrade id1\n", __func__);	
		return rc;
	} else if (rc != -EINVAL){
		pdata->info.upgrade_id_1 =  temp_val;
		printk("%s, focaltech,fw-upgrade-id1=%d\n", __func__, pdata->info.upgrade_id_1);
	}

	rc = of_property_read_u32(np, "focaltech,fw-upgrade-id2", &temp_val);
	if (rc && (rc != -EINVAL)) {
		pr_err("%s,Unable to read Unable to read fw upgrade id2\n", __func__);	
		return rc;
	} else if (rc != -EINVAL){
		pdata->info.upgrade_id_2 =  temp_val;
		printk("%s, focaltech,fw-upgrade-id2=%d\n", __func__, pdata->info.upgrade_id_2);
	}	

	rc = of_property_read_u32(np, "focaltech,fw-delay-readid-ms",&temp_val);
	if (rc && (rc != -EINVAL)) {		
		pr_err("%s,Unable to read fw-delay-readid-ms\n", __func__);	
		return rc;
	} else {
		pdata->info.delay_readid =  temp_val;
		printk("%s, fw-delay-readid-ms=%d\n", __func__, pdata->info.delay_readid);
	}

	rc = of_property_read_u32(np, "focaltech,fw-delay-era-flsh-ms",&temp_val);
	if (rc && (rc != -EINVAL)) {		
		pr_err("%s,Unable to read fw delay erase flash\n", __func__);	
		return rc;
	} else {
		pdata->info.delay_erase_flash =  temp_val;
		printk("%s, fw_delay_erase_flash=%d\n", __func__, pdata->info.delay_erase_flash);
	}
	pdata->info.auto_cal = of_property_read_bool(np,"focaltech,fw-auto-cal");

	pdata->fw_vkey_support = of_property_read_bool(np,"focaltech,fw-vkey-support");

	pdata->ignore_id_check = of_property_read_bool(np,"focaltech,ignore-id-check");

	rc = of_property_read_u32(np, "focaltech,family-id", &temp_val);
	if (rc && (rc != -EINVAL)) {
		pr_err("%s,Unable to read fw delay erase flash\n", __func__);	
		return rc;
	}else{
		pdata->family_id = temp_val;
		printk("%s, fw_delay_erase_flash=%d\n", __func__, pdata->family_id);
	}

	prop = of_find_property(np, "focaltech,button-map", NULL);
	if (prop) {
		num_buttons = prop->length / sizeof(temp_val);
		if (num_buttons <= MAX_BUTTONS){			
			rc = of_property_read_u32_array(np,"focaltech,button-map", button_map,num_buttons);
			if (rc) {				
				pr_err("%s,Unable to read key codes\n", __func__);	
				return rc;
			}
		}
	}
	
	/***********power regulator_get****************/
	pdata->vdd = regulator_get(dev, "vdd");
	if (IS_ERR(pdata->vdd)) {
			rc = PTR_ERR(pdata->vdd);
			dev_err(dev,"Regulator get failed vdd rc=%d\n", rc);
			return rc;
	}	
	
	
	pdata->vcc_i2c = regulator_get(dev, "vcc_i2c");
	if (IS_ERR(pdata->vcc_i2c)) {
			rc = PTR_ERR(pdata->vcc_i2c);
			dev_err(dev,"Regulator get failed vcc_i2c rc=%d\n", rc);
			return rc;
	}
	
	if (gpio_is_valid(pdata->irq_gpio)) {
		rc= gpio_request(pdata->irq_gpio, "focaltech_irq_gpio");
		if (rc) {
			dev_err(dev, "irq gpio request failed");
			pr_err("irq gpio request failed");
			return rc;
		}
/*		rc = gpio_direction_input(pdata->irq_gpio);
		if (rc) {
			dev_err(dev,"set_direction for irq gpio failed\n");
			pr_err("irq gpio request failed");
			return rc;
		}
*/
	}
	
	if (gpio_is_valid(pdata->reset_gpio)) {
		rc = gpio_request(pdata->reset_gpio, "focaltech_reset_gpio");
		if (rc) {
			dev_err(dev, "reset gpio request failed");			
			return rc;
		}
		rc = gpio_direction_output(pdata->reset_gpio, 0);
		if (rc) {
			dev_err(dev,"set_direction for reset gpio failed\n");
			return rc;
		}
		msleep(pdata->hard_rst_dly);
//		gpio_set_value_cansleep(pdata->reset_gpio, 1)
	}

	
	printk("%s done\n", __func__);
	return rc;
}
#else
static int focaltech_parse_dt(struct device *dev,
			struct focaltech_ts_platform_data *pdata)
{
	return -ENODEV;
}
#endif

static void get_tp_id(struct focaltech_ts_data *data)
{
	int ret,id1 = -1,id2 = -1,id3 = -1;
	if(data->pdata->id1_gpio  >= 0) {
		ret = gpio_request(data->pdata->id1_gpio,"TP_ID1");		
	}
	if(data->pdata->id2_gpio  >=  0) {
		ret = gpio_request(data->pdata->id2_gpio,"TP_ID2");		
	}	
	if(data->pdata->id3_gpio >= 0) {
		ret = gpio_request(data->pdata->id3_gpio,"TP_ID3");		
	}
//	msleep(80);
	if(data->pdata->id1_gpio >= 0) {
	    id1=gpio_get_value(data->pdata->id1_gpio);	
	}
	if(data->pdata->id2_gpio  >=  0) {
		id2=gpio_get_value(data->pdata->id2_gpio);		
	}	
	if(data->pdata->id3_gpio  >= 0) {
		id3=gpio_get_value(data->pdata->id3_gpio);
	}
	
	pr_err("[focaltech]:id1 = %d,id3 = %d,id3 = %d\n",id1,id2,id3);
    if((id1==1)&&(id2==0)&&(id3==0)) {
		pr_err("[focaltech%s::OFILM\n",__func__);
		tp_dev=TP_OFILM;
		CTPM_FW = CTPM_FW_Ofilm;
		FW_len = sizeof(CTPM_FW_Ofilm);
		data->tp_info.manufacture = "OFILM";
	}else if(((id1==1)&&(id2==1)&&(id3==0))|| ((id1==0)&&(id2==0)&&(id3==0))) {
		pr_err("[focaltech]%s::TRULY \n",__func__);
		tp_dev=TP_TRULY;
		CTPM_FW = CTPM_FW_Truly;
		FW_len = sizeof(CTPM_FW_Truly);	
		data->tp_info.manufacture = "TRULY";
	}else{		
		pr_err("[focaltech]%s::TP_UNKNOWN\n",__func__);
		tp_dev=TP_TRULY;
		CTPM_FW = CTPM_FW_Truly;
		FW_len = sizeof(CTPM_FW_Truly);	
		data->tp_info.manufacture = "TP_UNKNOWN";
	}
	
}


int focaltech_erase_pram(struct i2c_client * client)
{
	u8 reg_val[4] = {0};
	u32 i = 0;	
	u8 auc_i2c_write_buf[10];	
	int i_ret;
	u8 state;
	
	fts_get_upgrade_info(&upgradeinfo);	
	for (i = 0; i < FTS_UPGRADE_LOOP; i++) 
	{
		/*********Step 1:Reset  CTPM *****/
		fts_ctpm_hw_reset();
		
		usleep_range(6000,6000);
		
		/*********Step 2:Enter upgrade mode *****/
		auc_i2c_write_buf[0] = FT_UPGRADE_55;
		i_ret = focaltech_i2c_Write(client, auc_i2c_write_buf, 1);
		if(i_ret < 0)
		{
			pr_err("[FTS] failed writing  0x55 ! \n");
			continue;
		}
		

		/*********Step 3:check READ-ID***********************/
		usleep_range(1000,1000);
		auc_i2c_write_buf[0] = 0x90;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] =
			0x00;
		reg_val[0] = reg_val[1] = 0x00;
		
		focaltech_i2c_Read(client, auc_i2c_write_buf, 4, reg_val, 2);
		if (reg_val[0] == 0x54&& reg_val[1] == 0x22) 
		{
			focaltech_read_reg(client, 0xd0, &state);
			if(state == 0)
			{
				pr_err("[FTS] Step 3: READ State fail \n");
				continue;
			}
			printk("[FTS] focaltech_erase_pram: i_ret = %d \n", i_ret);			
			pr_err("[FTS] focaltech_erase_pram: READ CTPM ID OK,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0], reg_val[1]);			
			break;
		} 
		else 
		{
			dev_err(&client->dev, "[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0], reg_val[1]);
			continue;
		}
	}
	
	//AE 00 00 00 00 01 00 // 00地址的内容清00
	auc_i2c_write_buf[0] = 0xAE;
	auc_i2c_write_buf[1] = 0x00;
	auc_i2c_write_buf[2] = 0x00;
	auc_i2c_write_buf[3] = 0x00;
	auc_i2c_write_buf[4] = 0x00;
	auc_i2c_write_buf[5] = 0x01;
	auc_i2c_write_buf[6] = 0x00;
	i_ret = focaltech_i2c_Write(client, auc_i2c_write_buf, 7);
	if(i_ret < 0)
	{
		pr_err("[FTS] failed writing  AE 00 00 00 00 01 00 ! \n");		
	}
	msleep(10);
	
	if (i >= FTS_UPGRADE_LOOP )
		return -EIO;
		
	return 0;

}

extern void (*focaltech_chg_mode_enable)(bool);
static int focaltech_ts_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	struct focaltech_ts_platform_data *pdata;
	struct focaltech_ts_data *data;
	
	struct dentry *temp;
	u8 reg_value;
	u8 reg_addr;
	int err, len,i;	

	int boot_mode = 0;	
	
	boot_mode =get_boot_mode();

	pr_err(" focaltech_ts_probe\n");
	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,sizeof(struct focaltech_ts_platform_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			pr_err(" ft5x46 Failed to allocate memory\\n");
			return -ENOMEM;
		}
		err = focaltech_parse_dt(&client->dev, pdata);
		if (err) {
			dev_err(&client->dev, "DT parsing failed\n");
			pr_err(" ft5x46 DT parsing failed\n");
			return err;
		}
	}else{
		pdata = client->dev.platform_data;
	}
	if (!pdata) {
		dev_err(&client->dev, "Invalid pdata\n");
		pr_err("focaltech dev.platform_data");
		return -EINVAL;
	}
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "I2C not supported\n");
		pr_err("ft5x46 I2C not supported");
		return -ENODEV;
	}

	data = devm_kzalloc(&client->dev,sizeof(struct focaltech_ts_data), GFP_KERNEL);
	if (!data) {
		dev_err(&client->dev, "Not enough memory\n");
		pr_err("ft5x46 Not enough memory");
		return -ENOMEM;
	}
	pr_err(" ft5x46  read fw_name\n");
	if (pdata->fw_name) {
		len = strlen(pdata->fw_name);
		if (len > FT_FW_NAME_MAX_LEN - 1) {
			dev_err(&client->dev, "Invalid firmware name\n");
			pr_err(" ft5x46  Invalid firmware name\n");
			return -EINVAL;
		}
		strlcpy(data->fw_name, pdata->fw_name, len + 1);
	}

	data->tch_data_len = FT_TCH_LEN(pdata->num_max_touches);
	data->tch_data = devm_kzalloc(&client->dev,data->tch_data_len, GFP_KERNEL);
	if (!data) {
		dev_err(&client->dev, "Not enough memory\n");
		pr_err("ft5x46 Not enough memory");
		return -ENOMEM;
	}
	
	data->input_dev = input_allocate_device();
	if (!data->input_dev) {
		dev_err(&client->dev, "failed to allocate input device\n");
		//TPD_ERR("ft5x46 failed to allocate input device");
		pr_err(" ft5x46  failed to allocate input device\n");
		return -ENOMEM;
	}	
	data->client = client;
	data->pdata = pdata;
	data->family_id = pdata->family_id;
	data->input_dev->name = TPD_DEVICE;
	data->input_dev->id.bustype = BUS_I2C;
	data->input_dev->dev.parent = &client->dev;
	
	get_tp_id(data);
	data_g = data;
	
	gesture_enable = false;	
	

	input_set_drvdata(data->input_dev, data);
	i2c_set_clientdata(client, data);

	__set_bit(EV_KEY, data->input_dev->evbit);
	__set_bit(EV_ABS, data->input_dev->evbit);
	__set_bit(BTN_TOUCH, data->input_dev->keybit);
	__set_bit(INPUT_PROP_DIRECT, data->input_dev->propbit);

#ifdef SUPPORT_GESTURE	
	
	set_bit(KEY_F4 , data->input_dev->keybit);
#endif
	
	input_mt_init_slots(data->input_dev, pdata->num_max_touches, 0);
	input_set_abs_params(data->input_dev, ABS_MT_POSITION_X, pdata->x_min, pdata->x_max, 0, 0);
	input_set_abs_params(data->input_dev, ABS_MT_POSITION_Y, pdata->y_min, pdata->y_max, 0, 0);

	err = input_register_device(data->input_dev);
	if (err) {
		dev_err(&client->dev, "Input device registration failed\n");
		pr_err("ft5x46 input_register_device");
		goto free_inputdev;
	}
	
	init_proc_touchpanel_file_node();	
	
	speedup_resume_wq = create_singlethread_workqueue("speedup_resume_wq");
	if( !speedup_resume_wq ){
        err = -ENOMEM;
		pr_err("reate_singlethread_workqueue:speedup_resume_wq fail!\n");
		goto free_inputdev;	
    } 	
	INIT_WORK(&data->speed_up_work,speedup_focaltech_resume);
	INIT_WORK(&data->suspend_wk,speedup_focaltech_suspend);
	
	err = focaltech_power_on(data, true);
	if (err) {
		dev_err(&client->dev, "power on failed");
		pr_err("ft5x46 power on failed");
		goto pwr_deinit;
	}
	
	if( (boot_mode == MSM_BOOT_MODE__FACTORY || boot_mode == MSM_BOOT_MODE__RF || boot_mode == MSM_BOOT_MODE__WLAN) ){
		pr_err("focaltech regulator_disable is called,boot_mode = %d\n",boot_mode);
		if (gpio_is_valid(data->pdata->reset_gpio)) {
			pr_err("focaltech:enable the reset_gpio\n");
			gpio_direction_output(data->pdata->reset_gpio, 0);
			msleep(5);
		}	
		if (regulator_count_voltages(data->pdata->vdd) > 0) {
			err = regulator_disable(data->pdata->vdd);
			if (err) {
				pr_err("focaltech vdd regulator_count_voltages failed,errcode = %d\n",err);
			}
		}
		return 0;
	}
	
	reg_addr = FT_REG_ID;	
	err = focaltech_i2c_Read(client, &reg_addr, 1, &reg_value, 1);
	if (err < 0) {
		dev_err(&client->dev, "focaltech version read failed,focaltech_i2c_Read error = %d\n ",err);		
	}
	if(reg_value != 0x54){
		dev_err(&client->dev, "focaltech device id read failed,device id = 0x%x,erase_pram and reset tp\n",reg_value);
		focaltech_erase_pram(client);	
		if (gpio_is_valid(data->pdata->reset_gpio)) {
			gpio_set_value_cansleep(data->pdata->reset_gpio, 0);
			msleep(data->pdata->hard_rst_dly);
			gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
		}		
		msleep(50);		
		for(i = 0; i < 10; i++){
			focaltech_i2c_Read(data->client, &reg_addr, 1, &reg_value, 1);
			if(reg_value == 0x54){
				break;
			}
			msleep(20);
		}
	}	
	err = request_threaded_irq(data->client->irq, NULL,
				focaltech_ts_interrupt,
				IRQF_ONESHOT | IRQF_TRIGGER_FALLING,
				data->client->dev.driver->name, data);
	if (err) {
		dev_err(&client->dev, "request irq failed\n");
		goto free_reset_gpio;
	}
	focaltech_tpd_button_init(data);	
	
	err = focaltech_fw_check(data);
	if(err < 0)	{
		tp_fw_ok = 0;
		pr_err("fw will be update!");
	}else{
		tp_fw_ok = 1;		
	}

	
	/*get some register information */
	reg_addr = FT_REG_POINT_RATE;
	err = focaltech_i2c_Read(client, &reg_addr, 1, &reg_value, 1);
	if (err < 0){
		dev_err(&client->dev, "report rate read failed");
	}
	pr_err("focaltech report rate = %dHz\n", reg_value * 10);
	dev_info(&client->dev, "report rate = %dHz\n", reg_value * 10);

	reg_addr = FT_REG_THGROUP;
	err = focaltech_i2c_Read(client, &reg_addr, 1, &reg_value, 1);
	if (err < 0)
		dev_err(&client->dev, "threshold read failed");

	dev_info(&client->dev, "touch threshold = %d\n", reg_value * 4);

	data->ts_info = devm_kzalloc(&client->dev,FT_INFO_MAX_LEN, GFP_KERNEL);
	if (!data->ts_info) {
		dev_err(&client->dev, "Not enough memory\n");
		pr_err("focaltech Not enough memory\n");
		goto free_debug_dir;
	}	
	FT_STORE_TS_INFO(data->ts_info, data->family_id, data->pdata->name,
			data->pdata->num_max_touches, data->pdata->group_id,
			data->pdata->fw_vkey_support ? "yes" : "no",
			data->pdata->fw_name, data->fw_ver[0],
			data->fw_ver[1], data->fw_ver[2]);
	
	
	sprintf(data->fw_version, "150050%x",data->fw_ver[0] );		
	data->tp_info.version =   data->fw_version;	
	register_device_proc("tp", data->tp_info.version, data->tp_info.manufacture);
#if defined(CONFIG_FB)
	data->fb_notif.notifier_call = fb_notifier_callback;

	err = fb_register_client(&data->fb_notif);
	if (err)
		dev_err(&client->dev, "Unable to register fb_notifier: %d\n",err);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN +
						    FT_SUSPEND_LEVEL;
	data->early_suspend.suspend = focaltech_ts_early_suspend;
	data->early_suspend.resume = focaltech_ts_late_resume;
	register_early_suspend(&data->early_suspend);
#endif

#ifdef FTS_APK_DEBUG
		fts_create_apk_debug_channel(client);
#endif

#if defined(FTS_SCAP_TEST)
	mutex_init(&data->selftest_lock);	
#endif
	pr_err("driver_create_file:dev_attr_oppo_tp_fw_update\n");
	if (device_create_file(&client->dev, &dev_attr_oppo_tp_fw_update)) {            
		pr_err("driver_create_file failt\n");
		goto free_update_fw_sys;
	}  
	
	if (device_create_file(&client->dev, &dev_attr_oppo_tp_fw_download)) {            
		pr_err("driver_create_file failt\n");
		goto free_update_fw_sys;
	}
	
#ifdef FTS_APK_DEBUG
	if (device_create_file(&client->dev, &dev_attr_ftstpfwver)) 
	{            
		pr_err("driver_create_file  ftstpfwver failt\n");
	}

	if (device_create_file(&client->dev, &dev_attr_ftstprwreg)) 
	{            
		pr_err("driver_create_file  ftstprwreg failt\n");
	}

	if (device_create_file(&client->dev, &dev_attr_ftsfwupgradeapp)) 
	{            
		pr_err("driver_create_file  ftsfwupgradeapp failt\n");
	}
#endif	

	data->dir = debugfs_create_dir(FT_DEBUG_DIR_NAME, NULL);
	if (data->dir == NULL || IS_ERR(data->dir)) {
		pr_err("debugfs_create_dir failed(%ld)\n", PTR_ERR(data->dir));
		err = PTR_ERR(data->dir);
		goto free_force_update_fw_sys;
	}

	temp = debugfs_create_file("addr", S_IRUSR | S_IWUSR, data->dir, data,
				   &debug_addr_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}	
	temp = debugfs_create_file("data", S_IRUSR | S_IWUSR, data->dir, data,
				   &debug_data_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}

	temp = debugfs_create_file("suspend", S_IRUSR | S_IWUSR, data->dir,
					data, &debug_suspend_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}

	temp = debugfs_create_file("dump_info", S_IRUSR | S_IWUSR, data->dir,
					data, &debug_dump_info_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}
	focaltech_chg_mode_enable = focaltech_chg_mode_enble;
	tp_probe_ok = 1;
	printk("%s done\n", __func__);
	return 0;

free_debug_dir:
	debugfs_remove_recursive(data->dir);
free_force_update_fw_sys:
free_update_fw_sys:
	free_irq(client->irq, data);
free_reset_gpio:
	if (gpio_is_valid(pdata->reset_gpio))
		gpio_free(pdata->reset_gpio);		
pwr_deinit:
	input_unregister_device(data->input_dev);
	data->input_dev = NULL;
free_inputdev:
	input_free_device(data->input_dev);
	printk("%s error, err=%d\n", __func__, err);
	return err;
}

static int focaltech_ts_remove(struct i2c_client *client)
{
	struct focaltech_ts_data *data = i2c_get_clientdata(client);

#ifdef FTS_APK_DEBUG
		fts_release_apk_debug_channel();
#endif

#if defined(FTS_SCAP_TEST)
	mutex_destroy(&data->selftest_lock);
#endif
	debugfs_remove_recursive(data->dir);
	//device_remove_file(&client->dev, &dev_attr_force_update_fw);
	//device_remove_file(&client->dev, &dev_attr_update_fw);
	//device_remove_file(&client->dev, &dev_attr_fw_name);

#if defined(CONFIG_FB)
	if (fb_unregister_client(&data->fb_notif)){
		dev_err(&client->dev, "Error occurred while unregistering fb_notifier.\n");
	}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&data->early_suspend);
#endif
	free_irq(client->irq, data);

	if (gpio_is_valid(data->pdata->reset_gpio)){
		gpio_free(data->pdata->reset_gpio);
	}
	if (gpio_is_valid(data->pdata->irq_gpio)){
		gpio_free(data->pdata->irq_gpio);
	}	
	
	focaltech_power_on(data, false);
	
	input_unregister_device(data->input_dev);

	return 0;
}

static const struct i2c_device_id focaltech_ts_id[] = {
	{"focaltech_ts", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, focaltech_ts_id);

#ifdef CONFIG_OF
static struct of_device_id focaltech_match_table[] = {
	{ .compatible = "focaltech,5x06",},
	{ },
};
#else
#define focaltech_match_table NULL
#endif

static struct i2c_driver focaltech_ts_driver = {
	.probe = focaltech_ts_probe,
	.remove = focaltech_ts_remove,
	.driver = {
		   .name = "focaltech_ts",
		   .owner = THIS_MODULE,
		.of_match_table = focaltech_match_table,
#ifdef CONFIG_PM
		   .pm = &focaltech_ts_pm_ops,
#endif
		   },
	.id_table = focaltech_ts_id,
};

static int __init focaltech_ts_init(void)
{
	return i2c_add_driver(&focaltech_ts_driver);
}
module_init(focaltech_ts_init);

static void __exit focaltech_ts_exit(void)
{
	i2c_del_driver(&focaltech_ts_driver);
}
module_exit(focaltech_ts_exit);

MODULE_DESCRIPTION("FocalTech focaltech TouchScreen driver");
MODULE_LICENSE("GPL v2");

