/* Copyright (c) 2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/module.h>
#include <linux/export.h>
#include <mach/gpiomux.h>
#include "msm_camera_io_util.h"
#include "msm_led_flash.h"
#include <linux/proc_fs.h>

#include "../msm_sensor.h"
#include "../cci/msm_cci.h"


#include <mach/oppo_project.h> 
static struct mutex flash_mode_lock;

#define NEW_FLASH_NAME "skywork,sky81294"

#define CONFIG_MSMB_CAMERA_DEBUG

#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define sky81294_DBG(fmt, args...) pr_err(fmt, ##args)
#else
#define sky81294_DBG(fmt, args...)
#endif

static struct delayed_work led_blink_work;
static bool blink_test_status;

extern int32_t IS_FLASH_ON ;
extern bool camera_power_status;

static struct msm_led_flash_ctrl_t fctrl;

extern int32_t msm_led_get_dt_data(struct device_node *of_node,
		struct msm_led_flash_ctrl_t *fctrl);
extern int msm_flash_pinctrl_init(struct msm_led_flash_ctrl_t *ctrl);

static struct msm_camera_i2c_reg_array sky81294_init_array[] = {
	{0x03, 0x08},
};

static struct msm_camera_i2c_reg_array sky81294_off_array[] = {
	{0x03, 0x08},
};

static struct msm_camera_i2c_reg_array sky81294_release_array[] = {
	{0x03, 0x08},
};

static struct msm_camera_i2c_reg_array sky81294_low_array[] = {
	{0x02, 0x03},
	{0x03, 0x09},
};

static struct msm_camera_i2c_reg_array sky81294_high_array[] = {
//	{0x00, 0x0F},
	{0x03, 0x0A},
};


static const struct of_device_id sky81294_i2c_trigger_dt_match[] = {
	{.compatible = "skywork,sky81294"},
	{}
};

MODULE_DEVICE_TABLE(of, sky81294_i2c_trigger_dt_match);
static const struct i2c_device_id sky81294_i2c_id[] = {
	{NEW_FLASH_NAME, (kernel_ulong_t)&fctrl},
	{ }
};

int msm_flash_sky81294_led_init(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	sky81294_DBG("%s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
        fctrl->led_state = MSM_CAMERA_LED_RELEASE;
	if (power_info->gpio_conf->cam_gpiomux_conf_tbl != NULL) {
		pr_err("%s:%d mux install\n", __func__, __LINE__);
		msm_gpiomux_install(
			(struct msm_gpiomux_config *)
			power_info->gpio_conf->cam_gpiomux_conf_tbl,
			power_info->gpio_conf->cam_gpiomux_conf_tbl_size);
	}

	/* CCI Init */
	if (fctrl->flash_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_util(
			fctrl->flash_i2c_client, MSM_CCI_INIT);
		if (rc < 0) {
			pr_err("cci_init failed\n");
			return rc;
		}
	}
#if 0
	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 1);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
	}
	if (fctrl->pinctrl_info.use_pinctrl == true) {
		sky81294_DBG("%s:%d PC:: flash pins setting to active state",
				__func__, __LINE__);
		rc = pinctrl_select_state(fctrl->pinctrl_info.pinctrl,
				fctrl->pinctrl_info.gpio_state_active);
		if (rc)
			pr_err("%s:%d cannot set pin to active state",
					__func__, __LINE__);
	}
#endif
	msleep(20);

	sky81294_DBG("before FL_RESET\n");
	if (power_info->gpio_conf->gpio_num_info->
			valid[SENSOR_GPIO_FL_RESET] == 1)
		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_RESET],
			GPIO_OUT_HIGH);
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_LOW);

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_LOW);

        fctrl->led_state = MSM_CAMERA_LED_INIT;

	return rc;
}

int msm_flash_sky81294_led_release(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	sky81294_DBG("%s:%d called\n", __func__, __LINE__);
	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}

	if (fctrl->led_state != MSM_CAMERA_LED_INIT) {
		pr_err("%s:%d invalid led state\n", __func__, __LINE__);
		return -EINVAL;
	}
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_LOW);
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_LOW);
	if (power_info->gpio_conf->gpio_num_info->
			valid[SENSOR_GPIO_FL_RESET] == 1)
		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_RESET],
			GPIO_OUT_LOW);
#if 0
	if (fctrl->pinctrl_info.use_pinctrl == true) {
		rc = pinctrl_select_state(fctrl->pinctrl_info.pinctrl,
				fctrl->pinctrl_info.gpio_state_suspend);
		if (rc)
			pr_err("%s:%d cannot set pin to suspend state",
				__func__, __LINE__);
	}
	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 0);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		return rc;
	}
#endif
    fctrl->led_state = MSM_CAMERA_LED_RELEASE;
	
	/* CCI deInit */
	if (fctrl->flash_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_util(
			fctrl->flash_i2c_client, MSM_CCI_RELEASE);
		if (rc < 0)
			pr_err("cci_deinit failed\n");
	}
	return 0;
}

int msm_flash_sky81294_led_off(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	sky81294_DBG("%s:%d called\n", __func__, __LINE__);

	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->off_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_LOW);

	return rc;
}

int msm_flash_sky81294_led_low(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;

	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	sky81294_DBG("%s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->low_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);

	}

	return rc;
}

int msm_flash_sky81294_led_high(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	sky81294_DBG("%s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;

	power_info = &flashdata->power_info;

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->high_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	return rc;
}

#ifdef VENDOR_EDIT
/*OPPO 2014-08-01 hufeng add for flash engineer mode test*/
static struct regulator *vreg;
static int led_test_mode;


static int sky81294_flash_led_low(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	sky81294_DBG("%s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_LOW);

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_LOW);

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->low_setting); 
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	return rc;
}

static int sky81294_flash_led_high(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	sky81294_DBG("%s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_LOW);

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_LOW);

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->high_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	return rc;
}


static int msm_led_cci_test_init(void)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	//struct regulator *vreg;
	sky81294_DBG("%s:%d called\n", __func__, __LINE__);
	flashdata = fctrl.flashdata;
	power_info = &flashdata->power_info;

#ifdef VENDOR_EDIT
/* xianglie.liu 2014-09-05 Add for lm3642 14043 and 14045 use gpio-vio */
/* zhengrong.zhang 2014-11-08 Add for gpio contrl lm3642 */
	if (camera_power_status) {
		sky81294_DBG("%s:%d camera already power up\n", __func__, __LINE__);
		return rc;
	}
	msm_flash_led_init(&fctrl);
	if(is_project(OPPO_14043)||is_project(OPPO_14045)||is_project(OPPO_14037))
	{
		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_VIO],
			GPIO_OUT_HIGH);
	}
	else
	{
		msm_camera_config_single_vreg(&fctrl.pdev->dev,
			power_info->cam_vreg,
			&vreg,1);	
	}
#endif
	sky81294_DBG("%s exit\n", __func__);
	return rc;
}
static int msm_led_cci_test_off(void)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	flashdata = fctrl.flashdata;
	power_info = &flashdata->power_info;
	sky81294_DBG("%s:%d called\n", __func__, __LINE__);
	if (led_test_mode == 2)
		cancel_delayed_work_sync(&led_blink_work);
	if (fctrl.flash_i2c_client && fctrl.reg_setting) 
	{
		rc = fctrl.flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl.flash_i2c_client,
			fctrl.reg_setting->off_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_LOW);

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_LOW);

	if (camera_power_status) {
		sky81294_DBG("%s:%d camera already power up\n", __func__, __LINE__);
		return rc;
	}
	if(is_project(OPPO_14043)||is_project(OPPO_14045)|| is_project(OPPO_14037))
	{
		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_VIO],
			GPIO_OUT_LOW);
	}
	else
	{
		msm_camera_config_single_vreg(&fctrl.pdev->dev,
			power_info->cam_vreg,
			&vreg,0);
	}
	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 0);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
	}

	IS_FLASH_ON = 0;

	/* CCI deInit */
	if (fctrl.flash_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = fctrl.flash_i2c_client->i2c_func_tbl->i2c_util(
			fctrl.flash_i2c_client, 1);
		if (rc < 0) {
			pr_err("%s cci_init failed\n", __func__);
		}
	}
	return rc;
}
static void msm_led_cci_test_blink_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	if (blink_test_status)
	{
		sky81294_flash_led_low(&fctrl);
	}
	else
	{
		msm_flash_led_off(&fctrl);
	}
	blink_test_status = !blink_test_status;
	schedule_delayed_work(dwork, msecs_to_jiffies(1100));
}

static ssize_t flash_proc_read(struct file *filp, char __user *buff,
                        	size_t len, loff_t *data)
{
    char value[2] = {0};

    snprintf(value, sizeof(value), "%d", led_test_mode);
    return simple_read_from_buffer(buff, len, data, value,1);
}

static ssize_t flash_proc_write(struct file *filp, const char __user *buff,
                        	size_t len, loff_t *data)
{
	char buf[8] = {0};
	int new_mode = 0;
	if (len > 8)
		len = 8;
	if (copy_from_user(buf, buff, len)) 
	{
		pr_err("proc write error.\n");
		return -EFAULT;
	}
	new_mode = simple_strtoul(buf, NULL, 10);
	if (new_mode == led_test_mode) 
	{
		pr_err("the same mode as old\n");
		return len;
	}
	switch (new_mode) {
	case 0:
		mutex_lock(&flash_mode_lock);
		msm_led_cci_test_off();
		led_test_mode = 0;
		mutex_unlock(&flash_mode_lock);
		break;
	case 1:
		mutex_lock(&flash_mode_lock);
		msm_led_cci_test_init();
		led_test_mode = 1;
		mutex_unlock(&flash_mode_lock);
		sky81294_flash_led_low(&fctrl);

		break;
	case 2:
		mutex_lock(&flash_mode_lock);
		msm_led_cci_test_init();
		led_test_mode = 2;
		mutex_unlock(&flash_mode_lock);
		schedule_delayed_work(&led_blink_work, msecs_to_jiffies(50));
		break;
	case 3:
		mutex_lock(&flash_mode_lock);
		msm_led_cci_test_init();
		led_test_mode = 3;
		mutex_unlock(&flash_mode_lock);
		sky81294_flash_led_high(&fctrl);
		break;
	default:
#ifdef VENDOR_EDIT
/*OPPO hufeng 2014-11-22 add for individual flashlight*/
		mutex_lock(&flash_mode_lock);
		led_test_mode = new_mode;
		mutex_unlock(&flash_mode_lock);
#endif
		pr_err("invalid mode %d\n", led_test_mode);
		break;
	}
	return len;
}
static const struct file_operations led_test_fops = {
    .owner		= THIS_MODULE,
    .read		= flash_proc_read,
    .write		= flash_proc_write,
};
static int flash_proc_init(struct msm_led_flash_ctrl_t *flash_ctl)
{
	int ret=0;
	struct proc_dir_entry *proc_entry;

	INIT_DELAYED_WORK(&led_blink_work, msm_led_cci_test_blink_work);
	proc_entry = proc_create_data( "qcom_flash", 0666, NULL,&led_test_fops, (void*)&fctrl);
	if (proc_entry == NULL)
	{
		ret = -ENOMEM;
	  	pr_err("[%s]: Error! Couldn't create qcom_flash proc entry\n", __func__);
	}
	return ret;
}
#endif

static struct msm_camera_i2c_fn_t msm_sensor_cci_func_tbl = {
	.i2c_read = msm_camera_cci_i2c_read,
	.i2c_read_seq = msm_camera_cci_i2c_read_seq,
	.i2c_write = msm_camera_cci_i2c_write,
	.i2c_write_table = msm_camera_cci_i2c_write_table,
	.i2c_write_seq_table = msm_camera_cci_i2c_write_seq_table,
	.i2c_write_table_w_microdelay =
		msm_camera_cci_i2c_write_table_w_microdelay,
	.i2c_util = msm_sensor_cci_i2c_util,
	.i2c_write_conf_tbl = msm_camera_cci_i2c_write_conf_tbl,
};

extern void set_flashlight_id(uint16_t);

static int sky81294_id_match(struct platform_device *pdev,
	const void *data)
{
	int rc = 0;
	int have_read_id_flag = 0;

	struct msm_led_flash_ctrl_t *fctrl =
		(struct msm_led_flash_ctrl_t *)data;
	struct msm_camera_i2c_client *flash_i2c_client= NULL;
	struct msm_camera_slave_info *slave_info;
	const char *sensor_name;
	uint16_t chipid = 0;
	struct device_node *of_node = pdev->dev.of_node;
	struct msm_camera_cci_client *cci_client = NULL;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	if (!of_node) {
		pr_err("of_node NULL\n");
		goto probe_failure;
	}
	fctrl->pdev = pdev;

	rc = msm_led_get_dt_data(pdev->dev.of_node, fctrl);
	if (rc < 0) {
		pr_err("%s failed line %d rc = %d\n", __func__, __LINE__, rc);
		return rc;
	}

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;

    msm_flash_pinctrl_init(fctrl);

	/* Assign name for sub device */
	snprintf(fctrl->msm_sd.sd.name, sizeof(fctrl->msm_sd.sd.name),
			"%s", fctrl->flashdata->sensor_name);

	/* Set device type as Platform*/
	fctrl->flash_device_type = MSM_CAMERA_PLATFORM_DEVICE;

	if (NULL == fctrl->flash_i2c_client) {
		pr_err("%s flash_i2c_client NULL\n",
			__func__);
		rc = -EFAULT;
	}

	fctrl->flash_i2c_client->cci_client = kzalloc(sizeof(
		struct msm_camera_cci_client), GFP_KERNEL);
	if (!fctrl->flash_i2c_client->cci_client) {
		pr_err("%s failed line %d kzalloc failed\n",
			__func__, __LINE__);
		return rc;
	}

	cci_client = fctrl->flash_i2c_client->cci_client;
	cci_client->cci_subdev = msm_cci_get_subdev();
	cci_client->cci_i2c_master = fctrl->cci_i2c_master;
	cci_client->i2c_freq_mode = I2C_FAST_MODE;

	if (fctrl->flashdata->slave_info->sensor_slave_addr)
		cci_client->sid =
			fctrl->flashdata->slave_info->sensor_slave_addr >> 1;
	cci_client->retries = 3;
	cci_client->id_map = 0;

	if (!fctrl->flash_i2c_client->i2c_func_tbl)
		fctrl->flash_i2c_client->i2c_func_tbl =
			&msm_sensor_cci_func_tbl;

	flash_i2c_client = fctrl->flash_i2c_client;
	slave_info = fctrl->flashdata->slave_info;
	sensor_name = fctrl->flashdata->sensor_name;

	if (!flash_i2c_client || !slave_info || !sensor_name) {
		pr_err("%s:%d failed: %p %p %p\n",
			__func__, __LINE__, flash_i2c_client, slave_info,
			sensor_name);
		return -EINVAL;
	}

	if (power_info->gpio_conf->cam_gpiomux_conf_tbl != NULL) {
		pr_err("%s:%d mux install\n", __func__, __LINE__);
		msm_gpiomux_install(
			(struct msm_gpiomux_config *)
			power_info->gpio_conf->cam_gpiomux_conf_tbl,
			power_info->gpio_conf->cam_gpiomux_conf_tbl_size);
	}

	/* CCI Init */
	if (fctrl->flash_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_util(
			fctrl->flash_i2c_client, MSM_CCI_INIT);
		if (rc < 0) {
			pr_err("cci_init failed\n");
			return rc;
		}
	}
	msleep(2);
	if(is_project(OPPO_14043)||is_project(OPPO_14045)|| is_project(OPPO_14037))
	{
		rc = msm_camera_request_gpio_table(
			power_info->gpio_conf->cam_gpio_req_tbl,
			power_info->gpio_conf->cam_gpio_req_tbl_size, 1);
		if (rc < 0) {
			pr_err("%s: request gpio failed\n", __func__);
		}
		msleep(3);
    	gpio_set_value_cansleep(
    		power_info->gpio_conf->gpio_num_info->
    		gpio_num[SENSOR_GPIO_VIO],
    		GPIO_OUT_HIGH);
	}
	else
	{
		msm_camera_config_single_vreg(&fctrl->pdev->dev,
			power_info->cam_vreg,
			&vreg,1);
	}

	msleep(5);

	rc = flash_i2c_client->i2c_func_tbl->i2c_read(
		flash_i2c_client, slave_info->sensor_id_reg_addr,
		&chipid,MSM_CAMERA_I2C_BYTE_DATA);//MSM_CAMERA_I2C_BYTE_DATA  MSM_CAMERA_I2C_WORD_DATA

	if (rc < 0)
	{
		have_read_id_flag = 0;
		pr_err("sky81294_id_match i2c_read failed\n");
	}
	else
		have_read_id_flag = 1;

	if(is_project(OPPO_14043)||is_project(OPPO_14045)|| is_project(OPPO_14037))
	{
		rc = msm_camera_request_gpio_table(
			power_info->gpio_conf->cam_gpio_req_tbl,
			power_info->gpio_conf->cam_gpio_req_tbl_size, 0);
		if (rc < 0) {
			pr_err("%s: request0 gpio failed\n", __func__);
		}
	}
	else
	{
		msm_camera_config_single_vreg(&fctrl->pdev->dev,
			power_info->cam_vreg,
			&vreg,0);
	}
	
	if (fctrl->flash_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_util(
			fctrl->flash_i2c_client, MSM_CCI_RELEASE);
		if (rc < 0)
			pr_err("cci_deinit failed\n");
	}

	if (0 == have_read_id_flag)
		goto probe_failure;
	
	sky81294_DBG("%s: read id: 0x%x expected id 0x%x:\n", __func__, chipid,
		slave_info->sensor_id);
	
	chipid &= 0x000f;
	
	if (chipid != slave_info->sensor_id) {
		pr_err("sky81294_id_match chip id doesnot match\n");
		goto probe_failure;
	}
	set_flashlight_id(FL_SKY81284);
	pr_err("%s: probe success\n", __func__);
	rc = msm_led_flash_create_v4lsubdev(pdev, fctrl);
	if(rc < 0)
	{
		pr_err("%s msm_led_flash_create_v4lsubdev failed\n", __func__);
		goto probe_failure;
	}
	return 0;

probe_failure:
	pr_err("%s probe failed\n", __func__);	
	return -1;
}

static const struct of_device_id sky81294_trigger_dt_match[] = 
{
	{.compatible = NEW_FLASH_NAME, .data = &fctrl},
	{}
};
static int msm_flash_sky81294_platform_probe(struct platform_device *pdev)
{
	int rc = 0;
	const struct of_device_id *match;
#ifdef VENDOR_EDIT
/*OPPO 2014-11-11 zhengrong.zhang add for torch can't use when open subcamera after boot*/
	struct msm_camera_power_ctrl_t *power_info = NULL;
#endif
	sky81294_DBG("%s entry\n", __func__);
	match = of_match_device(sky81294_trigger_dt_match, &pdev->dev);
	if (!match)
	{
		pr_err("%s, of_match_device failed!\n", __func__);
		return -EFAULT;
	}
	sky81294_DBG("%s of_match_device success\n", __func__);

	rc = sky81294_id_match(pdev, match->data);
	if(rc < 0)
		return rc;

	IS_FLASH_ON = 0;
	mutex_init(&flash_mode_lock);

	flash_proc_init(&fctrl);

/*OPPO 2014-11-11 zhengrong.zhang add for torch can't use when open subcamera after boot*/
	power_info = &fctrl.flashdata->power_info;
	rc = msm_camera_request_gpio_table(
    	power_info->gpio_conf->cam_gpio_req_tbl,
    	power_info->gpio_conf->cam_gpio_req_tbl_size, 1);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
	}
	rc = msm_camera_request_gpio_table(
    	power_info->gpio_conf->cam_gpio_req_tbl,
    	power_info->gpio_conf->cam_gpio_req_tbl_size, 0);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
	}
	
	return rc;
}

static struct platform_driver sky81294_platform_driver = 
{
	.probe = msm_flash_sky81294_platform_probe,
	.driver = 
	{
		.name = NEW_FLASH_NAME,
		.owner = THIS_MODULE,
		.of_match_table = sky81294_trigger_dt_match,
	},
};
static int __init msm_flash_sky81294_init(void)
{
	int32_t rc = 0;
	sky81294_DBG("%s entry\n", __func__);
	rc = platform_driver_register(&sky81294_platform_driver);
	sky81294_DBG("%s:%d rc %d\n", __func__, __LINE__, rc);
	return rc;
}
static void __exit msm_flash_sky81294_exit(void)
{
	sky81294_DBG("%s entry\n", __func__);
	platform_driver_unregister(&sky81294_platform_driver);
	return;
}

static struct msm_camera_i2c_client sky81294_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};

static struct msm_camera_i2c_reg_setting sky81294_init_setting = {
	.reg_setting = sky81294_init_array,
	.size = ARRAY_SIZE(sky81294_init_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting sky81294_off_setting = {
	.reg_setting = sky81294_off_array,
	.size = ARRAY_SIZE(sky81294_off_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting sky81294_release_setting = {
	.reg_setting = sky81294_release_array,
	.size = ARRAY_SIZE(sky81294_release_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting sky81294_low_setting = {
	.reg_setting = sky81294_low_array,
	.size = ARRAY_SIZE(sky81294_low_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting sky81294_high_setting = {
	.reg_setting = sky81294_high_array,
	.size = ARRAY_SIZE(sky81294_high_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_led_flash_reg_t sky81294_regs = {
	.init_setting = &sky81294_init_setting,
	.off_setting = &sky81294_off_setting,
	.low_setting = &sky81294_low_setting,
	.high_setting = &sky81294_high_setting,
	.release_setting = &sky81294_release_setting,
};

static struct msm_flash_fn_t sky81294_func_tbl = {
	.flash_get_subdev_id = msm_led_i2c_trigger_get_subdev_id,
	.flash_led_config = msm_led_i2c_trigger_config,
	.flash_led_init = msm_flash_sky81294_led_init,
	.flash_led_release = msm_flash_sky81294_led_release,
	.flash_led_off = msm_flash_sky81294_led_off,
	.flash_led_low = msm_flash_sky81294_led_low,
	.flash_led_high = msm_flash_sky81294_led_high,
};

static struct msm_led_flash_ctrl_t fctrl = {
	.flash_i2c_client = &sky81294_i2c_client,
	.reg_setting = &sky81294_regs,
	.func_tbl = &sky81294_func_tbl,
};

module_init(msm_flash_sky81294_init);
module_exit(msm_flash_sky81294_exit);
MODULE_DESCRIPTION("sky81294 FLASH");
MODULE_LICENSE("GPL v2");
