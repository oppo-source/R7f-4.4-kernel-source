/* Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/qpnp/pin.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/leds.h>
#include <linux/qpnp/pwm.h>
#include <linux/err.h>

#include "mdss_dsi.h"

#ifdef VENDOR_EDIT
/* Xinqin.Yang@PhoneSW.Multimedia, 2014/08/19  Add for 14023 project */
#include <linux/switch.h>
#include <mach/oppo_project.h>
#include <mach/oppo_boot_mode.h>
#include <mach/device_info.h>

int lcd_dev=0;
//guoling@MM.lcddriver add for lcd ESD check flag
bool lcd_reset = false;
#endif /*CONFIG_VENDOR_EDIT*/

#ifdef VENDOR_EDIT
/* YongPeng.Yi@SWDP.MultiMedia, 2015/03/11  Add for 15005 RTC597125 TEST START */
extern int RTC597125_15005DEBUG;
/* YongPeng.Yi@SWDP.MultiMedia, 2015/04/18  Add for 15005 esd truly lcd recovery START */
extern int enter_esd_15005;
/*  YongPeng.Yi@SWDP.MultiMedia END */
#endif /*VENDOR_EDIT*/
#define DT_CMD_HDR 6

/* NT35596 panel specific status variables */
#define NT35596_BUF_3_STATUS 0x02
#define NT35596_BUF_4_STATUS 0x40
#define NT35596_BUF_5_STATUS 0x80
#define NT35596_MAX_ERR_CNT 2

#define MIN_REFRESH_RATE 30

DEFINE_LED_TRIGGER(bl_led_trigger);

#ifdef VENDOR_EDIT
/* Xiaori.Yuan@Mobile Phone Software Dept.Driver, 2014/08/01  Add for ESD */
int te_check_gpio = 910;
DEFINE_SPINLOCK(te_count_lock);
DEFINE_SPINLOCK(te_state_lock);
DEFINE_SPINLOCK(delta_lock);

unsigned long flags;
struct mdss_dsi_ctrl_pdata *gl_ctrl_pdata;

static bool first_run_reset=1;
static bool cont_splash_flag;
static int irq;
static int te_state = 0;
static struct switch_dev display_switch;
static struct delayed_work techeck_work;
bool te_reset_14045 = 0;
static bool ESD_TE_TEST = 0;
static int samsung_te_check_count = 2;
u32 delta = 0;
static struct completion te_comp;

static irqreturn_t TE_irq_thread_fn(int irq, void *dev_id)
{	   
	static u32 prev = 0;
	u32 cur = 0;
	//pr_err("TE_irq_thread_fn\n");  
	if(samsung_te_check_count < 2){
		ktime_t time = ktime_get();
		cur = ktime_to_us(time);
		if (prev) {
			spin_lock_irqsave(&delta_lock,flags);
			delta = cur - prev;
			spin_unlock_irqrestore(&delta_lock,flags);
			//pr_err("%s delta = %d\n",__func__,delta);
		}
		prev = cur;
	}
	complete(&te_comp);
	return IRQ_HANDLED;
}

static int operate_display_switch(void)
{
    int ret = 0;
    printk("%s:state=%d.\n", __func__, te_state);

    spin_lock_irqsave(&te_state_lock, flags);
    if(te_state)
        te_state = 0;
    else
        te_state = 1;
    spin_unlock_irqrestore(&te_state_lock, flags);

    switch_set_state(&display_switch, te_state);
    return ret;
}

static void techeck_work_func( struct work_struct *work )
{
	int ret = 0;
	//pr_err("techeck_work_func\n");
	INIT_COMPLETION(te_comp);
	
	enable_irq(irq);
    ret = wait_for_completion_killable_timeout(&te_comp,
						msecs_to_jiffies(100));
	if(ret == 0){
		disable_irq(irq);
		operate_display_switch();
		return;
	}
	//pr_err("ret = %d\n", ret);
	disable_irq(irq);
	schedule_delayed_work(&techeck_work, msecs_to_jiffies(2000));
}


static ssize_t attr_mdss_dispswitch(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
    printk("ESD function test--------\n");
	if(is_project(OPPO_14045)||is_project(OPPO_14051)){
		te_reset_14045 = 1;
	}
	operate_display_switch();

    return 0;
}

static struct class * mdss_lcd;
static struct device * dev_lcd;
static struct device_attribute mdss_lcd_attrs[] = {			
	__ATTR(dispswitch, S_IRUGO|S_IWUSR, attr_mdss_dispswitch, NULL),	
	__ATTR_NULL,		
	};
#endif /*VENDOR_EDIT*/

void mdss_dsi_panel_pwm_cfg(struct mdss_dsi_ctrl_pdata *ctrl)
{
	ctrl->pwm_bl = pwm_request(ctrl->pwm_lpg_chan, "lcd-bklt");
	if (ctrl->pwm_bl == NULL || IS_ERR(ctrl->pwm_bl)) {
		pr_err("%s: Error: lpg_chan=%d pwm request failed",
				__func__, ctrl->pwm_lpg_chan);
	}
}

static void mdss_dsi_panel_bklt_pwm(struct mdss_dsi_ctrl_pdata *ctrl, int level)
{
	int ret;
	u32 duty;

	if (ctrl->pwm_bl == NULL) {
		pr_err("%s: no PWM\n", __func__);
		return;
	}

	if (level == 0) {
		if (ctrl->pwm_enabled)
			pwm_disable(ctrl->pwm_bl);
		ctrl->pwm_enabled = 0;
		return;
	}

	duty = level * ctrl->pwm_period;
	duty /= ctrl->bklt_max;

	pr_debug("%s: bklt_ctrl=%d pwm_period=%d pwm_gpio=%d pwm_lpg_chan=%d\n",
			__func__, ctrl->bklt_ctrl, ctrl->pwm_period,
				ctrl->pwm_pmic_gpio, ctrl->pwm_lpg_chan);

	pr_debug("%s: ndx=%d level=%d duty=%d\n", __func__,
					ctrl->ndx, level, duty);

	if (ctrl->pwm_enabled) {
		pwm_disable(ctrl->pwm_bl);
		ctrl->pwm_enabled = 0;
	}

	ret = pwm_config_us(ctrl->pwm_bl, duty, ctrl->pwm_period);
	if (ret) {
		pr_err("%s: pwm_config_us() failed err=%d.\n", __func__, ret);
		return;
	}

	ret = pwm_enable(ctrl->pwm_bl);
	if (ret)
		pr_err("%s: pwm_enable() failed err=%d\n", __func__, ret);
	ctrl->pwm_enabled = 1;
}

static char dcs_cmd[2] = {0x54, 0x00}; /* DTYPE_DCS_READ */
static struct dsi_cmd_desc dcs_read_cmd = {
	{DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(dcs_cmd)},
	dcs_cmd
};

u32 mdss_dsi_panel_cmd_read(struct mdss_dsi_ctrl_pdata *ctrl, char cmd0,
		char cmd1, void (*fxn)(int), char *rbuf, int len)
{
	struct dcs_cmd_req cmdreq;

	dcs_cmd[0] = cmd0;
	dcs_cmd[1] = cmd1;
	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &dcs_read_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT;
	cmdreq.rlen = len;
	cmdreq.rbuf = rbuf;
	cmdreq.cb = fxn; /* call back */
	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
	/*
	 * blocked here, until call back called
	 */

	return 0;
}

static void mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl,
			struct dsi_panel_cmds *pcmds)
{
	struct dcs_cmd_req cmdreq;
#ifdef VENDOR_EDIT
/* YongPeng.Yi@SWDP.MultiMedia, 2015/03/11  Add for 15005 RTC597125 TEST START */
	if(is_project(OPPO_15005)&&RTC597125_15005DEBUG)
	pr_err("%s\n",__func__);
/*  YongPeng.Yi@SWDP.MultiMedia END */
#endif /*VENDOR_EDIT*/
	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = pcmds->cmds;
	cmdreq.cmds_cnt = pcmds->cmd_cnt;
	cmdreq.flags = CMD_REQ_COMMIT;

	/*Panel ON/Off commands should be sent in DSI Low Power Mode*/
	if (pcmds->link_state == DSI_LP_MODE)
		cmdreq.flags  |= CMD_REQ_LP_MODE;

	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
}

static char led_pwm1[2] = {0x51, 0x0};	/* DTYPE_DCS_WRITE1 */
static struct dsi_cmd_desc backlight_cmd = {
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1, sizeof(led_pwm1)},
	led_pwm1
};

static void mdss_dsi_panel_bklt_dcs(struct mdss_dsi_ctrl_pdata *ctrl, int level)
{
	struct dcs_cmd_req cmdreq;

	pr_debug("%s: level=%d\n", __func__, level);

	led_pwm1[1] = (unsigned char)level;

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &backlight_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
}



#ifdef VENDOR_EDIT
/* Xiaori.Yuan@Mobile Phone Software Dept.Driver, 2014/07/28  Add for LCD acl and hbm function */
bool flag_lcd_off = false;
struct dsi_panel_cmds cabc_off_sequence;
struct dsi_panel_cmds cabc_user_interface_image_sequence;
struct dsi_panel_cmds cabc_still_image_sequence;
struct dsi_panel_cmds cabc_video_image_sequence;
extern int set_backlight_pwm(int state);

enum
{
    CABC_CLOSE = 0,
    CABC_LOW_MODE,
    CABC_MIDDLE_MODE,
    CABC_HIGH_MODE,

};
int cabc_mode = CABC_CLOSE; //defaoult mode level 0 in dtsi file
enum
{
    ACL_LEVEL_0 = 0,
    ACL_LEVEL_1,
    ACL_LEVEL_2,
    ACL_LEVEL_3,

};
int acl_mode = ACL_LEVEL_0; //defaoult mode level 1

static DEFINE_MUTEX(lcd_mutex);
int set_cabc(int level)
{
    int ret = 0;
	if(!(is_project(OPPO_14045) && (lcd_dev == LCD_14045_17_VIDEO || lcd_dev == LCD_14045_17_CMD))){
		return 0;
	}
	pr_err("%s : %d \n",__func__,level);

    mutex_lock(&lcd_mutex);
	
	if(flag_lcd_off == true)
    {
        printk(KERN_INFO "lcd is off,don't allow to set cabc\n");
        cabc_mode = level;
        mutex_unlock(&lcd_mutex);
        return 0;
    }

	mdss_dsi_clk_ctrl(gl_ctrl_pdata, DSI_ALL_CLKS, 1);
	
    switch(level)
    {
        case 0:
            set_backlight_pwm(0);
			 mdss_dsi_panel_cmds_send(gl_ctrl_pdata, &cabc_off_sequence);
            cabc_mode = CABC_CLOSE;
            break;
        case 1:
            mdss_dsi_panel_cmds_send(gl_ctrl_pdata, &cabc_user_interface_image_sequence);
            cabc_mode = CABC_LOW_MODE;
			set_backlight_pwm(1);
            break;
        case 2:
            mdss_dsi_panel_cmds_send(gl_ctrl_pdata, &cabc_still_image_sequence);
            cabc_mode = CABC_MIDDLE_MODE;
			set_backlight_pwm(1);
            break;
        case 3:
            mdss_dsi_panel_cmds_send(gl_ctrl_pdata, &cabc_video_image_sequence);
            cabc_mode = CABC_HIGH_MODE;
			set_backlight_pwm(1);
            break;
        default:
            pr_err("%s Leavel %d is not supported!\n",__func__,level);
            ret = -1;
            break;
    }
    mdss_dsi_clk_ctrl(gl_ctrl_pdata, DSI_ALL_CLKS, 0);
    mutex_unlock(&lcd_mutex);
    return ret;

}
static int set_cabc_resume_mode(int mode)
{
    int ret;
	if(!(is_project(OPPO_14045) && (lcd_dev == LCD_14045_17_VIDEO || lcd_dev == LCD_14045_17_CMD))){
		return 0;
	}
	pr_err("%s : %d yxr \n",__func__,mode);
    switch(mode)
    {
        case 0:
            set_backlight_pwm(0);
			mdss_dsi_panel_cmds_send(gl_ctrl_pdata, &cabc_off_sequence);
            break;
        case 1:
            mdss_dsi_panel_cmds_send(gl_ctrl_pdata, &cabc_user_interface_image_sequence);
			set_backlight_pwm(1);
            break;
        case 2:
            mdss_dsi_panel_cmds_send(gl_ctrl_pdata, &cabc_still_image_sequence);
			set_backlight_pwm(1);
            break;
        case 3:
           mdss_dsi_panel_cmds_send(gl_ctrl_pdata, &cabc_video_image_sequence);
		   set_backlight_pwm(1);
            break;
        default:
            pr_err("%s  %d is not supported!\n",__func__,mode);
            ret = -1;
            break;
    }
    return ret;
}


static char set_acl[2] = {0x55, 0x0};	/* DTYPE_DCS_WRITE1 */
static struct dsi_cmd_desc set_acl_cmd = {
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1, sizeof(set_acl)},
	set_acl
};
void set_acl_mode(int level)
{
	struct dcs_cmd_req cmdreq;
	if(!is_project(14005) && !is_project(OPPO_15011) && !is_project(OPPO_15018))
		return;
	pr_err("%s: level=%d\n", __func__, level);
	if(level < 0 || level > 3){
		pr_err("%s: invalid input %d! \n",__func__,level);
		return;
	}
	acl_mode = level;
	mutex_lock(&lcd_mutex);
	if(flag_lcd_off == true)
    {
        printk(KERN_INFO "lcd is off,don't allow to set acl mode !\n");
        mutex_unlock(&lcd_mutex);
        return;
    }
	set_acl[1] = (unsigned char)level;

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &set_acl_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(gl_ctrl_pdata, &cmdreq);
	mutex_unlock(&lcd_mutex);
}

static char set_hbm[2] = {0x53, 0x20};	/* DTYPE_DCS_WRITE1 */
static struct dsi_cmd_desc set_hbm_cmd = {
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1, sizeof(set_hbm)},
	set_hbm
};
void set_hbm_mode(int level)
{
	struct dcs_cmd_req cmdreq;
	if(!is_project(14005) && !is_project(OPPO_15011) && !is_project(OPPO_15018))
		return;
	pr_err("%s: level=%d\n", __func__, level);
	if(level < 0 || level > 2){
		pr_err("%s: invalid input %d! \n",__func__,level);
		return;
	}
	mutex_lock(&lcd_mutex);
	if(flag_lcd_off == true)
    {
        printk(KERN_INFO "lcd is off,don't allow to set hbm !\n");
        mutex_unlock(&lcd_mutex);
        return;
    }
	if(is_project(14005)){
		switch(level)
		{
			case 0:
				set_hbm[1] = 0x20;
				break;
			case 1:
				set_hbm[1] = 0x60;
				break;
			case 2:
				set_hbm[1] = 0xe0;
				break;
		}
	}
	if(is_project(OPPO_15011) || is_project(OPPO_15018)){
		if(level == 0)
			set_hbm[1] = 0x20;
		else
			set_hbm[1] = 0xe0;
	}
	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &set_hbm_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(gl_ctrl_pdata, &cmdreq);
	mutex_unlock(&lcd_mutex);
}

#endif /*VENDOR_EDIT*/


#ifndef VENDOR_EDIT
/* Xiaori.Yuan@Mobile Phone Software Dept.Driver, 2014/11/01  Modify for 14005 warning info first suspend */
static int mdss_dsi_request_gpios(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
#else /*VENDOR_EDIT*/
int mdss_dsi_request_gpios(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
#endif /*VENDOR_EDIT*/
{
	int rc = 0;

	if (gpio_is_valid(ctrl_pdata->disp_en_gpio)) {
		rc = gpio_request(ctrl_pdata->disp_en_gpio,
						"disp_enable");
		if (rc) {
			pr_err("request disp_en gpio failed, rc=%d\n",
				       rc);
			goto disp_en_gpio_err;
		}
        pr_err("%s YXQ disp_en_gpio=0x%x\n", __func__, ctrl_pdata->disp_en_gpio);
	}
#ifndef VENDOR_EDIT
/* Xinqin.Yang@PhoneSW.Multimedia, 2014/08/19  Add for -5V */
    if (is_project(OPPO_14023)) {
        if (gpio_is_valid(ctrl_pdata->disp_en_negative_gpio)) {
    		rc = gpio_request(ctrl_pdata->disp_en_negative_gpio,
    						"disp_negative_enable");
    		if (rc) {
    			pr_err("request disp_negative_en gpio failed, rc=%d\n",
    				       rc);
    			goto disp_en_gpio_err;
    		}
            pr_err("%s YXQ disp_en_negative_gpio=0x%x\n", __func__, ctrl_pdata->disp_en_negative_gpio);
    	}
    }
#endif /*CONFIG_VENDOR_EDIT*/
	rc = gpio_request(ctrl_pdata->rst_gpio, "disp_rst_n");
	if (rc) {
		pr_err("request reset gpio failed, rc=%d\n",
			rc);
		goto rst_gpio_err;
	}
    pr_err("%s YXQ rst_gpio=0x%x\n", __func__, ctrl_pdata->rst_gpio);
	if (gpio_is_valid(ctrl_pdata->bklt_en_gpio)) {
		rc = gpio_request(ctrl_pdata->bklt_en_gpio,
						"bklt_enable");
		if (rc) {
			pr_err("request bklt gpio failed, rc=%d\n",
				       rc);
			goto bklt_en_gpio_err;
		}
	}
	if (gpio_is_valid(ctrl_pdata->mode_gpio)) {
		rc = gpio_request(ctrl_pdata->mode_gpio, "panel_mode");
		if (rc) {
			pr_err("request panel mode gpio failed,rc=%d\n",
								rc);
			goto mode_gpio_err;
		}
	}
	return rc;

mode_gpio_err:
	if (gpio_is_valid(ctrl_pdata->bklt_en_gpio))
		gpio_free(ctrl_pdata->bklt_en_gpio);
bklt_en_gpio_err:
	gpio_free(ctrl_pdata->rst_gpio);
rst_gpio_err:
	if (gpio_is_valid(ctrl_pdata->disp_en_gpio))
		gpio_free(ctrl_pdata->disp_en_gpio);
disp_en_gpio_err:
	return rc;
}

int mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mdss_panel_info *pinfo = NULL;
	int i, rc = 0;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	if (!gpio_is_valid(ctrl_pdata->disp_en_gpio)) {
		pr_debug("%s:%d, reset line not configured\n",
			   __func__, __LINE__);
	}

	if (!gpio_is_valid(ctrl_pdata->rst_gpio)) {
		pr_debug("%s:%d, reset line not configured\n",
			   __func__, __LINE__);
		return rc;
	}

	pr_err("%s: enable = %d\n", __func__, enable);
	pinfo = &(ctrl_pdata->panel_data.panel_info);

	if (enable) {
#ifndef VENDOR_EDIT
/* Xiaori.Yuan@Mobile Phone Software Dept.Driver, 2014/09/15  Modify for 14045 1.8v gpio repeat request */
		rc = mdss_dsi_request_gpios(ctrl_pdata);
		if (rc) {
			pr_err("gpio request failed\n");
			return rc;
		}
#else /*VENDOR_EDIT*/
		if(!(is_project(OPPO_14045)||is_project(OPPO_14051))){
			rc = mdss_dsi_request_gpios(ctrl_pdata);
			if (rc) {
				pr_err("gpio request failed\n");
				return rc;
			}
		}
#endif /*VENDOR_EDIT*/
		//VENDOR_EDIT yxr delete for samsung oled
		pr_err("%s YXQ pinfo->panel_powe_on=%d\n", __func__, pinfo->panel_power_on);
        //mdelay(1000000);
        if (is_project(OPPO_14005) || is_project(OPPO_15011) || is_project(OPPO_15018)) {
			 if (!pinfo->panel_power_on){
            	if (gpio_is_valid(ctrl_pdata->disp_en_gpio))
               	 gpio_set_value((ctrl_pdata->disp_en_gpio), 1);
                        
          	 	 for (i = 0; i < pdata->panel_info.rst_seq_len; ++i) {
            	    gpio_set_value((ctrl_pdata->rst_gpio),
            	        pdata->panel_info.rst_seq[i]);
            	    if (pdata->panel_info.rst_seq[++i])
            	        usleep(pinfo->rst_seq[i] * 1000);
          		  }
            
           		 if (gpio_is_valid(ctrl_pdata->bklt_en_gpio))
              	  gpio_set_value((ctrl_pdata->bklt_en_gpio), 1);
        	}
        }else if(is_project(OPPO_14045)||is_project(OPPO_14051)){
        	if (!pinfo->panel_power_on){
				if(lcd_dev == LCD_14045_17_VIDEO || lcd_dev == LCD_14045_17_CMD){
	        		if (gpio_is_valid(ctrl_pdata->disp_en_gpio))
	              		 gpio_set_value((ctrl_pdata->disp_en_gpio), 1);
					mdelay(15);
				}
				if (gpio_is_valid(ctrl_pdata->disp_en_negative_gpio))
					gpio_set_value(ctrl_pdata->disp_en_negative_gpio, 1);
				if( te_reset_14045 == 1 ){
					pr_err("te_reset_14045 = 1\n");
					mdelay(5);
					for (i = 0; i < pdata->panel_info.rst_seq_len; ++i) {
	                gpio_set_value((ctrl_pdata->rst_gpio),
	                    pdata->panel_info.rst_seq[i]);
	                if (pdata->panel_info.rst_seq[++i])
	                    usleep(pinfo->rst_seq[i] * 1000);
	            	}
				//te_reset_14045 = 0;
				}
        	}
		}else if(is_project(OPPO_14037) || is_project(OPPO_15057)){
			if (!pinfo->panel_power_on){
			/* Yongpeng.Yi@Mobile Phone Software Dept.Driver, 2015/04/28  Add for 14037 tianma lcd power on flick */
				if (lcd_dev == LCD_TM_HX8392B) {
					pr_debug("Add for 14037 tianma lcd power on flick LCD_TM_HX8392B\n");
					for (i = 0; i < pdata->panel_info.rst_seq_len; ++i) {
						gpio_set_value((ctrl_pdata->rst_gpio),
						pdata->panel_info.rst_seq[i]);
						if (pdata->panel_info.rst_seq[++i])
							usleep(pinfo->rst_seq[i] * 1000);
					}
				}

			    if (gpio_is_valid(ctrl_pdata->disp_en_gpio))
					    gpio_set_value((ctrl_pdata->disp_en_gpio), 1);

			/* Yongpeng.Yi@Mobile Phone Software Dept.Driver, 2015/04/28  Add for 14037 tianma lcd power on flick */
				if (lcd_dev == LCD_TM_HX8392B) {
					mdelay(10);
					mdss_dsi_panel_cmds_send(ctrl_pdata, &ctrl_pdata->spec_cmds);
					mdelay(10);
					pr_debug("Add for 14037 tianma lcd power on flick &ctrl_pdata->spec_cmds\n");
				}

			    //bklt_en_gpio is -5V
			    if (gpio_is_valid(ctrl_pdata->bklt_en_gpio))
					    gpio_set_value((ctrl_pdata->bklt_en_gpio), 1);
				
			    mdelay(1);
			    for (i = 0; i < pdata->panel_info.rst_seq_len; ++i) {
				    gpio_set_value((ctrl_pdata->rst_gpio),
					    pdata->panel_info.rst_seq[i]);
				    if (pdata->panel_info.rst_seq[++i])
					    usleep(pinfo->rst_seq[i] * 1000);
			    }
			}
		}else {
            if (!pinfo->panel_power_on) {
			    if (gpio_is_valid(ctrl_pdata->disp_en_gpio))
				    gpio_set_value((ctrl_pdata->disp_en_gpio), 1);
			
			mdelay(1);
			    for (i = 0; i < pdata->panel_info.rst_seq_len; ++i) {
				    gpio_set_value((ctrl_pdata->rst_gpio),
					    pdata->panel_info.rst_seq[i]);
				    if (pdata->panel_info.rst_seq[++i])
					    usleep(pinfo->rst_seq[i] * 1000);
			    }

			    if (gpio_is_valid(ctrl_pdata->bklt_en_gpio))
				    gpio_set_value((ctrl_pdata->bklt_en_gpio), 1);
		    }
        }
		/*VENDOR_EDIT*/
		if (gpio_is_valid(ctrl_pdata->mode_gpio)) {
			if (pinfo->mode_gpio_state == MODE_GPIO_HIGH)
				gpio_set_value((ctrl_pdata->mode_gpio), 1);
			else if (pinfo->mode_gpio_state == MODE_GPIO_LOW)
				gpio_set_value((ctrl_pdata->mode_gpio), 0);
		}
		if (ctrl_pdata->ctrl_state & CTRL_STATE_PANEL_INIT) {
			pr_debug("%s: Panel Not properly turned OFF\n",
						__func__);
			ctrl_pdata->ctrl_state &= ~CTRL_STATE_PANEL_INIT;
			pr_debug("%s: Reset panel done\n", __func__);
		}
	} else {
#ifdef VENDOR_EDIT
/* Xiaori.Yuan@Mobile Phone Software Dept.Driver, 2014/09/15  Add for 14045 LCD and TP */
		if(is_project(OPPO_14045)||is_project(OPPO_14051)) { 
			if( te_reset_14045 == 1 ){
				pr_err("te_reset_14045 = 1\n");
				gpio_set_value((ctrl_pdata->rst_gpio), 0);
				mdelay(12);
				if (gpio_is_valid(ctrl_pdata->disp_en_negative_gpio))
						gpio_set_value(ctrl_pdata->disp_en_negative_gpio, 0);
				if(lcd_dev == LCD_14045_17_VIDEO || lcd_dev == LCD_14045_17_CMD){
					mdelay(10);
					if (gpio_is_valid(ctrl_pdata->disp_en_gpio))
				   		 gpio_set_value((ctrl_pdata->disp_en_gpio), 0);
				}
				
			}
			return 0;
			}
#endif /*VENDOR_EDIT*/
#ifndef VENDOR_EDIT
//guoling@MM.lcddriver modify for panel off timing
		if (gpio_is_valid(ctrl_pdata->bklt_en_gpio)) {
			gpio_set_value((ctrl_pdata->bklt_en_gpio), 0);
			gpio_free(ctrl_pdata->bklt_en_gpio);
		}
		if (gpio_is_valid(ctrl_pdata->disp_en_gpio)) {
			gpio_set_value((ctrl_pdata->disp_en_gpio), 0);
			gpio_free(ctrl_pdata->disp_en_gpio);
		}
#endif
#ifdef VENDOR_EDIT
/* YongPeng.Yi@SWDP.MultiMedia, 2015/04/18  Add for 15005 esd truly lcd recovery START */
		if(is_project(OPPO_15005)&&(lcd_dev==LCD_15005_TRULY_HX8379C)&&enter_esd_15005){
			pr_err("for 15005 esd recovery truly lcd!!!\n");
			gpio_set_value((ctrl_pdata->rst_gpio), 0);
			msleep(5);
			gpio_set_value((ctrl_pdata->rst_gpio), 1);
			msleep(5);
			gpio_set_value((ctrl_pdata->rst_gpio), 0);
			gpio_set_value((ctrl_pdata->disp_en_gpio), 0);
			msleep(10);
			gpio_set_value((ctrl_pdata->disp_en_gpio), 1);
			msleep(5);
			gpio_set_value((ctrl_pdata->rst_gpio), 1);
			msleep(5);
			gpio_set_value((ctrl_pdata->rst_gpio), 0);
			msleep(5);
			gpio_set_value((ctrl_pdata->rst_gpio), 1);
			msleep(5);
			enter_esd_15005 = 0;
		}
/* YongPeng.Yi@SWDP.MultiMedia END */
#endif /*VENDOR_EDIT*/
		gpio_set_value((ctrl_pdata->rst_gpio), 0);
#ifdef VENDOR_EDIT
		/*Yongpeng.Yi@PhoneSW.Multimedia 2015-02-03 add for IC HX8379-C pull up rst avoid more electric when sleep*/
        if(is_project(OPPO_14043) || is_project(OPPO_14037) || is_project(OPPO_15005) 
		|| is_project(OPPO_15057) || is_project(OPPO_15025)){
    		msleep(30);
			if(!((lcd_dev == LCD_15025_TM_NT35512S)||(lcd_dev == LCD_TM_NT35512S)))
    		gpio_set_value((ctrl_pdata->rst_gpio), 1);
		}
#endif
		gpio_free(ctrl_pdata->rst_gpio);
#ifdef VENDOR_EDIT
	        if(is_project(OPPO_15005)||is_project(OPPO_15025)){
			/*YongPeng.Yi@PhoneSW.Multimedia 2015-02-02 add reduce 0.9ma electric when sleep*/
				if (gpio_is_valid(902)) {
					rc = gpio_request(902,"ID_1");
					if (rc) {
						pr_err("request ID_1 902  failed, rc=%d\n", rc);
						}
					else{
						gpio_set_value(902, 0);
						rc = gpio_direction_output(902, 0);
						if (rc) {
							pr_err("gpio_direction_output ID_1 902  failed, rc=%d\n", rc);
							}
						gpio_free(902);
						}
					}
				else{
						pr_err("ID_1 902 isn't valid");
				}

#ifdef VENDOR_EDIT
/* YongPeng.Yi@SWDP.MultiMedia, 2015/04/22 */
				if (gpio_is_valid(904)) {
					rc = gpio_request(904,"ID_2");
					if (rc) {
						pr_err("request ID_1 904  failed, rc=%d\n", rc);
						}
					else{
						gpio_set_value(904, 0);
						rc = gpio_direction_output(904, 0);
						if (rc) {
							pr_err("gpio_direction_output ID_2 904  failed, rc=%d\n", rc);
							}
						gpio_free(904);
						}
					}
				else{
						pr_err("ID_1 904 isn't valid");
				}
#endif /*VENDOR_EDIT*/

			/*YongPeng.Yi@PhoneSW.Multimedia 2015-02-03 add for BOE sleep electric when sleep*/
				if (gpio_is_valid(905)) {
					rc = gpio_request(905,"ID_3");
					if (rc) {
						pr_err("request ID_3 905  failed, rc=%d\n", rc);
						}
					else{
						gpio_set_value(905, 0);
						rc = gpio_direction_output(905, 0);
						if (rc) {
							pr_err("gpio_direction_output ID_3 905  failed, rc=%d\n", rc);
							}
						gpio_free(905);
						}
					}
				else{
						pr_err("ID_3 905 isn't valid");
				}
			}
//guoling@MM.lcddriver modify for panel off timing
        if(is_project(OPPO_14037) || is_project(OPPO_15057)){
            mdelay(5);
            //bklt_en_gpio is -5V
    		if (gpio_is_valid(ctrl_pdata->bklt_en_gpio)) {
    			gpio_set_value((ctrl_pdata->bklt_en_gpio), 0);
    			gpio_free(ctrl_pdata->bklt_en_gpio);
    		}
    		mdelay(5);

    		if (gpio_is_valid(ctrl_pdata->disp_en_gpio)) {
    			gpio_set_value((ctrl_pdata->disp_en_gpio), 0);
    			gpio_free(ctrl_pdata->disp_en_gpio);
    		}
			/*Start YongPeng.Yi@PhoneSW.Multimedia, 2014/12/15 Add for 14037 truely ID pin warning*/
			if(lcd_dev == LCD_HX8394_TRULY_P3_MURA){
			//add reduce electric when sleep for truly panel
			if (gpio_is_valid(905)) {
			rc = gpio_request(905,"ID_1");
			if (rc) {
			pr_err("request ID_1 905  failed, rc=%d\n", rc);
			}

			gpio_set_value(905, 0);
			rc = gpio_direction_output(905, 0);
			if (rc) {
			pr_err("gpio_direction_output ID_1 905  failed, rc=%d\n", rc);
			}
			gpio_free(905);
			}
			else{
			pr_err("ID_1 905 isn't valid");
			}

			if (gpio_is_valid(1004)) {
			rc = gpio_request(1004,"ID_2");
			if (rc) {
			pr_err("request ID_2 1004  failed, rc=%d\n", rc);
			}

			gpio_set_value(1004, 0);
			rc = gpio_direction_output(1004, 0);
			if (rc) {
			pr_err("gpio_direction_output ID_2 1004  failed, rc=%d\n", rc);
			}
			gpio_free(1004);
			}
			else{
			pr_err("ID_2 1004 isn't valid");
			}


			if (gpio_is_valid(1007)) {
			rc = gpio_request(1007,"ID_3");
			if (rc) {
			pr_err("request ID_3 1007  failed, rc=%d\n", rc);
			}

			gpio_set_value(1007, 0);
			rc = gpio_direction_output(1007, 0);
			if (rc) {
			pr_err("gpio_direction_output ID_3 1007  failed, rc=%d\n", rc);
			}
			gpio_free(1007);
			}
			else{
			pr_err("ID_3 1007 isn't valid");
			}
			}
			/*End YongPeng.Yi@PhoneSW.Multimedia, 2014/12/15 Add for 14037 truely ID pin warning*/
		}else{ 
		    if (gpio_is_valid(ctrl_pdata->bklt_en_gpio)) {
    			gpio_set_value((ctrl_pdata->bklt_en_gpio), 0);
    			gpio_free(ctrl_pdata->bklt_en_gpio);
    		}
    		if (gpio_is_valid(ctrl_pdata->disp_en_gpio)) {
    			gpio_set_value((ctrl_pdata->disp_en_gpio), 0);
    			gpio_free(ctrl_pdata->disp_en_gpio);
    		}
		}
#endif
		if (gpio_is_valid(ctrl_pdata->mode_gpio))
			gpio_free(ctrl_pdata->mode_gpio);
	}
	return rc;
}

static char caset[] = {0x2a, 0x00, 0x00, 0x03, 0x00};	/* DTYPE_DCS_LWRITE */
static char paset[] = {0x2b, 0x00, 0x00, 0x05, 0x00};	/* DTYPE_DCS_LWRITE */

static struct dsi_cmd_desc partial_update_enable_cmd[] = {
	{{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(caset)}, caset},
	{{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(paset)}, paset},
};

static int mdss_dsi_panel_partial_update(struct mdss_panel_data *pdata)
{
	struct mipi_panel_info *mipi;
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct dcs_cmd_req cmdreq;
	int rc = 0;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);
	mipi  = &pdata->panel_info.mipi;

	pr_debug("%s: ctrl=%p ndx=%d\n", __func__, ctrl, ctrl->ndx);

	caset[1] = (((pdata->panel_info.roi_x) & 0xFF00) >> 8);
	caset[2] = (((pdata->panel_info.roi_x) & 0xFF));
	caset[3] = (((pdata->panel_info.roi_x - 1 + pdata->panel_info.roi_w)
								& 0xFF00) >> 8);
	caset[4] = (((pdata->panel_info.roi_x - 1 + pdata->panel_info.roi_w)
								& 0xFF));
	partial_update_enable_cmd[0].payload = caset;

	paset[1] = (((pdata->panel_info.roi_y) & 0xFF00) >> 8);
	paset[2] = (((pdata->panel_info.roi_y) & 0xFF));
	paset[3] = (((pdata->panel_info.roi_y - 1 + pdata->panel_info.roi_h)
								& 0xFF00) >> 8);
	paset[4] = (((pdata->panel_info.roi_y - 1 + pdata->panel_info.roi_h)
								& 0xFF));
	partial_update_enable_cmd[1].payload = paset;

	pr_debug("%s: enabling partial update\n", __func__);
	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = partial_update_enable_cmd;
	cmdreq.cmds_cnt = 2;
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl, &cmdreq);

	return rc;
}

static void mdss_dsi_panel_switch_mode(struct mdss_panel_data *pdata,
							int mode)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mipi_panel_info *mipi;
	struct dsi_panel_cmds *pcmds;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return;
	}

	mipi  = &pdata->panel_info.mipi;

	if (!mipi->dynamic_switch_enabled)
		return;

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	if (mode == DSI_CMD_MODE)
		pcmds = &ctrl_pdata->video2cmd;
	else
		pcmds = &ctrl_pdata->cmd2video;

	mdss_dsi_panel_cmds_send(ctrl_pdata, pcmds);

	return;
}

#ifdef VENDOR_EDIT
/* Xinqin.Yang@PhoneSW.Multimedia, 2014/09/03  Add for lm3630 */
extern  int lm3630_bank_a_update_status(u32 bl_level);
#endif /*CONFIG_VENDOR_EDIT*/

extern int set_wakeup_gesture_mode(uint8_t value);


static char samsung_oncmd_0_0[] = {0x11, 0x00}; 
static char samsung_oncmd_0_1[] = {0x35, 0x00}; 
static char samsung_oncmd_0_2[] = {0xf0, 0x5a, 0x5a}; 
static char samsung_oncmd_0_3[] = {0xfc, 0x5a, 0x5a}; 
static char samsung_oncmd_0_4[] = {0xfd, 0xb8}; 
static char samsung_oncmd_0_5[] = {0xb0, 0x14}; 
static char samsung_oncmd_0_6[] = {0xd7, 0x75}; 
static char samsung_oncmd_0_7[] = {0xb0, 0x20}; 
static char samsung_oncmd_0_8[] = {0xd7, 0x00}; 
static char samsung_oncmd_0_9[] = {0xfe, 0x80}; 
static char samsung_oncmd_0_10[] = {0xfe, 0x00}; 
static struct dsi_cmd_desc samsung_oncmd_0[] = {
	{{DTYPE_DCS_WRITE, 1, 0, 1, 20, sizeof(samsung_oncmd_0_0)},	samsung_oncmd_0_0},
	{{DTYPE_DCS_WRITE, 1, 0, 1, 0, sizeof(samsung_oncmd_0_1)},	samsung_oncmd_0_1},
	{{DTYPE_GEN_LWRITE, 1, 0, 1, 0, sizeof(samsung_oncmd_0_2)},	samsung_oncmd_0_2},
	{{DTYPE_GEN_LWRITE, 1, 0, 1, 0, sizeof(samsung_oncmd_0_3)},	samsung_oncmd_0_3},
	{{DTYPE_DCS_WRITE1, 1, 0, 1, 0, sizeof(samsung_oncmd_0_4)},	samsung_oncmd_0_4},
	{{DTYPE_DCS_WRITE1, 1, 0, 1, 0, sizeof(samsung_oncmd_0_5)},	samsung_oncmd_0_5},
	{{DTYPE_DCS_WRITE1, 1, 0, 1, 0, sizeof(samsung_oncmd_0_6)},	samsung_oncmd_0_6},
	{{DTYPE_DCS_WRITE1, 1, 0, 1, 0, sizeof(samsung_oncmd_0_7)},	samsung_oncmd_0_7},
	{{DTYPE_DCS_WRITE1, 1, 0, 1, 0, sizeof(samsung_oncmd_0_8)},	samsung_oncmd_0_8},
	{{DTYPE_DCS_WRITE1, 1, 0, 1, 0, sizeof(samsung_oncmd_0_9)},	samsung_oncmd_0_9},
	{{DTYPE_DCS_WRITE1, 1, 0, 1, 0, sizeof(samsung_oncmd_0_10)},samsung_oncmd_0_10},
};

//static char samsung_oncmd_1_0[] = {0x35, 0x00}; 
static char samsung_oncmd_1_1[] = {0x53, 0x20}; 
static char samsung_oncmd_1_2[] = {0x29, 0x00}; 
 char samsung_oncmd_1_3[] = {0xf2, 0x01, 0x48, 0xa0, 0x09, 0x07, 0x00}; 
static char samsung_oncmd_1_4[] = {0xf0, 0xa5, 0xa5}; 
static char samsung_oncmd_1_5[] = {0xfc, 0xa5, 0xa5}; 

static char samsung_oncmd_1_6[] = {0x55, 0x00}; 


static struct dsi_cmd_desc samsung_oncmd_1[] = {
	//{{DTYPE_DCS_WRITE1, 1, 0, 1, 0, sizeof(samsung_oncmd_1_0)},	samsung_oncmd_1_0},
	{{DTYPE_GEN_LWRITE, 1, 0, 1, 0, sizeof(samsung_oncmd_1_3)},	samsung_oncmd_1_3},
	{{DTYPE_DCS_WRITE1, 1, 0, 1, 0, sizeof(samsung_oncmd_1_4)},	samsung_oncmd_1_4},
	{{DTYPE_DCS_WRITE1, 1, 0, 1, 0, sizeof(samsung_oncmd_1_5)},	samsung_oncmd_1_5},
	{{DTYPE_DCS_WRITE1, 1, 0, 1, 0, sizeof(samsung_oncmd_1_1)},	samsung_oncmd_1_1},
	{{DTYPE_DCS_WRITE1, 1, 0, 1, 0, sizeof(samsung_oncmd_1_6)},	samsung_oncmd_1_6},
	{{DTYPE_DCS_WRITE, 1, 0, 1, 40, sizeof(samsung_oncmd_1_2)},	samsung_oncmd_1_2},
};


static char samsung_oncmd_0_11[] = {0xb0, 0x08}; 
static char samsung_oncmd_0_12[] = {0xb2, 0x60}; 
static char samsung_oncmd_0_13[] = {0xf7, 0x03}; 

static struct dsi_cmd_desc samsung_oncmd_7[] = {     //change aid cycle 1 to cycle 8
	{{DTYPE_DCS_WRITE1, 1, 0, 1, 0, sizeof(samsung_oncmd_0_11)},	samsung_oncmd_0_11},
	{{DTYPE_DCS_WRITE1, 1, 0, 1, 0, sizeof(samsung_oncmd_0_12)},	samsung_oncmd_0_12},
	{{DTYPE_DCS_WRITE1, 1, 0, 1, 0, sizeof(samsung_oncmd_0_13)},	samsung_oncmd_0_13},
};

static char samsung_oncmd_0_b1_1[] = {0xb1, 0x00, 0x00, 0x42, 0xff, 0x04, 0x13, 0x2b, 0x42, 0x78, 0xb2, 0xff}; 
static char samsung_oncmd_0_b2[] = 	 {0xb2, 0xe0, 0x0e, 0xe0, 0x0b, 0xe0, 0x06, 0x06, 0x68, 0x70, 0xff, 0xff,0xff}; 
static char samsung_oncmd_0_f7[] = 	 {0xf7, 0x03};

static struct dsi_cmd_desc samsung_oncmd_2 = {
	{DTYPE_GEN_LWRITE, 1, 0, 1, 0, sizeof(samsung_oncmd_0_2)},	samsung_oncmd_0_2
};

static struct dsi_cmd_desc samsung_oncmd_3 = {
	{DTYPE_GEN_LWRITE, 1, 0, 1, 0, sizeof(samsung_oncmd_1_4)},	samsung_oncmd_1_4
};

static struct dsi_cmd_desc samsung_oncmd_4 = {
	{DTYPE_GEN_LWRITE, 1, 0, 1, 0, sizeof(samsung_oncmd_0_b1_1)}, samsung_oncmd_0_b1_1
};

static struct dsi_cmd_desc samsung_oncmd_5 = {
	{DTYPE_GEN_LWRITE, 1, 0, 1, 0, sizeof(samsung_oncmd_0_b2)}, samsung_oncmd_0_b2
};  //0xb2

static struct dsi_cmd_desc samsung_oncmd_6 = {
	{0x15, 1, 0, 1, 0, sizeof(samsung_oncmd_0_f7)}, samsung_oncmd_0_f7
}; //0xf7








static char samsung_fit_0[2] = {0xb0, 0x14}; 
static char samsung_fit_1[2] = {0xd7, 0x6f}; 
static char samsung_fit_2[2] = {0xfe, 0x80};
static char samsung_fit_3[2] = {0xfe, 0x00};


static struct dsi_cmd_desc samsung_fit[] = {
	{{DTYPE_DCS_WRITE1, 1, 0, 1, 0, sizeof(samsung_fit_0)},	samsung_fit_0},
	{{DTYPE_DCS_WRITE1, 1, 0, 1, 0, sizeof(samsung_fit_1)},	samsung_fit_1},
	{{DTYPE_DCS_WRITE1, 1, 0, 1, 0, sizeof(samsung_fit_2)},	samsung_fit_2},
	{{DTYPE_DCS_WRITE1, 1, 0, 1, 0, sizeof(samsung_fit_3)},	samsung_fit_3},
};
static int send_samsung_fit_cmd(struct dsi_cmd_desc * cmd , int count)
{
	struct dcs_cmd_req cmdreq;
	//pr_err("samsung_fit size : %d\n",count);
	//return;
	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = cmd;
	cmdreq.cmds_cnt = count;
	cmdreq.flags = CMD_REQ_COMMIT;// | CMD_REQ_LP_MODE;//CMD_REQ_LP_MODE
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;
	return mdss_dsi_cmdlist_put(gl_ctrl_pdata, &cmdreq);
}

static int set_acl_resume_mode(int level){
	if(!is_project(14005) && !is_project(OPPO_15011) && !is_project(OPPO_15018))
		return 0;
	pr_err("%s: level=%d\n", __func__, level);
	set_acl[1] = (unsigned char)level;
	return send_samsung_fit_cmd(&set_acl_cmd,1);
}


int samsung_lut_1[90][2] = {
	{33333, 0x32},
	{22999, 0x33},
	{22861, 0x34},
	{22726, 0x35},
	{22609, 0x36},
	{22490, 0x37},
	{22363, 0x38},
	{22237, 0x39},
	{22121, 0x3A},
	{22005, 0x3B},
	{21880, 0x3C},
	{21761, 0x3D},
	{21649, 0x3E},
	{21535, 0x3F},
	{21461, 0x40},
	{21385, 0x41},
	{21275, 0x42},
	{21170, 0x43},
	{21056, 0x44},
	{20943, 0x45},
	{20834, 0x46},
	{20723, 0x47},
	{20614, 0x48},
	{20506, 0x49},
	{20406, 0x4A},
	{20311, 0x4B},
	{20206, 0x4C},
	{20099, 0x4D},
	{20003, 0x4E},
	{19903, 0x4F},
	{19808, 0x50},
	{19717, 0x51},
	{19621, 0x52},
	{19526, 0x53},
	{19428, 0x54},
	{19331, 0x55},
	{19244, 0x56},
	{19153, 0x57},
	{19056, 0x58},
	{18963, 0x59},
	{18879, 0x5A},
	{18794, 0x5B},
	{18702, 0x5C},
	{18610, 0x5D},
	{18522, 0x5E},
	{18438, 0x5F},
	{18359, 0x60},
	{18277, 0x61},
	{18196, 0x62},
	{18116, 0x63},
	{18030, 0x64},
	{17943, 0x65},
	{17862, 0x66},
	{17783, 0x67},
	{17704, 0x68},
	{17625, 0x69},
	{17545, 0x6A},
	{17464, 0x6B},
	{17384, 0x6C},
	{17306, 0x6D},
	{17231, 0x6E},
	{17158, 0x6F},
	{17089, 0x70},
	{17016, 0x71},
	{16944, 0x72},
	{16874, 0x73},
	{16797, 0x74},
	{16717, 0x75},
	{16646, 0x76},
	{16578, 0x77},
	{16505, 0x78},
	{16433, 0x79},
	{16365, 0x7A},
	{16299, 0x7B},
	{16228, 0x7C},
	{16152, 0x7D},
	{16089, 0x7E},
	{16027, 0x7F},
	{15973, 0x80},
	{15921, 0x81},
	{15855, 0x82},
	{15787, 0x83},
	{15720, 0x84},
	{15657, 0x85},
	{15594, 0x86},
	{15532, 0x87},
	{15466, 0x88},
	{15401, 0x89},
	{15342, 0x8A},
	{15283, 0x8B},
};
int samsung_lut_2[67][2] = {
	{19655, -34}, 
	{19551, -33},
	{19449, -32}, 
	{19354, -31}, 
	{19264, -30}, 
	{19165, -29}, 
	{19063, -28}, 
	{18972, -27}, 
	{18877, -26}, 
	{18787, -25}, 
	{18701, -24}, 
	{18610, -23}, 
	{18520, -22}, 
	{18427, -21}, 
	{18335, -20}, 
	{18253, -19}, 
	{18167, -18}, 
	{18074, -17}, 
	{17986, -16}, 
	{17906, -15}, 
	{17826, -14}, 
	{17739, -13}, 
	{17651, -12}, 
	{17567, -11}, 
	{17488, -10}, 
	{17413, -9 },
	{17335, -8 },
	{17258, -7 },
	{17183, -6 },
	{17101, -5 },
	{17019, -4 },
	{16942, -3 },
	{16867, -2 },
	{16819, 0  },
	{16519, 2  },
	{16488, 3  },
	{16414, 4  },
	{16343, 5  },
	{16274, 6  },
	{16208, 7  },
	{16140, 8  },
	{16071, 9  },
	{16005, 10 },
	{15932, 11 },
	{15856, 12 },
	{15789, 13 },
	{15724, 14 },
	{15655, 15 },
	{15586, 16 },
	{15522, 17 },
	{15460, 18 },
	{15392, 19 },
	{15320, 20 },
	{15260, 21 },
	{15201, 22 },
	{15150, 23 },
	{15100, 24 },
	{15038, 25 },
	{14973, 26 },
	{14910, 27 },
	{14850, 28 },
	{14791, 29 },
	{14731, 30 },
	{14669, 31 },
	{14607, 32 },
	{14552, 33 },
	{14496, 34 },
};

static int samsung_gamma_offset[132] = {
	0,-7,0,-7,0,-9,3,	3,4,1,	0,	0,	0,	2,	0,	2,	2,	4,	 0,  0, -2, -1, 0,	2,	0,	1,	2,	0,	0,	0,	0,	0,	0,
	0,-7,0,-6,0,-10,1,0,2,0,0,	0,	2,	0,	0,	0,	2,	4,	-2, -2, -2, -2, 1,	4,	-10,	-3, 3,	0,	0,	0,	0,	0,	0,
	0,-9,0,-7,0,-11,2,1,3,0,0,	0,	1,	0,	0,	1,	1,	4,	-4, -2, -1, -5, 0,	5,	0,	12, 9,	0,	0,	0,	0,	0,	0,
	0,-13,0,-10,0,-14,1,1,1,-1, 1,	1,	0,	1,	1,	-2, 3,	6,	-4,  0, 1,	-1, 9,	2,	0,	1,	2,	0,	0,	0,	0,	0,	0,
};

static char samsung_c9_for_lowbrightness[133];
static char samsung_otp_c9[133];
static struct dsi_cmd_desc samsung_oncmd_c9 = {
	{DTYPE_GEN_LWRITE, 1, 0, 1, 0, sizeof(samsung_c9_for_lowbrightness)}, samsung_c9_for_lowbrightness
};
struct dsi_cmd_desc samsung_cmd_otp_c9 = {
	{DTYPE_GEN_LWRITE, 1, 0, 1, 0, sizeof(samsung_otp_c9)}, samsung_otp_c9
};

static bool flag_first_set_bl = true;
static bool samsung_fit_err = false;
char read[10];
static void mdss_dsi_panel_bl_ctrl(struct mdss_panel_data *pdata,
							u32 bl_level)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	/*VENDOR_EDIT*/
	int i,ret = 0;
#ifndef VENDOR_EDIT
/* Xinqin.Yang@PhoneSW.Multimedia, 2014/09/03  Modify for backlight */
	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	/*
	 * Some backlight controllers specify a minimum duty cycle
	 * for the backlight brightness. If the brightness is less
	 * than it, the controller can malfunction.
	 */

	if ((bl_level < pdata->panel_info.bl_min) && (bl_level != 0))
		bl_level = pdata->panel_info.bl_min;

	switch (ctrl_pdata->bklt_ctrl) {
	case BL_WLED:
		led_trigger_event(bl_led_trigger, bl_level);
		break;
	case BL_PWM:
		mdss_dsi_panel_bklt_pwm(ctrl_pdata, bl_level);
		break;
	case BL_DCS_CMD:
		
		mdss_dsi_panel_bklt_dcs(ctrl_pdata, bl_level);
		if (mdss_dsi_is_master_ctrl(ctrl_pdata)) {
			struct mdss_dsi_ctrl_pdata *sctrl =
				mdss_dsi_get_slave_ctrl();
			if (!sctrl) {
				pr_err("%s: Invalid slave ctrl data\n",
					__func__);
				return;
			}
			mdss_dsi_panel_bklt_dcs(sctrl, bl_level);
		}
		break;
	default:
		pr_err("%s: Unknown bl_ctrl configuration\n",
			__func__);
		break;
	}
#else /*VENDOR_EDIT*/
	if (is_project(OPPO_14023) || is_project(OPPO_14045) || is_project(OPPO_14051)) {
        lm3630_bank_a_update_status(bl_level);
	} else {
#ifdef VENDOR_EDIT
/* Xiaori.Yuan@Mobile Phone Software Dept.Driver, 2014/11/16  Add for decrease brightness of 14005 */
		if(is_project(OPPO_14005)){
			bl_level = (bl_level * 64 + 35)/70;
			/*Shield two brightness for flick*/
			if(bl_level==63) bl_level=62;
			if(bl_level==64) bl_level=65;
		}
#endif /*VENDOR_EDIT*/
        if (pdata == NULL) {
            pr_err("%s: Invalid input data\n", __func__);
            return;
        }
        
        ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
                    panel_data);
        
        /*
         * Some backlight controllers specify a minimum duty cycle
         * for the backlight brightness. If the brightness is less
         * than it, the controller can malfunction.
         */
        
        if ((bl_level < pdata->panel_info.bl_min) && (bl_level != 0))
            bl_level = pdata->panel_info.bl_min;
        
        switch (ctrl_pdata->bklt_ctrl) {
        case BL_WLED:
            led_trigger_event(bl_led_trigger, bl_level);
            break;
        case BL_PWM:
            mdss_dsi_panel_bklt_pwm(ctrl_pdata, bl_level);
            break;
        case BL_DCS_CMD:
			if(is_project(OPPO_14005) && MSM_BOOT_MODE__NORMAL == get_boot_mode()&& gl_ctrl_pdata->panel_data.panel_info.cont_splash_enabled==0){
			//if(0){
				if(flag_first_set_bl){
					ret = send_samsung_fit_cmd(&samsung_oncmd_2,1); //f0 5a 5a
					if(0!=ret){samsung_fit_err = true;}

					ret = mdss_dsi_panel_cmd_read(gl_ctrl_pdata,0xc9,0x00,NULL,&samsung_otp_c9[1],132);      //read c9
					if(0!=ret){samsung_fit_err = true;}
					
					samsung_otp_c9[0]=0xc9;
					
					for( i = 0; i < 132; i++ ){
							if(((abs(samsung_otp_c9[i+1])) - abs(samsung_gamma_offset[i]))>=0){
								samsung_c9_for_lowbrightness[i+1] = samsung_otp_c9[i+1] + samsung_gamma_offset[i];
							}else{
								samsung_c9_for_lowbrightness[i] -= 1;
								samsung_c9_for_lowbrightness[i+1] = 256 + samsung_otp_c9[i+1] + samsung_gamma_offset[i];
							}
					}
					samsung_c9_for_lowbrightness[0]=0xc9;
	
					pr_err("--------------------------22\n");
					
					ret=send_samsung_fit_cmd(&samsung_oncmd_4,1); // 0xb1, 0x00, 0x00, 0x42, 0xff, 0x04, 0x13, 0x2b, 0x42, 0x78, 0xb2, 0xff
					if(0!=ret){samsung_fit_err = true;}
					
					ret=send_samsung_fit_cmd(&samsung_oncmd_5,1); //0xb2
					if(0!=ret){samsung_fit_err = true;}
					
					ret=send_samsung_fit_cmd(&samsung_oncmd_6,1);  //0xf7
					if(0!=ret){samsung_fit_err = true;}
					
					if(!samsung_fit_err){
						ret=send_samsung_fit_cmd(&samsung_oncmd_c9,1); //send c9
						if(0!=ret){samsung_fit_err = true;}
					}
					if(!samsung_fit_err){  //change aid cycle 1 to cycle 8
						ret=send_samsung_fit_cmd(samsung_oncmd_7,sizeof(samsung_oncmd_7)/sizeof(struct dsi_cmd_desc));
						if(0!=ret){samsung_fit_err = true;}
					}
					ret=send_samsung_fit_cmd(&samsung_oncmd_3,1); //f0 a5 a5
					if(0!=ret){samsung_fit_err = true;}
					
					flag_first_set_bl = false;	
				}
			}
			
            mdss_dsi_panel_bklt_dcs(ctrl_pdata, bl_level);
            if (mdss_dsi_is_master_ctrl(ctrl_pdata)) {
                struct mdss_dsi_ctrl_pdata *sctrl =
                    mdss_dsi_get_slave_ctrl();
                if (!sctrl) {
                    pr_err("%s: Invalid slave ctrl data\n",
                        __func__);
                    return;
                }
                mdss_dsi_panel_bklt_dcs(sctrl, bl_level);
            }
            break;
        default:
            pr_err("%s: Unknown bl_ctrl configuration\n",
                __func__);
            break;
        }

	}
#endif /*VENDOR_EDIT*/
}

static int mdss_dsi_panel_on(struct mdss_panel_data *pdata)
{
	struct mipi_panel_info *mipi;
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	/*VENDOR_EDIT*/
	int i;
	/*if( (is_project(OPPO_14045)||is_project(OPPO_14051)) && te_reset_14045 ==0){
		set_wakeup_gesture_mode(0x08);
		mdelay(20);
	}*/
	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);
	mipi  = &pdata->panel_info.mipi;

	pr_err("%s: ctrl=%p ndx=%d\n", __func__, ctrl, ctrl->ndx);

#ifndef VENDOR_EDIT
/* Xiaori.Yuan@Mobile Phone Software Dept.Driver, 2014/11/11  Modify for 14005 stripe */
	if (ctrl->on_cmds.cmd_cnt)
		mdss_dsi_panel_cmds_send(ctrl, &ctrl->on_cmds);
#else /*VENDOR_EDIT*/
	if(!is_project(OPPO_14005)){
		if (ctrl->on_cmds.cmd_cnt)
		mdss_dsi_panel_cmds_send(ctrl, &ctrl->on_cmds);
		
		if(is_project(OPPO_14045) && ctrl->ndx==0 && (lcd_dev == LCD_14045_17_VIDEO || lcd_dev == LCD_14045_17_CMD)){
			//set_backlight_pwm(1);
			if(cabc_mode != CABC_CLOSE){
					set_cabc_resume_mode(cabc_mode);
			}
		}
	}
#endif /*VENDOR_EDIT*/
		//send_samsung_fit_cmd(samsung_oncmd_0,sizeof(samsung_oncmd_0)/sizeof(struct dsi_cmd_desc));
	//send_samsung_fit_cmd(samsung_oncmd_1,sizeof(samsung_oncmd_1)/sizeof(struct dsi_cmd_desc));
#ifdef VENDOR_EDIT
/* Xiaori.Yuan@Mobile Phone Software Dept.Driver, 2014/08/02  Add for ESD */
	if(is_project(OPPO_14005)){
		enable_irq(irq);
		samsung_te_check_count = 0;
		send_samsung_fit_cmd(samsung_oncmd_0,sizeof(samsung_oncmd_0)/sizeof(struct dsi_cmd_desc));
		mdelay(40);
		//pr_err("%s delta = %d\n",__func__,delta);
		//disable_irq(irq);
		spin_lock_irqsave(&delta_lock,flags);
		for(i=0;i<90;i++){
			if(delta > samsung_lut_1[i][0]){
				break;
			}
		}
		spin_unlock_irqrestore(&delta_lock,flags);
		i--;
		if(i<0){i=67;}
		//pr_err("i=%d\n",i);
		samsung_fit_1[1] = samsung_lut_1[i][1];
		//pr_err("%s delta = %d samsung_fit_1[1] = %x\n",__func__,delta,samsung_fit_1[1]);
		send_samsung_fit_cmd( samsung_fit,sizeof(samsung_fit)/sizeof(struct dsi_cmd_desc));
		//enable_irq(irq);
		mdelay(60);
		spin_lock_irqsave(&delta_lock,flags);
		for(i=0;i<67;i++){
			if(delta > samsung_lut_2[i][0]){
				break;
			}
		}
		spin_unlock_irqrestore(&delta_lock,flags);
		i--;
		if(i<0){i=33;}
		//pr_err("i=%d\n",i);
		disable_irq(irq);
		samsung_fit_1[1] +=	samsung_lut_2[i][1];
		send_samsung_fit_cmd( samsung_fit,sizeof(samsung_fit)/sizeof(struct dsi_cmd_desc));
		pr_err("samsung_fit_1[1]=%x\n",samsung_fit_1[1]);
		samsung_te_check_count = 2;
		send_samsung_fit_cmd(samsung_oncmd_1,sizeof(samsung_oncmd_1)/sizeof(struct dsi_cmd_desc));
		//pr_err("get_boot_mode= %d  cont_splash_enabled = %d\n",get_boot_mode(),cont_splash_flag);
		if(MSM_BOOT_MODE__NORMAL == get_boot_mode() && cont_splash_flag == true && flag_first_set_bl==false && samsung_fit_err == false){
						send_samsung_fit_cmd(&samsung_oncmd_2,1); //f0 5a 5a
						//mdss_dsi_panel_cmd_read(gl_ctrl_pdata,0xc9,0x00,NULL,read_samsung_c9,132);      //read c9
						send_samsung_fit_cmd(&samsung_oncmd_4,1); // 0xb1, 0x00, 0x00, 0x42, 0xff, 0x04, 0x13, 0x2b, 0x42, 0x78, 0xb2, 0xff
						send_samsung_fit_cmd(&samsung_oncmd_5,1); //0xb2
						send_samsung_fit_cmd(&samsung_oncmd_6,1);  //0xf7
						send_samsung_fit_cmd(&samsung_oncmd_c9,1); //send c9
						send_samsung_fit_cmd(samsung_oncmd_7,sizeof(samsung_oncmd_7)/sizeof(struct dsi_cmd_desc)); //change aid cycle 1 to cycle 8
						send_samsung_fit_cmd(&samsung_oncmd_3,1); //f0 a5 a5
		}
		if(acl_mode != ACL_LEVEL_0){
				set_acl_resume_mode(acl_mode);
		}
	}
#endif /*VENDOR_EDIT*/
	if(ESD_TE_TEST){
		if(first_run_reset==1 && !cont_splash_flag){
			first_run_reset=0;
		}
		else{
			schedule_delayed_work(&techeck_work, msecs_to_jiffies(3000));
		}
	}
	
#ifdef VENDOR_EDIT
/* Xiaori.Yuan@Mobile Phone Software Dept.Driver, 2014/07/28  Add for lcd acl and hbm mode */
	mutex_lock(&lcd_mutex);
	flag_lcd_off = false;
	mutex_unlock(&lcd_mutex);
#endif /*VENDOR_EDIT*/
	pr_err("%s:-\n", __func__);
	return 0;
}

static int mdss_dsi_panel_off(struct mdss_panel_data *pdata)
{
	struct mipi_panel_info *mipi;
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	pr_err("%s: ctrl=%p ndx=%d\n", __func__, ctrl, ctrl->ndx);

	mipi  = &pdata->panel_info.mipi;

	if (ctrl->off_cmds.cmd_cnt)
		mdss_dsi_panel_cmds_send(ctrl, &ctrl->off_cmds);

#ifdef VENDOR_EDIT
/* YongPeng.Yi@SWDP.MultiMedia, 2015/03/11  Add for 15005 RTC597125 TEST START */
if(is_project(OPPO_15005)&&RTC597125_15005DEBUG)
	pr_err("%s: after send off cmd\n",__func__);
/*  YongPeng.Yi@SWDP.MultiMedia END */
#endif /*VENDOR_EDIT*/
#ifdef VENDOR_EDIT
/* Xiaori.Yuan@Mobile Phone Software Dept.Driver, 2014/08/02  Add for ESD */
	if(ESD_TE_TEST){
		cancel_delayed_work_sync(&techeck_work);	 
		 	mdelay(6);    
	}
#endif /*VENDOR_EDIT*/
	pr_err("%s: before mutex_lock\n",__func__);
#ifdef VENDOR_EDIT
/* Xiaori.Yuan@Mobile Phone Software Dept.Driver, 2014/07/28  Add for lcd acl and hbm mode */
	mutex_lock(&lcd_mutex);
#ifdef VENDOR_EDIT
/* YongPeng.Yi@SWDP.MultiMedia, 2015/03/11  Add for 15005 RTC597125 TEST START */
if(is_project(OPPO_15005)&&RTC597125_15005DEBUG)
	pr_err("%s: in mutex_lock\n",__func__);
/*  YongPeng.Yi@SWDP.MultiMedia END */
#endif /*VENDOR_EDIT*/
	flag_lcd_off = true;
	mutex_unlock(&lcd_mutex);
#endif /*VENDOR_EDIT*/
	pr_err("%s:-\n", __func__);
	return 0;
}

static void mdss_dsi_parse_lane_swap(struct device_node *np, char *dlane_swap)
{
	const char *data;

	*dlane_swap = DSI_LANE_MAP_0123;
	data = of_get_property(np, "qcom,mdss-dsi-lane-map", NULL);
	if (data) {
		if (!strcmp(data, "lane_map_3012"))
			*dlane_swap = DSI_LANE_MAP_3012;
		else if (!strcmp(data, "lane_map_2301"))
			*dlane_swap = DSI_LANE_MAP_2301;
		else if (!strcmp(data, "lane_map_1230"))
			*dlane_swap = DSI_LANE_MAP_1230;
		else if (!strcmp(data, "lane_map_0321"))
			*dlane_swap = DSI_LANE_MAP_0321;
		else if (!strcmp(data, "lane_map_1032"))
			*dlane_swap = DSI_LANE_MAP_1032;
		else if (!strcmp(data, "lane_map_2103"))
			*dlane_swap = DSI_LANE_MAP_2103;
		else if (!strcmp(data, "lane_map_3210"))
			*dlane_swap = DSI_LANE_MAP_3210;
	}
}

static void mdss_dsi_parse_trigger(struct device_node *np, char *trigger,
		char *trigger_key)
{
	const char *data;

	*trigger = DSI_CMD_TRIGGER_SW;
	data = of_get_property(np, trigger_key, NULL);
	if (data) {
		if (!strcmp(data, "none"))
			*trigger = DSI_CMD_TRIGGER_NONE;
		else if (!strcmp(data, "trigger_te"))
			*trigger = DSI_CMD_TRIGGER_TE;
		else if (!strcmp(data, "trigger_sw_seof"))
			*trigger = DSI_CMD_TRIGGER_SW_SEOF;
		else if (!strcmp(data, "trigger_sw_te"))
			*trigger = DSI_CMD_TRIGGER_SW_TE;
	}
}


static int mdss_dsi_parse_dcs_cmds(struct device_node *np,
		struct dsi_panel_cmds *pcmds, char *cmd_key, char *link_key)
{
	const char *data;
	int blen = 0, len;
	char *buf, *bp;
	struct dsi_ctrl_hdr *dchdr;
	int i, cnt;

	data = of_get_property(np, cmd_key, &blen);
	if (!data) {
		pr_err("%s: failed, key=%s\n", __func__, cmd_key);
		return -ENOMEM;
	}

	buf = kzalloc(sizeof(char) * blen, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	memcpy(buf, data, blen);

	/* scan dcs commands */
	bp = buf;
	len = blen;
	cnt = 0;
	while (len >= sizeof(*dchdr)) {
		dchdr = (struct dsi_ctrl_hdr *)bp;
		dchdr->dlen = ntohs(dchdr->dlen);
		if (dchdr->dlen > len) {
			pr_err("%s: dtsi cmd=%x error, len=%d",
				__func__, dchdr->dtype, dchdr->dlen);
			goto exit_free;
		}
		bp += sizeof(*dchdr);
		len -= sizeof(*dchdr);
		bp += dchdr->dlen;
		len -= dchdr->dlen;
		cnt++;
	}

	if (len != 0) {
		pr_err("%s: dcs_cmd=%x len=%d error!",
				__func__, buf[0], blen);
		goto exit_free;
	}

	pcmds->cmds = kzalloc(cnt * sizeof(struct dsi_cmd_desc),
						GFP_KERNEL);
	if (!pcmds->cmds)
		goto exit_free;

	pcmds->cmd_cnt = cnt;
	pcmds->buf = buf;
	pcmds->blen = blen;

	bp = buf;
	len = blen;
	for (i = 0; i < cnt; i++) {
		dchdr = (struct dsi_ctrl_hdr *)bp;
		len -= sizeof(*dchdr);
		bp += sizeof(*dchdr);
		pcmds->cmds[i].dchdr = *dchdr;
		pcmds->cmds[i].payload = bp;
		bp += dchdr->dlen;
		len -= dchdr->dlen;
	}

	/*Set default link state to LP Mode*/
	pcmds->link_state = DSI_LP_MODE;

	if (link_key) {
		data = of_get_property(np, link_key, NULL);
		if (data && !strcmp(data, "dsi_hs_mode"))
			pcmds->link_state = DSI_HS_MODE;
		else
			pcmds->link_state = DSI_LP_MODE;
	}

	pr_debug("%s: dcs_cmd=%x len=%d, cmd_cnt=%d link_state=%d\n", __func__,
		pcmds->buf[0], pcmds->blen, pcmds->cmd_cnt, pcmds->link_state);

	return 0;

exit_free:
	kfree(buf);
	return -ENOMEM;
}


int mdss_panel_get_dst_fmt(u32 bpp, char mipi_mode, u32 pixel_packing,
				char *dst_format)
{
	int rc = 0;
	switch (bpp) {
	case 3:
		*dst_format = DSI_CMD_DST_FORMAT_RGB111;
		break;
	case 8:
		*dst_format = DSI_CMD_DST_FORMAT_RGB332;
		break;
	case 12:
		*dst_format = DSI_CMD_DST_FORMAT_RGB444;
		break;
	case 16:
		switch (mipi_mode) {
		case DSI_VIDEO_MODE:
			*dst_format = DSI_VIDEO_DST_FORMAT_RGB565;
			break;
		case DSI_CMD_MODE:
			*dst_format = DSI_CMD_DST_FORMAT_RGB565;
			break;
		default:
			*dst_format = DSI_VIDEO_DST_FORMAT_RGB565;
			break;
		}
		break;
	case 18:
		switch (mipi_mode) {
		case DSI_VIDEO_MODE:
			if (pixel_packing == 0)
				*dst_format = DSI_VIDEO_DST_FORMAT_RGB666;
			else
				*dst_format = DSI_VIDEO_DST_FORMAT_RGB666_LOOSE;
			break;
		case DSI_CMD_MODE:
			*dst_format = DSI_CMD_DST_FORMAT_RGB666;
			break;
		default:
			if (pixel_packing == 0)
				*dst_format = DSI_VIDEO_DST_FORMAT_RGB666;
			else
				*dst_format = DSI_VIDEO_DST_FORMAT_RGB666_LOOSE;
			break;
		}
		break;
	case 24:
		switch (mipi_mode) {
		case DSI_VIDEO_MODE:
			*dst_format = DSI_VIDEO_DST_FORMAT_RGB888;
			break;
		case DSI_CMD_MODE:
			*dst_format = DSI_CMD_DST_FORMAT_RGB888;
			break;
		default:
			*dst_format = DSI_VIDEO_DST_FORMAT_RGB888;
			break;
		}
		break;
	default:
		rc = -EINVAL;
		break;
	}
	return rc;
}


static int mdss_dsi_parse_fbc_params(struct device_node *np,
				struct mdss_panel_info *panel_info)
{
	int rc, fbc_enabled = 0;
	u32 tmp;

	fbc_enabled = of_property_read_bool(np,	"qcom,mdss-dsi-fbc-enable");
	if (fbc_enabled) {
		pr_debug("%s:%d FBC panel enabled.\n", __func__, __LINE__);
		panel_info->fbc.enabled = 1;
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-bpp", &tmp);
		panel_info->fbc.target_bpp =	(!rc ? tmp : panel_info->bpp);
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-packing",
				&tmp);
		panel_info->fbc.comp_mode = (!rc ? tmp : 0);
		panel_info->fbc.qerr_enable = of_property_read_bool(np,
			"qcom,mdss-dsi-fbc-quant-error");
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-bias", &tmp);
		panel_info->fbc.cd_bias = (!rc ? tmp : 0);
		panel_info->fbc.pat_enable = of_property_read_bool(np,
				"qcom,mdss-dsi-fbc-pat-mode");
		panel_info->fbc.vlc_enable = of_property_read_bool(np,
				"qcom,mdss-dsi-fbc-vlc-mode");
		panel_info->fbc.bflc_enable = of_property_read_bool(np,
				"qcom,mdss-dsi-fbc-bflc-mode");
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-h-line-budget",
				&tmp);
		panel_info->fbc.line_x_budget = (!rc ? tmp : 0);
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-budget-ctrl",
				&tmp);
		panel_info->fbc.block_x_budget = (!rc ? tmp : 0);
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-block-budget",
				&tmp);
		panel_info->fbc.block_budget = (!rc ? tmp : 0);
		rc = of_property_read_u32(np,
				"qcom,mdss-dsi-fbc-lossless-threshold", &tmp);
		panel_info->fbc.lossless_mode_thd = (!rc ? tmp : 0);
		rc = of_property_read_u32(np,
				"qcom,mdss-dsi-fbc-lossy-threshold", &tmp);
		panel_info->fbc.lossy_mode_thd = (!rc ? tmp : 0);
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-rgb-threshold",
				&tmp);
		panel_info->fbc.lossy_rgb_thd = (!rc ? tmp : 0);
		rc = of_property_read_u32(np,
				"qcom,mdss-dsi-fbc-lossy-mode-idx", &tmp);
		panel_info->fbc.lossy_mode_idx = (!rc ? tmp : 0);
	} else {
		pr_debug("%s:%d Panel does not support FBC.\n",
				__func__, __LINE__);
		panel_info->fbc.enabled = 0;
		panel_info->fbc.target_bpp =
			panel_info->bpp;
	}
	return 0;
}

static void mdss_panel_parse_te_params(struct device_node *np,
				       struct mdss_panel_info *panel_info)
{

	u32 tmp;
	int rc = 0;
	/*
	 * TE default: dsi byte clock calculated base on 70 fps;
	 * around 14 ms to complete a kickoff cycle if te disabled;
	 * vclk_line base on 60 fps; write is faster than read;
	 * init == start == rdptr;
	 */
	panel_info->te.tear_check_en =
		!of_property_read_bool(np, "qcom,mdss-tear-check-disable");
	rc = of_property_read_u32
		(np, "qcom,mdss-tear-check-sync-cfg-height", &tmp);
	panel_info->te.sync_cfg_height = (!rc ? tmp : 0xfff0);
	rc = of_property_read_u32
		(np, "qcom,mdss-tear-check-sync-init-val", &tmp);
	panel_info->te.vsync_init_val = (!rc ? tmp : panel_info->yres);
	rc = of_property_read_u32
		(np, "qcom,mdss-tear-check-sync-threshold-start", &tmp);
	panel_info->te.sync_threshold_start = (!rc ? tmp : 4);
	rc = of_property_read_u32
		(np, "qcom,mdss-tear-check-sync-threshold-continue", &tmp);
	panel_info->te.sync_threshold_continue = (!rc ? tmp : 4);
	rc = of_property_read_u32(np, "qcom,mdss-tear-check-start-pos", &tmp);
	panel_info->te.start_pos = (!rc ? tmp : panel_info->yres);
	rc = of_property_read_u32
		(np, "qcom,mdss-tear-check-rd-ptr-trigger-intr", &tmp);
	panel_info->te.rd_ptr_irq = (!rc ? tmp : panel_info->yres + 1);
	rc = of_property_read_u32(np, "qcom,mdss-tear-check-frame-rate", &tmp);
	panel_info->te.refx100 = (!rc ? tmp : 6000);
}


static int mdss_dsi_parse_reset_seq(struct device_node *np,
		u32 rst_seq[MDSS_DSI_RST_SEQ_LEN], u32 *rst_len,
		const char *name)
{
	int num = 0, i;
	int rc;
	struct property *data;
	u32 tmp[MDSS_DSI_RST_SEQ_LEN];
	*rst_len = 0;
	data = of_find_property(np, name, &num);
	num /= sizeof(u32);
	if (!data || !num || num > MDSS_DSI_RST_SEQ_LEN || num % 2) {
		pr_debug("%s:%d, error reading %s, length found = %d\n",
			__func__, __LINE__, name, num);
	} else {
		rc = of_property_read_u32_array(np, name, tmp, num);
		if (rc)
			pr_debug("%s:%d, error reading %s, rc = %d\n",
				__func__, __LINE__, name, rc);
		else {
			for (i = 0; i < num; ++i)
				rst_seq[i] = tmp[i];
			*rst_len = num;
		}
	}
	return 0;
}

static int mdss_dsi_gen_read_status(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
#ifdef VENDOR_EDIT
//guoling@MM.lcddriver add for tm panel
/* YongPeng.Yi@PhoneSW.Multimedia, 2015/02/10 Add for 15005 LCD ESD */
    if((lcd_dev == LCD_BOE_HX8379S) || (lcd_dev == LCD_15005_TRULY_HX8379C)){
        if (ctrl_pdata->status_buf.data[0] != 0x80
            || ctrl_pdata->status_buf.data[1] != 0x73
            || ctrl_pdata->status_buf.data[2] != 0x06){
            pr_err("%s: Read back value from panel reg1 %x reg2 %x reg3 %x reg4 %x\n",
							__func__,ctrl_pdata->status_buf.data[0],ctrl_pdata->status_buf.data[1]
							,ctrl_pdata->status_buf.data[2],ctrl_pdata->status_buf.data[3]);
			lcd_reset = true;
			return -EINVAL;
        }else{
            return 1;
        }
    }else if(lcd_dev == LCD_TM_HX8392B){
        if (ctrl_pdata->status_buf.data[0] != 0x80
            || ctrl_pdata->status_buf.data[1] != 0x73
            || ctrl_pdata->status_buf.data[2] != 0x04
            || ctrl_pdata->status_buf.data[3] != 0){
            pr_err("%s: Read back value from panel reg1 %x reg2 %x reg3 %x reg4 %x\n",
							__func__,ctrl_pdata->status_buf.data[0],ctrl_pdata->status_buf.data[1]
							,ctrl_pdata->status_buf.data[2],ctrl_pdata->status_buf.data[3]);
			lcd_reset = true;
			return -EINVAL;
        }else{
            return 1;
        }
    }else if(lcd_dev == LCD_HX8394_TRULY_P3_MURA){
	        if (ctrl_pdata->status_buf.data[0] != 0x80
            || ctrl_pdata->status_buf.data[1] != 0x73
            || ctrl_pdata->status_buf.data[2] != 0x06
            || ctrl_pdata->status_buf.data[3] != 0){
            pr_err("%s: Read back value from panel reg1 %x reg2 %x reg3 %x reg4 %x\n",
							__func__,ctrl_pdata->status_buf.data[0],ctrl_pdata->status_buf.data[1]
							,ctrl_pdata->status_buf.data[2],ctrl_pdata->status_buf.data[3]);
			lcd_reset = true;
			return -EINVAL;
	        }else{
	            return 1;
	        }
    }else if(lcd_dev == LCD_JDI_NT35521){
			/* YongPeng.Yi@PhoneSW.Multimedia, 2014/12/25 Add for 14037 JDI LCD ESD */
			if (ctrl_pdata->status_buf.data[0] != 0x9C
			|| ctrl_pdata->status_buf.data[1] != 0x0
			|| ctrl_pdata->status_buf.data[2] != 0x0
			|| ctrl_pdata->status_buf.data[3] != 0x0){
			pr_err("lcd_jdi_NT35521 enter ESD RST\n");
			pr_err("%s: Read back value from panel is incorrect reg1 %x reg2 %x reg3 %x reg4 %x\n",
			__func__,ctrl_pdata->status_buf.data[0],ctrl_pdata->status_buf.data[1],
					ctrl_pdata->status_buf.data[2],ctrl_pdata->status_buf.data[3]);
			lcd_reset = true;
			return -EINVAL;
        }else{
            return 1;
        }
    }
	else{
#endif
    	if (ctrl_pdata->status_buf.data[0] !=
    					ctrl_pdata->status_value) {
			pr_err("%s: Read back value from panel is incorrect value is =  %x\n", __func__, ctrl_pdata->status_value);
#ifdef VENDOR_EDIT
//guoling@MM.lcddriver add for lcd ESD check flag
    		lcd_reset = true;
#endif
    		return -EINVAL;
    	} else {
    		return 1;
    	}
#ifdef VENDOR_EDIT
//guoling@MM.lcddriver add for tm panel
   }
#endif
}

static int mdss_dsi_nt35596_read_status(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	if (ctrl_pdata->status_buf.data[0] !=
					ctrl_pdata->status_value) {
		ctrl_pdata->status_error_count = 0;
		pr_err("%s: Read back value from panel is incorrect\n",
							__func__);
		return -EINVAL;
	} else {
		if (ctrl_pdata->status_buf.data[3] != NT35596_BUF_3_STATUS) {
			ctrl_pdata->status_error_count = 0;
		} else {
			if ((ctrl_pdata->status_buf.data[4] ==
				NT35596_BUF_4_STATUS) ||
				(ctrl_pdata->status_buf.data[5] ==
				NT35596_BUF_5_STATUS))
				ctrl_pdata->status_error_count = 0;
			else
				ctrl_pdata->status_error_count++;
			if (ctrl_pdata->status_error_count >=
					NT35596_MAX_ERR_CNT) {
				ctrl_pdata->status_error_count = 0;
				pr_err("%s: Read value bad. Error_cnt = %i\n",
					 __func__,
					ctrl_pdata->status_error_count);
				return -EINVAL;
			}
		}
		return 1;
	}
}

static void mdss_dsi_parse_roi_alignment(struct device_node *np,
		struct mdss_panel_info *pinfo)
{
	int len = 0;
	u32 value[6];
	struct property *data;
	data = of_find_property(np, "qcom,panel-roi-alignment", &len);
	len /= sizeof(u32);
	if (!data || (len != 6)) {
		pr_debug("%s: Panel roi alignment not found", __func__);
	} else {
		int rc = of_property_read_u32_array(np,
				"qcom,panel-roi-alignment", value, len);
		if (rc)
			pr_debug("%s: Error reading panel roi alignment values",
					__func__);
		else {
			pinfo->xstart_pix_align = value[0];
			pinfo->width_pix_align = value[1];
			pinfo->ystart_pix_align = value[2];
			pinfo->height_pix_align = value[3];
			pinfo->min_width = value[4];
			pinfo->min_height = value[5];
		}

		pr_debug("%s: ROI alignment: [%d, %d, %d, %d, %d, %d]",
				__func__, pinfo->xstart_pix_align,
				pinfo->width_pix_align, pinfo->ystart_pix_align,
				pinfo->height_pix_align, pinfo->min_width,
				pinfo->min_height);
	}
}

static int mdss_dsi_parse_panel_features(struct device_node *np,
	struct mdss_dsi_ctrl_pdata *ctrl)
{
	struct mdss_panel_info *pinfo;

	if (!np || !ctrl) {
		pr_err("%s: Invalid arguments\n", __func__);
		return -ENODEV;
	}

	pinfo = &ctrl->panel_data.panel_info;

	pinfo->cont_splash_enabled = of_property_read_bool(np,
		"qcom,cont-splash-enabled");

	pinfo->partial_update_enabled = of_property_read_bool(np,
		"qcom,partial-update-enabled");
	pr_info("%s:%d Partial update %s\n", __func__, __LINE__,
		(pinfo->partial_update_enabled ? "enabled" : "disabled"));
	if (pinfo->partial_update_enabled)
		ctrl->partial_update_fnc = mdss_dsi_panel_partial_update;

	pinfo->ulps_feature_enabled = of_property_read_bool(np,
		"qcom,ulps-enabled");
	pr_info("%s: ulps feature %s", __func__,
		(pinfo->ulps_feature_enabled ? "enabled" : "disabled"));
#ifdef VENDOR_EDIT
/* Xiaori.Yuan@Mobile Phone Software Dept.Driver, 2015/04/08  Add for video mode ulps */
	pinfo->ulps_suspend_enabled = of_property_read_bool(np,
		"qcom,suspend-ulps-enabled");
	pr_info("%s: ulps during suspend feature %s", __func__,
		(pinfo->ulps_suspend_enabled ? "enabled" : "disabled"));
#endif /*VENDOR_EDIT*/
	pinfo->esd_check_enabled = of_property_read_bool(np,
		"qcom,esd-check-enabled");

	pinfo->mipi.dynamic_switch_enabled = of_property_read_bool(np,
		"qcom,dynamic-mode-switch-enabled");

	if (pinfo->mipi.dynamic_switch_enabled) {
		mdss_dsi_parse_dcs_cmds(np, &ctrl->video2cmd,
			"qcom,video-to-cmd-mode-switch-commands", NULL);

		mdss_dsi_parse_dcs_cmds(np, &ctrl->cmd2video,
			"qcom,cmd-to-video-mode-switch-commands", NULL);

		if (!ctrl->video2cmd.cmd_cnt || !ctrl->cmd2video.cmd_cnt) {
			pr_warn("No commands specified for dynamic switch\n");
			pinfo->mipi.dynamic_switch_enabled = 0;
		}
	}

	pr_info("%s: dynamic switch feature enabled: %d", __func__,
		pinfo->mipi.dynamic_switch_enabled);

	return 0;
}

static int mdss_dsi_set_refresh_rate_range(struct device_node *pan_node,
		struct mdss_panel_info *pinfo)
{
	int rc = 0;
	rc = of_property_read_u32(pan_node,
			"qcom,mdss-dsi-min-refresh-rate",
			&pinfo->min_fps);
	if (rc) {
		pr_warn("%s:%d, Unable to read min refresh rate\n",
				__func__, __LINE__);

		/*
		 * Since min refresh rate is not specified when dynamic
		 * fps is enabled, using minimum as 30
		 */
		pinfo->min_fps = MIN_REFRESH_RATE;
		rc = 0;
	}

	rc = of_property_read_u32(pan_node,
			"qcom,mdss-dsi-max-refresh-rate",
			&pinfo->max_fps);
	if (rc) {
		pr_warn("%s:%d, Unable to read max refresh rate\n",
				__func__, __LINE__);

		/*
		 * Since max refresh rate was not specified when dynamic
		 * fps is enabled, using the default panel refresh rate
		 * as max refresh rate supported.
		 */
		pinfo->max_fps = pinfo->mipi.frame_rate;
		rc = 0;
	}

	pr_info("dyn_fps: min = %d, max = %d\n",
			pinfo->min_fps, pinfo->max_fps);
	return rc;
}

static void mdss_dsi_parse_dfps_config(struct device_node *pan_node,
			struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	const char *data;
	bool dynamic_fps;
	struct mdss_panel_info *pinfo = &(ctrl_pdata->panel_data.panel_info);

	dynamic_fps = of_property_read_bool(pan_node,
			"qcom,mdss-dsi-pan-enable-dynamic-fps");

	if (!dynamic_fps)
		return;

	pinfo->dynamic_fps = true;
	data = of_get_property(pan_node, "qcom,mdss-dsi-pan-fps-update", NULL);
	if (data) {
		if (!strcmp(data, "dfps_suspend_resume_mode")) {
			pinfo->dfps_update = DFPS_SUSPEND_RESUME_MODE;
			pr_debug("dfps mode: suspend/resume\n");
		} else if (!strcmp(data, "dfps_immediate_clk_mode")) {
			pinfo->dfps_update = DFPS_IMMEDIATE_CLK_UPDATE_MODE;
			pr_debug("dfps mode: Immediate clk\n");
		} else if (!strcmp(data, "dfps_immediate_porch_mode")) {
			pinfo->dfps_update = DFPS_IMMEDIATE_PORCH_UPDATE_MODE;
			pr_debug("dfps mode: Immediate porch\n");
		} else {
			pinfo->dfps_update = DFPS_SUSPEND_RESUME_MODE;
			pr_debug("default dfps mode: suspend/resume\n");
		}
		mdss_dsi_set_refresh_rate_range(pan_node, pinfo);
	} else {
		pinfo->dynamic_fps = false;
		pr_debug("dfps update mode not configured: disable\n");
	}
	pinfo->new_fps = pinfo->mipi.frame_rate;

	return;
}

static int mdss_panel_parse_dt(struct device_node *np,
			struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	u32 tmp;
	int rc, i, len;
	const char *data;
	static const char *pdest;
	struct mdss_panel_info *pinfo = &(ctrl_pdata->panel_data.panel_info);

	rc = of_property_read_u32(np, "qcom,mdss-dsi-panel-width", &tmp);
	if (rc) {
		pr_err("%s:%d, panel width not specified\n",
						__func__, __LINE__);
		return -EINVAL;
	}
	pinfo->xres = (!rc ? tmp : 640);

	rc = of_property_read_u32(np, "qcom,mdss-dsi-panel-height", &tmp);
	if (rc) {
		pr_err("%s:%d, panel height not specified\n",
						__func__, __LINE__);
		return -EINVAL;
	}
	pinfo->yres = (!rc ? tmp : 480);

	rc = of_property_read_u32(np,
		"qcom,mdss-pan-physical-width-dimension", &tmp);
	pinfo->physical_width = (!rc ? tmp : 0);
	rc = of_property_read_u32(np,
		"qcom,mdss-pan-physical-height-dimension", &tmp);
	pinfo->physical_height = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-left-border", &tmp);
	pinfo->lcdc.xres_pad = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-right-border", &tmp);
	if (!rc)
		pinfo->lcdc.xres_pad += tmp;
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-top-border", &tmp);
	pinfo->lcdc.yres_pad = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-bottom-border", &tmp);
	if (!rc)
		pinfo->lcdc.yres_pad += tmp;
	rc = of_property_read_u32(np, "qcom,mdss-dsi-bpp", &tmp);
	if (rc) {
		pr_err("%s:%d, bpp not specified\n", __func__, __LINE__);
		return -EINVAL;
	}
	pinfo->bpp = (!rc ? tmp : 24);
	pinfo->mipi.mode = DSI_VIDEO_MODE;
	data = of_get_property(np, "qcom,mdss-dsi-panel-type", NULL);
	if (data && !strncmp(data, "dsi_cmd_mode", 12))
		pinfo->mipi.mode = DSI_CMD_MODE;
	tmp = 0;
	data = of_get_property(np, "qcom,mdss-dsi-pixel-packing", NULL);
	if (data && !strcmp(data, "loose"))
		pinfo->mipi.pixel_packing = 1;
	else
		pinfo->mipi.pixel_packing = 0;
	rc = mdss_panel_get_dst_fmt(pinfo->bpp,
		pinfo->mipi.mode, pinfo->mipi.pixel_packing,
		&(pinfo->mipi.dst_format));
	if (rc) {
		pr_debug("%s: problem determining dst format. Set Default\n",
			__func__);
		pinfo->mipi.dst_format =
			DSI_VIDEO_DST_FORMAT_RGB888;
	}
	pdest = of_get_property(np,
		"qcom,mdss-dsi-panel-destination", NULL);

	if (pdest) {
		if (strlen(pdest) != 9) {
			pr_err("%s: Unknown pdest specified\n", __func__);
			return -EINVAL;
		}
		if (!strcmp(pdest, "display_1"))
			pinfo->pdest = DISPLAY_1;
		else if (!strcmp(pdest, "display_2"))
			pinfo->pdest = DISPLAY_2;
		else {
			pr_debug("%s: incorrect pdest. Set Default\n",
				__func__);
			pinfo->pdest = DISPLAY_1;
		}
	} else {
		pr_debug("%s: pdest not specified. Set Default\n",
				__func__);
		pinfo->pdest = DISPLAY_1;
	}
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-front-porch", &tmp);
	pinfo->lcdc.h_front_porch = (!rc ? tmp : 6);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-back-porch", &tmp);
	pinfo->lcdc.h_back_porch = (!rc ? tmp : 6);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-pulse-width", &tmp);
	pinfo->lcdc.h_pulse_width = (!rc ? tmp : 2);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-sync-skew", &tmp);
	pinfo->lcdc.hsync_skew = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-back-porch", &tmp);
	pinfo->lcdc.v_back_porch = (!rc ? tmp : 6);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-front-porch", &tmp);
	pinfo->lcdc.v_front_porch = (!rc ? tmp : 6);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-pulse-width", &tmp);
	pinfo->lcdc.v_pulse_width = (!rc ? tmp : 2);
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-underflow-color", &tmp);
	pinfo->lcdc.underflow_clr = (!rc ? tmp : 0xff);
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-border-color", &tmp);
	pinfo->lcdc.border_clr = (!rc ? tmp : 0);
//#ifndef VENDOR_EDIT
/* Xiaori.Yuan@Mobile Phone Software Dept.Driver, 2014/07/21  Add for LCD rotate 180 degree */
	pinfo->is_panel_inverted = of_property_read_bool(np,		
		"qcom,mdss-dsi-panel-inverted");
//#endif /*VENDOR_EDIT*/
	data = of_get_property(np, "qcom,mdss-dsi-panel-orientation", NULL);
	if (data) {
		pr_debug("panel orientation is %s\n", data);
		if (!strcmp(data, "180"))
			pinfo->panel_orientation = MDP_ROT_180;
		else if (!strcmp(data, "hflip"))
			pinfo->panel_orientation = MDP_FLIP_LR;
		else if (!strcmp(data, "vflip"))
			pinfo->panel_orientation = MDP_FLIP_UD;
	}

	ctrl_pdata->bklt_ctrl = UNKNOWN_CTRL;
	data = of_get_property(np, "qcom,mdss-dsi-bl-pmic-control-type", NULL);
	if (data) {
		if (!strncmp(data, "bl_ctrl_wled", 12)) {
			led_trigger_register_simple("bkl-trigger",
				&bl_led_trigger);
			pr_debug("%s: SUCCESS-> WLED TRIGGER register\n",
				__func__);
			ctrl_pdata->bklt_ctrl = BL_WLED;
		} else if (!strncmp(data, "bl_ctrl_pwm", 11)) {
			ctrl_pdata->bklt_ctrl = BL_PWM;
			rc = of_property_read_u32(np,
				"qcom,mdss-dsi-bl-pmic-pwm-frequency", &tmp);
			if (rc) {
				pr_err("%s:%d, Error, panel pwm_period\n",
						__func__, __LINE__);
				return -EINVAL;
			}
			ctrl_pdata->pwm_period = tmp;
			rc = of_property_read_u32(np,
				"qcom,mdss-dsi-bl-pmic-bank-select", &tmp);
			if (rc) {
				pr_err("%s:%d, Error, dsi lpg channel\n",
						__func__, __LINE__);
				return -EINVAL;
			}
			ctrl_pdata->pwm_lpg_chan = tmp;
			tmp = of_get_named_gpio(np,
				"qcom,mdss-dsi-pwm-gpio", 0);
			ctrl_pdata->pwm_pmic_gpio = tmp;
		} else if (!strncmp(data, "bl_ctrl_dcs", 11)) {
			ctrl_pdata->bklt_ctrl = BL_DCS_CMD;
		}
	}
	rc = of_property_read_u32(np, "qcom,mdss-brightness-max-level", &tmp);
	pinfo->brightness_max = (!rc ? tmp : MDSS_MAX_BL_BRIGHTNESS);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-bl-min-level", &tmp);
	pinfo->bl_min = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-bl-max-level", &tmp);
	pinfo->bl_max = (!rc ? tmp : 255);
	ctrl_pdata->bklt_max = pinfo->bl_max;

	rc = of_property_read_u32(np, "qcom,mdss-dsi-interleave-mode", &tmp);
	pinfo->mipi.interleave_mode = (!rc ? tmp : 0);

	pinfo->mipi.vsync_enable = of_property_read_bool(np,
		"qcom,mdss-dsi-te-check-enable");
	pinfo->mipi.hw_vsync_mode = of_property_read_bool(np,
		"qcom,mdss-dsi-te-using-te-pin");

	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-h-sync-pulse", &tmp);
	pinfo->mipi.pulse_mode_hsa_he = (!rc ? tmp : false);

	pinfo->mipi.hfp_power_stop = of_property_read_bool(np,
		"qcom,mdss-dsi-hfp-power-mode");
	pinfo->mipi.hsa_power_stop = of_property_read_bool(np,
		"qcom,mdss-dsi-hsa-power-mode");
	pinfo->mipi.hbp_power_stop = of_property_read_bool(np,
		"qcom,mdss-dsi-hbp-power-mode");
	pinfo->mipi.last_line_interleave_en = of_property_read_bool(np,
		"qcom,mdss-dsi-last-line-interleave");
	pinfo->mipi.bllp_power_stop = of_property_read_bool(np,
		"qcom,mdss-dsi-bllp-power-mode");
	pinfo->mipi.eof_bllp_power_stop = of_property_read_bool(
		np, "qcom,mdss-dsi-bllp-eof-power-mode");
	pinfo->mipi.traffic_mode = DSI_NON_BURST_SYNCH_PULSE;
	data = of_get_property(np, "qcom,mdss-dsi-traffic-mode", NULL);
	if (data) {
		if (!strcmp(data, "non_burst_sync_event"))
			pinfo->mipi.traffic_mode = DSI_NON_BURST_SYNCH_EVENT;
		else if (!strcmp(data, "burst_mode"))
			pinfo->mipi.traffic_mode = DSI_BURST_MODE;
	}
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-te-dcs-command", &tmp);
	pinfo->mipi.insert_dcs_cmd =
			(!rc ? tmp : 1);
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-wr-mem-continue", &tmp);
	pinfo->mipi.wr_mem_continue =
			(!rc ? tmp : 0x3c);
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-wr-mem-start", &tmp);
	pinfo->mipi.wr_mem_start =
			(!rc ? tmp : 0x2c);
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-te-pin-select", &tmp);
	pinfo->mipi.te_sel =
			(!rc ? tmp : 1);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-virtual-channel-id", &tmp);
	pinfo->mipi.vc = (!rc ? tmp : 0);
	pinfo->mipi.rgb_swap = DSI_RGB_SWAP_RGB;
	data = of_get_property(np, "qcom,mdss-dsi-color-order", NULL);
	if (data) {
		if (!strcmp(data, "rgb_swap_rbg"))
			pinfo->mipi.rgb_swap = DSI_RGB_SWAP_RBG;
		else if (!strcmp(data, "rgb_swap_bgr"))
			pinfo->mipi.rgb_swap = DSI_RGB_SWAP_BGR;
		else if (!strcmp(data, "rgb_swap_brg"))
			pinfo->mipi.rgb_swap = DSI_RGB_SWAP_BRG;
		else if (!strcmp(data, "rgb_swap_grb"))
			pinfo->mipi.rgb_swap = DSI_RGB_SWAP_GRB;
		else if (!strcmp(data, "rgb_swap_gbr"))
			pinfo->mipi.rgb_swap = DSI_RGB_SWAP_GBR;
	}
	pinfo->mipi.data_lane0 = of_property_read_bool(np,
		"qcom,mdss-dsi-lane-0-state");
	pinfo->mipi.data_lane1 = of_property_read_bool(np,
		"qcom,mdss-dsi-lane-1-state");
	pinfo->mipi.data_lane2 = of_property_read_bool(np,
		"qcom,mdss-dsi-lane-2-state");
	pinfo->mipi.data_lane3 = of_property_read_bool(np,
		"qcom,mdss-dsi-lane-3-state");

	rc = of_property_read_u32(np, "qcom,mdss-dsi-t-clk-pre", &tmp);
	pinfo->mipi.t_clk_pre = (!rc ? tmp : 0x24);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-t-clk-post", &tmp);
	pinfo->mipi.t_clk_post = (!rc ? tmp : 0x03);

	pinfo->mipi.rx_eot_ignore = of_property_read_bool(np,
		"qcom,mdss-dsi-rx-eot-ignore");
	pinfo->mipi.tx_eot_append = of_property_read_bool(np,
		"qcom,mdss-dsi-tx-eot-append");

	rc = of_property_read_u32(np, "qcom,mdss-dsi-stream", &tmp);
	pinfo->mipi.stream = (!rc ? tmp : 0);

	data = of_get_property(np, "qcom,mdss-dsi-panel-mode-gpio-state", NULL);
	if (data) {
		if (!strcmp(data, "high"))
			pinfo->mode_gpio_state = MODE_GPIO_HIGH;
		else if (!strcmp(data, "low"))
			pinfo->mode_gpio_state = MODE_GPIO_LOW;
	} else {
		pinfo->mode_gpio_state = MODE_GPIO_NOT_VALID;
	}

	rc = of_property_read_u32(np, "qcom,mdss-dsi-panel-framerate", &tmp);
	pinfo->mipi.frame_rate = (!rc ? tmp : 60);
#ifdef VENDOR_EDIT
/* Xiaori.Yuan@Mobile Phone Software Dept.Driver, 2014/09/13  Add for cmcc */
/*
#ifdef OPPO_CMCC_TEST
	if(is_project(14005)){
		pr_err("14005 kernel cmcc set framerate to 45\n");
		pinfo->mipi.frame_rate = 45;
	}
#endif
*/
#endif /*VENDOR_EDIT*/
	rc = of_property_read_u32(np, "qcom,mdss-dsi-panel-clockrate", &tmp);
	pinfo->clk_rate = (!rc ? tmp : 0);
	data = of_get_property(np, "qcom,mdss-dsi-panel-timings", &len);
	if ((!data) || (len != 12)) {
		pr_err("%s:%d, Unable to read Phy timing settings",
		       __func__, __LINE__);
		goto error;
	}
	for (i = 0; i < len; i++)
		pinfo->mipi.dsi_phy_db.timing[i] = data[i];

	pinfo->mipi.lp11_init = of_property_read_bool(np,
					"qcom,mdss-dsi-lp11-init");
	rc = of_property_read_u32(np, "qcom,mdss-dsi-init-delay-us", &tmp);
	pinfo->mipi.init_delay = (!rc ? tmp : 0);

	mdss_dsi_parse_roi_alignment(np, pinfo);

	mdss_dsi_parse_trigger(np, &(pinfo->mipi.mdp_trigger),
		"qcom,mdss-dsi-mdp-trigger");

	mdss_dsi_parse_trigger(np, &(pinfo->mipi.dma_trigger),
		"qcom,mdss-dsi-dma-trigger");

	mdss_dsi_parse_lane_swap(np, &(pinfo->mipi.dlane_swap));

	mdss_dsi_parse_fbc_params(np, pinfo);

	mdss_panel_parse_te_params(np, pinfo);

	mdss_dsi_parse_reset_seq(np, pinfo->rst_seq, &(pinfo->rst_seq_len),
		"qcom,mdss-dsi-reset-sequence");
#ifdef VENDOR_EDIT
	/* Xiaori.Yuan@Mobile Phone Software Dept.Driver, 2014/02/17  Add for set cabc */
		mdss_dsi_parse_dcs_cmds(np, &cabc_off_sequence,
			"qcom,mdss-dsi-cabc-off-command", "qcom,mdss-dsi-off-command-state");
		mdss_dsi_parse_dcs_cmds(np, &cabc_user_interface_image_sequence,
			"qcom,mdss-dsi-cabc-ui-command", "qcom,mdss-dsi-off-command-state");
		mdss_dsi_parse_dcs_cmds(np, &cabc_still_image_sequence,
			"qcom,mdss-dsi-cabc-still-image-command", "qcom,mdss-dsi-off-command-state");
		mdss_dsi_parse_dcs_cmds(np, &cabc_video_image_sequence,
			"qcom,mdss-dsi-cabc-video-command", "qcom,mdss-dsi-off-command-state");
#endif /*VENDOR_EDIT*/

#ifndef VENDOR_EDIT
/* Xiaori.Yuan@Mobile Phone Software Dept.Driver, 2015/02/09  Modify for cmcc EQ setting*/
	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->on_cmds,
		"qcom,mdss-dsi-on-command", "qcom,mdss-dsi-on-command-state");
#else /*VENDOR_EDIT*/
	if( is_project(OPPO_14045) && (lcd_dev == LCD_14045_17_VIDEO || lcd_dev == LCD_14045_17_CMD) && 1 == get_Operator_Version()){
		mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->on_cmds,
			"qcom,mdss-dsi-on-command-cmcc", "qcom,mdss-dsi-on-command-state");
	}else{
		mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->on_cmds,
			"qcom,mdss-dsi-on-command", "qcom,mdss-dsi-on-command-state");
	}
#endif /*VENDOR_EDIT*/

	/* Yongpeng.Yi@Mobile Phone Software Dept.Driver, 2015/04/28  Add for 14037 tianma lcd power on flick */
	if (is_project(OPPO_14037)&&(lcd_dev == LCD_TM_HX8392B)) {
		mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->spec_cmds,
			"oppo,mdss_dsi_spec_command", NULL);
	}

	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->off_cmds,
		"qcom,mdss-dsi-off-command", "qcom,mdss-dsi-off-command-state");

	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->status_cmds,
			"qcom,mdss-dsi-panel-status-command",
				"qcom,mdss-dsi-panel-status-command-state");
	rc = of_property_read_u32(np, "qcom,mdss-dsi-panel-status-value", &tmp);
	ctrl_pdata->status_value = (!rc ? tmp : 0);


	ctrl_pdata->status_mode = ESD_MAX;
	rc = of_property_read_string(np,
				"qcom,mdss-dsi-panel-status-check-mode", &data);
	if (!rc) {
		if (!strcmp(data, "bta_check")) {
			ctrl_pdata->status_mode = ESD_BTA;
		} else if (!strcmp(data, "reg_read")) {
			ctrl_pdata->status_mode = ESD_REG;
#ifdef VENDOR_EDIT
//guoling@MM.lcddriver add for ESD check to read reg for tima panel 2014-09-10
/*Yongpeng.Yi@PhoneMulti add for 14037 LCD_JDI_NT35521 ESD check*/
            if((lcd_dev == LCD_TM_NT35512S)||(lcd_dev == LCD_JDI_NT35521)||(lcd_dev == LCD_15025_TM_NT35512S)){
                ctrl_pdata->status_cmds_rlen = 0;
            }else if((lcd_dev == LCD_TM_HX8392B) || (lcd_dev == LCD_HX8394_TRULY_P3_MURA)){
                ctrl_pdata->status_cmds_rlen = 4;
            }else{
			    ctrl_pdata->status_cmds_rlen = 8;
			}
#else
            ctrl_pdata->status_cmds_rlen = 0;
#endif
			ctrl_pdata->check_read_status =
						mdss_dsi_gen_read_status;
		} else if (!strcmp(data, "reg_read_nt35596")) {
			ctrl_pdata->status_mode = ESD_REG_NT35596;
			ctrl_pdata->status_error_count = 0;
			ctrl_pdata->status_cmds_rlen = 8;
			ctrl_pdata->check_read_status =
						mdss_dsi_nt35596_read_status;
		}
	}

	rc = mdss_dsi_parse_panel_features(np, ctrl_pdata);
	if (rc) {
		pr_err("%s: failed to parse panel features\n", __func__);
		goto error;
	}


	mdss_dsi_parse_dfps_config(np, ctrl_pdata);

	return 0;

error:
	return -EINVAL;
}

int mdss_dsi_panel_init(struct device_node *node,
	struct mdss_dsi_ctrl_pdata *ctrl_pdata,
	bool cmd_cfg_cont_splash)
{
	int rc = 0;
	static const char *panel_name;
	struct mdss_panel_info *pinfo;
#ifdef VENDOR_EDIT
/* Xiaori.Yuan@Mobile Phone Software Dept.Driver, 2014/09/23  Add for registe panel info */
	static const char *panel_manufacture;
    static const char *panel_version;
#endif /*VENDOR_EDIT*/

	if (!node || !ctrl_pdata) {
		pr_err("%s: Invalid arguments\n", __func__);
		return -ENODEV;
	}
#ifdef VENDOR_EDIT
/* Xiaori.Yuan@Mobile Phone Software Dept.Driver, 2014/07/28  Add for lcd acl and hbm mode */
	gl_ctrl_pdata = ctrl_pdata;
#endif /*VENDOR_EDIT*/
	pinfo = &ctrl_pdata->panel_data.panel_info;

	pr_debug("%s:%d\n", __func__, __LINE__);
	panel_name = of_get_property(node, "qcom,mdss-dsi-panel-name", NULL);
	if (!panel_name)
		pr_info("%s:%d, Panel name not specified\n",
						__func__, __LINE__);
	else
		pr_info("%s: Panel Name = %s\n", __func__, panel_name);
	
#ifdef VENDOR_EDIT
/* Xiaori.Yuan@Mobile Phone Software Dept.Driver, 2014/09/23  Add for registe panel info */
	panel_manufacture = of_get_property(node, "qcom,mdss-dsi-panel-manufacture", NULL);
	if (!panel_manufacture)
		pr_info("%s:%d, panel manufacture not specified\n", __func__, __LINE__);
	else
		pr_info("%s: Panel Manufacture = %s\n", __func__, panel_manufacture);
	panel_version = of_get_property(node, "qcom,mdss-dsi-panel-version", NULL);
	if (!panel_version)
		pr_info("%s:%d, panel version not specified\n", __func__, __LINE__);
	else
		pr_info("%s: Panel Version = %s\n", __func__, panel_version);
	register_device_proc("lcd", (char *)panel_version, (char *)panel_manufacture);
#endif /*VENDOR_EDIT*/

#ifdef VENDOR_EDIT
//guoling@MM.lcddriver add for lcd device info
    if(!strcmp(panel_name,"oppo14043tm fwvga video mode dsi panel")){
		lcd_dev = LCD_TM_HX8379;
	}else if(!strcmp(panel_name,"oppo14043tm nt35512s fwvga video mode dsi panel")){
		lcd_dev = LCD_TM_NT35512S;
	}else if(!strcmp(panel_name,"oppo14043truly fwvga video mode dsi panel")){
	    lcd_dev = LCD_TRULY;
	}else if(!strcmp(panel_name,"oppo14043byd fwvga video mode dsi panel")){
	    lcd_dev = LCD_BYD;
	}else if(!strcmp(panel_name,"oppo14037truly hx8394a 720p video mode dsi panel")){		
		lcd_dev = LCD_HX8394_TRULY;
	}else if(!strcmp(panel_name,"oppo14037truly hx8394a p3 720p video mode dsi panel")){ 	
		lcd_dev = LCD_HX8394_TRULY_P3;
	}else if(!strcmp(panel_name,"oppo14037tianma 720p video mode dsi panel")){
	    lcd_dev = LCD_TM_HX8392B;
	}else if(!strcmp(panel_name,"oppo14037truly hx8394a p3 mura 720p video mode dsi panel")){
	    lcd_dev = LCD_HX8394_TRULY_P3_MURA;
	}else if(!strcmp(panel_name,"oppo14037jdi 720p_4lane video mode dsi panel")){
		lcd_dev = LCD_JDI_NT35521;
	}else if(!strcmp(panel_name,"oppo14017synaptics 720p video mode dsi panel")){
	    lcd_dev = LCD_14045_17_VIDEO;
		pr_err("oppo14017synaptics 720p video mode dsi panel\n");
	}else if(!strcmp(panel_name,"oppo14017synaptics 720p cmd mode dsi panel")){
	    lcd_dev = LCD_14045_17_CMD;
		pr_err("oppo14017synaptics 720p cmd mode dsi panel\n");
	}else if(!strcmp(panel_name,"oppo15005tm nt35512s fwvga video mode dsi panel")){  		/*YongPeng.Yi@PhoneSW.Multimedia, 2015/01/21 Add for 15005 TianMa LCD ESD*/
		lcd_dev = LCD_TM_NT35512S;
		pr_err("oppo15005tm nt35512s fwvga video mode dsi panel\n");
	}else if(!strcmp(panel_name,"oppo15005boe fwvga video mode dsi panel")){
		lcd_dev = LCD_BOE_HX8379S;
		pr_err("oppo15005boe fwvga video mode dsi panel\n");
	}else if(!strcmp(panel_name,"oppo15005truly fwvga video mode dsi panel")){
		lcd_dev = LCD_15005_TRULY_HX8379C;
		pr_err("oppo15005truely fwvga video mode dsi panel\n");
	}else if(!strcmp(panel_name,"oppo15025boe hx8379c fwvga video mode dsi panel")){  		/*YongPeng.Yi@PhoneSW.Multimedia, 2015/01/21 Add for 15025 LCD ESD*/
		lcd_dev = LCD_15025_BOE_HX8379C;
		pr_err("oppo15025boe hx8379c fwvga video mode dsi panel\n");
	}else if(!strcmp(panel_name,"oppo15025tm hx8379 fwvga video mode dsi panel")){
		lcd_dev = LCD_15025_TM_HX8379;
		pr_err("oppo15025tm hx8379 fwvga video mode dsi panel\n");
	}else if(!strcmp(panel_name,"oppo15025tm nt35512s fwvga video mode dsi panel")){
		lcd_dev = LCD_15025_TM_NT35512S;
		pr_err("oppo15025tm nt35512s fwvga video mode dsi panel\n");
	}else if(!strcmp(panel_name,"oppo15025truly hx8379c fwvga video mode dsi panel")){
		lcd_dev = LCD_15025_TRULY_HX8379C;
		pr_err("oppo15025truly hx8379c fwvga video mode dsi panel\n");
	}
#endif  
	rc = mdss_panel_parse_dt(node, ctrl_pdata);
	if (rc) {
		pr_err("%s:%d panel dt parse failed\n", __func__, __LINE__);
		return rc;
	}

	if (!cmd_cfg_cont_splash)
		pinfo->cont_splash_enabled = false;
	pr_info("%s: Continuous splash %s", __func__,
		pinfo->cont_splash_enabled ? "enabled" : "disabled");
/* OPPO 2013-12-09 yxq Add begin for disable continous display for ftm mode */
#ifdef VENDOR_EDIT
	if(!is_project(OPPO_14005) && !is_project(OPPO_14045) && !is_project(OPPO_15011) && !is_project(OPPO_15018)){
	    if ((MSM_BOOT_MODE__FACTORY == get_boot_mode()) ||
			(MSM_BOOT_MODE__MOS == get_boot_mode())){
	        pinfo->cont_splash_enabled = false;
	    }
	}
#endif
#ifdef VENDOR_EDIT
/* Xiaori.Yuan@Mobile Phone Software Dept.Driver, 2014/08/01  Add for ESD */
		if(is_project(OPPO_14045) || is_project(OPPO_14005) || is_project(OPPO_15011) || is_project(OPPO_15018)){
			te_check_gpio = 910;
			ESD_TE_TEST = 1;
			gpio_direction_input(te_check_gpio);
			if(lcd_dev==LCD_14045_17_VIDEO || lcd_dev==LCD_14045_17_CMD){
				ESD_TE_TEST = 0;
			}
		}
		/* YongPeng.Yi@PhoneSW.Multimedia, 2014/12/02 Add for || is_project(OPPO_14051) */
		if(is_project(OPPO_14045) || is_project(OPPO_14051)){
			te_reset_14045  = 1;
		}
		if(is_project(OPPO_14051)){
		    te_check_gpio = 914;
			//ESD_TE_TEST = 1;
		}
		/*Yongpeng.Yi@PhoneSW.Multimedia Add for 14037 LCD_JDI_NT35521 ESD Check PWM Simulate TE*/
		if((is_project(OPPO_14037) || is_project(OPPO_15057))&&(lcd_dev == LCD_JDI_NT35521)){
		    te_check_gpio = 914;
			ESD_TE_TEST = 1;
		}
		if(MSM_BOOT_MODE__FACTORY == get_boot_mode()){
			ESD_TE_TEST = 0;
			if(is_project(OPPO_14045) || is_project(OPPO_14051)){
				te_reset_14045  = 1;
			}
		}
		if(ESD_TE_TEST){
			pr_err("te_check_gpio = %d \n",te_check_gpio);
		
			init_completion(&te_comp);
			 gpio_request(te_check_gpio,"te_check");
			 gpio_direction_input(te_check_gpio);
			irq = gpio_to_irq(te_check_gpio); 
			pr_err("te_check_gpio_irq = %d \n",irq);
			rc = request_threaded_irq(irq, TE_irq_thread_fn, NULL,
				IRQF_TRIGGER_RISING, "LCD_TE",NULL);	

			disable_irq(irq);
			if (rc < 0) {		
				pr_err("Unable to register IRQ handler\n"); 	
				//return -ENODEV; 
				}	
			INIT_DELAYED_WORK(&techeck_work, techeck_work_func );	
			schedule_delayed_work(&techeck_work, msecs_to_jiffies(20000));
			
			display_switch.name = "dispswitch";
		
			rc = switch_dev_register(&display_switch);
			if (rc)
			{
				pr_err("Unable to register display switch device\n");
				//return rc;
			}
		
			/*dir: /sys/class/mdss_lcd/lcd_control*/	
			mdss_lcd = class_create(THIS_MODULE,"mdss_lcd");		
			mdss_lcd->dev_attrs = mdss_lcd_attrs;				
			device_create(mdss_lcd,dev_lcd,0,NULL,"lcd_control");
			cont_splash_flag = pinfo->cont_splash_enabled;
		}
#endif /*VENDOR_EDIT*/
	pinfo->dynamic_switch_pending = false;
	pinfo->is_lpm_mode = false;

	ctrl_pdata->on = mdss_dsi_panel_on;
	ctrl_pdata->off = mdss_dsi_panel_off;
	ctrl_pdata->panel_data.set_backlight = mdss_dsi_panel_bl_ctrl;
	ctrl_pdata->switch_mode = mdss_dsi_panel_switch_mode;

	return 0;
}
