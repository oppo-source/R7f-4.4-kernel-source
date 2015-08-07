/*************************************************************
 ** Copyright (C), 2012-2016, OPPO Mobile Comm Corp., Ltd 
 ** VENDOR_EDIT
 ** File        : tmg399x.c
 ** Description : 
 ** Date        : 2014-08-21 10:42
 ** Author      : BSP.Sensor
 ** 
 ** ------------------ Revision History: ---------------------
 **      <author>        <date>          <desc>
 *************************************************************/

/*
 * Device driver for monitoring ambient light intensity in (lux)
 * proximity detection (prox), and Gesture functionality within the
 * AMS-TAOS TMG3992/3.
 *
 * Copyright (c) 2013, AMS-TAOS USA, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/unistd.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/regulator/consumer.h>
#include <linux/sensors.h>
#include <linux/of_gpio.h>
//#include <linux/i2c/tmg399x.h>
#include "tmg399x.h"

static u8 const tmg399x_ids[] = {
	0xAA,
	0xAB,
	0x9E,
	0x9F,
	0x84,
	0x60,
	0x49,
};


static char const *tmg399x_names[] = {
	"tmg39933",
	"tmg39933",
	"tmg399x",
	"tmg399x",
	"tmg399x",
	"tmg399x",
	"tmg399x",
};

static u8 const restorable_regs[] = {
	TMG399X_ALS_TIME,
	TMG399X_WAIT_TIME,
	TMG399X_PERSISTENCE,
	TMG399X_CONFIG_1,
	TMG399X_PRX_PULSE,
	TMG399X_GAIN,
	TMG399X_CONFIG_2,
	TMG399X_PRX_OFFSET_NE,
	TMG399X_PRX_OFFSET_SW,
	TMG399X_CONFIG_3,
};

static u8 const prox_gains[] = {
	1,
	2,
	4,
	8
};

static u8 const als_gains[] = {
	1,
	4,
	16,
	64
};

static u8 const prox_pplens[] = {
	4,
	8,
	16,
	32
};

static u8 const led_drives[] = {
	100,
	50,
	25,
	12
};

static u16 const led_boosts[] = {
	100,
	150,
	200,
	300
};

static struct tmg_lux_segment segment_default[] = {
	{
		.d_factor = D_Factor1,
		.r_coef = R_Coef1,
		.g_coef = G_Coef1,
		.b_coef = B_Coef1,
		.ct_coef = CT_Coef1,
		.ct_offset = CT_Offset1,
	},
	{
		.d_factor = D_Factor1,
		.r_coef = R_Coef1,
		.g_coef = G_Coef1,
		.b_coef = B_Coef1,
		.ct_coef = CT_Coef1,
		.ct_offset = CT_Offset1,
	},
	{
		.d_factor = D_Factor1,
		.r_coef = R_Coef1,
		.g_coef = G_Coef1,
		.b_coef = B_Coef1,
		.ct_coef = CT_Coef1,
		.ct_offset = CT_Offset1,
	},
	{
		.d_factor = D_Factor1,
		.r_coef = R_Coef1,
		.g_coef = G_Coef1,
		.b_coef = B_Coef1,
		.ct_coef = CT_Coef1,
		.ct_offset = CT_Offset1,
	},
	{
		.d_factor = D_Factor1,
		.r_coef = R_Coef1,
		.g_coef = G_Coef1,
		.b_coef = B_Coef1,
		.ct_coef = CT_Coef1,
		.ct_offset = CT_Offset1,
	},
	{
		.d_factor = D_Factor1,
		.r_coef = R_Coef1,
		.g_coef = G_Coef1,
		.b_coef = B_Coef1,
		.ct_coef = CT_Coef1,
		.ct_offset = CT_Offset1,
	},
	{
		.d_factor = D_Factor1,
		.r_coef = R_Coef1,
		.g_coef = G_Coef1,
		.b_coef = B_Coef1,
		.ct_coef = CT_Coef1,
		.ct_offset = CT_Offset1,
	},
};

static struct tmg399x_parameters param_default = {
	.als_time = 0xFE, /* 5.6ms */
	.als_gain = GAGAIN_64,
	.wait_time = 0xFF, /* 2.78ms */
	.prox_th_min = 0,
	.prox_th_max = 255,
	.persist = PRX_PERSIST(0) | ALS_PERSIST(0),
	.als_prox_cfg1 = 0x60,
	.prox_pulse = PPLEN_32US | PRX_PULSE_CNT(5),
	.prox_gain = GPGAIN_4,
	.ldrive = GPDRIVE_100MA,
	.als_prox_cfg2 = LEDBOOST_150 | 0x01,
	.prox_offset_ne = 0,
	.prox_offset_sw = 0,
	.als_prox_cfg3 = 0,

	.ges_entry_th = 0,
	.ges_exit_th = 255,
	.ges_cfg1 = FIFOTH_1 | GEXMSK_ALL | GEXPERS_1,
	.ges_cfg2 = GGAIN_4 | GLDRIVE_100 | GWTIME_3,
	.ges_offset_n = 0,
	.ges_offset_s = 0,
	.ges_pulse = GPLEN_32US | GES_PULSE_CNT(5),
	.ges_offset_w = 0,
	.ges_offset_e = 0,
	.ges_dimension = GBOTH_PAIR,
};


static struct sensors_classdev sensors_light_cdev = {
	.name = "tmg399x-light",
	.vendor = "AMS-TAOS",
	.version = 1,
	.handle = SENSORS_LIGHT_HANDLE,
	.type = SENSOR_TYPE_LIGHT,
	.max_range = "6500",
	.resolution = "0.0625",
	.sensor_power = "0.09",
	.min_delay = 0,	/* us */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 200,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static struct sensors_classdev sensors_proximity_cdev = {
	.name = "tmg399x-proximity",
	.vendor = "AMS-TAOS",
	.version = 1,
	.handle = SENSORS_PROXIMITY_HANDLE,
	.type = SENSOR_TYPE_PROXIMITY,
	.max_range = "5.0",
	.resolution = "5.0",
	.sensor_power = "0.1",
	.min_delay = 0,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 200,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};


/* for gesture and proximity offset calibartion */
static bool docalibration = true;
static bool pstatechanged = false;
static u8 caloffsetstate = START_CALOFF;
static u8 caloffsetdir = DIR_NONE;
static u8 callowtarget = 8;
static u8 calhightarget = 8;

#define TMG_MAX_BUTTON_CNT 		(28)


static char music_play = 0;
u8 work_mode = TMG_MODE_TOUCH;
bool ges_touch_style = false;//true for kernel,false for app
bool ges_dir8_enable = false;
bool ges_tap_enable = false;

#define ZOOM_MASK 				(0x0f)
#define ZOOM_STATUS_MASK		(0x80)
static char zoom_status = 0;
static char push_release = 1;
char enter_zoom = 0;
static u8 last_mode = 0;


struct input_dev  *gesture_input_dev;//kernel simulate TP input device,
	
enum tmg399x_ges_function_list
{
	GOTO_RIGHT = 0,
	GOTO_UP,
	GOTO_LEFT,
	GOTO_DOWN,
	GOTO_MENU,
	GOTO_BACK,
	GOTO_ENTER,
	GOTO_VOLUP,
	GOTO_VOLDOWN,
	GOTO_NEXTSONG,
	GOTO_PREVIOUSSONG,
	GOTO_PLAYPAUSE,
	GOTO_PLAY,
	GOTO_POWER,
	GOTO_WWW,
	GOTO_PHONE,
	GOTO_NEW,
	GOTO_CAMERA,
	GOTO_EMAIL,
	GOTO_REPLY,
	GOTO_CANCEL,
	GOTO_COFFEE,
	GOTO_PRINT,
	GOTO_SERACH,
	GOTO_SPORT,
	GOTO_MEMO,
	GOTO_CALENDAR,
	GOTO_CALCULATION,
};

static int tmg3993_Keycode[TMG_MAX_BUTTON_CNT] = {KEY_RIGHT, KEY_UP, KEY_LEFT, KEY_DOWN, KEY_MENU, KEY_BACK,KEY_ENTER,//0~6
											KEY_VOLUMEUP, KEY_VOLUMEDOWN, KEY_NEXTSONG, KEY_PREVIOUSSONG, KEY_PLAYPAUSE, KEY_PLAY,KEY_POWER,//7~13
											KEY_WWW,KEY_PHONE,KEY_NEW,KEY_CAMERA,KEY_EMAIL,KEY_REPLY,KEY_CANCEL,KEY_COFFEE,//14~21
											KEY_PRINT,KEY_SEARCH,KEY_SPORT,KEY_MEMO,KEY_CALENDAR,KEY_CALC};//22~27


extern void process_rgbc_prox_ges_raw_data(struct tmg399x_chip *chip, u8 type, u8* data, u8 datalen);
extern void init_params_rgbc_prox_ges(void);
extern void set_visible_data_mode(struct tmg399x_chip *chip);
void tmg399x_rgbc_poll_handle(unsigned long data);
void tmg399x_report_prox(struct tmg399x_chip *chip, u8 detected);
void tmg399x_get_prox(struct tmg399x_chip *chip,u8 prox);
void tmg399x_update_prx_thd(struct tmg399x_chip *chip);
	
static void gesture_report_status_zoom(u8 status,u8 ms)
{
	
	u16 i,x1,x2,y1;
	u8 num_points = 20;
	printk("%s <---\n", __func__);

       x1 = x2 = y1 = i = 0;
	switch(status)
	{
		case 0:
			x1 = 300;	
			x2 = 500;	
			y1 = 700;
			break;
			
		case 1:
			x1 = 700;	
			x2 = 200;	
			y1 = 700;
			break;
			
		default:
			break;
	}
	
   	for(i=0; i< num_points;i++)
   	{

		input_report_abs(gesture_input_dev, ABS_MT_POSITION_X, x1 - 10*i);
	       input_report_abs(gesture_input_dev, ABS_MT_POSITION_Y, y1);
	       input_report_abs(gesture_input_dev, ABS_MT_WIDTH_MAJOR, 200);
		input_report_abs(gesture_input_dev, ABS_MT_TOUCH_MAJOR, 255);
		input_report_abs(gesture_input_dev, ABS_MT_TRACKING_ID, 0);

		input_mt_sync(gesture_input_dev); 
		if(i < num_points - 1)
		{
			input_report_abs(gesture_input_dev, ABS_MT_TOUCH_MAJOR, 0);
			input_report_abs(gesture_input_dev, ABS_MT_WIDTH_MAJOR, 0);
		}
		
		input_report_abs(gesture_input_dev, ABS_MT_POSITION_X, x2 + 10*i);
	       input_report_abs(gesture_input_dev, ABS_MT_POSITION_Y, y1);
	       input_report_abs(gesture_input_dev, ABS_MT_WIDTH_MAJOR, 200);
		input_report_abs(gesture_input_dev, ABS_MT_TOUCH_MAJOR, 255);
		input_report_abs(gesture_input_dev, ABS_MT_TRACKING_ID, 1);
	 			
		input_mt_sync(gesture_input_dev); 
		input_sync(gesture_input_dev);
		if(i < num_points - 1)
		{
			input_report_abs(gesture_input_dev, ABS_MT_TOUCH_MAJOR, 0);
			input_report_abs(gesture_input_dev, ABS_MT_WIDTH_MAJOR, 0);
		}

		mdelay(ms);	
		
   	}
	input_mt_sync(gesture_input_dev); 
    	input_sync(gesture_input_dev);
		
}		

static void gesture_report_status(u8 status,u8 ms)
{
	int x1, y1, x2, y2;
	int delta_x, delta_y,i;
	int num_points = 5;

       x1 = y1 = x2 = y2 = 0;
       delta_x = delta_y = i =0;
	switch(status)
	{
		case 1: 	x1 = 400;
				x2 = 700;
				if(work_mode == TMG_MODE_TOUCH)
				{
					x1 = 100;
					x2 = 400;
					y1 = 600;
					y2 = 600;
				}
				else
				{
					y1 = 1000;
					y2 = 1000;
				}
				num_points = 5;
				break;//right

		case 2: 	x1 = 400;
				x2 = 100;
				if(work_mode == TMG_MODE_TOUCH)
				{
					x1 = 700;
					x2 = 400;
					y1 = 600;
					y2 = 600;
				}
				else
				{
					y1 = 1000;
					y2 = 1000;
				}
				num_points = 5;
				break;//left

		case 3: 	x1 = 400;
				x2 = 400;
				y1 = 600;
				y2 = 900;
				num_points = 5;
				break;//down
				
		case 4: 	x1 = 400;
				x2 = 400;
				y1 = 600;
				y2 = 300;
				num_points = 5;
				break;//up

		case 5: 	x1 = 430;
				x2 = 730;
				y1 = 930;
				y2 = 930;
				num_points = 5;
				break;//unlock
				
		case 6: 	x1 = 430;
				x2 = 130;
				y1 = 930;
				y2 = 930;
				num_points = 5;
				break;//unlock

		case 7: 	x1 = 400;
				x2 = 400;
				y1 = 1250;
				y2 = 950;
				num_points = 5;
			break;//unlock	
			
		default:
			break;
	}

	delta_x = (x2 -x1)/num_points;
	delta_y = (y2 -y1)/num_points;
	
   	for(i=0; i< num_points;i++)
   	{

	        input_report_abs(gesture_input_dev, ABS_MT_POSITION_X, x1 + delta_x * i);
	        input_report_abs(gesture_input_dev, ABS_MT_POSITION_Y, y1 + delta_y * i);
	        input_report_abs(gesture_input_dev, ABS_MT_WIDTH_MAJOR, 200);
		 input_report_abs(gesture_input_dev, ABS_MT_TRACKING_ID, 0);

	        if (i < num_points-1)
	        {
				input_report_abs(gesture_input_dev, ABS_MT_TOUCH_MAJOR, 255);
	        }
	        else
	        {
	           		input_report_abs(gesture_input_dev, ABS_MT_TOUCH_MAJOR, 0);
	        }

		input_mt_sync(gesture_input_dev); 
		input_sync(gesture_input_dev);
		mdelay(ms);
   	}
	
	input_mt_sync(gesture_input_dev); 
    	input_sync(gesture_input_dev);
	
} 

static void gesture_touch_pressed(u8 status,u8 ms)
{
	int x1, y1, x2, y2;
	int delta_x, delta_y,i;	
    
       x1 = y1 = x2 = y2 = 0;
       delta_x = delta_y = i =0;
	switch(status)
	{
		case 1: 	x1 = 400;
				x2 = 100;
				y1 = 800;
				y2 = 800;
				break;//right

		case 2: 	x1 = 400;
				x2 = 700;
				y1 = 800;
				y2 = 800;
				break;//left

		case 3: 	x1 = 400;
				x2 = 400;
				y1 = 600;
				y2 = 300;
				break;//down
				
		case 4: 	x1 = 400;
				x2 = 400;
				y1 = 600;
				y2 = 900;
				break;//up
							
		default:
			break;
	}

	delta_x = (x1 - x2)/5;
	delta_y = (y1 - y2)/5;
	
	for(i = 0; i < 6; i++)
	{
		 input_report_abs(gesture_input_dev, ABS_MT_POSITION_X, x2 + delta_x * i);
	        input_report_abs(gesture_input_dev, ABS_MT_POSITION_Y, y2 + delta_y * i );
	        input_report_abs(gesture_input_dev, ABS_MT_WIDTH_MAJOR, 200);
		 input_report_abs(gesture_input_dev, ABS_MT_TRACKING_ID, 0);

		 input_report_abs(gesture_input_dev, ABS_MT_TOUCH_MAJOR, 255);

		 input_mt_sync(gesture_input_dev); 
		 input_sync(gesture_input_dev);	
		 mdelay(ms);
	}
} 


static void gesture_touch_release(u8 status,u8 ms)
{
	int x1, y1, x2, y2;
	int delta_x, delta_y,i;	
	int num_points = 5;

       x1 = y1 = x2 = y2 = 0;
       delta_x = delta_y = i =0;	
	switch(status)
	{
		case 1: 	x1 = 400;
				x2 = 100;
				y1 = 800;
				y2 = 800;
				break;//right

		case 2: 	x1 = 400;
				x2 = 700;
				y1 = 800;
				y2 = 800;
				break;//left

		case 3: 	x1 = 400;
				x2 = 400;
				y1 = 600;
				y2 = 300;
				break;//down
				
		case 4: 	x1 = 400;
				x2 = 400;
				y1 = 600;
				y2 = 900;
				break;//up
		
		default:
			break;
	}
	num_points = 5;
	delta_x = (x1 -x2)/5;
	delta_y = (y1 -y2)/5;
	
   	for(i=0; i< num_points;i++)
   	{

	        input_report_abs(gesture_input_dev, ABS_MT_POSITION_X, x1+ delta_x * i);
	        input_report_abs(gesture_input_dev, ABS_MT_POSITION_Y, y1 + delta_y * i);
	        input_report_abs(gesture_input_dev, ABS_MT_WIDTH_MAJOR, 200);
		 input_report_abs(gesture_input_dev, ABS_MT_TRACKING_ID, 0);

	        if (i < num_points-1)
	        {
				input_report_abs(gesture_input_dev, ABS_MT_TOUCH_MAJOR, 255);
	        }
	        else
	        {
	           		input_report_abs(gesture_input_dev, ABS_MT_TOUCH_MAJOR, 0);
	        }

		input_mt_sync(gesture_input_dev); 
		input_sync(gesture_input_dev);
		mdelay(ms);
   	}
	
	input_mt_sync(gesture_input_dev); 
    	input_sync(gesture_input_dev);
	
}  


static void tmg399x_ges_key_mode(struct tmg399x_chip *chip,u8 ges_state)
{

//	dev_info(&chip->client->dev, "%s: ges_state = %d\n", __func__,ges_state);
	
	switch(ges_state&GES_MOTION_MASK)
	{
		case 0:			
			input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_RIGHT], 1);
			input_sync(chip->p_idev);
			
			input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_RIGHT], 0);	
			input_sync(chip->p_idev);
			
			break;//right
			
		case 2:
			input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_DOWN], 1);
			input_sync(chip->p_idev);
			
			input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_DOWN], 0);	
			input_sync(chip->p_idev);
			break;//down
			
		case 4:
			input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_LEFT], 1);
			input_sync(chip->p_idev);
			
			input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_LEFT], 0);	
			input_sync(chip->p_idev);
			break;//left
			
		case 6:
			input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_UP], 1);
			input_sync(chip->p_idev);
			
			input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_UP], 0);	
			input_sync(chip->p_idev);
			break;//up
			
		case 8:
			input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_ENTER], 1);
			input_sync(chip->p_idev);
			
			input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_ENTER], 0);	
			input_sync(chip->p_idev);
			break;//push
			
		case 9:
			break;//hold

		case 11:
			input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_BACK], 1);
			input_sync(chip->p_idev);
			
			input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_BACK], 0);	
			input_sync(chip->p_idev);
			break;//right tap
			
		case 12:
			input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_MENU], 1);
			input_sync(chip->p_idev);
			
			input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_MENU], 0);	
			input_sync(chip->p_idev);
			break;//down tap

		case 14:

			break;

		case 15:
			
			break;			
	
		default:
			break;
			
	}
	ges_motion = 0x80;
}


static void tmg399x_ges_music_mode(struct tmg399x_chip *chip,u8 ges_state)
{

//	dev_info(&chip->client->dev, "%s: ges_state = %d\n", __func__,ges_state);	
	switch(ges_state&GES_MOTION_MASK)
	{
		case 0:			
			input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_NEXTSONG], 1);
			input_sync(chip->p_idev);
			
			input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_NEXTSONG], 0);	
			input_sync(chip->p_idev);
			
			break;//right
			
		case 2:
			input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_VOLDOWN], 1);
			input_sync(chip->p_idev);
			
			input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_VOLDOWN], 0);	
			input_sync(chip->p_idev);
			break;//down
			
		case 4:
			input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_PREVIOUSSONG], 1);
			input_sync(chip->p_idev);
			
			input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_PREVIOUSSONG], 0);	
			input_sync(chip->p_idev);
			break;//left
			
		case 6:
			input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_VOLUP], 1);
			input_sync(chip->p_idev);
			
			input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_VOLUP], 0);	
			input_sync(chip->p_idev);
			break;//up
			
		case 8:
			if(music_play)
			{
				music_play = 0;
				input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_PLAY], 1);
				input_sync(chip->p_idev);
				
				input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_PLAY], 0);	
				input_sync(chip->p_idev);
				break;//push
			}
			else
			{
				music_play = 1;
				input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_PLAYPAUSE], 1);
				input_sync(chip->p_idev);
				
				input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_PLAYPAUSE], 0);	
				input_sync(chip->p_idev);
				break;//push
			}
			
		case 9:
			break;//hold

		case 11:
			input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_BACK], 1);
			input_sync(chip->p_idev);
			
			input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_BACK], 0);	
			input_sync(chip->p_idev);
			break;//right tap
			
		case 12:
			input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_MENU], 1);
			input_sync(chip->p_idev);
			
			input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_MENU], 0);	
			input_sync(chip->p_idev);
			break;//down tap
			
		default:
			break;
			
	}
	ges_motion = 0x80;
}


static void  tmg399x_ges_touch_mode(struct tmg399x_chip *chip,u8 ges_state)
{
//	dev_info(&chip->client->dev, "%s: ges_state = %d\n", __func__,ges_state);

	char i = 0;

	switch(ges_state&GES_MOTION_MASK)
	{
	
		case 0:
				if((ges_state&GES_SPEED_MASK) == GES_SPEED_SLOW)
				{
					gesture_report_status(1,20);//right	
				}
				else
				{
					gesture_report_status(1,3);//right
				}			
			break;

		case 4:
				if((ges_state&GES_SPEED_MASK) == GES_SPEED_SLOW)
				{
					gesture_report_status(2,20);//left
				}
				else
				{
					gesture_report_status(2,3);//left
				}			
			break;

		case 2:
				if((ges_state&GES_SPEED_MASK) == GES_SPEED_SLOW)
				{
					gesture_report_status(3,20);//down
				}
				else
				{
					gesture_report_status(3,3);//down
				}			
			break;

		case 6:
				if((ges_state&GES_SPEED_MASK) == GES_SPEED_SLOW)
				{
					gesture_report_status(4,20);//up
				}
				else
				{
					gesture_report_status(4,3);//up
				}			
			break;
	
		case 8://push
		case 9:
			if(push_release) 
			{
				if((zoom_status&ZOOM_MASK) < 3)
				{
					zoom_status++;
					zoom_status |=ZOOM_STATUS_MASK;
					
					gesture_report_status_zoom(0,20);//zoom in	
				}			
			}
			else
			{
				if(!(zoom_status & ZOOM_STATUS_MASK))
				{
					for(i = 0; i<(zoom_status = zoom_status&ZOOM_MASK );i++)
					{
						gesture_report_status_zoom(1,10);	//zoom out
					}
					{
						zoom_status = 0;
						zoom_status |=ZOOM_STATUS_MASK;
					}
				}

			}
			break;//hold
			
		case 10:
			if(zoom_status !=0)
			zoom_status &=~ZOOM_STATUS_MASK;

			if(push_release) 
				push_release = 0;
			else
				push_release = 1;			
			break;//release
		
		case 11:
			push_release = 1;
			input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_BACK], 1);
			input_sync(chip->p_idev);
			
			input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_BACK], 0);	
			input_sync(chip->p_idev);
			break;//right tap
			
		case 12:
			break;
			
			input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_MENU], 1);
			input_sync(chip->p_idev);
			
			input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_MENU], 0);	
			input_sync(chip->p_idev);
			break;//down tap
			
		default:
			break;
			
	}
	ges_motion = 0x80;
}



static void  tmg399x_ges_game_mode(struct tmg399x_chip *chip,u8 ges_state)
{
//	dev_info(&chip->client->dev, "%s: ges_state = %d\n", __func__,ges_state);
	
	switch(ges_state&GES_MOTION_MASK)
	{
		case 0:	
			gesture_report_status(1,8);//right
			break;//right
			
		case 2:
			gesture_report_status(3,8);//down
			break;//down
			
		case 4:
			gesture_report_status(2,8);//left
			break;//left
			
		case 6:
			gesture_report_status(4,8);//up
			break;//up
			
		case 9:
			break;//hold
			
		default:
			break;
			
	}
	ges_motion = 0x80;
}

static void  tmg399x_ges_book_mode(struct tmg399x_chip *chip,u8 ges_state)
{
//	dev_info(&chip->client->dev, "%s: ges_state = %d\n", __func__,ges_state);
	
	switch(ges_state&GES_MOTION_MASK)
	{

		case 17:	
			gesture_touch_pressed(1,15);//right
			break;//right
			
		case 18:
			gesture_touch_pressed(3,15);//down
			break;//down
			
		case 15:
			gesture_touch_pressed(2,15);//left
			break;//left
			
		case 16:
			gesture_touch_pressed(4,15);//up
			break;//up
/////////////////////////////////////////////////////////

		case 19:	
			gesture_touch_release(1,20);//right
			break;//right
			
		case 20:
			gesture_touch_release(3,20);//down
			break;//down
			
		case 21:
			gesture_touch_release(2,20);//left
			break;//left
			
		case 22:
			gesture_touch_release(4,20);//up
			break;//up
			
		default:
			break;
			
	}
	ges_motion = 0x80;
}




static void  tmg399x_ges_lock_mode(struct tmg399x_chip *chip,u8 ges_state)
{
//	dev_info(&chip->client->dev, "%s: ges_state = %d\n", __func__,ges_state);
	
	switch(ges_state&GES_MOTION_MASK)
	{
		case 0:
			gesture_report_status(5,5);//right	
//			work_mode = TMG_MODE_TOUCH;
			break;//unlock
			
		case 6:
			gesture_report_status(7,5);//up
			break;//up
			
		case 4:
			gesture_report_status(6,5);//left
//				work_mode = TMG_MODE_TOUCH;
			break;//open camera
			
		case 2:	
			input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_POWER], 1);
			input_sync(chip->p_idev);
			
			input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_POWER], 0);	
			input_sync(chip->p_idev);
			break;//down

		
		case 8:
			work_mode = TMG_MODE_LOCK;
			input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_POWER], 1);//hold
			input_sync(chip->p_idev);
			
			input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_POWER], 0);	
			input_sync(chip->p_idev);
			break;
			
		default:
			break;
			
	}
	ges_motion = 0x80;
}

static void tmg399x_ges_other_app_mode(struct tmg399x_chip *chip,u8 ges_state)
{

//	dev_info(&chip->client->dev, "%s: ges_state = %d\n", __func__,ges_state);
	
	switch(ges_state&GES_MOTION_MASK)
	{
		case 0:
			input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_WWW], 1);//right
			input_sync(chip->p_idev);
			
			input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_WWW], 0);	
			input_sync(chip->p_idev);
			break;//WWW		
			
		case 2:
			input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_EMAIL], 1);//down
			input_sync(chip->p_idev);
			
			input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_EMAIL], 0);	
			input_sync(chip->p_idev);
			break;//email
			
		case 4:
			input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_PHONE], 1);//left
			input_sync(chip->p_idev);
			
			input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_PHONE], 0);	
			input_sync(chip->p_idev);
			break;//phone
			
		case 6:
			input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_SERACH], 1);//up
			input_sync(chip->p_idev);
			
			input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_SERACH], 0);	
			input_sync(chip->p_idev);
			break;//serach
			
		case 8:
			break;//push
			
		case 9:
			work_mode = TMG_MODE_LOCK;
			input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_POWER], 1);//hold
			input_sync(chip->p_idev);
			
			input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_POWER], 0);	
			input_sync(chip->p_idev);
			break;//sleep

		case 11:
			input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_BACK], 1);//right tap
			input_sync(chip->p_idev);
			
			input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_BACK], 0);	
			input_sync(chip->p_idev);
			break;//back

		case 12:
			input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_MENU], 1);//down tap
			input_sync(chip->p_idev);
			
			input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_MENU], 0);	
			input_sync(chip->p_idev);
			break;//menu
			
		case 13:
			input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_CALENDAR], 1);//left tap
			input_sync(chip->p_idev);
			
			input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_CALENDAR], 0);	
			input_sync(chip->p_idev);
			break;//calendar

		case 14:
			input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_CALCULATION], 1);//up tap
			input_sync(chip->p_idev);
			
			input_report_key(chip->p_idev, tmg3993_Keycode[GOTO_CALCULATION], 0);	
			input_sync(chip->p_idev);
			break;//calculation
			
		default:
			break;
			
	}
	ges_motion = 0x80;
}


void tmg399x_ges_work_mode(struct tmg399x_chip *chip,u8 mode,u8 ges_state)
{
//	dev_info(&chip->client->dev, "%s: mode = %d,ges_state = %d\n", __func__,mode,ges_state);
	
	switch(mode)
	{
		case TMG_MODE_KEY:
			tmg399x_ges_key_mode(chip,ges_state);
			break;

		case TMG_MODE_MUSIC:
			tmg399x_ges_music_mode(chip,ges_state);
			break;
			
		case TMG_MODE_TOUCH:
			tmg399x_ges_touch_mode(chip,ges_state);
			break;

		case TMG_MODE_GAME:
			tmg399x_ges_game_mode(chip,ges_state);
			break;
			
		case TMG_MODE_LOCK:
			tmg399x_ges_lock_mode(chip,ges_state);
			break;

		case TMG_MODE_BOOK:
			tmg399x_ges_book_mode(chip,ges_state);
			break;
			
		case TMG_MODE_OTHER_APP:
			tmg399x_ges_other_app_mode(chip,ges_state);
			break;
			
		default:
			dev_err(&chip->client->dev, "%s: work mode failed!!!mode = %d,ges_state = %d\n", __func__,mode,ges_state);
			break;
	}
}

static int tmg399x_i2c_read(struct tmg399x_chip *chip, u8 reg, u8 *val)
{
	int ret;

	s32 read;
	struct i2c_client *client = chip->client;

	reg += I2C_ADDR_OFFSET;
	ret = i2c_smbus_write_byte(client, reg);
	if (ret < 0) {
		mdelay(3);
		ret = i2c_smbus_write_byte(client, reg);
		if (ret < 0) {
			dev_err(&client->dev, "%s: failed 2x to write register %x\n",
				__func__, reg);
		return ret;
		}
	}

	read = i2c_smbus_read_byte(client);
	if (read < 0) {
		mdelay(3);
		read = i2c_smbus_read_byte(client);
		if (read < 0) {
			dev_err(&client->dev, "%s: failed read from register %x\n",
				__func__, reg);
		}
		return ret;
	}

	*val = (u8)read;
	return 0;
}

static int tmg399x_i2c_write(struct tmg399x_chip *chip, u8 reg, u8 val)
{
	int ret;
	struct i2c_client *client = chip->client;

	reg += I2C_ADDR_OFFSET;
	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0) {
		mdelay(3);
		ret = i2c_smbus_write_byte_data(client, reg, val);
		if (ret < 0) {
			dev_err(&client->dev, "%s: failed to write register %x err= %d\n",
				__func__, reg, ret);
		}
	}
	return ret;
}

#if 0
static int tmg399x_i2c_modify(struct tmg399x_chip *chip, u8 reg, u8 mask, u8 val)
{
	int ret;
	u8 temp;

	ret = tmg399x_i2c_read(chip, reg, &temp);
	temp &= ~mask;
	temp |= val;
	ret |= tmg399x_i2c_write(chip, reg, temp);

	return ret;
}
#endif

static int tmg399x_i2c_reg_blk_write(struct tmg399x_chip *chip,
		u8 reg, u8 *val, int size)
{
	s32 ret;
	struct i2c_client *client = chip->client;

	reg += I2C_ADDR_OFFSET;
	ret =  i2c_smbus_write_i2c_block_data(client,
			reg, size, val);
	if (ret < 0) {
		dev_err(&client->dev, "%s: failed 2X at address %x (%d bytes)\n",
				__func__, reg, size);
	}

	return ret;
}

static int tmg399x_i2c_ram_blk_read(struct tmg399x_chip *chip,
		u8 reg, u8 *val, int size)
{
	s32 ret;
	struct i2c_client *client = chip->client;

	reg += I2C_ADDR_OFFSET;
	ret =  i2c_smbus_read_i2c_block_data(client,
			reg, size, val);
	if (ret < 0) {
		dev_err(&client->dev, "%s: failed 2X at address %x (%d bytes)\n",
				__func__, reg, size);
	}

	return ret;
}

static int tmg399x_i2c_ram_blk_write(struct tmg399x_chip *chip,
		u8 reg, u8 *val, int size)
{
	s32 ret;
	struct i2c_client *client = chip->client;
	ret =  i2c_smbus_write_i2c_block_data(client,
			reg, size, val);
	if (ret < 0)
		dev_err(&client->dev, "%s: failed at address %x (%d bytes)\n",
				__func__, reg, size);
	return ret;
}


#if 0
static int tmg399x_update_als_thres(struct tmg399x_chip *chip, bool on_enable)
{
	s32 ret;
	u8 *buf = &chip->shadow[TMG399X_ALS_MINTHRESHLO];
	u16 deltaP = chip->params.als_deltaP;
	u16 from, to, cur;
	u16 saturation = chip->als_inf.saturation;

	cur = chip->als_inf.clear_raw;

	if (on_enable) {
		/* move deltaP far away from current position to force an irq */
		from = to = cur > saturation / 2 ? 0 : saturation;
	} else {
		deltaP = cur * deltaP / 100;
		if (!deltaP)
			deltaP = 1;

		if (cur > deltaP)
			from = cur - deltaP;
		else
			from = 0;

		if (cur < (saturation - deltaP))
			to = cur + deltaP;
		else
			to = saturation;

	}

	*buf++ = from & 0xff;
	*buf++ = from >> 8;
	*buf++ = to & 0xff;
	*buf++ = to >> 8;
	ret = tmg399x_i2c_reg_blk_write(chip, TMG399X_ALS_MINTHRESHLO,
			&chip->shadow[TMG399X_ALS_MINTHRESHLO],
			TMG399X_ALS_MAXTHRESHHI - TMG399X_ALS_MINTHRESHLO + 1);

	return (ret < 0) ? ret : 0;
}
#endif

static ssize_t tmg399x_beam_init(struct tmg399x_chip *chip)
{
	return tmg399x_i2c_reg_blk_write(chip, TMG399X_IRBEAM_CFG,
			&chip->beam_settings.beam_cfg,
			(sizeof(chip->beam_settings)));
}

static ssize_t tmg399x_beam_xmit(struct tmg399x_chip *chip)
{
	int ret;

	/* Prepare for restoration */
	chip->pre_beam_xmit_state = chip->shadow[TMG399X_CONTROL];
	memcpy(&chip->beam_settings, &chip->beam_hop[chip->hop_index],
			sizeof(chip->beam_settings));

	/* If running continuous allow thread and i2c engine to keep up */
	if (chip->hop_count == 255)
		msleep(50);

	tmg399x_beam_init(chip);

	/* Disable ALS & prox - Power On - Enable IRbeam */
	/* (leave shadow reg alone) */
	ret = tmg399x_i2c_write(chip, TMG399X_CONTROL, 0x81);

	/* Length based on last bc symbol D/L */
	ret = tmg399x_i2c_write(chip, TMG399X_IRBEAM_LEN, chip->bc_nibbles);

	dev_info(&chip->client->dev, "%s\n", __func__);
	/* restored to previous control state after Beam interrupt occurs. */
	return ret;
}

static ssize_t tmg399x_beam_symbol_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	int i = 0;
	int value[128];
	u8      x[128];

	dev_info(&chip->client->dev, "%s\n", __func__);
	get_options(buf, ARRAY_SIZE(value), value);
	
	if ((value[0] >= 0x80) || value[0] == 0) {
		dev_info(dev, "SYMBOL TABLE INPUT ERROR Value[0]\n");
		return -EINVAL;
	}

	chip->bc_nibbles = (value[0] * 2);

	for (i = 0; i <= value[0]; i++) {
		if (value[i] > 0xff) {
			dev_info(dev, "ERROR Value[%02x]: %04x\n", i, value[i]);
			return -EINVAL;
		}
		x[i] = (u8)value[i];
		printk(KERN_INFO "x[%d] = %x\n",i,x[i]);
	}

	/* Init and Fill the table */
	memset(chip->bc_symbol_table, 0xFF, 128);
	memcpy(chip->bc_symbol_table, &x[1], x[0]);
	/* Copy table into chip */
	tmg399x_i2c_ram_blk_write(chip, 0x00, chip->bc_symbol_table,sizeof(chip->bc_symbol_table));

	return size;
}


static int tmg399x_flush_regs(struct tmg399x_chip *chip)
{
	unsigned i;
	int ret;
	u8 reg;

	dev_info(&chip->client->dev, "%s\n", __func__);

	for (i = 0; i < ARRAY_SIZE(restorable_regs); i++) {
		reg = restorable_regs[i];
		ret = tmg399x_i2c_write(chip, reg, chip->shadow[reg]);
		if (ret < 0) {
			dev_err(&chip->client->dev, "%s: err on reg 0x%02x\n",
					__func__, reg);
			break;
		}
	}
	return ret;
}

static int tmg399x_irq_clr(struct tmg399x_chip *chip, u8 int2clr)
{
	int ret, ret2;

	ret = i2c_smbus_write_byte(chip->client, int2clr);
	if (ret < 0) {
		mdelay(3);
		ret2 = i2c_smbus_write_byte(chip->client, int2clr);
		if (ret2 < 0) {
			dev_err(&chip->client->dev, "%s: failed 2x, int to clr %02x\n",
					__func__, int2clr);
		}
		return ret2;
	}

	return ret;
}

static int tmg399x_update_enable_reg(struct tmg399x_chip *chip)
{
	int ret;
	
	ret = tmg399x_i2c_write(chip, TMG399X_CONTROL,
			chip->shadow[TMG399X_CONTROL]);
	ret |= tmg399x_i2c_write(chip, TMG399X_GES_CFG_4,
			chip->shadow[TMG399X_GES_CFG_4]);
	
	return ret;
}

static int tmg399x_set_als_gain(struct tmg399x_chip *chip, int gain)
{
	int ret;
	u8 ctrl_reg  = chip->shadow[TMG399X_GAIN] & ~TMG399X_ALS_GAIN_MASK;

	switch (gain) {
	case 1:
		ctrl_reg |= GAGAIN_1;
		break;
	case 4:
		ctrl_reg |= GAGAIN_4;
		break;
	case 16:
		ctrl_reg |= GAGAIN_16;
		break;
	case 64:
		ctrl_reg |= GAGAIN_64;
		break;
	default:
		break;
	}

	ret = tmg399x_i2c_write(chip, TMG399X_GAIN, ctrl_reg);
	if (!ret) {
		chip->shadow[TMG399X_GAIN] = ctrl_reg;
		chip->params.als_gain = ctrl_reg & TMG399X_ALS_GAIN_MASK;
	}
	return ret;
}

static void tmg399x_calc_cpl(struct tmg399x_chip *chip)
{
	u32 cpl;
	u32 sat;
	u8 atime = chip->shadow[TMG399X_ALS_TIME];

	cpl = 256 - chip->shadow[TMG399X_ALS_TIME];
	cpl *= TMG399X_ATIME_PER_100;
	cpl /= 100;
	cpl *= als_gains[chip->params.als_gain];

	sat = min_t(u32, MAX_ALS_VALUE, (u32)(256 - atime) << 10);
	sat = sat * 8 / 10;
	chip->als_inf.cpl = cpl;
	chip->als_inf.saturation = sat;
}

static int tmg399x_get_lux(struct tmg399x_chip *chip)
{
	u32 rp1, gp1, bp1, cp1;
	u32 lux = 0;
	u32 cct;
	u32 sat;
	u32 sf;

	/* use time in ms get scaling factor */
	tmg399x_calc_cpl(chip);
	sat = chip->als_inf.saturation;

	if (!chip->als_gain_auto) {
		if (chip->als_inf.clear_raw <= MIN_ALS_VALUE) {
			dev_info(&chip->client->dev,
				"%s: darkness\n", __func__);
			lux = 0;
			goto exit;
		} else if (chip->als_inf.clear_raw >= sat) {
			dev_info(&chip->client->dev,
				"%s: saturation, keep lux & cct\n", __func__);
			lux = chip->als_inf.lux;
			goto exit;
		}
	} else {
		u8 gain = als_gains[chip->params.als_gain];
		int ret = -EIO;

		if (gain == 16 && chip->als_inf.clear_raw >= sat) {
			ret = tmg399x_set_als_gain(chip, 1);
		} else if (gain == 16 &&
			chip->als_inf.clear_raw < GAIN_SWITCH_LEVEL) {
			ret = tmg399x_set_als_gain(chip, 64);
		} else if ((gain == 64 &&
			chip->als_inf.clear_raw >= (sat - GAIN_SWITCH_LEVEL)) ||
			(gain == 1 && chip->als_inf.clear_raw < GAIN_SWITCH_LEVEL)) {
			ret = tmg399x_set_als_gain(chip, 16);
		}
		if (!ret) {
			dev_info(&chip->client->dev, "%s: gain adjusted, skip\n",
					__func__);
			tmg399x_calc_cpl(chip);
			ret = -EAGAIN;
			lux = chip->als_inf.lux;
			goto exit;
		}

		if (chip->als_inf.clear_raw <= MIN_ALS_VALUE) {
			dev_info(&chip->client->dev,
					"%s: darkness\n", __func__);
			lux = 0;
			goto exit;
		} else if (chip->als_inf.clear_raw >= sat) {
			dev_info(&chip->client->dev, "%s: saturation, keep lux\n",
					__func__);
			lux = chip->als_inf.lux;
			goto exit;
		}
	}

	/* remove ir from counts*/
	rp1 = chip->als_inf.red_raw - chip->als_inf.ir;
	gp1 = chip->als_inf.green_raw - chip->als_inf.ir;
	bp1 = chip->als_inf.blue_raw - chip->als_inf.ir;
	cp1 = chip->als_inf.clear_raw - chip->als_inf.ir;

	if (!chip->als_inf.cpl) {
		dev_info(&chip->client->dev, "%s: zero cpl. Setting to 1\n",
				__func__);
		chip->als_inf.cpl = 1;
	}

	if (chip->als_inf.red_raw > chip->als_inf.ir)
		lux += chip->segment[chip->device_index].r_coef * rp1;
	else
		dev_err(&chip->client->dev, "%s: lux rp1 = %d\n",
			__func__,
			(chip->segment[chip->device_index].r_coef * rp1));

	if (chip->als_inf.green_raw > chip->als_inf.ir)
		lux += chip->segment[chip->device_index].g_coef * gp1;
	else
		dev_err(&chip->client->dev, "%s: lux gp1 = %d\n",
			__func__,
			(chip->segment[chip->device_index].g_coef * rp1));

	if (chip->als_inf.blue_raw > chip->als_inf.ir)
		lux -= chip->segment[chip->device_index].b_coef * bp1;
	else
		dev_err(&chip->client->dev, "%s: lux bp1 = %d\n",
			__func__,
			(chip->segment[chip->device_index].b_coef * rp1));

	sf = chip->als_inf.cpl;

	if (sf > 131072)
		goto error;

	lux /= sf;
	lux *= chip->segment[chip->device_index].d_factor;
	lux += 500;
	lux /= 1000;
	chip->als_inf.lux = (u16) lux;

	cct = ((chip->segment[chip->device_index].ct_coef * bp1) / rp1) +
		chip->segment[chip->device_index].ct_offset;

	chip->als_inf.cct = (u16) cct;

	if(chip->als_inf.lux >= MAX_LUX_VALUE) chip->als_inf.lux = MAX_LUX_VALUE;
	if(chip->als_inf.cct >= MAX_CCT_VALUE) chip->als_inf.cct = MAX_CCT_VALUE;
		
exit:
	return 0;

error:
	dev_err(&chip->client->dev, "ERROR Scale factor = %d", sf);

	return 1;
}

static int tmg399x_ges_init(struct tmg399x_chip *chip, int on)
{
	int ret;
	
	if (on) 
	{
		if (chip->pdata) 
		{
			chip->params.ges_entry_th = chip->pdata->parameters.ges_entry_th;
			chip->params.ges_exit_th = chip->pdata->parameters.ges_exit_th;
			chip->params.ges_cfg1 = chip->pdata->parameters.ges_cfg1;
			chip->params.ges_cfg2 = chip->pdata->parameters.ges_cfg2;
			chip->params.ges_offset_n = chip->pdata->parameters.ges_offset_n;
			chip->params.ges_offset_s = chip->pdata->parameters.ges_offset_s;
			chip->params.ges_pulse = chip->pdata->parameters.ges_pulse;
			chip->params.ges_offset_w = chip->pdata->parameters.ges_offset_w;
			chip->params.ges_offset_e = chip->pdata->parameters.ges_offset_e;
			chip->params.ges_dimension = chip->pdata->parameters.ges_dimension;
		} 
		else
		{
			chip->params.ges_entry_th = param_default.ges_entry_th;
			chip->params.ges_exit_th = param_default.ges_exit_th;
			chip->params.ges_cfg1 = param_default.ges_cfg1;
			chip->params.ges_cfg2 = param_default.ges_cfg2;
			chip->params.ges_offset_n = param_default.ges_offset_n;
			chip->params.ges_offset_s = param_default.ges_offset_s;
			chip->params.ges_pulse = param_default.ges_pulse;
			chip->params.ges_offset_w = param_default.ges_offset_w;
			chip->params.ges_offset_e = param_default.ges_offset_e;
			chip->params.ges_dimension = param_default.ges_dimension;
		}
	}

	/* Initial gesture registers */
	chip->shadow[TMG399X_GES_ENTH]  = chip->params.ges_entry_th;
	chip->shadow[TMG399X_GES_EXTH]  = chip->params.ges_exit_th;
	chip->shadow[TMG399X_GES_CFG_1] = chip->params.ges_cfg1;
	chip->shadow[TMG399X_GES_CFG_2] = chip->params.ges_cfg2;
	chip->shadow[TMG399X_GES_OFFSET_N] = chip->params.ges_offset_n;
	chip->shadow[TMG399X_GES_OFFSET_S] = chip->params.ges_offset_s;
	chip->shadow[TMG399X_GES_PULSE] = chip->params.ges_pulse;
	chip->shadow[TMG399X_GES_OFFSET_W] = chip->params.ges_offset_w;
	chip->shadow[TMG399X_GES_OFFSET_E] = chip->params.ges_offset_e;
	chip->shadow[TMG399X_GES_CFG_3] = chip->params.ges_dimension;
	
	ret = tmg399x_i2c_write(chip, TMG399X_GES_ENTH,chip->shadow[TMG399X_GES_ENTH]);
	ret |= tmg399x_i2c_write(chip, TMG399X_GES_EXTH,chip->shadow[TMG399X_GES_EXTH]);
	ret |= tmg399x_i2c_write(chip, TMG399X_GES_CFG_1,chip->shadow[TMG399X_GES_CFG_1]);
	ret |= tmg399x_i2c_write(chip, TMG399X_GES_CFG_2,chip->shadow[TMG399X_GES_CFG_2]);
	ret |= tmg399x_i2c_write(chip, TMG399X_GES_OFFSET_N,chip->shadow[TMG399X_GES_OFFSET_N]);
	ret |= tmg399x_i2c_write(chip, TMG399X_GES_OFFSET_S,chip->shadow[TMG399X_GES_OFFSET_S]);
	ret |= tmg399x_i2c_write(chip, TMG399X_GES_PULSE,chip->shadow[TMG399X_GES_PULSE]);
	ret |= tmg399x_i2c_write(chip, TMG399X_GES_OFFSET_W,chip->shadow[TMG399X_GES_OFFSET_W]);
	ret |= tmg399x_i2c_write(chip, TMG399X_GES_OFFSET_E,chip->shadow[TMG399X_GES_OFFSET_E]);
	ret |= tmg399x_i2c_write(chip, TMG399X_GES_CFG_3,chip->shadow[TMG399X_GES_CFG_3]);

	return ret;
}

static int tmg399x_set_op_mode(struct tmg399x_chip *chip, u8 mode, u8 on)
{
	u8 cur_mode;
	int ret = 0;

	if (on)
		cur_mode = last_mode | mode;
	else
		cur_mode = last_mode & ~mode;
	last_mode = cur_mode;
	switch (cur_mode) 
	{
		case ALL_OFF:
			chip->shadow[TMG399X_CONTROL] = 0;
			chip->shadow[TMG399X_GES_CFG_4] &= ~TMG399X_GES_EN_IRQ;
			break;
		case ALS_ONLY:
			chip->shadow[TMG399X_CONTROL] = (TMG399X_EN_PWR_ON | TMG399X_EN_ALS | TMG399X_EN_ALS_IRQ);
			chip->shadow[TMG399X_GES_CFG_4] &= ~TMG399X_GES_EN_IRQ;
			break;
		case PROX_ONLY:
			chip->shadow[TMG399X_CONTROL] = (TMG399X_EN_PWR_ON | TMG399X_EN_ALS |
											TMG399X_EN_PRX | TMG399X_EN_PRX_IRQ);
			chip->shadow[TMG399X_GES_CFG_4] &= ~TMG399X_GES_EN_IRQ;
			break;
		case ALS_PROX:
			chip->shadow[TMG399X_CONTROL] = (TMG399X_EN_PWR_ON | TMG399X_EN_ALS | TMG399X_EN_ALS_IRQ |
											TMG399X_EN_PRX | TMG399X_EN_PRX_IRQ);
			chip->shadow[TMG399X_GES_CFG_4] &= ~TMG399X_GES_EN_IRQ;
			break;
		case GES_ONLY:
		case PROX_GES:
			chip->shadow[TMG399X_CONTROL] = (TMG399X_EN_PWR_ON | TMG399X_EN_ALS |
											TMG399X_EN_PRX | TMG399X_EN_PRX_IRQ | TMG399X_EN_GES);
                chip->shadow[TMG399X_GES_CFG_4] |= TMG399X_GES_EN_IRQ;
		
			break;
		case ALS_GES:
		case ALS_PROX_GES:
			chip->shadow[TMG399X_CONTROL] = (TMG399X_EN_PWR_ON | TMG399X_EN_ALS | TMG399X_EN_ALS_IRQ  |
											TMG399X_EN_PRX | TMG399X_EN_PRX_IRQ | TMG399X_EN_GES);
			chip->shadow[TMG399X_GES_CFG_4] |= TMG399X_GES_EN_IRQ;
			break;
		default:
			break;
	}
	
	ret = tmg399x_update_enable_reg(chip);
	
	return ret;
}

static void tmg399x_prox_enable(struct tmg399x_chip *chip, int on)
{

	dev_info(&chip->client->dev, "%s: on = %d\n", __func__, on);

	chip->shadow[TMG399X_CONTROL] = 0;
	tmg399x_update_enable_reg(chip);
	if (on)
	{
		set_visible_data_mode(chip);
	}
	else
	{
		chip->prx_inf.raw = 0;
	}

	tmg399x_set_op_mode(chip, PROX_FUNC, on);
	chip->prx_enabled = on;
    
}

static void tmg399x_ges_enable(struct tmg399x_chip *chip, int on)
{
	dev_info(&chip->client->dev, "%s: on = %d\n", __func__, on);
	
	chip->shadow[TMG399X_CONTROL] = 0;
	tmg399x_update_enable_reg(chip);
	if (on)
	{
		tmg399x_ges_init(chip,1);
		set_visible_data_mode(chip);
	}
	tmg399x_set_op_mode(chip, GES_FUNC, on);
	chip->ges_enabled= on;
}

static void tmg399x_als_enable(struct tmg399x_chip *chip, int on)
{

	dev_info(&chip->client->dev, "%s: on = %d\n", __func__, on);
	
	chip->shadow[TMG399X_CONTROL] = 0;
	tmg399x_update_enable_reg(chip);
	if (on) 
	{
		/* use auto gain setting */
		chip->als_gain_auto = false;
	} 
	else 
	{
		chip->als_inf.lux = 0;
		chip->als_inf.cct = 0;
	}
	tmg399x_set_op_mode(chip, ALS_FUNC, on);
	chip->als_enabled = on;

}


static ssize_t tmg399x_als_poll_delay_set(struct sensors_classdev *sensors_cdev,
						unsigned int delay_msec)
{
    return 0;
}
static ssize_t tmg399x_ps_enable_set(struct sensors_classdev *sensors_cdev,
						unsigned int enabled)
{
	struct tmg399x_chip *p_data = container_of(sensors_cdev,
						struct tmg399x_chip, ps_cdev);
       if (p_data == NULL) 
       {
           printk("tmg399x_ps_enable_set error \n");
           return -1;
       }
	//mutex_lock(&p_data->lock);
	tmg399x_prox_enable(p_data, enabled);
	//mutex_unlock(&p_data->lock);

	tmg399x_get_prox(p_data,p_data->prx_inf.raw);
    	input_report_abs(p_data->p_idev, ABS_DISTANCE,p_data->prx_inf.detected ? 0 : 1);
	input_sync(p_data->p_idev);

	return 0;
}

static ssize_t tmg399x_als_enable_set(struct sensors_classdev *sensors_cdev,
						unsigned int enabled)
{
	struct tmg399x_chip *p_data = container_of(sensors_cdev,
						struct tmg399x_chip, als_cdev);
       if (p_data == NULL) 
       {
           printk("tmg399x_als_enable_set error \n");
           return -1;
       }

	//mutex_lock(&p_data->lock);
	tmg399x_als_enable(p_data, enabled);
	//mutex_unlock(&p_data->lock);

	return 0;
}


static int tmg399x_wait_enable(struct tmg399x_chip *chip, int on)
{
	int ret;

	dev_info(&chip->client->dev, "%s: on = %d\n", __func__, on);
	if (on) {
		chip->shadow[TMG399X_CONTROL] |= TMG399X_EN_WAIT;

		ret = tmg399x_update_enable_reg(chip);
		if (ret < 0)
			return ret;
		mdelay(3);
	} else {
		chip->shadow[TMG399X_CONTROL] &= ~TMG399X_EN_WAIT;

		ret = tmg399x_update_enable_reg(chip);
		if (ret < 0)
			return ret;
	}
	if (!ret)
	        chip->wait_enabled = on;

	return ret;
}

static int tmg399x_beam_enable(struct tmg399x_chip *chip, int on)
{
	int rc;
	dev_info(&chip->client->dev, "%s: on = %d\n", __func__, on);
	if (on) {
		tmg399x_irq_clr(chip, TMG399X_CMD_IRBEAM_INT_CLR);
//		tmg399x_update_als_thres(chip, 1);
		chip->shadow[TMG399X_CONTROL] = (TMG399X_EN_PWR_ON | TMG399X_EN_BEAM);
		rc = tmg399x_update_enable_reg(chip);
		if (rc)
			return rc;
		mdelay(3);
	} 
	else
	{
		chip->shadow[TMG399X_CONTROL] &= ~TMG399X_EN_BEAM;
		if (!(chip->shadow[TMG399X_CONTROL] & TMG399X_EN_BEAM))
			chip->shadow[TMG399X_CONTROL] &= ~TMG399X_EN_PWR_ON;
		rc = tmg399x_update_enable_reg(chip);
		if (rc)
			return rc;
		tmg399x_irq_clr(chip, TMG399X_CMD_IRBEAM_INT_CLR);
	}
	if (!rc)
		chip->beam_enabled = on;

	return rc;
}

void tmg399x_update_prx_thd(struct tmg399x_chip *chip)
{
	int ret;
       //printk("%s  detected:%d  prx_raw:%d  \n ", __func__, chip->prx_inf.detected,chip->prx_inf.raw);
       //printk("%s  cur_thd  low_thd:%d  high_thd:%d \n", __func__, chip->shadow[TMG399X_PRX_MINTHRESHLO], chip->shadow[TMG399X_PRX_MAXTHRESHHI]);
	if (chip->prx_inf.detected)    // near
       {
			chip->shadow[TMG399X_PRX_MINTHRESHLO] = chip->pdata->parameters.prox_th_min;
			chip->shadow[TMG399X_PRX_MAXTHRESHHI] = 0xff;
			ret = tmg399x_i2c_write(chip, TMG399X_PRX_MINTHRESHLO,
					chip->shadow[TMG399X_PRX_MINTHRESHLO]);
			ret |= tmg399x_i2c_write(chip, TMG399X_PRX_MAXTHRESHHI,
					chip->shadow[TMG399X_PRX_MAXTHRESHHI]);
			if (ret < 0)
				dev_err(&chip->client->dev,"%s: failed to write proximity threshold\n",__func__);
		
	} 
       else 
       {
			chip->shadow[TMG399X_PRX_MINTHRESHLO] = 0x00;
			chip->shadow[TMG399X_PRX_MAXTHRESHHI] = chip->pdata->parameters.prox_th_max;
			ret = tmg399x_i2c_write(chip, TMG399X_PRX_MINTHRESHLO,
					chip->shadow[TMG399X_PRX_MINTHRESHLO]);
			ret |= tmg399x_i2c_write(chip, TMG399X_PRX_MAXTHRESHHI,
					chip->shadow[TMG399X_PRX_MAXTHRESHHI]);
			if (ret < 0)
				dev_err(&chip->client->dev,"%s: failed to write proximity threshold\n",__func__);

	}    
       //printk("%s  update  low_thd:%d  high_thd:%d \n", __func__, chip->shadow[TMG399X_PRX_MINTHRESHLO], chip->shadow[TMG399X_PRX_MAXTHRESHHI]);
}

void tmg399x_get_prox(struct tmg399x_chip *chip,u8 prox)
{
	int ret;
	printk("%s start ... \n", __func__);
	ret = tmg399x_i2c_read(chip, TMG399X_PRX_CHAN,
			&chip->shadow[TMG399X_PRX_CHAN]);
	if (ret < 0) {
		dev_err(&chip->client->dev,
			"%s: failed to read proximity raw data\n",
			__func__);
		return;
	} else
//	chip->shadow[TMG399X_PRX_CHAN] = prox;
	chip->prx_inf.raw = chip->shadow[TMG399X_PRX_CHAN];

	if (chip->prx_inf.detected == false) {
		if (chip->prx_inf.raw >= chip->shadow[TMG399X_PRX_MAXTHRESHHI]) {
			chip->prx_inf.detected = true;
			chip->shadow[TMG399X_PRX_MINTHRESHLO] =
					chip->pdata->parameters.prox_th_min;
			chip->shadow[TMG399X_PRX_MAXTHRESHHI] = 0xff;
			ret = tmg399x_i2c_write(chip, TMG399X_PRX_MINTHRESHLO,
					chip->shadow[TMG399X_PRX_MINTHRESHLO]);
			ret |= tmg399x_i2c_write(chip, TMG399X_PRX_MAXTHRESHHI,
					chip->shadow[TMG399X_PRX_MAXTHRESHHI]);
			if (ret < 0)
				dev_err(&chip->client->dev,
					"%s: failed to write proximity threshold\n",
					__func__);
		}
	} else {
		if (chip->prx_inf.raw <= chip->shadow[TMG399X_PRX_MINTHRESHLO]) {
			chip->prx_inf.detected = false;
			chip->shadow[TMG399X_PRX_MINTHRESHLO] = 0x00;
			chip->shadow[TMG399X_PRX_MAXTHRESHHI] =
					chip->pdata->parameters.prox_th_max;
			ret = tmg399x_i2c_write(chip, TMG399X_PRX_MINTHRESHLO,
					chip->shadow[TMG399X_PRX_MINTHRESHLO]);
			ret |= tmg399x_i2c_write(chip, TMG399X_PRX_MAXTHRESHHI,
					chip->shadow[TMG399X_PRX_MAXTHRESHHI]);
			if (ret < 0)
				dev_err(&chip->client->dev,
					"%s: failed to write proximity threshold\n",
					__func__);
		}
	}

	tmg399x_report_prox(chip,chip->prx_inf.detected );
	
	dev_info(&chip->client->dev, "prx_inf.raw=%d,prox_th_min=%d,prox_th_max=%d\n",chip->prx_inf.raw,chip->shadow[TMG399X_PRX_MINTHRESHLO],chip->shadow[TMG399X_PRX_MAXTHRESHHI]);
	
}

EXPORT_SYMBOL_GPL(tmg399x_get_prox);

static int tmg399x_read_rgb(struct tmg399x_chip *chip)
{
	int ret;

	ret = tmg399x_i2c_read(chip, TMG399X_CLR_CHANLO,
			&chip->shadow[TMG399X_CLR_CHANLO]);
	ret |= tmg399x_i2c_read(chip, TMG399X_CLR_CHANHI,
			&chip->shadow[TMG399X_CLR_CHANHI]);

	ret |= tmg399x_i2c_read(chip, TMG399X_RED_CHANLO,
			&chip->shadow[TMG399X_RED_CHANLO]);
	ret |= tmg399x_i2c_read(chip, TMG399X_RED_CHANHI,
			&chip->shadow[TMG399X_RED_CHANHI]);

	ret |= tmg399x_i2c_read(chip, TMG399X_GRN_CHANLO,
			&chip->shadow[TMG399X_GRN_CHANLO]);
	ret |= tmg399x_i2c_read(chip, TMG399X_GRN_CHANHI,
			&chip->shadow[TMG399X_GRN_CHANHI]);

	ret |= tmg399x_i2c_read(chip, TMG399X_BLU_CHANLO,
			&chip->shadow[TMG399X_BLU_CHANLO]);
	ret |= tmg399x_i2c_read(chip, TMG399X_BLU_CHANHI,
			&chip->shadow[TMG399X_BLU_CHANHI]);

	return (ret < 0) ? ret : 0;
}


static void tmg399x_get_als(struct tmg399x_chip *chip)
{
	int ret;

	ret = tmg399x_read_rgb(chip);
	if (ret < 0) {
		dev_err(&chip->client->dev,
			"%s: failed to read rgb raw data\n",
			__func__);
		return;
	} else {
		u8 *buf = &chip->shadow[TMG399X_CLR_CHANLO];
		/* extract raw channel data */
		chip->als_inf.clear_raw = le16_to_cpup((const __le16 *)&buf[0]);
		chip->als_inf.red_raw = le16_to_cpup((const __le16 *)&buf[2]);
		chip->als_inf.green_raw = le16_to_cpup((const __le16 *)&buf[4]);
		chip->als_inf.blue_raw = le16_to_cpup((const __le16 *)&buf[6]);
		chip->als_inf.ir =
			(chip->als_inf.red_raw + chip->als_inf.green_raw +
			chip->als_inf.blue_raw - chip->als_inf.clear_raw + 1) >> 1;
		if (chip->als_inf.ir < 0)
			chip->als_inf.ir = 0;
	}
}


static ssize_t tmg399x_ges_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->ges_enabled);
}

static ssize_t tmg399x_ges_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	bool value;

	mutex_lock(&chip->lock);
	if (strtobool(buf, &value)) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (value)
		tmg399x_ges_enable(chip, 1);
	else
		tmg399x_ges_enable(chip, 0);

	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmg399x_prox_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->prx_enabled);
}

static ssize_t tmg399x_prox_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	bool value;

	mutex_lock(&chip->lock);
	if (strtobool(buf, &value)) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (value)
		tmg399x_prox_enable(chip, 1);
	else
		tmg399x_prox_enable(chip, 0);

	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmg399x_als_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_enabled);
}

static ssize_t tmg399x_als_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	bool value;

	mutex_lock(&chip->lock);
	if (strtobool(buf, &value)) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (value)
		tmg399x_als_enable(chip, 1);
	else
		tmg399x_als_enable(chip, 0);

	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmg399x_wait_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->wait_enabled);
}

static ssize_t tmg399x_wait_enable_store(struct device *dev,
                                struct device_attribute *attr,
                                const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	bool value;

	mutex_lock(&chip->lock);
	if (strtobool(buf, &value)) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (value)
		tmg399x_wait_enable(chip, 1);
	else
		tmg399x_wait_enable(chip, 0);

		mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmg399x_als_itime_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	int t;
	t = 256 - chip->params.als_time;
	t *= TMG399X_ATIME_PER_100;
	t /= 100;
	return snprintf(buf, PAGE_SIZE, "%d (in ms)\n", t);
}

static ssize_t tmg399x_als_itime_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long itime;
	int ret;

	mutex_lock(&chip->lock);
	ret = kstrtoul(buf, 10, &itime);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}
	if (itime > 712 || itime < 3) {
		dev_err(&chip->client->dev,
			"als integration time range [3,712]\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	itime *= 100;
	itime /= TMG399X_ATIME_PER_100;
	itime = (256 - itime);
	chip->shadow[TMG399X_ALS_TIME] = (u8)itime;
	chip->params.als_time = (u8)itime;
	ret = tmg399x_i2c_write(chip, TMG399X_ALS_TIME,
		chip->shadow[TMG399X_ALS_TIME]);
	mutex_unlock(&chip->lock);
	return ret ? ret : size;
}

static ssize_t tmg399x_wait_time_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	int t;
	t = 256 - chip->params.wait_time;
	t *= TMG399X_ATIME_PER_100;
	t /= 100;
	if (chip->params.als_prox_cfg1 & WLONG)
		t *= 12;
	return snprintf(buf, PAGE_SIZE, "%d (in ms)\n", t);
}

static ssize_t tmg399x_wait_time_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long time;
	int ret;
	u8 cfg1;

	mutex_lock(&chip->lock);
	ret = kstrtoul(buf, 10, &time);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}
	if (time > 8540 || time < 3) {
		dev_err(&chip->client->dev,
			"wait time range [3,8540]\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}
	
	cfg1 = chip->shadow[TMG399X_CONFIG_1] & ~0x02;
	if (time > 712) {
		cfg1 |= WLONG;
		time /= 12;
	}

	time *= 100;
	time /= TMG399X_ATIME_PER_100;
	time = (256 - time);
	chip->shadow[TMG399X_WAIT_TIME] = (u8)time;
	chip->params.wait_time = (u8)time;
	chip->shadow[TMG399X_CONFIG_1] = cfg1;
	chip->params.als_prox_cfg1 = cfg1;
	ret = tmg399x_i2c_write(chip, TMG399X_WAIT_TIME,
		chip->shadow[TMG399X_WAIT_TIME]);
	ret |= tmg399x_i2c_write(chip, TMG399X_CONFIG_1,
		chip->shadow[TMG399X_CONFIG_1]);
	mutex_unlock(&chip->lock);
	return ret ? ret : size;
}

static ssize_t tmg399x_prox_persist_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n",
			(chip->params.persist & 0xF0) >> 4);
}

static ssize_t tmg399x_prox_persist_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long persist;
	int ret;

	mutex_lock(&chip->lock);
	ret = kstrtoul(buf, 10, &persist);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}
	
	if (persist > 15) {
		dev_err(&chip->client->dev,
			"prox persistence range [0,15]\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}
	
	chip->shadow[TMG399X_PERSISTENCE] &= 0x0F;
	chip->shadow[TMG399X_PERSISTENCE] |= (((u8)persist << 4) & 0xF0);
	chip->params.persist = chip->shadow[TMG399X_PERSISTENCE];
	ret = tmg399x_i2c_write(chip, TMG399X_PERSISTENCE,
		chip->shadow[TMG399X_PERSISTENCE]);
	mutex_unlock(&chip->lock);
	return ret ? ret : size;
}

static ssize_t tmg399x_als_persist_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n",
			(chip->params.persist & 0x0F));
}

static ssize_t tmg399x_als_persist_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long persist;
	int ret;

	mutex_lock(&chip->lock);
	ret = kstrtoul(buf, 10, &persist);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (persist > 15) {
		dev_err(&chip->client->dev,
			"als persistence range [0,15]\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	chip->shadow[TMG399X_PERSISTENCE] &= 0xF0;
	chip->shadow[TMG399X_PERSISTENCE] |= ((u8)persist & 0x0F);
	chip->params.persist = chip->shadow[TMG399X_PERSISTENCE];
	ret = tmg399x_i2c_write(chip, TMG399X_PERSISTENCE,
		chip->shadow[TMG399X_PERSISTENCE]);
	mutex_unlock(&chip->lock);
	return ret ? ret : size;
}

static ssize_t tmg399x_prox_pulse_len_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	int i = (chip->params.prox_pulse & 0xC0) >> 6;
	return snprintf(buf, PAGE_SIZE, "%duS\n", prox_pplens[i]);
}

static ssize_t tmg399x_prox_pulse_len_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long length;
	int ret;
	u8 ppulse;

	mutex_lock(&chip->lock);
	ret = kstrtoul(buf, 10, &length);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (length != 4 && length != 8 &&
		length != 16 &&	length != 32) {
		dev_err(&chip->client->dev, 
			"pulse length set: {4, 8, 16, 32}\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}
	
	ppulse = chip->shadow[TMG399X_PRX_PULSE] & 0x3F;
	switch (length){
	case 4:
		ppulse |= PPLEN_4US;
		break;
	case 8:
		ppulse |= PPLEN_8US;
		break;
	case 16:
		ppulse |= PPLEN_16US;
		break;
	case 32:
		ppulse |= PPLEN_32US;
		break;
	default:
		break;
	}
	chip->shadow[TMG399X_PRX_PULSE] = ppulse;
	chip->params.prox_pulse = ppulse;
	ret = tmg399x_i2c_write(chip, TMG399X_PRX_PULSE,
		chip->shadow[TMG399X_PRX_PULSE]);
	mutex_unlock(&chip->lock);
	return ret ? ret : size;
}

static ssize_t tmg399x_prox_pulse_cnt_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n",
			(chip->params.prox_pulse & 0x3F) + 1);
}

static ssize_t tmg399x_prox_pulse_cnt_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long count;
	int ret;

	mutex_lock(&chip->lock);
	ret = kstrtoul(buf, 10, &count);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (count > 64 || count == 0) {
		dev_err(&chip->client->dev,
			"prox pulse count range [1,64]\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}
	count -= 1;

	chip->shadow[TMG399X_PRX_PULSE] &= 0xC0;
	chip->shadow[TMG399X_PRX_PULSE] |= ((u8)count & 0x3F);
	chip->params.prox_pulse = chip->shadow[TMG399X_PRX_PULSE];
	ret = tmg399x_i2c_write(chip, TMG399X_PRX_PULSE,
		chip->shadow[TMG399X_PRX_PULSE]);
	mutex_unlock(&chip->lock);
	return ret ? ret : size;
}

static ssize_t tmg399x_prox_gain_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	int i = chip->params.prox_gain >> 2;
	return snprintf(buf, PAGE_SIZE, "%d\n", prox_gains[i]);
}

static ssize_t tmg399x_prox_gain_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long gain;
	int ret;
	u8 ctrl_reg;

	mutex_lock(&chip->lock);
	ret = kstrtoul(buf, 10, &gain);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (gain != 1 && gain != 2 && gain != 4 && gain != 8) {
		dev_err(&chip->client->dev,
			"prox gain set: {1, 2, 4, 8}\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	ctrl_reg = chip->shadow[TMG399X_GAIN] & ~TMG399X_PRX_GAIN_MASK;
	switch (gain){
	case 1:
		ctrl_reg |= GPGAIN_1;
		break;
	case 2:
		ctrl_reg |= GPGAIN_2;
		break;
	case 4:
		ctrl_reg |= GPGAIN_4;
		break;
	case 8:
		ctrl_reg |= GPGAIN_8;
		break;
	default:
		break;
	}

	ret = tmg399x_i2c_write(chip, TMG399X_GAIN, ctrl_reg);
	if (!ret) {
		chip->shadow[TMG399X_GAIN] = ctrl_reg;
		chip->params.prox_gain = ctrl_reg & TMG399X_PRX_GAIN_MASK;
	}
	mutex_unlock(&chip->lock);
	return ret ? ret : size;
}

static ssize_t tmg399x_led_drive_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%dmA\n",
			led_drives[chip->params.ldrive]);
}

static ssize_t tmg399x_led_drive_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long ldrive;
	int ret;
	u8 ctrl_reg;

	mutex_lock(&chip->lock);
	ret = kstrtoul(buf, 10, &ldrive);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (ldrive != 100 && ldrive != 50 &&
		ldrive != 25 && ldrive != 12) {
		dev_err(&chip->client->dev,
			"led drive set: {100, 50, 25, 12}\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	ctrl_reg = chip->shadow[TMG399X_GAIN] & ~TMG399X_LDRIVE_MASK;
	switch (ldrive){
	case 100:
		ctrl_reg |= GPDRIVE_100MA;
		chip->params.ldrive = 0;
		break;
	case 50:
		ctrl_reg |= GPDRIVE_50MA;
		chip->params.ldrive = 1;
		break;
	case 25:
		ctrl_reg |= GPDRIVE_25MA;
		chip->params.ldrive = 2;
		break;
	case 12:
		ctrl_reg |= GPDRIVE_12MA;
		chip->params.ldrive = 3;
		break;
	default:
		break;
	}
	chip->shadow[TMG399X_GAIN] = ctrl_reg;
	ret = tmg399x_i2c_write(chip, TMG399X_GAIN, chip->shadow[TMG399X_GAIN]);
	mutex_unlock(&chip->lock);
	return ret ? ret : size;
}

static ssize_t tmg399x_als_gain_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d (%s)\n",
			als_gains[chip->params.als_gain],
			chip->als_gain_auto ? "auto" : "manual");
}

static ssize_t tmg399x_als_gain_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long gain;
	int i = 0;
	int ret;

	mutex_lock(&chip->lock);
	ret = kstrtoul(buf, 10, &gain);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}
		
	if (gain != 0 && gain != 1 && gain != 4 &&
		gain != 16 && gain != 64) {
		dev_err(&chip->client->dev,
			"als gain set: {0(auto), 1, 4, 16, 64}\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	while (i < sizeof(als_gains)) {
		if (gain == als_gains[i])
			break;
		i++;
	}

	if (gain) {
		chip->als_gain_auto = false;
		ret = tmg399x_set_als_gain(chip, als_gains[i]);
		if (!ret)
			tmg399x_calc_cpl(chip);
	} else {
		chip->als_gain_auto = true;
	}
	mutex_unlock(&chip->lock);
	return ret ? ret : size;
}

static ssize_t tmg399x_led_boost_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	int i = (chip->params.als_prox_cfg2 & 0x30) >> 4;
	return snprintf(buf, PAGE_SIZE, "%d percents\n", led_boosts[i]);
}

static ssize_t tmg399x_led_boost_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long lboost;
	int ret;
	u8 cfg2;

	mutex_lock(&chip->lock);
	ret = kstrtoul(buf, 10, &lboost);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (lboost != 100 && lboost != 150 &&
		lboost != 200 && lboost != 300) {
		dev_err(&chip->client->dev,
			"led boost set: {100, 150, 200, 300}\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	cfg2 = chip->shadow[TMG399X_CONFIG_2] & ~0x30;
	switch (lboost){
	case 100:
		cfg2 |= LEDBOOST_100;
		break;
	case 150:
		cfg2 |= LEDBOOST_150;
		break;
	case 200:
		cfg2 |= LEDBOOST_200;
		break;
	case 300:
		cfg2 |= LEDBOOST_300;
		break;
	default:
		break;
	}
	chip->shadow[TMG399X_CONFIG_2] = cfg2;
	chip->params.als_prox_cfg2 = cfg2;
	ret = tmg399x_i2c_write(chip, TMG399X_CONFIG_2,
		chip->shadow[TMG399X_CONFIG_2]);
	mutex_unlock(&chip->lock);
	return ret ? ret : size;
}

static ssize_t tmg399x_sat_irq_en_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n",
			(chip->params.als_prox_cfg2 & 0x80) >> 7);
}

static ssize_t tmg399x_sat_irq_en_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	bool psien;
	int ret;

	mutex_lock(&chip->lock);
	if (strtobool(buf, &psien)) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	chip->shadow[TMG399X_CONFIG_2] &= 0x7F;
	if (psien)
		chip->shadow[TMG399X_CONFIG_2] |= PSIEN;
	chip->params.als_prox_cfg2 = chip->shadow[TMG399X_CONFIG_2];
	ret = tmg399x_i2c_write(chip, TMG399X_CONFIG_2,
		chip->shadow[TMG399X_CONFIG_2]);
	mutex_unlock(&chip->lock);
	return ret ? ret : size;
}

static ssize_t tmg399x_prox_offset_ne_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n",
			chip->params.prox_offset_ne);
}

static ssize_t tmg399x_prox_offset_ne_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	long offset_ne;
	int ret;
	u8 offset = 0;

	mutex_lock(&chip->lock);
	ret = kstrtol(buf, 10, &offset_ne);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (offset_ne > 127 || offset_ne < -127) {
		dev_err(&chip->client->dev, "prox offset range [-127, 127]\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}
	if (offset_ne < 0)
		offset = 128 - offset_ne;
	else
		offset = offset_ne;

	ret = tmg399x_i2c_write(chip, TMG399X_PRX_OFFSET_NE, offset);
	if (!ret) {
		chip->params.prox_offset_ne = (s8)offset_ne;
		chip->shadow[TMG399X_PRX_OFFSET_NE] = offset;
	}
	mutex_unlock(&chip->lock);
	return ret ? ret : size;
}

static ssize_t tmg399x_prox_offset_sw_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n",
			chip->params.prox_offset_sw);
}

static ssize_t tmg399x_prox_offset_sw_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	long offset_sw;
	int ret;
	u8 offset = 0;

	mutex_lock(&chip->lock);
	ret = kstrtol(buf, 10, &offset_sw);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (offset_sw > 127 || offset_sw < -127) {
		dev_err(&chip->client->dev, "prox offset range [-127, 127]\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}
	if (offset_sw < 0)
		offset = 128 - offset_sw;
	else
		offset = offset_sw;

	ret = tmg399x_i2c_write(chip, TMG399X_PRX_OFFSET_SW, offset);
	if (!ret) {
		chip->params.prox_offset_sw = (s8)offset_sw;
		chip->shadow[TMG399X_PRX_OFFSET_SW] = offset;
	}
	mutex_unlock(&chip->lock);
	return ret ? ret : size;
}

static ssize_t tmg399x_prox_mask_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%.2x\n",
			chip->params.als_prox_cfg3 & 0x0F);
}

static ssize_t tmg399x_prox_mask_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long prx_mask;
	int ret;

	mutex_lock(&chip->lock);
	ret = kstrtol(buf, 10, &prx_mask);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (prx_mask > 15) {
		dev_err(&chip->client->dev, "prox mask range [0, 15]\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}
	if ((prx_mask == 5) || (prx_mask == 7) ||
		(prx_mask == 10) || (prx_mask == 11) ||
		(prx_mask == 13) || (prx_mask == 14))
		prx_mask |= PCMP;

	chip->shadow[TMG399X_CONFIG_3] &= 0xD0;
	chip->shadow[TMG399X_CONFIG_3] |= (u8)prx_mask;
	chip->params.als_prox_cfg3 = chip->shadow[TMG399X_CONFIG_3];
	ret = tmg399x_i2c_write(chip, TMG399X_CONFIG_3,
		chip->shadow[TMG399X_CONFIG_3]);
	mutex_unlock(&chip->lock);
	return ret ? ret : size;
}

static ssize_t tmg399x_device_prx_raw(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);

	mutex_lock(&chip->lock);
	tmg399x_i2c_read(chip, TMG399X_PRX_CHAN,&chip->shadow[TMG399X_PRX_CHAN]);
	mutex_unlock(&chip->lock);

	chip->prx_inf.raw = chip->shadow[TMG399X_PRX_CHAN];
	buf[0] = chip->prx_inf.raw;
	
	//return 1 ;
	
      return snprintf(buf, PAGE_SIZE, "%d\n", chip->prx_inf.raw);
}

static ssize_t tmg399x_device_prx_detected(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);

	mutex_lock(&chip->lock);
	tmg399x_get_prox(chip,chip->prx_inf.raw);
	mutex_unlock(&chip->lock);

	buf[0] = chip->prx_inf.detected;
	
	//return 1 ;
	
       return snprintf(buf, PAGE_SIZE, "%d\n", chip->prx_inf.detected);
}

static ssize_t tmg399x_ges_offset_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "n:%d s:%d w:%d e:%d\n",
			chip->params.ges_offset_n, chip->params.ges_offset_s,
			chip->params.ges_offset_w, chip->params.ges_offset_e);
}

static ssize_t tmg399x_lux_table_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	struct tmg_lux_segment *s = chip->segment;
	int i, k;

	for (i = k = 0; i < chip->segment_num; i++)
		k += snprintf(buf + k, PAGE_SIZE - k,
				"%d:%d,%d,%d,%d,%d,%d\n", i,
				s[i].d_factor,
				s[i].r_coef,
				s[i].g_coef,
				s[i].b_coef,
				s[i].ct_coef,
				s[i].ct_offset
				);
	return k;
}

static ssize_t tmg399x_lux_table_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	int i;
	u32 d_factor, r_coef, g_coef, b_coef, ct_coef, ct_offset;

	mutex_lock(&chip->lock);
	if (7 != sscanf(buf, "%10d:%10d,%10d,%10d,%10d,%10d,%10d",
		&i, &d_factor, &r_coef, &g_coef, &b_coef, &ct_coef, &ct_offset)) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}
	if (i >= chip->segment_num) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}
	chip->segment[i].d_factor = d_factor;
	chip->segment[i].r_coef = r_coef;
	chip->segment[i].g_coef = g_coef;
	chip->segment[i].b_coef = b_coef;
	chip->segment[i].ct_coef = ct_coef;
	chip->segment[i].ct_offset = ct_offset;
	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmg399x_auto_gain_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%s\n",
				chip->als_gain_auto ? "auto" : "manual");
}

static ssize_t tmg399x_auto_gain_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	bool value;

	mutex_lock(&chip->lock);
	if (strtobool(buf, &value)) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (value)
		chip->als_gain_auto = true;
	else
		chip->als_gain_auto = false;

	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmg399x_als_lux_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);

	mutex_lock(&chip->lock);
	tmg399x_get_als(chip);
	tmg399x_get_lux(chip);
	mutex_unlock(&chip->lock);

	buf[0] = chip->als_inf.lux % 256;
	buf[1] = chip->als_inf.lux /256;
	buf[2] = '\0';

	dev_info(&chip->client->dev, "chip->als_inf.lux = %d\n",chip->als_inf.lux);
	return 3;
	
//	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.lux);
}

static ssize_t tmg399x_als_red_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);

	mutex_lock(&chip->lock);
	tmg399x_get_als(chip);
	mutex_unlock(&chip->lock);
	buf[0] = chip->als_inf.red_raw % 256;
	buf[1] = chip->als_inf.red_raw /256;
	buf[2] = '\0';

	dev_info(&chip->client->dev, "chip->als_inf.red_raw = %d\n",chip->als_inf.red_raw);
	return 3;
	
//	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.red_raw);
}

static ssize_t tmg399x_als_green_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);

	mutex_lock(&chip->lock);
	tmg399x_get_als(chip);
	mutex_unlock(&chip->lock);
	buf[0] = chip->als_inf.green_raw % 256;
	buf[1] = chip->als_inf.green_raw /256;
	buf[2] = '\0';

	dev_info(&chip->client->dev, "chip->als_inf.green_raw = %d\n",chip->als_inf.green_raw);
	return 3;	
//	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.green_raw);
}

static ssize_t tmg399x_als_blue_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);

	mutex_lock(&chip->lock);
	tmg399x_get_als(chip);
	mutex_unlock(&chip->lock);
	buf[0] = chip->als_inf.blue_raw % 256;
	buf[1] = chip->als_inf.blue_raw /256;
	buf[2] = '\0';

	dev_info(&chip->client->dev, "chip->als_inf.blue_raw = %d\n",chip->als_inf.blue_raw);
	return 3;
	
//	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.blue_raw);
}

static ssize_t tmg399x_als_clear_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);

	mutex_lock(&chip->lock);
	tmg399x_get_als(chip);
	mutex_unlock(&chip->lock);
	buf[0] = chip->als_inf.clear_raw % 256;
	buf[1] = chip->als_inf.clear_raw /256;
	buf[2] = '\0';

	dev_info(&chip->client->dev, "chip->als_inf.clear_raw = %d\n",chip->als_inf.clear_raw);
	return 3;
//	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.clear_raw);
}

static ssize_t tmg399x_als_cct_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);

	mutex_lock(&chip->lock);
	tmg399x_get_als(chip);
	tmg399x_get_lux(chip);
	mutex_unlock(&chip->lock);

	buf[0] = chip->als_inf.cct % 256;
	buf[1] = chip->als_inf.cct /256;
	buf[2] = '\0';

	dev_info(&chip->client->dev, "chip->als_inf.cct = %d\n",chip->als_inf.cct);
	return 3;		
//	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.cct);
}

static ssize_t tmg399x_beam_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	bool value;

	if (strtobool(buf, &value))
		return -EINVAL;

	if (value)
		tmg399x_beam_enable(chip, 1);
	else
		tmg399x_beam_enable(chip, 0);

	return size;
}

static ssize_t tmg399x_xmit_beam(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	u8 times2xmit;
	int rc;
	struct tmg399x_chip *chip = dev_get_drvdata(dev);

	rc = kstrtou8(buf, 10, &times2xmit);
	if (rc)
		return -EINVAL;

	if (times2xmit == 0) {
		/* Shut down Beam */
		chip->hop_count = 0;
		goto stop_beam;
	} else if (times2xmit >= 255)
		times2xmit = 255;
	else
		times2xmit -= 1;

	chip->hop_count = times2xmit;
	schedule_work(&chip->work_thresh);

stop_beam:
	return size;
}

static ssize_t tmg399x_beam_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n",	chip->beam_enabled);
}

static ssize_t tmg399x_beam_hop_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int i;
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	int bw, ns, isd, np, ipd;

	if (6 != sscanf(buf, "%10d:%10d,%10d,%10d,%10d,%10d",
		&i, &bw, &ns, &isd, &np, &ipd))
		return -EINVAL;

	if ((i > IRBEAM_MAX_NUM_HOPS) || (i > chip->hop_next_slot + 1))
		return -EINVAL;
	mutex_lock(&chip->lock);
		chip->beam_hop[i].beam_cfg  = IRBEAM_CFG;
		chip->beam_hop[i].beam_carr = IRBEAM_CARR;
		chip->beam_hop[i].beam_div = (u8)bw;
		chip->beam_hop[i].beam_ns  = (u8)ns;
		chip->beam_hop[i].beam_isd = (u8)isd;
		chip->beam_hop[i].beam_np  = (u8)np;
		chip->beam_hop[i].beam_ipd = (u8)ipd;
	mutex_unlock(&chip->lock);
//	if ((chip->hop_next_slot >= 0) && (i != 0))
	if (i != 0)
		chip->hop_next_slot++;
	return size;
}

static ssize_t tmg399x_beam_hop_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	int i = 0;
	int k = 0;

	for (i = k = 0; i < 15; i++)
		k += snprintf(buf + k, PAGE_SIZE - k,
				"%d:%d,%d,%d,%d,%d\n", i,
				chip->beam_hop[i].beam_div,
				chip->beam_hop[i].beam_ns,
				chip->beam_hop[i].beam_isd,
				chip->beam_hop[i].beam_np,
				chip->beam_hop[i].beam_ipd
				);

	return k;
}

static void tmg399x_beam_thread(struct work_struct *beam)
{
	struct tmg399x_chip *chip = container_of(beam, struct tmg399x_chip, work_thresh);
	tmg399x_beam_xmit(chip);
}


static ssize_t tmg399x_ges_touch_style_store(struct device *dev,struct device_attribute *attr,
				const char *buf, size_t size)
{
//	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	bool temp_ges_style;
	
	if (strtobool(buf, &temp_ges_style)) {
		return -EINVAL;
	}

	ges_touch_style = temp_ges_style;

	return size;
}

static ssize_t tmg399x_ges_touch_dir8_store(struct device *dev,struct device_attribute *attr,
				const char *buf, size_t size)
{
//	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	bool temp_ges_dir8;
	
	if (strtobool(buf, &temp_ges_dir8)) {
		return -EINVAL;
	}

	ges_dir8_enable = temp_ges_dir8;

	return size;
}

static ssize_t tmg399x_ges_touch_tap_store(struct device *dev,struct device_attribute *attr,
				const char *buf, size_t size)
{
//	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	bool temp_ges_tap;
	
	if (strtobool(buf, &temp_ges_tap)) {
		return -EINVAL;
	}

	ges_tap_enable = temp_ges_tap;

	return size;
}

static ssize_t tmg399x_ges_motion_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	
	buf[0] = ges_motion;
	
	return 1 ;
//	return snprintf(buf, PAGE_SIZE, "%d\n", chip->prx_inf.raw);
}

static ssize_t tmg399x_ges_work_mode_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	
	buf[0] = work_mode;
	
	return 1 ;
//	return snprintf(buf, PAGE_SIZE, "%d\n", chip->prx_inf.raw);
}

static ssize_t tmg399x_ges_work_mode_store(struct device *dev,
				struct device_attribute *attr,const char *buf, size_t size)
{
	u8 temp;
	int rc;
//	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	rc = kstrtou8(buf, 10, &temp);
	if (rc)
		return -EINVAL;
	work_mode = temp;
	
	dev_info(dev, "%s: work_mode value= %d\n", __func__,work_mode);
	
	return size;
}


static struct device_attribute prx_power_state =	__ATTR(prx_power_state, 0666, tmg399x_prox_enable_show,tmg399x_prox_enable_store);

static struct device_attribute ges_power_state =		__ATTR(ges_power_state, 0666, tmg399x_ges_enable_show,tmg399x_ges_enable_store);
static struct device_attribute prx_persist =		__ATTR(prx_persist, 0666, tmg399x_prox_persist_show,tmg399x_prox_persist_store);
static struct device_attribute prx_pulse_length =		__ATTR(prx_pulse_length, 0666, tmg399x_prox_pulse_len_show,tmg399x_prox_pulse_len_store);
static struct device_attribute prx_pulse_count =		__ATTR(prx_pulse_count, 0666, tmg399x_prox_pulse_cnt_show,tmg399x_prox_pulse_cnt_store);
static struct device_attribute prx_gain =		__ATTR(prx_gain, 0666, tmg399x_prox_gain_show,tmg399x_prox_gain_store);
static struct device_attribute led_drive =		__ATTR(led_drive, 0666, tmg399x_led_drive_show,tmg399x_led_drive_store);
static struct device_attribute prx_poweled_boostr_state =		__ATTR(led_boost, 0666, tmg399x_led_boost_show,tmg399x_led_boost_store);
static struct device_attribute prx_sat_irq_en =		__ATTR(prx_sat_irq_en, 0666, tmg399x_sat_irq_en_show,tmg399x_sat_irq_en_store);
static struct device_attribute prx_offset_ne =		__ATTR(prx_offset_ne, 0666, tmg399x_prox_offset_ne_show,tmg399x_prox_offset_ne_store);
static struct device_attribute prx_offset_sw =		__ATTR(prx_offset_sw, 0666, tmg399x_prox_offset_sw_show,tmg399x_prox_offset_sw_store);
static struct device_attribute prx_mask =		__ATTR(prx_mask, 0666, tmg399x_prox_mask_show,tmg399x_prox_mask_store);

static struct device_attribute prx_raw =		__ATTR(prx_raw, 0666, tmg399x_device_prx_raw, NULL);
static struct device_attribute prx_detect =		__ATTR(prx_detect, 0666, tmg399x_device_prx_detected, NULL);


static struct device_attribute ges_offset =			__ATTR(ges_offset, 0666, tmg399x_ges_offset_show, NULL);
static struct device_attribute ges_motion_state =			__ATTR(ges_motion_state, 0666, tmg399x_ges_motion_show, NULL);
static struct device_attribute ges_work_mode =			__ATTR(ges_work_mode, 0666, tmg399x_ges_work_mode_show,tmg399x_ges_work_mode_store);
static struct device_attribute ges_touch_style_state =			__ATTR(ges_touch_style_state, 0666, NULL,tmg399x_ges_touch_style_store);
static struct device_attribute ges_touch_dir8_state =			__ATTR(ges_touch_dir8_state, 0666, NULL,tmg399x_ges_touch_dir8_store);
static struct device_attribute ges_touch_tap_state =			__ATTR(ges_touch_tap_state, 0666, NULL,tmg399x_ges_touch_tap_store);
	
static struct device_attribute beam_xmit =			__ATTR(beam_xmit, 0666, NULL, tmg399x_xmit_beam);
static struct device_attribute beam_table =			__ATTR(beam_table, 0666, NULL, tmg399x_beam_symbol_store);
static struct device_attribute beam_hop =			__ATTR(beam_hop, 0666, tmg399x_beam_hop_show, tmg399x_beam_hop_store);
static struct device_attribute beam_power_state =			__ATTR(beam_power_state, 0666, tmg399x_beam_enable_show,tmg399x_beam_enable_store);



static struct attribute *prox_attrs [] =
{
    &prx_power_state.attr,
    &ges_power_state.attr,
    &prx_persist.attr,
    &prx_pulse_length.attr,
    &prx_pulse_count.attr,
    &prx_gain.attr,
    &led_drive.attr,
    &prx_poweled_boostr_state.attr,
    &prx_sat_irq_en.attr,
    &prx_offset_ne.attr,
    &prx_offset_sw.attr,
    &prx_mask.attr,
    &prx_raw.attr,
    &prx_detect.attr,
    &ges_offset.attr,    
    &ges_motion_state.attr,
    &ges_work_mode.attr,
    &ges_touch_style_state.attr,
    &ges_touch_dir8_state.attr,
    &ges_touch_tap_state.attr,
    &beam_xmit.attr,    
    &beam_table.attr,    
    &beam_hop.attr,    
    &beam_power_state.attr,         
    NULL
};



static struct device_attribute als_power_state =		__ATTR(als_power_state, 0666, tmg399x_als_enable_show,tmg399x_als_enable_store);
static struct device_attribute wait_time_en =		__ATTR(wait_time_en, 0666, tmg399x_wait_enable_show,tmg399x_wait_enable_store);
static struct device_attribute als_Itime =		__ATTR(als_Itime, 0666, tmg399x_als_itime_show,tmg399x_als_itime_store);
static struct device_attribute wait_time =		__ATTR(wait_time, 0666, tmg399x_wait_time_show,tmg399x_wait_time_store);
static struct device_attribute als_persist =		__ATTR(als_persist, 0666, tmg399x_als_persist_show,tmg399x_als_persist_store);
static struct device_attribute als_gain =		__ATTR(als_gain, 0666, tmg399x_als_gain_show,tmg399x_als_gain_store);
static struct device_attribute lux_table =		__ATTR(lux_table, 0666, tmg399x_lux_table_show,tmg399x_lux_table_store);
static struct device_attribute als_auto_gain =		__ATTR(als_auto_gain, 0666, tmg399x_auto_gain_enable_show,tmg399x_auto_gain_enable_store);

static struct device_attribute als_lux =	__ATTR(als_lux, 0666, tmg399x_als_lux_show, NULL);

static struct device_attribute als_red =		__ATTR(als_red, 0666, tmg399x_als_red_show, NULL);
static struct device_attribute als_green =		__ATTR(als_green, 0666, tmg399x_als_green_show, NULL);
static struct device_attribute als_blue =		__ATTR(als_blue, 0666, tmg399x_als_blue_show, NULL);
static struct device_attribute als_clear =		__ATTR(als_clear, 0666, tmg399x_als_clear_show, NULL);
static struct device_attribute als_cct =		__ATTR(als_cct, 0666, tmg399x_als_cct_show, NULL);



static struct attribute *als_attrs [] =
{
    &als_power_state.attr,
    &wait_time_en.attr,
    &als_Itime.attr,
    &wait_time.attr,
    &als_persist.attr,
    &als_gain.attr,      
    &lux_table.attr,
    &als_auto_gain.attr,
    &als_lux.attr,
    &als_red.attr,
    &als_green.attr,
    &als_blue.attr,        
    &als_clear.attr,
    &als_cct.attr,         
    NULL
};

static struct attribute_group tmg399x_ps_attribute_group = {
	.attrs = prox_attrs,
};

static struct attribute_group tmg399x_als_attribute_group = {
	.attrs = als_attrs,
};

void tmg399x_report_prox(struct tmg399x_chip *chip, u8 detected)
{
	if (chip->p_idev) {
		chip->prx_inf.detected = detected;
		printk(KERN_INFO "proximity detected = %d\n", chip->prx_inf.detected);
		printk(KERN_INFO "proximity rawdata =  %d\n", chip->prx_inf.raw);
		
		input_report_abs(chip->p_idev, ABS_DISTANCE,chip->prx_inf.detected ? 0 : 1);
             
		//input_report_abs(chip->p_idev, ABS_Y,chip->prx_inf.raw);
		
		input_sync(chip->p_idev);
	}
}

void tmg399x_report_ges(struct tmg399x_chip *chip, u8 ges_data)
{
	if (chip->g_idev) {
		printk(KERN_INFO "gesture detected =  %d\n", ges_data);
		//input_report_abs(chip->g_idev, ABS_X,ges_data);
		//input_sync(chip->g_idev);
	}
}

EXPORT_SYMBOL_GPL(tmg399x_report_prox);


void tmg399x_report_als(struct tmg399x_chip *chip)
{
	int lux;
	int cct;
	
	tmg399x_get_lux(chip);
	
	printk(KERN_INFO "%s lux=%d  cct=%d\n",__func__,  chip->als_inf.lux, chip->als_inf.cct);
	lux = chip->als_inf.lux;
	input_report_abs(chip->a_idev, ABS_MISC, lux);    //modify by lauson  ABS_MISC   ABS_X

	cct = chip->als_inf.cct;
	//input_report_abs(chip->a_idev, ABS_Y, cct);

	input_sync(chip->a_idev);
}
EXPORT_SYMBOL_GPL(tmg399x_report_als);

static u8 tmg399x_ges_nswe_min(struct tmg399x_ges_nswe nswe)
{
        u8 min = nswe.north;
        if (nswe.south < min) min = nswe.south;
        if (nswe.west < min) min = nswe.west;
        if (nswe.east < min) min = nswe.east;
        return min;
}

static u8 tmg399x_ges_nswe_max(struct tmg399x_ges_nswe nswe)
{
        u8 max = nswe.north;
        if (nswe.south > max) max = nswe.south;
        if (nswe.west > max) max = nswe.west;
        if (nswe.east > max) max = nswe.east;
        return max;
}

static bool tmg399x_change_prox_offset(struct tmg399x_chip *chip, u8 state, u8 direction)
{
	u8 offset;

	switch (state) {
	case CHECK_PROX_NE:
		if (direction == DIR_UP) {
			/* negtive offset will increase the results */
			if (chip->params.prox_offset_ne == -127)
				return false;
			chip->params.prox_offset_ne--;
		} else if (direction == DIR_DOWN) {
			/* positive offset will decrease the results */
			if (chip->params.prox_offset_ne == 127)
				return false;
			chip->params.prox_offset_ne++;
		}
		/* convert int value to offset */
		INT2OFFSET(offset, chip->params.prox_offset_ne);
		chip->shadow[TMG399X_PRX_OFFSET_NE] = offset;
		tmg399x_i2c_write(chip, TMG399X_PRX_OFFSET_NE, offset);
		break;
	case CHECK_PROX_SW:
		if (direction == DIR_UP) {
			if (chip->params.prox_offset_sw == -127)
				return false;
			chip->params.prox_offset_sw--;
		} else if (direction == DIR_DOWN) {
			if (chip->params.prox_offset_sw == 127)
				return false;
			chip->params.prox_offset_sw++;
		}
		INT2OFFSET(offset, chip->params.prox_offset_sw);
		chip->shadow[TMG399X_PRX_OFFSET_SW] = offset;
		tmg399x_i2c_write(chip, TMG399X_PRX_OFFSET_SW, offset);
		break;
	default:
		break;
	}

	return true;
}

static void tmg399x_cal_prox_offset(struct tmg399x_chip *chip, u8 prox)
{
	/* start to calibrate the offset of prox */
	if (caloffsetstate == START_CALOFF) {
		/* mask south and west diode */
		chip->shadow[TMG399X_CONFIG_3] = 0x06;
		tmg399x_i2c_write(chip, TMG399X_CONFIG_3,
			chip->shadow[TMG399X_CONFIG_3]);
		pstatechanged = true;
		caloffsetstate = CHECK_PROX_NE;
		caloffsetdir = DIR_NONE;
		return;
	}
	
	/* calibrate north and east diode of prox */
	if (caloffsetstate == CHECK_PROX_NE) {
		/* only one direction one time, up or down */
		if ((caloffsetdir != DIR_DOWN) && (prox < callowtarget/2)) {
			caloffsetdir = DIR_UP;
			if (tmg399x_change_prox_offset(chip, CHECK_PROX_NE, DIR_UP))
				return;
		} else if ((caloffsetdir != DIR_UP) && (prox > calhightarget/2)) {
			caloffsetdir = DIR_DOWN;
			if (tmg399x_change_prox_offset(chip, CHECK_PROX_NE, DIR_DOWN))
				return;
		}

		/* north and east diode offset calibration complete, mask
		north and east diode and start to calibrate south and west diode */
		chip->shadow[TMG399X_CONFIG_3] = 0x09;
		tmg399x_i2c_write(chip, TMG399X_CONFIG_3,
			chip->shadow[TMG399X_CONFIG_3]);
		pstatechanged = true;
		caloffsetstate = CHECK_PROX_SW;
		caloffsetdir = DIR_NONE;
		printk(KERN_INFO "cal_prox_offset is ok  prox_offset_ne=%d,prox_offset_sw=%d\n",chip->params.prox_offset_ne,chip->params.prox_offset_sw);
		return;
	}

	/* calibrate south and west diode of prox */
	if (caloffsetstate == CHECK_PROX_SW) {
		if ((caloffsetdir != DIR_DOWN) && (prox < callowtarget/2)) {
			caloffsetdir = DIR_UP;
			if (tmg399x_change_prox_offset(chip, CHECK_PROX_SW, DIR_UP))
				return;
		} else if ((caloffsetdir != DIR_UP) && (prox > calhightarget/2)) {
			caloffsetdir = DIR_DOWN;
			if (tmg399x_change_prox_offset(chip, CHECK_PROX_SW, DIR_DOWN))
				return;
		}

		/* prox offset calibration complete, mask none diode,
		start to calibrate gesture offset */
		chip->shadow[TMG399X_CONFIG_3] = 0x00;
		tmg399x_i2c_write(chip, TMG399X_CONFIG_3,chip->shadow[TMG399X_CONFIG_3]);
		pstatechanged = true;
		caloffsetstate = CHECK_NSWE_ZERO;
		caloffsetdir = DIR_NONE;
		
		return;
	}
}

static int tmg399x_change_ges_offset(struct tmg399x_chip *chip, struct tmg399x_ges_nswe nswe_data, u8 direction)
{
	u8 offset;
	int ret = false;

	/* calibrate north diode */
	if (direction == DIR_UP) {
		if (nswe_data.north < callowtarget) {
			/* negtive offset will increase the results */
			if (chip->params.ges_offset_n == -127)
				goto cal_ges_offset_south;
			chip->params.ges_offset_n--;
		}	
	} else if (direction == DIR_DOWN) {
		if (nswe_data.north > calhightarget) {
			/* positive offset will decrease the results */
			if (chip->params.ges_offset_n == 127)
				goto cal_ges_offset_south;
			chip->params.ges_offset_n++;
		}	
	}
	/* convert int value to offset */
	INT2OFFSET(offset, chip->params.ges_offset_n);
	chip->shadow[TMG399X_GES_OFFSET_N] = offset;
	tmg399x_i2c_write(chip, TMG399X_GES_OFFSET_N, offset);
	ret = true;

cal_ges_offset_south:
	/* calibrate south diode */
	if (direction == DIR_UP) {
		if (nswe_data.south < callowtarget) {
			if (chip->params.ges_offset_s == -127)
				goto cal_ges_offset_west;
			chip->params.ges_offset_s--;
		}	
	} else if (direction == DIR_DOWN) {
		if (nswe_data.south > calhightarget) {
			if (chip->params.ges_offset_s == 127)
				goto cal_ges_offset_west;
			chip->params.ges_offset_s++;
		}	
	}
	INT2OFFSET(offset, chip->params.ges_offset_s);
	chip->shadow[TMG399X_GES_OFFSET_S] = offset;
	tmg399x_i2c_write(chip, TMG399X_GES_OFFSET_S, offset);
	ret = true;

cal_ges_offset_west:
	/* calibrate west diode */
	if (direction == DIR_UP) {
		if (nswe_data.west < callowtarget) {
			if (chip->params.ges_offset_w == -127)
				goto cal_ges_offset_east;
			chip->params.ges_offset_w--;
		}	
	} else if (direction == DIR_DOWN) {
		if (nswe_data.west > calhightarget) {
			if (chip->params.ges_offset_w == 127)
				goto cal_ges_offset_east;
			chip->params.ges_offset_w++;
		}	
	}
	INT2OFFSET(offset, chip->params.ges_offset_w);
	chip->shadow[TMG399X_GES_OFFSET_W] = offset;
	tmg399x_i2c_write(chip, TMG399X_GES_OFFSET_W, offset);
	ret = true;

cal_ges_offset_east:
	/* calibrate east diode */
	if (direction == DIR_UP) {
		if (nswe_data.east < callowtarget) {
			if (chip->params.ges_offset_e == -127)
				goto cal_ges_offset_exit;
			chip->params.ges_offset_e--;
		}	
	} else if (direction == DIR_DOWN) {
		if (nswe_data.east > calhightarget) {
			if (chip->params.ges_offset_e == 127)
				goto cal_ges_offset_exit;
			chip->params.ges_offset_e++;
		}	
	}
	INT2OFFSET(offset, chip->params.ges_offset_e);
	chip->shadow[TMG399X_GES_OFFSET_E] = offset;
	tmg399x_i2c_write(chip, TMG399X_GES_OFFSET_E, offset);
	ret = true;

cal_ges_offset_exit:
	return ret;
}

static void tmg399x_cal_ges_offset(struct tmg399x_chip *chip, struct tmg399x_ges_nswe* nswe_data, u8 len)
{
	u8 i;
	u8 min;
	u8 max;
	for (i = 0; i < len; i++) {
		if (caloffsetstate == CHECK_NSWE_ZERO) {
			min = tmg399x_ges_nswe_min(nswe_data[i]);
			max = tmg399x_ges_nswe_max(nswe_data[i]);

			/* only one direction one time, up or down */
			if ((caloffsetdir != DIR_DOWN) && (min <= callowtarget)) {
				caloffsetdir = DIR_UP;
				if (tmg399x_change_ges_offset(chip, nswe_data[i], DIR_UP))
					return;
			} else if ((caloffsetdir != DIR_UP) && (max > calhightarget)) {
				caloffsetdir = DIR_DOWN;
				if (tmg399x_change_ges_offset(chip, nswe_data[i], DIR_DOWN))
					return;
			}

			/* calibration is ok */
			caloffsetstate = CALOFF_OK;
			caloffsetdir = DIR_NONE;
			docalibration = false;
			printk(KERN_INFO "cal_ges_offset is ok\n n_offset=%d,s_offset=%d,w_offset=%d,e_offset=%d\n",
							chip->params.ges_offset_n,
							chip->params.ges_offset_s,
							chip->params.ges_offset_w,
							chip->params.ges_offset_e);
			break;
		}
	}
}

void tmg399x_start_calibration(struct tmg399x_chip *chip)
{
	docalibration = true;
	caloffsetstate = START_CALOFF;
	/* entry threshold is set min 0, exit threshold is set max 255,
	and NSWE are all masked for exit, gesture state will be force
	enter and exit every cycle */
	chip->params.ges_entry_th = 0;
	chip->shadow[TMG399X_GES_ENTH] = 0;
	tmg399x_i2c_write(chip, TMG399X_GES_ENTH, 0);
	chip->params.ges_exit_th = 255;
	chip->shadow[TMG399X_GES_EXTH] = 255;
	tmg399x_i2c_write(chip, TMG399X_GES_EXTH, 255);
	chip->params.ges_cfg1 |= GEXMSK_ALL;
	chip->shadow[TMG399X_GES_CFG_1] = chip->params.ges_cfg1;
	tmg399x_i2c_write(chip, TMG399X_GES_CFG_1, chip->params.ges_cfg1);
}
EXPORT_SYMBOL_GPL(tmg399x_start_calibration);

void tmg399x_set_ges_thresh(struct tmg399x_chip *chip, u8 entry, u8 exit)
{
	/* set entry and exit threshold for gesture state enter and exit */
	chip->params.ges_entry_th = entry;
	chip->shadow[TMG399X_GES_ENTH] = entry;
	tmg399x_i2c_write(chip, TMG399X_GES_ENTH, entry);
	chip->params.ges_exit_th = exit;
	chip->shadow[TMG399X_GES_EXTH] = exit;
	tmg399x_i2c_write(chip, TMG399X_GES_EXTH, exit);
	chip->params.ges_cfg1 &= ~GEXMSK_ALL;
	chip->shadow[TMG399X_GES_CFG_1] = chip->params.ges_cfg1;
	tmg399x_i2c_write(chip, TMG399X_GES_CFG_1, chip->params.ges_cfg1);
}
EXPORT_SYMBOL_GPL(tmg399x_set_ges_thresh);

void tmg399x_rgbc_poll_handle(unsigned long data)
{
	struct tmg399x_chip *chip = (struct tmg399x_chip *)data;
	chip->rgbc_poll_flag = true;
	
}

void tmg399x_update_rgbc(struct tmg399x_chip *chip)
{
}
EXPORT_SYMBOL_GPL(tmg399x_update_rgbc);

static int tmg399x_check_and_report(struct tmg399x_chip *chip)
{
	int ret;
	u8 status;
	u8 numofdset;
	u8 len;

	mutex_lock(&chip->lock);
	ret = tmg399x_i2c_read(chip, TMG399X_STATUS,&chip->shadow[TMG399X_STATUS]);
	if (ret < 0) {
		dev_err(&chip->client->dev,
			"%s: failed to read tmg399x status\n",
			__func__);
		goto exit_clr;
	}

	status = chip->shadow[TMG399X_STATUS];

	if ((status & TMG399X_ST_PRX_IRQ) == TMG399X_ST_PRX_IRQ) {
//		printk(KERN_INFO "proximity interrupt generated\n");
		/* read prox raw data */
		tmg399x_i2c_read(chip, TMG399X_PRX_CHAN,&chip->shadow[TMG399X_PRX_CHAN]);
		if (chip->prx_enabled)
			chip->prx_inf.raw = chip->shadow[TMG399X_PRX_CHAN];
			/* ignore the first prox data when proximity state changed */
			if (pstatechanged) {
				pstatechanged = false;
			} else {
				if (docalibration && caloffsetstate != CHECK_NSWE_ZERO && (chip->shadow[TMG399X_STATUS] <=250)) {
					/* do prox offset calibration */
					tmg399x_cal_prox_offset(chip, chip->shadow[TMG399X_PRX_CHAN]);
				} else {
					/* process prox data */
					process_rgbc_prox_ges_raw_data(chip, PROX_DATA, (u8*)&chip->shadow[TMG399X_PRX_CHAN], 1);
				}
			}
              tmg399x_update_prx_thd(chip);
		/* clear the irq of prox */
		tmg399x_irq_clr(chip, TMG399X_CMD_PROX_INT_CLR);
	}

	if ((status & TMG399X_ST_GES_IRQ) == TMG399X_ST_GES_IRQ) {
//		printk(KERN_INFO "gesture interrupt generated\n");
		len = 0;
	        while(1) 
		{
			/* get how many data sets in fifo */
	                tmg399x_i2c_read(chip, TMG399X_GES_FLVL, &numofdset);
			if (numofdset == 0) {
				/* fifo empty, skip fifo reading */
				break;
                	}
			/* read gesture data from fifo to SW buffer */
                	tmg399x_i2c_ram_blk_read(chip, TMG399X_GES_NFIFO,
                        	(u8 *)&chip->ges_raw_data[len], numofdset*4);
			/* calculate number of gesture data sets */
			len += numofdset;
//			printk (KERN_INFO "gesture buffer len=%d\n",len);
			
			if (len > 32) {
				printk (KERN_INFO "gesture buffer overflow!\n");
				len = 0;
				goto als_function;
			}
        	}

		if (docalibration && caloffsetstate != CALOFF_OK) {
			/* do gesture offset calibration */
			tmg399x_cal_ges_offset(chip, (struct tmg399x_ges_nswe*)chip->ges_raw_data, len);
		} else {
			process_rgbc_prox_ges_raw_data(chip, GES_DATA, (u8*)chip->ges_raw_data, (len*4));
		}
	}

als_function:
	if (chip->als_enabled && (status & TMG399X_ST_ALS_IRQ) == TMG399X_ST_ALS_IRQ) {
//		printk(KERN_INFO "als enable interrupt generated\n");
		tmg399x_i2c_ram_blk_read(chip, TMG399X_CLR_CHANLO, (u8 *)&chip->shadow[TMG399X_CLR_CHANLO], 8);
		process_rgbc_prox_ges_raw_data(chip, RGBC_DATA, (u8*)&chip->shadow[TMG399X_CLR_CHANLO], 8);
		tmg399x_irq_clr(chip, TMG399X_CMD_ALS_INT_CLR);
	}
	else if((status & TMG399X_ST_ALS_IRQ) == TMG399X_ST_ALS_IRQ)
	{
//		printk(KERN_INFO "als interrupt generated\n");
		tmg399x_irq_clr(chip, TMG399X_CMD_ALS_INT_CLR);
	}


	if ((status & (TMG399X_ST_BEAM_IRQ)) == (TMG399X_ST_BEAM_IRQ)) 
	{
		printk(KERN_INFO "IRbeam interrupt generated\n");
		if (chip->hop_count == 255) {
			/* don't decrement count - it's forever until..*/
			schedule_work(&chip->work_thresh);
		}
		else if (chip->hop_count > 0) 
		{
			chip->hop_index++;
			chip->hop_count--;
			if (chip->hop_index > chip->hop_next_slot)
				chip->hop_index = 0;
			schedule_work(&chip->work_thresh);
		} 
		else 
		{
			printk(KERN_INFO "IRbeam interrupt =====>end\n");
			chip->shadow[TMG399X_CONTROL] = chip->pre_beam_xmit_state;
			chip->hop_index = 0;
//			tmg399x_update_enable_reg(chip);
		}
		tmg399x_irq_clr(chip, TMG399X_CMD_IRBEAM_INT_CLR);
	}

exit_clr:
	mutex_unlock(&chip->lock);

	return ret;
}

#if 0 /* delete it by lauson.   for debug */
static void tmg399x_get_irq_status(struct tmg399x_chip *chip)
{
    int int_pin = 0;
    int gpio_status;
    struct device_node *np = chip->client->dev.of_node;
    int_pin = of_get_named_gpio(np, "tmg,irq-gpio", 0);
    gpio_status = __gpio_get_value(int_pin);
    printk("%s int_pin status:%d \n", __func__,gpio_status);
}
#endif 

static void tmg399x_irq_work(struct work_struct *work)
{
	struct tmg399x_chip *chip = container_of(work, struct tmg399x_chip, irq_work);
       msleep(100);// add by lauson 

	tmg399x_check_and_report(chip);
       //tmg399x_update_prx_thd(chip);
	enable_irq(chip->client->irq);

}

static irqreturn_t tmg399x_irq(int irq, void *handle)
{
	struct tmg399x_chip *chip = handle;
	struct device *dev = &chip->client->dev;
       //printk("%s 11111 \n", __func__);

	disable_irq_nosync(chip->client->irq);

	if (chip->in_suspend) {
		dev_info(dev, "%s: in suspend\n", __func__);
		chip->irq_pending = 1;
		goto bypass;
	}
	schedule_work(&chip->irq_work);
bypass:
	return IRQ_HANDLED;
}

static int tmg399x_set_segment_table(struct tmg399x_chip *chip,
		struct tmg_lux_segment *segment, int seg_num)
{
	int i;
	struct device *dev = &chip->client->dev;

	chip->seg_num_max = chip->pdata->segment_num ?
			chip->pdata->segment_num : ARRAY_SIZE(segment_default);

	if (!chip->segment) {
		dev_info(dev, "%s: allocating segment table\n", __func__);
		chip->segment = kzalloc(sizeof(*chip->segment) *
				chip->seg_num_max, GFP_KERNEL);
		if (!chip->segment) {
			dev_err(dev, "%s: no memory!\n", __func__);
			return -ENOMEM;
		}
	}
	if (seg_num > chip->seg_num_max) {
		dev_warn(dev, "%s: %d segment requested, %d applied\n",
				__func__, seg_num, chip->seg_num_max);
		chip->segment_num = chip->seg_num_max;
	} else {
		chip->segment_num = seg_num;
	}
	memcpy(chip->segment, segment,
			chip->segment_num * sizeof(*chip->segment));
	dev_info(dev, "%s: %d segment requested, %d applied\n", __func__,
			seg_num, chip->seg_num_max);
	for (i = 0; i < chip->segment_num; i++)
		dev_info(dev,
		"seg %d: d_factor %d, r_coef %d, g_coef %d, b_coef %d, ct_coef %d ct_offset %d\n",
		i, chip->segment[i].d_factor, chip->segment[i].r_coef,
		chip->segment[i].g_coef, chip->segment[i].b_coef,
		chip->segment[i].ct_coef, chip->segment[i].ct_offset);
	return 0;
}

static void tmg399x_set_defaults(struct tmg399x_chip *chip)
{
	struct device *dev = &chip->client->dev;

	if (chip->pdata) {
		dev_info(dev, "%s: Loading pltform data\n", __func__);
		chip->params.als_time = chip->pdata->parameters.als_time;
		chip->params.als_gain = chip->pdata->parameters.als_gain;
		chip->params.wait_time = chip->pdata->parameters.wait_time;
		chip->params.prox_th_min = chip->pdata->parameters.prox_th_min;
		chip->params.prox_th_max = chip->pdata->parameters.prox_th_max;
		chip->params.persist = chip->pdata->parameters.persist;
		chip->params.als_prox_cfg1 = chip->pdata->parameters.als_prox_cfg1;
		chip->params.prox_pulse = chip->pdata->parameters.prox_pulse;
		chip->params.prox_gain = chip->pdata->parameters.prox_gain;
		chip->params.ldrive = chip->pdata->parameters.ldrive;
		chip->params.als_prox_cfg2 = chip->pdata->parameters.als_prox_cfg2;
		chip->params.prox_offset_ne = chip->pdata->parameters.prox_offset_ne;
		chip->params.prox_offset_sw = chip->pdata->parameters.prox_offset_sw;
		chip->params.als_prox_cfg3 = chip->pdata->parameters.als_prox_cfg3;
	} else {
		dev_info(dev, "%s: use defaults\n", __func__);
		chip->params.als_time = param_default.als_time;
		chip->params.als_gain = param_default.als_gain;
		chip->params.wait_time = param_default.wait_time;
		chip->params.prox_th_min = param_default.prox_th_min;
		chip->params.prox_th_max = param_default.prox_th_max;
		chip->params.persist = param_default.persist;
		chip->params.als_prox_cfg1 = param_default.als_prox_cfg1;
		chip->params.prox_pulse = param_default.prox_pulse;
		chip->params.prox_gain = param_default.prox_gain;
		chip->params.ldrive = param_default.ldrive;
		chip->params.als_prox_cfg2 = param_default.als_prox_cfg2;
		chip->params.prox_offset_ne = param_default.prox_offset_ne;
		chip->params.prox_offset_sw = param_default.prox_offset_sw;
		chip->params.als_prox_cfg3 = param_default.als_prox_cfg3;
	}

	chip->als_gain_auto = false;

	if (chip->pdata->beam_settings.beam_cfg) 
	{
		dev_info(dev, "%s: Loading IRBeam data from pltform.\n",__func__);

		chip->beam_settings.beam_cfg  = chip->pdata->beam_settings.beam_cfg;
		chip->beam_settings.beam_carr = chip->pdata->beam_settings.beam_carr;
		chip->beam_settings.beam_ns   = chip->pdata->beam_settings.beam_ns;
		chip->beam_settings.beam_isd  = chip->pdata->beam_settings.beam_isd;
		chip->beam_settings.beam_np   = chip->pdata->beam_settings.beam_np;
		chip->beam_settings.beam_ipd  = chip->pdata->beam_settings.beam_ipd;
		chip->beam_settings.beam_div  = chip->pdata->beam_settings.beam_div;
	} 
	else
	{
		dev_info(dev, "%s: Loading IRBeam data from DEFAULT.\n",__func__);
		chip->beam_settings.beam_cfg  = IRBEAM_CFG;
		chip->beam_settings.beam_carr = IRBEAM_CARR;
		chip->beam_settings.beam_ns   = IRBEAM_NS;
		chip->beam_settings.beam_isd  = IRBEAM_ISD;
		chip->beam_settings.beam_np   = IRBEAM_NP;
		chip->beam_settings.beam_ipd  = IRBEAM_IPD;
		chip->beam_settings.beam_div  = IRBEAM_DIV;
	}

	/* Load the initial/default HOP parameters into the table. */
	/* Can be changed via application                          */
	memcpy(&chip->beam_hop[0], &chip->beam_settings,sizeof(chip->beam_settings));
	chip->hop_next_slot = 0;
	
	/* Initial proximity threshold */
	chip->shadow[TMG399X_PRX_MINTHRESHLO] = 0;//chip->params.prox_th_min;
	chip->shadow[TMG399X_PRX_MAXTHRESHHI] = chip->params.prox_th_max;
	tmg399x_i2c_write(chip, TMG399X_PRX_MINTHRESHLO,chip->shadow[TMG399X_PRX_MINTHRESHLO]);
	tmg399x_i2c_write(chip, TMG399X_PRX_MAXTHRESHHI,chip->shadow[TMG399X_PRX_MAXTHRESHHI]);

	tmg399x_i2c_write(chip, TMG399X_ALS_MINTHRESHLO, 0xff);
	tmg399x_i2c_write(chip, TMG399X_ALS_MINTHRESHHI, 0xff);
	tmg399x_i2c_write(chip, TMG399X_ALS_MAXTHRESHLO, 0);
	tmg399x_i2c_write(chip, TMG399X_ALS_MAXTHRESHHI, 0);
			
	chip->shadow[TMG399X_ALS_TIME]      = chip->params.als_time;
	chip->shadow[TMG399X_WAIT_TIME]     = chip->params.wait_time;
	chip->shadow[TMG399X_PERSISTENCE]   = chip->params.persist;
	chip->shadow[TMG399X_CONFIG_1]      = chip->params.als_prox_cfg1;	
	chip->shadow[TMG399X_PRX_PULSE]     = chip->params.prox_pulse;
	chip->shadow[TMG399X_GAIN]          = chip->params.als_gain |chip->params.prox_gain | chip->params.ldrive;
	chip->shadow[TMG399X_CONFIG_2]      = chip->params.als_prox_cfg2;
	chip->shadow[TMG399X_PRX_OFFSET_NE] = chip->params.prox_offset_ne;
	chip->shadow[TMG399X_PRX_OFFSET_SW] = chip->params.prox_offset_sw;
	chip->shadow[TMG399X_CONFIG_3]      = chip->params.als_prox_cfg3;
}

static int tmg399x_get_id(struct tmg399x_chip *chip, u8 *id, u8 *rev)
{
	int ret;
	ret = tmg399x_i2c_read(chip, TMG399X_REVID, rev);
	ret |= tmg399x_i2c_read(chip, TMG399X_CHIPID, id);
	return ret;
}

static int tmg399x_pltf_power_on(struct tmg399x_chip *chip)
{
	int ret = 0;
	if (chip->pdata->platform_power) {
		ret = chip->pdata->platform_power(&chip->client->dev,
			GPOWER_ON);
		mdelay(10);
	}
	chip->unpowered = ret != 0;
	return ret;
}

static int tmg399x_pltf_power_off(struct tmg399x_chip *chip)
{
	int ret = 0;
	if (chip->pdata->platform_power) {
		ret = chip->pdata->platform_power(&chip->client->dev,
			GPOWER_OFF);
		chip->unpowered = ret == 0;
	} else {
		chip->unpowered = false;
	}
	return ret;
}

static int tmg399x_power_on(struct tmg399x_chip *chip)
{
	int ret;
	ret = tmg399x_pltf_power_on(chip);
	if (ret)
		return ret;
	dev_info(&chip->client->dev, "%s: chip was off, restoring regs\n",
			__func__);
	return tmg399x_flush_regs(chip);
}

#if 0
static int tmg399x_prox_idev_open(struct input_dev *idev)
{
	struct tmg399x_chip *chip = dev_get_drvdata(&idev->dev);
	int ret;
	bool als = chip->a_idev && chip->a_idev->users;

	dev_info(&idev->dev, "%s\n", __func__);
	mutex_lock(&chip->lock);
	if (chip->unpowered) {
		ret = tmg399x_power_on(chip);
		if (ret)
			goto chip_on_err;
	}
	ret = tmg399x_prox_enable(chip, 1);
	if (ret && !als)
		tmg399x_pltf_power_off(chip);
chip_on_err:
	mutex_unlock(&chip->lock);
	return ret;
}

static void tmg399x_prox_idev_close(struct input_dev *idev)
{
	struct tmg399x_chip *chip = dev_get_drvdata(&idev->dev);

	dev_info(&idev->dev, "%s\n", __func__);
	mutex_lock(&chip->lock);
	tmg399x_prox_enable(chip, 0);
	if (!chip->a_idev || !chip->a_idev->users)
		tmg399x_pltf_power_off(chip);
	mutex_unlock(&chip->lock);
}

static int tmg399x_als_idev_open(struct input_dev *idev)
{
	struct tmg399x_chip *chip = dev_get_drvdata(&idev->dev);
	int ret;
	bool prox = chip->p_idev && chip->p_idev->users;

	dev_info(&idev->dev, "%s\n", __func__);
	mutex_lock(&chip->lock);
	if (chip->unpowered) {
		ret = tmg399x_power_on(chip);
		if (ret)
			goto chip_on_err;
	}
	ret = tmg399x_als_enable(chip, 1);
	if (ret && !prox)
		tmg399x_pltf_power_off(chip);
chip_on_err:
	mutex_unlock(&chip->lock);
	return ret;
}

static void tmg399x_als_idev_close(struct input_dev *idev)
{
	struct tmg399x_chip *chip = dev_get_drvdata(&idev->dev);
	dev_info(&idev->dev, "%s\n", __func__);
	mutex_lock(&chip->lock);
	tmg399x_als_enable(chip, 0);
	if (!chip->p_idev || !chip->p_idev->users)
		tmg399x_pltf_power_off(chip);
	mutex_unlock(&chip->lock);
}
#endif

#if 0 /* delete it by lauson. */
static int tmg399x_add_sysfs_interfaces(struct device *dev,
	struct device_attribute *a, int size)
{
	int i;
	for (i = 0; i < size; i++)
		if (device_create_file(dev, a + i))
			goto undo;
	return 0;
undo:
	for (; i >= 0 ; i--)
		device_remove_file(dev, a + i);
	dev_err(dev, "%s: failed to create sysfs interface\n", __func__);
	return -ENODEV;
}
#endif 

#if 0 /* delete it by lauson. */
static void tmg399x_remove_sysfs_interfaces(struct device *dev,
	struct device_attribute *a, int size)
{
	int i;
	for (i = 0; i < size; i++)
		device_remove_file(dev, a + i);
}
#endif 

#if 1  // add by lauson , for dts and power.
#define TMG399X_VDD_MIN_UV	2750000
#define TMG399X_VDD_MAX_UV	2900000
#define TMG399X_VIO_MIN_UV	1750000
#define TMG399X_VIO_MAX_UV	1950000
static int tmg399x_power_init(struct tmg399x_chip *data)
{
	int rc;

	data->vdd = devm_regulator_get(&data->client->dev, "vdd");
	if (IS_ERR(data->vdd)) {
		rc = PTR_ERR(data->vdd);
		dev_err(&data->client->dev,
				"Regualtor get failed vdd rc=%d\n", rc);
		return rc;
	}
	if (regulator_count_voltages(data->vdd) > 0) {
		rc = regulator_set_voltage(data->vdd,
				TMG399X_VDD_MIN_UV, TMG399X_VDD_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
					"Regulator set failed vdd rc=%d\n",
					rc);
			goto exit;
		}
	}

	rc = regulator_enable(data->vdd);
	if (rc) {
		dev_err(&data->client->dev,
				"Regulator enable vdd failed rc=%d\n", rc);
		goto exit;
	}
	data->vio = devm_regulator_get(&data->client->dev, "vio");
	if (IS_ERR(data->vio)) {
		rc = PTR_ERR(data->vio);
		dev_err(&data->client->dev,
				"Regulator get failed vio rc=%d\n", rc);
		goto reg_vdd_set;
	}

	if (regulator_count_voltages(data->vio) > 0) {
		rc = regulator_set_voltage(data->vio,
				TMG399X_VIO_MIN_UV, TMG399X_VIO_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
					"Regulator set failed vio rc=%d\n", rc);
			goto reg_vdd_set;
		}
	}
	rc = regulator_enable(data->vio);
	if (rc) {
		dev_err(&data->client->dev,
				"Regulator enable vio failed rc=%d\n", rc);
		goto reg_vdd_set;
	}

	 /* The minimum time to operate device after VDD valid is 10 ms. */
	usleep_range(15000, 20000);

	data->unpowered = true;

	return 0;

reg_vdd_set:
	regulator_disable(data->vdd);
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, TMG399X_VDD_MAX_UV);
exit:
	return rc;

}

static int tmg399x_power_deinit(struct tmg399x_chip *data)
{
	if (!IS_ERR_OR_NULL(data->vio)) {
		if (regulator_count_voltages(data->vio) > 0)
			regulator_set_voltage(data->vio, 0,
					TMG399X_VIO_MAX_UV);

		regulator_disable(data->vio);
	}

	if (!IS_ERR_OR_NULL(data->vdd)) {
		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd, 0,
					TMG399X_VDD_MAX_UV);

		regulator_disable(data->vdd);
	}

	data->unpowered = false;

	return 0;
}

static int tmg399x_power_set(struct tmg399x_chip *data, bool on)
{
	int rc = 0;

	if (!on && data->unpowered) {
		mutex_lock(&data->lock);

		rc = regulator_disable(data->vdd);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vdd disable failed rc=%d\n", rc);
			goto err_vdd_disable;
		}

		rc = regulator_disable(data->vio);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vio disable failed rc=%d\n", rc);
			goto err_vio_disable;
		}
		data->unpowered = false;

		mutex_unlock(&data->lock);
		return rc;
	} 
       else if (on && !data->unpowered)
	{
		mutex_lock(&data->lock);

		rc = regulator_enable(data->vdd);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
			goto err_vdd_enable;
		}

		rc = regulator_enable(data->vio);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vio enable failed rc=%d\n", rc);
			goto err_vio_enable;
		}
		data->unpowered = true;

		mutex_unlock(&data->lock);

		/* The minimum time to operate after VDD valid is 10 ms */
		usleep_range(15000, 20000);

		return rc;
	} else {
		dev_warn(&data->client->dev,
				"Power on=%d. enabled=%d\n",
				on, data->unpowered);
		return rc;
	}

err_vio_enable:
	regulator_disable(data->vio);
err_vdd_enable:
	mutex_unlock(&data->lock);
	return rc;

err_vio_disable:
	if (regulator_enable(data->vdd))
		dev_warn(&data->client->dev, "Regulator vdd enable failed\n");
err_vdd_disable:
	mutex_unlock(&data->lock);
	return rc;
}

#if 0 /* delete it by lauson. */
static int tmg399x_parse_dt(struct i2c_client *client,
		struct tmg399x_data *memsic)
{
	struct device_node *np = client->dev.of_node;
	const char *tmp;
	int rc;
	int i;
    
	return 0;
}
#endif 
#endif

#if 1  //move and modify by lauson 

static int board_tmg399x_init(void)
{
	return 0;
}

static int tmg399x_power(struct device *dev, enum tmg399x_pwr_state state)
{

	printk(KERN_INFO "tmg399x_power CALLED\n");
	if (state == GPOWER_ON )
       {
	}
       else if (state == GPOWER_OFF)
       {
       
       }
	return 0;
}

static void board_tmg399x_teardown(struct device *dev)
{

}

static const struct tmg_lux_segment tmg399x_segment[] = {
        {
                .d_factor = D_Factor1,
                .r_coef = R_Coef1,
                .g_coef = G_Coef1,
                .b_coef = B_Coef1,
                .ct_coef = CT_Coef1,
                .ct_offset = CT_Offset1,
        },
        {
                .d_factor = D_Factor1,
                .r_coef = R_Coef1,
                .g_coef = G_Coef1,
                .b_coef = B_Coef1,
                .ct_coef = CT_Coef1,
                .ct_offset = CT_Offset1,
        },
        {
                .d_factor = D_Factor1,
                .r_coef = R_Coef1,
                .g_coef = G_Coef1,
                .b_coef = B_Coef1,
                .ct_coef = CT_Coef1,
                .ct_offset = CT_Offset1,
        },
        {
                .d_factor = D_Factor1,
                .r_coef = R_Coef1,
                .g_coef = G_Coef1,
                .b_coef = B_Coef1,
                .ct_coef = CT_Coef1,
                .ct_offset = CT_Offset1,
        },
        {
                .d_factor = D_Factor1,
                .r_coef = R_Coef1,
                .g_coef = G_Coef1,
                .b_coef = B_Coef1,
                .ct_coef = CT_Coef1,
                .ct_offset = CT_Offset1,
        },
        {
                .d_factor = D_Factor1,
                .r_coef = R_Coef1,
                .g_coef = G_Coef1,
                .b_coef = B_Coef1,
                .ct_coef = CT_Coef1,
                .ct_offset = CT_Offset1,
        },
        {
                .d_factor = D_Factor1,
                .r_coef = R_Coef1,
                .g_coef = G_Coef1,
                .b_coef = B_Coef1,
                .ct_coef = CT_Coef1,
                .ct_offset = CT_Offset1,
        },
};

struct tmg399x_i2c_platform_data tmg399x_data = {
	.platform_power = tmg399x_power,
	.platform_init = board_tmg399x_init,
	.platform_teardown = board_tmg399x_teardown,
	.prox_name = "proximity",
	.als_name = "light",
	.ges_name = "tmg399x_gesture",
	.parameters = {
		.als_time = 0xFE, /* 5.6ms */
		.als_gain = GAGAIN_16,
		.wait_time = 0xFF, /* 2.78ms */
		.prox_th_min = 80,
		.prox_th_max = 100,
		.persist = PRX_PERSIST(0) | ALS_PERSIST(8),
		.als_prox_cfg1 = 0x60,
		.prox_pulse = PPLEN_16US | PRX_PULSE_CNT(7),
		.prox_gain = GPGAIN_4,
		.ldrive = GPDRIVE_100MA,
		.als_prox_cfg2 = LEDBOOST_150 | 0x01,
		.prox_offset_ne = 0,
		.prox_offset_sw = 0,
		.als_prox_cfg3 = 0,
		
            .ges_entry_th = 0,
            .ges_exit_th = 255,
            .ges_cfg1 = FIFOTH_1 | GEXMSK_ALL | GEXPERS_1,
            .ges_cfg2 = GGAIN_4 | GLDRIVE_100 | GWTIME_0,
            .ges_offset_n = 0,
            .ges_offset_s = 0,
            .ges_pulse = GPLEN_16US | GES_PULSE_CNT(11),
            .ges_offset_w = 0,
            .ges_offset_e = 0,
            .ges_dimension = GBOTH_PAIR,
	},
	.als_can_wake = false,
	.proximity_can_wake = true,
	.segment = (struct tmg_lux_segment *) tmg399x_segment,
	.segment_num = ARRAY_SIZE(tmg399x_segment),

};
#endif


static int  tmg399x_probe(struct i2c_client *client,
	const struct i2c_device_id *idp)
{
	int i, ret;
	u8 id, rev;
	struct device *dev = &client->dev;
	static struct tmg399x_chip *chip;
	struct tmg399x_i2c_platform_data *pdata = &tmg399x_data;

	bool powered = 0;

	dev_info(dev, "%s: start .... \n", __func__);

       // get irq info and set the gpio.
       {
            int int_pin = 0;
            int gpio_status;
            struct device_node *np = dev->of_node;
            int_pin = of_get_named_gpio(np, "tmg,irq-gpio", 0);
            client->irq = gpio_to_irq(int_pin);
            printk("%s int_pin:%d  irq:%d \n", __func__,int_pin, client->irq);
            ret = gpio_request(int_pin,"tmg399x-int");
            if(ret < 0)
            {
                goto init_failed;
            }
            ret = gpio_direction_input(int_pin);
            if(ret < 0)
            {
        	   printk(KERN_ERR "%s: gpio_direction_input, err=%d", __func__, ret);
                 goto init_failed;
             }
            gpio_status = __gpio_get_value(int_pin);
            printk("%s int_pin status:%d \n", __func__,gpio_status);
       }
    
	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(dev, "%s: i2c smbus byte data unsupported\n", __func__);
		ret = -EOPNOTSUPP;
		goto init_failed;
	}
	if (!pdata) {
		dev_err(dev, "%s: platform data required\n", __func__);
		ret = -EINVAL;
		goto init_failed;
	}
	if (!(pdata->prox_name || pdata->als_name|| pdata->ges_name) || client->irq < 0) {
		dev_err(dev, "%s: no reason to run.\n", __func__);
		ret = -EINVAL;
		goto init_failed;
	}
	if (pdata->platform_init) {
		ret = pdata->platform_init();
		if (ret)
			goto init_failed;
	}

	if (pdata->platform_power) {
		ret = pdata->platform_power(dev,GPOWER_ON);
		if (ret) {
			dev_err(dev, "%s: pltf power on failed\n", __func__);
			goto pon_failed;
		}
		powered = true;
		mdelay(10);
	}

	chip = kzalloc(sizeof(struct tmg399x_chip), GFP_KERNEL);
	if (!chip) {
		ret = -ENOMEM;
		goto malloc_failed;
	}
	chip->client = client;
	chip->pdata = pdata;
	i2c_set_clientdata(client, chip);
       dev_set_drvdata(&client->dev, chip);

	chip->seg_num_max = chip->pdata->segment_num ?
			chip->pdata->segment_num : ARRAY_SIZE(segment_default);
	if (chip->pdata->segment)
		ret = tmg399x_set_segment_table(chip, chip->pdata->segment,chip->pdata->segment_num);
	else
		ret =  tmg399x_set_segment_table(chip, segment_default,ARRAY_SIZE(segment_default));
	if (ret)
		goto set_segment_failed;

       ret = tmg399x_power_init(chip);
       if (ret)
       {
            dev_err(&chip->client->dev,"%s: failed to power init \n",__func__);
            goto set_segment_failed;
       }

	ret = tmg399x_get_id(chip, &id, &rev);
	if (ret < 0)
		dev_err(&chip->client->dev,"%s: failed to get tmg399x id\n",__func__);

	printk("%s: device id:%02x device rev:%02x\n", __func__,id, rev);

	for (i = 0; i < ARRAY_SIZE(tmg399x_ids); i++) {
		if (id == tmg399x_ids[i])
			break;
	}
	if (i < ARRAY_SIZE(tmg399x_names)) {
		dev_info(dev, "%s: '%s rev. %d' detected\n", __func__,tmg399x_names[i], rev);
		chip->device_index = i;
	} else {
		dev_err(dev, "%s: not supported chip id\n", __func__);
		ret = -EOPNOTSUPP;
		goto id_failed;
	}

	mutex_init(&chip->lock);

	/* disable all */
	tmg399x_prox_enable(chip, 0);
	tmg399x_ges_enable(chip, 0);
	tmg399x_als_enable(chip, 0);
	tmg399x_beam_enable(chip,0);
		
	tmg399x_set_defaults(chip);
	ret = tmg399x_flush_regs(chip);
	if (ret)
		goto flush_regs_failed;
    
#if 0 /* delete it by lauson. */
	if (pdata->platform_power) {
		pdata->platform_power(dev, GPOWER_OFF);
		powered = false;
		chip->unpowered = true;
	}
#endif 

	ret = tmg399x_power_set(chip, false);
	if (ret) {
		dev_err(&client->dev, "Power off failed\n");
		goto power_deinit_failed;
	}


	/* gesture processing initialize */
	init_params_rgbc_prox_ges();

	if (!pdata->prox_name)
		goto bypass_prox_idev;
	chip->p_idev = input_allocate_device();
	
	if (!chip->p_idev) {
		dev_err(dev, "%s: no memory for input_dev '%s'\n",__func__, pdata->prox_name);
		ret = -ENODEV;
		goto input_p_alloc_failed;
	}


	chip->p_idev->name = pdata->prox_name;
	chip->p_idev->id.bustype = BUS_I2C;
	set_bit(EV_ABS, chip->p_idev->evbit);
	set_bit(ABS_DISTANCE, chip->p_idev->absbit);
	//set_bit(ABS_Y, chip->p_idev->absbit);
	input_set_abs_params(chip->p_idev, ABS_DISTANCE, 0, 1, 0, 0);
	//input_set_abs_params(chip->p_idev, ABS_Y, 0, 65535, 0, 0);
	
	set_bit(EV_KEY,chip->p_idev->evbit);
	chip->p_idev->keycode = tmg3993_Keycode;
	for(i = 0; i < TMG_MAX_BUTTON_CNT; i++)	
		set_bit(tmg3993_Keycode[i], chip->p_idev->keybit);


//	chip->p_idev->open = tmg399x_prox_idev_open;
//	chip->p_idev->close = tmg399x_prox_idev_close;
	dev_set_drvdata(&chip->p_idev->dev, chip);
	ret = input_register_device(chip->p_idev);
	if (ret) {
		input_free_device(chip->p_idev);
		dev_err(dev, "%s: cant register input '%s'\n",
				__func__, pdata->prox_name);
		goto input_p_register_failed;
	}
#if 0 /* delete it by lauson. */
	ret = tmg399x_add_sysfs_interfaces(&chip->p_idev->dev,
			prox_attrs, ARRAY_SIZE(prox_attrs));
	if (ret)
		goto input_p_sysfs_failed;
#endif 
	ret = sysfs_create_group(&chip->p_idev->dev.kobj, &tmg399x_ps_attribute_group);
	if (ret < 0)
	{
		printk(KERN_ERR "%s:could not create sysfs group for ps\n", __func__);
		goto input_p_sysfs_failed;
	}

bypass_prox_idev:
	if (!pdata->ges_name)
		goto bypass_ges_idev;
	
	chip->g_idev = input_allocate_device();
	if (!chip->g_idev) {
		dev_err(dev, "%s: no memory for input_dev '%s'\n",__func__, pdata->ges_name);
		ret = -ENODEV;
		goto input_g_alloc_failed;
	}
	chip->g_idev->name = pdata->ges_name;
	chip->g_idev->id.bustype = BUS_I2C;
	//set_bit(ABS_X, chip->g_idev->absbit);
	//input_set_abs_params(chip->g_idev, ABS_DISTANCE, 0, 65535, 0, 0);
	//set_bit(EV_ABS, chip->g_idev->evbit);

	dev_set_drvdata(&chip->g_idev->dev, chip);
	ret = input_register_device(chip->g_idev);
	if (ret) {
		input_free_device(chip->g_idev);
		dev_err(dev, "%s: cant register input '%s'\n",__func__, pdata->ges_name);
		goto input_g_register_failed;
	}


#if 0 /* delete it by lauson. */
	ret = tmg399x_add_sysfs_interfaces(&chip->g_idev->dev,
			prox_attrs, ARRAY_SIZE(prox_attrs));
	if (ret)
		goto input_g_sysfs_failed;
#endif 


bypass_ges_idev:
	if (!pdata->als_name)
		goto bypass_als_idev;
	chip->a_idev = input_allocate_device();
	if (!chip->a_idev) {
		dev_err(dev, "%s: no memory for input_dev '%s'\n",__func__, pdata->als_name);
		ret = -ENODEV;
		goto input_a_alloc_failed;
	}
	chip->a_idev->name = pdata->als_name;
	chip->a_idev->id.bustype = BUS_I2C;
	set_bit(EV_ABS, chip->a_idev->evbit);
	//set_bit(ABS_X, chip->a_idev->absbit);
	//set_bit(ABS_Y, chip->a_idev->absbit);
       set_bit(ABS_MISC, chip->a_idev->absbit);
	
	//input_set_abs_params(chip->a_idev, ABS_X, 0, 65535, 0, 0);
	//input_set_abs_params(chip->a_idev, ABS_Y, 0, 65535, 0, 0);
       input_set_abs_params(chip->a_idev, ABS_MISC, 0, 65535, 0, 0);

//	chip->a_idev->open = tmg399x_als_idev_open;
//	chip->a_idev->close = tmg399x_als_idev_close;
	dev_set_drvdata(&chip->a_idev->dev, chip);
	ret = input_register_device(chip->a_idev);
	if (ret) {
		input_free_device(chip->a_idev);
		dev_err(dev, "%s: cant register input '%s'\n",__func__, pdata->prox_name);
		goto input_a_register_failed;
	}
#if 0 /* delete it by lauson. */
	ret = tmg399x_add_sysfs_interfaces(&chip->a_idev->dev,
			als_attrs, ARRAY_SIZE(als_attrs));
	if (ret)
		goto input_a_sysfs_failed;
#endif 
	ret = sysfs_create_group(&chip->a_idev->dev.kobj, &tmg399x_als_attribute_group);
	if (ret < 0)
	{
		printk(KERN_ERR "%s:could not create sysfs group for ps\n", __func__);
		goto input_a_sysfs_failed;
	}

	init_timer(&chip->rgbc_timer);

bypass_als_idev:
	INIT_WORK(&chip->irq_work, tmg399x_irq_work);
	ret = request_threaded_irq(client->irq, NULL, &tmg399x_irq,
		      IRQF_TRIGGER_LOW | IRQF_ONESHOT,
		      dev_name(dev), chip);
	if (ret) {
		dev_info(dev, "Failed to request irq %d\n", client->irq);
		goto irq_register_fail;
	}

	INIT_WORK(&chip->work_thresh, tmg399x_beam_thread);

	/* make sure everything is ok before registering the class device */
	chip->als_cdev = sensors_light_cdev;
	chip->als_cdev.sensors_enable = tmg399x_als_enable_set;
	chip->als_cdev.sensors_poll_delay = tmg399x_als_poll_delay_set;
	ret = sensors_classdev_register(&client->dev, &chip->als_cdev);
	if (ret)
		goto irq_register_fail;

	chip->ps_cdev = sensors_proximity_cdev;
	chip->ps_cdev.sensors_enable = tmg399x_ps_enable_set;
	ret = sensors_classdev_register(&client->dev, &chip->ps_cdev);
	if (ret)
		goto irq_register_fail;    
	
	dev_info(dev, "tmg399x probe ok !!!!!\n");
	return 0;

irq_register_fail:
	if (chip->a_idev) {
	sysfs_remove_group(&chip->a_idev->dev.kobj, &tmg399x_als_attribute_group);
input_a_sysfs_failed:
		input_unregister_device(chip->a_idev);
input_a_register_failed:
		input_free_device(chip->a_idev);
	}
input_a_alloc_failed:
	if (chip->g_idev) {
	sysfs_remove_group(&chip->a_idev->dev.kobj, &tmg399x_ps_attribute_group);
//input_g_sysfs_failed:
		input_unregister_device(chip->g_idev);
input_g_register_failed:
		input_free_device(chip->g_idev);
	}	
input_g_alloc_failed:
	if (chip->p_idev) {
	sysfs_remove_group(&chip->a_idev->dev.kobj, &tmg399x_ps_attribute_group);
input_p_sysfs_failed:
		input_unregister_device(chip->p_idev);
input_p_register_failed:
		input_free_device(chip->p_idev);
	}	
input_p_alloc_failed:
power_deinit_failed:
    tmg399x_power_deinit(chip);
flush_regs_failed:
id_failed:
	kfree(chip->segment);
set_segment_failed:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
malloc_failed:
	if (powered && pdata->platform_power)
		pdata->platform_power(dev, GPOWER_OFF);
pon_failed:
	if (pdata->platform_teardown)
		pdata->platform_teardown(dev);
init_failed:
	dev_err(dev, "Probe failed.\n");
	return ret;
}

static int tmg399x_suspend(struct device *dev)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	struct tmg399x_i2c_platform_data *pdata = dev->platform_data;

	dev_info(dev, "%s\n", __func__);
	mutex_lock(&chip->lock);
	chip->in_suspend = 1;

	if (chip->p_idev && chip->p_idev->users) {
		if (pdata->proximity_can_wake) {
			dev_info(dev, "set wake on proximity\n");
			chip->wake_irq = 1;
		} else {
			dev_info(dev, "proximity off\n");
			tmg399x_prox_enable(chip, 0);
		}
	}
	if (chip->a_idev && chip->a_idev->users) {
		if (pdata->als_can_wake) {
			dev_info(dev, "set wake on als\n");
			chip->wake_irq = 1;
		} else {
			dev_info(dev, "als off\n");
			tmg399x_als_enable(chip, 0);
		}
	}
	if (chip->wake_irq) {
		irq_set_irq_wake(chip->client->irq, 1);
	} else if (!chip->unpowered) {
		dev_info(dev, "powering off\n");
		tmg399x_pltf_power_off(chip);
	}
	mutex_unlock(&chip->lock);

	return 0;
}

static int tmg399x_resume(struct device *dev)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	bool als_on, prx_on;
	int ret = 0;
	mutex_lock(&chip->lock);
	prx_on = chip->p_idev && chip->p_idev->users;
	als_on = chip->a_idev && chip->a_idev->users;
	chip->in_suspend = 0;

	dev_info(dev, "%s: powerd %d, als: needed %d  enabled %d",
			__func__, !chip->unpowered, als_on,
			chip->als_enabled);
	dev_info(dev, " %s: prox: needed %d  enabled %d\n",
			__func__, prx_on, chip->prx_enabled);

	if (chip->wake_irq) {
		irq_set_irq_wake(chip->client->irq, 0);
		chip->wake_irq = 0;
	}
	if (chip->unpowered && (prx_on || als_on)) {
		dev_info(dev, "powering on\n");
		ret = tmg399x_power_on(chip);
		if (ret)
			goto err_power;
	}
	if (prx_on && !chip->prx_enabled)
		(void)tmg399x_prox_enable(chip, 1);
	if (als_on && !chip->als_enabled)
		(void)tmg399x_als_enable(chip, 1);
	if (chip->irq_pending) {
		dev_info(dev, "%s: pending interrupt\n", __func__);
		chip->irq_pending = 0;
		(void)tmg399x_check_and_report(chip);
		enable_irq(chip->client->irq);
	}
err_power:
	mutex_unlock(&chip->lock);

	return 0;
}

static int  tmg399x_remove(struct i2c_client *client)
{
	struct tmg399x_chip *chip = i2c_get_clientdata(client);
	mutex_lock(&chip->lock);
	free_irq(client->irq, chip);
	if (chip->a_idev) {
	sysfs_remove_group(&chip->a_idev->dev.kobj, &tmg399x_als_attribute_group);
		input_unregister_device(chip->a_idev);
	}
	if (chip->p_idev) {
	sysfs_remove_group(&chip->a_idev->dev.kobj, &tmg399x_ps_attribute_group);
		input_unregister_device(chip->p_idev);
	}

	if (chip->g_idev) {
	//sysfs_remove_group(&chip->a_idev->dev.kobj, &tmg399x_ps_attribute_group);
		input_unregister_device(chip->g_idev);
	}
		
	if (chip->pdata->platform_teardown)
		chip->pdata->platform_teardown(&client->dev);
	i2c_set_clientdata(client, NULL);
	kfree(chip->segment);
	kfree(chip);
	mutex_unlock(&chip->lock);
	return 0;
}

static struct i2c_device_id tmg399x_idtable[] = {
	{ "tmg399x", 0 },
	{}
};

static struct of_device_id tmg399x_match_table[] = {
	{ .compatible = "tmg,tmg399x", },
	{ },
};

MODULE_DEVICE_TABLE(i2c, tmg399x_idtable);

static const struct dev_pm_ops tmg399x_pm_ops = {
	.suspend = tmg399x_suspend,
	.resume  = tmg399x_resume,
};

static struct i2c_driver tmg399x_driver = {
	.driver = {
		.name = "tmg399x",
              .of_match_table =  tmg399x_match_table,
		.pm = &tmg399x_pm_ops,
	},
	.id_table = tmg399x_idtable,
	.probe = tmg399x_probe,
	.remove = tmg399x_remove,
};

static int __init tmg399x_init(void)
{
	return i2c_add_driver(&tmg399x_driver);
}

static void __exit tmg399x_exit(void)
{
	i2c_del_driver(&tmg399x_driver);
}

module_init(tmg399x_init);
module_exit(tmg399x_exit);

MODULE_AUTHOR("J. August Brenner<jon.brenner@ams.com>");
MODULE_AUTHOR("Byron Shi<byron.shi@ams.com>");
MODULE_DESCRIPTION("AMS-TAOS tmg3992/3 Ambient, Proximity, Gesture sensor driver");
MODULE_LICENSE("GPL");
