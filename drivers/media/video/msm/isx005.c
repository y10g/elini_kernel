/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Sony 3M ISX005 camera sensor driver
 * Auther: Lee Hyung Tae[hyungtae.lee@lge.com], 2010-04-09
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include <linux/kthread.h>

#include "isx005.h"
#include "isx005_reg.h"

//LGE_UPDATE_S, haeyeon.park@lge.com 2010-10-22 For check CAM Preview
#include "../../../../arch/arm/mach-msm/smd_private.h"
//LGE_UPDATE_E,haeyeon.park@lge.com 2010-10-22 For check CAM Preview

#define ISX005_INTERVAL_T2		8	/* 8ms */
#define ISX005_INTERVAL_T3		1	/* 0.5ms */
#define ISX005_INTERVAL_T4		2	/* 15ms */
#define ISX005_INTERVAL_T5		25	/* 200ms */

#if defined(CONFIG_MACH_MSM7X27_THUNDERG) || defined(CONFIG_MACH_MSM7X27_THUNDERC) || defined(CONFIG_MACH_MSM7X27_ELINI)
/* LGE_CHANGE_S. Change code to apply new LUT for display quality. 2010-08-13. minjong.gong@lge.com */
extern mdp_load_thunder_lut(int lut_type);
#endif

/*
* AF Total steps parameters
*/
#define ISX005_TOTAL_STEPS_NEAR_TO_FAR	30

/*  ISX005 Registers  */
#define REG_ISX005_INTSTS_ID			0x00F8	/* Interrupt status */
#define REG_ISX005_INTCLR_ID			0x00FC	/* Interrupt clear */

#define ISX005_OM_CHANGED				0x0001	/* Operating mode */
#define ISX005_CM_CHANGED				0x0002	/* Camera mode */

DEFINE_MUTEX(isx005_tuning_mutex);
static int tuning_thread_run;

#define CFG_WQ_SIZE		64

struct config_work_queue {
	int cfgtype;
	int mode;
};

struct config_work_queue *cfg_wq = 0;
static int cfg_wq_num = 0;

/* It is distinguish normal from macro focus */
static int prev_af_mode;
/* It is distinguish scene mode */
static int prev_scene_mode;
/* It is distinguish scene mode */
static int prev_effect_mode;
/* It is distinguish scene mode */
static int prev_balance_mode;


struct isx005_work {
	struct work_struct work;
};
static struct isx005_work *isx005_sensorw;

static struct i2c_client *isx005_client;

struct isx005_ctrl {
	const struct msm_camera_sensor_info *sensordata;
};

static struct isx005_ctrl *isx005_ctrl;
// Start john.park@lge.com 2010-07-30 LGE_Domestic to add preview mode 
static uint16_t cm_light_check_sts = 0;
static DECLARE_WAIT_QUEUE_HEAD(isx005_wait_queue);

DEFINE_MUTEX(isx005_mutex);

struct platform_device *isx005_pdev;

int pclk_rate;
extern int mclk_rate;
static int always_on = 0;

//LGE_UPDATE_S, haeyeon.park@lge.com 2010-10-22 For check CAM Preview
void isx005_preview_onoff(int onoff)
{
     int* cam_preview_on = -1;		
     cam_preview_on = smem_alloc(SMEM_ID_VENDOR2_CAMERA_PREVIEW, sizeof(int));

     if(onoff == 1)
     {
	*cam_preview_on = 1;
     }
     else 
     {
	*cam_preview_on = 0;
     }
}
//LGE_UPDATE_E,haeyeon.park@lge.com 2010-10-22 For check CAM Preview

static int32_t isx005_i2c_txdata(unsigned short saddr,
	unsigned char *txdata, int length)
{
	struct i2c_msg msg[] = {
		{
			.addr = saddr,
			.flags = 0,
			.len = length,
			.buf = txdata,
		},
	};

	if (i2c_transfer(isx005_client->adapter, msg, 1) < 0) {
		printk(KERN_ERR "isx005_i2c_txdata failed\n");
		return -EIO;
	}

	return 0;
}

static int32_t isx005_i2c_write(unsigned short saddr,
	unsigned short waddr, unsigned short wdata, enum isx005_width width)
{
	int32_t rc = -EIO;
	unsigned char buf[4];

	memset(buf, 0, sizeof(buf));
	switch (width) {
		case BYTE_LEN:	
			buf[0] = (waddr & 0xFF00) >> 8;
			buf[1] = (waddr & 0x00FF);
			buf[2] = wdata;
			rc = isx005_i2c_txdata(saddr, buf, 3);
            printk(KERN_ERR "[### BYTE_LEN check] i2c_write , addr = 0x%04x, val = 0x%02x!\n", waddr, wdata);      
			break;
			
		case WORD_LEN:
			buf[0] = (waddr & 0xFF00) >> 8;
			buf[1] = (waddr & 0x00FF);
			buf[2] = (wdata & 0xFF00) >> 8;
			buf[3] = (wdata & 0x00FF);
			rc = isx005_i2c_txdata(saddr, buf, 4);
            printk(KERN_ERR "[### WORD_LEN check] i2c_write , addr = 0x%04x, val = 0x%04x!\n", waddr, wdata);            
			break;

		default:
			break;
	}

	if (rc < 0)
		printk(KERN_ERR "i2c_write failed, addr = 0x%x, val = 0x%x!\n", waddr, wdata);

	return rc;
}

static int32_t isx005_i2c_write_table(
	struct isx005_register_address_value_pair const *reg_conf_tbl,
	int num_of_items_in_table)
{
	int i;
	int32_t rc = -EIO;

	for (i = 0; i < num_of_items_in_table; ++i) {
		rc = isx005_i2c_write(isx005_client->addr,
			reg_conf_tbl->register_address, reg_conf_tbl->register_value,
			reg_conf_tbl->register_length);
		if (rc < 0)
			break;

		reg_conf_tbl++;
	}

	return rc;
}

static int isx005_i2c_rxdata(unsigned short saddr,
	unsigned char *rxdata, int length)
{
	struct i2c_msg msgs[] = {
		{
			.addr   = saddr,
			.flags = 0,
			.len   = 2,
			.buf   = rxdata,
		},
		{
			.addr   = saddr,
			.flags = I2C_M_RD,
			.len   = length,
			.buf   = rxdata,
		},
	};

	if (i2c_transfer(isx005_client->adapter, msgs, 2) < 0) {
		printk(KERN_ERR "isx005_i2c_rxdata failed!\n");
		return -EIO;
	}

	return 0;
}

static int32_t isx005_i2c_read(unsigned short   saddr,
	unsigned short raddr, unsigned short *rdata, enum isx005_width width)
{
	int32_t rc = 0;
	unsigned char buf[4];

	if (!rdata)
		return -EIO;

	memset(buf, 0, sizeof(buf));

	switch (width) {
		case BYTE_LEN:
			buf[0] = (raddr & 0xFF00) >> 8;
			buf[1] = (raddr & 0x00FF);
			rc = isx005_i2c_rxdata(saddr, buf, 2);
			if (rc < 0)
				return rc;
			*rdata = buf[0];
			break;
			
		case WORD_LEN:
			buf[0] = (raddr & 0xFF00) >> 8;
			buf[1] = (raddr & 0x00FF);
			rc = isx005_i2c_rxdata(saddr, buf, 2);
			if (rc < 0)
				return rc;
			*rdata = buf[0] << 8 | buf[1];
			break;

		default:
			break;
	}

	if (rc < 0)
		printk(KERN_ERR "isx005_i2c_read failed!\n");

	return rc;
}

#if defined(CONFIG_MACH_MSM7X27_THUNDERG) || defined(CONFIG_MACH_MSM7X27_THUNDERA)	 || defined(CONFIG_MACH_MSM7X27_ELINI)	
static int isx005_reg_init(void)
{
	int rc = 0;
	int i;

	/* Configure sensor for Initial setting (PLL, Clock, etc) */
	for (i = 0; i < isx005_regs.init_reg_settings_size; ++i) {
		rc = isx005_i2c_write(isx005_client->addr,
			isx005_regs.init_reg_settings[i].register_address,
			isx005_regs.init_reg_settings[i].register_value,
			isx005_regs.init_reg_settings[i].register_length);

		if (rc < 0)
			return rc;
	}

	return rc;
}
#else
static int isx005_reg_init(void)
{
	int rc = 0;
	int i;

	if (pclk_rate == 27) {
		/* Configure sensor for Initial setting (PLL, Clock, etc) */
		for (i = 0; i < isx005_regs.init_reg_settings_size; ++i) {
			rc = isx005_i2c_write(isx005_client->addr,
				isx005_regs.init_reg_settings[i].register_address,
				isx005_regs.init_reg_settings[i].register_value,
				isx005_regs.init_reg_settings[i].register_length);

			if (rc < 0)
				return rc;
		}
	} else if (pclk_rate == 32) {
		/* Configure sensor for Initial setting (PLL, Clock, etc) */
		for (i = 0; i < isx005_regs.init_reg32_settings_size; ++i) {
			rc = isx005_i2c_write(isx005_client->addr,
				isx005_regs.init_reg32_settings[i].register_address,
				isx005_regs.init_reg32_settings[i].register_value,
				isx005_regs.init_reg32_settings[i].register_length);
			
			if (rc < 0)
				return rc;
		}
	} else {
		printk(KERN_ERR "invalid pclk rate!\n");
		return -EINVAL;
	}

	return rc;
}
#endif

static int dequeue_sensor_config(int cfgtype, int mode);

static void dequeue_cfg_wq(struct config_work_queue *cfg_wq)
{
	int rc;
	int i;

	for (i = 0; i < cfg_wq_num; ++i) {
		rc = dequeue_sensor_config(cfg_wq[i].cfgtype, cfg_wq[i].mode);
		if (rc < 0) {
			printk(KERN_ERR "[ERROR]%s: dequeue sensor config error!\n",
				__func__);
			break;
		}
	}

	cfg_wq_num = 0;
}

static void enqueue_cfg_wq(int cfgtype, int mode)
{
	if (!cfg_wq) {
		cfg_wq_num = 0;
		return;
	}

	if (cfg_wq_num == CFG_WQ_SIZE)
		return;

	cfg_wq[cfg_wq_num].cfgtype = cfgtype;
	cfg_wq[cfg_wq_num].mode= mode;

	++cfg_wq_num;
}

int isx005_reg_tuning(void *data)
{
	int rc = 0;
	int i;

	mutex_lock(&isx005_tuning_mutex);
	cfg_wq = kmalloc(sizeof(struct config_work_queue) * CFG_WQ_SIZE,
		GFP_KERNEL);
	cfg_wq_num = 0;
	tuning_thread_run = 1;
	mutex_unlock(&isx005_tuning_mutex);
	
	/* Configure sensor for various tuning */
	for (i = 0; i < isx005_regs.tuning_reg_settings_size; ++i) {
		rc = isx005_i2c_write(isx005_client->addr,
			isx005_regs.tuning_reg_settings[i].register_address,
			isx005_regs.tuning_reg_settings[i].register_value,
			isx005_regs.tuning_reg_settings[i].register_length);

		if (rc < 0)
			break;
	}

	mutex_lock(&isx005_tuning_mutex);
	dequeue_cfg_wq(cfg_wq);
	kfree(cfg_wq);
	cfg_wq = 0;
	tuning_thread_run = 0;
	mutex_unlock(&isx005_tuning_mutex);

	return rc;
}

static int isx005_reg_preview(void)
{
	int rc = 0;
	int i;
      unsigned short cm_changed_status = 0;

	if(prev_scene_mode == CAMERA_SCENE_SPORTS )
	{
		rc = isx005_i2c_write(isx005_client->addr,0x4020,0x3202, WORD_LEN);       //{0x4020, 0xCA03, WORD_LEN},//  agcmaxscl_l : 18.33db  for Snapshot brightness	// 0x8d01
		if (rc < 0)
			return rc;		
	}
	else if(prev_scene_mode != CAMERA_SCENE_NIGHT)
	{
		rc = isx005_i2c_write(isx005_client->addr,0x4020,0xFF01, WORD_LEN);    // {0x4020,0xFF01, WORD_LEN},//  agcmaxscl_l : 16db  for Preview brightness // ver 0.1 : 0x0100
		if (rc < 0)
			return rc;		
	}

	/* Configure sensor for Preview mode */
	for (i = 0; i < isx005_regs.prev_reg_settings_size; ++i) {
		rc = isx005_i2c_write(isx005_client->addr,
		  isx005_regs.prev_reg_settings[i].register_address,
		  isx005_regs.prev_reg_settings[i].register_value,
		  isx005_regs.prev_reg_settings[i].register_length);

		if (rc < 0)
			return rc;
	}

	/* Checking the mode change status */
	/* BUG FIX : msm is changed preview mode but sensor is not change preview mode. */
	/*so there is case that vfe get capture image on preview mode */	
	/* eunyoung.shin@lge.com 2010.07.13*/
	for(i = 0; i < 300; i++)
	{
		unsigned short cm_changed_status = 0;
		rc = isx005_i2c_read(isx005_client->addr, 0x00F8, &cm_changed_status, BYTE_LEN);

		if(cm_changed_status & 0x0002)
		{
			printk(KERN_ERR "[%s]:Sensor Preview Mode check : %d-> success \n", __func__, cm_changed_status);
			break;
		}
		else
			msleep(10);
	}


	return rc;
}

static int isx005_reg_snapshot(void)
{
	int rc = 0;
	int i;
    unsigned short cm_changed_status= 0;

	/* Configure sensor for Snapshot mode */
	if(prev_scene_mode == CAMERA_SCENE_SPORTS )
	{
		rc = isx005_i2c_write(isx005_client->addr,0x4020,0x9406, WORD_LEN);       //{0x4020, 0xCA03, WORD_LEN},//  agcmaxscl_l : 18.33db  for Snapshot brightness	// 0x8d01
		if (rc < 0)
			return rc;		
	}
	else if(prev_scene_mode != CAMERA_SCENE_NIGHT)
	{
		rc = isx005_i2c_write(isx005_client->addr,0x4020,0xCA03, WORD_LEN);       //{0x4020, 0xCA03, WORD_LEN},//  agcmaxscl_l : 18.33db  for Snapshot brightness	// 0x8d01
		if (rc < 0)
			return rc;		
	}
	
	for (i = 0; i < isx005_regs.snap_reg_settings_size; ++i) {
		rc = isx005_i2c_write(isx005_client->addr,
			isx005_regs.snap_reg_settings[i].register_address,
			isx005_regs.snap_reg_settings[i].register_value,
			isx005_regs.snap_reg_settings[i].register_length);

		if (rc < 0)
			return rc;
	}

	/* Checking the mode change status */
	/* eunyoung.shin@lge.com 2010.07.13*/	
	for(i = 0; i < 300; i++)
	{
		printk(KERN_ERR "[%s]:Sensor Snapshot Mode Start\n", __func__);
		cm_changed_status = 0;
		rc = isx005_i2c_read(isx005_client->addr, 0x00F8, &cm_changed_status, BYTE_LEN);

		if(cm_changed_status & 0x0002)
		{
			printk(KERN_ERR "[%s]:Sensor Snapshot Mode check : %d-> success \n", __func__, cm_changed_status);
			break;
		}
		else
			msleep(10);

		printk(KERN_ERR "[%s]:Sensor Snapshot Mode checking : %d \n", __func__, cm_changed_status);
	}

	 return rc;
}

static int isx005_set_sensor_mode(int mode)
{
	int rc = 0;
	int retry = 0;

	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		for (retry = 0; retry < 3; ++retry) {
			rc = isx005_reg_preview();
			if (rc < 0)
				printk(KERN_ERR "[ERROR]%s:Sensor Preview Mode Fail\n", __func__);
			else
				break;
		}
		break;

	case SENSOR_SNAPSHOT_MODE:
	case SENSOR_RAW_SNAPSHOT_MODE:	/* Do not support */
		for (retry = 0; retry < 3; ++retry) {
			rc = isx005_reg_snapshot();
			if (rc < 0)
				printk(KERN_ERR "[ERROR]%s:Sensor Snapshot Mode Fail\n", __func__);
			else
				break;
		}
		break;

	default:
		return -EINVAL;
	}

	printk(KERN_ERR "Sensor Mode : %d, rc = %d\n", mode, rc);

	return rc;
}

static int isx005_cancel_focus(int mode)
{
	int rc;
	int lense_po_back=0;
	
	switch(mode){
	case 0:
		lense_po_back = 0x3200;
		break;
	
	case 1:
		lense_po_back = 0x0403;
		break;
	}

	rc = isx005_i2c_write(isx005_client->addr,
			0x002E, 0x02, BYTE_LEN);
	if (rc < 0)
		return rc;
	
	rc = isx005_i2c_write(isx005_client->addr,
			0x4852, lense_po_back, WORD_LEN);
	if (rc < 0)
		return rc;
	
	rc = isx005_i2c_write(isx005_client->addr,
			0x0012, 0x01, BYTE_LEN);
	if (rc < 0)
		return rc;
	
	rc = isx005_i2c_write(isx005_client->addr,
			0x4850, 0x01, BYTE_LEN);
	if (rc < 0)
		return rc;

	rc = isx005_i2c_write(isx005_client->addr,
			0x00FC, 0x1F, BYTE_LEN);
	if (rc < 0)
		return rc;

	return rc;
}

static int isx005_check_af_lock(void)
{
	int rc;
	int i;
	unsigned short af_lock;
	

	for (i = 0; i < 10; ++i) {
		/*INT state read -*/
		rc = isx005_i2c_read(isx005_client->addr,
			0x00F8, &af_lock, BYTE_LEN);
		
		if (rc < 0) {
			printk(KERN_ERR "isx005: reading af_lock fail\n");
			return rc;
		}

		/* af interruption lock state read compelete */
		if((af_lock & 0x10) == 0x10)
			break;

		msleep(10);
	}

	/* INT clear */
	rc = isx005_i2c_write(isx005_client->addr,
		0x00FC, 0x10, BYTE_LEN);
	if (rc < 0)
		return rc;

	for (i = 0; i < 10; ++i) {
		/*INT state read to confirm INT release state*/
		rc = isx005_i2c_read(isx005_client->addr,
				0x00F8, &af_lock, BYTE_LEN);
		
		if (rc < 0) {
			printk(KERN_ERR "isx005: reading af_lock fail\n");
			return rc;
		}

		if ((af_lock & 0x10) == 0x00) {
			printk(KERN_ERR "af_lock is released\n");
			break;
		}
		msleep(10);
	}

	return rc;
}

static int isx005_check_focus(int *lock)
{
	int rc;
	int i;	
	unsigned short af_status;
	unsigned short af_result;

	printk("isx005_check_focus\n");

	/*af status check  0:load, 1: init,  8: af_lock */
	for (i = 0; i < 120; ++i)
	{
		rc = isx005_i2c_read(isx005_client->addr,
			0x6D76, &af_status, BYTE_LEN);
		if (rc < 0) {
			printk("[isx005.c]%s: fail in reading af_status\n",__func__);
			return rc;
		}
    
		if(af_status == 0x8) 
      break;
		
		msleep(10);
	}

//	if (af_status != 0x8)
//		return -ETIME;
	
	isx005_check_af_lock();

	/* af result read  success / fail*/
	rc = isx005_i2c_read(isx005_client->addr, 0x6D77, &af_result, BYTE_LEN);
	if (rc < 0) {
		printk("[isx005.c]%s: fail in reading af_result\n",__func__);
		return rc;
	}

	/* single autofocus off */
	rc = isx005_i2c_write(isx005_client->addr, 0x002E, 0x03, BYTE_LEN);
	if (rc < 0)
		return rc;
		
	/* single autofocus refresh*/
	rc = isx005_i2c_write(isx005_client->addr, 0x0012, 0x01, BYTE_LEN);
	if (rc < 0)
		return rc;

	if (af_result == 1) {
		*lock = CFG_AF_LOCKED;		// success
		return rc;
	} else {
		*lock = CFG_AF_UNLOCKED;	// fail
		return rc;
	}

	return -ETIME;
}

static int isx005_set_af_start(int mode)
{
	int rc = 0;
   uint16_t cm_light_check_sts_local;

   printk(KERN_ERR "###john.park ; [CHECK]%s: prev_af_mode -> %d, mode -> %d \n", __func__, prev_af_mode, mode);
   

   rc = isx005_i2c_read(isx005_client->addr, 0x027A, &cm_light_check_sts , WORD_LEN);
   cm_light_check_sts_local = be16_to_cpu(cm_light_check_sts);
   printk(KERN_ERR "[#############################]\n");   
   printk(KERN_ERR "[### check] %s: cm_light_check_sts : 0x%x, cm_light_check_sts_local : 0x%x\n", __func__, cm_light_check_sts, cm_light_check_sts_local );
   printk(KERN_ERR "[#############################]\n");   

   #if 1 // john.park , test code  100908
	if(prev_af_mode == mode) {
	     //by sony_ 2010.08.26 low light Threshold(AE Scale) Swap.
             if(cm_light_check_sts_local >= 0x0994){ // 0x0994 -> 100 lux // low light // 0x0B33 : 50 lux, 0x0AAB : 70lux  , 0x076C : 200 lux 
            		rc = isx005_i2c_write_table(isx005_regs.af_start_low_light_reg_settings,
            			isx005_regs.af_start_low_light_reg_settings_size);
              }
             else{  // high light
            		rc = isx005_i2c_write_table(isx005_regs.af_start_high_light_reg_settings,
            			isx005_regs.af_start_high_light_reg_settings_size);
              }
	} else {
		switch (mode) {
			case FOCUS_NORMAL:
				rc = isx005_i2c_write_table(isx005_regs.af_normal_reg_settings,
					isx005_regs.af_normal_reg_settings_size);
				break;

			case FOCUS_MACRO:
				rc = isx005_i2c_write_table(isx005_regs.af_macro_reg_settings,
					isx005_regs.af_macro_reg_settings_size);
				break;

			case FOCUS_AUTO:	
				rc = isx005_i2c_write_table(isx005_regs.af_normal_reg_settings,
					isx005_regs.af_normal_reg_settings_size);
				break;

			case FOCUS_MANUAL:	
				rc = isx005_i2c_write_table(isx005_regs.af_manual_reg_settings,
					isx005_regs.af_manual_reg_settings_size);
				break;

			default:
				printk(KERN_ERR "[ERROR]%s: invalid af mode\n", __func__);
				break;
		}
		/*af start*/
	        //by sony_ 2010.08.26 low light Threshold(AE Scale) Swap.
             if(cm_light_check_sts_local >= 0x0994){ // low light // 0x0B33 : 50 lux, 0x0AAB : 70lux  , 0x076C : 200 lux 
            		rc = isx005_i2c_write_table(isx005_regs.af_start_low_light_reg_settings,
            			isx005_regs.af_start_low_light_reg_settings_size);
              }
             else{  // high light
            		rc = isx005_i2c_write_table(isx005_regs.af_start_high_light_reg_settings,
            			isx005_regs.af_start_high_light_reg_settings_size);
              }
	}	
#endif

	prev_af_mode = mode;
		
	return rc;
}

//BEGIN, sm.kwon@lge.com, 20100929, Add the function for setting af mode.[[
static int isx005_set_af_mode(int mode)
{
  int rc = 0;

  printk(KERN_ERR "[CHECK]%s: mode -> %d\n", __func__, mode);
	   
  isx005_cancel_focus(mode);

  switch (mode) {
  	case FOCUS_NORMAL:
  		rc = isx005_i2c_write_table(isx005_regs.af_normal_reg_settings,
  			isx005_regs.af_normal_reg_settings_size);
  		break;

  	case FOCUS_MACRO:
  		rc = isx005_i2c_write_table(isx005_regs.af_macro_reg_settings,
  			isx005_regs.af_macro_reg_settings_size);
  		break;

  	case FOCUS_AUTO:	
  		rc = isx005_i2c_write_table(isx005_regs.af_normal_reg_settings,
  			isx005_regs.af_normal_reg_settings_size);
  		break;

  	case FOCUS_MANUAL:	
  		rc = isx005_i2c_write_table(isx005_regs.af_manual_reg_settings,
  			isx005_regs.af_manual_reg_settings_size);
  		break;

  	default:
  		printk(KERN_ERR "[ERROR]%s: invalid af mode\n", __func__);
  		break;
  }

  prev_af_mode = mode;

  return rc;
}
//]]END, sm.kwon@lge.com
static int isx005_move_focus(int32_t steps)
{
	int32_t rc = 0;
	unsigned short cm_changed_sts, cm_changed_clr, af_pos, manual_pos;
	int i;
	/* LGE_CHANGE_S, DCM_JP [kwangnyung.ryu], 2010.12.24, <manual focus table 수정> */
	static uint32_t manual_focus_value[] = {530, 468, 408, 375, 342, 309, 276, 243, 210, 150, 50};
	//static uint32_t manual_focus_value[] = {580, 468, 408, 375, 342, 309, 276, 243, 210, 150, 50};
	//static uint32_t manual_focus_value[] = {410, 318, 278, 250, 235, 215, 200, 190, 170, 150, 50};
	/* LGE_CHANGE_E, DCM_JP [kwangnyung.ryu], 2010.12.24, <manual focus table 수정> */


       printk(KERN_ERR "###john.park ; [CHECK]%s: steps -> %d\n", __func__, steps);

	isx005_cancel_focus(1);
	   
	rc = isx005_i2c_write_table(isx005_regs.af_manual_reg_settings,
			isx005_regs.af_manual_reg_settings_size);

	prev_af_mode = FOCUS_MANUAL;

	if (rc < 0) {
		printk(KERN_ERR "[ERROR]%s:fail in writing for move focus\n",
			__func__);
		return rc;
	}
  
//BEGIN, sm.kwon@lge.com, 20101001, fix move focus at capturing time.
//  if(steps < 0)
//  {
//      printk(KERN_ERR "If the steps is smaller than 0, keep previous step.");
//      return rc;
//  }
//END, sm.kwon@lge.com

	/* check cm_changed_sts */
	for(i = 0; i < 24; ++i) {
		rc = isx005_i2c_read(isx005_client->addr,
				0x00F8, &cm_changed_sts, BYTE_LEN);
		if (rc < 0){
			printk(KERN_ERR "[ERROR]%s; fail in reading cm_changed_sts\n",
				__func__);
			return rc;
		}

		if((cm_changed_sts & 0x02) == 0x02)
			break;

		msleep(10);
	}

	/* clear the interrupt register */
	rc = isx005_i2c_write(isx005_client->addr, 0x00FC, 0x02, BYTE_LEN);
	if (rc < 0)
		return rc;

	/* check cm_changed_clr */
	for(i = 0; i < 24; ++i) {
		rc = isx005_i2c_read(isx005_client->addr,
			0x00FC, &cm_changed_clr, BYTE_LEN);
		if (rc < 0) {
			printk(KERN_ERR "[ERROR]%s:fail in reading cm_changed_clr\n",
				__func__);
			return rc;
		}

		if((cm_changed_clr & 0x00) == 0x00)
			break;

		msleep(10);
	}

	if (steps <= 10)
	    manual_pos = cpu_to_be16(manual_focus_value[steps]);
	else
		manual_pos = 50;

	rc = isx005_i2c_write(isx005_client->addr, 0x4852, manual_pos, WORD_LEN);
	if (rc < 0)
		return rc;

	rc = isx005_i2c_write(isx005_client->addr, 0x4850, 0x01, BYTE_LEN);
	if (rc < 0)
		return rc;
	
	rc = isx005_i2c_write(isx005_client->addr, 0x0015, 0x01, BYTE_LEN);
	if (rc < 0)
		return rc;

	isx005_check_af_lock();
	
	/* check lens position */
	for(i = 0; i < 24; ++i) {
		rc = isx005_i2c_read(isx005_client->addr, 0x6D7A, &af_pos, WORD_LEN);
		if (rc < 0) {
			printk(KERN_ERR "[ERROR]%s:fail in reading af_lenspos\n",
				__func__);
			return rc;
		}
	
		if(af_pos == manual_pos)
			break;
		
		msleep(10);
	}

	return rc;
}

static int isx005_set_default_focus(void)
{
	int rc;

	rc = isx005_cancel_focus(prev_af_mode);
	if (rc < 0) {
		printk(KERN_ERR "[ERROR]%s:fail in cancel_focus\n", __func__);
		return rc;
	}

	rc = isx005_i2c_write_table(isx005_regs.af_normal_reg_settings,
		isx005_regs.af_normal_reg_settings_size);

	prev_af_mode = FOCUS_AUTO;

	if (rc < 0) {
		printk(KERN_ERR "[ERROR]%s:fail in writing for focus\n", __func__);
		return rc;
	}

	msleep(60);
	
	isx005_check_focus(&rc);

	return rc;
}


static int isx005_set_effect(int effect)
{
	int rc = 0;

	if(prev_effect_mode == effect)
	{
		printk(KERN_ERR "###john.park ; [CHECK]%s: skip this function, effect_mode -> %d\n", __func__, effect);
		return rc;
	}

       printk(KERN_ERR "###john.park ; [CHECK]%s: effect -> %d\n", __func__, effect);
	
	switch (effect) {
	case CAMERA_EFFECT_OFF:
		rc = isx005_i2c_write(isx005_client->addr, 0x005F, 0x00, BYTE_LEN);
		if (rc < 0)
			return rc;

		rc = isx005_i2c_write(isx005_client->addr, 0x038A, 0x6911, WORD_LEN);
		if (rc < 0)
			return rc;

		break;

	case CAMERA_EFFECT_MONO:
		rc = isx005_i2c_write(isx005_client->addr, 0x005F, 0x04, BYTE_LEN);
		if (rc < 0)
			return rc;

		rc = isx005_i2c_write(isx005_client->addr, 0x038A, 0x6911, WORD_LEN);
		if (rc < 0)
			return rc;

		break;

	case CAMERA_EFFECT_NEGATIVE:
		rc = isx005_i2c_write(isx005_client->addr, 0x005F, 0x02, BYTE_LEN);
		if (rc < 0)
			return rc;

		rc = isx005_i2c_write(isx005_client->addr, 0x038A, 0x6911, WORD_LEN);
		if (rc < 0)
			return rc;

		break;

	case CAMERA_EFFECT_SOLARIZE:
		rc = isx005_i2c_write(isx005_client->addr, 0x005F, 0x01, BYTE_LEN);
		if (rc < 0)
			return rc;

		rc = isx005_i2c_write(isx005_client->addr, 0x038A, 0x6911, WORD_LEN);
		if (rc < 0)
			return rc;

		break;

	case CAMERA_EFFECT_SEPIA:
		rc = isx005_i2c_write(isx005_client->addr, 0x005F, 0x03, BYTE_LEN);
		if (rc < 0)
			return rc;

		rc = isx005_i2c_write(isx005_client->addr, 0x038A, 0x6911, WORD_LEN);
		if (rc < 0)
			return rc;
		
		break;

	/* This effect is not supported in ISX005 */
	case CAMERA_EFFECT_POSTERIZE:
		rc = isx005_i2c_write(isx005_client->addr, 0x005F, 0x00, BYTE_LEN);
		if (rc < 0)
			return rc;

		rc = isx005_i2c_write(isx005_client->addr, 0x038A, 0x6911, WORD_LEN);
		if (rc < 0)
			return rc;

		break;

	/* This effect is not supported in ISX005 */
	case CAMERA_EFFECT_AQUA:
		rc = isx005_i2c_write(isx005_client->addr, 0x005F, 0x00, BYTE_LEN);
		if (rc < 0)
			return rc;

		rc = isx005_i2c_write(isx005_client->addr, 0x038A, 0x6911, WORD_LEN);
		if (rc < 0)
			return rc;

		break;

	case CAMERA_EFFECT_NEGATIVE_SEPIA:
		rc = isx005_i2c_write(isx005_client->addr, 0x005F, 0x02, BYTE_LEN);
		if (rc < 0)
			return rc;

		rc = isx005_i2c_write(isx005_client->addr, 0x038A, 0x6D11, WORD_LEN);
		if (rc < 0)
			return rc;
		
		break;

	case CAMERA_EFFECT_BLUE:
		rc = isx005_i2c_write(isx005_client->addr, 0x005F, 0x03, BYTE_LEN);
		if (rc < 0)
			return rc;

		rc = isx005_i2c_write(isx005_client->addr, 0x038A, 0x6D11, WORD_LEN);
		if (rc < 0)
			return rc;
		break;

	case CAMERA_EFFECT_PASTEL:
		rc = isx005_i2c_write(isx005_client->addr, 0x005F, 0x05, BYTE_LEN);
		if (rc < 0)
			return rc;

		rc = isx005_i2c_write(isx005_client->addr, 0x038A, 0x6911, WORD_LEN);
		if (rc < 0)
			return rc;
		break;		

	default:
		return -EINVAL;
	}
	prev_effect_mode = effect;

	CDBG("Effect : %d, rc = %d\n", effect, rc);

	return rc;
}

static int isx005_set_wb(int mode)
{
	int32_t rc = 0;

	if(prev_balance_mode == mode)
	{
		printk(KERN_ERR "###john.park ; [CHECK]%s: skip this function, wb_mode -> %d\n", __func__, mode);
		return rc;
	}
       printk(KERN_ERR "###john.park ; [CHECK]%s: mode -> %d\n", __func__, mode);

	switch (mode) {
		case CAMERA_WB_AUTO:
			rc = isx005_i2c_write(isx005_client->addr, 0x4453, 0x7B, BYTE_LEN);
			if (rc < 0)
				return rc;

			rc = isx005_i2c_write(isx005_client->addr, 0x0102, 0x20, BYTE_LEN);
			if (rc < 0)
				return rc;
			
			break;
			
		case CAMERA_WB_CUSTOM:	/* Do not support */
			rc = isx005_i2c_write(isx005_client->addr, 0x4453, 0x7B, BYTE_LEN);
			if (rc < 0)
				return rc;

			rc = isx005_i2c_write(isx005_client->addr, 0x0102, 0x20, BYTE_LEN);
			if (rc < 0)
				return rc;
			
			break;

		case CAMERA_WB_INCANDESCENT:
			rc = isx005_i2c_write(isx005_client->addr, 0x4453, 0x3B, BYTE_LEN);
			if (rc < 0)
				return rc;

			rc = isx005_i2c_write(isx005_client->addr, 0x0102, 0x08, BYTE_LEN);
			if (rc < 0)
				return rc;

			break;

		case CAMERA_WB_FLUORESCENT:
			rc = isx005_i2c_write(isx005_client->addr, 0x4453, 0x3B, BYTE_LEN);
			if (rc < 0)
				return rc;

			rc = isx005_i2c_write(isx005_client->addr, 0x0102, 0x07, BYTE_LEN);
			if (rc < 0)
				return rc;

			break;
			
		case CAMERA_WB_DAYLIGHT:
			rc = isx005_i2c_write(isx005_client->addr, 0x4453, 0x3B, BYTE_LEN);
			if (rc < 0)
				return rc;

			rc = isx005_i2c_write(isx005_client->addr, 0x0102, 0x04, BYTE_LEN);
			if (rc < 0)
				return rc;

			break;

		case CAMERA_WB_CLOUDY_DAYLIGHT:
			rc = isx005_i2c_write(isx005_client->addr, 0x4453, 0x3B, BYTE_LEN);
			if (rc < 0)
				return rc;

			rc = isx005_i2c_write(isx005_client->addr, 0x0102, 0x06, BYTE_LEN);
			if (rc < 0)
				return rc;
			
			break;

		case CAMERA_WB_TWILIGHT:	/* Do not support */
			rc = isx005_i2c_write(isx005_client->addr, 0x4453, 0x7B, BYTE_LEN);
			if (rc < 0)
				return rc;

			rc = isx005_i2c_write(isx005_client->addr, 0x0102, 0x20, BYTE_LEN);
			if (rc < 0)
				return rc;

			break;

		case CAMERA_WB_SHADE:		/* Do not support */
			rc = isx005_i2c_write(isx005_client->addr, 0x4453, 0x7B, BYTE_LEN);
			if (rc < 0)
				return rc;

			rc = isx005_i2c_write(isx005_client->addr, 0x0102, 0x20, BYTE_LEN);
			if (rc < 0)
				return rc;
			
			break;

		default:
			return -EINVAL;
	}
	prev_balance_mode = mode;
	return rc;
}

static int isx005_set_antibanding(int mode)
{
	int rc;

	switch (mode) {
		case CAMERA_ANTIBANDING_OFF:
			rc = isx005_i2c_write(isx005_client->addr, 0x4001, 0x00, BYTE_LEN);
			if (rc < 0)
				return rc;

			break;

		case CAMERA_ANTIBANDING_60HZ:
			rc = isx005_i2c_write(isx005_client->addr, 0x4001, 0x04, BYTE_LEN);
			if (rc < 0)
				return rc;

			break;

		case CAMERA_ANTIBANDING_50HZ:
			rc = isx005_i2c_write(isx005_client->addr, 0x4001, 0x03, BYTE_LEN);
			if (rc < 0)
				return rc;

			break;

		case CAMERA_ANTIBANDING_AUTO:
			rc = isx005_i2c_write(isx005_client->addr, 0x4001, 0x00, BYTE_LEN);
			if (rc < 0)
				return rc;

			break;

		case CAMERA_MAX_ANTIBANDING:
			rc = isx005_i2c_write(isx005_client->addr, 0x4001, 0x04, BYTE_LEN);
			if (rc < 0)
				return rc;

			break;

		default:
			return -EINVAL;
	}

	return rc;	
}

static int isx005_set_iso(int iso)
{
	int32_t rc;
	
	switch (iso) {
		case CAMERA_ISO_AUTO:
			rc = isx005_i2c_write(isx005_client->addr,
					0x01E5, 0x00, BYTE_LEN);
			break;

		case CAMERA_ISO_DEBLUR:	/* Do not support */
		case CAMERA_ISO_100:
			rc = isx005_i2c_write(isx005_client->addr,
					0x01E5, 0x04, BYTE_LEN);
			break;

		case CAMERA_ISO_200:
			rc = isx005_i2c_write(isx005_client->addr,
					0x01E5, 0x07, BYTE_LEN);
			break;

		case CAMERA_ISO_400:
			rc = isx005_i2c_write(isx005_client->addr,
					0x01E5, 0x0A, BYTE_LEN);
			break;
			
		case CAMERA_ISO_800:
			rc = isx005_i2c_write(isx005_client->addr,
					0x01E5, 0x0D, BYTE_LEN);
			break;

		default:
			rc = -EINVAL;
	}
	
	return rc;
}

static int32_t isx005_set_scene_mode(int8_t mode)
{
	int32_t rc = 0;

	if (prev_scene_mode == mode)
		return rc;

	switch (mode) {
		case CAMERA_SCENE_AUTO:
			rc = isx005_i2c_write_table(isx005_regs.scene_auto_reg_settings,
				isx005_regs.scene_auto_reg_settings_size);
			break;

		case CAMERA_SCENE_PORTRAIT:
			rc = isx005_i2c_write_table(isx005_regs.scene_portrait_reg_settings,
				isx005_regs.scene_portrait_reg_settings_size);
			break;

		case CAMERA_SCENE_LANDSCAPE:
			rc = isx005_i2c_write_table(isx005_regs.scene_landscape_reg_settings,
				isx005_regs.scene_landscape_reg_settings_size);
			break;

		case CAMERA_SCENE_SPORTS:
			rc = isx005_i2c_write_table(isx005_regs.scene_sports_reg_settings,
				isx005_regs.scene_sports_reg_settings_size);
			break;

		case CAMERA_SCENE_SUNSET:
			rc = isx005_i2c_write_table(isx005_regs.scene_sunset_reg_settings,
				isx005_regs.scene_sunset_reg_settings_size);
			break;

		case CAMERA_SCENE_NIGHT:
			rc = isx005_i2c_write_table(isx005_regs.scene_night_reg_settings,
				isx005_regs.scene_night_reg_settings_size);
			break;

		default:
			printk(KERN_ERR "[ERROR]%s:Incorrect scene mode value\n", __func__);
	}

	prev_scene_mode = mode;

	return rc;
}

static int32_t isx005_set_brightness(int8_t brightness)
{
	int32_t rc=0;
	
	switch (brightness) {
		case 0:
			rc = isx005_i2c_write(isx005_client->addr,
					0x0060, 0x80, BYTE_LEN);
			if(rc<0)
				return rc;
			
			rc = isx005_i2c_write(isx005_client->addr,
					0x0061, 0x50, BYTE_LEN);
			if(rc<0)
				return rc;
			
			break;

		case 1:
			rc = isx005_i2c_write(isx005_client->addr,
					0x0060, 0x80, BYTE_LEN);
			if(rc<0)
				return rc;	

			rc = isx005_i2c_write(isx005_client->addr,
					0x0061, 0x60, BYTE_LEN);
			if(rc<0)
				return rc;	

			break;

		case 2:
			rc = isx005_i2c_write(isx005_client->addr,
					0x0060, 0x80, BYTE_LEN);
			if(rc<0)
				return rc;	

			rc = isx005_i2c_write(isx005_client->addr,
					0x0061, 0x70, BYTE_LEN);
			if(rc<0)
				return rc;	

			break;

		case 3:
			rc = isx005_i2c_write(isx005_client->addr,
					0x0060, 0xCD, BYTE_LEN);
			if(rc<0)
				return rc;	

			rc = isx005_i2c_write(isx005_client->addr,
					0x0061, 0x80, BYTE_LEN);
			if(rc<0)
			return rc;	

			break;

		case 4:
			rc = isx005_i2c_write(isx005_client->addr,
					0x0060, 0xEF, BYTE_LEN);
			if(rc<0)
				return rc;	

			rc = isx005_i2c_write(isx005_client->addr,
					0x0061, 0x80, BYTE_LEN);
			if(rc<0)
				return rc;	

			break;

		case 5:
			rc = isx005_i2c_write(isx005_client->addr,
					0x0060, 0x00, BYTE_LEN);
			if(rc<0)
				return rc;	

			rc = isx005_i2c_write(isx005_client->addr,
					0x0061, 0x80, BYTE_LEN);
			if(rc<0)
				return rc;	

			break;

		case 6:
			rc = isx005_i2c_write(isx005_client->addr,
					0x0060, 0x18, BYTE_LEN);
			if(rc<0)
				return rc;	

			rc = isx005_i2c_write(isx005_client->addr,
					0x0061, 0x80, BYTE_LEN);
			if(rc<0)
				return rc;	

			break;

		case 7:
			rc = isx005_i2c_write(isx005_client->addr,
					0x0060, 0x7F, BYTE_LEN);
			if(rc<0)
				return rc;	

			rc = isx005_i2c_write(isx005_client->addr,
					0x0061, 0x8A, BYTE_LEN);
			if(rc<0)
				return rc;	

			break;

		case 8:
			rc = isx005_i2c_write(isx005_client->addr,
					0x0060, 0x7F, BYTE_LEN);
			if(rc<0)
				return rc;	

			rc = isx005_i2c_write(isx005_client->addr,
					0x0061, 0x9C, BYTE_LEN);
			if(rc<0)
				return rc;	

			break;

		case 9:
			rc = isx005_i2c_write(isx005_client->addr,
					0x0060, 0x7F, BYTE_LEN);
			if(rc<0)
				return rc;	

			rc = isx005_i2c_write(isx005_client->addr,
					0x0061, 0xAA, BYTE_LEN);
			if(rc<0)
				return rc;	

			break;

		case 10:
			rc = isx005_i2c_write(isx005_client->addr,
					0x0060, 0x7F, BYTE_LEN);
			if(rc<0)
				return rc;	

			rc = isx005_i2c_write(isx005_client->addr,
					0x0061, 0xC8, BYTE_LEN);
			if(rc<0)
				return rc;	

			break;

		default:
			printk(KERN_ERR "[ERROR]%s:incoreect brightness value\n",
				__func__);
	}
	
	return rc;
}

static int isx005_init_sensor(const struct msm_camera_sensor_info *data)
{
	int rc;
	int nNum = 0;
	struct task_struct *p;

	rc = data->pdata->camera_power_on();
	if (rc < 0) {
		printk(KERN_ERR "[ERROR]%s:failed to power on!\n", __func__);
		return rc;
	}

	/*pll register write*/
	rc = isx005_reg_init();
	if (rc < 0) {
		for(nNum = 0; nNum<5; nNum++)
		{
		 msleep(2);
			printk(KERN_ERR "[ERROR]%s:Set initial register error! retry~! \n", __func__);
			rc = isx005_reg_init();
			if(rc < 0)
			{
				nNum++;
				printk(KERN_ERR "[ERROR]%s:Set initial register error!- loop no:%d \n", __func__, nNum);
			}
			else
			{
				printk(KERN_DEBUG"[%s]:Set initial register Success!\n", __func__);
				break;
			}
		}
	}

	mdelay(16);  // T3+T4

	while (tuning_thread_run)
		msleep(10);

	mutex_lock(&isx005_tuning_mutex);
	p = kthread_run(isx005_reg_tuning, 0, "reg_tuning");
	mutex_unlock(&isx005_tuning_mutex);

	if (IS_ERR(p))
		return PTR_ERR(p);

	return rc;
}

static int isx005_sensor_init_probe(const struct msm_camera_sensor_info *data)
{
	int rc = 0;

	printk(KERN_ERR "init entry \n");

	if (data == 0) {
		printk(KERN_ERR "[ERROR]%s: data is null!\n", __func__);
		return -1;
	}

#if defined(CONFIG_MACH_MSM7X27_THUNDERG) || defined(CONFIG_MACH_MSM7X27_THUNDERC) || defined(CONFIG_MACH_MSM7X27_ELINI)
	/* LGE_CHANGE_S. Change code to apply new LUT for display quality. 2010-08-13. minjong.gong@lge.com */
	mdp_load_thunder_lut(2);	// Camera LUT
#endif

	mutex_lock(&isx005_mutex);
	rc = isx005_init_sensor(data);
	mutex_unlock(&isx005_mutex);

	if (rc < 0) {
		printk(KERN_ERR "[ERROR]%s:failed to initialize sensor!\n", __func__);
		goto init_probe_fail;
	}

	prev_af_mode = -1;
	prev_scene_mode = -1;
	prev_effect_mode = -1;
	prev_balance_mode = -1;

	//LGE_UPDATE_S, haeyeon.park@lge.com 2010-10-22, For check to Camera Preview
	isx005_preview_onoff(1);
	//LGE_UPDATE_E, haeyeon.park@lge.com 2010-10-22

	return rc;

init_probe_fail:
	return rc;
}

int isx005_sensor_init(const struct msm_camera_sensor_info *data)
{
	int rc = 0;

	isx005_ctrl = kzalloc(sizeof(struct isx005_ctrl), GFP_KERNEL);
	if (!isx005_ctrl) {
		printk(KERN_ERR "[ERROR]%s:isx005_init failed!\n", __func__);
		rc = -ENOMEM;
		goto init_done;
	}

	if (data)
		isx005_ctrl->sensordata = data;

	rc = isx005_sensor_init_probe(data);
	if (rc < 0) {
		printk(KERN_ERR "[ERROR]%s:isx005_sensor_init failed!\n", __func__);
		goto init_fail;
	}

init_done:
	return rc;

init_fail:
	kfree(isx005_ctrl);
	return rc;
}

int isx005_sensor_release(void)
{
	int rc = 0;

	if(always_on) {
		printk("always power-on camera.\n");
		return rc;
	}

	mutex_lock(&isx005_mutex);

	rc = isx005_ctrl->sensordata->pdata->camera_power_off();

	kfree(isx005_ctrl);

	mutex_unlock(&isx005_mutex);

#if defined(CONFIG_MACH_MSM7X27_THUNDERG) || defined(CONFIG_MACH_MSM7X27_THUNDERC) || defined(CONFIG_MACH_MSM7X27_ELINI)
		/* LGE_CHANGE_S. Change code to apply new LUT for display quality. 2010-08-13. minjong.gong@lge.com */
		mdp_load_thunder_lut(1);	// Normal LUT
#endif

	//LGE_UPDATE_S, haeyeon.park@lge.com 2010-10-22, For check to Camera Preview
	isx005_preview_onoff(0);
	//LGE_UPDATE_E, haeyeon.park@lge.com 2010-10-22

	return rc;
}

static int dequeue_sensor_config(int cfgtype, int mode)
{
	int rc;

	switch (cfgtype) {
		case CFG_SET_MODE:
			rc = isx005_set_sensor_mode(mode);
			break;

		case CFG_SET_EFFECT:
			rc = isx005_set_effect(mode);
			break;

		case CFG_MOVE_FOCUS:
			rc = isx005_move_focus(mode);
			break;

		case CFG_SET_DEFAULT_FOCUS:
			rc = isx005_set_default_focus();
			break;

//BEGIN, sm.kwon@lge.com, 20100929, Add the function for setting af mode.[[
    case CFG_SET_PARM_AF_MODE:
      rc = isx005_set_af_mode(mode);
      break;
//]]END sm.kwon@lge.com

		case CFG_SET_WB:
			rc = isx005_set_wb(mode);
			break;

		case CFG_SET_ANTIBANDING:
			rc= isx005_set_antibanding(mode);
			break;

		case CFG_SET_ISO:
			rc = isx005_set_iso(mode);
			break;

		case CFG_SET_SCENE:
			rc = isx005_set_scene_mode(mode);
			break;

		case CFG_SET_BRIGHTNESS:
			rc = isx005_set_brightness(mode);
			break;

		default:
			rc = 0;
			break;
	}

	return rc;
}

int isx005_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cfg_data;
	int rc;

	rc = copy_from_user(&cfg_data, (void *)argp,
		sizeof(struct sensor_cfg_data));

	if (rc < 0)
		return -EFAULT;

	printk(KERN_ERR "isx005_ioctl, cfgtype = %d, mode = %d\n",
		cfg_data.cfgtype, cfg_data.mode);

	mutex_lock(&isx005_tuning_mutex);
	if (tuning_thread_run) {
		if (cfg_data.cfgtype == CFG_MOVE_FOCUS)
			cfg_data.mode = cfg_data.cfg.focus.steps;

		enqueue_cfg_wq(cfg_data.cfgtype, cfg_data.mode);
		mutex_unlock(&isx005_tuning_mutex);
		return rc;
	}
	mutex_unlock(&isx005_tuning_mutex);

	mutex_lock(&isx005_mutex);

	switch (cfg_data.cfgtype) {
		case CFG_SET_MODE:
			rc = isx005_set_sensor_mode(cfg_data.mode);
			break;

		case CFG_SET_EFFECT:
			rc = isx005_set_effect(cfg_data.mode);
			break;

		case CFG_MOVE_FOCUS:
			rc = isx005_move_focus(cfg_data.cfg.focus.steps);
			break;

		case CFG_SET_DEFAULT_FOCUS:
			rc = isx005_set_default_focus();
			break;

		case CFG_GET_AF_MAX_STEPS:
			cfg_data.max_steps = ISX005_TOTAL_STEPS_NEAR_TO_FAR;
			if (copy_to_user((void *)argp,
					&cfg_data,
					sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_START_AF_FOCUS:
			rc = isx005_set_af_start(cfg_data.mode);
			break;

		case CFG_CHECK_AF_DONE:
			rc = isx005_check_focus(&cfg_data.mode);
			if (copy_to_user((void *)argp,
					&cfg_data,
					sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_CHECK_AF_CANCEL:
			rc = isx005_cancel_focus(cfg_data.mode);
			break;

//BEGIN, sm.kwon@lge.com, 20100929, Add the function for setting af mode.[[
    case CFG_SET_PARM_AF_MODE:
      rc = isx005_set_af_mode(cfg_data.mode);
      break;
//]]END, sm.kwon@lge.com

		case CFG_SET_WB:
			rc = isx005_set_wb(cfg_data.mode);
			break;

		case CFG_SET_ANTIBANDING:
			rc= isx005_set_antibanding(cfg_data.mode);
			break;

		case CFG_SET_ISO:
			rc = isx005_set_iso(cfg_data.mode);
			break;

		case CFG_SET_SCENE:
			rc = isx005_set_scene_mode(cfg_data.mode);
			break;

		case CFG_SET_BRIGHTNESS:
			rc = isx005_set_brightness(cfg_data.mode);
			break;

		default:
			rc = -EINVAL;
			break;
	}

	mutex_unlock(&isx005_mutex);

	return rc;
}

static const struct i2c_device_id isx005_i2c_id[] = {
	{ "isx005", 0},
	{ },
};

static int isx005_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&isx005_wait_queue);
	return 0;
}

static int isx005_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		rc = -ENOTSUPP;
		goto probe_failure;
	}

	isx005_sensorw = kzalloc(sizeof(struct isx005_work), GFP_KERNEL);
	if (!isx005_sensorw) {
		printk(KERN_ERR "kzalloc failed.\n");
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, isx005_sensorw);
	isx005_init_client(client);
	isx005_client = client;

	printk(KERN_ERR "isx005_probe succeeded!\n");

	return rc;

probe_failure:
	printk(KERN_ERR "[ERROR]%s:isx005_probe failed!\n", __func__);
	return rc;
}

static struct i2c_driver isx005_i2c_driver = {
	.id_table = isx005_i2c_id,
	.probe  = isx005_i2c_probe,
	.remove = __exit_p(isx005_i2c_remove),
	.driver = {
		.name = "isx005",
	},
};

static ssize_t pclk_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk("mclk_rate = %d\n", pclk_rate);
	return 0;
}

static ssize_t pclk_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int value;

	sscanf(buf, "%d", &value);
	pclk_rate = value;

	printk("pclk_rate = %d\n", pclk_rate);
	return size;
}

static DEVICE_ATTR(pclk, S_IRWXUGO, pclk_show, pclk_store);

static ssize_t mclk_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk("mclk_rate = %d\n", mclk_rate);
	return 0;
}

static ssize_t mclk_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int value;

	sscanf(buf, "%d", &value);
	mclk_rate = value;

	printk("mclk_rate = %d\n", mclk_rate);
	return size;
}

static DEVICE_ATTR(mclk, S_IRWXUGO, mclk_show, mclk_store);

static ssize_t always_on_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk("always_on = %d\n", always_on);
	return 0;
}

static ssize_t always_on_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int value;

	sscanf(buf, "%d", &value);
	always_on = value;

	printk("always_on = %d\n", always_on);
	return size;
}

static DEVICE_ATTR(always_on, S_IRWXUGO, always_on_show, always_on_store);

static int isx005_sensor_probe(const struct msm_camera_sensor_info *info,
				struct msm_sensor_ctrl *s)
{
	int rc = i2c_add_driver(&isx005_i2c_driver);
	if (rc < 0 || isx005_client == NULL) {
		rc = -ENOTSUPP;
		goto probe_done;
	}

	s->s_init = isx005_sensor_init;
	s->s_release = isx005_sensor_release;
	s->s_config  = isx005_sensor_config;

	rc = device_create_file(&isx005_pdev->dev, &dev_attr_pclk);
	if (rc < 0) {
		printk("device_create_file error!\n");
		return rc;
	}

	rc = device_create_file(&isx005_pdev->dev, &dev_attr_mclk);
	if (rc < 0) {
		printk("device_create_file error!\n");
		return rc;
	}

	rc = device_create_file(&isx005_pdev->dev, &dev_attr_always_on);
	if (rc < 0) {
		printk("device_create_file error!\n");
		return rc;
	}

probe_done:
	printk(KERN_ERR "%s %s:%d\n", __FILE__, __func__, __LINE__);
	return rc;
}

static int __isx005_probe(struct platform_device *pdev)
{
	isx005_pdev = pdev;
	return msm_camera_drv_start(pdev, isx005_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __isx005_probe,
	.driver = {
		.name = "msm_camera_isx005",
		.owner = THIS_MODULE,
	},
};

static int __init isx005_init(void)
{
	return platform_driver_register(&msm_camera_driver);
}

late_initcall(isx005_init);
