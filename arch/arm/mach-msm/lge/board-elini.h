/* arch/arm/mach-msm/include/mach/board_elini.h
 * Copyright (C) 2009 LGE, Inc.
 * Author: SungEun Kim <cleaneye@lge.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef __ARCH_MSM_BOARD_ELINI_H
#define __ARCH_MSM_BOARD_ELINI_H

#include <linux/types.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <asm/setup.h>
#include "pm.h"

/* LGE_S [ynj.kim@lge.com] 2010-05-21 : atcmd - virtual device */
#define KEY_SPEAKERMODE 241 // KEY_VIDEO_NEXT is not used in GED
#define KEY_CAMERAFOCUS 242 // KEY_VIDEO_PREV is not used in GED
#define KEY_FOLDER_HOME 243
#define KEY_FOLDER_MENU 244

#define ATCMD_VIRTUAL_KEYPAD_ROW	8
#define ATCMD_VIRTUAL_KEYPAD_COL	8
/* LGE_E [ynj.kim@lge.com] 2010-05-21 : atcmd - virtual device */

/* sdcard related macros */
#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
#define GPIO_SD_DETECT_N	49
#define GPIO_MMC_COVER_DETECT 77
#define VREG_SD_LEVEL       3000

#define GPIO_SD_DATA_3      51
#define GPIO_SD_DATA_2      52
#define GPIO_SD_DATA_1      53
#define GPIO_SD_DATA_0      54
#define GPIO_SD_CMD         55
#define GPIO_SD_CLK         56
#endif

/* touch-screen macros */
#define TS_X_MIN			0
#define TS_X_MAX			320
#define TS_Y_MIN			0
#define TS_Y_MAX			480
#define TS_GPIO_I2C_SDA		91
#define TS_GPIO_I2C_SCL		90
#define TS_GPIO_IRQ			92	
#define TS_I2C_SLAVE_ADDR	0x20

/* max17040 fuel gauge macros */
#define MAX17040_GPIO_I2C_SDA	85
#define MAX17040_GPIO_I2C_SCL	84
#define MAX17040_I2C_SLAVE_ADDR	(0x6C >> 1)

/* camera */
#define CAM_I2C_SLAVE_ADDR	            0X1A
#define GPIO_CAM_RESET		 		0		/* GPIO_0 */
#define GPIO_CAM_PWDN		 		1		/* GPIO_1 */
#define GPIO_CAM_MCLK				15		/* GPIO_15 */
#define ISX005_DEFAULT_CLOCK_RATE     27000000 //sm.kwon@lge.com  change 24MHz => 27MHz   24000000

/* aat1270 flash */
#define FLASH_EN			17
#define MOVIE_MODE_EN		23
#define FLASH_INHIBIT		32

/* hall ic macros */
#define GPIO_HALLIC_IRQ		18
#define PROHIBIT_TIME		1000	/* default 1 sec */


/* acceleration */
#define MOTION_GPIO_INT	 	39
#define MOTION_GPIO_I2C_SCL  	40
#define MOTION_GPIO_I2C_SDA  	41
#define MOTION_I2C_ADDRESS	0x09    /* slave address 7bit */

/* proximity sensor & ecompass */
#define PROX_COMPASS_GPIO_I2C_SCL	107
#define PROX_COMPASS_GPIO_I2C_SDA	108

#define PROX_GPIO_DOUT			109
#define ECOM_GPIO_DRDY			31

#define PROXI_I2C_ADDRESS		0x44 //0x55 /* slave address 7bit */
#define ECOM_I2C_ADDRESS		0x0F /* slave address 7bit */

/* msm pmic leds */
#define EL_EN_GPIO			57

// LGE_UPDATE_S  BCM4325 GPIO PIN Configuration
/* 4325 & 4329 	KT_Nam
   RESET, REGON이 하나의 핀으로 될 경우 주석처리....
   각각 독립적인 핀으로 된 경우 define 해야 한다.   */
#define BT_MODULE_SEPARATE_REG_RESET

/* lcd & backlight */
#define GPIO_LCD_BL_EN		82
#define GPIO_BL_I2C_SCL		88
#define GPIO_BL_I2C_SDA		89
#define GPIO_LCD_VSYNC_O	97
#define GPIO_LCD_MAKER_LOW	101
#define GPIO_LCD_RESET_N	102


/* bluetooth gpio pin */
enum {
#ifdef BT_MODULE_SEPARATE_REG_RESET
	BT_REGON		= 23,
#endif
	BT_WAKE         = 42,
	BT_RFR          = 43,
	BT_CTS          = 44,
	BT_RX           = 45,
	BT_TX           = 46,
	BT_PCM_DOUT     = 68,
	BT_PCM_DIN      = 69,
	BT_PCM_SYNC     = 70,
	BT_PCM_CLK      = 71,
	BT_HOST_WAKE    = 83,
#ifdef BT_MODULE_SEPARATE_REG_RESET
	BT_WLAN_RESET_N	= 93,
#endif
	BT_RESET_N			= 123,
};
// LGE_UPDATE_E  hongwon.lee@lge.com  07/05/10

/* pp2106 qwerty keypad macros */
#define KEY_SPEAKERMODE 241 // KEY_VIDEO_NEXT is not used in GED
#define KEY_CAMERAFOCUS 242 // KEY_VIDEO_PREV is not used in GED
#define KEY_FOLDER_HOME 243
#define KEY_FOLDER_MENU 244

#define PP2106_KEYPAD_ROW	8
#define PP2106_KEYPAD_COL	8

#define GPIO_PP2106_RESET	33
#define GPIO_PP2106_IRQ		36
#define GPIO_PP2106_SDA		34
#define GPIO_PP2106_SCL		35

/* carkit driver macros */
#define GPIO_CARKIT_DETECT	21//41

/* ear sense driver macros */
#define GPIO_EAR_SENSE		29
#define GPIO_HS_MIC_BIAS_EN	26

/* interface variable */
extern struct platform_device msm_device_snd;
extern struct platform_device msm_device_adspdec;
extern struct i2c_board_info i2c_devices[1];

/* interface functions */
void config_camera_on_gpios(void);
void config_camera_off_gpios(void);
struct device* elini_backlight_dev(void);

/*LGE_CHANGE_S,DCM_JP[heari.kim@lge.com,10/11/30,< > Elini]*/
void __init msm_init_timed_vibrator(void);
/*LGE_CHANGE_E,DCM_JP[heari.kim@lge.com,10/11/30,< > Elini]*/
#endif
