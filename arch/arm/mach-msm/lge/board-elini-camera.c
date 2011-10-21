/* arch/arm/mach-msm/board-elini-camera.c
 * Copyright (C) 2009 LGE, Inc.
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
#include <linux/types.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <asm/setup.h>
#include <mach/gpio.h>
#include <mach/board.h>
#include <mach/camera.h>
#include <mach/msm_iomap.h>
#include <mach/vreg.h>
#include <mach/board_lge.h>
#include "board-elini.h"

//BEGIN[[, 20100723, sm.kwon@lge.com, Change senrsor from ISX006 to ISX005
//extern struct device* elini_backlight_dev(void);
int mclk_rate = ISX005_DEFAULT_CLOCK_RATE;

struct i2c_board_info i2c_devices[1] = {
	{
		I2C_BOARD_INFO("isx005", CAM_I2C_SLAVE_ADDR),
	},
};
//]]END, 20100723, sm.kwon@lge.com

#if defined (CONFIG_MSM_CAMERA)
static uint32_t camera_off_gpio_table[] = {
	/* parallel CAMERA interfaces */
	GPIO_CFG(4,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT0 */
	GPIO_CFG(5,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT1 */
	GPIO_CFG(6,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT2 */
	GPIO_CFG(7,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT3 */
	GPIO_CFG(8,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT4 */
	GPIO_CFG(9,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT5 */
	GPIO_CFG(10, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT6 */
	GPIO_CFG(11, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT7 */
	GPIO_CFG(12, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* PCLK */
	GPIO_CFG(13, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* HSYNC_IN */
	GPIO_CFG(14, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* VSYNC_IN */
	GPIO_CFG(GPIO_CAM_MCLK, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), /* MCLK */
};

static uint32_t camera_on_gpio_table[] = {
	/* parallel CAMERA interfaces */
	GPIO_CFG(4,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT0 */
	GPIO_CFG(5,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT1 */
	GPIO_CFG(6,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT2 */
	GPIO_CFG(7,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT3 */
	GPIO_CFG(8,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT4 */
	GPIO_CFG(9,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT5 */
	GPIO_CFG(10, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT6 */
	GPIO_CFG(11, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT7 */
	GPIO_CFG(12, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_16MA), /* PCLK */
	GPIO_CFG(13, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* HSYNC_IN */
	GPIO_CFG(14, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* VSYNC_IN */
	GPIO_CFG(GPIO_CAM_MCLK, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA), /* MCLK */
};

static void config_gpio_table(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_ENABLE);
		if (rc) {
			printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}

void config_camera_on_gpios(void)
{
	config_gpio_table(camera_on_gpio_table,
		ARRAY_SIZE(camera_on_gpio_table));
}

void config_camera_off_gpios(void)
{
	config_gpio_table(camera_off_gpio_table,
		ARRAY_SIZE(camera_off_gpio_table));
}

int camera_power_on (void)
{
	int rc;
	struct device *dev = elini_backlight_dev();
  
	/* clear RESET, PWDN to Low*/
	gpio_set_value(GPIO_CAM_RESET, 0);
	gpio_set_value(GPIO_CAM_PWDN, 0);
//BEGIN[[, 20100723, sm.kwon@lge.com, Change senrsor from ISX006 to ISX005

	/*AVDD power 1.2V*/
	rc = aat28xx_ldo_set_level(dev, 3, 1200);
	if (rc < 0) {
		printk(KERN_ERR "%s: ldo %d set level error\n", __func__, 3);
		return rc;
	}
	rc = aat28xx_ldo_enable(dev, 3, 1);
	if (rc < 0) {
		printk(KERN_ERR "%s: ldo %d control error\n", __func__, 3);
		return rc;
	}

	/*IOVDD power  2.7V*/
	if(lge_bd_rev < LGE_REV_10) // Rev1.0 IOVDD Power change, 2010-11-18
	{
		rc = aat28xx_ldo_set_level(dev, 4, 2600);
	}
	else
	{
		rc = aat28xx_ldo_set_level(dev, 4, 1800);
	}
	
	if (rc < 0) {
		printk(KERN_ERR "%s: ldo %d set level error\n", __func__, 4);
		return rc;
	}
	rc = aat28xx_ldo_enable(dev, 4, 1);
	if (rc < 0) {
		printk(KERN_ERR "%s: ldo %d control error\n", __func__, 4);
		return rc;
	}	
  
  /*DVDD power 2.7V*/
	rc = aat28xx_ldo_set_level(dev, 2, 2700);
	if (rc < 0) {
		printk(KERN_ERR "%s: ldo %d set level error\n", __func__, 2);
		return rc;
	}
	rc = aat28xx_ldo_enable(dev, 2, 1);
	if (rc < 0) {
		printk(KERN_ERR "%s: ldo %d control error\n", __func__, 2);
		return rc;
	}

  /* AF power 2.8V */
	rc = aat28xx_ldo_set_level(dev, 1, 2800);
	if (rc < 0) {
		printk(KERN_ERR "%s: ldo %d set level error\n", __func__, 1);
		return rc;
	}
	rc = aat28xx_ldo_enable(dev, 1, 1);
	if (rc < 0) {
		printk(KERN_ERR "%s: ldo %d control error\n", __func__, 1);
		return rc;
	}


//]]END, 20100723, sm.kwon@lge.com
	mdelay(5);
	/*M Clock -24Mhz*/
	msm_camio_clk_rate_set(mclk_rate);
	mdelay(5);
	msm_camio_camif_pad_reg_reset();
	mdelay(5);

	/*reset high*/
	gpio_set_value(GPIO_CAM_RESET, 1);

	mdelay(5); 
	/*Nstandby high*/
	gpio_set_value(GPIO_CAM_PWDN, 1);
	
	mdelay(8);  // T2 


	return rc;

}

int camera_power_off (void)
{
	int rc = 0;
	struct device *dev = elini_backlight_dev();

	/*Nstandby low*/
	gpio_set_value(GPIO_CAM_PWDN, 0);
	mdelay(5);

	/*reset low*/
	gpio_set_value(GPIO_CAM_RESET, 0);
//BEGIN[[, 20100723, sm.kwon@lge.com, Change senrsor from ISX006 to ISX005

	/*AF power 2.8V*/
	rc = aat28xx_ldo_set_level(dev, 1, 0);
	if (rc < 0) {
		printk(KERN_ERR "%s: ldo %d set level error\n", __func__, 1);
		return rc;
	}
	rc = aat28xx_ldo_enable(dev, 1, 0);
	if (rc < 0) {
		printk(KERN_ERR "%s: ldo %d control error\n", __func__, 1);
		return rc;
	}

	/*DVDD power 2.7V*/
	rc = aat28xx_ldo_set_level(dev, 2, 0);
	if (rc < 0) {
		printk(KERN_ERR "%s: ldo %d set level error\n", __func__, 2);
		return rc;
	}
	rc = aat28xx_ldo_enable(dev, 2, 0);
	if (rc < 0) {
		printk(KERN_ERR "%s: ldo %d control error\n", __func__, 2);
		return rc;
	}

  /* OVDD power 2.6V*/
	rc = aat28xx_ldo_set_level(dev, 4, 0);
	if (rc < 0) {
		printk(KERN_ERR "%s: ldo %d set level error\n", __func__, 4);
		return rc;
	}
	rc = aat28xx_ldo_enable(dev, 4, 0);
	if (rc < 0) {
		printk(KERN_ERR "%s: ldo %d control error\n", __func__, 4);
		return rc;
	}


	/*AVDD power 1.2V*/
	rc = aat28xx_ldo_set_level(dev, 3, 0);
	if (rc < 0) {
		printk(KERN_ERR "%s: ldo %d set level error\n", __func__, 3);
		return rc;
	}
	rc = aat28xx_ldo_enable(dev, 3, 0);
	if (rc < 0) {
		printk(KERN_ERR "%s: ldo %d control error\n", __func__, 3);
		return rc;
	}
//]]END, 20100723, sm.kwon@lge.com

	return rc;

}

static struct msm_camera_device_platform_data msm_camera_device_data = {
	.camera_gpio_on   = config_camera_on_gpios,
	.camera_gpio_off  = config_camera_off_gpios,
	.ioext.mdcphy     = MSM_MDC_PHYS,
	.ioext.mdcsz      = MSM_MDC_SIZE,
	.ioext.appphy     = MSM_CLK_CTL_PHYS,
	.ioext.appsz      = MSM_CLK_CTL_SIZE,
	.camera_power_on  = camera_power_on,
	.camera_power_off = camera_power_off,
};

//BEGIN[[, 20100723, sm.kwon@lge.com, Change senrsor from ISX006 to ISX005
#if defined (CONFIG_ISX005)
static struct msm_camera_sensor_flash_data flash_none = {
	.flash_type = MSM_CAMERA_FLASH_NONE,
};

static struct msm_camera_sensor_info msm_camera_sensor_isx005_data = {
	.sensor_name    = "isx005",
	.sensor_reset   = GPIO_CAM_RESET,
	.sensor_pwd     = GPIO_CAM_PWDN,
	.vcm_pwd        = 0,
	.vcm_enable		  = 0,
	.pdata          = &msm_camera_device_data,
	.flash_data		  = &flash_none,
};

struct platform_device msm_camera_sensor_isx005 = {
	.name      = "msm_camera_isx005",
	.dev       = {
		.platform_data = &msm_camera_sensor_isx005_data,
	},
};
#endif/*CONFIG_ISX005*/
#endif/*CONFIG_MSM_CAMERA*/

static struct platform_device *elini_camera_devices[] __initdata = {
	&msm_camera_sensor_isx005,	
};
//]]END, 20100723, sm.kwon@lge.com

void __init lge_add_camera_devices(void)
{
	platform_add_devices(elini_camera_devices, ARRAY_SIZE(elini_camera_devices));
}
