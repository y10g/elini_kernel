/* arch/arm/mach-msm/board-elini-input.c
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
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/gpio_event.h>
#include <linux/keyreset.h>
#include <mach/gpio.h>
#include <mach/vreg.h>
#include <mach/board.h>
#include <mach/board_lge.h>
#include <mach/rpc_server_handset.h>
//LGE_MODIFY_S kwangrim.ha@lge.com 2010.11.05 L04C kernel mode
#include <linux/kernel.h>
//#include <linux/syscalls.h>
//LGE_MODIFY_E kwangrim.ha@lge.com 2010.11.05 L04C kernel mode

#include "board-elini.h"

static int prox_power_set(unsigned char onoff);

static unsigned short pp2106_keycode[PP2106_KEYPAD_ROW][PP2106_KEYPAD_COL] = {
        {KEY_1,         KEY_8,          KEY_Q,          KEY_I,          KEY_D,          KEY_MENU,               KEY_B,          KEY_LEFT        },
        {KEY_2,         KEY_9,          KEY_W,          KEY_O,          KEY_F,          KEY_RIGHTSHIFT,         KEY_N,          KEY_RIGHT       },
        {KEY_3,         KEY_0,          KEY_E,          KEY_P,          KEY_G,          KEY_Z,                  KEY_M,          KEY_DOT/**/	},
        {KEY_4,         KEY_BACKSPACE,  KEY_R,          KEY_PROG1/**/,  KEY_H,          KEY_X,                  KEY_LEFTSHIFT,  KEY_MINUS/**/   },
        {KEY_5,         KEY_SEARCH,     KEY_T,          KEY_LEFTALT,    KEY_J,          KEY_C,                  KEY_PROG4/**/,  KEY_SPACE       },
        {KEY_6,         KEY_ENTER,      KEY_Y,          KEY_A,          KEY_K,          KEY_V,                  KEY_UP,         KEY_COMMA       },
        {KEY_7,         KEY_BACK,       KEY_U,          KEY_S,          KEY_L,          KEY_SPACE,              KEY_DOWN,       KEY_SEND        },
        {KEY_UNKNOWN,   KEY_HOME,       KEY_UNKNOWN,    KEY_UNKNOWN,    KEY_UNKNOWN,    KEY_UNKNOWN,            KEY_END,        KEY_HOME        },

};

/* LGE_S [ynj.kim@lge.com] 2010-05-15 : atcmd virtual device */
static unsigned short atcmd_virtual_keycode[ATCMD_VIRTUAL_KEYPAD_ROW][ATCMD_VIRTUAL_KEYPAD_COL] = {
	{KEY_1, 		KEY_8, 				KEY_Q,  	 KEY_I,          KEY_D,      	KEY_HOME,	KEY_B,          KEY_UP},
	{KEY_2, 		KEY_9, 		  		KEY_W,		 KEY_O,       	 KEY_F,		 	KEY_RIGHTSHIFT, 	KEY_N,			KEY_DOWN},
	{KEY_3, 		KEY_0, 		  		KEY_E,		 KEY_P,          KEY_G,      	KEY_Z,        	KEY_M, 			KEY_UNKNOWN},
	{KEY_4, 		KEY_BACK,  			KEY_R,		 KEY_SEARCH,     KEY_H,			KEY_X,    		KEY_LEFTSHIFT,	KEY_UNKNOWN},
	{KEY_5, 		KEY_BACKSPACE, 		KEY_T,		 KEY_LEFTALT,    KEY_J,      	KEY_C,     		KEY_REPLY,    KEY_CAMERA},
	{KEY_6, 		KEY_ENTER,  		KEY_Y,  	 KEY_A,		     KEY_K,			KEY_V,  	    KEY_RIGHT,     	KEY_CAMERAFOCUS},
	{KEY_7, 		KEY_MENU,	KEY_U,  	 KEY_S,    		 KEY_L, 	    KEY_SPACE,      KEY_LEFT,     	KEY_SEND},
	{KEY_UNKNOWN, 	KEY_UNKNOWN,  		KEY_UNKNOWN, KEY_UNKNOWN, 	 KEY_UNKNOWN,	KEY_UNKNOWN,    KEY_FOLDER_MENU,      	KEY_FOLDER_HOME},
		
};

static struct atcmd_virtual_platform_data atcmd_virtual_pdata = {
	.keypad_row = ATCMD_VIRTUAL_KEYPAD_ROW,
	.keypad_col = ATCMD_VIRTUAL_KEYPAD_COL,
	.keycode = (unsigned char *)atcmd_virtual_keycode,
};

static struct platform_device atcmd_virtual_device = {
	.name = "atcmd_virtual_kbd",
	.id = -1,
	.dev = {
		.platform_data = &atcmd_virtual_pdata,
	},
};
static struct pp2106_platform_data pp2106_pdata = {
	.keypad_row = PP2106_KEYPAD_ROW,
	.keypad_col = PP2106_KEYPAD_COL,
	.keycode = (unsigned char *)pp2106_keycode,
	.reset_pin = GPIO_PP2106_RESET,
	.irq_pin = GPIO_PP2106_IRQ,
	.sda_pin = GPIO_PP2106_SDA,
	.scl_pin = GPIO_PP2106_SCL,
};

static struct platform_device qwerty_device = {
	.name = "elini_keypad",
	.id = -1,
	.dev = {
		.platform_data = &pp2106_pdata,
         },
};
/* LGE_E [ynj.kim@lge.com] 2010-05-15 : atcmd virtual device */
#if defined (CONFIG_LGE_SUPPORT_AT_CMD)
struct input_dev* at_cmd_hall_ic;
EXPORT_SYMBOL(at_cmd_hall_ic);
#endif 

static struct gpio_event_direct_entry elini_slide_switch_map[] = {
        { GPIO_HALLIC_IRQ,              SW_LID              },
};

// LGE_MODIFY_S kwangrim.ha@lge.com 2010.12.22 L04C slide enable when usb connected
extern int msm_hsusb_detect_chg_type(void);
// LGE_MODIFY_E kwangrim.ha@lge.com 2010.11.05 L04C slide enable when usb connected

static int elini_gpio_slide_input_func(struct input_dev *input_dev,
                        struct gpio_event_info *info, void **data, int func)
{
        int mUsbOnline = 0;
#if defined (CONFIG_LGE_SUPPORT_AT_CMD)
        at_cmd_hall_ic = input_dev;
#endif /* add val for C710 AT_CMD */

        if (func == GPIO_EVENT_FUNC_INIT)
        {
// LGE_MODIFY_S kwangrim.ha@lge.com 2010.12.22 L04C slide enable when usb connected        
	    mUsbOnline = msm_hsusb_detect_chg_type();
	    if(mUsbOnline == 0 /* USB_CHARGER_TYPE_USB_PC */)
          {
                gpio_tlmm_config(GPIO_CFG(GPIO_HALLIC_IRQ, 0, GPIO_INPUT, GPIO_PULL_UP,
                                        GPIO_2MA), GPIO_ENABLE);
          }
// LGE_MODIFY_E kwangrim.ha@lge.com 2010.12.22 L04C slide enable when usb connected		
          else
          {
// LGE_MODIFY_S kwangrim.ha@lge.com 2010.11.05 L04C kernel mod
                gpio_tlmm_config(GPIO_CFG(GPIO_HALLIC_IRQ, 0, GPIO_INPUT, GPIO_PULL_UP,
                                        GPIO_2MA), GPIO_DISABLE);
// LGE_MODIFY_E kwangrim.ha@lge.com 2010.11.05 L04C kernel mode
          }
      	 }

        return gpio_event_input_func(input_dev, info, data, func);
}

static struct gpio_event_input_info elini_slide_switch_info = {
        .info.func = elini_gpio_slide_input_func,
        .debounce_time.tv64 = 0,
        .flags = 0,
        .type = EV_SW,
        .keymap = elini_slide_switch_map,
        .keymap_size = ARRAY_SIZE(elini_slide_switch_map)
};

static struct gpio_event_info *elini_gpio_slide_info[] = {
        &elini_slide_switch_info.info,
};

static struct gpio_event_platform_data elini_gpio_slide_data = {
        .name = "gpio-slide-detect",
        .info = elini_gpio_slide_info,
        .info_count = ARRAY_SIZE(elini_gpio_slide_info)
};

static struct platform_device elini_gpio_slide_device = {
        .name = GPIO_EVENT_DEV_NAME,
        .id = 0,
        .dev        = {
                .platform_data  = &elini_gpio_slide_data,
        },
};

/* head set device */
static struct msm_handset_platform_data hs_platform_data = {
	.hs_name = "7k_handset",
	.pwr_key_delay_ms = 500, /* 0 will disable end key */
};

static struct platform_device hs_device = {
	.name   = "msm-handset",
	.id     = -1,
	.dev    = {
		.platform_data = &hs_platform_data,
	},
};

/* gpio keypad device */
#define GPIO_VOL_UP		37
#define GPIO_VOL_DOWN	38

static struct gpio_event_direct_entry elini_keypad_switch_map[] = {
	{ GPIO_VOL_UP,       	KEY_VOLUMEUP   	    },
	{ GPIO_VOL_DOWN,       	KEY_VOLUMEDOWN	    },
};


static int elini_gpio_event_input_func(struct input_dev *input_dev,
			struct gpio_event_info *info, void **data, int func)
{
	int ret;

	if (func == GPIO_EVENT_FUNC_INIT) {
//		for (i = 0; i < ARRAY_SIZE(keypad_virtual_keys); i++)
//		{
//			set_bit(keypad_virtual_keys[i] & KEY_MAX,	input_dev->keybit);
//		}
		gpio_tlmm_config(GPIO_CFG(GPIO_VOL_UP, 0, GPIO_INPUT, GPIO_PULL_UP,
					GPIO_2MA), GPIO_ENABLE);
		gpio_tlmm_config(GPIO_CFG(GPIO_VOL_DOWN, 0, GPIO_INPUT, GPIO_PULL_UP,
					GPIO_2MA), GPIO_ENABLE);
		// add wakeup source vol up/down key  [younchan.kim@lge.com]
		enable_irq_wake(MSM_GPIO_TO_INT(GPIO_VOL_UP));
		enable_irq_wake(MSM_GPIO_TO_INT(GPIO_VOL_DOWN));
		
		ret = gpio_event_input_func(input_dev, info, data, func);
	
		return ret;
	}
/*	noting to do SUSPAND & RESUME [younchan.kim@lge.com]
	if (func == GPIO_EVENT_FUNC_SUSPEND) {
		disable_irq_wake(MSM_GPIO_TO_INT(GPIO_VOL_UP));
		disable_irq_wake(MSM_GPIO_TO_INT(GPIO_VOL_DOWN));
	}

	if (func == GPIO_EVENT_FUNC_RESUME) {
		gpio_tlmm_config(GPIO_CFG(GPIO_VOL_UP, 0, GPIO_INPUT, GPIO_PULL_UP,
					GPIO_2MA), GPIO_ENABLE);
		gpio_tlmm_config(GPIO_CFG(GPIO_VOL_DOWN, 0, GPIO_INPUT, GPIO_PULL_UP,
					GPIO_2MA), GPIO_ENABLE);
		
		enable_irq_wake(MSM_GPIO_TO_INT(GPIO_VOL_UP));
		enable_irq_wake(MSM_GPIO_TO_INT(GPIO_VOL_DOWN));
	}

	ret = gpio_event_input_func(input_dev, info, data, func);

	return ret;
*/
/* myungwon.kim 2010-10-22 : ret is Garbage Value so we make it just return 0 */
        return 0;
/* myungwon.kim */
}

static int elini_gpio_keypad_power(
		const struct gpio_event_platform_data *pdata, bool on)
{
	/* this is dummy function to make gpio_event driver register suspend function
	 * 2010-01-29, cleaneye.kim@lge.com
	 */

	return 0;
}

static struct gpio_event_input_info elini_keypad_switch_info = {
	.info.func = elini_gpio_event_input_func,
	.flags = 0,
	.type = EV_KEY,
	.keymap = elini_keypad_switch_map,
	.keymap_size = ARRAY_SIZE(elini_keypad_switch_map)
};

static struct gpio_event_info *elini_keypad_info[] = {
	&elini_keypad_switch_info.info,
};

static struct gpio_event_platform_data elini_keypad_data = {
	.name = "gpio-side-keypad",
	.info = elini_keypad_info,
	.info_count = ARRAY_SIZE(elini_keypad_info),
	.power = elini_gpio_keypad_power,
};

static struct platform_device elini_gpio_keypad_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 1,
	.dev        = {
		.platform_data  = &elini_keypad_data,
	},
};

/* keyreset platform device */
static int elini_reset_keys_up[] = {
	KEY_HOME,
	0
};

static struct keyreset_platform_data elini_reset_keys_pdata = {
	.keys_up = elini_reset_keys_up,
	.keys_down = {
		KEY_LEFTALT,
		KEY_LEFTSHIFT,
		KEY_DOT,
		0
	},
};

struct platform_device elini_reset_keys_device = {
	.name = KEYRESET_NAME,
	.dev.platform_data = &elini_reset_keys_pdata,
};

/* input platform device */
static struct platform_device *elini_input_devices[] __initdata = {
	&hs_device,
	&qwerty_device,
	&elini_gpio_keypad_device,
	&elini_gpio_slide_device,
	&atcmd_virtual_device,
/* myungwon.kim 1120 GPIO Interrupt Occur Kernel Panic*/
        //	&elini_reset_keys_device,
/* myungwon.kim 1120 GPIO Interrupt Occur Kernel Panic*/
};

/* MCS6000 Touch */
static struct gpio_i2c_pin ts_i2c_pin[] = {
	[0] = {
		.sda_pin	= TS_GPIO_I2C_SDA,
		.scl_pin	= TS_GPIO_I2C_SCL,
		.reset_pin	= 0,
		.irq_pin	= TS_GPIO_IRQ,
	},
};

static struct i2c_gpio_platform_data ts_i2c_pdata = {
	.sda_is_open_drain	= 0,
	.scl_is_open_drain	= 0,
	.udelay				= 2,
};

static struct platform_device ts_i2c_device = {
	.name	= "i2c-gpio",
	.dev.platform_data = &ts_i2c_pdata,
};

static int ts_power_save_on = 0;
static int ts_set_vreg(unsigned char onoff)
{
	struct vreg *vreg_touch;
	int rc;
	int flag_on = !!onoff;

	if (ts_power_save_on == flag_on)
		return;

	ts_power_save_on = flag_on;
	
//	printk("[Touch] %s() onoff:%d\n",__FUNCTION__, onoff);

#ifdef CONFIG_MACH_MSM7X27_ELINI
	vreg_touch = vreg_get(0, "synt");
#else
	vreg_touch = vreg_get(0, "gp3");
#endif

	if(IS_ERR(vreg_touch)) {
		printk("[Touch] vreg_get fail : touch\n");
		return -1;
	}
	
	if (onoff) {
		rc = vreg_set_level(vreg_touch, 3050);
		if (rc != 0) {
			printk("[Touch] vreg_set_level failed\n");
			return -1;
		}
		vreg_enable(vreg_touch);
	} else {
		vreg_set_level(vreg_touch, 0);
		vreg_disable(vreg_touch);
	}

	return 0;	
}

static struct touch_platform_data ts_pdata = {
	.ts_x_min = TS_X_MIN,
	.ts_x_max = TS_X_MAX,
	.ts_y_min = TS_Y_MIN,
	.ts_y_max = TS_Y_MAX,
	.power 	  = ts_set_vreg,
	.irq 	  = TS_GPIO_IRQ,
	.scl      = TS_GPIO_I2C_SCL,
	.sda      = TS_GPIO_I2C_SDA,
};

static struct i2c_board_info ts_i2c_bdinfo[] = {
	[0] = {
		I2C_BOARD_INFO("touch_mcs6000", TS_I2C_SLAVE_ADDR),
		.type = "touch_mcs6000",
		.platform_data = &ts_pdata,
	},
};

static void __init elini_init_i2c_touch(int bus_num)
{
	ts_i2c_device.id = bus_num;

	init_gpio_i2c_pin(&ts_i2c_pdata, ts_i2c_pin[0],	&ts_i2c_bdinfo[0]);
	i2c_register_board_info(bus_num, &ts_i2c_bdinfo[0], 1);
	platform_device_register(&ts_i2c_device);
}

/* acceleration */
static struct gpio_i2c_pin motion_i2c_pin[] = {
	[0] = {
		.sda_pin	= MOTION_GPIO_I2C_SDA,
		.scl_pin	= MOTION_GPIO_I2C_SCL,
		.reset_pin	= 0,		
		.irq_pin	= MOTION_GPIO_INT,
	},
};
 
static struct i2c_gpio_platform_data motion_i2c_pdata = {
	.sda_is_open_drain = 0,
	.scl_is_open_drain = 0,
	.udelay = 2,
};

static struct platform_device motion_i2c_device = {
	.name = "i2c-gpio",
	.dev.platform_data = &motion_i2c_pdata,
};

static int motion_power_set(unsigned char onoff)
{
	int ret = 0;
	return ret;
}
	
static struct acceleration_platform_data accel_pdata = {
	.power		= motion_power_set,
};

static struct i2c_board_info motion_i2c_bdinfo[] = {
	[0] = {
		I2C_BOARD_INFO("KR3DM", MOTION_I2C_ADDRESS),
		.type = "KR3DM",
		.platform_data = &accel_pdata,
	}
};

static void __init elini_init_i2c_motion(int bus_num)
{
	motion_i2c_device.id = bus_num;

	init_gpio_i2c_pin(&motion_i2c_pdata, motion_i2c_pin[0], &motion_i2c_bdinfo[0]);

	i2c_register_board_info(bus_num, &motion_i2c_bdinfo[0], 1);
	platform_device_register(&motion_i2c_device);
}

/* ecompass */
/*
static struct gpio_i2c_pin ecom_i2c_pin[] = {
	[0] = {
		.sda_pin	= ECOM_GPIO_I2C_SDA,
		.scl_pin	= ECOM_GPIO_I2C_SCL,
		.reset_pin	= ECOM_GPIO_RST,		
		.irq_pin	= ECOM_GPIO_INT,
	},
};

static struct i2c_gpio_platform_data ecom_i2c_pdata = {
	.sda_is_open_drain = 0,
	.scl_is_open_drain = 0,
	.udelay = 2,
};

static struct platform_device ecom_i2c_device = {
	.name = "i2c-gpio",
	.dev.platform_data = &ecom_i2c_pdata,
};
*/

static int ecom_prox_power_save_on = 0;
static int ecom_power_set(unsigned char onoff)
{
	struct vreg *vreg_power;
	int err;
	int flag_on = !!onoff;

	if (ecom_prox_power_save_on == flag_on)
		return;

	ecom_prox_power_save_on = flag_on;

#ifdef CONFIG_MACH_MSM7X27_ELINI
	vreg_power = vreg_get(0, "gp6");
#else
	vreg_power = vreg_get(0, "mmc");
#endif

	if (onoff) {
		vreg_enable(vreg_power);

		err = vreg_set_level(vreg_power, 2600);
		if (err != 0) {
			printk("vreg_compass failed.\n");
			return -1;
		}
	} 
	else {
	//LGE_CHANGE_S euikyeom.kim@lge.com
//		vreg_set_level(vreg_power, 0);
//		vreg_disable(vreg_power);
	//LGE_CHANGE_E euikyeom.kim@lge.com
	}

	return 0;
}

static struct ecom_platform_data ecom_pdata = {
	.pin_int        	= ECOM_GPIO_DRDY,
	.pin_rst		= 0,
	.power          	= ecom_power_set,
};

/*
static struct i2c_board_info ecom_i2c_bdinfo[] = {
	[0] = {
		I2C_BOARD_INFO("akm8973", ECOM_I2C_ADDRESS),
		.type = "akm8973",
		.platform_data = &ecom_pdata,
	}
};

static void __init elini_init_i2c_ecompass(int bus_num)
{
	ecom_i2c_device.id = bus_num;

	init_gpio_i2c_pin(&ecom_i2c_pdata, ecom_i2c_pin[0], &ecom_i2c_bdinfo[0]);

	i2c_register_board_info(bus_num, &ecom_i2c_bdinfo[0], 1);
	platform_device_register(&ecom_i2c_device);
}
*/

/* gp2ap proximity sensor */
/*
static struct gpio_i2c_pin proxi_i2c_pin[] = {
	[0] = {
		.sda_pin	= PROXI_GPIO_I2C_SDA,
		.scl_pin	= PROXI_GPIO_I2C_SCL,
		.reset_pin	= 0,		
		.irq_pin	= PROXI_GPIO_DOUT,
	},
};

static struct i2c_gpio_platform_data proxi_i2c_pdata = {
	.sda_is_open_drain = 0,
	.scl_is_open_drain = 0,
	.udelay = 2,
};

static struct platform_device proxi_i2c_device = {
	.name = "i2c-gpio",
	.dev.platform_data = &proxi_i2c_pdata,
};
*/

//static int prox_power_save_on = 0;
static int prox_power_set(unsigned char onoff)
{
	int ret = 0;
	int flag_on = !!onoff;
#ifdef CONFIG_MACH_MSM7X27_ELINI
	struct vreg *vreg = vreg_get(0, "gp6");
#else
	struct vreg *vreg = vreg_get(0, "mmc");
#endif

	if (ecom_prox_power_save_on == flag_on)
		return;

	ecom_prox_power_save_on = flag_on;	

	if (onoff) {
		vreg_set_level(vreg, 2600);
		vreg_enable(vreg);
	} else {
	//LGE_CHANGE_S euikyeom.kim@lge.com
//		vreg_set_level(vreg, 0);
//		vreg_disable(vreg);
	//LGE_CHANGE_E euikyeom.kim@lge.com
	}

	return ret;
}

static struct proximity_platform_data proxi_pdata = {
	.irq_num	= PROX_GPIO_DOUT,
	.power		= prox_power_set,
	.methods		= 0,
	.operation_mode		= 0,
	.debounce	 = 0,
	.cycle = 2,
};

static struct i2c_board_info prox_ecom_i2c_bdinfo[] = {
	[0] = {
		I2C_BOARD_INFO("proximity_gp2ap", PROXI_I2C_ADDRESS),
		.type = "proximity_gp2ap",
		.platform_data = &proxi_pdata,
	},
	[1] = {
		I2C_BOARD_INFO("ami304_sensor", ECOM_I2C_ADDRESS),
		.type = "ami304_sensor",
		.platform_data = &ecom_pdata,
	},
};

static struct gpio_i2c_pin proxi_ecom_i2c_pin[] = {
	[0] = {
		.sda_pin	= PROX_COMPASS_GPIO_I2C_SDA,
		.scl_pin	= PROX_COMPASS_GPIO_I2C_SCL,
		.reset_pin	= 0,		
		.irq_pin	= PROX_GPIO_DOUT,
	},
	[1] = {
		.sda_pin	= PROX_COMPASS_GPIO_I2C_SDA,
		.scl_pin	= PROX_COMPASS_GPIO_I2C_SCL,
		.reset_pin	= 0,		
		.irq_pin	= ECOM_GPIO_DRDY,
	},
};

static struct i2c_gpio_platform_data proxi_ecom_i2c_pdata = {
	.sda_is_open_drain = 0,
	.scl_is_open_drain = 0,
	.udelay = 2,
};

static struct platform_device proxi_ecom_i2c_device = {
        .name = "i2c-gpio",
        .dev.platform_data = &proxi_ecom_i2c_pdata,
};

static void __init elini_init_i2c_prox_ecom(int bus_num)
{
	proxi_ecom_i2c_device.id = bus_num;

	init_gpio_i2c_pin(&proxi_ecom_i2c_pdata, proxi_ecom_i2c_pin[0], &prox_ecom_i2c_bdinfo[0]);
	init_gpio_i2c_pin(&proxi_ecom_i2c_pdata, proxi_ecom_i2c_pin[1], &prox_ecom_i2c_bdinfo[1]);

	i2c_register_board_info(bus_num, &prox_ecom_i2c_bdinfo[0], 2);
	platform_device_register(&proxi_ecom_i2c_device);
}

/* common function */
void __init lge_add_input_devices(void)
{
	platform_add_devices(elini_input_devices, ARRAY_SIZE(elini_input_devices));

	lge_add_gpio_i2c_device(elini_init_i2c_touch);
	lge_add_gpio_i2c_device(elini_init_i2c_motion);
	lge_add_gpio_i2c_device(elini_init_i2c_prox_ecom);
}

//LGE_MODIFY_S kwangrim.ha@lge.com 2010.11.05 L04C kernel mode
asmlinkage int lge_hallic_gpio_enable(void)
{
  gpio_tlmm_config(GPIO_CFG(GPIO_HALLIC_IRQ, 0, GPIO_INPUT, GPIO_PULL_UP,
                                        GPIO_2MA), GPIO_ENABLE);

  return 1;
}
//LGE_MODIFY_E kwangrim.ha@lge.com 2010.11.05 L04C kernel mode
