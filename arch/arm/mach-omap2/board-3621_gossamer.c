/*
 *
 * Copyright (C) 2008 Texas Instruments Inc.
 * Vikram Pandita <vikram.pandita@ti.com>
 *
 * Modified from mach-omap2/board-ldp.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/workqueue.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/zforce.h>
#include <linux/leds.h>
#include <linux/mmc/host.h>

#include "timer-gp.h"
#include "omap_ion.h"


#if defined(CONFIG_TOUCHSCREEN_ZFORCE) || defined(CONFIG_TOUCHSCREEN_ZFORCE_MODULE)
#include <linux/zforce.h>
#define ZFORCE_I2C_SLAVE_ADDRESS   0x50
#define ZFORCE_GPIO_FOR_IRQ        113
#endif

#ifdef CONFIG_MACH_OMAP3621_GOSSAMER_EVT1C
#include <linux/i2c/pmic-tps65185-i2c.h>
#endif

#define GOSSAMER_CHARGE_ENAB_GPIO 44
#define GOSSAMER_CHARGE_ILM1_GPIO 45
#define GOSSAMER_CHARGE_ILM2_GPIO 61

#define GOSSAMER_PRE1C_CHARGE_ENAB_GPIO 110
#define GOSSAMER_PRE1C_CHARGE_ILM1_GPIO 102
#define GOSSAMER_PRE1C_CHARGE_ILM2_GPIO 61


#define GOSSAMER_DEBUG_LED_GPIO   90

#define GOSSAMER_WIFI_PMENA_GPIO	22
#define GOSSAMER_WIFI_IRQ_GPIO		15
#define GOSSAMER_WIFI_EN_POW		16

/*there probably is no bt uart wired up at all */
#define WILINK_UART_DEV_NAME            "/dev/ttyO1"

#include <linux/spi/spi.h>
#include <linux/i2c/twl.h>
#include <linux/interrupt.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/bq24073.h>
#include <linux/switch.h>
#include <linux/dma-mapping.h>

#include <linux/skbuff.h>
#include <linux/ti_wilink_st.h>
#include <linux/wl12xx.h>

#include <plat/hardware.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board-boxer.h>
#include <plat/mcspi.h>
#include <plat/gpio.h>
#include <plat/board.h>
#include <plat/common.h>
#include <plat/gpmc.h>
#include <plat/mmc.h>
#include "hsmmc.h"

#include <plat/usb.h>
#include <plat/mux.h>

#include <asm/system.h> // For system_serial_high & system_serial_low
#include <asm/io.h>
#include <asm/delay.h>
#include "control.h"
#include <plat/sram.h>

#include <video/omapdss.h>
#include <linux/i2c/twl.h>

//#include <linux/usb/android.h>

//#include "mmc-twl4030.h"
#include "omap3-opp.h"
#include "prcm-common.h"
#include "prm.h"

#include "sdram-samsung-k4x2g323pd.h"

#include <media/v4l2-int-device.h>

#include "omap_ram_console.h"

#ifdef CONFIG_PM
//#include <../drivers/media/video/omap/omap_voutdef.h>
#endif

#ifndef CONFIG_TWL4030_CORE
#error "no power companion board defined!"
#endif

#ifdef CONFIG_WL127X_RFKILL
#include <linux/wl127x-rfkill.h>
#endif

#ifdef CONFIG_ANDROID_RAM_CONSOLE
#include <linux/bootmem.h>
#endif

#ifdef CONFIG_LEDS_AS3676
#include <linux/leds-as3676.h>
#endif
#define DEFAULT_BACKLIGHT_BRIGHTNESS 105

#define CONFIG_DISABLE_HFCLK 1
#define ENABLE_VAUX1_DEDICATED	0x03
#define ENABLE_VAUX1_DEV_GRP	0x20

#define ENABLE_VAUX3_DEDICATED  0x03
#define ENABLE_VAUX3_DEV_GRP	0x20
#define TWL4030_MSECURE_GPIO	22

#define WL127X_BTEN_GPIO	60

#define GOSSAMER_EXT_QUART_PHYS	0x48000000
#define GOSSAMER_EXT_QUART_VIRT	0xfa000000
#define GOSSAMER_EXT_QUART_SIZE	SZ_256

/* Define for Gossamer keyboard: */
static int gossamer_twl4030_keymap[] = {
#ifdef CONFIG_MACH_OMAP3621_GOSSAMER_EVT1C
	KEY(0, 0, KEY_NEXT),
	KEY(1, 0, KEY_BACK),
	KEY(2, 0, KEY_PREVIOUS),
	KEY(3, 0, KEY_MENU),
///external build key mapping - KEY(0, 0, KEY_MENU),KEY(0, 1, KEY_BACK),KEY(0, 2, KEY_VOLUMEUP),KEY(0, 3, KEY_VOLUMEDOWN),
#else
	KEY(0, 0, KEY_HOME),
	KEY(1, 0, KEY_MENU),
	KEY(0, 1, KEY_BACK),
	KEY(2, 2, KEY_VOLUMEUP),
	KEY(3, 2, KEY_VOLUMEDOWN),
#endif /* CONFIG_MACH_OMAP3621_GOSSAMER_EVT1C */
};

#ifdef CONFIG_LEDS_AS3676
static struct as3676_platform_data as3676_pdata = {
	.pwronkey_shutdown_msec = 6500, /* long-press to shutdown in 6.5s */
	.step_up_frequ = 1,	/* 1 = 500 kHz */
	.step_up_vtuning = 0x0E,       /* 0x0E = 14uA on DCDC_FB */
	.leds[0] = {
		.name = "lcd-backlight",
		.on_charge_pump = 0,
		.max_current_uA = 10000,
	},
	.leds[1] = {
		.name = "lcd-backlight2",
		.on_charge_pump = 0,
		.max_current_uA = 10000,
	}
};
#endif

static struct matrix_keymap_data board_map_data = {
	.keymap			= gossamer_twl4030_keymap,
	.keymap_size		= ARRAY_SIZE(gossamer_twl4030_keymap),
};

static struct twl4030_keypad_data gossamer_kp_twl4030_data = {
	.keymap_data	= &board_map_data,
	.rows		= 8,
	.cols		= 8,
	.rep		= 0, /* Need to detect long key presses */
};

#ifdef CONFIG_MACH_OMAP3621_GOSSAMER_EVT1C
static struct gpio_keys_button gossamer_gpio_buttons[] = {
	{
		.code			= KEY_POWER,	
		.gpio			= 14,
		.desc			= "POWER",
		.active_low		= 0,
		.wakeup			= 1,
	},
	{
		.code			= KEY_HOME,
		.gpio			= 48,
		.desc			= "key-home",
		.active_low		= 1,
		.wakeup			= 1,
	},
};


static struct gpio_led debug_leds[] = {
	{ /* LED90 */
		.name			= "led90",
		.gpio			= GOSSAMER_DEBUG_LED_GPIO,
		.active_low		= 0,
		.default_trigger	= "none",
	}
};

static struct gpio_keys_platform_data gossamer_gpio_key_info = {
	.buttons	= gossamer_gpio_buttons,
	.nbuttons	= ARRAY_SIZE(gossamer_gpio_buttons),
//	.rep		= 1,		/* auto-repeat */
};

static struct platform_device gossamer_keys_gpio = {
	.name	= "gpio-keys",
	.id	= -1,
	.dev	= {
		.platform_data	= &gossamer_gpio_key_info,
	},
};
#endif /* CONFIG_MACH_OMAP3621_GOSSAMER_EVT1C */

#if defined(CONFIG_REGULATOR_BQ24073) || defined(CONFIG_REGULATOR_BQ24073_MODULE)
static struct bq24073_mach_info bq24073_init_dev_data = {
	.gpio_nce = GOSSAMER_CHARGE_ENAB_GPIO,
	.gpio_en1 = GOSSAMER_CHARGE_ILM1_GPIO,
	.gpio_en2 = GOSSAMER_CHARGE_ILM2_GPIO,
	.gpio_nce_state = 1,
	.gpio_en1_state = 0,
	.gpio_en2_state = 0,
};

static struct regulator_consumer_supply bq24073_vcharge_supply = {
       .supply         = "bq24073",
};

static struct regulator_init_data bq24073_init  = {

       .constraints = {
               .min_uV                 = 0,
               .max_uV                 = 5000000,
               .min_uA                 = 0,
               .max_uA                 = 1500000,
               .valid_modes_mask       = REGULATOR_MODE_NORMAL
                                       | REGULATOR_MODE_STANDBY,
               .valid_ops_mask         = REGULATOR_CHANGE_CURRENT
                                       | REGULATOR_CHANGE_MODE
                                       | REGULATOR_CHANGE_STATUS,
               .boot_on                = 0,
               .always_on              = 0,
	       .state_mem = {
		       .enabled = 1,
	       },
       },
       .num_consumer_supplies  = 1,
       .consumer_supplies      = &bq24073_vcharge_supply,

       .driver_data = &bq24073_init_dev_data,
};

/* GPIOS need to be in order of BQ24073 */
static struct platform_device gossamer_curr_regulator_device = {
	.name           = "bq24073", /* named after init manager for ST */
	.id             = -1,
	.dev 		= {
		.platform_data = &bq24073_init,
	},
};
#endif /* CONFIG_REGULATOR_BQ24073 */

/* Use address that is most likely unused and untouched by u-boot */
#define GOSSAMER_RAM_CONSOLE_START 0x8e000000
#define GOSSAMER_RAM_CONSOLE_SIZE (0x20000)

static int plat_kim_suspend(struct platform_device *pdev, pm_message_t state)
{
	/* TODO: wait for HCI-LL sleep */
	return 0;
}
static int plat_kim_resume(struct platform_device *pdev)
{
	return 0;
}

/* wl127x BT, FM, GPS connectivity chip */
struct ti_st_plat_data wilink_pdata = {
	.nshutdown_gpio = 60,
	.dev_name = WILINK_UART_DEV_NAME,
	.flow_cntrl = 1,
	.baud_rate = 3000000,
	.suspend = plat_kim_suspend,
	.resume = plat_kim_resume,
};
static struct platform_device wl127x_device = {
	.name           = "kim",
	.id             = -1,
	.dev.platform_data = &wilink_pdata,
};
static struct platform_device btwilink_device = {
	.name = "btwilink",
	.id = -1,
};

static struct platform_device *gossamer_devices[] __initdata = {
	&wl127x_device,
	&btwilink_device,
#ifdef CONFIG_WL127X_RFKILL
//	&gossamer_wl127x_device,
#endif
#ifdef CONFIG_MACH_OMAP3621_GOSSAMER_EVT1C
	&gossamer_keys_gpio,
#endif /* CONFIG_MACH_OMAP3621_GOSSAMER_EVT1C */
	//&gossamer_vout_device,
#if defined(CONFIG_REGULATOR_BQ24073) || defined(CONFIG_REGULATOR_BQ24073_MODULE)
	&gossamer_curr_regulator_device,
#endif /* CONFIG_REGULATOR_BQ24073 */
};

static struct regulator_consumer_supply gossamer_vmmc1_supply = {
	.supply		= "vmmc",
};

static struct regulator_consumer_supply gossamer_vsim_supply = {
	.supply		= "vmmc_aux",
};

static struct regulator_consumer_supply gossamer_vmmc2_supply = {
	.supply		= "vmmc",
};

/* VMMC1 for OMAP VDD_MMC1 (i/o) and MMC1 card */
static struct regulator_init_data gossamer_vmmc1 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 3150000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &gossamer_vmmc1_supply,
};
#if 1
/* VMMC2 for MMC2 card */
static struct regulator_init_data gossamer_vmmc2 = {
	.constraints = {
		.min_uV			= 1850000,
		/* This is a virtual regulator on TPS65921, so we can
		 * write anything that will please regulator FW.
		 * Apparently ROM code sometimes leaves VMMC2=3.15V,
		 * yet we restrict it to 1.8V. This causes the regulator
		 * framework to reject all updates, and more importantly,
		 * to return errors for all voltage orations.
		 */
		.max_uV			= 3150000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &gossamer_vmmc2_supply,
};
#endif

/* VSIM for OMAP VDD_MMC1A (i/o for DAT4..DAT7) */
static struct regulator_init_data gossamer_vsim = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 3000000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &gossamer_vsim_supply,
};

static struct fixed_voltage_config gossamer_vmmc2_fixed_config = {
	.supply_name		= "vmmc2",
	.microvolts		= 1850000,
	.gpio			= -EINVAL,
	.enabled_at_boot	= 1,
	.init_data		= &gossamer_vmmc2,
};

static struct platform_device gossamer_vmmc2_fixed_device = {
	.name		= "reg-fixed-voltage",
	.id		= 2,
	.dev = {
		.platform_data	= &gossamer_vmmc2_fixed_config,
	},
};

/* The order is reverted in this table so that internal eMMC is presented
 * as first mmc card for compatibility with existing android installations */
static struct omap2_hsmmc_info mmc[] = {
	{
		.name		= "internal",
		.mmc		= 2,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.nonremovable	= true,
		.power_saving	= true,
	},
	{
		.name		= "external",
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_wp	= -EINVAL,
		.power_saving	= true,
	},
	{
		.name		= "wl1271",
		.mmc		= 3,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_POWER_OFF_CARD,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.ocr_mask	= MMC_VDD_165_195,
		.nonremovable	= true,
	},

	{}      /* Terminator */
};

static void gossamer_wifi_init(void);

static int __ref gossamer_twl_gpio_setup(struct device *dev,
		unsigned gpio, unsigned ngpio)
{
	/* gpio + 0 is "mmc0_cd" (input/IRQ),
	 * gpio + 1 is "mmc1_cd" (input/IRQ)
	 */
	mmc[1].gpio_cd = gpio + 0;
	mmc[0].gpio_cd = gpio + 1;
	omap2_hsmmc_init(mmc);

	/* link regulators to MMC adapters ... we "know" the
	 * regulators will be set up only *after* we return.
	*/
	gossamer_vmmc1_supply.dev = mmc[1].dev;
	gossamer_vsim_supply.dev = mmc[1].dev;
	gossamer_vmmc2_supply.dev = mmc[0].dev;

	/*we call this here because it relies on mmc already being setup. */
	gossamer_wifi_init();

	return 0;
}

static struct twl4030_usb_data gossamer_usb_data = {
      .usb_mode	= T2_USB_MODE_ULPI,
#if defined(CONFIG_REGULATOR_BQ24073) || defined(CONFIG_REGULATOR_BQ24073_MODULE)
      .bci_supply     = &bq24073_vcharge_supply,
#endif /* CONFIG_REGULATOR_BQ24073 */
};
static struct twl4030_gpio_platform_data gossamer_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	.setup		= gossamer_twl_gpio_setup,
};

static struct twl4030_madc_platform_data gossamer_madc_data = {
	.irq_line	= 1,
};

/*
 * Sequence to control the TRITON Power resources,
 * when the system goes into sleep.
 * Executed upon P1_P2/P3 transition for sleep.
 */
static struct twl4030_ins __initdata sleep_on_seq[] = {
	/* Broadcast message to put res to sleep */
	{MSG_BROADCAST(DEV_GRP_ALL, RES_GRP_ALL, RES_TYPE_ALL, RES_TYPE2_R1,
							RES_STATE_SLEEP), 2},
	{MSG_BROADCAST(DEV_GRP_ALL, RES_GRP_ALL, RES_TYPE_ALL, RES_TYPE2_R2,
							RES_STATE_SLEEP), 2},
};

static struct twl4030_script sleep_on_script __initdata = {
	.script	= sleep_on_seq,
	.size	= ARRAY_SIZE(sleep_on_seq),
	.flags	= TWL4030_SLEEP_SCRIPT,
};

/*
 * Sequence to control the TRITON Power resources,
 * when the system wakeup from sleep.
 * Executed upon P1_P2 transition for wakeup.
 */
static struct twl4030_ins wakeup_p12_seq[] __initdata = {
	/* Broadcast message to put res to active */
	{MSG_BROADCAST(DEV_GRP_ALL, RES_GRP_ALL, RES_TYPE_ALL, RES_TYPE2_R1,
							RES_STATE_ACTIVE), 2},
};

static struct twl4030_script wakeup_p12_script __initdata = {
	.script	= wakeup_p12_seq,
	.size	= ARRAY_SIZE(wakeup_p12_seq),
	.flags	= TWL4030_WAKEUP12_SCRIPT,
};

/*
 * Sequence to control the TRITON Power resources,
 * when the system wakeup from sleep.
 * Executed upon P3 transition for wakeup.
 */
static struct twl4030_ins wakeup_p3_seq[] __initdata = {
	/* Broadcast message to put res to active */
	{MSG_BROADCAST(DEV_GRP_ALL, RES_GRP_ALL, RES_TYPE_ALL, RES_TYPE2_R2,
							RES_STATE_ACTIVE), 2},
};

static struct twl4030_script wakeup_p3_script __initdata = {
	.script = wakeup_p3_seq,
	.size   = ARRAY_SIZE(wakeup_p3_seq),
	.flags  = TWL4030_WAKEUP3_SCRIPT,
};

/*
 * Sequence to reset the TRITON Power resources,
 * when the system gets warm reset.
 * Executed upon warm reset signal.
 */
static struct twl4030_ins wrst_seq[] __initdata = {
/*
 * Reset twl4030.
 * Reset Main_Ref.
 * Reset All type2_group2.
 * Reset VUSB_3v1.
 * Reset All type2_group1.
 * Reset RC.
 * Reenable twl4030.
 */
	{MSG_SINGULAR(DEV_GRP_NULL, RES_RESET, RES_STATE_OFF), 2},
	{MSG_SINGULAR(DEV_GRP_NULL, RES_MAIN_REF, RES_STATE_WRST), 2},
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_ALL, RES_TYPE2_R2,
							RES_STATE_WRST), 2},
	{MSG_SINGULAR(DEV_GRP_NULL, RES_VUSB_3V1, RES_STATE_WRST), 2},
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_ALL, RES_TYPE2_R1,
							RES_STATE_WRST), 2},
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_RC, RES_TYPE_ALL, RES_TYPE2_R0,
							RES_STATE_WRST), 2},
	{MSG_SINGULAR(DEV_GRP_NULL, RES_RESET, RES_STATE_ACTIVE), 2},
};

static struct twl4030_script wrst_script __initdata = {
	.script = wrst_seq,
	.size   = ARRAY_SIZE(wrst_seq),
	.flags  = TWL4030_WRST_SCRIPT,
};

/* TRITON script for sleep, wakeup & warm_reset */
static struct twl4030_script *twl4030_scripts[] __initdata = {
	&wakeup_p12_script,
	&wakeup_p3_script,
	&sleep_on_script,
	&wrst_script,
};

static struct twl4030_resconfig twl4030_rconfig[] __initdata = {
	{ .resource = RES_VPLL1, .devgroup = DEV_GRP_P1, .type = 3,
		.type2 = 1, .remap_sleep = RES_STATE_OFF },
	{ .resource = RES_VINTANA1, .devgroup = DEV_GRP_ALL, .type = 1,
		.type2 = 2, .remap_sleep = RES_STATE_SLEEP },
	{ .resource = RES_VINTANA2, .devgroup = DEV_GRP_ALL, .type = 0,
		.type2 = 2, .remap_sleep = RES_STATE_SLEEP },
	{ .resource = RES_VINTDIG, .devgroup = DEV_GRP_ALL, .type = 1,
		.type2 = 2, .remap_sleep = RES_STATE_SLEEP },
	{ .resource = RES_VIO, .devgroup = DEV_GRP_ALL, .type = 2,
		.type2 = 2, .remap_sleep = RES_STATE_SLEEP },
	{ .resource = RES_VDD1, .devgroup = DEV_GRP_P1,
		.type = 4, .type2 = 1, .remap_sleep = RES_STATE_OFF },
	{ .resource = RES_VDD2, .devgroup = DEV_GRP_P1,
		.type = 3, .type2 = 1, .remap_sleep = RES_STATE_OFF },
	{ .resource = RES_REGEN, .devgroup = DEV_GRP_ALL, .type = 2,
		.type2 = 1, .remap_sleep = RES_STATE_OFF },
	{ .resource = RES_NRES_PWRON, .devgroup = DEV_GRP_ALL, .type = 0,
		.type2 = 1, .remap_sleep = RES_STATE_SLEEP },
	{ .resource = RES_CLKEN, .devgroup = DEV_GRP_ALL, .type = 3,
		.type2 = 2, .remap_sleep = RES_STATE_SLEEP },
	{ .resource = RES_SYSEN, .devgroup = DEV_GRP_ALL, .type = 6,
		.type2 = 1, .remap_sleep = RES_STATE_OFF },
	{ .resource = RES_HFCLKOUT, .devgroup = DEV_GRP_P3,
		.type = 0, .type2 = 2, .remap_sleep = RES_STATE_SLEEP },
	{ 0, 0},
};

static struct twl4030_power_data gossamer_t2scripts_data __initdata = {
	.scripts	= twl4030_scripts,
	.num		= ARRAY_SIZE(twl4030_scripts),
	.resource_config = twl4030_rconfig,
	//.use_poweroff	 = true,
};

static struct twl4030_platform_data __refdata gossamer_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	.madc		= &gossamer_madc_data,
	.usb		= &gossamer_usb_data,
	.gpio		= &gossamer_gpio_data,
	.keypad		= &gossamer_kp_twl4030_data,
	.power		= &gossamer_t2scripts_data, // Only valid during init
	.vmmc1          = &gossamer_vmmc1,
//	.vmmc2          = &gossamer_vmmc2,
	.vsim           = &gossamer_vsim,
//	.vdac		= &gossamer_vdac,
//	.vpll2		= &gossamer_vdsi,
};


#if defined(CONFIG_TOUCHSCREEN_ZFORCE) || defined(CONFIG_TOUCHSCREEN_ZFORCE_MODULE)
static struct zforce_platform_data zforce_platform = {
    .width = 600,
    .height = 800,
	.irqflags = IRQF_TRIGGER_FALLING,
};
#endif

static struct i2c_board_info __initdata gossamer_i2c_bus1_info[] = {
	{
		I2C_BOARD_INFO("tps65921", 0x48), //pass in bullshit to crash the kernel so ramconsole captures the mesg log
		.flags = I2C_CLIENT_WAKE,
		.irq = INT_34XX_SYS_NIRQ,
		.platform_data = &gossamer_twldata,
	},
#ifndef CONFIG_MACH_OMAP3621_GOSSAMER_EVT1C
#if defined(CONFIG_BATTERY_BQ27510) || defined(CONFIG_BATTERY_BQ27510_MODULE)
	{
		I2C_BOARD_INFO("bq27510",  0x55),
	},
#endif /* CONFIG_BATTERY_BQ27510 */
#endif
};

static struct i2c_board_info __initdata gossamer_i2c_bus2_info[] = {
#ifdef CONFIG_MACH_OMAP3621_GOSSAMER_EVT1C
#if defined(CONFIG_BATTERY_BQ27510) || defined(CONFIG_BATTERY_BQ27510_MODULE)
	{
		I2C_BOARD_INFO("bq27510",  0x55),
	},
#endif /* CONFIG_BATTERY_BQ27510 */
#if defined(CONFIG_TOUCHSCREEN_ZFORCE) || defined(CONFIG_TOUCHSCREEN_ZFORCE_MODULE)
	{
		I2C_BOARD_INFO(ZFORCE_NAME, ZFORCE_I2C_SLAVE_ADDRESS),
		.platform_data = &zforce_platform,
		.irq = OMAP_GPIO_IRQ(ZFORCE_GPIO_FOR_IRQ),
	},
#endif /* CONFIG_TOUCHSCREEN_ZFORCE */
#endif /* CONFIG_MACH_OMAP3621_GOSSAMER_EVT1C */

#if defined(CONFIG_LEDS_AS3676)
	{
		I2C_BOARD_INFO("as3676", 0x40 ),  
		.flags = I2C_CLIENT_WAKE,
		.irq = 0,
		.platform_data = &as3676_pdata,
	}
#endif /* CONFIG_LEDS_AS3676 */
};


#if defined(CONFIG_USB_ANDROID) || defined(CONFIG_USB_ANDROID_MODULE)
static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.vendor = "B&N     ",
	.product = "NOOK SimpleTouch",
	.release = 0x0100,
};

static struct platform_device usb_mass_storage_device = {
	.name = "usb_mass_storage",
	.id = -1,
	.dev = {
		.platform_data = &mass_storage_pdata,
		},
};

// Reserved for serial number passed in from the bootloader.
static char adb_serial_number[32] = "";

static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= BN_USB_VENDOR_ID,
	.product_id	= BN_USB_PRODUCT_ID_GOSSAMER,
	.adb_product_id	= BN_USB_PRODUCT_ID_GOSSAMER,
	.version	= 0x0100,
	.product_name	= "NOOK SimpleTouch",
	.manufacturer_name = "B&N",
	.serial_number	= "11223344556677",
	.nluns = 2,
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id		= -1,
	.dev		= {
		.platform_data = &android_usb_pdata,
	},
};
#endif

static int __init omap_i2c_init(void)
{

	int i2c1_devices;
	int i2c2_devices;

#if defined(CONFIG_TOUCHSCREEN_ZFORCE) || defined(CONFIG_TOUCHSCREEN_ZFORCE_MODULE)
	/*Request zforce irq gpio so clocks get enabled */
	if (gpio_request(ZFORCE_GPIO_FOR_IRQ, "ZFORCE_GPIO_FOR_IRQ") < 0)
	{
		printk(KERN_INFO "Couldn't get ZFORCE_GPIO_FOR_IRQ\n");
	}
#endif

	i2c1_devices = ARRAY_SIZE(gossamer_i2c_bus1_info);
	i2c2_devices = ARRAY_SIZE(gossamer_i2c_bus2_info);

	omap_register_i2c_bus(1, 100, gossamer_i2c_bus1_info,
			i2c1_devices);
	omap_register_i2c_bus(2, 400, gossamer_i2c_bus2_info,
			i2c2_devices);
	return 0;
}

static int __init wl127x_vio_leakage_fix(void)
{
	int ret = 0;

	ret = gpio_request(WL127X_BTEN_GPIO, "wl127x_bten");
	if (ret < 0) {
		printk(KERN_ERR "wl127x_bten gpio_%d request fail",
						WL127X_BTEN_GPIO);
		goto fail;
	}

	gpio_direction_output(WL127X_BTEN_GPIO, 1);
	mdelay(10);
	gpio_direction_output(WL127X_BTEN_GPIO, 0);
	udelay(64);

	gpio_free(WL127X_BTEN_GPIO);
fail:
	return ret;
}

static void  dump_board_revision(void)
{
	switch (system_rev) {
    case BOARD_GOSSAMER_REV_EVT1A:
        printk("gossamer board revision BOARD_GOSSAMER_REV_EVT1A\n");
        break;
	case BOARD_GOSSAMER_REV_EVTPRE1C:
		printk("gossamer board revision BOARD_GOSSAMER_REV_EVT_PRE_1C\n");
		break;
	case BOARD_GOSSAMER_REV_EVT1C:
		printk("gossamer board revision BOARD_GOSSAMER_REV_EVT1C\n");
		break;
	default:
		printk("gossamer unknown board revision: 0x%0x\n",system_rev);
		break;
	}
}

static struct gpio_led_platform_data led_data;

static struct platform_device gpio_leds_device = {
	.name			= "leds-gpio",
	.id			= -1,
	.dev.platform_data	= &led_data,
};

void __init gpio_leds(struct gpio_led *leds, int nr)
{
	if (!nr)
		return;

	led_data.leds = leds;
	led_data.num_leds = nr;
	platform_device_register(&gpio_leds_device);
}


static int wl12xx_set_power(struct device *dev, int slot, int on, int vdd)
{
	printk(KERN_WARNING"%s: %d\n", __func__, on);

	if (on) {
		gpio_set_value(GOSSAMER_WIFI_EN_POW, on);
		udelay(800);
		gpio_set_value(GOSSAMER_WIFI_PMENA_GPIO, on);
		mdelay(70);
	}
	else {
		gpio_set_value(GOSSAMER_WIFI_PMENA_GPIO, on);
		gpio_set_value(GOSSAMER_WIFI_EN_POW, on);
	}
	return 0;
}

static struct wl12xx_platform_data gossamer_wlan_data __initdata = {
	.irq = OMAP_GPIO_IRQ(GOSSAMER_WIFI_IRQ_GPIO),
	.board_ref_clock = WL12XX_REFCLOCK_38,
	/* 2.6.32 has edge triggered falling interrupt */
	//.platform_quirks = WL12XX_PLATFORM_QUIRK_EDGE_IRQ,
};

static void gossamer_wifi_init(void)
{
	struct device *dev;
	struct omap_mmc_platform_data *pdata;
	int ret;

	printk(KERN_WARNING"%s: start\n", __func__);

	ret = gpio_request(GOSSAMER_WIFI_PMENA_GPIO, "wifi_pmena");
	if (ret < 0) {
		pr_err("%s: can't reserve GPIO: %d\n", __func__,
						GOSSAMER_WIFI_PMENA_GPIO);
		goto out;
	}
	gpio_direction_output(GOSSAMER_WIFI_PMENA_GPIO, 0);
	gpio_export(GOSSAMER_WIFI_PMENA_GPIO, true);

	ret = gpio_request(GOSSAMER_WIFI_EN_POW, "wifi_pwen");
	if (ret < 0) {
		pr_err("%s: can't reserve GPIO: %d\n", __func__,
					GOSSAMER_WIFI_EN_POW);
		goto out;
	}
	gpio_direction_output(GOSSAMER_WIFI_EN_POW, 0);

	gpio_export(GOSSAMER_WIFI_EN_POW, true);
	ret = gpio_request(GOSSAMER_WIFI_IRQ_GPIO, "wifi_irq");
	if (ret < 0) {
		printk(KERN_ERR "%s: can't reserve GPIO: %d\n", __func__,
						GOSSAMER_WIFI_IRQ_GPIO);
		goto out;
	}
	gpio_direction_input(GOSSAMER_WIFI_IRQ_GPIO);

	dev = mmc[2].dev;
	if (!dev) {
		pr_err("wl12xx mmc device initialization failed\n");
		goto out;
	}

	pdata = dev->platform_data;
	if (!pdata) {
		pr_err("Platfrom data of wl12xx device not set\n");
		goto out;
	}

	pdata->slots[0].set_power = wl12xx_set_power;

	if (wl12xx_set_platform_data(&gossamer_wlan_data))
		pr_err("Error setting wl12xx data\n");
out:
	return;
}


static void __init omap_gossamer_init(void)
{
	/*we need to have this enable function here to lit up the BL*/
	dump_board_revision();

#if defined(CONFIG_TOUCHSCREEN_ZFORCE) || defined(CONFIG_TOUCHSCREEN_ZFORCE_MODULE)
#ifdef CONFIG_MACH_OMAP3621_GOSSAMER_EVT1C
	// Note: the Papyrus i2c address is now automatically detected
	//       by the driver
	if ( is_gossamer_board_evt_pre1c() )
	{
		zforce_platform.irqflags = IRQF_TRIGGER_RISING;
    }
#endif
#endif /* CONFIG_TOUCHSCREEN_ZFORCE */


	omap_i2c_init();
	platform_device_register(&gossamer_vmmc2_fixed_device);
	omap_register_ion();
	

	platform_add_devices(gossamer_devices, ARRAY_SIZE(gossamer_devices));

//	msecure_init();
	omap_serial_init();
	usb_musb_init(NULL);
#if defined(CONFIG_USB_ANDROID) || defined(CONFIG_USB_ANDROID_MODULE)
	platform_device_register(&usb_mass_storage_device);
	// Set the device serial number passed in from the bootloader.
	if (system_serial_high != 0 || system_serial_low != 0) {
		snprintf(adb_serial_number, sizeof(adb_serial_number), "%08x%08x", system_serial_high, system_serial_low);
		adb_serial_number[16] = '\0';
		android_usb_pdata.serial_number = adb_serial_number;
	}
	platform_device_register(&android_usb_device);
#endif

#ifdef CONFIG_MACH_OMAP3621_GOSSAMER
	if  ( is_gossamer_board_evt_pre1c() )
	{
		gossamer_twl4030_keymap[2] = KEY(1, 2, KEY_VOLUMEUP);
		gossamer_twl4030_keymap[3] = KEY(1, 3, KEY_VOLUMEDOWN);

#if defined(CONFIG_REGULATOR_BQ24073) || defined(CONFIG_REGULATOR_BQ24073_MODULE)
		bq24073_init_dev_data.gpio_nce = GOSSAMER_PRE1C_CHARGE_ENAB_GPIO;
		bq24073_init_dev_data.gpio_en1 = GOSSAMER_PRE1C_CHARGE_ILM1_GPIO;
	    bq24073_init_dev_data.gpio_en2 = GOSSAMER_PRE1C_CHARGE_ILM2_GPIO;
#endif

	}
#endif /* CONFIG_MACH_OMAP3621_GOSSAMER */

#ifdef CONFIG_MACH_OMAP3621_GOSSAMER_EVT1C
	gpio_leds(debug_leds, ARRAY_SIZE(debug_leds));
#endif /* CONFIG_MACH_OMAP3621_GOSSAMER_EVT1C */

	/* Fix to prevent VIO leakage on wl127x */
	wl127x_vio_leakage_fix();

    BUG_ON(!cpu_is_omap3630());
}

static void __init omap_gossamer_init_early(void)
{
	omap2_init_common_infrastructure();
	omap2_init_common_devices(samsung_k4x2g323pd_sdrc_params,
				samsung_k4x2g323pd_sdrc_params);
#ifdef CONFIG_OMAP_32K_TIMER
	omap2_gp_clockevent_set_gptimer(1);
#endif
}

static void __init omap_gossamer_reserve(void)
{
	/* Must be 2M or board fails early in arch/arm/mm/mmu.c*/
	omap_ram_console_init(GOSSAMER_RAM_CONSOLE_START,SZ_2M);
	omap_reserve();
}
 
MACHINE_START(OMAP3621_GOSSAMER, "OMAP3621 GOSSAMER board")
	.boot_params	= 0x80000100,
	.reserve	= omap_gossamer_reserve,
	.map_io		= omap3_map_io,
	.init_early	= omap_gossamer_init_early,
	.init_irq	= omap_init_irq,
	.init_machine	= omap_gossamer_init,
	.timer		= &omap_timer,
MACHINE_END
