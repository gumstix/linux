/*
 * linux/arch/arm/mach-omap2/board-omap3beagle.c
 *
 * Copyright (C) 2008 Texas Instruments
 *
 * Modified from mach-omap2/board-3430sdp.c
 *
 * Initial code: Syed Mohammed Khasim
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/opp.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/mmc/host.h>

#include <linux/regulator/machine.h>
#include <linux/i2c/twl.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>

#include <plat/board.h>
#include <plat/common.h>
#include <video/omapdss.h>
#include <video/omap-panel-dvi.h>
#include <plat/gpmc.h>
#include <plat/nand.h>
#include <plat/usb.h>
#include <plat/omap_device.h>

#include "mux.h"
#include "hsmmc.h"
#include "pm.h"
#include "common-board-devices.h"

/*
 * OMAP3 Beagle revision
 * Run time detection of Beagle revision is done by reading GPIO.
 * GPIO ID -
 *	AXBX	= GPIO173, GPIO172, GPIO171: 1 1 1
 *	C1_3	= GPIO173, GPIO172, GPIO171: 1 1 0
 *	C4	= GPIO173, GPIO172, GPIO171: 1 0 1
 *	XMA/XMB = GPIO173, GPIO172, GPIO171: 0 0 0
 *	XMC = GPIO173, GPIO172, GPIO171: 0 1 0
 */
enum {
	OMAP3BEAGLE_BOARD_UNKN = 0,
	OMAP3BEAGLE_BOARD_AXBX,
	OMAP3BEAGLE_BOARD_C1_3,
	OMAP3BEAGLE_BOARD_C4,
	OMAP3BEAGLE_BOARD_XM,
	OMAP3BEAGLE_BOARD_XMC,
};

static u8 omap3_beagle_version;

/*
 * Board-specific configuration
 * Defaults to BeagleBoard-xMC
 */
static struct {
	int mmc1_gpio_wp;
	int usb_pwr_level;
	int reset_gpio;
	int usr_button_gpio;
} beagle_config = {
	.mmc1_gpio_wp = -EINVAL,
	.usb_pwr_level = GPIOF_OUT_INIT_LOW,
	.reset_gpio = 129,
	.usr_button_gpio = 4,
};

static struct gpio omap3_beagle_rev_gpios[] __initdata = {
	{ 171, GPIOF_IN, "rev_id_0"    },
	{ 172, GPIOF_IN, "rev_id_1" },
	{ 173, GPIOF_IN, "rev_id_2"    },
};

static void __init omap3_beagle_init_rev(void)
{
	int ret;
	u16 beagle_rev = 0;

	omap_mux_init_gpio(171, OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_gpio(172, OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_gpio(173, OMAP_PIN_INPUT_PULLUP);

	ret = gpio_request_array(omap3_beagle_rev_gpios,
				 ARRAY_SIZE(omap3_beagle_rev_gpios));
	if (ret < 0) {
		printk(KERN_ERR "Unable to get revision detection GPIO pins\n");
		omap3_beagle_version = OMAP3BEAGLE_BOARD_UNKN;
		return;
	}

	beagle_rev = gpio_get_value(171) | (gpio_get_value(172) << 1)
			| (gpio_get_value(173) << 2);

	gpio_free_array(omap3_beagle_rev_gpios,
			ARRAY_SIZE(omap3_beagle_rev_gpios));

	switch (beagle_rev) {
	case 7:
		printk(KERN_INFO "OMAP3 Beagle Rev: Ax/Bx\n");
		omap3_beagle_version = OMAP3BEAGLE_BOARD_AXBX;
		beagle_config.mmc1_gpio_wp = 29;
		beagle_config.reset_gpio = 170;
		beagle_config.usr_button_gpio = 7;
		break;
	case 6:
		printk(KERN_INFO "OMAP3 Beagle Rev: C1/C2/C3\n");
		omap3_beagle_version = OMAP3BEAGLE_BOARD_C1_3;
		beagle_config.mmc1_gpio_wp = 23;
		beagle_config.reset_gpio = 170;
		beagle_config.usr_button_gpio = 7;
		break;
	case 5:
		printk(KERN_INFO "OMAP3 Beagle Rev: C4\n");
		omap3_beagle_version = OMAP3BEAGLE_BOARD_C4;
		beagle_config.mmc1_gpio_wp = 23;
		beagle_config.reset_gpio = 170;
		beagle_config.usr_button_gpio = 7;
		break;
	case 0:
		printk(KERN_INFO "OMAP3 Beagle Rev: xM Ax/Bx\n");
		omap3_beagle_version = OMAP3BEAGLE_BOARD_XM;
		beagle_config.usb_pwr_level = GPIOF_OUT_INIT_HIGH;
		break;
	case 2:
		printk(KERN_INFO "OMAP3 Beagle Rev: xM C\n");
		omap3_beagle_version = OMAP3BEAGLE_BOARD_XMC;
		break;
	default:
		printk(KERN_INFO "OMAP3 Beagle Rev: unknown %hd\n", beagle_rev);
		omap3_beagle_version = OMAP3BEAGLE_BOARD_UNKN;
	}
}

char expansionboard_name[16];

#if defined(CONFIG_WL12XX) || defined(CONFIG_WL12XX_MODULE)
#include <linux/regulator/fixed.h>
#include <linux/wl12xx.h>

#define OMAP_BEAGLE_WLAN_EN_GPIO    (139)
#define OMAP_BEAGLE_BT_EN_GPIO      (138)
#define OMAP_BEAGLE_WLAN_IRQ_GPIO   (137)
#define OMAP_BEAGLE_FM_EN_BT_WU     (136)

struct wl12xx_platform_data omap_beagle_wlan_data __initdata = {
	.irq = OMAP_GPIO_IRQ(OMAP_BEAGLE_WLAN_IRQ_GPIO),
	.board_ref_clock = 2, /* 38.4 MHz */
};

static int gpios[] = {OMAP_BEAGLE_BT_EN_GPIO, OMAP_BEAGLE_FM_EN_BT_WU, -1};
static struct platform_device wl12xx_device = {
		.name		= "kim",
		.id			= -1,
		.dev.platform_data = &gpios,
};

static struct omap2_hsmmc_info mmcbbt[] = {
 	{
 		.mmc		= 1,
 		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
 		.gpio_wp	= 29,
 	},
	{
		.name		= "wl1271",
		.mmc		= 2,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_POWER_OFF_CARD,
		.gpio_wp	= -EINVAL,
		.gpio_cd	= -EINVAL,
		.ocr_mask	= MMC_VDD_165_195,
		.nonremovable	= true,
	},
 	{}	/* Terminator */
 };

static struct regulator_consumer_supply beagle_vmmc2_supply = 
	REGULATOR_SUPPLY("vmmc", "omap_hsmmc.1");

static struct regulator_init_data beagle_vmmc2 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 1850000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &beagle_vmmc2_supply,
};

static struct fixed_voltage_config beagle_vwlan = {
	.supply_name = "vwl1271",
	.microvolts = 1800000,  /* 1.8V */
	.gpio = OMAP_BEAGLE_WLAN_EN_GPIO,
	.startup_delay = 70000, /* 70ms */
	.enable_high = 1,
	.enabled_at_boot = 0,
	.init_data = &beagle_vmmc2,
};

static struct platform_device omap_vwlan_device = {
	.name           = "reg-fixed-voltage",
	.id             = 1,
	.dev = {
		.platform_data = &beagle_vwlan,
	},
};
#endif

#if defined(CONFIG_ENC28J60) || defined(CONFIG_ENC28J60_MODULE)

#include <plat/mcspi.h>
#include <linux/spi/spi.h>

#define OMAP3BEAGLE_GPIO_ENC28J60_IRQ 157

static struct omap2_mcspi_device_config enc28j60_spi_chip_info = {
	.turbo_mode	= 0,
	.single_channel	= 1,	/* 0: slave, 1: master */
};

static struct spi_board_info omap3beagle_zippy_spi_board_info[] __initdata = {
	{
		.modalias		= "enc28j60",
		.bus_num		= 4,
		.chip_select		= 0,
		.max_speed_hz		= 20000000,
		.controller_data	= &enc28j60_spi_chip_info,
	},
};

static void __init omap3beagle_enc28j60_init(void)
{
	if ((gpio_request(OMAP3BEAGLE_GPIO_ENC28J60_IRQ, "ENC28J60_IRQ") == 0) &&
	    (gpio_direction_input(OMAP3BEAGLE_GPIO_ENC28J60_IRQ) == 0)) {
		gpio_export(OMAP3BEAGLE_GPIO_ENC28J60_IRQ, 0);
		omap3beagle_zippy_spi_board_info[0].irq	= OMAP_GPIO_IRQ(OMAP3BEAGLE_GPIO_ENC28J60_IRQ);
		irq_set_irq_type(omap3beagle_zippy_spi_board_info[0].irq, IRQ_TYPE_EDGE_FALLING);
	} else {
		printk(KERN_ERR "could not obtain gpio for ENC28J60_IRQ\n");
		return;
	}

	spi_register_board_info(omap3beagle_zippy_spi_board_info,
			ARRAY_SIZE(omap3beagle_zippy_spi_board_info));
}

#else
static inline void __init omap3beagle_enc28j60_init(void) { return; }
#endif

#if defined(CONFIG_KS8851) || defined(CONFIG_KS8851_MODULE)

#include <plat/mcspi.h>
#include <linux/spi/spi.h>

#define OMAP3BEAGLE_GPIO_KS8851_IRQ 157

static struct omap2_mcspi_device_config ks8851_spi_chip_info = {
	.turbo_mode	= 0,
	.single_channel	= 1,	/* 0: slave, 1: master */
};

static struct spi_board_info omap3beagle_zippy2_spi_board_info[] __initdata = {
	{
		.modalias		= "ks8851",
		.bus_num		= 4,
		.chip_select		= 0,
		.max_speed_hz		= 36000000,
		.controller_data	= &ks8851_spi_chip_info,
	},
};

static void __init omap3beagle_ks8851_init(void)
{
	if ((gpio_request(OMAP3BEAGLE_GPIO_KS8851_IRQ, "KS8851_IRQ") == 0) &&
	    (gpio_direction_input(OMAP3BEAGLE_GPIO_KS8851_IRQ) == 0)) {
		gpio_export(OMAP3BEAGLE_GPIO_KS8851_IRQ, 0);
		omap3beagle_zippy2_spi_board_info[0].irq	= OMAP_GPIO_IRQ(OMAP3BEAGLE_GPIO_KS8851_IRQ);
		irq_set_irq_type(omap3beagle_zippy2_spi_board_info[0].irq, IRQ_TYPE_EDGE_FALLING);
	} else {
		printk(KERN_ERR "could not obtain gpio for KS8851_IRQ\n");
		return;
	}
	
	spi_register_board_info(omap3beagle_zippy2_spi_board_info,
							ARRAY_SIZE(omap3beagle_zippy2_spi_board_info));
}

#else
static inline void __init omap3beagle_ks8851_init(void) { return; }
#endif

static struct mtd_partition omap3beagle_nand_partitions[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{
		.name		= "X-Loader",
		.offset		= 0,
		.size		= 4 * NAND_BLOCK_SIZE,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "U-Boot",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x80000 */
		.size		= 15 * NAND_BLOCK_SIZE,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "U-Boot Env",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x260000 */
		.size		= 1 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "Kernel",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x280000 */
		.size		= 32 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "File System",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x680000 */
		.size		= MTDPART_SIZ_FULL,
	},
};

/* DSS */

static int beagle_enable_dvi(struct omap_dss_device *dssdev)
{
	if (gpio_is_valid(dssdev->reset_gpio))
		gpio_set_value(dssdev->reset_gpio, 1);

	return 0;
}

static void beagle_disable_dvi(struct omap_dss_device *dssdev)
{
	if (gpio_is_valid(dssdev->reset_gpio))
		gpio_set_value(dssdev->reset_gpio, 0);
}

static struct panel_dvi_platform_data dvi_panel = {
	.platform_enable = beagle_enable_dvi,
	.platform_disable = beagle_disable_dvi,
	.i2c_bus_num = 3,
};

static struct omap_dss_device beagle_dvi_device = {
	.type = OMAP_DISPLAY_TYPE_DPI,
	.name = "dvi",
	.driver_name = "dvi",
	.data = &dvi_panel,
	.phy.dpi.data_lines = 24,
	.reset_gpio = -EINVAL,
};

static struct omap_dss_device beagle_tv_device = {
	.name = "tv",
	.driver_name = "venc",
	.type = OMAP_DISPLAY_TYPE_VENC,
	.phy.venc.type = OMAP_DSS_VENC_TYPE_SVIDEO,
};

static struct omap_dss_device *beagle_dss_devices[] = {
	&beagle_dvi_device,
	&beagle_tv_device,
};

static struct omap_dss_board_info beagle_dss_data = {
	.num_devices = ARRAY_SIZE(beagle_dss_devices),
	.devices = beagle_dss_devices,
	.default_device = &beagle_dvi_device,
};

static void __init beagle_display_init(void)
{
	int r;

	r = gpio_request_one(beagle_dvi_device.reset_gpio, GPIOF_OUT_INIT_LOW,
			     "DVI reset");
	if (r < 0)
		printk(KERN_ERR "Unable to get DVI reset GPIO\n");
}

#include "sdram-micron-mt46h32m32lf-6.h"

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
		.gpio_wp	= -EINVAL,
	},
	{
		.mmc		= 2,
		.caps       = MMC_CAP_4_BIT_DATA,
		.transceiver	= true,
		.ocr_mask	= 0x00100000,	/* 3.3V */
	},
	{}	/* Terminator */
};

static struct regulator_consumer_supply beagle_vmmc1_supply[] = {
	REGULATOR_SUPPLY("vmmc", "omap_hsmmc.0"),
};

static struct regulator_consumer_supply beagle_vsim_supply[] = {
	REGULATOR_SUPPLY("vmmc_aux", "omap_hsmmc.0"),
};

static struct gpio_led gpio_leds[];

static int beagle_twl_gpio_setup(struct device *dev,
		unsigned gpio, unsigned ngpio)
{
	int r;

	if (beagle_config.mmc1_gpio_wp != -EINVAL)
		omap_mux_init_gpio(beagle_config.mmc1_gpio_wp, OMAP_PIN_INPUT);
	mmc[0].gpio_wp = beagle_config.mmc1_gpio_wp;
	/* gpio + 0 is "mmc0_cd" (input/IRQ) */
	mmc[0].gpio_cd = gpio + 0;
#if defined(CONFIG_WL12XX) || defined(CONFIG_WL12XX_MODULE)
	if(!strcmp(expansionboard_name, "bbtoys-wifi")) { 
		omap2_hsmmc_init(mmcbbt);
	} else {
		omap2_hsmmc_init(mmc);
	}
#else
	omap2_hsmmc_init(mmc);
#endif

	/*
	 * TWL4030_GPIO_MAX + 0 == ledA, EHCI nEN_USB_PWR (out, XM active
	 * high / others active low)
	 * DVI reset GPIO is different between beagle revisions
	 */
	/* Valid for all -xM revisions */
	if (cpu_is_omap3630()) {
		/*
		 * gpio + 1 on Xm controls the TFP410's enable line (active low)
		 * gpio + 2 control varies depending on the board rev as below:
		 * P7/P8 revisions(prototype): Camera EN
		 * A2+ revisions (production): LDO (DVI, serial, led blocks)
		 */
		r = gpio_request_one(gpio + 1, GPIOF_OUT_INIT_LOW,
				     "nDVI_PWR_EN");
		if (r)
			pr_err("%s: unable to configure nDVI_PWR_EN\n",
				__func__);
		r = gpio_request_one(gpio + 2, GPIOF_OUT_INIT_HIGH,
				     "DVI_LDO_EN");
		if (r)
			pr_err("%s: unable to configure DVI_LDO_EN\n",
				__func__);
	} else {
		/*
		 * REVISIT: need ehci-omap hooks for external VBUS
		 * power switch and overcurrent detect
		 */
		if (gpio_request_one(gpio + 1, GPIOF_IN, "EHCI_nOC"))
			pr_err("%s: unable to configure EHCI_nOC\n", __func__);
	}
	beagle_dvi_device.reset_gpio = beagle_config.reset_gpio;

	gpio_request_one(gpio + TWL4030_GPIO_MAX, beagle_config.usb_pwr_level,
			"nEN_USB_PWR");

	/* TWL4030_GPIO_MAX + 1 == ledB, PMU_STAT (out, active low LED) */
	gpio_leds[2].gpio = gpio + TWL4030_GPIO_MAX + 1;

	return 0;
}

static struct twl4030_gpio_platform_data beagle_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	.use_leds	= true,
	.pullups	= BIT(1),
	.pulldowns	= BIT(2) | BIT(6) | BIT(7) | BIT(8) | BIT(13)
				| BIT(15) | BIT(16) | BIT(17),
	.setup		= beagle_twl_gpio_setup,
};

/* VMMC1 for MMC1 pins CMD, CLK, DAT0..DAT3 (20 mA, plus card == max 220 mA) */
static struct regulator_init_data beagle_vmmc1 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 3150000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= ARRAY_SIZE(beagle_vmmc1_supply),
	.consumer_supplies	= beagle_vmmc1_supply,
};

/* VSIM for MMC1 pins DAT4..DAT7 (2 mA, plus card == max 50 mA) */
static struct regulator_init_data beagle_vsim = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 3000000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= ARRAY_SIZE(beagle_vsim_supply),
	.consumer_supplies	= beagle_vsim_supply,
};

static struct twl4030_platform_data beagle_twldata = {
	/* platform_data for children goes here */
	.gpio		= &beagle_gpio_data,
	.vmmc1		= &beagle_vmmc1,
	.vsim		= &beagle_vsim,
};

static struct i2c_board_info __initdata beagle_i2c_eeprom[] = {
       {
               I2C_BOARD_INFO("eeprom", 0x50),
       },
};

#if defined(CONFIG_RTC_DRV_DS1307) || \
	defined(CONFIG_RTC_DRV_DS1307_MODULE)

static struct i2c_board_info __initdata beagle_i2c2_zippy[] = {
	{
		I2C_BOARD_INFO("eeprom", 0x50),
		I2C_BOARD_INFO("ds1307", 0x68),
	},
};
#else
static struct i2c_board_info __initdata beagle_i2c2_zippy[] = {};
#endif

static int __init omap3_beagle_i2c_init(void)
{
	omap3_pmic_get_config(&beagle_twldata,
			TWL_COMMON_PDATA_USB | TWL_COMMON_PDATA_MADC |
			TWL_COMMON_PDATA_AUDIO,
			TWL_COMMON_REGULATOR_VDAC | TWL_COMMON_REGULATOR_VPLL2);

	beagle_twldata.vpll2->constraints.name = "VDVI";

	omap3_pmic_init("twl4030", &beagle_twldata);
	/* Bus 3 is attached to the DVI port where devices like the pico DLP
	 * projector don't work reliably with 400kHz */
	omap_register_i2c_bus(3, 100, beagle_i2c_eeprom, ARRAY_SIZE(beagle_i2c_eeprom));
	return 0;
}

static struct gpio_led gpio_leds[] = {
	{
		.name			= "beagleboard::usr0",
		.default_trigger	= "heartbeat",
		.gpio			= 150,
	},
	{
		.name			= "beagleboard::usr1",
		.default_trigger	= "mmc0",
		.gpio			= 149,
	},
	{
		.name			= "beagleboard::pmu_stat",
		.gpio			= -EINVAL,	/* gets replaced */
		.active_low		= true,
	},
};

static struct gpio_led_platform_data gpio_led_info = {
	.leds		= gpio_leds,
	.num_leds	= ARRAY_SIZE(gpio_leds),
};

static struct platform_device leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_led_info,
	},
};

static struct gpio_keys_button gpio_buttons[] = {
	{
		.code			= BTN_EXTRA,
		/* Dynamically assigned depending on board */
		.gpio			= -EINVAL,
		.desc			= "user",
		.wakeup			= 1,
	},
};

static struct gpio_keys_platform_data gpio_key_info = {
	.buttons	= gpio_buttons,
	.nbuttons	= ARRAY_SIZE(gpio_buttons),
};

static struct platform_device keys_gpio = {
	.name	= "gpio-keys",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_key_info,
	},
};

static struct platform_device *omap3_beagle_devices[] __initdata = {
	&leds_gpio,
	&keys_gpio,
};

static const struct usbhs_omap_board_data usbhs_bdata __initconst = {

	.port_mode[0] = OMAP_EHCI_PORT_MODE_PHY,
	.port_mode[1] = OMAP_EHCI_PORT_MODE_PHY,
	.port_mode[2] = OMAP_USBHS_PORT_MODE_UNUSED,

	.phy_reset  = true,
	.reset_gpio_port[0]  = -EINVAL,
	.reset_gpio_port[1]  = 147,
	.reset_gpio_port[2]  = -EINVAL
};

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#endif

static int __init expansionboard_setup(char *str)
{
	if (!str)
		return -EINVAL;
	strncpy(expansionboard_name, str, 16);
	printk(KERN_INFO "Beagle expansionboard: %s\n", expansionboard_name);
	return 0;
}

static void __init beagle_opp_init(void)
{
	int r = 0;

	/* Initialize the omap3 opp table */
	if (omap3_opp_init()) {
		pr_err("%s: opp default init failed\n", __func__);
		return;
	}

	/* Custom OPP enabled for all xM versions */
	if (cpu_is_omap3630()) {
		struct device *mpu_dev, *iva_dev;

		mpu_dev = omap_device_get_by_hwmod_name("mpu");
		iva_dev = omap_device_get_by_hwmod_name("iva");

		if (!mpu_dev || !iva_dev) {
			pr_err("%s: Aiee.. no mpu/dsp devices? %p %p\n",
				__func__, mpu_dev, iva_dev);
			return;
		}
		/* Enable MPU 1GHz and lower opps */
		r = opp_enable(mpu_dev, 800000000);
		r |= opp_enable(mpu_dev, 1000000000);

		/* Enable IVA 800MHz and lower opps */
		r |= opp_enable(iva_dev, 660000000);
		r |= opp_enable(iva_dev, 800000000);

		if (r) {
			pr_err("%s: failed to enable higher opp %d\n",
				__func__, r);
			/*
			 * Cleanup - disable the higher freqs - we dont care
			 * about the results
			 */
			opp_disable(mpu_dev, 800000000);
			opp_disable(mpu_dev, 1000000000);
			opp_disable(iva_dev, 660000000);
			opp_disable(iva_dev, 800000000);
		}
	}
	return;
}

static void __init omap3_beagle_init(void)
{
	omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);
	omap3_beagle_init_rev();
	omap3_beagle_i2c_init();

	gpio_buttons[0].gpio = beagle_config.usr_button_gpio;

	platform_add_devices(omap3_beagle_devices,
			ARRAY_SIZE(omap3_beagle_devices));
	omap_display_init(&beagle_dss_data);
	omap_serial_init();
	omap_sdrc_init(mt46h32m32lf6_sdrc_params,
				  mt46h32m32lf6_sdrc_params);

	omap_mux_init_gpio(170, OMAP_PIN_INPUT);
	/* REVISIT leave DVI powered down until it's needed ... */
	gpio_request_one(170, GPIOF_OUT_INIT_HIGH, "DVI_nPD");

	if(!strcmp(expansionboard_name, "zippy")) 
	{
		printk(KERN_INFO "Beagle expansionboard: initializing enc28j60\n");
		omap3beagle_enc28j60_init();
		printk(KERN_INFO "Beagle expansionboard: assigning GPIO 141 and 162 to MMC1\n");
		mmc[1].gpio_wp = 141;
		mmc[1].gpio_cd = 162;
		printk(KERN_INFO "Beagle expansionboard: registering I2C2 for zippy board\n");
		omap_register_i2c_bus(2, 400,  beagle_i2c2_zippy,
							ARRAY_SIZE(beagle_i2c2_zippy));

	}
	
	if(!strcmp(expansionboard_name, "zippy2")) 
	{
		printk(KERN_INFO "Beagle expansionboard: initializing ks_8851\n");
		omap3beagle_ks8851_init();
		printk(KERN_INFO "Beagle expansionboard: assigning GPIO 141 and 162 to MMC1\n");
		mmc[1].gpio_wp = 141;
		mmc[1].gpio_cd = 162;
		printk(KERN_INFO "Beagle expansionboard: registering I2C2 for zippy2 board\n");
		omap_register_i2c_bus(2, 400,  beagle_i2c2_zippy,
							ARRAY_SIZE(beagle_i2c2_zippy));		
	}

	if(!strcmp(expansionboard_name, "trainer"))
	{
		printk(KERN_INFO "Beagle expansionboard: exporting GPIOs 130-141,162 to userspace\n");
		gpio_request(130, "sysfs");
		gpio_export(130, 1);
		gpio_request(131, "sysfs");
		gpio_export(131, 1);
		gpio_request(132, "sysfs");
		gpio_export(132, 1);
		gpio_request(133, "sysfs");
		gpio_export(133, 1);
		gpio_request(134, "sysfs");
		gpio_export(134, 1);
		gpio_request(135, "sysfs");
		gpio_export(135, 1);
		gpio_request(136, "sysfs");
		gpio_export(136, 1);
		gpio_request(137, "sysfs");
		gpio_export(137, 1);
		gpio_request(138, "sysfs");
		gpio_export(138, 1);
		gpio_request(139, "sysfs");
		gpio_export(139, 1);
		gpio_request(140, "sysfs");
		gpio_export(140, 1);
		gpio_request(141, "sysfs");
		gpio_export(141, 1);
		gpio_request(162, "sysfs");
		gpio_export(162, 1);
	}

	if(!strcmp(expansionboard_name, "bbtoys-wifi"))
	{
		if (wl12xx_set_platform_data(&omap_beagle_wlan_data))
			pr_err("error setting wl12xx data\n");
		printk(KERN_INFO "Beagle expansionboard: registering wl12xx bt platform device\n");
		platform_device_register(&wl12xx_device);
		printk(KERN_INFO "Beagle expansionboard: registering wl12xx wifi platform device\n");
		platform_device_register(&omap_vwlan_device);
	}

	usb_musb_init(NULL);
	usbhs_init(&usbhs_bdata);
	omap_nand_flash_init(NAND_BUSWIDTH_16, omap3beagle_nand_partitions,
			     ARRAY_SIZE(omap3beagle_nand_partitions));

	/* Ensure msecure is mux'd to be able to set the RTC. */
	omap_mux_init_signal("sys_drm_msecure", OMAP_PIN_OFF_OUTPUT_HIGH);

	/* Ensure SDRC pins are mux'd for self-refresh */
	omap_mux_init_signal("sdrc_cke0", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("sdrc_cke1", OMAP_PIN_OUTPUT);

	beagle_display_init();
	beagle_opp_init();
}

early_param("buddy", expansionboard_setup);

MACHINE_START(OMAP3_BEAGLE, "OMAP3 Beagle Board")
	/* Maintainer: Syed Mohammed Khasim - http://beagleboard.org */
	.atag_offset	= 0x100,
	.reserve	= omap_reserve,
	.map_io		= omap3_map_io,
	.init_early	= omap3_init_early,
	.init_irq	= omap3_init_irq,
	.init_machine	= omap3_beagle_init,
	.timer		= &omap3_timer,
MACHINE_END
