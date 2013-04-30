/*
 * Board support file for OMAP4430 based duovero.
 *
 * Maintainer: Steve Sakoman <steve@sakoman.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/leds.h>
#include <linux/platform_device.h>

#include <linux/i2c/twl.h>
#include <linux/mfd/twl6040.h>
#include <linux/platform_data/omap-abe-twl6040.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/usb/otg.h>

#include <mach/hardware.h>
#include <asm/hardware/gic.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <video/omapdss.h>

#include <plat/board.h>
#include <plat/usb.h>
#include <plat/mmc.h>

#include "common.h"
#include "control.h"
#include "hsmmc.h"
#include "mux.h"
#include "common-board-devices.h"

#define HDMI_GPIO_LS_OE		41
#define HDMI_GPIO_HPD		63

/* smsc911x ethernet */
#if defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE)

#include <linux/smsc911x.h>
#include <plat/gpmc-smsc911x.h>

#define SMSC911X_CS      5
#define SMSC911X_GPIO    44

static struct omap_smsc911x_platform_data smsc911x_cfg = {
	.id		= 0,
	.cs             = SMSC911X_CS,
	.gpio_irq       = SMSC911X_GPIO,
	.gpio_reset     = -EINVAL,
	.flags		= SMSC911X_USE_32BIT,
};

static struct regulator_consumer_supply dummy_supplies[] = {
	REGULATOR_SUPPLY("vddvario", "smsc911x.0"),
	REGULATOR_SUPPLY("vdd33a", "smsc911x.0"),
};

static void __init duovero_init_smsc911x(void)
{
	regulator_register_fixed(0, dummy_supplies, ARRAY_SIZE(dummy_supplies));
	gpmc_smsc911x_init(&smsc911x_cfg);
}
#else
static inline void __init duovero_init_smsc911x(void) { return; }
#endif

/* hsusb */
#if defined(CONFIG_USB_EHCI_HCD_OMAP) || defined(CONFIG_USB_EHCI_HCD_OMAP_MODULE)

#define GPIO_EHCI_NRESET	62

static struct gpio duovero_usbhs_gpios[] __initdata = {
	{ GPIO_EHCI_NRESET,	GPIOF_OUT_INIT_LOW,  "ehci_nreset" },
};

static const struct usbhs_omap_board_data usbhs_bdata __initconst = {
	.port_mode[0] = OMAP_EHCI_PORT_MODE_PHY,
	.port_mode[1] = OMAP_USBHS_PORT_MODE_UNUSED,
	.port_mode[2] = OMAP_USBHS_PORT_MODE_UNUSED,
	.phy_reset  = false,
	.reset_gpio_port[0]  = -EINVAL,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL
};

static void __init duovero_usbhs_init(void)
{
	int ret;
	struct clk *phy_ref_clk;

	/* FREF_CLK3 provides the 19.2 MHz reference clock to the PHY */
	phy_ref_clk = clk_get(NULL, "auxclk3_ck");
	if (IS_ERR(phy_ref_clk)) {
		pr_err("Cannot request auxclk3\n");
		return;
	}
	ret = clk_set_rate(phy_ref_clk, 19200000);
	if (ret < 0) {
		pr_err("Cannot set auxclk3\n");
		return;
	}

	ret = clk_enable(phy_ref_clk);
	if (ret < 0) {
		pr_err("Cannot enable auxclk3\n");
		return;
	}

	/* request ehci gpios */
	ret = gpio_request_array(duovero_usbhs_gpios,
				 ARRAY_SIZE(duovero_usbhs_gpios));
	if (ret) {
		pr_err("Unable to initialize EHCI reset\n");
		return;
	}

	gpio_export(GPIO_EHCI_NRESET, 0);
	gpio_set_value(GPIO_EHCI_NRESET, 1);

	usbhs_init(&usbhs_bdata);
}

#else
static inline void __init duovero_usbhs_init(void) { return; }
#endif

/* musb */
#if defined(CONFIG_USB_MUSB_OMAP2PLUS) || defined(CONFIG_USB_MUSB_OMAP2PLUS_MODULE)

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_UTMI,
	.mode			= MUSB_OTG,
	.power			= 100,
};

static void __init duovero_musb_init(void)
{
	usb_musb_init(&musb_board_data);
}
#else
static inline void __init duovero_musb_init(void) { return; }
#endif

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_wp	= -EINVAL,
		.gpio_cd	= -EINVAL,
	},
	{
		.mmc		= 5,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_POWER_OFF_CARD,
		.gpio_wp	= -EINVAL,
		.gpio_cd	= -EINVAL,
                .ocr_mask	= MMC_VDD_30_31,
		.nonremovable	= true,
	},
	{}	/* Terminator */
};

static struct regulator_consumer_supply omap4_duovero_vmmc5_supply[] = {
	REGULATOR_SUPPLY("vmmc", "omap_hsmmc.4"),
};

static struct regulator_init_data duovero_vmmc5 = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = ARRAY_SIZE(omap4_duovero_vmmc5_supply),
	.consumer_supplies = omap4_duovero_vmmc5_supply,
};

static struct fixed_voltage_config duovero_vwlan = {
	.supply_name = "vwlan",
	.microvolts = 3000000, /* we lie here -- supply is actually 1.8V */
	.gpio = 43,
	.startup_delay = 70000, /* 70msec */
	.enable_high = 1,
	.enabled_at_boot = 1,
	.init_data = &duovero_vmmc5,
};

static struct platform_device omap_vwlan_device = {
	.name		= "reg-fixed-voltage",
	.id		= 1,
	.dev = {
		.platform_data = &duovero_vwlan,
	},
};


static struct twl6040_codec_data twl6040_codec = {
	/* single-step ramp for headset and handsfree */
	.hs_left_step	= 0x0f,
	.hs_right_step	= 0x0f,
	.hf_left_step	= 0x1d,
	.hf_right_step	= 0x1d,
};

static struct twl6040_platform_data twl6040_data = {
	.codec		= &twl6040_codec,
	.audpwron_gpio	= 160,
	.irq_base	= TWL6040_CODEC_IRQ_BASE,
};

/* duovero uses the common PMIC configuration */
static struct twl4030_platform_data duovero_twldata;

static int __init duovero_i2c_init(void)
{
	omap4_pmic_get_config(&duovero_twldata, TWL_COMMON_PDATA_USB,
			TWL_COMMON_REGULATOR_VDAC |
			TWL_COMMON_REGULATOR_VAUX2 |
			TWL_COMMON_REGULATOR_VAUX3 |
			TWL_COMMON_REGULATOR_VMMC |
			TWL_COMMON_REGULATOR_VPP |
			TWL_COMMON_REGULATOR_VANA |
			TWL_COMMON_REGULATOR_VCXIO |
			TWL_COMMON_REGULATOR_VUSB |
			TWL_COMMON_REGULATOR_CLK32KG |
			TWL_COMMON_REGULATOR_V1V8 |
			TWL_COMMON_REGULATOR_V2V1);
	omap4_pmic_init("twl6030", &duovero_twldata,
			&twl6040_data, OMAP44XX_IRQ_SYS_2N);
	omap_register_i2c_bus(2, 400, NULL, 0);
	omap_register_i2c_bus(3, 400, NULL, 0);
	omap_register_i2c_bus(4, 400, NULL, 0);
	return 0;
}

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	/* WLAN IRQ - GPIO 53 */
	OMAP4_MUX(GPMC_NCS3, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	/* WLAN POWER ENABLE - GPIO 43 */
	OMAP4_MUX(GPMC_A19, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	/* WLAN SDIO: MMC5 CMD */
	OMAP4_MUX(SDMMC5_CMD, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* WLAN SDIO: MMC5 CLK */
	OMAP4_MUX(SDMMC5_CLK, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* WLAN SDIO: MMC5 DAT[0-3] */
	OMAP4_MUX(SDMMC5_DAT0, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(SDMMC5_DAT1, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(SDMMC5_DAT2, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(SDMMC5_DAT3, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* User GPIOs */
	OMAP4_MUX(ABE_DMIC_DIN2, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	OMAP4_MUX(ABE_DMIC_DIN3, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	/* NIRQ2 for twl6040 */
	OMAP4_MUX(SYS_NIRQ2, OMAP_MUX_MODE0 |
		  OMAP_PIN_INPUT_PULLUP | OMAP_PIN_OFF_WAKEUPENABLE),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};

#else
#define board_mux	NULL
#endif

static struct gpio duovero_hdmi_gpios[] = {
	{ HDMI_GPIO_LS_OE, GPIOF_OUT_INIT_HIGH, "hdmi_gpio_ls_oe" },
	{ HDMI_GPIO_HPD, GPIOF_DIR_IN, "hdmi_gpio_hpd" },
};

static int duovero_panel_enable_hdmi(struct omap_dss_device *dssdev)
{
	int status;

	status = gpio_request_array(duovero_hdmi_gpios,
				    ARRAY_SIZE(duovero_hdmi_gpios));
	if (status)
		pr_err("Cannot request HDMI GPIOs\n");

	return status;
}

static void duovero_panel_disable_hdmi(struct omap_dss_device *dssdev)
{
	gpio_free_array(duovero_hdmi_gpios, ARRAY_SIZE(duovero_hdmi_gpios));
}

static struct omap_dss_hdmi_data duovero_hdmi_data = {
	.hpd_gpio = HDMI_GPIO_HPD,
};

static struct omap_dss_device  duovero_hdmi_device = {
	.name = "hdmi",
	.driver_name = "hdmi_panel",
	.type = OMAP_DISPLAY_TYPE_HDMI,
	.platform_enable = duovero_panel_enable_hdmi,
	.platform_disable = duovero_panel_disable_hdmi,
	.channel = OMAP_DSS_CHANNEL_DIGIT,
	.data = &duovero_hdmi_data,
};

static struct omap_dss_device *duovero_dss_devices[] = {
	&duovero_hdmi_device,
};

static struct omap_dss_board_info duovero_dss_data = {
	.num_devices	= ARRAY_SIZE(duovero_dss_devices),
	.devices	= duovero_dss_devices,
	.default_device	= &duovero_hdmi_device,
};

void duovero_display_init(void)
{
	omap_display_init(&duovero_dss_data);
	omap_hdmi_init(OMAP_HDMI_SDA_SCL_EXTERNAL_PULLUP);
}

static struct omap_abe_twl6040_data duovero_abe_audio_data = {
	.card_name	= "duovero",
	.has_hf		= ABE_TWL6040_LEFT | ABE_TWL6040_RIGHT,
	.jack_detection	= 0,
	.mclk_freq	= 38400000,

};

static struct platform_device duovero_abe_audio = {
	.name		= "omap-abe-twl6040",
	.id		= -1,
	.dev = {
		.platform_data = &duovero_abe_audio_data,
	},
};

static struct platform_device duovero_hdmi_audio_codec = {
	.name	= "hdmi-audio-codec",
	.id	= -1,
};

static struct platform_device *duovero_devices[] __initdata = {
	&duovero_hdmi_audio_codec,
	&duovero_abe_audio,
};


static void __init duovero_init(void)
{
	int package = OMAP_PACKAGE_CBS;
	omap4_mux_init(board_mux, NULL, package);

	duovero_i2c_init();
	platform_add_devices(duovero_devices, ARRAY_SIZE(duovero_devices));
	platform_device_register(&omap_vwlan_device);
	omap_serial_init();
	omap_sdrc_init(NULL, NULL);
	omap4_twl6030_hsmmc_init(mmc);
	duovero_usbhs_init();
	duovero_musb_init();
	duovero_display_init();
	duovero_init_smsc911x();
}

MACHINE_START(DUOVERO, "OMAP4 duovero board")
	.atag_offset	= 0x100,
	.reserve	= omap_reserve,
	.map_io		= omap4_map_io,
	.init_early	= omap4430_init_early,
	.init_irq	= gic_init_irq,
	.handle_irq	= gic_handle_irq,
	.init_machine	= duovero_init,
	.init_late	= omap4430_init_late,
	.timer		= &omap4_timer,
	.restart	= omap_prcm_restart,
MACHINE_END
