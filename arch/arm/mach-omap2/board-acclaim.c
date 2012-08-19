/*
 * Board support file for OMAP4430 ACCLAIM.
 *
 * Copyright (C) 2009 Texas Instruments
 *
 * Author: Santosh Shilimkar <santosh.shilimkar@ti.com>
 *
 * Based on mach-omap2/board-3430sdp.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/usb/otg.h>
#include <linux/spi/spi.h>
#include <linux/i2c/twl.h>
#include <linux/i2c/cma3000.h>
#include <linux/i2c/bq2415x.h>
#include <linux/regulator/machine.h>
#include <linux/input/sfh7741.h>
#include <linux/leds.h>
#include <linux/leds_pwm.h>
#include <linux/leds-omap4430sdp-display.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/twl6040-vib.h>
#include <linux/wl12xx.h>
#include <linux/cdc_tcxo.h>
#include <linux/bootmem.h>

#include <mach/hardware.h>
#include <mach/omap4-common.h>
#include <mach/emif.h>
#include <mach/lpddr2-elpida.h>
#include <mach/board-4430-acclaim.h>
#include <mach/dmm.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/android-display.h>
#include <plat/board.h>
#include <plat/common.h>
#include <plat/control.h>
#include <plat/timer-gp.h>
#include <plat/display.h>
#include <plat/usb.h>
#include <plat/omap_device.h>
#include <plat/omap_hwmod.h>
#include <linux/memblock.h>
#include "omap4_ion.h"

#include "mux.h"
#include "smartreflex-class3.h"

#define ACCLAIM_FB_RAM_SIZE             SZ_16M /* 1920×1080*4 * 2 */

extern unsigned int system_modelid;
volatile unsigned int KERNEL_SV = 0x0;

void __init acclaim_peripherals_init(void);

static void __init omap_4430_acclaim_init_irq(void)
{
	omap2_init_common_hw(NULL, NULL);
#ifdef CONFIG_OMAP_32K_TIMER
	omap2_gp_clockevent_set_gptimer(1);
#endif
	gic_init_irq();
	sr_class3_init();
}

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
#ifndef CONFIG_TIWLAN_SDIO
	/* WLAN IRQ - GPIO 53 */
	OMAP4_MUX(GPMC_NCS3, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	/* WLAN_EN - GPIO 54 */
	OMAP4_MUX(GPMC_NWP, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	/* WLAN SDIO: MMC5 CMD */
	OMAP4_MUX(SDMMC5_CMD, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* WLAN SDIO: MMC5 CLK */
	OMAP4_MUX(SDMMC5_CLK, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* WLAN SDIO: MMC5 DAT[0-3] */
	OMAP4_MUX(SDMMC5_DAT0, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(SDMMC5_DAT1, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(SDMMC5_DAT2, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(SDMMC5_DAT3, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
#endif
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux	NULL
#endif

#ifdef CONFIG_ANDROID_RAM_CONSOLE
static inline void ramconsole_reserve_sdram(void)
{
	// make the ram console the size of the printk log buffer
	int ret;
	ret = memblock_remove (ACCLAIM_RAM_CONSOLE_START, 
			       ACCLAIM_RAM_CONSOLE_SIZE);
	if (ret)
		printk("unable to remove memory for ram console");
}
#else
static inline void ramconsole_reserve_sdram(void) {}
#endif /* CONFIG_ANDROID_RAM_CONSOLE */

static struct omap_dss_device acclaim_boxer_device = {
	.phy		= {
		.dpi	= {
			.data_lines	= 24,
		},
	},
	.panel          = {
		.config		= OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS |
		OMAP_DSS_LCD_IHS,
		.timings	= {
			.x_res          = 1024,
			.y_res          = 600,
			.pixel_clock    = 46000, /* in kHz */
			.hfp            = 160,   /* HFP fix 160 */
			.hsw            = 10,    /* HSW = 1~140 */
			.hbp            = 150,   /* HSW + HBP = 160 */
			.vfp            = 12,    /* VFP fix 12 */
			.vsw            = 3,     /* VSW = 1~20 */
			.vbp            = 20,    /* VSW + VBP = 23 */
		},
		.width_in_um = 158000,
		.height_in_um = 92000,
	},
	.name			= "lcd2",
	.driver_name		= "boxer_panel",
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.channel		= OMAP_DSS_CHANNEL_LCD2,
};

static struct omap_dss_device *acclaim_dss_devices[] = {
	&acclaim_boxer_device,
};
 
static struct omap_dss_board_info acclaim_dss_data = {
	.num_devices	= ARRAY_SIZE(acclaim_dss_devices),
	.devices	= acclaim_dss_devices,
	.default_device	= &acclaim_boxer_device,
};

static struct spi_board_info tablet_spi_board_info[] __initdata = {
	{
		.modalias= "boxer_disp_spi",
		.bus_num= 4,     /* McSPI4 */
		.chip_select= 0,
		.max_speed_hz= 375000,
	},
};

static struct omapfb_platform_data acclaim_fb_pdata = {
	.mem_desc = {
		.region_cnt = 1,
		.region = {
			[0] = {
				.size = ACCLAIM_FB_RAM_SIZE,
			},
		},
	},
};

static void __init omap_4430_acclaim_init(void)
{
	omap_dmm_init();
#ifdef CONFIG_ION_OMAP
	omap_ion_init();
#endif
	acclaim_peripherals_init();
	omap4_create_board_props();
}

static void __init omap_4430_acclaim_map_io(void)
{
	omap2_set_globals_443x();
	omap44xx_map_common_io();
}

static void __init acclaim_reserve(void)
{
	ramconsole_reserve_sdram();

#ifdef CONFIG_ION_OMAP
	omap_android_display_setup(&acclaim_dss_data,
				   NULL,
				   NULL,
				   &acclaim_fb_pdata,
				   get_omap_ion_platform_data());
	omap_ion_init();
#else
	omap_android_display_setup(&acclaim_dss_data,
				   NULL,
				   NULL,
				   &acclaim_fb_pdata,
				   NULL);
#endif

	/* do the static reservations first */
	memblock_remove(PHYS_ADDR_SMC_MEM, PHYS_ADDR_SMC_SIZE);

	omap_reserve();
}

MACHINE_START(OMAP4_ACCLAIM, "acclaim")
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xfa000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= omap_4430_acclaim_map_io,
	.reserve	= acclaim_reserve,
	.init_irq	= omap_4430_acclaim_init_irq,
	.init_machine	= omap_4430_acclaim_init,
	.timer		= &omap_timer,
MACHINE_END
