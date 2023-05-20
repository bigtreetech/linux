// SPDX-License-Identifier: GPL-2.0+
/*
 * FB driver for the ST7796S LCD Controller
 *
 * Copyright (c) 2023 Alan Ma
 * Copyright (c) 2014 Petr Olivka
 * Copyright (c) 2013 Noralf Tronnes
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <video/mipi_display.h>

#include "fbtft.h"

#define DRVNAME	"fb_st7796s"
#define WIDTH		480
#define HEIGHT		320

static const s16 default_init_sequence[] = {
	-1, 0xC0, 0x0C, 0x02,
	-1, 0xC1, 0x44,
	-1, 0xC5, 0x00, 0x16, 0x80,
	-1, 0x36, 0x28,
	/* Interface Mode Control */
	-1, 0x3A, 0x55,
	-1, 0XB0, 0x00,
	/* Frame rate 70HZ */
	-1, 0xB1, 0xB0,
	-1, 0xB4, 0x02,
	/* RGB/MCU Interface Control */
	-1, 0xB6, 0x02, 0x02,
	-1, 0xE9, 0x00,
	-1, 0XF7, 0xA9, 0x51, 0x2C, 0x82,
	/* SLP_OUT - Sleep out */
	-1, MIPI_DCS_EXIT_SLEEP_MODE,
	-2, 50,
	/* DISP_ON */
	-1, MIPI_DCS_SET_DISPLAY_ON,
	-3
};

static void set_addr_win(struct fbtft_par *par, int xs, int ys, int xe, int ye)
{
	write_reg(par, MIPI_DCS_SET_COLUMN_ADDRESS,
		  xs >> 8, xs & 0xff, xe >> 8, xe & 0xff);

	write_reg(par, MIPI_DCS_SET_PAGE_ADDRESS,
		  ys >> 8, ys & 0xff, ye >> 8, ye & 0xff);

	write_reg(par, MIPI_DCS_WRITE_MEMORY_START);
}

static int set_var(struct fbtft_par *par)
{
	switch (par->info->var.rotate) {
	case 270:
		write_reg(par, MIPI_DCS_SET_ADDRESS_MODE,
			  0x40 | (par->bgr << 3));
		break;
	case 180:
		write_reg(par, MIPI_DCS_SET_ADDRESS_MODE,
			  0xE0 | (par->bgr << 3));
		break;
	case 90:
		write_reg(par, MIPI_DCS_SET_ADDRESS_MODE,
			  0x80 | (par->bgr << 3));
		break;
	default:
		write_reg(par, MIPI_DCS_SET_ADDRESS_MODE,
			  0x20 | (par->bgr << 3));
		break;
	}

	return 0;
}

static struct fbtft_display display = {
	.regwidth = 8,
	.width = WIDTH,
	.height = HEIGHT,
	.init_sequence = default_init_sequence,
	.fbtftops = {
		.set_addr_win = set_addr_win,
		.set_var = set_var,
	},
};

FBTFT_REGISTER_DRIVER(DRVNAME, "sitronix,st7796s", &display);

MODULE_ALIAS("spi:" DRVNAME);
MODULE_ALIAS("platform:" DRVNAME);
MODULE_ALIAS("spi:st7796s");
MODULE_ALIAS("platform:st7796s");

MODULE_DESCRIPTION("FB driver for the ST7796S LCD Controller");
MODULE_AUTHOR("Alan Ma");
MODULE_LICENSE("GPL");
