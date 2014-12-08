/*
 * Copyright (c) 2013 Ash Charles <ash@gumstix.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */
#ifndef _FT5X0X_TS_H__
#define _FT5X0X_TS_H__

#define SCREEN_MAX_X    470
#define SCREEN_MAX_Y    275
#define PRESS_MAX       255

#define FT5X0X_NAME	"ft5x0x_ts"
#define FT5X0X_REG_POWER_MODE	0xa5
#define FT5X0X_REG_FIRMID	0xa6


struct ft5x0x_platform_data {
	unsigned int irq_gpio;
	unsigned int wake_gpio;
};

#endif /* _FT5X0X_TS_H__ */
