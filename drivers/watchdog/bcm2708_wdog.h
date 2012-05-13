/*
 * Broadcom BCM2708 watchdog driver.
 *
 * (c) Copyright 2010 Broadcom Europe Ltd
 * Copyright 2012 Simon Arlott
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 *
 * BCM2708 watchdog driver. Loosely based on wdt driver.
 */

#ifndef BCM2708_WDOG_H
#define BCM2708_WDOG_H

extern void bcm2708_wdog_restart(char str, const char *cmd);

#endif
