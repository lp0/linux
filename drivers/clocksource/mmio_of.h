/*
 * Generic MMIO clocksource/clockevent support (Device Tree)
 *
 * Copyright 2012 Simon Arlott
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef _MMIO_OF_H
#define _MMIO_OF_H

#include <linux/clocksource.h>
#include <linux/clockchips.h>

struct mmio_of_clock {
	bool system;
	u32 rating;
	u32 freq;
	u32 size;
	u32 invert;

	struct resource rvalue;
	void __iomem *value;
	struct resource rcontrol;
	void __iomem *control;

	cycle_t (*read)(struct clocksource *);
	struct clocksource_mmio *cs;
};

struct mmio_of_timer {
	u32 rating;
	u32 cpu;
	u32 index;
	u32 min_delta;
	u32 max_delta;

	struct resource rcompare;
	void __iomem *compare;

	struct mmio_of_clock *clock;
	int irq;

	bool (*get)(struct mmio_of_timer *);
	int (*set)(struct mmio_of_timer *, unsigned int);
	void (*clear)(struct mmio_of_timer *);

	/* this is a fake clocksource_mmio that is
	 * compatible with its read functions
	 */
	struct clocksource_mmio cs;
	struct clock_event_device ce;
};

extern void clocksource_mmio_of_init(void);
extern void clockevent_mmio_of_init(void);

#endif
