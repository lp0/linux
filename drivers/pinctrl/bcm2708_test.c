/*
 * Copyright 2012 Simon Arlott
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * publishhed by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/consumer.h>

void bcm2708_pinctrl_test_dev(const char *name)
{
	struct device d;
	struct pinctrl *p;
	struct pinctrl_state *s;
	int ret;

	d.init_name = name;
	d.driver = NULL;
	d.bus = NULL;
	d.class = NULL;

	dev_info(&d, "testing pinctrl...");

	p = pinctrl_get(&d);
	if (IS_ERR(p)) {
		dev_warn(&d, "test p failed (%ld)\n", PTR_ERR(p));
		return;
	}

	s = pinctrl_lookup_state(p, PINCTRL_STATE_DEFAULT);
	if (IS_ERR(s)) {
		dev_warn(&d, "test s failed (%ld)\n", PTR_ERR(s));
		pinctrl_put(p);
		return;
	}

	ret = pinctrl_select_state(p, s);	
	if (ret < 0) {
		dev_warn(&d, "test r failed (%d)\n", ret);
		pinctrl_put(p);
		return;
	}

	dev_info(&d, "releasing\n");
	pinctrl_put(p);
	dev_info(&d, "tested pinctrl\n");
}

void bcm2708_pinctrl_test(void)
{
	bcm2708_pinctrl_test_dev("20000000.arm_jtag");
	bcm2708_pinctrl_test_dev("20000000.bsc0");
	bcm2708_pinctrl_test_dev("20000000.bsc1");
	bcm2708_pinctrl_test_dev("20000000.bscsl");
	bcm2708_pinctrl_test_dev("20000000.cam");
	bcm2708_pinctrl_test_dev("20000000.gpclk0");
	bcm2708_pinctrl_test_dev("20000000.gpclk1");
	bcm2708_pinctrl_test_dev("20000000.gpclk2");
	bcm2708_pinctrl_test_dev("20000000.led0");
	bcm2708_pinctrl_test_dev("20000000.pcm");
	bcm2708_pinctrl_test_dev("20000000.s5");
	bcm2708_pinctrl_test_dev("20000000.spi0");
	bcm2708_pinctrl_test_dev("20000000.spisl");
	//bcm2708_pinctrl_test_dev("20201000.uart0");
	//bcm2708_pinctrl_test_dev("20215040.uart1");
}
