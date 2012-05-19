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

#include <linux/err.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/spinlock.h>

#include "bcm2708.h"

#define MODULE_NAME "pinctrl-bcm2708"

#define TEST_REPROBE

static int __devinit bcm2708_pinctrl_probe(struct platform_device *pdev)
{
	struct bcm2708_pinctrl *pc = bcm2708_pinctrl_of_init(pdev);
	int ret;

	if (IS_ERR(pc))
		return PTR_ERR(pc);

	ret = bcm2708_pinctrl_sysfs_register(pc);
	if (ret) {
		bcm2708_pinctrl_of_free(pc);
		return ret;
	}

	spin_lock_irq(&pc->lock);
	pc->active = true;
	spin_unlock_irq(&pc->lock);

	dev_info(pc->dev, "%d pins at MMIO %#lx\n", PINS,
		(unsigned long)pc->res.start);
	
	platform_set_drvdata(pdev, pc);
	return 0;
}

#ifdef TEST_REPROBE
static int bcm2708_pinctrl_remove(struct platform_device *pdev)
#else
static int __devexit bcm2708_pinctrl_remove(struct platform_device *pdev)
#endif
{
	struct bcm2708_pinctrl *pc = platform_get_drvdata(pdev);

	/* make sure no sysfs activity can occur */
	spin_lock_irq(&pc->lock);
	pc->active = false;
	spin_unlock_irq(&pc->lock);

	bcm2708_pinctrl_sysfs_unregister(pc);
	bcm2708_pinctrl_of_free(pc);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

#ifdef TEST_REPROBE
static int __devinit bcm2708_pinctrl_reprobe(struct platform_device *pdev)
{
	int ret;

	dev_info(&pdev->dev, "probe 1");
	BUG_ON(platform_get_drvdata(pdev) != NULL);
	ret = bcm2708_pinctrl_probe(pdev);
	WARN_ON(ret);
	if (ret)
		return ret;

	dev_info(&pdev->dev, "remove 1");
	BUG_ON(platform_get_drvdata(pdev) == NULL);
	ret = bcm2708_pinctrl_remove(pdev);
	WARN_ON(ret);
	if (ret)
		return ret;

	dev_info(&pdev->dev, "probe 2");
	BUG_ON(platform_get_drvdata(pdev) != NULL);
	ret = bcm2708_pinctrl_probe(pdev);
	WARN_ON(ret);
	if (ret)
		return ret;

	dev_info(&pdev->dev, "remove 2");
	BUG_ON(platform_get_drvdata(pdev) == NULL);
	ret = bcm2708_pinctrl_remove(pdev);
	WARN_ON(ret);
	if (ret)
		return ret;

	dev_info(&pdev->dev, "probe 3");
	BUG_ON(platform_get_drvdata(pdev) != NULL);
	ret = bcm2708_pinctrl_probe(pdev);
	WARN_ON(ret);
	return ret;
}
#endif

static struct of_device_id bcm2708_pinctrl_match[] __devinitconst = {
	{ .compatible = "broadcom,bcm2708-pinctrl" },
	{}
};
MODULE_DEVICE_TABLE(of, bcm2708_pinctrl_match);

static struct platform_driver bcm2708_pinctrl_driver = {
#ifdef TEST_REPROBE
	.probe = bcm2708_pinctrl_reprobe,
#else
	.probe = bcm2708_pinctrl_probe,
#endif
	.remove = bcm2708_pinctrl_remove,
	.driver = {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = bcm2708_pinctrl_match
	}
};
module_platform_driver(bcm2708_pinctrl_driver);

MODULE_AUTHOR("Simon Arlott");
MODULE_DESCRIPTION("BCM2708 Pin control driver");
MODULE_LICENSE("GPL");
