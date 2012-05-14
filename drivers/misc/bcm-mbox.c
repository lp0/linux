/*
 * Copyright (C) 2010 Broadcom
 * Copyright 2012 Simon Arlott
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This device provides a shared mechanism for writing to the mailbox
 * shared between the ARM and the VideoCore processor.
 *
 *
 * Use MBOX_STA_IRQ_DATA to get continuous interrupts when we have messages
 * to be read.
 *
 * Messages are received by the interrupt handler which adds them to a
 * per-channel inbox and then incrementing a semaphore.
 *
 * Messages are read by decrementing the per-channel semaphore and taking
 * them from the inbox list.
 *
 *
 * Use MBOX_STA_IRQ_WSPACE to get continuous interrupts when we can send
 * messages.
 *
 * Messages are written by adding them to the outbox list and enabling
 * the send interrupt (if not already enabled).
 *
 * Messages are sent by the interrupt handler which removes them from
 * the outbox and then disables the send interrupt (if there are no more
 * messages).
 */

#include <linux/device.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/semaphore.h>
#include <linux/slab.h>

#include "bcm-mbox.h"

#define MODULE_NAME "bcm-mbox"

/* offsets from a mail box base address */
#define MBOX_RW		0x00	/* read/write next 4 words */
#define MBOX_POL	0x10	/* read without popping the fifo */
#define MBOX_SND	0x14	/* sender ID (bottom two bits) */
#define MBOX_STA	0x18	/* status */
#define MBOX_CNF	0x1c	/* configuration */
#define MBOX_OFF0	0x00	/* mailbox 0 */
#define MBOX_OFF1	0x20	/* mailbox 1 */
#define MBOX_REGSZ	0x40	/* total size for two mailboxes */
#define MAX_CHANS	0x0f

#define MBOX_MSG(chan, data28)	(((data28) & ~0xf) | ((chan) & 0xf))
#define MBOX_CHAN(msg)		((msg) & 0xf)
#define MBOX_DATA28(msg)	((msg) & ~0xf)

#define MBOX_STA_FULL		0x80000000
#define MBOX_STA_EMPTY		0x40000000
#define MBOX_STA_LEVEL		0x400000FF	/* Max. value depends on mailbox depth parameter */

#define MBOX_STA_IRQ_DATA	0x00000001	/* irq enable: this has data */
#define MBOX_STA_IRQ_WSPACE	0x00000002	/* irq enable: other has space */
#define MBOX_STA_IRQ_WEMPTY	0x00000004	/* irq enable: other is empty */
#define MBOX_STA_CLEAR_MSGS	0x00000008	/* clear msgs: write 1, then 0 */
#define MBOX_STA_HAS_DATA	0x00000010	/* irq pending: this has data */
#define MBOX_STA_HAS_WSPACE	0x00000020	/* irq pending: other has space */
#define MBOX_STA_HAS_WEMPTY	0x00000040	/* irq pending: other is empty */
/* Bit 7 is unused */
#define MBOX_ERR_EACCESS	0x00000100	/* error: non-owner read from mailbox */
#define MBOX_ERR_OVERFLOW	0x00000200	/* error: write to full mailbox */
#define MBOX_ERR_UNDERFLOW	0x00000400	/* error: read from empty mailbox */
#define MBOX_ERR_MASK		0x00000700

/* Mailboxes are used to configure the frame buffer on bcm2708 while the
 * console is locked, so this is the only way to debug it if it doesn't
 * work.
 *
 * Warning: if we cause a bug/oops/panic then nothing will show up on the
 * console unless you also disable the console lock.
 */
//#define MBOX_USE_EARLY_PRINTK
#ifdef MBOX_USE_EARLY_PRINTK
# undef dev_emerg
# undef dev_alert
# undef dev_crit
# undef dev_err
# undef dev_warn
# undef dev_notice
# undef dev_info
# undef dev_dbg
# undef dev_vdbg

# define dev_emerg(x, fmt, args...)  early_printk("%s: EMERG  " fmt, dev_name(x), ##args)
# define dev_alert(x, fmt, args...)  early_printk("%s: ALERT  " fmt, dev_name(x), ##args)
# define dev_crit(x, fmt, args...)   early_printk("%s: CRIT   " fmt, dev_name(x), ##args)
# define dev_err(x, fmt, args...)    early_printk("%s: ERROR  " fmt, dev_name(x), ##args)
# define dev_warn(x, fmt, args...)   early_printk("%s: WARN   " fmt, dev_name(x), ##args)
# define dev_notice(x, fmt, args...) early_printk("%s: NOTICE " fmt, dev_name(x), ##args)
# define dev_info(x, fmt, args...)   early_printk("%s: INFO   " fmt, dev_name(x), ##args)
# define dev_dbg(x, fmt, args...)    early_printk("%s: DEBUG  " fmt, dev_name(x), ##args)
# define dev_vdbg(x, fmt, args...)   early_printk("%s: TRACE  " fmt, dev_name(x), ##args)
#endif

struct bcm_mbox;

struct bcm_mbox_msg {
	struct list_head list;
	u32 val;
};

struct bcm_mbox_chan {
	struct list_head list;

	char *name;
	struct bcm_mbox *mbox;
	u32 index;

	/* used to lock inbox */
	spinlock_t lock;
	struct semaphore recv;
	struct list_head inbox;
};

struct bcm_mbox {
	struct list_head list;

	struct device *dev;
	struct resource res;
	void __iomem *status;
	void __iomem *config;
	void __iomem *read;
	void __iomem *write;

	/* use chan.name to check if the chan is valid */
	struct bcm_mbox_chan chan[MAX_CHANS+1];

	/* used to lock running, waiting, outbox
	 * and synchronise access to config
	 */
	spinlock_t lock;
	bool running;
	bool waiting;
	struct list_head outbox;
	
	u32 irq;
	struct irqaction irqaction;
};

/* Assume unique mailbox names across all devices,
 * if there's a conflict only the first name will
 * be used.
 */
DECLARE_RWSEM(devices);
LIST_HEAD(mboxes);
LIST_HEAD(chans);

static void bcm_mbox_free(struct bcm_mbox *mbox)
{
	struct bcm_mbox_msg *msg;
	struct bcm_mbox_msg *tmp;
	int i;

	for (i = 0; i < MAX_CHANS; i++) {
		kfree(mbox->chan[i].name);

		list_for_each_entry_safe(msg, tmp, &mbox->chan[i].inbox, list)
			list_del(&msg->list);
	}

	list_for_each_entry_safe(msg, tmp, &mbox->outbox, list)
		list_del(&msg->list);

	kfree(mbox);
}

static irqreturn_t bcm_mbox_irqhandler(int irq, void *dev_id)
{
	unsigned long flags;
	struct bcm_mbox *mbox = dev_id;
	bool active = true;
	int ret = IRQ_NONE;
	int status;

	spin_lock_irqsave(&mbox->lock, flags);
	if (!mbox->running)
		active = false;
	spin_unlock_irqrestore(&mbox->lock, flags);

	while (active) {
		status = readl(mbox->status);
		active = false;

		if (status & MBOX_ERR_MASK) {
			dev_err(mbox->dev, "mailbox error %08x\n",
				status & MBOX_ERR_MASK);

			/* clear it */
			spin_lock_irqsave(&mbox->lock, flags);
			if (mbox->running) {
				if (mbox->waiting) {
					writel(MBOX_STA_IRQ_DATA
						| MBOX_STA_IRQ_WSPACE,
						mbox->config);
				} else {
					writel(MBOX_STA_IRQ_DATA, mbox->config);
				}
			} else {
				writel(0, mbox->config);
			}
			spin_unlock_irqrestore(&mbox->lock, flags);
		}

		if (!(status & MBOX_STA_EMPTY)) {
			/* we have data to read */
			u32 val = readl(mbox->read);
			int index = MBOX_CHAN(val);
			struct bcm_mbox_chan *chan = &mbox->chan[index];

			if (chan->name) {
				struct bcm_mbox_msg *msg = kzalloc(sizeof(*msg), GFP_ATOMIC);

				if (msg == NULL) {
					dev_warn(mbox->dev, "out of memory: dropped message %08x\n", val);
					continue;
				}

				msg->val = val | 0xf;
				dev_dbg(mbox->dev, "received message %08x\n", val);

				spin_lock_irqsave(&chan->lock, flags);
				list_add_tail(&msg->list, &chan->inbox);
				spin_unlock_irqrestore(&chan->lock, flags);
				up(&chan->recv);
			} else {
				dev_warn(mbox->dev, "message received for unknown channel: %08x\n", val);
			}

			active = true;
			ret = IRQ_HANDLED;
		}

		if (!(status & MBOX_STA_FULL)) {
			/* we can send data */
			struct bcm_mbox_msg *msg;
			bool empty;

			spin_lock_irqsave(&mbox->lock, flags);
			if (list_empty(&mbox->outbox)) {
				msg = NULL;
				empty = true;
			} else {
				msg = list_first_entry(&mbox->outbox, struct bcm_mbox_msg, list);
				list_del(&msg->list);
				empty = list_empty(&mbox->outbox);
			}
			if (mbox->running && mbox->waiting && empty) {
				/* we don't want to send data, disable the interrupt */
				dev_dbg(mbox->dev, "disable send interrupt\n");
				writel(MBOX_STA_IRQ_DATA, mbox->config);

				mbox->waiting = false;
				ret = IRQ_HANDLED;
			}
			spin_unlock_irqrestore(&mbox->lock, flags);

			if (msg != NULL) {
				dev_dbg(mbox->dev, "sending message %08x\n", msg->val);
				writel(msg->val, mbox->write);
				kfree(msg);

				active = true;
				ret = IRQ_HANDLED;
			} else {
				dev_dbg(mbox->dev, "no message to send\n");
			}
		}
	}

	return ret;
}

static int __devinit bcm_mbox_probe(struct platform_device *of_dev)
{
	struct device_node *node = of_dev->dev.of_node;
	struct bcm_mbox *mbox = kzalloc(sizeof(*mbox), GFP_KERNEL);
	void __iomem *base;
	const char *access;
	void __iomem *r_off;
	void __iomem *w_off;
	const u32 *channels;
	int nr_chans;
	bool found_chan;
	int ret, i;

	if (mbox == NULL)
		return -ENOMEM;

	mbox->dev = &of_dev->dev;
	spin_lock_init(&mbox->lock);
	INIT_LIST_HEAD(&mbox->outbox);

	if (of_address_to_resource(node, 0, &mbox->res)) {
		ret = -EINVAL;
		goto err;
	}

	if (resource_size(&mbox->res) < MBOX_REGSZ) {
		dev_err(mbox->dev, "resource too small (%#x)\n",
			resource_size(&mbox->res));
		ret = -EINVAL;
		goto err;
	}

	if (!request_region(mbox->res.start, resource_size(&mbox->res),
			dev_name(mbox->dev))) {
		dev_err(mbox->dev, "resource %#lx unavailable\n",
			(unsigned long)mbox->res.start);
		ret = -EBUSY;
		goto err;
	}

	base = ioremap(mbox->res.start, resource_size(&mbox->res));
	if (!base) {
		dev_err(mbox->dev, "error mapping io at %#lx\n",
			(unsigned long)mbox->res.start);
		ret = -EIO;
		goto err;
	}

	if (of_property_read_string(node, "access", &access)) {
		dev_err(mbox->dev, "unable to read access configuration\n");
		ret = -EINVAL;
		goto err;
	}

	/* read this carefully so that the device tree format
	 * can be exended in the future
	 */
	if (access[0] == 'r' && access[1] == 'w') {
		r_off = base + MBOX_OFF0;
		w_off = base + MBOX_OFF1;
	} else if (access[0] == 'w' && access[1] == 'r') {
		w_off = base + MBOX_OFF0;
		r_off = base + MBOX_OFF1;
	} else {
		dev_err(mbox->dev, "invalid access configuration: %s\n", access);
		ret = -EINVAL;
		goto err;
	}

	mbox->status = r_off + MBOX_STA;
	mbox->config = r_off + MBOX_CNF;
	mbox->read = r_off + MBOX_RW;
	mbox->write = w_off + MBOX_RW;

	channels = of_get_property(node, "channels", &nr_chans);
	if (!channels) {
		nr_chans = 0;
	} else {
		nr_chans /= 4;
	}

	if (nr_chans > MAX_CHANS) {
		dev_warn(mbox->dev, "mailbox has too many channels (%d)\n",
			nr_chans);
		nr_chans = MAX_CHANS;
	}

	found_chan = false;
	for (i = 0; i < nr_chans; i++) {
		u32 chan = be32_to_cpup(&channels[i]);
		const char *name;

		if (chan > MAX_CHANS) {
			dev_warn(mbox->dev, "mailbox has invalid channel %d\n",
				chan);
			continue;
		}

		if (mbox->chan[chan].name) {
			dev_warn(mbox->dev, "mailbox has duplicate channel %d\n",
				chan);
			continue;
		}

		ret = of_property_read_string_index(node, "channel-names",
			i, &name);
		if (ret) {
			dev_warn(mbox->dev, "mailbox %d has no name\n", chan);
			continue;
		}

		mbox->chan[chan].name = kstrdup(name, GFP_KERNEL);
		if (!mbox->chan[chan].name) {
			ret = -ENOMEM;
			goto err;
		}

		mbox->chan[chan].mbox = mbox;
		mbox->chan[chan].index = chan;
		sema_init(&mbox->chan[chan].recv, 0);
		spin_lock_init(&mbox->chan[chan].lock);
		INIT_LIST_HEAD(&mbox->chan[chan].inbox);

		found_chan = true;
	}

	if (!found_chan) {
		dev_err(mbox->dev, "mailbox has no channels\n");
		ret = -EINVAL;
		goto err;
	}

	/* disable interrupts and clear the mailbox */
	writel(0, mbox->config);
	writel(MBOX_STA_CLEAR_MSGS, mbox->config);
	writel(0, mbox->config);

	/* register the interrupt handler */
	mbox->irq = irq_of_parse_and_map(node, 0);
	mbox->irqaction.name = dev_name(mbox->dev);
	mbox->irqaction.flags = IRQF_SHARED | IRQF_IRQPOLL;
	mbox->irqaction.dev_id = mbox;
	mbox->irqaction.handler = bcm_mbox_irqhandler;

	mbox->running = false;
	mbox->waiting = false;

	ret = setup_irq(mbox->irq, &mbox->irqaction);
	if (ret) {
		dev_err(mbox->dev, "unable to setup irq %d", mbox->irq);
		spin_unlock_irq(&mbox->lock);
		goto err;
	}

	/* enable the interrupt on data reception */
	spin_lock_irq(&mbox->lock);
	writel(MBOX_STA_IRQ_DATA, mbox->config);
	mbox->running = true;
	spin_unlock_irq(&mbox->lock);

	/* register the mailbox and channels */
	down_write(&devices);
	list_add_tail(&mbox->list, &mboxes);
	for (i = 0; i < MAX_CHANS; i++)
		if (mbox->chan[i].name)
			list_add_tail(&mbox->chan[i].list, &chans);

	dev_info(mbox->dev, "mailbox at MMIO %#lx (irq = %d)\n",
		(unsigned long)mbox->res.start, mbox->irq);
	for (i = 0; i < MAX_CHANS; i++)
		if (mbox->chan[i].name)
			dev_info(mbox->dev, "channel %d: %s\n", i,
				mbox->chan[i].name);
	platform_set_drvdata(of_dev, mbox);

	up_write(&devices);
	return 0;

err:
	bcm_mbox_free(mbox);
	return ret;
}

static int bcm_mbox_remove(struct platform_device *of_dev)
{
	struct bcm_mbox *mbox = platform_get_drvdata(of_dev);
	int i;

	/* unregister the mailbox and channels */
	down_write(&devices);
	list_del(&mbox->list);
	for (i = 0; i < MAX_CHANS; i++)
		if (mbox->chan[i].name)
			list_del(&mbox->chan[i].list);
	up_write(&devices);

	/* stop the interrupt handler */
	spin_lock_irq(&mbox->lock);
	mbox->running = false;
	dev_dbg(mbox->dev, "disable interrupts\n");
	writel(0, mbox->config);
	spin_unlock_irq(&mbox->lock);

	/* remove the interrupt handler */
	dev_dbg(mbox->dev, "waiting for irq handler\n");
	synchronize_irq(mbox->irq);
	dev_dbg(mbox->dev, "removing irq handler\n");
	remove_irq(mbox->irq, &mbox->irqaction);

	/* clear the mailbox */
	writel(MBOX_STA_CLEAR_MSGS, mbox->config);
	writel(0, mbox->config);

	release_region(mbox->res.start, resource_size(&mbox->res));
	bcm_mbox_free(mbox);
	platform_set_drvdata(of_dev, NULL);
	return 0;
}

static struct bcm_mbox_chan *bcm_mbox_find_channel(const char *name)
{
	struct bcm_mbox_chan *chan;

	list_for_each_entry(chan, &chans, list)
		if (!strcmp(chan->name, name))
			return chan;

	return NULL;
}

int bcm_mbox_write(const char *name, u32 data28)
{
	struct bcm_mbox_chan *chan;
	struct bcm_mbox *mbox;
	struct bcm_mbox_msg *msg;

	down_read(&devices);
	chan = bcm_mbox_find_channel(name);
	if (name == NULL) {
		up_read(&devices);
		return -EINVAL;
	}
	mbox = chan->mbox;

	msg = kmalloc(sizeof(*msg), GFP_KERNEL);
	if (msg == NULL) {
		up_read(&devices);
		return -ENOMEM;
	}

	msg->val = MBOX_MSG(chan->index, data28);
	dev_dbg(mbox->dev, "writing message %08x\n", msg->val);

	spin_lock_irq(&mbox->lock);
	list_add_tail(&msg->list, &mbox->outbox);
	if (!mbox->waiting) {
		/* enable the interrupt on write space available */
		dev_dbg(mbox->dev, "enable send interrupt\n");
		writel(MBOX_STA_IRQ_DATA | MBOX_STA_IRQ_WSPACE, mbox->config);

		mbox->waiting = true;
	} else {
		dev_dbg(mbox->dev, "send interrupt already enabled\n");
	}
	spin_unlock_irq(&mbox->lock);

	up_read(&devices);
	return 0;
}
EXPORT_SYMBOL_GPL(bcm_mbox_write);

int bcm_mbox_read(const char *name, u32 *data28)
{
	struct bcm_mbox_chan *chan;
	struct bcm_mbox *mbox;
	struct bcm_mbox_msg *msg;
	int ret;

	down_read(&devices);
	chan = bcm_mbox_find_channel(name);
	if (name == NULL) {
		up_read(&devices);
		return -EINVAL;
	}
	mbox = chan->mbox;

	dev_dbg(mbox->dev, "waiting for message from channel %d\n",
		chan->index);
	if (down_interruptible(&chan->recv)) {
		/* The wait was interrupted */
		ret = -EINTR;
		goto failed;
	}
	dev_dbg(mbox->dev, "message available for channel %d\n", chan->index);

	spin_lock_irq(&chan->lock);
	if (list_empty(&chan->inbox)) {
		msg = NULL;
	} else {
		msg = list_first_entry(&chan->inbox, struct bcm_mbox_msg, list);
		list_del(&msg->list);
	}
	spin_unlock_irq(&chan->lock);
	WARN_ON(msg == NULL);

	if (msg != NULL) {
		dev_dbg(mbox->dev, "read message %08x\n", msg->val);
		*data28 = MBOX_DATA28(msg->val);
		kfree(msg);
		ret = 0;
	} else {
		ret = -EIO;
	}

failed:
	up_read(&devices);
	return ret;
}
EXPORT_SYMBOL_GPL(bcm_mbox_read);

int bcm_mbox_call(const char *name, u32 out_data28, u32 *in_data28)
{
	struct bcm_mbox_chan *chan;
	struct bcm_mbox *mbox;
	struct bcm_mbox_msg *msg;
	int ret;

	down_read(&devices);
	chan = bcm_mbox_find_channel(name);
	if (name == NULL) {
		up_read(&devices);
		return -EINVAL;
	}
	mbox = chan->mbox;

	msg = kmalloc(sizeof(*msg), GFP_KERNEL);
	if (msg == NULL) {
		up_read(&devices);
		return -ENOMEM;
	}

	msg->val = MBOX_MSG(chan->index, out_data28);
	dev_dbg(mbox->dev, "writing message %08x\n", msg->val);

	spin_lock_irq(&mbox->lock);
	list_add_tail(&msg->list, &mbox->outbox);
	if (!mbox->waiting) {
		/* enable the interrupt on write space available */
		dev_dbg(mbox->dev, "enable send interrupt\n");
		writel(MBOX_STA_IRQ_DATA | MBOX_STA_IRQ_WSPACE, mbox->config);

		mbox->waiting = true;
	} else {
		dev_dbg(mbox->dev, "send interrupt already enabled\n");
	}
	spin_unlock_irq(&mbox->lock);

	dev_dbg(mbox->dev, "waiting for message from channel %d\n",
		chan->index);
	if (down_interruptible(&chan->recv)) {
		/* The wait was interrupted */
		ret = -EINTR;
		goto failed;
	}
	dev_dbg(mbox->dev, "message available for channel %d\n", chan->index);

	spin_lock_irq(&chan->lock);
	if (list_empty(&chan->inbox)) {
		msg = NULL;
	} else {
		msg = list_first_entry(&chan->inbox, struct bcm_mbox_msg, list);
		list_del(&msg->list);
	}
	spin_unlock_irq(&chan->lock);
	WARN_ON(msg == NULL);

	if (msg != NULL) {
		dev_dbg(mbox->dev, "read message %08x\n", msg->val);
		*in_data28 = MBOX_DATA28(msg->val);
		kfree(msg);
		ret = 0;
	} else {
		ret = -EIO;
	}

failed:
	up_read(&devices);
	return ret;
}
EXPORT_SYMBOL_GPL(bcm_mbox_call);

static struct of_device_id bcm_mbox_dt_match[] __devinitconst = {
	{ .compatible = "broadcom,bcm2708-mbox" },
	{}
};
MODULE_DEVICE_TABLE(of, bcm_mbox_dt_match);

static struct platform_driver bcm_mbox_driver = {
	.probe = bcm_mbox_probe,
	.remove = bcm_mbox_remove,
	.driver = {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = bcm_mbox_dt_match
	}
};

static int __init bcm_mbox_init(void)
{
	int ret;
	printk(KERN_INFO MODULE_NAME ": Broadcom VideoCore Mailbox driver\n");

	ret = platform_driver_register(&bcm_mbox_driver);
	if (ret)
		printk(KERN_ERR MODULE_NAME ": registration failed (%d)\n", ret);

	return ret;
}

static void __exit bcm_mbox_exit(void)
{
	platform_driver_unregister(&bcm_mbox_driver);
}

arch_initcall(bcm_mbox_init);
module_exit(bcm_mbox_exit);

MODULE_AUTHOR("Gray Girling, Simon Arlott");
MODULE_DESCRIPTION("ARM mailbox I/O to VideoCore processor");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:bcm-mbox");
