/*
 * Copyright (C) 2010 Broadcom
 * Copyright 2012 Simon Arlott
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _BCM_BELL_H
#define _BCM_BELL_H

struct bcm_bell;

/*
 * @node: calling node
 * @pbell: property name for bell phandle
 *
 * Returns an error pointer if not found, otherwise holds a reference to the
 * device that must be put back with bcm_bell_put.
 */
extern struct bcm_bell *bcm_bell_get(struct device_node *node,
	const char *pbell);
extern void bcm_bell_put(struct bcm_bell *bell);

extern int bcm_bell_read(struct bcm_bell *bell, void (*handler)(void *),
	void *data);
extern int bcm_bell_write(struct bcm_bell *bell);

#endif
