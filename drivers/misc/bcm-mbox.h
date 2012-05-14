/*
 * Copyright (C) 2010 Broadcom
 * Copyright 2012 Simon Arlott
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _BCM_MBOX_H
#define _BCM_MBOX_H

extern int bcm_mbox_read(const char *name, u32 *data28);
extern int bcm_mbox_write(const char *name, u32 data28);
extern int bcm_mbox_call(const char *name, u32 out_data28, u32 *in_data28);

#endif
