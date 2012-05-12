#ifndef __ASM_ARM_HARDWARE_ARM_TIMER_H
#define __ASM_ARM_HARDWARE_ARM_TIMER_H

/*
 * ARM timer implementation, found in Integrator, Versatile, Realview
 * and Broadcom platforms.  Not all platforms support all registers and
 * bits in these registers, so we mark them with A for Integrator AP, C
 * for Integrator CP, V for Versatile, R for Realview and B for Broadcom.
 *
 * Integrator AP has 16-bit timers, Integrator CP, Versatile, Broadcom
 * and Realview can have 16-bit or 32-bit selectable via a bit in the
 * control register.
 *
 * Every SP804 contains two identical timers except Broadcom.
 */
#define TIMER_1_BASE	0x00			/* ABVCR */
#define TIMER_2_BASE	0x20			/* A VCR */

#define TIMER_LOAD	0x00			/* ABCVR rw */
#define TIMER_VALUE	0x04			/* ABCVR ro */
#define TIMER_CTRL	0x08			/* ABCVR rw */
#define TIMER_CTRL_ONESHOT	(1 << 0)	/*   CVR */
#define TIMER_CTRL_32BIT	(1 << 1)	/*  BCVR */
#define TIMER_CTRL_DIV1		(0 << 2)	/* ABCVR */
#define TIMER_CTRL_DIV16	(1 << 2)	/* ABCVR */
#define TIMER_CTRL_DIV256	(2 << 2)	/* ABCVR */
#define TIMER_CTRL_DIV1_TOO	(3 << 2)	/*  B    */
#define TIMER_CTRL_IE		(1 << 5)	/*    VR */
#define TIMER_CTRL_PERIODIC	(1 << 6)	/* A CVR */
#define TIMER_CTRL_ENABLE	(1 << 7)	/* A CVR */
#define TIMER_CTRL_DBGHLT	(1 << 8)	/*  B    */
#define TIMER_CTRL_COUNTER	(1 << 9)	/*  B    */
#define TIMER_CTRL_COUNTER_PRESCALE 0xFF0000	/*  B    (default 0x3E) */

#define TIMER_INTCLR	0x0c			/* ABCVR wo */
#define TIMER_RIS	0x10			/*  BCVR ro */
#define TIMER_MIS	0x14			/*  BCVR ro */
#define TIMER_BGLOAD	0x18			/*  BCVR rw */
#define TIMER_PREDIV	0x1C			/*  B    rw (default 0x7D) */
#define TIMER_COUNTER	0x20			/*  B    rw (free running counter) */

#endif
