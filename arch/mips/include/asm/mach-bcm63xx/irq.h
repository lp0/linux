#ifndef __ASM_MACH_BCM63XX_IRQ_H
#define __ASM_MACH_BCM63XX_IRQ_H

#ifdef CONFIG_BCM63XX_CPU_63168
# define NR_IRQS 144
#else
# define NR_IRQS 128
#endif
#define MIPS_CPU_IRQ_BASE 0

#endif
