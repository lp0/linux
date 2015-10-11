#ifndef BCM63XX_IRQ_H_
#define BCM63XX_IRQ_H_

#include <bcm63xx_cpu.h>

#define IRQ_INTERNAL_BASE		8
#ifdef CONFIG_BCM63XX_CPU_63168
# define IRQ_EXTERNAL_BASE		140
#else
# define IRQ_EXTERNAL_BASE		100
#endif
#define IRQ_EXT_0			(IRQ_EXTERNAL_BASE + 0)
#define IRQ_EXT_1			(IRQ_EXTERNAL_BASE + 1)
#define IRQ_EXT_2			(IRQ_EXTERNAL_BASE + 2)
#define IRQ_EXT_3			(IRQ_EXTERNAL_BASE + 3)

#endif /* ! BCM63XX_IRQ_H_ */
