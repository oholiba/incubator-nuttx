#ifndef __ARCH_RISCV_INCLUDE_IBEX_IRQ_H
#define __ARCH_RISCV_INCLUDE_IBEX_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Map RISC-V exception code to NuttX IRQ */

/* IRQ 0-15 : (exception:interrupt=0) */

#define IBEX_IRQ_IAFAULT       (1) /* Instruction Address Fault */
#define IBEX_IRQ_IINSTRUCTION  (2) /* Illegal Instruction */
#define IBEX_IRQ_BPOINT        (3) /* Break Point */
#define IBEX_IRQ_LAFAULT       (5) /* Load Access Fault */
#define IBEX_IRQ_SAFAULT       (7) /* Store Access Fault */
#define IBEX_IRQ_ECALLU        (8) /* Environment Call from U-mode */
                                    /* 9-10: Reserved */

#define IBEX_IRQ_ECALLM       (11) /* Environment Call from M-mode */
                                    /* 12-15: Reserved */

/* IRQ 32- : (async event:interrupt=1) */
// useful when dealing with interrupts (mcause only uses bits 4:0 to store exception code)
// look at ibeq_irq_dispatch

#define IBEX_IRQ_ASYNC        (32)
#define IBEX_IRQ_MSOFT    (IBEX_IRQ_ASYNC + 3)  /* Machine Software Int */
#define IBEX_IRQ_MTIMER   (IBEX_IRQ_ASYNC + 7)  /* Machine Timer Int */
#define IBEX_IRQ_MEXT     (IBEX_IRQ_ASYNC + 11) /* Machine External Int */

/* Machine Global External Interrupt */

#define IBEX_IRQ_UART0    (IBEX_IRQ_MEXT + 1)
#define IBEX_IRQ_TIMER0   (IBEX_IRQ_MEXT + 2)

/* Total number of IRQs */

#define NR_IRQS            (IBEX_IRQ_TIMER0 + 1)

#endif /* __ARCH_RISCV_INCLUDE_IBEX_IRQ_H */
