/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <time.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <arch/board/board.h>

#include "riscv_arch.h"

#include "ibex.h"
#include "ibex_clockconfig.h"

/****************************************************************************
 * Name:  litex_timerisr
 ****************************************************************************/

static int ibex_timerisr(int irq, void *context, FAR void *arg)
{
  // clear interrupt flag
  // TODO: how set interrupt flags correctly?
  volatile long *const interrupt_flags = (volatile long *const)0xFF000200;
  *interrupt_flags = 1;/* Process timer interrupt */
  
  riscv_lowputc('A');

  nxsched_process_timer();
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_timer_initialize
 *
 * Description:
 *   This function is called during start-up to initialize
 *   the timer interrupt.
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  /* Attach timer interrupt handler */

  irq_attach(IBEX_IRQ_MTIMER, ibex_timerisr, NULL);

  /* And enable the timer interrupt */

  up_enable_irq(IBEX_IRQ_MTIMER);
}
