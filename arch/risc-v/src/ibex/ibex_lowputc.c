/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <arch/board/board.h>

#include "riscv_internal.h"
#include "riscv_arch.h"

#include "ibex_config.h"
#include "hardware/ibex_memorymap.h"
#include "hardware/ibex_uart.h"
#include "ibex_clockconfig.h"
#include "ibex.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Select UART parameters for the selected console */

#ifdef HAVE_SERIAL_CONSOLE
#  if defined(CONFIG_UART0_SERIAL_CONSOLE)
#    define IBEX_CONSOLE_BASE        IBEX_UART0_BASE
#    define IBEX_CONSOLE_BAUD        CONFIG_UART0_BAUD
#    define IBEX_CONSOLE_BITS        CONFIG_UART0_BITS
#    define IBEX_CONSOLE_PARITY      CONFIG_UART0_PARITY
#    define IBEX_CONSOLE_2STOP       CONFIG_UART0_2STOP
#    define IBEX_CONSOLE_TX          GPIO_UART0_TX
#    define IBEX_CONSOLE_RX          GPIO_UART0_RX
#    define HAVE_UART
#  elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#    define IBEX_CONSOLE_BASE        IBEX_UART1_BASE
#    define IBEX_CONSOLE_BAUD        CONFIG_UART1_BAUD
#    define IBEX_CONSOLE_BITS        CONFIG_UART1_BITS
#    define IBEX_CONSOLE_PARITY      CONFIG_UART1_PARITY
#    define IBEX_CONSOLE_2STOP       CONFIG_UART1_2STOP
#    define IBEX_CONSOLE_TX          GPIO_UART1_TX
#    define IBEX_CONSOLE_RX          GPIO_UART1_RX
#    define HAVE_UART
#  endif
#endif /* HAVE_CONSOLE */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: riscv_lowputc
 *
 * Description:
 *   Output one byte on the serial console
 *
 ****************************************************************************/

void riscv_lowputc(char ch)
{
#ifdef HAVE_SERIAL_CONSOLE
  
  // wait until transmitter ready:
  while (getreg8(IBEX_CONSOLE_BASE + UART_STATUS_OFFSET) & 1)
        ;

  putreg8(ch, IBEX_CONSOLE_BASE + UART_DATA_OFFSET);
  
#endif /* HAVE_CONSOLE */
}

/****************************************************************************
 * Name: litex_lowsetup
 *
 * Description:
 *   This performs basic initialization of the UART used for the serial
 *   console.  Its purpose is to get the console output available as soon
 *   as possible.
 *
 ****************************************************************************/

void ibex_lowsetup(void)
{
#if defined(HAVE_UART)

  /* Enable and configure the selected console device */

#if defined(HAVE_SERIAL_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)

#endif /* HAVE_SERIAL_CONSOLE && !CONFIG_SUPPRESS_UART_CONFIG */
#endif /* HAVE_UART */
}
