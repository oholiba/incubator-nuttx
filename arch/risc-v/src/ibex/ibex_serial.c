/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>

#include <arch/board/board.h>

#include "riscv_arch.h"
#include "riscv_internal.h"

#include "ibex_config.h"
#include "chip.h"
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
#  endif
#endif /* HAVE_CONSOLE */

/* If we are not using the serial driver for the console, then we still must
 * provide some minimal implementation of up_putc.
 */

#ifdef USE_SERIALDRIVER

/* Which UART with be tty0/console and which tty1?  The console will always
 * be ttyS0.  If there is no console then will use the lowest numbered UART.
 */

#ifdef HAVE_SERIAL_CONSOLE
#  if defined(CONFIG_UART0_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart0port     /* UART0 is console */
#    define TTYS0_DEV       g_uart0port     /* UART0 is ttyS0 */
#    undef  TTYS1_DEV                       /* No ttyS1 */
#    define SERIAL_CONSOLE  1
#  else
#    error "I'm confused... Do we have a serial console or not?"
#  endif
#else
#  undef  CONSOLE_DEV                        /* No console */
#  undef  CONFIG_UART0_SERIAL_CONSOLE
#  if defined(CONFIG_IBEX_UART0)
#    define TTYS0_DEV       g_uart0port     /* UART0 is ttyS0 */
#    undef  TTYS1_DEV                       /* No ttyS1 */
#    define SERIAL_CONSOLE  1
#  else
#    undef  TTYS0_DEV
#    undef  TTYS1_DEV
#  endif
#endif

/* Common initialization logic will not not know that the all of the UARTs
 * have been disabled.  So, as a result, we may still have to provide
 * stub implementations of riscv_earlyserialinit(), riscv_serialinit(), and
 * up_putc().
 */

#ifdef HAVE_UART_DEVICE

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  uintptr_t uartbase; /* Base address of UART registers */
  uint8_t   irq_RX;      /* RX IRQ associated with this UART */
  uint8_t   irq_TX;      /* TX IRQ associated with this UART */
  uint8_t   im;       /* Interrupt mask state */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Low-level helpers */

static uint32_t up_serialin(struct up_dev_s *priv, int offset);
static void up_serialout(struct up_dev_s *priv, int offset, uint32_t value);
static void up_restoreuartint(struct up_dev_s *priv, uint8_t im);
static void up_disableuartint(struct up_dev_s *priv, uint8_t *im);

/* Serial driver methods */

static int  up_setup(struct uart_dev_s *dev);
static void up_shutdown(struct uart_dev_s *dev);
static int  up_attach(struct uart_dev_s *dev);
static void up_detach(struct uart_dev_s *dev);
static int  up_interrupt(int irq, void *context, FAR void *arg);
static int  up_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  up_receive(struct uart_dev_s *dev, unsigned int *status);
static void up_rxint(struct uart_dev_s *dev, bool enable);
static bool up_rxavailable(struct uart_dev_s *dev);
static void up_send(struct uart_dev_s *dev, int ch);
static void up_txint(struct uart_dev_s *dev, bool enable);
static bool up_txready(struct uart_dev_s *dev);
static bool up_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_uart_ops =
{
  .setup          = up_setup,
  .shutdown       = up_shutdown,
  .attach         = up_attach,
  .detach         = up_detach,
  .ioctl          = up_ioctl,
  .receive        = up_receive,
  .rxint          = up_rxint,
  .rxavailable    = up_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = NULL,
#endif
  .send           = up_send,
  .txint          = up_txint,
  .txready        = up_txready,
  .txempty        = up_txempty,
};

/* I/O buffers */

#ifdef CONFIG_IBEX_UART0
static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];
#endif

#ifdef CONFIG_IBEX_UART0
static struct up_dev_s g_uart0priv =
{
  .uartbase  = IBEX_UART0_BASE,
  .irq_RX       = IBEX_IRQ_UART0_RX,
  .irq_TX       = IBEX_IRQ_UART0_TX,
};

static uart_dev_t g_uart0port =
{
#if SERIAL_CONSOLE == 1
  .isconsole = 1,
#endif
  .recv      =
  {
    .size    = CONFIG_UART0_RXBUFSIZE,
    .buffer  = g_uart0rxbuffer,
  },
  .xmit      =
  {
    .size    = CONFIG_UART0_TXBUFSIZE,
    .buffer  = g_uart0txbuffer,
  },
  .ops       = &g_uart_ops,
  .priv      = &g_uart0priv,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_serialin
 ****************************************************************************/

static uint32_t up_serialin(struct up_dev_s *priv, int offset)
{
	return getreg32(priv->uartbase + offset);
}

/****************************************************************************
 * Name: up_serialout
 ****************************************************************************/

static void up_serialout(struct up_dev_s *priv, int offset, uint32_t value)
{
	putreg32(value, priv->uartbase + offset);
}

/****************************************************************************
 * Name: up_restoreuartint
 ****************************************************************************/

static void up_restoreuartint(struct up_dev_s *priv, uint8_t im)
{
}

/****************************************************************************
 * Name: up_disableuartint
 ****************************************************************************/

static void up_disableuartint(struct up_dev_s *priv, uint8_t *im)
{
}

/****************************************************************************
 * Name: up_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static int up_setup(struct uart_dev_s *dev)
{
	// UART can be configured from hdl code
	return 0;
}

/****************************************************************************
 * Name: up_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void up_shutdown(struct uart_dev_s *dev)
{
	// disable interrupts
	up_rxint(dev, false);
	up_txint(dev, false);
}

/****************************************************************************
 * Name: up_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.  This method
 *   is called when the serial port is opened.  Normally, this is just after
 *   the the setup() method is called, however, the serial console may
 *   operate in a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled by the attach method (unless the
 *   hardware supports multiple levels of interrupt enabling).  The RX and TX
 *   interrupts are not enabled until the txint() and rxint() are called.
 *
 ****************************************************************************/

static int up_attach(struct uart_dev_s *dev)
{
	struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

	irq_attach(priv->irq_RX, up_interrupt, dev);
	irq_attach(priv->irq_TX, up_interrupt, dev);

	return 0;
}

/****************************************************************************
 * Name: up_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The
 *   exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void up_detach(struct uart_dev_s *dev)
{
	struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

	up_disable_irq((priv->irq_RX) - IBEX_IRQ_ASYNC);
	up_disable_irq((priv->irq_TX) - IBEX_IRQ_ASYNC);

	irq_detach(priv->irq_RX);
	irq_detach(priv->irq_TX);
}

/****************************************************************************
 * Name: up_interrupt
 *
 * Description:
 *   This is the UART interrupt handler.  It will be invoked when an
 *   interrupt received on the 'irq'  It should call uart_transmitchars or
 *   uart_receivechar to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'irq' number into the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int up_interrupt(int irq, void *context, FAR void *arg)
{
	struct uart_dev_s *dev = (struct uart_dev_s *)arg;
	struct up_dev_s   *priv;
	uint32_t           status;
	int                passes;

	DEBUGASSERT(dev != NULL && dev->priv != NULL);
	priv = (struct up_dev_s *)dev->priv;

	/* Loop until there are no characters to be transferred or,
	 * until we have been looping for a long time.
	 */

	/*for (passes = 0; passes < 256; passes++)
	{
		 Retrieve interrupt pending status 
		
		asm volatile("csrr %0, mip" : "=r" (status):);

		// RX interrupt
		if ((status & (1 << 16))
		{
			uart_recvchars(dev);
		}
		else if ((status & (1 << 17)) // TX interrupt
		{
			uart_xmitchars(dev);
		}
		else
		{
			break;
		}	
	}*/

	// RX interrupt
	if (irq == IBEX_IRQ_UART0_RX)
	{
		uart_recvchars(dev);
	}

	// TX interrupt
	if (irq == IBEX_IRQ_UART0_TX)
	{
		uart_xmitchars(dev);
	}
		
	return 0;
}

/****************************************************************************
 * Name: up_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int up_ioctl(struct file *filep, int cmd, unsigned long arg)
{
	return -ENOTTY;
}

/****************************************************************************
 * Name: up_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int up_receive(struct uart_dev_s *dev, unsigned int *status)
{
	struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

	int rxdata = (int)up_serialin(priv, UART_DATA_OFFSET);
	
	return rxdata;
}

/****************************************************************************
 * Name: up_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void up_rxint(struct uart_dev_s *dev, bool enable)
{
	if (enable)
	{
		up_enable_irq(IBEX_IRQ_UART0_RX - IBEX_IRQ_ASYNC);	
	}
	else
	{
		up_disable_irq(IBEX_IRQ_UART0_RX - IBEX_IRQ_ASYNC);	
	}
}

/****************************************************************************
 * Name: up_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/

static bool up_rxavailable(struct uart_dev_s *dev)
{  
	struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
	
	// 1.
	// problem: interrupt gets reset when uart data is read
	// return (up_serialin(priv, UART_DATA_OFFSET) >= 0);
	
	// 2.
	// problem: mip only works when interrupts is enabled
	
	uint32_t           status;

	asm volatile("csrr %0, mip" : "=r" (status):);

	// RX interrupt
	if (status & (1 << 16))
	{
		return true;
	}

	return false;
}

/****************************************************************************
 * Name: up_send
 *
 * Description:
 *   This method will send one byte on the UART.
 *
 ****************************************************************************/

static void up_send(struct uart_dev_s *dev, int ch)
{
	struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
	up_serialout(priv, UART_DATA_OFFSET, (uint32_t)ch);
}

/****************************************************************************
 * Name: up_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void up_txint(struct uart_dev_s *dev, bool enable)
{
	if (enable)
	{
		up_enable_irq(IBEX_IRQ_UART0_TX - IBEX_IRQ_ASYNC);	
	}
	else
	{
		up_disable_irq(IBEX_IRQ_UART0_TX - IBEX_IRQ_ASYNC);	
	}
}

/****************************************************************************
 * Name: up_txready
 *
 * Description:
 *   Return true if the tranmsit data register is not full
 *
 ****************************************************************************/

static bool up_txready(struct uart_dev_s *dev)
{
	struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
	
	return (up_serialin(priv, UART_STATUS_OFFSET) == 0);
}

/****************************************************************************
 * Name: up_txempty
 *
 * Description:
 *   Return true if the tranmsit data register is empty
 *
 ****************************************************************************/

static bool up_txempty(struct uart_dev_s *dev)
{
	struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
	
	return (up_serialin(priv, UART_STATUS_OFFSET) == 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT

/****************************************************************************
 * Name: riscv_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before riscv_serialinit.  NOTE:  This function depends on GPIO pin
 *   configuration performed in up_consoleinit() and main clock
 *   initialization performed in up_clkinitialize().
 *
 ****************************************************************************/

void riscv_earlyserialinit(void)
{  
	/* Disable interrupts from all UARTS.  The console is enabled in
	 * * litex_consoleinit().
	 * */
	
	up_disable_irq(IBEX_IRQ_UART0_RX - IBEX_IRQ_ASYNC);	
	up_disable_irq(IBEX_IRQ_UART0_TX - IBEX_IRQ_ASYNC);	
 
       	/* Configuration whichever one is the console */

#ifdef HAVE_SERIAL_CONSOLE
	CONSOLE_DEV.isconsole = true;
	up_setup(&CONSOLE_DEV);
#endif
}
#endif

/****************************************************************************
 * Name: riscv_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that riscv_earlyserialinit was called previously.
 *
 ****************************************************************************/

void riscv_serialinit(void)
{
	/* Register the console */

#ifdef HAVE_SERIAL_CONSOLE
	uart_register("/dev/console", &CONSOLE_DEV);
#endif

	/* Register all UARTs */

	uart_register("/dev/ttyS0", &TTYS0_DEV);
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug  writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
#ifdef HAVE_SERIAL_CONSOLE
  riscv_lowputc(ch);
#endif
  return ch;
}

/****************************************************************************
 * Name: riscv_earlyserialinit, riscv_serialinit, and up_putc
 *
 * Description:
 *   stubs that may be needed.  These stubs would be used if all UARTs are
 *   disabled.  In that case, the logic in common/up_initialize() is not
 *   smart enough to know that there are not UARTs and will still expect
 *   these interfaces to be provided.
 *
 ****************************************************************************/

#else /* HAVE_UART_DEVICE */
void riscv_earlyserialinit(void)
{
}

void riscv_serialinit(void)
{
}

int up_putc(int ch)
{
  return ch;
}

#endif /* HAVE_UART_DEVICE */
#else /* USE_SERIALDRIVER */

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
	return 0;
}

#endif /* USE_SERIALDRIVER */
