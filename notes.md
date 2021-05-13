Interrupts/Exception: risc v privileged seite 37 (mcause)
    - warum interrupts +16?

''So NuttX does not support processes.  NuttX will support an **MMU**
but it will not use the MMU to support processes. NuttX operates only in a
flat address space. (NuttX  will use the MMU to control the instruction and data caches and to support protected memory regions)'' (https://cwiki.apache.org/confluence/display/NUTTX/NuttX+Overview?preview=/139629402/140774631/nuttx-overview.pdf)


What is?: /* Colorize the interrupt stack */

  up_color_intstack();

  Description:
 *   Set the interrupt stack to a value so that later we can determine how
 *   much stack space was used by interrupt handling logic


 Betriebssystem: https://www.cs.cornell.edu/courses/cs3410/2019sp/schedule/slides/14-ecf-pre.pdf


 Interrupts: /sched/irq/irq.h


 syscall/README.txt
 ==================

 This directory supports a syscall layer from communication between a
 monolithic, kernel-mode NuttX kernel and a separately built, user-mode
 application set.

 With most MCUs, NuttX is built as a flat, single executable image
 containing the NuttX RTOS along with all application code.  The RTOS code
 and the application run in the same address space and at the same kernel-
 mode privileges.  In order to exploit security features of certain
 processors, an alternative build model is also supported:  NuttX can
 be built separately as a monolithic, kernel-mode module and the applications
 can be added as a separately built, user-mode module.

 nuttx overview: https://cwiki.apache.org/confluence/pages/viewpage.action?pageId=139629399&preview=/139629402/140774623/nuttx-3-archoverview.pdf

 ablauf boot (nach include/nuttx/board.h)(in klammer: litex specific):
 * startfile: arch/risc-v/src/litex/litex_head.S
 - (__start)
 - (__litex_start)
 - sched/init/nx_start
 - nx_bringup
 - up_initialize
 - board_early_initialize(optional) ("simple device drivers")
 - board_late_initialize(optional) ("initialize board-specific device drivers, waiting for events permissible")

 Interrupt handler bei litex:
    arch/risc-v/src/litex/litex_irq_dispatch.c

aus arch/risc-v/src/rv32im/riscv_initialstate.c:
Since various RISC-V platforms use different interrupt methodologies, the value of the interrupt context is part specific.


functions, laut include/nuttx/board.h **immer** notwendig zu implementieren:

int board_app_initialize(uintptr_t arg);

functions, laut include/nuttx/arch.h **immer** notwendig zu implementieren:

void up_initialize(void);
void up_systemreset(void) noreturn_function;
void up_idle(void);
void up_initial_state(FAR struct tcb_s *tcb);
int up_create_stack(FAR struct tcb_s *tcb, size_t stack_size, uint8_t ttype);
int up_use_stack(FAR struct tcb_s *tcb, FAR void *stack, size_t stack_size);
FAR void *up_stack_frame(FAR struct tcb_s *tcb, size_t frame_size);
void up_release_stack(FAR struct tcb_s *dtcb, uint8_t ttype);
void up_unblock_task(FAR struct tcb_s *tcb);
void up_block_task(FAR struct tcb_s *tcb, tstate_t task_state);
void up_release_pending(void);
void up_reprioritize_rtr(FAR struct tcb_s *tcb, uint8_t priority);
void up_exit() noreturn_function;
void up_assert(FAR const char *filename, int linenum);
void up_schedule_sigaction(FAR struct tcb_s *tcb, sig_deliver_t sigdeliver);

void up_allocate_heap(FAR void **heap_start, size_t *heap_size);

void up_irqinitialize(void);
bool up_interrupt_context(void);
irqstate_t up_irq_save(void);
void up_irq_restore(irqstate_t irqstate);
void up_timer_initialize(void);

int up_cpu_idlestack(int cpu, FAR struct tcb_s *tcb, size_t stack_size);

void irq_dispatch(int irq, FAR void *context); (exported by the OS)

int up_putc(int ch);
void up_puts(FAR const char *str);

**Fragen**
(- github: suche in files ab bestimmtem subfolder)

- riscv_pminitialize() wo implementiert? lösung: wird leer #defined (wo findet riscv power management statt? ->zB. riscv_idle.c führt kein pm aus)

- muss linker script rücksicht nehmen auf aufbau vom rest des Betriebssystems?

- wie hängt architecture api mit notwendigen files in include und src zusammen?

up_initialize:
- void riscv_addregion(void) in arch/risc-v/src/litex/litex_allocateheap.c leer?
--> auch riscv_serialinit? -> wird chip specific implementiert obwohl riscv_*?
    **achtung** manche sachen werden auch in src/rv32im/ oder src/rv64gc/ implementiert

[get|put]reg32: in arch/risc-v/src/common/riscv_arch.h
