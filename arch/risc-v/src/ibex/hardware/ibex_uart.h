/****************************************************************************
 * arch/risc-v/src/litex/hardware/litex_uart.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef ARCH_RISCV_SRC_IBEX_CHIP_IBEX_UART_H
#define ARCH_RISCV_SRC_IBEX_CHIP_IBEX_UART_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define UART_DATA_OFFSET        0x00
#define UART_STATUS_OFFSET      0x04

#ifdef CONFIG_IBEX_UART0
#  define IBEX_UART0_DATA          (IBEX_UART0_BASE + UART_DATA_OFFSET)
#  define IBEX_UART0_STATUS        (IBEX_UART0_BASE + UART_STATUS_OFFSET)
#endif

#endif /* _ARCH_RISCV_SRC_IBEX_CHIP_IBEX_UART_H */
