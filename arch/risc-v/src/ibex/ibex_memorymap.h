/****************************************************************************
 * arch/risc-v/src/litex/litex_memorymap.h
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

#ifndef _ARCH_RISCV_SRC_IBEX_IBEX_MEMORYMAP_H
#define _ARCH_RISCV_SRC_IBEX_IBEX_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/ibex_memorymap.h"
#include "hardware/ibex_uart.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Idle thread stack starts from _ebss */

#ifndef __ASSEMBLY__
#define IBEX_IDLESTACK_BASE  (uint32_t)&_ebss
#else
#define IBEX_IDLESTACK_BASE  _ebss
#endif

#define IBEX_IDLESTACK_TOP  (IBEX_IDLESTACK_BASE + CONFIG_IDLETHREAD_STACKSIZE)

#endif /* _ARCH_RISCV_SRC_IBEX_IBEX_MEMORYMAP_H */
