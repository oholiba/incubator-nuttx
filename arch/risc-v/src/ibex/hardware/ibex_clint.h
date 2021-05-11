/****************************************************************************
 * arch/risc-v/src/litex/hardware/litex_clint.h
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

#ifndef __ARCH_RISCV_SRC_IBEX_HARDWARE_IBEX_CLINT_H
#define __ARCH_RISCV_SRC_IBEX_HARDWARE_IBEX_CLINT_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IBEX_CLINT_LATCH      (IBEX_CPUTIMER_BASE)
#define IBEX_CLINT_MTIME      (IBEX_CPUTIMER_BASE + 0x04)
#define IBEX_CLINT_MTIMECMP   (IBEX_CPUTIMER_BASE + 0x0C)

#endif /* __ARCH_RISCV_SRC_IBEX_HARDWARE_IBEX_CLINT_H */
