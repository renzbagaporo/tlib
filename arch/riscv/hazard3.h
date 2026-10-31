/* Hazard3 extensions. SPDX-License-Identifier: LGPL-2.0-or-later */
#pragma once
#include "cpu.h"

bool hazard3_csr(CPUState *env, uint32_t csr, target_ulong source, unsigned operation,
                 bool read, bool write, target_ulong *previous);
void hazard3_set_irq(CPUState *env, unsigned irq, bool value);
void hazard3_interrupt_enter(CPUState *env, bool external);
void hazard3_interrupt_return(CPUState *env);
