/*
 * Hazard3 local interrupt controller and M-mode PMP extension.
 * RP2350 datasheet sections 3.8.6.1 and 3.8.6.2.
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */
#include "cpu.h"
#include "hazard3.h"

static uint32_t modify(uint32_t old, uint32_t source, unsigned operation)
{
    return operation == 1 ? old | source : operation == 2 ? old & ~source : source;
}

static int next_irq(CPUState *env, unsigned minimum)
{
    uint64_t pending = (env->hazard3_irq_levels | env->hazard3_irq_force) & env->hazard3_irq_enable;
    int result = -1;
    for(unsigned irq = 0; irq < 52; irq++) {
        if((pending & (UINT64_C(1) << irq)) && env->hazard3_irq_priority[irq] >= minimum &&
           (result < 0 || env->hazard3_irq_priority[irq] > env->hazard3_irq_priority[result])) {
            result = irq;
        }
    }
    return result;
}

static void update_irq(CPUState *env)
{
    if(next_irq(env, (env->hazard3_meicontext >> 16) & 31) >= 0) {
        env->mip |= IRQ_ME;
        cpu_interrupt(env, CPU_INTERRUPT_HARD);
    } else {
        env->mip &= ~IRQ_ME;
    }
}

void hazard3_set_irq(CPUState *env, unsigned irq, bool value)
{
    if(irq >= 52) return;
    pthread_mutex_lock(&env->mip_lock);
    if(value) env->hazard3_irq_levels |= UINT64_C(1) << irq;
    else env->hazard3_irq_levels &= ~(UINT64_C(1) << irq);
    update_irq(env);
    pthread_mutex_unlock(&env->mip_lock);
}

bool hazard3_csr(CPUState *env, uint32_t csr, target_ulong source, unsigned operation,
                 bool read, bool write, target_ulong *previous)
{
    if(!env->hazard3_enabled || (csr != 0xbd0 && csr != 0xbf0 && (csr < 0xbe0 || csr > 0xbe5))) return false;
    pthread_mutex_lock(&env->mip_lock);
    *previous = 0;
    unsigned index = source & 31;
    switch(csr) {
    case 0xbd0:
        *previous = env->hazard3_pmp_m_mask;
        if(write) {
            env->hazard3_pmp_m_mask = modify(*previous, source, operation) & 0xff;
            tlb_flush(env, 1, true);
        }
        break;
    case 0xbf0:
        *previous = env->hazard3_msleep;
        if(write) env->hazard3_msleep = modify(*previous, source, operation) & 7;
        break;
    case 0xbe0: case 0xbe1: case 0xbe2: {
        uint64_t *array = csr == 0xbe0 ? &env->hazard3_irq_enable :
                         csr == 0xbe1 ? NULL : &env->hazard3_irq_force;
        uint64_t pending = env->hazard3_irq_levels | env->hazard3_irq_force;
        if(index < 4) {
            unsigned shift = index * 16;
            uint32_t old = ((array ? *array : pending) >> shift) & 0xffff;
            *previous = old << 16;
            if(write && array) {
                uint64_t mask = UINT64_C(0xffff) << shift;
                uint64_t value = modify(old, source >> 16, operation) & 0xffff;
                *array = ((*array & ~mask) | (value << shift)) & ((UINT64_C(1) << 52) - 1);
            }
        }
        break;
    }
    case 0xbe3:
        if(index < 13) {
            uint32_t old = 0;
            for(unsigned n = 0; n < 4; n++) old |= env->hazard3_irq_priority[index * 4 + n] << (n * 4);
            *previous = old << 16;
            if(write) {
                uint32_t value = modify(old, source >> 16, operation);
                for(unsigned n = 0; n < 4; n++) env->hazard3_irq_priority[index * 4 + n] = (value >> (n * 4)) & 15;
            }
        }
        break;
    case 0xbe4: {
        int irq = next_irq(env, (env->hazard3_meicontext >> 24) & 15);
        *previous = irq < 0 ? UINT32_C(0x80000000) : irq << 2;
        if(read && irq >= 0) env->hazard3_irq_force &= ~(UINT64_C(1) << irq);
        if(write && operation != 2 && (source & 1)) {
            env->hazard3_meicontext &= ~UINT32_C(0x001f9ff0);
            env->hazard3_meicontext |= irq < 0 ? UINT32_C(0x00108000) :
                ((env->hazard3_irq_priority[irq] + 1) << 16) | (irq << 4);
        }
        break;
    }
    case 0xbe5: {
        *previous = env->hazard3_meicontext;
        if(write && operation != 2 && (source & 2)) {
            *previous |= (env->mie & IRQ_MS) ? 4 : 0;
            *previous |= (env->mie & IRQ_MT) ? 8 : 0;
            env->mie &= ~(IRQ_MS | IRQ_MT);
        } else if(write && operation != 2) {
            if(source & 4) env->mie |= IRQ_MS;
            if(source & 8) env->mie |= IRQ_MT;
        }
        if(write) env->hazard3_meicontext = modify(*previous, source, operation) & UINT32_C(0xff1f9ff1);
        break;
    }
    }
    update_irq(env);
    pthread_mutex_unlock(&env->mip_lock);
    return true;
}

void hazard3_interrupt_enter(CPUState *env, bool external)
{
    if(!env->hazard3_enabled) return;
    pthread_mutex_lock(&env->mip_lock);
    uint32_t old = env->hazard3_meicontext;
    env->hazard3_meicontext &= ~1u;
    int irq = external ? next_irq(env, (old >> 16) & 31) : -1;
    if(irq >= 0) {
        env->hazard3_meicontext = (old & ~UINT32_C(0xff1f0001)) |
            ((old << 4) & UINT32_C(0xf0000000)) | ((old << 8) & UINT32_C(0x0f000000)) |
            ((env->hazard3_irq_priority[irq] + 1) << 16) | 1;
    }
    update_irq(env);
    pthread_mutex_unlock(&env->mip_lock);
}

void hazard3_interrupt_return(CPUState *env)
{
    if(!env->hazard3_enabled) return;
    pthread_mutex_lock(&env->mip_lock);
    uint32_t old = env->hazard3_meicontext;
    env->hazard3_meicontext &= ~1u;
    if(old & 1) {
        env->hazard3_meicontext = (old & ~UINT32_C(0xff1f0001)) |
            ((old >> 8) & UINT32_C(0x001f0000)) | ((old >> 4) & UINT32_C(0x0f000000));
    }
    update_irq(env);
    pthread_mutex_unlock(&env->mip_lock);
}
