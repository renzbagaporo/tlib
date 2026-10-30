/*
 * RP2350 GPIO coprocessor port (datasheet section 3.6.1).
 *
 * This library is licensed under the GNU Lesser General Public License,
 * version 2 or later. See LICENSE for the full license.
 */
#include "cpu.h"
#include "helper.h"
#include "arch_callbacks.h"

#ifdef TARGET_PROTO_ARM_M
uint32_t HELPER(rp2350_gpio)(CPUState *env, uint32_t insn)
{
    unsigned access = extract32(env->v7m.cpacr[env->secure], 0, 2);
    unsigned rt = extract32(insn, 12, 4);
    unsigned rt2 = extract32(insn, 16, 4);
    unsigned crm = extract32(insn, 0, 4);
    unsigned group = crm >> 2;
    unsigned opc;
    bool read = extract32(insn, 20, 1);
    bool pair = (insn & 0x0fe00000) == 0x0c400000;
    bool transfer = (insn & 0x0f000010) == 0x0e000010;
    uint64_t value = 0;
    uint64_t mask = UINT64_MAX;
    uint64_t result = 0;
    unsigned shift = (crm & 1) * 32;

    if(!env->rp2350_dcp_enabled || access == 0 || access == 2 ||
       (access == 1 && in_user_mode(env)) ||
       (!env->secure && !(env->v7m.nsacr & 1))) {
        return EXCP_NOCP;
    }
    /* The tables specify the ordinary MCR/MRC/MCRR/MRRC encodings only. */
    if(extract32(insn, 28, 1) || extract32(insn, 8, 4) != 0) return EXCP_UDEF;
    if(pair) {
        opc = extract32(insn, 4, 4);
        if(rt == 15 || rt2 == 15 || rt == rt2 || (crm != 0 && crm != 4 && crm != 8)) return EXCP_UDEF;
        if(read) {
            if(opc != 0) return EXCP_UDEF;
        } else {
            if(group > 1 || opc > 11) return EXCP_UDEF;
            value = ((uint64_t)env->regs[rt2] << 32) | env->regs[rt];
            if(opc >= 8) {
                /* Only register indices 0 and 1 exist on RP2350. */
                if(env->regs[rt2] > 1) return EXCP_UDEF;
                shift = env->regs[rt2] * 32;
                value = (uint64_t)env->regs[rt] << shift;
                mask = UINT64_C(0xffffffff) << shift;
            } else if(opc >= 4) {
                if(env->regs[rt] >= 64) return EXCP_UDEF;
                mask = UINT64_C(1) << env->regs[rt];
                value = (env->regs[rt2] & 1) ? mask : 0;
                if(opc == 4) {
                    /* Bit put is a masked write, conditional aliases use data. */
                    opc = 0;
                }
            }
        }
    } else if(transfer) {
        opc = extract32(insn, 21, 3);
        if(rt2 != 0 || extract32(insn, 5, 3) != 0 || (crm & 2) || group > 2) return EXCP_UDEF;
        if(read) {
            if(opc != 0) return EXCP_UDEF;
        } else {
            if(rt == 15 || group > 1 || opc == 4) return EXCP_UDEF;
            value = (uint64_t)env->regs[rt] << shift;
            mask = UINT64_C(0xffffffff) << shift;
            if(opc >= 5) {
                if((crm & 1) || env->regs[rt] >= 64) return EXCP_UDEF;
                value = mask = UINT64_C(1) << env->regs[rt];
            }
        }
    } else {
        return EXCP_UDEF;
    }

    /* The callback is one shared SIO transaction, including paired accesses.
     * Return before raising any fault: never unwind across a managed callback. */
    uint32_t control = (group << 2) | (read ? 16 : 0) | (opc & 3);
    if(!tlib_rp2350_gpio_access(control, value, mask, &result)) return EXCP_NOCP;
    if(read) {
        if(pair) {
            env->regs[rt] = (uint32_t)result;
            env->regs[rt2] = result >> 32;
        } else if(rt == 15) {
            cpsr_write(env, (uint32_t)(result >> shift), CPSR_NZCV);
        } else {
            env->regs[rt] = (uint32_t)(result >> shift);
        }
    }
    return 0;
}
#endif
