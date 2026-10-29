/*
 * RP2350 double-precision coprocessor control and direct mantissa transfers.
 *
 * Instruction encodings: Raspberry Pi pico-sdk hardware/dcp_instr.inc.S.
 * Semantics: RP2350 Datasheet section 3.6.2, tables 114-116.
 *
 * This library is licensed under the GNU Lesser General Public License,
 * version 2 or later. See LICENSE for the full license.
 */
#include <string.h>

#include "cpu.h"
#include "helper.h"

#ifdef TARGET_PROTO_ARM_M

uint32_t HELPER(rp2350_dcp)(CPUState *env, uint32_t insn)
{
    unsigned cp = extract32(insn, 8, 4);
    unsigned access = extract32(env->v7m.cpacr[env->secure], 2 * cp, 2);
    bool peek = extract32(insn, 28, 1);
    unsigned bank = cp == 5 ? M_REG_NS : env->secure;
    unsigned rt = extract32(insn, 12, 4);
    unsigned rt2 = extract32(insn, 16, 4);
    unsigned crm = extract32(insn, 0, 4);
    unsigned opc1;
    bool read = extract32(insn, 20, 1);

    if(!env->rp2350_dcp_enabled || (cp != 4 && cp != 5) ||
       (cp == 5 && !env->secure) ||
       (!env->secure && !(env->v7m.nsacr & (1u << cp))) ||
       access == 0 || access == 2 || (access == 1 && in_user_mode(env))) {
        return EXCP_NOCP;
    }

    /* MCRR / MRRC and their peek (MRRC2) variants. */
    if((insn & 0x0fe00000) == 0x0c400000) {
        opc1 = extract32(insn, 4, 4);
        if(rt == 15 || rt2 == 15 || rt == rt2 || opc1 != 0) {
            goto unsupported;
        }
        if(read && (crm == 8 || crm == 9)) {
            uint64_t value = crm == 8 ? env->rp2350_dcp[bank].xm : env->rp2350_dcp[bank].ym;
            env->regs[rt] = (uint32_t)value;
            env->regs[rt2] = value >> 32;
            if(crm == 9 && !peek) {
                env->rp2350_dcp[bank].engaged = false;
            }
            return 0;
        }
        if(!read && !peek && (crm == 0 || crm == 1)) {
            uint64_t value = ((uint64_t)env->regs[rt2] << 32) | env->regs[rt];
            if(crm == 0) {
                env->rp2350_dcp[bank].xm = value;
            } else {
                env->rp2350_dcp[bank].ym = value;
            }
            env->rp2350_dcp[bank].engaged = true;
            return 0;
        }
        goto unsupported;
    }

    /* RCMP / PCMP: processed comparison status, with engaged in bit 31.
     * Comparison arithmetic is unsupported; its state is zero after INIT. */
    if((insn & 0x0fff0fff) == (0x0e100030 | (cp << 8))) {
        uint32_t value = env->rp2350_dcp[bank].engaged ? (1u << 31) : 0;
        if(rt == 15) {
            cpsr_write(env, value, CPSR_NZCV);
        } else {
            env->regs[rt] = value;
        }
        if(!peek) {
            env->rp2350_dcp[bank].engaged = false;
        }
        return 0;
    }

    /* INIT zeros all internal registers, including the engaged flag. */
    if(!peek && (insn & 0x0fffffff) == (0x0e000000 | (cp << 8))) {
        memset(&env->rp2350_dcp[bank], 0, sizeof(env->rp2350_dcp[bank]));
        return 0;
    }

unsupported:
    tlib_printf(LOG_LEVEL_WARNING, "Unsupported RP2350 DCP instruction 0x%08x", insn);
    return EXCP_UDEF;
}

#endif
