/*
 * RP2350 redundancy coprocessor (RP2350 Datasheet section 3.6.3).
 *
 * This library is licensed under the GNU Lesser General Public License,
 * version 2 or later. See LICENSE for the full license.
 */
#include "cpu.h"
#include "helper.h"
#include "arch_callbacks.h"
#include "unwind.h"

#ifdef TARGET_PROTO_ARM_M

#define RCP_TRUE  0xa500a500u
#define RCP_FALSE 0x00c300c3u

uint32_t tlib_rp2350_rcp_set_salt(uint64_t salt)
{
    if(!cpu->rp2350_dcp_enabled || cpu->rp2350_rcp.salt_valid) {
        return 0;
    }
    cpu->rp2350_rcp.salt = salt;
    cpu->rp2350_rcp.random = salt >> 40;
    cpu->rp2350_rcp.salt_valid = true;
    return 1;
}
EXC_INT_1(uint32_t, tlib_rp2350_rcp_set_salt, uint64_t, salt)

void tlib_rp2350_rcp_set_fault(void)
{
    if(cpu->rp2350_dcp_enabled) {
        cpu->rp2350_rcp.fault = true;
    }
}
EXC_VOID_0(tlib_rp2350_rcp_set_fault)

static uint32_t rcp_canary(CPUState *env, unsigned tag)
{
    uint32_t expanded = tag | (((~tag) & 0xff) << 8) | (tag << 16);
    expanded &= 0xff0000u | ((env->rp2350_rcp.salt >> 24) & 0xffff);
    return (expanded ^ env->rp2350_rcp.salt) << 8;
}

static bool rcp_bool(uint32_t value)
{
    return value == RCP_TRUE || value == RCP_FALSE;
}

/* Table 119: nonlinear output of the four independent LFSRs in salt[63:40].
 * Translation does not model coprocessor pipeline cycle delays, but consumes
 * the same PRNG sample for each delayed instruction and random-byte read. */
static uint32_t rcp_random(CPUState *env)
{
    static const uint8_t taps[8][9] = {
        {11, 3, 4, 19, 10, 14, 1, 2, 9},
        {4, 16, 11, 18, 9, 6, 14, 21, 16},
        {15, 13, 20, 21, 8, 12, 7, 22, 9},
        {23, 12, 7, 16, 14, 5, 17, 3, 15},
        {4, 19, 17, 0, 18, 7, 18, 11, 3},
        {7, 5, 2, 18, 11, 1, 18, 14, 7},
        {14, 21, 19, 6, 16, 13, 4, 14, 6},
        {7, 17, 6, 16, 13, 8, 9, 12, 21},
    };
    static const unsigned shifts[4] = {0, 8, 15, 20};
    static const unsigned widths[4] = {8, 7, 5, 4};
    static const unsigned feedback[4] = {0xb4, 0x60, 0x14, 0xc};
    uint32_t state = env->rp2350_rcp.random;
    uint32_t result = 0;
    uint32_t next = 0;
    for(unsigned i = 0; i < 8; ++i) {
        unsigned bit = 0;
        for(unsigned j = 0; j < 6; ++j) {
            bit ^= (state >> taps[i][j]) & 1;
        }
        unsigned majority = 0;
        for(unsigned j = 6; j < 9; ++j) {
            majority += (state >> taps[i][j]) & 1;
        }
        result |= (bit ^ (majority >= 2)) << i;
    }
    for(unsigned i = 0; i < 4; ++i) {
        unsigned mask = (1u << widths[i]) - 1;
        unsigned part = (state >> shifts[i]) & mask;
        unsigned bit = part == 0 || (__builtin_parity(part & feedback[i]) != 0);
        next |= (((part << 1) | bit) & mask) << shifts[i];
    }
    env->rp2350_rcp.random = next;
    return result;
}

static uint32_t rcp_fault(CPUState *env)
{
    env->rp2350_rcp.fault = true;
    tlib_rp2350_rcp_fault(0);
    return EXCP_IRQ;
}

static uint32_t rcp_stall(CPUState *env)
{
    env->rp2350_rcp.stalled = true;
    return EXCP_WFI;
}

uint32_t HELPER(rp2350_rcp)(CPUState *env, uint32_t insn)
{
    unsigned access = extract32(env->v7m.cpacr[env->secure], 14, 2);
    unsigned rt = extract32(insn, 12, 4);
    unsigned crn = extract32(insn, 16, 4);
    unsigned crm = extract32(insn, 0, 4);
    unsigned opc1;
    unsigned tag = (crn << 4) | crm;
    unsigned opc2 = extract32(insn, 5, 3);
    bool read = extract32(insn, 20, 1);
    bool pair = (insn & 0x0fe00000) == 0x0c400000;
    bool transfer = (insn & 0x0f000010) == 0x0e000010;
    bool secure = !env->v7m.has_trustzone || env->secure;
    bool valid;
    bool pass = true;
    uint32_t result = 0;

    if(!env->rp2350_dcp_enabled || access == 0 || access == 2 ||
       (access == 1 && in_user_mode(env)) ||
       (!secure && !(env->v7m.nsacr & (1u << 7)))) {
        return EXCP_NOCP;
    }
    if(env->rp2350_rcp.fault) {
        return rcp_stall(env);
    }

    /* All instruction listings in 3.6.3.7 and the SDK use odd parity.
     * The even-parity wording in 3.6.3.3 conflicts with those encodings. */
    if(pair) {
        opc1 = extract32(insn, 4, 4);
        valid = !read && rt < 15 && crn < 15 && opc1 <= 8 &&
                __builtin_parity(opc1 ^ (crm >> 3)) &&
                (opc1 == 8 ? (crm & 7) <= 1 : (crm & 7) == 0);
    } else if(transfer) {
        opc1 = extract32(insn, 21, 3);
        valid = __builtin_parity(opc1 ^ (opc2 & 1)) && (opc2 & 6) == 0;
        if(read) {
            valid &= opc1 <= 2 && (opc1 == 0 || tag == 0);
        } else {
            valid &= opc1 <= 5 && rt < 15 &&
                     (opc1 == 0 || opc1 == 4 || opc1 == 5 || tag == 0);
        }
    } else {
        opc1 = extract32(insn, 20, 4);
        valid = (insn & 0x0fffffff) == 0x0e000720; /* CDP panic */
    }
    if(!valid) {
        return secure ? rcp_fault(env) : EXCP_UDEF;
    }

    /* Non-secure reads are zero and writes have no side effects, including
     * salt, counter and fault state. Invalid opcodes still raise UNDEFINSTR. */
    if(!secure) {
        if(transfer && read) {
            if(rt == 15) {
                cpsr_write(env, 0, CPSR_NZCV);
            } else {
                env->regs[rt] = 0;
            }
        }
        return 0;
    }

    if(pair && opc1 == 8) {
        uint64_t salt = ((uint64_t)env->regs[crn] << 32) | env->regs[rt];
        if(!tlib_rp2350_rcp_write_salt(crm & 1, salt)) {
            return rcp_fault(env);
        }
    } else if(transfer && read && opc1 == 1) {
        result = env->rp2350_rcp.salt_valid ? RCP_TRUE : RCP_FALSE;
    } else {
        if(!env->rp2350_rcp.salt_valid) {
            return rcp_fault(env);
        }
        if(pair) {
            uint32_t a = env->regs[rt];
            uint32_t b = env->regs[crn];
            switch(opc1) {
                case 0: pass = rcp_bool(a) && rcp_bool(b); break;
                case 1: pass = a == RCP_TRUE && b == RCP_TRUE; break;
                case 2: pass = rcp_bool(a) && rcp_bool(b) && (a == RCP_TRUE || b == RCP_TRUE); break;
                case 3: pass = rcp_bool(a ^ b); break;
                case 4: pass = (a ^ b) == RCP_TRUE; break;
                case 5: pass = (a ^ b) == RCP_FALSE; break;
                case 6: pass = (a ^ b) == 0x96009600u; break;
                case 7: pass = a == b; break;
            }
        } else if(transfer && read) {
            result = opc1 == 0 ? rcp_canary(env, tag) : rcp_random(env);
        } else if(transfer) {
            uint32_t value = env->regs[rt];
            switch(opc1) {
                case 0: pass = value == rcp_canary(env, tag); break;
                case 1: pass = rcp_bool(value); break;
                case 2: pass = value == RCP_TRUE; break;
                case 3: pass = value == RCP_FALSE; break;
                case 4: env->rp2350_rcp.count = tag; break;
                case 5:
                    pass = env->rp2350_rcp.count == tag;
                    if(pass) { ++env->rp2350_rcp.count; }
                    break;
            }
        } else {
            /* PANIC stalls immediately and also faults the peer. */
            env->rp2350_rcp.fault = true;
            tlib_rp2350_rcp_fault(1);
            return rcp_stall(env);
        }
    }
    if(!pass) {
        return rcp_fault(env);
    }
    if(transfer && read) {
        if(rt == 15) {
            cpsr_write(env, result, CPSR_NZCV);
        } else {
            env->regs[rt] = result;
        }
    }
    if(!extract32(insn, 28, 1) && !(transfer && read && opc1 == 2) && env->rp2350_rcp.salt_valid) {
        rcp_random(env);
    }
    return 0;
}
#endif
