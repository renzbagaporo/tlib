/*
 * Armv8-M Flash Patch and Breakpoint instruction comparators.
 *
 * This library is licensed under the GNU Lesser General Public License,
 * version 2 or later. See LICENSE for the full license.
 */
#include "cpu.h"
#include "helper.h"
#include "arch_callbacks.h"
#include "unwind.h"

#ifdef TARGET_PROTO_ARM_M

void tlib_add_fpb_breakpoint(uint32_t address)
{
    cpu_breakpoint_insert(cpu, address & ~1u, BP_CPU, NULL);
}
EXC_VOID_1(tlib_add_fpb_breakpoint, uint32_t, address)

void tlib_remove_fpb_breakpoint(uint32_t address)
{
    cpu_breakpoint_remove(cpu, address & ~1u, BP_CPU);
}
EXC_VOID_1(tlib_remove_fpb_breakpoint, uint32_t, address)

void HELPER(fpb_breakpoint)(CPUState *env, uint32_t address, uint32_t condexec, uint32_t preceding_instructions)
{
    uint32_t declaration = env->instructions_count_declaration;
    env->regs[15] = address;
    env->condexec_bits = condexec;
    /* The matching instruction has not executed. A halt callback may also
     * synchronize time, so expose only the preceding instructions to it. */
    env->instructions_count_declaration = preceding_instructions;
    if(tlib_try_halt_on_fpb_breakpoint()) {
        env->exception_index = EXCP_DEBUG;
        cpu_loop_exit_without_hook(env);
    }
    /* Arm DDI0553B.y RBFPK permits ignoring a match when halting debug and
     * DebugMonitor are disabled. Continue with the original instruction. */
    env->instructions_count_declaration = declaration;
}
#endif
