/*
 * Copyright (c) 2014 DSP Group, Institute of Microelectronics, Tsinghua University.
 * All rights reserved.
 */

#ifndef __ARCH_LILY2_OPCODES_HH__
#define __ARCH_LILY2_OPCODES_HH__

#include "types.hh"
#include "registers.hh"
#include "operands.hh"

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a) sizeof (a) / sizeof (a[0])
#endif

namespace Lily2ISA
{
// Type for opcode table of conditions.
struct OpcCond_t
{
    MachInst insn;
    Cond_t cond;
    // Condition is positive or negative.
    bool condZ;
    // Condition operand.
    OpLabel_t opLabel;
    RegFile_t regFile;
    RegIndex_t regIndex;
    // Condition printable string.
    const char *str;
};

// Type for opcode table of functional units.
struct OpcFuncUnit_t
{
    MachInst insn;
    FuncUnit_t FuncUnit;
    const char *str;
};

// Type for opcode table of registers.
struct OpcReg_t
{
    MachInst insn;
    RegFile_t regFile;
    RegIndex_t regIndex;
    const char *str;
};

// Typedefs.
typedef struct OpcFuncUnit_t OpcFuncUnit_t;
typedef struct OpcReg_t OpcReg_t;
typedef struct OpcCond_t OpcCond_t;

// Gets the opcodes of the functional units.
const OpcFuncUnit_t& getOpcFuncUnit (MachInst insnFuncUnit);

// Gets the opcodes of the conditions.
const OpcCond_t& getOpcCond (MachInst insnCond);

// Gets the opcodes of the registers.
// OPTION = 0: Inner-Cluster.
// OPTION = 1: Cross-Cluster.
// OPTION = 2: Miscellaneous.
const OpcReg_t& getOpcReg (MachInst insnRegFile, MachInst insnRegIndex,
                           int option = 0);

// Gets the opcodes of the register pairs.
// OPTION = 0: Inner-Cluster.
// OPTION = 1: Cross-Cluster.
// OPTION = 2: Miscellaneous.
const OpcReg_t& getOpcRegPair (MachInst insnRegFile, MachInst insnRegIndex,
                               int option = 0);

// Gets the opcodes of the register pair pairs.
// OPTION = 0: Inner-Cluster.
// OPTION = 1: Cross-Cluster.
// OPTION = 2: Miscellaneous.
const OpcReg_t& getOpcRegPairPair (MachInst insnRegFile, MachInst insnRegIndex,
                                   int option = 0);

} // namespace Lily2ISA

#endif // __ARCH_LILY2_OPCODES_HH__
