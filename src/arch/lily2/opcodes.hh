/*
 * Copyright (c) 2014 DSP Group, Institute of Microelectronics, Tsinghua University.
 * All rights reserved.
 */

#ifndef __ARCH_LILY2_OPCODES_HH__
#define __ARCH_LILY2_OPCODES_HH__

#include "types.hh"
#include "registers.hh"
#include "operands.hh"

namespace Lily2ISA
{
// Functional unit opcode table interfaces.
FU_t findOpcFU (MachInst opcFUKey);
MachInst genOpcFUKey (MachInst insnFU);

} // namespace Lily2ISA

#endif // __ARCH_LILY2_OPCODES_HH__
