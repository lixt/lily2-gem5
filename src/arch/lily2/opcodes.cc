/*
 * Copyright (c) 2014 DSP Group, Institute of Microelectronics, Tsinghua University.
 * All rights reserved.
 */

#include "opcodes.hh"

namespace Lily2ISA
{

const FU_t OpcFU[] =
{
    FU_XA, // 0b'000
    FU_XA, // 0b'001
    FU_XM, // 0b'010
    FU_XD, // 0b'011
    FU_YA, // 0b'100
    FU_YA, // 0b'101
    FU_YM, // 0b'110
    FU_YD, // 0b'111
};

FU_t findOpcFU (MachInst key)
{
    return OpcFU[key];
}

MachInst genOpcFUKey (MachInst insnFU)
{
    return insnFU;
}

} // namespace Lily2ISA
