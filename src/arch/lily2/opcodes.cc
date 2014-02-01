/*
 * Copyright (c) 2014 DSP Group, Institute of Microelectronics, Tsinghua University.
 * All rights reserved.
 */

#include "opcodes.hh"

namespace Lily2ISA
{
const OpcFU_t OpcFUs[] =
{
     {0, FU_XA},
     {1, FU_XA},
     {2, FU_XM},
     {3, FU_XD},
     {4, FU_YA},
     {5, FU_YA},
     {6, FU_YM},
     {7, FU_YD},
};
const size_t NumOpcFUs = ARRAY_SIZE (OpcFUs);

const OpcCond_t OpcConds[] =
{
    {0, COND_ALWAYS},
    {1, COND_CR0   },
    {2, COND_NCR0  },
    {3, COND_CR1   },
    {4, COND_NCR1  },
    {5, COND_CR2   },
    {6, COND_NCR2  },
};
const size_t NumOpcConds = ARRAY_SIZE (OpcConds);

const OpcReg_t OpcXRegs[] =
{
    { 0, REG_X,  0}, { 1, REG_X,  1}, { 2, REG_X,  2}, { 3, REG_X,  3},
    { 4, REG_X,  4}, { 5, REG_X,  5}, { 6, REG_X,  6}, { 7, REG_X,  7},
    { 8, REG_X,  8}, { 9, REG_X,  9}, {10, REG_X, 10}, {11, REG_X, 11},
    {12, REG_X, 12}, {13, REG_X, 13}, {14, REG_X, 14}, {15, REG_X, 15},
    {16, REG_X, 16}, {17, REG_X, 17}, {18, REG_X, 18}, {19, REG_X, 19},
    {20, REG_X, 20}, {21, REG_X, 21}, {22, REG_X, 22}, {23, REG_X, 23},
    {24, REG_G, 24}, {25, REG_G, 25}, {26, REG_G, 26}, {27, REG_G, 27},
    {28, REG_G, 28}, {29, REG_G, 29}, {30, REG_G, 30}, {31, REG_G, 31},
};
const size_t NumOpcXRegs = ARRAY_SIZE (OpcXRegs);

const OpcReg_t OpcYRegs[] =
{
    { 0, REG_Y,  0}, { 1, REG_Y,  1}, { 2, REG_Y,  2}, { 3, REG_Y,  3},
    { 4, REG_Y,  4}, { 5, REG_Y,  5}, { 6, REG_Y,  6}, { 7, REG_Y,  7},
    { 8, REG_Y,  8}, { 9, REG_Y,  9}, {10, REG_Y, 10}, {11, REG_Y, 11},
    {12, REG_Y, 12}, {13, REG_Y, 13}, {14, REG_Y, 14}, {15, REG_Y, 15},
    {16, REG_Y, 16}, {17, REG_Y, 17}, {18, REG_Y, 18}, {19, REG_Y, 19},
    {20, REG_Y, 20}, {21, REG_Y, 21}, {22, REG_Y, 22}, {23, REG_Y, 23},
    {24, REG_G, 24}, {25, REG_G, 25}, {26, REG_G, 26}, {27, REG_G, 27},
    {28, REG_G, 28}, {29, REG_G, 29}, {30, REG_G, 30}, {31, REG_G, 31},
};
const size_t NumOpcYRegs = ARRAY_SIZE (OpcYRegs);

const OpcReg_t OpcMRegs[] =
{
    { 0, REG_M,  0},
};
const size_t NumOpcMRegs = ARRAY_SIZE (OpcMRegs);

const OpcFU_t &
getOpcFU (MachInst insnFU)
{
    for (int i = 0; i != NumOpcFUs; ++i) {
        if (OpcFUs[i].insn == insnFU) {
            return OpcFUs[i];
        }
    }

    // Program should never run to here.
    assert (0);
}

const OpcCond_t &
getOpcCond (MachInst insnCond)
{
    for (int i = 0; i != NumOpcConds; ++i) {
        if (OpcConds[i].insn == insnCond) {
            return OpcConds[i];
        }
    }

    // Program should never run to here.
    assert (0);
}

static const OpcReg_t &
getOpcXReg (MachInst insnRegIndex)
{
    for (int i = 0; i != NumOpcXRegs; ++i) {
        if (OpcXRegs[i].insn == insnRegIndex) {
            return OpcXRegs[i];
        }
    }

    // Program should never run to here.
    assert (0);
}

static const OpcReg_t &
getOpcYReg (MachInst insnRegIndex)
{
    for (int i = 0; i != NumOpcYRegs; ++i) {
        if (OpcYRegs[i].insn == insnRegIndex) {
            return OpcYRegs[i];
        }
    }

    // Program should never run to here.
    assert (0);
}

static const OpcReg_t &
getOpcMReg (MachInst insnRegIndex)
{
    for (int i = 0; i != NumOpcMRegs; ++i) {
        if (OpcMRegs[i].insn == insnRegIndex) {
            return OpcMRegs[i];
        }
    }

    // Program should never run to here.
    assert (0);
}

const OpcReg_t &
getOpcReg (MachInst insnRegFile, MachInst insnRegIndex, int option)
{
    const OpcReg_t& (*f) (MachInst);
    switch (option) {
        case 0 : // Inner-Cluster.
            f = insnRegFile ? getOpcYReg : getOpcXReg; break;
        case 1 : // Cross-Cluster.
            f = insnRegFile ? getOpcXReg : getOpcYReg; break;
        case 2 : // Miscellaneous.
            f = getOpcMReg; break;
        default:
            assert (0);
    }
    return f (insnRegIndex);
}
} // namespace Lily2ISA
