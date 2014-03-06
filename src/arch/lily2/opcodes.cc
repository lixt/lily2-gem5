/*
 * Copyright (c) 2014 DSP Group, Institute of Microelectronics, Tsinghua University.
 * All rights reserved.
 */

#include "opcodes.hh"

namespace Lily2ISA
{
const OpcFuncUnit_t OpcFuncUnits[] =
{
     {0, FU_XA, "[xa]"},
     {1, FU_XA, "[xa]"},
     {2, FU_XM, "[xm]"},
     {3, FU_XD, "[xd]"},
     {4, FU_YA, "[ya]"},
     {5, FU_YA, "[ya]"},
     {6, FU_YM, "[ym]"},
     {7, FU_YD, "[yd]"},
};
const size_t NumOpcFuncUnits = ARRAY_SIZE (OpcFuncUnits);

const OpcCond_t OpcConds[] =
{
    {0, COND_ALWAYS, true , OP_32I, REG_NIL, 0, "      "},
    {1, COND_CR0   , true , OP_32I, REG_X  , 0, "{ cr0}"},
    {2, COND_NCR0  , false, OP_32I, REG_X  , 0, "{!cr0}"},
    {3, COND_CR1   , true , OP_32I, REG_Y  , 0, "{ cr1}"},
    {4, COND_NCR1  , false, OP_32I, REG_Y  , 0, "{!cr1}"},
    {5, COND_CR2   , true , OP_32I, REG_G  , 0, "{ cr2}"},
    {6, COND_NCR2  , false, OP_32I, REG_G  , 0, "{!cr2}"},
};
const size_t NumOpcConds = ARRAY_SIZE (OpcConds);

const OpcReg_t OpcXRegs[] =
{
    { 0, REG_X,  0, "x0" }, { 1, REG_X,  1, "x1" },
    { 2, REG_X,  2, "x2" }, { 3, REG_X,  3, "x3" },
    { 4, REG_X,  4, "x4" }, { 5, REG_X,  5, "x5" },
    { 6, REG_X,  6, "x6" }, { 7, REG_X,  7, "x7" },
    { 8, REG_X,  8, "x8" }, { 9, REG_X,  9, "x9" },
    {10, REG_X, 10, "x10"}, {11, REG_X, 11, "x11"},
    {12, REG_X, 12, "x12"}, {13, REG_X, 13, "x13"},
    {14, REG_X, 14, "x14"}, {15, REG_X, 15, "x15"},
    {16, REG_X, 16, "x16"}, {17, REG_X, 17, "x17"},
    {18, REG_X, 18, "x18"}, {19, REG_X, 19, "x19"},
    {20, REG_X, 20, "x20"}, {21, REG_X, 21, "x21"},
    {22, REG_X, 22, "x22"}, {23, REG_X, 23, "x23"},
    {24, REG_G,  0, "g0" }, {25, REG_G,  1, "g1" },
    {26, REG_G,  2, "g2" }, {27, REG_G,  3, "g3" },
    {28, REG_G,  4, "g4" }, {29, REG_G,  5, "g5" },
    {30, REG_G,  6, "g6" }, {31, REG_G,  7, "g7" },
};
const size_t NumOpcXRegs = ARRAY_SIZE (OpcXRegs);

const OpcReg_t OpcXRegPairs[] =
{
    { 0, REG_X,  0, "x1:x0"  }, { 2, REG_X,  2, "x3:x2"  },
    { 4, REG_X,  4, "x5:x4"  }, { 6, REG_X,  6, "x7:x6"  },
    { 8, REG_X,  8, "x9:x8"  }, {10, REG_X, 10, "x11:x10"},
    {12, REG_X, 12, "x13:x12"}, {14, REG_X, 14, "x15:x14"},
    {16, REG_X, 16, "x17:x16"}, {18, REG_X, 16, "x19:x18"},
    {20, REG_X, 20, "x21:x20"}, {22, REG_X, 22, "x23:x22"},
    {24, REG_G,  0, "g1:g0"  }, {26, REG_G,  2, "g3:g2"  },
    {28, REG_G,  4, "g5:g4"  }, {30, REG_G,  6, "g7:g6"  },
};
const size_t NumOpcXRegPairs = ARRAY_SIZE (OpcXRegPairs);

const OpcReg_t OpcXRegPairPairs[] =
{
    { 0, REG_X,  0, "x3:x2:x1:x0"    },
    { 4, REG_X,  4, "x7:x6:x5:x4"    },
    { 8, REG_X,  8, "x11:x10:x9:x8"  },
    {12, REG_X, 12, "x15:x14:x13:x12"},
    {16, REG_X, 16, "x19:x18:x17:x16"},
    {20, REG_X, 20, "x23:x22:x21:x20"},
    {24, REG_G,  0, "g3:g2:g1:g0"    },
    {28, REG_G,  4, "g7:g6:g5:g4"    },
};
const size_t NumOpcXRegPairPairs = ARRAY_SIZE (OpcXRegPairPairs);

const OpcReg_t OpcYRegs[] =
{
    { 0, REG_Y,  0, "y0" }, { 1, REG_Y,  1, "y1" },
    { 2, REG_Y,  2, "y2" }, { 3, REG_Y,  3, "y3" },
    { 4, REG_Y,  4, "y4" }, { 5, REG_Y,  5, "y5" },
    { 6, REG_Y,  6, "y6" }, { 7, REG_Y,  7, "y7" },
    { 8, REG_Y,  8, "y8" }, { 9, REG_Y,  9, "y9" },
    {10, REG_Y, 10, "y10"}, {11, REG_Y, 11, "y11"},
    {12, REG_Y, 12, "y12"}, {13, REG_Y, 13, "y13"},
    {14, REG_Y, 14, "y14"}, {15, REG_Y, 15, "y15"},
    {16, REG_Y, 16, "y16"}, {17, REG_Y, 17, "y17"},
    {18, REG_Y, 18, "y18"}, {19, REG_Y, 19, "y19"},
    {20, REG_Y, 20, "y20"}, {21, REG_Y, 21, "y21"},
    {22, REG_Y, 22, "y22"}, {23, REG_Y, 23, "y23"},
    {24, REG_G,  0, "g0" }, {25, REG_G,  1, "g1" },
    {26, REG_G,  2, "g2" }, {27, REG_G,  3, "g3" },
    {28, REG_G,  4, "g4" }, {29, REG_G,  5, "g5" },
    {30, REG_G,  6, "g6" }, {31, REG_G,  7, "g7" },
};
const size_t NumOpcYRegs = ARRAY_SIZE (OpcYRegs);

const OpcReg_t OpcYRegPairs[] =
{
    { 0, REG_Y,  0, "y1:y0"  }, { 2, REG_Y,  2, "y3:y2"  },
    { 4, REG_Y,  4, "y5:y4"  }, { 6, REG_Y,  6, "y7:y6"  },
    { 8, REG_Y,  8, "y9:y8"  }, {10, REG_Y, 10, "y11:y10"},
    {12, REG_Y, 12, "y13:y12"}, {14, REG_Y, 14, "y15:y14"},
    {16, REG_Y, 16, "y17:y16"}, {18, REG_Y, 16, "y19:y18"},
    {20, REG_Y, 20, "y21:y20"}, {22, REG_Y, 22, "y23:y22"},
    {24, REG_G,  0, "g1:g0"  }, {26, REG_G,  2, "g3:g2"  },
    {28, REG_G,  4, "g5:g4"  }, {30, REG_G,  6, "g7:g6"  },
};
const size_t NumOpcYRegPairs = ARRAY_SIZE (OpcYRegPairs);

const OpcReg_t OpcYRegPairPairs[] =
{
    { 0, REG_Y,  0, "y3:y2:y1:y0"    },
    { 4, REG_Y,  4, "y7:y6:y5:y4"    },
    { 8, REG_Y,  8, "y11:y10:y9:y8"  },
    {12, REG_Y, 12, "y15:y14:y13:y12"},
    {16, REG_Y, 16, "y19:y18:y17:y16"},
    {20, REG_Y, 20, "y23:y22:y21:y20"},
    {24, REG_G,  0, "g3:g2:g1:g0"    },
    {28, REG_G,  4, "g7:g6:g5:g4"    },
};
const size_t NumOpcYRegPairPairs = ARRAY_SIZE (OpcYRegPairPairs);

const OpcReg_t OpcMRegs[] =
{
    { 0, REG_M,  0, "m0"},
};
const size_t NumOpcMRegs = ARRAY_SIZE (OpcMRegs);

const OpcReg_t OpcMRegPairs[] =
{
    { 0, REG_M,  0, "m0"},
};
const size_t NumOpcMRegPairs = ARRAY_SIZE (OpcMRegPairs);

const OpcReg_t OpcMRegPairPairs[] =
{
    { 0, REG_M,  0, "m0"},
};
const size_t NumOpcMRegPairPairs = ARRAY_SIZE (OpcMRegPairPairs);

const OpcFuncUnit_t &
getOpcFuncUnit (MachInst insnFuncUnit)
{
    for (int i = 0; i != NumOpcFuncUnits; ++i) {
        if (OpcFuncUnits[i].insn == insnFuncUnit) {
            return OpcFuncUnits[i];
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
getOpcXRegPair (MachInst insnRegIndex)
{
    for (int i = 0; i != NumOpcXRegPairs; ++i) {
        if (OpcXRegPairs[i].insn == insnRegIndex) {
            return OpcXRegPairs[i];
        }
    }

    // Program should never run to here.
    assert (0);
}

static const OpcReg_t &
getOpcXRegPairPair (MachInst insnRegIndex)
{
    for (int i = 0; i != NumOpcXRegPairPairs; ++i) {
        if (OpcXRegPairPairs[i].insn == insnRegIndex) {
            return OpcXRegPairPairs[i];
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
getOpcYRegPair (MachInst insnRegIndex)
{
    for (int i = 0; i != NumOpcYRegPairs; ++i) {
        if (OpcYRegPairs[i].insn == insnRegIndex) {
            return OpcYRegPairs[i];
        }
    }

    // Program should never run to here.
    assert (0);
}

static const OpcReg_t &
getOpcYRegPairPair (MachInst insnRegIndex)
{
    for (int i = 0; i != NumOpcYRegPairPairs; ++i) {
        if (OpcYRegPairPairs[i].insn == insnRegIndex) {
            return OpcYRegPairPairs[i];
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

static const OpcReg_t &
getOpcMRegPair (MachInst insnRegIndex)
{
    for (int i = 0; i != NumOpcMRegPairs; ++i) {
        if (OpcMRegPairs[i].insn == insnRegIndex) {
            return OpcMRegPairs[i];
        }
    }

    // Program should never run to here.
    assert (0);
}

static const OpcReg_t &
getOpcMRegPairPair (MachInst insnRegIndex)
{
    for (int i = 0; i != NumOpcMRegPairPairs; ++i) {
        if (OpcMRegPairPairs[i].insn == insnRegIndex) {
            return OpcMRegPairPairs[i];
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

const OpcReg_t &
getOpcRegPair (MachInst insnRegFile, MachInst insnRegIndex, int option)
{
    const OpcReg_t& (*f) (MachInst);
    switch (option) {
        case 0 : // Inner-Cluster.
            f = insnRegFile ? getOpcYRegPair : getOpcXRegPair; break;
        case 1 : // Cross-Cluster.
            f = insnRegFile ? getOpcXRegPair : getOpcYRegPair; break;
        case 2 : // Miscellaneous.
            f = getOpcMRegPair; break;
        default:
            assert (0);
    }
    return f (insnRegIndex);
}

const OpcReg_t &
getOpcRegPairPair (MachInst insnRegFile, MachInst insnRegIndex, int option)
{
    const OpcReg_t& (*f) (MachInst);
    switch (option) {
        case 0 : // Inner-Cluster.
            f = insnRegFile ? getOpcYRegPairPair : getOpcXRegPairPair; break;
        case 1 : // Cross-Cluster.
            f = insnRegFile ? getOpcXRegPairPair : getOpcYRegPairPair; break;
        case 2 : // Miscellaneous.
            f = getOpcMRegPairPair; break;
        default:
            assert (0);
    }
    return f (insnRegIndex);
}

} // namespace Lily2ISA
