/*
 * Copyright (c) 2006 The Regents of The University of Michigan
 * Copyright (c) 2014 DSP Group, Institute of Microelectronics, Tsinghua University.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __ARCH_LILY2_REGISTERS_HH__
#define __ARCH_LILY2_REGISTERS_HH__

#include <vector>
#include <iostream>
#include <iomanip>
#include "arch/lily2/generated/max_inst_regs.hh"
#include "base/misc.hh"
#include "base/types.hh"
#include "base/table.hh"

class ThreadContext;

namespace Lily2ISA
{
// Type for register counting.
typedef int8_t RegCount_t;

// Type for register index.
typedef int16_t RegIndex_t;

// Register number of three general register files.
const RegIndex_t NumXRegs = 24;
const RegIndex_t NumYRegs = 24;
const RegIndex_t NumGRegs = 8;
const RegIndex_t NumMRegs = 8;

// Types for register files.
typedef enum RegFile_t
{
    REG_NIL,     // Illegal.

    REG_X,       // Register file X.
    REG_Y,       // Register file Y.
    REG_G,       // Register file G.
    REG_M,       // Miscellaneous.

    NUM_REG_FILE // Number of register labels.
} RegFile_t;

// Characters for register files.
static const char *RegFileStr[NUM_REG_FILE] =
{
    "nil", "x", "y", "g", "m",
};

// Types for three general register file.
typedef uint32_t XRegValue_t;
typedef uint32_t YRegValue_t;
typedef uint32_t GRegValue_t;
typedef uint32_t MRegValue_t;

// Types for register files.
template <RegFile_t FileName, size_t RegNum, class RegValue_t>
class RegFile : public Table<RegNum, 1, RegIndex_t, RegValue_t>
{
  public:
    typedef Table<RegNum, 1, RegIndex_t, RegValue_t> Base;

    // Typedef for the types in base class.
    typedef typename Base::key_type key_type;
    typedef typename Base::mapped_type mapped_type;
    typedef typename Base::value_type value_type;

    // Typedef for the POSITION.
    typedef typename Base::Position Position;

  public:
    // Constructor.
    // This constructor sets all the registers to zero.
    RegFile (void) {}

  public:
    // Gets the register value according to the given REGINDEX.
    RegValue_t getRegValue (const RegIndex_t &regIndex) const;

    // Sets the register value according to the given REGINDEX and new REGVALUE.
    void setRegValue (const RegIndex_t &regIndex, const RegValue_t &regValue);

  public:
    // Prints out debug information of the register file.
    void print (std::ostream &os) const;
    void printReg (std::ostream &os, const RegIndex_t &regIndex) const;

  private:
    class PrintFunctor
    {
      public:
        void operator() (std::ostream &os, const RegIndex_t &regIndex, const RegValue_t &regValue)
        {
            os << RegFileStr[FileName]
               << "RegIndex = " << DEC << regIndex
               << ", "
               << "RegValue = " << REG_OUTPUT_FORMAT << regValue;
        }
    };
};

template <RegFile_t FileName, size_t RegNum, class RegValue_t>
RegValue_t
RegFile<FileName, RegNum, RegValue_t>::getRegValue (const RegIndex_t &regIndex) const
{
    Position accessPos (regIndex, 1);
    return Base::access (accessPos);
}

template <RegFile_t FileName, size_t RegNum, class RegValue_t>
void
RegFile<FileName, RegNum, RegValue_t>::setRegValue (const RegIndex_t &regIndex,
                                                    const RegValue_t &regValue)
{
    Position mutatePos (regIndex, 1);

    if (isPosValid (mutatePos)) {
        Base::mutate (mutatePos, regValue);
    } else {
        Base::insert (regIndex, regValue);
    }
}

template <RegFile_t FileName, size_t RegNum, class RegValue_t>
void
RegFile<FileName, RegNum, RegValue_t>::print (std::ostream &os) const
{
    PrintFunctor pfunc;
    Base::print (os, pfunc);
}

template <RegFile_t FileName, size_t RegNum, class RegValue_t>
void
RegFile<FileName, RegNum, RegValue_t>::printReg (std::ostream &os,
                                                 const RegIndex_t &regIndex) const
{
    PrintFunctor pfunc;
    pfunc (os, regIndex, getRegValue (regIndex));
}


// Types for the mapped type of the register buffer.
template <class RegValue_t>
struct RegFileBufMapped_t
{
    RegValue_t regValue;
    RegValue_t regMask;
    Cycles regBackCycle;
};

// Each register has ten register buffers. This seems to be big enough.
const size_t BufNum = 2;

// Types for register file buffers.
template <RegFile_t FileName, size_t RegNum, class RegValue_t>
class RegFileBuf : public Table<RegNum, BufNum, RegIndex_t, RegFileBufMapped_t<RegValue_t>>
{
  public:
    // Typedef for the base class.
    typedef Table<RegNum, BufNum, RegIndex_t, RegFileBufMapped_t<RegValue_t>> Base;

    // Typedef for the types in base class.
    typedef typename Base::key_type key_type;
    typedef typename Base::mapped_type mapped_type;
    typedef typename Base::value_type value_type;

    // Typedef for the POSITION.
    typedef typename Base::Position Position;

  public:
    // Sets the register buffer.
    void insert (const RegIndex_t &regIndex, const RegValue_t &regValue,
                 const RegValue_t &regMask, const Cycles &regBackCycle);

    // Gets the register value and register mask according to the given position.
    RegValue_t getRegValue (const Position &accessPos) const;
    RegValue_t getRegMask (const Position &accessPos) const;

    // Updates the register back cycles per cycle and returns a vector of position
    // whose register back cycle is equal to zero.
    std::vector<Position> updateRegBackCycle (Cycles regBackCycleDelta);

  public:
    // Prints the debug information of register file buffer.
    void print (std::ostream &os) const;

  private:
    // Functor used in UPDATEREGBACKCYCLE.
    class UpdateRegBackCycleFunctor
    {
      public:
        explicit UpdateRegBackCycleFunctor (Cycles regBackCycleDelta) :
            regBackCycleDelta (regBackCycleDelta) {}

      public:
        bool operator() (key_type &key, mapped_type &mapped)
        {
            bool retval;

            if (mapped.regBackCycle >= regBackCycleDelta) {
                retval = false;
                mapped.regBackCycle = mapped.regBackCycle - regBackCycleDelta;
            } else {
                retval = true;
                mapped.regBackCycle = 0;
            }

            return retval;
        }

      private:
        Cycles regBackCycleDelta;
    };

    // Functor used in PRINT.
    class PrintFunctor
    {
      public:
        void operator() (std::ostream &os, const key_type &key, const mapped_type &mapped)
        {
            os << RegFileStr[FileName]
               << "RegIndex = " << DEC << key
               << ", "
               << "RegValue = " << REG_OUTPUT_FORMAT << mapped.regValue
               << ", "
               << "RegMask = " << REG_OUTPUT_FORMAT << mapped.regMask
               << ", "
               << "RegBackCycle = " << DEC << mapped.regBackCycle;
        }
    };
};

template <RegFile_t FileName, size_t RegNum, class RegValue_t>
void
RegFileBuf<FileName, RegNum, RegValue_t>::insert (const RegIndex_t &regIndex,
                                                  const RegValue_t &regValue,
                                                  const RegValue_t &regMask,
                                                  const Cycles &regBackCycle)
{
    mapped_type mapped;
    mapped.regValue = regValue;
    mapped.regMask = regMask;
    mapped.regBackCycle = regBackCycle;

    Base::insert (regIndex, mapped);
}

template <RegFile_t FileName, size_t RegNum, class RegValue_t>
RegValue_t
RegFileBuf<FileName, RegNum, RegValue_t>::getRegValue (const Position &accessPos) const
{
    return (Base::access (accessPos)).regValue;
}

template <RegFile_t FileName, size_t RegNum, class RegValue_t>
RegValue_t
RegFileBuf<FileName, RegNum, RegValue_t>::getRegMask (const Position &accessPos) const
{
    return (Base::access (accessPos)).regMask;
}

template <RegFile_t FileName, size_t RegNum, class RegValue_t>
std::vector<typename RegFileBuf<FileName, RegNum, RegValue_t>::Position>
RegFileBuf<FileName, RegNum, RegValue_t>::updateRegBackCycle (Cycles regBackCycleDelta)
{
    UpdateRegBackCycleFunctor func (regBackCycleDelta);
    return Base::traverseAndReturn (func);
}

template <RegFile_t FileName, size_t RegNum, class RegValue_t>
void
RegFileBuf<FileName, RegNum, RegValue_t>::print (std::ostream &os) const
{
    PrintFunctor pfunc;
    Base::print (os, pfunc);
}


using Lily2ISAInst::MaxInstSrcRegs;
using Lily2ISAInst::MaxInstDestRegs;
using Lily2ISAInst::MaxMiscDestRegs;

// Constants Related to the number of registers
const int NumIntArchRegs = 32;
const int NumIntSpecialRegs = 9;
const int NumFloatArchRegs = 32;
const int NumFloatSpecialRegs = 5;

const int MaxShadowRegSets = 16; // Maximum number of shadow register sets
const int NumIntRegs = NumIntArchRegs + NumIntSpecialRegs;        //HI & LO Regs
const int NumFloatRegs = NumFloatArchRegs + NumFloatSpecialRegs;//

const uint32_t LILY232_QNAN = 0x7fbfffff;
const uint64_t LILY264_QNAN = ULL(0x7ff7ffffffffffff);

enum FPControlRegNums {
   FLOATREG_FIR = NumFloatArchRegs,
   FLOATREG_FCCR,
   FLOATREG_FEXR,
   FLOATREG_FENR,
   FLOATREG_FCSR
};

enum FCSRBits {
    Inexact = 1,
    Underflow,
    Overflow,
    DivideByZero,
    Invalid,
    Unimplemented
};

enum FCSRFields {
    Flag_Field = 1,
    Enable_Field = 6,
    Cause_Field = 11
};

enum MiscIntRegNums {
   INTREG_LO = NumIntArchRegs,
   INTREG_DSP_LO0 = INTREG_LO,
   INTREG_HI,
   INTREG_DSP_HI0 = INTREG_HI,
   INTREG_DSP_ACX0,
   INTREG_DSP_LO1,
   INTREG_DSP_HI1,
   INTREG_DSP_ACX1,
   INTREG_DSP_LO2,
   INTREG_DSP_HI2,
   INTREG_DSP_ACX2,
   INTREG_DSP_LO3,
   INTREG_DSP_HI3,
   INTREG_DSP_ACX3,
   INTREG_DSP_CONTROL
};

// semantically meaningful register indices
const int ZeroReg = 0;
const int AssemblerReg = 1;
const int SyscallSuccessReg = 7;
const int FirstArgumentReg = 4;
const int ReturnValueReg = 2;

const int KernelReg0 = 26;
const int KernelReg1 = 27;
const int GlobalPointerReg = 28;
const int StackPointerReg = 29;
const int FramePointerReg = 30;
const int ReturnAddressReg = 31;

const int SyscallPseudoReturnReg = 3;

//@TODO: Implementing ShadowSets needs to
//edit this value such that:
//TotalArchRegs = NumIntArchRegs * ShadowSets
const int TotalArchRegs = NumIntArchRegs;

// These help enumerate all the registers for dependence tracking.
const int FP_Base_DepTag = NumIntRegs;
const int Ctrl_Base_DepTag = FP_Base_DepTag + NumFloatRegs;

// Enumerate names for 'Control' Registers in the CPU
// Reference LILY232 Arch. for Programmers, Vol. III, Ch.8
// (Register Number-Register Select) Summary of Register
//------------------------------------------------------
// The first set of names classify the CP0 names as Register Banks
// for easy indexing when using the 'RD + SEL' index combination
// in CP0 instructions.
enum MiscRegIndex{
    MISCREG_INDEX = 0,       //Bank 0: 0 - 3
    MISCREG_MVP_CONTROL,
    MISCREG_MVP_CONF0,
    MISCREG_MVP_CONF1,

    MISCREG_CP0_RANDOM = 8,      //Bank 1: 8 - 15
    MISCREG_VPE_CONTROL,
    MISCREG_VPE_CONF0,
    MISCREG_VPE_CONF1,
    MISCREG_YQMASK,
    MISCREG_VPE_SCHEDULE,
    MISCREG_VPE_SCHEFBACK,
    MISCREG_VPE_OPT,

    MISCREG_ENTRYLO0 = 16,   //Bank 2: 16 - 23
    MISCREG_TC_STATUS,
    MISCREG_TC_BIND,
    MISCREG_TC_RESTART,
    MISCREG_TC_HALT,
    MISCREG_TC_CONTEXT,
    MISCREG_TC_SCHEDULE,
    MISCREG_TC_SCHEFBACK,

    MISCREG_ENTRYLO1 = 24,   // Bank 3: 24

    MISCREG_CONTEXT = 32,    // Bank 4: 32 - 33
    MISCREG_CONTEXT_CONFIG,

    MISCREG_PAGEMASK = 40, //Bank 5: 40 - 41
    MISCREG_PAGEGRAIN = 41,

    MISCREG_WIRED = 48,          //Bank 6:48-55
    MISCREG_SRS_CONF0,
    MISCREG_SRS_CONF1,
    MISCREG_SRS_CONF2,
    MISCREG_SRS_CONF3,
    MISCREG_SRS_CONF4,

    MISCREG_HWRENA = 56,         //Bank 7: 56-63

    MISCREG_BADVADDR = 64,       //Bank 8: 64-71

    MISCREG_COUNT = 72,          //Bank 9: 72-79

    MISCREG_ENTRYHI = 80,        //Bank 10: 80-87

    MISCREG_COMPARE = 88,        //Bank 11: 88-95

    MISCREG_STATUS = 96,         //Bank 12: 96-103
    MISCREG_INTCTL,
    MISCREG_SRSCTL,
    MISCREG_SRSMAP,

    MISCREG_CAUSE = 104,         //Bank 13: 104-111

    MISCREG_EPC = 112,           //Bank 14: 112-119

    MISCREG_PRID = 120,          //Bank 15: 120-127,
    MISCREG_EBASE,

    MISCREG_CONFIG = 128,        //Bank 16: 128-135
    MISCREG_CONFIG1,
    MISCREG_CONFIG2,
    MISCREG_CONFIG3,
    MISCREG_CONFIG4,
    MISCREG_CONFIG5,
    MISCREG_CONFIG6,
    MISCREG_CONFIG7,


    MISCREG_LLADDR = 136,        //Bank 17: 136-143

    MISCREG_WATCHLO0 = 144,      //Bank 18: 144-151
    MISCREG_WATCHLO1,
    MISCREG_WATCHLO2,
    MISCREG_WATCHLO3,
    MISCREG_WATCHLO4,
    MISCREG_WATCHLO5,
    MISCREG_WATCHLO6,
    MISCREG_WATCHLO7,

    MISCREG_WATCHHI0 = 152,     //Bank 19: 152-159
    MISCREG_WATCHHI1,
    MISCREG_WATCHHI2,
    MISCREG_WATCHHI3,
    MISCREG_WATCHHI4,
    MISCREG_WATCHHI5,
    MISCREG_WATCHHI6,
    MISCREG_WATCHHI7,

    MISCREG_XCCONTEXT64 = 160, //Bank 20: 160-167

                       //Bank 21: 168-175

                       //Bank 22: 176-183

    MISCREG_DEBUG = 184,       //Bank 23: 184-191
    MISCREG_TRACE_CONTROL1,
    MISCREG_TRACE_CONTROL2,
    MISCREG_USER_TRACE_DATA,
    MISCREG_TRACE_BPC,

    MISCREG_DEPC = 192,        //Bank 24: 192-199

    MISCREG_PERFCNT0 = 200,    //Bank 25: 200-207
    MISCREG_PERFCNT1,
    MISCREG_PERFCNT2,
    MISCREG_PERFCNT3,
    MISCREG_PERFCNT4,
    MISCREG_PERFCNT5,
    MISCREG_PERFCNT6,
    MISCREG_PERFCNT7,

    MISCREG_ERRCTL = 208,      //Bank 26: 208-215

    MISCREG_CACHEERR0 = 216,   //Bank 27: 216-223
    MISCREG_CACHEERR1,
    MISCREG_CACHEERR2,
    MISCREG_CACHEERR3,

    MISCREG_TAGLO0 = 224,      //Bank 28: 224-231
    MISCREG_DATALO1,
    MISCREG_TAGLO2,
    MISCREG_DATALO3,
    MISCREG_TAGLO4,
    MISCREG_DATALO5,
    MISCREG_TAGLO6,
    MISCREG_DATALO7,

    MISCREG_TAGHI0 = 232,      //Bank 29: 232-239
    MISCREG_DATAHI1,
    MISCREG_TAGHI2,
    MISCREG_DATAHI3,
    MISCREG_TAGHI4,
    MISCREG_DATAHI5,
    MISCREG_TAGHI6,
    MISCREG_DATAHI7,


    MISCREG_ERROR_EPC = 240,    //Bank 30: 240-247

    MISCREG_DESAVE = 248,       //Bank 31: 248-256

    MISCREG_LLFLAG = 257,
    MISCREG_TP_VALUE,

    MISCREG_NUMREGS
};

const int TotalDataRegs = NumIntRegs + NumFloatRegs;

const int NumMiscRegs = MISCREG_NUMREGS;
const int Max_DepTag = Ctrl_Base_DepTag + NumMiscRegs;

const int TotalNumRegs = NumIntRegs + NumFloatRegs + NumMiscRegs;

typedef uint16_t  RegIndex;

typedef uint32_t IntReg;

// floating point register file entry type
typedef uint32_t FloatRegBits;
typedef float FloatReg;

// cop-0/cop-1 system control register
typedef uint64_t MiscReg;

typedef union {
    IntReg   intreg;
    FloatReg fpreg;
    MiscReg  ctrlreg;
} AnyReg;

} // namespace Lily2ISA

#endif
