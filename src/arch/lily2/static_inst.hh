/*
 * Copyright (C) DSP Group, Institute of Microelectronics, Tsinghua University
 * All rights reserved.
 */

#ifndef __ARCH_LILY2_STATIC_INST_HH__
#define __ARCH_LILY2_STATIC_INST_HH__

#include "types.hh"
#include "registers.hh"
#include "operands.hh"
#include "cpu/static_inst.hh"

namespace Lily2ISAInst
{
class Lily2StaticInst : public StaticInst
{
  public:
    typedef TheISA::MachInst MachInst;
    typedef TheISA::RegCount_t RegCount_t;
    typedef TheISA::RegFile_t RegFile_t;
    typedef TheISA::RegIndex_t RegIndex_t;
    typedef TheISA::OpCount_t OpCount_t;
    typedef TheISA::OpLabel_t OpLabel_t;
    typedef TheISA::Op_t Op_t;
    typedef TheISA::FU_t FU_t;
    typedef TheISA::Cond_t Cond_t;

    // Constructor.
    Lily2StaticInst (const char *mnemonic, ExtMachInst extMachInst, OpClass opClass)
        : StaticInst (mnemonic, extMachInst, opClass) {}

    // Accesses instruction flags.
    bool isNop (void) const { return flags[IsNop]; }

    // Accessor and mutator of number of source operands.
    OpCount_t numSrcOps (void) const { return _numSrcOps; }
    void setNumSrcOps (OpCount_t _numSrcOps) { this->_numSrcOps = _numSrcOps; }

    // Accessor and mutator of number of destination operands.
    OpCount_t numDestOps (void) const { return _numDestOps; }
    void setNumDestOps (OpCount_t _numDestOps) { this->_numDestOps = _numDestOps; }

    // Gets the source operand pointers.
    Op_t *srcOp (OpCount_t i) const { return _srcOp[i]; }

    // Gets the destination operand pointers.
    Op_t *destOp (OpCount_t i) const { return _destOp[i]; }

    // Accessor and mutator of static functional unit.
    FU_t staticFU (void) const { return _staticFU; }
    void setStaticFU (FU_t _staticFU) { this->_staticFU = _staticFU; }

    // Accessor and mutator of dynamic functional unit.
    FU_t dynFU (void) const { return _dynFU; }
    void setDynFU (FU_t _dynFU) { this->_dynFU = _dynFU; }

    // Accessor and mutator of execution condition.
    Cond_t cond (void) const { return _cond; };
    void setCond (Cond_t _cond) { this->_cond = _cond; }

    // Accessor and mutator of an execution condition Z bit.
    bool condZ (void) const { return _condZ; }
    void setCondZ (bool _condZ) { this->_condZ = _condZ; }

    // Gets the execution condition pointer.
    Op_t *condOp (void) const { return _condOp; }

    void advancePC (TheISA::PCState &pcState) const {}

    // Machine instruction.
    ExtMachInst extMachInst;

  protected:
    enum Flags {
        IsNop,
        NumFlags
    };
    /*
    // Decodes the functional unit from the given machine code.
    void decodeFU (MachInst insnFU)
    {
        FU_t FU;
        switch (insnFU) {
            case 0 : // Fall through.
            case 1 : FU = FU_XA; break;
            case 2 : FU = FU_XM; break;
            case 3 : FU = FU_XD; break;
            case 4 : // Fall through.
            case 5 : FU = FU_YA; break;
            case 6 : FU = FU_YM; break;
            case 7 : FU = FU_YD; break;
            default: assert (0);
        }
        setStaticFU (FU);
    }

    // Decodes the condition from the given machine code.
    void decodeCond (MachInst insnCond)
    {
        Cond_t cond;
        switch (insnCond) {
            case 0 : cond = COND_ALWAYS; break;
            case 1 : cond = COND_CR0; break;
            case 2 : cond = COND_NCR0; break;
            case 3 : cond = COND_CR1; break;
            case 4 : cond = COND_NCR1; break;
            case 5 : cond = COND_CR2; break;
            case 6 : cond = COND_NCR2; break;
            default: assert (0);
        }
        setCond (cond);
    }*/

    void decodeSrcOp (OpLabel_t opLabel, MachInst insnSrcRegFile, MachInst insnSrcRegIndex)
    {
        ;
    }

    void decodeDestOp (OpLabel_t opLabel, MachInst insnDestRegFile, MachInst insnDestRegIndex)
    {
        ;
    }

    void printFU (std::stringstream &ss) const
    {
        switch (staticFU ()) {
            case TheISA::FU_XA: ss << "[xa]"; break;
            case TheISA::FU_XM: ss << "[xm]"; break;
            case TheISA::FU_XD: ss << "[xd]"; break;
            case TheISA::FU_YA: ss << "[ya]"; break;
            case TheISA::FU_YM: ss << "[ym]"; break;
            case TheISA::FU_YD: ss << "[yd]"; break;
            default           : ss << "[??]"; break;
        }
    }

    void printCond (std::stringstream &ss) const
    {
        switch (cond ()) {
            case TheISA::COND_ALWAYS: ss << "      "; break;
            case TheISA::COND_CR0   : ss << "{ cr0}"; break;
            case TheISA::COND_NCR0  : ss << "{!cr0}"; break;
            case TheISA::COND_CR1   : ss << "{ cr1}"; break;
            case TheISA::COND_NCR1  : ss << "{!cr1}"; break;
            case TheISA::COND_CR2   : ss << "{ cr2}"; break;
            case TheISA::COND_NCR2  : ss << "{!cr2}"; break;
            default                 : ss << "{????}"; break;
        }
    }

    void printName (std::stringstream &ss) const
    {
        ss << mnemonic;
    }

    void printOp (std::stringstream &ss, const Op_t *op) const
    {
        RegFile_t regFile = op->regFile ();
        RegIndex_t regIndex = op->regIndex ();

        switch (op->numRegs ()) {
            case 1 : printReg (ss, regFile, regIndex); break;
            case 2 : printRegPair (ss, regFile, regIndex); break;
            case 4 : printRegPairPair (ss, regFile, regIndex); break;
            default: assert (0);
        }
    }

    void printSrcOps (std::stringstream &ss) const
    {
        for (OpCount_t i = 0; i != numSrcOps (); ++i) {
            printOp (ss, srcOp (i));
        }
    }

    void printDestOps (std::stringstream &ss) const
    {
        for (OpCount_t i = 0; i != numDestOps (); ++i) {
            printOp (ss, destOp (i));
        }
    }

    void printReg (std::stringstream &ss,
                   const RegFile_t &regFile,
                   const RegIndex_t &regIndex) const
    {
        printRegFile (ss, regFile);
        printRegIndex (ss, regIndex);
    }

    void printRegPair (std::stringstream &ss,
                       const RegFile_t &regFile,
                       const RegIndex_t &regIndex) const
    {
        printReg (ss, regFile, regIndex);
        ss << ":";
        printReg (ss, regFile, regIndex + 1);
    }

    void printRegPairPair (std::stringstream &ss,
                           const RegFile_t &regFile,
                           const RegIndex_t &regIndex) const
    {
        printReg (ss, regFile, regIndex);
        ss << ":";
        printReg (ss, regFile, regIndex + 1);
        ss << ":";
        printReg (ss, regFile, regIndex + 2);
        ss << ":";
        printReg (ss, regFile, regIndex + 3);
    }

    void printRegFile (std::stringstream &ss, const RegFile_t &regFile) const
    {
        switch (regFile) {
            case TheISA::REG_X: ss << "x"; break;
            case TheISA::REG_Y: ss << "y"; break;
            case TheISA::REG_G: ss << "g"; break;
            default           : ss << "?"; break;
        }
    }

    void printRegIndex (std::stringstream &ss, const RegIndex_t &regIndex) const
    {
        ss << regIndex;
    }

    // Number of source and destination operands.
    OpCount_t _numSrcOps;
    OpCount_t _numDestOps;

    // Pointers of source and destination operands.
    Op_t *_srcOp[1/*TheISA::MAX_INST_SRC_OPS*/];
    Op_t *_destOp[1/*TheISA::MAX_INST_DEST_OPS*/];

    // Functional unit of an instruction.
    // Static FU is for VLIW and dynamic FU is for Superscalar.
    FU_t _staticFU;
    FU_t _dynFU;

    // Execution condition of an instruction.
    Cond_t _cond;
    bool _condZ;
    Op_t *_condOp;

    // Flags of an instruction.
    std::bitset<NumFlags> flags;
};

} // namespace Lily2ISAInst

#endif // __ARCH_LILY2_STATIC_INST_HH__
