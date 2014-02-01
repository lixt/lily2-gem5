/*
 * Copyright (C) DSP Group, Institute of Microelectronics, Tsinghua University
 * All rights reserved.
 */

#ifndef __ARCH_LILY2_STATIC_INST_HH__
#define __ARCH_LILY2_STATIC_INST_HH__

#include "types.hh"
#include "registers.hh"
#include "operands.hh"
#include "opcodes.hh"
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
    // Typedef for opcode table.
    typedef TheISA::OpcFU_t OpcFU_t;
    typedef TheISA::OpcCond_t OpcCond_t;
    typedef TheISA::OpcReg_t OpcReg_t;

    // Constructor.
    Lily2StaticInst (const char *mnemonic, ExtMachInst extMachInst, OpClass opClass)
        : StaticInst (mnemonic, extMachInst, opClass) {}

    // Accesses instruction flags.
    bool isNop (void) const { return flags[IsNop]; }

    // Accessor and mutator of number of source operands.
    OpCount_t getNumSrcOps (void) const
    {
        return numSrcOps;
    }
    void setNumSrcOps (OpCount_t numSrcOps)
    {
        this->numSrcOps = numSrcOps;
    }

    // Accessor and mutator of number of destination operands.
    OpCount_t getNumDestOps (void) const
    {
        return numDestOps;
    }
    void setNumDestOps (OpCount_t _numDestOps)
    {
        this->numDestOps = numDestOps;
    }

    // Accessor and mutator of the source operand pointers.
    Op_t *getSrcOp (OpCount_t i) const
    {
        return srcOp[i];
    }
    void setSrcOp (OpCount_t i, Op_t *srcOp)
    {
        this->srcOp[i] = srcOp;
    }

    // Accessor and mutator of the destination operand pointers.
    Op_t *getDestOp (OpCount_t i) const
    {
        return destOp[i];
    }
    void setDestOp (OpCount_t i, Op_t *destOp)
    {
        this->destOp[i] = destOp;
    }

    // Accessor and mutator of static functional unit.
    FU_t getStaticFU (void) const
    {
        return staticFU;
    }
    void setStaticFU (FU_t staticFU)
    {
        this->staticFU = staticFU;
    }

    // Accessor and mutator of dynamic functional unit.
    FU_t getDynFU (void) const
    {
        return dynFU;
    }
    void setDynFU (FU_t dynFU)
    {
        this->dynFU = dynFU;
    }

    // Accessor and mutator of execution condition.
    Cond_t getCond (void) const
    {
        return cond;
    }
    void setCond (Cond_t cond)
    {
        this->cond = cond;
    }

    // Accessor and mutator of the execution condition pointer.
    Op_t *getCondOp (void) const
    {
        return condOp;
    }
    void setCondOp (Op_t *condOp)
    {
        this->condOp = condOp;
    }

    // Accessor and mutator of the string of static functional unit.
    std::string getStaticFUStr (void) const
    {
        return staticFUStr;
    }
    void setStaticFUStr (const char *staticFUStr)
    {
        (this->staticFUStr).assign (staticFUStr);
    }

    // Accessor and mutator of the string of dynamic functional unit.
    std::string getDynFUStr (void) const
    {
        return dynFUStr;
    }
    void setDynFUStr (const char *dynFUStr)
    {
        (this->dynFUStr).assign (dynFUStr);
    }

    // Accessor and mutator of the string of condition.
    std::string getCondStr (void) const
    {
        return condStr;
    }
    void setCondStr (const char *condStr)
    {
        (this->condStr).assign (condStr);
    }

    // Accessor and mutator of the string of source operand.
    std::string getSrcOpStr (OpCount_t i) const
    {
        return srcOpStr[i];
    }
    void setSrcOpStr (OpCount_t i, const char *srcOpStr)
    {
        (this->srcOpStr[i]).assign (srcOpStr);
    }

    // Accessor and mutator of the string of destination operand.
    std::string getDestOpStr (OpCount_t i) const
    {
        return destOpStr[i];
    }
    void setDestOpStr (OpCount_t i, const char *destOpStr)
    {
        (this->destOpStr[i]).assign (destOpStr);
    }

    void advancePC (TheISA::PCState &pcState) const {}

    // Machine instruction.
    ExtMachInst extMachInst;

  protected:
    enum Flags {
        IsNop,
        NumFlags
    };

    // Decodes the functional unit from the given machine code.
    void decodeFU (MachInst insnFU)
    {
        OpcFU_t opcFU = Lily2ISA::getOpcFU (insnFU);
        setStaticFU (opcFU.FU);
    }

    // Decodes the condition from the given machine code.
    void decodeCond (MachInst insnCond)
    {
        OpcCond_t opcCond = Lily2ISA::getOpcCond (insnCond);
        setCond (opcCond.cond);
    }

    // Decodes the source operand from the given machine code.
    // OPTION = 0: Inner-Cluster.
    // OPTION = 1: Cross-Cluster.
    // OPTION = 2: Miscellaneous.
    void decodeSrcOp (OpLabel_t opLabel, MachInst insnSrcRegFile,
                      MachInst insnSrcRegIndex, int option = 0)
    {
        OpcReg_t opcReg = Lily2ISA::getOpcReg (insnSrcRegFile, insnSrcRegIndex, option);

        Op_t *op = opFactory (opLabel);
        op->setRegFile (opcReg.regFile);
        op->setRegIndex (opcReg.regIndex);

        // Increments the number of source operands.
        OpCount_t i = getNumSrcOps ();
        setSrcOp (i++, op);
        setNumSrcOps (i);
    }

    void decodeDestOp (OpLabel_t opLabel, MachInst insnDestRegFile,
                       MachInst insnDestRegIndex, int option = 0)
    {
        OpcReg_t opcReg = Lily2ISA::getOpcReg (insnDestRegFile, insnDestRegIndex, option);

        Op_t *op = opFactory (opLabel);
        op->setRegFile (opcReg.regFile);
        op->setRegIndex (opcReg.regIndex);

        // Increments the number of destination operands.
        OpCount_t i = getNumDestOps ();
        setDestOp (i++, op);
        setNumDestOps (i);
    }

    void printFU (std::stringstream &ss) const
    {
        switch (getStaticFU ()) {
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
        switch (getCond ()) {
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
        for (OpCount_t i = 0; i != getNumSrcOps (); ++i) {
            printOp (ss, getSrcOp (i));
        }
    }

    void printDestOps (std::stringstream &ss) const
    {
        for (OpCount_t i = 0; i != getNumDestOps (); ++i) {
            printOp (ss, getDestOp (i));
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

  protected:
    // Number of source and destination operands.
    OpCount_t numSrcOps;
    OpCount_t numDestOps;

    // Pointers of source and destination operands.
    Op_t *srcOp[MaxInstSrcOps];
    Op_t *destOp[MaxInstDestOps];

    // Functional unit of an instruction.
    // Static FU is for VLIW and dynamic FU is for Superscalar.
    FU_t staticFU;
    FU_t dynFU;

    // Execution condition of an instruction.
    Cond_t cond;
    Op_t *condOp;

    // Flags of an instruction.
    std::bitset<NumFlags> flags;

  protected:
    // Printable variables.

    // String of functional unit.
    std::string staticFUStr;
    std::string dynFUStr;

    // String of condition.
    std::string condStr;

    // String of operands.
    std::string srcOpStr[MaxInstSrcOps];
    std::string destOpStr[MaxInstDestOps];

};

} // namespace Lily2ISAInst

#endif // __ARCH_LILY2_STATIC_INST_HH__
