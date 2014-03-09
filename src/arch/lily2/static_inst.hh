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
    typedef TheISA::FuncUnit_t FuncUnit_t;
    typedef TheISA::Cond_t Cond_t;
    // Typedef for opcode table.
    typedef TheISA::OpcFuncUnit_t OpcFuncUnit_t;
    typedef TheISA::OpcCond_t OpcCond_t;
    typedef TheISA::OpcReg_t OpcReg_t;

    // Constructor.
    Lily2StaticInst (const char *mnemonic, MachInst machInst, OpClass opClass)
        : StaticInst (mnemonic, machInst, opClass),
          numSrcOps (0), numDestOps (0),
          staticFuncUnit (TheISA::FU_NIL), dynFuncUnit (TheISA::FU_NIL),
          cond (TheISA::COND_NIL), staticFuncUnitStr (), dynFuncUnitStr (),
          condStr ()
    {
        for (OpCount_t i = 0; i != MaxInstSrcOps; ++i) {
            srcOp[i] = NULL;
            srcOpStr[i].assign ("");
        }

        for (OpCount_t i = 0; i != MaxInstDestOps; ++i) {
            destOp[i] = NULL;
            destOpStr[i].assign ("");
        }
    }

    // Checks the execution condition of an instruction.
    virtual bool execCond (HybridCPU *xc) const = 0;

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
    void setNumDestOps (OpCount_t numDestOps)
    {
        this->numDestOps = numDestOps;
    }

    // Accessor and mutator of the source operand pointers.
    Op_t *getSrcOp (OpCount_t i)
    {
        return srcOp[i];
    }
    Op_t *getSrcOp (OpCount_t i) const
    {
        return srcOp[i];
    }
    void setSrcOp (OpCount_t i, Op_t *srcOp)
    {
        this->srcOp[i] = srcOp;
    }

    // Accessor and mutator of the destination operand pointers.
    Op_t *getDestOp (OpCount_t i)
    {
        return destOp[i];
    }
    Op_t *getDestOp (OpCount_t i) const
    {
        return destOp[i];
    }
    void setDestOp (OpCount_t i, Op_t *destOp)
    {
        this->destOp[i] = destOp;
    }

    // Accessor and mutator of static functional unit.
    FuncUnit_t getStaticFuncUnit (void) const
    {
        return staticFuncUnit;
    }
    void setStaticFuncUnit (FuncUnit_t staticFuncUnit)
    {
        this->staticFuncUnit = staticFuncUnit;
    }

    // Accessor and mutator of dynamic functional unit.
    FuncUnit_t getDynFuncUnit (void) const
    {
        return dynFuncUnit;
    }
    void setDynFuncUnit (FuncUnit_t dynFuncUnit)
    {
        this->dynFuncUnit = dynFuncUnit;
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

    // Accessor and mutator of the execution condition Z.
    bool getCondZ (void) const
    {
        return condZ;
    }
    void setCondZ (bool condZ)
    {
        this->condZ = condZ;
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
    std::string getStaticFuncUnitStr (void) const
    {
        return staticFuncUnitStr;
    }
    void setStaticFuncUnitStr (const char *staticFuncUnitStr)
    {
        (this->staticFuncUnitStr).assign (staticFuncUnitStr);
    }

    // Accessor and mutator of the string of dynamic functional unit.
    std::string getDynFuncUnitStr (void) const
    {
        return dynFuncUnitStr;
    }
    void setDynFuncUnitStr (const char *dynFuncUnitStr)
    {
        (this->dynFuncUnitStr).assign (dynFuncUnitStr);
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
    void setSrcOpStr (OpCount_t i, const char *str)
    {
        (this->srcOpStr[i]) += str;
    }

    // Accessor and mutator of the string of destination operand.
    std::string getDestOpStr (OpCount_t i) const
    {
        return destOpStr[i];
    }
    void setDestOpStr (OpCount_t i, const char *str)
    {
        (destOpStr[i]) += str;
    }

    void advancePC (TheISA::PCState &pcState) const {}

    // Accessor and mutator of branch address.
    Addr getBAddr (void) const
    {
        return bAddr;
    }
    void setBAddr (Addr bAddr)
    {
        this->bAddr = bAddr;
    }

    // Accessor and mutator of predict branch address.
    Addr getBPredAddr (void) const
    {
        return bPredAddr;
    }
    void setBPredAddr (Addr bPredAddr)
    {
        this->bPredAddr = bPredAddr;
    }

    // Accessor and mutator of branch predict result.
    bool getBPreded (void) const
    {
        return bPreded;
    }
    void setBPreded (bool bPreded)
    {
        this->bPreded = bPreded;
    }

    // Accessor and mutator of value address.
    Addr getVAddr (void) const
    {
        return vAddr;
    }
    void setVAddr (Addr vAddr)
    {
        this->vAddr = vAddr;
    }

    // Accessor and mutator of predict value address.
    Addr getVPredAddr (void) const
    {
        return vPredAddr;
    }
    void setVPredAddr (Addr vPredAddr)
    {
        this->vPredAddr = vPredAddr;
    }

    // Accessor and mutator of value predict result.
    bool getVPreded (void) const
    {
        return vPreded;
    }
    void setVPreded (bool vPreded)
    {
        this->vPreded = vPreded;
    }

  public:
    // Shows the operation of an instruction.
    std::string operate (void) const
    {
        return generateOperation ();
    }


  public:
    bool isNop (void) const { return flags[IsNop]; }
    bool isIter (void) const { return flags[IsIter]; }
    bool isIntDiv (void) const { return flags[IsIntDiv]; }
    bool isIntRem (void) const { return flags[IsIntRem]; }
    bool isMemRef (void) const { return flags[IsMemRef]; }
    bool isLoad (void) const { return flags[IsLoad]; }
    bool isStore (void) const { return flags[IsStore]; }
    bool isMemRefDS (void) const { return flags[IsMemRefDS]; }
    bool isLoadDS (void) const { return flags[IsLoadDS]; }
    bool isStoreDS (void) const { return flags[IsStoreDS]; }
    bool isControl (void) const { return flags[IsControl]; }
    bool isBranch (void) const { return flags[IsBranch]; }
    bool isCall (void) const { return flags[IsCall]; }
    bool isModeSwitch (void) const { return flags[IsModeSwitch]; }
    bool isToRisc (void) const { return flags[IsToRisc]; }
    bool isToVliw (void) const { return flags[IsToVliw]; }
    bool isSyscall (void) const { return flags[IsSyscall]; }

  public:
    // Machine instruction.
    MachInst machInst;

  protected:
    enum Flags {
        IsNop,

        IsIter,   // Iterative instruction.
        IsIntDiv, // Integer division instruction.
        IsIntRem, // Integer remainder instruction.

        IsMemRef, // Memory reference.
        IsLoad, // Load.
        IsStore, // Store.

        IsMemRefDS, // Memory reference with delay slot.
        IsLoadDS, // Load with delay slot.
        IsStoreDS, // Store with delay slot.

        IsControl, // Flow control.
        IsBranch,  // Jump.
        IsCall,    // Jump and link.

        IsModeSwitch, // Mode switching.
        IsToRisc, // Switch to Risc.
        IsToVliw, // Switch to Vliw.

        IsSyscall, // Syscall.

        NumFlags
    };

    // Decodes the functional unit from the given machine code.
    void decodeFuncUnit (MachInst insnFuncUnit)
    {
        OpcFuncUnit_t opcFuncUnit = Lily2ISA::getOpcFuncUnit (insnFuncUnit);
        setStaticFuncUnit (opcFuncUnit.FuncUnit);
        setStaticFuncUnitStr (opcFuncUnit.str);
    }

    // Decodes the condition from the given machine code.
    void decodeCond (MachInst insnCond)
    {
        OpcCond_t opcCond = Lily2ISA::getOpcCond (insnCond);
        setCond (opcCond.cond);
        setCondZ (opcCond.condZ);

        Op_t *op = opFactory (opcCond.opLabel);
        op->setImmFlag (false);
        op->setRegFile (opcCond.regFile);
        op->setRegIndex (opcCond.regIndex);

        setCondStr (opcCond.str);
    }

    // Decodes the source register from the given machine code.
    // OPTION = 0: Inner-Cluster.
    // OPTION = 1: Cross-Cluster.
    // OPTION = 2: Miscellaneous.
    void decodeSrcRegOp (OpLabel_t opLabel, MachInst insnSrcRegFile,
                         MachInst insnSrcRegIndex, int option = 0)
    {
        Op_t *op = opFactory (opLabel);

        OpcReg_t opcReg;
        switch (op->numRegs ()) {
            case 1 : opcReg = Lily2ISA::getOpcReg
                     (insnSrcRegFile, insnSrcRegIndex, option);
                     break;
            case 2 : opcReg = Lily2ISA::getOpcRegPair
                     (insnSrcRegFile, insnSrcRegIndex, option);
                     break;
            case 4 : opcReg = Lily2ISA::getOpcRegPairPair
                     (insnSrcRegFile, insnSrcRegIndex, option);
                     break;
            default: assert (0);
        }

        op->setImmFlag (false);
        op->setRegFile (opcReg.regFile);
        op->setRegIndex (opcReg.regIndex);

        OpCount_t i = getNumSrcOps ();
        setSrcOp (i, op);
        setSrcOpStr (i, opcReg.str);
        setNumSrcOps (i + 1);
    }

    void decodeSrcImmOp (OpLabel_t opLabel, MachInst insnSrcImm)
    {
        Op_t *op = opFactory (opLabel);

        op->setImmFlag (true);
        op->setImmValue (insnSrcImm);

        // 32-bit for printable immediate is enough.
        // 2 for ``0x'' and 8 for hex format immediate string.
        char *immStr = new char[10];
        immStr[0] = '0';
        immStr[1] = 'x';
        sprintf (immStr + 2, "%x", insnSrcImm);

        OpCount_t i = getNumSrcOps ();
        setSrcOp (i, op);
        setSrcOpStr (i, immStr);
        setNumSrcOps (i + 1);
    }

    // Decodes the destination register from the given machine code.
    // OPTION = 0: Inner-Cluster.
    // OPTION = 1: Cross-Cluster.
    // OPTION = 2: Miscellaneous.
    // OPTION = 3: Load destination.
    void decodeDestRegOp (OpLabel_t opLabel, MachInst insnDestRegFile,
                          MachInst insnDestRegIndex, int option = 0)
    {
        Op_t *op = opFactory (opLabel);

        // Load destination.
        if (option == 3) {
            op->setMemFlag (true);
            option = 0;
        } else {
            op->setMemFlag (false);
        }

        OpcReg_t opcReg;
        switch (op->numRegs ()) {
            case 1 : opcReg = Lily2ISA::getOpcReg
                     (insnDestRegFile, insnDestRegIndex, option);
                     break;
            case 2 : opcReg = Lily2ISA::getOpcRegPair
                     (insnDestRegFile, insnDestRegIndex, option);
                     break;
            case 4 : opcReg = Lily2ISA::getOpcRegPairPair
                     (insnDestRegFile, insnDestRegIndex, option);
                     break;
            default: assert (0);
        }

        op->setImmFlag (false);
        op->setRegFile (opcReg.regFile);
        op->setRegIndex (opcReg.regIndex);

        OpCount_t i = getNumDestOps ();
        setDestOp (i, op);
        setDestOpStr (i, opcReg.str);
        setNumDestOps (i + 1);
    }

    void printFuncUnit (std::stringstream &ss) const
    {
        ss << getStaticFuncUnitStr () << " ";
    }

    void printCond (std::stringstream &ss) const
    {
        ss << getCondStr () << " ";
    }

    void printName (std::stringstream &ss) const
    {
        ss << mnemonic << " ";
    }

    void printSrcOp (std::stringstream &ss, OpCount_t i) const
    {
        ss << getSrcOpStr (i);
    }

    void printDestOp (std::stringstream &ss, OpCount_t i) const
    {
        ss << getDestOpStr (i);
    }

    void printSrcOpValue (std::stringstream &ss, OpCount_t i) const
    {
        ss << (*getSrcOp (i));
    }

    void printDestOpValue (std::stringstream &ss, OpCount_t i) const
    {
        ss << *(getDestOp (i));
    }

    // Shows the results of the instructions.
    virtual std::string generateOperation (void) const = 0;

  protected:
    // Number of source and destination operands.
    OpCount_t numSrcOps;
    OpCount_t numDestOps;

    // Pointers of source and destination operands.
    Op_t *srcOp[MaxInstSrcOps];
    Op_t *destOp[MaxInstDestOps];

    // Functional unit of an instruction.
    // Static FU is for VLIW and dynamic FU is for Superscalar.
    FuncUnit_t staticFuncUnit;
    FuncUnit_t dynFuncUnit;

    // Execution condition of an instruction.
    Cond_t cond;
    bool condZ;
    Op_t *condOp;

    // Flags of an instruction.
    std::bitset<NumFlags> flags;

  protected:
    // Printable variables.

    // String of functional unit.
    std::string staticFuncUnitStr;
    std::string dynFuncUnitStr;

    // String of condition.
    std::string condStr;

    // String of operands.
    std::string srcOpStr[MaxInstSrcOps];
    std::string destOpStr[MaxInstDestOps];

  protected:
    // Prediction variables.

    // Branch prediction.
    Addr bAddr;
    Addr bPredAddr;
    bool bPreded;

    // Value prediction.
    Addr vAddr;
    Addr vPredAddr;
    bool vPreded;
};

// Smart pointer of LILY2STATICINST.
typedef RefCountingPtr<Lily2StaticInst> Lily2StaticInstPtr;

} // namespace Lily2ISAInst

#endif // __ARCH_LILY2_STATIC_INST_HH__
