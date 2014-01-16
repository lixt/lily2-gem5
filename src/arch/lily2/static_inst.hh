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
    typedef TheISA::OpCount_t OpCount_t;
    typedef TheISA::OpLabel_t OpLabel_t;
    typedef TheISA::Op_t Op_t;
    typedef TheISA::FU_t FU_t;

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

    void decodeSrcOp (OpLabel_t opLabel, ExtMachInst insnSrcReg)
    {
        ;
    }

    void decodeDestOp (OpLabel_t opLabel, ExtMachInst insnDestReg)
    {
        ;
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
    bool _condZ;
    Op_t *_condOp;

    // Flags of an instruction.
    std::bitset<NumFlags> flags;
};

} // namespace Lily2ISAInst

#endif // __ARCH_LILY2_STATIC_INST_HH__
