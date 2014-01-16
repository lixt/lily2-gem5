/*
 * Copyright (C) DSP Group, Institute of Microelectronics, Tsinghua University
 * All rights reserved.
 */

#ifndef __ARCH_LILY2_OPERANDS_HH__
#define __ARCH_LILY2_OPERANDS_HH__

#include "base/types.hh"
#include "base/bitfield.hh"
#include "arch/registers.hh"

namespace Lily2ISA
{
// Type for operand counting.
typedef int8_t OpCount_t;

// Type for operand labels.
typedef enum OpLabel_t
{
    OP_NIL,      // Illegal.

    OP_WORD,     // Word.
    OP_SP,       // Single-Precision.
    OP_DP,       // Double-Precision.
    OP_BYTE_4,   // 4-Way byte vector.
    OP_HWORD_2,  // 2-Way halfword vector.
    OP_HWORD_4,  // 4-Way halfword vector.
    OP_WORD_2,   // 2-Way word vector.
    OP_SP_2,     // 2-Way single-precision.

    NUM_OP_TAG   // Number of operand tags.
} OpLabel_t;

class Op_t
{
  public:
    // Constructors.
    Op_t (void) :
        _regFile (REG_NIL), _regIndex (0), _immFlag (0) {}
    Op_t (RegFile_t regFile, RegIndex regIndex, bool immFlag) :
        _regFile (regFile), _regIndex (regIndex), _immFlag (immFlag) {}

    // Accesses the number of registers an operand contains.
    virtual RegCount_t numRegs (void) const = 0;

    // Accesses the register file of an operand.
    virtual RegFile_t regFile (void) const
    {
        return _regFile;
    }
    // Mutates the register file of an operand.
    virtual void setRegFile (RegFile_t _regFile)
    {
        this->_regFile = _regFile;
    }

    // Accesses the index of registers an operand contains.
    virtual RegIndex_t regIndex (void) const
    {
        return _regIndex;
    }
    // Mutates the index of registers an operand contains.
    virtual void setRegIndex (RegIndex_t _regIndex)
    {
        this->_regIndex = _regIndex;
    }

    // Accesses the immediate operand identifier.
    virtual bool immFlag (void) const
    {
        return _immFlag;
    }
    // Mutates the immediate operand identifier.
    virtual void setImmFlag (bool _immFlag)
    {
        this->_immFlag = _immFlag;
    }

  protected:
    // Prints readable operand value.
    virtual void print (std::ostream &os) const = 0;

    // Register file and index to record a register.
    RegFile_t _regFile;
    RegIndex_t _regIndex;

    // Immediate operand identifier, 1 indicates an immediate.
    bool _immFlag;

  private:
    // Implements the virtualization of output operator ``<<''.
    friend std::ostream& operator<< (std::ostream &os, Op_t &op)
    {
        op.print (os);
        return os;
    }
};

class OpWord_t : public Op_t
{
  public:
    // Constructors.
    OpWord_t (void) : Op_t (), _val (0) {}
    explicit OpWord_t (int32_t val) : Op_t (), _val (val) {}
    explicit OpWord_t (uint32_t val) : Op_t (), _val (val) {}

    // Copy constructor.
    OpWord_t (const OpWord_t &op)
        : Op_t (op._regFile, op._regIndex, op._immFlag),
          _val (op._val) {}


    // Implements pure virtual function.
    // Accesses number of registers an operand contains.
    RegCount_t numRegs (void) const { return NumRegs; }

    // Implements pure virtual function.
    // Prints readable operand value.
    void print (std::ostream &os) const {}

    // Accesses the value of operand.
    int32_t sval (void) const { return static_cast<int32_t> (_val); }
    uint32_t uval (void) const { return static_cast<uint32_t> (_val); }

  private:
    // Number of registers a word contains.
    static const RegCount_t NumRegs = 1;

    // Value of operand.
    uint32_t _val;
};

inline
OpWord_t genWordOperandMask (int first, int last)
{
    return OpWord_t (static_cast<uint32_t> (mask (first, last)));
}

/*
class OpSP_t : public Op_t
{
  public:
    // Constructor.
    OpSP_t (void) : Op_t (), _val (0.0) {}

    // Implements pure virtual functions.
    // Prints readable operand value.
    void print (std::ostream &os) const
    {
    }

    // Implements pure virtual functions.
    // Accesses the number of registers an operand contained.
    RegCount_t numRegs (void) const { return NumRegs; }

    // Accesses the value of operand.
    float val (void) const { return _val; }

  private:
    // Number of registers an operand contained.
    static const RegCount_t NumRegs = 1;

    // Value of operand.
    float _val;
};

class OpSP_2_t : public Op_t
{
  public:
    // Constructor.
    OpSP_2_t (void) : Op_t (), _vhi (0.0), _vlo (0.0) {}

    // Implements pure virtual functions.
    // Prints readable operand value.
    void print (std::ostream &os) const
    {
    }

    // Implements pure virtual functions.
    // Accesses the number of registers an operand contained.
    RegCount_t numRegs (void) const { return NumRegs; }

    // Accesses the value of operand.
    float vhi (void) const { return _vhi; }
    float vlo (void) const { return _vlo; }
    // Mutates the value of operand.
    void setVhi (float vhi_) { _vhi = vhi_; }
    void setVlo (float vlo_) { _vlo = vlo_; }

  private:
    // Number of registers an operand contained.
    static const RegCount_t NumRegs = 2;

    // Value of operand.
    float _vhi, _vlo;
};*/


} // namespace Lily2ISA

#endif // __ARCH_LILY2_OPERANDS_HH__
