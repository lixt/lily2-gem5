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
    OP_NIL,

    OP_32I,
    OP_32F,
    OP_64F,
    OP_Q8I,
    OP_D16I,
    OP_Q16I,
    OP_D32I,
    OP_D32F,

    NUM_OP_LABEL
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

class Op32i_t : public Op_t
{
  public:
    // Constructors.
    Op32i_t (void) : Op_t ()
    {
        setUval (0);
    }
    explicit Op32i_t (int32_t val) : Op_t ()
    {
        setSval (val);
    }
    explicit Op32i_t (uint32_t val) : Op_t ()
    {
        setUval (val);
    }

    // Copy constructor.
    Op32i_t (const Op32i_t &op)
        : Op_t (op._regFile, op._regIndex, op._immFlag)
    {
        setUval (op.val.uval);
    }

  public:
    // Implements pure virtual function.
    // Accesses number of registers an operand contains.
    RegCount_t numRegs (void) const { return NumRegs; }

    // Implements pure virtual function.
    // Prints readable operand value.
    void print (std::ostream &os) const {}

    // Accessors and mutators of the operand value.
    int32_t  sval (void) const { return val.sval; }
    uint32_t uval (void) const { return val.uval; }
    void setSval (int32_t  val) { this->val.sval = val; }
    void setUval (uint32_t val) { this->val.uval = val; }

  private:
    // Number of registers a word contains.
    static const RegCount_t NumRegs = 1;

  private:
    union {
        int32_t  sval;
        uint32_t uval;
    } val;
};

class Op32f_t : public Op_t
{
  public:
    // Constructors.
    Op32f_t (void) : Op_t ()
    {
        setFval (0.0);
    }
    explicit Op32f_t (float val) : Op_t ()
    {
        setFval (val);
    }

    // Copy constructor.
    Op32f_t (const Op32f_t &op)
        : Op_t (op._regFile, op._regIndex, op._immFlag)
    {
        setFval (op.val.fval);
    }

  public:
    // Implements pure virtual function.
    // Accesses number of registers an operand contains.
    RegCount_t numRegs (void) const { return NumRegs; }

    // Implements pure virtual function.
    // Prints readable operand value.
    void print (std::ostream &os) const {}

    // Accessor and mutator of operand value.
    float fval (void) const { return val.fval; }
    void setFval (float val) { this->val.fval = val; }

  private:
    // Number of registers a word contains.
    static const RegCount_t NumRegs = 1;

  private:
    union {
        float fval;
    } val;
};

class Op64f_t : public Op_t
{
  public:
    // Constructors.
    Op64f_t (void) : Op_t ()
    {
        setFval (0.0);
    }
    explicit Op64f_t (double val) : Op_t ()
    {
        setFval (val);
    }

    // Copy constructor.
    Op64f_t (const Op64f_t &op)
        : Op_t (op._regFile, op._regIndex, op._immFlag)
    {
        setFval (op.val.fval);
    }

  public:
    // Implements pure virtual function.
    // Accesses number of registers an operand contains.
    RegCount_t numRegs (void) const { return NumRegs; }

    // Implements pure virtual function.
    // Prints readable operand value.
    void print (std::ostream &os) const {}

    // Accessor and mutators of operand value.
    float fval (void) const { return val.fval; }
    void setFval (double val) { this->val.fval = val; }

  private:
    // Number of registers a word contains.
    static const RegCount_t NumRegs = 2;

  private:
    union {
        double fval;
    } val;
};

class Opd32i_t : public Op_t
{
  public:
    // Constructors.
    Opd32i_t (void) : Op_t ()
    {
        setUvhi (0);
        setUvlo (0);
    }
    explicit Opd32i_t (int32_t val) : Op_t ()
    {
        setSvhi (val);
        setSvlo (val);
    }
    explicit Opd32i_t (uint32_t val) : Op_t ()
    {
        setUvhi (val);
        setUvlo (val);
    }
    Opd32i_t (int32_t vhi, int32_t vlo) : Op_t ()
    {
        setUvhi (vhi);
        setUvlo (vlo);
    }
    Opd32i_t (uint32_t vhi, uint32_t vlo) : Op_t ()
    {
        setSvhi (vhi);
        setSvlo (vlo);
    }

    // Copy constructor.
    Opd32i_t (const Opd32i_t &op)
        : Op_t (op._regFile, op._regIndex, op._immFlag)
    {
        setUvhi (op.vhi.uval);
        setUvlo (op.vlo.uval);
    }

  public:
    // Implements pure virtual function.
    // Accesses number of registers an operand contains.
    RegCount_t numRegs (void) const { return NumRegs; }

    // Implements pure virtual function.
    // Prints readable operand value.
    void print (std::ostream &os) const {}

    // Accessors and mutators of oeprand value.
    int32_t  svhi (void) const { return vhi.sval; }
    int32_t  svlo (void) const { return vlo.sval; }
    uint32_t uvhi (void) const { return vhi.uval; }
    uint32_t uvlo (void) const { return vlo.uval; }
    void setSvhi (int32_t vhi) { this->vhi.sval = vhi; }
    void setSvlo (int32_t vlo) { this->vlo.sval = vlo; }
    void setUvhi (uint32_t vhi) { this->vhi.uval = vhi; }
    void setUvlo (uint32_t vlo) { this->vlo.uval = vlo; }

  private:
    // Number of registers a word contains.
    static const RegCount_t NumRegs = 2;

  private:
    union {
        int32_t  sval;
        uint32_t uval;
    } vhi,vlo;
};

class Opq8i_t : public Op_t
{
  public:
    // Constructors.
    Opq8i_t (void) : Op_t ()
    {
        setUvvh (0);
        setUvhi (0);
        setUvlo (0);
        setUvvl (0);
    }
    explicit Opq8i_t (int8_t val) : Op_t ()
    {
        setSvvh (val);
        setSvhi (val);
        setSvlo (val);
        setSvvl (val);
    }
    explicit Opq8i_t (uint8_t val) : Op_t ()
    {
        setUvvh (val);
        setUvhi (val);
        setUvlo (val);
        setUvvl (val);
    }
    Opq8i_t (int8_t vvh, int8_t vhi, int8_t vlo, int8_t vvl) : Op_t ()
    {
        setSvvh (vvh);
        setSvhi (vhi);
        setSvlo (vlo);
        setSvvl (vvl);
    }
    Opq8i_t (uint8_t vvh, uint8_t vhi, uint8_t vlo, uint8_t vvl) : Op_t ()
    {
        setUvvh (vvh);
        setUvhi (vhi);
        setUvlo (vlo);
        setUvvl (vvl);
    }

    // Copy constructor.
    Opq8i_t (const Opq8i_t &op)
        : Op_t (op._regFile, op._regIndex, op._immFlag)
    {
        setUvvh (op.vvh.uval);
        setUvhi (op.vhi.uval);
        setUvlo (op.vlo.uval);
        setUvvl (op.vvl.uval);
    }

  public:
    // Implements pure virtual function.
    // Accesses number of registers an operand contains.
    RegCount_t numRegs (void) const { return NumRegs; }

    // Implements pure virtual function.
    // Prints readable operand value.
    void print (std::ostream &os) const {}

    // Accessors and mutators of operand value.
    int8_t  svvh (void) const { return vvh.sval; }
    int8_t  svhi (void) const { return vhi.sval; }
    int8_t  svlo (void) const { return vlo.sval; }
    int8_t  svvl (void) const { return vvl.sval; }
    uint8_t uvvh (void) const { return vvh.uval; }
    uint8_t uvhi (void) const { return vhi.uval; }
    uint8_t uvlo (void) const { return vlo.uval; }
    uint8_t uvvl (void) const { return vvl.uval; }
    void setSvvh (int8_t vvh) { this->vvh.sval = vvh; }
    void setSvhi (int8_t vhi) { this->vhi.sval = vhi; }
    void setSvlo (int8_t vlo) { this->vlo.sval = vlo; }
    void setSvvl (int8_t vvl) { this->vvl.sval = vvl; }
    void setUvvh (uint8_t vvh) { this->vvh.uval = vvh; }
    void setUvhi (uint8_t vhi) { this->vhi.uval = vhi; }
    void setUvlo (uint8_t vlo) { this->vlo.uval = vlo; }
    void setUvvl (uint8_t vvl) { this->vvl.uval = vvl; }

  private:
    // Number of registers a word contains.
    static const RegCount_t NumRegs = 1;

  private:
    union {
        int8_t  sval;
        uint8_t uval;
    } vvh, vhi, vlo, vvl;
};
/*
class Opd16i_t : public Op_t
{
  public:
    // Constructors.
    Opd16i_t (void)
        : Op_t (), vhi.uval (0), vlo.uval (0) {}
    explicit Opd16i_t (int16_t val)
        : Op_t (), vhi.sval (val), vlo.sval (val) {}
    explicit Opd16i_t (uint16_t val)
        : Op_t (), vhi.uval (val), vlo.uval (val) {}
    Opd16i_t (int16_t vhi, int16_t vlo)
        : Op_t (), vhi.sval (vhi), vlo.sval (vlo) {}
    Opd16i_t (uint16_t vhi, uint16_t vlo)
        : Op_t (), vhi.uval (vhi), vlo.uval (vlo) {}

    // Copy constructor.
    Opd16i_t (const Opd16i_t &op)
        : Op_t (op._regFile, op._regIndex, op._immFlag),
          vhi.uval (op.vhi.uval), vlo.uval (op.vlo.uval) {}

  public:
    // Implements pure virtual function.
    // Accesses number of registers an operand contains.
    RegCount_t numRegs (void) const { return NumRegs; }

    // Implements pure virtual function.
    // Prints readable operand value.
    void print (std::ostream &os) const {}

    // Accesses the value of operand.
    int16_t  svhi (void) const { return vhi.sval; }
    int16_t  svlo (void) const { return vlo.sval; }
    uint16_t uvhi (void) const { return vhi.uval; }
    uint16_t uvlo (void) const { return vlo.uval; }

  private:
    // Number of registers a word contains.
    static const RegCount_t NumRegs = 1;

  private:
    union {
        int16_t  sval;
        uint16_t uval;
    } vhi, vlo;
};

class Opq16i_t : public Op_t
{
  public:
    // Constructors.
    Opq16i_t (void)
        : Op_t (), vvh.uval (0), vhi.uval (0), vlo.uval (0), vvl.uval (0) {}
    explicit Opq16i_t (int16_t val)
        : Op_t (), vvh.sval (val), vhi.sval (val), vlo.sval (val), vvl.sval (val) {}
    explicit Opq16i_t (uint16_t val)
        : Op_t (), vvh.uval (val), vhi.uval (val), vlo.uval (val), vvl.uval (val) {}
    Opq16i_t (int16_t vvh, int16_t vhi, int16_t vlo, int16_t vvl)
        : Op_t (), vvh.sval (vvh), vhi.sval (vhi), vlo.sval (vlo), vvl.sval (vvl) {}
    Opq16i_t (uint16_t vvh, uint16_t vhi, uint16_t vlo, uint16_t vvl)
        : Op_t (), vvh.uval (vvh), vhi.uval (vhi), vlo.uval (vlo), vvl.uval (vvl) {}

    // Copy constructor.
    Opq16i_t (const Opq16i_t &op)
        : Op_t (op._regFile, op._regIndex, op._immFlag),
          vvh.uval (op.vvh.uval), vhi.uval (op.vhi.uval),
          vlo.uval (op.vlo.uval), vvl.uval (op.vvl.uval) {}

  public:
    // Implements pure virtual function.
    // Accesses number of registers an operand contains.
    RegCount_t numRegs (void) const { return NumRegs; }

    // Implements pure virtual function.
    // Prints readable operand value.
    void print (std::ostream &os) const {}

    // Accesses the value of operand.
    int16_t  svvh (void) const { return vvh.sval; }
    int16_t  svhi (void) const { return vhi.sval; }
    int16_t  svlo (void) const { return vlo.sval; }
    int16_t  svvl (void) const { return vvl.sval; }
    uint16_t uvvh (void) const { return vvh.uval; }
    uint16_t uvhi (void) const { return vhi.uval; }
    uint16_t uvlo (void) const { return vlo.uval; }
    uint16_t uvvl (void) const { return vvl.uval; }

  private:
    // Number of registers a word contains.
    static const RegCount_t NumRegs = 2;

  private:
    union {
        int16_t  sval;
        uint16_t uval;
    } vvh, vhi, vlo, vvl;
};

class Opd32f_t : public Op_t
{
  public:
    // Constructors.
    Opd32f_t (void)
        : Op_t (), vhi.fval (0.0), vlo.fval (0.0) {}
    explicit Opd32f_t (float val)
        : Op_t (), vhi.fval (val), vlo.fval (val) {}
    explicit Opd32f_t (double val)
        : Op_t (), vhi.fval (val), vlo.fval (val) {}

    // Copy constructor.
    Opd32f_t (const Opd32f_t &op)
        : Op_t (op._regFile, op._regIndex, op._immFlag),
          vhi.fval (op.vhi.fval), vlo.fval (op.vlo.fval) {}

  public:
    // Implements pure virtual function.
    // Accesses number of registers an operand contains.
    RegCount_t numRegs (void) const { return NumRegs; }

    // Implements pure virtual function.
    // Prints readable operand value.
    void print (std::ostream &os) const {}

    // Accesses the value of operand.
    float fvhi (void) const { return vhi.fval; }
    float fvlo (void) const { return vlo.fval; }

  private:
    // Number of registers a word contains.
    static const RegCount_t NumRegs = 2;

  private:
    union {
        float fval;
    } vhi, vlo;
};*/

inline
Op32i_t maskGenOp32i (int first, int last)
{
    return Op32i_t (static_cast <uint32_t> (mask (first, last)));
}

} // namespace Lily2ISA

#endif // __ARCH_LILY2_OPERANDS_HH__
