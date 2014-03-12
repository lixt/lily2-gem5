/*
 * Copyright (C) 2014 DSP Group, Institute of Microelectronics, Tsinghua University.
 * All rights reserved.
 */

#ifndef __CPU_HYBRID_RESOURCES_REG_DEP_HH__
#define __CPU_HYBRID_RESOURCES_REG_DEP_HH__

#include "base/table.hh"
#include "arch/types.hh"
#include "arch/lily2/registers.hh"

template <size_t RegNum>
class RegDepTable : public Table<RegNum, 1, TheISA::RegIndex_t, Cycles>
{
  public:
    typedef TheISA::RegFile_t RegFile_t;
    typedef TheISA::RegIndex_t RegIndex_t;

  public:
    // Base class.
    typedef Table<RegNum, 1, RegIndex_t, Cycles> Base;
    // Base class types.
    typedef typename Base::key_type key_type;
    typedef typename Base::mapped_type mapped_type;
    typedef typename Base::value_type value_type;
    typedef typename Base::Position Position;

  public:
    // Inserts the register, register-pair, register-pair-pair and register back
    // cycle into the register dependence table.
    void insertReg (const RegIndex_t&, const Cycles&);

    // Mutates the register back cycle in the register dependence table.
    void mutateReg (const RegIndex_t&, const Cycles&);

    // Checks register, register-pair or register-pair-pair dependences. Returns
    // true if dependence exists.
    bool isRegDep (const RegIndex_t& regIndex) const;

    // Checks whether the register dependence table is empty.
    bool empty (void) const;

    // Returns the maximum register back cycle in the register dependence table.
    Cycles maxCycle (void) const;

    // Updates the register dependence table. Decreases the register back cycles
    // and removes out the due registers.
    void update (const Cycles& regBackCycleDelta);

    // Prints readable debug infos of the register dependence table.
    template <size_t ShadowRegNum>
    friend std::ostream& operator<< (std::ostream&, const RegDepTable<ShadowRegNum>&);

  private:
    // Bottom implementation of the output operator.
    void print (std::ostream&) const;

    // Decreases the register back cycles in the register dependence table.
    std::vector<Position> decrRegBackCycle (const Cycles& regBackCycleDelta);

    // Removes out the due registers.
    void remove (const Position& removePos);

  private:
    // Functor used in DECRREGBACKCYCLE.
    struct DecrRegBackCycleFunctor
    {
        Cycles regBackCycleDelta;

        // Constructor.
        explicit DecrRegBackCycleFunctor (const Cycles& regBackCycleDelta)
            : regBackCycleDelta (regBackCycleDelta) {}

        bool operator() (key_type& key, mapped_type& mapped)
        {
            if (mapped <= regBackCycleDelta) {
                mapped = Cycles ();
                return true;
            } else {
                mapped = mapped - regBackCycleDelta;
                return false;
            }
        }
    };

    // Functor used in MAXCYCLE.
    struct MaxCycleFunctor
    {
        Cycles max;

        // Constructor.
        MaxCycleFunctor (void) : max (0) {}

        void operator() (const key_type& key, const mapped_type& mapped)
        {
            max = (max < mapped) ? mapped : max;
        }
    };

    // Funtor used in PRINT.
    struct PrintFunctor
    {
        void operator() (std::ostream& os, const key_type& key, const mapped_type& mapped)
        {
            os << "(" << key << "," << mapped << ")";
        }
    };
};

template <size_t RegNum>
void
RegDepTable<RegNum>::insertReg (const RegIndex_t& regIndex, const Cycles& regBackCycle)
{
    Base::insert (regIndex, regBackCycle);
}

template <size_t RegNum>
void
RegDepTable<RegNum>::mutateReg (const RegIndex_t& regIndex, const Cycles& regBackCycle)
{
    Position mutatePos = Base::search (regIndex);
    Base::mutate (mutatePos, regBackCycle);
}

template <size_t RegNum>
bool
RegDepTable<RegNum>::isRegDep (const RegIndex_t& regIndex) const
{
    Position searchPos = Base::search (regIndex);
    return (searchPos == Base::nil ()) ? false : true;
}

template <size_t RegNum>
bool
RegDepTable<RegNum>::empty (void) const
{
    for (size_t i = 0; i != RegNum; ++i) {
        if (isRegDep (i)) {
            return false;
        }
    }

    return true;
}

template <size_t RegNum>
Cycles
RegDepTable<RegNum>::maxCycle (void) const
{
    MaxCycleFunctor maxCycleFunctor;
    Base::traverse (maxCycleFunctor);
    return maxCycleFunctor.max;
}

template <size_t RegNum>
void
RegDepTable<RegNum>::update (const Cycles& regBackCycleDelta)
{
    std::vector<Position> removePosVec = decrRegBackCycle (regBackCycleDelta);

    for (typename std::vector<Position>::iterator it = removePosVec.begin ();
         it != removePosVec.end (); ++it) {
        remove (*it);
    }
}

template <size_t RegNum>
std::vector<typename RegDepTable<RegNum>::Position>
RegDepTable<RegNum>::decrRegBackCycle (const Cycles& regBackCycleDelta)
{
    DecrRegBackCycleFunctor decrRegBackCycleFunctor (regBackCycleDelta);
    return Base::traverseAndReturn (decrRegBackCycleFunctor);
}

template <size_t RegNum>
void
RegDepTable<RegNum>::remove (const Position& removePos)
{
    Base::remove (removePos);
}

template <size_t RegNum>
void
RegDepTable<RegNum>::print (std::ostream& os) const
{
    PrintFunctor pfunc;
    Base::print (os, pfunc);
}

template <size_t ShadowRegNum>
std::ostream&
operator<< (std::ostream& os, const RegDepTable<ShadowRegNum>& regDepTable)
{
    regDepTable.print (os);
    return os;
}

#endif // __CPU_HYBRID_RESOURCES_REG_DEP_HH__
