/*
 * Copyright (C) 2014 DSP Group, Institute of Microelectronics, Tsinghua University.
 * All rights reserved.
 */

#ifndef __CPU_HYBRID_RESOURCES_REG_DEP_HH__
#define __CPU_HYBRID_RESOURCES_REG_DEP_HH__

#include "base/table.hh"
#include "arch/types.hh"
#include "arch/lily2/registers.hh"

template <TheISA::RegFile_t FileName, size_t RegNum>
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
    // Inserts the register and register back cycle into the register dependence
    // table.
    void insert (const RegIndex_t& regIndex, const Cycles& regBackCycle);

    // Checks register, register-pair or register-pair-pair dependences. Returns
    // true if dependence exists.
    bool isRegDep (const RegIndex_t& regIndex) const;
    bool isRegPairDep (const RegIndex_t& regIndex) const;
    bool isRegPairPairDep (const RegIndex_t& regIndex) const;

    // Updates the register dependence table. Decreases the register back cycles
    // and removes out the due registers.
    void update (const Cycles& regBackCycleDelta);

  private:
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
        explicit DecrRegBackCycleFunctor (const Cycles& regbackCycleDelta)
            : regBackCycleDelta (regBackCycleDelta) {}

        bool operator() (const key_type& key, const mapped_type& mapped)
        {
            if (mapped <= regBackCycleDelta) {
                mapped = 0;
                return true;
            } else {
                mapped -= regBackCycleDelta;
                return false;
            }
        }
    };
};

template <TheISA::RegFile_t FileName, size_t RegNum>
void
RegDepTable<FileName, RegNum>::insert (const RegIndex_t& regIndex, const Cycles& regBackCycle)
{
    Base::insert (regIndex, regBackCycle);
}

template <TheISA::RegFile_t FileName, size_t RegNum>
bool
RegDepTable<FileName, RegNum>::isRegDep (const RegIndex_t& regIndex) const
{
    Position searchPos = Base::search (regIndex);
    return (searchPos == Base::nil ()) ? false : true;
}

template <TheISA::RegFile_t FileName, size_t RegNum>
bool
RegDepTable<FileName, RegNum>::isRegPairDep (const RegIndex_t& regIndex) const
{
    return isRegDep (regIndex) ||
           isRegDep (regIndex + 1);
}

template <TheISA::RegFile_t FileName, size_t RegNum>
bool
RegDepTable<FileName, RegNum>::isRegPairPairDep (const RegIndex_t& regIndex) const
{
    return isRegDep (regIndex) ||
           isRegDep (regIndex + 1) ||
           isRegDep (regIndex + 2) ||
           isRegDep (regIndex + 3);
}

template <TheISA::RegFile_t FileName, size_t RegNum>
void
RegDepTable<FileName, RegNum>::update (const Cycles& regBackCycleDelta)
{
    std::vector<Position> removePosVec = decrRegBackCycle (regBackCycleDelta);

    for (typename std::vector<Position>::iterator it = removePosVec.begin ();
         it != removePosVec.end (); ++it) {
        remove (*it);
    }
}

template <TheISA::RegFile_t FileName, size_t RegNum>
std::vector<typename RegDepTable<FileName, RegNum>::Position>
RegDepTable<FileName, RegNum>::decrRegBackCycle (const Cycles& regBackCycleDelta)
{
    DecrRegBackCycleFunctor decrRegBackCycleFunctor (regBackCycleDelta);
    Base::traverseAndReturn (decrRegBackCycleFunctor);
}

template <TheISA::RegFile_t FileName, size_t RegNum>
void
RegDepTable<FileName, RegNum>::remove (const Position& removePos)
{
    Base::remove (removePos);
}

#endif // __CPU_HYBRID_RESOURCES_REG_DEP_HH__
