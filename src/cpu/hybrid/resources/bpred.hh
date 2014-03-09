/*
 * Copyright (C) 2014 DSP Group, Institute of Microelectronics, Tsinghua University.
 * All rights reserved.
 */

#ifndef __CPU_HYBRID_RESOURCES_BPRED_HH__
#define __CPU_HYBRID_RESOURCES_BPRED_HH__

#include "bimodal.hh"
#include "base/table.hh"
#include "arch/types.hh"

template <size_t CorrNum>
struct EntryType
{
    Bimodal bimodal[CorrNum];
    Addr branchTarget[CorrNum];
};

template <size_t EntryNum, size_t CorrNum>
class BPredictor : public Table<EntryNum, 1, Addr, EntryType<CorrNum>>
{
  public:
    // Predicts the branch behaviour according to the given address.
    // If the branch is predicted taken, returns the predicted branch target.
    // Otherwise, returns 0.
    Addr predict (Addr instAddr) const
    {
        return 0;
    }

    void feedback (Addr instAddr, Addr branchTarget)
    {
    }
};

#endif // __CPU_HYBRID_RESOURCES_BPRED_HH__
