/*
 * Copyright (C) 2014 DSP Group, Institute of Microelectronics, Tsinghua University.
 * All rights reserved.
 */

#ifndef __CPU_HYBRID_RESOURCES_BPRED_HH__
#define __CPU_HYBRID_RESOURCES_BPRED_HH__

#include "base/table.hh"
#include "arch/types.hh"

class Bimodal
{
  public:
    // Constructor.
    Bimodal (void) : curState (0) {}

  public:
    // Returns 1 if branch is predicted taken.
    bool getBranchTaken (void) const
    {
        switch (curState) {
            case 0x0: return false;
            case 0x1: return false;
            case 0x2: return true;
            case 0x3: return true;
            default : assert (0);
        }
    }

    // Sets the state according to the feedback.
    void setBranchTaken (bool branchTaken)
    {
        switch (curState) {
            case 0x0: curState = branchTaken ? 0x1 : 0x0; break;
            case 0x1: curState = branchTaken ? 0x2 : 0x0; break;
            case 0x2: curState = branchTaken ? 0x3 : 0x2; break;
            case 0x3: curState = branchTaken ? 0x2 : 0x3; break;
            default : assert (0);
        }
    }

  private:
    int curState;
};

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
