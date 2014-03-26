/*
 * Copyright (C) 2014 DSP Group, Institute of Microelectronics, Tsinghua University.
 * All rights reserved.
 */

#ifndef __CPU_HYBRID_RESOURCES_BPRED_HH__
#define __CPU_HYBRID_RESOURCES_BPRED_HH__

#include "bimodal.hh"
#include "base/bitfield.hh"
#include "base/table.hh"
#include "arch/types.hh"

template <size_t Correlations>
struct EntryType
{
    Bimodal bimodal[Correlations];
    Addr branchTarget[Correlations];
};

template <size_t Entries, size_t CorrBits>
class BTB : public Table<Entries, 1, Addr, EntryType<(1 << CorrBits)>>
{
  private:
    typedef Table<Entries, 1, Addr, EntryType<(1 << CorrBits)>> Base;
    typedef typename Base::Position Position;
    typedef typename Base::key_type key_type;
    typedef typename Base::mapped_type mapped_type;

  public:
    Addr predict (Addr instAddr, int64_t corr) const;

    void feedback (Addr instAddr, int64_t corr, Addr branchTarget);

  private:
    bool getBimodal (Position accessPos, int64_t corr) const
    {
        return (Base::access (accessPos)).bimodal[corr].getAdopt ();
    }

    Addr getBranchTarget (Position accessPos, int64_t corr) const
    {
        return (Base::access (accessPos)).branchTarget[corr];
    }

    void setBimodal (Position mutatePos, int64_t corr, bool taken)
    {
        mapped_type mapped = Base::access (mutatePos);
        mapped.bimodal[corr].setAdopt (taken);
        Base::mutate (mutatePos, mapped);
    }

    void setBranchTarget (Position mutatePos, int64_t corr, Addr branchTarget)
    {
        mapped_type mapped = Base::access (mutatePos);
        mapped.branchTarget[corr] = branchTarget;
        Base::mutate (mutatePos, mapped);
    }
};

template <size_t Entries, size_t CorrBits>
Addr
BTB<Entries, CorrBits>::predict (Addr instAddr, int64_t corr) const
{
    Position accessPos = Base::search (instAddr);

    if (accessPos == Base::nil ()) {
        return 0;
    } else {
        return (getBimodal (accessPos, corr) ? getBranchTarget (accessPos, corr) : 0);
    }
}

template <size_t Entries, size_t CorrBits>
void
BTB<Entries, CorrBits>::feedback (Addr instAddr, int64_t corr, Addr branchTarget)
{
    Position mutatePos = Base::search (instAddr);

    if (mutatePos == Base::nil ()) { // Replaces.
        mapped_type mapped;
        Base::insert (instAddr, mapped);
    } else { // Mutates.

        if (branchTarget) { // Actual taken.
            setBimodal (mutatePos, corr, true);
            setBranchTarget (mutatePos, corr, branchTarget);
        } else { // Actual not taken.
            setBimodal (mutatePos, corr, false);
        }
    }
}

template <size_t Entries, size_t CorrBits>
class BPredictor
{
  public:
    // Constructor.
    BPredictor (void) : corr (0) {}

  public:
    Addr predict (Addr instAddr) const;

    // Feeds back the branch info to the branch predictor.
    // If the branch is not taken, sets the BRANCHTARGET 0.
    void feedback (Addr instAddr, Addr branchTarget);

  private:
    // Feeds back the correlation branches.
    // If the branch is not taken, sets the BRANCHTARGET 0.
    void feedbackCorr (Addr branchTarget);

  private:
    // BTB table.
    BTB<Entries, CorrBits> btb;

    // Correlation branches.
    uint64_t corr;
};

template <size_t Entries, size_t CorrBits>
Addr
BPredictor<Entries, CorrBits>::predict (Addr instAddr) const
{
    return btb.predict (instAddr, corr);
}

template <size_t Entries, size_t CorrBits>
void
BPredictor<Entries, CorrBits>::feedback (Addr instAddr, Addr branchTarget)
{
    btb.feedback (instAddr, corr, branchTarget);
    feedbackCorr (branchTarget);
}

template <size_t Entries, size_t CorrBits>
void
BPredictor<Entries, CorrBits>::feedbackCorr (Addr branchTarget)
{
    int taken = (branchTarget != 0) ? 1 : 0;
    corr = (corr << 1) + taken;
    corr = bits (corr, CorrBits - 1, 0);
}



#endif // __CPU_HYBRID_RESOURCES_BPRED_HH__
