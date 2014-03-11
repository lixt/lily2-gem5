/*
 * Copyright (C) 2014 DSP Group, Institute of Microelectronics, Tsinghua University.
 * All rights reserved.
 */

#ifndef __CPU_HYBRID_RESOURCES_VPRED_HH__
#define __CPU_HYBRID_RESOURCES_VPRED_HH__

#include "bimodal.hh"
#include "base/table.hh"
#include "arch/types.hh"

template <size_t Entries>
class VPredictor
{
  public:
    Addr predict (Addr instAddr) const
    {
        return 0;
    }

    void feedback (Addr instAddr, Addr effAddr)
    {
    }

    void feedbackLoadHistory (Addr instAddr, Addr effAddr)
    {
    }

    void feedbackStoreHistory (Addr instAddr, Addr effAddr)
    {
    }
};

#endif // __CPU_HYBRID_RESOURCES_VPRED_HH__
