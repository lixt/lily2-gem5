/*
 * Copyright (C) 2014 DSP Group, Institute of Microelectronics, Tsinghua University
 * All rights reserved.
 */

#include "arch/lily2/isa_exec.hh"
#include "arch/lily2/isa_util.hh"

namespace Lily2ISA
{
Op32i_t add (Op32i_t opa, Op32i_t opb)
{
    return Op32i_t (_add (opa.sval (), opb.sval ()));
}

} // namespace Lily2ISA
