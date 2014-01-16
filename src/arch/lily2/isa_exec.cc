/*
 * Copyright (C) 2014 DSP Group, Institute of Microelectronics, Tsinghua University
 * All rights reserved.
 */

#include "arch/lily2/isa_exec.hh"
#include "arch/lily2/isa_util.hh"

namespace Lily2ISA
{
OpWord_t add (OpWord_t opa, OpWord_t opb)
{
    return OpWord_t (_add (opa.sval (), opb.sval ()));
}

} // namespace Lily2ISA
