/*
 * Copyright (C) 2014 DSP Group, Institute of Microelectronics, Tsinghua University
 * All rights reserved.
 */

#ifndef __ARCH_LILY2_ISA_EXEC_HH__
#define __ARCH_LILY2_ISA_EXEC_HH__

#include "arch/lily2/operands.hh"
#include "arch/lily2/isa_util.hh"

namespace Lily2ISA
{
inline Op32i_t add (Op32i_t opa, Op32i_t opb)
{
    return Op32i_t (_add (opa.sval (), opb.sval ()));
}
inline Opq8i_t add_b_4 (Opq8i_t opa, Opq8i_t opb)
{
    return Opq8i_t (_add (opa.svvh (), opb.svvh ()),
                    _add (opa.svhi (), opb.svhi ()),
                    _add (opa.svlo (), opb.svlo ()),
                    _add (opa.svvl (), opb.svvl ()));
}
inline Opd16i_t add_h_2 (Opd16i_t opa, Opd16i_t opb)
{
    return Opd16i_t (_add (opa.svhi (), opb.svhi ()),
                     _add (opa.svlo (), opb.svlo ()));
}
inline Opq16i_t add_h_4 (Opq16i_t opa, Opq16i_t opb)
{
    return Opq16i_t (_add (opa.svvh (), opb.svvh ()),
                     _add (opa.svhi (), opb.svhi ()),
                     _add (opa.svlo (), opb.svlo ()),
                     _add (opa.svvl (), opb.svvl ()));
}
inline Opd32i_t add_w_2 (Opd32i_t opa, Opd32i_t opb)
{
    return Opd32i_t (_add (opa.svhi (), opb.svhi ()),
                     _add (opa.svlo (), opb.svlo ()));
}

} // namespace Lily2ISA

#endif // __ARCH_LILY2_ISA_EXEC_HH__
