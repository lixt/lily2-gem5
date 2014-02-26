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

// Logic instructions.
// bitAnd.
inline Op32i_t bitAnd (Op32i_t opa, Op32i_t opb)
{
    return Op32i_t (_bitAnd (opa.uval (), opb.uval ()));
}
inline Opq8i_t bitAnd_b_4 (Opq8i_t opa, Opq8i_t opb)
{
    return Opq8i_t (_bitAnd (opa.uvvh (), opb.uvvh ()),
                    _bitAnd (opa.uvhi (), opb.uvhi ()),
                    _bitAnd (opa.uvlo (), opb.uvlo ()),
                    _bitAnd (opa.uvvl (), opb.uvvl ()));
}
inline Opd16i_t bitAnd_h_2 (Opd16i_t opa, Opd16i_t opb)
{
    return Opd16i_t (_bitAnd (opa.uvhi (), opb.uvhi ()),
                     _bitAnd (opa.uvlo (), opb.uvlo ()));
}
inline Opq16i_t bitAnd_h_4 (Opq16i_t opa, Opq16i_t opb)
{
    return Opq16i_t (_bitAnd (opa.uvvh (), opb.uvvh ()),
                     _bitAnd (opa.uvhi (), opb.uvhi ()),
                     _bitAnd (opa.uvlo (), opb.uvlo ()),
                     _bitAnd (opa.uvvl (), opb.uvvl ()));
}
inline Opd32i_t bitAnd_w_2 (Opd32i_t opa, Opd32i_t opb)
{
    return Opd32i_t (_bitAnd (opa.uvhi (), opb.uvhi ()),
                     _bitAnd (opa.uvlo (), opb.uvlo ()));
}

// bitNad
inline Op32i_t bitNad (Op32i_t opa, Op32i_t opb)
{
    return Op32i_t (_bitNad (opa.uval (), opb.uval ()));
}
inline Opq8i_t bitNad_b_4 (Opq8i_t opa, Opq8i_t opb)
{
    return Opq8i_t (_bitNad (opa.uvvh (), opb.uvvh ()),
                    _bitNad (opa.uvhi (), opb.uvhi ()),
                    _bitNad (opa.uvlo (), opb.uvlo ()),
                    _bitNad (opa.uvvl (), opb.uvvl ()));
}
inline Opd16i_t bitNad_h_2 (Opd16i_t opa, Opd16i_t opb)
{
    return Opd16i_t (_bitNad (opa.uvhi (), opb.uvhi ()),
                     _bitNad (opa.uvlo (), opb.uvlo ()));
}
inline Opq16i_t bitNad_h_4 (Opq16i_t opa, Opq16i_t opb)
{
    return Opq16i_t (_bitNad (opa.uvvh (), opb.uvvh ()),
                     _bitNad (opa.uvhi (), opb.uvhi ()),
                     _bitNad (opa.uvlo (), opb.uvlo ()),
                     _bitNad (opa.uvvl (), opb.uvvl ()));
}
inline Opd32i_t bitNad_w_2 (Opd32i_t opa, Opd32i_t opb)
{
    return Opd32i_t (_bitNad (opa.uvhi (), opb.uvhi ()),
                     _bitNad (opa.uvlo (), opb.uvlo ()));
}

// bitOrr.
inline Op32i_t bitOrr (Op32i_t opa, Op32i_t opb)
{
    return Op32i_t (_bitOrr (opa.uval (), opb.uval ()));
}
inline Opq8i_t bitOrr_b_4 (Opq8i_t opa, Opq8i_t opb)
{
    return Opq8i_t (_bitOrr (opa.uvvh (), opb.uvvh ()),
                    _bitOrr (opa.uvhi (), opb.uvhi ()),
                    _bitOrr (opa.uvlo (), opb.uvlo ()),
                    _bitOrr (opa.uvvl (), opb.uvvl ()));
}
inline Opd16i_t bitOrr_h_2 (Opd16i_t opa, Opd16i_t opb)
{
    return Opd16i_t (_bitOrr (opa.uvhi (), opb.uvhi ()),
                     _bitOrr (opa.uvlo (), opb.uvlo ()));
}
inline Opq16i_t bitOrr_h_4 (Opq16i_t opa, Opq16i_t opb)
{
    return Opq16i_t (_bitOrr (opa.uvvh (), opb.uvvh ()),
                     _bitOrr (opa.uvhi (), opb.uvhi ()),
                     _bitOrr (opa.uvlo (), opb.uvlo ()),
                     _bitOrr (opa.uvvl (), opb.uvvl ()));
}
inline Opd32i_t bitOrr_w_2 (Opd32i_t opa, Opd32i_t opb)
{
    return Opd32i_t (_bitOrr (opa.uvhi (), opb.uvhi ()),
                     _bitOrr (opa.uvlo (), opb.uvlo ()));
}

// bitNor
inline Op32i_t bitNor (Op32i_t opa, Op32i_t opb)
{
    return Op32i_t (_bitNor (opa.uval (), opb.uval ()));
}
inline Opq8i_t bitNor_b_4 (Opq8i_t opa, Opq8i_t opb)
{
    return Opq8i_t (_bitNor (opa.uvvh (), opb.uvvh ()),
                    _bitNor (opa.uvhi (), opb.uvhi ()),
                    _bitNor (opa.uvlo (), opb.uvlo ()),
                    _bitNor (opa.uvvl (), opb.uvvl ()));
}
inline Opd16i_t bitNor_h_2 (Opd16i_t opa, Opd16i_t opb)
{
    return Opd16i_t (_bitNor (opa.uvhi (), opb.uvhi ()),
                     _bitNor (opa.uvlo (), opb.uvlo ()));
}
inline Opq16i_t bitNor_h_4 (Opq16i_t opa, Opq16i_t opb)
{
    return Opq16i_t (_bitNor (opa.uvvh (), opb.uvvh ()),
                     _bitNor (opa.uvhi (), opb.uvhi ()),
                     _bitNor (opa.uvlo (), opb.uvlo ()),
                     _bitNor (opa.uvvl (), opb.uvvl ()));
}
inline Opd32i_t bitNor_w_2 (Opd32i_t opa, Opd32i_t opb)
{
    return Opd32i_t (_bitNor (opa.uvhi (), opb.uvhi ()),
                     _bitNor (opa.uvlo (), opb.uvlo ()));
}

// bitXor.
inline Op32i_t bitXor (Op32i_t opa, Op32i_t opb)
{
    return Op32i_t (_bitXor (opa.uval (), opb.uval ()));
}
inline Opq8i_t bitXor_b_4 (Opq8i_t opa, Opq8i_t opb)
{
    return Opq8i_t (_bitXor (opa.uvvh (), opb.uvvh ()),
                    _bitXor (opa.uvhi (), opb.uvhi ()),
                    _bitXor (opa.uvlo (), opb.uvlo ()),
                    _bitXor (opa.uvvl (), opb.uvvl ()));
}
inline Opd16i_t bitXor_h_2 (Opd16i_t opa, Opd16i_t opb)
{
    return Opd16i_t (_bitXor (opa.uvhi (), opb.uvhi ()),
                     _bitXor (opa.uvlo (), opb.uvlo ()));
}
inline Opq16i_t bitXor_h_4 (Opq16i_t opa, Opq16i_t opb)
{
    return Opq16i_t (_bitXor (opa.uvvh (), opb.uvvh ()),
                     _bitXor (opa.uvhi (), opb.uvhi ()),
                     _bitXor (opa.uvlo (), opb.uvlo ()),
                     _bitXor (opa.uvvl (), opb.uvvl ()));
}
inline Opd32i_t bitXor_w_2 (Opd32i_t opa, Opd32i_t opb)
{
    return Opd32i_t (_bitXor (opa.uvhi (), opb.uvhi ()),
                     _bitXor (opa.uvlo (), opb.uvlo ()));
}

// bitNxr
inline Op32i_t bitNxr (Op32i_t opa, Op32i_t opb)
{
    return Op32i_t (_bitNxr (opa.uval (), opb.uval ()));
}
inline Opq8i_t bitNxr_b_4 (Opq8i_t opa, Opq8i_t opb)
{
    return Opq8i_t (_bitNxr (opa.uvvh (), opb.uvvh ()),
                    _bitNxr (opa.uvhi (), opb.uvhi ()),
                    _bitNxr (opa.uvlo (), opb.uvlo ()),
                    _bitNxr (opa.uvvl (), opb.uvvl ()));
}
inline Opd16i_t bitNxr_h_2 (Opd16i_t opa, Opd16i_t opb)
{
    return Opd16i_t (_bitNxr (opa.uvhi (), opb.uvhi ()),
                     _bitNxr (opa.uvlo (), opb.uvlo ()));
}
inline Opq16i_t bitNxr_h_4 (Opq16i_t opa, Opq16i_t opb)
{
    return Opq16i_t (_bitNxr (opa.uvvh (), opb.uvvh ()),
                     _bitNxr (opa.uvhi (), opb.uvhi ()),
                     _bitNxr (opa.uvlo (), opb.uvlo ()),
                     _bitNxr (opa.uvvl (), opb.uvvl ()));
}
inline Opd32i_t bitNxr_w_2 (Opd32i_t opa, Opd32i_t opb)
{
    return Opd32i_t (_bitNxr (opa.uvhi (), opb.uvhi ()),
                     _bitNxr (opa.uvlo (), opb.uvlo ()));
}

// bitNot
inline Op32i_t bitNot (Op32i_t op)
{
    return Op32i_t (_bitNot (op.uval ()));
}
inline Opq8i_t bitNot_b_4 (Opq8i_t op)
{
    return Opq8i_t (_bitNot (op.uvvh ()),
                    _bitNot (op.uvhi ()),
                    _bitNot (op.uvlo ()),
                    _bitNot (op.uvvl ()));
}
inline Opd16i_t bitNot_h_2 (Opd16i_t op)
{
    return Opd16i_t (_bitNot (op.uvhi ()),
                     _bitNot (op.uvlo ()));
}
inline Opq16i_t bitNot_h_4 (Opq16i_t op)
{
    return Opq16i_t (_bitNot (op.uvvh ()),
                     _bitNot (op.uvhi ()),
                     _bitNot (op.uvlo ()),
                     _bitNot (op.uvvl ()));
}
inline Opd32i_t bitNot_w_2 (Opd32i_t op)
{
    return Opd32i_t (_bitNot (op.uvhi ()),
                     _bitNot (op.uvlo ()));
}

// Test instructions.
inline Op32i_t tgt (Op32i_t opa, Op32i_t opb)
{
    return Op32i_t (_tgt (opa.sval (), opb.sval ()));
}
inline Op32i_t tgt_u (Op32i_t opa, Op32i_t opb)
{
    return Op32i_t (_tgt (opa.uval (), opb.uval ()));
}

inline Op32i_t tge (Op32i_t opa, Op32i_t opb)
{
    return Op32i_t (_tge (opa.sval (), opb.sval ()));
}
inline Op32i_t tge_u (Op32i_t opa, Op32i_t opb)
{
    return Op32i_t (_tge (opa.uval (), opb.uval ()));
}

inline Op32i_t tlt (Op32i_t opa, Op32i_t opb)
{
    return Op32i_t (_tlt (opa.sval (), opb.sval ()));
}
inline Op32i_t tlt_u (Op32i_t opa, Op32i_t opb)
{
    return Op32i_t (_tlt (opa.uval (), opb.uval ()));
}

inline Op32i_t tle (Op32i_t opa, Op32i_t opb)
{
    return Op32i_t (_tle (opa.sval (), opb.sval ()));
}
inline Op32i_t tle_u (Op32i_t opa, Op32i_t opb)
{
    return Op32i_t (_tle (opa.uval (), opb.uval ()));
}

inline Op32i_t teq (Op32i_t opa, Op32i_t opb)
{
    return Op32i_t (_teq (opa.uval (), opb.uval ()));
}
inline Op32i_t tne (Op32i_t opa, Op32i_t opb)
{
    return Op32i_t (_tne (opa.uval (), opb.uval ()));
}


inline Op32i_t mkl (Op32i_t op)
{
    return Op32i_t (_mkl (op.uval ()));
}

inline Op32i_t mkh (Op32i_t op)
{
    return Op32i_t (_mkh (op.uval ()));
}

} // namespace Lily2ISA

#endif // __ARCH_LILY2_ISA_EXEC_HH__
