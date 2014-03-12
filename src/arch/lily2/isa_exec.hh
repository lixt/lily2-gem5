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
// Arithmetic instructions.
// ADD.
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

// ADC.
inline Op32i_t adc (Op32i_t opa, Op32i_t opb, Op32i_t opc)
{
    return Op32i_t (_adc (opa.sval (), opb.sval (), opc.sval ()));
}

// SUB.
inline Op32i_t sub (Op32i_t opa, Op32i_t opb)
{
    return Op32i_t (_sub (opa.sval (), opb.sval ()));
}
inline Opq8i_t sub_b_4 (Opq8i_t opa, Opq8i_t opb)
{
    return Opq8i_t (_sub (opa.svvh (), opb.svvh ()),
                    _sub (opa.svhi (), opb.svhi ()),
                    _sub (opa.svlo (), opb.svlo ()),
                    _sub (opa.svvl (), opb.svvl ()));
}
inline Opd16i_t sub_h_2 (Opd16i_t opa, Opd16i_t opb)
{
    return Opd16i_t (_sub (opa.svhi (), opb.svhi ()),
                     _sub (opa.svlo (), opb.svlo ()));
}
inline Opq16i_t sub_h_4 (Opq16i_t opa, Opq16i_t opb)
{
    return Opq16i_t (_sub (opa.svvh (), opb.svvh ()),
                     _sub (opa.svhi (), opb.svhi ()),
                     _sub (opa.svlo (), opb.svlo ()),
                     _sub (opa.svvl (), opb.svvl ()));
}
inline Opd32i_t sub_w_2 (Opd32i_t opa, Opd32i_t opb)
{
    return Opd32i_t (_sub (opa.svhi (), opb.svhi ()),
                     _sub (opa.svlo (), opb.svlo ()));
}

// SBC.
inline Op32i_t sbc (Op32i_t opa, Op32i_t opb, Op32i_t opc)
{
    return Op32i_t (_sbc (opa.sval (), opb.sval (), opc.sval ()));
}

// RSB.
inline Op32i_t rsb (Op32i_t opa, Op32i_t opb)
{
    return Op32i_t (_rsb (opa.sval (), opb.sval ()));
}
inline Opq8i_t rsb_b_4 (Opq8i_t opa, Opq8i_t opb)
{
    return Opq8i_t (_rsb (opa.svvh (), opb.svvh ()),
                    _rsb (opa.svhi (), opb.svhi ()),
                    _rsb (opa.svlo (), opb.svlo ()),
                    _rsb (opa.svvl (), opb.svvl ()));
}
inline Opd16i_t rsb_h_2 (Opd16i_t opa, Opd16i_t opb)
{
    return Opd16i_t (_rsb (opa.svhi (), opb.svhi ()),
                     _rsb (opa.svlo (), opb.svlo ()));
}
inline Opq16i_t rsb_h_4 (Opq16i_t opa, Opq16i_t opb)
{
    return Opq16i_t (_rsb (opa.svvh (), opb.svvh ()),
                     _rsb (opa.svhi (), opb.svhi ()),
                     _rsb (opa.svlo (), opb.svlo ()),
                     _rsb (opa.svvl (), opb.svvl ()));
}
inline Opd32i_t rsb_w_2 (Opd32i_t opa, Opd32i_t opb)
{
    return Opd32i_t (_rsb (opa.svhi (), opb.svhi ()),
                     _rsb (opa.svlo (), opb.svlo ()));
}

// RSC.
inline Op32i_t rsc (Op32i_t opa, Op32i_t opb, Op32i_t opc)
{
    return Op32i_t (_rsc (opa.sval (), opb.sval (), opc.sval ()));
}

// ABS.
inline Op32i_t abs (Op32i_t op)
{
    return Op32i_t (_abs (op.sval ()));
}
inline Opq8i_t abs_b_4 (Opq8i_t op)
{
    return Opq8i_t (_abs (op.svvh ()),
                    _abs (op.svhi ()),
                    _abs (op.svlo ()),
                    _abs (op.svvl ()));
}
inline Opd16i_t abs_h_2 (Opd16i_t op)
{
    return Opd16i_t (_abs (op.svhi ()),
                     _abs (op.svlo ()));
}
inline Opq16i_t abs_h_4 (Opq16i_t op)
{
    return Opq16i_t (_abs (op.svvh ()),
                     _abs (op.svhi ()),
                     _abs (op.svlo ()),
                     _abs (op.svvl ()));
}
inline Opd32i_t abs_w_2 (Opd32i_t op)
{
    return Opd32i_t (_abs (op.svhi ()),
                     _abs (op.svlo ()));
}

// NEG.
inline Op32i_t neg (Op32i_t op)
{
    return Op32i_t (_neg (op.sval ()));
}
inline Opq8i_t neg_b_4 (Opq8i_t op)
{
    return Opq8i_t (_neg (op.svvh ()),
                    _neg (op.svhi ()),
                    _neg (op.svlo ()),
                    _neg (op.svvl ()));
}
inline Opd16i_t neg_h_2 (Opd16i_t op)
{
    return Opd16i_t (_neg (op.svhi ()),
                     _neg (op.svlo ()));
}
inline Opq16i_t neg_h_4 (Opq16i_t op)
{
    return Opq16i_t (_neg (op.svvh ()),
                     _neg (op.svhi ()),
                     _neg (op.svlo ()),
                     _neg (op.svvl ()));
}
inline Opd32i_t neg_w_2 (Opd32i_t op)
{
    return Opd32i_t (_neg (op.svhi ()),
                     _neg (op.svlo ()));
}

// SXB.
inline Op32i_t sxb (Op32i_t op)
{
    return Op32i_t (_sxb (op.uval ()));
}

// SXH.
inline Op32i_t sxh (Op32i_t op)
{
    return Op32i_t (_sxh (op.uval ()));
}

// ZXB.
inline Op32i_t zxb (Op32i_t op)
{
    return Op32i_t (_zxb (op.uval ()));
}

// ZXH.
inline Op32i_t zxh (Op32i_t op)
{
    return Op32i_t (_zxh (op.uval ()));
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

inline Op32i_t gto_b_4 (Opq8i_t opa, Opq8i_t opb)
{
    return Op32i_t (_tgt (opa.svvh (), opb.svvh ()) ||
                    _tgt (opa.svhi (), opb.svhi ()) ||
                    _tgt (opa.svlo (), opb.svlo ()) ||
                    _tgt (opa.svvl (), opb.svvl ()));
}
inline Op32i_t gto_h_2 (Opd16i_t opa, Opd16i_t opb)
{
    return Op32i_t (_tgt (opa.svhi (), opb.svhi ()) ||
                    _tgt (opa.svlo (), opb.svlo ()));
}
inline Op32i_t gto_h_4 (Opq16i_t opa, Opq16i_t opb)
{
    return Op32i_t (_tgt (opa.svvh (), opb.svvh ()) ||
                    _tgt (opa.svhi (), opb.svhi ()) ||
                    _tgt (opa.svlo (), opb.svlo ()) ||
                    _tgt (opa.svvl (), opb.svvl ()));
}
inline Op32i_t gto_w_2 (Opd32i_t opa, Opd32i_t opb)
{
    return Op32i_t (_tgt (opa.svhi (), opb.svhi ()) ||
                    _tgt (opa.svlo (), opb.svlo ()));
}

inline Op32i_t gto_u_b_4 (Opq8i_t opa, Opq8i_t opb)
{
    return Op32i_t (_tgt (opa.uvvh (), opb.uvvh ()) ||
                    _tgt (opa.uvhi (), opb.uvhi ()) ||
                    _tgt (opa.uvlo (), opb.uvlo ()) ||
                    _tgt (opa.uvvl (), opb.uvvl ()));
}
inline Op32i_t gto_u_h_2 (Opd16i_t opa, Opd16i_t opb)
{
    return Op32i_t (_tgt (opa.uvhi (), opb.uvhi ()) ||
                    _tgt (opa.uvlo (), opb.uvlo ()));
}
inline Op32i_t gto_u_h_4 (Opq16i_t opa, Opq16i_t opb)
{
    return Op32i_t (_tgt (opa.uvvh (), opb.uvvh ()) ||
                    _tgt (opa.uvhi (), opb.uvhi ()) ||
                    _tgt (opa.uvlo (), opb.uvlo ()) ||
                    _tgt (opa.uvvl (), opb.uvvl ()));
}
inline Op32i_t gto_u_w_2 (Opd32i_t opa, Opd32i_t opb)
{
    return Op32i_t (_tgt (opa.uvhi (), opb.uvhi ()) ||
                    _tgt (opa.uvlo (), opb.uvlo ()));
}

inline Op32i_t gta_b_4 (Opq8i_t opa, Opq8i_t opb)
{
    return Op32i_t (_tgt (opa.svvh (), opb.svvh ()) &&
                    _tgt (opa.svhi (), opb.svhi ()) &&
                    _tgt (opa.svlo (), opb.svlo ()) &&
                    _tgt (opa.svvl (), opb.svvl ()));
}
inline Op32i_t gta_h_2 (Opd16i_t opa, Opd16i_t opb)
{
    return Op32i_t (_tgt (opa.svhi (), opb.svhi ()) &&
                    _tgt (opa.svlo (), opb.svlo ()));
}
inline Op32i_t gta_h_4 (Opq16i_t opa, Opq16i_t opb)
{
    return Op32i_t (_tgt (opa.svvh (), opb.svvh ()) &&
                    _tgt (opa.svhi (), opb.svhi ()) &&
                    _tgt (opa.svlo (), opb.svlo ()) &&
                    _tgt (opa.svvl (), opb.svvl ()));
}
inline Op32i_t gta_w_2 (Opd32i_t opa, Opd32i_t opb)
{
    return Op32i_t (_tgt (opa.svhi (), opb.svhi ()) &&
                    _tgt (opa.svlo (), opb.svlo ()));
}

inline Op32i_t gta_u_b_4 (Opq8i_t opa, Opq8i_t opb)
{
    return Op32i_t (_tgt (opa.uvvh (), opb.uvvh ()) &&
                    _tgt (opa.uvhi (), opb.uvhi ()) &&
                    _tgt (opa.uvlo (), opb.uvlo ()) &&
                    _tgt (opa.uvvl (), opb.uvvl ()));
}
inline Op32i_t gta_u_h_2 (Opd16i_t opa, Opd16i_t opb)
{
    return Op32i_t (_tgt (opa.uvhi (), opb.uvhi ()) &&
                    _tgt (opa.uvlo (), opb.uvlo ()));
}
inline Op32i_t gta_u_h_4 (Opq16i_t opa, Opq16i_t opb)
{
    return Op32i_t (_tgt (opa.uvvh (), opb.uvvh ()) &&
                    _tgt (opa.uvhi (), opb.uvhi ()) &&
                    _tgt (opa.uvlo (), opb.uvlo ()) &&
                    _tgt (opa.uvvl (), opb.uvvl ()));
}
inline Op32i_t gta_u_w_2 (Opd32i_t opa, Opd32i_t opb)
{
    return Op32i_t (_tgt (opa.uvhi (), opb.uvhi ()) &&
                    _tgt (opa.uvlo (), opb.uvlo ()));
}

inline Op32i_t geo_b_4 (Opq8i_t opa, Opq8i_t opb)
{
    return Op32i_t (_tge (opa.svvh (), opb.svvh ()) ||
                    _tge (opa.svhi (), opb.svhi ()) ||
                    _tge (opa.svlo (), opb.svlo ()) ||
                    _tge (opa.svvl (), opb.svvl ()));
}
inline Op32i_t geo_h_2 (Opd16i_t opa, Opd16i_t opb)
{
    return Op32i_t (_tge (opa.svhi (), opb.svhi ()) ||
                    _tge (opa.svlo (), opb.svlo ()));
}
inline Op32i_t geo_h_4 (Opq16i_t opa, Opq16i_t opb)
{
    return Op32i_t (_tge (opa.svvh (), opb.svvh ()) ||
                    _tge (opa.svhi (), opb.svhi ()) ||
                    _tge (opa.svlo (), opb.svlo ()) ||
                    _tge (opa.svvl (), opb.svvl ()));
}
inline Op32i_t geo_w_2 (Opd32i_t opa, Opd32i_t opb)
{
    return Op32i_t (_tge (opa.svhi (), opb.svhi ()) ||
                    _tge (opa.svlo (), opb.svlo ()));
}

inline Op32i_t geo_u_b_4 (Opq8i_t opa, Opq8i_t opb)
{
    return Op32i_t (_tge (opa.uvvh (), opb.uvvh ()) ||
                    _tge (opa.uvhi (), opb.uvhi ()) ||
                    _tge (opa.uvlo (), opb.uvlo ()) ||
                    _tge (opa.uvvl (), opb.uvvl ()));
}
inline Op32i_t geo_u_h_2 (Opd16i_t opa, Opd16i_t opb)
{
    return Op32i_t (_tge (opa.uvhi (), opb.uvhi ()) ||
                    _tge (opa.uvlo (), opb.uvlo ()));
}
inline Op32i_t geo_u_h_4 (Opq16i_t opa, Opq16i_t opb)
{
    return Op32i_t (_tge (opa.uvvh (), opb.uvvh ()) ||
                    _tge (opa.uvhi (), opb.uvhi ()) ||
                    _tge (opa.uvlo (), opb.uvlo ()) ||
                    _tge (opa.uvvl (), opb.uvvl ()));
}
inline Op32i_t geo_u_w_2 (Opd32i_t opa, Opd32i_t opb)
{
    return Op32i_t (_tge (opa.uvhi (), opb.uvhi ()) ||
                    _tge (opa.uvlo (), opb.uvlo ()));
}

inline Op32i_t gea_b_4 (Opq8i_t opa, Opq8i_t opb)
{
    return Op32i_t (_tge (opa.svvh (), opb.svvh ()) &&
                    _tge (opa.svhi (), opb.svhi ()) &&
                    _tge (opa.svlo (), opb.svlo ()) &&
                    _tge (opa.svvl (), opb.svvl ()));
}
inline Op32i_t gea_h_2 (Opd16i_t opa, Opd16i_t opb)
{
    return Op32i_t (_tge (opa.svhi (), opb.svhi ()) &&
                    _tge (opa.svlo (), opb.svlo ()));
}
inline Op32i_t gea_h_4 (Opq16i_t opa, Opq16i_t opb)
{
    return Op32i_t (_tge (opa.svvh (), opb.svvh ()) &&
                    _tge (opa.svhi (), opb.svhi ()) &&
                    _tge (opa.svlo (), opb.svlo ()) &&
                    _tge (opa.svvl (), opb.svvl ()));
}
inline Op32i_t gea_w_2 (Opd32i_t opa, Opd32i_t opb)
{
    return Op32i_t (_tge (opa.svhi (), opb.svhi ()) &&
                    _tge (opa.svlo (), opb.svlo ()));
}

inline Op32i_t gea_u_b_4 (Opq8i_t opa, Opq8i_t opb)
{
    return Op32i_t (_tge (opa.uvvh (), opb.uvvh ()) &&
                    _tge (opa.uvhi (), opb.uvhi ()) &&
                    _tge (opa.uvlo (), opb.uvlo ()) &&
                    _tge (opa.uvvl (), opb.uvvl ()));
}
inline Op32i_t gea_u_h_2 (Opd16i_t opa, Opd16i_t opb)
{
    return Op32i_t (_tge (opa.uvhi (), opb.uvhi ()) &&
                    _tge (opa.uvlo (), opb.uvlo ()));
}
inline Op32i_t gea_u_h_4 (Opq16i_t opa, Opq16i_t opb)
{
    return Op32i_t (_tge (opa.uvvh (), opb.uvvh ()) &&
                    _tge (opa.uvhi (), opb.uvhi ()) &&
                    _tge (opa.uvlo (), opb.uvlo ()) &&
                    _tge (opa.uvvl (), opb.uvvl ()));
}
inline Op32i_t gea_u_w_2 (Opd32i_t opa, Opd32i_t opb)
{
    return Op32i_t (_tge (opa.uvhi (), opb.uvhi ()) &&
                    _tge (opa.uvlo (), opb.uvlo ()));
}

inline Op32i_t lto_b_4 (Opq8i_t opa, Opq8i_t opb)
{
    return Op32i_t (_tlt (opa.svvh (), opb.svvh ()) ||
                    _tlt (opa.svhi (), opb.svhi ()) ||
                    _tlt (opa.svlo (), opb.svlo ()) ||
                    _tlt (opa.svvl (), opb.svvl ()));
}
inline Op32i_t lto_h_2 (Opd16i_t opa, Opd16i_t opb)
{
    return Op32i_t (_tlt (opa.svhi (), opb.svhi ()) ||
                    _tlt (opa.svlo (), opb.svlo ()));
}
inline Op32i_t lto_h_4 (Opq16i_t opa, Opq16i_t opb)
{
    return Op32i_t (_tlt (opa.svvh (), opb.svvh ()) ||
                    _tlt (opa.svhi (), opb.svhi ()) ||
                    _tlt (opa.svlo (), opb.svlo ()) ||
                    _tlt (opa.svvl (), opb.svvl ()));
}
inline Op32i_t lto_w_2 (Opd32i_t opa, Opd32i_t opb)
{
    return Op32i_t (_tlt (opa.svhi (), opb.svhi ()) ||
                    _tlt (opa.svlo (), opb.svlo ()));
}

inline Op32i_t lto_u_b_4 (Opq8i_t opa, Opq8i_t opb)
{
    return Op32i_t (_tlt (opa.uvvh (), opb.uvvh ()) ||
                    _tlt (opa.uvhi (), opb.uvhi ()) ||
                    _tlt (opa.uvlo (), opb.uvlo ()) ||
                    _tlt (opa.uvvl (), opb.uvvl ()));
}
inline Op32i_t lto_u_h_2 (Opd16i_t opa, Opd16i_t opb)
{
    return Op32i_t (_tlt (opa.uvhi (), opb.uvhi ()) ||
                    _tlt (opa.uvlo (), opb.uvlo ()));
}
inline Op32i_t lto_u_h_4 (Opq16i_t opa, Opq16i_t opb)
{
    return Op32i_t (_tlt (opa.uvvh (), opb.uvvh ()) ||
                    _tlt (opa.uvhi (), opb.uvhi ()) ||
                    _tlt (opa.uvlo (), opb.uvlo ()) ||
                    _tlt (opa.uvvl (), opb.uvvl ()));
}
inline Op32i_t lto_u_w_2 (Opd32i_t opa, Opd32i_t opb)
{
    return Op32i_t (_tlt (opa.uvhi (), opb.uvhi ()) ||
                    _tlt (opa.uvlo (), opb.uvlo ()));
}

inline Op32i_t lta_b_4 (Opq8i_t opa, Opq8i_t opb)
{
    return Op32i_t (_tlt (opa.svvh (), opb.svvh ()) &&
                    _tlt (opa.svhi (), opb.svhi ()) &&
                    _tlt (opa.svlo (), opb.svlo ()) &&
                    _tlt (opa.svvl (), opb.svvl ()));
}
inline Op32i_t lta_h_2 (Opd16i_t opa, Opd16i_t opb)
{
    return Op32i_t (_tlt (opa.svhi (), opb.svhi ()) &&
                    _tlt (opa.svlo (), opb.svlo ()));
}
inline Op32i_t lta_h_4 (Opq16i_t opa, Opq16i_t opb)
{
    return Op32i_t (_tlt (opa.svvh (), opb.svvh ()) &&
                    _tlt (opa.svhi (), opb.svhi ()) &&
                    _tlt (opa.svlo (), opb.svlo ()) &&
                    _tlt (opa.svvl (), opb.svvl ()));
}
inline Op32i_t lta_w_2 (Opd32i_t opa, Opd32i_t opb)
{
    return Op32i_t (_tlt (opa.svhi (), opb.svhi ()) &&
                    _tlt (opa.svlo (), opb.svlo ()));
}

inline Op32i_t lta_u_b_4 (Opq8i_t opa, Opq8i_t opb)
{
    return Op32i_t (_tlt (opa.uvvh (), opb.uvvh ()) &&
                    _tlt (opa.uvhi (), opb.uvhi ()) &&
                    _tlt (opa.uvlo (), opb.uvlo ()) &&
                    _tlt (opa.uvvl (), opb.uvvl ()));
}
inline Op32i_t lta_u_h_2 (Opd16i_t opa, Opd16i_t opb)
{
    return Op32i_t (_tlt (opa.uvhi (), opb.uvhi ()) &&
                    _tlt (opa.uvlo (), opb.uvlo ()));
}
inline Op32i_t lta_u_h_4 (Opq16i_t opa, Opq16i_t opb)
{
    return Op32i_t (_tlt (opa.uvvh (), opb.uvvh ()) &&
                    _tlt (opa.uvhi (), opb.uvhi ()) &&
                    _tlt (opa.uvlo (), opb.uvlo ()) &&
                    _tlt (opa.uvvl (), opb.uvvl ()));
}
inline Op32i_t lta_u_w_2 (Opd32i_t opa, Opd32i_t opb)
{
    return Op32i_t (_tlt (opa.uvhi (), opb.uvhi ()) &&
                    _tlt (opa.uvlo (), opb.uvlo ()));
}

inline Op32i_t leo_b_4 (Opq8i_t opa, Opq8i_t opb)
{
    return Op32i_t (_tle (opa.svvh (), opb.svvh ()) ||
                    _tle (opa.svhi (), opb.svhi ()) ||
                    _tle (opa.svlo (), opb.svlo ()) ||
                    _tle (opa.svvl (), opb.svvl ()));
}
inline Op32i_t leo_h_2 (Opd16i_t opa, Opd16i_t opb)
{
    return Op32i_t (_tle (opa.svhi (), opb.svhi ()) ||
                    _tle (opa.svlo (), opb.svlo ()));
}
inline Op32i_t leo_h_4 (Opq16i_t opa, Opq16i_t opb)
{
    return Op32i_t (_tle (opa.svvh (), opb.svvh ()) ||
                    _tle (opa.svhi (), opb.svhi ()) ||
                    _tle (opa.svlo (), opb.svlo ()) ||
                    _tle (opa.svvl (), opb.svvl ()));
}
inline Op32i_t leo_w_2 (Opd32i_t opa, Opd32i_t opb)
{
    return Op32i_t (_tle (opa.svhi (), opb.svhi ()) ||
                    _tle (opa.svlo (), opb.svlo ()));
}

inline Op32i_t leo_u_b_4 (Opq8i_t opa, Opq8i_t opb)
{
    return Op32i_t (_tle (opa.uvvh (), opb.uvvh ()) ||
                    _tle (opa.uvhi (), opb.uvhi ()) ||
                    _tle (opa.uvlo (), opb.uvlo ()) ||
                    _tle (opa.uvvl (), opb.uvvl ()));
}
inline Op32i_t leo_u_h_2 (Opd16i_t opa, Opd16i_t opb)
{
    return Op32i_t (_tle (opa.uvhi (), opb.uvhi ()) ||
                    _tle (opa.uvlo (), opb.uvlo ()));
}
inline Op32i_t leo_u_h_4 (Opq16i_t opa, Opq16i_t opb)
{
    return Op32i_t (_tle (opa.uvvh (), opb.uvvh ()) ||
                    _tle (opa.uvhi (), opb.uvhi ()) ||
                    _tle (opa.uvlo (), opb.uvlo ()) ||
                    _tle (opa.uvvl (), opb.uvvl ()));
}
inline Op32i_t leo_u_w_2 (Opd32i_t opa, Opd32i_t opb)
{
    return Op32i_t (_tle (opa.uvhi (), opb.uvhi ()) ||
                    _tle (opa.uvlo (), opb.uvlo ()));
}

inline Op32i_t lea_b_4 (Opq8i_t opa, Opq8i_t opb)
{
    return Op32i_t (_tle (opa.svvh (), opb.svvh ()) &&
                    _tle (opa.svhi (), opb.svhi ()) &&
                    _tle (opa.svlo (), opb.svlo ()) &&
                    _tle (opa.svvl (), opb.svvl ()));
}
inline Op32i_t lea_h_2 (Opd16i_t opa, Opd16i_t opb)
{
    return Op32i_t (_tle (opa.svhi (), opb.svhi ()) &&
                    _tle (opa.svlo (), opb.svlo ()));
}
inline Op32i_t lea_h_4 (Opq16i_t opa, Opq16i_t opb)
{
    return Op32i_t (_tle (opa.svvh (), opb.svvh ()) &&
                    _tle (opa.svhi (), opb.svhi ()) &&
                    _tle (opa.svlo (), opb.svlo ()) &&
                    _tle (opa.svvl (), opb.svvl ()));
}
inline Op32i_t lea_w_2 (Opd32i_t opa, Opd32i_t opb)
{
    return Op32i_t (_tle (opa.svhi (), opb.svhi ()) &&
                    _tle (opa.svlo (), opb.svlo ()));
}

inline Op32i_t lea_u_b_4 (Opq8i_t opa, Opq8i_t opb)
{
    return Op32i_t (_tle (opa.uvvh (), opb.uvvh ()) &&
                    _tle (opa.uvhi (), opb.uvhi ()) &&
                    _tle (opa.uvlo (), opb.uvlo ()) &&
                    _tle (opa.uvvl (), opb.uvvl ()));
}
inline Op32i_t lea_u_h_2 (Opd16i_t opa, Opd16i_t opb)
{
    return Op32i_t (_tle (opa.uvhi (), opb.uvhi ()) &&
                    _tle (opa.uvlo (), opb.uvlo ()));
}
inline Op32i_t lea_u_h_4 (Opq16i_t opa, Opq16i_t opb)
{
    return Op32i_t (_tle (opa.uvvh (), opb.uvvh ()) &&
                    _tle (opa.uvhi (), opb.uvhi ()) &&
                    _tle (opa.uvlo (), opb.uvlo ()) &&
                    _tle (opa.uvvl (), opb.uvvl ()));
}
inline Op32i_t lea_u_w_2 (Opd32i_t opa, Opd32i_t opb)
{
    return Op32i_t (_tle (opa.uvhi (), opb.uvhi ()) &&
                    _tle (opa.uvlo (), opb.uvlo ()));
}

inline Op32i_t eqo_b_4 (Opq8i_t opa, Opq8i_t opb)
{
    return Op32i_t (_teq (opa.svvh (), opb.svvh ()) ||
                    _teq (opa.svhi (), opb.svhi ()) ||
                    _teq (opa.svlo (), opb.svlo ()) ||
                    _teq (opa.svvl (), opb.svvl ()));
}
inline Op32i_t eqo_h_2 (Opd16i_t opa, Opd16i_t opb)
{
    return Op32i_t (_teq (opa.svhi (), opb.svhi ()) ||
                    _teq (opa.svlo (), opb.svlo ()));
}
inline Op32i_t eqo_h_4 (Opq16i_t opa, Opq16i_t opb)
{
    return Op32i_t (_teq (opa.svvh (), opb.svvh ()) ||
                    _teq (opa.svhi (), opb.svhi ()) ||
                    _teq (opa.svlo (), opb.svlo ()) ||
                    _teq (opa.svvl (), opb.svvl ()));
}
inline Op32i_t eqo_w_2 (Opd32i_t opa, Opd32i_t opb)
{
    return Op32i_t (_teq (opa.svhi (), opb.svhi ()) ||
                    _teq (opa.svlo (), opb.svlo ()));
}

inline Op32i_t eqa_b_4 (Opq8i_t opa, Opq8i_t opb)
{
    return Op32i_t (_teq (opa.svvh (), opb.svvh ()) &&
                    _teq (opa.svhi (), opb.svhi ()) &&
                    _teq (opa.svlo (), opb.svlo ()) &&
                    _teq (opa.svvl (), opb.svvl ()));
}
inline Op32i_t eqa_h_2 (Opd16i_t opa, Opd16i_t opb)
{
    return Op32i_t (_teq (opa.svhi (), opb.svhi ()) &&
                    _teq (opa.svlo (), opb.svlo ()));
}
inline Op32i_t eqa_h_4 (Opq16i_t opa, Opq16i_t opb)
{
    return Op32i_t (_teq (opa.svvh (), opb.svvh ()) &&
                    _teq (opa.svhi (), opb.svhi ()) &&
                    _teq (opa.svlo (), opb.svlo ()) &&
                    _teq (opa.svvl (), opb.svvl ()));
}
inline Op32i_t eqa_w_2 (Opd32i_t opa, Opd32i_t opb)
{
    return Op32i_t (_teq (opa.svhi (), opb.svhi ()) &&
                    _teq (opa.svlo (), opb.svlo ()));
}

inline Op32i_t neo_b_4 (Opq8i_t opa, Opq8i_t opb)
{
    return Op32i_t (_tne (opa.svvh (), opb.svvh ()) ||
                    _tne (opa.svhi (), opb.svhi ()) ||
                    _tne (opa.svlo (), opb.svlo ()) ||
                    _tne (opa.svvl (), opb.svvl ()));
}
inline Op32i_t neo_h_2 (Opd16i_t opa, Opd16i_t opb)
{
    return Op32i_t (_tne (opa.svhi (), opb.svhi ()) ||
                    _tne (opa.svlo (), opb.svlo ()));
}
inline Op32i_t neo_h_4 (Opq16i_t opa, Opq16i_t opb)
{
    return Op32i_t (_tne (opa.svvh (), opb.svvh ()) ||
                    _tne (opa.svhi (), opb.svhi ()) ||
                    _tne (opa.svlo (), opb.svlo ()) ||
                    _tne (opa.svvl (), opb.svvl ()));
}
inline Op32i_t neo_w_2 (Opd32i_t opa, Opd32i_t opb)
{
    return Op32i_t (_tne (opa.svhi (), opb.svhi ()) ||
                    _tne (opa.svlo (), opb.svlo ()));
}

inline Op32i_t nea_b_4 (Opq8i_t opa, Opq8i_t opb)
{
    return Op32i_t (_tne (opa.svvh (), opb.svvh ()) &&
                    _tne (opa.svhi (), opb.svhi ()) &&
                    _tne (opa.svlo (), opb.svlo ()) &&
                    _tne (opa.svvl (), opb.svvl ()));
}
inline Op32i_t nea_h_2 (Opd16i_t opa, Opd16i_t opb)
{
    return Op32i_t (_tne (opa.svhi (), opb.svhi ()) &&
                    _tne (opa.svlo (), opb.svlo ()));
}
inline Op32i_t nea_h_4 (Opq16i_t opa, Opq16i_t opb)
{
    return Op32i_t (_tne (opa.svvh (), opb.svvh ()) &&
                    _tne (opa.svhi (), opb.svhi ()) &&
                    _tne (opa.svlo (), opb.svlo ()) &&
                    _tne (opa.svvl (), opb.svvl ()));
}
inline Op32i_t nea_w_2 (Opd32i_t opa, Opd32i_t opb)
{
    return Op32i_t (_tne (opa.svhi (), opb.svhi ()) &&
                    _tne (opa.svlo (), opb.svlo ()));
}

// Shift instructions.
inline Op32i_t lsl (Op32i_t opa, Op32i_t opb)
{
    return Op32i_t (_lsl (opa.uval (), opb.uval ()));
}
inline Opq8i_t lsl_b_4 (Opq8i_t opa, Op32i_t opb)
{
    return Opq8i_t (_lsl (opa.uvvh (), opb.uval ()),
                    _lsl (opa.uvhi (), opb.uval ()),
                    _lsl (opa.uvlo (), opb.uval ()),
                    _lsl (opa.uvvl (), opb.uval ()));
}
inline Opd16i_t lsl_h_2 (Opd16i_t opa, Op32i_t opb)
{
    return Opd16i_t (_lsl (opa.uvhi (), opb.uval ()),
                     _lsl (opa.uvlo (), opb.uval ()));
}
inline Opq16i_t lsl_h_4 (Opq16i_t opa, Op32i_t opb)
{
    return Opq16i_t (_lsl (opa.uvvh (), opb.uval ()),
                     _lsl (opa.uvhi (), opb.uval ()),
                     _lsl (opa.uvlo (), opb.uval ()),
                     _lsl (opa.uvvl (), opb.uval ()));
}
inline Opd32i_t lsl_w_2 (Opd32i_t opa, Op32i_t opb)
{
    return Opd32i_t (_lsl (opa.uvhi (), opb.uval ()),
                     _lsl (opa.uvlo (), opb.uval ()));
}

inline Op32i_t lsr (Op32i_t opa, Op32i_t opb)
{
    return Op32i_t (_lsr (opa.uval (), opb.uval ()));
}
inline Opq8i_t lsr_b_4 (Opq8i_t opa, Op32i_t opb)
{
    return Opq8i_t (_lsr (opa.uvvh (), opb.uval ()),
                    _lsr (opa.uvhi (), opb.uval ()),
                    _lsr (opa.uvlo (), opb.uval ()),
                    _lsr (opa.uvvl (), opb.uval ()));
}
inline Opd16i_t lsr_h_2 (Opd16i_t opa, Op32i_t opb)
{
    return Opd16i_t (_lsr (opa.uvhi (), opb.uval ()),
                     _lsr (opa.uvlo (), opb.uval ()));
}
inline Opq16i_t lsr_h_4 (Opq16i_t opa, Op32i_t opb)
{
    return Opq16i_t (_lsr (opa.uvvh (), opb.uval ()),
                     _lsr (opa.uvhi (), opb.uval ()),
                     _lsr (opa.uvlo (), opb.uval ()),
                     _lsr (opa.uvvl (), opb.uval ()));
}
inline Opd32i_t lsr_w_2 (Opd32i_t opa, Op32i_t opb)
{
    return Opd32i_t (_lsr (opa.uvhi (), opb.uval ()),
                     _lsr (opa.uvlo (), opb.uval ()));
}

inline Op32i_t asr (Op32i_t opa, Op32i_t opb)
{
    return Op32i_t (_asr (opa.sval (), opb.uval ()));
}
inline Opq8i_t asr_b_4 (Opq8i_t opa, Op32i_t opb)
{
    return Opq8i_t (_asr (opa.uvvh (), opb.uval ()),
                    _asr (opa.uvhi (), opb.uval ()),
                    _asr (opa.uvlo (), opb.uval ()),
                    _asr (opa.uvvl (), opb.uval ()));
}
inline Opd16i_t asr_h_2 (Opd16i_t opa, Op32i_t opb)
{
    return Opd16i_t (_asr (opa.uvhi (), opb.uval ()),
                     _asr (opa.uvlo (), opb.uval ()));
}
inline Opq16i_t asr_h_4 (Opq16i_t opa, Op32i_t opb)
{
    return Opq16i_t (_asr (opa.uvvh (), opb.uval ()),
                     _asr (opa.uvhi (), opb.uval ()),
                     _asr (opa.uvlo (), opb.uval ()),
                     _asr (opa.uvvl (), opb.uval ()));
}
inline Opd32i_t asr_w_2 (Opd32i_t opa, Op32i_t opb)
{
    return Opd32i_t (_asr (opa.uvhi (), opb.uval ()),
                     _asr (opa.uvlo (), opb.uval ()));
}

inline Op32i_t rsr (Op32i_t opa, Op32i_t opb)
{
    return Op32i_t (_rsr (opa.uval (), opb.uval ()));
}
inline Opq8i_t rsr_b_4 (Opq8i_t opa, Op32i_t opb)
{
    return Opq8i_t (_rsr (opa.uvvh (), opb.uval ()),
                    _rsr (opa.uvhi (), opb.uval ()),
                    _rsr (opa.uvlo (), opb.uval ()),
                    _rsr (opa.uvvl (), opb.uval ()));
}
inline Opd16i_t rsr_h_2 (Opd16i_t opa, Op32i_t opb)
{
    return Opd16i_t (_rsr (opa.uvhi (), opb.uval ()),
                     _rsr (opa.uvlo (), opb.uval ()));
}
inline Opq16i_t rsr_h_4 (Opq16i_t opa, Op32i_t opb)
{
    return Opq16i_t (_rsr (opa.uvvh (), opb.uval ()),
                     _rsr (opa.uvhi (), opb.uval ()),
                     _rsr (opa.uvlo (), opb.uval ()),
                     _rsr (opa.uvvl (), opb.uval ()));
}
inline Opd32i_t rsr_w_2 (Opd32i_t opa, Op32i_t opb)
{
    return Opd32i_t (_rsr (opa.uvhi (), opb.uval ()),
                     _rsr (opa.uvlo (), opb.uval ()));
}

// Move instructions.
inline Op32i_t mkl (Op32i_t op)
{
    return Op32i_t (_mkl (op.uval ()));
}

inline Op32i_t mkh (Op32i_t op)
{
    return Op32i_t (_mkh (op.uval ()));
}

// Bit operation instructions.
inline Opd32i_t zip_b_4 (Op32i_t opa, Op32i_t opb)
{
    return Opd32i_t (_zip_4_hi (opa.uval (), opb.uval ()),
                     _zip_4_lo (opa.uval (), opb.uval ()));
}

inline Opd32i_t zip_h_2 (Op32i_t opa, Op32i_t opb)
{
    return Opd32i_t (_zip_2_hi (opa.uval (), opb.uval ()),
                     _zip_2_lo (opa.uval (), opb.uval ()));
}

inline Opd32i_t uzp_b_4 (Op32i_t opa, Op32i_t opb)
{
    return Opd32i_t (_uzp_4_hi (opa.uval (), opb.uval ()),
                     _uzp_4_lo (opa.uval (), opb.uval ()));
}

inline Opd32i_t uzp_h_2 (Op32i_t opa, Op32i_t opb)
{
    return Opd32i_t (_uzp_2_hi (opa.uval (), opb.uval ()),
                     _uzp_2_lo (opa.uval (), opb.uval ()));
}

inline Op32i_t uct_b_4 (Op32i_t opa, Op32i_t opb)
{
    return Op32i_t (_uct_4 (opa.uval (), opb.uval ()));
}

inline Op32i_t uct_h_2 (Op32i_t opa, Op32i_t opb)
{
    return Op32i_t (_uct_2 (opa.uval (), opb.uval ()));
}

inline Op32i_t mct_b_4 (Op32i_t opa, Op32i_t opb)
{
    return Op32i_t (_mct_4 (opa.uval (), opb.uval ()));
}

inline Op32i_t mct_h_2 (Op32i_t opa, Op32i_t opb)
{
    return Op32i_t (_mct_2 (opa.uval (), opb.uval ()));
}

inline Op32i_t swp_b_4 (Op32i_t op)
{
    return Op32i_t (_swp_4 (op.uval ()));
}

inline Op32i_t swp_h_2 (Op32i_t op)
{
    return Op32i_t (_swp_2 (op.uval ()));
}

inline Op32i_t rvs_b_4 (Op32i_t op)
{
    return Op32i_t (_rvs_4 (op.uval ()));
}

// Multiply instructions.
inline Op32i_t mul (Op32i_t opa, Op32i_t opb)
{
    return Op32i_t (_mul (opa.sval (), opb.sval ()));
}
inline Opq8i_t mul_b_4 (Opq8i_t opa, Opq8i_t opb)
{
    return Opq8i_t (_mul (opa.svvh (), opb.svvh ()),
                    _mul (opa.svhi (), opb.svhi ()),
                    _mul (opa.svlo (), opb.svlo ()),
                    _mul (opa.svvl (), opb.svvl ()));
}
inline Opd16i_t mul_h_2 (Opd16i_t opa, Opd16i_t opb)
{
    return Opd16i_t (_mul (opa.svhi (), opb.svhi ()),
                     _mul (opa.svlo (), opb.svlo ()));
}
inline Opq16i_t mul_h_4 (Opq16i_t opa, Opq16i_t opb)
{
    return Opq16i_t (_mul (opa.svvh (), opb.svvh ()),
                     _mul (opa.svhi (), opb.svhi ()),
                     _mul (opa.svlo (), opb.svlo ()),
                     _mul (opa.svvl (), opb.svvl ()));
}
inline Opd32i_t mul_w_2 (Opd32i_t opa, Opd32i_t opb)
{
    return Opd32i_t (_mul (opa.svhi (), opb.svhi ()),
                     _mul (opa.svlo (), opb.svlo ()));
}
inline Op32i_t mul_u (Op32i_t opa, Op32i_t opb)
{
    return Op32i_t (_mul (opa.uval (), opb.uval ()));
}
inline Opq8i_t mul_u_b_4 (Opq8i_t opa, Opq8i_t opb)
{
    return Opq8i_t (_mul (opa.uvvh (), opb.uvvh ()),
                    _mul (opa.uvhi (), opb.uvhi ()),
                    _mul (opa.uvlo (), opb.uvlo ()),
                    _mul (opa.uvvl (), opb.uvvl ()));
}
inline Opd16i_t mul_u_h_2 (Opd16i_t opa, Opd16i_t opb)
{
    return Opd16i_t (_mul (opa.uvhi (), opb.uvhi ()),
                     _mul (opa.uvlo (), opb.uvlo ()));
}
inline Opq16i_t mul_u_h_4 (Opq16i_t opa, Opq16i_t opb)
{
    return Opq16i_t (_mul (opa.uvvh (), opb.uvvh ()),
                     _mul (opa.uvhi (), opb.uvhi ()),
                     _mul (opa.uvlo (), opb.uvlo ()),
                     _mul (opa.uvvl (), opb.uvvl ()));
}
inline Opd32i_t mul_u_w_2 (Opd32i_t opa, Opd32i_t opb)
{
    return Opd32i_t (_mul (opa.uvhi (), opb.uvhi ()),
                     _mul (opa.uvlo (), opb.uvlo ()));
}

inline Op32i_t nmu (Op32i_t opa, Op32i_t opb)
{
    return Op32i_t (_nmu (opa.sval (), opb.sval ()));
}
inline Opq8i_t nmu_b_4 (Opq8i_t opa, Opq8i_t opb)
{
    return Opq8i_t (_nmu (opa.svvh (), opb.svvh ()),
                    _nmu (opa.svhi (), opb.svhi ()),
                    _nmu (opa.svlo (), opb.svlo ()),
                    _nmu (opa.svvl (), opb.svvl ()));
}
inline Opd16i_t nmu_h_2 (Opd16i_t opa, Opd16i_t opb)
{
    return Opd16i_t (_nmu (opa.svhi (), opb.svhi ()),
                     _nmu (opa.svlo (), opb.svlo ()));
}
inline Opq16i_t nmu_h_4 (Opq16i_t opa, Opq16i_t opb)
{
    return Opq16i_t (_nmu (opa.svvh (), opb.svvh ()),
                     _nmu (opa.svhi (), opb.svhi ()),
                     _nmu (opa.svlo (), opb.svlo ()),
                     _nmu (opa.svvl (), opb.svvl ()));
}
inline Opd32i_t nmu_w_2 (Opd32i_t opa, Opd32i_t opb)
{
    return Opd32i_t (_nmu (opa.svhi (), opb.svhi ()),
                     _nmu (opa.svlo (), opb.svlo ()));
}
inline Op32i_t nmu_u (Op32i_t opa, Op32i_t opb)
{
    return Op32i_t (_nmu (opa.uval (), opb.uval ()));
}
inline Opq8i_t nmu_u_b_4 (Opq8i_t opa, Opq8i_t opb)
{
    return Opq8i_t (_nmu (opa.uvvh (), opb.uvvh ()),
                    _nmu (opa.uvhi (), opb.uvhi ()),
                    _nmu (opa.uvlo (), opb.uvlo ()),
                    _nmu (opa.uvvl (), opb.uvvl ()));
}
inline Opd16i_t nmu_u_h_2 (Opd16i_t opa, Opd16i_t opb)
{
    return Opd16i_t (_nmu (opa.uvhi (), opb.uvhi ()),
                     _nmu (opa.uvlo (), opb.uvlo ()));
}
inline Opq16i_t nmu_u_h_4 (Opq16i_t opa, Opq16i_t opb)
{
    return Opq16i_t (_nmu (opa.uvvh (), opb.uvvh ()),
                     _nmu (opa.uvhi (), opb.uvhi ()),
                     _nmu (opa.uvlo (), opb.uvlo ()),
                     _nmu (opa.uvvl (), opb.uvvl ()));
}
inline Opd32i_t nmu_u_w_2 (Opd32i_t opa, Opd32i_t opb)
{
    return Opd32i_t (_nmu (opa.uvhi (), opb.uvhi ()),
                     _nmu (opa.uvlo (), opb.uvlo ()));
}

inline Op32i_t mac (Op32i_t opa, Op32i_t opb, Op32i_t opc)
{
    return Op32i_t (_mac (opa.sval (), opb.sval (), opc.sval ()));
}
inline Opq8i_t mac_b_4 (Opq8i_t opa, Opq8i_t opb, Opq8i_t opc)
{
    return Opq8i_t (_mac (opa.svvh (), opb.svvh (), opc.svvh ()),
                    _mac (opa.svhi (), opb.svhi (), opc.svhi ()),
                    _mac (opa.svlo (), opb.svlo (), opc.svlo ()),
                    _mac (opa.svvl (), opb.svvl (), opc.svvl ()));
}
inline Opd16i_t mac_h_2 (Opd16i_t opa, Opd16i_t opb, Opd16i_t opc)
{
    return Opd16i_t (_mac (opa.svhi (), opb.svhi (), opc.svhi ()),
                     _mac (opa.svlo (), opb.svlo (), opc.svlo ()));
}
inline Opq16i_t mac_h_4 (Opq16i_t opa, Opq16i_t opb, Opq16i_t opc)
{
    return Opq16i_t (_mac (opa.svvh (), opb.svvh (), opc.svvh ()),
                     _mac (opa.svhi (), opb.svhi (), opc.svhi ()),
                     _mac (opa.svlo (), opb.svlo (), opc.svlo ()),
                     _mac (opa.svvl (), opb.svvl (), opc.svvl ()));
}
inline Opd32i_t mac_w_2 (Opd32i_t opa, Opd32i_t opb, Opd32i_t opc)
{
    return Opd32i_t (_mac (opa.svhi (), opb.svhi (), opc.svhi ()),
                     _mac (opa.svlo (), opb.svlo (), opc.svlo ()));
}
inline Op32i_t mac_u (Op32i_t opa, Op32i_t opb, Op32i_t opc)
{
    return Op32i_t (_mac (opa.uval (), opb.uval (), opc.uval ()));
}
inline Opq8i_t mac_u_b_4 (Opq8i_t opa, Opq8i_t opb, Opq8i_t opc)
{
    return Opq8i_t (_mac (opa.uvvh (), opb.uvvh (), opc.uvvh ()),
                    _mac (opa.uvhi (), opb.uvhi (), opc.uvhi ()),
                    _mac (opa.uvlo (), opb.uvlo (), opc.uvlo ()),
                    _mac (opa.uvvl (), opb.uvvl (), opc.uvvl ()));
}
inline Opd16i_t mac_u_h_2 (Opd16i_t opa, Opd16i_t opb, Opd16i_t opc)
{
    return Opd16i_t (_mac (opa.uvhi (), opb.uvhi (), opc.uvhi ()),
                     _mac (opa.uvlo (), opb.uvlo (), opc.uvlo ()));
}
inline Opq16i_t mac_u_h_4 (Opq16i_t opa, Opq16i_t opb, Opq16i_t opc)
{
    return Opq16i_t (_mac (opa.uvvh (), opb.uvvh (), opc.uvvh ()),
                     _mac (opa.uvhi (), opb.uvhi (), opc.uvhi ()),
                     _mac (opa.uvlo (), opb.uvlo (), opc.uvlo ()),
                     _mac (opa.uvvl (), opb.uvvl (), opc.uvvl ()));
}
inline Opd32i_t mac_u_w_2 (Opd32i_t opa, Opd32i_t opb, Opd32i_t opc)
{
    return Opd32i_t (_mac (opa.uvhi (), opb.uvhi (), opc.uvhi ()),
                     _mac (opa.uvlo (), opb.uvlo (), opc.uvlo ()));
}

inline Op32i_t nma (Op32i_t opa, Op32i_t opb, Op32i_t opc)
{
    return Op32i_t (_nma (opa.sval (), opb.sval (), opc.sval ()));
}
inline Opq8i_t nma_b_4 (Opq8i_t opa, Opq8i_t opb, Opq8i_t opc)
{
    return Opq8i_t (_nma (opa.svvh (), opb.svvh (), opc.svvh ()),
                    _nma (opa.svhi (), opb.svhi (), opc.svhi ()),
                    _nma (opa.svlo (), opb.svlo (), opc.svlo ()),
                    _nma (opa.svvl (), opb.svvl (), opc.svvl ()));
}
inline Opd16i_t nma_h_2 (Opd16i_t opa, Opd16i_t opb, Opd16i_t opc)
{
    return Opd16i_t (_nma (opa.svhi (), opb.svhi (), opc.svhi ()),
                     _nma (opa.svlo (), opb.svlo (), opc.svlo ()));
}
inline Opq16i_t nma_h_4 (Opq16i_t opa, Opq16i_t opb, Opq16i_t opc)
{
    return Opq16i_t (_nma (opa.svvh (), opb.svvh (), opc.svvh ()),
                     _nma (opa.svhi (), opb.svhi (), opc.svhi ()),
                     _nma (opa.svlo (), opb.svlo (), opc.svlo ()),
                     _nma (opa.svvl (), opb.svvl (), opc.svvl ()));
}
inline Opd32i_t nma_w_2 (Opd32i_t opa, Opd32i_t opb, Opd32i_t opc)
{
    return Opd32i_t (_nma (opa.svhi (), opb.svhi (), opc.svhi ()),
                     _nma (opa.svlo (), opb.svlo (), opc.svlo ()));
}
inline Op32i_t nma_u (Op32i_t opa, Op32i_t opb, Op32i_t opc)
{
    return Op32i_t (_nma (opa.uval (), opb.uval (), opc.uval ()));
}
inline Opq8i_t nma_u_b_4 (Opq8i_t opa, Opq8i_t opb, Opq8i_t opc)
{
    return Opq8i_t (_nma (opa.uvvh (), opb.uvvh (), opc.uvvh ()),
                    _nma (opa.uvhi (), opb.uvhi (), opc.uvhi ()),
                    _nma (opa.uvlo (), opb.uvlo (), opc.uvlo ()),
                    _nma (opa.uvvl (), opb.uvvl (), opc.uvvl ()));
}
inline Opd16i_t nma_u_h_2 (Opd16i_t opa, Opd16i_t opb, Opd16i_t opc)
{
    return Opd16i_t (_nma (opa.uvhi (), opb.uvhi (), opc.uvhi ()),
                     _nma (opa.uvlo (), opb.uvlo (), opc.uvlo ()));
}
inline Opq16i_t nma_u_h_4 (Opq16i_t opa, Opq16i_t opb, Opq16i_t opc)
{
    return Opq16i_t (_nma (opa.uvvh (), opb.uvvh (), opc.uvvh ()),
                     _nma (opa.uvhi (), opb.uvhi (), opc.uvhi ()),
                     _nma (opa.uvlo (), opb.uvlo (), opc.uvlo ()),
                     _nma (opa.uvvl (), opb.uvvl (), opc.uvvl ()));
}
inline Opd32i_t nma_u_w_2 (Opd32i_t opa, Opd32i_t opb, Opd32i_t opc)
{
    return Opd32i_t (_nma (opa.uvhi (), opb.uvhi (), opc.uvhi ()),
                     _nma (opa.uvlo (), opb.uvlo (), opc.uvlo ()));
}

inline Op32i_t msu (Op32i_t opa, Op32i_t opb, Op32i_t opc)
{
    return Op32i_t (_msu (opa.sval (), opb.sval (), opc.sval ()));
}
inline Opq8i_t msu_b_4 (Opq8i_t opa, Opq8i_t opb, Opq8i_t opc)
{
    return Opq8i_t (_msu (opa.svvh (), opb.svvh (), opc.svvh ()),
                    _msu (opa.svhi (), opb.svhi (), opc.svhi ()),
                    _msu (opa.svlo (), opb.svlo (), opc.svlo ()),
                    _msu (opa.svvl (), opb.svvl (), opc.svvl ()));
}
inline Opd16i_t msu_h_2 (Opd16i_t opa, Opd16i_t opb, Opd16i_t opc)
{
    return Opd16i_t (_msu (opa.svhi (), opb.svhi (), opc.svhi ()),
                     _msu (opa.svlo (), opb.svlo (), opc.svlo ()));
}
inline Opq16i_t msu_h_4 (Opq16i_t opa, Opq16i_t opb, Opq16i_t opc)
{
    return Opq16i_t (_msu (opa.svvh (), opb.svvh (), opc.svvh ()),
                     _msu (opa.svhi (), opb.svhi (), opc.svhi ()),
                     _msu (opa.svlo (), opb.svlo (), opc.svlo ()),
                     _msu (opa.svvl (), opb.svvl (), opc.svvl ()));
}
inline Opd32i_t msu_w_2 (Opd32i_t opa, Opd32i_t opb, Opd32i_t opc)
{
    return Opd32i_t (_msu (opa.svhi (), opb.svhi (), opc.svhi ()),
                     _msu (opa.svlo (), opb.svlo (), opc.svlo ()));
}
inline Op32i_t msu_u (Op32i_t opa, Op32i_t opb, Op32i_t opc)
{
    return Op32i_t (_msu (opa.uval (), opb.uval (), opc.uval ()));
}
inline Opq8i_t msu_u_b_4 (Opq8i_t opa, Opq8i_t opb, Opq8i_t opc)
{
    return Opq8i_t (_msu (opa.uvvh (), opb.uvvh (), opc.uvvh ()),
                    _msu (opa.uvhi (), opb.uvhi (), opc.uvhi ()),
                    _msu (opa.uvlo (), opb.uvlo (), opc.uvlo ()),
                    _msu (opa.uvvl (), opb.uvvl (), opc.uvvl ()));
}
inline Opd16i_t msu_u_h_2 (Opd16i_t opa, Opd16i_t opb, Opd16i_t opc)
{
    return Opd16i_t (_msu (opa.uvhi (), opb.uvhi (), opc.uvhi ()),
                     _msu (opa.uvlo (), opb.uvlo (), opc.uvlo ()));
}
inline Opq16i_t msu_u_h_4 (Opq16i_t opa, Opq16i_t opb, Opq16i_t opc)
{
    return Opq16i_t (_msu (opa.uvvh (), opb.uvvh (), opc.uvvh ()),
                     _msu (opa.uvhi (), opb.uvhi (), opc.uvhi ()),
                     _msu (opa.uvlo (), opb.uvlo (), opc.uvlo ()),
                     _msu (opa.uvvl (), opb.uvvl (), opc.uvvl ()));
}
inline Opd32i_t msu_u_w_2 (Opd32i_t opa, Opd32i_t opb, Opd32i_t opc)
{
    return Opd32i_t (_msu (opa.uvhi (), opb.uvhi (), opc.uvhi ()),
                     _msu (opa.uvlo (), opb.uvlo (), opc.uvlo ()));
}

inline Op32i_t nms (Op32i_t opa, Op32i_t opb, Op32i_t opc)
{
    return Op32i_t (_nms (opa.sval (), opb.sval (), opc.sval ()));
}
inline Opq8i_t nms_b_4 (Opq8i_t opa, Opq8i_t opb, Opq8i_t opc)
{
    return Opq8i_t (_nms (opa.svvh (), opb.svvh (), opc.svvh ()),
                    _nms (opa.svhi (), opb.svhi (), opc.svhi ()),
                    _nms (opa.svlo (), opb.svlo (), opc.svlo ()),
                    _nms (opa.svvl (), opb.svvl (), opc.svvl ()));
}
inline Opd16i_t nms_h_2 (Opd16i_t opa, Opd16i_t opb, Opd16i_t opc)
{
    return Opd16i_t (_nms (opa.svhi (), opb.svhi (), opc.svhi ()),
                     _nms (opa.svlo (), opb.svlo (), opc.svlo ()));
}
inline Opq16i_t nms_h_4 (Opq16i_t opa, Opq16i_t opb, Opq16i_t opc)
{
    return Opq16i_t (_nms (opa.svvh (), opb.svvh (), opc.svvh ()),
                     _nms (opa.svhi (), opb.svhi (), opc.svhi ()),
                     _nms (opa.svlo (), opb.svlo (), opc.svlo ()),
                     _nms (opa.svvl (), opb.svvl (), opc.svvl ()));
}
inline Opd32i_t nms_w_2 (Opd32i_t opa, Opd32i_t opb, Opd32i_t opc)
{
    return Opd32i_t (_nms (opa.svhi (), opb.svhi (), opc.svhi ()),
                     _nms (opa.svlo (), opb.svlo (), opc.svlo ()));
}
inline Op32i_t nms_u (Op32i_t opa, Op32i_t opb, Op32i_t opc)
{
    return Op32i_t (_nms (opa.uval (), opb.uval (), opc.uval ()));
}
inline Opq8i_t nms_u_b_4 (Opq8i_t opa, Opq8i_t opb, Opq8i_t opc)
{
    return Opq8i_t (_nms (opa.uvvh (), opb.uvvh (), opc.uvvh ()),
                    _nms (opa.uvhi (), opb.uvhi (), opc.uvhi ()),
                    _nms (opa.uvlo (), opb.uvlo (), opc.uvlo ()),
                    _nms (opa.uvvl (), opb.uvvl (), opc.uvvl ()));
}
inline Opd16i_t nms_u_h_2 (Opd16i_t opa, Opd16i_t opb, Opd16i_t opc)
{
    return Opd16i_t (_nms (opa.uvhi (), opb.uvhi (), opc.uvhi ()),
                     _nms (opa.uvlo (), opb.uvlo (), opc.uvlo ()));
}
inline Opq16i_t nms_u_h_4 (Opq16i_t opa, Opq16i_t opb, Opq16i_t opc)
{
    return Opq16i_t (_nms (opa.uvvh (), opb.uvvh (), opc.uvvh ()),
                     _nms (opa.uvhi (), opb.uvhi (), opc.uvhi ()),
                     _nms (opa.uvlo (), opb.uvlo (), opc.uvlo ()),
                     _nms (opa.uvvl (), opb.uvvl (), opc.uvvl ()));
}
inline Opd32i_t nms_u_w_2 (Opd32i_t opa, Opd32i_t opb, Opd32i_t opc)
{
    return Opd32i_t (_nms (opa.uvhi (), opb.uvhi (), opc.uvhi ()),
                     _nms (opa.uvlo (), opb.uvlo (), opc.uvlo ()));
}

inline Op32i_t div (Op32i_t opa, Op32i_t opb)
{
    return Op32i_t (_div (opa.sval (), opb.sval ()));
}
inline Opq8i_t div_b_4 (Opq8i_t opa, Opq8i_t opb)
{
    return Opq8i_t (_div (opa.svvh (), opb.svvh ()),
                    _div (opa.svhi (), opb.svhi ()),
                    _div (opa.svlo (), opb.svlo ()),
                    _div (opa.svvl (), opb.svvl ()));
}
inline Opd16i_t div_h_2 (Opd16i_t opa, Opd16i_t opb)
{
    return Opd16i_t (_div (opa.svhi (), opb.svhi ()),
                     _div (opa.svlo (), opb.svlo ()));
}
inline Opq16i_t div_h_4 (Opq16i_t opa, Opq16i_t opb)
{
    return Opq16i_t (_div (opa.svvh (), opb.svvh ()),
                     _div (opa.svhi (), opb.svhi ()),
                     _div (opa.svlo (), opb.svlo ()),
                     _div (opa.svvl (), opb.svvl ()));
}
inline Opd32i_t div_w_2 (Opd32i_t opa, Opd32i_t opb)
{
    return Opd32i_t (_div (opa.svhi (), opb.svhi ()),
                     _div (opa.svlo (), opb.svlo ()));
}
inline Op32i_t div_u (Op32i_t opa, Op32i_t opb)
{
    return Op32i_t (_div (opa.uval (), opb.uval ()));
}
inline Opq8i_t div_u_b_4 (Opq8i_t opa, Opq8i_t opb)
{
    return Opq8i_t (_div (opa.uvvh (), opb.uvvh ()),
                    _div (opa.uvhi (), opb.uvhi ()),
                    _div (opa.uvlo (), opb.uvlo ()),
                    _div (opa.uvvl (), opb.uvvl ()));
}
inline Opd16i_t div_u_h_2 (Opd16i_t opa, Opd16i_t opb)
{
    return Opd16i_t (_div (opa.uvhi (), opb.uvhi ()),
                     _div (opa.uvlo (), opb.uvlo ()));
}
inline Opq16i_t div_u_h_4 (Opq16i_t opa, Opq16i_t opb)
{
    return Opq16i_t (_div (opa.uvvh (), opb.uvvh ()),
                     _div (opa.uvhi (), opb.uvhi ()),
                     _div (opa.uvlo (), opb.uvlo ()),
                     _div (opa.uvvl (), opb.uvvl ()));
}
inline Opd32i_t div_u_w_2 (Opd32i_t opa, Opd32i_t opb)
{
    return Opd32i_t (_div (opa.uvhi (), opb.uvhi ()),
                     _div (opa.uvlo (), opb.uvlo ()));
}

inline Op32i_t rem (Op32i_t opa, Op32i_t opb)
{
    return Op32i_t (_rem (opa.sval (), opb.sval ()));
}
inline Opq8i_t rem_b_4 (Opq8i_t opa, Opq8i_t opb)
{
    return Opq8i_t (_rem (opa.svvh (), opb.svvh ()),
                    _rem (opa.svhi (), opb.svhi ()),
                    _rem (opa.svlo (), opb.svlo ()),
                    _rem (opa.svvl (), opb.svvl ()));
}
inline Opd16i_t rem_h_2 (Opd16i_t opa, Opd16i_t opb)
{
    return Opd16i_t (_rem (opa.svhi (), opb.svhi ()),
                     _rem (opa.svlo (), opb.svlo ()));
}
inline Opq16i_t rem_h_4 (Opq16i_t opa, Opq16i_t opb)
{
    return Opq16i_t (_rem (opa.svvh (), opb.svvh ()),
                     _rem (opa.svhi (), opb.svhi ()),
                     _rem (opa.svlo (), opb.svlo ()),
                     _rem (opa.svvl (), opb.svvl ()));
}
inline Opd32i_t rem_w_2 (Opd32i_t opa, Opd32i_t opb)
{
    return Opd32i_t (_rem (opa.svhi (), opb.svhi ()),
                     _rem (opa.svlo (), opb.svlo ()));
}
inline Op32i_t rem_u (Op32i_t opa, Op32i_t opb)
{
    return Op32i_t (_rem (opa.uval (), opb.uval ()));
}
inline Opq8i_t rem_u_b_4 (Opq8i_t opa, Opq8i_t opb)
{
    return Opq8i_t (_rem (opa.uvvh (), opb.uvvh ()),
                    _rem (opa.uvhi (), opb.uvhi ()),
                    _rem (opa.uvlo (), opb.uvlo ()),
                    _rem (opa.uvvl (), opb.uvvl ()));
}
inline Opd16i_t rem_u_h_2 (Opd16i_t opa, Opd16i_t opb)
{
    return Opd16i_t (_rem (opa.uvhi (), opb.uvhi ()),
                     _rem (opa.uvlo (), opb.uvlo ()));
}
inline Opq16i_t rem_u_h_4 (Opq16i_t opa, Opq16i_t opb)
{
    return Opq16i_t (_rem (opa.uvvh (), opb.uvvh ()),
                     _rem (opa.uvhi (), opb.uvhi ()),
                     _rem (opa.uvlo (), opb.uvlo ()),
                     _rem (opa.uvvl (), opb.uvvl ()));
}
inline Opd32i_t rem_u_w_2 (Opd32i_t opa, Opd32i_t opb)
{
    return Opd32i_t (_rem (opa.uvhi (), opb.uvhi ()),
                     _rem (opa.uvlo (), opb.uvlo ()));
}

// Memory access instructions.
inline Addr getEA (const Op32i_t& base, const Op32i_t& ofst)
{
    return (Addr) (base.sval () + ofst.sval ());
}

inline Op32i_t preSetBA (const Op32i_t& base, const Op32i_t& ofst, int mode)
{
    switch (mode) {
        case 0x0: return Op32i_t (base.sval () + ofst.sval ());
        case 0x1: return Op32i_t (base.sval () - ofst.sval ());
        case 0x2: // Fall through.
        case 0x3: return base;
        default : assert (0);
    }
}

inline Op32i_t postSetBA (const Op32i_t& base, const Op32i_t& ofst, int mode)
{
    switch (mode) {
        case 0x0: return base;
        case 0x1: return base;
        case 0x2: return Op32i_t (base.sval () + ofst.sval ());
        case 0x3: return Op32i_t (base.sval () - ofst.sval ());
        default : assert (0);
    }
}

inline Op32i_t ldb_u_ (uint8_t *data)
{
    uint8_t *reinterpret_data = reinterpret_cast<uint8_t *> (data);
    uint8_t mem = *reinterpret_data;
    mem = gtobe (mem);
    uint32_t val = static_cast<uint32_t> (mem);
    return Op32i_t (val);
}

inline Op32i_t ldb_ (uint8_t *data)
{
    uint8_t *reinterpret_data = reinterpret_cast<uint8_t *> (data);
    uint8_t mem = *reinterpret_data;
    mem = gtobe (mem);
    uint32_t val = sext<8> (static_cast<uint32_t> (mem));
    return Op32i_t (val);
}

inline Op32i_t ldh_u_ (uint8_t *data)
{
    uint16_t *reinterpret_data = reinterpret_cast<uint16_t *> (data);
    uint16_t mem = *reinterpret_data;
    mem = gtobe (mem);
    uint32_t val = static_cast<uint32_t> (mem);
    return Op32i_t (val);
}

inline Op32i_t ldh_ (uint8_t *data)
{
    uint16_t *reinterpret_data = reinterpret_cast<uint16_t *> (data);
    uint16_t mem = *reinterpret_data;
    mem = gtobe (mem);
    uint32_t val = sext<16> (static_cast<uint32_t> (mem));
    return Op32i_t (val);
}

inline Op32i_t ldw_ (uint8_t *data)
{
    uint32_t *reinterpret_data = reinterpret_cast<uint32_t *> (data);
    uint32_t mem = *reinterpret_data;
    mem = gtobe (mem);
    uint32_t val = static_cast<uint32_t> (mem);
    return Op32i_t (val);
}

inline Opd32i_t ldd_ (uint8_t *data)
{
    uint64_t *reinterpret_data = reinterpret_cast<uint64_t *> (data);
    uint64_t mem = *reinterpret_data;
    mem = gtobe (mem);
    uint32_t vlo = static_cast<uint32_t> (mem);
    uint32_t vhi = static_cast<uint32_t> (mem >> 32);
    return Opd32i_t (vlo, vhi);
}

inline void stb_ (uint8_t *data, const Op32i_t& op)
{
    uint8_t mem = op.uval ();
    mem = gtobe (mem);
    memcpy (data, &mem, 1);
}

inline void sth_ (uint8_t *data, const Op32i_t& op)
{
    uint16_t mem = op.uval ();
    mem = gtobe (mem);
    memcpy (data, &mem, 2);
}

inline void stw_ (uint8_t *data, const Op32i_t& op)
{
    uint32_t mem = op.uval ();
    mem = gtobe (mem);
    memcpy (data, &mem, 4);
}

inline void std_ (uint8_t *data, const Opd32i_t& op)
{
    uint32_t mlo = op.uvlo ();
    uint32_t mhi = op.uvhi ();
    uint64_t mem = static_cast<uint64_t> (mlo) + ((static_cast<uint64_t> (mhi)) << 32);
    mem = gtobe (mem);
    memcpy (data, &mem, 8);
}

// Flow control instructions.
inline Addr getTA (const Op32i_t& base)
{
    return (Addr) base.sval ();
}

inline Addr getTA (Addr base)
{
    return (Addr) base;
}

inline Addr getTA (Addr base, const Op32i_t& disp)
{
    return (Addr) (base + disp.sval ());
}

inline Op32i_t getRA (Addr addr)
{
    return Op32i_t (static_cast<uint32_t> (addr));
}

} // namespace Lily2ISA

#endif // __ARCH_LILY2_ISA_EXEC_HH__
