/*
 * Copyright (C) 2014 DSP Group, Institute of Microelectronics, Tsinghua University
 * All rights reserved.
 */

#ifndef __ARCH_LILY2_ISA_UTIL_HH__
#define __ARCH_LILY2_ISA_UTIL_HH__

#include <cmath>
#include "base/bitfield.hh"
#include "base/types.hh"

namespace Lily2ISA
{
// Arithmetic instructions.
template <class T>
T _add (T opa, T opb)
{
    return opa + opb;
}

template <class T>
T _adc (T opa, T opb, T opc)
{
    return opa + opb + opc;
}

template <class T>
T _sub (T opa, T opb)
{
    return opa - opb;
}

template <class T>
T _sbc (T opa, T opb, T opc)
{
    return opa - opb + opc - 1;
}

template <class T>
T _rsb (T opa, T opb)
{
    return opb - opa;
}

template <class T>
T _rsc (T opa, T opb, T opc)
{
    return opb - opa + opc - 1;
}

template <class T>
T _abs (T op)
{
    return op > 0 ? op : -op;
}

template <class T>
T _neg (T op)
{
    return -op;
}

inline
uint32_t _sxb (uint32_t op)
{
    return sext<8> (op);
}

inline
uint32_t _sxh (uint32_t op)
{
    return sext<16> (op);
}

inline
uint32_t _zxb (uint32_t op)
{
    return bits (op, 7, 0);
}

inline
uint32_t _zxh (uint32_t op)
{
    return bits (op, 15, 0);
}

// Logic instructions.
template <class T>
T _bitAnd (T opa, T opb)
{
    return opa & opb;
}

template <class T>
T _bitNad (T opa, T opb)
{
    return ~(opa & opb);
}

template <class T>
T _bitOrr (T opa, T opb)
{
    return opa | opb;
}

template <class T>
T _bitNor (T opa, T opb)
{
    return ~(opa | opb);
}

template <class T>
T _bitXor (T opa, T opb)
{
    return opa ^ opb;
}

template <class T>
T _bitNxr (T opa, T opb)
{
    return ~(opa ^ opb);
}

template <class T>
T _bitNot (T op)
{
    return ~op;
}

template <class T>
uint32_t _tgt (T opa, T opb)
{
    return (opa > opb) ? 1 : 0;
}

template <class T>
uint32_t _tge (T opa, T opb)
{
    return (opa >= opb) ? 1 : 0;
}

template <class T>
uint32_t _tlt (T opa, T opb)
{
    return (opa < opb) ? 1 : 0;
}

template <class T>
uint32_t _tle (T opa, T opb)
{
    return (opa <= opb) ? 1 : 0;
}

template <class T>
uint32_t _teq (T opa, T opb)
{
    return (opa == opb) ? 1 : 0;
}

template <class T>
uint32_t _tne (T opa, T opb)
{
    return (opa != opb) ? 1 : 0;
}

template <class T>
T _lsl (T opa, uint32_t shiftAmt)
{
    return opa << shiftAmt;
}

template <class T>
T _lsr (T opa, uint32_t shiftAmt)
{
    return opa >> shiftAmt;
}

template <class T>
T _asr (T opa, uint32_t shiftAmt)
{
    return opa >> shiftAmt;
}

template <class T>
T _rsr (T opa, uint32_t shiftAmt)
{
    return (opa >> shiftAmt) | (opa << (8 * sizeof (T) - shiftAmt));
}

inline
uint32_t _mkl (uint32_t op)
{
    return op;
}

inline
uint32_t _mkh (uint32_t op)
{
    return op << 16;
}

inline
uint32_t _zip_4_hi (uint32_t opa, uint32_t opb)
{
    //uint32_t opa_vvl = bits (opa,  7,  0);
    //uint32_t opa_vlo = bits (opa, 15,  8);
    uint32_t opa_vhi = bits (opa, 23, 16);
    uint32_t opa_vvh = bits (opa, 31, 24);
    //uint32_t opb_vvl = bits (opb,  7,  0);
    //uint32_t opb_vlo = bits (opb, 15,  8);
    uint32_t opb_vhi = bits (opb, 23, 16);
    uint32_t opb_vvh = bits (opb, 31, 24);

    return (opa_vvh << 24) | (opb_vvh << 16) | (opa_vhi << 8) | (opb_vhi);
}

inline
uint32_t _zip_4_lo (uint32_t opa, uint32_t opb)
{
    uint32_t opa_vvl = bits (opa,  7,  0);
    uint32_t opa_vlo = bits (opa, 15,  8);
    //uint32_t opa_vhi = bits (opa, 23, 16);
    //uint32_t opa_vvh = bits (opa, 31, 24);
    uint32_t opb_vvl = bits (opb,  7,  0);
    uint32_t opb_vlo = bits (opb, 15,  8);
    //uint32_t opb_vhi = bits (opb, 23, 16);
    //uint32_t opb_vvh = bits (opb, 31, 24);

    return (opa_vlo << 24) | (opb_vlo << 16) | (opa_vvl << 8) | (opb_vvl);
}

inline
uint32_t _zip_2_hi (uint32_t opa, uint32_t opb)
{
    //uint32_t opa_vvl = bits (opa,  7,  0);
    //uint32_t opa_vlo = bits (opa, 15,  8);
    uint32_t opa_vhi = bits (opa, 23, 16);
    uint32_t opa_vvh = bits (opa, 31, 24);
    //uint32_t opb_vvl = bits (opb,  7,  0);
    //uint32_t opb_vlo = bits (opb, 15,  8);
    uint32_t opb_vhi = bits (opb, 23, 16);
    uint32_t opb_vvh = bits (opb, 31, 24);

    return (opa_vvh << 24) | (opa_vhi << 16) | (opb_vvh << 8) | (opb_vhi);
}

inline
uint32_t _zip_2_lo (uint32_t opa, uint32_t opb)
{
    uint32_t opa_vvl = bits (opa,  7,  0);
    uint32_t opa_vlo = bits (opa, 15,  8);
    //uint32_t opa_vhi = bits (opa, 23, 16);
    //uint32_t opa_vvh = bits (opa, 31, 24);
    uint32_t opb_vvl = bits (opb,  7,  0);
    uint32_t opb_vlo = bits (opb, 15,  8);
    //uint32_t opb_vhi = bits (opb, 23, 16);
    //uint32_t opb_vvh = bits (opb, 31, 24);

    return (opa_vlo << 24) | (opa_vvl << 16) | (opb_vlo << 8) | (opb_vvl);
}

inline
uint32_t _uzp_4_hi (uint32_t opa, uint32_t opb)
{
    //uint32_t opa_vvl = bits (opa,  7,  0);
    uint32_t opa_vlo = bits (opa, 15,  8);
    //uint32_t opa_vhi = bits (opa, 23, 16);
    uint32_t opa_vvh = bits (opa, 31, 24);
    //uint32_t opb_vvl = bits (opb,  7,  0);
    uint32_t opb_vlo = bits (opb, 15,  8);
    //uint32_t opb_vhi = bits (opb, 23, 16);
    uint32_t opb_vvh = bits (opb, 31, 24);

    return (opa_vvh << 24) | (opa_vlo << 16) | (opb_vvh << 8) | (opb_vlo);
}

inline
uint32_t _uzp_4_lo (uint32_t opa, uint32_t opb)
{
    uint32_t opa_vvl = bits (opa,  7,  0);
    //uint32_t opa_vlo = bits (opa, 15,  8);
    uint32_t opa_vhi = bits (opa, 23, 16);
    //uint32_t opa_vvh = bits (opa, 31, 24);
    uint32_t opb_vvl = bits (opb,  7,  0);
    //uint32_t opb_vlo = bits (opb, 15,  8);
    uint32_t opb_vhi = bits (opb, 23, 16);
    //uint32_t opb_vvh = bits (opb, 31, 24);

    return (opa_vhi << 24) | (opa_vvl << 16) | (opb_vhi << 8) | (opb_vvl);
}

inline
uint32_t _uzp_2_hi (uint32_t opa, uint32_t opb)
{
    //uint32_t opa_vvl = bits (opa,  7,  0);
    //uint32_t opa_vlo = bits (opa, 15,  8);
    uint32_t opa_vhi = bits (opa, 23, 16);
    uint32_t opa_vvh = bits (opa, 31, 24);
    //uint32_t opb_vvl = bits (opb,  7,  0);
    //uint32_t opb_vlo = bits (opb, 15,  8);
    uint32_t opb_vhi = bits (opb, 23, 16);
    uint32_t opb_vvh = bits (opb, 31, 24);

    return (opa_vvh << 24) | (opa_vhi << 16) | (opb_vvh << 8) | (opb_vhi);
}

inline
uint32_t _uzp_2_lo (uint32_t opa, uint32_t opb)
{
    uint32_t opa_vvl = bits (opa,  7,  0);
    uint32_t opa_vlo = bits (opa, 15,  8);
    //uint32_t opa_vhi = bits (opa, 23, 16);
    //uint32_t opa_vvh = bits (opa, 31, 24);
    uint32_t opb_vvl = bits (opb,  7,  0);
    uint32_t opb_vlo = bits (opb, 15,  8);
    //uint32_t opb_vhi = bits (opb, 23, 16);
    //uint32_t opb_vvh = bits (opb, 31, 24);

    return (opa_vlo << 24) | (opa_vvl << 16) | (opb_vlo << 8) | (opb_vvl);
}

inline
uint32_t _uct_4 (uint32_t op, uint32_t select)
{
    uint32_t val;

    switch (select) {
        case 0 : val = bits (op,  7,  0);
        case 1 : val = bits (op, 15,  8);
        case 2 : val = bits (op, 23, 16);
        case 3 : val = bits (op, 31, 24);
        default: assert (0);
    }

    return val;
}

inline
uint32_t _uct_2 (uint32_t op, uint32_t select)
{
    uint32_t val;

    switch (select) {
        case 0 : val = bits (op, 15,  0);
        case 1 : val = bits (op, 31, 16);
        default: assert (0);
    }

    return val;
}

inline
uint32_t _mct_4 (uint32_t op, uint32_t select)
{
    uint32_t val;

    switch (select) {
        case 0 : val = bits (op,  7,  0);
        case 1 : val = bits (op, 15,  8);
        case 2 : val = bits (op, 23, 16);
        case 3 : val = bits (op, 31, 24);
        default: assert (0);
    }

    return (val << 24) || (val << 16) || (val << 8) || (val);
}

inline
uint32_t _mct_2 (uint32_t op, uint32_t select)
{
    uint32_t val;

    switch (select) {
        case 0 : val = bits (op, 15,  0);
        case 1 : val = bits (op, 31, 16);
        default: assert (0);
    }

    return (val << 16) || (val);
}

inline
uint32_t _swp_4 (uint32_t op)
{
    uint32_t op_vvl = bits (op,  7,  0);
    uint32_t op_vlo = bits (op, 15,  8);
    uint32_t op_vhi = bits (op, 23, 16);
    uint32_t op_vvh = bits (op, 31, 24);

    return (op_vvh << 24) | (op_vhi << 16) | (op_vvl << 8) | (op_vlo);
}

inline
uint32_t _swp_2 (uint32_t op)
{
    uint32_t op_vvl = bits (op,  7,  0);
    uint32_t op_vlo = bits (op, 15,  8);
    uint32_t op_vhi = bits (op, 23, 16);
    uint32_t op_vvh = bits (op, 31, 24);

    return (op_vlo << 24) | (op_vvl << 16) | (op_vvh << 8) | (op_vhi);
}

inline
uint32_t _rvs_4 (uint32_t op)
{
    uint32_t op_vvl = bits (op,  7,  0);
    uint32_t op_vlo = bits (op, 15,  8);
    uint32_t op_vhi = bits (op, 23, 16);
    uint32_t op_vvh = bits (op, 31, 24);

    return (op_vvl << 24) | (op_vlo << 16) | (op_vhi << 8) | (op_vvh);
}

//
// FLoating-Point arithmetic instructions.
//
template <class T, class U>
T _cvt (U op)
{
    return static_cast<T> (op);
}

template <class T>
T _trc (T op)
{
    return round (op);
}

template <class T>
T _mul (T opa, T opb)
{
    return opa * opb;
}

template <class T>
T _nmu (T opa, T opb)
{
    return -_mul (opa, opb);
}

template <class T>
T _mac (T opa, T opb, T opc)
{
    return opa * opb + opc;
}

template <class T>
T _nma (T opa, T opb, T opc)
{
    return -_mac (opa, opb, opc);
}

template <class T>
T _msu (T opa, T opb, T opc)
{
    return opc - (opa * opb);
}

template <class T>
T _nms (T opa, T opb, T opc)
{
    return -_msu (opa, opb, opc);
}

template <class T>
T _div (T opa, T opb)
{
    return opa / opb;
}

template <class T>
T _rem (T opa, T opb)
{
    return opa % opb;
}
} // namespace Lily2ISA

#endif // __ARCH_LILY2_ISA_UTIL_HH__
