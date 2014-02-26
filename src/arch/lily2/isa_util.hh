/*
 * Copyright (C) 2014 DSP Group, Institute of Microelectronics, Tsinghua University
 * All rights reserved.
 */

#ifndef __ARCH_LILY2_ISA_UTIL_HH__
#define __ARCH_LILY2_ISA_UTIL_HH__

#include "base/types.hh"

namespace Lily2ISA
{
template <class T>
T _add (T opa, T opb)
{
    return opa + opb;
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
T _tgt (T opa, T opb)
{
    return (opa > opb) ? static_cast<T> (1) : static_cast<T> (0);
}

template <class T>
T _tge (T opa, T opb)
{
    return (opa >= opb) ? static_cast<T> (1) : static_cast<T> (0);
}

template <class T>
T _tlt (T opa, T opb)
{
    return (opa < opb) ? static_cast<T> (1) : static_cast<T> (0);
}

template <class T>
T _tle (T opa, T opb)
{
    return (opa <= opb) ? static_cast<T> (1) : static_cast<T> (0);
}

template <class T>
T _teq (T opa, T opb)
{
    return (opa == opb) ? static_cast<T> (1) : static_cast<T> (0);
}

template <class T>
T _tne (T opa, T opb)
{
    return (opa != opb) ? static_cast<T> (1) : static_cast<T> (0);
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

} // namespace Lily2ISA

#endif // __ARCH_LILY2_ISA_UTIL_HH__
