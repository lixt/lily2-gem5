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
