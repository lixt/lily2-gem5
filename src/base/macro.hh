/*
 * Copyright (c) 2014 DSP Group, Institute of Microelectronics, Tsinghua University.
 * All rights reserved.
 */

#ifndef __BASE_MACRO_HH__
#define __BASE_MACRO_HH__

#define DEC std::dec
#define HEX_08 "0x" << std::hex << std::setfill ('0') << std::setw (2)
#define HEX_16 "0x" << std::hex << std::setfill ('0') << std::setw (4)
#define HEX_32 "0x" << std::hex << std::setfill ('0') << std::setw (8)
#define HEX_64 "0x" << std::hex << std::setfill ('0') << std::setw (16)

// Output formats of operands.
#define OP32I_DEC DEC
#define OP32I_HEX HEX_32
#define OP32F_DEC DEC
#define OP32F_HEX HEX_32
#define OP64F_DEC DEC
#define OP64F_HEX HEX_64

#define REG_OUTPUT_FORMAT "0x" << std::hex << std::setfill (0) << std::setw (8)

#endif // __BASE_MACRO_HH__
