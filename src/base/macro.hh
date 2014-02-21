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

// Output formats of cycles.
#define IO_CYCLE DEC

// Output formats of addresses.
#define IO_ADDR HEX_32

// Output formats of machine codes.
#define IO_MACHINST HEX_32

// Output formats of operands.
#define IO_OP32I_DEC DEC
#define IO_OP32I_HEX HEX_32
#define IO_OP32F_DEC DEC
#define IO_OP32F_HEX HEX_32
#define IO_OP64F_DEC DEC
#define IO_OP64F_HEX HEX_64

// Output formats of registers.
#define IO_REGINDEX DEC
#define IO_REGVALUE "0x" << std::hex << std::setfill (0) << std::setw (8)

#endif // __BASE_MACRO_HH__
