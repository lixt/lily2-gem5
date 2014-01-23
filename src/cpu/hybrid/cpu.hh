/*
 * Copyright (c) 2014 DSP Group, Institute of Microelectronics, Tsinghua University
 * All rights reserved.
 *
 * Copyright (c) 2012-2013 ARM Limited
 * All rights reserved.
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __CPU_HYBRID_CPU_HH__
#define __CPU_HYBRID_CPU_HH__

#include "base/hashmap.hh"
#include "cpu/hybrid/base.hh"
#include "params/HybridCPU.hh"
#include "arch/lily2/operands.hh"
#include "arch/lily2/static_inst.hh"

/**
 *  Start and end address of basic block for SimPoint profiling.
 *  This structure is used to look up the hash table of BBVs.
 *  - first: PC of first inst in basic block
 *  - second: PC of last inst in basic block
 */
typedef std::pair<Addr, Addr> BasicBlockRange;

/** Overload hash function for BasicBlockRange type */
__hash_namespace_begin
template <>
struct hash<BasicBlockRange>
{
  public:
    size_t operator()(const BasicBlockRange &bb) const {
        return hash<Addr>()(bb.first + bb.second);
    }
};
__hash_namespace_end


class HybridCPU : public BaseSimpleCPU
{
  public:

    HybridCPU(HybridCPUParams *params);
    virtual ~HybridCPU();

    virtual void init();

  private:

    struct TickEvent : public Event
    {
        HybridCPU *cpu;

        TickEvent(HybridCPU *c);
        void process();
        const char *description() const;
    };

    TickEvent tickEvent;

    const int width;
    bool locked;
    const bool simulate_data_stalls;
    const bool simulate_inst_stalls;

    /**
     * Drain manager to use when signaling drain completion
     *
     * This pointer is non-NULL when draining and NULL otherwise.
     */
    DrainManager *drain_manager;

    // main simulation loop (one cycle)
    void tick();

    /**
     * Check if a system is in a drained state.
     *
     * We need to drain if:
     * <ul>
     * <li>We are in the middle of a microcode sequence as some CPUs
     *     (e.g., HW accelerated CPUs) can't be started in the middle
     *     of a gem5 microcode sequence.
     *
     * <li>The CPU is in a LLSC region. This shouldn't normally happen
     *     as these are executed atomically within a single tick()
     *     call. The only way this can happen at the moment is if
     *     there is an event in the PC event queue that affects the
     *     CPU state while it is in an LLSC region.
     *
     * <li>Stay at PC is true.
     * </ul>
     */
    bool isDrained() {
        return microPC() == 0 &&
            !locked &&
            !stayAtPC;
    }

    /**
     * Try to complete a drain request.
     *
     * @returns true if the CPU is drained, false otherwise.
     */
    bool tryCompleteDrain();

    /**
     * An AtomicCPUPort overrides the default behaviour of the
     * recvAtomicSnoop and ignores the packet instead of panicking. It
     * also provides an implementation for the purely virtual timing
     * functions and panics on either of these.
     */
    class AtomicCPUPort : public MasterPort
    {

      public:

        AtomicCPUPort(const std::string &_name, BaseCPU* _cpu)
            : MasterPort(_name, _cpu)
        { }

      protected:

        virtual Tick recvAtomicSnoop(PacketPtr pkt)
        {
            // Snooping a coherence request, just return
            return 0;
        }

        bool recvTimingResp(PacketPtr pkt)
        {
            panic("Atomic CPU doesn't expect recvTimingResp!\n");
            return true;
        }

        void recvRetry()
        {
            panic("Atomic CPU doesn't expect recvRetry!\n");
        }

    };

    AtomicCPUPort icachePort;
    AtomicCPUPort dcachePort;

    bool fastmem;
    Request ifetch_req;
    Request data_read_req;
    Request data_write_req;

    bool dcache_access;
    Tick dcache_latency;

    /**
     * Profile basic blocks for SimPoints.
     * Called at every macro inst to increment basic block inst counts and
     * to profile block if end of block.
     */
    void profileSimPoint();

    /** Data structures for SimPoints BBV generation
     *  @{
     */

    /** Whether SimPoint BBV profiling is enabled */
    const bool simpoint;
    /** SimPoint profiling interval size in instructions */
    const uint64_t intervalSize;

    /** Inst count in current basic block */
    uint64_t intervalCount;
    /** Excess inst count from previous interval*/
    uint64_t intervalDrift;
    /** Pointer to SimPoint BBV output stream */
    std::ostream *simpointStream;

    /** Basic Block information */
    struct BBInfo {
        /** Unique ID */
        uint64_t id;
        /** Num of static insts in BB */
        uint64_t insts;
        /** Accumulated dynamic inst count executed by BB */
        uint64_t count;
    };

    /** Hash table containing all previously seen basic blocks */
    m5::hash_map<BasicBlockRange, BBInfo> bbMap;
    /** Currently executing basic block */
    BasicBlockRange currentBBV;
    /** inst count in current basic block */
    uint64_t currentBBVInstCount;

    /** @}
     *  End of data structures for SimPoints BBV generation
     */

  protected:

    /** Return a reference to the data port. */
    virtual MasterPort &getDataPort() { return dcachePort; }

    /** Return a reference to the instruction port. */
    virtual MasterPort &getInstPort() { return icachePort; }

  public:

    unsigned int drain(DrainManager *drain_manager);
    void drainResume();

    void switchOut();
    void takeOverFrom(BaseCPU *oldCPU);

    void verifyMemoryMode() const;

    virtual void activateContext(ThreadID thread_num, Cycles delay);
    virtual void suspendContext(ThreadID thread_num);

    Fault readMem(Addr addr, uint8_t *data, unsigned size, unsigned flags);

    Fault writeMem(uint8_t *data, unsigned size,
                   Addr addr, unsigned flags, uint64_t *res);

    /**
     * Print state of address in memory system via PrintReq (for
     * debugging).
     */
    void printAddr(Addr a);

    // LILY2 interfaces.
    void setupFetchRequest (Request *req);


    typedef TheISA::Op32i_t Op32i_t;
    typedef TheISA::Op32f_t Op32f_t;
    typedef TheISA::Op64f_t Op64f_t;
    typedef TheISA::Opq8i_t Opq8i_t;
    typedef TheISA::Opd16i_t Opd16i_t;
    typedef TheISA::Opq16i_t Opq16i_t;
    typedef TheISA::Opd32i_t Opd32i_t;
    typedef TheISA::Opd32f_t Opd32f_t;
    typedef Lily2ISAInst::Lily2StaticInst Lily2StaticInst;

    Op32i_t readOp32i (const Lily2StaticInst *si, int idx)
    {
        return Op32i_t (static_cast<uint32_t> (0));
    }

    Op32f_t readOp32f (const Lily2StaticInst *si, int idx)
    {
        return Op32f_t (static_cast<float> (0.0));
    }

    Op64f_t readOp64f (const Lily2StaticInst *si, int idx)
    {
        return Op64f_t (static_cast<double> (0.0));
    }

    Opq8i_t readOpq8i (const Lily2StaticInst *si, int idx)
    {
        return Opq8i_t (static_cast<uint8_t> (0));
    }

    Opd16i_t readOpd16i (const Lily2StaticInst *si, int idx)
    {
        return Opd16i_t (static_cast<uint16_t> (0));
    }

    Opq16i_t readOpq16i (const Lily2StaticInst *si, int idx)
    {
        return Opq16i_t (static_cast<uint16_t> (0));
    }

    Opd32i_t readOpd32i (const Lily2StaticInst *si, int idx)
    {
        return Opd32i_t (static_cast<uint16_t> (0));
    }

    Opd32f_t readOpd32f (const Lily2StaticInst *si, int idx)
    {
        return Opd32f_t (static_cast<float> (0.0));
    }

    void setOp32i (const Lily2StaticInst *si, int idx,
                   Op32i_t &val, Op32i_t &mask)
    {}

    void setOp32f (const Lily2StaticInst *si, int idx,
                   Op32f_t &val, Op32f_t &mask)
    {}

    void setOp64f (const Lily2StaticInst *si, int idx,
                   Op64f_t &val, Op64f_t &mask)
    {}

    void setOpq8i (const Lily2StaticInst *si, int idx,
                   Opq8i_t &val, Opq8i_t &mask)
    {}

    void setOpd16i (const Lily2StaticInst *si, int idx,
                    Opd16i_t &val, Opd16i_t &mask)
    {}

    void setOpq16i (const Lily2StaticInst *si, int idx,
                    Opq16i_t &val, Opq16i_t &mask)
    {}

    void setOpd32i (const Lily2StaticInst *si, int idx,
                    Opd32i_t &val, Opd32i_t &mask)
    {}

    void setOpd32f (const Lily2StaticInst *si, int idx,
                    Opd32f_t &val, Opd32f_t &mask)
    {}
};

#endif // __CPU_HYBRID_CPU_HH__
