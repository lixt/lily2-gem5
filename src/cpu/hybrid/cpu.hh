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

#include <iostream>
#include <iomanip>
#include "base/hashmap.hh"
#include "base/macho.hh"
#include "cpu/hybrid/base.hh"
#include "cpu/hybrid/resources/reg_dep.hh"
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

    /************************* LILY2 Parts ***************************/

  private:
    // arch/lily2/types.hh
    typedef TheISA::MachInst MachInst;
    typedef TheISA::DispMode_t DispMode_t;
    // arch/lily2/registers.hh
    typedef TheISA::RegCount_t RegCount_t;
    typedef TheISA::RegFile_t RegFile_t;
    typedef TheISA::RegIndex_t RegIndex_t;
    typedef TheISA::XRegValue_t XRegValue_t;
    typedef TheISA::YRegValue_t YRegValue_t;
    typedef TheISA::GRegValue_t GRegValue_t;
    typedef TheISA::MRegValue_t MRegValue_t;
    // arch/lily2/operands.hh
    typedef TheISA::OpCount_t OpCount_t;
    typedef TheISA::OpLabel_t OpLabel_t;
    typedef TheISA::Op_t Op_t;
    // arch/lily2/static_inst.hh
    typedef Lily2ISAInst::Lily2StaticInstPtr Lily2StaticInstPtr;
    // cpu/simple_thread.hh
    typedef SimpleThread::XRegFile XRegFile;
    typedef SimpleThread::YRegFile YRegFile;
    typedef SimpleThread::GRegFile GRegFile;
    typedef SimpleThread::MRegFile MRegFile;
    typedef SimpleThread::XRegFileBuf XRegFileBuf;
    typedef SimpleThread::YRegFileBuf YRegFileBuf;
    typedef SimpleThread::GRegFileBuf GRegFileBuf;
    typedef SimpleThread::MRegFileBuf MRegFileBuf;

  private:
    // Types for pipeline states.
    enum PipelineState {
        FaultState     ,

        R_Idle         , // No instruction executed this iteration.
        R_Run          , // At least 1 inst executed this iteration.
        R_Flush        , // Branch misprediction penalty.
        R_InstWait     , // Waits for iterative inst.
        R_Advance      , // Quits this iteration.

        V_Idle         ,
        R_2_R          ,
        R_2_V          ,
        V_2_R          ,
        V_2_V          ,

        NumStates      ,
    };

    static const std::string PipelineStateStr[NumStates];

    // Types for pipeline events.
    enum PipelineEvent {
        Issue     , // Issue success.
        NoIssue   , // Issue failure.
        BPreded   , // Branch predicted.
        MisBPred  , // Branch misprediction.
        VPreded   , // Value predicted.
        MisVPred  , // Value misprediction.
        IterInst  , // Iterative instructions.
        ICacheMiss, // Instruction cache misses.
        DCacheMiss, // Data cache misses.
        ToRiscInst, // Risc to Risc.
        ToVliwInst, // Vliw to Vliw.
    };

    // Types for pipeline callbacks.
    typedef void (HybridCPU::*PipelineCallback) (void);

    // Pipeline callbacks.
    void callback_R_Idle (void);
    void callback_R_Run (void);
    void callback_R_Flush (void);
    void callback_R_InstWait (void);
    void callback_R_Advance (void);
    void callback_R_2_R (void);
    void callback_R_2_V (void);

  private:
    Macho<PipelineState, PipelineEvent> pipelineMacho;
    PipelineState curPipelineState;
    PipelineEvent curPipelineEvent;
    PipelineCallback curPipelineCallback;
    PipelineCallback prePipelineCallback;
    PipelineCallback postPipelineCallback;

    Cycles cycle;

    MachInst inst;
    Lily2StaticInstPtr curStaticInst;

    // Issued instructions.
    int issued;

    // Register dependence tables.
    RegDepTable<TheISA::NumXRegs> xRegDepTable;
    RegDepTable<TheISA::NumYRegs> yRegDepTable;
    RegDepTable<TheISA::NumGRegs> gRegDepTable;
    RegDepTable<TheISA::NumMRegs> mRegDepTable;

  private:
    // Initializes the pipeline state machine.
    void initPipelineMacho (void);
    // Calls pipeline callbacks.
    void callPrePipelineCallback (void);
    void callPostPipelineCallback (void);
    // Sets the pre/post callbacks.
    void setCallback (void);

    // Fetches the instruction from icache and returns the fetch cycles.
    // The fetched instruction machine code is stored in the member
    // variable INST.
    Cycles fetch (void);
    // FETCH auxiliary function.
    void setupFetchRequest (Request *req);

    // Decodes the instruction machine code stored in the member variable
    // INST. The decoded pointer to static instruction is stored in the
    // member variable CURSTATICINST.
    void decode (void);

    // Dispatches the instruction stored in the member variable CURSTATICINST.
    // If the instruction can be issued, hold the CURSTATICINST. Otherwise,
    // set the CURSTATICINST a null pointer.
    void dispatch (void);
    void rDispatch (void);
    void vDispatch (void);

    // Things done before the execution. Updates the dispatch infos.
    void preExecute (void);
    void rPreExecute (void);
    void vPreExecute (void);

    // Executes the instruction stored in the member variable CURSTATICINST
    // if it is not a null pointer.
    void execute (void);

    // Things done after the execution. Updates the pipeline state.
    void postExecute (void);

    // Updates the cycle-related modules.
    void renew (void);

  private:
    // Interfaces for member functions.
    bool isOverIssueWidth     (void) const;
    void writeIssued          (void);
    // Refresh.
    // Refreshes the issued instructions.
    void refreshIssued (void);

    // Refreshes the register dependence table.
    void refreshRegDepTable (const Cycles& decrRegBackCycleDelta);
    template <size_t RegNum>
    void refreshRegDepTable (RegDepTable<RegNum>&, const Cycles& decrRegBackCycleDelta);

    // Refreshes the register file buffers and writes value back to the register file.
    void refreshRegs (const Cycles& decrRegBackCycleDelta);
    template <size_t RegNum, class RegValue_t>
    void refreshRegs (TheISA::RegFile<RegNum, RegValue_t>&,
            TheISA::RegFileBuf<RegNum, RegValue_t>&, const Cycles& decrRegBackCycleDelta);

    // Refreshes the cycles.
    void refreshCycle (const Cycles& decrRegBackCycleDelta);

  private:
    // Register dependence table interfaces.
    bool isRegDepTableEmpty (void) const;

    template <size_t RegNum>
    bool isRegDepTableEmpty (const RegDepTable<RegNum>&) const;

    bool isRegDep (void) const;

    bool isRegDep (const Op_t& op) const;

    bool isRegDep (const RegFile_t&, const RegIndex_t&) const;

    template <size_t RegNum>
    bool isRegDep (const RegDepTable<RegNum>&, const RegIndex_t&) const;

    void setRegDep (void);

    void setRegDep (const Op_t& op);

    void setRegDep (const RegFile_t&, const RegIndex_t&, const Cycles&);

    template <size_t RegNum>
    void setRegDep (RegDepTable<RegNum>&, const RegIndex_t&, const Cycles&);

  private:
    // Register file interfaces.
    template <size_t RegNum, class RegValue_t>
    RegValue_t getRegValue (const TheISA::RegFile<RegNum, RegValue_t>&, const RegIndex_t&) const;

    template <size_t RegNum, class RegValue_t>
    void setRegValue (TheISA::RegFile<RegNum, RegValue_t>&, const RegIndex_t&, const RegValue_t&);

    // Register file buffer interfaces.
    template <size_t RegNum, class RegValue_t>
    RegValue_t getRegBufValue (const TheISA::RegFileBuf<RegNum, RegValue_t>&, const RegIndex_t&) const;

    template <size_t RegNum, class RegValue_t>
    void setRegBufValue (TheISA::RegFileBuf<RegNum, RegValue_t>&, const RegIndex_t&,
                         const RegValue_t&, const RegValue_t&, const Cycles&);

  public:
    // PC state interfaces.
    void setBranchTarget (Addr);

  private:
    // Interfaces for debugging.
    void debugPipeline (std::ostream&) const;

  private:
    // Factory of dispatch modes.
    DispMode_t dispModeFactory (const PipelineState& pipelineState) const;

    // Factory of functional unit delay slots.
    Cycles funcUnitLatencyFactory (const OpClass &opClass, bool memFlag) const;

    typedef TheISA::FuncUnit_t FuncUnit_t;

    typedef TheISA::Op32i_t Op32i_t;
    typedef TheISA::Op32f_t Op32f_t;
    typedef TheISA::Op64f_t Op64f_t;
    typedef TheISA::Opq8i_t Opq8i_t;
    typedef TheISA::Opd16i_t Opd16i_t;
    typedef TheISA::Opq16i_t Opq16i_t;
    typedef TheISA::Opd32i_t Opd32i_t;
    typedef TheISA::Opd32f_t Opd32f_t;
    typedef Lily2ISAInst::Lily2StaticInst Lily2StaticInst;

  public:
    bool readCond (const Lily2StaticInst *si)
    {
        return 0;
    }


    const Op32i_t&  readOp32i  (Lily2StaticInst*, const OpCount_t&);
    const Op32f_t&  readOp32f  (Lily2StaticInst*, const OpCount_t&);
    const Op64f_t&  readOp64f  (Lily2StaticInst*, const OpCount_t&);
    const Opq8i_t&  readOpq8i  (Lily2StaticInst*, const OpCount_t&);
    const Opd16i_t& readOpd16i (Lily2StaticInst*, const OpCount_t&);
    const Opq16i_t& readOpq16i (Lily2StaticInst*, const OpCount_t&);
    const Opd32i_t& readOpd32i (Lily2StaticInst*, const OpCount_t&);
    const Opd32f_t& readOpd32f (Lily2StaticInst*, const OpCount_t&);

    void setOp32i  (Lily2StaticInst*, const OpCount_t&, const Op32i_t& , const Op32i_t& );
    void setOp32f  (Lily2StaticInst*, const OpCount_t&, const Op32f_t& , const Op32f_t& );
    void setOp64f  (Lily2StaticInst*, const OpCount_t&, const Op64f_t& , const Op64f_t& );
    void setOpq8i  (Lily2StaticInst*, const OpCount_t&, const Opq8i_t& , const Opq8i_t& );
    void setOpd16i (Lily2StaticInst*, const OpCount_t&, const Opd16i_t&, const Opd16i_t&);
    void setOpq16i (Lily2StaticInst*, const OpCount_t&, const Opq16i_t&, const Opq16i_t&);
    void setOpd32i (Lily2StaticInst*, const OpCount_t&, const Opd32i_t&, const Opd32i_t&);
    void setOpd32f (Lily2StaticInst*, const OpCount_t&, const Opd32f_t&, const Opd32f_t&);

  private:
    // Python configuration variables.

    // Issue width.
    int IssueWidth;

    // Functional unit latency.
    int IntArithLatency;
    int IntLogicLatency;
    int IntTestLatency;
    int IntShiftLatency;
    int IntBitLatency;
    int IntMoveLatency;
    int IntMulLatency;
    int IntMacLatency;
    int IntIterLatency;
    int IntMemAccessLatency;
    int IntMemOffsetLatency;
    int IntMiscLatency;
    int SimdIntArithLatency;
    int SimdIntLogicLatency;
    int SimdIntTestLatency;
    int SimdIntShiftLatency;
    int SimdIntMulLatency;
    int SimdIntMacLatency;
    int SimdIntIterLatency;
};

template <size_t RegNum>
bool
HybridCPU::isRegDepTableEmpty (const RegDepTable<RegNum>& regDepTable) const
{
    return regDepTable.empty ();
}

template <size_t RegNum>
bool
HybridCPU::isRegDep (const RegDepTable<RegNum>& regDepTable,
                     const RegIndex_t& regIndex) const
{
    return regDepTable.isRegDep (regIndex);
}

template <size_t RegNum>
void
HybridCPU::setRegDep (RegDepTable<RegNum>& regDepTable,
                      const RegIndex_t& regIndex, const Cycles& regBackCycle)
{
    regDepTable.insertReg (regIndex, regBackCycle);
}

template <size_t RegNum, class RegValue_t>
RegValue_t
HybridCPU::getRegValue (const TheISA::RegFile<RegNum, RegValue_t>& regFile,
                        const RegIndex_t& regIndex) const
{
    return regFile.getRegValue (regIndex);
}

template <size_t RegNum, class RegValue_t>
void
HybridCPU::setRegValue (TheISA::RegFile<RegNum, RegValue_t>& regFile,
                        const RegIndex_t& regIndex,
                        const RegValue_t& regValue)
{
    return regFile.setRegValue (regIndex, regValue);
}

template <size_t RegNum, class RegValue_t>
RegValue_t
HybridCPU::getRegBufValue (const TheISA::RegFileBuf<RegNum, RegValue_t>& regFileBuf,
                           const RegIndex_t& regIndex) const
{
    return regFileBuf.getRegValue (regIndex);
}

template <size_t RegNum, class RegValue_t>
void
HybridCPU::setRegBufValue (TheISA::RegFileBuf<RegNum, RegValue_t>& regFileBuf,
                           const RegIndex_t& regIndex, const RegValue_t& regValue,
                           const RegValue_t& regMask, const Cycles& regBackCycle)
{
    return regFileBuf.insert (regIndex, regValue, regMask, regBackCycle);
}

#endif // __CPU_HYBRID_CPU_HH__
