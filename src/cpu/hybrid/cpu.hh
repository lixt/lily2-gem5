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
#include "cpu/hybrid/resources/bpred.hh"
#include "cpu/hybrid/resources/bpred_config.hh"
#include "cpu/hybrid/resources/vpred.hh"
#include "cpu/hybrid/resources/vpred_config.hh"
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
    typedef TheISA::RunMode_t RunMode_t;
    typedef TheISA::FuncUnit_t FuncUnit_t;
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
        R_Advance      , // Normal issue quit.

        V_Idle         , // 0 inst has issued.
        V_Run          , // At least 1 inst has issued.
        V_Advance      , // Normal issue quit.

        NumStates      ,
    };

    static const std::string PipelineStateStr[NumStates];

    // Types for pipeline events.
    enum PipelineEvent {
        Issue     , // Issue success.
        NoIssue   , // Issue failure.
        Block     ,
        Bubble    ,
        Mode      ,
        EndTick   , // End this tick().
    };

  private:
    Macho<PipelineState, PipelineEvent> pipelineMacho;
    PipelineState curPipelineState;
    PipelineEvent curPipelineEvent;

    MachInst inst;
    Lily2StaticInstPtr curStaticInst;
    StaticInstPtr preStaticInst;

    // Branch predictor.
    BPredictor<BPredEntries, BPredCorrBits> bPredictor;

    // Branch predictor statistics.
    size_t numPredict;
    size_t numPreded;

    // Value predictor.
    VPredictor<VPredEntries> vPredictor;

  private:
    // Initializes the pipeline state machine.
    void initPipelineMacho (void);

  private:
    // Things to be done at the beginning of the tick function.
    void beginTick (void);

    // Things to be done at the end of the tick function.
    void endTick (void);

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
    void rPostExecute (void);
    void vPostExecute (void);

    // Updates the cycle-related modules.
    void refresh (void);
    void rRefresh (void);
    void vRefresh (void);

  private:
    //
    // CPU running cycles.
    //

    Cycles cycle;

    // Get the cpu running cycles.
    Cycles getCycle (void) const
    {
        return cycle;
    }

    // Set the cpu running cycles.
    void setCycle (const Cycles& cycle)
    {
        this->cycle = cycle;
    }

  private:
    //
    // Number of instruction issued.
    //

    size_t numIssued;

    // Gets number of the instructions issued.
    size_t getNumIssued (void) const
    {
        return numIssued;
    }

    // Sets number of the instructions issued.
    void setNumIssued (size_t numIssued)
    {
        this->numIssued = numIssued;
    }

  private:
    //
    // Issue width.
    //

    const size_t IssueWidth;

    // Gets the issue width.
    size_t getIssueWidth (void) const
    {
        return IssueWidth;
    }

  private:
    //
    // Number of the store instructions per issue interfaces.
    //

    // Number of the store instruction per issue.
    size_t numStore;

    // Gets the number of the store instruction per issue.
    size_t getNumStore (void) const
    {
        return numStore;
    }

    // Sets the number of the store instruction per issue.
    void setNumStore (size_t numStore)
    {
        this->numStore = numStore;
    }

  private:
    //
    // Functional unit of the last issued instruction.
    //

    FuncUnit_t lastFuncUnit;

    // Gets the functional unit of the last issued instruction.
    FuncUnit_t getLastFuncUnit (void) const
    {
        return lastFuncUnit;
    }

    // Sets the functional unit of the last issued instruction.
    void setLastFuncUnit (FuncUnit_t lastFuncUnit)
    {
        this->lastFuncUnit = lastFuncUnit;
    }

  private:
    //
    // Maximum block cycles.
    //

    Cycles block;

    // Gets the maximum block cycles.
    Cycles getBlock (void) const
    {
        return block;
    }

    // Sets the maximum block cycles.
    void setBlock (const Cycles &block)
    {
        this->block = block;
    }

    // Tries to set the maximum block cycles.
    void trySetBlock (const Cycles &block)
    {
        this->block = block > this->block ? block : this->block;
    }

  private:
    //
    // Maximum bubble cycles.
    //

    Cycles bubble;

    // Gets the maximum bubble cycles.
    Cycles getBubble (void) const
    {
        return bubble;
    }

    // Sets the maximum bubble cycles.
    void setBubble (const Cycles &bubble)
    {
        this->bubble = bubble;
    }

    // Tries to set the maximum bubble cycles.
    void trySetBubble (const Cycles &bubble)
    {
        this->bubble = bubble > this->bubble ? bubble : this->bubble;
    }

  private:
    //
    // Register dependence tables.
    //

    RegDepTable<TheISA::NumXRegs> xRegDepTable; // X.
    RegDepTable<TheISA::NumYRegs> yRegDepTable; // Y.
    RegDepTable<TheISA::NumGRegs> gRegDepTable; // G.
    RegDepTable<TheISA::NumMRegs> mRegDepTable; // M.

    // Mutates the cycle of a given operand in the register dependence table.
    void mutateRegDep (const Op_t& op, const Cycles&);

    // Mutates the cycle of a given register in the register dependence table.
    void mutateRegDep (const RegFile_t&, const RegIndex_t&, const Cycles&);

    // Mutates the cycle of a given register in a given register dependence table.
    template <size_t RegNum>
    void mutateRegDep (RegDepTable<RegNum>&, const RegIndex_t&, const Cycles&);

    // Checks the register dependence of the current instruction.
    bool isRegDep (void) const;

    // Checks the register dependence of an operand.
    bool isRegDep (const Op_t& op) const;

    // Checks the register dependence of a register.
    bool isRegDep (const RegFile_t&, const RegIndex_t&) const;

    // Checks the register dependence in the given register dependence table.
    template <size_t RegNum>
    bool isRegDep (const RegDepTable<RegNum>&, const RegIndex_t&) const;

    // Inserts registers of the current instruction into the register dependence tables.
    void insertRegDep (void);

    // Inserts registers of an operand into the register dependence tables.
    void insertRegDep (const Op_t& op);

    // Inserts a given register into the register dependence table.
    void insertRegDep (const RegFile_t&, const RegIndex_t&, const Cycles&);

    // Inserts a given register into the given register dependence table.
    template <size_t RegNum>
    void insertRegDep (RegDepTable<RegNum>&, const RegIndex_t&, const Cycles&);

    // Checks if all the register dependence tables are empty.
    bool isRegDepTableEmpty (void) const;

    // Checks if the given register dependence table is empty.
    template <size_t RegNum>
    bool isRegDepTableEmpty (const RegDepTable<RegNum>&) const;

    // Gets the maximum register back cycle in all the register dependence table.
    Cycles maxRegDepCycle (void) const;

    // Gets the maximum register back cycle in the given register dependence table.
    template <size_t RegNum>
    Cycles maxRegDepCycle (const RegDepTable<RegNum>&) const;

    // Refreshes all the register dependence table by the given cycle.
    void refreshRegDepTable (const Cycles&);

    // Refreshes the given register dependence table by the given cycle.
    template <size_t RegNum>
    void refreshRegDepTable (RegDepTable<RegNum>&, const Cycles&);

  private:
    //
    // Register file and register file buffers.
    //

    // Gets the value of a given register in a given register file.
    template <size_t RegNum, class RegValue_t>
    RegValue_t getRegValue (const TheISA::RegFile<RegNum, RegValue_t>&, const RegIndex_t&) const;

    // Sets the value of a given register in a given register file.
    template <size_t RegNum, class RegValue_t>
    void setRegValue (TheISA::RegFile<RegNum, RegValue_t>&, const RegIndex_t&, const RegValue_t&);

    // Gets the value of a given register in a given register file buffer.
    template <size_t RegNum, class RegValue_t>
    RegValue_t getRegBufValue (const TheISA::RegFileBuf<RegNum, RegValue_t>&, const RegIndex_t&) const;

    // Sets the value of a given register in a given register file buffer.
    template <size_t RegNum, class RegValue_t>
    void setRegBufValue (TheISA::RegFileBuf<RegNum, RegValue_t>&, const RegIndex_t&,
                         const RegValue_t&, const RegValue_t&, const Cycles&);

    // Sets the register back cycle of a given operand in the register file buffers.
    void setRegBufCycle (const Op_t& op, const Cycles&);

    void setRegBufCycle (const RegFile_t& fileName, const RegIndex_t& regIndex, const Cycles&);

    // Sets the register back cycle of a given register in a given register file buffer.
    template <size_t RegNum, class RegValue_t>
    void setRegBufCycle (TheISA::RegFileBuf<RegNum, RegValue_t>&, const RegIndex_t&, const Cycles&);

    void refreshRegs (const Cycles&);

    template <size_t RegNum, class RegValue_t>
    void refreshRegs (TheISA::RegFile<RegNum, RegValue_t>&,
                      TheISA::RegFileBuf<RegNum, RegValue_t>&, const Cycles&);

  public:
    //
    // PC state interfaces.
    //

    // Gets the current instruction address.
    Addr pc (void) const
    {
        TheISA::PCState pcState = thread->pcState ();
        return pcState.pc ();
    }

    // Gets the next instruction address.
    Addr npc (void) const
    {
        TheISA::PCState pcState = thread->pcState ();
        return pcState.npc ();
    }

    // Gets the return address.
    Addr rpc (void) const
    {
        GRegValue_t retAddr
            = getRegValue (thread->getGRegFile (), TheISA::RAddrReg);
        return static_cast<Addr> (retAddr);
    }

    // Sets the branch target.
    void bpc (Addr branchTarget)
    {
        TheISA::PCState pcState = thread->pcState ();
        pcState.bpc (branchTarget);
        thread->pcState (pcState);
    }

    // Sets the load instruction effective address.
    void lpc (Addr effAddr)
    {
        TheISA::PCState pcState = thread->pcState ();
        pcState.lpc (effAddr);
        thread->pcState (pcState);
    }

    // Sets the store instruction effective address.
    void spc (Addr effAddr)
    {
        TheISA::PCState pcState = thread->pcState ();
        pcState.spc (effAddr);
        thread->pcState (pcState);
    }

  private:
    // Interfaces for debugging.
    void debugPipeline (std::ostream&) const;

  private:
    // What's the meaning of "bubble" and "block" here?
    // Word "bubble" means no dispatch, but keeps on executing and writing back.
    // Word "block" means no dispatch, no execute and no write back.

    // Gets the bubble cycles.
    // In RISC, branch misprediction and mode switching to VLIW causes bubbles.
    // In VLIW, nothing causes bubbles.
    Cycles rBubbles (const OpClass&) const;
    Cycles vBubbles (const OpClass&) const;

    // Gets the block cycles.
    // In RISC, iterative calculation causes blocks.
    // In VLIW, iterative calculation, branch misprediction, value misprediction causes
    // blocks.
    Cycles rBlocks (const OpClass&) const;
    Cycles vBlocks (const OpClass&) const;

    //
    RunMode_t curRunMode () const;

    // Factory of functional unit delay slots.
    Cycles funcUnitLatencyFactory (const OpClass &opClass, bool memFlag) const;

    typedef TheISA::Op32i_t Op32i_t;
    typedef TheISA::Op64i_t Op64i_t;
    typedef TheISA::Op32f_t Op32f_t;
    typedef TheISA::Op64f_t Op64f_t;
    typedef TheISA::Opq8i_t Opq8i_t;
    typedef TheISA::Opd16i_t Opd16i_t;
    typedef TheISA::Opq16i_t Opq16i_t;
    typedef TheISA::Opd32i_t Opd32i_t;
    typedef TheISA::Opd32f_t Opd32f_t;
    typedef Lily2ISAInst::Lily2StaticInst Lily2StaticInst;

  public:
    bool readCond (const Lily2StaticInst *) const;

    const Op32i_t&  readOp32i  (Lily2StaticInst*, const OpCount_t&);
    const Op64i_t&  readOp64i  (Lily2StaticInst*, const OpCount_t&);
    const Op32f_t&  readOp32f  (Lily2StaticInst*, const OpCount_t&);
    const Op64f_t&  readOp64f  (Lily2StaticInst*, const OpCount_t&);
    const Opq8i_t&  readOpq8i  (Lily2StaticInst*, const OpCount_t&);
    const Opd16i_t& readOpd16i (Lily2StaticInst*, const OpCount_t&);
    const Opq16i_t& readOpq16i (Lily2StaticInst*, const OpCount_t&);
    const Opd32i_t& readOpd32i (Lily2StaticInst*, const OpCount_t&);
    const Opd32f_t& readOpd32f (Lily2StaticInst*, const OpCount_t&);

    void setOp32i  (Lily2StaticInst*, const OpCount_t&, const Op32i_t& , const Op32i_t& );
    void setOp64i  (Lily2StaticInst*, const OpCount_t&, const Op64i_t& , const Op64i_t& );
    void setOp32f  (Lily2StaticInst*, const OpCount_t&, const Op32f_t& , const Op32f_t& );
    void setOp64f  (Lily2StaticInst*, const OpCount_t&, const Op64f_t& , const Op64f_t& );
    void setOpq8i  (Lily2StaticInst*, const OpCount_t&, const Opq8i_t& , const Opq8i_t& );
    void setOpd16i (Lily2StaticInst*, const OpCount_t&, const Opd16i_t&, const Opd16i_t&);
    void setOpq16i (Lily2StaticInst*, const OpCount_t&, const Opq16i_t&, const Opq16i_t&);
    void setOpd32i (Lily2StaticInst*, const OpCount_t&, const Opd32i_t&, const Opd32i_t&);
    void setOpd32f (Lily2StaticInst*, const OpCount_t&, const Opd32f_t&, const Opd32f_t&);

  private:
    // Python configuration variables.

    // Functional unit latency.
    int IntArithLatency;
    int IntLogicLatency;
    int IntTestLatency;
    int IntShiftLatency;
    int IntBitLatency;
    int IntMoveLatency;
    int IntMulLatency;
    int IntMacLatency;
    int IntDivLatency;
    int IntRemLatency;
    int IntMemSLatency;
    int IntMemLLatency;
    int IntMemAddrLatency;
    int IntMiscLatency;
    int IntBranchLatency;
    int FloatArithLatency;
    int FloatTestLatency;
    int FloatMulLatency;
    int FloatMacLatency;
    int FloatDivLatency;
    int FloatSqrLatency;

    // Branch delay slot.
    int BranchDelaySlot;

    // Iterative instruction stalls.
    int IntDivBlock;
    int IntRemBlock;
    int FloatDivBlock;
    int FloatSqrBlock;

    // Mode switching delay slot.
    int ModeDelaySlot;
};

template <size_t RegNum>
void
HybridCPU::mutateRegDep (RegDepTable<RegNum>& regDepTable,
                         const RegIndex_t& regIndex,
                         const Cycles& regBackCycle)
{
    regDepTable.mutateReg (regIndex, regBackCycle);
}

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
HybridCPU::insertRegDep (RegDepTable<RegNum>& regDepTable,
                      const RegIndex_t& regIndex, const Cycles& regBackCycle)
{
    regDepTable.insertReg (regIndex, regBackCycle);
}

template <size_t RegNum>
Cycles
HybridCPU::maxRegDepCycle (const RegDepTable<RegNum>& regDepTable) const
{
    return regDepTable.maxCycle ();
}

template <size_t RegNum>
void
HybridCPU::refreshRegDepTable (RegDepTable<RegNum>& regDepTable, const Cycles& decrRegBackCycleDelta)
{
    regDepTable.update (decrRegBackCycleDelta);
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

template <size_t RegNum, class RegValue_t>
void
HybridCPU::setRegBufCycle (TheISA::RegFileBuf<RegNum, RegValue_t>& regFileBuf,
                           const RegIndex_t& regIndex, const Cycles& regBackCycle)
{
    return regFileBuf.setRegBackCycle (regIndex, regBackCycle);
}

template <size_t RegNum, class RegValue_t>
void
HybridCPU::refreshRegs (TheISA::RegFile<RegNum, RegValue_t>& regFile,
                        TheISA::RegFileBuf<RegNum, RegValue_t>& regFileBuf,
                        const Cycles& decrRegBackCycleDelta)
{
    // Vector to store the due registers.
    std::vector<typename TheISA::RegFileBuf<RegNum, RegValue_t>::Position> vecRemovePos =
        regFileBuf.decrRegBackCycle (decrRegBackCycleDelta);

    for (typename std::vector<typename TheISA::RegFileBuf<RegNum, RegValue_t>::Position>::iterator it
            = vecRemovePos.begin (); it != vecRemovePos.end (); ++it) {

        RegIndex_t regIndex = regFileBuf.getRegIndex (*it);

        RegValue_t regMask = regFileBuf.getRegMask (*it);
        RegValue_t regNewValue = regFileBuf.getRegValue (*it);
        RegValue_t regOldValue = regFile.getRegValue (regIndex);

        // Writes the buffered value back to the register.
        regFile.setRegValue (regIndex, (regNewValue & regMask) | (regOldValue & ~regMask));

        // Removes the due registers in the register file buffer.
        regFileBuf.remove (*it);
    }
}

#endif // __CPU_HYBRID_CPU_HH__
