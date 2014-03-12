/*
 * Copyright (c) 2014 DSP Group, Institute of Microeletronics, Tsinghua University
 * All rights reserved.
 *
 * Copyright (c) 2012 ARM Limited
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
 * Authors: Xiaotian Li
 */

#include "arch/locked_mem.hh"
#include "arch/mmapped_ipr.hh"
#include "arch/utility.hh"
#include "base/bigint.hh"
#include "base/output.hh"
#include "config/the_isa.hh"
#include "cpu/hybrid/cpu.hh"
#include "cpu/exetrace.hh"
#include "debug/Drain.hh"
#include "debug/ExecFaulting.hh"
#include "debug/HybridCPU.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "mem/physical.hh"
#include "params/HybridCPU.hh"
#include "sim/faults.hh"
#include "sim/system.hh"
#include "sim/full_system.hh"

#define DEBUG 1

using namespace std;
using namespace TheISA;

HybridCPU::TickEvent::TickEvent(HybridCPU *c)
    : Event(CPU_Tick_Pri), cpu(c)
{
}


void
HybridCPU::TickEvent::process()
{
    cpu->tick();
}

const char *
HybridCPU::TickEvent::description() const
{
    return "HybridCPU tick";
}

void
HybridCPU::init()
{
    BaseCPU::init();

    // Initialise the ThreadContext's memory proxies
    tcBase()->initMemProxies(tcBase());

    if (FullSystem && !params()->switched_out) {
        ThreadID size = threadContexts.size();
        for (ThreadID i = 0; i < size; ++i) {
            ThreadContext *tc = threadContexts[i];
            // initialize CPU, including PC
            TheISA::initCPU(tc, tc->contextId());
        }
    }

    // Atomic doesn't do MT right now, so contextId == threadId
    ifetch_req.setThreadContext(_cpuId, 0); // Add thread ID if we add MT
    data_read_req.setThreadContext(_cpuId, 0); // Add thread ID here too
    data_write_req.setThreadContext(_cpuId, 0); // Add thread ID here too

    initPipelineMacho ();
}

const std::string HybridCPU::PipelineStateStr[] =
{
    "FaultState",
    "R_Idle",
    "R_Run",
    "R_Flush",
    "R_InstWait",
    "R_Advance",

    "V_Idle",
    "R_2_R",
    "R_2_V",
    "V_2_R",
    "V_2_V",
};

HybridCPU::HybridCPU(HybridCPUParams *p)
    : BaseSimpleCPU(p), tickEvent(this), width(p->width), locked(false),
      simulate_data_stalls(p->simulate_data_stalls),
      simulate_inst_stalls(p->simulate_inst_stalls),
      drain_manager(NULL),
      icachePort(name() + ".icache_port", this),
      dcachePort(name() + ".dcache_port", this),
      fastmem(p->fastmem),
      simpoint(p->simpoint_profile),
      intervalSize(p->simpoint_interval),
      intervalCount(0),
      intervalDrift(0),
      simpointStream(NULL),
      currentBBV(0, 0),
      currentBBVInstCount(0),
      pipelineMacho (R_Idle),
      cycle (0),
      numIssued (0), IssueWidth (p->IssueWidth),
      xRegDepTable (), yRegDepTable (), gRegDepTable (), mRegDepTable (),
      IntArithLatency     (p->IntArithLatency),
      IntLogicLatency     (p->IntLogicLatency),
      IntTestLatency      (p->IntTestLatency),
      IntShiftLatency     (p->IntShiftLatency),
      IntBitLatency       (p->IntBitLatency),
      IntMoveLatency      (p->IntMoveLatency),
      IntMulLatency       (p->IntMulLatency),
      IntMacLatency       (p->IntMacLatency),
      IntIterLatency      (p->IntIterLatency),
      IntMemLatency       (p->IntMemLatency),
      IntMemAddrLatency   (p->IntMemAddrLatency),
      IntMiscLatency      (p->IntMiscLatency),
      SimdIntArithLatency (p->SimdIntArithLatency),
      SimdIntLogicLatency (p->SimdIntLogicLatency),
      SimdIntTestLatency  (p->SimdIntTestLatency),
      SimdIntShiftLatency (p->SimdIntShiftLatency),
      SimdIntMulLatency   (p->SimdIntMulLatency),
      SimdIntMacLatency   (p->SimdIntMacLatency),
      SimdIntIterLatency  (p->SimdIntIterLatency),
      BranchDelaySlot     (p->BranchDelaySlot),
      IntDivStall         (p->IntDivStall),
      IntRemStall         (p->IntRemStall),
      IntMemPrededLatency (p->IntMemPrededLatency)
{
    _status = Idle;

    if (simpoint) {
        simpointStream = simout.create(p->simpoint_profile_file, false);
    }
}


HybridCPU::~HybridCPU()
{
    if (tickEvent.scheduled()) {
        deschedule(tickEvent);
    }
    if (simpointStream) {
        simout.close(simpointStream);
    }
}

unsigned int
HybridCPU::drain(DrainManager *dm)
{
    assert(!drain_manager);
    if (switchedOut())
        return 0;

    if (!isDrained()) {
        DPRINTF(Drain, "Requesting drain: %s\n", pcState());
        drain_manager = dm;
        return 1;
    } else {
        if (tickEvent.scheduled())
            deschedule(tickEvent);

        DPRINTF(Drain, "Not executing microcode, no need to drain.\n");
        return 0;
    }
}

void
HybridCPU::drainResume()
{
    assert(!tickEvent.scheduled());
    assert(!drain_manager);
    if (switchedOut())
        return;

    DPRINTF(HybridCPU, "Resume\n");
    verifyMemoryMode();

    assert(!threadContexts.empty());
    if (threadContexts.size() > 1)
        fatal("The hybrid CPU only supports one thread.\n");

    if (thread->status() == ThreadContext::Active) {
        schedule(tickEvent, nextCycle());
        _status = BaseSimpleCPU::Running;
    } else {
        _status = BaseSimpleCPU::Idle;
    }

    system->totalNumInsts = 0;
}

bool
HybridCPU::tryCompleteDrain()
{
    if (!drain_manager)
        return false;

    DPRINTF(Drain, "tryCompleteDrain: %s\n", pcState());
    if (!isDrained())
        return false;

    DPRINTF(Drain, "CPU done draining, processing drain event\n");
    drain_manager->signalDrainDone();
    drain_manager = NULL;

    return true;
}


void
HybridCPU::switchOut()
{
    BaseSimpleCPU::switchOut();

    assert(!tickEvent.scheduled());
    assert(_status == BaseSimpleCPU::Running || _status == Idle);
    assert(isDrained());
}


void
HybridCPU::takeOverFrom(BaseCPU *oldCPU)
{
    BaseSimpleCPU::takeOverFrom(oldCPU);

    // The tick event should have been descheduled by drain()
    assert(!tickEvent.scheduled());

    ifetch_req.setThreadContext(_cpuId, 0); // Add thread ID if we add MT
    data_read_req.setThreadContext(_cpuId, 0); // Add thread ID here too
    data_write_req.setThreadContext(_cpuId, 0); // Add thread ID here too
}

void
HybridCPU::verifyMemoryMode() const
{
    if (!system->isAtomicMode()) {
        fatal("The hybrid CPU requires the memory system to be in "
              "'atomic' mode.\n");
    }
}

void
HybridCPU::activateContext(ThreadID thread_num, Cycles delay)
{
    DPRINTF(HybridCPU, "ActivateContext %d (%d cycles)\n", thread_num, delay);

    assert(thread_num == 0);
    assert(thread);

    assert(_status == Idle);
    assert(!tickEvent.scheduled());

    notIdleFraction++;
    numCycles += ticksToCycles(thread->lastActivate - thread->lastSuspend);

    //Make sure ticks are still on multiples of cycles
    schedule(tickEvent, clockEdge(delay));
    _status = BaseSimpleCPU::Running;
}


void
HybridCPU::suspendContext(ThreadID thread_num)
{
    DPRINTF(HybridCPU, "SuspendContext %d\n", thread_num);

    assert(thread_num == 0);
    assert(thread);

    if (_status == Idle)
        return;

    assert(_status == BaseSimpleCPU::Running);

    // tick event may not be scheduled if this gets called from inside
    // an instruction's execution, e.g. "quiesce"
    if (tickEvent.scheduled())
        deschedule(tickEvent);

    notIdleFraction--;
    _status = Idle;
}


Fault
HybridCPU::readMem(Addr addr, uint8_t * data,
                         unsigned size, unsigned flags)
{
    // use the CPU's statically allocated read request and packet objects
    Request *req = &data_read_req;

    if (traceData) {
        traceData->setAddr(addr);
    }

    //The block size of our peer.
    unsigned blockSize = dcachePort.peerBlockSize();
    //The size of the data we're trying to read.
    int fullSize = size;

    //The address of the second part of this access if it needs to be split
    //across a cache line boundary.
    Addr secondAddr = roundDown(addr + size - 1, blockSize);

    if (secondAddr > addr)
        size = secondAddr - addr;

    dcache_latency = 0;

    while (1) {
        req->setVirt(0, addr, size, flags, dataMasterId(), thread->pcState().instAddr());

        // translate to physical address
        Fault fault = thread->dtb->translateAtomic(req, tc, BaseTLB::Read);

        // Now do the access.
        if (fault == NoFault && !req->getFlags().isSet(Request::NO_ACCESS)) {
            Packet pkt = Packet(req,
                                req->isLLSC() ? MemCmd::LoadLockedReq :
                                MemCmd::ReadReq);
            pkt.dataStatic(data);

            if (req->isMmappedIpr())
                dcache_latency += TheISA::handleIprRead(thread->getTC(), &pkt);
            else {
                if (fastmem && system->isMemAddr(pkt.getAddr()))
                    system->getPhysMem().access(&pkt);
                else
                    dcache_latency += dcachePort.sendAtomic(&pkt);
            }
            dcache_access = true;

            assert(!pkt.isError());

            if (req->isLLSC()) {
                TheISA::handleLockedRead(thread, req);
            }
        }

        //If there's a fault, return it
        if (fault != NoFault) {
            if (req->isPrefetch()) {
                return NoFault;
            } else {
                return fault;
            }
        }

        //If we don't need to access a second cache line, stop now.
        if (secondAddr <= addr)
        {
            if (req->isLocked() && fault == NoFault) {
                assert(!locked);
                locked = true;
            }
            return fault;
        }

        /*
         * Set up for accessing the second cache line.
         */

        //Move the pointer we're reading into to the correct location.
        data += size;
        //Adjust the size to get the remaining bytes.
        size = addr + fullSize - secondAddr;
        //And access the right address.
        addr = secondAddr;
    }
}


Fault
HybridCPU::writeMem(uint8_t *data, unsigned size,
                          Addr addr, unsigned flags, uint64_t *res)
{
    // use the CPU's statically allocated write request and packet objects
    Request *req = &data_write_req;

    if (traceData) {
        traceData->setAddr(addr);
    }

    //The block size of our peer.
    unsigned blockSize = dcachePort.peerBlockSize();
    //The size of the data we're trying to read.
    int fullSize = size;

    //The address of the second part of this access if it needs to be split
    //across a cache line boundary.
    Addr secondAddr = roundDown(addr + size - 1, blockSize);

    if(secondAddr > addr)
        size = secondAddr - addr;

    dcache_latency = 0;

    while(1) {
        req->setVirt(0, addr, size, flags, dataMasterId(), thread->pcState().instAddr());

        // translate to physical address
        Fault fault = thread->dtb->translateAtomic(req, tc, BaseTLB::Write);

        // Now do the access.
        if (fault == NoFault) {
            MemCmd cmd = MemCmd::WriteReq; // default
            bool do_access = true;  // flag to suppress cache access

            if (req->isLLSC()) {
                cmd = MemCmd::StoreCondReq;
                do_access = TheISA::handleLockedWrite(thread, req);
            } else if (req->isSwap()) {
                cmd = MemCmd::SwapReq;
                if (req->isCondSwap()) {
                    assert(res);
                    req->setExtraData(*res);
                }
            }

            if (do_access && !req->getFlags().isSet(Request::NO_ACCESS)) {
                Packet pkt = Packet(req, cmd);
                pkt.dataStatic(data);

                if (req->isMmappedIpr()) {
                    dcache_latency +=
                        TheISA::handleIprWrite(thread->getTC(), &pkt);
                } else {
                    if (fastmem && system->isMemAddr(pkt.getAddr()))
                        system->getPhysMem().access(&pkt);
                    else
                        dcache_latency += dcachePort.sendAtomic(&pkt);
                }
                dcache_access = true;
                assert(!pkt.isError());

                if (req->isSwap()) {
                    assert(res);
                    memcpy(res, pkt.getPtr<uint8_t>(), fullSize);
                }
            }

            if (res && !req->isSwap()) {
                *res = req->getExtraData();
            }
        }

        //If there's a fault or we don't need to access a second cache line,
        //stop now.
        if (fault != NoFault || secondAddr <= addr)
        {
            if (req->isLocked() && fault == NoFault) {
                assert(locked);
                locked = false;
            }
            if (fault != NoFault && req->isPrefetch()) {
                return NoFault;
            } else {
                return fault;
            }
        }

        /*
         * Set up for accessing the second cache line.
         */

        //Move the pointer we're reading into to the correct location.
        data += size;
        //Adjust the size to get the remaining bytes.
        size = addr + fullSize - secondAddr;
        //And access the right address.
        addr = secondAddr;
    }
}


void
HybridCPU::tick()
{
#if DEBUG
    std::cout << std::endl;
    std::cout << "********************** CYCLE "
              << cycle
              << " **********************" << std::endl;
#endif

    DPRINTF(HybridCPU, "Tick\n");

    Tick latency = 0;

    for (int i = 0; i < width || locked; ++i) {
        numCycles++;

        if (!curStaticInst || !curStaticInst->isDelayedCommit())
            checkForInterrupts();

        checkPcEventQueue();
        // We must have just got suspended by a PC event
        if (_status == Idle) {
            tryCompleteDrain();
            return;
        }

        Fault fault = NoFault;

        /************************ Enter LILY2 region **********************/

        beginTick ();

        while (curPipelineState == R_Idle || curPipelineState == R_Run) {
            // Fetches the instruction.
            fetch ();

            // Decodes the instruction.
            decode ();

            // Dispatches the instruction.
            dispatch ();

            // Updates the dispatch infos.
            preExecute ();

            // Executes the instruction.
            execute ();

            // Do statistics and updates the pipeline state.
            postExecute ();
        }

        // Sets the pipeline state callbacks.
        setCallback ();

        // Calls the pre-update corresponding callbacks.
        callPrePipelineCallback ();

        // Renews the pipeline.
        renew ();

        // Calls the post-update corresponding callbacks.
        callPostPipelineCallback ();

        endTick ();

        /*
        bool needToFetch = !isRomMicroPC(pcState.microPC()) &&
                           !curMacroStaticInst;
        if (needToFetch) {
            setupFetchRequest(&ifetch_req);
            fault = thread->itb->translateAtomic(&ifetch_req, tc,
                                                 BaseTLB::Execute);
         }

        if (fault == NoFault) {
            Tick icache_latency = 0;
            bool icache_access = false;
            dcache_access = false; // assume no dcache access

            if (needToFetch) {
                // This is commented out because the decoder would act like
                // a tiny cache otherwise. It wouldn't be flushed when needed
                // like the I cache. It should be flushed, and when that works
                // this code should be uncommented.
                //Fetch more instruction memory if necessary
                //if(decoder.needMoreBytes())
                //{
                    icache_access = true;
                    Packet ifetch_pkt = Packet(&ifetch_req, MemCmd::ReadReq);
                    ifetch_pkt.dataStatic(&inst);

                    if (fastmem && system->isMemAddr(ifetch_pkt.getAddr()))
                        system->getPhysMem().access(&ifetch_pkt);
                    else
                        icache_latency = icachePort.sendAtomic(&ifetch_pkt);

                    assert(!ifetch_pkt.isError());
                    //return;
                    // ifetch_req is initialized to read the instruction directly
                    // into the CPU object's inst field.
                //}
            }

            preExecute();

            if (curStaticInst) {
                fault = curStaticInst->execute(this, traceData);

                // keep an instruction count
                if (fault == NoFault)
                    countInst();
                else if (traceData && !DTRACE(ExecFaulting)) {
                    delete traceData;
                    traceData = NULL;
                }

                postExecute();
            }

            // @todo remove me after debugging with legion done
            if (curStaticInst && (!curStaticInst->isMicroop() ||
                        curStaticInst->isFirstMicroop()))
                instCnt++;

            // profile for SimPoints if enabled and macro inst is finished
            if (simpoint && curStaticInst && (fault == NoFault) &&
                    (!curStaticInst->isMicroop() ||
                     curStaticInst->isLastMicroop())) {
                profileSimPoint();
            }

            Tick stall_ticks = 0;
            if (simulate_inst_stalls && icache_access)
                stall_ticks += icache_latency;

            if (simulate_data_stalls && dcache_access)
                stall_ticks += dcache_latency;

            if (stall_ticks) {
                // the hybrid cpu does its accounting in ticks, so
                // keep counting in ticks but round to the clock
                // period
                latency += divCeil(stall_ticks, clockPeriod()) *
                    clockPeriod();
            }

        }
        if(fault != NoFault || !stayAtPC)
            advancePC(fault);
        */

    }

    if (tryCompleteDrain())
        return;

    // instruction takes at least one cycle
    if (latency < clockPeriod())
        latency = clockPeriod();

    if (_status != Idle)
        schedule(tickEvent, curTick() + latency);
}

void
HybridCPU::beginTick (void)
{
    // Gets the current pipeline state.
    curPipelineState = pipelineMacho.getCurState ();

    // Sets the right pc address if a branch is taken.
    TheISA::PCState pcState = thread->pcState ();
    if (pcState.bpc () != 0) {
        pcState.set (pcState.bpc ());
        thread->pcState (pcState);
    }
}

void
HybridCPU::endTick ()
{
    // Sets the current pipeline state to IDLE.
    curPipelineState = pipelineMacho.transfer (EndTick);
}

void
HybridCPU::setupFetchRequest(Request *req)
{
    Addr instAddr = thread->instAddr();
    Addr PCMask = ~(sizeof (MachInst) - 1);
    Addr fetchPC = instAddr & PCMask;

    req->setVirt
        (0, fetchPC, sizeof(MachInst), Request::INST_FETCH, instMasterId(), instAddr);

#if DEBUG
    std::cout << "fetchPC = " << IO_ADDR << instAddr << std::endl;
    std::cout << "aligned fetchPC = " << IO_ADDR << fetchPC << std::endl;
#endif
}

Cycles
HybridCPU::fetch (void)
{
#if DEBUG
    std::cout << "<---------- fetch" << std::endl;
#endif

    setupFetchRequest (&ifetch_req);

    thread->itb->translateAtomic(&ifetch_req, tc, BaseTLB::Execute);

    Packet ifetch_pkt = Packet(&ifetch_req, MemCmd::ReadReq);
    ifetch_pkt.dataStatic(&inst);

    /* Do the actual fetch here. */
    icachePort.sendAtomic(&ifetch_pkt);
    inst = gtobe (inst);

#if DEBUG
    std::cout << "fetch instruction = " << IO_MACHINST << inst << std::endl;
    std::cout << "----------> fetch" << std::endl << std::endl;
#endif

    /* TODO */
    return Cycles (0);
}

void
HybridCPU::decode (void)
{
#if DEBUG
    std::cout << "<---------- decode" << std::endl;
#endif

    curStaticInst = NULL;
    TheISA::Decoder *decoder = &(thread->decoder);

    StaticInstPtr tmpStaticInst = decoder->decodeInst (inst, this);
    curStaticInst = dynamic_cast<Lily2StaticInst *> (tmpStaticInst.get ());

#if DEBUG
    std::cout << "disassemble: "
              << curStaticInst->disassemble (thread->instAddr ()) << std::endl;
    std::cout << "----------> decode" << std::endl << std::endl;
#endif
}

void
HybridCPU::dispatch (void)
{
    rDispatch ();
}

void
HybridCPU::rDispatch (void)
{
#if DEBUG
    std::cout << "<---------- rDispatch" << std::endl;
#endif

    // Checks the following things in Risc dispatch.
    // 1. Issue width.
    // 2. Register dependences.
    // 3. Memory references after store.

    // Checks the issue width.
    bool rIsOverIssueWidth = (getNumIssued () >= getIssueWidth ());

    bool rIsRegDep = isRegDep ();

    // Checks the memory hazard.
    bool rIsMemHazard = (getNumStore () > 0) && (curStaticInst->isMemRef ());

    bool rIsNoIssue = rIsOverIssueWidth || rIsRegDep || rIsMemHazard;

    if (rIsNoIssue) {
        // Sets the CURSTATICINST null if dispatch is failure.
        curStaticInst = NULL;
    }

#if DEBUG
    if (!rIsNoIssue) {
        std::cout << "dispatch success." << std::endl;
    } else {
        std::cout << "dispatch failure." << std::endl;
        if (rIsOverIssueWidth) {
            std::cout << "failure reason: issue width." << std::endl;
        }
        if (rIsRegDep) {
            std::cout << "failure reason: register dependence." << std::endl;
        }
    }
    std::cout << "----------> rDispatch" << std::endl << std::endl;
#endif
}

void
HybridCPU::vDispatch (void)
{
#if DEBUG
    std::cout << "<---------- vDispatch" << std::endl;
#endif

    // Checks the following things in Vliw dispatch.
    // 1. Functional unit ascending order.

}

void
HybridCPU::execute (void)
{
#if DEBUG
    std::cout << "<---------- execute" << std::endl;
#endif

    if (curStaticInst) {
        curStaticInst->execute (this, traceData);
    }

#if DEBUG
    if (curStaticInst) {
        std::cout << "operation: " << curStaticInst->operate () << std::endl;
    } else {
        std::cout << "(nil)" << std::endl;
    }
    std::cout << "----------> execute" << std::endl << std::endl;
#endif
}

void
HybridCPU::preExecute (void)
{
    switch (dispModeFactory (curPipelineState)) {
        case TheISA::DISPMODE_RISC: rPreExecute (); break;
        case TheISA::DISPMODE_VLIW: vPreExecute (); break;
        default: assert (0);
    }
}

void
HybridCPU::rPreExecute (void)
{
#if DEBUG
    std::cout << "<---------- rPreExecute" << std::endl;
#endif

    if (curStaticInst) {
        // Increase the number of issued instructions.
        setNumIssued (getNumIssued () + 1);

        // Increase the number of store instructions per issue.
        if (curStaticInst->isStore ()) {
            setNumStore (getNumStore () + 1);
        }

        // Updates the register dependence table.
        insertRegDep ();
    }

#if DEBUG
    std::cout << "issued insts: " << getNumIssued () << std::endl;
    std::cout << "x register dependence table:" << std::endl;
    std::cout << xRegDepTable << std::endl;
    std::cout << "g register dependence table:" << std::endl;
    std::cout << gRegDepTable << std::endl;
    std::cout << "----------> rPreExecute" << std::endl << std::endl;
#endif
}

void
HybridCPU::vPreExecute (void)
{
}

void
HybridCPU::postExecute (void)
{
#if DEBUG
    std::cout << "<---------- postExecute" << std::endl;
#endif

    if (!curStaticInst) {
        curPipelineEvent = NoIssue;
    } else {
        if (curStaticInst->isIter ()) {
            // Iterative inst.
            curPipelineEvent = IterInst;
        } else if (curStaticInst->isControl ()) {
            // Flow control inst.
            TheISA::PCState pcState = thread->pcState ();

            if (bPredictor.predict (pcState.pc ()) == pcState.bpc ()) {
                // Branch prediction is right.
                curPipelineEvent = BPreded;
            } else {
                // Branch prediction is wrong.
                curPipelineEvent = MisBPred;
            }

            // Feeds back to the branch predictor.
            bPredictor.feedback (pcState.pc (), pcState.bpc ());

        } else if (curStaticInst->isMemRef ()) {
            // Memory reference inst.
            TheISA::PCState pcState = thread->pcState ();

            if (curStaticInst->isLoad ()) {
                // Load instruction.
                if (vPredictor.predict (pcState.pc ()) == pcState.lpc ()) {
                    // Value prediction is right.
                    curPipelineEvent = VPreded;

                    // Mutates the register back cycle in register dependence
                    // table and register file buffer.
                    for (OpCount_t i = 0; i != curStaticInst->getNumDestOps (); ++i) {
                        Op_t *op = curStaticInst->getDestOp (i);
                        if (op->memFlag ()) {
                            Cycles newLoadLatency (IntMemPrededLatency);
                            mutateRegDep (*op, newLoadLatency);
                            setRegBufCycle (*op, newLoadLatency);
                        }
                    }

                } else {
                    // Value prediction is wrong.
                    curPipelineEvent = MisVPred;
                }

                // Feeds back to the value predictor.
                vPredictor.feedback (pcState.pc (), pcState.lpc ());
                vPredictor.feedbackLoadHistory (pcState.pc (), pcState.lpc ());

            } else if (curStaticInst->isLoadD ()) {
                // Load instruction with delay slot.
                curPipelineEvent = Issue;
                // Feeds back to the value predictor.
                vPredictor.feedbackLoadHistory (pcState.pc (), pcState.lpc ());

            } else if (curStaticInst->isStore ()) {
                // Store instruction.
                curPipelineEvent = Issue;
                // Feeds back to the value predictor.
                vPredictor.feedbackStoreHistory (pcState.pc (), pcState.spc ());

            } else {
                assert (0);
            }

        } else if (curStaticInst->isModeSwitch ()) {
            // Mode switch inst.
            curPipelineEvent = (curStaticInst->isToRisc ()) ? ToRiscInst : ToVliwInst;
        } else {
            // Default.
            curPipelineEvent = Issue;
        }
    }

    curPipelineState = pipelineMacho.transfer (curPipelineEvent);

    // Advance the pc if dispatch is success.
    if (curStaticInst) {
        TheISA::PCState pcState = thread->pcState ();
        pcState.advance ();
        thread->pcState (pcState);
    }

#if DEBUG
    debugPipeline (std::cout);
    std::cout << "----------> postExecute" << std::endl << std::endl;
#endif
}


void
HybridCPU::renew (void)
{
#if DEBUG
    std::cout << "<---------- renew" << std::endl;
    std::cout << "x register file:" << std::endl;
    std::cout << *(thread->getXRegs ()) << std::endl;
#endif

    Cycles decrRegBackCycleDelta (1);

    // Clear the number of issued instructions.
    setNumIssued (0);

    // Clear the number of store instructions.
    setNumStore (0);

    refreshRegDepTable (decrRegBackCycleDelta);

    // Writes the register file buffers back to register files.
    refreshRegs (decrRegBackCycleDelta);

    // Increase the cycles.
    setCycle (getCycle () + decrRegBackCycleDelta);

#if DEBUG
    std::cout << "x register dependence table:" << std::endl;
    std::cout << xRegDepTable << std::endl;
    std::cout << "----------> renew" << std::endl << std::endl;
#endif
}

void
HybridCPU::mutateRegDep (const Op_t& op, const Cycles& regBackCycle)
{
    RegFile_t fileName = op.regFile ();
    RegIndex_t regIndex = op.regIndex ();

    switch (op.numRegs ()) {
        case 1:
            mutateRegDep (fileName, regIndex    , regBackCycle);
            break;

        case 2:
            mutateRegDep (fileName, regIndex    , regBackCycle);
            mutateRegDep (fileName, regIndex + 1, regBackCycle);
            break;

        case 4:
            mutateRegDep (fileName, regIndex    , regBackCycle);
            mutateRegDep (fileName, regIndex + 1, regBackCycle);
            mutateRegDep (fileName, regIndex + 2, regBackCycle);
            mutateRegDep (fileName, regIndex + 3, regBackCycle);
            break;

        default:
            assert (0);
    }
}

void
HybridCPU::mutateRegDep (const RegFile_t& fileName,
                         const RegIndex_t& regIndex, const Cycles& cycle)
{
    switch (fileName) {
        case TheISA::REG_X:
            mutateRegDep (xRegDepTable, regIndex, cycle); break;

        case TheISA::REG_Y:
            mutateRegDep (yRegDepTable, regIndex, cycle); break;

        case TheISA::REG_G:
            mutateRegDep (gRegDepTable, regIndex, cycle); break;

        case TheISA::REG_M:
            mutateRegDep (mRegDepTable, regIndex, cycle); break;

        default:
            assert (0);
    }
}

bool
HybridCPU::isRegDepTableEmpty (void) const
{
    return isRegDepTableEmpty (xRegDepTable) &&
           isRegDepTableEmpty (yRegDepTable) &&
           isRegDepTableEmpty (gRegDepTable) &&
           isRegDepTableEmpty (mRegDepTable);
}

bool
HybridCPU::isRegDep (void) const
{
   // Special instruction dealing.
   if (curStaticInst->isSyscall ()) {
        if (isRegDepTableEmpty ()) {
            return false;
        } else {
            return true;
        }
    }

    // Source operands.
    for (OpCount_t i = 0; i != curStaticInst->getNumSrcOps (); ++i) {
        if (isRegDep (*(curStaticInst->getSrcOp (i)))) {
            return true;
        }
    }

    // Destination operands.
    for (OpCount_t i = 0; i != curStaticInst->getNumDestOps (); ++i) {
        if (isRegDep (*(curStaticInst->getDestOp (i)))) {
            return true;
        }
    }

    return false;
}

bool
HybridCPU::isRegDep (const Op_t& op) const
{
    if (op.immFlag ()) {
        return false;
    } else {

        RegFile_t fileName = op.regFile ();
        RegIndex_t regIndex = op.regIndex ();

        switch (op.numRegs ()) {
            case 1:
                return isRegDep (fileName, regIndex    );

            case 2:
                return isRegDep (fileName, regIndex    ) ||
                       isRegDep (fileName, regIndex + 1);

            case 4:
                return isRegDep (fileName, regIndex    ) ||
                       isRegDep (fileName, regIndex + 1) ||
                       isRegDep (fileName, regIndex + 2) ||
                       isRegDep (fileName, regIndex + 3);

            default:
                assert (0);
        }
    }
}

bool
HybridCPU::isRegDep (const RegFile_t& fileName, const RegIndex_t& regIndex) const
{
    switch (fileName) {
        case TheISA::REG_X:
            return isRegDep (xRegDepTable, regIndex);

        case TheISA::REG_Y:
            return isRegDep (yRegDepTable, regIndex);

        case TheISA::REG_G:
            return isRegDep (gRegDepTable, regIndex);

        case TheISA::REG_M:
            return isRegDep (mRegDepTable, regIndex);

        default:
            assert (0);
    }
}

void
HybridCPU::insertRegDep (void)
{
    for (OpCount_t i = 0; i != curStaticInst->getNumDestOps (); ++i) {
        insertRegDep (*(curStaticInst->getDestOp (i)));
    }
}

void
HybridCPU::insertRegDep (const Op_t& op)
{
    if (op.immFlag ()) {
        assert (0);
    } else {

        RegFile_t fileName = op.regFile ();
        RegIndex_t regIndex = op.regIndex ();

        Cycles regBackCycle =
            funcUnitLatencyFactory (curStaticInst->opClass (), op.memFlag ());

        switch (op.numRegs ()) {
            case 1:
                insertRegDep (fileName, regIndex    , regBackCycle);
                break;

            case 2:
                insertRegDep (fileName, regIndex    , regBackCycle);
                insertRegDep (fileName, regIndex + 1, regBackCycle);
                break;

            case 4:
                insertRegDep (fileName, regIndex    , regBackCycle);
                insertRegDep (fileName, regIndex + 1, regBackCycle);
                insertRegDep (fileName, regIndex + 2, regBackCycle);
                insertRegDep (fileName, regIndex + 3, regBackCycle);
                break;

            default:
                assert (0);
        }
    }
}

void
HybridCPU::insertRegDep (const RegFile_t& fileName,
                      const RegIndex_t& regIndex, const Cycles& regBackCycle)
{
    switch (fileName) {
        case TheISA::REG_X:
            insertRegDep (xRegDepTable, regIndex, regBackCycle);
            break;

        case TheISA::REG_Y:
            insertRegDep (yRegDepTable, regIndex, regBackCycle);
            break;

        case TheISA::REG_G:
            insertRegDep (gRegDepTable, regIndex, regBackCycle);
            break;

        case TheISA::REG_M:
            insertRegDep (mRegDepTable, regIndex, regBackCycle);
            break;

        default:
            assert (0);
    }
}

Cycles
HybridCPU::maxRegDepCycle (void) const
{
    // X.
    Cycles xMax = xRegDepTable.maxCycle ();

    // Y.
    Cycles yMax = yRegDepTable.maxCycle ();

    // G.
    Cycles gMax = gRegDepTable.maxCycle ();

    // M.
    Cycles mMax = mRegDepTable.maxCycle ();

    // Maximum in X,Y,G,M.
    Cycles max;
    max = (xMax >= yMax) ? xMax : yMax;
    max = ( max >= gMax) ?  max : gMax;
    max = ( max >= mMax) ?  max : mMax;

    return max;
}

void
HybridCPU::debugPipeline (std::ostream& os) const
{
    std::string pipelineStateStr = PipelineStateStr[curPipelineState];
    os << "pipeline state = " << pipelineStateStr << std::endl;
}

void
HybridCPU::refreshRegDepTable (const Cycles& decrRegBackCycleDelta)
{
    // X.
    refreshRegDepTable (xRegDepTable, decrRegBackCycleDelta);

    // Y.
    refreshRegDepTable (yRegDepTable, decrRegBackCycleDelta);

    // G.
    refreshRegDepTable (gRegDepTable, decrRegBackCycleDelta);

    // M.
    refreshRegDepTable (mRegDepTable, decrRegBackCycleDelta);
}

void
HybridCPU::setRegBufCycle (const Op_t& op, const Cycles& regBackCycle)
{
    RegFile_t fileName = op.regFile ();
    RegIndex_t regIndex = op.regIndex ();

    switch (op.numRegs ()) {
        case 1:
            setRegBufCycle (fileName, regIndex    , regBackCycle);
            break;

        case 2:
            setRegBufCycle (fileName, regIndex    , regBackCycle);
            setRegBufCycle (fileName, regIndex + 1, regBackCycle);
            break;

        case 4:
            setRegBufCycle (fileName, regIndex    , regBackCycle);
            setRegBufCycle (fileName, regIndex + 1, regBackCycle);
            setRegBufCycle (fileName, regIndex + 2, regBackCycle);
            setRegBufCycle (fileName, regIndex + 3, regBackCycle);
            break;

        default:
            assert (0);
    }
}

void
HybridCPU::setRegBufCycle (const RegFile_t& fileName,
                           const RegIndex_t& regIndex,
                           const Cycles& regBackCycle)
{
    switch (fileName) {
        case TheISA::REG_X:
            setRegBufCycle (thread->getXRegFileBuf (), regIndex, regBackCycle);
            break;

        case TheISA::REG_Y:
            setRegBufCycle (thread->getYRegFileBuf (), regIndex, regBackCycle);
            break;

        case TheISA::REG_G:
            setRegBufCycle (thread->getGRegFileBuf (), regIndex, regBackCycle);
            break;

        case TheISA::REG_M:
            setRegBufCycle (thread->getMRegFileBuf (), regIndex, regBackCycle);
            break;

        default:
            assert (0);
    }
}

void
HybridCPU::refreshRegs (const Cycles& decrRegBackCycleDelta)
{
    // X.
    refreshRegs (thread->getXRegFile (), thread->getXRegFileBuf (),
            decrRegBackCycleDelta);

    // Y.
    refreshRegs (thread->getYRegFile (), thread->getYRegFileBuf (),
            decrRegBackCycleDelta);

    // G.
    refreshRegs (thread->getGRegFile (), thread->getGRegFileBuf (),
            decrRegBackCycleDelta);

    // M.
    refreshRegs (thread->getMRegFile (), thread->getMRegFileBuf (),
            decrRegBackCycleDelta);
}


void
HybridCPU::printAddr(Addr a)
{
    dcachePort.printAddr(a);
}

void
HybridCPU::profileSimPoint()
{
    if (!currentBBVInstCount)
        currentBBV.first = thread->pcState().instAddr();

    ++intervalCount;
    ++currentBBVInstCount;

    // If inst is control inst, assume end of basic block.
    if (curStaticInst->isControl()) {
        currentBBV.second = thread->pcState().instAddr();

        auto map_itr = bbMap.find(currentBBV);
        if (map_itr == bbMap.end()){
            // If a new (previously unseen) basic block is found,
            // add a new unique id, record num of insts and insert into bbMap.
            BBInfo info;
            info.id = bbMap.size() + 1;
            info.insts = currentBBVInstCount;
            info.count = currentBBVInstCount;
            bbMap.insert(std::make_pair(currentBBV, info));
        } else {
            // If basic block is seen before, just increment the count by the
            // number of insts in basic block.
            BBInfo& info = map_itr->second;
            assert(info.insts == currentBBVInstCount);
            info.count += currentBBVInstCount;
        }
        currentBBVInstCount = 0;

        // Reached end of interval if the sum of the current inst count
        // (intervalCount) and the excessive inst count from the previous
        // interval (intervalDrift) is greater than/equal to the interval size.
        if (intervalCount + intervalDrift >= intervalSize) {
            // summarize interval and display BBV info
            std::vector<pair<uint64_t, uint64_t> > counts;
            for (auto map_itr = bbMap.begin(); map_itr != bbMap.end();
                    ++map_itr) {
                BBInfo& info = map_itr->second;
                if (info.count != 0) {
                    counts.push_back(std::make_pair(info.id, info.count));
                    info.count = 0;
                }
            }
            std::sort(counts.begin(), counts.end());

            // Print output BBV info
            *simpointStream << "T";
            for (auto cnt_itr = counts.begin(); cnt_itr != counts.end();
                    ++cnt_itr) {
                *simpointStream << ":" << cnt_itr->first
                                << ":" << cnt_itr->second << " ";
            }
            *simpointStream << "\n";

            intervalDrift = (intervalCount + intervalDrift) - intervalSize;
            intervalCount = 0;
        }
    }
}

void
HybridCPU::initPipelineMacho (void)
{
    // Pipeline is always in idle state at the beginning of tick().
    // In Risc, the state transfer prototype is showed below:
    //     idle -> ... -> run -> ... -> advance
    // In Vliw, the state transfer prototype is showed below:

    // R_Idle.
    // The initial state when entering the tick() is R_Idle.
    // Tries to issue inst in R_Idle state.
    pipelineMacho.regStateEvent (R_Idle, Issue, R_Run);
    // Stays R_Idle if issue is failure.
    pipelineMacho.regStateEvent (R_Idle, NoIssue, R_Advance);
    // Turns to R_Advance or R_Flush if branch inst is met.
    pipelineMacho.regStateEvent (R_Idle, BPreded, R_Advance);
    pipelineMacho.regStateEvent (R_Idle, MisBPred, R_Flush);
    pipelineMacho.regStateEvent (R_Idle, VPreded, R_Run);
    pipelineMacho.regStateEvent (R_Idle, MisVPred, R_Run);
    // Turns to R_InstWait if iterative inst is met.
    pipelineMacho.regStateEvent (R_Idle, IterInst, R_InstWait);
    // Turns to R_2_R or R_2_V if mode switching inst is met.
    pipelineMacho.regStateEvent (R_Idle, ToRiscInst, R_2_R);
    pipelineMacho.regStateEvent (R_Idle, ToVliwInst, R_2_V);
    // Fault event.
    pipelineMacho.regLocalDefaultState (R_Idle, FaultState);

    // R_Run.
    // Tries to issue inst in R_Run state.
    pipelineMacho.regStateEvent (R_Run, Issue, R_Run);
    // Turns to R_Advance if issue is failure.
    pipelineMacho.regStateEvent (R_Run, NoIssue, R_Advance);
    // Turns to R_Advance or R_Flush if branch inst is met.
    pipelineMacho.regStateEvent (R_Run, BPreded, R_Advance);
    pipelineMacho.regStateEvent (R_Run, MisBPred, R_Flush);
    pipelineMacho.regStateEvent (R_Run, VPreded, R_Run);
    pipelineMacho.regStateEvent (R_Run, MisVPred, R_Run);
    // Turns to R_InstWait if iterative inst is met.
    pipelineMacho.regStateEvent (R_Run, IterInst, R_InstWait);
    // Turns to R_2_R or R_2_V if mode switching inst is met.
    pipelineMacho.regStateEvent (R_Run, ToRiscInst, R_2_R);
    pipelineMacho.regStateEvent (R_Run, ToVliwInst, R_2_V);
    // Fault event.
    pipelineMacho.regLocalDefaultState (R_Run, FaultState);

    // R_Flush.
    // Caused by branch mispredictions.
    // Break the issue iteration.
    pipelineMacho.regStateEvent (R_Flush, EndTick, R_Idle);
    pipelineMacho.regLocalDefaultState (R_Flush, FaultState);

    // R_InstWait.
    // Caused by iterative instructions.
    // Break the issue iteration.
    pipelineMacho.regLocalDefaultState (R_InstWait, R_Idle);

    // R_Advance.
    // Caused by normal dependencies.
    // Break the issue iteration.
    pipelineMacho.regStateEvent (R_Advance, EndTick, R_Idle);
    pipelineMacho.regLocalDefaultState (R_Advance, FaultState);

    // R_2_R.
    // Caused by SETR instruction.
    // Break the issue iteration.
    pipelineMacho.regLocalDefaultState (R_2_R, R_Idle);

    // R_2_V.
    // Caused by SETV instruction.
    // Break the issue iteration.
    pipelineMacho.regLocalDefaultState (R_2_V, V_Idle);
}

void
HybridCPU::callPrePipelineCallback (void)
{
    if (prePipelineCallback != NULL) {
        (this->*prePipelineCallback) ();
    }
}

void
HybridCPU::callPostPipelineCallback (void)
{
    if (postPipelineCallback != NULL) {
        (this->*postPipelineCallback) ();
    }
}

void
HybridCPU::setCallback (void)
{
    switch (curPipelineState) {
        case R_InstWait:
            prePipelineCallback = &HybridCPU::callback_R_InstWait;
            postPipelineCallback = NULL;
            break;
        case R_Flush:
            prePipelineCallback = NULL;
            postPipelineCallback = &HybridCPU::callback_R_Flush;
            break;
        case R_Advance:
            prePipelineCallback = NULL;
            postPipelineCallback = &HybridCPU::callback_R_Advance;
            break;
        case R_2_R:
            prePipelineCallback = NULL;
            postPipelineCallback = &HybridCPU::callback_R_2_R;
            break;
        case R_2_V:
            prePipelineCallback = NULL;
            postPipelineCallback = &HybridCPU::callback_R_2_V;
            break;
        default:
            assert (0);
    }
}

void
HybridCPU::callback_R_Idle (void)
{
    // Loops (1).
    // URDT.
    // URFB.
    // UC.
}

void
HybridCPU::callback_R_Run (void)
{
    // Loops (0).
}

void
HybridCPU::callback_R_Flush (void)
{
    Cycles flushCycle (BranchDelaySlot);
    refreshRegDepTable (flushCycle);
    refreshRegs (flushCycle);
    setCycle (getCycle () + flushCycle);
}

void
HybridCPU::callback_R_InstWait (void)
{
    Cycles stallCycle;

    if (curStaticInst->isIntDiv ()) {
        stallCycle = Cycles (IntDivStall);
    } else if (curStaticInst->isIntRem ()) {
        stallCycle = Cycles (IntRemStall);
    } else {
        assert (0);
    }

    setCycle (getCycle () + stallCycle);
}

void
HybridCPU::callback_R_Advance (void)
{
    // Loops (0).
}

void
HybridCPU::callback_R_2_R (void)
{
    return;
}

void
HybridCPU::callback_R_2_V (void)
{

    // Loops (mode switching cycles).
    // URDT.
    // URFB.
    // UC.
}

const Op32i_t&
HybridCPU::readOp32i (Lily2StaticInst *si, const OpCount_t& idx)
{
    Op32i_t *op;

    // Dynamic cast checking.
    assert (op = dynamic_cast<Op32i_t *> (si->getSrcOp (idx)));

    if (op->immFlag ()) {
        // The immediate value is already stored in OP.
    } else {
        uint32_t val;
        RegIndex_t regIndex = op->regIndex ();
        RegFile_t fileName = op->regFile ();

        switch (fileName) {
            case TheISA::REG_X:
                val = getRegValue (thread->getXRegFile (), regIndex);
                break;

            case TheISA::REG_Y:
                val = getRegValue (thread->getYRegFile (), regIndex);
                break;

            case TheISA::REG_G:
                val = getRegValue (thread->getGRegFile (), regIndex);
                break;

            case TheISA::REG_M:
                val = getRegValue (thread->getMRegFile (), regIndex);
                break;

            default:
                assert (0);
        }

        // Sets the operand value.
        op->setUval (val);
    }

    return *op;
}

const Op32f_t&
HybridCPU::readOp32f (Lily2StaticInst *si, const OpCount_t& idx)
{
    Op32f_t *op;

    // Dynamic cast checking.
    assert (op = dynamic_cast<Op32f_t *> (si->getSrcOp (idx)));

    if (op->immFlag ()) {
        // The immediate value is already stored in OP.
    } else {
        uint32_t val;
        RegIndex_t regIndex = op->regIndex ();
        RegFile_t fileName = op->regFile ();

        switch (fileName) {
            case TheISA::REG_X:
                val = getRegValue (thread->getXRegFile (), regIndex);
                break;

            case TheISA::REG_Y:
                val = getRegValue (thread->getYRegFile (), regIndex);
                break;

            case TheISA::REG_G:
                val = getRegValue (thread->getGRegFile (), regIndex);
                break;

            case TheISA::REG_M:
                val = getRegValue (thread->getMRegFile (), regIndex);
                break;

            default:
                assert (0);
        }

        // Sets the operand value.
        op->setBval (val);
    }

    return *op;
}

const Op64f_t&
HybridCPU::readOp64f (Lily2StaticInst *si, const OpCount_t& idx)
{
    Op64f_t *op;

    // Dynamic cast checking.
    assert (op = dynamic_cast<Op64f_t *> (si->getSrcOp (idx)));

    if (op->immFlag ()) {
        // The immediate value is already stored in OP.
    } else {
        uint64_t vlo, vhi;
        RegIndex_t regIndexLo = op->regIndex ();
        RegIndex_t regIndexHi = regIndexLo + 1;
        RegFile_t fileName = op->regFile ();

        switch (fileName) {
            case TheISA::REG_X:
                vlo = getRegValue (thread->getXRegFile (), regIndexLo);
                vhi = getRegValue (thread->getXRegFile (), regIndexHi);
                break;

            case TheISA::REG_Y:
                vlo = getRegValue (thread->getYRegFile (), regIndexLo);
                vhi = getRegValue (thread->getYRegFile (), regIndexHi);
                break;

            case TheISA::REG_G:
                vlo = getRegValue (thread->getGRegFile (), regIndexLo);
                vhi = getRegValue (thread->getGRegFile (), regIndexHi);
                break;

            case TheISA::REG_M:
                vlo = getRegValue (thread->getMRegFile (), regIndexLo);
                vhi = getRegValue (thread->getMRegFile (), regIndexHi);
                break;

            default:
                assert (0);
        }

        // Sets the operand value.
        op->setBval (vlo + (vhi << 32));
    }

    return *op;
}

const Opq8i_t&
HybridCPU::readOpq8i (Lily2StaticInst *si, const OpCount_t& idx)
{
    Opq8i_t *op;

    // Dynamic cast checking.
    assert (op = dynamic_cast<Opq8i_t *> (si->getSrcOp (idx)));

    if (op->immFlag ()) {
        // The immediate value is already stored in OP.
    } else {
        uint8_t vvl, vlo, vhi, vvh;
        uint32_t val;
        RegIndex_t regIndex = op->regIndex ();
        RegFile_t fileName = op->regFile ();

        switch (fileName) {
            case TheISA::REG_X:
                val = getRegValue (thread->getXRegFile (), regIndex);
                break;

            case TheISA::REG_Y:
                val = getRegValue (thread->getYRegFile (), regIndex);
                break;

            case TheISA::REG_G:
                val = getRegValue (thread->getGRegFile (), regIndex);
                break;

            case TheISA::REG_M:
                val = getRegValue (thread->getMRegFile (), regIndex);
                break;

            default:
                assert (0);
        }

        // Sets the operand value.
        vvl = val;
        vlo = val >> 8;
        vhi = val >> 16;
        vvh = val >> 24;
        op->setUvvl (vvl);
        op->setUvlo (vlo);
        op->setUvhi (vhi);
        op->setUvvh (vvh);
    }

    return *op;
}

const Opd16i_t&
HybridCPU::readOpd16i (Lily2StaticInst *si, const OpCount_t& idx)
{
    Opd16i_t *op;

    // Dynamic cast checking.
    assert (op = dynamic_cast<Opd16i_t *> (si->getSrcOp (idx)));

    if (op->immFlag ()) {
        // The immediate value is already stored in OP.
    } else {
        uint16_t vlo, vhi;
        uint32_t val;
        RegIndex_t regIndex = op->regIndex ();
        RegFile_t fileName = op->regFile ();

        switch (fileName) {
            case TheISA::REG_X:
                val = getRegValue (thread->getXRegFile (), regIndex);
                break;

            case TheISA::REG_Y:
                val = getRegValue (thread->getYRegFile (), regIndex);
                break;

            case TheISA::REG_G:
                val = getRegValue (thread->getGRegFile (), regIndex);
                break;

            case TheISA::REG_M:
                val = getRegValue (thread->getMRegFile (), regIndex);
                break;

            default:
                assert (0);
        }

        // Sets the operand value.
        vlo = val;
        vhi = val >> 16;
        op->setUvlo (vlo);
        op->setUvhi (vhi);
    }

    return *op;
}

const Opq16i_t&
HybridCPU::readOpq16i (Lily2StaticInst *si, const OpCount_t& idx)
{
    Opq16i_t *op;

    // Dynamic cast checking.
    assert (op = dynamic_cast<Opq16i_t *> (si->getSrcOp (idx)));

    if (op->immFlag ()) {
        // The immediate value is already stored in OP.
    } else {
        uint16_t vvl, vlo, vhi, vvh;
        uint32_t valLo, valHi;
        RegIndex_t regIndexLo = op->regIndex ();
        RegIndex_t regIndexHi = regIndexLo + 1;
        RegFile_t fileName = op->regFile ();

        switch (fileName) {
            case TheISA::REG_X:
                valLo = getRegValue (thread->getXRegFile (), regIndexLo);
                valHi = getRegValue (thread->getXRegFile (), regIndexHi);
                break;

            case TheISA::REG_Y:
                valLo = getRegValue (thread->getYRegFile (), regIndexLo);
                valHi = getRegValue (thread->getXRegFile (), regIndexHi);
                break;

            case TheISA::REG_G:
                valLo = getRegValue (thread->getGRegFile (), regIndexLo);
                valHi = getRegValue (thread->getXRegFile (), regIndexHi);
                break;

            case TheISA::REG_M:
                valLo = getRegValue (thread->getMRegFile (), regIndexLo);
                valHi = getRegValue (thread->getXRegFile (), regIndexHi);
                break;

            default:
                assert (0);
        }

        // Sets the operand value.
        vvl = valLo;
        vlo = valLo >> 16;
        vhi = valHi;
        vvh = valHi >> 16;
        op->setUvvl (vvl);
        op->setUvlo (vlo);
        op->setUvhi (vhi);
        op->setUvvh (vvh);
    }

    return *op;
}

const Opd32i_t&
HybridCPU::readOpd32i (Lily2StaticInst *si, const OpCount_t& idx)
{
    Opd32i_t *op;

    // Dynamic cast checking.
    assert (op = dynamic_cast<Opd32i_t *> (si->getSrcOp (idx)));

    if (op->immFlag ()) {
        // The immediate value is already stored in OP.
    } else {
        uint32_t vlo, vhi;

        RegIndex_t regIndexLo = op->regIndex ();
        RegIndex_t regIndexHi = regIndexLo + 1;

        RegFile_t fileName = op->regFile ();

        switch (fileName) {
            case TheISA::REG_X:
                vlo = getRegValue (thread->getXRegFile (), regIndexLo);
                vhi = getRegValue (thread->getXRegFile (), regIndexHi);
                break;

            case TheISA::REG_Y:
                vlo = getRegValue (thread->getYRegFile (), regIndexLo);
                vhi = getRegValue (thread->getYRegFile (), regIndexHi);
                break;

            case TheISA::REG_G:
                vlo = getRegValue (thread->getGRegFile (), regIndexLo);
                vhi = getRegValue (thread->getGRegFile (), regIndexHi);
                break;

            case TheISA::REG_M:
                vlo = getRegValue (thread->getMRegFile (), regIndexLo);
                vhi = getRegValue (thread->getMRegFile (), regIndexHi);
                break;

            default:
                assert (0);
        }

        // Sets the operand value.
        op->setUvlo (vlo);
        op->setUvhi (vhi);
    }

    return *op;
}

const Opd32f_t&
HybridCPU::readOpd32f (Lily2StaticInst *si, const OpCount_t& idx)
{
    Opd32f_t *op;

    // Dynamic cast checking.
    assert (op = dynamic_cast<Opd32f_t *> (si->getSrcOp (idx)));

    if (op->immFlag ()) {
        // The immediate value is already stored in OP.
    } else {
        uint32_t vlo, vhi;

        RegIndex_t regIndexLo = op->regIndex ();
        RegIndex_t regIndexHi = regIndexLo + 1;

        RegFile_t fileName = op->regFile ();

        switch (fileName) {
            case TheISA::REG_X:
                vlo = getRegValue (thread->getXRegFile (), regIndexLo);
                vhi = getRegValue (thread->getXRegFile (), regIndexHi);
                break;

            case TheISA::REG_Y:
                vlo = getRegValue (thread->getYRegFile (), regIndexLo);
                vhi = getRegValue (thread->getYRegFile (), regIndexHi);
                break;

            case TheISA::REG_G:
                vlo = getRegValue (thread->getGRegFile (), regIndexLo);
                vhi = getRegValue (thread->getGRegFile (), regIndexHi);
                break;

            case TheISA::REG_M:
                vlo = getRegValue (thread->getMRegFile (), regIndexLo);
                vhi = getRegValue (thread->getMRegFile (), regIndexHi);
                break;

            default:
                assert (0);
        }

        // Sets the operand value.
        op->setBvlo (vlo);
        op->setBvhi (vhi);
    }

    return *op;
}

void
HybridCPU::setOp32i (Lily2StaticInst *si, const OpCount_t& idx,
                     const Op32i_t& val, const Op32i_t& mask)
{
    Op32i_t *op;

    // Dynamic cast checking.
    assert (op = dynamic_cast<Op32i_t *> (si->getDestOp (idx)));

    if (op->immFlag ()) {
        // Destination operand can not be an immediate.
        assert (0);
    } else {
        RegIndex_t regIndex = op->regIndex ();
        uint32_t regValue = val.uval ();
        uint32_t regMask = mask.uval ();

        // Gets the functional unit latency.
        Cycles regBackCycle = funcUnitLatencyFactory (si->opClass (), op->memFlag ());

        RegFile_t fileName = op->regFile ();

        switch (fileName) {
            case TheISA::REG_X:
                setRegBufValue (thread->getXRegFileBuf (), regIndex,
                                regValue, regMask, regBackCycle);
                break;

            case TheISA::REG_Y:
                setRegBufValue (thread->getYRegFileBuf (), regIndex,
                                regValue, regMask, regBackCycle);
                break;

            case TheISA::REG_G:
                setRegBufValue (thread->getGRegFileBuf (), regIndex,
                                regValue, regMask, regBackCycle);
                break;

            case TheISA::REG_M:
                setRegBufValue (thread->getMRegFileBuf (), regIndex,
                                regValue, regMask, regBackCycle);
                break;

            default:
                assert (0);
        }

        // Sets the operand value.
        op->setUval (regValue);
    }

    return;
}

void
HybridCPU::setOp32f (Lily2StaticInst *si, const OpCount_t& idx,
                     const Op32f_t& val, const Op32f_t& mask)
{
    Op32f_t *op;

    // Dynamic cast checking.
    assert (op = dynamic_cast<Op32f_t *> (si->getDestOp (idx)));

    if (op->immFlag ()) {
        // Destination operand can not be an immediate.
        assert (0);
    } else {
        RegIndex_t regIndex = op->regIndex ();
        uint32_t regValue = val.bval ();
        uint32_t regMask = mask.bval ();

        // Gets the functional unit latency.
        Cycles regBackCycle = funcUnitLatencyFactory (si->opClass (), op->memFlag ());

        RegFile_t fileName = op->regFile ();

        switch (fileName) {
            case TheISA::REG_X:
                setRegBufValue (thread->getXRegFileBuf (), regIndex,
                                regValue, regMask, regBackCycle);
                break;

            case TheISA::REG_Y:
                setRegBufValue (thread->getYRegFileBuf (), regIndex,
                                regValue, regMask, regBackCycle);
                break;

            case TheISA::REG_G:
                setRegBufValue (thread->getGRegFileBuf (), regIndex,
                                regValue, regMask, regBackCycle);
                break;

            case TheISA::REG_M:
                setRegBufValue (thread->getMRegFileBuf (), regIndex,
                                regValue, regMask, regBackCycle);
                break;

            default:
                assert (0);
        }

        // Sets the operand value.
        op->setBval (regValue);
    }

    return;
}

void
HybridCPU::setOp64f (Lily2StaticInst *si, const OpCount_t& idx,
                     const Op64f_t& val, const Op64f_t& mask)
{
    Op64f_t *op;

    // Dynamic cast checking.
    assert (op = dynamic_cast<Op64f_t *> (si->getDestOp (idx)));

    if (op->immFlag ()) {
        // Destination operand can not be an immediate.
        assert (0);
    } else {
        RegIndex_t regIndexLo = op->regIndex ();
        RegIndex_t regIndexHi = regIndexLo + 1;
        uint64_t regValue = val.bval ();
        uint32_t regValueLo = bits (regValue, 31, 0);
        uint32_t regValueHi = bits (regValue, 63, 32);
        uint64_t regMask = mask.bval ();
        uint32_t regMaskLo = bits (regMask, 31, 0);
        uint32_t regMaskHi = bits (regMask, 63, 32);

        // Gets the functional unit latency.
        Cycles regBackCycle = funcUnitLatencyFactory (si->opClass (), op->memFlag ());

        RegFile_t fileName = op->regFile ();

        switch (fileName) {
            case TheISA::REG_X:
                setRegBufValue (thread->getXRegFileBuf (), regIndexLo,
                                regValueLo, regMaskLo, regBackCycle);
                setRegBufValue (thread->getXRegFileBuf (), regIndexHi,
                                regValueHi, regMaskHi, regBackCycle);
                break;

            case TheISA::REG_Y:
                setRegBufValue (thread->getYRegFileBuf (), regIndexLo,
                                regValueLo, regMaskLo, regBackCycle);
                setRegBufValue (thread->getYRegFileBuf (), regIndexHi,
                                regValueHi, regMaskHi, regBackCycle);
                break;

            case TheISA::REG_G:
                setRegBufValue (thread->getGRegFileBuf (), regIndexLo,
                                regValueLo, regMaskLo, regBackCycle);
                setRegBufValue (thread->getGRegFileBuf (), regIndexHi,
                                regValueHi, regMaskHi, regBackCycle);
                break;

            case TheISA::REG_M:
                setRegBufValue (thread->getMRegFileBuf (), regIndexLo,
                                regValueLo, regMaskLo, regBackCycle);
                setRegBufValue (thread->getMRegFileBuf (), regIndexHi,
                                regValueHi, regMaskHi, regBackCycle);
                break;

            default:
                assert (0);
        }

        // Sets the operand value.
        op->setBval ((uint64_t)regValueLo + (((uint64_t)regValueHi) << 32));
    }

    return;
}

void
HybridCPU::setOpq8i (Lily2StaticInst *si, const OpCount_t& idx,
                     const Opq8i_t& val, const Opq8i_t& mask)
{
    Opq8i_t *op;

    // Dynamic cast checking.
    assert (op = dynamic_cast<Opq8i_t *> (si->getDestOp (idx)));

    if (op->immFlag ()) {
        // Destination operand can not be an immediate.
        assert (0);
    } else {
        RegIndex_t regIndex = op->regIndex ();
        uint32_t regValue = 0;
        uint8_t regValueVl = val.uvvl ();
        uint8_t regValueLo = val.uvlo ();
        uint8_t regValueHi = val.uvhi ();
        uint8_t regValueVh = val.uvvh ();
        replaceBits (regValue, 7 , 0 , regValueVl);
        replaceBits (regValue, 15, 8 , regValueLo);
        replaceBits (regValue, 23, 16, regValueHi);
        replaceBits (regValue, 31, 24, regValueVh);
        uint32_t regMask = 0;
        uint8_t regMaskVl = mask.uvvl ();
        uint8_t regMaskLo = mask.uvlo ();
        uint8_t regMaskHi = mask.uvhi ();
        uint8_t regMaskVh = mask.uvvh ();
        replaceBits (regMask, 7 , 0 , regMaskVl);
        replaceBits (regMask, 15, 8 , regMaskLo);
        replaceBits (regMask, 23, 16, regMaskHi);
        replaceBits (regMask, 31, 24, regMaskVh);

        // Gets the functional unit latency.
        Cycles regBackCycle = funcUnitLatencyFactory (si->opClass (), op->memFlag ());

        RegFile_t fileName = op->regFile ();

        switch (fileName) {
            case TheISA::REG_X:
                setRegBufValue (thread->getXRegFileBuf (), regIndex,
                                regValue, regMask, regBackCycle);
                break;

            case TheISA::REG_Y:
                setRegBufValue (thread->getYRegFileBuf (), regIndex,
                                regValue, regMask, regBackCycle);
                break;

            case TheISA::REG_G:
                setRegBufValue (thread->getGRegFileBuf (), regIndex,
                                regValue, regMask, regBackCycle);
                break;

            case TheISA::REG_M:
                setRegBufValue (thread->getMRegFileBuf (), regIndex,
                                regValue, regMask, regBackCycle);
                break;

            default:
                assert (0);
        }

        // Sets the operand value.
        op->setUvvl (regValueVl);
        op->setUvlo (regValueLo);
        op->setUvhi (regValueHi);
        op->setUvvh (regValueVh);
    }

    return;
}

void
HybridCPU::setOpd16i (Lily2StaticInst *si, const OpCount_t& idx,
                      const Opd16i_t& val, const Opd16i_t& mask)
{
    Opd16i_t *op;

    // Dynamic cast checking.
    assert (op = dynamic_cast<Opd16i_t *> (si->getDestOp (idx)));

    if (op->immFlag ()) {
        // Destination operand can not be an immediate.
        assert (0);
    } else {
        RegIndex_t regIndex = op->regIndex ();
        uint32_t regValue = 0;
        uint16_t regValueLo = val.uvlo ();
        uint16_t regValueHi = val.uvhi ();
        replaceBits (regValue, 15, 0 , regValueLo);
        replaceBits (regValue, 31, 16, regValueHi);
        uint32_t regMask = 0;
        uint16_t regMaskLo = mask.uvlo ();
        uint16_t regMaskHi = mask.uvhi ();
        replaceBits (regMask, 15, 0 , regMaskLo);
        replaceBits (regMask, 31, 16, regMaskHi);

        // Gets the functional unit latency.
        Cycles regBackCycle = funcUnitLatencyFactory (si->opClass (), op->memFlag ());

        RegFile_t fileName = op->regFile ();

        switch (fileName) {
            case TheISA::REG_X:
                setRegBufValue (thread->getXRegFileBuf (), regIndex,
                                regValue, regMask, regBackCycle);
                break;

            case TheISA::REG_Y:
                setRegBufValue (thread->getYRegFileBuf (), regIndex,
                                regValue, regMask, regBackCycle);
                break;

            case TheISA::REG_G:
                setRegBufValue (thread->getGRegFileBuf (), regIndex,
                                regValue, regMask, regBackCycle);
                break;

            case TheISA::REG_M:
                setRegBufValue (thread->getMRegFileBuf (), regIndex,
                                regValue, regMask, regBackCycle);
                break;

            default:
                assert (0);
        }

        // Sets the operand value.
        op->setUvlo (regValueLo);
        op->setUvhi (regValueHi);
    }

    return;
}

void
HybridCPU::setOpq16i (Lily2StaticInst *si, const OpCount_t& idx,
                      const Opq16i_t& val, const Opq16i_t& mask)
{
    Opq16i_t *op;

    // Dynamic cast checking.
    assert (op = dynamic_cast<Opq16i_t *> (si->getDestOp (idx)));

    if (op->immFlag ()) {
        // Destination operand can not be an immediate.
        assert (0);
    } else {
        RegIndex_t regIndexLo = op->regIndex ();
        RegIndex_t regIndexHi = regIndexLo + 1;
        uint32_t regValue_0 = 0, regValue_1 = 0;
        uint16_t regValueVl = val.uvvl ();
        uint16_t regValueLo = val.uvlo ();
        uint16_t regValueHi = val.uvhi ();
        uint16_t regValueVh = val.uvvh ();
        replaceBits (regValue_0, 15, 0 , regValueVl);
        replaceBits (regValue_0, 31, 16, regValueLo);
        replaceBits (regValue_1, 15, 0 , regValueHi);
        replaceBits (regValue_1, 31, 16, regValueVh);
        uint32_t regMask_0 = 0, regMask_1 = 0;
        uint16_t regMaskVl = mask.uvvl ();
        uint16_t regMaskLo = mask.uvlo ();
        uint16_t regMaskHi = mask.uvhi ();
        uint16_t regMaskVh = mask.uvvh ();
        replaceBits (regMask_0, 15, 0 , regMaskVl);
        replaceBits (regMask_0, 31, 16, regMaskLo);
        replaceBits (regMask_1, 15, 0 , regMaskHi);
        replaceBits (regMask_1, 31, 16, regMaskVh);

        // Gets the functional unit latency.
        Cycles regBackCycle = funcUnitLatencyFactory (si->opClass (), op->memFlag ());

        RegFile_t fileName = op->regFile ();

        switch (fileName) {
            case TheISA::REG_X:
                setRegBufValue (thread->getXRegFileBuf (), regIndexLo,
                                regValue_0, regMask_0, regBackCycle);
                setRegBufValue (thread->getXRegFileBuf (), regIndexHi,
                                regValue_1, regMask_1, regBackCycle);
                break;

            case TheISA::REG_Y:

                setRegBufValue (thread->getYRegFileBuf (), regIndexLo,
                                regValue_0, regMask_0, regBackCycle);
                setRegBufValue (thread->getYRegFileBuf (), regIndexHi,
                                regValue_1, regMask_1, regBackCycle);
                break;

            case TheISA::REG_G:
                setRegBufValue (thread->getGRegFileBuf (), regIndexLo,
                                regValue_0, regMask_0, regBackCycle);
                setRegBufValue (thread->getGRegFileBuf (), regIndexHi,
                                regValue_1, regMask_1, regBackCycle);
                break;

            case TheISA::REG_M:
                setRegBufValue (thread->getMRegFileBuf (), regIndexLo,
                                regValue_0, regMask_0, regBackCycle);
                setRegBufValue (thread->getMRegFileBuf (), regIndexHi,
                                regValue_1, regMask_1, regBackCycle);
                break;

            default:
                assert (0);
        }

        // Sets the operand value.
        op->setUvvl (regValueVl);
        op->setUvlo (regValueLo);
        op->setUvhi (regValueHi);
        op->setUvvh (regValueVh);
    }

    return;
}

void
HybridCPU::setOpd32i (Lily2StaticInst *si, const OpCount_t& idx,
                      const Opd32i_t& val, const Opd32i_t& mask)
{
    Opd32i_t *op;

    // Dynamic cast checking.
    assert (op = dynamic_cast<Opd32i_t *> (si->getDestOp (idx)));

    if (op->immFlag ()) {
        // Destination operand can not be an immediate.
        assert (0);
    } else {
        RegIndex_t regIndexLo = op->regIndex ();
        RegIndex_t regIndexHi = regIndexLo + 1;
        uint32_t regValueLo = val.uvlo ();
        uint32_t regValueHi = val.uvhi ();
        uint32_t regMaskLo = mask.uvlo ();
        uint32_t regMaskHi = mask.uvhi ();

        // Gets the functional unit latency.
        Cycles regBackCycle = funcUnitLatencyFactory (si->opClass (), op->memFlag ());

        RegFile_t fileName = op->regFile ();

        switch (fileName) {
            case TheISA::REG_X:
                setRegBufValue (thread->getXRegFileBuf (), regIndexLo,
                                regValueLo, regMaskLo, regBackCycle);
                setRegBufValue (thread->getXRegFileBuf (), regIndexHi,
                                regValueHi, regMaskHi, regBackCycle);
                break;

            case TheISA::REG_Y:
                setRegBufValue (thread->getYRegFileBuf (), regIndexLo,
                                regValueLo, regMaskLo, regBackCycle);
                setRegBufValue (thread->getYRegFileBuf (), regIndexHi,
                                regValueHi, regMaskHi, regBackCycle);
                break;

            case TheISA::REG_G:
                setRegBufValue (thread->getGRegFileBuf (), regIndexLo,
                                regValueLo, regMaskLo, regBackCycle);
                setRegBufValue (thread->getGRegFileBuf (), regIndexHi,
                                regValueHi, regMaskHi, regBackCycle);
                break;

            case TheISA::REG_M:
                setRegBufValue (thread->getMRegFileBuf (), regIndexLo,
                                regValueLo, regMaskLo, regBackCycle);
                setRegBufValue (thread->getMRegFileBuf (), regIndexHi,
                                regValueHi, regMaskHi, regBackCycle);
                break;

            default:
                assert (0);
        }

        // Sets the operand value.
        op->setUvlo (regValueLo);
        op->setUvhi (regValueHi);
    }

    return;
}

void
HybridCPU::setOpd32f (Lily2StaticInst *si, const OpCount_t& idx,
                      const Opd32f_t& val, const Opd32f_t& mask)
{
    Opd32f_t *op;

    // Dynamic cast checking.
    assert (op = dynamic_cast<Opd32f_t *> (si->getDestOp (idx)));

    if (op->immFlag ()) {
        // Destination operand can not be an immediate.
        assert (0);
    } else {
        RegIndex_t regIndexLo = op->regIndex ();
        RegIndex_t regIndexHi = regIndexLo + 1;
        uint32_t regValueLo = val.bvlo ();
        uint32_t regValueHi = val.bvhi ();
        uint32_t regMaskLo = mask.bvlo ();
        uint32_t regMaskHi = mask.bvhi ();

        // Gets the functional unit latency.
        Cycles regBackCycle = funcUnitLatencyFactory (si->opClass (), op->memFlag ());

        RegFile_t fileName = op->regFile ();

        switch (fileName) {
            case TheISA::REG_X:
                setRegBufValue (thread->getXRegFileBuf (), regIndexLo,
                                regValueLo, regMaskLo, regBackCycle);
                setRegBufValue (thread->getXRegFileBuf (), regIndexHi,
                                regValueHi, regMaskHi, regBackCycle);
                break;

            case TheISA::REG_Y:
                setRegBufValue (thread->getYRegFileBuf (), regIndexLo,
                                regValueLo, regMaskLo, regBackCycle);
                setRegBufValue (thread->getYRegFileBuf (), regIndexHi,
                                regValueHi, regMaskHi, regBackCycle);
                break;

            case TheISA::REG_G:
                setRegBufValue (thread->getGRegFileBuf (), regIndexLo,
                                regValueLo, regMaskLo, regBackCycle);
                setRegBufValue (thread->getGRegFileBuf (), regIndexHi,
                                regValueHi, regMaskHi, regBackCycle);
                break;

            case TheISA::REG_M:
                setRegBufValue (thread->getMRegFileBuf (), regIndexLo,
                                regValueLo, regMaskLo, regBackCycle);
                setRegBufValue (thread->getMRegFileBuf (), regIndexHi,
                                regValueHi, regMaskHi, regBackCycle);
                break;

            default:
                assert (0);
        }

        // Sets the operand value.
        op->setBvlo (regValueLo);
        op->setBvhi (regValueHi);
    }

    return;
}


DispMode_t
HybridCPU::dispModeFactory (const PipelineState& pipelineState) const
{
    return TheISA::DISPMODE_RISC;
}

Cycles
HybridCPU::funcUnitLatencyFactory (const OpClass& opClass, bool memFlag) const
{
    switch (opClass) {
        case IntArithOp    : return Cycles (IntArithLatency);
        case IntLogicOp    : return Cycles (IntLogicLatency);
        case IntTestOp     : return Cycles (IntTestLatency);
        case IntShiftOp    : return Cycles (IntShiftLatency);
        case IntBitOp      : return Cycles (IntBitLatency);
        case IntMoveOp     : return Cycles (IntMoveLatency);
        case IntMulOp      : return Cycles (IntMulLatency);
        case IntMacOp      : return Cycles (IntMacLatency);
        case IntIterOp     : return Cycles (IntIterLatency);
        case SimdIntArithOp: return Cycles (SimdIntArithLatency);
        case SimdIntLogicOp: return Cycles (SimdIntLogicLatency);
        case SimdIntTestOp : return Cycles (SimdIntTestLatency);
        case SimdIntShiftOp: return Cycles (SimdIntShiftLatency);
        case SimdIntMulOp  : return Cycles (SimdIntMulLatency);
        case SimdIntMacOp  : return Cycles (SimdIntMacLatency);
        case SimdIntIterOp : return Cycles (SimdIntIterLatency);
        case IntMemOp      : if (memFlag) {
                                 return Cycles (IntMemLatency);
                             } else {
                                 return Cycles (IntMemAddrLatency);
                             }
        default            : assert (0);
    }
}

////////////////////////////////////////////////////////////////////////
//
//  HybridCPU Simulation Object
//
HybridCPU *
HybridCPUParams::create()
{
    numThreads = 1;
    if (!FullSystem && workload.size() != 1)
        panic("only one workload allowed");
    return new HybridCPU(this);
}
