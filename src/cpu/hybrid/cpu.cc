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
      IntArithDS (p->IntArithDS),
      IntMoveDS (p->IntMoveDS),
      SimdIntArithDS (p->SimdIntArithDS)
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
    std::cout << "********************** NEW CYCLE **********************" << std::endl;
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

        curPipelineState = pipelineMacho.getCurState ();

        while (curPipelineState == R_Idle || curPipelineState == R_Run) {
            // Gets the PCState from the thread.
            TheISA::PCState pcState = thread->pcState ();

            // Fetches the instruction.
            fetch ();

            // Decodes the instruction.
            decode ();

            // Dispatches the instruction.
            dispatch ();

            // Pipeline state transfer.
            if (!curStaticInst) {
                curPipelineEvent = NoIssue;
            } else {
                // Executes the instruction.
                execute ();

                if (curStaticInst->isIter ()) {
                    // Iterative inst.
                    curPipelineEvent = IterInst;
                } else if (curStaticInst->isControl ()) {
                    // Flow control inst.
                    curPipelineEvent = (curStaticInst->getBPreded ()) ? BPreded : MisBPred;
                } else if (curStaticInst->isMemRef ()) {
                    // Memory reference inst.
                    curPipelineEvent = (curStaticInst->getVPreded ()) ? VPreded : MisVPred;
                } else if (curStaticInst->isModeSwitch ()) {
                    // Mode switch inst.
                    curPipelineEvent = (curStaticInst->isToRisc ()) ? ToRiscInst : ToVliwInst;
                } else {
                    // Default.
                    curPipelineEvent = Issue;
                }

                //pcState.advance ();
                //thread->pcState (pcState);
            }
            curPipelineState = pipelineMacho.transfer (curPipelineEvent);
        }

        // Calls the pre-update corresponding callbacks.
        callPrePipelineCallback ();

        // Commits the instructions.
        update ();

        // Calls the post-update corresponding callbacks.
        callPostPipelineCallback ();

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
HybridCPU::setupFetchRequest(Request *req)
{
    Addr instAddr = thread->instAddr();
    Addr PCMask = ~(sizeof (MachInst) - 1);
    Addr fetchPC = instAddr & PCMask;

    req->setVirt
        (0, fetchPC, sizeof(MachInst), Request::INST_FETCH, instMasterId(), instAddr);

#if DEBUG
    std::cout << "<----- fetch" << std::endl;
    std::cout << "       fetchPC = 0x"
              << std::hex << std::setfill ('0') << std::setw (8)
              << instAddr << std::endl;
    std::cout << "       aligned fetchPC = 0x"
              << std::hex << std::setfill ('0') << std::setw (8)
              << fetchPC << std::endl;
#endif
}

Cycles
HybridCPU::fetch (void)
{
    setupFetchRequest (&ifetch_req);

    thread->itb->translateAtomic(&ifetch_req, tc, BaseTLB::Execute);

    Packet ifetch_pkt = Packet(&ifetch_req, MemCmd::ReadReq);
    ifetch_pkt.dataStatic(&inst);

    /* Do the actual fetch here. */
    icachePort.sendAtomic(&ifetch_pkt);
    inst = gtobe (inst);

#if DEBUG
    std::cout << "       fetch instruction = 0x"
              << std::hex << std::setfill ('0') << std::setw (8)
              << inst << std::endl;
    std::cout << "-----> fetch" << std::endl;
    std::cout << std::endl;
#endif

    /* TODO */
    return Cycles (0);
}

void
HybridCPU::decode (void)
{
    curStaticInst = NULL;
    TheISA::Decoder *decoder = &(thread->decoder);

    StaticInstPtr tmpStaticInst = decoder->decodeInst (inst, this);
    curStaticInst = dynamic_cast<Lily2StaticInst *> (tmpStaticInst.get ());

#if DEBUG
    std::cout << "<----- decode" << std::endl;
    std::cout << "       disassemble: "
              << curStaticInst->disassemble (thread->instAddr ()) << std::endl;
    std::cout << "-----> decode" << std::endl;
    std::cout << std::endl;
#endif
}

void
HybridCPU::dispatch (void)
{
    ;

#if DEBUG
    std::cout << "<----- dispatch" << std::endl;
    std::cout << "-----> dispatch" << std::endl;
    std::cout << std::endl;
#endif
}

void
HybridCPU::execute (void)
{
#if DEBUG
    std::cout << "<----- execute" << std::endl;
#endif

    curStaticInst->execute (this, traceData);
#if DEBUG
    std::cout << "       operation:"
              << curStaticInst->operate () << std::endl;
    std::cout << "-----> execute" << std::endl;
    std::cout << std::endl;
#endif
}

void
HybridCPU::update (void)
{
    ;

#if DEBUG
    std::cout << "<----- update" << std::endl;
    std::cout << "-----> update" << std::endl;
    std::cout << std::endl;
#endif
}

void
HybridCPU::updateRegDepTable (void)
{
}

void
HybridCPU::updateRegFileBuf (void)
{
}

void
HybridCPU::updateCycle (void)
{
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
    pipelineMacho.regStateEvent (R_Idle, NoIssue, R_Idle);
    // Turns to R_Advance or R_Flush if branch inst is met.
    pipelineMacho.regStateEvent (R_Idle, BPreded, R_Advance);
    pipelineMacho.regStateEvent (R_Idle, MisBPred, R_Flush);
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
    pipelineMacho.regLocalDefaultState (R_Flush, R_Idle);

    // R_InstWait.
    // Caused by iterative instructions.
    // Break the issue iteration.
    pipelineMacho.regLocalDefaultState (R_InstWait, R_Idle);

    // R_Advance.
    // Caused by normal dependencies.
    // Break the issue iteration.
    pipelineMacho.regLocalDefaultState (R_Advance, R_Idle);

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
HybridCPU::setCallback (PipelineState state)
{
    switch (state) {
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
    // Loops (delay slot).
    // URDT.
    // URFB.
    // UC.
}

void
HybridCPU::callback_R_InstWait (void)
{
    // Loops (execution cycles - 1).
    // UC.
}

void
HybridCPU::callback_R_Advance (void)
{
    // Loops (0).
}

void
HybridCPU::callback_R_2_R (void)
{
    // Loops (0).
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
    Op32i_t *op = dynamic_cast<Op32i_t *> (si->getSrcOp (idx));
    assert (op);

    if (op->immFlag ()) {
        // The immediate value is already stored in OP.
        ;
    } else {
        uint32_t val = 0;
        RegIndex_t regIndex = op->regIndex ();

        switch (op->regFile ()) {
            case TheISA::REG_X:
                val = thread->readXReg (regIndex);
                op->setUval (val);
                break;
            case TheISA::REG_Y:
                val = thread->readYReg (regIndex);
                op->setUval (val);
                break;
            case TheISA::REG_G:
                val = thread->readGReg (regIndex);
                op->setUval (val);
                break;
            case TheISA::REG_M:
                val = thread->readMReg (regIndex);
                op->setUval (val);
                break;
            default:
                assert (0);
        }
    }

    return *op;
}

const Op32f_t&
HybridCPU::readOp32f (Lily2StaticInst *si, const OpCount_t& idx)
{
    Op32f_t *op = dynamic_cast<Op32f_t *> (si->getSrcOp (idx));
    assert (op);

    if (op->immFlag ()) {
        // The Immediate value is already stored in OP.
        ;
    } else {
        uint32_t val;
        switch (op->regFile ()) {
            case TheISA::REG_X:
                val = (thread->getXRegs ()).getRegValue (op->regIndex ());
                op->setFval (*(reinterpret_cast<float *> (&val)));
                break;
            case TheISA::REG_Y:
                val = (thread->getYRegs ()).getRegValue (op->regIndex ());
                op->setFval (*(reinterpret_cast<float *> (&val)));
                break;
            case TheISA::REG_G:
                val = (thread->getGRegs ()).getRegValue (op->regIndex ());
                op->setFval (*(reinterpret_cast<float *> (&val)));
                break;
            case TheISA::REG_M:
                val = (thread->getMRegs ()).getRegValue (op->regIndex ());
                op->setFval (*(reinterpret_cast<float *> (&val)));
                break;
            default:
                assert (0);
        }
    }

    return *op;
}

void
HybridCPU::setOp32i (Lily2StaticInst *si, const OpCount_t& idx,
                     const Op32i_t& val, const Op32i_t& mask)
{
    Op32i_t *op;
    assert (op = dynamic_cast<Op32i_t *> (si->getDestOp (idx)));

    RegIndex_t regIndex = op->regIndex ();
    uint32_t regValue = val.uval ();
    uint32_t regMask = mask.uval ();
    Cycles regBackCycle = funcUnitDSFactory (si->opClass ());

    op->setUval (regValue);

    switch (op->regFile ()) {
        case TheISA::REG_X:
            thread->setXRegBuf (regIndex, regValue, regMask, regBackCycle);
            break;

        case TheISA::REG_Y:
            thread->setYRegBuf (regIndex, regValue, regMask, regBackCycle);
            break;

        case TheISA::REG_G:
            thread->setGRegBuf (regIndex, regValue, regMask, regBackCycle);
            break;

        case TheISA::REG_M:
            thread->setMRegBuf (regIndex, regValue, regMask, regBackCycle);
            break;

        default:
            assert (0);
    }
}

void
HybridCPU::setOp32f (Lily2StaticInst *si, const OpCount_t& idx,
                     const Op32f_t& val, const Op32f_t& mask)
{
    Op32f_t *op;
    assert (op = dynamic_cast<Op32f_t *> (si->getDestOp (idx)));

    uint32_t regValue = val.bval ();
    uint32_t regMask = mask.bval ();
    Cycles regBackCycle = funcUnitDSFactory (si->opClass ());

    op->setBval (regValue);

    switch (op->regFile ()) {
        case TheISA::REG_X:
            (thread->getXRegBufs ()).insert (op->regIndex (),
                    regValue, regMask, regBackCycle);
            break;

        case TheISA::REG_Y:
            (thread->getYRegBufs ()).insert (op->regIndex (),
                    regValue, regMask, regBackCycle);
            break;

        case TheISA::REG_G:
            (thread->getGRegBufs ()).insert (op->regIndex (),
                    regValue, regMask, regBackCycle);
            break;

        case TheISA::REG_M:
            (thread->getMRegBufs ()).insert (op->regIndex (),
                    regValue, regMask, regBackCycle);
            break;

        default:
            assert (0);
    }
}

Cycles
HybridCPU::funcUnitDSFactory (const OpClass& opClass) const
{
    switch (opClass) {
        case IntArithOp: return Cycles (IntArithDS);
        case IntMoveOp: return Cycles (IntMoveDS);
        case SimdIntArithOp: return Cycles (SimdIntArithDS);
        default: assert (0);
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
