/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
 * Copyright (c) 2007 MIPS Technologies, Inc.
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
 * Authors: Gabe Black
 *          Korey Sewell
 *          Jaidev Patwardhan
 *          Zhengxing Li
 *          Deyuan Guo
 */

#ifndef __LILY2_FAULTS_HH__
#define __LILY2_FAULTS_HH__

#include "arch/lily2/pra_constants.hh"
#include "cpu/thread_context.hh"
#include "debug/Lily2PRA.hh"
#include "sim/faults.hh"
#include "sim/full_system.hh"

namespace Lily2ISA
{

typedef const Addr FaultVect;

enum ExcCode {
    // A dummy value to use when the code isn't defined or doesn't matter.
    ExcCodeDummy = 0,

    ExcCodeInt = 0,
    ExcCodeMod = 1,
    ExcCodeTlbL = 2,
    ExcCodeTlbS = 3,
    ExcCodeAdEL = 4,
    ExcCodeAdES = 5,
    ExcCodeIBE = 6,
    ExcCodeDBE = 7,
    ExcCodeSys = 8,
    ExcCodeBp = 9,
    ExcCodeRI = 10,
    ExcCodeCpU = 11,
    ExcCodeOv = 12,
    ExcCodeTr = 13,
    ExcCodeC2E = 18,
    ExcCodeMDMX = 22,
    ExcCodeWatch = 23,
    ExcCodeMCheck = 24,
    ExcCodeThread = 25,
    ExcCodeCacheErr = 30
};

class Lily2FaultBase : public FaultBase
{
  public:
    struct FaultVals
    {
        const FaultName name;
        const FaultVect offset;
        const ExcCode code;
    };

    void setExceptionState(ThreadContext *, uint8_t);

    virtual FaultVect offset(ThreadContext *tc) const = 0;
    virtual ExcCode code() const = 0;
    virtual FaultVect base(ThreadContext *tc) const
    {
        StatusReg status = tc->readMiscReg(MISCREG_STATUS);
        if (!status.bev)
            return tc->readMiscReg(MISCREG_EBASE);
        else
            return 0xbfc00200;
    }

    FaultVect
    vect(ThreadContext *tc) const
    {
        return base(tc) + offset(tc);
    }

    void invoke(ThreadContext * tc,
            StaticInstPtr inst = StaticInst::nullStaticInstPtr);
};

template <typename T>
class Lily2Fault : public Lily2FaultBase
{
  protected:
    static FaultVals vals;
  public:
    FaultName name() const { return vals.name; }
    FaultVect offset(ThreadContext *tc) const { return vals.offset; }
    ExcCode code() const { return vals.code; }
};

class SystemCallFault : public Lily2Fault<SystemCallFault> {};
class ReservedInstructionFault : public Lily2Fault<ReservedInstructionFault> {};
class ThreadFault : public Lily2Fault<ThreadFault> {};
class IntegerOverflowFault : public Lily2Fault<IntegerOverflowFault> {};
class TrapFault : public Lily2Fault<TrapFault> {};
class BreakpointFault : public Lily2Fault<BreakpointFault> {};
class DspStateDisabledFault : public Lily2Fault<DspStateDisabledFault> {};

class MachineCheckFault : public Lily2Fault<MachineCheckFault>
{
  public:
    bool isMachineCheckFault() { return true; }
};

class ResetFault : public Lily2Fault<ResetFault>
{
  public:
    void invoke(ThreadContext * tc,
            StaticInstPtr inst = StaticInst::nullStaticInstPtr);

};

class SoftResetFault : public Lily2Fault<SoftResetFault>
{
  public:
    void invoke(ThreadContext * tc,
            StaticInstPtr inst = StaticInst::nullStaticInstPtr);
};

class NonMaskableInterrupt : public Lily2Fault<NonMaskableInterrupt>
{
  public:
    void invoke(ThreadContext * tc,
            StaticInstPtr inst = StaticInst::nullStaticInstPtr);
};

class CoprocessorUnusableFault : public Lily2Fault<CoprocessorUnusableFault>
{
  protected:
    int coProcID;
  public:
    CoprocessorUnusableFault(int _procid) : coProcID(_procid)
    {}

    void
    invoke(ThreadContext * tc,
            StaticInstPtr inst = StaticInst::nullStaticInstPtr)
    {
        Lily2Fault<CoprocessorUnusableFault>::invoke(tc, inst);
        if (FullSystem) {
            CauseReg cause = tc->readMiscReg(MISCREG_CAUSE);
            cause.ce = coProcID;
            tc->setMiscRegNoEffect(MISCREG_CAUSE, cause);
        }
    }
};

class InterruptFault : public Lily2Fault<InterruptFault>
{
  public:
    FaultVect
    offset(ThreadContext *tc) const
    {
        CauseReg cause = tc->readMiscRegNoEffect(MISCREG_CAUSE);
        // offset 0x200 for release 2, 0x180 for release 1.
        return cause.iv ? 0x200 : 0x180;
    }
};

template <typename T>
class AddressFault : public Lily2Fault<T>
{
  protected:
    Addr vaddr;
    bool store;

    AddressFault(Addr _vaddr, bool _store) : vaddr(_vaddr), store(_store)
    {}

    void
    invoke(ThreadContext * tc,
            StaticInstPtr inst = StaticInst::nullStaticInstPtr)
    {
        Lily2Fault<T>::invoke(tc, inst);
        if (FullSystem)
            tc->setMiscRegNoEffect(MISCREG_BADVADDR, vaddr);
    }
};

class AddressErrorFault : public AddressFault<AddressErrorFault>
{
  public:
    AddressErrorFault(Addr _vaddr, bool _store) :
        AddressFault<AddressErrorFault>(_vaddr, _store)
    {}

    ExcCode
    code() const
    {
        return store ? ExcCodeAdES : ExcCodeAdEL;
    }

};

template <typename T>
class TlbFault : public AddressFault<T>
{
  protected:
    Addr asid;
    Addr vpn;

    TlbFault(Addr _asid, Addr _vaddr, Addr _vpn, bool _store) :
        AddressFault<T>(_vaddr, _store), asid(_asid), vpn(_vpn)
    {}

    void
    setTlbExceptionState(ThreadContext *tc, uint8_t excCode)
    {
        this->setExceptionState(tc, excCode);

        tc->setMiscRegNoEffect(MISCREG_BADVADDR, this->vaddr);
        EntryHiReg entryHi = tc->readMiscReg(MISCREG_ENTRYHI);
        entryHi.asid = this->asid;
        entryHi.vpn2 = this->vpn >> 2;
        entryHi.vpn2x = this->vpn & 0x3;
        tc->setMiscRegNoEffect(MISCREG_ENTRYHI, entryHi);

        ContextReg context = tc->readMiscReg(MISCREG_CONTEXT);
        context.badVPN2 = this->vpn >> 2;
        tc->setMiscRegNoEffect(MISCREG_CONTEXT, context);
    }

    void
    invoke(ThreadContext * tc,
            StaticInstPtr inst = StaticInst::nullStaticInstPtr)
    {
        if (FullSystem) {
            DPRINTF(Lily2PRA, "Fault %s encountered.\n", this->name());
            Addr vect = this->vect(tc);
            setTlbExceptionState(tc, this->code());
            tc->pcState(vect);
        } else {
            AddressFault<T>::invoke(tc, inst);
        }
    }

    ExcCode
    code() const
    {
        return this->store ? ExcCodeTlbS : ExcCodeTlbL;
    }
};

class TlbRefillFault : public TlbFault<TlbRefillFault>
{
  public:
    TlbRefillFault(Addr asid, Addr vaddr, Addr vpn, bool store) :
        TlbFault<TlbRefillFault>(asid, vaddr, vpn, store)
    {}

    FaultVect
    offset(ThreadContext *tc) const
    {
        StatusReg status = tc->readMiscReg(MISCREG_STATUS);
        return status.exl ? 0x180 : 0x000;
    }
};

class TlbInvalidFault : public TlbFault<TlbInvalidFault>
{
  public:
    TlbInvalidFault(Addr asid, Addr vaddr, Addr vpn, bool store) :
        TlbFault<TlbInvalidFault>(asid, vaddr, vpn, store)
    {}
};

class TlbModifiedFault : public TlbFault<TlbModifiedFault>
{
  public:
    TlbModifiedFault(Addr asid, Addr vaddr, Addr vpn) :
        TlbFault<TlbModifiedFault>(asid, vaddr, vpn, false)
    {}

    ExcCode code() const { return Lily2Fault<TlbModifiedFault>::code(); }
};

} // namespace Lily2ISA

#endif // __LILY2_FAULTS_HH__
