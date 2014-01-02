/*
 * Copyright (c) 2004-2006 The Regents of The University of Michigan
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
 * Authors: Ali Saidi
 *          Lisa Hsu
 *          Nathan Binkert
 */

/**
 * @file
 * This code loads the linux kernel, console, pal and patches certain
 * functions.  The symbol tables are loaded so that traces can show
 * the executing function and we can skip functions. Various delay
 * loops are skipped and their final values manually computed to speed
 * up boot time.
 */

#include "arch/generic/linux/threadinfo.hh"
#include "arch/lily2/linux/system.hh"
#include "arch/lily2/idle_event.hh"
#include "arch/lily2/system.hh"
#include "arch/vtophys.hh"
#include "base/loader/symtab.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "debug/Thread.hh"
#include "dev/platform.hh"
#include "kern/linux/events.hh"
#include "kern/linux/printk.hh"
#include "mem/physical.hh"
#include "mem/port.hh"
#include "sim/arguments.hh"
#include "sim/byteswap.hh"

using namespace std;
using namespace Lily2ISA;
using namespace Linux;

LinuxLily2System::LinuxLily2System(Params *p)
    : Lily2System(p)
{
}

LinuxLily2System::~LinuxLily2System()
{
}


void
LinuxLily2System::setDelayLoop(ThreadContext *tc)
{
    panic("setDelayLoop not implemented.\n");
}


void
LinuxLily2System::SkipDelayLoopEvent::process(ThreadContext *tc)
{
    SkipFuncEvent::process(tc);
    // calculate and set loops_per_jiffy
    ((LinuxLily2System *)tc->getSystemPtr())->setDelayLoop(tc);
}

void
LinuxLily2System::PrintThreadInfo::process(ThreadContext *tc)
{
    Linux::ThreadInfo ti(tc);

    DPRINTF(Thread, "Currently Executing Thread %s, pid %d, started at: %d\n",
            ti.curTaskName(), ti.curTaskPID(), ti.curTaskStart());
}

LinuxLily2System *
LinuxLily2SystemParams::create()
{
    return new LinuxLily2System(this);
}
