/*
 ****************************************************************************
 *
 * simulavr - A simulator for the Atmel AVR family of microcontrollers.
 * Copyright (C) 2001, 2002, 2003   Klaus Rudolph
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 ****************************************************************************
 *
 *  $Id$
 */

#include "hwcache.h"
#include "avrdevice.h"
#include "systemclock.h"
#include "irqsystem.h"
#include "avrerror.h"
#include <assert.h>
#include <cmath>
#include <sstream>

using namespace std;

static inline bool powerof_two(int n) {
    return n && !(n & (n - 1));
}

HWCache::HWCache(AvrDevice *_core,
                 unsigned int lines,
                 unsigned int linesize,
                 unsigned int assoc,
                 HWIrqSystem *_irqSystem,
                 unsigned int size,
                 unsigned int irqVec):
    Hardware(_core),
    TraceValueRegister(_core, "CACHE"),
    core(_core),
    irqSystem(_irqSystem),
    irqVectorNo(irqVec),
    ccr_reg(this, "CCR", this, &HWCache::GetCcr, &HWCache::SetCcr),
    opState(OPSTATE_ENABLED),
    opMode(OPMODE_WRITETHROUGH),
    cacheHitCycles(0),
    cacheMissCycles(3),
    cache_lines(lines),
    cache_linesize(linesize),
    cache_assoc(assoc)
{
    if(irqSystem)
        irqSystem->DebugVerifyInterruptVector(irqVectorNo, this);

    if(opMode == OPMODE_WRITETHROUGH) {
        cacheClearTime = 1500000LL; // 1.5ms - just drops data
    } else {
        cacheClearTime = 8500000LL; // 8.5ms - writeback needs to store to backing memory
    }
    if(irqSystem == NULL) {
        ccr_mask = ~CTRL_IRQ;  // ignore IRQ bit if system is not set up
    }
    ccr = CTRL_UNINITIALIZED;
    Reset();
    _init_cache_model();
}

void HWCache::Reset() {
    cpuHoldCycles = 0;

    // by default: cache ON
    ccr = CTRL_ENABLE;
    if (irqSystem)
        ccr |= CTRL_IRQ;
    opState = OPSTATE_ENABLED;
    opMode = OPMODE_WRITETHROUGH;
}

HWCache::~HWCache() {

}

void HWCache::_init_cache_model() {
    assert(powerof_two(cache_linesize));
    assert(powerof_two(cache_lines));
    cache_offsetbits = (int)(log(cache_linesize) / log(2.));
    cache_sets = cache_lines / cache_assoc;

    // verbose config
    stringstream ss;
    ss << " CACHE: lines=" << cache_lines << " each " << cache_linesize
       << "bytes (" << cache_offsetbits << "bits), assoc=" << cache_assoc
       << ", sets=" << cache_sets << ", policy=LRU";
    avr_warning(ss.str().c_str());
}

/**
 * @brief read/write item at [addr, addr + len[.
 * @param allow_update if true, accessed item is cached thereafter, otherwise cache is bypassed
 */
int HWCache::_serve_request
(unsigned int addr, unsigned char len, bool write, bool allow_update) {
    int cycles = 0;
    // 1. compute set & check alignment
    unsigned int block = addr >> cache_offsetbits;
    unsigned int set = block % cache_sets;
    // 2. search sets
    // 2.1 start walk list.
    // 2.2. if found, goto 4.
    // 2.3. if not found && set full, evict (writeback?)
    // 3. load/update age

    return cycles;
}

int HWCache::access(unsigned int addr, unsigned char len, bool write) {
    int cycles = 0;
    if (opState == OPSTATE_ENABLED || opState == OPSTATE_LOCKED) {
        if(core->trace_on == 1)
            traceOut << "CACHE: Read at 0x" << hex << addr << dec << " len=" << (int)len << " ";
        cycles = _serve_request(addr, len, write, opState == OPSTATE_LOCKED);
    }
    return cycles;
}

void HWCache::SetCcr(unsigned char newval) {
    if(core->trace_on == 1)
        traceOut << "CCR=0x" << hex << (unsigned int)newval << dec;

    ccr = newval & ccr_mask;

    switch(opState) {

        case OPSTATE_LOCKED:
        case OPSTATE_ENABLED:
            if ((ccr & CTRL_ENABLE) != CTRL_ENABLE) {
                cpuHoldCycles = 1;
                opState = OPSTATE_DISABLED;
                if(core->trace_on == 1)
                    traceOut << " CACHE: disabled";
                break;
            }

            if ((ccr & CTRL_CLEAR) == CTRL_CLEAR) {
                cpuHoldCycles = 4;
                // start timer ...
                SystemClockOffset t = cacheClearTime;
                clearDoneTime = SystemClock::Instance().GetCurrentTime() + t;
                opState = OPSTATE_CLEARING;
                ccr &= ~CTRL_CLEAR;  // immediately revoke bit
                if(core->trace_on == 1)
                    traceOut << " CACHE: Clear start";
                break; // to ignore any other requests
            }

            if ((ccr & CTRL_LOCK) == CTRL_LOCK) {
                // lock request
                if (opState == OPSTATE_ENABLED) {
                    cpuHoldCycles = 1;
                    opState = OPSTATE_LOCKED;
                    if(core->trace_on == 1)
                        traceOut << " CACHE: locked";
                }
            } else {
                // unlock request
                if (opState == OPSTATE_LOCKED) {
                    cpuHoldCycles = 1;
                    // abort enable state, switch to write state
                    opState = OPSTATE_ENABLED;
                    if(core->trace_on == 1)
                        traceOut << " CACHE: locked";
                }
            }
            break;

        case OPSTATE_DISABLED:
            if (ccr & CTRL_ENABLE == CTRL_ENABLE) {
                cpuHoldCycles = 1;
                opState = OPSTATE_ENABLED;
                if(core->trace_on == 1)
                    traceOut << " CACHE: enabled";
            }
            break;

        default:
            if (ccr & CTRL_MODE_WRITEBACK == CTRL_MODE_WRITEBACK) {
                if (opMode != OPMODE_WRITEBACK) {
                    opMode = OPMODE_WRITEBACK;
                    if(core->trace_on == 1)
                        traceOut << " CACHE: writeback mode";
                }
            } else {
                if (opMode != OPMODE_WRITETHROUGH) {
                    opMode = OPMODE_WRITETHROUGH;
                    if(core->trace_on == 1)
                        traceOut << " CACHE: writethrough mode";
                }
            }
            break;
    }
}

unsigned int HWCache::CpuCycle() {

    // handle clear state
    if(opState == OPSTATE_CLEARING) {
        if(SystemClock::Instance().GetCurrentTime() >= clearDoneTime) {
            // go back to ready state
            opState = OPSTATE_ENABLED;
            // process operation
            if(core->trace_on == 1)
                traceOut << " CACHE: Clear done";
            // now raise irq if enabled and available
            if((irqSystem != NULL) && ((ccr & CTRL_IRQ) == CTRL_IRQ))
                irqSystem->SetIrqFlag(this, irqVectorNo);
        }
    }

    // deactivate engine, if not used
    //if((opState == OPSTATE_READY) && (cpuHoldCycles == 0) && (opEnableCycles == 0))
    //    core->RemoveFromCycleList(this);

    // handle cpu hold state
    if(cpuHoldCycles > 0) {
        cpuHoldCycles--;
        return 1;
    } else
        return 0;

}

void HWCache::ClearIrqFlag(unsigned int vector) {
    if(vector == irqVectorNo)
        irqSystem->ClearIrqFlag(irqVectorNo);
}

