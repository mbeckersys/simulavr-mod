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

using namespace std;

HWCache::HWCache(AvrDevice *_core,
                   HWIrqSystem *_irqSystem,
                   unsigned int size,
                   unsigned int irqVec):
    Hardware(_core),
    TraceValueRegister(_core, "CACHE"),
    core(_core),
    irqSystem(_irqSystem),
    irqVectorNo(irqVec),
    ccr_reg(this, "CCR",
            this, &HWCache::GetCcr, &HWCache::SetCcr),
    devMode(DEVMODE_NORMAL)
{
    if(irqSystem)
        irqSystem->DebugVerifyInterruptVector(irqVectorNo, this);

    // operation duration, see datasheets for time!
    if(devMode == DEVMODE_NORMAL) {
        eraseWriteDelayTime = 8500000LL; // 8.5ms
        eraseDelayTime = 0LL; // isn't available in this op mode!
        writeDelayTime = 0LL; // isn't available in this op mode!
    } else {
        eraseWriteDelayTime = 3400000LL; // 3.4ms
        eraseDelayTime = 1800000LL; // 1.8ms
        writeDelayTime = 1800000LL; // 1.8ms
    }

    // in normal mode only erase + write in one operation is available
    if((devMode == DEVMODE_NORMAL)) {
        if(irqSystem == NULL)
            ccr_mask = 0x07; // without operation mode bits and irq enable
        else
            ccr_mask = 0x0f; // without operation mode bits
    } else
        ccr_mask = 0x3f; // with operation mode bits
    ccr = 0;

    opState = OPSTATE_READY;

    Reset();
}

void HWCache::Reset() {
    ccr &= 0x32; // bit 1 reflect CACHE statemachine state before reset!
                  // bit 4 and bit 5 are operation modes, which are hold over reset

    opEnableCycles = 0;
    cpuHoldCycles = 0;
}


HWCache::~HWCache() {

}

int HWCache::access(unsigned int addr) {
    return 0;  // TODO: model cache
}

void HWCache::SetCcr(unsigned char newval) {
    if(core->trace_on == 1)
        traceOut << "CCR=0x" << hex << (unsigned int)newval << dec;

    ccr = newval & ccr_mask;

    switch(opState) {

        default:
        case OPSTATE_READY:
            // enable write mode
            if((ccr & CTRL_ENABLE) == CTRL_ENABLE) {
                opState = OPSTATE_ENABLED;
                opEnableCycles = 4;
                core->AddToCycleList(this);
            }
            // read will be processed immediately
            if((ccr & CTRL_READ) == CTRL_READ) {
                cpuHoldCycles = 4;
                ccr &= ~CTRL_READ; // reset read bit isn't described in document!
                core->AddToCycleList(this);
                if(core->trace_on == 1)
                    traceOut << " CACHE: Read = 0x" << hex << 0 << dec;
            }
            // write will not processed
            ccr &= ~CTRL_WRITE;
            break;

        case OPSTATE_ENABLED:
            // enable bit will be hold in this state
            ccr |= CTRL_ENABLE;
            // read will be processed immediately
            if((ccr & CTRL_READ) == CTRL_READ) {
                cpuHoldCycles = 4;  // Datasheet: "When the CACHE is read, the CPU is halted for four cycles"
                ccr &= ~CTRL_READ; // reset read bit isn't described in document!
                if(core->trace_on == 1)
                    traceOut << " CACHE: Read = 0x" << hex << 0 << dec;
                break; // to ignore possible write request!
            }
            // start write operation
            if((ccr & CTRL_WRITE) == CTRL_WRITE) {
                cpuHoldCycles = 2;  // Datasheet: "When EEWE has been set, the CPU is halted for two cycles"
                // abort enable state, switch to write state
                opMode = ccr & CTRL_MODES;
                ccr &= ~CTRL_ENABLE;
                // start timer ...
                SystemClockOffset t;
                switch(opMode & CTRL_MODES) {
                    default:
                    case CTRL_MODE_ERASEWRITE:
                        t = eraseWriteDelayTime;
                        break;
                    case CTRL_MODE_ERASE:
                        t = eraseDelayTime;
                        break;
                    case CTRL_MODE_WRITE:
                        t = writeDelayTime;
                        break;
                }
                writeDoneTime = SystemClock::Instance().GetCurrentTime() + t;
                if(core->trace_on == 1)
                    traceOut << " CACHE: Write start";
            }
            break;

        case OPSTATE_WRITE:
            // enable write mode, mode change will not happen!
            if((ccr & CTRL_ENABLE) == CTRL_ENABLE) {
                opEnableCycles = 4;
            }
            // read is ignored here
            ccr &= ~CTRL_READ;
            // write is hold
            ccr |= CTRL_WRITE;
            break;

    }
}

unsigned int HWCache::CpuCycle() {

    // handle enable state and fallback to ready
    if(opEnableCycles > 0) {
        opEnableCycles--;
        if(opEnableCycles == 0) {
            ccr &= ~CTRL_ENABLE;
            if(opState == OPSTATE_ENABLED)
                opState = OPSTATE_READY;
            if(core->trace_on == 1)
                traceOut << " CACHE: WriteEnable cleared";
        }
    }

    // handle write state
    if(opState == OPSTATE_WRITE) {
        if(SystemClock::Instance().GetCurrentTime() >= writeDoneTime) {
            // go to ready state
            opState = OPSTATE_READY;
            // reset write enable bit
            ccr &= ~CTRL_WRITE;
            // process operation
            switch(opMode & CTRL_MODES) {
                default:
                case CTRL_MODE_ERASEWRITE:
                    break;
                case CTRL_MODE_ERASE:
                    break;
                case CTRL_MODE_WRITE:
                    break;
            }
            if(core->trace_on == 1)
                traceOut << " CACHE: Write done";
            // now raise irq if enabled and available
            if((irqSystem != NULL) && ((ccr & CTRL_IRQ) == CTRL_IRQ))
                irqSystem->SetIrqFlag(this, irqVectorNo);
        }
    }

    // deactivate engine, if not used
    if((opState == OPSTATE_READY) && (cpuHoldCycles == 0) && (opEnableCycles == 0))
        core->RemoveFromCycleList(this);

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

