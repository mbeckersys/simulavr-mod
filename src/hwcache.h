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

#ifndef HWCACHE
#define HWCACHE

#include "rwmem.h"
#include "hardware.h"
#include "memory.h"
#include "traceval.h"
#include "irqsystem.h"

//! abstract cache model copied from HWEeprom. Does *not* track contents, only addresses and states.
class HWCache: public Hardware, public TraceValueRegister {
    protected:
        AvrDevice *core;
        unsigned char ccr;
        unsigned char ccr_mask;
        HWIrqSystem* irqSystem;
        unsigned int irqVectorNo;
        int opEnableCycles;
        int cpuHoldCycles;
        int opState;
        int opMode;
        unsigned int opAddr;
        SystemClockOffset eraseWriteDelayTime;
        SystemClockOffset eraseDelayTime;
        SystemClockOffset writeDelayTime;
        SystemClockOffset writeDoneTime;

    public:
        typedef enum {
          DEVMODE_NORMAL = 0,
          DEVMODE_EXTENDED
        } c_mode;

        enum {
          OPSTATE_READY,
          OPSTATE_ENABLED,
          OPSTATE_WRITE
        };

        enum {
          CTRL_MODE_ERASEWRITE = 0,
          CTRL_READ = 1,
          CTRL_WRITE = 2,
          CTRL_ENABLE = 4,
          CTRL_IRQ = 8,
          CTRL_MODE_ERASE = 16,
          CTRL_MODE_WRITE = 32,
          CTRL_MODES = 48,
        };

        HWCache(AvrDevice *core, HWIrqSystem *irqs, unsigned int size, unsigned int irqVec);
        virtual ~HWCache();

        //! returns number of cpu cycles taken to access data item
        int access(unsigned int addr);

        virtual unsigned int CpuCycle();
        void Reset();
        void ClearIrqFlag(unsigned int vector);

        void SetCcr(unsigned char);
        unsigned char GetCcr() { return ccr; }

        IOReg<HWCache> ccr_reg;  //! cache control register
        c_mode devMode;
};

#endif
