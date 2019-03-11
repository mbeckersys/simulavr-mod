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

/**
 * @brief abstract cache model copied from HWEeprom.
 * Does *not* track contents, only addresses and states.
 *
 * IRQ: raised when after cache clear request has been processed.
 */
class HWCache: public Hardware, public TraceValueRegister {
    public:
        //! cache operational state machine:
        typedef enum {
          OPSTATE_DISABLED = 0, ///< all misses
          OPSTATE_ENABLED,
          OPSTATE_LOCKED,
          OPSTATE_CLEARING
        } op_state_e;

        //! if enabled, we have the following modes:
        typedef enum {
          OPMODE_WRITETHROUGH = 0,
          OPMODE_WRITEBACK
        } op_mode_e;

    protected:
        AvrDevice *core;
        unsigned char ccr;
        unsigned char ccr_mask;
        HWIrqSystem* irqSystem;
        unsigned int irqVectorNo;
        int cpuHoldCycles;
        op_state_e opState;  ///< state machine
        op_mode_e opMode;
        unsigned int opAddr;
        int cacheHitCycles;
        int cacheMissCycles;
        SystemClockOffset cacheClearTime;  ///< time, not clocks
        SystemClockOffset clearDoneTime;

        unsigned int cache_lines;
        unsigned int cache_linesize;
        unsigned int cache_assoc;
        // computed:
        unsigned int cache_sets;
        unsigned int cache_offsetbits;

        int _serve_request(unsigned int addr, unsigned char len, bool write, bool allow_update);
        void _init_cache_model();

    public:
        //! bits in ctrl register
        enum {
          CTRL_UNINITIALIZED = 0,
          CTRL_ENABLE = 1,  ///< cache enable
          CTRL_LOCK = 2,    ///< lock cache
          CTRL_CLEAR = 4,   ///< cache is cleared if set
          CTRL_IRQ = 8,     ///< interrupt enable
          CTRL_MODE_WRITEBACK = 16,  ///< 0=writethrough
          CTRL_MODE_OTHER = 32,
          CTRL_MODES = 48,
        };

        HWCache(AvrDevice *core,
                unsigned int lines,
                unsigned int linesize,
                unsigned int assoc,
                HWIrqSystem *irqs,
                unsigned int size,
                unsigned int irqVec);

        virtual ~HWCache();

        //! returns number of cpu cycles taken to access data item
        int access(unsigned int addr, unsigned char len, bool write=false);

        //! returns > 0 if wait states are required
        virtual unsigned int CpuCycle();
        void Reset();
        void ClearIrqFlag(unsigned int vector);

        void SetCcr(unsigned char);
        unsigned char GetCcr() { return ccr; }

        IOReg<HWCache> ccr_reg;  //! cache control register
};

#endif
