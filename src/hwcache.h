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

#include <string>
#include <stdio.h>
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
          OPMODE_WRITEBACK = 0,
          OPMODE_WRITETHROUGH  ///< implies write-allocate
        } op_mode_e;

        typedef struct {
            unsigned long num_access;
            unsigned long num_miss;
            unsigned long num_evict;
            unsigned long num_writeback;
            unsigned long num_unaligned;
            unsigned long num_clears;
        } cache_stats_t;

    protected:

        //! one cache item of a set
        typedef struct cache_entry_s {
            unsigned int tag;
            bool dirty;
            cache_entry_s* next; ///< NULL if no older entries than this
        } cache_entry_t;

        /**
         * @brief each set points to a linked list which is contiguous in memory and has
         * length assoc+1, whereas the first element of the list is always present and a DUMMY.
         * Dummy points to the actual list, whereas the items in the list are sorted by their age.
         */
        typedef struct cache_set_s {
            cache_entry_t* begin;  ///< points to NULL-terminated list of
            unsigned int num_entries;
        } cache_set_t;

        AvrDevice *core;
        // register stuff
        unsigned char ccr;
        unsigned char ccr_mask;
        // irq stuff:
        HWIrqSystem* irqSystem;
        unsigned int irqVectorNo;
        // device state:
        op_state_e opState;  ///< state machine
        op_mode_e opMode;
        int cpuHoldCycles;
        SystemClockOffset clearDoneTime;
        // user params:
        unsigned int cache_config_nlines;
        unsigned int cache_config_linesize;
        unsigned int cache_config_assoc;
        // model properties:
        int cacheHitCycles;
        int cacheMissCycles;
        int cacheWritethroughCycles;
        int cacheWritebackCycles;
        SystemClockOffset cacheClearTime;  ///< time, not clocks
        // computed:
        unsigned int cache_config_nsets;
        unsigned int cache_offsetbits;
        // data for model:
        cache_entry_t* cache_model_lines;
        cache_set_t* cache_model_sets;

        // for stats
        cache_stats_t stats;

        void _init_cache_model(void);
        int _serve_access(unsigned int addr, unsigned char len, bool write, bool allow_update);
        inline int _access_set(unsigned int set, unsigned int tag, bool write, bool allow_update);
        inline int _update_set_lru(unsigned set, cache_entry_t* prev_item,
                                   cache_entry_t* accessed_item, cache_entry_t* checked_item,
                                   unsigned tag, bool write);
        void _clear_cache(void);
        void _cleanup_cache_model(void);
        void trace(const char *fmt, ...);

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
                unsigned int irqVec,
                bool trace_on=false);

        virtual ~HWCache();

        //! returns number of cpu cycles taken to access data item
        int access(unsigned int addr, unsigned char len, bool write=false);

        std::string get_stats(void);
        void print_stats(void);
        void fprint_stats(FILE* fp);

        //! returns > 0 if wait states are required
        virtual unsigned int CpuCycle();
        void Reset();
        void ClearIrqFlag(unsigned int vector);

        void SetCcr(unsigned char);
        unsigned char GetCcr() { return ccr; }

        IOReg<HWCache> ccr_reg;  //! cache control register
        FILE* traceFile;
};

#endif
