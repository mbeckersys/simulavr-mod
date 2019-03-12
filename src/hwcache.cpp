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
#include "avrmalloc.h"
#include <assert.h>
#include <string.h>
#include <cmath>
#include <sstream>
#include <stdio.h>

using namespace std;

/**
 * Cache model (LRU):
 *
 * we allocate a continuous chunk of memory for all lines
 * each line has: tag, dirty, valid, next*
 * lines within set are stored as NULL-terminated linked lists, using the next* pointer
 * each cache set is simply the ptr to the head of the list (=newest)
 */

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
    opMode(OPMODE_WRITEBACK),
    cache_config_nlines(lines),
    cache_config_linesize(linesize),
    cache_config_assoc(assoc),
    cacheHitCycles(0),
    cacheMissCycles(3),
    cacheWritethroughCycles(5),
    cacheWritebackCycles(5)
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
    opMode = OPMODE_WRITEBACK;
}

HWCache::~HWCache() {
    print_stats();
    _cleanup_cache_model();
}

void HWCache::_cleanup_cache_model(void) {
    avr_free(cache_model_lines);
    avr_free(cache_model_sets);
}

void HWCache::_init_cache_model(void) {
    assert(powerof_two(cache_config_linesize));
    assert(powerof_two(cache_config_nlines));
    cache_offsetbits = (int)(log(cache_config_linesize) / log(2.));
    cache_config_nsets = cache_config_nlines / cache_config_assoc;

    memset((void*)&stats, 0, sizeof(stats));

    // set up contiguous memory chunk to model cache; each set has one additional dummy entry
    cache_model_sets = avr_new(cache_set_t, cache_config_nsets);
    cache_model_lines = avr_new(cache_entry_t, cache_config_nsets * (cache_config_assoc + 1));
    cache_entry_t* ce = cache_model_lines;
    for (int k = 0; k < cache_config_nsets; ++k, ce += (cache_config_assoc + 1)) {
        cache_model_sets[k].begin = ce;
    }
    _clear_cache();

    // verbose config
    stringstream ss;
    ss << " CACHE: lines=" << cache_config_nlines << " each " << cache_config_linesize
       << "bytes (" << cache_offsetbits << "bits), assoc=" << cache_config_assoc
       << ", sets=" << cache_config_nsets << ", policy=LRU";
    avr_warning(ss.str().c_str());
}

void HWCache::print_stats(void) {
    stringstream ss;
    unsigned lines_used = 0;
    for (int set = 0; set < cache_config_nsets; ++set) {
        lines_used += cache_model_sets[set].num_entries;
    }
    ss << "CACHE statistics:" << endl
       << "  usage%:     " << 100.f*(((float)lines_used) / cache_config_nlines) << endl
       << "  accesses:   " << stats.num_access << endl
       << "  misses:     " << stats.num_miss << endl
       << "  hit ratio%: " << 100.f - 100.f*(((float)stats.num_miss) / stats.num_access) << endl
       << "  evictions:  " << stats.num_evict << endl
       << "  writeback:  " << stats.num_writeback << endl
       << "  unaligned:  " << stats.num_unaligned << endl
       << "  clears:     " << stats.num_clears << endl;
    avr_warning(ss.str().c_str());
}

/**
 * @brief update given set by loading tag, if not loaded. Possibly evict another one.
 * @param accessed_item pointer to item, in case tag is already in set. Otherwise NULL.
 * @param checked_item the last (=oldest) item in the set that was checked
 * @param prev_item the one previous to last
 **/
inline int HWCache::_update_set_lru
(unsigned set, cache_entry_t* prev_item, cache_entry_t* accessed_item, cache_entry_t* checked_item,
 unsigned tag, bool write)
{
    int cycles = 0;
    if (accessed_item == NULL) {
        // not in cache
        if (cache_model_sets[set].num_entries == cache_config_assoc) {
            // eviction needed: throw out oldest (LRU)
            assert(checked_item);
            if (opMode == OPMODE_WRITEBACK && checked_item->dirty) {
                cycles += cacheWritebackCycles;
                stats.num_writeback++;
            }
            // take its line
            accessed_item = checked_item;
            assert(prev_item != cache_model_sets[set].begin);
            prev_item->next = NULL; // prev is now the oldest
            stats.num_evict++;
        } else {
            // space left in set: take free line
            cache_model_sets[set].num_entries++;
            accessed_item = cache_model_sets[set].begin + cache_model_sets[set].num_entries;
        }
        accessed_item->tag = tag;
        accessed_item->next = cache_model_sets[set].begin->next; // formerly youngest ages now

    } else {
        // is in cache. just update LRU sorting
        prev_item->next = accessed_item->next;
    }

    // accessed is the youngest in the set now
    cache_model_sets[set].begin->next = accessed_item;

    // finally, set bits
    accessed_item->dirty = (write && opMode == OPMODE_WRITEBACK);

    return cycles;
}

inline int HWCache::_access_set
(unsigned int set, unsigned int tag, bool write, bool allow_update)
{
    int cycles = 0;
    assert(set < cache_config_nsets);
    cache_entry_t* checked_item = cache_model_sets[set].begin;  ///< begin is the dummy item

    // linear search for tag in set
    cache_entry_t* prev_item;
    cache_entry_t* found_item = NULL;
    for (int k = 0; k < cache_config_assoc; ++k) {
        prev_item = checked_item;
        if (!checked_item->next) break;

        checked_item = checked_item->next;
        if (checked_item->tag == tag) {
            found_item = checked_item;
            break;
        }
    }

    // hit/miss penalties
    if (found_item) {
        assert(found_item != cache_model_sets[set].begin);
        cycles = cacheHitCycles;
    } else {
        cycles = cacheMissCycles;
        stats.num_miss++;
    }
    if (write && opMode == OPMODE_WRITETHROUGH) {
        cycles += cacheWritethroughCycles;
    }

    if (allow_update) {
        cycles += _update_set_lru(set, prev_item, found_item, checked_item, tag, write);
    }

    stats.num_access++;
    return cycles;
}

/**
 * @brief read/write item at [addr, addr + len[.
 * @param allow_update if true, accessed item is cached thereafter, otherwise cache is bypassed
 */
int HWCache::_serve_access
(unsigned int addr, unsigned char len, bool write, bool allow_update)
{
    assert(len <= cache_config_linesize);
    int cycles = 0;
    // compute set & check alignment
    const unsigned block = addr >> cache_offsetbits;
    const unsigned offset = addr - block;
    const unsigned set = block % cache_config_nsets;
    cycles += _access_set(set, block, write, allow_update);

    if (offset + len > cache_config_linesize) {
        // unaligned access
        const unsigned next_block = block + 1;
        const unsigned next_set = next_block % cache_config_nsets;
        cycles += _access_set(next_set, next_block, write, allow_update);
        stats.num_unaligned++;
    }
    return cycles;
}

int HWCache::access(unsigned int addr, unsigned char len, bool write) {
    int cycles = 0;
    if (opState == OPSTATE_ENABLED || opState == OPSTATE_LOCKED) {
        if(core->trace_on == 1)
            traceOut << "CACHE: Read at 0x" << hex << addr << dec << " len=" << (int)len << " ";
        cycles = _serve_access(addr, len, write, opState != OPSTATE_LOCKED);
    }
    return cycles;
}

void HWCache::_clear_cache(void) {
    for (int k = 0; k < cache_config_nsets; ++k) {
        cache_model_sets[k].num_entries = 0;
    }
    // each set has one leading dummy entry
    const unsigned nbytes = sizeof(cache_entry_t) * cache_config_nsets * (cache_config_assoc + 1);
    memset((void*)cache_model_lines, 0, nbytes);
    stats.num_clears++;
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
                _clear_cache();
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

