/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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
 */

/*
 * Branch Target Buffer (BTB) Implementation
 * 
 * The BTB is a cache-like structure that stores information about branches:
 * - Branch type (conditional, unconditional, indirect, call, return)
 * - Branch target address
 * - Branch prediction information (counter for conditional branches)
 * 
 * Key Features:
 * - N-way set associative organization
 * - MRU (Most Recently Used) replacement policy
 * - Support for multiple branch types
 * - Support for multiple prediction stages (L0/L1 BTB)
 */

#ifndef __CPU_PRED_BTB_BTB_HH__
#define __CPU_PRED_BTB_BTB_HH__

#include "arch/generic/pcstate.hh"
#include "base/logging.hh"
#include "base/types.hh"
#include "config/the_isa.hh"
#include "cpu/pred/btb/stream_struct.hh"
#include "cpu/pred/btb/timed_base_pred.hh"
#include "debug/BTB.hh"
#include "debug/BTBStats.hh"
#include "params/DefaultBTB.hh"


namespace gem5
{

namespace branch_prediction
{

namespace btb_pred
{

class DefaultBTB : public TimedBaseBTBPredictor
{
  private:

  public:

    typedef DefaultBTBParams Params;

    DefaultBTB(const Params& p);

    /*
     * BTB Entry with timestamp for MRU replacement
     * Inherits from BTBEntry which contains:
     * - valid: whether this entry is valid
     * - pc: branch instruction address
     * - target: branch target address
     * - size: branch instruction size
     * - isCond/isIndirect/isCall/isReturn: branch type flags
     * - alwaysTaken: whether this conditional branch is always taken
     * - ctr: 2-bit counter for conditional branch prediction
     */
    typedef struct TickedBTBEntry : public BTBEntry
    {
        uint64_t tick;  // timestamp for MRU replacement
        TickedBTBEntry(const BTBEntry &entry, uint64_t tick)
            : BTBEntry(entry), tick(tick) {}
        TickedBTBEntry() : tick(0) {}
    }TickedBTBEntry;

    // A BTB set is a vector of entries (ways)
    using BTBSet = std::vector<TickedBTBEntry>;
    using BTBSetIter = typename BTBSet::iterator;
    // MRU heap for each set
    using BTBHeap = std::vector<BTBSetIter>;

    /*
     * Comparator for MRU heap
     * Returns true if a's timestamp is larger than b's
     * This creates a min-heap where the oldest entry is at the top
     */
    struct older
    {
        bool operator()(const BTBSetIter &a, const BTBSetIter &b) const
        {
            return a->tick > b->tick;
        }
    };

    void tickStart() override;
    
    void tick() override;

    /*
     * Main prediction function
     * @param startAddr: start address of the fetch block
     * @param history: branch history register
     * @param stagePreds: predictions for each pipeline stage
     * 
     * This function:
     * 1. Looks up BTB entries for the fetch block
     * 2. Updates prediction statistics
     * 3. Fills predictions for each pipeline stage
     */
    void putPCHistory(Addr startAddr, const boost::dynamic_bitset<> &history,
                      std::vector<FullBTBPrediction> &stagePreds) override;

    std::shared_ptr<void> getPredictionMeta() override;

    void specUpdateHist(const boost::dynamic_bitset<> &history, FullBTBPrediction &pred) override;

    /** Creates a BTB with the given number of entries, number of bits per
     *  tag, and instruction offset amount.
     *  @param numEntries Number of entries for the BTB.
     *  @param tagBits Number of bits for each tag in the BTB.
     *  @param instShiftAmt Offset amount for instructions to ignore alignment.
     */
    DefaultBTB(unsigned numEntries, unsigned tagBits,
               unsigned instShiftAmt, unsigned numThreads);

    void reset();

    /** Looks up an address for all possible entries in the BTB. Address are aligned in this function
     *  @param inst_PC The address of the block to look up.
     *  @return Returns all hit BTB entries.
     */
    std::vector<TickedBTBEntry> lookup(Addr block_pc);


    /** Updates the BTB with the branch info of a block and execution result.
     *  This function:
     *  1. Updates existing entries with new information
     *  2. Adds new entries if necessary
     *  3. Updates MRU information
     */
    void update(const FetchStream &stream) override;

    void commitBranch(const FetchStream &stream, const DynInstPtr &inst) override;

    void setTrace() override;

    TraceManager *btbTrace;

    /**
     * @brief derive new btb entry from old ones and set updateNewBTBEntry field in stream
     *        only in L1BTB will this function be called when update
     * 
     * @param stream 
     */
    void getAndSetNewBTBEntry(FetchStream &stream);

    void printBTBEntry(BTBEntry &e, uint64_t tick = 0) {
        DPRINTF(BTB, "BTB entry: valid %d, pc:%#lx, tag: %#lx, size:%d, target:%#lx, cond:%d, indirect:%d, call:%d, return:%d, always_taken:%d, tick:%llu\n",
            e.valid, e.pc, e.tag, e.size, e.target, e.isCond, e.isIndirect, e.isCall, e.isReturn, e.alwaysTaken, tick);
    }

    void printTickedBTBEntry(TickedBTBEntry &e) {
        printBTBEntry(e, e.tick);
    }

    void dumpBTBEntries(std::vector<BTBEntry> &es) {
        DPRINTF(BTB, "BTB entries:\n");
        for (auto &entry : es) {
            printBTBEntry(entry);
        }
    }

    void dumpTickedBTBEntries(std::vector<TickedBTBEntry> &es) {
        DPRINTF(BTB, "BTB entries:\n");
        for (auto &entry : es) {
            printTickedBTBEntry(entry);
        }
    }

    void dumpMruList(BTBHeap &list) {
        DPRINTF(BTB, "MRU list:\n");
        for (auto &it: list) {
            printTickedBTBEntry(*it);
        }
    }

    void checkAscending(std::vector<BTBEntry> &es) {
        Addr last = 0;
        bool misorder = false;
        for (auto &entry : es) {
            if (entry.pc <= last) {
                misorder = true;
                break;
            }
            last = entry.pc;
        }
        if (misorder) {
            DPRINTF(BTB, "BTB entries are not in ascending order:\n");
            dumpBTBEntries(es);
            assert(false);
        }
    }


  private:
    /** Returns the index into the BTB, based on the branch's PC.
     *  The index is calculated as: (pc >> idxShiftAmt) & idxMask
     *  where idxShiftAmt is:
     *  - log2(blockSize) if aligned to blockSize
     *  - 1 if not aligned to blockSize
     *  @param inst_PC The branch to look up.
     *  @return Returns the index into the BTB.
     */
    inline Addr getIndex(Addr instPC);

    /** Returns the tag bits of a given address.
     *  The tag is calculated as: (pc >> tagShiftAmt) & tagMask
     *  where tagShiftAmt = idxShiftAmt + log2(numSets)
     *  @param inst_PC The branch's address.
     *  @return Returns the tag bits.
     */
    inline Addr getTag(Addr instPC);

    /** Helper function to check if this is L0 BTB
     *  L0 BTB has zero delay (getDelay() == 0)
     */
    bool isL0() { return getDelay() == 0; }

    /** Update the 2-bit saturating counter for conditional branches
     *  Counter range: [-2, 1]
     *  - Increment on taken (max 1)
     *  - Decrement on not taken (min -2)
     */
    void updateCtr(int &ctr, bool taken) {
        if (taken && ctr < 1) {ctr++;}
        if (!taken && ctr > -2) {ctr--;}
    }

    /** The BTB structure:
     *  - Organized as numSets sets
     *  - Each set has numWays ways
     *  - Total size = numSets * numWays = numEntries
     */
    std::vector<BTBSet> btb;

    /** MRU tracking:
     *  - One heap per set
     *  - Each heap tracks the MRU order of entries in that set
     *  - Oldest entry is at the top of heap
     */
    std::vector<BTBHeap> mruList;

    /** BTB configuration parameters */
    unsigned numEntries;    // Total number of entries
    unsigned numWays;       // Number of ways per set
    unsigned numSets;       // Number of sets (numEntries/numWays)

    /** Address calculation masks and shifts */
    Addr idxMask;          // Mask for extracting index bits
    unsigned tagBits;      // Number of tag bits
    Addr tagMask;          // Mask for extracting tag bits
    unsigned idxShiftAmt;  // Amount to shift PC for index
    unsigned tagShiftAmt;  // Amount to shift PC for tag

    /** Thread handling */
    unsigned log2NumThreads;  // Log2 of number of threads for hashing

    /** Branch counter */
    unsigned numBr;  // Number of branches seen

    typedef struct BTBMeta {
        std::vector<BTBEntry> hit_entries;
        std::vector<BTBEntry> l0_hit_entries;
        BTBMeta() {
            std::vector<BTBEntry> es;
            hit_entries = es;
            l0_hit_entries = es;
        }
    }BTBMeta;

    BTBMeta meta;

    enum Mode {
        READ, WRITE, EVICT
    };

    struct BTBStats : public statistics::Group {
        statistics::Scalar newEntry;
        statistics::Scalar newEntryWithCond;
        statistics::Scalar newEntryWithUncond;
        statistics::Scalar oldEntry;
        statistics::Scalar oldEntryIndirectTargetModified;
        statistics::Scalar oldEntryWithNewCond;
        statistics::Scalar oldEntryWithNewUncond;

        statistics::Scalar predMiss;
        statistics::Scalar predHit;
        statistics::Scalar updateMiss;
        statistics::Scalar updateHit;

        statistics::Scalar eraseSlotBehindUncond;

        statistics::Scalar predUseL0OnL1Miss;
        statistics::Scalar updateUseL0OnL1Miss;

        // per branch statistics
        statistics::Scalar allBranchHits;
        statistics::Scalar allBranchHitTakens;
        statistics::Scalar allBranchHitNotTakens;
        statistics::Scalar allBranchMisses;
        statistics::Scalar allBranchMissTakens;
        statistics::Scalar allBranchMissNotTakens;

        statistics::Scalar condHits;
        statistics::Scalar condHitTakens;
        statistics::Scalar condHitNotTakens;
        statistics::Scalar condMisses;
        statistics::Scalar condMissTakens;
        statistics::Scalar condMissNotTakens;
        statistics::Scalar condPredCorrect;
        statistics::Scalar condPredWrong;

        statistics::Scalar uncondHits;
        statistics::Scalar uncondMisses;

        statistics::Scalar indirectHits;
        statistics::Scalar indirectMisses;
        statistics::Scalar indirectPredCorrect;
        statistics::Scalar indirectPredWrong;

        statistics::Scalar callHits;
        statistics::Scalar callMisses;

        statistics::Scalar returnHits;
        statistics::Scalar returnMisses;

        BTBStats(statistics::Group* parent);
    } btbStats;

    void incNonL0Stat(statistics::Scalar &stat) {
        if (!isL0()) {
            stat++;
        }
    }
};

} // namespace btb_pred
} // namespace branch_prediction
} // namespace gem5

#endif // __CPU_PRED_BTB_BTB_HH__
