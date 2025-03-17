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


#include "base/intmath.hh"
#include "cpu/pred/btb/stream_common.hh"
#include "cpu/pred/btb/test/mockbtb.hh"

namespace gem5
{

namespace branch_prediction
{

namespace btb_pred
{

namespace test
{

/*
 * BTB Constructor
 * Initializes:
 * - BTB structure (sets and ways)
 * - MRU tracking for each set
 * - Address calculation parameters (index/tag masks and shifts)
 */
DefaultBTB::DefaultBTB(unsigned numEntries, unsigned tagBits, unsigned numWays, unsigned numDelay)
    : numEntries(numEntries),
    numWays(numWays),
    tagBits(tagBits),
    log2NumThreads(1),
    numDelay(numDelay)
{
    // for test, TODO: remove this
    alignToBlockSize = true;
    // Calculate shift amounts for index calculation
    if (alignToBlockSize) { // if aligned to blockSize, | tag | idx | block offset | instShiftAmt
        idxShiftAmt = floorLog2(blockSize);
    } else { // if not aligned to blockSize, | tag | idx | instShiftAmt
        idxShiftAmt = 1;
    }

    assert(numEntries % numWays == 0);
    numSets = numEntries / numWays;

    if (!isPowerOf2(numEntries)) {
        fatal("BTB entries is not a power of 2!");
    }

    // Initialize BTB structure and MRU tracking
    btb.resize(numSets);
    mruList.resize(numSets);
    for (unsigned i = 0; i < numSets; ++i) {
        auto &set = btb[i];
        set.resize(numWays);
        auto it = set.begin();
        for (; it != set.end(); it++) {
            it->valid = false;
            mruList[i].push_back(it);
        }
        std::make_heap(mruList[i].begin(), mruList[i].end(), older());
    }

    // | tag | idx | block offset | instShiftAmt

    idxMask = numSets - 1;

    tagMask = (1UL << tagBits) - 1;
    tagShiftAmt = idxShiftAmt + floorLog2(numSets);
}

/**
 * Process BTB entries:
 * 1. Sort entries by PC order
 * 2. Remove entries before the start PC
 */
std::vector<DefaultBTB::TickedBTBEntry>
DefaultBTB::processEntries(const std::vector<TickedBTBEntry>& entries, Addr startAddr)
{
    int hitNum = entries.size();
    bool hit = hitNum > 0;
    
    // Update prediction statistics
    if (hit) {
        DPRINTF(BTB, "BTB: lookup hit, dumping hit entry\n");
        btbStats.predHit += hitNum;
        for (auto &entry: entries) {
            printTickedBTBEntry(entry);
        }
    } else {
        btbStats.predMiss++;
        DPRINTF(BTB, "BTB: lookup miss\n");
    }

    auto processed_entries = entries;
    
    // Sort by instruction order
    std::sort(processed_entries.begin(), processed_entries.end(), 
             [](const BTBEntry &a, const BTBEntry &b) {
                 return a.pc < b.pc;
             });
    
    // Remove entries before the start PC
    auto it = std::remove_if(processed_entries.begin(), processed_entries.end(),
                           [startAddr](const BTBEntry &e) {
                               return e.pc < startAddr;
                           });
    processed_entries.erase(it, processed_entries.end());
    
    return processed_entries;
}

/**
 * Fill predictions for each pipeline stage:
 * 1. Copy BTB entries
 * 2. Set conditional branch predictions
 * 3. Set indirect branch targets
 */
void
DefaultBTB::fillStagePredictions(const std::vector<TickedBTBEntry>& entries,
                                    std::vector<FullBTBPrediction>& stagePreds)
{
    for (int s = getDelay(); s < stagePreds.size(); ++s) {
        // if (!isL0() && !hit && stagePreds[s].valid) {
        //     DPRINTF(BTB, "BTB: ubtb hit and btb miss, use ubtb result");
        //     incNonL0Stat(btbStats.predUseL0OnL1Miss);
        //     break;
        // }
        DPRINTF(BTB, "BTB: assigning prediction for stage %d\n", s);
        
        // Copy BTB entries to stage prediction
        stagePreds[s].btbEntries.clear();
        for (auto e: entries) {
            stagePreds[s].btbEntries.push_back(BTBEntry(e));
        }
        checkAscending(stagePreds[s].btbEntries);
        dumpBTBEntries(stagePreds[s].btbEntries);

        // Set predictions for each branch
        for (auto &e : entries) {
            assert(e.valid);
            if (e.isCond) {
                // TODO: a performance bug here, mbtb should not update condTakens!
                // if (isL0()) {  // only L0 BTB has saturating counter
                    // use saturating counter of L0 BTB
                    stagePreds[s].condTakens[e.pc] = e.alwaysTaken || (e.ctr >= 0);
                // } else {  // L1 BTB condTakens depends on the TAGE predictor
                // }
            } else if (e.isIndirect) {
                // Set predicted target for indirect branches
                DPRINTF(BTB, "setting indirect target for pc %#lx to %#lx\n", e.pc, e.target);
                stagePreds[s].indirectTargets[e.pc] = e.target;
            }
        }

        stagePreds[s].predTick = curTick();
    }
}

/**
 * Update metadata for later stages:
 * 1. Clear old metadata
 * 2. Save L0 BTB entries for L1 BTB's reference
 * 3. Save current BTB entries
 */
void
DefaultBTB::updatePredictionMeta(const std::vector<TickedBTBEntry>& entries,
                                   std::vector<FullBTBPrediction>& stagePreds)
{
    meta.l0_hit_entries.clear();
    meta.hit_entries.clear();
    
    // Save L0 BTB entries for L1 BTB's reference
    if (getDelay() >= 1) {
        // L0 should be zero-bubble
        meta.l0_hit_entries = stagePreds[0].btbEntries;
    }

    // Save current BTB entries
    for (auto e: entries) {
        meta.hit_entries.push_back(BTBEntry(e));
    }
}

void
DefaultBTB::putPCHistory(Addr startAddr,
                         const boost::dynamic_bitset<> &history,
                         std::vector<FullBTBPrediction> &stagePreds)
{
    // Lookup all matching entries in BTB
    auto find_entries = lookup(startAddr);
    
    // Process BTB entries
    auto processed_entries = processEntries(find_entries, startAddr);
    
    // Fill predictions for each pipeline stage
    fillStagePredictions(processed_entries, stagePreds);
    
    // Update metadata for later stages
    updatePredictionMeta(processed_entries, stagePreds);
}

std::shared_ptr<void>
DefaultBTB::getPredictionMeta()
{
    std::shared_ptr<void> meta_void_ptr = std::make_shared<BTBMeta>(meta);
    return meta_void_ptr;
}

void
DefaultBTB::specUpdateHist(const boost::dynamic_bitset<> &history, FullBTBPrediction &pred) {}

inline
Addr
DefaultBTB::getIndex(Addr instPC)
{
    // Need to shift PC over by the word offset.
    return (instPC >> idxShiftAmt) & idxMask;
}

inline
Addr
DefaultBTB::getTag(Addr instPC)
{
    return (instPC >> tagShiftAmt) & tagMask;
}

/*
 * Main lookup function
 * Returns all BTB entries that match the given PC
 * Steps:
 * 1. Calculate index and tag from PC
 * 2. Check all ways in the set for matching entries
 * 3. Update MRU information for hits
 */
std::vector<DefaultBTB::TickedBTBEntry>
DefaultBTB::lookup(Addr block_pc)
{
    std::vector<TickedBTBEntry> res;
    if (block_pc & 0x1) {
        return res; // ignore false hit when lowest bit is 1
    }
    Addr btb_idx = getIndex(block_pc);
    Addr btb_tag = getTag(block_pc);
    DPRINTF(BTB, "BTB: Looking up BTB entry index %#lx tag %#lx\n", btb_idx, btb_tag);

    assert(btb_idx < numSets);
    for (auto &way : btb[btb_idx]) {
        if (way.valid && way.tag == btb_tag) {
            res.push_back(way);
            way.tick = curTick();  // Update timestamp for MRU
            std::make_heap(mruList[btb_idx].begin(), mruList[btb_idx].end(), older());
        }
    }
    return res;
}

/*
 * Generate a new BTB entry or update an existing one based on execution results
 * 
 * This function is called during BTB update to:
 * 1. Check if the executed branch was predicted (hit in BTB)
 * 2. If hit, prepare to update the existing entry
 * 3. If miss and branch was taken:
 *    - Create a new entry
 *    - For conditional branches, initialize as always taken with counter = 1
 * 4. Set the tag and update stream metadata for later use in update()
 * 
 * Note: This is only called in L1 BTB during update
 */
void
DefaultBTB::getAndSetNewBTBEntry(FetchStream &stream)
{
    DPRINTF(BTB, "generating new btb entry\n");
    // Get prediction metadata from previous stages
    auto meta = std::static_pointer_cast<BTBMeta>(stream.predMetas[getComponentIdx()]);
    auto &predBTBEntries = meta->hit_entries;
    
    // Check if this branch was predicted (exists in BTB)
    bool pred_branch_hit = false;
    BTBEntry entry_to_write = BTBEntry();
    for (auto &e: predBTBEntries) {
        if (stream.exeBranchInfo == e) {
            pred_branch_hit = true;
            entry_to_write = e;
            break;
        }
    }
    bool is_old_entry = pred_branch_hit;

    // If branch was not predicted but was actually taken in execution, create new entry
    if (!pred_branch_hit && stream.exeTaken) {
        BTBEntry new_entry = BTBEntry(stream.exeBranchInfo);
        new_entry.valid = true;
        // For conditional branches, initialize as always taken
        if (new_entry.isCond) {
            new_entry.alwaysTaken = true;
            new_entry.ctr = 1;  // Start with positive prediction
        }
        entry_to_write = new_entry;
        is_old_entry = false;
    } else {
        // Existing entries will be updated in update()
    }

    // Set tag and update stream metadata for use in update()
    entry_to_write.tag = getTag(entry_to_write.pc);
    stream.updateNewBTBEntry = entry_to_write;
    stream.updateIsOldEntry = is_old_entry;
}

/**
 * Process old BTB entries from prediction metadata
 * 1. Get prediction metadata
 * 2. Remove entries that were not executed
 */
std::vector<BTBEntry>
DefaultBTB::processOldEntries(const FetchStream &stream)
{
    auto meta = std::static_pointer_cast<BTBMeta>(stream.predMetas[getComponentIdx()]);
    // hit entries whose corresponding insts are acutally executed
    Addr end_inst_pc = stream.updateEndInstPC;
    DPRINTF(BTB, "end_inst_pc: %#lx\n", end_inst_pc);
    // remove not executed btb entries, pc > end_inst_pc
    auto old_entries = meta->hit_entries;
    DPRINTF(BTB, "old_entries.size(): %lu\n", old_entries.size());
    dumpBTBEntries(old_entries);
    auto remove_it = std::remove_if(old_entries.begin(), old_entries.end(),
        [end_inst_pc](const BTBEntry &e) { return e.pc > end_inst_pc; });
    old_entries.erase(remove_it, old_entries.end());
    DPRINTF(BTB, "after removing not executed insts, old_entries.size(): %lu\n", old_entries.size());
    dumpBTBEntries(old_entries);

    btbStats.updateHit += old_entries.size();
    
    return old_entries;
}

/**
 * Check if the branch was predicted correctly
 * Also check L0 BTB prediction status
 */
std::pair<bool, bool>
DefaultBTB::checkPredictionHit(const FetchStream &stream, const BTBMeta* meta)
{
    bool pred_branch_hit = false;
    for (auto &e : meta->hit_entries) {
        if (stream.exeBranchInfo == e) {
            pred_branch_hit = true;
            break;
        }
    }
    if (!pred_branch_hit && stream.exeTaken) {
        DPRINTF(BTB, "update miss detected, pc %#lx, predTick %lu\n", stream.exeBranchInfo.pc, stream.predTick);
        btbStats.updateMiss++;
    }

    // Check if L0 BTB had a hit but L1 BTB missed
    bool pred_l0_branch_hit = false;
    for (auto &e : meta->l0_hit_entries) {
        if (stream.exeBranchInfo == e) {
            pred_l0_branch_hit = true;
            break;
        }
    }
    if (!isL0()) {
        bool l0_hit_l1_miss = pred_l0_branch_hit && !pred_branch_hit;
        if (l0_hit_l1_miss) {
            DPRINTF(BTB, "BTB: skipping entry write because of l0 hit\n");
            incNonL0Stat(btbStats.updateUseL0OnL1Miss);
            // return;
        }
    }
    return std::make_pair(pred_branch_hit, pred_l0_branch_hit);
}


/**
 * Collect all entries that need to be updated
 * 1. Process old entries
 * 2. Add new entry if necessary
 */
std::vector<BTBEntry>
DefaultBTB::collectEntriesToUpdate(const std::vector<BTBEntry>& old_entries,
                                     const FetchStream &stream)
{
    auto all_entries = old_entries;
    if (!stream.updateIsOldEntry || isL0()) { // L0 BTB always updates
        all_entries.push_back(stream.updateNewBTBEntry);
    }
    DPRINTF(BTB, "all_entries_to_update.size(): %lu\n", all_entries.size());
    dumpBTBEntries(all_entries);
    return all_entries;
}

/**
 * Update or replace BTB entry
 * 1. Look for matching entry
 * 2. for cond entry, if found, use the one in btb, since we need the up-to-date counter
 * 3. for indirect entry, update target if necessary
 * 4. Update existing entry or replace oldest entry
 * 5. Update MRU information
 */
void
DefaultBTB::updateBTBEntry(unsigned btb_idx, const BTBEntry& entry, const FetchStream &stream)
{
    // Look for matching entry
    bool found = false;
    auto it = btb[btb_idx].begin();
    for (; it != btb[btb_idx].end(); it++) {
        if (*it == entry) {
            found = true;
            break;
        }
    }
    // if cond entry in btb now, use the one in btb, since we need the up-to-date counter
    // else use the recorded entry
    auto entry_to_write = entry.isCond && found ? BTBEntry(*it) : entry;
    // update saturating counter if necessary
    if (entry_to_write.isCond) {
        bool this_cond_taken = stream.exeTaken && stream.getControlPC() == entry_to_write.pc;
        if (!this_cond_taken) {
            entry_to_write.alwaysTaken = false;
        }
        // if (isL0()) {  // only L0 BTB has saturating counter
            updateCtr(entry_to_write.ctr, this_cond_taken);
        // }
    }
    // update indirect target if necessary
    if (entry_to_write.isIndirect && stream.exeTaken && stream.getControlPC() == entry_to_write.pc) {
        entry_to_write.target = stream.exeBranchInfo.target;
    }
    auto ticked_entry = TickedBTBEntry(entry_to_write, curTick());
    if (found) {
        // Update existing entry
        *it = ticked_entry;
    } else {
        // Replace oldest entry in the set
        DPRINTF(BTB, "trying to replace entry in set %d\n", btb_idx);
        dumpMruList(mruList[btb_idx]);
        // put the oldest entry in this set to the back of heap
        std::pop_heap(mruList[btb_idx].begin(), mruList[btb_idx].end(), older());
        const auto& entry_in_btb_now = mruList[btb_idx].back();
        *entry_in_btb_now = ticked_entry;
    }
    std::make_heap(mruList[btb_idx].begin(), mruList[btb_idx].end(), older());
}

/*
 * Update BTB with execution results
 * Steps:
 * 1. Get old entries that were hit during prediction
 * 2. Remove entries that were not actually executed
 * 3. Update statistics
 * 4. Update existing entries or create new ones
 * 5. Update MRU information
 */
void
DefaultBTB::update(const FetchStream &stream)
{
    // 1. Process old entries
    auto old_entries = processOldEntries(stream);
    
    // 2. Check prediction hit status, for stats recording
    auto [pred_hit, l0_hit] = checkPredictionHit(stream, 
        std::static_pointer_cast<BTBMeta>(stream.predMetas[getComponentIdx()]).get());
    
    // Record statistics for L0 hit but L1 miss case
    if (!isL0()) {
        bool l0_hit_l1_miss = l0_hit && !pred_hit;
        if (l0_hit_l1_miss) {
            // DPRINTF(BTB, "BTB: skipping entry write because of l0 hit\n");
            // incNonL0Stat(btbStats.updateUseL0OnL1Miss);
        }
    }
    
    // 3. Calculate BTB index and tag
    Addr startPC = stream.getRealStartPC();
    Addr btb_idx = getIndex(startPC);
    Addr btb_tag = getTag(startPC);
    
    // 4. Collect entries to update
    auto entries_to_update = collectEntriesToUpdate(old_entries, stream);
    
    // 5. Update BTB entries
    for (auto &entry : entries_to_update) {
        entry.tag = btb_tag;
        updateBTBEntry(btb_idx, entry, stream);
    }
    
    assert(btb_idx < numSets);
    assert(btb[btb_idx].size() <= numWays);
    assert(mruList[btb_idx].size() <= numWays);
}

} // namespace test
} // namespace btb_pred
} // namespace branch_prediction
} // namespace gem5
