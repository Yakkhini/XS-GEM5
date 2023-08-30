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
#include "base/trace.hh"
#include "cpu/o3/dyn_inst.hh"
#include "cpu/pred/btb/btb.hh"
#include "debug/Fetch.hh"

namespace gem5
{

namespace branch_prediction
{

namespace btb_pred
{

DefaultBTB::DefaultBTB(const Params &p)
    : TimedBaseBTBPredictor(p),
    numEntries(p.numEntries),
    tagBits(p.tagBits),
    idxShiftAmt(floorLog2(p.blockSize)),
    log2NumThreads(floorLog2(p.numThreads)),
    numWays(p.numWays),
    numDelay(p.numDelay),
    btbStats(this)
{
    assert(numEntries % numWays == 0);
    numSets = numEntries / numWays;

    if (!isPowerOf2(numEntries)) {
        fatal("BTB entries is not a power of 2!");
    }

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
    DPRINTF(BTB, "numEntries %d, numSets %d, numWays %d, tagBits %d, tagShiftAmt %d, idxMask %#lx, tagMask %#lx\n",
        numEntries, numSets, numWays, tagBits, tagShiftAmt, idxMask, tagMask);
}

void
DefaultBTB::tickStart()
{
    // nothing to do
}

void
DefaultBTB::tick() {}

void
DefaultBTB::putPCHistory(Addr startAddr,
                         const boost::dynamic_bitset<> &history,
                         std::vector<FullBTBPrediction> &stagePreds)
{
    auto find_entries = lookup(startAddr);
    int hitNum = find_entries.size();
    bool hit = hitNum > 0;
    if (hit) {
        DPRINTF(BTB, "BTB: lookup hit, dumping hit entry\n");
        btbStats.predHit += hitNum;
        for (auto &entry: find_entries) {
            printTickedBTBEntry(entry);
        }
    } else {
        btbStats.predMiss++;
        DPRINTF(BTB, "BTB: lookup miss\n");
    }
    assert(getDelay() < stagePreds.size());
    // sort entries by inst order
    std::sort(find_entries.begin(), find_entries.end(), [](const BTBEntry &a, const BTBEntry &b) {
            return a.pc < b.pc;
    });
    dumpTickedBTBEntries(find_entries);
    // remove entries that are before startPC
    auto it = std::remove_if(find_entries.begin(), find_entries.end(), [startAddr](const BTBEntry &e) {
            return e.pc < startAddr;
    });
    find_entries.erase(it, find_entries.end());
    dumpTickedBTBEntries(find_entries);
    // assign prediction for s2 and later stages
    for (int s = getDelay(); s < stagePreds.size(); ++s) {
        // if (!isL0() && !hit && stagePreds[s].valid) {
        //     DPRINTF(BTB, "BTB: ubtb hit and btb miss, use ubtb result");
        //     incNonL0Stat(btbStats.predUseL0OnL1Miss);
        //     break;
        // }
        DPRINTF(BTB, "BTB: assigning prediction for stage %d\n", s);
        stagePreds[s].btbEntries.clear();
        for (auto e: find_entries) {
            stagePreds[s].btbEntries.push_back(BTBEntry(e));
        }
        checkAscending(stagePreds[s].btbEntries);
        dumpBTBEntries(stagePreds[s].btbEntries);

        for (auto &e : find_entries) {
            assert(e.valid);
            // other branches are defaultly predicted not taken
            if (e.isCond) {
                stagePreds[s].condTakens[e.pc] = e.alwaysTaken || (e.ctr >= 0);
            } else if (e.isIndirect) {
                DPRINTF(BTB, "setting indirect target for pc %#lx to %#lx\n", e.pc, e.target);
                stagePreds[s].indirectTargets[e.pc] = e.target;
            }
        }

        stagePreds[s].predTick = curTick();
    }

    meta.l0_hit_entries.clear();
    meta.hit_entries.clear();
    if (getDelay() >= 1) {
        // l0 should be zero-bubble
        meta.l0_hit_entries = stagePreds[0].btbEntries;
    }

    for (auto e: find_entries) {
        meta.hit_entries.push_back(BTBEntry(e));
    }
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

// @todo Create some sort of return struct that has both whether or not the
// address is valid, and also the address.  For now will just use addr = 0 to
// represent invalid entry.
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
            way.tick = curTick();
            std::make_heap(mruList[btb_idx].begin(), mruList[btb_idx].end(), older());
        }
    }
    return res;
}

void
DefaultBTB::getAndSetNewBTBEntry(FetchStream &stream)
{
    DPRINTF(BTB, "generating new btb entry\n");
    // generate btb entry
    auto meta = std::static_pointer_cast<BTBMeta>(stream.predMetas[getComponentIdx()]);
    auto &predBTBEntries = meta->hit_entries;
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
    // modify this specific btb entry that is responsible for the stream (in exeBranchInfo)
    if (!pred_branch_hit && stream.exeTaken) {
        BTBEntry new_entry = BTBEntry(stream.exeBranchInfo);
        new_entry.valid = true;
        if (new_entry.isCond) {
            new_entry.alwaysTaken = true;
            new_entry.ctr = 1;
        }
        entry_to_write = new_entry;
        is_old_entry = false;
    } else {
        // old entries are modified in update
    }
    entry_to_write.tag = getTag(entry_to_write.pc);
    stream.updateNewBTBEntry = entry_to_write;
    stream.updateIsOldEntry = is_old_entry;

}

void
DefaultBTB::update(const FetchStream &stream)
{
    auto meta = std::static_pointer_cast<BTBMeta>(stream.predMetas[getComponentIdx()]);
    // hit entries whose corresponding insts are acutally executed
    Addr end_inst_pc = stream.updateEndInstPC;
    DPRINTF(BTB, "end_inst_pc: %#lx\n", end_inst_pc);
    // remove not executed btb entries
    auto old_entries_to_update = meta->hit_entries;
    DPRINTF(BTB, "old_entries_to_update.size(): %d\n", old_entries_to_update.size());
    dumpBTBEntries(old_entries_to_update);
    auto remove_it = std::remove_if(old_entries_to_update.begin(), old_entries_to_update.end(),
        [end_inst_pc](const BTBEntry &e) { return e.pc > end_inst_pc; });
    old_entries_to_update.erase(remove_it, old_entries_to_update.end());
    DPRINTF(BTB, "after removing not executed insts, old_entries_to_update.size(): %d\n", old_entries_to_update.size());
    dumpBTBEntries(old_entries_to_update);

    btbStats.updateHit += old_entries_to_update.size();
    bool pred_branch_hit = false;
    for (auto &e : meta->hit_entries) {
        if (stream.exeBranchInfo == e) {
            pred_branch_hit = true;
            break;
        }
    }
    if (!pred_branch_hit && stream.exeTaken) {
        DPRINTF(BTB, "update miss detected, pc %#lx, predTick %llu\n", stream.exeBranchInfo.pc, stream.predTick);
        btbStats.updateMiss++;
    }
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
    Addr startPC = stream.getRealStartPC();
    Addr btb_idx = getIndex(startPC);
    Addr btb_tag = getTag(startPC);


    auto all_entries_to_update = old_entries_to_update;
    if (!stream.updateIsOldEntry || isL0()) {
        all_entries_to_update.push_back(stream.updateNewBTBEntry);
    }
    DPRINTF(BTB, "all_entries_to_update.size(): %d\n", all_entries_to_update.size());
    dumpBTBEntries(all_entries_to_update);

    for (auto &e : all_entries_to_update) {
        DPRINTF(BTB, "BTB: Updating BTB entry index %#lx tag %#lx, branch_pc %#lx\n", btb_idx, e.tag, e.pc);
        // check if it is in btb now
        bool found = false;
        auto it = btb[btb_idx].begin();
        for (; it != btb[btb_idx].end(); it++) {
            // if pc match, tag and offset should all match
            if (*it == e) {
                found = true;
                break;
            }
        }
        // if cond entry in btb now, use the one in btb, since we need the up-to-date counter
        // else use the recorded entry
        auto entry_to_write = e.isCond && found ? BTBEntry(*it) : e;
        // update saturating counter if necessary
        if (entry_to_write.isCond) {
            bool this_cond_taken = stream.exeTaken && stream.getControlPC() == entry_to_write.pc;
            if (!this_cond_taken) {
                entry_to_write.alwaysTaken = false;
            }
            updateCtr(entry_to_write.ctr, this_cond_taken);
        }
        if (entry_to_write.isIndirect && stream.exeTaken && stream.getControlPC() == entry_to_write.pc) {
            entry_to_write.target = stream.exeBranchInfo.target;
        }
        auto ticked_entry_to_write = TickedBTBEntry(entry_to_write, curTick());
        ticked_entry_to_write.tag = btb_tag;
        // write into btb
        if (found) {
            *it = ticked_entry_to_write;
        } else {
            DPRINTF(BTB, "trying to replace entry in set %#lx\n", btb_idx);
            dumpMruList(mruList[btb_idx]);
            // put the oldest entry in this set to the back of heap
            std::pop_heap(mruList[btb_idx].begin(), mruList[btb_idx].end(), older());
            const auto& entry_in_btb_now = mruList[btb_idx].back();
            DPRINTF(BTB, "BTB: Replacing entry with tag %#lx, pc %#lx in set %#lx\n",
                entry_in_btb_now->tag, entry_in_btb_now->pc, btb_idx);
            *entry_in_btb_now = ticked_entry_to_write;
            dumpMruList(mruList[btb_idx]);
        }
        std::make_heap(mruList[btb_idx].begin(), mruList[btb_idx].end(), older());
    }
    assert(btb_idx < numSets);
    assert(btb[btb_idx].size() <= numWays);
    assert(mruList[btb_idx].size() <= numWays);

    // btb[btb_idx].valid = true;
    // set(btb[btb_idx].target, target);
    // btb[btb_idx].tag = getTag(block_pc);
}

void
DefaultBTB::commitBranch(const FetchStream &stream, const DynInstPtr &inst)
{
    auto meta = std::static_pointer_cast<BTBMeta>(stream.predMetas[getComponentIdx()]);
    auto &hit_entries = meta->hit_entries;
    auto pc = inst->getPC();
    auto npc = inst->getNPC();
    // auto &static_inst = inst->staticInst();
    bool this_branch_hit = false;
    auto entry = BTBEntry();
    for (auto e : hit_entries) {
        if (e.pc == pc) {
            this_branch_hit = true;
            entry = e;
            break;
        }
    }
    // bool this_branch_miss = !this_branch_hit;
    bool cond_not_taken = inst->isCondCtrl() && !inst->branching();
    bool this_branch_taken = stream.exeTaken && stream.getControlPC() == pc; // all uncond should be taken
    Addr this_branch_target = npc;
    if (this_branch_hit) {
        btbStats.allBranchHits++;
        if (this_branch_taken) {
            btbStats.allBranchHitTakens++;
        } else {
            btbStats.allBranchHitNotTakens++;
        }
        if (inst->isCondCtrl()) {
            btbStats.condHits++;
            if (this_branch_taken) {
                btbStats.condHitTakens++;
            } else {
                btbStats.condHitNotTakens++;
            }
            // if (isL0()) {
                bool pred_taken = entry.ctr >= 0;
                if (pred_taken == this_branch_taken) {
                    btbStats.condPredCorrect++;
                } else {
                    btbStats.condPredWrong++;
                }
            // }
        }
        if (inst->isUncondCtrl()) {
            btbStats.uncondHits++;
        }
        // ignore non-speculative branches (e.g. syscall)
        if (!inst->isNonSpeculative()) {
            if (inst->isIndirectCtrl()) {
                btbStats.indirectHits++;
                Addr pred_target = entry.target;
                if (pred_target == this_branch_target) {
                    btbStats.indirectPredCorrect++;
                } else {
                    btbStats.indirectPredWrong++;
                }
            }
            if (inst->isCall()) {
                btbStats.callHits++;
            }
            if (inst->isReturn()) {
                btbStats.returnHits++;
            }
        }
    } else {
        btbStats.allBranchMisses++;
        if (this_branch_taken) {
            btbStats.allBranchMissTakens++;
        } else {
            btbStats.allBranchMissNotTakens++;
        }
        if (inst->isCondCtrl()) {
            btbStats.condMisses++;
            if (this_branch_taken) {
                btbStats.condMissTakens++;
                // if (isL0()) {
                    // only L0 BTB has saturating counters to predict conditional branches
                    // taken branches that is missed in btb must have been mispredicted
                    btbStats.condPredWrong++;
                // }
            } else {
                btbStats.condMissNotTakens++;
                // if (isL0()) {
                    // only L0 BTB has saturating counters to predict conditional branches
                    // taken branches that is missed in btb must have been mispredicted
                    btbStats.condPredCorrect++;
                // }
            }
        }
        if (inst->isUncondCtrl()) {
            btbStats.uncondMisses++;
        }
        // ignore non-speculative branches (e.g. syscall)
        if (!inst->isNonSpeculative()) {
            if (inst->isIndirectCtrl()) {
                btbStats.indirectMisses++;
                btbStats.indirectPredWrong++;
            }
            if (inst->isCall()) {
                btbStats.callMisses++;
            }
            if (inst->isReturn()) {
                btbStats.returnMisses++;
            }
        }
    }
}

DefaultBTB::BTBStats::BTBStats(statistics::Group* parent) :
    statistics::Group(parent),
    ADD_STAT(newEntry, statistics::units::Count::get(), "number of new btb entries generated"),
    ADD_STAT(newEntryWithCond, statistics::units::Count::get(), "number of new btb entries generated with conditional branch"),
    ADD_STAT(newEntryWithUncond, statistics::units::Count::get(), "number of new btb entries generated with unconditional branch"),
    ADD_STAT(oldEntry, statistics::units::Count::get(), "number of old btb entries updated"),
    ADD_STAT(oldEntryIndirectTargetModified, statistics::units::Count::get(), "number of old btb entries with indirect target modified"),
    ADD_STAT(oldEntryWithNewCond, statistics::units::Count::get(), "number of old btb entries with new conditional branches"),
    ADD_STAT(oldEntryWithNewUncond, statistics::units::Count::get(), "number of old btb entries with new unconditional branches"),
    ADD_STAT(predMiss, statistics::units::Count::get(), "misses encountered on prediction"),
    ADD_STAT(predHit, statistics::units::Count::get(), "hits encountered on prediction"),
    ADD_STAT(updateMiss, statistics::units::Count::get(), "misses encountered on update"),
    ADD_STAT(updateHit, statistics::units::Count::get(), "hits encountered on update"),
    ADD_STAT(eraseSlotBehindUncond, statistics::units::Count::get(), "erase slots behind unconditional slot"),
    ADD_STAT(predUseL0OnL1Miss, statistics::units::Count::get(), "use l0 result on l1 miss when pred"),
    ADD_STAT(updateUseL0OnL1Miss, statistics::units::Count::get(), "use l0 result on l1 miss when update"),

    ADD_STAT(allBranchHits, statistics::units::Count::get(), "all types of branches committed that was predicted hit"),
    ADD_STAT(allBranchHitTakens, statistics::units::Count::get(), "all types of taken branches committed was that predicted hit"),
    ADD_STAT(allBranchHitNotTakens, statistics::units::Count::get(), "all types of not taken branches committed was that predicted hit"),
    ADD_STAT(allBranchMisses, statistics::units::Count::get(), "all types of branches committed that was predicted miss"),
    ADD_STAT(allBranchMissTakens, statistics::units::Count::get(), "all types of taken branches committed was that predicted miss"),
    ADD_STAT(allBranchMissNotTakens, statistics::units::Count::get(), "all types of not taken branches committed was that predicted miss"),
    ADD_STAT(condHits, statistics::units::Count::get(), "conditional branches committed that was predicted hit"),
    ADD_STAT(condHitTakens, statistics::units::Count::get(), "taken conditional branches committed was that predicted hit"),
    ADD_STAT(condHitNotTakens, statistics::units::Count::get(), "not taken conditional branches committed was that predicted hit"),
    ADD_STAT(condMisses, statistics::units::Count::get(), "conditional branches committed that was predicted miss"),
    ADD_STAT(condMissTakens, statistics::units::Count::get(), "taken conditional branches committed was that predicted miss"),
    ADD_STAT(condMissNotTakens, statistics::units::Count::get(), "not taken conditional branches committed was that predicted miss"),
    ADD_STAT(condPredCorrect, statistics::units::Count::get(), "conditional branches committed was that correctly predicted by btb"),
    ADD_STAT(condPredWrong, statistics::units::Count::get(), "conditional branches committed was that mispredicted by btb"),
    ADD_STAT(uncondHits, statistics::units::Count::get(), "unconditional branches committed that was predicted hit"),
    ADD_STAT(uncondMisses, statistics::units::Count::get(), "unconditional branches committed that was predicted miss"),
    ADD_STAT(indirectHits, statistics::units::Count::get(), "indirect branches committed that was predicted hit"),
    ADD_STAT(indirectMisses, statistics::units::Count::get(), "indirect branches committed that was predicted miss"),
    ADD_STAT(indirectPredCorrect, statistics::units::Count::get(), "indirect branches committed whose target was correctly predicted by btb"),
    ADD_STAT(indirectPredWrong, statistics::units::Count::get(), "indirect branches committed whose target was mispredicted by btb"),
    ADD_STAT(callHits, statistics::units::Count::get(), "calls committed that was predicted hit"),
    ADD_STAT(callMisses, statistics::units::Count::get(), "calls committed that was predicted miss"),
    ADD_STAT(returnHits, statistics::units::Count::get(), "returns committed that was predicted hit"),
    ADD_STAT(returnMisses, statistics::units::Count::get(), "returns committed that was predicted miss")

{
    auto btb = dynamic_cast<branch_prediction::btb_pred::DefaultBTB*>(parent);
    // do not need counter below in L0 btb
    if (btb->isL0()) {
        predUseL0OnL1Miss.prereq(predUseL0OnL1Miss);
        updateUseL0OnL1Miss.prereq(updateUseL0OnL1Miss);
        newEntry.prereq(newEntry);
        newEntryWithCond.prereq(newEntryWithCond);
        newEntryWithUncond.prereq(newEntryWithUncond);
        oldEntry.prereq(oldEntry);
        oldEntryIndirectTargetModified.prereq(oldEntryIndirectTargetModified);
        oldEntryWithNewCond.prereq(oldEntryWithNewCond);
        oldEntryWithNewUncond.prereq(oldEntryWithNewUncond);
        eraseSlotBehindUncond.prereq(eraseSlotBehindUncond);
    }
}

} // namespace btb_pred
} // namespace branch_prediction
} // namespace gem5
