#include "cpu/pred/btb/btb_tage.hh"

#include <algorithm>
#include <cmath>
#include <ctime>

#include "base/debug_helper.hh"
#include "base/intmath.hh"
#include "base/trace.hh"
#include "cpu/o3/dyn_inst.hh"
#include "cpu/pred/btb/stream_common.hh"
#include "debug/TAGE.hh"

namespace gem5 {

namespace branch_prediction {

namespace btb_pred{

BTBTAGE::BTBTAGE(const Params& p):
TimedBaseBTBPredictor(p),
numPredictors(p.numPredictors),
tableSizes(p.tableSizes),
tableTagBits(p.TTagBitSizes),
tablePcShifts(p.TTagPcShifts),
histLengths(p.histLengths),
maxHistLen(p.maxHistLen),
numTablesToAlloc(p.numTablesToAlloc),
tageStats(this, p.numPredictors)
{
    DPRINTF(TAGE, "BTBTAGE constructor\n");
    tageTable.resize(numPredictors);
    tableIndexBits.resize(numPredictors);
    tableIndexMasks.resize(numPredictors);
    tableTagBits.resize(numPredictors);
    tableTagMasks.resize(numPredictors);
    // baseTable.resize(2048); // need modify
    for (unsigned int i = 0; i < p.numPredictors; ++i) {
        //initialize ittage predictor
        assert(tableSizes.size() >= numPredictors);
        tageTable[i].resize(tableSizes[i]);

        tableIndexBits[i] = ceilLog2(tableSizes[i]);
        tableIndexMasks[i].resize(tableIndexBits[i], true);

        assert(histLengths.size() >= numPredictors);

        assert(tableTagBits.size() >= numPredictors);
        tableTagMasks[i].resize(tableTagBits[i], true);

        assert(tablePcShifts.size() >= numPredictors);

        tagFoldedHist.push_back(FoldedHist((int)histLengths[i], (int)tableTagBits[i], 8));
        altTagFoldedHist.push_back(FoldedHist((int)histLengths[i], (int)tableTagBits[i]-1, 8));
        indexFoldedHist.push_back(FoldedHist((int)histLengths[i], (int)tableIndexBits[i], 8));
    }
    // for (unsigned i = 0; i < baseTable.size(); ++i) {
    //     baseTable[i].resize(numBr);
    // }
    usefulResetCnt = 0;

    useAlt.resize(128);
    // for (unsigned i = 0; i < useAlt.size(); ++i) {
    //     useAlt[i].resize(numBr, 0);
    // }
    
    // enableSC = true;
    // std::vector<TageBankStats *> statsPtr;
    // for (int i = 0; i < numBr; i++) {
    //     statsPtr.push_back(tageBankStats[i]);
    // }
    // sc.setStats(statsPtr);
}

BTBTAGE::~BTBTAGE()
{
    // for (int i = 0; i < numBr; i++) {
    //     delete tageBankStats[i];
    // }
    // delete [] tageBankStats;
}

void
BTBTAGE::setTrace()
{
    if (enableDB) {
        std::vector<std::pair<std::string, DataType>> fields_vec = {
            std::make_pair("startPC", UINT64),
            std::make_pair("branchPC", UINT64),
            std::make_pair("lgcBank", UINT64),
            std::make_pair("phyBank", UINT64),
            std::make_pair("mainFound", UINT64),
            std::make_pair("mainCounter", UINT64),
            std::make_pair("mainUseful", UINT64),
            std::make_pair("altCounter", UINT64),
            std::make_pair("mainTable", UINT64),
            std::make_pair("mainIndex", UINT64),
            std::make_pair("altIndex", UINT64),
            std::make_pair("tag", UINT64),
            std::make_pair("useAlt", UINT64),
            std::make_pair("predTaken", UINT64),
            std::make_pair("actualTaken", UINT64),
            std::make_pair("allocSuccess", UINT64),
            std::make_pair("allocFailure", UINT64),
            std::make_pair("predUseSC", UINT64),
            std::make_pair("predSCDisagree", UINT64),
            std::make_pair("predSCCorrect", UINT64)
        };
        tageMissTrace = _db->addAndGetTrace("TAGEMISSTRACE", fields_vec);
        tageMissTrace->init_table();
    }
}

void
BTBTAGE::tick() {}

void
BTBTAGE::tickStart() {}

std::map<Addr, bool>
BTBTAGE::lookupHelper(const Addr &startPC, const std::vector<BTBEntry> &btbEntries)
{
    // clear old metas
    meta.preds.clear();
    // assign history for meta
    meta.tagFoldedHist = tagFoldedHist;
    meta.altTagFoldedHist = altTagFoldedHist;
    meta.indexFoldedHist = indexFoldedHist;

    DPRINTF(TAGE, "lookupHelper startAddr: %#lx\n", startPC);
    std::vector<TageEntry> entries;
    std::vector<Addr> indices, tags;
    bitset useful_mask(numPredictors, false);
    // all btb entries should use the same lookup result
    // but each btb entry can use prediction from different tables
    for (int i = 0; i < numPredictors; ++i) {
        Addr index = getTageIndex(startPC, i);
        Addr tag = getTageTag(startPC, i);
        auto &entry = tageTable[i][index];
        entries.push_back(entry);
        indices.push_back(index);
        tags.push_back(tag);
        useful_mask[i] = entry.useful;
        DPRINTF(TAGE, "lookup table %d[%d]: valid %d, tag %d, ctr %d, useful %d\n",
            i, index, entry.valid, entry.tag, entry.counter, entry.useful);
    }
    meta.usefulMask = useful_mask;

    std::vector<TagePrediction> preds;
    std::map<Addr, bool> cond_takens;
    for (auto &btb_entry : btbEntries) {
        if (btb_entry.isCond && btb_entry.valid) {
            DPRINTF(TAGE, "lookupHelper btbEntry: %#lx, always taken %d\n", btb_entry.pc, btb_entry.alwaysTaken);
            bool provided = false;
            bool alt_provided = false;

            TageTableInfo main_info, alt_info;

            for (int i = numPredictors - 1; i >= 0; --i) {
                auto &way = entries[i];
                // TODO: count alias hit (offset match but pc differs)
                bool match = way.valid && tags[i] == way.tag && btb_entry.pc == way.pc;
                DPRINTF(TAGE, "hit %d, table %d, index %d, lookup tag %d, tag %d, useful %d, btb_pc %#lx, entry_pc %#lx\n",
                    match, i, indices[i], tags[i], way.tag, way.useful, btb_entry.pc, way.pc);

                if (match) {
                    if (!provided) {
                        main_info = TageTableInfo(true, way, i, indices[i], tags[i]);
                        provided = true;
                    } else if (!alt_provided) {
                        alt_info = TageTableInfo(true, way, i, indices[i], tags[i]);
                        alt_provided = true;
                        break;
                    }
                }
            }

            bool main_taken = main_info.taken();
            bool alt_taken = alt_info.taken();
            bool base_taken = btb_entry.ctr >= 0;

            bool alt_pred = alt_provided ? alt_taken : base_taken;

            // TODO: dynamic control whether to use alt prediction
            bool use_alt = main_info.entry.counter == 0 || main_info.entry.counter == -1 || !provided;
            bool taken = use_alt ? alt_pred : main_taken;
            DPRINTF(TAGE, "tage predict %#lx taken %d\n", btb_entry.pc, taken);

            TagePrediction pred(btb_entry.pc, main_info, alt_info, use_alt, main_taken);
            meta.preds[btb_entry.pc] = pred;
            tageStats.updateStatsWithTagePrediction(pred, true);
            cond_takens[btb_entry.pc] = taken || btb_entry.alwaysTaken;
        }
    }
    return cond_takens;
}

void
BTBTAGE::putPCHistory(Addr stream_start, const bitset &history, std::vector<FullBTBPrediction> &stagePreds) {
    DPRINTF(TAGE, "putPCHistory startAddr: %#lx\n", stream_start);

    // IMPORTANT: when this function is called,
    // btb entries should already be in stagePreds
    // get prediction and save it
    for (int s = getDelay(); s < stagePreds.size(); s++) {
        // TODO: only lookup once for one btb entry in different stages
        auto &stage_pred = stagePreds[s];
        auto cond_takens = lookupHelper(stream_start, stage_pred.btbEntries);
        stage_pred.condTakens = cond_takens;
    }

}

std::shared_ptr<void>
BTBTAGE::getPredictionMeta() {
    std::shared_ptr<void> meta_void_ptr = std::make_shared<TageMeta>(meta);
    return meta_void_ptr;
}

void
BTBTAGE::update(const FetchStream &stream)
{

    Addr startAddr = stream.getRealStartPC();
    DPRINTF(TAGE, "update startAddr: %#lx\n", startAddr);
    // update at the basis of btb entries
    auto all_entries_to_update = stream.updateBTBEntries;
    // only update conditional branches that is not always taken
    auto remove_it = std::remove_if(all_entries_to_update.begin(), all_entries_to_update.end(),
        [](const BTBEntry &e) { return !e.isCond && !e.alwaysTaken; });
    all_entries_to_update.erase(remove_it, all_entries_to_update.end());

    // check if pred btb miss branch need to be updated
    auto &potential_new_entry = stream.updateNewBTBEntry;
    if (!stream.updateIsOldEntry && potential_new_entry.isCond && !potential_new_entry.alwaysTaken) {
        all_entries_to_update.push_back(potential_new_entry);
    }

    // get tage predictions and branch history info from meta
    auto meta = std::static_pointer_cast<TageMeta>(stream.predMetas[getComponentIdx()]);
    auto preds = meta->preds;
    auto updateTagFoldedHist = meta->tagFoldedHist;
    auto updateAltTagFoldedHist = meta->altTagFoldedHist;
    auto updateIndexFoldedHist = meta->indexFoldedHist;
    auto &stat = tageStats;
    // update each branch
    for (auto &btb_entry : all_entries_to_update) {
        bool this_cond_actual_taken = stream.exeTaken && stream.exeBranchInfo == btb_entry;
        auto pred_it = preds.find(btb_entry.pc);
        TagePrediction pred;
        if (pred_it != preds.end()) {
            pred = pred_it->second;
        }
        tageStats.updateStatsWithTagePrediction(pred, false);
        auto &main_info = pred.mainInfo;
        bool &main_found = main_info.found;
        auto &main_counter = main_info.entry.counter;
        bool main_taken = main_counter >= 0;
        bool main_weak  = main_counter == 0 || main_counter == -1;

        bool &used_alt = pred.useAlt;
        auto &alt_info = pred.altInfo;
        bool base_as_alt = !alt_info.found;
        bool alt_taken = base_as_alt ? btb_entry.ctr >= 0 : alt_info.entry.counter >= 0;
        bool alt_diff = main_taken != alt_taken;
        // update provider
        if (main_found) {
            DPRINTF(TAGE, "prediction provided by table %d, idx %d, updating corresponding entry\n",
                main_info.table, main_info.index);
            auto &way = tageTable[main_info.table][main_info.index];
            if (alt_diff) {
                way.useful = this_cond_actual_taken == main_taken;
            }
            DPRINTF(TAGE, "useful bit set to %d\n", way.useful);
            updateCounter(this_cond_actual_taken, 3, way.counter);
        }

        // update alt provider
        if (used_alt && !base_as_alt) {
            auto &way = tageTable[alt_info.table][alt_info.index];
            updateCounter(this_cond_actual_taken, 3, way.counter);
        }

        // base table is btb, updated in btb as well

        // TODO: use alt dynamic algorithm training
        
        // stats
        if (used_alt) {
            bool alt_correct = alt_taken == this_cond_actual_taken;
            if (alt_correct) {
                tageStats.updateUseAltCorrect++;
            } else {
                tageStats.updateUseAltWrong++;
            }
            if (alt_diff) {
                tageStats.updateAltDiffers++;
            }
        }

        // TODO: stats on use-alt-on-na
        bool this_cond_mispred = stream.squashType == SquashType::SQUASH_CTRL && stream.squashPC == btb_entry.pc;
        if (this_cond_mispred) {
            tageStats.updateMispred++;
            if (!used_alt && main_found) {
                tageStats.updateTableMispreds[main_info.table]++;
            }
        }

        bool use_alt_on_main_found_correct = used_alt && main_found && main_taken == this_cond_actual_taken;
        bool need_to_allocate = this_cond_mispred && !use_alt_on_main_found_correct;
        DPRINTF(TAGE, "this_cond_mispred %d, use_alt_on_main_found_correct %d, needToAllocate %d\n",
            this_cond_mispred, use_alt_on_main_found_correct, need_to_allocate);
        
        // useful bit reset algorithm
        std::string useful_str;
        boost::to_string(meta->usefulMask, useful_str);
        auto useful_mask = meta->usefulMask;

        DPRINTF(TAGEUseful, "useful mask: %s\n", useful_str.c_str());
        int alloc_table_num = numPredictors - (main_info.found ? main_info.table + 1 : 0);
        if (main_info.found) {
            useful_mask >>= main_info.table + 1;
            useful_mask.resize(alloc_table_num);
        }
        int num_tables_can_allocate = (~useful_mask).count();
        int total_tables_to_allocate = useful_mask.size();
        bool incUsefulResetCounter = num_tables_can_allocate < (total_tables_to_allocate - num_tables_can_allocate);
        bool decUsefulResetCounter = num_tables_can_allocate > (total_tables_to_allocate - num_tables_can_allocate);
        int changeVal = std::abs(num_tables_can_allocate - (total_tables_to_allocate - num_tables_can_allocate));
        if (need_to_allocate) {
            if (incUsefulResetCounter) { // need modify: clear the useful bit of all entries
                tageStats.updateResetUCtrInc.sample(changeVal, 1);
                usefulResetCnt += changeVal;
                if (usefulResetCnt >= 128) {
                    usefulResetCnt = 128;
                }
                // usefulResetCnt = (usefulResetCnt + changeVal >= 128) ? 128 : (usefulResetCnt + changeVal);
                DPRINTF(TAGEUseful, "incUsefulResetCounter, changeVal %d, usefulResetCnt %d\n", changeVal, usefulResetCnt);
            } else if (decUsefulResetCounter) {
                tageStats.updateResetUCtrDec.sample(changeVal, 1);
                usefulResetCnt -= changeVal;
                if (usefulResetCnt <= 0) {
                    usefulResetCnt = 0;
                }
                // usefulResetCnt = (usefulResetCnt - changeVal <= 0) ? 0 : (usefulResetCnt - changeVal);
                DPRINTF(TAGEUseful, "decUsefulResetCounter, changeVal %d, usefulResetCnt %d\n", changeVal, usefulResetCnt);
            }
            if (usefulResetCnt == 128) {
                tageStats.updateResetU++;
                DPRINTF(TAGEUseful, "reset useful bit of all entries\n");
                for (auto &table : tageTable) {
                    for (auto &entry : table) {
                        entry.useful = 0;
                    }
                }
                usefulResetCnt = 0;
            }
        }

        // allocate new entry
        bool alloc_success, alloc_failure;
        if (need_to_allocate) {
            // allocate new entry
            unsigned maskMaxNum = std::pow(2, alloc_table_num);
            unsigned mask = allocLFSR.get() % maskMaxNum;
            bitset allocateLFSR(alloc_table_num, mask);
            std::string buf;
            boost::to_string(allocateLFSR, buf);
            DPRINTF(TAGEUseful, "allocateLFSR %s, size %d\n", buf, allocateLFSR.size());
            auto flipped_usefulMask = ~useful_mask;
            boost::to_string(flipped_usefulMask, buf);
            DPRINTF(TAGEUseful, "pred usefulmask %s, size %d\n", buf, useful_mask.size());
            bitset masked = allocateLFSR & flipped_usefulMask;
            boost::to_string(masked, buf);
            DPRINTF(TAGEUseful, "masked %s, size %d\n", buf, masked.size());
            bitset allocate = masked.any() ? masked : flipped_usefulMask;
            boost::to_string(allocate, buf);
            DPRINTF(TAGEUseful, "allocate %s, size %d\n", buf, allocate.size());
            short newCounter = this_cond_actual_taken ? 0 : -1;

            bool allocateValid = flipped_usefulMask.any();
            if (allocateValid) {
                DPRINTF(TAGE, "allocate new entry\n");
                tageStats.updateAllocSuccess++;
                alloc_success = true;
                unsigned startTable = main_found ? main_info.table + 1 : 0;

                for (int ti = startTable; ti < numPredictors; ti++) {
                    Addr newIndex = getTageIndex(startAddr, ti, updateIndexFoldedHist[ti].get());
                    Addr newTag = getTageTag(startAddr, ti, updateTagFoldedHist[ti].get(), updateAltTagFoldedHist[ti].get());
                    auto &entry = tageTable[ti][newIndex];

                    if (allocate[ti - startTable]) {
                        DPRINTF(TAGE, "found allocatable entry, table %d, index %d, tag %d, counter %d\n",
                            ti, newIndex, newTag, newCounter);
                        entry = TageEntry(newTag, newCounter, btb_entry.pc);
                        break; // allocate only 1 entry
                    }
                }
            } else {
                alloc_failure = true;
                tageStats.updateAllocFailure++;
            }
        }
    }

    // auto scMeta = meta->scMeta;

        // // update use_alt_counters
        // if (pred.mainFound && mainWeak && mainTaken != altTaken) {
        //     DPRINTF(TAGE, "use_alt_on_provider_weak, alt %s, updating use_alt_counter\n",
        //         altTaken == this_cond_actually_taken ? "correct" : "incorrect");
        //     auto &use_alt_counter = useAlt.at(getUseAltIdx(startAddr))[b];
        //     if (altTaken == this_cond_actually_taken) {
        //         stat->updateUseAltOnNaInc++;
        //         satIncrement(7, use_alt_counter);
        //     } else {
        //         stat->updateUseAltOnNaDec++;
        //         satDecrement(-8, use_alt_counter);
        //     }
        // }


        // if (pred.mainFound && mainWeak) {
        //     stat->updateProviderNa++;
        //     if (!pred.useAlt) {
        //         bool mainCorrect = mainTaken == this_cond_actually_taken;
        //         if (mainCorrect) {
        //             stat->updateUseNaCorrect++;
        //         } else {
        //             stat->updateUseNaWrong++;
        //         }
        //     } else {
        //         stat->updateUseAltOnNa++;
        //         bool altCorrect = altTaken == this_cond_actually_taken;
        //         if (altCorrect) {
        //             stat->updateUseAltOnNaCorrect++;
        //         } else {
        //             stat->updateUseAltOnNaWrong++;
        //         }
        //     }
        // }

        // if (enableDB) {
        //     TageMissTrace t;
        //     t.set(startAddr, btb_entry.slots[b].pc, b, phyBrIdx, mainFound, pred.mainCounter,
        //         pred.mainUseful, pred.altCounter, pred.table, pred.index, getBaseTableIndex(startAddr),
        //         pred.tag, pred.useAlt, pred.taken, this_cond_actually_taken, allocSuccess, allocFailure,
        //         scMeta.scPreds[b].scUsed, scMeta.scPreds[b].scPred != scMeta.scPreds[b].tageTaken,
        //         scMeta.scPreds[b].scPred == this_cond_actually_taken);
        //     tageMissTrace->write_record(t);
        // }
    // }

    // update sc
    // if (enableSC) {
    //     sc.update(startAddr, scMeta, need_to_update, actualTakens);
    // }
    DPRINTF(TAGE, "end update\n");
}

void
BTBTAGE::updateCounter(bool taken, unsigned width, short &counter) {
    int max = (1 << (width-1)) - 1;
    int min = -(1 << (width-1));
    if (taken) {
        satIncrement(max, counter);
    } else {
        satDecrement(min, counter);
    }
}

Addr
BTBTAGE::getTageTag(Addr pc, int t, bitset &foldedHist, bitset &altFoldedHist)
{
    bitset buf(tableTagBits[t], pc >> floorLog2(blockSize));  // lower bits of PC
    bitset altTagBuf(altFoldedHist);
    altTagBuf.resize(tableTagBits[t]);
    altTagBuf <<= 1;
    buf ^= foldedHist;
    buf ^= altTagBuf;
    return buf.to_ulong();
}

Addr
BTBTAGE::getTageTag(Addr pc, int t)
{
    return getTageTag(pc, t, tagFoldedHist[t].get(), altTagFoldedHist[t].get());
}

Addr
BTBTAGE::getTageIndex(Addr pc, int t, bitset &foldedHist)
{
    bitset buf(tableIndexBits[t], pc >> floorLog2(blockSize));  // lower bits of PC
    buf ^= foldedHist;
    return buf.to_ulong();
}

Addr
BTBTAGE::getTageIndex(Addr pc, int t)
{
    return getTageIndex(pc, t, indexFoldedHist[t].get());
}

bool
BTBTAGE::matchTag(Addr expected, Addr found)
{
    return expected == found;
}

bool
BTBTAGE::satIncrement(int max, short &counter)
{
    if (counter < max) {
        ++counter;
    }
    return counter == max;
}

bool
BTBTAGE::satDecrement(int min, short &counter)
{
    if (counter > min) {
        --counter;
    }
    return counter == min;
}

Addr
BTBTAGE::getUseAltIdx(Addr pc) {
    return (pc >> instShiftAmt) & (useAlt.size() - 1); // need modify
}

void
BTBTAGE::doUpdateHist(const boost::dynamic_bitset<> &history, int shamt, bool taken)
{
    std::string buf;
    boost::to_string(history, buf);
    DPRINTF(TAGE, "in doUpdateHist, shamt %d, taken %d, history %s\n", shamt, taken, buf);
    if (shamt == 0) {
        DPRINTF(TAGE, "shamt is 0, returning\n");
        return;
    }

    for (int t = 0; t < numPredictors; t++) {
        for (int type = 0; type < 3; type++) {
            DPRINTF(TAGE, "t: %d, type: %d\n", t, type);

            auto &foldedHist = type == 0 ? indexFoldedHist[t] : type == 1 ? tagFoldedHist[t] : altTagFoldedHist[t];
            foldedHist.update(history, shamt, taken);
        }
    }
}

void
BTBTAGE::specUpdateHist(const boost::dynamic_bitset<> &history, FullBTBPrediction &pred)
{
    int shamt;
    bool cond_taken;
    std::tie(shamt, cond_taken) = pred.getHistInfo();
    doUpdateHist(history, shamt, cond_taken);
    // if (enableSC) {
    //     sc.doUpdateHist(history, shamt, cond_taken);
    // }
}

void
BTBTAGE::recoverHist(const boost::dynamic_bitset<> &history,
    const FetchStream &entry, int shamt, bool cond_taken)
{
    std::shared_ptr<TageMeta> predMeta = std::static_pointer_cast<TageMeta>(entry.predMetas[getComponentIdx()]);
    for (int i = 0; i < numPredictors; i++) {
        tagFoldedHist[i].recover(predMeta->tagFoldedHist[i]);
        altTagFoldedHist[i].recover(predMeta->altTagFoldedHist[i]);
        indexFoldedHist[i].recover(predMeta->indexFoldedHist[i]);
    }
    doUpdateHist(history, shamt, cond_taken);
    // if (enableSC) {
    //     sc.recoverHist(predMeta->scMeta.indexFoldedHist);
    //     sc.doUpdateHist(history, shamt, cond_taken);
    // }
}

void
BTBTAGE::checkFoldedHist(const boost::dynamic_bitset<> &hist, const char * when)
{
    // DPRINTF(TAGE, "checking folded history when %s\n", when);
    std::string hist_str;
    boost::to_string(hist, hist_str);
    // DPRINTF(TAGE, "history:\t%s\n", hist_str.c_str());
    for (int t = 0; t < numPredictors; t++) {
        for (int type = 0; type < 3; type++) {

            // DPRINTF(TAGE, "t: %d, type: %d\n", t, type);
            std::string buf2, buf3;
            auto &foldedHist = type == 0 ? indexFoldedHist[t] : type == 1 ? tagFoldedHist[t] : altTagFoldedHist[t];
            foldedHist.check(hist);
        }
    }
}

// std::vector<BTBTAGE::StatisticalCorrector::SCPrediction>
// BTBTAGE::StatisticalCorrector::getPredictions(Addr pc, std::vector<TagePrediction> &tagePreds)
// {
//     std::vector<int> scSums = {0,0};
//     std::vector<int> tageCtrCentereds;
//     std::vector<SCPrediction> scPreds;
//     std::vector<bool> sumAboveThresholds;
//     scPreds.resize(numBr);
//     sumAboveThresholds.resize(numBr);
//     for (int b = 0; b < numBr; b++) {
//         int phyBrIdx = tage->getShuffledBrIndex(pc, b);
//         std::vector<int> scOldCounters;
//         tageCtrCentereds.push_back((2 * tagePreds[b].mainCounter + 1) * 8);
//         for (int i = 0;i < scCntTable.size();i++) {
//             int index = getIndex(pc, i);
//             int tOrNt = tagePreds[b].taken ? 1 : 0;
//             int ctr = scCntTable[i][index][phyBrIdx][tOrNt];
//             scSums[b] += 2 * ctr + 1;
//         }
//         scSums[b] += tageCtrCentereds[b];
//         sumAboveThresholds[b] = abs(scSums[b]) > thresholds[b];

//         scPreds[b].tageTaken = tagePreds[b].taken;
//         scPreds[b].scUsed = tagePreds[b].mainFound;
//         scPreds[b].scPred = tagePreds[b].mainFound && sumAboveThresholds[b] ?
//             scSums[b] >= 0 : tagePreds[b].taken;
//         scPreds[b].scSum = scSums[b];

//         // stats
//         auto &stat = stats[b];
//         if (tagePreds[b].mainFound) {
//             stat->scUsedAtPred++;
//             if (sumAboveThresholds[b]) {
//                 stat->scConfAtPred++;
//                 if (scPreds[b].scPred == scPreds[b].tageTaken) {
//                     stat->scAgreeAtPred++;
//                 } else {
//                     stat->scDisagreeAtPred++;
//                 }
//             } else {
//                 stat->scUnconfAtPred++;
//             }
//         }
//     }
//     return scPreds;
// }

// std::vector<FoldedHist>
// BTBTAGE::StatisticalCorrector::getFoldedHist()
// {
//     return foldedHist;
// }

// bool
// BTBTAGE::StatisticalCorrector::satPos(int &counter, int counterBits)
// {
//     return counter == ((1 << (counterBits-1)) - 1);
// }

// bool
// BTBTAGE::StatisticalCorrector::satNeg(int &counter, int counterBits)
// {
//     return counter == -(1 << (counterBits-1));
// }

// Addr
// BTBTAGE::StatisticalCorrector::getIndex(Addr pc, int t)
// {
//     return getIndex(pc, t, foldedHist[t].get());
// }

// Addr
// BTBTAGE::StatisticalCorrector::getIndex(Addr pc, int t, bitset &foldedHist)
// {
//     bitset buf(tableIndexBits[t], pc >> tablePcShifts[t]);  // lower bits of PC
//     buf ^= foldedHist;
//     return buf.to_ulong();
// }

// void
// BTBTAGE::StatisticalCorrector::counterUpdate(int &ctr, int nbits, bool taken)
// {
//     if (taken) {
// 		if (ctr < ((1 << (nbits-1)) - 1))
// 			ctr++;
// 	} else {
// 		if (ctr > -(1 << (nbits-1)))
// 			ctr--;
//     }
// }

// void
// BTBTAGE::StatisticalCorrector::update(Addr pc, SCMeta meta, std::vector<bool> needToUpdates,
//     std::vector<bool> actualTakens)
// {
//     auto predHist = meta.indexFoldedHist;
//     auto preds = meta.scPreds;

//     for (int b = 0; b < numBr; b++) {
//         if (!needToUpdates[b]) {
//             continue;
//         }
//         int phyBrIdx = tage->getShuffledBrIndex(pc, b);
//         auto &p = preds[b];
//         bool scTaken = p.scPred;
//         bool actualTaken = actualTakens[b];
//         int tOrNt = p.tageTaken ? 1 : 0;
//         int sumAbs = std::abs(p.scSum);
//         // perceptron update
//         if (p.scUsed) {
//             if (sumAbs <= (thresholds[b] * 8 + 21) || scTaken != actualTaken) {
//                 for (int i = 0; i < numPredictors; i++) {
//                     auto idx = getIndex(pc, i, predHist[i].get());
//                     auto &ctr = scCntTable[i][idx][phyBrIdx][tOrNt];
//                     counterUpdate(ctr, scCounterWidth, actualTaken);
//                 }
//                 if (scTaken != actualTaken) {
//                     stats[b]->scUpdateOnMispred++;
//                 } else {
//                     stats[b]->scUpdateOnUnconf++;
//                 }
//             }

//             if (scTaken != p.tageTaken && sumAbs >= thresholds[b] - 4 && sumAbs <= thresholds[b] - 2) {

//                 bool cause = scTaken != actualTaken;
//                 counterUpdate(TCs[b], TCWidth, cause);
//                 if (satPos(TCs[b], TCWidth) && thresholds[b] <= maxThres) {
//                     thresholds[b] += 2;
//                 } else if (satNeg(TCs[b], TCWidth) && thresholds[b] >= minThres) {
//                     thresholds[b] -= 2;
//                 }

//                 if (satPos(TCs[b], TCWidth) || satNeg(TCs[b], TCWidth)) {
//                     TCs[b] = neutralVal;
//                 }
//             }

//             // stats
//             auto &stat = stats[b];
//             // bool sumAboveUpdateThreshold = sumAbs >= (thresholds[b] * 8 + 21);
//             bool sumAboveUseThreshold = sumAbs >= thresholds[b];
//             stat->scUsedAtCommit++;
//             if (sumAboveUseThreshold) {
//                 stat->scConfAtCommit++;
//                 if (scTaken == p.tageTaken) {
//                     stat->scAgreeAtCommit++;
//                 } else {
//                     stat->scDisagreeAtCommit++;
//                     if (scTaken == actualTaken) {
//                         stat->scCorrectTageWrong++;
//                     } else {
//                         stat->scWrongTageCorrect++;
//                     }
//                 }
//             } else {
//                 stat->scUnconfAtCommit++;
//             }
//         }

//     }
// }

// void
// BTBTAGE::StatisticalCorrector::recoverHist(std::vector<FoldedHist> &fh)
// {
//     for (int i = 0; i < numPredictors; i++) {
//         foldedHist[i].recover(fh[i]);
//     }
// }

// void
// BTBTAGE::StatisticalCorrector::doUpdateHist(const boost::dynamic_bitset<> &history,
//     int shamt, bool cond_taken)
// {
//     if (shamt == 0) {
//         return;
//     }
//     for (int t = 0; t < numPredictors; t++) {
//         foldedHist[t].update(history, shamt, cond_taken);
//     }
// }

BTBTAGE::TageStats::TageStats(statistics::Group* parent, int numPredictors):
    statistics::Group(parent),
    ADD_STAT(predTableHits, statistics::units::Count::get(), "hit of each tage table on prediction"),
    ADD_STAT(predNoHitUseBim, statistics::units::Count::get(), "use bimodal when no hit on prediction"),
    ADD_STAT(predUseAlt, statistics::units::Count::get(), "use alt on prediction"),
    ADD_STAT(updateTableHits, statistics::units::Count::get(), "hit of each tage table on update"),
    ADD_STAT(updateNoHitUseBim, statistics::units::Count::get(), "use bimodal when no hit on update"),
    ADD_STAT(updateUseAlt, statistics::units::Count::get(), "use alt on update"),
    ADD_STAT(updateUseAltCorrect, statistics::units::Count::get(), "use alt on update and correct"),
    ADD_STAT(updateUseAltWrong, statistics::units::Count::get(), "use alt on update and wrong"),
    ADD_STAT(updateAltDiffers, statistics::units::Count::get(), "alt differs on update"),
    ADD_STAT(updateUseAltOnNaUpdated, statistics::units::Count::get(), "use alt on na ctr updated when update"),
    ADD_STAT(updateUseAltOnNaInc, statistics::units::Count::get(), "use alt on na ctr inc when update"),
    ADD_STAT(updateUseAltOnNaDec, statistics::units::Count::get(), "use alt on na ctr dec when update"),
    ADD_STAT(updateProviderNa, statistics::units::Count::get(), "provider weak when update"),
    ADD_STAT(updateUseNaCorrect, statistics::units::Count::get(), "use na on update and correct"),
    ADD_STAT(updateUseNaWrong, statistics::units::Count::get(), "use na on update and wrong"),
    ADD_STAT(updateUseAltOnNa, statistics::units::Count::get(), "use alt on na when update"),
    ADD_STAT(updateUseAltOnNaCorrect, statistics::units::Count::get(), "use alt on na correct when update"),
    ADD_STAT(updateUseAltOnNaWrong, statistics::units::Count::get(), "use alt on na wrong when update"),
    ADD_STAT(updateAllocFailure, statistics::units::Count::get(), "alloc failure when update"),
    ADD_STAT(updateAllocSuccess, statistics::units::Count::get(), "alloc success when update"),
    ADD_STAT(updateMispred, statistics::units::Count::get(), "mispred when update"),
    ADD_STAT(updateResetU, statistics::units::Count::get(), "reset u when update"),
    ADD_STAT(updateResetUCtrInc, statistics::units::Count::get(), "reset u ctr inc when update"),
    ADD_STAT(updateResetUCtrDec, statistics::units::Count::get(), "reset u ctr dec when update"),
    ADD_STAT(updateTableMispreds, statistics::units::Count::get(), "mispreds of each table when update")
    // ADD_STAT(scAgreeAtPred, statistics::units::Count::get(), "sc agrees with tage on prediction"),
    // ADD_STAT(scAgreeAtCommit, statistics::units::Count::get(), "sc agrees with tage when update"),
    // ADD_STAT(scDisagreeAtPred, statistics::units::Count::get(), "sc disagrees with tage on prediction"),
    // ADD_STAT(scDisagreeAtCommit, statistics::units::Count::get(), "sc disagrees with tage when update"),
    // ADD_STAT(scConfAtPred, statistics::units::Count::get(), "sc is confident on prediction"),
    // ADD_STAT(scConfAtCommit, statistics::units::Count::get(), "sc is confident when update"),
    // ADD_STAT(scUnconfAtPred, statistics::units::Count::get(), "sc is unconfident on prediction"),
    // ADD_STAT(scUnconfAtCommit, statistics::units::Count::get(), "sc is unconfident when update"),
    // ADD_STAT(scUpdateOnMispred, statistics::units::Count::get(), "sc update because of misprediction"),
    // ADD_STAT(scUpdateOnUnconf, statistics::units::Count::get(), "sc update because of unconfidence"),
    // ADD_STAT(scUsedAtPred, statistics::units::Count::get(), "sc used on prediction"),
    // ADD_STAT(scUsedAtCommit, statistics::units::Count::get(), "sc used when update"),
    // ADD_STAT(scCorrectTageWrong, statistics::units::Count::get(), "sc correct and tage wrong when update"),
    // ADD_STAT(scWrongTageCorrect, statistics::units::Count::get(), "sc wrong and tage correct when update")
{
    predTableHits.init(0, numPredictors-1, 1);
    updateTableHits.init(0, numPredictors-1, 1);
    updateResetUCtrInc.init(1, numPredictors, 1);
    updateResetUCtrDec.init(1, numPredictors, 1);
    updateTableMispreds.init(numPredictors);
}

void
BTBTAGE::TageStats::updateStatsWithTagePrediction(const TagePrediction &pred, bool when_pred)
{
    bool hit = pred.mainInfo.found;
    unsigned hit_table = pred.mainInfo.table;
    bool useAlt = pred.useAlt;
    if (when_pred) {
        if (hit) {
            predTableHits.sample(hit_table, 1);
        } else {
            predNoHitUseBim++;
        }
        if (!hit || useAlt) {
            predUseAlt++;
        }
    } else {
        if (hit) {
            updateTableHits.sample(hit_table, 1);
        } else {
            updateNoHitUseBim++;
        }
        if (!hit || useAlt) {
            updateUseAlt++;
        }
    }
}

void
BTBTAGE::commitBranch(const FetchStream &stream, const DynInstPtr &inst)
{
}

} // namespace btb_pred

}  // namespace branch_prediction

}  // namespace gem5
