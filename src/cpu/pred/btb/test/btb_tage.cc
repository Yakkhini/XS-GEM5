#include "cpu/pred/btb/test/btb_tage.hh"

// #include <algorithm>
// #include <cmath>
// #include <ctime>

// #include "base/debug_helper.hh"
// #include "base/intmath.hh"
// // #include "base/trace.hh"
// #include "cpu/o3/dyn_inst.hh"
// #include "cpu/pred/btb/stream_common.hh"
// #include "debug/TAGE.hh"

namespace gem5 {

namespace branch_prediction {

namespace btb_pred{

namespace test
{

// Constructor: Initialize TAGE predictor with given parameters
BTBTAGE::BTBTAGE() : numPredictors(4), tageStats(4)
{
    DPRINTF(TAGE, "BTBTAGE constructor\n");
    tableSizes = {1024, 1024, 1024, 1024};
    tableTagBits = {10, 10, 10, 10};
    tablePcShifts = {10, 10, 10, 10};
    histLengths = {10, 10, 10, 10};
    maxHistLen = 10;
    numTablesToAlloc = 4;
    enableSC = false;

    tageTable.resize(numPredictors);
    tableIndexBits.resize(numPredictors);
    tableIndexMasks.resize(numPredictors);
    tableTagBits.resize(numPredictors);
    tableTagMasks.resize(numPredictors);
    // baseTable.resize(2048); // need modify
    for (unsigned int i = 0; i < numPredictors; ++i) {
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
}

BTBTAGE::~BTBTAGE()
{
}

void
BTBTAGE::tick() {}

void
BTBTAGE::tickStart() {}

/**
 * @brief Helper function to look up predictions in TAGE tables for a stream of instructions
 * 
 * This function performs the core prediction logic of the TAGE predictor:
 * 1. Looks up entries in all TAGE tables using the start PC
 * 2. For each conditional branch, finds the main and alternative predictions
 * 3. Determines the final prediction based on prediction confidence
 * 4. Manages the useful bit mask for entry allocation
 * 
 * @param startPC The starting PC address for the instruction stream
 * @param btbEntries Vector of BTB entries to make predictions for
 * @return Map of branch PC addresses to their predicted outcomes (taken/not taken)
 */
std::map<Addr, bool>
BTBTAGE::lookupHelper(const Addr &startPC, const std::vector<BTBEntry> &btbEntries)
{
    // Step 1: Clear old prediction metadata and save current history state
    meta.preds.clear();
    // assign history for meta
    meta.tagFoldedHist = tagFoldedHist;
    meta.altTagFoldedHist = altTagFoldedHist;
    meta.indexFoldedHist = indexFoldedHist;

    DPRINTF(TAGE, "lookupHelper startAddr: %#lx\n", startPC);
    
    // Step 2: Look up all TAGE tables for the given start PC
    std::vector<TageEntry> entries;
    std::vector<Addr> indices, tags;
    bitset useful_mask(numPredictors, false);
    // all btb entries should use the same lookup result
    // but each btb entry can use prediction from different tables
    
    // For each TAGE table, calculate index and tag, then look up the entry
    for (int i = 0; i < numPredictors; ++i) {
        Addr index = getTageIndex(startPC, i);
        Addr tag = getTageTag(startPC, i);
        auto &entry = tageTable[i][index];
        entries.push_back(entry);
        indices.push_back(index);
        tags.push_back(tag);
        useful_mask[i] = entry.useful;
        DPRINTF(TAGE, "lookup table %d[%lu]: valid %d, tag %lu, ctr %d, useful %d\n",
            i, index, entry.valid, entry.tag, entry.counter, entry.useful);
    }
    meta.usefulMask = useful_mask;

    // Step 3: Process each BTB entry to make predictions
    std::vector<TagePrediction> preds;
    std::map<Addr, bool> cond_takens;
    
    for (auto &btb_entry : btbEntries) {
        // Only predict for valid conditional branches
        if (btb_entry.isCond && btb_entry.valid) {
            DPRINTF(TAGE, "lookupHelper btbEntry: %#lx, always taken %d\n", 
                btb_entry.pc, btb_entry.alwaysTaken);
            
            // Step 3.1: Find main and alternative predictions
            bool provided = false;
            bool alt_provided = false;
            TageTableInfo main_info, alt_info;

            // Search from highest to lowest table for matches
            for (int i = numPredictors - 1; i >= 0; --i) {
                auto &way = entries[i];
                // TODO: count alias hit (offset match but pc differs)
                // Check if entry matches (valid, tag matches, and PC matches)
                bool match = way.valid && tags[i] == way.tag && btb_entry.pc == way.pc;
                DPRINTF(TAGE, "hit %d, table %d, index %lu, lookup tag %lu, tag %lu, useful %d, btb_pc %#lx, entry_pc %#lx\n",
                    match, i, indices[i], tags[i], way.tag, way.useful, btb_entry.pc, way.pc);

                if (match) {
                    if (!provided) {
                        // First match becomes main prediction
                        main_info = TageTableInfo(true, way, i, indices[i], tags[i]);
                        provided = true;
                    } else if (!alt_provided) {
                        // Second match becomes alternative prediction
                        alt_info = TageTableInfo(true, way, i, indices[i], tags[i]);
                        alt_provided = true;
                        break;
                    }
                }
            }

            // Step 3.2: Determine final prediction
            bool main_taken = main_info.taken();
            bool alt_taken = alt_info.taken();
            bool base_taken = btb_entry.ctr >= 0;

            // Use alternative prediction if available, otherwise use base prediction
            bool alt_pred = alt_provided ? alt_taken : base_taken;

            // TODO: dynamic control whether to use alt prediction
            // Step 3.3: Decide whether to use alternative prediction
            // Use alternative if main prediction is weak (counter = 0 or -1) or no main prediction
            bool use_alt = main_info.entry.counter == 0 || main_info.entry.counter == -1 || !provided;
            bool taken = use_alt ? alt_pred : main_taken;
            DPRINTF(TAGE, "tage predict %#lx taken %d\n", btb_entry.pc, taken);
            // print the prediction logic: taken = use_alt ? alt_provided ? alt_taken : base_taken : main_taken
            DPRINTF(TAGE, "tage use_alt %d ? alt_provided %d ? alt_taken %d : base_taken %d: main_taken %d\n",
                use_alt, alt_provided, alt_taken, base_taken, main_taken);

            // Step 3.4: Save prediction and update statistics
            TagePrediction pred(btb_entry.pc, main_info, alt_info, use_alt, taken);
            meta.preds[btb_entry.pc] = pred;
            tageStats.updateStatsWithTagePrediction(pred, true);
            // Consider branch as taken if either predicted taken or always taken
            cond_takens[btb_entry.pc] = taken || btb_entry.alwaysTaken;
        }
    }
    return cond_takens;
}

/**
 * @brief Makes predictions for a stream of instructions using TAGE predictor
 * 
 * This function is called during the prediction stage and:
 * 1. Uses lookupHelper to get predictions for all BTB entries
 * 2. Stores predictions in the stage prediction structure
 * 3. Handles multiple prediction stages with different delays
 * 
 * @param stream_start Starting PC of the instruction stream
 * @param history Current branch history
 * @param stagePreds Vector of predictions for different pipeline stages
 */
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

/**
 * @brief Updates the TAGE predictor state based on actual branch execution results
 * 
 * This function performs several key operations:
 * 1. Updates prediction counters based on actual branch outcomes
 * 2. Updates useful bits for entries that made correct predictions
 * 3. Manages the allocation of new entries in case of mispredictions
 * 4. Implements the useful bit reset mechanism to prevent predictor saturation
 * 
 * The update process follows these main steps:
 * - Processes each conditional branch in the fetch stream
 * - Updates main and alternative prediction tables
 * - Handles misprediction cases by allocating new entries
 * - Manages useful bits and implements periodic reset mechanism
 * 
 * @param stream The fetch stream containing branch execution information
 *               including actual outcomes and prediction metadata
 */
void
BTBTAGE::update(const FetchStream &stream)
{
    // Step 1: Get the starting address and prepare entries to update
    Addr startAddr = stream.getRealStartPC();
    DPRINTF(TAGE, "update startAddr: %#lx\n", startAddr);
    // update at the basis of btb entries
    // Get all BTB entries that need to be updated
    auto all_entries_to_update = stream.updateBTBEntries;
    // only update conditional branches that is not always taken
    // Filter out non-conditional branches and always-taken branches
    auto remove_it = std::remove_if(all_entries_to_update.begin(), all_entries_to_update.end(),
        [](const BTBEntry &e) { return !e.isCond && !e.alwaysTaken; });
    all_entries_to_update.erase(remove_it, all_entries_to_update.end());

    // Step 2: Handle potential new BTB entries
    auto &potential_new_entry = stream.updateNewBTBEntry;
    if (!stream.updateIsOldEntry && potential_new_entry.isCond && !potential_new_entry.alwaysTaken) {
        all_entries_to_update.push_back(potential_new_entry);
    }

    // Step 3: Get prediction metadata and history information
    auto meta = std::static_pointer_cast<TageMeta>(stream.predMetas[getComponentIdx()]);
    auto preds = meta->preds;
    auto updateTagFoldedHist = meta->tagFoldedHist;
    auto updateAltTagFoldedHist = meta->altTagFoldedHist;
    auto updateIndexFoldedHist = meta->indexFoldedHist;
    auto &stat = tageStats;

    // Step 4: Update each branch prediction
    for (auto &btb_entry : all_entries_to_update) {
        // Step 4.1: Get actual branch outcome and prediction
        bool this_cond_actual_taken = stream.exeTaken && stream.exeBranchInfo == btb_entry;
        auto pred_it = preds.find(btb_entry.pc);
        TagePrediction pred;
        if (pred_it != preds.end()) {
            pred = pred_it->second;
        }
        tageStats.updateStatsWithTagePrediction(pred, false);

        // Step 4.2: Analyze main prediction
        auto &main_info = pred.mainInfo;
        bool &main_found = main_info.found;
        auto &main_counter = main_info.entry.counter;
        bool main_taken = main_counter >= 0;
        bool main_weak  = main_counter == 0 || main_counter == -1;

        // Step 4.3: Analyze alternative prediction
        bool &used_alt = pred.useAlt;
        auto &alt_info = pred.altInfo;
        bool base_as_alt = !alt_info.found;
        bool alt_taken = base_as_alt ? btb_entry.ctr >= 0 : alt_info.entry.counter >= 0;
        bool alt_diff = main_taken != alt_taken;

        // Step 4.4: Update main prediction provider
        if (main_found) {
            DPRINTF(TAGE, "prediction provided by table %d, idx %lu, updating corresponding entry\n",
                main_info.table, main_info.index);
            auto &way = tageTable[main_info.table][main_info.index];
            
            // Update useful bit if predictions differ from alt and the prediction is correct
            if (alt_diff) {
                way.useful = this_cond_actual_taken == main_taken;
            }
            DPRINTF(TAGE, "useful bit set to %d\n", way.useful);
            
            // Update prediction counter
            updateCounter(this_cond_actual_taken, 3, way.counter);
        }

        // Step 4.5: Update alternative prediction provider
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
        // Step 4.6: Handle mispredictions and allocation
        bool this_cond_mispred = stream.squashType == SquashType::SQUASH_CTRL && stream.squashPC == btb_entry.pc;
        if (this_cond_mispred) {
            tageStats.updateMispred++;
            if (!used_alt && main_found) {
                tageStats.updateTableMispreds[main_info.table]++;
            }
        }

        // Step 4.7: Determine if new entry allocation is needed
        bool use_alt_on_main_found_correct = used_alt && main_found && main_taken == this_cond_actual_taken;
        bool need_to_allocate = this_cond_mispred && !use_alt_on_main_found_correct;
        DPRINTF(TAGE, "this_cond_mispred %d, use_alt_on_main_found_correct %d, needToAllocate %d\n",
            this_cond_mispred, use_alt_on_main_found_correct, need_to_allocate);
        
        // Step 4.8: Handle useful bit reset algorithm
        std::string useful_str;
        boost::to_string(meta->usefulMask, useful_str);
        auto useful_mask = meta->usefulMask;

        DPRINTF(TAGEUseful, "useful mask: %s\n", useful_str.c_str());
        int alloc_table_num = numPredictors - (main_info.found ? main_info.table + 1 : 0); // number of tables can allocate
        if (main_info.found) {
            useful_mask >>= main_info.table + 1;
            useful_mask.resize(alloc_table_num);
        }
        int num_tables_can_allocate = (~useful_mask).count();
        int total_tables_to_allocate = useful_mask.size();
        bool incUsefulResetCounter = num_tables_can_allocate < (total_tables_to_allocate - num_tables_can_allocate);
        bool decUsefulResetCounter = num_tables_can_allocate > (total_tables_to_allocate - num_tables_can_allocate);
        int changeVal = std::abs(num_tables_can_allocate - (total_tables_to_allocate - num_tables_can_allocate));
        
        // Step 4.9: Update useful reset counter and reset useful bits if needed
        if (need_to_allocate) {
            if (incUsefulResetCounter) { // need modify: clear the useful bit of all entries
                // tageStats.updateResetUCtrInc.sample(changeVal, 1);
                usefulResetCnt += changeVal;
                if (usefulResetCnt >= 128) {
                    usefulResetCnt = 128;
                }
                // usefulResetCnt = (usefulResetCnt + changeVal >= 128) ? 128 : (usefulResetCnt + changeVal);
                DPRINTF(TAGEUseful, "incUsefulResetCounter, changeVal %d, usefulResetCnt %d\n", changeVal, usefulResetCnt);
            } else if (decUsefulResetCounter) {
                // tageStats.updateResetUCtrDec.sample(changeVal, 1);
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

        // Step 4.10: Allocate new entries if needed
        bool alloc_success, alloc_failure;
        if (need_to_allocate) {
            // Generate allocation mask using LFSR
            unsigned maskMaxNum = std::pow(2, alloc_table_num);
            unsigned mask = allocLFSR.get() % maskMaxNum;
            bitset allocateLFSR(alloc_table_num, mask);
            
            // Debug output for allocation process
            std::string buf;
            boost::to_string(allocateLFSR, buf);
            DPRINTF(TAGEUseful, "allocateLFSR %s, size %lu\n", buf.c_str(), allocateLFSR.size());
            auto flipped_usefulMask = ~useful_mask;
            boost::to_string(flipped_usefulMask, buf);
            DPRINTF(TAGEUseful, "pred flipped usefulmask %s, size %lu\n", buf.c_str(), useful_mask.size());
            bitset masked = allocateLFSR & flipped_usefulMask;
            boost::to_string(masked, buf);
            DPRINTF(TAGEUseful, "masked %s, size %lu\n", buf.c_str(), masked.size());
            bitset allocate = masked.any() ? masked : flipped_usefulMask;
            boost::to_string(allocate, buf);
            DPRINTF(TAGEUseful, "allocate %s, size %lu\n", buf.c_str(), allocate.size());
            
            // Initialize new counter based on actual outcome
            short newCounter = this_cond_actual_taken ? 0 : -1;

            // Step 4.11: Perform allocation if possible
            bool allocateValid = flipped_usefulMask.any();
            if (allocateValid) {
                DPRINTF(TAGE, "allocate new entry\n");
                tageStats.updateAllocSuccess++;
                alloc_success = true;
                unsigned startTable = main_found ? main_info.table + 1 : 0; // start from the table after the main prediction table

                // Try to allocate in each table according to allocation mask
                for (int ti = startTable; ti < numPredictors; ti++) {
                    Addr newIndex = getTageIndex(startAddr, ti, updateIndexFoldedHist[ti].get());
                    Addr newTag = getTageTag(startAddr, ti, updateTagFoldedHist[ti].get(), updateAltTagFoldedHist[ti].get());
                    auto &entry = tageTable[ti][newIndex];

                    if (allocate[ti - startTable]) {
                        DPRINTF(TAGE, "found allocatable entry, table %d, index %lu, tag %lu, counter %d\n",
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

    DPRINTF(TAGE, "end update\n");
}

// Update prediction counter with saturation
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

// Calculate TAGE tag with folded history
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

/**
 * @brief Updates branch history for speculative execution
 * 
 * This function updates three types of folded histories:
 * - Tag folded history: Used for tag computation
 * - Alternative tag folded history: Used for alternative tag computation
 * - Index folded history: Used for table index computation
 * 
 * @param history The current branch history
 * @param shamt The number of bits to shift
 * @param taken Whether the branch was taken
 */
void
BTBTAGE::doUpdateHist(const boost::dynamic_bitset<> &history, int shamt, bool taken)
{
    std::string buf;
    boost::to_string(history, buf);
    DPRINTF(TAGE, "in doUpdateHist, shamt %d, taken %d, history %s\n", shamt, taken, buf.c_str());
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

/**
 * @brief Updates branch history for speculative execution
 * 
 * This function updates the branch history for speculative execution
 * based on the provided history and prediction information.
 * 
 * It first retrieves the history information from the prediction metadata
 * and then calls the doUpdateHist function to update the folded histories.
 * 
 * @param history The current branch history
 * @param pred The prediction metadata containing history information
 */
void
BTBTAGE::specUpdateHist(const boost::dynamic_bitset<> &history, FullBTBPrediction &pred)
{
    int shamt;
    bool cond_taken;
    std::tie(shamt, cond_taken) = pred.getHistInfo();
    doUpdateHist(history, shamt, cond_taken);
}

/**
 * @brief Recovers branch history state after a misprediction
 * 
 * This function:
 * 1. Restores the folded histories from the saved metadata
 * 2. Updates the histories with the correct branch outcome
 * 3. Ensures predictor state is consistent after recovery
 * 
 * @param history The branch history to recover to
 * @param entry The fetch stream entry containing recovery information
 * @param shamt Number of bits to shift in history update
 * @param cond_taken The actual branch outcome
 */
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
}

// Check folded history after speculative update and recovery
void
BTBTAGE::checkFoldedHist(const boost::dynamic_bitset<> &hist, const char * when)
{
    DPRINTF(TAGE, "checking folded history when %s\n", when);
    std::string hist_str;
    boost::to_string(hist, hist_str);
    DPRINTF(TAGE, "history:\t%s\n", hist_str.c_str());
    for (int t = 0; t < numPredictors; t++) {
        for (int type = 0; type < 3; type++) {

            // DPRINTF(TAGE, "t: %d, type: %d\n", t, type);
            std::string buf2, buf3;
            auto &foldedHist = type == 0 ? indexFoldedHist[t] : type == 1 ? tagFoldedHist[t] : altTagFoldedHist[t];
            foldedHist.check(hist);
        }
    }
}

BTBTAGE::TageStats::TageStats(int numPredictors) : 
    predTableHits(0),
    predNoHitUseBim(0),
    predUseAlt(0),
    updateTableHits(0),
    updateNoHitUseBim(0),
    updateUseAlt(0),
    updateUseAltCorrect(0),
    updateUseAltWrong(0),
    updateAltDiffers(0),
    updateUseAltOnNaUpdated(0),
    updateUseAltOnNaInc(0),
    updateUseAltOnNaDec(0),
    updateProviderNa(0),
    updateUseNaCorrect(0),
    updateUseNaWrong(0),
    updateUseAltOnNa(0),
    updateUseAltOnNaCorrect(0),
    updateUseAltOnNaWrong(0),
    updateAllocFailure(0),
    updateAllocSuccess(0),
    updateMispred(0),
    updateResetU(0),
    updateResetUCtrInc(0),
    updateResetUCtrDec(0),
    numPredictors(numPredictors) {
    updateTableMispreds.resize(numPredictors, 0);
}


// Update statistics based on TAGE prediction
void
BTBTAGE::TageStats::updateStatsWithTagePrediction(const TagePrediction &pred, bool when_pred)
{
    bool hit = pred.mainInfo.found;
    unsigned hit_table = pred.mainInfo.table;
    bool useAlt = pred.useAlt;
    if (when_pred) {
        if (hit) {
            // predTableHits.sample(hit_table, 1);
        } else {
            predNoHitUseBim++;
        }
        if (!hit || useAlt) {
            predUseAlt++;
        }
    } else {
        if (hit) {
            // updateTableHits.sample(hit_table, 1);
        } else {
            updateNoHitUseBim++;
        }
        if (!hit || useAlt) {
            updateUseAlt++;
        }
    }
}

// void
// BTBTAGE::commitBranch(const FetchStream &stream, const DynInstPtr &inst)
// {
// }

} // namespace test

} // namespace btb_pred

}  // namespace branch_prediction

}  // namespace gem5
