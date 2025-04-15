#include "cpu/pred/btb/test/decoupled_bpred.hh"

#include "cpu/pred/btb/test/test_dprintf.hh"

// #include "base/output.hh"
#include "base/debug_helper.hh"


namespace gem5
{
namespace branch_prediction
{
namespace btb_pred
{
namespace test
{

DecoupledBPUWithBTB::DecoupledBPUWithBTB()
      : fetchTargetQueue(20),
      fetchStreamQueueSize(20),
      historyBits(128), // TODO: for test!
      ubtb(new DefaultBTB(32, 38, 32, 0, true)),
      btb(new DefaultBTB(2048, 20, 8, 1, true)),
      tage(new BTBTAGE()),
      numStages(3),
      historyManager(16), // TODO: fix this
      dbpBtbStats()
{
    numStages = 3;
    // TODO: better impl (use vector to assign in python)
    // problem: btb->getAndSetNewBTBEntry
    components.push_back(ubtb);
    // components.push_back(uras);
    components.push_back(btb);
    components.push_back(tage);
    // components.push_back(ras);
    // components.push_back(ittage);
    numComponents = components.size();

    predsOfEachStage.resize(numStages);
    for (unsigned i = 0; i < numStages; i++) {
        predsOfEachStage[i].predSource = i;
        clearPreds();
    }

    s0PC = 0x80000000;

    s0History.resize(historyBits, 0);
    fetchTargetQueue.setName("fetchTargetQueue");

    commitHistory.resize(historyBits, 0);
    squashing = true;
}

void
DecoupledBPUWithBTB::tick()
{
    // 1. Monitor FSQ size for statistics
    // dbpBtbStats.fsqEntryDist.sample(fetchStreamQueue.size(), 1);
    if (streamQueueFull()) {
        dbpBtbStats.fsqFullCannotEnq++;
        DPRINTF(Override, "FSQ is full (%lu entries)\n", fetchStreamQueue.size());
    }

    // 2. Handle pending prediction if available
    if (!receivedPred && numOverrideBubbles == 0 && sentPCHist) {
        DPRINTF(Override, "Generating final prediction for PC %#lx\n", s0PC);
        generateFinalPredAndCreateBubbles();
    }

    // 3. Process enqueue operations and bubble counter
    processEnqueueAndBubbles();

    // 4. Request new prediction if needed
    requestNewPrediction();

    DPRINTF(Override, "Prediction cycle complete\n");

    // 5. Clear squashing state for next cycle
    squashing = false;
}

/**
 * @brief Processes prediction enqueue operations and bubble counter
 *
 * Tries to enqueue new predictions if not squashing and decrements override bubbles
 */
void
DecoupledBPUWithBTB::processEnqueueAndBubbles()
{
    // Try to enqueue new predictions if not squashing
    if (!squashing) {
        DPRINTF(Override, "DecoupledBPUWithBTB::tick()\n");
        tryEnqFetchTarget();
        tryEnqFetchStream();
    } else {
        receivedPred = false;
        DPRINTF(Override, "Squashing, skip this cycle, receivedPred is %d.\n", receivedPred);
    }

    // Decrement override bubbles counter
    if (numOverrideBubbles > 0) {
        numOverrideBubbles--;
        dbpBtbStats.overrideBubbleNum++;
        DPRINTF(Override, "Consuming override bubble, %d remaining\n", numOverrideBubbles);
    }

    sentPCHist = false;
}

/**
 * @brief Requests new predictions from predictor components
 *
 * If no prediction is in progress and FSQ has space, requests new predictions
 * from each predictor component by sending the current PC and history
 */
void
DecoupledBPUWithBTB::requestNewPrediction()
{
    // Request new prediction if FSQ not full and not using loop buffer
    if (!receivedPred && !streamQueueFull()) {
        DPRINTF(Override, "Requesting new prediction for PC %#lx\n", s0PC);

        // Initialize prediction state for each stage
        for (int i = 0; i < numStages; i++) {
            predsOfEachStage[i].bbStart = s0PC;
        }

        // Query each predictor component with current PC and history
        for (int i = 0; i < numComponents; i++) {
            components[i]->putPCHistory(s0PC, s0History, predsOfEachStage);
        }

        // Mark that we've sent PC and history to predictors
        sentPCHist = true;
    }
}

void DecoupledBPUWithBTB::OverrideStats(OverrideReason overrideReason)
{
    if (numOverrideBubbles > 0) {
        dbpBtbStats.overrideCount++;
        
        // Track specific override reasons for statistics
        switch (overrideReason) {
            case OverrideReason::validity:
                dbpBtbStats.overrideValidityMismatch++;
                break;
            case OverrideReason::controlAddr:
                dbpBtbStats.overrideControlAddrMismatch++;
                break;
            case OverrideReason::target:
                dbpBtbStats.overrideTargetMismatch++;
                break;
            case OverrideReason::end:
                dbpBtbStats.overrideEndMismatch++;
                break;
            case OverrideReason::histInfo:
                dbpBtbStats.overrideHistInfoMismatch++;
                break;
            default:
                break;
        }
    }
}

// this function collects predictions from all stages and generate bubbles
// when loop buffer is active, predictions are from saved stream
void
DecoupledBPUWithBTB::generateFinalPredAndCreateBubbles()
{
    DPRINTF(Override, "In generateFinalPredAndCreateBubbles().\n");

    // 1. Debug output: dump predictions from all stages
    for (int i = 0; i < numStages; i++) {
        printFullBTBPrediction(predsOfEachStage[i]);
    }

    // 2. Select the most accurate prediction (prioritize later stages)
    // Initially assume stage 0 (UBTB) prediction
    FullBTBPrediction *chosenPrediction = &predsOfEachStage[0];

    // Search from last stage to first for valid predictions
    for (int i = (int)numStages - 1; i >= 0; i--) {
        if (predsOfEachStage[i].btbEntries.size() > 0) {
            chosenPrediction = &predsOfEachStage[i];
            DPRINTF(Override, "Selected prediction from stage %d\n", i);
            break;
        }
    }

    // Store the chosen prediction as our final prediction
    finalPred = *chosenPrediction;

    // 3. Calculate override bubbles needed for pipeline consistency
    // Override bubbles are needed when earlier stages predict differently from later stages
    unsigned first_hit_stage = 0;
    OverrideReason overrideReason = OverrideReason::no_override;

    // Find first stage that matches the chosen prediction
    while (first_hit_stage < numStages - 1) {
        auto [matches, reason] = predsOfEachStage[first_hit_stage].match(*chosenPrediction);
        if (matches) {
            break;
        }
        first_hit_stage++;
        overrideReason = reason;
    }

    // 4. Record override bubbles and update statistics
    numOverrideBubbles = first_hit_stage;
    OverrideStats(overrideReason);

    // 5. Finalize prediction process
    finalPred.predSource = first_hit_stage;
    receivedPred = true;

    // Debug output for final prediction
    printFullBTBPrediction(finalPred);
    // dbpBtbStats.predsOfEachStage[first_hit_stage]++;

    // Clear stage predictions for next cycle
    clearPreds();

    DPRINTF(Override, "Prediction complete: override bubbles=%d, receivedPred=true\n", 
            numOverrideBubbles);
}

bool
DecoupledBPUWithBTB::trySupplyFetchWithTarget(Addr fetch_demand_pc, bool &fetch_target_in_loop)
{
    return fetchTargetQueue.trySupplyFetchWithTarget(fetch_demand_pc, fetch_target_in_loop);
}

/**
 * @brief Interface between fetch stage and branch predictor for instruction prediction
 *
 * This function is called by the fetch stage to get branch prediction information
 * for the current fetch address. It checks the Fetch Target Queue (FTQ) for available
 * prediction entries and returns the prediction result for the current PC.
 *
 * Key operations:
 * 1. Check if a prediction is available in the FTQ
 * 2. Compare current PC against the prediction entry's PC range
 * 3. Determine if current instruction is a predicted branch
 * 4. Update PC for taken branches or advance to next instruction for not-taken
 * 5. Handle entry dequeuing when prediction stream is exhausted
 *
 * @param inst The static instruction being fetched
 * @param seqNum The sequence number of the instruction
 * @param pc Current program counter (updated with prediction target if taken)
 * @param tid Thread ID
 * @param currentLoopIter Reference to current loop iteration counter
 *
 * @return A pair containing:
 *   - First value: Whether the branch is predicted taken
 *   - Second value: Whether we've exhausted the current FTQ entry
 */
std::pair<bool, bool>
DecoupledBPUWithBTB::decoupledPredict(const StaticInstPtr &inst,
                               const InstSeqNum &seqNum, PCStateBase &pc,
                               ThreadID tid, unsigned &currentLoopIter)
{
    DPRINTF(DecoupleBP, "looking up pc %#lx, Supplying target ID %lu\n",
        pc.instAddr(), fetchTargetQueue.getSupplyingTargetId());

    // Check if fetch target queue has prediction available
    auto target_avail = fetchTargetQueue.fetchTargetAvailable();
    if (!target_avail) {
        DPRINTF(DecoupleBP,
                "No ftq entry to fetch, return dummy prediction\n");
        // Return (not taken, exhausted entry) to indicate no valid prediction
        return std::make_pair(false, true);
    }

    // Get current prediction entry from FTQ
    const auto &target_to_fetch = fetchTargetQueue.getTarget();
    DPRINTF(DecoupleBP, "Responsing fetch with");
    printFetchTarget(target_to_fetch, "");
    // Extract prediction entry information
    auto start = target_to_fetch.startPC;    // Start of basic block
    auto end = target_to_fetch.endPC;        // End of basic block
    auto taken_pc = target_to_fetch.takenPC; // Branch instruction address if taken
    // Verify current PC is within the predicted entry range
    assert(start <= pc.instAddr() && pc.instAddr() < end);

    // Check if current PC matches predicted branch address and is taken
    bool taken = pc.instAddr() == taken_pc && target_to_fetch.taken;
    bool run_out_of_this_entry = false;
    // Clone PC for potential updates
    std::unique_ptr<PCStateBase> target(pc.clone());

    // Handle taken prediction by updating PC to target address
    if (taken) {
        // auto &rtarget = target->as<GenericISA::PCStateWithNext>();
        // rtarget.pc(target_to_fetch.target);   // Set new PC to predicted target

        // // Set next PC (NPC) for pipeline logic
        // rtarget.npc(target_to_fetch.target + 4);
        // rtarget.uReset();

        // DPRINTF(DecoupleBP,
        //         "Predicted pc: %#lx, upc: %u, npc(meaningless): %#lx, instSeqNum: %lu\n",
        //         target->instAddr(), rtarget.upc(), rtarget.npc(), seqNum);

        // // Update passed-in PC reference with prediction
        // set(pc, *target);

        run_out_of_this_entry = true;
    } else {
        // For not-taken branches or non-branches, advance to next instruction
        inst->advancePC(*target);

        // Check if we've reached the end of the current basic block
        if (target->instAddr() >= end) {
            run_out_of_this_entry = true;
        }
    }

    // Increment instruction counter for current FTQ entry
    currentFtqEntryInstNum++;
    if (run_out_of_this_entry) {
        processFetchTargetCompletion(target_to_fetch);
    }

    DPRINTF(DecoupleBP, "Predict it %staken to %#lx\n", taken ? "" : "not ",
            target->instAddr());

    return std::make_pair(taken, run_out_of_this_entry);
}

/**
 * @brief Process the completion of a fetch target queue entry
 *
 * This function handles the logic when a fetch target queue entry is exhausted:
 * - Dequeues the entry from FTQ
 * - Updates instruction count statistics in the corresponding FSQ entry
 * - Resets instruction counter for the next FTQ entry
 *
 * @param target_to_fetch The FTQ entry being completed
 */
void
DecoupledBPUWithBTB::processFetchTargetCompletion(const FtqEntry &target_to_fetch)
{
    DPRINTF(DecoupleBP, "running out of ftq entry %lu with %d insts\n",
            fetchTargetQueue.getSupplyingTargetId(), currentFtqEntryInstNum);

    // Remove the current entry from FTQ
    fetchTargetQueue.finishCurrentFetchTarget();

    // Get stream ID for the current fetch target
    const auto fsqId = target_to_fetch.fsqID;
    // Update instruction count in the fetch stream entry
    auto it = fetchStreamQueue.find(fsqId);
    assert(it != fetchStreamQueue.end());
    it->second.fetchInstNum = currentFtqEntryInstNum;

    // Reset instruction counter for next FTQ entry
    currentFtqEntryInstNum = 0;
}

/**
 * @brief Common logic for handling squash events
 *
 * This function encapsulates the shared logic between different types of squashes:
 * - Setting squashing state
 * - Finding and updating the stream
 * - Recovering history information
 * - Clearing predictions
 * - Updating FTQ and FSQ state
 *
 * @param target_id ID of the target being squashed
 * @param stream_id ID of the stream being squashed
 * @param squash_type Type of squash (CTRL/OTHER/TRAP)
 * @param squash_pc PC where the squash occurred
 * @param redirect_pc PC to redirect to after squash
 * @param is_conditional Whether the squash is caused by a conditional branch
 * @param actually_taken Whether the branch was actually taken (for conditional branches)
 * @param static_inst Static instruction pointer (for control squash)
 * @param control_inst_size Size of the control instruction (for control squash)
 */
void
DecoupledBPUWithBTB::handleSquash(unsigned target_id,
                                 unsigned stream_id,
                                 SquashType squash_type,
                                 const PCStateBase &squash_pc,
                                 Addr redirect_pc,
                                 bool is_conditional,
                                 bool actually_taken,
                                 const StaticInstPtr &static_inst,
                                 unsigned control_inst_size)
{
    // Set squashing state
    squashing = true;

    // Find the stream being squashed
    auto stream_it = fetchStreamQueue.find(stream_id);
    if (stream_it == fetchStreamQueue.end()) {
        assert(!fetchStreamQueue.empty());
        DPRINTF(DecoupleBP, "The squashing stream is insane, ignore squash on it");
        return;
    }

    // Get reference to the stream
    auto &stream = stream_it->second;

    // Update stream state
    stream.resolved = true;
    stream.exeTaken = actually_taken;
    stream.squashPC = squash_pc.instAddr();
    stream.squashType = squash_type;

    // Special handling for control squash - create branch info
    if (squash_type == SQUASH_CTRL && static_inst) {
        // Use full branch info with static_inst if available
        // stream.exeBranchInfo = BranchInfo(squash_pc.instAddr(), redirect_pc, static_inst, control_inst_size);
        dumpFsq("Before control squash");
    }

    // Remove streams after the squashed one
    squashStreamAfter(stream_id);

    // Recover history using the extracted function
    recoverHistoryForSquash(stream, stream_id, squash_pc, is_conditional, actually_taken, squash_type);

    // Clear predictions for next cycle
    clearPreds();

    // Update PC and stream ID
    s0PC = redirect_pc;
    fsqId = stream_id + 1;

    // Squash fetch target queue and redirect to new PC
    fetchTargetQueue.squash(target_id + 1, fsqId, redirect_pc);

    // Additional debugging for control squash
    if (squash_type == SQUASH_CTRL) {
        fetchTargetQueue.dump("After control squash");
    }

    DPRINTF(DecoupleBP,
            "After squash, FSQ head Id=%lu, s0pc=%#lx, demand stream Id=%lu, "
            "Fetch demanded target Id=%lu\n",
            fsqId, s0PC, fetchTargetQueue.getEnqState().streamId,
            fetchTargetQueue.getSupplyingTargetId());
}

void
DecoupledBPUWithBTB::controlSquash(unsigned target_id, unsigned stream_id,
                            const PCStateBase &control_pc,
                            const PCStateBase &corr_target,
                            const StaticInstPtr &static_inst,
                            unsigned control_inst_size, bool actually_taken,
                            const InstSeqNum &seq, ThreadID tid,
                            const unsigned &currentLoopIter, const bool fromCommit)
{
    dbpBtbStats.controlSquash++;

    // Get branch type information
    bool is_conditional = static_inst->isCondCtrl();
    bool is_indirect = static_inst->isIndirectCtrl();

    auto stream_it = fetchStreamQueue.find(stream_id);
    if (stream_it == fetchStreamQueue.end()) {
        DPRINTF(DecoupleBP, "The squashing stream is insane, ignore squash on it");
        return;
    }
    auto &stream = stream_it->second;
    // Get target address
    Addr real_target = corr_target.instAddr();
    // if (!fromCommit && static_inst->isReturn() && !static_inst->isNonSpeculative()) {
    //     // get ret addr from ras meta
    //     real_target = ras->getTopAddrFromMetas(stream);
    //     // TODO: set real target to dynamic inst
    // }

    // Detailed debugging for control squash
    DPRINTF(DecoupleBP,
            "Control squash: ftq_id=%d, fsq_id=%d,"
            " control_pc=%#lx, real_target=%#lx, is_conditional=%u, "
            "is_indirect=%u, actually_taken=%u, branch seq: %lu\n",
            target_id, stream_id, control_pc.instAddr(),
            real_target, is_conditional, is_indirect,
            actually_taken, seq);

    // Call shared squash handling logic
    handleSquash(target_id, stream_id, SQUASH_CTRL, control_pc,
                real_target, is_conditional, actually_taken, static_inst, control_inst_size);
}

void
DecoupledBPUWithBTB::nonControlSquash(unsigned target_id, unsigned stream_id,
                               const PCStateBase &inst_pc,
                               const InstSeqNum seq, ThreadID tid, const unsigned &currentLoopIter)
{
    dbpBtbStats.nonControlSquash++;
    DPRINTF(DecoupleBP,
            "non control squash: target id: %d, stream id: %d, inst_pc: %#lx, "
            "seq: %lu\n",
            target_id, stream_id, inst_pc.instAddr(), seq);

    // Call shared squash handling logic
    handleSquash(target_id, stream_id, SQUASH_OTHER, inst_pc, inst_pc.instAddr());
}

void
DecoupledBPUWithBTB::trapSquash(unsigned target_id, unsigned stream_id,
                         Addr last_committed_pc, const PCStateBase &inst_pc,
                         ThreadID tid, const unsigned &currentLoopIter)
{
    dbpBtbStats.trapSquash++;
    DPRINTF(DecoupleBP,
            "Trap squash: target id: %d, stream id: %d, inst_pc: %#lx\n",
            target_id, stream_id, inst_pc.instAddr());

    // Call shared squash handling logic
    handleSquash(target_id, stream_id, SQUASH_TRAP, inst_pc, inst_pc.instAddr());
}

void DecoupledBPUWithBTB::update(unsigned stream_id, ThreadID tid)
{
    // No need to dequeue when queue is empty
    if (fetchStreamQueue.empty())
        return;

    auto it = fetchStreamQueue.begin();

    // Process all streams that have been committed (stream_id >= stream's id)
    while (it != fetchStreamQueue.end() && stream_id >= it->first) {
        auto &stream = it->second;

        DPRINTF(DecoupleBP,
            "Commit stream start %#lx, which is predicted, "
            "final br addr: %#lx, final target: %#lx, pred br addr: %#lx, "
            "pred target: %#lx\n",
            stream.startPC,
            stream.exeBranchInfo.pc, stream.exeBranchInfo.target,
            stream.predBranchInfo.pc, stream.predBranchInfo.target);

        // Update statistics
        updateStatistics(stream);

        // Update predictor components
        updatePredictorComponents(stream);

        it = fetchStreamQueue.erase(it);
        dbpBtbStats.fsqEntryCommitted++;
    }

    DPRINTF(DecoupleBP, "after commit stream, fetchStreamQueue size: %lu\n",
            fetchStreamQueue.size());

    if (it != fetchStreamQueue.end()) {
        printStream(it->second);
    }

    historyManager.commit(stream_id);
}

void
DecoupledBPUWithBTB::updateStatistics(const FetchStream &stream)
{
    // Check if this stream was mispredicted
    bool miss_predicted = stream.squashType == SQUASH_CTRL;
    // Track indirect mispredictions
    if (miss_predicted && stream.exeBranchInfo.isIndirect) {
        topMispredIndirect[stream.startPC]++;
    }

    // --- BTB Statistics ---
    if (stream.isHit) {
        // Count BTB hits
        dbpBtbStats.btbHit++;
    } else {
        // Count BTB misses for taken branches
        if (stream.exeTaken) {
            dbpBtbStats.btbMiss++;
            DPRINTF(BTB, "BTB miss detected when update, stream start %#lx, predTick %lu, printing branch info:\n",
                    stream.startPC, stream.predTick);
            auto &slot = stream.exeBranchInfo;
            DPRINTF(BTB, "    pc:%#lx, size:%d, target:%#lx, cond:%d, indirect:%d, call:%d, return:%d\n",
                slot.pc, slot.size, slot.target, slot.isCond, slot.isIndirect, slot.isCall, slot.isReturn);
        }

        // Count false hits
        if (stream.falseHit) {
            dbpBtbStats.commitFalseHit++;
        }
    }

    if (stream.isHit || stream.exeTaken) {
        // Update BTB entry statistics
        auto it = totalBTBEntries.find(stream.startPC);
        if (it == totalBTBEntries.end()) {
            auto &btb_entry = stream.updateNewBTBEntry;
            totalBTBEntries[stream.startPC] = std::make_pair(btb_entry, 1);
            dbpBtbStats.btbEntriesWithDifferentStart++;
        } else {
            it->second.second++;
            it->second.first = stream.updateNewBTBEntry;
        }
    }

}

void
DecoupledBPUWithBTB::updatePredictorComponents(FetchStream &stream)
{
    // Update predictor components only if the stream is hit or taken
    if (stream.isHit || stream.exeTaken) {
        // Prepare stream for update
        stream.setUpdateInstEndPC(predictWidth);
        stream.setUpdateBTBEntries();

        // only mbtb can generate new entry
        btb->getAndSetNewBTBEntry(stream);

        // Update all predictor components
        for (int i = 0; i < numComponents; ++i) {
            components[i]->update(stream);
        }
    }
}

void
DecoupledBPUWithBTB::squashStreamAfter(unsigned squash_stream_id)
{
    // Erase all streams after the squashed one
    // upper_bound returns the first element greater than squash_stream_id
    auto erase_it = fetchStreamQueue.upper_bound(squash_stream_id);
    while (erase_it != fetchStreamQueue.end()) {
        DPRINTF(DecoupleBP || erase_it->second.startPC == ObservingPC,
                "Erasing stream %lu when squashing %d\n", erase_it->first,
                squash_stream_id);
        printStream(erase_it->second);
        fetchStreamQueue.erase(erase_it++);
    }
}

void
DecoupledBPUWithBTB::dumpFsq(const char *when)
{
    DPRINTF(DecoupleBPProbe, "dumping fsq entries %s...\n", when);
    for (auto it = fetchStreamQueue.begin(); it != fetchStreamQueue.end();
         it++) {
        DPRINTFR(DecoupleBPProbe, "StreamID %lu, ", it->first);
        printStream(it->second);
    }
}

bool
DecoupledBPUWithBTB::validateFSQEnqueue()
{
    // 1. Check if a prediction is available to enqueue
    if (!receivedPred) {
        DPRINTF(Override, "No prediction available to enqueue into FSQ\n");
        return false;
    }

    // 2. Validate PC value
    if (s0PC == MaxAddr) {
        DPRINTF(DecoupleBP, "Invalid PC value %#lx, cannot make prediction\n", s0PC);
        return false;
    }

    // 3. Check for override bubbles
    // When higher stages override lower stages, bubbles are needed for pipeline consistency
    if (numOverrideBubbles > 0) {
        DPRINTF(Override, "Waiting for %u override bubbles before enqueuing\n", numOverrideBubbles);
        return false;
    }

    // Ensure FSQ has space for the new entry
    assert(!streamQueueFull());
    return true;
}

/**
 * @brief Attempts to enqueue a new entry into the Fetch Stream Queue (FSQ)
 * 
 * This function is called after a prediction has been generated and checks 
 * if the prediction can be enqueued into the FSQ. It will:
 * 1. Verify that FTQ has space for new entries
 * 2. Create a new FSQ entry with the prediction
 * 3. Clear prediction state for the next cycle
 */
void
DecoupledBPUWithBTB::tryEnqFetchStream()
{
    if (!validateFSQEnqueue()) {
        return;
    }

    // Create new FSQ entry with current prediction
    makeNewPrediction(true);

    // Reset prediction state for next cycle
    for (int i = 0; i < numStages; i++) {
        predsOfEachStage[i].btbEntries.clear();
    }
    
    receivedPred = false;
    DPRINTF(Override, "FSQ entry enqueued, prediction state reset\n");
}

void
DecoupledBPUWithBTB::setTakenEntryWithStream(FtqEntry &ftq_entry, const FetchStream &stream_entry)
{
    ftq_entry.taken = true;
    ftq_entry.takenPC = stream_entry.getControlPC();
    ftq_entry.target = stream_entry.getTakenTarget();
    ftq_entry.endPC = stream_entry.predEndPC;
}

void
DecoupledBPUWithBTB::setNTEntryWithStream(FtqEntry &ftq_entry, Addr end_pc)
{
    ftq_entry.taken = false;
    ftq_entry.takenPC = 0;
    ftq_entry.target = 0;
    ftq_entry.endPC = end_pc;
}

/**
 * @brief Validate FTQ and FSQ state before enqueueing a fetch target
 *
 * This function checks:
 * 1. If FTQ has space for new entries
 * 2. If FSQ has valid entries
 * 3. If the requested stream exists in the FSQ
 *
 * @return true if validation passes, false otherwise
 */
bool
DecoupledBPUWithBTB::validateFTQEnqueue()
{
    // 1. Check if FTQ can accept new entries
    if (fetchTargetQueue.full()) {
        DPRINTF(DecoupleBP, "Cannot enqueue - FTQ is full\n");
        return false;
    }

    // 2. Check if FSQ has valid entries
    if (fetchStreamQueue.empty()) {
        dbpBtbStats.fsqNotValid++;
        DPRINTF(DecoupleBP, "Cannot enqueue - FSQ is empty\n");
        return false;
    }

    // 3. Get FTQ enqueue state and find corresponding stream
    auto &ftq_enq_state = fetchTargetQueue.getEnqState();
    auto streamIt = fetchStreamQueue.find(ftq_enq_state.streamId);
    
    if (streamIt == fetchStreamQueue.end()) {
        dbpBtbStats.fsqNotValid++;
        DPRINTF(DecoupleBP, "Cannot enqueue - Stream ID %lu not found in FSQ\n",
                ftq_enq_state.streamId);
        return false;
    }

    // Validation check - warn if FTQ enqueue PC is beyond FSQ end
    if (ftq_enq_state.pc > streamIt->second.predEndPC) {
        warn("Warning: FTQ enqueue PC %#lx is beyond FSQ end %#lx\n",
             ftq_enq_state.pc, streamIt->second.predEndPC);
    }

    return true;
}

/**
 * @brief Creates a FTQ entry from a stream entry at specific PC
 *
 * @param stream The fetch stream to use as source
 * @param ftq_enq_state The fetch target enqueue state to use
 * @return FtqEntry The created fetch target queue entry
 */
FtqEntry
DecoupledBPUWithBTB::createFtqEntryFromStream(
    const FetchStream &stream, const FetchTargetEnqState &ftq_enq_state)
{
    FtqEntry ftq_entry;
    ftq_entry.startPC = ftq_enq_state.pc;
    ftq_entry.fsqID = ftq_enq_state.streamId;

    // Configure based on taken/not-taken
    if (stream.getTaken()) {
        setTakenEntryWithStream(ftq_entry, stream);
    } else {
        setNTEntryWithStream(ftq_entry, stream.predEndPC);
    }

    return ftq_entry;
}

void
DecoupledBPUWithBTB::tryEnqFetchTarget()
{
    DPRINTF(DecoupleBP, "Attempting to enqueue fetch target into FTQ\n");

    // 1. Validate FTQ and FSQ state before proceeding
    if (!validateFTQEnqueue()) {
        return; // Validation failed, cannot proceed
    }

    // 2. Get FTQ enqueue state and find corresponding stream
    auto &ftq_enq_state = fetchTargetQueue.getEnqState();
    auto streamIt = fetchStreamQueue.find(ftq_enq_state.streamId);
    assert(streamIt != fetchStreamQueue.end()); // This should never fail since we validated

    // 3. Get fetch stream and process it
    auto &stream_to_enq = streamIt->second;

    DPRINTF(DecoupleBP, "Processing stream %lu (PC: %#lx)\n",
            streamIt->first, ftq_enq_state.pc);
    printStream(stream_to_enq);

    // 4. Create FTQ entry from stream
    FtqEntry ftq_entry = createFtqEntryFromStream(stream_to_enq, ftq_enq_state);

    // 5. Update FTQ enqueue state for next entry
    ftq_enq_state.pc = ftq_entry.taken ? stream_to_enq.getBranchInfo().target : ftq_entry.endPC;
    ftq_enq_state.streamId++;

    DPRINTF(DecoupleBP, "Updated FTQ state: PC=%#lx, next stream ID=%lu\n",
            ftq_enq_state.pc, ftq_enq_state.streamId);

    // 6. Enqueue the entry and verify state
    fetchTargetQueue.enqueue(ftq_entry);
    assert(ftq_enq_state.streamId <= fsqId + 1);

    // 7. Debug output
    printFetchTarget(ftq_entry, "Insert to FTQ");
    fetchTargetQueue.dump("After insert new entry");
}

void
DecoupledBPUWithBTB::histShiftIn(int shamt, bool taken, boost::dynamic_bitset<> &history)
{
    if (shamt == 0) {
        return;
    }
    history <<= shamt;
    history[0] = taken;
}

/**
 * @brief Creates a new FetchStream entry with prediction information
 *
 * @return FetchStream The created fetch stream
 */
FetchStream
DecoupledBPUWithBTB::createFetchStreamEntry()
{
    // Create a new fetch stream entry
    FetchStream entry;
    entry.startPC = s0PC;

    // Extract branch prediction information
    bool taken = finalPred.isTaken();
    Addr fallThroughAddr = finalPred.getFallThrough();
    Addr nextPC = finalPred.getTarget();

    // Configure stream entry with prediction details
    entry.isHit = !finalPred.btbEntries.empty();
    entry.falseHit = false;
    entry.predBTBEntries = finalPred.btbEntries;
    entry.predTaken = taken;
    entry.predEndPC = fallThroughAddr;

    // Set branch info for taken predictions
    if (taken) {
        entry.predBranchInfo = finalPred.getTakenEntry().getBranchInfo();
        entry.predBranchInfo.target = nextPC; // Use final target (may not be from BTB)
    }

    // Record current history and prediction metadata
    entry.history = s0History;
    entry.predTick = finalPred.predTick;
    entry.predSource = finalPred.predSource;

    // Save predictors' metadata
    for (int i = 0; i < numComponents; i++) {
        entry.predMetas[i] = components[i]->getPredictionMeta();
    }

    // Initialize default resolution state
    entry.setDefaultResolve();

    return entry;
}

/**
 * @brief fill ahead pipeline entry.previousPCs
 */
void
DecoupledBPUWithBTB::fillAheadPipeline(FetchStream &entry)
{
    // Handle ahead pipelined predictors
    unsigned max_ahead_pipeline_stages = 0;
    for (int i = 0; i < numComponents; i++) {
        max_ahead_pipeline_stages = std::max(max_ahead_pipeline_stages, components[i]->aheadPipelinedStages);
    }

    // Get previous PCs from fetchStreamQueue if needed
    if (max_ahead_pipeline_stages > 0) {
        for (int i = 0; i < max_ahead_pipeline_stages; i++) {
            auto it = fetchStreamQueue.find(fsqId - max_ahead_pipeline_stages + i);
            if (it != fetchStreamQueue.end()) {
                // FIXME: it may not work well with jump ahead predictor
                entry.previousPCs.push(it->second.getRealStartPC());
            }
        }
    }
}

// this function enqueues fsq and update s0PC and s0History
void
DecoupledBPUWithBTB::makeNewPrediction(bool create_new_stream)
{
    DPRINTF(DecoupleBP, "Creating new prediction for PC %#lx\n", s0PC);

    // 1. Create a new fetch stream entry with prediction information
    FetchStream entry = createFetchStreamEntry();

    // 2. Update global PC state to target or fall-through
    s0PC = finalPred.getTarget();;

    // 3. Update history information
    updateHistoryForPrediction(entry);

    // 4. Fill ahead pipeline
    fillAheadPipeline(entry);

    // 5. Add entry to fetch stream queue
    auto [insertIt, inserted] = fetchStreamQueue.emplace(fsqId, entry);
    assert(inserted);

    // 6. Debug output and update statistics
    dumpFsq("after insert new stream");
    DPRINTF(DecoupleBP, "Inserted fetch stream %lu starting at PC %#lx\n", 
            fsqId, entry.startPC);
    
    // 7. Update FSQ ID and increment statistics
    fsqId++;
    printStream(entry);
    dbpBtbStats.fsqEntryEnqueued++;
}

void
DecoupledBPUWithBTB::checkHistory(const boost::dynamic_bitset<> &history)
{
    // This function performs a crucial validation of branch history consistency
    // It rebuilds the "ideal" history from HistoryManager's records and compares
    // it with the actual history being used by the branch predictor

    // Initialize counter for total history bits and a bitset for rebuilt history
    unsigned ideal_size = 0;
    boost::dynamic_bitset<> ideal_hash_hist(historyBits, 0);

    // Iterate through all speculative history entries stored in HistoryManager
    for (const auto entry: historyManager.getSpeculativeHist()) {
        // Only process entries that have non-zero shift amount (actual branches)
        if (entry.shamt != 0) {
            // Accumulate total history bits
            ideal_size += entry.shamt;
            DPRINTF(DecoupleBPVerbose, "pc: %#lx, shamt: %lu, cond_taken: %d\n", entry.pc,
                    entry.shamt, entry.cond_taken);

            // Rebuild history by shifting and setting bits based on recorded outcomes
            // This emulates how history would be built if all branches were predicted perfectly
            ideal_hash_hist <<= entry.shamt;
            ideal_hash_hist[0] = entry.cond_taken;
        }
    }

    // Determine how many bits to compare (minimum of ideal size and actual history bits)
    unsigned comparable_size = std::min(ideal_size, historyBits);

    // Prepare actual history for comparison by creating a copy
    boost::dynamic_bitset<> sized_real_hist(history);

    // Resize both histories to the comparable size for accurate comparison
    ideal_hash_hist.resize(comparable_size);
    sized_real_hist.resize(comparable_size);

    // boost::to_string(ideal_hash_hist, buf1);
    // boost::to_string(sized_real_hist, buf2);
    DPRINTF(DecoupleBP,
            "Ideal size:\t%u, real history size:\t%u, comparable size:\t%u\n",
            ideal_size, historyBits, comparable_size);
    // DPRINTF(DecoupleBP, "Ideal history:\t%s\nreal history:\t%s\n",
    //         buf1.c_str(), buf2.c_str());

    assert(ideal_hash_hist == sized_real_hist);
}

void
DecoupledBPUWithBTB::resetPC(Addr new_pc)
{
    s0PC = new_pc;
    fetchTargetQueue.resetPC(new_pc);
}

// Addr
// DecoupledBPUWithBTB::getPreservedReturnAddr(const DynInstPtr &dynInst)
// {
//     DPRINTF(DecoupleBP, "acquiring reutrn address for inst pc %#lx from decode\n", dynInst->pcState().instAddr());
//     auto fsqid = dynInst->getFsqId();
//     auto it = fetchStreamQueue.find(fsqid);
//     auto retAddr = ras->getTopAddrFromMetas(it->second);
//     DPRINTF(DecoupleBP, "get ret addr %#lx\n", retAddr);
//     return retAddr;
// }

/**
 * @brief Updates global history based on prediction results
 *
 * @param entry The fetch stream entry to update history for
 */
void
DecoupledBPUWithBTB::updateHistoryForPrediction(FetchStream &entry)
{
    // Update component-specific history, for TAGE/ITTAGE
    for (int i = 0; i < numComponents; i++) {
        components[i]->specUpdateHist(s0History, finalPred);
    }

    // Get prediction information for history updates
    int shamt;
    bool taken;
    std::tie(shamt, taken) = finalPred.getHistInfo();

    // Update global history
    histShiftIn(shamt, taken, s0History);

    // Update history manager and verify TAGE folded history
    historyManager.addSpeculativeHist(
        entry.startPC, shamt, taken, entry.predBranchInfo, fsqId);
    tage->checkFoldedHist(s0History, "speculative update");
}

/**
 * @brief Recovers branch history during a squash event
 *
 * @param stream The stream being squashed
 * @param stream_id ID of the stream being squashed
 * @param squash_pc PC where the squash occurred
 * @param is_conditional Whether the branch is conditional
 * @param actually_taken Whether the branch was actually taken
 * @param squash_type Type of squash (CTRL/OTHER/TRAP)
 */
void
DecoupledBPUWithBTB::recoverHistoryForSquash(
    FetchStream &stream,
    unsigned stream_id,
    const PCStateBase &squash_pc,
    bool is_conditional,
    bool actually_taken,
    SquashType squash_type)
{
    // Restore history from the stream
    s0History = stream.history;

    // Get actual history shift information
    int real_shamt;
    bool real_taken;
    std::tie(real_shamt, real_taken) = stream.getHistInfoDuringSquash(
        squash_pc.instAddr(), is_conditional, actually_taken);

    // Recover component-specific history
    for (int i = 0; i < numComponents; ++i) {
        components[i]->recoverHist(s0History, stream, real_shamt, real_taken);
    }

    // Update global history with actual outcome
    histShiftIn(real_shamt, real_taken, s0History);

    // Update history manager with appropriate branch info
    if (squash_type == SQUASH_CTRL) {
        historyManager.squash(stream_id, real_shamt, real_taken, stream.exeBranchInfo);
    } else {
        historyManager.squash(stream_id, real_shamt, real_taken, BranchInfo());
    }

    // Perform history consistency checks
    checkHistory(s0History);
    tage->checkFoldedHist(s0History,
        squash_type == SQUASH_CTRL ? "control squash" :
        squash_type == SQUASH_OTHER ? "non control squash" : "trap squash");
}

}  // namespace test

}  // namespace btb_pred

}  // namespace branch_prediction

}  // namespace gem5
