#include "cpu/pred/btb/test/decoupled_bpred.hh"

#include "cpu/pred/btb/test/test_dprintf.hh"

// #include "base/output.hh"
#include "base/debug_helper.hh"

// #include "cpu/o3/cpu.hh"
// #include "cpu/o3/dyn_inst.hh"
// #include "cpu/pred/btb/stream_common.hh"

// #include "sim/core.hh"

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
      alignToBlockSize(true),
      historyBits(128), // TODO: for test!
      ubtb(new DefaultBTB(32, 38, 32, 0, true)),
      btb(new DefaultBTB(2048, 20, 8, 1, true)),
      tage(new BTBTAGE()),
      numStages(3),
      historyManager(8), // TODO: fix this
      dbpBtbStats()
{
    btb_pred::predictWidth = 64;  // set global variable, used in stream_struct.hh
    btb_pred::alignToBlockSize = false;
    numBr = 8; //TODO: remove numBr

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
    // Monitor FSQ size for statistics
    // dbpBtbStats.fsqEntryDist.sample(fetchStreamQueue.size(), 1);
    if (streamQueueFull()) {
        dbpBtbStats.fsqFullCannotEnq++;
        DPRINTF(Override, "FSQ is full (%lu entries)\n", fetchStreamQueue.size());
    }

    // Generate final prediction if we have received PC and history but no prediction yet
    if (!receivedPred && numOverrideBubbles == 0 && sentPCHist) {
        DPRINTF(Override, "Generating final prediction for PC %#lx\n", s0PC);
        generateFinalPredAndCreateBubbles();
    }

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

    DPRINTF(Override, "Prediction cycle complete\n");
    
    // Clear squashing state for next cycle
    squashing = false;
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

std::pair<bool, bool>
DecoupledBPUWithBTB::decoupledPredict(const StaticInstPtr &inst,
                               const InstSeqNum &seqNum, PCStateBase &pc,
                               ThreadID tid, unsigned &currentLoopIter)
{
    std::unique_ptr<PCStateBase> target(pc.clone());

    DPRINTF(DecoupleBP, "looking up pc %#lx\n", pc.instAddr());
    auto target_avail = fetchTargetQueue.fetchTargetAvailable();

    DPRINTF(DecoupleBP, "Supplying fetch with target ID %lu\n",
            fetchTargetQueue.getSupplyingTargetId());

    if (!target_avail) {
        DPRINTF(DecoupleBP,
                "No ftq entry to fetch, return dummy prediction\n");
        // todo pass these with reference
        // TODO: do we need to update PC if not taken?
        return std::make_pair(false, true);
    }
    currentFtqEntryInstNum++;

    const auto &target_to_fetch = fetchTargetQueue.getTarget();

    // found corresponding entry
    auto start = target_to_fetch.startPC;
    auto end = target_to_fetch.endPC;
    auto taken_pc = target_to_fetch.takenPC;
    DPRINTF(DecoupleBP, "Responsing fetch with");
    printFetchTarget(target_to_fetch, "");

    // supplying ftq entry might be taken before pc
    // because it might just be updated last cycle
    // but last cycle ftq tells fetch that this is a miss stream
    assert(pc.instAddr() < end && pc.instAddr() >= start);
    bool raw_taken = pc.instAddr() == taken_pc && target_to_fetch.taken;
    bool taken = raw_taken;
    bool run_out_of_this_entry = false;

    if (taken) {
        run_out_of_this_entry = true;
    }


    // if (taken) {
    //     auto &rtarget = target->as<GenericISA::PCStateWithNext>();
    //     rtarget.pc(target_to_fetch.target);
    //     // TODO: how about compressed?
    //     rtarget.npc(target_to_fetch.target + 4);
    //     rtarget.uReset();
    //     DPRINTF(DecoupleBP,
    //             "Predicted pc: %#lx, upc: %u, npc(meaningless): %#lx, instSeqNum: %lu\n",
    //             target->instAddr(), rtarget.upc(), rtarget.npc(), seqNum);
    //     set(pc, *target);
    // } else {
    //     inst->advancePC(*target);
    //     if (target->instAddr() >= end) {
    //         run_out_of_this_entry = true;
    //     }
    // }
    DPRINTF(DecoupleBP, "Predict it %staken to %#lx\n", taken ? "" : "not ",
            target->instAddr());

    if (run_out_of_this_entry) {
        // dequeue the entry
        const auto fsqId = target_to_fetch.fsqID;
        DPRINTF(DecoupleBP, "running out of ftq entry %lu with %d insts\n",
                fetchTargetQueue.getSupplyingTargetId(), currentFtqEntryInstNum);
        fetchTargetQueue.finishCurrentFetchTarget();
        // record inst fetched in fsq entry
        auto it = fetchStreamQueue.find(fsqId);
        assert(it != fetchStreamQueue.end());
        it->second.fetchInstNum = currentFtqEntryInstNum;
        currentFtqEntryInstNum = 0;
    }

    return std::make_pair(taken, run_out_of_this_entry);
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
    // bool is_call = static_inst->isCall() && !static_inst->isNonSpeculative();
    // bool is_return = static_inst->isReturn() && !static_inst->isNonSpeculative();




    squashing = true;

    // Find the stream being squashed
    auto squashing_stream_it = fetchStreamQueue.find(stream_id);

    if (squashing_stream_it == fetchStreamQueue.end()) {
        assert(!fetchStreamQueue.empty());
        // assert(fetchStreamQueue.rbegin()->second.getNextStreamStart() == MaxAddr);
        DPRINTF(
            DecoupleBP || debugFlagOn,
            "The squashing stream is insane, ignore squash on it");
        return;
    }


    // get corresponding stream entry
    auto &stream = squashing_stream_it->second;
    // get target from ras preserved info for decode-detected unpredicted returns
    Addr real_target = corr_target.instAddr();
    // if (!fromCommit && static_inst->isReturn() && !static_inst->isNonSpeculative()) {
    //     // get ret addr from ras meta
    //     real_target = ras->getTopAddrFromMetas(stream);
    //     // TODO: set real target to dynamic inst
    // }


    // recover pc
    s0PC = real_target;

    // Create branch info for squash
    auto squashBranchInfo = BranchInfo(control_pc.instAddr(), real_target, false);

    auto pc = stream.startPC;
    defer _(nullptr, std::bind([this]{ debugFlagOn = false; }));
    if (pc == ObservingPC) {
        debugFlagOn = true;
    }
    if (control_pc.instAddr() == ObservingPC || control_pc.instAddr() == ObservingPC2) {
        debugFlagOn = true;
    }

    DPRINTF(DecoupleBPHist,
            "stream start=%#lx, predict on hist\n", stream.startPC);

    DPRINTF(DecoupleBP || debugFlagOn,
            "Control squash: ftq_id=%d, fsq_id=%d,"
            " control_pc=%#lx, real_target=%#lx, is_conditional=%u, "
            "is_indirect=%u, actually_taken=%u, branch seq: %lu\n",
            target_id, stream_id, control_pc.instAddr(),
            real_target, is_conditional, is_indirect,
            actually_taken, seq);

    dumpFsq("Before control squash");

    // streamLoopPredictor->restoreLoopTable(stream.mruLoop);
    // streamLoopPredictor->controlSquash(stream_id, stream, control_pc.instAddr(), corr_target.instAddr());

    stream.squashType = SQUASH_CTRL;

    FetchTargetId ftq_demand_stream_id;


    stream.exeBranchInfo = squashBranchInfo;
    stream.exeTaken = actually_taken;
    stream.squashPC = control_pc.instAddr();

    squashStreamAfter(stream_id);

    stream.resolved = true;

    // recover history to the moment doing prediction
    // DPRINTF(DecoupleBPHist,
    //          "Recover history %s\nto %s\n", s0History, stream.history);
    s0History = stream.history;

    // recover history info
    int real_shamt;
    bool real_taken;
    std::tie(real_shamt, real_taken) = stream.getHistInfoDuringSquash(control_pc.instAddr(), is_conditional, actually_taken, numBr);
    for (int i = 0; i < numComponents; ++i) {
        components[i]->recoverHist(s0History, stream, real_shamt, real_taken);
    }
    histShiftIn(real_shamt, real_taken, s0History);
    historyManager.squash(stream_id, real_shamt, real_taken, stream.exeBranchInfo);
    checkHistory(s0History);
    tage->checkFoldedHist(s0History, "control squash");

    // DPRINTF(DecoupleBPHist,
    //             "Shift in history %s\n", s0History);

    printStream(stream);

    clearPreds();

    // inc stream id because current stream ends
    // now stream always ends
    ftq_demand_stream_id = stream_id + 1;
    fsqId = stream_id + 1;

    dumpFsq("After control squash");

    fetchTargetQueue.squash(target_id + 1, ftq_demand_stream_id,
                            real_target);

    fetchTargetQueue.dump("After control squash");

    DPRINTFV(this->debugFlagOn || ::gem5::debug::DecoupleBP,
            "After squash, FSQ head Id=%lu, demand stream Id=%lu, Fetch "
            "demanded target Id=%lu\n",
            fsqId, fetchTargetQueue.getEnqState().streamId,
            fetchTargetQueue.getSupplyingTargetId());


}

void
DecoupledBPUWithBTB::nonControlSquash(unsigned target_id, unsigned stream_id,
                               const PCStateBase &inst_pc,
                               const InstSeqNum seq, ThreadID tid, const unsigned &currentLoopIter)
{
    dbpBtbStats.nonControlSquash++;
    DPRINTFV(this->debugFlagOn || ::gem5::debug::DecoupleBP,
            "non control squash: target id: %d, stream id: %d, inst_pc: %#lx, "
            "seq: %lu\n",
            target_id, stream_id, inst_pc.instAddr(), seq);
    squashing = true;

    dumpFsq("before non-control squash");

    // make sure the stream is in FSQ
    auto it = fetchStreamQueue.find(stream_id);
    assert(it != fetchStreamQueue.end());

    auto ftq_demand_stream_id = stream_id;
    auto &stream = it->second;

    squashStreamAfter(stream_id);

    stream.exeTaken = false;
    stream.resolved = true;
    stream.squashPC = inst_pc.instAddr();
    stream.squashType = SQUASH_OTHER;

    // recover history info
    s0History = it->second.history;
    int real_shamt;
    bool real_taken;
    std::tie(real_shamt, real_taken) = stream.getHistInfoDuringSquash(inst_pc.instAddr(), false, false, numBr);
    for (int i = 0; i < numComponents; ++i) {
        components[i]->recoverHist(s0History, stream, real_shamt, real_taken);
    }
    histShiftIn(real_shamt, real_taken, s0History);
    historyManager.squash(stream_id, real_shamt, real_taken, BranchInfo());
    checkHistory(s0History);
    tage->checkFoldedHist(s0History, "non control squash");
    // fetching from a new fsq entry
    auto pc = inst_pc.instAddr();
    fetchTargetQueue.squash(target_id + 1, ftq_demand_stream_id + 1, pc);

    clearPreds();

    s0PC = pc;
    fsqId = stream_id + 1;

    if (pc == ObservingPC) dumpFsq("after non-control squash");
    DPRINTFV(this->debugFlagOn || ::gem5::debug::DecoupleBP,
            "After squash, FSQ head Id=%lu, s0pc=%#lx, demand stream Id=%lu, "
            "Fetch demanded target Id=%lu\n",
            fsqId, s0PC, fetchTargetQueue.getEnqState().streamId,
            fetchTargetQueue.getSupplyingTargetId());
}

void
DecoupledBPUWithBTB::trapSquash(unsigned target_id, unsigned stream_id,
                         Addr last_committed_pc, const PCStateBase &inst_pc,
                         ThreadID tid, const unsigned &currentLoopIter)
{
    dbpBtbStats.trapSquash++;
    DPRINTF(DecoupleBP || debugFlagOn,
            "Trap squash: target id: %d, stream id: %d, inst_pc: %#lx\n",
            target_id, stream_id, inst_pc.instAddr());
    squashing = true;

    auto pc = inst_pc.instAddr();

    if (pc == ObservingPC) dumpFsq("before trap squash");

    auto it = fetchStreamQueue.find(stream_id);
    assert(it != fetchStreamQueue.end());
    auto &stream = it->second;

    stream.resolved = true;
    stream.exeTaken = false;
    stream.squashPC = inst_pc.instAddr();
    stream.squashType = SQUASH_TRAP;

    squashStreamAfter(stream_id);

    // recover history info
    s0History = stream.history;
    int real_shamt;
    bool real_taken;
    std::tie(real_shamt, real_taken) = stream.getHistInfoDuringSquash(inst_pc.instAddr(), false, false, numBr);
    for (int i = 0; i < numComponents; ++i) {
        components[i]->recoverHist(s0History, stream, real_shamt, real_taken);
    }
    histShiftIn(real_shamt, real_taken, s0History);
    historyManager.squash(stream_id, real_shamt, real_taken, BranchInfo());
    checkHistory(s0History);
    tage->checkFoldedHist(s0History, "trap squash");

    // inc stream id because current stream is disturbed
    auto ftq_demand_stream_id = stream_id + 1;
    fsqId = stream_id + 1;

    fetchTargetQueue.squash(target_id + 1, ftq_demand_stream_id,
                            inst_pc.instAddr());
    clearPreds();

    s0PC = inst_pc.instAddr();

    DPRINTF(DecoupleBP,
            "After trap squash, FSQ head Id=%lu, s0pc=%#lx, demand stream "
            "Id=%lu, Fetch demanded target Id=%lu\n",
            fsqId, s0PC, fetchTargetQueue.getEnqState().streamId,
            fetchTargetQueue.getSupplyingTargetId());
}

void DecoupledBPUWithBTB::update(unsigned stream_id, ThreadID tid)
{
    // aka, commit stream
    // commit controls in local prediction history buffer to committedSeq
    // mark all committed control instructions as correct
    // do not need to dequeue when empty
    if (fetchStreamQueue.empty())
        return;
    auto it = fetchStreamQueue.begin();
    defer _(nullptr, std::bind([this]{ debugFlagOn = false; }));
    while (it != fetchStreamQueue.end() && stream_id >= it->first) {
        auto &stream = it->second;
        // dequeue
        DPRINTF(DecoupleBP, "dequeueing stream id: %lu, entry below:\n",
                it->first);
        bool miss_predicted = stream.squashType == SQUASH_CTRL;
        if (miss_predicted) {
            DPRINTF(ITTAGE || (stream.squashPC == 0x1e0eb6), "miss predicted stream.startAddr=%#lx\n", stream.startPC);
        }
        if (miss_predicted && stream.exeBranchInfo.isIndirect) {
            topMispredIndirect[stream.startPC]++;
        }
        DPRINTF(DecoupleBP || debugFlagOn,
                "Commit stream start %#lx, which is %s predicted, "
                "final br addr: %#lx, final target: %#lx, pred br addr: %#lx, "
                "pred target: %#lx\n",
                stream.startPC, miss_predicted ? "miss" : "correctly",
                stream.exeBranchInfo.pc, stream.exeBranchInfo.target,
                stream.predBranchInfo.pc, stream.predBranchInfo.target);
        
        if (stream.isHit) {
            // FIXME: should count in terms of instruction instead of block
            dbpBtbStats.btbHit++;
        } else {
            if (stream.exeTaken) {
                dbpBtbStats.btbMiss++;
                DPRINTF(BTB, "BTB miss detected when update, stream start %#lx, predTick %lu, printing branch info:\n", stream.startPC, stream.predTick);
                auto &slot = stream.exeBranchInfo;
                DPRINTF(BTB, "    pc:%#lx, size:%d, target:%#lx, cond:%d, indirect:%d, call:%d, return:%d\n",
                slot.pc, slot.size, slot.target, slot.isCond, slot.isIndirect, slot.isCall, slot.isReturn);
            }
            if (stream.falseHit) {
                dbpBtbStats.commitFalseHit++;
            }
        }
        // dbpBtbStats.commitPredsFromEachStage[stream.predSource]++;


        if (stream.isHit || stream.exeTaken) {
            stream.setUpdateInstEndPC(predictWidth);
            stream.setUpdateBTBEntries();
            //abtb->getAndSetNewBTBEntry(stream);
            btb->getAndSetNewBTBEntry(stream);
            for (int i = 0; i < numComponents; ++i) {
                components[i]->update(stream);
            }
            // btb entry stats
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

        it = fetchStreamQueue.erase(it);

        dbpBtbStats.fsqEntryCommitted++;
    }
    DPRINTF(DecoupleBP, "after commit stream, fetchStreamQueue size: %lu\n",
            fetchStreamQueue.size());
    printStream(it->second);

    historyManager.commit(stream_id);
}


void
DecoupledBPUWithBTB::squashStreamAfter(unsigned squash_stream_id)
{
    auto erase_it = fetchStreamQueue.upper_bound(squash_stream_id);
    while (erase_it != fetchStreamQueue.end()) {
        DPRINTF(DecoupleBP || debugFlagOn || erase_it->second.startPC == ObservingPC,
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



/**
 * @brief Attempts to enqueue a new entry into the Fetch Stream Queue (FSQ)
 * 
 * This function is called after a prediction has been generated and checks 
 * if the prediction can be enqueued into the FSQ. It will:
 * 1. Verify that a prediction is available
 * 2. Check if the PC is valid
 * 3. Wait for any override bubbles to be consumed
 * 4. Create a new FSQ entry with the prediction
 * 5. Clear prediction state for the next cycle
 */
void
DecoupledBPUWithBTB::tryEnqFetchStream()
{
    // 1. Check if a prediction is available to enqueue
    if (!receivedPred) {
        DPRINTF(Override, "No prediction available to enqueue into FSQ\n");
        return;
    }
    
    // 2. Validate PC value
    if (s0PC == MaxAddr) {
        DPRINTF(DecoupleBP, "Invalid PC value %#lx, cannot make prediction\n", s0PC);
        return;
    }
    
    // 3. Check for override bubbles
    // When higher stages override lower stages, bubbles are needed for pipeline consistency
    if (numOverrideBubbles > 0) {
        DPRINTF(Override, "Waiting for %u override bubbles before enqueuing\n", numOverrideBubbles);
        return;
    }
    
    // Ensure FSQ has space for the new entry
    assert(!streamQueueFull());
    
    // 4. Create new FSQ entry with current prediction
    makeNewPrediction(true);

    // 5. Reset prediction state for next cycle
    for (int i = 0; i < numStages; i++) {
        predsOfEachStage[i].btbEntries.clear();
    }
    
    receivedPred = false;
    DPRINTF(Override, "FSQ entry enqueued, prediction state reset\n");
}

void
DecoupledBPUWithBTB::setTakenEntryWithStream(const FetchStream &stream_entry, FtqEntry &ftq_entry)
{
    ftq_entry.taken = true;
    ftq_entry.takenPC = stream_entry.getControlPC();
    ftq_entry.endPC = stream_entry.predEndPC;
    ftq_entry.target = stream_entry.getTakenTarget();
}

void
DecoupledBPUWithBTB::setNTEntryWithStream(FtqEntry &ftq_entry, Addr end_pc)
{
    ftq_entry.taken = false;
    ftq_entry.takenPC = 0;
    ftq_entry.target = 0;
    ftq_entry.endPC = end_pc;
}

void
DecoupledBPUWithBTB::tryEnqFetchTarget()
{
    DPRINTF(DecoupleBP, "Attempting to enqueue fetch target into FTQ\n");

    // 1. Check if FTQ can accept new entries
    if (fetchTargetQueue.full()) {
        DPRINTF(DecoupleBP, "Cannot enqueue - FTQ is full\n");
        return;
    }

    // 2. Check if FSQ has valid entries
    if (fetchStreamQueue.empty()) {
        dbpBtbStats.fsqNotValid++;
        DPRINTF(DecoupleBP, "Cannot enqueue - FSQ is empty\n");
        return;
    }

    // 3. Get FTQ enqueue state and find corresponding stream
    auto &ftq_enq_state = fetchTargetQueue.getEnqState();
    auto streamIt = fetchStreamQueue.find(ftq_enq_state.streamId);
    
    if (streamIt == fetchStreamQueue.end()) {
        dbpBtbStats.fsqNotValid++;
        DPRINTF(DecoupleBP, "Cannot enqueue - Stream ID %lu not found in FSQ\n",
                ftq_enq_state.streamId);
        return;
    }

    // 4. Get fetch stream and verify addresses
    auto &stream_to_enq = streamIt->second;
    Addr streamEndPC = stream_to_enq.predEndPC;
    
    DPRINTF(DecoupleBP, "Processing stream %lu (PC: %#lx)\n",
            streamIt->first, ftq_enq_state.pc);
    printStream(stream_to_enq);

    // Validation check - warn if FTQ enqueue PC is beyond FSQ end
    if (ftq_enq_state.pc > streamEndPC) {
        warn("Warning: FTQ enqueue PC %#lx is beyond FSQ end %#lx\n",
             ftq_enq_state.pc, streamEndPC);
    }

    // 5. Create and initialize new FTQ entry
    FtqEntry ftq_entry;
    ftq_entry.startPC = ftq_enq_state.pc;
    ftq_entry.fsqID = ftq_enq_state.streamId;

    // 6. Calculate FTQ entry boundaries
    Addr entryEndPC = streamEndPC;
    bool taken = stream_to_enq.getTaken();

    // 8. Configure FTQ entry based on prediction (taken/not-taken)
    if (taken) {
        setTakenEntryWithStream(stream_to_enq, ftq_entry);
    } else {
        setNTEntryWithStream(ftq_entry, entryEndPC);
    }

    // 9. Update FTQ enqueue state for next entry
    // For non-loops: next PC depends on branch outcome
    ftq_enq_state.pc = taken ? stream_to_enq.getBranchInfo().target : entryEndPC;
    ftq_enq_state.streamId++;

    DPRINTF(DecoupleBP, "Updated FTQ state: PC=%#lx, next stream ID=%lu\n",
            ftq_enq_state.pc, ftq_enq_state.streamId);

    // 11. Enqueue the entry and verify state
    fetchTargetQueue.enqueue(ftq_entry);
    assert(ftq_enq_state.streamId <= fsqId + 1);

    // 12. Debug output
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


// this function enqueues fsq and update s0PC and s0History
// use loop predictor and loop buffer here
void
DecoupledBPUWithBTB::makeNewPrediction(bool create_new_stream)
{
    DPRINTF(DecoupleBP, "Creating new prediction for PC %#lx\n", s0PC);

    // Create a new fetch stream entry
    FetchStream entry;
    entry.startPC = s0PC;
    
    // Initialize loop prediction info containers
    bool endLoop, isDouble, loopConf;
    std::vector<LoopRedirectInfo> lpRedirectInfos(numBr);
    std::vector<bool> fixNotExits(numBr);
    std::vector<LoopRedirectInfo> unseenLpRedirectInfos;
    
    // Normal prediction path (when loop buffer is not active)
    // 2. Extract branch prediction information
    bool taken = finalPred.isTaken();
    Addr fallThroughAddr = finalPred.getFallThrough();
    Addr nextPC = finalPred.getTarget();

    // 3. Configure stream entry with prediction details
    entry.isHit = !finalPred.btbEntries.empty(); // TODO: fix isHit and falseHit
    entry.falseHit = false;
    entry.predBTBEntries = finalPred.btbEntries;
    entry.predTaken = taken;
    entry.predEndPC = fallThroughAddr;

    // 4. Set branch info for taken predictions
    if (taken) {
        entry.predBranchInfo = finalPred.getTakenEntry().getBranchInfo();
        entry.predBranchInfo.target = nextPC; // Use final target (may not be from BTB)
    }

    // 5. Update global PC state to target or fall-through
    s0PC = nextPC;
    // 7. Record current history and prediction metadata
    entry.history = s0History;
    entry.predTick = finalPred.predTick;
    entry.predSource = finalPred.predSource;
    // 8. Update component-specific history
    for (int i = 0; i < numComponents; i++) {
        components[i]->specUpdateHist(s0History, finalPred);
        entry.predMetas[i] = components[i]->getPredictionMeta();
    }

    // 9. Update global history with new prediction
    int shamt;
    std::tie(shamt, taken) = finalPred.getHistInfo();
    histShiftIn(shamt, taken, s0History);

    // 10. Update history manager and verify TAGE folded history
    historyManager.addSpeculativeHist(
        entry.startPC, shamt, taken, entry.predBranchInfo, fsqId);
    tage->checkFoldedHist(s0History, "speculative update");

    // 11. Initialize default resolution state
    entry.setDefaultResolve();

    // if there are ahead pipelined predictors, get prevoius PCs
    unsigned max_ahead_pipeline_stages = 0;
    for (int i = 0; i < numComponents; i++) {
        max_ahead_pipeline_stages = std::max(max_ahead_pipeline_stages, components[i]->aheadPipelinedStages);
    }
    // get number of max_ahead_pipeline_stages previous PCs from fetchStreamQueue
    if (max_ahead_pipeline_stages > 0) {
        for (int i = 0; i < max_ahead_pipeline_stages; i++) {
            auto it = fetchStreamQueue.find(fsqId - max_ahead_pipeline_stages + i);
            if (it != fetchStreamQueue.end()) {
                // FIXME: it may not work well with jump ahead predictor
                entry.previousPCs.push(it->second.getRealStartPC());
            }
        }
    }
    auto [insertIt, inserted] = fetchStreamQueue.emplace(fsqId, entry);
    assert(inserted);

    // 14. Debug output and statistics
    dumpFsq("after insert new stream");
    DPRINTF(DecoupleBP, "Inserted fetch stream %lu starting at PC %#lx\n", 
            fsqId, entry.startPC);
    
    // 15. Update FSQ ID and increment statistics
    fsqId++;
    printStream(entry);
    dbpBtbStats.fsqEntryEnqueued++;
}

void
DecoupledBPUWithBTB::checkHistory(const boost::dynamic_bitset<> &history)
{
    unsigned ideal_size = 0;
    boost::dynamic_bitset<> ideal_hash_hist(historyBits, 0);
    for (const auto entry: historyManager.getSpeculativeHist()) {
        if (entry.shamt != 0) {
            ideal_size += entry.shamt;
            DPRINTF(DecoupleBPVerbose, "pc: %#lx, shamt: %lu, cond_taken: %d\n", entry.pc,
                    entry.shamt, entry.cond_taken);
            ideal_hash_hist <<= entry.shamt;
            ideal_hash_hist[0] = entry.cond_taken;
        }
    }
    unsigned comparable_size = std::min(ideal_size, historyBits);
    boost::dynamic_bitset<> sized_real_hist(history);
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


}  // namespace test

}  // namespace btb_pred

}  // namespace branch_prediction

}  // namespace gem5
