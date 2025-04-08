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

    lp = LoopPredictor(16, 4, enableLoopDB);
    lb.setLp(&lp);

    jap = JumpAheadPredictor(16, 4);

    if (!enableLoopPredictor && enableLoopBuffer) {
        fatal("loop buffer cannot be enabled without loop predictor\n");
    }
}

void
DecoupledBPUWithBTB::tick()
{
    // Monitor FSQ size for statistics
    // dbpBtbStats.fsqEntryDist.sample(fetchStreamQueue.size(), 1);
    if (streamQueueFull()) {
        dbpBtbStats.fsqFullCannotEnq++;
    }

    // Generate final prediction if we have received PC and history but no prediction yet
    if (!receivedPred && numOverrideBubbles == 0 && sentPCHist) {
        generateFinalPredAndCreateBubbles();
    }

    // Try to enqueue new predictions if not squashing
    if (!squashing) {
        DPRINTF(DecoupleBP, "DecoupledBPUWithBTB::tick()\n");
        DPRINTF(Override, "DecoupledBPUWithBTB::tick()\n");
        tryEnqFetchTarget();
        tryEnqFetchStream();
    } else {
        receivedPred = false;
        DPRINTF(DecoupleBP, "Squashing, skip this cycle, receivedPred is %d.\n", receivedPred);
        DPRINTF(Override, "Squashing, skip this cycle, receivedPred is %d.\n", receivedPred);
    }

    // Decrement override bubbles counter
    if (numOverrideBubbles > 0) {
        numOverrideBubbles--;
        dbpBtbStats.overrideBubbleNum++;
    }

    sentPCHist = false;

    // Request new prediction if FSQ not full and not using loop buffer
    if (!receivedPred && !streamQueueFull()) {
        if (!enableLoopBuffer || (enableLoopBuffer && !lb.isActive())) {
            if (s0PC == ObservingPC) {
                DPRINTFV(true, "Predicting block %#lx, id: %lu\n", s0PC, fsqId);
            }   
            DPRINTF(DecoupleBP, "Requesting prediction for stream start=%#lx\n", s0PC);
            DPRINTF(Override, "Requesting prediction for stream start=%#lx\n", s0PC);
            // Initialize prediction state for each stage
            for (int i = 0; i < numStages; i++) {
                predsOfEachStage[i].bbStart = s0PC;
            }
            // Query each predictor component with current PC and history
            for (int i = 0; i < numComponents; i++) {
                components[i]->putPCHistory(s0PC, s0History, predsOfEachStage);
            }
        } else {
            DPRINTF(LoopBuffer, "Do not query bpu when loop buffer is active\n");
            DPRINTF(DecoupleBP, "Do not query bpu when loop buffer is active\n");
        }


        sentPCHist = true;
    }
    

    // query loop buffer with start pc
    if (enableLoopBuffer && !lb.isActive() &&
            lb.streamBeforeLoop.getTakenTarget() == lb.streamBeforeLoop.startPC &&
            !lb.streamBeforeLoop.resolved) { // do not activate loop buffer right after squash
        lb.tryActivateLoop(s0PC);
    }

    DPRINTF(Override, "after putPCHistory\n");
    // for (int i = 0; i < numStages; i++) {
    //     printFullBTBPrediction(predsOfEachStage[i]);
    // }
    
    if (streamQueueFull()) {
        DPRINTF(DecoupleBP, "Stream queue is full, don't request prediction\n");
        DPRINTF(Override, "Stream queue is full, don't request prediction\n");
    }
    squashing = false;
}

// this function collects predictions from all stages and generate bubbles
// when loop buffer is active, predictions are from saved stream
void
DecoupledBPUWithBTB::generateFinalPredAndCreateBubbles()
{
    DPRINTF(Override, "In generateFinalPredAndCreateBubbles().\n");

    if (!enableLoopBuffer || (enableLoopBuffer && !lb.isActive())) {
        // predsOfEachStage should be ready now
        for (int i = 0; i < numStages; i++) {
            printFullBTBPrediction(i, predsOfEachStage[i]);
        }
        // choose the most accurate prediction
        FullBTBPrediction *chosen = &predsOfEachStage[0];

        for (int i = (int) numStages - 1; i >= 0; i--) {
            if (predsOfEachStage[i].btbEntries.size() > 0) {
                chosen = &predsOfEachStage[i];
                DPRINTF(Override, "choose stage %d.\n", i);
                break;
            }
        }
        finalPred = *chosen;
        // calculate bubbles
        unsigned first_hit_stage = 0;
        OverrideReason overrideReason = OverrideReason::no_override;
        while (first_hit_stage < numStages-1) {
            auto matchResult = predsOfEachStage[first_hit_stage].match(*chosen);
            if (matchResult.first) {
                break;
            }
            first_hit_stage++;
            overrideReason = matchResult.second;
        }
        // generate bubbles
        numOverrideBubbles = first_hit_stage;
        if (numOverrideBubbles > 0){
            dbpBtbStats.overrideCount++;
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
        // assign pred source
        finalPred.predSource = first_hit_stage;
        receivedPred = true;


        // printFullBTBPrediction(*chosen);
        // dbpBtbStats.predsOfEachStage[first_hit_stage]++;

        clearPreds();
    } else {
        numOverrideBubbles = 0;
        receivedPred = true;
        DPRINTF(LoopBuffer, "Do not generate final pred when loop buffer is active\n");
        DPRINTF(DecoupleBP, "Do not generate final pred when loop buffer is active\n");
    }

    DPRINTF(Override, "Ends generateFinalPredAndCreateBubbles(), numOverrideBubbles is %d, receivedPred is set true.\n", numOverrideBubbles);

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
    auto in_loop = target_to_fetch.inLoop;
    auto loop_iter = target_to_fetch.iter;
    auto loop_exit = target_to_fetch.isExit;
    DPRINTF(DecoupleBP, "Responsing fetch with");
    printFetchTarget(target_to_fetch, "");

    auto current_loop_iter = fetchTargetQueue.getCurrentLoopIter();
    currentLoopIter = current_loop_iter;

    // supplying ftq entry might be taken before pc
    // because it might just be updated last cycle
    // but last cycle ftq tells fetch that this is a miss stream
    assert(pc.instAddr() < end && pc.instAddr() >= start);
    bool raw_taken = pc.instAddr() == taken_pc && target_to_fetch.taken;
    bool taken = raw_taken;
    bool run_out_of_this_entry = false;
    // an ftq entry may consists of multiple loop iterations,
    // so we need to check if we are at the end of this loop iteration,
    // since taken and not taken can both exist in the same ftq entry
    if (in_loop) {
        DPRINTF(LoopBuffer, "current loop iter %d, loop_iter %d, loop_exit %d\n",
            current_loop_iter, loop_iter, loop_exit);
        if (raw_taken) {
            if (current_loop_iter >= loop_iter - 1) {
                run_out_of_this_entry = true;
                if (loop_exit) {
                    taken = false;
                    lb.tryUnpin();
                    DPRINTF(LoopBuffer, "modifying taken to false because of loop exit\n");
                }
            }
            fetchTargetQueue.incCurrentLoopIter(loop_iter);
        }
    } else {
        if (taken) {
            run_out_of_this_entry = true;
        }
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
    if (stream.isExit) {
        dbpBtbStats.controlSquashOnLoopPredictorPredExit++;
    }
    if (stream.fromLoopBuffer) {
        dbpBtbStats.squashOnLoopBufferPredBlock++;
        if (stream.isDouble) {
            dbpBtbStats.squashOnLoopBufferDoublePredBlock++;
        }
    }

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

    if (enableJumpAheadPredictor && stream.jaHit) {
        jap.invalidate(stream.startPC);
        dbpBtbStats.controlSquashOnJaHitBlocks++;
    }

    FetchTargetId ftq_demand_stream_id;


    stream.exeBranchInfo = squashBranchInfo;
    stream.exeTaken = actually_taken;
    stream.squashPC = control_pc.instAddr();

    if (enableLoopPredictor) {
        lp.startRepair();
        // recover loop predictor
        // we should check if the numBr possible loop branches should be recovered
        for (int i = 0; i < numBr; ++i) {
            // loop branches behind the squashed branch should be recovered
            if (stream.loopRedirectInfos[i].e.valid && control_pc.instAddr() <= stream.loopRedirectInfos[i].branch_pc) {
                DPRINTF(DecoupleBP, "Recover loop predictor for %#lx\n", stream.loopRedirectInfos[i].branch_pc);
                lp.recover(stream.loopRedirectInfos[i], actually_taken, control_pc.instAddr(), true, false, currentLoopIter);
            }
        }
        for (auto &info : stream.unseenLoopRedirectInfos) {
            if (info.e.valid && control_pc.instAddr() <= info.branch_pc) {
                DPRINTF(DecoupleBP, "Recover loop predictor for unseen branch %#lx\n", info.branch_pc);
                lp.recover(info, actually_taken, control_pc.instAddr(), true, false, currentLoopIter);
            }
        }
    }

    squashStreamAfter(stream_id);

    if (enableLoopPredictor) {
        lp.endRepair();
    }

    if (enableLoopBuffer) {
        lb.clearState();
    }

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

    if (enableLoopBuffer) {
        lb.recordNewestStreamOutsideLoop(stream);
    }

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

    if (enableLoopPredictor) {
        lp.startRepair();
        // recover loop predictor
        // we should check if the numBr possible loop branches should be recovered
        for (int i = 0; i < numBr; ++i) {
            // loop branches behind the squashed branch should be recovered
            if (stream.loopRedirectInfos[i].e.valid && inst_pc.instAddr() <= stream.loopRedirectInfos[i].branch_pc) {
                DPRINTF(DecoupleBP, "Recover loop predictor for %#lx\n", stream.loopRedirectInfos[i].branch_pc);
                lp.recover(stream.loopRedirectInfos[i], false, inst_pc.instAddr(), false, false, currentLoopIter);
            }
        }
        for (auto &info : stream.unseenLoopRedirectInfos) {
            if (info.e.valid && inst_pc.instAddr() <= info.branch_pc) {
                DPRINTF(DecoupleBP, "Recover loop predictor for unseen branch %#lx\n", info.branch_pc);
                lp.recover(info, false, inst_pc.instAddr(), false, false, currentLoopIter);
            }
        }
    }


    squashStreamAfter(stream_id);

    if (enableLoopPredictor) {
        lp.endRepair();
    }

    if (enableLoopBuffer) {
        lb.clearState();
    }

    
    if (stream.isExit) {
        dbpBtbStats.nonControlSquashOnLoopPredictorPredExit++;
    }
    if (stream.fromLoopBuffer) {
        dbpBtbStats.squashOnLoopBufferPredBlock++;
        if (stream.isDouble) {
            dbpBtbStats.squashOnLoopBufferDoublePredBlock++;
        }
    }

    stream.exeTaken = false;
    stream.resolved = true;
    stream.squashPC = inst_pc.instAddr();
    stream.squashType = SQUASH_OTHER;

    if (enableJumpAheadPredictor && stream.jaHit) {
        dbpBtbStats.nonControlSquashOnJaHitBlocks++;
    }

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

    if (enableLoopBuffer) {
        lb.recordNewestStreamOutsideLoop(stream);
    }
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

    if (stream.isExit) {
        dbpBtbStats.trapSquashOnLoopPredictorPredExit++;
    }
    if (stream.fromLoopBuffer) {
        dbpBtbStats.squashOnLoopBufferPredBlock++;
        if (stream.isDouble) {
            dbpBtbStats.squashOnLoopBufferDoublePredBlock++;
        }
    }

    stream.resolved = true;
    stream.exeTaken = false;
    stream.squashPC = inst_pc.instAddr();
    stream.squashType = SQUASH_TRAP;

    if (enableJumpAheadPredictor && stream.jaHit) {
        dbpBtbStats.trapSquashOnJaHitBlocks++;
    }

    if (enableLoopPredictor) {
        // recover loop predictor
        // we should check if the numBr possible loop branches should be recovered
        for (int i = 0; i < numBr; ++i) {
            // loop branches behind the squashed branch should be recovered
            if (stream.loopRedirectInfos[i].e.valid && inst_pc.instAddr() <= stream.loopRedirectInfos[i].branch_pc) {
                DPRINTF(DecoupleBP, "Recover loop predictor for %#lx\n", stream.loopRedirectInfos[i].branch_pc);
                lp.recover(stream.loopRedirectInfos[i], false, inst_pc.instAddr(), false, false, currentLoopIter);
            }
        }
        for (auto &info : stream.unseenLoopRedirectInfos) {
            if (info.e.valid && inst_pc.instAddr() <= info.branch_pc) {
                DPRINTF(DecoupleBP, "Recover loop predictor for unseen branch %#lx\n", info.branch_pc);
                lp.recover(info, false, inst_pc.instAddr(), false, false, currentLoopIter);
            }
        }
    }

    squashStreamAfter(stream_id);

    if (enableLoopPredictor) {
        lp.endRepair();
    }

    if (enableLoopBuffer) {
        lb.clearState();
    }

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
    
    if (enableLoopBuffer) {
        lb.recordNewestStreamOutsideLoop(stream);
    }
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
        // if (stream.startPC == ObservingPC) {
        //     debugFlagOn = true;
        // }
        // if (stream.exeBranchPC == ObservingPC2) {
        //     debugFlagOn = true;
        // }
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

// this funtion use finalPred to enq fsq(ftq) and update s0PC
void
DecoupledBPUWithBTB::tryEnqFetchStream()
{
    defer _(nullptr, std::bind([this]{ debugFlagOn = false; }));
    if (s0PC == ObservingPC) {
        debugFlagOn = true;
    }
    if (!receivedPred) {
        DPRINTF(DecoupleBP, "No received prediction, cannot enq fsq\n");
        DPRINTF(Override, "In tryEnqFetchStream(), received is false.\n");
        return;
    } else {
        DPRINTF(Override, "In tryEnqFetchStream(), received is true.\n");
    }
    if (s0PC == MaxAddr) {
        DPRINTF(DecoupleBP, "s0PC %#lx is insane, cannot make prediction\n", s0PC);
        return;
    }
    // prediction valid, but not ready to enq because of bubbles
    if (numOverrideBubbles > 0) {
        DPRINTF(DecoupleBP, "Waiting for bubble caused by overriding, bubbles rest: %u\n", numOverrideBubbles);
        DPRINTF(Override, "Waiting for bubble caused by overriding, bubbles rest: %u\n", numOverrideBubbles);
        return;
    }
    assert(!streamQueueFull());
    if (true) {
        bool should_create_new_stream = true;
        makeNewPrediction(should_create_new_stream);
    } else {
        DPRINTF(DecoupleBP || debugFlagOn, "FSQ is full: %lu\n",
                fetchStreamQueue.size());
    }
    for (int i = 0; i < numStages; i++) {
        predsOfEachStage[i].btbEntries.clear();
    }
    receivedPred = false;
    DPRINTF(Override, "In tryFetchEnqStream(), receivedPred reset to false.\n");
    DPRINTF(DecoupleBP || debugFlagOn, "fsqId=%lu\n", fsqId);
}

void
DecoupledBPUWithBTB::setTakenEntryWithStream(const FetchStream &stream_entry, FtqEntry &ftq_entry)
{
    ftq_entry.taken = true;
    ftq_entry.takenPC = stream_entry.getControlPC();
    ftq_entry.endPC = stream_entry.predEndPC;
    ftq_entry.target = stream_entry.getTakenTarget();
    ftq_entry.inLoop = stream_entry.fromLoopBuffer;
    ftq_entry.iter = stream_entry.isDouble ? 2 : stream_entry.fromLoopBuffer ? 1 : 0;
    ftq_entry.isExit = stream_entry.isExit;
    ftq_entry.loopEndPC = stream_entry.getBranchInfo().getEnd();
}

void
DecoupledBPUWithBTB::setNTEntryWithStream(FtqEntry &ftq_entry, Addr end_pc)
{
    ftq_entry.taken = false;
    ftq_entry.takenPC = 0;
    ftq_entry.target = 0;
    ftq_entry.endPC = end_pc;
    ftq_entry.inLoop = false;
    ftq_entry.iter = 0;
    ftq_entry.isExit = false;
    ftq_entry.loopEndPC = 0;
}

void
DecoupledBPUWithBTB::tryEnqFetchTarget()
{
    DPRINTF(DecoupleBP, "Try to enq fetch target\n");
    if (fetchTargetQueue.full()) {
        DPRINTF(DecoupleBP, "FTQ is full\n");
        return;
    }
    if (fetchStreamQueue.empty()) {
        dbpBtbStats.fsqNotValid++;
        // no stream that have not entered ftq
        DPRINTF(DecoupleBP, "No stream to enter ftq in fetchStreamQueue\n");
        return;
    }
    // ftq can accept new cache lines,
    // try to get cache lines from fetchStreamQueue
    // find current stream with ftqEnqfsqID in fetchStreamQueue
    auto &ftq_enq_state = fetchTargetQueue.getEnqState();
    auto it = fetchStreamQueue.find(ftq_enq_state.streamId);
    if (it == fetchStreamQueue.end()) {
        dbpBtbStats.fsqNotValid++;
        // desired stream not found in fsq
        DPRINTF(DecoupleBP, "FTQ enq desired Stream ID %lu is not found\n",
                ftq_enq_state.streamId);
        return;
    }

    auto &stream_to_enq = it->second;
    Addr end = stream_to_enq.predEndPC;
    DPRINTF(DecoupleBP, "Serve enq PC: %#lx with stream %lu:\n",
            ftq_enq_state.pc, it->first);
    printStream(stream_to_enq);
    

    // We does let ftq to goes beyond fsq now
    if (ftq_enq_state.pc > end) {
        warn("FTQ enq PC %#lx is beyond fsq end %#lx\n",
         ftq_enq_state.pc, end);
    }
    
    assert(ftq_enq_state.pc <= end || (end < predictWidth && (ftq_enq_state.pc + predictWidth < predictWidth)));

    // create a new target entry
    FtqEntry ftq_entry;
    ftq_entry.startPC = ftq_enq_state.pc;
    ftq_entry.fsqID = ftq_enq_state.streamId;

    // set prediction results to ftq entry
    Addr thisFtqEntryShouldEndPC = end;
    bool taken = stream_to_enq.getTaken();
    bool inLoop = stream_to_enq.fromLoopBuffer;
    bool loopExit = stream_to_enq.isExit;
    if (enableJumpAheadPredictor) {
        bool jaHit = stream_to_enq.jaHit;
        if (jaHit) {
            int &currentSentBlock = stream_to_enq.currentSentBlock;
            thisFtqEntryShouldEndPC = stream_to_enq.startPC + (currentSentBlock + 1) * predictWidth;
            currentSentBlock++;
        }
    }
    Addr loopEndPC = stream_to_enq.getBranchInfo().getEnd();
    if (taken) {
        setTakenEntryWithStream(stream_to_enq, ftq_entry);
    } else {
        setNTEntryWithStream(ftq_entry, thisFtqEntryShouldEndPC);
    }

    // update ftq_enq_state
    // if in loop, next pc will either be loop exit or loop start
    ftq_enq_state.pc = inLoop ?
        loopExit ? loopEndPC : stream_to_enq.getBranchInfo().target :
        taken ? stream_to_enq.getBranchInfo().target : thisFtqEntryShouldEndPC;
    
    // we should not increment streamId to enqueue when ja blocks are not fully consumed
    if (!(enableJumpAheadPredictor && stream_to_enq.jaHit && stream_to_enq.jaHit &&
            stream_to_enq.currentSentBlock < stream_to_enq.jaEntry.jumpAheadBlockNum)) {
        ftq_enq_state.streamId++;
    }
    DPRINTF(DecoupleBP,
            "Update ftqEnqPC to %#lx, FTQ demand stream ID to %lu\n",
            ftq_enq_state.pc, ftq_enq_state.streamId);

    fetchTargetQueue.enqueue(ftq_entry);

    assert(ftq_enq_state.streamId <= fsqId + 1);

    // DPRINTF(DecoupleBP, "a%s stream, next enqueue target: %lu\n",
    //         stream_to_enq.getEnded() ? "n ended" : " miss", ftq_enq_state.nextEnqTargetId);
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
    DPRINTF(DecoupleBP, "Try to make new prediction\n");
    FetchStream entry_new;
    auto &entry = entry_new;
    entry.startPC = s0PC;
    defer _(nullptr, std::bind([this]{ debugFlagOn = false; }));
    if (s0PC == ObservingPC) {
        debugFlagOn = true;
    }
    if (finalPred.controlAddr() == ObservingPC || finalPred.controlAddr() == ObservingPC2) {
        debugFlagOn = true;
    }
    // DPRINTF(DecoupleBP || debugFlagOn, "Make pred with %s, pred valid: %i, taken: %i\n",
    //          create_new_stream ? "new stream" : "last missing stream",
    //          finalPred.valid, finalPred.isTaken());

    // if loop buffer is not activated, use normal prediction from branch predictors
    bool endLoop, isDouble, loopConf;
    std::vector<LoopRedirectInfo> lpRedirectInfos(numBr);
    std::vector<bool> fixNotExits(numBr);
    std::vector<LoopRedirectInfo> unseenLpRedirectInfos;
    if (!enableLoopBuffer || (enableLoopBuffer && !lb.isActive())) {
        entry.fromLoopBuffer = false;
        entry.isDouble = false;
        entry.isExit = false;

        bool taken = finalPred.isTaken();
        // bool predReasonable = finalPred.isReasonable();
        // if (predReasonable) {

        Addr fallThroughAddr = finalPred.getFallThrough();
        entry.isHit = finalPred.btbEntries.size() > 0; // TODO: fix isHit and falseHit
        entry.falseHit = false;
        entry.predBTBEntries = finalPred.btbEntries;
        entry.predTaken = taken;
        entry.predEndPC = fallThroughAddr;
        // update s0PC
        Addr nextPC = finalPred.getTarget();
        if (taken) {
            entry.predBranchInfo = finalPred.getTakenEntry().getBranchInfo();
            entry.predBranchInfo.target = nextPC; // use the final target which may be not from btb
        }
        s0PC = nextPC;
        // } else {
        //     DPRINTF(DecoupleBP || debugFlagOn, "Prediction is not reasonable, printing btb entry\n");
        //     btb->printBTBEntry(finalPred.btbEntry);
        //     dbpBtbStats.predFalseHit++;
        //     // prediction is not reasonable, use fall through
        //     entry.isHit = false;
        //     entry.falseHit = true;
        //     entry.predTaken = false;
        //     entry.predEndPC = entry.startPC + 32;
        //     entry.predBTBEntry = BTBEntry();
        //     s0PC = entry.startPC + 32; // TODO: parameterize
        //     // TODO: when false hit, act like a miss, do not update history
        // }

        // jump ahead lookups
        if (enableJumpAheadPredictor) {
            bool jaHit = false;
            bool jaConf = false;
            JAEntry jaEntry;
            Addr jaTarget;
            std::tie(jaHit, jaConf, jaEntry, jaTarget) = jap.lookup(entry.startPC);
            // ensure this block does not hit btb
            // TODO: fix this
            // if (!finalPred.valid) {
            //     if (jaHit && jaConf) {
            //         entry.jaHit = true;
            //         entry.predEndPC = jaTarget;
            //         entry.jaEntry = jaEntry;
            //         entry.currentSentBlock = 0;
            //         s0PC = jaTarget;
            //         dbpBtbStats.predJATotalSkippedBlocks += jaEntry.jumpAheadBlockNum - 1;
            //         dbpBtbStats.predJASkippedBlockNum.sample(jaEntry.jumpAheadBlockNum - 1, 1);
            //     }
            // }
        }


        entry.history = s0History;
        entry.predTick = finalPred.predTick;
        entry.predSource = finalPred.predSource;

        // update (folded) histories for components
        for (int i = 0; i < numComponents; i++) {
            components[i]->specUpdateHist(s0History, finalPred);
            entry.predMetas[i] = components[i]->getPredictionMeta();
        }
        // update ghr
        int shamt;
        std::tie(shamt, taken) = finalPred.getHistInfo();
        // boost::to_string(s0History, buf1);
        histShiftIn(shamt, taken, s0History);
        // boost::to_string(s0History, buf2);

        historyManager.addSpeculativeHist(entry.startPC, shamt, taken, entry.predBranchInfo, fsqId);
        tage->checkFoldedHist(s0History, "speculative update");
        
        entry.setDefaultResolve();


    }
    entry.loopRedirectInfos = lpRedirectInfos;
    entry.fixNotExits = fixNotExits;
    entry.unseenLoopRedirectInfos = unseenLpRedirectInfos;

    auto [insert_it, inserted] = fetchStreamQueue.emplace(fsqId, entry);
    assert(inserted);

    dumpFsq("after insert new stream");
    DPRINTF(DecoupleBP || debugFlagOn, "Insert fetch stream %lu\n", fsqId);

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
