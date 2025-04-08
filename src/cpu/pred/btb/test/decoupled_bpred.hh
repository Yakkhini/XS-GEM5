#ifndef __CPU_PRED_BTB_DECOUPLED_BPRED_HH__
#define __CPU_PRED_BTB_DECOUPLED_BPRED_HH__

#include <array>
#include <queue>
#include <stack>
#include <utility>
#include <vector>

#include "cpu/pred/btb/fetch_target_queue.hh"
#include "cpu/pred/btb/history_manager.hh"
#include "cpu/pred/btb/jump_ahead_predictor.hh"
#include "cpu/pred/btb/loop_buffer.hh"
#include "cpu/pred/btb/loop_predictor.hh"
#include "cpu/pred/btb/stream_struct.hh"
#include "cpu/pred/btb/test/btb_tage.hh"
#include "cpu/pred/btb/test/mock_PCState.hh"
#include "cpu/pred/btb/test/mockbtb.hh"
#include "cpu/pred/btb/test/test_dprintf.hh"
#include "cpu/pred/btb/test/timed_base_pred.hh"

namespace gem5
{

namespace branch_prediction
{

namespace btb_pred
{

namespace test
{

/**
 * @class DecoupledBPUWithBTB
 * @brief A decoupled branch predictor implementation using BTB-based design
 *
 * This predictor implements a decoupled front-end with:
 * - Multiple prediction stages (UBTB -> BTB/TAGE/ITTAGE)
 * - Fetch Target Queue (FTQ) for managing predicted targets
 * - Fetch Stream Queue (FSQ) for managing instruction streams
 * - Support for loop prediction and jump-ahead prediction
 */
class DecoupledBPUWithBTB
{
    using defer = std::shared_ptr<void>;
  public:
    // typedef DecoupledBPUWithBTBParams Params;

    DecoupledBPUWithBTB();

    LoopPredictor lp;
    LoopBuffer lb;
    bool enableLoopBuffer{false};
    bool enableLoopPredictor{false};

    JumpAheadPredictor jap;
    bool enableJumpAheadPredictor{false};

// for testing
  public:
    std::string _name;

    FetchTargetQueue fetchTargetQueue;

    std::map<FetchStreamId, FetchStream> fetchStreamQueue;
    unsigned fetchStreamQueueSize;
    FetchStreamId fsqId{1};
    FetchStream lastCommittedStream;

    unsigned numBr;
    bool alignToBlockSize;

    unsigned cacheLineOffsetBits{6};  // TODO: parameterize this
    unsigned cacheLineSize{64};

    const unsigned historyTokenBits{8};

    constexpr Addr foldingTokenMask() { return (1 << historyTokenBits) - 1; }

    constexpr unsigned numFoldingTokens() { return 64/historyTokenBits; }

    const unsigned historyBits{64}; // TODO: for test!

    const Addr MaxAddr{~(0ULL)};

    DefaultBTB *ubtb{};
    // DefaultBTB *abtb{};
    DefaultBTB *btb{};
    BTBTAGE *tage{};

    // bool enableDB;
    std::vector<std::string> bpDBSwitches;
    bool someDBenabled{false};
    bool enableBranchTrace{false};
    bool enableLoopDB{false};
    bool checkGivenSwitch(std::vector<std::string> switches, std::string switchName) {
        for (auto &sw : switches) {
            if (sw == switchName) {
                return true;
            }
        }
        return false;
    }
    void removeGivenSwitch(std::vector<std::string> &switches, std::string switchName) {
        auto it = std::remove(switches.begin(), switches.end(), switchName);
        switches.erase(it, switches.end());
    }


    std::vector<TimedBaseBTBPredictor*> components{};
    std::vector<FullBTBPrediction> predsOfEachStage{};
    unsigned numComponents{};
    unsigned numStages{};

    bool sentPCHist{false};     ///< get prediction from BP
    bool receivedPred{false};   ///< get final prediction from predsOfEachStage[numStages-1]

    Addr s0PC;                  ///< Current PC
    // Addr s0StreamStartPC;
    boost::dynamic_bitset<> s0History;  ///< History bits
    FullBTBPrediction finalPred;      ///< Final prediction

    boost::dynamic_bitset<> commitHistory;

    bool squashing{false};

    HistoryManager historyManager;

    unsigned numOverrideBubbles{0};


    using JAInfo = JumpAheadPredictor::JAInfo;
    JAInfo jaInfo;

    void tryEnqFetchStream();

    void tryEnqFetchTarget();

    void makeNewPrediction(bool create_new_stream);

    void makeLoopPredictions(FetchStream &entry, bool &endLoop, bool &isDouble, bool &loopConf,
        std::vector<LoopRedirectInfo> &lpRedirectInfos, std::vector<bool> &fixNotExits,
        std::vector<LoopRedirectInfo> &unseenLpRedirectInfos, bool &taken);

    Addr alignToCacheLine(Addr addr)
    {
        return addr & ~((1 << cacheLineOffsetBits) - 1);
    }

    bool crossCacheLine(Addr addr)
    {
        return (addr & (1 << (cacheLineOffsetBits - 1))) != 0;
    }

    Addr computePathHash(Addr br, Addr target);

    // TODO: compare phr and ghr
    void histShiftIn(int shamt, bool taken, boost::dynamic_bitset<> &history);

    void printStream(const FetchStream &e)
    {
        if (!e.resolved) {
            DPRINTFR(DecoupleBP, "FSQ Predicted stream: ");
        } else {
            DPRINTFR(DecoupleBP, "FSQ Resolved stream: ");
        }
        // TODO:fix this
        DPRINTFR(DecoupleBP,
                 "%#lx-[%#lx, %#lx) --> %#lx, taken: %lu\n",
                 e.startPC, e.getBranchInfo().pc, e.getEndPC(),
                 e.getTakenTarget(), e.getTaken());
    }

    void printStreamFull(const FetchStream &e)
    {
        // TODO: fix this
        // DPRINTFR(
        //     DecoupleBP,
        //     "FSQ prediction:: %#lx-[%#lx, %#lx) --> %#lx\n",
        //     e.startPC, e.predBranchPC, e.predEndPC, e.predTarget);
        // DPRINTFR(
        //     DecoupleBP,
        //     "Resolved: %i, resolved stream:: %#lx-[%#lx, %#lx) --> %#lx\n",
        //     e.exeEnded, e.startPC, e.exeBranchPC, e.exeEndPC,
        //     e.exeTarget);
    }

    void printFetchTarget(const FtqEntry &e, const char *when)
    {
        DPRINTFR(DecoupleBP,
                 "%s:: %#lx - [%#lx, %#lx) --> %#lx, taken: %d, fsqID: %lu, loop: %d, iter: %d, exit: %d\n",
                 when, e.startPC, e.takenPC, e.endPC, e.target, e.taken,
                 e.fsqID, e.inLoop, e.iter, e.isExit);
    }

    void printFetchTargetFull(const FtqEntry &e)
    {
        DPRINTFR(DecoupleBP, "Fetch Target:: %#lx-[%#lx, %#lx) --> %#lx\n",
                 e.startPC, e.takenPC, e.endPC, e.target);
    }

    bool streamQueueFull() const
    {
        return fetchStreamQueue.size() >= fetchStreamQueueSize;
    }

    /**
     * @brief Generate final prediction from all stages
     *
     * Collects predictions from all stages and:
     * - Selects most accurate prediction
     * - Generates necessary bubbles
     * - Updates prediction state
     */
    void generateFinalPredAndCreateBubbles();

    void clearPreds() {
        for (auto &stagePred : predsOfEachStage) {
            stagePred.condTakens.clear();
            stagePred.indirectTargets.clear();
            stagePred.btbEntries.clear();
        }
    }

    // const bool dumpLoopPred;

    void printBTBEntry(const BTBEntry &e) {
        DPRINTF(BTB, "BTB entry: valid %d, pc:%#lx, tag: %#lx, size:%d, target:%#lx, cond:%d, indirect:%d, call:%d, return:%d, always_taken:%d\n",
            e.valid, e.pc, e.tag, e.size, e.target, e.isCond, e.isIndirect, e.isCall, e.isReturn, e.alwaysTaken);
    }

    void printFullBTBPrediction(const int stage, const FullBTBPrediction &pred) {
        DPRINTF(DecoupleBP, "dumping FullBTBPrediction stage %d\n", stage);
        DPRINTF(DecoupleBP, "bbStart: %#lx, btbEntry:\n", pred.bbStart);
        for (auto &e: pred.btbEntries) {
            printBTBEntry(e);
        }
        DPRINTF(DecoupleBP, "condTakens: ");
        for (auto pair : pred.condTakens) {
            DPRINTFR(DecoupleBP, "%#lx %d ,", pair.first, pair.second);
        }
        DPRINTFR(DecoupleBP, "\n");
        for (auto pair : pred.indirectTargets) {
            DPRINTF(DecoupleBP, "indirectTarget of %#lx: %#lx\n",
            pair.first, pair.second);
        }
        DPRINTF(DecoupleBP, "returnTarget %#lx\n", pred.returnTarget);
    }

    /**
     * @brief Statistics collection for branch prediction
     * 
     * Tracks detailed statistics about:
     * - Branch types and mispredictions
     * - Predictor component usage
     * - Queue utilization
     * - Loop and jump-ahead prediction performance
     */
    struct DBPBTBStats {
        // Branch type statistics
        uint64_t condNum;      ///< Number of conditional branches
        uint64_t uncondNum;    ///< Number of unconditional branches
        uint64_t returnNum;    ///< Number of return instructions
        uint64_t otherNum;     ///< Number of other control instructions

        // Misprediction statistics
        uint64_t condMiss;     ///< Conditional branch mispredictions
        uint64_t uncondMiss;   ///< Unconditional branch mispredictions
        uint64_t returnMiss;   ///< Return mispredictions
        uint64_t otherMiss;    ///< Other control mispredictions

        // Branch coverage statistics
        uint64_t staticBranchNum;           ///< Total static branches seen
        uint64_t staticBranchNumEverTaken;  ///< Static branches ever taken

        // statistics::Vector predsOfEachStage;
        uint64_t overrideBubbleNum;
        uint64_t overrideCount;
        // Track override reasons
        uint64_t overrideValidityMismatch;
        uint64_t overrideControlAddrMismatch;
        uint64_t overrideTargetMismatch;
        uint64_t overrideEndMismatch;
        uint64_t overrideHistInfoMismatch;
        // statistics::Vector commitPredsFromEachStage;
        // statistics::Distribution fsqEntryDist;
        uint64_t fsqEntryEnqueued;
        uint64_t fsqEntryCommitted;
        // statistics::Distribution ftqEntryDist;
        uint64_t controlSquash;
        uint64_t nonControlSquash;
        uint64_t trapSquash;

        uint64_t ftqNotValid;
        uint64_t fsqNotValid;
        uint64_t fsqFullCannotEnq;
        //
        // statistics::Distribution commitFsqEntryHasInsts;
        // write back once an fsq entry finishes fetch
        // statistics::Distribution commitFsqEntryFetchedInsts;
        uint64_t commitFsqEntryOnlyHasOneJump;

        uint64_t btbHit;
        uint64_t btbMiss;
        uint64_t btbEntriesWithDifferentStart;
        uint64_t btbEntriesWithOnlyOneJump;

        uint64_t predFalseHit;
        uint64_t commitFalseHit;

        uint64_t predLoopPredictorExit;
        uint64_t predLoopPredictorUnconfNotExit;
        uint64_t predLoopPredictorConfFixNotExit;
        uint64_t predBTBUnseenLoopBranchInLp;
        uint64_t predBTBUnseenLoopBranchExitInLp;
        uint64_t commitLoopPredictorExit;
        uint64_t commitLoopPredictorExitCorrect;
        uint64_t commitLoopPredictorExitWrong;
        uint64_t commitBTBUnseenLoopBranchInLp;
        uint64_t commitBTBUnseenLoopBranchExitInLp;
        uint64_t commitLoopPredictorConfFixNotExit;
        uint64_t commitLoopPredictorConfFixNotExitCorrect;
        uint64_t commitLoopPredictorConfFixNotExitWrong;
        uint64_t commitLoopExitLoopPredictorNotPredicted;
        uint64_t commitLoopExitLoopPredictorNotConf;
        uint64_t controlSquashOnLoopPredictorPredExit;
        uint64_t nonControlSquashOnLoopPredictorPredExit;
        uint64_t trapSquashOnLoopPredictorPredExit;

        uint64_t predBlockInLoopBuffer;
        uint64_t predDoubleBlockInLoopBuffer;
        uint64_t squashOnLoopBufferPredBlock;
        uint64_t squashOnLoopBufferDoublePredBlock;
        uint64_t commitBlockInLoopBuffer;
        uint64_t commitDoubleBlockInLoopBuffer;
        uint64_t commitBlockInLoopBufferSquashed;
        uint64_t commitDoubleBlockInLoopBufferSquashed;
        // statistics::Distribution commitLoopBufferEntryInstNum;
        // statistics::Distribution commitLoopBufferDoubleEntryInstNum;

        uint64_t predJATotalSkippedBlocks;
        uint64_t commitJATotalSkippedBlocks;
        uint64_t squashOnJaHitBlocks;
        uint64_t controlSquashOnJaHitBlocks;
        uint64_t nonControlSquashOnJaHitBlocks;
        uint64_t trapSquashOnJaHitBlocks;
        uint64_t commitSquashedOnJaHitBlocks;
        uint64_t commitControlSquashedOnJaHitBlocks;
        uint64_t commitNonControlSquashedOnJaHitBlocks;
        uint64_t commitTrapSquashedOnJaHitBlocks;
        // statistics::Distribution predJASkippedBlockNum;
        // statistics::Distribution commitJASkippedBlockNum;

    } dbpBtbStats;

  public:
    /**
     * @brief Main prediction cycle function
     * 
     * This function handles:
     * - FSQ/FTQ management
     * - Prediction generation
     * - Loop buffer management
     * - Statistics collection
     */
    void tick();

    bool trySupplyFetchWithTarget(Addr fetch_demand_pc, bool &fetchTargetInLoop);

    void squash(const InstSeqNum &squashed_sn, ThreadID tid)
    {
        panic("Squashing decoupled BP with tightly coupled API\n");
    }
    void squash(const InstSeqNum &squashed_sn, const PCStateBase &corr_target,
                bool actually_taken, ThreadID tid)
    {
        panic("Squashing decoupled BP with tightly coupled API\n");
    }

    std::pair<bool, bool> decoupledPredict(const StaticInstPtr &inst,
                                           const InstSeqNum &seqNum,
                                           PCStateBase &pc, ThreadID tid,
                                           unsigned &currentLoopIter);

    // redirect the stream
    void controlSquash(unsigned ftq_id, unsigned fsq_id,
                       const PCStateBase &control_pc,
                       const PCStateBase &target_pc,
                       const StaticInstPtr &static_inst, unsigned inst_bytes,
                       bool actually_taken, const InstSeqNum &squashed_sn,
                       ThreadID tid, const unsigned &currentLoopIter,
                       const bool fromCommit);

    // keep the stream: original prediction might be right
    // For memory violation, stream continues after squashing
    void nonControlSquash(unsigned ftq_id, unsigned fsq_id,
                          const PCStateBase &inst_pc, const InstSeqNum seq,
                          ThreadID tid, const unsigned &currentLoopIter);

    // Not a control. But stream is actually disturbed
    void trapSquash(unsigned ftq_id, unsigned fsq_id, Addr last_committed_pc,
                    const PCStateBase &inst_pc, ThreadID tid, const unsigned &currentLoopIter);

    void update(unsigned fsqID, ThreadID tid);

    void squashStreamAfter(unsigned squash_stream_id);

    bool fetchTargetAvailable()
    {
        return fetchTargetQueue.fetchTargetAvailable();
    }

    FtqEntry& getSupplyingFetchTarget()
    {
        return fetchTargetQueue.getTarget();
    }

    unsigned getSupplyingTargetId()
    {
        return fetchTargetQueue.getSupplyingTargetId();
    }
    unsigned getSupplyingStreamId()
    {
        return fetchTargetQueue.getSupplyingStreamId();
    }

    void dumpFsq(const char *when);

    // Dummy overriding
    void uncondBranch(ThreadID tid, Addr pc, void *&bp_history) {}

    void squash(ThreadID tid, void *bp_history) {}

    void btbUpdate(ThreadID tid, Addr instPC, void *&bp_history) {}

    void update(ThreadID tid, Addr instPC, bool taken, void *bp_history,
                bool squashed, const StaticInstPtr &inst,
                Addr corrTarget)
    {
    }

    bool lookup(ThreadID tid, Addr instPC, void *&bp_history) { return false; }

    void checkHistory(const boost::dynamic_bitset<> &history);

    bool useStreamRAS(FetchStreamId sid);

    // Addr getPreservedReturnAddr(const DynInstPtr &dynInst);

    std::string buf1, buf2;

    std::stack<Addr> streamRAS;
    
    bool debugFlagOn{false};
    std::map<Addr, std::pair<BTBEntry, int>> totalBTBEntries;
    void setTakenEntryWithStream(const FetchStream &stream_entry, FtqEntry &ftq_entry);

    void setNTEntryWithStream(FtqEntry &ftq_entry, Addr endPC);

    bool popRAS(FetchStreamId stream_id, const char *when);

    void pushRAS(FetchStreamId stream_id, const char *when, Addr ra);

    void updateTAGE(FetchStream &stream);

    std::vector<Addr> storeTargets;

    void resetPC(Addr new_pc);

    void addFtqNotValid() {
        dbpBtbStats.ftqNotValid++;
    }

    // void commitBranch(const DynInstPtr &inst, bool miss);

    // void notifyInstCommit(const DynInstPtr &inst);

    std::map<Addr, unsigned> topMispredIndirect;
    int currentFtqEntryInstNum;

};

} // namespace test
}  // namespace btb_pred
}  // namespace branch_prediction
}  // namespace gem5

#endif // __CPU_PRED_BTB_DECOUPLED_BPRED_HH__
