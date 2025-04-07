#ifndef __CPU_PRED_BTB_DECOUPLED_BPRED_HH__
#define __CPU_PRED_BTB_DECOUPLED_BPRED_HH__

#include <array>
#include <queue>
#include <stack>
#include <utility> 
#include <vector>

#include "arch/generic/pcstate.hh"
#include "config/the_isa.hh"
#include "cpu/o3/cpu_def.hh"
#include "cpu/o3/dyn_inst_ptr.hh"
#include "cpu/pred/bpred_unit.hh"
#include "cpu/pred/general_arch_db.hh"
#include "cpu/pred/btb/fetch_target_queue.hh"
#include "cpu/pred/btb/btb.hh"
#include "cpu/pred/btb/btb_tage.hh"
#include "cpu/pred/btb/btb_ittage.hh"
#include "cpu/pred/btb/jump_ahead_predictor.hh"
#include "cpu/pred/btb/loop_predictor.hh"
#include "cpu/pred/btb/loop_buffer.hh"
#include "cpu/pred/btb/ras.hh"
// #include "cpu/pred/btb/uras.hh"
#include "cpu/pred/btb/stream_struct.hh"
#include "cpu/pred/btb/timed_base_pred.hh"
#include "debug/DecoupleBP.hh"
#include "debug/DecoupleBPHist.hh"
#include "debug/DecoupleBPProbe.hh"
#include "debug/DecoupleBPRAS.hh"
#include "debug/DecoupleBPuRAS.hh"
#include "debug/DecoupleBPVerbose.hh"
#include "debug/DBPBTBStats.hh"
#include "debug/JumpAheadPredictor.hh"
#include "debug/LoopBuffer.hh"
#include "debug/LoopPredictor.hh"
#include "debug/LoopPredictorVerbose.hh"
#include "params/DecoupledBPUWithBTB.hh"

namespace gem5
{

namespace branch_prediction
{

namespace btb_pred
{

using DynInstPtr = o3::DynInstPtr;
using CPU = o3::CPU;

/**
 * @class HistoryManager
 * @brief Manages branch prediction history including speculative updates and recovery
 * 
 * This class tracks branch prediction history and handles:
 * - Speculative history updates for predicted branches
 * - History recovery on misprediction
 * - History commit when branches are resolved
 */
class HistoryManager
{
  public:
    struct HistoryEntry
    {
        HistoryEntry(Addr _pc, int _shamt, bool _cond_taken, bool _is_call, bool _is_return,
            Addr _retAddr, uint64_t stream_id)
            : pc(_pc), shamt(_shamt), cond_taken(_cond_taken), is_call(_is_call),
                is_return(_is_return), retAddr(_retAddr), streamId(stream_id)
        {
        }
      Addr pc;
      Addr shamt;
      bool cond_taken;
      bool is_call;
      bool is_return;
      Addr retAddr;
      uint64_t streamId;
    };

    HistoryManager(unsigned _maxShamt) : maxShamt(_maxShamt) {}

  private:
    std::list<HistoryEntry> speculativeHists;

    unsigned IdealHistLen{246};

    unsigned maxShamt;

  public:
    void addSpeculativeHist(const Addr addr, const int shamt,
                            bool cond_taken, BranchInfo &bi,
                            const uint64_t stream_id)
    {
        bool is_call = bi.isCall;
        bool is_return = bi.isReturn;
        Addr retAddr = bi.getEnd();

        speculativeHists.emplace_back(addr, shamt, cond_taken, is_call,
            is_return, retAddr, stream_id);

        const auto &it = speculativeHists.back();
        printEntry("Add", it);

    }


    void commit(const uint64_t stream_id)
    {
        auto it = speculativeHists.begin();
        while (it != speculativeHists.end()) {
            if (it->streamId <= stream_id) {
                printEntry("Commit", *it);
                it = speculativeHists.erase(it);
            } else {
                ++it;
            }
        }
    }

    const std::list<HistoryEntry> &getSpeculativeHist()
    {
        return speculativeHists;
    }

    void squash(const uint64_t stream_id, const int shamt,
                const bool cond_taken, BranchInfo bi)
    {
        dump("before squash");
        auto it = speculativeHists.begin();
        while (it != speculativeHists.end()) {
            // why is it empty in logs?
            if (it->streamId == stream_id) {
                it->cond_taken = cond_taken;
                it->shamt = shamt;
                bool is_call = bi.isCall;
                bool is_return = bi.isReturn;
                Addr retAddr = bi.getEnd();
                it->is_call = is_call;
                it->is_return = is_return;
                it->retAddr = retAddr;
            } if (it->streamId > stream_id) {
                printEntry("Squash", *it);
                it = speculativeHists.erase(it);
            } else {
                DPRINTF(DecoupleBPVerbose,
                        "Skip stream %i when squashing stream %i\n",
                        it->streamId, stream_id);
                ++it;
            }
        }
        dump("after squash");
        checkSanity();
    }

    void checkSanity()
    {
        if (speculativeHists.size() < 2) {
            return;
        }
        auto last = speculativeHists.begin();
        auto cur = speculativeHists.begin();
        cur++;
        while (cur != speculativeHists.end()) {
            if (cur->shamt > maxShamt) {
                dump("before warn");
                warn("entry shifted more than %d bits\n", maxShamt);
            }
            last = cur;
            cur++;
        }
    }

    void dump(const char* when)
    {
        DPRINTF(DecoupleBPVerbose, "Dump ideal history %s:\n", when);
        for (auto it = speculativeHists.begin(); it != speculativeHists.end();
             it++) {
            printEntry("", *it);
        }
    }

    void printEntry(const char* when, const HistoryEntry& entry)
    {
        DPRINTF(DecoupleBPVerbose, "%s stream: %lu, pc %#lx, shamt %d, cond_taken %d, is_call %d, is_ret %d, retAddr %#lx\n",
            when, entry.streamId, entry.pc, entry.shamt, entry.cond_taken, entry.is_call, entry.is_return, entry.retAddr);
    }
};

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
class DecoupledBPUWithBTB : public BPredUnit
{
    using defer = std::shared_ptr<void>;
  public:
    typedef DecoupledBPUWithBTBParams Params;

    DecoupledBPUWithBTB(const Params &params);
    LoopPredictor lp;
    LoopBuffer lb;
    bool enableLoopBuffer{false};
    bool enableLoopPredictor{false};

    JumpAheadPredictor jap;
    bool enableJumpAheadPredictor{true};

  private:
    std::string _name;

    FetchTargetQueue fetchTargetQueue;

    std::map<FetchStreamId, FetchStream> fetchStreamQueue;
    unsigned fetchStreamQueueSize;
    FetchStreamId fsqId{1};
    FetchStream lastCommittedStream;

    CPU *cpu;

    unsigned numBr;
    bool alignToBlockSize;

    unsigned cacheLineOffsetBits{6};  // TODO: parameterize this
    unsigned cacheLineSize{64};

    const unsigned historyTokenBits{8};

    constexpr Addr foldingTokenMask() { return (1 << historyTokenBits) - 1; }

    constexpr unsigned numFoldingTokens() { return 64/historyTokenBits; }

    const unsigned historyBits{488};

    const Addr MaxAddr{~(0ULL)};

    DefaultBTB *ubtb{};
    DefaultBTB *abtb{};
    DefaultBTB *btb{};
    BTBTAGE *tage{};
    BTBITTAGE *ittage{};
    
    btb_pred::BTBRAS *ras{};
    // btb_pred::BTBuRAS *uras{};

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
    DataBase bpdb;
    TraceManager *bptrace;
    TraceManager *lptrace;




    std::vector<TimedBaseBTBPredictor*> components{};
    std::vector<FullBTBPrediction> predsOfEachStage{};
    unsigned numComponents{};
    unsigned numStages{};

    bool sentPCHist{false};     ///< PC/history sent this cycle
    bool receivedPred{false};   ///< Prediction received this cycle

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
                 "%#lx-[%#lx, %#lx) --> %#lx, taken: %i\n",
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

    void printFullBTBPrediction(const FullBTBPrediction &pred) {
        DPRINTF(DecoupleBP, "dumping FullBTBPrediction\n");
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
    struct DBPBTBStats : public statistics::Group {
        // Branch type statistics
        statistics::Scalar condNum;      ///< Number of conditional branches
        statistics::Scalar uncondNum;    ///< Number of unconditional branches
        statistics::Scalar returnNum;    ///< Number of return instructions
        statistics::Scalar otherNum;     ///< Number of other control instructions

        // Misprediction statistics
        statistics::Scalar condMiss;     ///< Conditional branch mispredictions
        statistics::Scalar uncondMiss;   ///< Unconditional branch mispredictions
        statistics::Scalar returnMiss;   ///< Return mispredictions
        statistics::Scalar otherMiss;    ///< Other control mispredictions

        // Branch coverage statistics
        statistics::Scalar staticBranchNum;           ///< Total static branches seen
        statistics::Scalar staticBranchNumEverTaken;  ///< Static branches ever taken

        statistics::Vector predsOfEachStage;
        statistics::Scalar overrideBubbleNum;
        statistics::Scalar overrideCount;
        statistics::Vector commitPredsFromEachStage;
        statistics::Distribution fsqEntryDist;
        statistics::Scalar fsqEntryEnqueued;
        statistics::Scalar fsqEntryCommitted;
        // statistics::Distribution ftqEntryDist;
        statistics::Scalar controlSquash;
        statistics::Scalar nonControlSquash;
        statistics::Scalar trapSquash;

        statistics::Scalar ftqNotValid;
        statistics::Scalar fsqNotValid;
        statistics::Scalar fsqFullCannotEnq;
        // 
        statistics::Distribution commitFsqEntryHasInsts;
        // write back once an fsq entry finishes fetch
        statistics::Distribution commitFsqEntryFetchedInsts;
        statistics::Scalar commitFsqEntryOnlyHasOneJump;

        statistics::Scalar btbHit;
        statistics::Scalar btbMiss;
        statistics::Scalar btbEntriesWithDifferentStart;
        statistics::Scalar btbEntriesWithOnlyOneJump;

        statistics::Scalar predFalseHit;
        statistics::Scalar commitFalseHit;

        statistics::Scalar predLoopPredictorExit;
        statistics::Scalar predLoopPredictorUnconfNotExit;
        statistics::Scalar predLoopPredictorConfFixNotExit;
        statistics::Scalar predBTBUnseenLoopBranchInLp;
        statistics::Scalar predBTBUnseenLoopBranchExitInLp;
        statistics::Scalar commitLoopPredictorExit;
        statistics::Scalar commitLoopPredictorExitCorrect;
        statistics::Scalar commitLoopPredictorExitWrong;
        statistics::Scalar commitBTBUnseenLoopBranchInLp;
        statistics::Scalar commitBTBUnseenLoopBranchExitInLp;
        statistics::Scalar commitLoopPredictorConfFixNotExit;
        statistics::Scalar commitLoopPredictorConfFixNotExitCorrect;
        statistics::Scalar commitLoopPredictorConfFixNotExitWrong;
        statistics::Scalar commitLoopExitLoopPredictorNotPredicted;
        statistics::Scalar commitLoopExitLoopPredictorNotConf;
        statistics::Scalar controlSquashOnLoopPredictorPredExit;
        statistics::Scalar nonControlSquashOnLoopPredictorPredExit;
        statistics::Scalar trapSquashOnLoopPredictorPredExit;

        statistics::Scalar predBlockInLoopBuffer;
        statistics::Scalar predDoubleBlockInLoopBuffer;
        statistics::Scalar squashOnLoopBufferPredBlock;
        statistics::Scalar squashOnLoopBufferDoublePredBlock;
        statistics::Scalar commitBlockInLoopBuffer;
        statistics::Scalar commitDoubleBlockInLoopBuffer;
        statistics::Scalar commitBlockInLoopBufferSquashed;
        statistics::Scalar commitDoubleBlockInLoopBufferSquashed;
        statistics::Distribution commitLoopBufferEntryInstNum;
        statistics::Distribution commitLoopBufferDoubleEntryInstNum;

        statistics::Scalar predJATotalSkippedBlocks;
        statistics::Scalar commitJATotalSkippedBlocks;
        statistics::Scalar squashOnJaHitBlocks;
        statistics::Scalar controlSquashOnJaHitBlocks;
        statistics::Scalar nonControlSquashOnJaHitBlocks;
        statistics::Scalar trapSquashOnJaHitBlocks;
        statistics::Scalar commitSquashedOnJaHitBlocks;
        statistics::Scalar commitControlSquashedOnJaHitBlocks;
        statistics::Scalar commitNonControlSquashedOnJaHitBlocks;
        statistics::Scalar commitTrapSquashedOnJaHitBlocks;
        statistics::Distribution predJASkippedBlockNum;
        statistics::Distribution commitJASkippedBlockNum;

        DBPBTBStats(statistics::Group* parent, unsigned numStages, unsigned fsqSize);
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

    void setCpu(CPU *_cpu) { cpu = _cpu; }

    struct BpTrace : public Record {
        void set(uint64_t startPC, uint64_t controlPC, uint64_t controlType,
            uint64_t taken, uint64_t mispred, uint64_t fallThruPC,
            uint64_t source, uint64_t target) {
            _uint64_data["startPC"] = startPC;
            _uint64_data["controlPC"] = controlPC;
            _uint64_data["controlType"] = controlType;
            _uint64_data["taken"] = taken;
            _uint64_data["mispred"] = mispred;
            _uint64_data["fallThruPC"] = fallThruPC;
            _uint64_data["source"] = source;
            _uint64_data["target"] = target;
        }
        BpTrace(FetchStream &stream, const DynInstPtr &inst, bool mispred);
    };

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
    void uncondBranch(ThreadID tid, Addr pc, void *&bp_history) override {}

    void squash(ThreadID tid, void *bp_history) override {}

    void btbUpdate(ThreadID tid, Addr instPC, void *&bp_history) override {}

    void update(ThreadID tid, Addr instPC, bool taken, void *bp_history,
                bool squashed, const StaticInstPtr &inst,
                Addr corrTarget) override
    {
    }

    bool lookup(ThreadID tid, Addr instPC, void *&bp_history) override { return false; }

    void checkHistory(const boost::dynamic_bitset<> &history);

    bool useStreamRAS(FetchStreamId sid);

    Addr getPreservedReturnAddr(const DynInstPtr &dynInst);

    std::string buf1, buf2;

    std::stack<Addr> streamRAS;
    
    bool debugFlagOn{false};

    std::map<Addr, int> takenBranches;
    std::map<Addr, int> currentPhaseTakenBranches;
    std::map<Addr, int> currentSubPhaseTakenBranches;

    /**
     * @brief Types of control flow instruction mispredictions
     */
    enum MispredType {
        DIR_WRONG,      ///< Direction prediction was wrong
        TARGET_WRONG,   ///< Target address prediction was wrong  
        NO_PRED,        ///< No prediction was made
        FAKE_LAST       ///< Sentinel value
    };
    using MispredReasonMap = std::map<MispredType, int>;
    //                         mispred cnt
    using MispredDesc = std::pair<int, MispredReasonMap>;
    //                                    ((mispredict, reason_map),        total)
    using MispredData = std::pair<MispredDesc, int>;
    //                             (pc, type) 
    using MispredIndex = std::pair<Addr, int>;
    using MispredRecord = std::pair<MispredIndex, MispredData>;
    using MispredMap = std::map<MispredIndex, MispredData>;
    // int getMispredCount(MispredData &data) { return data.first.first; }
    int getMispredCount(const MispredRecord &data) { return data.second.first.first; }
    
    std::map<std::pair<Addr, Addr>, int> topMispredicts;
    MispredMap topMispredictsByBranch;
    std::map<uint64_t, uint64_t> topMispredHist;
    std::map<int, int> misPredTripCount;

    MispredMap lastPhaseTopMispredictsByBranch;
    std::vector<MispredMap> topMispredictsByBranchByPhase;
    std::vector<std::map<Addr, int>> takenBranchesByPhase;

    //      startPC          entry    visited
    std::map<Addr, std::pair<BTBEntry, int>> lastPhaseBTBEntries;
    std::map<Addr, std::pair<BTBEntry, int>> totalBTBEntries;
    std::vector<std::map<Addr, std::pair<BTBEntry, int>>> BTBEntriesByPhase;

    int phaseIdToDump{1};
    int numInstCommitted{0};
    int phaseSizeByInst{100000};
    int subPhaseIdToDump{1};
    int subPhaseRatio{10};
    int subPhaseSizeByInst() { return phaseSizeByInst/subPhaseRatio; }

    std::vector<int> lastPhaseFsqEntryNumCommittedInstDist;
    std::vector<int> commitFsqEntryHasInstsVector;
    std::vector<std::vector<int>> fsqEntryNumCommittedInstDistByPhase;
    std::vector<int> lastPhaseFsqEntryNumFetchedInstDist;
    std::vector<int> commitFsqEntryFetchedInstsVector;
    std::vector<std::vector<int>> fsqEntryNumFetchedInstDistByPhase;
    unsigned int missCount{0};

    MispredMap lastSubPhaseTopMispredictsByBranch;
    std::vector<MispredMap> topMispredictsByBranchBySubPhase;
    std::vector<std::map<Addr, int>> takenBranchesBySubPhase;
    

    void setTakenEntryWithStream(const FetchStream &stream_entry, FtqEntry &ftq_entry);

    void setNTEntryWithStream(FtqEntry &ftq_entry, Addr endPC);

    bool popRAS(FetchStreamId stream_id, const char *when);

    void pushRAS(FetchStreamId stream_id, const char *when, Addr ra);

    void updateTAGE(FetchStream &stream);

    std::vector<Addr> storeTargets;

    void resetPC(Addr new_pc);

    enum CfiType {
        COND,
        UNCOND,
        RETURN,
        OTHER
    };

    void addCfi(CfiType type, bool miss) {
        switch (type) {
            case COND:
                dbpBtbStats.condNum++;
                if (miss)
                    dbpBtbStats.condMiss++;
                break;
            case UNCOND:
                dbpBtbStats.uncondNum++;
                if (miss)
                    dbpBtbStats.uncondMiss++;
                break;
            case RETURN:
                dbpBtbStats.returnNum++;
                if (miss)
                    dbpBtbStats.returnMiss++;
                break;
            case OTHER:
                dbpBtbStats.otherNum++;
                if (miss)
                    dbpBtbStats.otherMiss++;
                break;
        }
        DPRINTF(DBPBTBStats, "Miss type: %d\n", type);
    }

    void addFtqNotValid() {
        dbpBtbStats.ftqNotValid++;
    }

    void commitBranch(const DynInstPtr &inst, bool miss);

    void notifyInstCommit(const DynInstPtr &inst);

    std::map<Addr, unsigned> topMispredIndirect;
    int currentFtqEntryInstNum;

};

}  // namespace btb_pred
}  // namespace branch_prediction
}  // namespace gem5

#endif  // __CPU_PRED_BTB_DECOUPLED_BPRED_HH__