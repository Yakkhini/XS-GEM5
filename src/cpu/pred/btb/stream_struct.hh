#ifndef __CPU_PRED_BTB_STREAM_STRUCT_HH__
#define __CPU_PRED_BTB_STREAM_STRUCT_HH__

#include <boost/dynamic_bitset.hpp>

#include "arch/generic/pcstate.hh"
#include "base/types.hh"
#include "cpu/inst_seq.hh"
#include "cpu/pred/general_arch_db.hh"
#include "cpu/pred/btb/stream_common.hh"
#include "cpu/static_inst.hh"
#include "debug/DecoupleBP.hh"
#include "debug/BTB.hh"
#include "debug/Override.hh"
// #include "cpu/pred/btb/btb.hh"

namespace gem5 {

namespace branch_prediction {

namespace btb_pred {

enum EndType {
    END_CALL=0,
    END_RET,
    END_OTHER_TAKEN,
    END_NOT_TAKEN,
    END_CONT,  // to be continued
    END_NONE
};

enum SquashType {
    SQUASH_NONE=0,
    SQUASH_TRAP,
    SQUASH_CTRL,
    SQUASH_OTHER
};


/**
 * @brief Branch information structure containing branch properties and targets
 * 
 * Stores essential information about a branch instruction including:
 * - PC and target address
 * - Branch type (conditional, indirect, call, return)
 * - Instruction size
 */
typedef struct BranchInfo {
    Addr pc;
    Addr target;
    bool isCond;
    bool isIndirect;
    bool isCall;
    bool isReturn;
    uint8_t size;
    bool isUncond() const { return !this->isCond; }
    Addr getEnd() { return this->pc + this->size; }
    BranchInfo() : pc(0), target(0), isCond(false), isIndirect(false), isCall(false), isReturn(false), size(0) {}
    BranchInfo (const Addr &control_pc,
                const Addr &target_pc,
                const StaticInstPtr &static_inst,
                unsigned size) :
        pc(control_pc),
        target(target_pc),
        isCond(static_inst->isCondCtrl()),
        isIndirect(static_inst->isIndirectCtrl()),
        isCall(static_inst->isCall()),
        isReturn(static_inst->isReturn() && !static_inst->isNonSpeculative() && !static_inst->isDirectCtrl()),
        size(size) {}
    int getType() {
        if (isCond) {
            return 0;
        } else if (!isIndirect) {
            if (isReturn) {
                fatal("jal return detected!\n");
                return 7;
            }
            if (!isCall) {
                return 1;
            } else {
                return 2;
            }
        } else {
            if (!isCall) {
                if (!isReturn) {
                    return 3; // normal indirect
                } else {
                    return 4; // indirect return
                }
            } else {
                if (!isReturn) { // indirect call
                    return 5;
                } else { // call & return
                    return 6;
                }
            }
        }
    }
    // BranchInfo(BTBSlot _e) : pc(_e.pc), target(_e.target), isCond(_e.isCond), isIndirect(_e.isIndirect), isCall(_e.isCall), isReturn(_e.isReturn), size(_e.size) {}

    bool operator < (const BranchInfo &other) const
    {
        return this->pc < other.pc;
    }

    bool operator == (const BranchInfo &other) const
    {
        return this->pc == other.pc;
    }

    bool operator > (const BranchInfo &other) const
    {
        return this->pc > other.pc;
    }

    bool operator != (const BranchInfo &other) const
    {
        return this->pc != other.pc;
    }
}BranchInfo;


/**
 * @brief Branch Target Buffer entry extending BranchInfo with prediction metadata
 * 
 * Contains branch information plus prediction state:
 * - Valid bit
 * - Always taken bit
 * - Counter for prediction
 * - Tag for BTB lookup
 */
typedef struct BTBEntry : BranchInfo
{
    bool valid;
    bool alwaysTaken;
    int ctr;
    Addr tag;
    // Addr offset; // retrived from lowest bits of pc
    BTBEntry() : valid(false) {}
    BTBEntry(const BranchInfo &bi) : BranchInfo(bi), valid(true), alwaysTaken(true), ctr(0) {}
    BranchInfo getBranchInfo() { return BranchInfo(*this); }

}BTBEntry;

typedef struct LFSR64 {
    uint64_t lfsr;
    LFSR64() : lfsr(0x1234567887654321UL) {}
    uint64_t get() {
        next();
        return lfsr;
    }
    void next() {
        if (lfsr == 0) {
            lfsr = 1;
        } else {
            uint64_t bit = ((lfsr >> 0) ^ (lfsr >> 1) ^ (lfsr >> 3) ^ (lfsr >> 4)) & 1;
            lfsr = (lfsr >> 1) | (bit << 63);
        }
    }
}LFSR64;


struct BlockDecodeInfo {
    std::vector<bool> condMask;
    BranchInfo jumpInfo;
};


using FetchStreamId = uint64_t;
using FetchTargetId = uint64_t;
using PredictionID = uint64_t;

typedef struct LoopEntry {
    bool valid;
    int tripCnt;
    int specCnt;
    int conf;
    bool repair;
    LoopEntry() : valid(false), tripCnt(0), specCnt(0), conf(0), repair(false) {}
} LoopEntry;

typedef struct LoopRedirectInfo {
    LoopEntry e;
    Addr branch_pc;
    bool end_loop;
} LoopRedirectInfo;

typedef struct JAEntry {
    // jump target: indexPC + jumpAheadBlockNum * blockSize
    int jumpAheadBlockNum;
    int conf;
    JAEntry() : jumpAheadBlockNum(0), conf(0) {}
    Addr getJumpTarget(Addr indexPC, int blockSize) {
        return indexPC + jumpAheadBlockNum * blockSize;
    }
} JAEntry;

// NOTE: now this corresponds to an ftq entry in
//       XiangShan nanhu architecture
/**
 * @brief Fetch Stream representing a sequence of instructions with prediction info
 * 
 * Key structure for decoupled frontend that contains:
 * - Stream boundaries (start PC, end PC)
 * - Prediction information (branch info, targets)
 * - Execution results for verification
 * - Loop and jump-ahead prediction state
 * - Statistics for profiling
 */
typedef struct FetchStream
{
    Addr startPC;

    // indicating whether a backing prediction has finished
    // bool predEnded;
    bool predTaken;

    // predicted stream end pc (fall through pc)
    Addr predEndPC;
    BranchInfo predBranchInfo;
    // record predicted BTB entry
    bool isHit;
    bool falseHit;
    std::vector<BTBEntry> predBTBEntries;

    bool sentToICache;

    // for commit, write at redirect or fetch
    // bool exeEnded;
    bool exeTaken;
    // Addr exeEndPC;
    BranchInfo exeBranchInfo;

    BTBEntry updateNewBTBEntry; // the possible new entry
    bool updateIsOldEntry;
    bool resolved;
    // used to decide which branches to update (don't update if not actually executed)
    // set before components update
    Addr updateEndInstPC; 
    // for components to decide which entries to update
    // set before components update
    std::vector<BTBEntry> updateBTBEntries;

    int squashType;
    Addr squashPC;
    unsigned predSource;

    // for loop buffer
    bool fromLoopBuffer;
    bool isDouble;
    bool isExit;

    // for ja predictor
    bool jaHit;
    JAEntry jaEntry;
    int currentSentBlock;

    // prediction metas
    // FIXME: use vec
    std::array<std::shared_ptr<void>, 6> predMetas;

    // for loop
    std::vector<LoopRedirectInfo> loopRedirectInfos;
    std::vector<bool> fixNotExits;
    std::vector<LoopRedirectInfo> unseenLoopRedirectInfos;

    Tick predTick;
    boost::dynamic_bitset<> history;

    // for profiling
    int fetchInstNum;
    int commitInstNum;

    FetchStream()
        : startPC(0),
          predTaken(false),
          predEndPC(0),
          predBranchInfo(BranchInfo()),
          isHit(false),
          falseHit(false),
        //   predBTBEntry(BTBEntry()),
          sentToICache(false),
          exeTaken(false),
          exeBranchInfo(BranchInfo()),
          updateNewBTBEntry(BTBEntry()),
          updateIsOldEntry(false),
          resolved(false),
          squashType(SquashType::SQUASH_NONE),
          predSource(0),
          fromLoopBuffer(false),
          isDouble(false),
          isExit(false),
          jaHit(false),
          jaEntry(JAEntry()),
          currentSentBlock(0),
          fetchInstNum(0),
          commitInstNum(0)
    {
    }

    // the default exe result should be consistent with prediction
    void setDefaultResolve() {
        resolved = false;
        exeBranchInfo = predBranchInfo;
        exeTaken = predTaken;
    }

    // bool getEnded() const { return resolved ? exeEnded : predEnded; }
    BranchInfo getBranchInfo() const { return resolved ? exeBranchInfo : predBranchInfo; }
    Addr getControlPC() const { return getBranchInfo().pc; }
    Addr getEndPC() const { return getBranchInfo().getEnd(); } // FIXME: should be end of squash inst when non-control squash of trap squash
    Addr getTaken() const { return resolved ? exeTaken : predTaken; }
    Addr getTakenTarget() const { return getBranchInfo().target; }
    // Addr getFallThruPC() const { return getEndPC(); }
    // Addr getNextStreamStart() const {return getTaken() ? getTakenTarget() : getFallThruPC(); }
    // bool isCall() const { return endType == END_CALL; }
    // bool isReturn() const { return endType == END_RET; }

    // for ja hit blocks, should be the biggest addr of startPC + k*blockSize where k is interger
    Addr getRealStartPC() const {
        if (jaHit && squashType == SQUASH_CTRL) {
            Addr realStart = startPC;
            Addr squashBranchPC = exeBranchInfo.pc;
            while (realStart + 0x20 <= squashBranchPC) {
                realStart += 0x20;
            }
            return realStart;
        } else {
            return startPC;
        }
    }

    std::pair<int, bool> getHistInfoDuringSquash(Addr squash_pc, bool is_cond, bool actually_taken, unsigned maxShamt)
    {
        int shamt = 0;
        int cond_taken = false;
        for (auto &entry : predBTBEntries) {
            if (entry.valid && entry.pc >= startPC && entry.pc < squash_pc) {
                shamt++;
            }
        }
        if (is_cond) {
            shamt++;
            cond_taken = actually_taken;
        }
        return std::make_pair(shamt, cond_taken);
    }
    
    // should be called before components update
    void setUpdateInstEndPC(unsigned blockSize)
    {
        if (squashType == SQUASH_NONE) {
            if (exeTaken) { // taken inst pc
                updateEndInstPC = getControlPC();
            } else { // natural fall through, align to the next block
                Addr alignBlockMask = ~((Addr)blockSize - 1);
                updateEndInstPC = (startPC & alignBlockMask) + blockSize;
            }
        } else {
            updateEndInstPC = squashPC;
        }
    }

    // should be called before components update, after setUpdateInstEndPC
    void setUpdateBTBEntries()
    {
        updateBTBEntries.clear();
        for (auto &entry : predBTBEntries) {
            if (entry.valid && entry.pc >= startPC && entry.pc <= updateEndInstPC) {
                updateBTBEntries.push_back(entry);
            }
        }
    }

}FetchStream;

/**
 * @brief Full branch prediction combining predictions from all predictors
 * 
 * Aggregates predictions from:
 * - BTB entries for targets
 * - Direction predictors for conditional branches
 * - Indirect predictors for indirect branches
 * - RAS for return instructions
 */
typedef struct FullBTBPrediction
{
    Addr bbStart;
    std::vector<BTBEntry> btbEntries; // for BTB, only assigned when hit, sorted by inst order
    // for conditional branch predictors, mapped with lowest bits of branches
    std::map<Addr, bool> condTakens;

    // for indirect predictor, mapped with lowest bits of branches
    std::map<Addr, Addr> indirectTargets;
    Addr returnTarget; // for RAS

    // std::vector<bool> valids; // hit
    unsigned predSource;
    Tick predTick;
    boost::dynamic_bitset<> history;



    BTBEntry getTakenEntry() {
        // IMPORTANT: assume entries are sorted
        for (auto &entry : this->btbEntries) {
            // hit
            if (entry.valid) {
                if (entry.isCond) {
                    // find corresponding direction pred in condTakens
                    // TODO: use lower-bit offset of branch instruction
                    auto it = condTakens.find(entry.pc);
                    if (it != condTakens.end()) {
                        if (it->second) {
                            return entry;
                        }
                    }
                }
                if (entry.isUncond()) {
                    return entry;
                }
            }
        }
        return BTBEntry();
    }

    bool isTaken() {
        return getTakenEntry().valid;
    }

    Addr getFallThrough() {
        return (bbStart + 32) & ~mask(floorLog2(32));
    }

    Addr getTarget() {
        Addr target;
        const auto &entry = getTakenEntry();
        DPRINTF(DecoupleBP, "getTarget: pc %lx, target %#lx, cond %d, indirect %d, return %d\n",
            entry.pc, entry.target, entry.isCond, entry.isIndirect, entry.isReturn);
        if (entry.valid) {
            target = entry.target;
            if (entry.isIndirect) {
                if (!entry.isReturn) {
                    // indirect target should come from ipred or ras,
                    // or btb itself when ipred miss
                    auto it = indirectTargets.find(entry.pc);
                    if (it != indirectTargets.end()) {
                        target = it->second;
                    }
                } else {
                    target = returnTarget;
                }
            }
        } else {
            target = getFallThrough(); //TODO: +predictWidth
        }
        return target;
    }

    Addr getEnd() {
        if (isTaken()) {
            return getTakenEntry().getEnd();
        } else {
            return getFallThrough();
        }
    }


    Addr controlAddr() {
        return getTakenEntry().pc;
    }

    // int getTakenBranchIdx() {
    //     auto &btbEntry = this->btbEntry;
    //     if (valid) {
    //         int i = 0;
    //         for (auto &slot : btbEntry.slots) {
    //             if ((slot.condValid() && condTakens[i]) ||
    //                 slot.uncondValid()) {
    //                     return i;
    //                 }
    //             i++;
    //         }
    //     }
    //     return -1;
    // }

    bool match(FullBTBPrediction &other)
    {
        auto this_taken_entry = this->getTakenEntry();
        auto other_taken_entry = other.getTakenEntry();
        if (this_taken_entry.valid != other_taken_entry.valid) {
            return false;
        } else {
            // all taken or all not taken, check target and end
            if (this_taken_entry.valid && other_taken_entry.valid) {
                return this->controlAddr() == other.controlAddr() &&
                       this->getTarget() == other.getTarget() &&
                    //    this->getEnd() == other.getEnd();
                       this->getEnd() == other.getEnd() &&
                       this->getHistInfo() == other.getHistInfo();
            } else {
                return true;
            }
        }
    }

    std::vector<boost::dynamic_bitset<>> indexFoldedHist;
    std::vector<boost::dynamic_bitset<>> tagFoldedHist;

    std::pair<int, bool> getHistInfo()
    {
        int shamt = 0;
        bool taken = false;
        for (auto &entry : btbEntries) {
            if (entry.valid) {
                if (entry.isCond) {
                    shamt++;
                    auto it = condTakens.find(entry.pc);
                    if (it != condTakens.end()) {
                        if (it->second) {
                            taken = true;
                            break;
                        }
                    }
                } else {
                    // uncond
                    break;
                }
            }
        }
        // if (valid) {
        //     int i = 0;
        //     for (auto &slot : btbEntry.slots) {
        //         DPRINTF(Override, "slot %d: condValid %d, uncondValid %d\n",
        //             i, slot.condValid(), slot.uncondValid());
        //         DPRINTF(Override, "condTakens.size() %d\n", condTakens.size());
        //         if (slot.condValid()) {
        //             shamt++;
        //             if (condTakens[i]) {
        //                 taken = true;
        //                 break;
        //             }
        //         }
        //         assert(condTakens.size() >= i+1);
                
        //         i++;
        //     }
        // }
        return std::make_pair(shamt, taken);
    }

    // bool isReasonable() {
    //     return !valid || btbEntry.isReasonable(bbStart);
    // }


}FullBTBPrediction;

/**
 * @brief Fetch Target Queue entry representing a fetch block
 * 
 * Contains information needed for instruction fetch:
 * - Address range (start PC, end PC)
 * - Branch information (taken PC, target)
 * - Loop information for loop buffer
 * - Stream tracking (FSQ ID)
 */
struct FtqEntry
{
    Addr startPC;
    Addr endPC;    // TODO: use PCState and it can be included in takenPC

    // When it is a taken branch, takenPC is the control (starting) PC
    // When it is yet missing, takenPC is the ``known'' PC,
    // decoupledPredict cannot goes beyond takenPC and should be blocked
    // when current PC == takenPC
    Addr takenPC;

    bool taken;
    Addr target;  // TODO: use PCState
    FetchStreamId fsqID;

    // for loop buffer
    bool inLoop;
    int iter;
    bool isExit;
    Addr loopEndPC;

    // for ja predictor
    int noPredBlocks;

    FtqEntry()
        : startPC(0)
        , endPC(0)
        , takenPC(0)
        , taken(false)
        , target(0)
        , fsqID(0)
        , inLoop(false)
        , iter(0)
        , isExit(false)
        , loopEndPC(0)
        , noPredBlocks(0) {}
    
    bool miss() const { return !taken; }
    // bool filledUp() const { return (endPC & fetchTargetMask) == 0; }
    // unsigned predLoopIteration;
};


struct TageMissTrace : public Record {
    void set(uint64_t startPC, uint64_t branchPC, uint64_t lgcBank, uint64_t phyBank, uint64_t mainFound, uint64_t mainCounter, uint64_t mainUseful,
        uint64_t altCounter, uint64_t mainTable, uint64_t mainIndex, uint64_t altIndex, uint64_t tag,
        uint64_t useAlt, uint64_t predTaken, uint64_t actualTaken, uint64_t allocSuccess, uint64_t allocFailure,
        uint64_t predUseSC, uint64_t predSCDisagree, uint64_t predSCCorrect)
    {
        _tick = curTick();
        _uint64_data["startPC"] = startPC;
        _uint64_data["branchPC"] = branchPC;
        _uint64_data["lgcBank"] = lgcBank;
        _uint64_data["phyBank"] = phyBank;
        _uint64_data["mainFound"] = mainFound;
        _uint64_data["mainCounter"] = mainCounter;
        _uint64_data["mainUseful"] = mainUseful;
        _uint64_data["altCounter"] = altCounter;
        _uint64_data["mainTable"] = mainTable;
        _uint64_data["mainIndex"] = mainIndex;
        _uint64_data["altIndex"] = altIndex;
        _uint64_data["tag"] = tag;
        _uint64_data["useAlt"] = useAlt;
        _uint64_data["predTaken"] = predTaken;
        _uint64_data["actualTaken"] = actualTaken;
        _uint64_data["allocSuccess"] = allocSuccess;
        _uint64_data["allocFailure"] = allocFailure;
        _uint64_data["predUseSC"] = predUseSC;
        _uint64_data["predSCDisagree"] = predSCDisagree;
        _uint64_data["predSCCorrect"] = predSCCorrect;
    }
};

struct LoopTrace : public Record {
    void set(uint64_t pc, uint64_t target, uint64_t mispred, uint64_t training,
        uint64_t trainSpecCnt, uint64_t trainTripCnt, uint64_t trainConf,
        uint64_t inMain, uint64_t mainTripCnt, uint64_t mainConf, uint64_t predSpecCnt,
        uint64_t predTripCnt, uint64_t predConf)
    {
        _tick = curTick();
        _uint64_data["pc"] = pc;
        _uint64_data["target"] = target;
        _uint64_data["mispred"] = mispred;
        _uint64_data["predSpecCnt"] = predSpecCnt;
        _uint64_data["predTripCnt"] = predTripCnt;
        _uint64_data["predConf"] = predConf;
        // from lp
        _uint64_data["training"] = training;
        _uint64_data["trainSpecCnt"] = trainSpecCnt;
        _uint64_data["trainTripCnt"] = trainTripCnt;
        _uint64_data["trainConf"] = trainConf;
        _uint64_data["inMain"] = inMain;
        _uint64_data["mainTripCnt"] = mainTripCnt;
        _uint64_data["mainConf"] = mainConf;
    }
    void set_in_lp(uint64_t training, uint64_t trainSpecCnt, uint64_t trainTripCnt, uint64_t trainConf,
        uint64_t inMain, uint64_t mainTripCnt, uint64_t mainConf)
    {
        _uint64_data["training"] = training;
        _uint64_data["trainSpecCnt"] = trainSpecCnt;
        _uint64_data["trainTripCnt"] = trainTripCnt;
        _uint64_data["trainConf"] = trainConf;
        _uint64_data["inMain"] = inMain;
        _uint64_data["mainTripCnt"] = mainTripCnt;
        _uint64_data["mainConf"] = mainConf;
    }
    void set_outside_lp(uint64_t pc, uint64_t target, uint64_t mispred,
        uint64_t predSpecCnt, uint64_t predTripCnt, uint64_t predConf)
    {
        _tick = curTick();
        _uint64_data["pc"] = pc;
        _uint64_data["target"] = target;
        _uint64_data["mispred"] = mispred;
        _uint64_data["predSpecCnt"] = predSpecCnt;
        _uint64_data["predTripCnt"] = predTripCnt;
        _uint64_data["predConf"] = predConf;
    }
};

struct BTBTrace : public Record {
    // mode: read, write, evict
    void set(uint64_t pc, uint64_t brType, uint64_t target, uint64_t idx, uint64_t mode, uint64_t hit) {
        _tick = curTick();
        _uint64_data["pc"] = pc;
        _uint64_data["brType"] = brType;
        _uint64_data["target"] = target;
        _uint64_data["idx"] = idx;
        _uint64_data["mode"] = mode;
        _uint64_data["hit"] = hit;
    }
};

}  // namespace btb_pred

}  // namespace branch_prediction

}  // namespace gem5
#endif  // __CPU_PRED_BTB_STREAM_STRUCT_HH__
