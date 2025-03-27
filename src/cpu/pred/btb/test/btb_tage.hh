#ifndef __CPU_PRED_BTB_TAGE_HH__
#define __CPU_PRED_BTB_TAGE_HH__

#include <deque>
#include <map>
#include <vector>
#include <utility>

#include "base/sat_counter.hh"
#include "base/types.hh"
#include "cpu/inst_seq.hh"
#include "cpu/pred/btb/folded_hist.hh"
#include "cpu/pred/btb/stream_struct.hh"
// #include "cpu/pred/btb/timed_base_pred.hh"
// #include "debug/DecoupleBP.hh"
// #include "debug/TAGEUseful.hh"
// #include "params/BTBTAGE.hh"
// #include "sim/sim_object.hh"

#include "cpu/pred/btb/test/test_dprintf.hh"

namespace gem5
{

namespace branch_prediction
{

namespace btb_pred
{

namespace test
{

class BTBTAGE
{
    using defer = std::shared_ptr<void>;
    using bitset = boost::dynamic_bitset<>;
  public:
    // typedef BTBTAGEParams Params;

    // Represents a single entry in the TAGE prediction table
    struct TageEntry
    {
        public:
            bool valid;      // Whether this entry is valid
            Addr tag;       // Tag for matching
            short counter;  // Prediction counter (-4 to 3), 3bits， 0 and -1 are weak
            bool useful;    // Whether this entry is useful for prediction, set true if alt differs from main and main is correct
            Addr pc;        // branch pc, like branch position, for btb entry pc check

            TageEntry() : valid(false), tag(0), counter(0), useful(false), pc(0) {}

            TageEntry(Addr tag, short counter, Addr pc) :
                      valid(true), tag(tag), counter(counter), useful(false), pc(pc) {}
            bool taken() const {
                return counter >= 0;
            }
    };

    // Contains information about a TAGE table lookup
    struct TageTableInfo
    {
        public:
            bool found;     // Whether a matching entry was found
            TageEntry entry; // The matching entry
            unsigned table; // Which table this entry was found in
            Addr index;     // Index in the table
            Addr tag;       // Tag that was matched
            TageTableInfo() : found(false), table(0), index(0), tag(0) {}
            TageTableInfo(bool found, TageEntry entry, unsigned table, Addr index, Addr tag) :
                        found(found), entry(entry), table(table), index(index), tag(tag) {}
            bool taken() const {
                return entry.taken();
            }
    };

    // Contains the complete prediction result
    struct TagePrediction
    {
        public:
            Addr btb_pc;           // btb entry pc, same as tage entry pc
            TageTableInfo mainInfo; // Main prediction info
            TageTableInfo altInfo;  // Alternative prediction info
            bool useAlt;           // Whether to use alternative prediction, true if main is weak or no main prediction
            bool taken;            // Final prediction (taken/not taken) = use_alt ? alt_provided ? alt_taken : base_taken : main_taken

            TagePrediction() : btb_pc(0), useAlt(false), taken(false) {}

            TagePrediction(Addr btb_pc, TageTableInfo mainInfo, TageTableInfo altInfo,
                            bool useAlt, bool taken) :
                            btb_pc(btb_pc), mainInfo(mainInfo), altInfo(altInfo),
                            useAlt(useAlt), taken(taken) {}
    };

    // Structure to hold TAGE table lookup results
    struct TableLookupResult {
        std::vector<TageEntry> entries;  // Entries found in each table
        std::vector<Addr> indices;       // Indices used for lookup
        std::vector<Addr> tags;          // Tags used for lookup
        bitset useful_mask;              // Mask of useful bits from entries

        TableLookupResult() {}
    };

    // Structure to hold allocation results
    struct AllocationResult {
        bool allocate_valid;             // Whether allocation is valid
        bitset allocate_mask;            // Mask for allocation
        
        AllocationResult() : allocate_valid(false) {}
    };

  public:
    BTBTAGE();
    ~BTBTAGE();

    void tickStart() ;

    void tick() ;
    // Make predictions for a stream of instructions and record in stage preds
    void putPCHistory(Addr startAddr,
                      const boost::dynamic_bitset<> &history,
                      std::vector<FullBTBPrediction> &stagePreds) ;

    std::shared_ptr<void> getPredictionMeta() ;

    // speculative update 3 folded history, according history and pred.taken
    void specUpdateHist(const boost::dynamic_bitset<> &history, FullBTBPrediction &pred) ;

    // Recover 3 folded history after a misprediction, then update 3 folded history according to history and pred.taken
    void recoverHist(const boost::dynamic_bitset<> &history, const FetchStream &entry, int shamt, bool cond_taken) ;

    // Update predictor state based on actual branch outcomes
    void update(const FetchStream &entry) ;

    // void commitBranch(const FetchStream &stream, const DynInstPtr &inst) ;

    void setTrace() ;

    // check folded hists after speculative update and recover
    void checkFoldedHist(const bitset &history, const char *when);

// for test
  public:

    // Look up predictions in TAGE tables for a stream of instructions
    std::map<Addr, bool> lookupHelper(const Addr &stream_start, const std::vector<BTBEntry> &btbEntries);

    // Calculate TAGE index for a given PC and table
    Addr getTageIndex(Addr pc, int table);

    // Calculate TAGE index with folded history
    Addr getTageIndex(Addr pc, int table, bitset &foldedHist);

    // Calculate TAGE tag for a given PC and table
    Addr getTageTag(Addr pc, int table);

    // Calculate TAGE tag with folded history
    Addr getTageTag(Addr pc, int table, bitset &foldedHist, bitset &altFoldedHist);

    // Get offset within a block for a given PC
    Addr getOffset(Addr pc) {
        return (pc & (blockSize - 1)) >> 1;
    }

    // Update branch history
    void doUpdateHist(const bitset &history, int shamt, bool taken);

    // Number of TAGE predictor tables
    const unsigned numPredictors;

    // Size of each prediction table
    std::vector<unsigned> tableSizes;

    // Number of bits used for indexing each table
    std::vector<unsigned> tableIndexBits;

    // Masks for table indexing
    std::vector<bitset> tableIndexMasks;

    // Number of bits used for tags in each table
    std::vector<unsigned> tableTagBits;

    // Masks for tag matching
    std::vector<bitset> tableTagMasks;

    // PC shift amounts for each table
    std::vector<unsigned> tablePcShifts;

    // History lengths for each table
    std::vector<unsigned> histLengths;

    // Folded history for tag calculation
    std::vector<FoldedHist> tagFoldedHist;

    // Folded history for alternative tag calculation
    std::vector<FoldedHist> altTagFoldedHist;

    // Folded history for index calculation
    std::vector<FoldedHist> indexFoldedHist;

    // Linear feedback shift register for allocation
    LFSR64 allocLFSR;

    // Maximum history length
    unsigned maxHistLen;

    // The actual TAGE prediction tables
    std::vector<std::vector<TageEntry>> tageTable;

    // Table for tracking when to use alternative prediction
    std::vector<std::vector<short>> useAlt;

    // Check if a tag matches
    bool matchTag(Addr expected, Addr found);

    // Set tag bits for a given table
    void setTag(Addr &dest, Addr src, int table);

    // Debug flag
    bool debugFlagOn{true};

    // Number of tables to allocate on misprediction
    unsigned numTablesToAlloc;

    // Instruction shift amount
    unsigned instShiftAmt {1};

    // Update prediction counter with saturation
    void updateCounter(bool taken, unsigned width, short &counter);

    // Increment counter with saturation
    bool satIncrement(int max, short &counter);

    // Decrement counter with saturation
    bool satDecrement(int min, short &counter);

    // Get index for useAlt table
    Addr getUseAltIdx(Addr pc);

    // Counter for useful bit reset algorithm
    int usefulResetCnt;

    // Cache for TAGE indices
    std::vector<Addr> tageIndex;

    // Cache for TAGE tags
    std::vector<Addr> tageTag;

    // Whether statistical corrector is enabled
    bool enableSC;

    // Statistics for TAGE predictor
    struct TageStats {
        uint64_t predTableHits;
        uint64_t predNoHitUseBim;
        uint64_t predUseAlt;
        uint64_t updateTableHits;
        uint64_t updateNoHitUseBim;
        uint64_t updateUseAlt;
    
        uint64_t updateUseAltCorrect;
        uint64_t updateUseAltWrong;
        uint64_t updateAltDiffers;
        uint64_t updateUseAltOnNaUpdated;
        uint64_t updateUseAltOnNaInc;
        uint64_t updateUseAltOnNaDec;
        uint64_t updateProviderNa;
        uint64_t updateUseNaCorrect;
        uint64_t updateUseNaWrong;
        uint64_t updateUseAltOnNa;
        uint64_t updateUseAltOnNaCorrect;
        uint64_t updateUseAltOnNaWrong;
        uint64_t updateAllocFailure;
        uint64_t updateAllocSuccess;
        uint64_t updateMispred;
        uint64_t updateResetU;
        uint64_t updateResetUCtrInc;
        uint64_t updateResetUCtrDec;

        std::vector<uint64_t> updateTableMispreds;

        int bankIdx;
        int numPredictors;

        TageStats(int numPredictors);
        void updateStatsWithTagePrediction(const TagePrediction &pred, bool when_pred);
    } ;
    
    TageStats tageStats;

    // TraceManager *tageMissTrace;

    // below are for test
    int getComponentIdx() {
        return 0;
    }

    int getDelay() {
        return 1;
    }

public:

    // Recover folded history after misprediction
    void recoverFoldedHist(const bitset& history);

public:

    // Metadata for TAGE predictions
    typedef struct TageMeta {
        std::map<Addr, TagePrediction> preds;
        bitset usefulMask;
        std::vector<FoldedHist> tagFoldedHist;
        std::vector<FoldedHist> altTagFoldedHist;
        std::vector<FoldedHist> indexFoldedHist;
        TageMeta(std::map<Addr, TagePrediction> preds, bitset usefulMask, std::vector<FoldedHist> tagFoldedHist,
            std::vector<FoldedHist> altTagFoldedHist, std::vector<FoldedHist> indexFoldedHist) :
            preds(preds), usefulMask(usefulMask), tagFoldedHist(tagFoldedHist),
            altTagFoldedHist(altTagFoldedHist), indexFoldedHist(indexFoldedHist) {}
        TageMeta() {}
        TageMeta(const TageMeta &other) {
            preds = other.preds;
            usefulMask = other.usefulMask;
            tagFoldedHist = other.tagFoldedHist;
            altTagFoldedHist = other.altTagFoldedHist;
            indexFoldedHist = other.indexFoldedHist;
            // scMeta = other.scMeta;
        }
    } TageMeta;

private:
    // Helper method to lookup entries in all TAGE tables
    TableLookupResult lookupTageTable(const Addr &startPC);

    // Helper method to generate prediction for a single BTB entry
    TagePrediction generateSinglePrediction(const BTBEntry &btb_entry, 
                                          const TableLookupResult &table_result);

    // Helper method to prepare BTB entries for update
    std::vector<BTBEntry> prepareUpdateEntries(const FetchStream &stream);

    // Helper method to update predictor state for a single entry
    bool updatePredictorStateAndCheckAllocation(const BTBEntry &entry, 
                                 bool actual_taken,
                                 const TagePrediction &pred,
                                 const FetchStream &stream);

    // Helper method to handle useful bit reset
    void handleUsefulBitReset(const bitset &useful_mask);

    // Helper method to handle new entry allocation
    void handleNewEntryAllocation(const Addr &startPC,
                                 const BTBEntry &entry,
                                 bool actual_taken,
                                 const bitset &useful_mask,
                                 unsigned main_table,
                                 std::shared_ptr<TageMeta> meta);

    // Helper method to generate allocation mask
    AllocationResult generateAllocationMask(const bitset &useful_mask,
                                          unsigned start_table);

    TageMeta meta;
}; // end of class BTBTAGE

}   // end of namespace test

}   // end of namespace btb_pred

}   // end of namespace branch_prediction

}   // end of namespace gem5

#endif  // __CPU_PRED_BTB_TAGE_HH__
