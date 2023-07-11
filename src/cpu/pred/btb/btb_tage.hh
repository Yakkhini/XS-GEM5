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
#include "cpu/pred/btb/timed_base_pred.hh"
#include "debug/DecoupleBP.hh"
#include "debug/TAGEUseful.hh"
#include "params/BTBTAGE.hh"
#include "sim/sim_object.hh"

namespace gem5
{

namespace branch_prediction
{

namespace btb_pred
{

class BTBTAGE : public TimedBaseBTBPredictor
{
    using defer = std::shared_ptr<void>;
    using bitset = boost::dynamic_bitset<>;
  public:
    typedef BTBTAGEParams Params;

    struct TageEntry
    {
        public:
            bool valid;
            Addr tag;
            short counter;
            bool useful;
            Addr pc; // TODO: should use lowest bits only

            TageEntry() : valid(false), tag(0), counter(0), useful(false), pc(0) {}

            TageEntry(Addr tag, short counter, Addr pc) :
                      valid(true), tag(tag), counter(counter), useful(false), pc(pc) {}
            bool taken() {
                return counter >= 0;
            }

    };

    struct TageTableInfo
    {
        public:
            bool found;
            TageEntry entry;
            unsigned table;
            Addr index;
            Addr tag;
            TageTableInfo() : found(false), table(0), index(0), tag(0) {}
            TageTableInfo(bool found, TageEntry entry, unsigned table, Addr index, Addr tag) :
                        found(found), entry(entry), table(table), index(index), tag(tag) {}
            bool taken() {
                return entry.taken();
            }
    };

    struct TagePrediction
    {
        public:
            Addr btb_pc;
            TageTableInfo mainInfo;
            TageTableInfo altInfo;

            bool useAlt;
            // bitset usefulMask;
            bool taken;

            TagePrediction() : btb_pc(0), useAlt(false), taken(false) {}

            TagePrediction(Addr btb_pc, TageTableInfo mainInfo, TageTableInfo altInfo,
                            bool useAlt, bool taken) :
                            btb_pc(btb_pc), mainInfo(mainInfo), altInfo(altInfo),
                            useAlt(useAlt), taken(taken) {}


    };

  public:
    BTBTAGE(const Params& p);
    ~BTBTAGE();

    void tickStart() override;

    void tick() override;
    // make predictions, record in stage preds
    void putPCHistory(Addr startAddr,
                      const boost::dynamic_bitset<> &history,
                      std::vector<FullBTBPrediction> &stagePreds) override;

    std::shared_ptr<void> getPredictionMeta() override;

    void specUpdateHist(const boost::dynamic_bitset<> &history, FullBTBPrediction &pred) override;

    void recoverHist(const boost::dynamic_bitset<> &history, const FetchStream &entry, int shamt, bool cond_taken) override;

    void update(const FetchStream &entry) override;

    unsigned getDelay() override { return 1; }

    void commitBranch(const FetchStream &stream, const DynInstPtr &inst) override;

    void setTrace() override;

    // check folded hists after speculative update and recover
    void checkFoldedHist(const bitset &history, const char *when);


  private:

    // return condTakens
    std::map<Addr, bool> lookupHelper(const Addr &stream_start, const std::vector<BTBEntry> &btbEntries);

    // use blockPC
    Addr getTageIndex(Addr pc, int table);

    // use blockPC
    Addr getTageIndex(Addr pc, int table, bitset &foldedHist);

    // use blockPC
    Addr getTageTag(Addr pc, int table);

    // use blockPC
    Addr getTageTag(Addr pc, int table, bitset &foldedHist, bitset &altFoldedHist);

    Addr getOffset(Addr pc) {
        return (pc & (blockSize - 1)) >> 1;
    }

    // unsigned getBaseTableIndex(Addr pc);

    void doUpdateHist(const bitset &history, int shamt, bool taken);

    const unsigned numPredictors;

    std::vector<unsigned> tableSizes;
    std::vector<unsigned> tableIndexBits;
    std::vector<bitset> tableIndexMasks;
    // std::vector<uint64_t> tablePcMasks;
    std::vector<unsigned> tableTagBits;
    std::vector<bitset> tableTagMasks;
    std::vector<unsigned> tablePcShifts;
    std::vector<unsigned> histLengths;
    std::vector<FoldedHist> tagFoldedHist;
    std::vector<FoldedHist> altTagFoldedHist;
    std::vector<FoldedHist> indexFoldedHist;

    LFSR64 allocLFSR;

    unsigned maxHistLen;

    std::vector<std::vector<TageEntry>> tageTable;

    // std::vector<std::vector<short>> baseTable;

    std::vector<std::vector<short>> useAlt;

    bool matchTag(Addr expected, Addr found);

    void setTag(Addr &dest, Addr src, int table);

    bool debugFlagOn{false};

    unsigned numTablesToAlloc;

    unsigned instShiftAmt {1};

    void updateCounter(bool taken, unsigned width, short &counter);

    bool satIncrement(int max, short &counter);

    bool satDecrement(int min, short &counter);

    Addr getUseAltIdx(Addr pc);

    int usefulResetCnt;



    std::vector<Addr> tageIndex;

    std::vector<Addr> tageTag;

    bool enableSC;

    struct TageStats : public statistics::Group {
        statistics::Distribution predTableHits;
        statistics::Scalar predNoHitUseBim;
        statistics::Scalar predUseAlt;
        statistics::Distribution updateTableHits;
        statistics::Scalar updateNoHitUseBim;
        statistics::Scalar updateUseAlt;
    
        statistics::Scalar updateUseAltCorrect;
        statistics::Scalar updateUseAltWrong;
        statistics::Scalar updateAltDiffers;
        statistics::Scalar updateUseAltOnNaUpdated;
        statistics::Scalar updateUseAltOnNaInc;
        statistics::Scalar updateUseAltOnNaDec;
        statistics::Scalar updateProviderNa;
        statistics::Scalar updateUseNaCorrect;
        statistics::Scalar updateUseNaWrong;
        statistics::Scalar updateUseAltOnNa;
        statistics::Scalar updateUseAltOnNaCorrect;
        statistics::Scalar updateUseAltOnNaWrong;
        statistics::Scalar updateAllocFailure;
        statistics::Scalar updateAllocSuccess;
        statistics::Scalar updateMispred;
        statistics::Scalar updateResetU;
        statistics::Distribution updateResetUCtrInc;
        statistics::Distribution updateResetUCtrDec;

        statistics::Vector updateTableMispreds;

        // statistics::Scalar scAgreeAtPred;
        // statistics::Scalar scAgreeAtCommit;
        // statistics::Scalar scDisagreeAtPred;
        // statistics::Scalar scDisagreeAtCommit;
        // statistics::Scalar scConfAtPred;
        // statistics::Scalar scConfAtCommit;
        // statistics::Scalar scUnconfAtPred;
        // statistics::Scalar scUnconfAtCommit;
        // statistics::Scalar scUpdateOnMispred;
        // statistics::Scalar scUpdateOnUnconf;
        // statistics::Scalar scUsedAtPred;
        // statistics::Scalar scUsedAtCommit;
        // statistics::Scalar scCorrectTageWrong;
        // statistics::Scalar scWrongTageCorrect;

        int bankIdx;
        int numPredictors;

        TageStats(statistics::Group* parent, int numPredictors);
        void updateStatsWithTagePrediction(const TagePrediction &pred, bool when_pred);
    } ;
    
    TageStats tageStats;

    TraceManager *tageMissTrace;

public:

    void recoverFoldedHist(const bitset& history);

    // void checkFoldedHist(const bitset& history);
public:
    // class StatisticalCorrector {
    //   public:
    //     // TODO: parameterize
    //     StatisticalCorrector(BTBTAGE *tage) : tage(tage) {
    //       scCntTable.resize(numPredictors);
    //       tableIndexBits.resize(numPredictors);
    //       for (int i = 0; i < numPredictors; i++) {
    //         tableIndexBits[i] = ceilLog2(tableSizes[i]);
    //         foldedHist.push_back(FoldedHist(histLens[i], tableIndexBits[i], numBr));
    //         scCntTable[i].resize(tableSizes[i]);
    //         for (auto &br_counters : scCntTable[i]) {
    //           br_counters.resize(numBr);
    //           for (auto &tOrNt : br_counters) {
    //             tOrNt.resize(2, 0);
    //           }
    //         }
    //       }
    //       // initial theshold
    //       thresholds.resize(numBr, 6);
    //       TCs.resize(numBr, neutralVal);
    //     };

    //     typedef struct SCPrediction {
    //         bool tageTaken;
    //         bool scUsed;
    //         bool scPred;
    //         int scSum;
    //         SCPrediction() : tageTaken(false), scUsed(false), scPred(false), scSum(0) {}
    //     } SCPrediction;

    //     typedef struct SCMeta {
    //         std::vector<FoldedHist> indexFoldedHist;
    //         std::vector<SCPrediction> scPreds;
    //     } SCMeta;

    //   public:
    //     Addr getIndex(Addr pc, int t);

    //     Addr getIndex(Addr pc, int t, bitset &foldedHist);

    //     std::vector<FoldedHist> getFoldedHist();

    //     std::vector<SCPrediction> getPredictions(Addr pc, std::vector<TagePrediction> &tagePreds);

    //     void update(Addr pc, SCMeta meta, std::vector<bool> needToUpdates, std::vector<bool> actualTakens);

    //     void recoverHist(std::vector<FoldedHist> &fh);

    //     void doUpdateHist(const boost::dynamic_bitset<> &history, int shamt, bool cond_taken);

    //     void setStats(std::vector<TageBankStats *> stats) {
    //       this->stats = stats;
    //     }

    //   private:
    //     int numBr;

    //     BTBTAGE *tage;

    //     int numPredictors = 4;

    //     int scCounterWidth = 6;

    //     std::vector<int> thresholds;

    //     int minThres = 5;

    //     int maxThres = 31;

    //     std::vector<int> TCs;

    //     int TCWidth = 6;

    //     int neutralVal = 0;

    //     std::vector<FoldedHist> foldedHist;

    //     std::vector<int> tableIndexBits;

    //     // std::vector<bool> tagVec;

    //     // table - table index - numBr - taken/not taken
    //     std::vector<std::vector<std::vector<std::vector<int>>>> scCntTable;

    //     std::vector<unsigned> histLens {0, 4, 10, 16};

    //     std::vector<int> tableSizes {256, 256, 256, 256};

    //     std::vector<int> tablePcShifts {1, 1, 1, 1};

    //     bool satPos(int &counter, int counterBits);

    //     bool satNeg(int &counter, int counterBits);

    //     void counterUpdate(int &ctr, int nbits, bool taken);

    //     std::vector<TageBankStats*> stats;

    // };

    // StatisticalCorrector sc;

private:
    // using SCMeta = StatisticalCorrector::SCMeta;
    typedef struct TageMeta {
        std::map<Addr, TagePrediction> preds;
        bitset usefulMask;
        std::vector<FoldedHist> tagFoldedHist;
        std::vector<FoldedHist> altTagFoldedHist;
        std::vector<FoldedHist> indexFoldedHist;
        // SCMeta scMeta;
        TageMeta(std::map<Addr, TagePrediction> preds, bitset usefulMask, std::vector<FoldedHist> tagFoldedHist,
            std::vector<FoldedHist> altTagFoldedHist, std::vector<FoldedHist> indexFoldedHist/* , SCMeta scMeta */) :
            preds(preds), usefulMask(usefulMask), tagFoldedHist(tagFoldedHist),
            altTagFoldedHist(altTagFoldedHist), indexFoldedHist(indexFoldedHist)/* ,
            scMeta(scMeta) */ {}
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

    TageMeta meta;
};
}

}

}

#endif  // __CPU_PRED_BTB_TAGE_HH__
