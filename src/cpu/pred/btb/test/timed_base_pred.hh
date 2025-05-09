#ifndef __CPU_PRED_BTB_TIMED_BASE_PRED_HH__
#define __CPU_PRED_BTB_TIMED_BASE_PRED_HH__


#include <boost/dynamic_bitset.hpp>

#include "cpu/pred/btb/stream_struct.hh"

namespace gem5
{

namespace branch_prediction
{

namespace btb_pred
{

namespace test
{

// using DynInstPtr = o3::DynInstPtr;

class TimedBaseBTBPredictor
{
    public:

    // typedef TimedBaseBTBPredictorParams Params;

    TimedBaseBTBPredictor();

    // virtual void tickStart() {}
    // virtual void tick() {}
    // make predictions, record in stage preds
    virtual void putPCHistory(Addr startAddr,
                              const boost::dynamic_bitset<> &history,
                              std::vector<FullBTBPrediction> &stagePreds) {}

    virtual std::shared_ptr<void> getPredictionMeta() { return nullptr; }

    virtual void specUpdateHist(const boost::dynamic_bitset<> &history, FullBTBPrediction &pred) {}
    virtual void recoverHist(const boost::dynamic_bitset<> &history, const FetchStream &entry, int shamt, bool cond_taken) {}
    virtual void update(const FetchStream &entry) {}
    virtual unsigned getDelay() {return numDelay;}
    // do some statistics on a per-branch and per-predictor basis
    // virtual void commitBranch(const FetchStream &entry, const DynInstPtr &inst) {}

    int componentIdx = 0;
    unsigned aheadPipelinedStages{0};
    int getComponentIdx() { return componentIdx; }
    void setComponentIdx(int idx) { componentIdx = idx; }

    void setNumDelay(unsigned num) { numDelay = num; }
    void setBlockSize(unsigned size) { blockSize = size; }

    unsigned blockSize;
    unsigned predictWidth;


private:
    unsigned numDelay;
};

} // namespace test

} // namespace btb_pred

} // namespace branch_prediction

} // namespace gem5

#endif // __CPU_PRED_BTB_TIMED_BASE_PRED_HH__
