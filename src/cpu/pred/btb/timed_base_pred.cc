#include "cpu/pred/btb/timed_base_pred.hh"

namespace gem5
{

namespace branch_prediction
{

namespace btb_pred
{

TimedBaseBTBPredictor::TimedBaseBTBPredictor(const Params &p)
    : SimObject(p),
      blockSize(p.blockSize),
      numDelay(p.numDelay)
{
}

} // namespace btb_pred

} // namespace branch_prediction

} // namespace gem5