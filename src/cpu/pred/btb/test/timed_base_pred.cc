#include "cpu/pred/btb/test/timed_base_pred.hh"

namespace gem5
{

namespace branch_prediction
{

namespace btb_pred
{

namespace test
{

TimedBaseBTBPredictor::TimedBaseBTBPredictor()
    : blockSize(32),
      numDelay(0)
{
}

} // namespace test

} // namespace btb_pred

} // namespace branch_prediction

} // namespace gem5