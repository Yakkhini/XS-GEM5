#ifndef __CPU_PRED_BTB_STREAM_COMMON_HH__
#define __CPU_PRED_BTB_STREAM_COMMON_HH__

#include "base/types.hh"
#include "cpu/inst_seq.hh"

namespace gem5 {

namespace branch_prediction {

namespace btb_pred {

enum class OverrideReason
{
    no_override,
    validity,
    controlAddr,
    target,
    end,
    histInfo
};

extern unsigned streamChunkSize;

extern unsigned fetchTargetSize;
extern unsigned fetchTargetMask;

extern unsigned predictWidth;
extern bool alignToBlockSize;
extern bool halfAligned;
Addr computeLastChunkStart(Addr taken_control_pc, Addr stream_start_pc);

}  // namespace btb_pred

}  // namespace branch_prediction

}  // namespace gem5
#endif  // __CPU_PRED_BTB_STREAM_COMMON_HH__
