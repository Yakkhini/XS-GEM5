#ifndef __CPU_PRED_BTB_HISTORY_MANAGER_HH__
#define __CPU_PRED_BTB_HISTORY_MANAGER_HH__

#include <list>

#include "cpu/pred/btb/stream_struct.hh"

#ifdef UNIT_TEST
#include "cpu/pred/btb/test/test_dprintf.hh"

#else
#include "debug/DecoupleBPVerbose.hh"

#endif

namespace gem5
{

namespace branch_prediction
{

namespace btb_pred
{

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
                        "Skip stream %lu when squashing stream %lu\n",
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
        DPRINTF(DecoupleBPVerbose, "%s stream: %lu, pc %#lx, shamt %ld, cond_taken %d, \
            is_call %d, is_ret %d, retAddr %#lx\n",
            when, entry.streamId, entry.pc, entry.shamt, entry.cond_taken,
            entry.is_call, entry.is_return, entry.retAddr);
    }
};

} // namespace btb_pred

} // namespace branch_prediction

} // namespace gem5

#endif // __CPU_PRED_BTB_HISTORY_MANAGER_HH__
