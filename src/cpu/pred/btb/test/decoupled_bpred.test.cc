#include <gtest/gtest.h>

#include <algorithm>
#include <map>
#include <memory>
#include <vector>

#include <boost/dynamic_bitset.hpp>

// #include "arch/generic/pcstate.hh"
#include "cpu/pred/btb/fetch_target_queue.hh"
#include "cpu/pred/btb/stream_struct.hh"
#include "cpu/pred/btb/test/decoupled_bpred.hh"
#include "cpu/pred/btb/test/test_dprintf.hh"

// #include "cpu/static_inst.hh"


namespace gem5
{
namespace branch_prediction
{
namespace btb_pred
{
namespace test
{

// // Forward declarations for mock classes
// class MockStaticInst;
// class MockPCState;

// /**
//  * @brief Helper function to create a branch information object
//  */
// BranchInfo createBranchInfo(Addr pc, Addr target, bool isCond = false,
//                            bool isIndirect = false, bool isCall = false,
//                            bool isReturn = false, uint8_t size = 4) {
//     BranchInfo info;
//     info.pc = pc;
//     info.target = target;
//     info.isCond = isCond;
//     info.isIndirect = isIndirect;
//     info.isCall = isCall;
//     info.isReturn = isReturn;
//     info.size = size;
//     return info;
// }

// /**
//  * @brief Helper function to create a BTB entry
//  */
// BTBEntry createBTBEntry(Addr pc, Addr target, Addr tag = 0, bool isCond = false,
//                        bool isIndirect = false, bool isCall = false,
//                        bool isReturn = false, bool valid = true,
//                        bool alwaysTaken = false, int ctr = 0,
//                        uint8_t size = 4) {
//     BTBEntry entry;
//     entry.pc = pc;
//     entry.target = target;
//     entry.tag = tag;
//     entry.isCond = isCond;
//     entry.isIndirect = isIndirect;
//     entry.isCall = isCall;
//     entry.isReturn = isReturn;
//     entry.valid = valid;
//     entry.alwaysTaken = alwaysTaken;
//     entry.ctr = ctr;
//     entry.size = size;
//     return entry;
// }

// /**
//  * @brief Mock PCState for testing
//  */
// class MockPCState : public PCStateBase
// {
// public:
//     MockPCState(Addr pc = 0) { _pc = pc; }

//     // Implementation of abstract methods
//     PCStateBase *clone() const override { return new MockPCState(_pc); }
//     void output(std::ostream &os) const override { os << "PC:" << _pc; }
//     void advance() override { _pc += 4; }
//     bool branching() const override { return false; }
// };

/**
 * @brief Mock StaticInst for testing
 */
// class MockStaticInst : public StaticInst
// {
// public:
//     MockStaticInst() : StaticInst("mock", No_OpClass) {}

//     Fault
//     initiateAcc(ExecContext *xc, Trace::InstRecord *traceData) const override
//     {
//         return NoFault;
//     }

//     Fault
//     completeAcc(Packet *pkt, ExecContext *xc,
//                 Trace::InstRecord *traceData) const override
//     {
//         return NoFault;
//     }

//     Fault execute(ExecContext *xc, Trace::InstRecord *traceData) const override
//     {
//         return NoFault;
//     }

//     void advancePC(PCStateBase &pc_state) const override
//     {
//         pc_state.advance();
//     }

//     std::string generateDisassembly(
//             Addr pc, const loader::SymbolTable *symtab) const override
//     {
//         return "mock";
//     }

//     // Helper to set flags easily
//     void setFlag(Flags flag) {
//         flags[flag] = true;
//     }
// };

/**
 * @brief Execute a prediction cycle in the decoupled BPU
 */
void executePredictionCycle(DecoupledBPUWithBTB& dbpu) {
    // Execute a single prediction cycle
    dbpu.tick();
    bool in_loop = false;
    dbpu.trySupplyFetchWithTarget(0x1000, in_loop);
}

// /**
//  * @brief Add a branch to BTB and update decoupled BPU
//  */
// void updateWithBranch(DecoupledBPUWithBTB& dbpu, Addr startPC,
//                      const BranchInfo& branch, bool taken) {
//     // Create PC states for branch PC and target PC
//     MockPCState branchPC(branch.pc);
//     MockPCState targetPC(branch.target);

//     // Create appropriate static instruction
//     StaticInstPtr inst = nullptr;
//     // StaticInstPtr inst = new MockStaticInst();
//     // if (branch.isCond) inst->setFlag(StaticInst::IsCondControl);
//     // if (branch.isIndirect) inst->setFlag(StaticInst::IsIndirectControl);
//     // if (branch.isCall) inst->setFlag(StaticInst::IsCall);
//     // if (branch.isReturn) inst->setFlag(StaticInst::IsReturn);
//     // inst->setFlag(StaticInst::IsControl);

//     // Get current FSQ and FTQ IDs
//     FetchStreamId fsqId = dbpu.fsqId - 1;
//     unsigned targetId = dbpu.fetchTargetQueue.getSupplyingTargetId();

//     // Control squash to update predictor
//     dbpu.controlSquash(
//         targetId,       // FTQ ID
//         fsqId,          // FSQ ID
//         branchPC,       // Control PC
//         targetPC,       // Target PC
//         inst,           // Static instruction
//         branch.size,    // Instruction size
//         taken,          // Actually taken
//         1,              // Sequence number
//         0,              // Thread ID
//         0,              // Current loop iteration
//         false           // From commit
//     );

//     // Run a few ticks to process the update
//     for (int i = 0; i < 3; i++) {
//         dbpu.tick();
//     }
// }

// /**
//  * Function to check if a branch is in the FSQ
//  * Since fetchStreamQueue is private, we'll use the existing public interface
//  */
// bool checkBranchInFSQ(DecoupledBPUWithBTB* dbpu, Addr pc) {
//     // We need to simulate a call to the predictor with the branch's starting PC
//     // and check if it predicts the branch correctly

//     // Execute a prediction cycle first
//     executePredictionCycle(*dbpu);

//     // Create a dummy instruction to query the predictor
//     StaticInstPtr inst = new MockStaticInst();
//     InstSeqNum seq = 1;
//     MockPCState pcState(pc);
//     unsigned currentLoopIter = 0;

//     // Use decoupledPredict to check if the branch is predicted
//     auto [taken, run_out] = dbpu->decoupledPredict(inst, seq, pcState, 0, currentLoopIter);

//     // For branches that are in the predictor, the prediction would return meaningful results
//     // This is a simplistic check but works for basic testing
//     return (pcState.instAddr() != pc + 4);
// }

class DecoupledBPUTest : public ::testing::Test
{
protected:
    Addr pc = 0x1000;
    void SetUp() override {
        dbpu = new DecoupledBPUWithBTB();
        // Set initial PC
        dbpu->resetPC(pc);
    }

    void pc_advance() {
        pc += 4;
    }

    void TearDown() override {
        delete dbpu;
    }

    DecoupledBPUWithBTB* dbpu;
};

// Test basic initialization
TEST_F(DecoupledBPUTest, Initialization) {
    // Basic initialization test passes if no crashes/assertions
    SUCCEED();
}

// Test basic prediction cycle
TEST_F(DecoupledBPUTest, BasicPredictionCycle) {
    // warm up 3 cycles: BP -> FSQ -> FTQ
    for (int i = 0; i < 3; i++) {
        executePredictionCycle(*dbpu);
    }

    // Verify that a prediction happened
    // We can't directly check FSQ contents, but FTQ should have entries
    EXPECT_TRUE(dbpu->fetchTargetAvailable());
}

// // Test prediction and update for conditional branch
// TEST_F(DecoupledBPUTest, ConditionalBranchPrediction) {
//     // Initialize the prediction cycle
//     executePredictionCycle(*dbpu);

//     // Create a conditional branch and update the predictor
//     BranchInfo branch = createBranchInfo(0x1010, 0x2000, true);
//     updateWithBranch(*dbpu, 0x1000, branch, true);

//     // Run a few more cycles to make sure BTB is updated
//     for (int i = 0; i < 3; i++) {
//         executePredictionCycle(*dbpu);
//     }

//     // Verify the branch was added to BTB (indirectly)
//     EXPECT_TRUE(dbpu->fetchTargetAvailable());
// }

// // Test prediction for multiple branches in sequence
// TEST_F(DecoupledBPUTest, MultipleBranchSequence) {
//     // Initialize the prediction cycle
//     executePredictionCycle(*dbpu);

//     // Create and update with first branch (taken)
//     BranchInfo branch1 = createBranchInfo(0x1008, 0x2000, true);
//     updateWithBranch(*dbpu, 0x1000, branch1, true);

//     // Run cycles to update
//     for (int i = 0; i < 3; i++) {
//         executePredictionCycle(*dbpu);
//     }

//     // Create and update with second branch at target (not taken)
//     BranchInfo branch2 = createBranchInfo(0x2008, 0x3000, true);
//     updateWithBranch(*dbpu, 0x2000, branch2, false);

//     // Run cycles to update
//     for (int i = 0; i < 3; i++) {
//         executePredictionCycle(*dbpu);
//     }

//     // Verify both branches are in the prediction system (indirectly)
//     EXPECT_TRUE(dbpu->fetchTargetAvailable());
// }

// // Test prediction for indirect branches
// TEST_F(DecoupledBPUTest, IndirectBranchPrediction) {
//     // Initialize the prediction cycle
//     executePredictionCycle(*dbpu);

//     // Create an indirect branch
//     BranchInfo branch = createBranchInfo(0x1010, 0x2000, false, true);
//     updateWithBranch(*dbpu, 0x1000, branch, true);

//     // Run cycles to update
//     for (int i = 0; i < 3; i++) {
//         executePredictionCycle(*dbpu);
//     }

//     // Verify the branch was added (indirectly)
//     EXPECT_TRUE(dbpu->fetchTargetAvailable());

//     // Update with a different target
//     branch.target = 0x3000;
//     updateWithBranch(*dbpu, 0x1000, branch, true);

//     // Run cycles to update
//     for (int i = 0; i < 3; i++) {
//         executePredictionCycle(*dbpu);
//     }
// }

// // Test call/return prediction
// TEST_F(DecoupledBPUTest, CallReturnPrediction) {
//     // Initialize the prediction cycle
//     executePredictionCycle(*dbpu);

//     // Create a call instruction
//     BranchInfo call = createBranchInfo(0x1010, 0x2000, false, false, true, false);
//     updateWithBranch(*dbpu, 0x1000, call, true);

//     // Run cycles to update
//     for (int i = 0; i < 3; i++) {
//         executePredictionCycle(*dbpu);
//     }

//     // Create a return instruction
//     BranchInfo ret = createBranchInfo(0x2020, 0x1014, false, false, false, true);
//     updateWithBranch(*dbpu, 0x2000, ret, true);

//     // Run cycles to update
//     for (int i = 0; i < 3; i++) {
//         executePredictionCycle(*dbpu);
//     }

//     // Verify both instructions were added (indirectly)
//     EXPECT_TRUE(dbpu->fetchTargetAvailable());
// }

// // Test non-control squash
// TEST_F(DecoupledBPUTest, NonControlSquash) {
//     // Initialize the prediction cycle
//     executePredictionCycle(*dbpu);

//     // Get current FSQ and FTQ IDs
//     FetchStreamId fsqId = dbpu->fsqId - 1;
//     unsigned targetId = dbpu->fetchTargetQueue.getSupplyingTargetId();

//     // Create PC state
//     MockPCState pcState(0x1020);

//     // Execute non-control squash
//     dbpu->nonControlSquash(targetId, fsqId, pcState, 1, 0, 0);

//     // Run a few cycles to recover
//     for (int i = 0; i < 3; i++) {
//         executePredictionCycle(*dbpu);
//     }

//     // Verify recovery happened (indirectly)
//     EXPECT_TRUE(dbpu->fetchTargetAvailable());
// }

} // namespace test
} // namespace btb_pred
} // namespace branch_prediction
} // namespace gem5

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


