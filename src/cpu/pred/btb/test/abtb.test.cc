#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "cpu/pred/btb/test/mockbtb.hh"

namespace gem5
{

namespace branch_prediction
{

namespace btb_pred
{

namespace test
{


FetchStream createStream(Addr startPC, FullBTBPrediction &pred, DefaultBTB *abtb) {
    FetchStream stream;
    stream.startPC = startPC;
    Addr fallThroughAddr = pred.getFallThrough();
    stream.isHit = pred.btbEntries.size() > 0; // TODO: fix isHit and falseHit
    stream.falseHit = false;
    stream.predBTBEntries = pred.btbEntries;
    stream.predTaken = pred.isTaken();
    stream.predEndPC = fallThroughAddr;
    stream.predMetas[0] = abtb->getPredictionMeta();
    return stream;
}

void resolveStream(FetchStream &stream, bool taken, Addr brPc, Addr target, bool isCond, int size=4) {
    stream.resolved = true;
    stream.exeBranchInfo.pc = brPc;
    stream.exeBranchInfo.target = target;
    stream.exeBranchInfo.isCond = isCond;
    stream.exeBranchInfo.size = size;
    stream.exeTaken = taken;
}

FullBTBPrediction makePrediction(Addr startPC, DefaultBTB *abtb) {
    std::vector<FullBTBPrediction> stagePreds(2);  // 2 stages
    boost::dynamic_bitset<> history(8, 0); // history does not matter for BTB
    abtb->putPCHistory(startPC, history, stagePreds);
    return stagePreds[1];
}

void updateBTB(FetchStream &stream, DefaultBTB *abtb) {
    abtb->getAndSetNewBTBEntry(stream); // usually called by mbtb, here for testing purpose
    abtb->update(stream);
}


// Test fixture for Pipelined BTB tests
class ABTBTest : public ::testing::Test
{
protected:
    void SetUp() override {
        // Create a BTB with 16 entries, 8-bit tags, 4-way associative, 1-cycle delay
        // The last parameter (true) enables pipelined operation
        abtb = new DefaultBTB(16, 20, 4, 1, false, 1);
        assert(!abtb->halfAligned);

        bigAbtb = new DefaultBTB(1024, 20, 1, 1, false, 1);
    }

    void TearDown() override {
        delete abtb;
        delete bigAbtb;
    }

    DefaultBTB* abtb;
    DefaultBTB* bigAbtb;
};

TEST_F(ABTBTest, BasicPredictionUpdateCycle){
    // Some constants
    // Stream A addresses
    Addr startPC_A = 0x1000;
    Addr brPC_A = 0x1004;
    Addr target_A = 0x2000;

    // Stream B addresses
    Addr startPC_B = 0x2000;
    Addr brPC_B = 0x2004;
    Addr target_B = 0x3000;

    // ---------------- training phase ----------------
    // make predictions and create Fetch Streams
    auto pred_A = makePrediction(startPC_A, abtb);
    auto stream_A = createStream(startPC_A, pred_A, abtb);
    auto pred_B = makePrediction(startPC_B, abtb);
    auto stream_B = createStream(startPC_B, pred_B, abtb);
    stream_B.previousPCs.push(stream_A.startPC); // crucial! set previous PC for ahead pipelining
    // resolve Fetch Stream (FS reached commit stage of backend)
    resolveStream(stream_A, true, brPC_A, target_A, true);
    resolveStream(stream_B, true, brPC_B, target_B, true);
    // update BTB with branch information
    updateBTB(stream_A, abtb);
    updateBTB(stream_B, abtb);

    // ---------------- testing phase ----------------
    // make predictions and check if BTB is updated correctly
    auto pred_A_test = makePrediction(startPC_A, abtb);
    auto pred_B_test = makePrediction(startPC_B, abtb);
    EXPECT_EQ(pred_B_test.btbEntries.size(), 1);
    if (!pred_B_test.btbEntries.empty()) {
        EXPECT_EQ(pred_B_test.btbEntries[0].pc, brPC_B);
        EXPECT_EQ(pred_B_test.btbEntries[0].target, target_B);
    }

}

TEST_F(ABTBTest, AliasAvoidance){
    // Some constants
    // Stream A addresses
    Addr startPC_A = 0x100;
    Addr brPC1_A = 0x104;
    Addr brPC2_A = 0x108;
    Addr target1_A = 0x200;
    Addr target2_A = 0x300;
    // Stream B addresses
    Addr startPC_B = 0x300;
    Addr brPC_B = 0x304;
    Addr target_B = 0x3000;

    // Stream C addresses
    Addr startPC_C = 0x200;
    Addr brPC_C = 0x204;
    Addr target_C = 0x2000;

    // ---------------- training phase ----------------
    // make predictions and create Fetch Streams
    auto pred_A = makePrediction(startPC_A, bigAbtb);
    auto stream_A = createStream(startPC_A, pred_A, bigAbtb);
    auto pred_B = makePrediction(startPC_B, bigAbtb);
    auto stream_B = createStream(startPC_B, pred_B, bigAbtb);
    stream_B.previousPCs.push(stream_A.startPC); // crucial! set previous PC for ahead pipelining
    // resolve Fetch Stream (FS reached commit stage of backend)
    resolveStream(stream_A, true, brPC1_A, target1_A, true);
    resolveStream(stream_B, true, brPC_B, target_B, true);
    // update BTB with branch information
    // now aBTB ought to have a entry, indexed by startPC_A, tagged with startPC_B
    updateBTB(stream_A, bigAbtb);
    updateBTB(stream_B, bigAbtb);

    // ---------------- testing phase ----------------
    // when we've arrived at Fetch Block C, aBTB shouldn't return the entry trained with Fetch Block B
    // though the mistake is likely to happen, because FB C and FB B share the same tag bits
    // the solution is to store the startPC in a aBTB entry
    auto pred_A_test = makePrediction(startPC_A, bigAbtb);
    auto pred_C_test = makePrediction(startPC_C, bigAbtb);
    EXPECT_EQ(pred_C_test.btbEntries.size(), 0);
}

// TODO: for now the rest of the tests aren't working
// Test pipelined prediction with multiple branches
TEST_F(ABTBTest, MultipleBranchPipeline) {
    // Phase 1: Initial prediction
    Addr startAddr = 0x1000;
    boost::dynamic_bitset<> history(8, 0);
    std::vector<FullBTBPrediction> stagePreds(2);

    abtb->putPCHistory(startAddr, history, stagePreds);
    auto meta = abtb->getPredictionMeta();

    // Phase 2: Update with first branch
    FetchStream stream;
    stream.startPC = startAddr;
    stream.predMetas[0] = meta;
    stream.resolved = true;
    stream.exeBranchInfo.pc = 0x1000;
    stream.exeBranchInfo.target = 0x2000;
    stream.exeBranchInfo.isCond = true;
    stream.exeBranchInfo.size = 4;
    stream.exeTaken = true;
    stream.updateEndInstPC = 0x1008;  // Cover both branches

    // Set previous PC
    stream.previousPCs.push(startAddr);

    // Update with first branch
    abtb->getAndSetNewBTBEntry(stream);
    abtb->update(stream);

    // Phase 3: Second prediction to get new metadata
    stagePreds.clear();
    stagePreds.resize(2);
    abtb->putPCHistory(startAddr, history, stagePreds);
    meta = abtb->getPredictionMeta();

    // Phase 4: Update with second branch
    stream.predMetas[0] = meta;
    stream.exeBranchInfo.pc = 0x1004;
    stream.exeBranchInfo.target = 0x3000;

    // Clear and set previous PC again
    stream.previousPCs.pop();
    stream.previousPCs.push(startAddr);

    // Update with second branch
    abtb->getAndSetNewBTBEntry(stream);
    abtb->update(stream);

    // Phase 5: Final prediction to verify both branches
    stagePreds.clear();
    stagePreds.resize(2);
    abtb->putPCHistory(startAddr, history, stagePreds);

    // Verify both branches are predicted correctly
    ASSERT_EQ(stagePreds[1].btbEntries.size(), 2);
    EXPECT_EQ(stagePreds[1].btbEntries[0].pc, 0x1000);
    EXPECT_EQ(stagePreds[1].btbEntries[0].target, 0x2000);
    EXPECT_EQ(stagePreds[1].btbEntries[1].pc, 0x1004);
    EXPECT_EQ(stagePreds[1].btbEntries[1].target, 0x3000);
}

// Test prediction after multi-cycle pipeline
TEST_F(ABTBTest, MultiCyclePipeline) {
    // Sequence of addresses we'll predict and update
    std::vector<Addr> addresses = {0x1000, 0x1100, 0x1200};
    boost::dynamic_bitset<> history(8, 0);

    // Phase 1: First address - predict and update
    std::vector<FullBTBPrediction> stagePreds(2);
    abtb->putPCHistory(addresses[0], history, stagePreds);
    auto meta = abtb->getPredictionMeta();

    FetchStream stream;
    stream.startPC = addresses[0];
    stream.predMetas[0] = meta;
    stream.resolved = true;
    stream.exeBranchInfo.pc = addresses[0];
    stream.exeBranchInfo.target = 0x2000;
    stream.exeBranchInfo.isCond = true;
    stream.exeBranchInfo.size = 4;
    stream.exeTaken = true;
    stream.updateEndInstPC = addresses[0] + 4;

    // Set previous PC for pipeline
    stream.previousPCs.push(addresses[0]);

    // Update with first address
    abtb->getAndSetNewBTBEntry(stream);
    abtb->update(stream);

    // Phase 2: Second address - predict and update
    stagePreds.clear();
    stagePreds.resize(2);
    abtb->putPCHistory(addresses[1], history, stagePreds);
    meta = abtb->getPredictionMeta();

    stream.startPC = addresses[1];
    stream.predMetas[0] = meta;
    stream.exeBranchInfo.pc = addresses[1];
    stream.exeBranchInfo.target = 0x3000;
    stream.updateEndInstPC = addresses[1] + 4;

    // Clear and set previous PC for second address
    stream.previousPCs.pop();
    stream.previousPCs.push(addresses[1]);

    // Update with second address
    abtb->getAndSetNewBTBEntry(stream);
    abtb->update(stream);

    // Phase 3: Third address - predict and update
    stagePreds.clear();
    stagePreds.resize(2);
    abtb->putPCHistory(addresses[2], history, stagePreds);
    meta = abtb->getPredictionMeta();

    stream.startPC = addresses[2];
    stream.predMetas[0] = meta;
    stream.exeBranchInfo.pc = addresses[2];
    stream.exeBranchInfo.target = 0x4000;
    stream.updateEndInstPC = addresses[2] + 4;

    // Clear and set previous PC for third address
    stream.previousPCs.pop();
    stream.previousPCs.push(addresses[2]);

    // Update with third address
    abtb->getAndSetNewBTBEntry(stream);
    abtb->update(stream);

    // Phase 4: Verify predictions for all addresses
    for (int i = 0; i < addresses.size(); i++) {
        stagePreds.clear();
        stagePreds.resize(2);
        abtb->putPCHistory(addresses[i], history, stagePreds);

        // Check prediction
        ASSERT_FALSE(stagePreds[1].btbEntries.empty());
        ASSERT_EQ(stagePreds[1].btbEntries.size(), 1);
        EXPECT_EQ(stagePreds[1].btbEntries[0].pc, addresses[i]);
        EXPECT_EQ(stagePreds[1].btbEntries[0].target, 0x2000 + i * 0x1000);
    }
}

// Test pipelined prediction with changing targets
TEST_F(ABTBTest, ChangingTargetPipeline) {
    Addr startAddr = 0x1000;
    boost::dynamic_bitset<> history(8, 0);
    std::vector<FullBTBPrediction> stagePreds(2);

    // Phase 1: Initial prediction and update
    abtb->putPCHistory(startAddr, history, stagePreds);
    auto meta = abtb->getPredictionMeta();

    FetchStream stream;
    stream.startPC = startAddr;
    stream.predMetas[0] = meta;
    stream.resolved = true;
    stream.exeBranchInfo.pc = startAddr;
    stream.exeBranchInfo.target = 0x2000;
    stream.exeBranchInfo.isIndirect = true;  // Must be indirect to update target
    stream.exeBranchInfo.size = 4;
    stream.exeTaken = true;
    stream.updateEndInstPC = startAddr + 4;

    // Set previous PC
    stream.previousPCs.push(startAddr);

    // First update
    abtb->getAndSetNewBTBEntry(stream);
    abtb->update(stream);

    // Phase 2: Verify initial update
    stagePreds.clear();
    stagePreds.resize(2);
    abtb->putPCHistory(startAddr, history, stagePreds);
    meta = abtb->getPredictionMeta();

    // Verify first target
    ASSERT_FALSE(stagePreds[1].btbEntries.empty());
    EXPECT_EQ(stagePreds[1].btbEntries[0].target, 0x2000);

    // Phase 3: Update with new target
    stream.predMetas[0] = meta;
    stream.exeBranchInfo.target = 0x3000;  // New target

    // Set previous PC again
    stream.previousPCs.pop();
    stream.previousPCs.push(startAddr);

    // Update with new target
    abtb->getAndSetNewBTBEntry(stream);
    abtb->update(stream);

    // Phase 4: Verify target was updated
    stagePreds.clear();
    stagePreds.resize(2);
    abtb->putPCHistory(startAddr, history, stagePreds);

    // Check updated target
    ASSERT_FALSE(stagePreds[1].btbEntries.empty());
    EXPECT_EQ(stagePreds[1].btbEntries[0].target, 0x3000);
}

// Test missing previousPC in pipeline
TEST_F(ABTBTest, MissingPreviousPC) {
    Addr startAddr = 0x1000;
    boost::dynamic_bitset<> history(8, 0);
    std::vector<FullBTBPrediction> stagePreds(2);

    // Perform initial prediction
    abtb->putPCHistory(startAddr, history, stagePreds);
    auto meta = abtb->getPredictionMeta();

    // Set up update stream WITHOUT previous PC
    FetchStream stream;
    stream.startPC = startAddr;
    stream.predMetas[0] = meta;
    stream.resolved = true;
    stream.exeBranchInfo.pc = startAddr;
    stream.exeBranchInfo.target = 0x2000;
    stream.exeBranchInfo.isCond = true;
    stream.exeBranchInfo.size = 4;
    stream.exeTaken = true;
    stream.updateEndInstPC = startAddr + 4;

    // Note: NOT setting previousPCs - this would cause issues in real implementation
    // but we're just testing the MockBTB behavior

    // Update BTB
    abtb->getAndSetNewBTBEntry(stream);
    abtb->update(stream);

    // Check if prediction still works
    stagePreds.clear();
    stagePreds.resize(2);
    abtb->putPCHistory(startAddr, history, stagePreds);

    // The MockBTB should still function without previousPC
    ASSERT_FALSE(stagePreds[1].btbEntries.empty());
    EXPECT_EQ(stagePreds[1].btbEntries[0].pc, startAddr);
    EXPECT_EQ(stagePreds[1].btbEntries[0].target, 0x2000);
}

} // namespace test
} // namespace btb_pred
} // namespace branch_prediction
} // namespace gem5
