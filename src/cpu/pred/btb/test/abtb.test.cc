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

// Test fixture for Pipelined BTB tests
class ABTBTest : public ::testing::Test
{
protected:
    void SetUp() override {
        // Create a BTB with 16 entries, 8-bit tags, 4-way associative, 1-cycle delay
        // The last parameter (true) enables pipelined operation
        abtb = new DefaultBTB(16, 20, 4, 1, 1);
    }

    void TearDown() override {
        delete abtb;
    }

    DefaultBTB* abtb;
};

// Test basic ahead pipeline functionality
TEST_F(ABTBTest, BasicPipelineOperation) {
    // Phase 1: First prediction
    boost::dynamic_bitset<> history(8, 0);
    std::vector<FullBTBPrediction> stagePreds(2);  // 2 stages
    // these fetch streams are predicted by a perfect predictor, no squash is needed, though we train aBTB in the process
    FetchStream streamA;
    streamA.startPC = 0x1000;;
    streamA.resolved = true;
    streamA.exeBranchInfo.pc = 0x1004;
    streamA.exeBranchInfo.target = 0x2000;
    streamA.exeBranchInfo.isCond = true;
    streamA.exeBranchInfo.size = 4;
    streamA.exeTaken = true;
    streamA.updateEndInstPC = 0x1004;

    FetchStream streamB;
    streamB.startPC = 0x2000;
    streamB.resolved = true;
    streamB.exeBranchInfo.pc = 0x2004;
    streamB.exeBranchInfo.target = 0x3000;
    streamB.exeBranchInfo.isCond = true;
    streamB.exeBranchInfo.size = 4;
    streamB.exeTaken = true;
    streamB.updateEndInstPC = 0x2004;


    // training phase
    // Perform initial prediction
    abtb->putPCHistory(streamA.startPC, history, stagePreds);
    // Get prediction metadata
    streamA.predMetas[0] = abtb->getPredictionMeta();
    // second prediction
    abtb->putPCHistory(streamB.startPC, history, stagePreds);
    streamB.predMetas[0] = abtb->getPredictionMeta();
    // Push the previous PC into previousPCs field (key for ahead pipeline testing)
    streamB.previousPCs.push(streamA.startPC);
    // Update BTB with the branch information
    abtb->getAndSetNewBTBEntry(streamA);
    abtb->update(streamA);
    abtb->getAndSetNewBTBEntry(streamB);
    abtb->update(streamB);


    // testing phase

    abtb->putPCHistory(streamA.startPC, history, stagePreds);
    abtb->putPCHistory(streamB.startPC, history, stagePreds);
    // Check predictions from streamB
    ASSERT_FALSE(stagePreds[1].btbEntries.empty());
    ASSERT_EQ(stagePreds[1].btbEntries.size(), 1);
    EXPECT_EQ(stagePreds[1].btbEntries[0].pc, 0x2004);
    EXPECT_EQ(stagePreds[1].btbEntries[0].target, 0x3000);
}

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
