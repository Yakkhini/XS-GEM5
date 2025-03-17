#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "cpu/pred/btb/test/mockbtb.hh"

namespace gem5
{

namespace branch_prediction
{

namespace btb_pred
{

namespace test
{

// Test fixture for BTB tests
class BTBTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create a BTB with 16 entries, 8-bit tags, and 4-way set associative
        ubtb = new DefaultBTB(16, 8, 4, 0);  // ubtb
        mbtb = new DefaultBTB(16, 8, 4, 1); // mbtb
    }
    
    void TearDown() override {
        delete ubtb;
        delete mbtb;
    }
    
    DefaultBTB* ubtb;
    DefaultBTB* mbtb;
};

// Test basic initialization
TEST_F(BTBTest, Initialization) {
    // Create a new BTB with different parameters
    DefaultBTB testBtb(32, 12, 8, 0);
    // Basic initialization test passes if no crashes/assertions
    SUCCEED();
}

// Test basic prediction with empty BTB
TEST_F(BTBTest, EmptyPrediction) {
    Addr startAddr = 0x1000;
    boost::dynamic_bitset<> history(8, 0);  // 8-bit history, all zeros
    std::vector<FullBTBPrediction> stagePreds(4);  // 4 stages
    
    mbtb->putPCHistory(startAddr, history, stagePreds);
    
    // Check predictions for all stages
    for (int i = 0; i < stagePreds.size(); i++) {
        EXPECT_TRUE(stagePreds[i].btbEntries.empty());
        EXPECT_FALSE(stagePreds[i].isTaken());
    }
}

// BTB actual update process:
// 1. putPCHistory, store result in stagePreds, update meta
// 2. getPredictionMeta, set to stream.predMetas[0]
// 3. getAndSetNewBTBEntry, only L1 BTB has this function
// 4. update, update btb entries

// Test basic prediction after update
TEST_F(BTBTest, PredictionAfterUpdate) {
    // First update BTB with a branch
    FetchStream stream;
    stream.startPC = 0x1000;
    stream.resolved = true;
    stream.exeBranchInfo.pc = 0x1000;
    stream.exeBranchInfo.target = 0x2000;
    stream.exeBranchInfo.isCond = true;
    stream.exeBranchInfo.size = 4;
    stream.exeTaken = true;
    
    // Set up prediction metadata before update
    boost::dynamic_bitset<> history(8, 0);
    std::vector<FullBTBPrediction> stagePreds(4);
    mbtb->putPCHistory(0x1000, history, stagePreds);
    stream.predMetas[0] = mbtb->getPredictionMeta();
    
    // Set update info
    stream.updateEndInstPC = 0x1004;  // pc + size
    mbtb->getAndSetNewBTBEntry(stream);  // Generate new entry
    
    // Update BTB with this branch
    mbtb->update(stream);
    
    // Now try to predict
    stagePreds.clear();
    stagePreds.resize(4);
    mbtb->putPCHistory(0x1000, history, stagePreds);
    
    // Check predictions
    for (int i = mbtb->getDelay(); i < stagePreds.size(); i++) {
        EXPECT_FALSE(stagePreds[i].btbEntries.empty());
        auto &entries = stagePreds[i].btbEntries;
        ASSERT_EQ(entries.size(), 1);
        EXPECT_EQ(entries[0].pc, 0x1000);
        EXPECT_EQ(entries[0].target, 0x2000);
    }
}

// Test conditional branch prediction counter, for L0 BTB/uBTB
TEST_F(BTBTest, ConditionalCounter) {
    FetchStream stream;
    stream.startPC = 0x1000;
    stream.resolved = true;     // resolved is true, so exeBranchInfo is used
    stream.exeBranchInfo.pc = 0x1000;
    stream.exeBranchInfo.target = 0x2000;
    stream.exeBranchInfo.isCond = true;
    stream.exeBranchInfo.size = 4;
    
    // Set up prediction metadata
    boost::dynamic_bitset<> history(8, 0);
    std::vector<FullBTBPrediction> stagePreds(4);
    
    // First update with taken
    stream.exeTaken = true;
    ubtb->putPCHistory(0x1000, history, stagePreds);
    stream.predMetas[0] = ubtb->getPredictionMeta();
    stream.updateEndInstPC = 0x1004;
    mbtb->getAndSetNewBTBEntry(stream);  // Generate new entry, add to ubtb, ctr init to 1
    ubtb->update(stream);
    
    // Check counter after taken
    stagePreds.clear();
    stagePreds.resize(4);
    ubtb->putPCHistory(0x1000, history, stagePreds);
    
    // Counter should be initialized to 1 and stay at 1 after taken
    for (int i = ubtb->getDelay(); i < stagePreds.size(); i++) {
        ASSERT_FALSE(stagePreds[i].btbEntries.empty());
        auto &entries = stagePreds[i].btbEntries;
        EXPECT_EQ(entries[0].ctr, 1);
        EXPECT_TRUE(entries[0].alwaysTaken);
    }
    
    // Then update with not taken
    stream.exeTaken = false;
    ubtb->putPCHistory(0x1000, history, stagePreds);
    stream.predMetas[0] = ubtb->getPredictionMeta();
    stream.updateEndInstPC = 0x1004;
    mbtb->getAndSetNewBTBEntry(stream);
    ubtb->update(stream); // update ctr, alwaysTaken = false, ctr -- = 0
    
    // Check prediction
    stagePreds.clear();
    stagePreds.resize(4);
    ubtb->putPCHistory(0x1000, history, stagePreds);
    
    // Counter should be 0 after not taken (1 -> 0)
    for (int i = ubtb->getDelay(); i < stagePreds.size(); i++) {
        ASSERT_FALSE(stagePreds[i].btbEntries.empty());
        auto &entries = stagePreds[i].btbEntries;
        EXPECT_EQ(entries[0].ctr, -1);  // corner case: ctr -- = -1
        EXPECT_FALSE(entries[0].alwaysTaken);
    }
}

// Test counter saturation behavior, for L0 BTB/uBTB
TEST_F(BTBTest, CounterSaturation) {
    FetchStream stream;
    stream.startPC = 0x1000;
    stream.resolved = true;     // resolved is true, so exeBranchInfo is used
    stream.exeBranchInfo.pc = 0x1000;
    stream.exeBranchInfo.target = 0x2000;
    stream.exeBranchInfo.isCond = true;
    stream.exeBranchInfo.size = 4;
    
    boost::dynamic_bitset<> history(8, 0);
    std::vector<FullBTBPrediction> stagePreds(4);
    
    // First entry is initialized with ctr=1
    stream.exeTaken = true;
    ubtb->putPCHistory(0x1000, history, stagePreds);
    stream.predMetas[0] = ubtb->getPredictionMeta();
    stream.updateEndInstPC = 0x1004;
    ubtb->getAndSetNewBTBEntry(stream);
    ubtb->update(stream);
    
    // Check counter is at 1
    stagePreds.clear();
    stagePreds.resize(4);
    ubtb->putPCHistory(0x1000, history, stagePreds);
    
    for (int i = ubtb->getDelay(); i < stagePreds.size(); i++) {
        ASSERT_FALSE(stagePreds[i].btbEntries.empty());
        auto &entries = stagePreds[i].btbEntries;
        EXPECT_EQ(entries[0].ctr, 1);  // Counter should be at 1
        EXPECT_TRUE(entries[0].alwaysTaken);
    }
    
    // Update multiple times with not taken to test negative saturation
    for (int i = 0; i < 3; i++) {  // 3 times
        stream.exeTaken = false;
        ubtb->putPCHistory(0x1000, history, stagePreds);
        stream.predMetas[0] = ubtb->getPredictionMeta();
        stream.updateEndInstPC = 0x1004;
        ubtb->getAndSetNewBTBEntry(stream);
        ubtb->update(stream);
    }
    
    // Check counter is saturated at -2
    stagePreds.clear();
    stagePreds.resize(4);
    ubtb->putPCHistory(0x1000, history, stagePreds);
    
    for (int i = ubtb->getDelay(); i < stagePreds.size(); i++) {
        ASSERT_FALSE(stagePreds[i].btbEntries.empty());
        auto &entries = stagePreds[i].btbEntries;
        EXPECT_EQ(entries[0].ctr, -2);  // Counter should saturate at -2
        EXPECT_FALSE(entries[0].alwaysTaken);
    }
}

// Test MRU replacement policy
TEST_F(BTBTest, ReplacementPolicy) {
    // Fill up a BTB set completely
    FetchStream stream;
    for (int i = 0; i < 5; i++) {  // numWays = 4, last one is for replacement
        stream.startPC = 0x1000 + i * 0x1000;  // same sets 0, different ways
        stream.resolved = true;
        stream.exeBranchInfo.pc = stream.startPC;
        stream.exeBranchInfo.target = 0x2000 + i * 0x1000;
        stream.exeBranchInfo.isCond = true;
        stream.exeBranchInfo.size = 4;
        stream.exeTaken = true;
        
        // Predict and update
        boost::dynamic_bitset<> history(8, 0);
        std::vector<FullBTBPrediction> stagePreds(4);
        mbtb->putPCHistory(stream.startPC, history, stagePreds);
        stream.predMetas[0] = mbtb->getPredictionMeta();
        stream.updateEndInstPC = stream.startPC + 4;
        mbtb->getAndSetNewBTBEntry(stream);
        mbtb->update(stream);
    }
    
    // The least recently used entry should be replaced
    boost::dynamic_bitset<> history(8, 0);
    std::vector<FullBTBPrediction> stagePreds(4);
    mbtb->putPCHistory(0x1000, history, stagePreds);
    EXPECT_TRUE(stagePreds[mbtb->getDelay()].btbEntries.empty());  // First entry should be evicted
}

// Test indirect branch prediction
TEST_F(BTBTest, IndirectBranchPrediction) {
    FetchStream stream;
    stream.startPC = 0x1000;
    stream.resolved = true;
    stream.exeBranchInfo.pc = 0x1000;
    stream.exeBranchInfo.target = 0x2000;
    stream.exeBranchInfo.isIndirect = true;
    stream.exeBranchInfo.size = 4;
    stream.exeTaken = true;
    
    // Initial prediction and update
    boost::dynamic_bitset<> history(8, 0);
    std::vector<FullBTBPrediction> stagePreds(4);
    mbtb->putPCHistory(stream.startPC, history, stagePreds);
    stream.predMetas[0] = mbtb->getPredictionMeta();
    stream.updateEndInstPC = stream.startPC + 4;
    mbtb->getAndSetNewBTBEntry(stream);
    mbtb->update(stream);
    
    // Check prediction
    stagePreds.clear();
    stagePreds.resize(4);
    mbtb->putPCHistory(0x1000, history, stagePreds);
    
    // Verify indirect target
    for (int i = mbtb->getDelay(); i < stagePreds.size(); i++) {
        ASSERT_FALSE(stagePreds[i].btbEntries.empty());
        EXPECT_EQ(stagePreds[i].indirectTargets[0x1000], 0x2000);
    }
    
    // Update with new target
    stream.exeBranchInfo.target = 0x3000;
    mbtb->putPCHistory(stream.startPC, history, stagePreds);
    stream.predMetas[0] = mbtb->getPredictionMeta();
    mbtb->getAndSetNewBTBEntry(stream);
    mbtb->update(stream);
    
    // Check updated prediction
    stagePreds.clear();
    stagePreds.resize(4);
    mbtb->putPCHistory(0x1000, history, stagePreds);
    
    // Verify new indirect target
    for (int i = mbtb->getDelay(); i < stagePreds.size(); i++) {
        EXPECT_EQ(stagePreds[i].indirectTargets[0x1000], 0x3000);
    }
}

// Test multiple branch predictions in same fetch block
TEST_F(BTBTest, MultipleBranchPrediction) {
    // Add two branches in same fetch block
    FetchStream stream;
    stream.startPC = 0x1000;
    stream.resolved = true;
    // First branch
    stream.exeBranchInfo.pc = 0x1000;
    stream.exeBranchInfo.target = 0x2000;
    stream.exeBranchInfo.isCond = true;
    stream.exeBranchInfo.size = 4;
    stream.exeTaken = true;
    
    boost::dynamic_bitset<> history(8, 0);
    std::vector<FullBTBPrediction> stagePreds(4);
    mbtb->putPCHistory(stream.startPC, history, stagePreds);
    stream.predMetas[0] = mbtb->getPredictionMeta();
    stream.updateEndInstPC = stream.startPC + 8;  // Include both branches
    mbtb->getAndSetNewBTBEntry(stream);
    mbtb->update(stream);
    
    // Second branch
    stream.exeBranchInfo.pc = 0x1004;
    stream.exeBranchInfo.target = 0x3000;
    mbtb->getAndSetNewBTBEntry(stream);
    mbtb->update(stream);
    
    // Check predictions
    stagePreds.clear();
    stagePreds.resize(4);
    mbtb->putPCHistory(0x1000, history, stagePreds);
    
    // Verify both branches are predicted
    for (int i = mbtb->getDelay(); i < stagePreds.size(); i++) {
        ASSERT_EQ(stagePreds[i].btbEntries.size(), 2);
        EXPECT_EQ(stagePreds[i].btbEntries[0].pc, 0x1000);
        EXPECT_EQ(stagePreds[i].btbEntries[0].target, 0x2000);
        EXPECT_EQ(stagePreds[i].btbEntries[1].pc, 0x1004);
        EXPECT_EQ(stagePreds[i].btbEntries[1].target, 0x3000);
    }
}

// Test recovery from misprediction
TEST_F(BTBTest, MispredictionRecovery) {
    FetchStream stream;
    stream.startPC = 0x1000;
    stream.resolved = true;
    stream.exeBranchInfo.pc = 0x1000;
    stream.exeBranchInfo.target = 0x2000;
    stream.exeBranchInfo.isCond = true;
    stream.exeBranchInfo.size = 4;
    stream.exeTaken = true;
    
    // Initial prediction and update, save the new entry in BTB
    boost::dynamic_bitset<> history(8, 0);
    std::vector<FullBTBPrediction> stagePreds(4);
    mbtb->putPCHistory(stream.startPC, history, stagePreds);
    stream.predMetas[0] = mbtb->getPredictionMeta();
    stream.updateEndInstPC = stream.startPC + 4;
    mbtb->getAndSetNewBTBEntry(stream);
    mbtb->update(stream);

    // predict again, get the old entry as the prediction
    stagePreds.clear();
    stagePreds.resize(4);
    mbtb->putPCHistory(stream.startPC, history, stagePreds);
    stream.predMetas[0] = mbtb->getPredictionMeta();
    
    // Simulate misprediction
    stream.exeTaken = false;
    stream.exeBranchInfo.target = 0x1004;  // Fall through
    
    // Update with correct information
    mbtb->getAndSetNewBTBEntry(stream);
    mbtb->update(stream);
    
    // Check updated prediction
    stagePreds.clear();
    stagePreds.resize(4);
    mbtb->putPCHistory(0x1000, history, stagePreds);
    
    // Verify prediction is updated
    for (int i = mbtb->getDelay(); i < stagePreds.size(); i++) {
        ASSERT_FALSE(stagePreds[i].btbEntries.empty());
        auto &entries = stagePreds[i].btbEntries;
        EXPECT_FALSE(entries[0].alwaysTaken);
        // EXPECT_FALSE(stagePreds[i].condTakens[0x1000]);
    }
}

} // namespace test
} // namespace btb_pred
} // namespace branch_prediction
} // namespace gem5
