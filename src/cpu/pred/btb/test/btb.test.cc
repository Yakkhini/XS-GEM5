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

// Test half-aligned mode basic functionality
TEST_F(BTBTest, HalfAlignedBasicTest) {
    // Create a BTB with half-aligned mode enabled
    DefaultBTB btb(1024, 20, 8, 1, true);  // numEntries=1024, tagBits=20, numWays=8, numDelay=1, halfAligned=true

    // Phase 1: Initial prediction to get metadata
    std::vector<FullBTBPrediction> stagePreds(2);
    btb.putPCHistory(0x100, boost::dynamic_bitset<>(64, 0), stagePreds);
    auto meta = btb.getPredictionMeta();

    // Phase 2: Setup update stream with first branch
    FetchStream stream;
    stream.startPC = 0x100;
    stream.predMetas[0] = meta;  // Must set meta from prediction phase
    stream.resolved = true;
    stream.exeBranchInfo.pc = 0x100;
    stream.exeBranchInfo.target = 0x200;
    stream.exeBranchInfo.isCond = true;
    stream.exeBranchInfo.size = 4;
    stream.exeTaken = true;
    stream.updateEndInstPC = 0x140;  // Cover 64B range

    // Update BTB with first branch
    btb.getAndSetNewBTBEntry(stream);
    btb.update(stream);

    // Phase 3: Setup second branch in next 32B block
    // Need to predict again to get new meta
    stagePreds.clear();
    stagePreds.resize(2);
    btb.putPCHistory(0x120, boost::dynamic_bitset<>(64, 0), stagePreds);
    meta = btb.getPredictionMeta();

    stream.startPC = 0x120;
    stream.predMetas[0] = meta;
    stream.exeBranchInfo.pc = 0x120;
    stream.exeBranchInfo.target = 0x300;

    // Update BTB with second branch
    btb.getAndSetNewBTBEntry(stream);
    btb.update(stream);

    // Phase 4: Final prediction to verify results
    stagePreds.clear();
    stagePreds.resize(2);
    btb.putPCHistory(0x100, boost::dynamic_bitset<>(64, 0), stagePreds);

    // Verify both branches are found and in correct order
    ASSERT_EQ(stagePreds[1].btbEntries.size(), 2);
    EXPECT_EQ(stagePreds[1].btbEntries[0].pc, 0x100);
    EXPECT_EQ(stagePreds[1].btbEntries[0].target, 0x200);
    EXPECT_EQ(stagePreds[1].btbEntries[1].pc, 0x120);
    EXPECT_EQ(stagePreds[1].btbEntries[1].target, 0x300);
}

// Test half-aligned mode with unaligned addresses
TEST_F(BTBTest, HalfAlignedUnalignedTest) {
    // Create a BTB with half-aligned mode enabled
    DefaultBTB btb(1024, 20, 8, 1, true);

    // Phase 1: Initial prediction to get metadata
    std::vector<FullBTBPrediction> stagePreds(2);
    btb.putPCHistory(0x104, boost::dynamic_bitset<>(64, 0), stagePreds);
    auto meta = btb.getPredictionMeta();

    // Phase 2: Setup update stream with first unaligned branch
    FetchStream stream;
    stream.startPC = 0x104;
    stream.predMetas[0] = meta;
    stream.resolved = true;
    stream.exeBranchInfo.pc = 0x104;  // Unaligned address in first block
    stream.exeBranchInfo.target = 0x200;
    stream.exeBranchInfo.isCond = true;
    stream.exeBranchInfo.size = 4;
    stream.exeTaken = true;
    stream.updateEndInstPC = 0x144;  // Cover 64B range

    // Update BTB with first branch
    btb.getAndSetNewBTBEntry(stream);
    btb.update(stream);

    // Phase 3: Setup second unaligned branch in next 32B block
    // Need to predict again to get new meta
    stagePreds.clear();
    stagePreds.resize(2);
    btb.putPCHistory(0x124, boost::dynamic_bitset<>(64, 0), stagePreds);
    meta = btb.getPredictionMeta();

    stream.startPC = 0x124;
    stream.predMetas[0] = meta;
    stream.exeBranchInfo.pc = 0x124;  // Unaligned address in second block
    stream.exeBranchInfo.target = 0x300;

    // Update BTB with second branch
    btb.getAndSetNewBTBEntry(stream);
    btb.update(stream);

    // Phase 4: Final prediction to verify results
    stagePreds.clear();
    stagePreds.resize(2);
    btb.putPCHistory(0x104, boost::dynamic_bitset<>(64, 0), stagePreds);

    // Verify both unaligned branches are found and in correct order
    ASSERT_EQ(stagePreds[1].btbEntries.size(), 2);
    EXPECT_EQ(stagePreds[1].btbEntries[0].pc, 0x104);
    EXPECT_EQ(stagePreds[1].btbEntries[0].target, 0x200);
    EXPECT_EQ(stagePreds[1].btbEntries[1].pc, 0x124);
    EXPECT_EQ(stagePreds[1].btbEntries[1].target, 0x300);
}

// Test half-aligned mode update with branch in second block
TEST_F(BTBTest, HalfAlignedUpdateSecondBlock) {
    // Create a BTB with half-aligned mode enabled
    DefaultBTB btb(1024, 20, 8, 1, true);  // numEntries=1024, tagBits=20, numWays=8, numDelay=1, halfAligned=true

    // Phase 1: Initial prediction to get metadata
    // Start address in first 32B block
    Addr startPC = 0x100;
    std::vector<FullBTBPrediction> stagePreds(2);
    btb.putPCHistory(startPC, boost::dynamic_bitset<>(64, 0), stagePreds);
    auto meta = btb.getPredictionMeta();

    // Phase 2: Setup update stream with branch in second 32B block
    FetchStream stream;
    stream.startPC = startPC;
    stream.predMetas[0] = meta;
    stream.resolved = true;
    // Branch is in second 32B block (0x120 - 0x13F)
    stream.exeBranchInfo.pc = 0x124;
    stream.exeBranchInfo.target = 0x200;
    stream.exeBranchInfo.isCond = true;
    stream.exeBranchInfo.size = 4;
    stream.exeTaken = true;
    stream.updateEndInstPC = 0x140;  // Cover 64B range

    // Update BTB with branch in second block
    btb.getAndSetNewBTBEntry(stream);
    btb.update(stream);

    // Phase 3: Verify the update worked correctly
    // Predict from first block
    stagePreds.clear();
    stagePreds.resize(2);
    btb.putPCHistory(startPC, boost::dynamic_bitset<>(64, 0), stagePreds);

    // Should find the branch in second block
    ASSERT_EQ(stagePreds[1].btbEntries.size(), 1);
    EXPECT_EQ(stagePreds[1].btbEntries[0].pc, 0x124);
    EXPECT_EQ(stagePreds[1].btbEntries[0].target, 0x200);

    // Predict from second block
    stagePreds.clear();
    stagePreds.resize(2);
    btb.putPCHistory(0x120, boost::dynamic_bitset<>(64, 0), stagePreds);

    // Should still find the branch
    ASSERT_EQ(stagePreds[1].btbEntries.size(), 1);
    EXPECT_EQ(stagePreds[1].btbEntries[0].pc, 0x124);
    EXPECT_EQ(stagePreds[1].btbEntries[0].target, 0x200);
}

// Test half-aligned mode with branches in both blocks
TEST_F(BTBTest, HalfAlignedBothBlocks) {
    // Create a BTB with half-aligned mode enabled
    DefaultBTB btb(1024, 20, 8, 1, true);

    // Phase 1: Add branch in first block
    std::vector<FullBTBPrediction> stagePreds(2);
    btb.putPCHistory(0x100, boost::dynamic_bitset<>(64, 0), stagePreds);
    auto meta = btb.getPredictionMeta();

    FetchStream stream;
    stream.startPC = 0x100;
    stream.predMetas[0] = meta;
    stream.resolved = true;
    stream.exeBranchInfo.pc = 0x108;  // Branch in first block
    stream.exeBranchInfo.target = 0x200;
    stream.exeBranchInfo.isCond = true;
    stream.exeBranchInfo.size = 4;
    stream.exeTaken = true;
    stream.updateEndInstPC = 0x140;

    btb.getAndSetNewBTBEntry(stream);
    btb.update(stream);

    // Phase 2: Add branch in second block
    stagePreds.clear();
    stagePreds.resize(2);
    btb.putPCHistory(0x100, boost::dynamic_bitset<>(64, 0), stagePreds);
    meta = btb.getPredictionMeta();

    stream.startPC = 0x100;
    stream.predMetas[0] = meta;
    stream.exeBranchInfo.pc = 0x128;  // Branch in second block
    stream.exeBranchInfo.target = 0x300;

    btb.getAndSetNewBTBEntry(stream);
    btb.update(stream);

    // Phase 3: Verify both branches are found
    stagePreds.clear();
    stagePreds.resize(2);
    btb.putPCHistory(0x100, boost::dynamic_bitset<>(64, 0), stagePreds);

    // Should find both branches
    ASSERT_EQ(stagePreds[1].btbEntries.size(), 2);
    EXPECT_EQ(stagePreds[1].btbEntries[0].pc, 0x108);
    EXPECT_EQ(stagePreds[1].btbEntries[0].target, 0x200);
    EXPECT_EQ(stagePreds[1].btbEntries[1].pc, 0x128);
    EXPECT_EQ(stagePreds[1].btbEntries[1].target, 0x300);
}

// Test half-aligned mode with unaligned start address
TEST_F(BTBTest, HalfAlignedUnalignedStart) {
    // Create a BTB with half-aligned mode enabled
    DefaultBTB btb(1024, 20, 8, 1, true);

    // Phase 1: Initial prediction with unaligned start address
    Addr startPC = 0x10A;  // Unaligned address in first block
    std::vector<FullBTBPrediction> stagePreds(2);
    btb.putPCHistory(startPC, boost::dynamic_bitset<>(64, 0), stagePreds);
    auto meta = btb.getPredictionMeta();

    // Phase 2: Setup update stream with branch in second block
    FetchStream stream;
    stream.startPC = startPC;
    stream.predMetas[0] = meta;
    stream.resolved = true;
    stream.exeBranchInfo.pc = 0x12C;  // Branch in second block
    stream.exeBranchInfo.target = 0x200;
    stream.exeBranchInfo.isCond = true;
    stream.exeBranchInfo.size = 4;
    stream.exeTaken = true;
    stream.updateEndInstPC = 0x140;

    // Update BTB
    btb.getAndSetNewBTBEntry(stream);
    btb.update(stream);

    // Phase 3: Verify the update worked correctly
    stagePreds.clear();
    stagePreds.resize(2);
    btb.putPCHistory(startPC, boost::dynamic_bitset<>(64, 0), stagePreds);

    // Should find the branch
    ASSERT_EQ(stagePreds[1].btbEntries.size(), 1);
    EXPECT_EQ(stagePreds[1].btbEntries[0].pc, 0x12C);
    EXPECT_EQ(stagePreds[1].btbEntries[0].target, 0x200);
}

// Test half-aligned mode with multiple updates to same branch
TEST_F(BTBTest, HalfAlignedMultipleUpdates) {
    // Create a BTB with half-aligned mode enabled
    DefaultBTB btb(1024, 20, 8, 1, true);

    // Phase 1: Add branch in second block
    std::vector<FullBTBPrediction> stagePreds(2);
    btb.putPCHistory(0x100, boost::dynamic_bitset<>(64, 0), stagePreds);
    auto meta = btb.getPredictionMeta();

    FetchStream stream;
    stream.startPC = 0x100;
    stream.predMetas[0] = meta;
    stream.resolved = true;
    stream.exeBranchInfo.pc = 0x124;  // Branch in second block
    stream.exeBranchInfo.target = 0x200;
    stream.exeBranchInfo.isIndirect = true; // only indirect branch's target will be updated!
    stream.exeBranchInfo.size = 4;
    stream.exeTaken = true;
    stream.updateEndInstPC = 0x140;

    btb.getAndSetNewBTBEntry(stream);
    btb.update(stream);

    // Phase 2: Update same branch with different target
    stagePreds.clear();
    stagePreds.resize(2);
    btb.putPCHistory(0x100, boost::dynamic_bitset<>(64, 0), stagePreds);
    meta = btb.getPredictionMeta();

    stream.startPC = 0x100;
    stream.predMetas[0] = meta;
    stream.exeBranchInfo.pc = 0x124;  // Same branch
    stream.exeBranchInfo.target = 0x300;  // Different target

    btb.getAndSetNewBTBEntry(stream);
    btb.update(stream);

    // Phase 3: Verify the update changed the target
    stagePreds.clear();
    stagePreds.resize(2);
    btb.putPCHistory(0x100, boost::dynamic_bitset<>(64, 0), stagePreds);

    // Should find the branch with updated target
    ASSERT_EQ(stagePreds[1].btbEntries.size(), 1);
    EXPECT_EQ(stagePreds[1].btbEntries[0].pc, 0x124);
    EXPECT_EQ(stagePreds[1].btbEntries[0].target, 0x300);  // Updated target
}


} // namespace test
} // namespace btb_pred
} // namespace branch_prediction
} // namespace gem5
