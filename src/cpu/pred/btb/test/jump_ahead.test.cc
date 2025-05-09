#include <gtest/gtest.h>

#include <algorithm>
#include <map>
#include <memory>
#include <vector>

#include "cpu/pred/btb/jump_ahead_predictor.hh"
#include "cpu/pred/btb/stream_struct.hh"
#include "cpu/pred/btb/test/test_dprintf.hh"

namespace gem5
{
namespace branch_prediction
{
namespace btb_pred
{

// Test fixture for Jump Ahead Predictor
class JumpAheadPredictorTest : public ::testing::Test
{
protected:
    std::unique_ptr<JumpAheadPredictor> jap;
    const unsigned defaultSets = 16;
    const unsigned defaultWays = 4;
    const unsigned blockSize = 32;

    void SetUp() override {
        jap = std::make_unique<JumpAheadPredictor>(defaultSets, defaultWays);
        jap->blockSize = blockSize;
    }

    void TearDown() override {
        jap.reset();
    }

    // Helper method to create a JAInfo object with consecutive unpredicted blocks
    JumpAheadPredictor::JAInfo createJAInfo(int noPredBlocks, Addr firstBlockPC) {
        JumpAheadPredictor::JAInfo info;
        info.firstNoPredBlockStart = firstBlockPC;
        info.noPredBlockCount = noPredBlocks;
        return info;
    }

    // Helper method to create predicted block info
    void addPredictedBlock(JumpAheadPredictor::JAInfo& info, Addr predictedBlockPC) {
        BTBEntry entry;
        entry.valid = true;
        entry.target = predictedBlockPC + blockSize; // Target is end of block
        info.setPredictedBlock(predictedBlockPC, entry);
    }

    // Helper method to simulate a sequence of unpredicted blocks
    void simulateUnpredictedBlocks(JumpAheadPredictor::JAInfo& info, int count, Addr startPC) {
        for (int i = 0; i < count; i++) {
            info.incrementNoPredBlockCount(startPC + i * blockSize);
        }
    }
};

// Test basic initialization
TEST_F(JumpAheadPredictorTest, BasicInitialization) {
    EXPECT_EQ(jap->numSets, defaultSets);
    EXPECT_EQ(jap->numWays, defaultWays);
    EXPECT_EQ(jap->maxConf, 7);
    EXPECT_EQ(jap->minNoPredBlockNum, 2);
    EXPECT_EQ(jap->blockSize, blockSize);
}

// Test index and tag extraction
TEST_F(JumpAheadPredictorTest, IndexAndTagExtraction) {
    Addr pc = 0x10000;
    int index = jap->getIndex(pc);
    Addr tag = jap->getTag(pc);

    // Index is extracted from bits 1-4 (for 16 sets)
    EXPECT_EQ(index, (pc >> 1) & ((1 << 4) - 1));

    // Tag is high-order bits beyond the index
    EXPECT_EQ(tag, (pc >> (1 + 4)) & jap->tagMask);
}

// Test lookup with no entries
TEST_F(JumpAheadPredictorTest, LookupEmpty) {
    Addr pc = 0x10000;
    bool hit, confMet;
    JAEntry entry;
    Addr target;

    std::tie(hit, confMet, entry, target) = jap->lookup(pc);

    EXPECT_FALSE(hit);
    EXPECT_FALSE(confMet);
    EXPECT_EQ(target, 0);
}

// Test update and lookup with matching pattern
TEST_F(JumpAheadPredictorTest, UpdateAndLookupMatch) {
    Addr firstBlockPC = 0x10000;
    Addr nextPredBlockPC = firstBlockPC + 3 * blockSize; // 3 blocks ahead

    // Create a pattern with 3 non-predicted blocks
    auto info = createJAInfo(3, firstBlockPC);

    // Update JAP with this pattern
    jap->tryUpdate(info, nextPredBlockPC);

    // Lookup should find the entry but confidence is not high enough yet
    bool hit, confMet;
    JAEntry entry;
    Addr target;
    std::tie(hit, confMet, entry, target) = jap->lookup(firstBlockPC);

    EXPECT_TRUE(hit);
    EXPECT_FALSE(confMet); // Confidence starts at 0
    EXPECT_EQ(entry.jumpAheadBlockNum, 3);

    // Simulate seeing the same pattern multiple times to increase confidence
    for (int i = 0; i < jap->maxConf + 1; i++) {
        jap->tryUpdate(info, nextPredBlockPC);
    }

    // Now lookup should indicate high confidence
    std::tie(hit, confMet, entry, target) = jap->lookup(firstBlockPC);

    EXPECT_TRUE(hit);
    EXPECT_TRUE(confMet);
    EXPECT_EQ(entry.jumpAheadBlockNum, 3);
    EXPECT_EQ(target, firstBlockPC + 3 * blockSize); // Should jump ahead 3 blocks
}

// Test update with changing pattern
TEST_F(JumpAheadPredictorTest, UpdateChangingPattern) {
    Addr firstBlockPC = 0x10000;
    Addr nextPredBlockPC = firstBlockPC + 3 * blockSize;

    // First create pattern with 3 blocks
    auto info = createJAInfo(3, firstBlockPC);

    // Update multiple times to increase confidence
    for (int i = 0; i < jap->maxConf + 1; i++) {
        jap->tryUpdate(info, nextPredBlockPC);
    }

    // Verify high confidence
    bool hit, confMet;
    JAEntry entry;
    Addr target;
    std::tie(hit, confMet, entry, target) = jap->lookup(firstBlockPC);
    EXPECT_TRUE(confMet);

    // Now change pattern to 2 blocks
    info.noPredBlockCount = 2;
    jap->tryUpdate(info, firstBlockPC + 2 * blockSize);

    // Confidence should be reduced
    std::tie(hit, confMet, entry, target) = jap->lookup(firstBlockPC);
    EXPECT_TRUE(hit);
    EXPECT_FALSE(confMet); // Confidence decreases when pattern changes
    EXPECT_EQ(entry.jumpAheadBlockNum, 2); // Block count updated
}

// Test invalidation
TEST_F(JumpAheadPredictorTest, Invalidation) {
    Addr firstBlockPC = 0x10000;
    Addr nextPredBlockPC = firstBlockPC + 3 * blockSize;

    // Create a high-confidence pattern
    auto info = createJAInfo(3, firstBlockPC);
    for (int i = 0; i < jap->maxConf + 1; i++) {
        jap->tryUpdate(info, nextPredBlockPC);
    }

    // Verify high confidence
    bool hit, confMet;
    JAEntry entry;
    Addr target;
    std::tie(hit, confMet, entry, target) = jap->lookup(firstBlockPC);
    EXPECT_TRUE(confMet);

    // Invalidate the entry
    jap->invalidate(firstBlockPC);

    // Confidence should be reset
    std::tie(hit, confMet, entry, target) = jap->lookup(firstBlockPC);
    EXPECT_TRUE(hit);
    EXPECT_FALSE(confMet);
    EXPECT_EQ(entry.conf, 0);
}

// Test JAInfo functionality
TEST_F(JumpAheadPredictorTest, JAInfoOperations) {
    JumpAheadPredictor::JAInfo info;

    // Initially empty
    EXPECT_EQ(info.noPredBlockCount, 0);
    EXPECT_EQ(info.firstNoPredBlockStart, 0);

    // Add unpredicted blocks
    Addr firstBlockPC = 0x10000;
    info.incrementNoPredBlockCount(firstBlockPC);
    EXPECT_EQ(info.noPredBlockCount, 1);
    EXPECT_EQ(info.firstNoPredBlockStart, firstBlockPC);

    // Add more unpredicted blocks
    info.incrementNoPredBlockCount(firstBlockPC + blockSize);
    info.incrementNoPredBlockCount(firstBlockPC + 2 * blockSize);
    EXPECT_EQ(info.noPredBlockCount, 3);
    EXPECT_EQ(info.firstNoPredBlockStart, firstBlockPC); // Still the first block

    // Set predicted block
    BTBEntry entry;
    entry.valid = true;
    entry.target = firstBlockPC + 3 * blockSize;
    Addr predictedPC = firstBlockPC + 3 * blockSize;
    info.setPredictedBlock(predictedPC, entry);

    // noPredBlockCount should be reset
    EXPECT_EQ(info.noPredBlockCount, 0);
    EXPECT_EQ(info.recentPredictedBlockStart, predictedPC);
    EXPECT_EQ(info.recentPredictedBTBEntry.target, entry.target);
}

// Test multiple entries and set conflict
TEST_F(JumpAheadPredictorTest, MultipleEntries) {
    // Create two PCs that map to same set but different tags
    Addr pc1 = 0x10000;
    // Find a PC with same index but different tag
    int index1 = jap->getIndex(pc1);
    Addr tag1 = jap->getTag(pc1);

    // Find pc2 with same index but different tag
    Addr pc2 = pc1;
    Addr tag2;
    do {
        pc2 += 0x10000; // Increment by large amount to change tag
        tag2 = jap->getTag(pc2);
    } while (jap->getIndex(pc2) != index1 || tag2 == tag1);

    // Create patterns for both PCs
    auto info1 = createJAInfo(2, pc1);
    auto info2 = createJAInfo(3, pc2);

    // Update both patterns multiple times
    for (int i = 0; i < jap->maxConf + 1; i++) {
        jap->tryUpdate(info1, pc1 + 2 * blockSize);
        jap->tryUpdate(info2, pc2 + 3 * blockSize);
    }

    // Both should be retrievable independently
    bool hit1, conf1, hit2, conf2;
    JAEntry entry1, entry2;
    Addr target1, target2;

    std::tie(hit1, conf1, entry1, target1) = jap->lookup(pc1);
    std::tie(hit2, conf2, entry2, target2) = jap->lookup(pc2);

    EXPECT_TRUE(hit1);
    EXPECT_TRUE(conf1);
    EXPECT_EQ(entry1.jumpAheadBlockNum, 2);

    EXPECT_TRUE(hit2);
    EXPECT_TRUE(conf2);
    EXPECT_EQ(entry2.jumpAheadBlockNum, 3);
}

// Test patterns below minimum threshold
TEST_F(JumpAheadPredictorTest, BelowThreshold) {
    Addr pc = 0x10000;

    // Create pattern with only 1 non-predicted block (below threshold)
    auto info = createJAInfo(1, pc);
    jap->tryUpdate(info, pc + blockSize);

    // Lookup should not find an entry (pattern not stored)
    bool hit, confMet;
    JAEntry entry;
    Addr target;
    std::tie(hit, confMet, entry, target) = jap->lookup(pc);

    EXPECT_FALSE(hit);
    EXPECT_FALSE(confMet);
}

}  // namespace btb_pred
}  // namespace branch_prediction
}  // namespace gem5

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
