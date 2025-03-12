#include <gtest/gtest.h>
#include "cpu/pred/btb/test/btb_tage.hh"
#include "cpu/pred/btb/stream_struct.hh"
#include "base/types.hh"

namespace gem5
{

namespace branch_prediction
{

namespace btb_pred
{

namespace test
{

class BTBTAGETest : public ::testing::Test {
protected:
    void SetUp() override {
        tage = new BTBTAGE();
    }

    void TearDown() override {
        delete tage;
    }

    BTBTAGE* tage;
};

// Test basic prediction functionality
TEST_F(BTBTAGETest, BasicPrediction) {
    // Create a simple branch history
    boost::dynamic_bitset<> history(64, 0);  // 64-bit history initialized to 0
    
    // Create some BTB entries for testing
    std::vector<BTBEntry> btbEntries;
    
    // Add a conditional branch entry
    BTBEntry entry1;
    entry1.pc = 0x1000;
    entry1.isCond = true;
    entry1.valid = true;
    entry1.alwaysTaken = false;
    entry1.ctr = 1;  // Initially biased towards taken
    btbEntries.push_back(entry1);
    
    // Create stage predictions container
    std::vector<FullBTBPrediction> stagePreds(2);  // 2 stages
    stagePreds[1].btbEntries = btbEntries;
    
    // Test prediction
    tage->putPCHistory(0x1000, history, stagePreds);
    
    // Verify prediction
    ASSERT_TRUE(stagePreds[1].condTakens.find(0x1000) != stagePreds[1].condTakens.end());
    // Initially should predict taken due to counter bias, use btb entry.ctr to get the prediction
    EXPECT_TRUE(stagePreds[1].condTakens[0x1000]);
}

// Test basic history update functionality
TEST_F(BTBTAGETest, BasicHistoryUpdate) {
    // Create initial history (64-bit all zeros)
    boost::dynamic_bitset<> history(64, 0);
    
    // Test case 1: Update with taken branch
    // shamt = 1, taken = true
    history <<= 1;  // Shift left by 1
    history[0] = true;  // Set lowest bit to 1 for taken branch
    tage->doUpdateHist(history, 1, true);
    
    // Verify folded history state after update
    // Check each predictor's folded history
    for (int t = 0; t < tage->numPredictors; t++) {
        // Get original folded histories
        auto &tagHist = tage->tagFoldedHist[t];
        auto &altTagHist = tage->altTagFoldedHist[t];
        auto &indexHist = tage->indexFoldedHist[t];
        
        // Each folded history should reflect the taken branch
        tagHist.check(history);
        altTagHist.check(history);
        indexHist.check(history);
    }
    
    // Test case 2: Update with not taken branch
    // shamt = 1, taken = false
    history <<= 1;  // Shift left by 1
    history[0] = false;  // Set lowest bit to 0 for not taken branch
    tage->doUpdateHist(history, 1, false);
    
    // Verify folded history state again
    for (int t = 0; t < tage->numPredictors; t++) {
        auto &tagHist = tage->tagFoldedHist[t];
        auto &altTagHist = tage->altTagFoldedHist[t];
        auto &indexHist = tage->indexFoldedHist[t];
        
        // Each folded history should be consistent with history
        tagHist.check(history);
        altTagHist.check(history);
        indexHist.check(history);
    }
}

// Test table lookup functionality
TEST_F(BTBTAGETest, TestTableLookup) {
    // Create a simple branch history
    boost::dynamic_bitset<> history(64, 0);  // 64-bit history initialized to 0
    
    // Create test BTB entries
    std::vector<BTBEntry> btbEntries;
    
    // Create multiple conditional branch entries at different addresses
    BTBEntry entry1, entry2, entry3;
    
    // First entry: PC = 0x1000
    entry1.pc = 0x1000;
    entry1.isCond = true;
    entry1.valid = true;
    entry1.alwaysTaken = false;
    entry1.ctr = 1;
    btbEntries.push_back(entry1);
    
    // Second entry: PC = 0x2000
    entry2.pc = 0x2000;
    entry2.isCond = true;
    entry2.valid = true;
    entry2.alwaysTaken = false;
    entry2.ctr = -1;
    btbEntries.push_back(entry2);
    
    // Third entry: PC = 0x3000
    entry3.pc = 0x3000;
    entry3.isCond = true;
    entry3.valid = true;
    entry3.alwaysTaken = false;
    entry3.ctr = 0;
    btbEntries.push_back(entry3);
    
    // Create stage predictions container
    std::vector<FullBTBPrediction> stagePreds(2);  // 2 stages
    stagePreds[1].btbEntries = btbEntries;
    
    // First lookup: should create new entries in TAGE tables
    tage->putPCHistory(0x1000, history, stagePreds);
    
    // Verify predictions exist for all entries
    ASSERT_TRUE(stagePreds[1].condTakens.find(0x1000) != stagePreds[1].condTakens.end());
    ASSERT_TRUE(stagePreds[1].condTakens.find(0x2000) != stagePreds[1].condTakens.end());
    ASSERT_TRUE(stagePreds[1].condTakens.find(0x3000) != stagePreds[1].condTakens.end());
    
    // Update history to create some pattern
    history <<= 1;
    history[0] = true;  // Set lowest bit to 1
    
    // Second lookup: should use existing entries
    tage->putPCHistory(0x1000, history, stagePreds);
    
    // Verify predictions still exist
    ASSERT_TRUE(stagePreds[1].condTakens.find(0x1000) != stagePreds[1].condTakens.end());
    ASSERT_TRUE(stagePreds[1].condTakens.find(0x2000) != stagePreds[1].condTakens.end());
    ASSERT_TRUE(stagePreds[1].condTakens.find(0x3000) != stagePreds[1].condTakens.end());
    
    // Create a stream to update the predictor
    FetchStream stream;
    stream.startPC = 0x1000;
    stream.exeBranchInfo = entry1;
    stream.exeTaken = true;
    // Simulate a misprediction by setting squash type to SQUASH_CTRL
    stream.squashType = SquashType::SQUASH_CTRL;
    stream.squashPC = 0x1000;  // The PC that caused the squash
    stream.updateBTBEntries = btbEntries;
    stream.updateIsOldEntry = true;
    stream.predMetas[0] = tage->getPredictionMeta();
    
    // Update predictor with actual outcomes
    tage->update(stream);
    
    // Third lookup: should reflect updated predictions
    tage->putPCHistory(0x1000, history, stagePreds);
    
    // Verify predictions are updated
    ASSERT_TRUE(stagePreds[1].condTakens.find(0x1000) != stagePreds[1].condTakens.end());
    EXPECT_TRUE(stagePreds[1].condTakens[0x1000]);  // Should predict taken after update
    
    // Verify that at least one TAGE table has an entry allocated
    bool found_valid_entry = false;
    for (int t = 0; t < tage->numPredictors; t++) {
        Addr index = tage->getTageIndex(0x1000, t);
        auto &entry = tage->tageTable[t][index];
        if (entry.valid && entry.pc == 0x1000) {
            found_valid_entry = true;
            break;
        }
    }
    EXPECT_TRUE(found_valid_entry) << "No valid TAGE table entry was allocated";
}

// Test main and alternative prediction mechanism
TEST_F(BTBTAGETest, TestMainAltPrediction) {
    // Create initial history and BTB entry
    boost::dynamic_bitset<> history(64, 0);
    std::vector<BTBEntry> btbEntries;
    BTBEntry entry;
    entry.pc = 0x1000;
    entry.isCond = true;
    entry.valid = true;
    entry.alwaysTaken = false;
    entry.ctr = 0;
    btbEntries.push_back(entry);

    // Setup prediction stages
    std::vector<FullBTBPrediction> stagePreds(2);
    stagePreds[1].btbEntries = btbEntries;

    // Training phase: Update multiple times with different histories
    for (int i = 0; i < 4; i++) {  // at least train 4 times, corresponding to 4 predictors
        // Make prediction
        tage->putPCHistory(0x1000, history, stagePreds);
        
        // Create update stream
        FetchStream stream;
        stream.startPC = 0x1000;
        stream.exeBranchInfo = entry;
        stream.exeTaken = (i % 2 == 0);  // alternate between taken/not taken
        stream.squashType = SquashType::SQUASH_CTRL;
        stream.squashPC = 0x1000;
        stream.updateBTBEntries = btbEntries;
        stream.updateIsOldEntry = true;
        stream.predMetas[0] = tage->getPredictionMeta();

        // Update predictor
        // The update process follows this pattern for each iteration:
        // 1st update:
        //   - Initial state: All tables are invalid
        //   - Prediction: taken=1 (default prediction when no valid entries)
        //   - Result: Misprediction -> allocate in table 0 with counter=0
        //
        // 2nd update:
        //   - Initial state: Table 0 has valid entry
        //   - Prediction: taken=1 (using table 0's prediction)
        //   - Result: Misprediction -> allocate in table 1 with counter=-1
        //
        // 3rd update:
        //   - Initial state: Tables 0,1 have valid entries
        //   - Prediction: taken=0 (using table 1's prediction as highest table)
        //   - Result: Misprediction -> allocate in table 2 with counter=0
        //
        // 4th update:
        //   - Initial state: Tables 0,1,2 have valid entries
        //   - Prediction: taken=1 (using table 2's prediction as highest table)
        //   - Result: Misprediction -> allocate in table 3 with counter=-1
        //
        // Final state after all updates:
        // - All tables have valid entries with different counter values
        // - This creates a scenario where we have both main and alternative predictions
        // - The predictions come from different tables, allowing us to test the selection mechanism
        tage->update(stream);
        
        // Update history
        history <<= 1;
        history[0] = stream.exeTaken;
    }

    // Final prediction to check tables
    tage->putPCHistory(0x1000, history, stagePreds);
    
    // Now verify predictions
    auto meta = std::static_pointer_cast<BTBTAGE::TageMeta>(tage->getPredictionMeta());
    auto pred = meta->preds[0x1000];
    
    // Verify we have both main and alternative predictions
    ASSERT_TRUE(pred.mainInfo.found) << "Main prediction table not found";
    ASSERT_TRUE(pred.altInfo.found) << "Alternative prediction table not found";
    
    // Verify they are from different tables
    ASSERT_NE(pred.mainInfo.table, pred.altInfo.table) 
        << "Main and alternative predictions should come from different tables";
}

// Test useful bit mechanism
TEST_F(BTBTAGETest, TestUsefulBitMechanism) {
    // Create initial history and BTB entry
    boost::dynamic_bitset<> history(64, 0);
    std::vector<BTBEntry> btbEntries;
    BTBEntry entry;
    entry.pc = 0x1000;
    entry.isCond = true;
    entry.valid = true;
    entry.alwaysTaken = false;
    entry.ctr = 0;  // Neutral prediction in BTB
    btbEntries.push_back(entry);

    // Setup prediction stages
    std::vector<FullBTBPrediction> stagePreds(2);
    stagePreds[1].btbEntries = btbEntries;

    // Phase 1: Training phase - Allocate entries in multiple tables
    // We need at least 2 tables with valid entries to test useful bit mechanism
    for (int i = 0; i < 2; i++) {
        // Make prediction to generate meta data
        tage->putPCHistory(0x1000, history, stagePreds);
        
        // Create update stream
        FetchStream stream;
        stream.startPC = 0x1000;
        stream.exeBranchInfo = entry;
        stream.exeTaken = (i % 2 == 0);  // alternate between taken/not taken
        stream.squashType = SquashType::SQUASH_CTRL;
        stream.squashPC = 0x1000;
        stream.updateBTBEntries = btbEntries;
        stream.updateIsOldEntry = true;
        stream.predMetas[0] = tage->getPredictionMeta();
        
        // Update predictor to allocate entries
        // First update: allocates entry in table 0
        // Second update: allocates entry in table 1
        tage->update(stream);
        
        // Update history
        history <<= 1;
        history[0] = stream.exeTaken;
    }

    // Phase 2: Find allocated entries and train them with different predictions
    int mainTable = -1, altTable = -1;
    for (int t = tage->numPredictors - 1; t >= 0; t--) {
        Addr index = tage->getTageIndex(0x1000, t);
        auto &entry = tage->tageTable[t][index];
        if (entry.valid && entry.pc == 0x1000) {
            if (mainTable == -1) {
                mainTable = t;
                // Train main predictor to strongly predict taken
                entry.counter = 1;  // Strong taken
            } else if (altTable == -1) {
                altTable = t;
                // Train alt predictor to strongly predict not taken
                entry.counter = -1;  // Strong not taken
                break;
            }
        }
    }

    // Verify we found both tables
    ASSERT_NE(mainTable, -1) << "Main prediction table not found";
    ASSERT_NE(altTable, -1) << "Alternative prediction table not found";

    // Get indices for both entries
    Addr mainIndex = tage->getTageIndex(0x1000, mainTable);
    Addr altIndex = tage->getTageIndex(0x1000, altTable);

    // Phase 3: Test useful bit mechanism
    // Initially both entries should have useful bit = false
    EXPECT_FALSE(tage->tageTable[mainTable][mainIndex].useful) 
        << "Main entry's useful bit should be initially false";
    EXPECT_FALSE(tage->tageTable[altTable][altIndex].useful) 
        << "Alt entry's useful bit should be initially false";

    // Make prediction to generate new meta data
    tage->putPCHistory(0x1000, history, stagePreds);

    // Phase 4: Update with actual outcome matching main predictor
    // Create a stream where main predictor is correct (taken) and alt is wrong (not taken)
    FetchStream stream;
    stream.startPC = 0x1000;
    stream.exeBranchInfo = entry;
    stream.exeTaken = true;  // Actual outcome is taken, matching main predictor
    stream.squashType = SquashType::SQUASH_CTRL;
    stream.squashPC = 0x1000;
    stream.updateBTBEntries = btbEntries;
    stream.updateIsOldEntry = true;
    stream.predMetas[0] = tage->getPredictionMeta();

    // Update predictor - this should set the useful bit for main predictor
    tage->update(stream);

    // Phase 5: Verify useful bit updates
    // Main predictor's useful bit should be set since it predicted correctly
    EXPECT_TRUE(tage->tageTable[mainTable][mainIndex].useful) 
        << "Main entry's useful bit should be set after correct prediction";
    
    // Alt predictor's useful bit should remain false
    EXPECT_FALSE(tage->tageTable[altTable][altIndex].useful) 
        << "Alt entry's useful bit should remain false after wrong prediction";
}

// Test recovery with useful bit state preservation
TEST_F(BTBTAGETest, UsefulBitRecovery) {
    // Create initial history
    boost::dynamic_bitset<> history(64, 0);
    std::vector<BTBEntry> btbEntries;
    BTBEntry entry;
    entry.pc = 0x1000;
    entry.isCond = true;
    entry.valid = true;
    entry.alwaysTaken = false;
    entry.ctr = 0;
    btbEntries.push_back(entry);

    // Setup prediction stages
    std::vector<FullBTBPrediction> stagePreds(2);
    stagePreds[1].btbEntries = btbEntries;

    // Train the predictor to set useful bits
    for (int i = 0; i < 10; i++) {
        tage->putPCHistory(0x1000, history, stagePreds);
        auto meta = std::static_pointer_cast<BTBTAGE::TageMeta>(tage->getPredictionMeta());
        
        FetchStream stream;
        stream.startPC = 0x1000;
        stream.exeBranchInfo = entry;
        stream.exeTaken = (i % 2 == 0);  // Alternate between taken/not taken
        stream.squashType = SquashType::SQUASH_CTRL;
        stream.squashPC = 0x1000;
        stream.updateBTBEntries = btbEntries;
        stream.updateIsOldEntry = true;
        stream.predMetas[0] = meta;

        tage->update(stream);
        
        // Update history
        history <<= 1;
        history[0] = stream.exeTaken;
    }

    // Save useful bit state before recovery
    auto meta_before = std::static_pointer_cast<BTBTAGE::TageMeta>(tage->getPredictionMeta());
    auto useful_mask_before = meta_before->usefulMask;

    // Create misprediction and recover
    FetchStream stream;
    stream.startPC = 0x1000;
    stream.exeBranchInfo = entry;
    stream.exeTaken = true;
    stream.squashType = SquashType::SQUASH_CTRL;
    stream.squashPC = 0x1000;
    stream.updateBTBEntries = btbEntries;
    stream.updateIsOldEntry = true;
    stream.predMetas[0] = meta_before;

    tage->recoverHist(history, stream, 1, true);

    // Verify useful bits are preserved after recovery
    tage->putPCHistory(0x1000, history, stagePreds);
    auto meta_after = std::static_pointer_cast<BTBTAGE::TageMeta>(tage->getPredictionMeta());
    EXPECT_EQ(useful_mask_before, meta_after->usefulMask);
}

// Test basic recovery mechanism after misprediction
TEST_F(BTBTAGETest, BasicRecoveryMechanism) {
    // Create initial history and BTB entry
    boost::dynamic_bitset<> history(64, 0);
    std::vector<BTBEntry> btbEntries;
    BTBEntry entry;
    entry.pc = 0x1000;
    entry.isCond = true;
    entry.valid = true;
    entry.alwaysTaken = false;
    entry.ctr = 1;      // pred taken
    btbEntries.push_back(entry);

    // Setup prediction stages
    std::vector<FullBTBPrediction> stagePreds(2);
    stagePreds[1].btbEntries = btbEntries;

    // Phase 1: Initial prediction
    tage->putPCHistory(0x1000, history, stagePreds);
    // assert taken prediction
    ASSERT_TRUE(stagePreds[1].condTakens[0x1000]);
    tage->specUpdateHist(history, stagePreds[1]);
    auto meta1 = std::static_pointer_cast<BTBTAGE::TageMeta>(tage->getPredictionMeta());

    // Phase 2: Update history for misprediction
    history <<= 1;
    history[0] = true;  // Branch was predicted taken
    tage->checkFoldedHist(history, "speculative update");

    // Phase 3: Create update stream with misprediction
    FetchStream stream;
    stream.startPC = 0x1000;
    stream.exeBranchInfo = entry;
    stream.exeTaken = false;  // Actual NT
    stream.squashType = SquashType::SQUASH_CTRL;
    stream.squashPC = 0x1000;
    stream.updateBTBEntries = btbEntries;
    stream.updateIsOldEntry = true;
    stream.predMetas[0] = meta1;

    // Phase 4: Recover from misprediction
    tage->recoverHist(history, stream, 1, false);
    history >>= 1;  // shift right to get the correct history
    history <<= 1;
    history[0] = false;  // Branch was predicted NT
    tage->checkFoldedHist(history, "recover");

}

// Test consecutive correct predictions
TEST_F(BTBTAGETest, ConsecutiveCorrectPredictions) {
    boost::dynamic_bitset<> history(64, 0);
    std::vector<BTBEntry> btbEntries;
    BTBEntry entry;
    entry.pc = 0x1000;
    entry.isCond = true;
    entry.valid = true;
    entry.alwaysTaken = false;
    entry.ctr = 1;  // pred taken
    btbEntries.push_back(entry);

    std::vector<FullBTBPrediction> stagePreds(2);
    stagePreds[1].btbEntries = btbEntries;

    // Test 5 consecutive correct predictions
    for (int i = 0; i < 5; i++) {
        // Phase 1: Make prediction
        tage->putPCHistory(0x1000, history, stagePreds);
        ASSERT_TRUE(stagePreds[1].condTakens[0x1000]);
        tage->specUpdateHist(history, stagePreds[1]);
        auto meta = tage->getPredictionMeta();

        // Phase 2: Update speculative history
        history <<= 1;
        history[0] = true;  // predicted taken
        tage->checkFoldedHist(history, "speculative update");

        // Phase 3: Update with correct prediction
        FetchStream stream;
        stream.startPC = 0x1000;
        stream.exeBranchInfo = entry;
        stream.exeTaken = true;  // actual taken
        stream.squashType = SquashType::SQUASH_CTRL;
        stream.squashPC = 0x1000;
        stream.updateBTBEntries = btbEntries;
        stream.updateIsOldEntry = true;
        stream.predMetas[0] = meta;

        // Update predictor
        tage->update(stream);
    }
}

// Test alternating predictions with recovery
TEST_F(BTBTAGETest, AlternatingPredictions) {
    boost::dynamic_bitset<> history(64, 0);
    std::vector<BTBEntry> btbEntries;
    BTBEntry entry;
    entry.pc = 0x1000;
    entry.isCond = true;
    entry.valid = true;
    entry.alwaysTaken = false;
    entry.ctr = 0;  // neutral prediction
    btbEntries.push_back(entry);

    std::vector<FullBTBPrediction> stagePreds(2);
    stagePreds[1].btbEntries = btbEntries;

    bool expected_taken = true;
    // Test alternating taken/not-taken pattern
    for (int i = 0; i < 6; i++) {
        // Phase 1: Make prediction
        tage->putPCHistory(0x1000, history, stagePreds);
        bool predicted_taken = stagePreds[1].condTakens[0x1000];
        tage->specUpdateHist(history, stagePreds[1]);
        auto meta = tage->getPredictionMeta();

        // Phase 2: Update speculative history
        history <<= 1;
        history[0] = predicted_taken;
        tage->checkFoldedHist(history, "speculative update");

        // Phase 3: Handle prediction result
        FetchStream stream;
        stream.startPC = 0x1000;
        stream.exeBranchInfo = entry;
        stream.exeTaken = expected_taken;
        stream.squashType = SquashType::SQUASH_CTRL;
        stream.squashPC = 0x1000;
        stream.updateBTBEntries = btbEntries;
        stream.updateIsOldEntry = true;
        stream.predMetas[0] = meta;

        if (predicted_taken != expected_taken) {
            // Recover from misprediction
            tage->recoverHist(history, stream, 1, expected_taken);
            history >>= 1;  // shift right to get the correct history
            history <<= 1;
            history[0] = expected_taken;
            tage->checkFoldedHist(history, "recover");

            // Re-predict after recovery
            tage->putPCHistory(0x1000, history, stagePreds);
            auto meta_recovered = tage->getPredictionMeta();
            stream.predMetas[0] = meta_recovered;
        }

        // Update predictor
        tage->update(stream);
        
        // Toggle expected outcome for next iteration
        expected_taken = !expected_taken;
    }
}

// Test multiple branch sequence with recovery
TEST_F(BTBTAGETest, MultipleBranchSequence) {
    boost::dynamic_bitset<> history(64, 0);
    std::vector<BTBEntry> btbEntries;
    
    // Create three branches with different initial predictions
    for (int i = 0; i < 3; i++) {
        BTBEntry entry;
        entry.pc = 0x1000 + i * 4;
        entry.isCond = true;
        entry.valid = true;
        entry.alwaysTaken = false;
        entry.ctr = i - 1;  // -1, 0, 1 for NT, neutral, T
        btbEntries.push_back(entry);
    }

    std::vector<FullBTBPrediction> stagePreds(2);
    stagePreds[1].btbEntries = btbEntries;

    // define test groups
    struct BranchGroupTest {
        std::vector<bool> predicted_takens;
        std::vector<bool> actual_takens;
        bool expected_hist_taken;  // expected history taken bit
        int expected_shamt;        // expected shift amount
        bool actual_hist_taken;    // actual history taken bit
        int actual_shamt;          // actual shift amount
    };

    std::vector<BranchGroupTest> test_groups = {
        {{false, false, false}, {false, false, false}, false, 3, false, 3},  // all NT
        {{false, true, true}, {false, true, true}, true, 2, true, 2},      // NT,T,T -> update T
        {{true, false, true}, {true, false, true}, true, 1, true, 1},      // T,NT,T -> update T
        {{false, false, true}, {false, false, true}, true, 3, true, 3},    // NT,NT,T -> update T
        {{true, true, true}, {false, false, false}, true, 1, false, 3}     // all T, actual all NT
    };

    for (const auto& group : test_groups) {
        // Phase 1: Setup predictions
        stagePreds[1].btbEntries = btbEntries;


        // Phase 2: Make prediction and update history
        tage->putPCHistory(0x1000, history, stagePreds);
        // change default predictions to test groups
        for (size_t i = 0; i < group.predicted_takens.size(); i++) {
            stagePreds[1].condTakens[0x1000 + i * 4] = group.predicted_takens[i];
        }
        tage->specUpdateHist(history, stagePreds[1]);
        auto meta = tage->getPredictionMeta();

        // verify history update
        int shamt;
        bool hist_taken;
        std::tie(shamt, hist_taken) = stagePreds[1].getHistInfo();
        EXPECT_EQ(shamt, group.expected_shamt) 
            << "Shift amount mismatch";
        EXPECT_EQ(hist_taken, group.expected_hist_taken) 
            << "History taken bit mismatch";

        // Phase 3: Update speculative history
        history <<= shamt;
        history[0] = hist_taken;
        tage->checkFoldedHist(history, "speculative update");

        // Phase 4: Handle predictions
        for (size_t i = 0; i < group.actual_takens.size(); i++) {
            Addr current_pc = 0x1000 + i * 4;
            bool predicted_taken = group.predicted_takens[i];
            bool actual_taken = group.actual_takens[i];

            FetchStream stream;
            stream.startPC = current_pc;
            stream.exeBranchInfo = btbEntries[i];
            stream.exeTaken = actual_taken;
            stream.squashType = SquashType::SQUASH_CTRL;
            stream.squashPC = current_pc;
            stream.updateBTBEntries = btbEntries;
            stream.updateIsOldEntry = true;
            stream.predMetas[0] = meta;

            if (predicted_taken != actual_taken) {
                // Handle misprediction
                tage->recoverHist(history, stream, group.actual_shamt, group.actual_hist_taken);
                history >>= group.actual_shamt;
                history <<= group.actual_shamt;
                history[0] = group.actual_hist_taken;
                tage->checkFoldedHist(history, "recover");

                // Re-predict after recovery
                tage->putPCHistory(current_pc, history, stagePreds);
                auto meta_recovered = tage->getPredictionMeta();
                stream.predMetas[0] = meta_recovered;
            }

            // Update predictor
            tage->update(stream);
        }
    }
}

}  // namespace test

}  // namespace btb_pred

}  // namespace branch_prediction

}  // namespace gem5