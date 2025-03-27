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

// Helper functions for TAGE testing

/**
 * @brief Create a BTB entry with specified parameters
 *
 * @param pc Branch instruction address
 * @param isCond Whether the branch is conditional
 * @param valid Whether the entry is valid
 * @param alwaysTaken Whether the branch is always taken
 * @param ctr Prediction counter value
 * @return BTBEntry Initialized branch entry
 */
BTBEntry createBTBEntry(Addr pc, bool isCond = true, bool valid = true,
                        bool alwaysTaken = false, int ctr = 0) {
    BTBEntry entry;
    entry.pc = pc;
    entry.isCond = isCond;
    entry.valid = valid;
    entry.alwaysTaken = alwaysTaken;
    entry.ctr = ctr;
    // Other fields are set to default
    return entry;
}

/**
 * @brief Create a stream for update or recovery
 *
 * @param startPC Starting PC for the stream
 * @param entry Branch entry information
 * @param taken Actual outcome (taken/not taken)
 * @param meta Prediction metadata from prediction phase
 * @param squashType Type of squash (control or non-control)
 * @return FetchStream Initialized stream for update or recovery
 */
FetchStream createStream(Addr startPC, const BTBEntry& entry, bool taken,
                         std::shared_ptr<void> meta,
                         SquashType squashType = SquashType::SQUASH_CTRL) {
    FetchStream stream;
    stream.startPC = startPC;
    stream.exeBranchInfo = entry;
    stream.exeTaken = taken;
    stream.squashType = squashType;
    stream.squashPC = entry.pc;
    stream.updateBTBEntries = {entry};
    stream.updateIsOldEntry = true;
    stream.predMetas[0] = meta;
    return stream;
}

/**
 * @brief Execute a complete TAGE prediction cycle
 *
 * @param tage The TAGE predictor
 * @param startPC Starting PC for prediction
 * @param entries Vector of BTB entries
 * @param history Branch history register
 * @param stagePreds Prediction results container
 * @return bool Prediction result (taken/not taken) for the first entry
 */
bool predictTAGE(BTBTAGE* tage, Addr startPC,
                const std::vector<BTBEntry>& entries,
                boost::dynamic_bitset<>& history,
                std::vector<FullBTBPrediction>& stagePreds) {
    // Setup stage predictions with BTB entries
    stagePreds[1].btbEntries = entries;

    // Make prediction
    tage->putPCHistory(startPC, history, stagePreds);

    // Return prediction for first entry if exists
    if (!entries.empty() && stagePreds[1].condTakens.find(entries[0].pc) != stagePreds[1].condTakens.end()) {
        return stagePreds[1].condTakens[entries[0].pc];
    }
    return false;
}

/**
 * @brief Execute a complete prediction-update cycle
 *
 * @param tage The TAGE predictor
 * @param startPC Starting PC for prediction
 * @param entry BTB entry to predict
 * @param actual_taken Actual outcome (taken/not taken)
 * @param history Branch history register
 * @param stagePreds Prediction results container
 */
void predictUpdateCycle(BTBTAGE* tage, Addr startPC,
                      const BTBEntry& entry,
                      bool actual_taken,
                      boost::dynamic_bitset<>& history,
                      std::vector<FullBTBPrediction>& stagePreds) {
    // 1. Make prediction
    stagePreds[1].btbEntries = {entry};
    tage->putPCHistory(startPC, history, stagePreds);

    // 2. Get predicted result
    bool predicted_taken = stagePreds[1].condTakens[entry.pc];

    // 3. Speculatively update history
    tage->specUpdateHist(history, stagePreds[1]);
    auto meta = tage->getPredictionMeta();

    // 4. Update history register
    history <<= 1;
    history[0] = predicted_taken;
    tage->checkFoldedHist(history, "speculative update");

    // 5. Create update stream
    FetchStream stream = createStream(startPC, entry, actual_taken, meta);

    // 6. Handle possible misprediction
    if (predicted_taken != actual_taken) {
        // Recover from misprediction
        tage->recoverHist(history, stream, 1, actual_taken);

        // Update history with correct outcome
        history >>= 1;  // Undo speculative update
        history <<= 1;  // Re-shift
        history[0] = actual_taken;  // Set with actual outcome
        tage->checkFoldedHist(history, "recover");
    }

    // 7. Update predictor
    tage->update(stream);
}

/**
 * @brief Directly setup TAGE table entries for testing
 *
 * @param tage The TAGE predictor
 * @param pc Branch PC
 * @param table_idx Index of the table to set
 * @param counter Counter value
 * @param useful Useful bit value
 */
void setupTageEntry(BTBTAGE* tage, Addr pc, int table_idx,
                    short counter, bool useful = false) {
    Addr index = tage->getTageIndex(pc, table_idx);
    Addr tag = tage->getTageTag(pc, table_idx);

    auto& entry = tage->tageTable[table_idx][index];
    entry.valid = true;
    entry.tag = tag;
    entry.counter = counter;
    entry.useful = useful;
    entry.pc = pc;
}

/**
 * @brief Verify TAGE table entries
 *
 * @param tage The TAGE predictor
 * @param pc Branch instruction address to check
 * @param expected_tables Vector of expected table indices to have valid entries
 */
void verifyTageEntries(BTBTAGE* tage, Addr pc, const std::vector<int>& expected_tables) {
    for (int t = 0; t < tage->numPredictors; t++) {
        Addr index = tage->getTageIndex(pc, t);
        auto &entry = tage->tageTable[t][index];

        // Check if this table should have a valid entry
        bool should_be_valid = std::find(expected_tables.begin(),
                                        expected_tables.end(), t) != expected_tables.end();

        if (should_be_valid) {
            EXPECT_TRUE(entry.valid && entry.pc == pc)
                << "Table " << t << " should have valid entry for PC " << std::hex << pc;
        }
    }
}

/**
 * @brief Find the table with a valid entry for a given PC
 *
 * @param tage The TAGE predictor
 * @param pc Branch instruction address to check
 * @return int Index of the table with valid entry (-1 if not found)
 */
int findTableWithEntry(BTBTAGE* tage, Addr pc) {
    auto meta = std::static_pointer_cast<BTBTAGE::TageMeta>(tage->getPredictionMeta());
    // use meta to find the table, predicted info
    for (int t = 0; t < tage->numPredictors; t++) {
        Addr index = tage->getTageIndex(pc, t, meta->indexFoldedHist[t].get());
        auto &entry = tage->tageTable[t][index];
        if (entry.valid && entry.pc == pc) {
            return t;
        }
    }
    return -1;
}

class BTBTAGETest : public ::testing::Test
{
protected:
    void SetUp() override {
        tage = new BTBTAGE();
        history.resize(64, false);  // 64-bit history initialized to 0
        stagePreds.resize(2);  // 2 stages
    }

    void TearDown() override {
        delete tage;
    }

    BTBTAGE* tage;
    boost::dynamic_bitset<> history;
    std::vector<FullBTBPrediction> stagePreds;
};

// Test basic prediction functionality
TEST_F(BTBTAGETest, BasicPrediction) {
    // Create a conditional branch entry biased towards taken
    BTBEntry entry = createBTBEntry(0x1000, true, true, false, 1);

    // Predict and verify
    bool taken = predictTAGE(tage, 0x1000, {entry}, history, stagePreds);

    // Should predict taken due to initial counter bias
    EXPECT_TRUE(taken) << "Initial prediction should be taken";

    // Update predictor with actual outcome = taken
    predictUpdateCycle(tage, 0x1000, entry, true, history, stagePreds);

    // Verify at least one table has an entry allocated
    int table = findTableWithEntry(tage, 0x1000);
    EXPECT_GE(table, 0) << "No TAGE table entry was allocated";
}

// Test basic history update functionality
TEST_F(BTBTAGETest, HistoryUpdate) {
    // Test case 1: Update with taken branch
    history <<= 1;
    history[0] = true;  // Set lowest bit to 1 for taken branch
    tage->doUpdateHist(history, 1, true);

    // Verify folded history state
    tage->checkFoldedHist(history, "taken update");

    // Test case 2: Update with not-taken branch
    history <<= 1;
    history[0] = false;  // Set lowest bit to 0 for not taken branch
    tage->doUpdateHist(history, 1, false);
    
    // Verify folded history state again
    tage->checkFoldedHist(history, "not-taken update");
}

// Test main and alternative prediction mechanism by direct setup
TEST_F(BTBTAGETest, MainAltPredictionBehavior) {
    // Create a branch entry for testing
    BTBEntry entry = createBTBEntry(0x1000);

    // Setup a strong main prediction (taken) in table 3
    setupTageEntry(tage, 0x1000, 3, 2); // Strong taken

    // Setup a weak alternative prediction (not taken) in table 1
    setupTageEntry(tage, 0x1000, 1, -1); // Weak not taken

    // Predict with these entries
    predictTAGE(tage, 0x1000, {entry}, history, stagePreds);

    // Check prediction metadata
    auto meta = std::static_pointer_cast<BTBTAGE::TageMeta>(tage->getPredictionMeta());
    auto pred = meta->preds[0x1000];

    // Should use main prediction (strong counter)
    EXPECT_FALSE(pred.useAlt) << "Should use main prediction with strong counter";
    EXPECT_TRUE(pred.taken) << "Main prediction should be taken";
    EXPECT_EQ(pred.mainInfo.table, 3) << "Main prediction should come from table 3";
    EXPECT_EQ(pred.altInfo.table, 1) << "Alt prediction should come from table 1";

    // Now set main prediction to weak
    setupTageEntry(tage, 0x1000, 3, 0); // Weak taken

    // Predict again
    predictTAGE(tage, 0x1000, {entry}, history, stagePreds);

    // Check prediction metadata again
    meta = std::static_pointer_cast<BTBTAGE::TageMeta>(tage->getPredictionMeta());
    pred = meta->preds[0x1000];

    // Should use alt prediction (main is weak)
    EXPECT_TRUE(pred.useAlt) << "Should use alt prediction with weak main counter";
    EXPECT_FALSE(pred.taken) << "Alt prediction should be not taken";
}

// Test useful bit update mechanism
TEST_F(BTBTAGETest, UsefulBitMechanism) {
    // Setup a test branch
    BTBEntry entry = createBTBEntry(0x1000);

    // Setup entries in main and alternative tables
    setupTageEntry(tage, 0x1000, 3, 2, false); // Main: strong taken, useful=false
    setupTageEntry(tage, 0x1000, 1, -2, false); // Alt: strong not taken, useful=false

    // Verify initial useful bit state
    Addr mainIndex = tage->getTageIndex(0x1000, 3);
    EXPECT_FALSE(tage->tageTable[3][mainIndex].useful) << "Useful bit should start as false";

    // Predict
    predictTAGE(tage, 0x1000, {entry}, history, stagePreds);
    auto meta = tage->getPredictionMeta();

    // Update with actual outcome matching main prediction (taken)
    FetchStream stream = createStream(0x1000, entry, true, meta);
    tage->update(stream);

    // Verify useful bit is set (main prediction was correct and differed from alt)
    EXPECT_TRUE(tage->tageTable[3][mainIndex].useful)
        << "Useful bit should be set when main predicts correctly and differs from alt";

    // Predict again
    predictTAGE(tage, 0x1000, {entry}, history, stagePreds);
    meta = tage->getPredictionMeta();

    // Update with actual outcome opposite to main prediction (not taken)
    stream = createStream(0x1000, entry, false, meta);
    tage->update(stream);

    // Verify useful bit is cleared (main prediction was incorrect)
    EXPECT_FALSE(tage->tageTable[3][mainIndex].useful)
        << "Useful bit should be cleared when main predicts incorrectly";
}

// Test entry allocation mechanism
TEST_F(BTBTAGETest, EntryAllocationAndReplacement) {
    // Instead of creating two different PCs, we'll create two entries with the same PC
    // This ensures they map to the same indices in the tables
    BTBEntry entry1 = createBTBEntry(0x1000);
    BTBEntry entry2 = createBTBEntry(0x1000); // Same PC to ensure same indices

    // Set all tables to have entries with useful=true
    for (int t = 0; t < tage->numPredictors; t++) {
        setupTageEntry(tage, 0x1000, t, 0, true); // Counter=0, useful=true
    }

    // Force a misprediction to trigger allocation attempt
    // First, make a prediction
    predictTAGE(tage, 0x1000, {entry1}, history, stagePreds);
    auto meta = tage->getPredictionMeta();
    bool predicted = false;
    if (stagePreds[1].condTakens.find(0x1000) != stagePreds[1].condTakens.end()) {
        predicted = stagePreds[1].condTakens[0x1000];
    }

    // Create a stream for entry2 with opposite outcome to force allocation
    // Although it has the same PC, we'll treat it as a different branch context
    // by setting a specific tag that doesn't match existing entries
    FetchStream stream = createStream(0x1000, entry2, !predicted, meta);
    stream.squashType = SquashType::SQUASH_CTRL; // Mark as control misprediction
    stream.squashPC = 0x1000;

    // Temporarily store the original entries to check for changes later
    std::vector<std::vector<BTBTAGE::TageEntry>> originalEntries;
    originalEntries.resize(tage->numPredictors);
    for (int t = 0; t < tage->numPredictors; t++) {
        Addr index = tage->getTageIndex(0x1000, t);
        originalEntries[t].push_back(tage->tageTable[t][index]);
    }

    // Update the predictor (this should try to allocate but fail)
    tage->update(stream);

    // Check if entries were modified (should not be since all useful bits were set)
    bool any_entry_modified = false;
    for (int t = 0; t < tage->numPredictors; t++) {
        Addr index = tage->getTageIndex(0x1000, t);
        // only check tag, because counter will update!
        bool entry_modified = (tage->tageTable[t][index].tag != originalEntries[t][0].tag);
        if (entry_modified) {
            any_entry_modified = true;
            break;
        }
    }

    // Verify no allocation occurred (entries should not be modified)
    EXPECT_FALSE(any_entry_modified) << "Entries should not be modified when all useful bits are set";

}

// Test history recovery mechanism
TEST_F(BTBTAGETest, HistoryRecoveryCorrectness) {
    BTBEntry entry = createBTBEntry(0x1000);

    // Record initial history state
    boost::dynamic_bitset<> originalHistory = history;

    // Store original folded history state
    std::vector<FoldedHist> originalTagFoldedHist;
    std::vector<FoldedHist> originalAltTagFoldedHist;
    std::vector<FoldedHist> originalIndexFoldedHist;

    for (int i = 0; i < tage->numPredictors; i++) {
        originalTagFoldedHist.push_back(tage->tagFoldedHist[i]);
        originalAltTagFoldedHist.push_back(tage->altTagFoldedHist[i]);
        originalIndexFoldedHist.push_back(tage->indexFoldedHist[i]);
    }

    // Make a prediction
    bool predicted_taken = predictTAGE(tage, 0x1000, {entry}, history, stagePreds);

    // Speculatively update history
    tage->specUpdateHist(history, stagePreds[1]);
    auto meta = tage->getPredictionMeta();

    // Update history register
    history <<= 1;
    history[0] = predicted_taken;

    // Create a recovery stream with opposite outcome
    FetchStream stream = createStream(0x1000, entry, !predicted_taken, meta);

    // Recover to pre-speculative state and update with correct outcome
    boost::dynamic_bitset<> recoveryHistory = originalHistory;
    tage->recoverHist(recoveryHistory, stream, 1, !predicted_taken);

    // Expected history should be original shifted with correct outcome
    boost::dynamic_bitset<> expectedHistory = originalHistory;
    expectedHistory <<= 1;
    expectedHistory[0] = !predicted_taken;

    // Verify recovery produced the expected history
    for (int i = 0; i < tage->numPredictors; i++) {
        tage->tagFoldedHist[i].check(expectedHistory);
        tage->altTagFoldedHist[i].check(expectedHistory);
        tage->indexFoldedHist[i].check(expectedHistory);
    }
}

// Simplified test for multiple branch sequence
TEST_F(BTBTAGETest, MultipleBranchSequence) {
    // Create two branches
    std::vector<BTBEntry> btbEntries = {
        createBTBEntry(0x1000),
        createBTBEntry(0x1004)
    };

    // Predict for both branches
    predictTAGE(tage, 0x1000, btbEntries, history, stagePreds);
    auto meta = tage->getPredictionMeta();

    // Get predictions for both branches
    bool first_pred = false, second_pred = false;
    if (stagePreds[1].condTakens.find(0x1000) != stagePreds[1].condTakens.end()) {
        first_pred = stagePreds[1].condTakens[0x1000];
    }
    if (stagePreds[1].condTakens.find(0x1004) != stagePreds[1].condTakens.end()) {
        second_pred = stagePreds[1].condTakens[0x1004];
    }

    // Update first branch (correct prediction)
    FetchStream stream1 = createStream(0x1000, btbEntries[0], first_pred, meta);
    tage->update(stream1);

    // Update second branch (incorrect prediction)
    FetchStream stream2 = createStream(0x1000, btbEntries[1], !second_pred, meta);
    stream2.squashType = SquashType::SQUASH_CTRL;
    stream2.squashPC = 0x1004;
    tage->update(stream2);

    // Verify both branches have entries allocated
    EXPECT_GE(findTableWithEntry(tage, 0x1000), 0) << "First branch should have an entry";
    EXPECT_GE(findTableWithEntry(tage, 0x1004), 0) << "Second branch should have an entry";
}

// Test counter update mechanism
TEST_F(BTBTAGETest, CounterUpdateMechanism) {
    BTBEntry entry = createBTBEntry(0x1000);

    // Setup a TAGE entry with a neutral counter
    int testTable = 3;
    setupTageEntry(tage, 0x1000, testTable, 0);

    // Verify initial counter value
    Addr index = tage->getTageIndex(0x1000, testTable);
    EXPECT_EQ(tage->tageTable[testTable][index].counter, 0) << "Initial counter should be 0";

    // Train with taken outcomes multiple times
    for (int i = 0; i < 3; i++) {
        predictTAGE(tage, 0x1000, {entry}, history, stagePreds);
        auto meta = tage->getPredictionMeta();

        FetchStream stream = createStream(0x1000, entry, true, meta);
        tage->update(stream);
    }

    // Verify counter saturates at maximum
    EXPECT_EQ(tage->tageTable[testTable][index].counter, 3)
        << "Counter should saturate at maximum value";

    // Train with not-taken outcomes multiple times
    for (int i = 0; i < 7; i++) {
        predictTAGE(tage, 0x1000, {entry}, history, stagePreds);
        auto meta = tage->getPredictionMeta();

        FetchStream stream = createStream(0x1000, entry, false, meta);
        tage->update(stream);
    }

    // Verify counter saturates at minimum
    EXPECT_EQ(tage->tageTable[testTable][index].counter, -4)
        << "Counter should saturate at minimum value";
}

}  // namespace test

}  // namespace btb_pred

}  // namespace branch_prediction

}  // namespace gem5
