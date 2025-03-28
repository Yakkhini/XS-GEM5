# BTB-TAGE Predictor Design Document

## 1. Overview

The BTB-TAGE (Branch Target Buffer - Tagged Geometric Length) predictor is a sophisticated branch prediction mechanism implemented in gem5. It combines a BTB (Branch Target Buffer) for target address prediction with a TAGE (Tagged Geometric Length) predictor for direction prediction of conditional branches. The TAGE component uses multiple prediction tables with different history lengths to capture various correlation patterns in branch behavior.

This document describes the design and implementation of the BTBTAGE class, which serves as the direction predictor component in the overall branch prediction system.

## 2. Key Data Structures

### 2.1 TAGE Entry

```cpp
struct TageEntry {
    bool valid;      // Whether this entry is valid
    Addr tag;        // Tag for matching
    short counter;   // Prediction counter (-4 to 3), 3 bits
    bool useful;     // Whether this entry is useful for prediction
    Addr pc;         // Branch PC, for BTB entry PC check
};
```

The `TageEntry` structure represents a single entry in a TAGE prediction table. The `counter` field is a saturating counter that determines the taken/not-taken prediction, with values ≥ 0 indicating taken. The `useful` bit is set when this entry provided a correct prediction different from the alternative prediction.

### 2.2 Table Lookup Result

```cpp
struct TageTableInfo {
    bool found;      // Whether a matching entry was found
    TageEntry entry; // The matching entry
    unsigned table;  // Which table this entry was found in
    Addr index;      // Index in the table
    Addr tag;        // Tag that was matched
};
```

This structure contains information about a TAGE table lookup, including whether an entry was found and the details of the matching entry.

### 2.3 TAGE Prediction

```cpp
struct TagePrediction {
    Addr btb_pc;            // BTB entry PC
    TageTableInfo mainInfo; // Main prediction info
    TageTableInfo altInfo;  // Alternative prediction info
    bool useAlt;            // Whether to use alternative prediction
    bool taken;             // Final prediction
};
```

The `TagePrediction` structure contains the complete prediction result, including both the main and alternative predictions, and whether the alternative prediction should be used.

### 2.4 Prediction Metadata

```cpp
struct TageMeta {
    std::map<Addr, TagePrediction> preds;
    bitset usefulMask;
    std::vector<FoldedHist> tagFoldedHist;
    std::vector<FoldedHist> altTagFoldedHist;
    std::vector<FoldedHist> indexFoldedHist;
};
```

The `TageMeta` structure stores metadata for predictions, which is essential for recovering from mispredictions and updating the predictor state. It contains the predictions for each PC, a mask of useful bits, and the folded history states.

## 3. Key Algorithms

### 3.1 Prediction Generation

The prediction process involves:

1. Looking up entries in all TAGE tables using the `lookupTageTable` method
2. Finding the main and alternative predictions
3. Determining whether to use the alternative prediction
4. Generating the final prediction

```cpp
std::map<Addr, bool> lookupHelper(const Addr &startPC, const std::vector<BTBEntry> &btbEntries) {
    // Clear old prediction metadata and save current history state
    meta.preds.clear();
    meta.tagFoldedHist = tagFoldedHist;
    meta.altTagFoldedHist = altTagFoldedHist;
    meta.indexFoldedHist = indexFoldedHist;

    // Look up all TAGE tables
    auto table_result = lookupTageTable(startPC);
    meta.usefulMask = table_result.useful_mask;

    // Generate predictions for each BTB entry
    std::map<Addr, bool> cond_takens;
    for (auto &btb_entry : btbEntries) {
        if (btb_entry.isCond && btb_entry.valid) {
            auto pred = generateSinglePrediction(btb_entry, table_result);
            meta.preds[btb_entry.pc] = pred;
            cond_takens[btb_entry.pc] = pred.taken || btb_entry.alwaysTaken;
        }
    }
    return cond_takens;
}
```

### 3.2 Table Lookup

The table lookup process involves:

1. Calculating indices and tags for each table
2. Checking for matching entries
3. Collecting the entries and updating useful masks

```cpp
std::vector<BTBTAGE::TageEntry> lookupTageTable(const Addr &startPC) {
    std::vector<BTBTAGE::TageEntry> entries(numPredictors);
    meta.usefulMask.resize(numPredictors);
    
    // Look up entries in all TAGE tables
    for (int i = 0; i < numPredictors; ++i) {
        Addr index = getTageIndex(startPC, i);
        Addr tag = getTageTag(startPC, i);
        auto &entry = tageTable[i][index];
        
        result.entries[i] = entry;
        result.indices[i] = index;
        result.tags[i] = tag;
        result.useful_mask[i] = entry.useful;
    }
    return result;
}
```

### 3.3 Predictor Update

The update process involves:

1. Preparing BTB entries for update
2. Updating prediction counters based on actual branch outcomes
3. Managing useful bits
4. Allocating new entries on mispredictions

```cpp
void update(const FetchStream &stream) {
    Addr startAddr = stream.getRealStartPC();
    
    // Prepare entries to update
    auto entries_to_update = prepareUpdateEntries(stream);
    
    // Get prediction metadata
    auto meta = std::static_pointer_cast<TageMeta>(stream.predMetas[getComponentIdx()]);
    
    // Update each BTB entry
    for (auto &entry : entries_to_update) {
        bool actual_taken = stream.exeTaken && stream.exeBranchInfo == entry;
        auto pred_it = meta->preds.find(entry.pc);
        
        if (pred_it == meta->preds.end()) {
            continue;
        }
        
        // Update predictor state and check for allocation
        bool need_allocate = updatePredictorStateAndCheckAllocation(
            entry, actual_taken, pred_it->second, stream);
        
        if (need_allocate) {
            // Handle useful bit reset
            handleUsefulBitReset(meta->usefulMask);
            
            // Handle allocation of new entries
            uint start_table = 0;
            auto useful_mask = meta->usefulMask;
            if (pred_it->second.mainInfo.found) {
                start_table = pred_it->second.mainInfo.table + 1;
                useful_mask >>= start_table;
                useful_mask.resize(numPredictors - start_table);
            }
            handleNewEntryAllocation(startAddr, entry, actual_taken, 
                                   useful_mask, start_table, meta);
        }
    }
}
```

### 3.4 History Management

The predictor maintains three types of folded histories:

1. Tag folded history: Used for tag computation
2. Alternative tag folded history: Used for alternative tag computation
3. Index folded history: Used for table index computation

These histories are updated speculatively and recovered on mispredictions.

```cpp
void specUpdateHist(const boost::dynamic_bitset<> &history, FullBTBPrediction &pred) {
    int shamt;
    bool cond_taken;
    std::tie(shamt, cond_taken) = pred.getHistInfo();
    doUpdateHist(history, shamt, cond_taken);
}

void recoverHist(const boost::dynamic_bitset<> &history, const FetchStream &entry, int shamt, bool cond_taken) {
    std::shared_ptr<TageMeta> predMeta = std::static_pointer_cast<TageMeta>(
        entry.predMetas[getComponentIdx()]);
    
    // Recover folded histories
    for (int i = 0; i < numPredictors; i++) {
        tagFoldedHist[i].recover(predMeta->tagFoldedHist[i]);
        altTagFoldedHist[i].recover(predMeta->altTagFoldedHist[i]);
        indexFoldedHist[i].recover(predMeta->indexFoldedHist[i]);
    }
    
    // Update histories with correct information
    doUpdateHist(history, shamt, cond_taken);
}
```

## 4. Interaction with BTB

The BTBTAGE predictor works in conjunction with the BTB to provide complete branch prediction:

1. The BTB identifies branches and provides target addresses
2. The BTBTAGE predictor provides direction predictions for conditional branches
3. The two components communicate through the `BTBEntry` structure and the `FullBTBPrediction` structure

The key interactions include:

- The BTB identifies branches and creates `BTBEntry` structures
- The BTBTAGE predictor takes these entries and produces direction predictions
- The predictions are combined in the `FullBTBPrediction` structure
- The combined predictions guide fetch and execution
- Execution results update both predictors

## 5. Integration with Fetch Stream

The BTBTAGE predictor integrates with the fetch stream mechanism through:

1. `putPCHistory`: Makes predictions for a stream of instructions
2. `specUpdateHist`: Speculatively updates history based on predictions
3. `recoverHist`: Recovers history state after mispredictions
4. `update`: Updates predictor state based on actual branch outcomes

The fetch stream mechanism provides a unified interface for all branch predictors and manages the flow of predictions and updates.

## 6. Design Considerations

### 6.1 Multiple Tables with Different History Lengths

The TAGE predictor uses multiple tables with different history lengths to capture different correlation patterns. This allows it to adapt to branches with different correlation behaviors.

### 6.2 Useful Bit Management

The useful bit mechanism helps manage the allocation of new entries and the replacement of existing entries. It ensures that valuable entries are preserved and less valuable ones are replaced.

### 6.3 Main and Alternative Predictions

The predictor provides both main and alternative predictions, and uses a mechanism to decide when to use the alternative prediction. This improves accuracy for branches that are difficult to predict.

### 6.4 Predictor Recovery

The predictor maintains metadata for each prediction, which allows it to recover from mispredictions and ensure consistent state.

## 7. Performance Optimizations

### 7.1 Folded History

The predictor uses folded history to reduce the storage requirements for the history registers while maintaining correlation information.

### 7.2 Allocation Strategy

The allocation strategy uses a combination of LFSR-based random allocation and useful bit guidance to efficiently allocate new entries.

### 7.3 Useful Bit Reset

The periodic reset of useful bits prevents the predictor from becoming stuck with outdated entries.

## 8. Testing and Validation

The BTBTAGE predictor includes comprehensive testing capabilities:

1. Basic prediction testing
2. History update verification
3. Table lookup testing
4. Main/alternative prediction selection testing
5. Useful bit mechanism testing

These tests ensure the correct functionality of the predictor and validate its behavior in various scenarios.

## 9. Conclusion

The BTBTAGE predictor is a sophisticated branch direction prediction mechanism that integrates with the BTB to provide accurate branch prediction. Its multiple table design, useful bit management, and history folding techniques make it highly effective at capturing branch behavior patterns and adapting to different types of branches.

## 10. Testing Framework

### 10.1 TAGE Testing Flow

The TAGE predictor follows a specific prediction-update-recovery flow in its testing, which mirrors its actual operation in the processor pipeline:

#### Prediction Phase
```cpp
// Setup BTB entries and history
std::vector<BTBEntry> btbEntries;
boost::dynamic_bitset<> history(64, 0);  // Global history register

// Make prediction
tage->putPCHistory(startPC, history, stagePreds);

// Get prediction metadata (checkpoint state)
auto meta = tage->getPredictionMeta();

// Speculatively update history (folded histories)
tage->specUpdateHist(history, stagePreds[1]);  // Use final prediction to update

// Shift actual history register
history <<= shamt;  // Shift by amount corresponding to instructions
history[0] = taken;  // Set lowest bit based on prediction
```

#### Update Phase (Correct Prediction)
When the branch is resolved and found to be correctly predicted:

```cpp
// Setup update stream with actual outcome
FetchStream stream;
stream.startPC = pc;
stream.exeBranchInfo = branch_info;
stream.exeTaken = actual_taken;
stream.predMetas[0] = meta;  // Must include metadata from prediction phase

// Update predictor state
tage->update(stream);
```

#### Recovery Phase (Misprediction)
When a misprediction is detected, history must be recovered:

```cpp
// Setup recovery stream
FetchStream recoverStream;
recoverStream.startPC = pc;
recoverStream.exeBranchInfo = branch_info;
recoverStream.exeTaken = actual_taken;
recoverStream.predMetas[0] = meta;  // Must include metadata from prediction

// Manually update global history register to match
history >>= shamt;  // Undo speculative update
// Recover history to checkpoint state, then update with actual outcome
tage->recoverHist(history, recoverStream, shamt, actual_taken);

history <<= shamt;  // Re-shift
history[0] = actual_taken;  // Set with actual outcome
```
