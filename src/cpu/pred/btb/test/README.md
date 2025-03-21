# Branch Prediction Test Suite

This document describes the test suite for branch prediction components in gem5, including μRAS, TAGE, and BTB predictors.

## Overview

The test suite verifies the functionality of different branch prediction components:
- μRAS (Micro Return Address Stack)
- TAGE (Tagged Geometric Length) Predictor
- BTB (Branch Target Buffer)

## Test Components

### 1. μRAS Test Suite

The test suite verifies the functionality of the Return Address Stack (RAS) predictor used in the branch prediction system. It uses Google Test framework and implements a mock version of the RAS to facilitate testing.

#### Mock Implementation

##### Key Classes

###### `MockuRAS`
A simplified version of `BTBuRAS` that implements core RAS functionality:
- Maintains a speculative stack (`specStack`) and stack pointer (`specSp`)
- Implements push/pop operations with counter management
- Provides prediction and recovery mechanisms
- Omits gem5-specific features like debugging and tracing

###### `uRASEntry`
Represents a single entry in the RAS:
- `retAddr`: Return address
- `ctr`: Counter for multiple returns to the same address

###### `uRASMeta`
Stores metadata for prediction recovery:
- `sp`: Stack pointer
- `tos`: Top of stack entry

##### Relationship with BTBuRAS

The mock implementation mirrors the core functionality of `BTBuRAS` while:
- Removing gem5-specific dependencies
- Simplifying the interface for testing
- Maintaining the same behavioral characteristics

### 2. TAGE Predictor Test Suite

#### Key Classes

##### `BTBTAGE`
A simplified version of TAGE predictor that implements:
- Multiple prediction tables with different history lengths
- Tag-based prediction mechanism
- Useful bit management
- Alternative prediction selection

##### `TageMeta`
Stores metadata for prediction:
- `preds`: Map of predictions for each PC
- `usefulMask`: Mask for useful bits
- `tagFoldedHist/altTagFoldedHist/indexFoldedHist`: Folded history states

#### Test Flow Pattern

TAGE predictor follows a specific prediction-update pattern:

1. Prediction Phase:
```cpp
// Setup BTB entries and history
std::vector<BTBEntry> btbEntries;
boost::dynamic_bitset<> history(64, 0);

// Make prediction to generate meta data
tage->putPCHistory(startPC, history, stagePreds);

// update (folded) histories for tage
tage->specUpdateHist(s0History, finalPred);
tage->getPredictionMeta();

// shift history
histShiftIn(shamt, taken, s0History);
//     history <<= shamt;
//     history[0] = taken;
tage->checkFoldedHist(s0History, "speculative update");

```

2. Update Phase:
```cpp
// Setup update stream
FetchStream stream;
stream.startPC = pc;
stream.exeBranchInfo = entry;
stream.exeTaken = taken;
stream.predMetas[0] = meta;  // Must set meta from prediction phase

// Update predictor
tage->update(stream);
```

3. control squash/Recovery Phase:
```cpp
// set up recover stream
FetchStream recoverStream;
recoverStream.startPC = pc;
recoverStream.exeBranchInfo = entry;
recoverStream.exeTaken = taken;
recoverStream.predMetas[0] = meta;

tage.recoverHist(s0History, recoverStream, shamt, taken);
// update real history
histShiftIn(shamt, taken, s0History);
tage.checkFoldedHist(s0History, "recover");
```

Note: if branch predict is correct, then goto Update Phase, otherwise goto Recovery Phase

#### Key Test Cases

1. `BasicPrediction`: Tests basic prediction functionality
2. `BasicHistoryUpdate`: Verifies history update mechanism
3. `TestTableLookup`: Tests TAGE table allocation and lookup
4. `TestMainAltPrediction`: Verifies main/alternative prediction selection
5. `TestUsefulBitMechanism`: Tests useful bit management

### 3. BTB Test Suite

#### Key Classes

##### `BTBBase`
Base class for BTB implementations with core functionality:
- Entry storage and lookup
- Prediction generation
- Update mechanism

#### Test Flow Pattern

BTB follows a similar prediction-update pattern:

1. Prediction Phase:
```cpp
// Make prediction
btb->putPCHistory(startPC, history, stagePreds);

// Get prediction metadata
auto meta = btb->getPredictionMeta();
```

2. Update Phase:
```cpp
// Setup update stream
FetchStream stream;
stream.startPC = pc;
stream.predMetas[0] = meta;

// Optional: Get and set new BTB entry (only for L1 BTB)
if (isL1BTB) {
    btb->getAndSetNewBTBEntry(stream);
}

// Update BTB
btb->update(stream);
```

#### Important Notes

1. Meta Data Handling:
- Always generate meta through prediction before update
- Never manually create meta data
- Always pass meta from prediction to update phase

2. Test Case Structure:
- Setup: Create history and BTB entries
- Predict: Generate predictions and meta
- Update: Apply updates with actual outcomes
- Verify: Check predictor state and predictions

3. Common Pitfalls:
- Missing meta data in update phase
- Incorrect order of prediction/update calls
- Not maintaining history consistency

## Running Tests

```bash
# Build all branch prediction tests
scons build/RISCV/cpu/pred/btb/test/uras.test.debug
scons build/RISCV/cpu/pred/btb/test/tage.test.debug
scons build/RISCV/cpu/pred/btb/test/btb.test.debug

# Run all tests (with debug output)
./build/RISCV/cpu/pred/btb/test/tage.test.debug

# First build debug version
scons build/RISCV/cpu/pred/btb/test/tage.test.debug
# Then run with filter
./build/RISCV/cpu/pred/btb/test/tage.test.debug --gtest_filter=BTBTAGETest.BasicPrediction
```

## Adding New Tests

When adding new tests:
1. Create a new test class inheriting from ::testing::Test
2. Follow the prediction-update pattern
3. Add test to SConscript
4. Document test purpose and patterns
5. Ensure proper meta data handling
