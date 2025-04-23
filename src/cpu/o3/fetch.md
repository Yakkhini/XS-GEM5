# GEM5 O3 CPU Fetch Stage Analysis

## Overview

The Fetch stage is the first pipeline stage in the GEM5 O3 processor model. It is responsible for fetching instructions from the instruction cache and passing them to the Decode stage. In the XiangShan GEM5 customized version, the Fetch stage implements a decoupled frontend design to align with the XiangShan processor architecture.

## Key Interfaces with Other Pipeline Stages

### 1. Time Buffer Interfaces

The Fetch stage communicates with other pipeline stages through a time buffer mechanism, which models the delay in communication between different stages:

```cpp
TimeBuffer<TimeStruct> *timeBuffer;  // Main time buffer for communication

// Wires from other stages to Fetch
TimeBuffer<TimeStruct>::wire fromDecode;   // For stall signals and instruction counts
TimeBuffer<TimeStruct>::wire fromRename;   // For stall signals
TimeBuffer<TimeStruct>::wire fromIEW;      // For stall signals and branch resolution
TimeBuffer<TimeStruct>::wire fromCommit;   // For squash signals and interrupts

// Wire to Decode stage
TimeBuffer<FetchStruct>::wire toDecode;    // For sending fetched instructions
```

### 2. Interface with Decode Stage

The Fetch stage forwards fetched instructions to the Decode stage through the `toDecode` wire:

```cpp
// In tick() method:
// Send instruction packet to decode
if (numInst) {
    toDecode->insts = std::move(insts);
    toDecode->size = numInst;
    wroteToTimeBuffer = true;
}
```

The Decode stage can send stall signals back to Fetch via the `fromDecode` wire:

```cpp
// In checkSignalsAndUpdate() method:
// Check if the decode stage is stalled
if (fromDecode->decodeStall) {
    stalls[tid].decode = true;
}
```

### 3. Interface with Commit Stage

The Commit stage sends several important signals to the Fetch stage:

- **Branch misprediction signals**: Indicate a branch was mispredicted and the pipeline needs to be squashed
- **Interrupt signals**: Indicate an interrupt needs to be processed
- **Drain signals**: For simulation control

```cpp
// In checkSignalsAndUpdate() method:
// Check for squash from commit
if (fromCommit->commitInfo[tid].squash) {
    DPRINTF(Fetch, "[tid:%i] Squashing from commit.\n", tid);
    squash(fromCommit->commitInfo[tid].pc,
           fromCommit->commitInfo[tid].doneSeqNum,
           fromCommit->commitInfo[tid].squashInst, tid);
}

// Check for commit's interrupt signals
if (fromCommit->commitInfo[tid].interruptPending) {
    interruptPending = true;
}
```

### 4. Interface with IEW Stage

The IEW (Issue/Execute/Writeback) stage provides branch resolution feedback to the Fetch stage:

```cpp
// In checkSignalsAndUpdate() method:
// Check for squash from IEW (mispredicted branch)
if (fromIEW->iewInfo[tid].squash) {
    DPRINTF(Fetch, "[tid:%i] Squashing from IEW.\n", tid);
    squash(fromIEW->iewInfo[tid].pc,
           fromIEW->iewInfo[tid].doneSeqNum,
           fromIEW->iewInfo[tid].squashInst, tid);
}
```

### 5. Branch Predictor Interface

The Fetch stage interacts with the branch predictor to determine the next PC to fetch:

```cpp
bool lookupAndUpdateNextPC(const DynInstPtr &inst, PCStateBase &pc) {
    // Access branch predictor and update PC
    bool predicted_taken = getBp()->predict(inst, pc, inst->pcState());
    // ... additional logic ...
    return predicted_taken;
}
```

## Key Data Structures

### 1. Fetch Buffer

The fetch buffer holds raw instruction data fetched from the instruction cache:

```cpp
uint8_t *fetchBuffer[MaxThreads];     // Raw instruction data
Addr fetchBufferPC[MaxThreads];       // PC of first instruction in buffer
bool fetchBufferValid[MaxThreads];    // Whether buffer data is valid
unsigned fetchBufferSize;             // Size of fetch buffer in bytes
Addr fetchBufferMask;                 // Mask to align PC to fetch buffer boundary
```

The fetch buffer works as a temporary storage between the instruction cache and the instruction queue. Instructions are fetched from the instruction cache in cache-line-sized chunks and stored in the fetch buffer.

### 2. Fetch Queue

The fetch queue stores the processed dynamic instructions before they are sent to decode:

```cpp
std::deque<DynInstPtr> fetchQueue[MaxThreads];  // Queue of fetched instructions
unsigned fetchQueueSize;                        // Maximum size of fetch queue
```

### 3. Memory Request Structures

For I-cache access management:

```cpp
RequestPtr memReq[MaxThreads];        // Primary memory request
RequestPtr anotherMemReq[MaxThreads]; // Used for unaligned access
PacketPtr firstPkt[MaxThreads];       // First packet for I-cache access
PacketPtr secondPkt[MaxThreads];      // Second packet for unaligned access
std::pair<Addr, Addr> accessInfo[MaxThreads];  // Address info for cache access
```

### 4. Status Tracking Structures

```cpp
// Overall fetch status
enum FetchStatus { Active, Inactive } _status;

// Per-thread status
enum ThreadStatus {
    Running, Idle, Squashing, Blocked, Fetching, TrapPending,
    QuiescePending, ItlbWait, IcacheWaitResponse, IcacheWaitRetry,
    IcacheAccessComplete, NoGoodAddr, NumFetchStatus
} fetchStatus[MaxThreads];

// Stall tracking
struct Stalls {
    bool decode;
    bool drain;
} stalls[MaxThreads];

// Stall reason tracking
std::vector<StallReason> stallReason;
```

### 5. Branch Prediction Structures

```cpp
branch_prediction::BPredUnit *branchPred;  // Main branch predictor
branch_prediction::stream_pred::DecoupledStreamBPU *dbsp;  // Stream predictor
branch_prediction::ftb_pred::DecoupledBPUWithFTB *dbpftb;  // FTB predictor
branch_prediction::btb_pred::DecoupledBPUWithBTB *dbpbtb;  // BTB predictor
```

### 6. Loop Buffer Structures

```cpp
branch_prediction::ftb_pred::LoopBuffer *loopBuffer;  // Loop buffer
bool enableLoopBuffer;                                // Loop buffer enable flag
unsigned currentLoopIter;                             // Current loop iteration counter
bool currentFetchTargetInLoop;                        // If current fetch is in a loop
```

## Core Function Workflow

### 1. Main Fetch Cycle

```
tick()
  |
  +--> checkSignalsAndUpdate()  // Check for control signals from other stages
  |
  +--> fetch()  // Perform actual instruction fetching
        |
        +--> getFetchingThread()  // Select thread to fetch from
        |
        +--> fetchCacheLine()  // Access I-cache
        |
        +--> lookupAndUpdateNextPC()  // Consult branch predictor
        |
        +--> buildInst()  // Create dynamic instructions
        |
        +--> writeToTimeBuffer()  // Send instructions to decode
```

### 2. Signal Processing Workflow

```
checkSignalsAndUpdate()
  |
  +--> Process stall signals from Decode
  |
  +--> Process branch resolution from IEW
  |
  +--> Process squash signals from Commit
  |
  +--> Process interrupt signals from Commit
```

### 3. I-Cache Access Workflow

```
fetchCacheLine()
  |
  +--> Create memory request(s)
  |
  +--> Send request(s) to I-cache
  |
  +--> Wait for response in recvTimingResp()
        |
        +--> Process received data
        |
        +--> Update fetch buffer
        |
        +--> Update fetch status
```

### 4. Branch Prediction Handling

```
lookupAndUpdateNextPC()
  |
  +--> Query branch predictor
  |
  +--> Update next PC based on prediction
  |
  +--> Track prediction metadata
```

## Decoupled Frontend Implementation

The decoupled frontend design separates branch prediction from instruction fetching, using specialized queues:

```cpp
// Check if using decoupled frontend
bool isDecoupledFrontend() { return branchPred->isDecoupled(); }

// Different predictor types
bool isStreamPred() { return branchPred->isStream(); }
bool isFTBPred() { return branchPred->isFTB(); }
bool isBTBPred() { return branchPred->isBTB(); }

// Track if FTQ is empty
bool ftqEmpty() { return isDecoupledFrontend() && usedUpFetchTargets; }
```

In the decoupled design, the branch predictor works ahead of the fetch unit, generating fetch targets that are stored in the Fetch Target Queue (FTQ). The fetch unit then consumes entries from this queue.

## Key Implementation Details

### 1. Instruction Cache Interface

The Fetch stage has its own port to access the instruction cache:

```cpp
class IcachePort : public RequestPort {
    // Handles timing requests to I-cache
    virtual bool recvTimingResp(PacketPtr pkt);
    
    // Handles retry signals from I-cache
    virtual void recvReqRetry();
};

IcachePort icachePort;
```

### 2. Address Translation Handling

The Fetch stage handles instruction address translation:

```cpp
class FetchTranslation : public BaseMMU::Translation {
    // Called when translation completes
    void finish(const Fault &fault, const RequestPtr &req,
                ThreadContext *tc, BaseMMU::Mode mode);
};

// Event to handle delayed translation results
class FinishTranslationEvent : public Event {
    // Process translation result
    void process();
};

FinishTranslationEvent finishTranslationEvent;
```

### 3. SMT Thread Selection

The Fetch stage has multiple thread selection policies:

```cpp
// Thread selection policies
ThreadID getFetchingThread();  // Main policy selection function
ThreadID roundRobin();         // Round robin policy
ThreadID iqCount();            // Based on instruction queue count
ThreadID lsqCount();           // Based on load/store queue count
ThreadID branchCount();        // Based on branch count
```

## Performance Monitoring and Statistics

```cpp
struct FetchStatGroup : public statistics::Group {
    // Stall statistics
    statistics::Scalar icacheStallCycles;
    statistics::Scalar tlbCycles;
    statistics::Scalar idleCycles;
    statistics::Scalar blockedCycles;
    
    // Instruction statistics
    statistics::Scalar insts;
    statistics::Scalar branches;
    statistics::Scalar predictedBranches;
    
    // Performance metrics
    statistics::Formula idleRate;
    statistics::Formula branchRate;
    statistics::Formula rate;
    
    // Frontend performance metrics
    statistics::Formula frontendBound;
    statistics::Formula frontendLatencyBound;
    statistics::Formula frontendBandwidthBound;
};
```

## XiangShan-Specific Enhancements

1. **Decoupled Frontend Designs**: Support for BTB, FTB, and Stream-based prediction
2. **TAGE, ITTAGE and Loop Predictor**: Advanced branch prediction aligned with XiangShan
3. **Instruction Latency Calibration**: Timing calibrated to match Kunminghu hardware

## Advanced Features

1. **Loop Buffer**: Caches loop instructions for energy efficiency
2. **Pipelined I-cache Access**: Allows overlapping multiple I-cache accesses
3. **Fetch Throttling**: Controls fetch rate based on backend pressure
