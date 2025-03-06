# μRAS (Micro Return Address Stack) Test Suite

This document describes the test suite for the μRAS (Micro Return Address Stack) implementation in gem5.

## Overview

The test suite verifies the functionality of the Return Address Stack (RAS) predictor used in the branch prediction system. It uses Google Test framework and implements a mock version of the RAS to facilitate testing.

## Mock Implementation

### Key Classes

#### `MockuRAS`
A simplified version of `BTBuRAS` that implements core RAS functionality:
- Maintains a speculative stack (`specStack`) and stack pointer (`specSp`)
- Implements push/pop operations with counter management
- Provides prediction and recovery mechanisms
- Omits gem5-specific features like debugging and tracing

#### `uRASEntry`
Represents a single entry in the RAS:
- `retAddr`: Return address
- `ctr`: Counter for multiple returns to the same address

#### `uRASMeta`
Stores metadata for prediction recovery:
- `sp`: Stack pointer
- `tos`: Top of stack entry

### Relationship with BTBuRAS

The mock implementation mirrors the core functionality of `BTBuRAS` while:
- Removing gem5-specific dependencies
- Simplifying the interface for testing
- Maintaining the same behavioral characteristics

## Running Tests

To run the μRAS tests specifically:

```bash
# Build the test
scons build/RISCV/cpu/pred/btb/test/uras.test.opt

# Run all μRAS tests
./build/RISCV/cpu/pred/btb/test/uras.test.opt

# Run a specific test
./build/RISCV/cpu/pred/btb/test/uras.test.opt --gtest_filter=URASTest.BasicPush
```

## Adding Other Classes Tests

When adding new tests:
1. Add a new Mock test class in test
2. add GTest macro in SConscript
