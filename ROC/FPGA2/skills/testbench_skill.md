# GitHub Copilot Skill: Create a VHDL Testbench for DAQ Firmware

## Purpose

When I provide a VHDL entity, create a professional testbench suitable for FPGA firmware used in particle-physics data acquisition systems.

The goal is to produce a self-checking testbench that verifies functionality, timing assumptions, state-machine behavior, handshake protocols, FIFO operation, and clock-domain-crossing robustness.

---

## General Requirements

Create a complete testbench that:

* Instantiates the DUT (Device Under Test).
* Generates all required clocks and resets.
* Exercises normal operation.
* Exercises edge cases and error conditions.
* Verifies outputs automatically using assertions.
* Produces clear simulation logs.
* Avoids relying solely on waveform inspection.

Before generating code, briefly explain your understanding of what the DUT is intended to do.

---

## Required Testbench Structure

Generate:

1. Library declarations
2. Testbench entity
3. Testbench architecture
4. Clock generation processes
5. Reset generation process
6. DUT instantiation
7. Stimulus process(es)
8. Monitor/checker process(es)
9. Assertions
10. End-of-simulation report

Use descriptive signal names and clear comments throughout.

---

## Clock Generation

For each clock:

* Generate realistic periodic clocks.
* Define clock periods using named constants.
* Keep clock generation separate from stimulus logic.

Example clocks:

* `SysClk` = 100 MHz
* `i50MHz` = 50 MHz

When multiple clock domains exist, ensure independent clock generation.

---

## Reset Verification

Verify:

* Proper initialization after reset
* Reset held for several clock cycles
* Correct recovery after reset release
* Mid-run reset behavior
* Recovery from unexpected resets

Use assertions to confirm expected post-reset states.

---

## State Machine Verification

If the DUT contains an FSM:

### Analyze

* Identify all states.
* Determine expected transitions.
* Determine entry and exit conditions.

### Verify

* Visit every reachable state.
* Verify every expected transition.
* Check for unreachable states.
* Check for dead-end states.
* Check for stuck-state conditions.

### Coverage

Generate a state coverage summary showing:

| State   | Visited |
| ------- | ------- |
| STATE_A | Yes     |
| STATE_B | Yes     |

Report any states that were not reached.

---

## Handshake Verification

Check all handshake protocols including:

* Request/Acknowledge
* Ready/Valid
* Start/Done
* Enable strobes

Verify:

* Requests are acknowledged.
* Acknowledgments are not missed.
* Signals deassert correctly.
* Multiple transactions work correctly.
* Back-to-back transactions are handled safely.

Use assertions whenever possible.

Example:

```vhdl
assert TxEnAck = '1'
report "Expected acknowledge not received"
severity error;
```

---

## FIFO Verification

If FIFOs are present:

### Test

* Empty condition
* Full condition
* Single-entry operation
* Burst operation
* Continuous operation
* Underflow attempts
* Overflow attempts

### Verify

* Correct ordering
* No data corruption
* Proper status flags
* Proper handling of boundary conditions

Generate checks for every data word written and read.

---

## Clock Domain Crossing (CDC) Verification

Identify:

* Signals crossing clock domains
* Synchronizer outputs
* FIFO status signals
* Gray-code counters
* Cross-domain handshakes

Review for:

* Metastability risks
* Unsynchronized signals
* Incorrect FIFO flag assumptions

Create stimulus that stresses CDC boundaries and timing assumptions.

---

## Self-Checking Requirements

The testbench must automatically determine pass/fail status.

Use assertions to verify:

* Outputs
* Timing requirements
* State transitions
* Counters
* FIFO contents
* Handshake behavior

Do not rely solely on manual waveform inspection.

---

## Corner Case Testing

Generate tests for:

* Simultaneous events
* Back-to-back transactions
* Maximum counter values
* Timeout conditions
* Missing acknowledgments
* Unexpected resets
* Empty/full FIFO transitions
* Boundary timing conditions

Explain why each corner case is important.

---

## DAQ Firmware-Specific Verification

Look specifically for:

* Lost events
* Duplicate events
* Stalled readout
* Deadlocks
* Data ordering errors
* Trigger timing issues
* Readout latency problems
* Synchronization failures

Discuss any DAQ-specific risks discovered during review.

---

## Mu2e CRV Firmware Focus

Pay particular attention to:

* Dual-clock FIFOs
* Gray-code synchronized status flags
* TxEnReq / TxEnAck handshakes
* FIFO empty/full latency
* One-clock-cycle timing assumptions
* Controller_FPGA2-style readout state machines
* Data aggregation logic
* Readout controller interactions
* Conditions that could cause readout stalls or deadlocks

Flag any assumptions that depend on exact clock-cycle timing.

---

## Deliverables

Provide:

1. Brief explanation of DUT behavior.
2. Verification strategy.
3. Complete VHDL testbench.
4. List of test scenarios covered.
5. State coverage summary.
6. Identified risks or weaknesses.
7. Additional recommended tests for hardware validation.

The resulting testbench should be suitable for professional FPGA firmware development and review in a particle-physics DAQ environment.
