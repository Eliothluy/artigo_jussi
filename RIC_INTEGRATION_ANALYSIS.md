# RIC (RAN Intelligent Controller) Integration Analysis for O-RAN Simulations

## Executive Summary

This document analyzes how to create a functional RIC for slice control in O-RAN simulations based on the ns-3-dev and oran-e2sim codebase.

---

## 1. RIC Architecture in O-RAN

### 1.1 Non-RT RIC vs Near-RT RIC

**Near-RT RIC (Near-Real-Time RIC):**
- **Purpose**: Real-time control of RAN functions (time scale: 10-500 ms)
- **Interface**: E2 interface to E2 nodes (gNB, eNB)
- **Functionality**: 
  - Real-time optimization
  - Closed-loop control
  - Slice management
  - Handover coordination

**Non-RT RIC (Non-Real-Time RIC):**
- **Purpose**: Non-real-time management (time scale: > 500 ms)
- **Interface**: A1 interface to Near-RT RIC
- **Functionality**:
  - Long-term optimization
  - Policy enforcement
  - ML model training
  - Service orchestration

### 1.2 xApp Framework

An xApp (Radio Access Network Application) is a software component that runs on Near-RT RIC to provide intelligent RAN functions.

**Key Characteristics:**
- Containerized applications
- Subscribe to RAN data via E2 interface
- Send control messages back to RAN nodes
- Can implement ML/DRL algorithms
- Isolated from RIC platform

**xApp Lifecycle:**
1. **Discovery**: xApp discovers available RAN functions via RIC
2. **Subscription**: xApp subscribes to specific KPIs/measurements
3. **Indication**: RAN sends periodic indication messages with data
4. **Decision**: xApp processes data and makes control decisions
5. **Control**: xApp sends control messages to RAN nodes
6. **Feedback**: RAN executes control and sends outcome/acknowledgment

### 1.3 RIC Service Models (E2SM)

E2SM (E2 Service Model) defines how data is exchanged between RIC and RAN nodes.

**E2SM-KPM (Key Performance Measurement):**
- **Purpose**: Monitoring RAN KPIs
- **Data Types**:
  - Cell metrics: PRB usage, throughput, latency
  - UE metrics: per-flow statistics, QoS metrics
  - Container types:
    - O-CU-CP: Active UEs, RRC states
    - O-CU-UP: PDCP byte counters
    - O-DU: Resource usage per slice/QCI
- **Action Type**: REPORT (0)
- **Key Files**: `kpm-function-description.h`, `kpm-indication.h`

**E2SM-RC (Radio Control):**
- **Purpose**: Control RAN parameters
- **Control Actions**:
  - Slice control: PRB allocation, slice weights
  - Handover control: Target cell selection
  - QoS control: 5QI mapping, scheduler parameters
- **Action Types**: INSERT (1), POLICY (2)
- **Key Files**: `ric-control-function-description.h`, `ric-control-message.h`

### 1.4 A1 vs E2 Interfaces

**A1 Interface (Non-RT to Near-RT RIC):**
- Protocol: REST/HTTP or MQTT
- Purpose: Policy delivery, service requirements, SLA management
- Timing: Non-real-time (>500ms)

**E2 Interface (RIC to E2 Nodes):**
- Protocol: SCTP with ASN.1 encoded E2AP
- Purpose: Control and monitoring of RAN functions
- Timing: Near-real-time (10-500ms)
- **Key Procedures**:
  - E2 Setup: Initialize connection, advertise RAN functions
  - RIC Subscription: Subscribe to measurements
  - RIC Indication: Periodic data reports
  - RIC Control: Send control commands
  - RIC Service Update: Add/remove RAN functions

---

## 2. Existing xApp Implementations

### 2.1 Framework in ns-O-RAN

**Key Components:**

**E2Termination Class** (`oran-interface.h`):
```cpp
class E2Termination : public Object {
  // Initialize E2 connection to RIC
  E2Termination(ricAddress, ricPort, clientPort, gnbId, plmnId);
  
  // Register KPM callback for monitoring
  void RegisterKpmCallbackToE2Sm(ranFunctionId, description, callback);
  
  // Register Service Model callback for control
  void RegisterSmCallbackToE2Sm(ranFunctionId, description, callback);
  
  // Process subscription requests
  RicSubscriptionRequest_rval_s ProcessRicSubscriptionRequest(pdu);
  
  // Send messages to RIC
  void SendE2Message(pdu);
};
```

**RIC Control Message** (`ric-control-message.h`):
```cpp
class RicControlMessage {
  enum ControlMessageRequestIdType { TS = 1001, QoS = 1002 };
  
  // Extract parameters from control message
  std::vector<RANParameterItem> ExtractRANParametersFromControlMessage(
    E2SM_RC_ControlMessage_Format1_t *format);
  
  // Get secondary cell ID for handover
  std::string GetSecondaryCellIdHO();
};
```

### 2.2 RIC Action Types

From `RICactionType.h`:
- **report (0)**: Periodic data reporting (KPM)
- **insert (1)**: Insert control actions (RC)
- **policy (2)**: Policy-based control

### 2.3 Sample E2 Messages

**E2 Setup Request**: Establishes connection and advertises available RAN functions
**RIC Subscription Request**: xApp subscribes to specific measurements
**RIC Indication**: Periodic data reports from RAN to xApp
**RIC Control Request**: xApp sends control commands to RAN
**RIC Service Update**: Add, modify, or remove RAN functions

---

## 3. O-RAN Documentation Patterns

### 3.1 Slice Management xApp

**Key Requirements:**

1. **Subscribe to KPM Data:**
   - Slice-specific PRB usage (DL/UL)
   - Throughput per slice (bytes, packets)
   - Latency per slice
   - Active flows per slice

2. **Send Control Commands:**
   - Adjust static slice weights
   - Modify dynamic resource shares
   - Update slice configuration (QoS, priority)
   - PRB allocation per slice

3. **Closed-Loop Control:**
   ```
   Measurement → Analysis → Decision → Control → Feedback
   ```

**Integration Points from Slicing Examples:**

From `oran_slicing_bwp.cc` and `oran_slicing_prb.cc`:

```cpp
struct ControlParams {
  double embbResourceShare = 0.5;
  double mmtcResourceShare = 0.5;
  std::string schedulerMode = "qos";
};

// DRL/xApp integration points:
// - SetSliceStaticWeight(sliceId, weight)
// - SetSliceDynamicShare(sliceId, share)
// - GetSliceMetrics(sliceId)
```

**Slice Metrics Structure:**
```cpp
struct SliceMetrics {
  uint8_t sliceId;
  uint8_t bwpId;
  std::string sliceName;
  uint64_t txBytes;
  uint64_t rxBytes;
  uint32_t txPackets;
  uint32_t rxPackets;
  double delaySum;
  uint32_t activeFlows;
};
```

### 3.2 Best Practices for Closed-Loop Control

**1. Observation Period:**
- Choose appropriate reporting interval (e.g., 100-500 ms)
- Balance measurement accuracy vs. control delay

**2. Action Delay:**
- Account for propagation, processing, and execution delays
- Use predictive control (DRL, MPC)

**3. Safety Mechanisms:**
- Limit control action magnitude (avoid oscillation)
- Fallback to static allocation if control fails
- Validate control outcomes before next action

**4. Multi-Objective Optimization:**
- Trade off between competing objectives:
  - Throughput vs. latency
  - Fairness vs. efficiency
  - Slice isolation vs. resource utilization

**5. Robustness:**
- Handle measurement errors
- Graceful degradation on failures
- Adaptive thresholds based on load

### 3.3 Handling Multiple gNBs

**Approaches:**

1. **Centralized Control:**
   - Single xApp controls all gNBs
   - Global optimization possible
   - Higher computational load

2. **Distributed Control:**
   - One xApp per gNB or cluster
   - Faster decisions
   - May need coordination

3. **Hierarchical Control:**
   - High-level coordinator + local controllers
   - Best of both worlds
   - More complex implementation

---

## 4. Creating a Functional RIC for Slice Control

### 4.1 Required Components

**Core RIC Components:**
1. **E2Termination** (`oran-interface.h/cc`): E2 node interface
2. **KpmIndication** (`kpm-indication.h/cc`): KPM data structures
3. **KpmFunctionDescription** (`kpm-function-description.h/cc`): KPM service model
4. **RicControlMessage** (`ric-control-message.h/cc`): Control message handling
5. **RicControlFunctionDescription** (`ric-control-function-description.h/cc`): RC service model
6. **ASN.1 Types** (`asn1c-types.h/cc`): ASN.1 encoding/decoding

### 4.2 Key Parameters for Slice Control

**Control Parameters:**
1. **Static Resource Share**: Guaranteed resources per slice
2. **Dynamic Resource Share**: Shared pool for excess demand
3. **QoS Mapping**: 5QI/QCI to slice mapping
4. **BWP Allocation**: Physical isolation (BWP 0 for eMBB, BWP 1 for mMTC)
5. **Priority**: Slice priority for resource contention

**Measurement Parameters:**
1. **PRB Usage**: DL/UL PRBs per slice
2. **Throughput**: Bytes/packets per slice
3. **Latency**: Packet delay per slice
4. **Active Flows**: Number of active flows per slice
5. **Packet Loss**: Loss ratio per slice

**SLA Parameters:**
1. **Minimum Throughput**: Required throughput per UE/slice
2. **Maximum Latency**: Maximum acceptable delay
3. **Reliability**: Minimum PDR (Packet Delivery Ratio)
4. **Priority**: Slice priority class

---

## 5. Summary and Recommendations

### 5.1 Architecture Choice

**Recommended**: Near-RT RIC with xApp for slice control

**Reasoning:**
- Real-time response needed for slice control (10-500 ms)
- Closed-loop control requires fast feedback
- Direct E2 interface to RAN nodes
- Can implement ML/DRL algorithms

### 5.2 Implementation Approach

**Phase 1: Basic Slice Control**
1. Implement KPM subscription for slice metrics
2. Implement rule-based control (PID-like)
3. Test with existing slicing scenarios

**Phase 2: DRL Integration**
1. Integrate Gymnasium environment
2. Train DRL agent (DQN, PPO, SAC)
3. Replace rule-based control with DRL policy
4. Compare performance against baseline

**Phase 3: Multi-Cell Control**
1. Extend to multiple gNBs
2. Implement centralized vs. distributed control
3. Add coordination mechanism

### 5.3 Key Challenges

1. **Timing Accuracy**: Ensure simulation reflects real RIC timing
2. **Message Overhead**: Balance reporting interval vs. control delay
3. **Control Stability**: Avoid oscillation in slice weights
4. **SLA Compliance**: Guarantee slice performance under dynamic conditions
5. **Scalability**: Handle many UEs and multiple gNBs

### 5.4 Best Practices

1. **Start Simple**: Begin with rule-based control, then add ML
2. **Validate Incrementally**: Test each component independently
3. **Log Extensively**: Track messages, metrics, control actions
4. **Use Existing Patterns**: Leverage examples in `oran-interface/examples/`
5. **Document Decisions**: Track parameter choices and their impact

### 5.5 Resources

**Documentation:**
- O-RAN Alliance Specifications
- ns-O-RAN README: `/home/elioth/Documentos/artigo_jussi/ns-3-dev/contrib/oran-interface/README.md`
- e2sim README: `/home/elioth/Documentos/artigo_jussi/oran-e2sim/e2sim/README.md`

**Examples:**
- `oran-interface-example.cc`: Basic E2 setup and KPM subscription
- `e2sim-integration-example.cc`: e2sim integration
- `ric-indication-messages.cc`: Indication message encoding
- `oran_slicing_bwp.cc`: BWP-based slicing (with xApp integration points)
- `oran_slicing_prb.cc`: RBG-based slicing (with xApp integration points)

**External xApps:**
- KPIMON xApp: https://github.com/wineslab/ns-o-ran-scp-ric-app-kpimon
- RC xApp: https://github.com/wineslab/ns-o-ran-xapp-rc
- Gymnasium Environment: https://github.com/wineslab/ns-o-ran-gym-environment

**Papers:**
- RSLAQ Paper: `_RSLAQ-_A_Robust_SLA-driven_6G_O-RAN_QoS_xApp_Using_Deep_Reinforcement_Learning.pdf`
- ns-O-RAN Paper: Reference in `contrib/oran-interface/README.md`

---

## 6. Conclusion

Creating a functional RIC for slice control in O-RAN simulations requires:

1. **E2 Interface Setup**: Initialize `E2Termination` and register E2SMs
2. **KPM Subscription**: Subscribe to slice-specific metrics (PRB usage, throughput)
3. **Control Logic**: Implement decision algorithm (rule-based or DRL)
4. **RIC Control**: Send control messages to adjust slice weights/allocation
5. **Closed-Loop**: Continuously monitor, decide, act, and evaluate

The ns-O-RAN and e2sim codebase provides a solid foundation with all necessary components:
- E2 protocol implementation
- KPM and RC service models
- Message encoding/decoding
- Integration with ns-3 simulation

Next steps should focus on implementing xApp, integrating it with existing slicing scenarios, and validating control performance against SLA requirements.
