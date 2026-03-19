# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a research codebase for simulating O-RAN (Open Radio Access Network) and 5G NR networks using ns-3. The project focuses on network slicing with RIC (RAN Intelligent Controller) integration for traffic steering and QoS management.

### Structure

- **ns-3-dev/**: ns-3 network simulator v3.46 with two contrib modules:
  - `contrib/nr/`: 3GPP NR (5G) module by CTTC (5g-lena-v4.1.1)
  - `contrib/oran-interface/`: O-RAN E2 interface integration module by Northeastern University
- **oran-e2sim/**: Custom fork of OSC e2sim library for E2 interface simulation within ns-3
- **scratch/**: Custom simulation scenarios:
  - `oran_slicing_bwp.cc`: O-RAN aligned network slicing with BWP-based physical isolation (RSLAQ compliant)
  - `baseline_slicing_two_slices_fixed.cc`: Baseline 5G-LENA simulation without RIC components

## Build System

### ns-3 Build Commands (from `ns-3-dev/` directory)

```bash
# Configure ns-3 with examples and tests enabled
./ns3 configure --enable-examples --enable-tests

# Build ns-3
./ns3 build

# Run tests
./test.py

# Run examples
./ns3 run cttc-nr-demo
./ns3 run scratch/oran_slicing_bwp
./ns3 run scratch/baseline_slicing_two_slices_fixed
```

The `ns3` tool is a custom wrapper around CMake that provides a Waf-like API.

### e2sim Build Commands (from `oran-e2sim/e2sim/` directory)

```bash
# Build and install e2sim (requires sudo)
./build_e2sim.sh [log_level]

# Rebuild without reinstalling
./make_e2sim.sh [log_level]

# log_level: 0=UNCOND, 1=ERROR, 2=INFO (default), 3=DEBUG
```

### Clean Traces

```bash
cd ns-3-dev
./delete_traces.sh  # Removes all .txt trace files
```

## Key Architectural Components

### NR Module (5G-LENA)

The NR module implements 3GPP 5G NR features:

- **Bandwidth Parts (BWP)**: Physical isolation mechanism for slices - each BWP is a frequency segment of a Component Carrier (CC)
- **CcBwpCreator**: Helper class that automatically creates CC and BWP configurations based on parameters
- **QoS Scheduling**: `NrMacSchedulerOfdmaQos` for QCI-based scheduling across slices
- **BWP Management**: `BwpManagerAlgorithmStatic` maps QCIs to specific BWPs for slice separation

Key files:
- `contrib/nr/model/bwp-manager-algorithm.cc/.h`: BWP management algorithms
- `contrib/nr/model/nr-mac-scheduler-ofdma-*.cc/.h`: QoS-aware MAC schedulers
- `contrib/nr/model/bandwidth-part-*.cc/.h`: BWP data structures

### O-RAN Interface Module

Implements E2 interface between RAN and RIC:

- **E2 Termination**: Manages E2AP protocol communication
- **KPM Functions**: Key Performance Metric indicators for RIC monitoring
- **RIC Messages**: Control and indication messages between RIC and RAN
- **ASN.1 Encoding**: E2AP message encoding/decoding

Key files:
- `contrib/oran-interface/model/oran-interface.cc/.h`: Main E2 interface
- `contrib/oran-interface/model/kpm-indication.cc/.h`: KPM indication messages
- `contrib/oran-interface/model/ric-control-message.cc/.h`: RIC control handling

### Network Slicing Architecture

The codebase implements slicing through multiple layers:

1. **Physical Layer**: BWP-based frequency partitioning
2. **MAC Layer**: QCI-based scheduling with dedicated resources per slice
3. **Bearer Layer**: EPS bearers with specific QCI values per slice type
4. **Application Layer**: Distinct traffic patterns (eMBB: high throughput, mMTC: small packets, low rate)

Slice Types:
- **eMBB (Enhanced Mobile Broadband)**: QCI 7, 10 MHz BWP, high throughput traffic
- **mMTC (Massive Machine Type Communications)**: QCI 5, 10 MHz BWP, small packet traffic

## Important Build Gotchas

1. **Git Repository Structure**: NR and ORAN modules in `contrib/` are separate git repositories and appear as "Untracked files" - this is normal.

2. **Build Order**: First build and install e2sim from `oran-e2sim/e2sim/`, then configure and build ns-3.

3. **BWP Configuration**: `CcBwpCreator` automatically adjusts BWP parameters. Actual values may differ from requested - use `--enableLogging` to verify.

4. **Trace Files**: Simulations generate `.txt` trace files and `.pcap` files in the working directory. The `default` file contains flow monitor output, not configuration.

## Code Style

- **Namespace**: All code uses `ns3` namespace
- **License Headers**: Include appropriate SPDX-License-Identifier (GPL-2.0-only for NR, GPL-2.0 for ORAN)
- **Formatting**: Use `.clang-format` configuration (Microsoft style, 100 char limit)
- **Forward Declarations**: In `.h` files, forward-declare classes used as `Ptr<>` to reduce compile times

Example format:
```cpp
/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2023 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */
```

## Common Commands

### Running Specific Examples

```bash
# NR module examples
./ns3 run cttc-nr-demo
./ns3 run cttc-nr-cc-bwp-demo
./ns3 run cttc-nr-mimo-demo

# ORAN examples
./ns3 run oran-interface-example
./ns3 run ric-indication-messages

# Custom scratch simulations
./ns3 run scratch/oran_slicing_bwp
./ns3 run scratch/baseline_slicing_two_slices_fixed
```

### Listing Available Targets

```bash
./ns3 show targets | grep -E "(nr|oran|cttc)"
```

### Running Tests

```bash
# All tests
./test.py

# Specific test suite
./test.py --suite=<test-suite-name>

# With verbose output
./test.py -v
```

## Dependencies

### NR Module Additional Requirements

```bash
sudo apt-get install libc6-dev           # For semaphore.h
sudo apt-get install sqlite sqlite3 libsqlite3-dev  # For calibration
sudo apt-get install libeigen3-dev       # For MIMO features
```

### Module Dependencies

- NR module depends on LTE module (reuses some LTE components)
- ORAN module depends on custom e2sim library
- Both depend on core ns-3 modules: core, network, internet, applications, mobility, antenna

## Version Information

- **ns-3**: 3.46 (see `ns-3-dev/VERSION`)
- **NR module**: 5g-lena-v4.1.1
- **CMake**: 3.20+
- **C++**: C++17 or later
- **Python**: 3.8+ (for optional bindings)

## Documentation

- **NR Module Manual**: `contrib/nr/doc/nrmodule.pdf`
- **NR Tutorial**: `contrib/nr/doc/cttc-nr-demo-tutorial.pdf`
- **ORAN Quick Start**: https://openrangym.com/tutorials/ns-o-ran
- **ns-3 Documentation**: https://www.nsnam.org/documentation/
