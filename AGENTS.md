# AGENTS.md

This document provides essential information for agents working effectively in this repository.

## Project Overview

This is a research codebase combining network simulation with O-RAN (Open Radio Access Network) and 5G NR (New Radio) technologies. The project focuses on simulating O-RAN-compliant E2 interfaces and 5G network scenarios.

### Structure

- **ns-3-dev/**: ns-3 network simulator (version 3.46) with two contrib modules:
  - `contrib/nr/`: 3GPP NR (5G) module by CTTC (Centre Tecnologic de Telecomunicacions de Catalunya)
  - `contrib/oran-interface/`: O-RAN E2 interface integration module by Northeastern University
- **oran-e2sim/**: Fork of OSC e2sim library for E2 interface simulation within ns-3
- **exemplos.txt**: List of available example scenarios (Portuguese)
- **README.md**: Minimal project documentation

### Purpose

Research repository for simulating Open RAN architectures, traffic steering, and intelligent network management in 5G networks. Used for academic research and publication.

## Build System

### ns-3 Build Commands

All commands should be run from `/home/elioth/Documentos/artigo_jussi/ns-3-dev/`:

```bash
# Configure ns-3 with examples and tests enabled
./ns3 configure --enable-examples --enable-tests

# Build ns-3
./ns3 build

# Run tests
./test.py

# Run a specific example
./ns3 run cttc-nr-demo

# List available targets (examples, modules)
./ns3 show targets | grep -E "(nr|oran)"
```

**Important**: The `ns3` tool is a custom wrapper around CMake that provides a Waf-like API.

### e2sim Build Commands

Run from `/home/elioth/Documentos/artigo_jussi/oran-e2sim/e2sim/`:

```bash
# Build and install e2sim (requires sudo)
./build_e2sim.sh [log_level]

# Or rebuild without reinstalling
./make_e2sim.sh [log_level]

# log_level values:
# 0 = LOG_LEVEL_UNCOND
# 1 = LOG_LEVEL_ERROR
# 2 = LOG_LEVEL_INFO (default)
# 3 = LOG_LEVEL_DEBUG
```

The build script creates a `build/` directory, runs CMake, packages as `.deb` files, and installs them system-wide.

### Clean Build

To delete simulation trace files:
```bash
cd /home/elioth/Documentos/artigo_jussi/ns-3-dev
./delete_traces.sh  # Removes all .txt files except CMakeLists.txt
```

## Prerequisites

### ns-3 Requirements

- g++-11.1+ or clang++-17+
- Python 3.8+
- CMake 3.20+
- Standard ns-3 dependencies (see ns-3 documentation)

### NR Module Additional Requirements

```bash
sudo apt-get install libc6-dev  # For semaphore.h
sudo apt-get install sqlite sqlite3 libsqlite3-dev  # For calibration examples
sudo apt-get install libeigen3-dev  # For MIMO features
```

### ORAN Requirements

- Custom e2sim library (located in oran-e2sim/)
- ns-3-mmWave module (external dependency)
- SCTP libraries (for E2 interface communication)

## Code Organization

### ns-3 Module Structure

Each module follows the standard ns-3 pattern:
```
module-name/
├── CMakeLists.txt
├── doc/          # Documentation
├── examples/     # Example programs
├── helper/       # Helper classes for configuration
├── model/        # Core model implementations
└── test/         # Unit tests
```

### Key Modules

**NR Module** (`contrib/nr/`):
- 3GPP NR/5G simulation
- Channel models (3GPP TR 38.901)
- MIMO beamforming
- Component Carrier (CC) and Bandwidth Part (BWP) configuration
- Examples: `cttc-nr-demo`, `cttc-nr-mimo-demo`, `cttc-nr-fh-xr`

**ORAN Interface Module** (`contrib/oran-interface/`):
- E2 termination and RIC (RAN Intelligent Controller) integration
- KPM (Key Performance Indicator) function definitions
- Indication messages encoding/decoding
- Examples: `oran-interface-example`, `e2sim-integration-example`

### User Code Location

For custom simulations:
- Place C++ scripts in `ns-3-dev/scratch/` (they will be automatically compiled)
- For complex projects, create subdirectories within `scratch/` with their own `CMakeLists.txt`
- Or create new modules in `contrib/` for reusable code

## Code Style and Conventions

### License Headers

All C++ source files must include:
```cpp
// Copyright (c) 2023 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
```

For ORAN interface module (Northeastern University):
```cpp
/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2022 Northeastern University
 * Copyright (c) 2022 Sapienza, University of Rome
 * Copyright (c) 2022 University of Padova
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 * ...
 */
```

### Naming Conventions

- Classes: `CamelCase` (e.g., `NrMacScheduler`)
- Methods/functions: `CamelCase` (e.g., `SendPacket`)
- Member variables: `m_camelCase` with `Ptr<>` pointers (e.g., `m_spectrum`)
- Constants: `UPPER_SNAKE_CASE` or `kCamelCase`
- Filenames: `kebab-case.cc` and `kebab-case.h`

### Namespace

All code uses `ns3` namespace:
```cpp
using namespace ns3;

// Or prefix with ns3::
ns3::Ptr<ns3::Node> node;
```

### Forward Declarations

**Critical**: In `.h` files, include only headers that are absolutely needed. Forward-declare everything else to reduce compile times and avoid circular dependencies:

```cpp
// header file
#include "MyBossClass.h"
#include "Mac.h"

class Spectrum; // Forward-declaration
class Phy; // Forward-declaration

class MyPreciousClass : public MyBossClass
{
public:
  MyPreciousClass();

private:
  Ptr<Spectrum> m_spectrum;  // Can be forward-declared
  Mac m_mac;                 // Cannot be forward-declared (instance)
  std::shared_ptr<Phy> m_phy;// Can be forward-declared
};
```

### Clang-Format

The project uses `.clang-format` configuration:
- Based on Microsoft style
- Column limit: 100 characters
- Line endings: LF
- Include blocks regrouped (ns3/ headers first, then system headers)

Format code with:
```bash
cd /home/elioth/Documentos/artigo_jussi/ns-3-dev
./utils/check-style-clang-format.py --fix <file>
```

## Testing

### Running Tests

```bash
# Run all ns-3 tests
./test.py

# Run specific test suite
./test.py --suite=<test-suite-name>

# Run with verbose output
./test.py -v

# Run NR-specific examples (see contrib/nr/test/examples-to-run.py)
./ns3 run cttc-nr-demo
```

### Example Test Configuration

NR module tests are defined in `contrib/nr/test/examples-to-run.py` with tuples of:
```python
("example-name --param1=value1", "True", "True")
```
Where: example command, do_run (True/False), do_valgrind_run (True/False)

### Test Files

- Unit tests: `contrib/nr/test/*.cc`
- Test framework: `ns-3-dev/test.py` (Python script)
- Example validation: `contrib/nr/test/examples-to-run.py`

## Running Examples

### NR Examples

Located in `ns-3-dev/contrib/nr/examples/`:

```bash
# Basic 5G demo
./ns3 run cttc-nr-demo

# MIMO demo
./ns3 run cttc-nr-mimo-demo

# Component Carrier and BWP demo
./ns3 run cttc-nr-cc-bwp-demo

# Fronthaul with XR traffic
./ns3 run cttc-nr-fh-xr

# 3GPP channel calibration
./ns3 run cttc-3gpp-indoor-calibration
```

### ORAN Examples

Located in `ns-3-dev/contrib/oran-interface/examples/`:

```bash
# Basic ORAN interface example
./ns3 run oran-interface-example

# E2SIM integration example
./ns3 run e2sim-integration-example

# Indication messages example
./ns3 run ric-indication-messages
```

### Listing Available Examples

```bash
# List all NR examples
./ns3 show targets | grep -E "^cttc"

# List ORAN examples
./ns3 show targets | grep -E "^oran|^ric|^test-wrappers"
```

## Important Gotchas

### Module Integration

1. **NR and ORAN modules are separate git repositories** in `contrib/`. They will show as "Untracked files" in `git status` - this is normal.

2. **Version compatibility is critical**: The NR module README shows specific ns-3 versions for each NR release. Currently using ns-3.46 with NR v4.1.1.

3. **Build order matters**:
   - First build and install e2sim from `oran-e2sim/e2sim/`
   - Then configure and build ns-3 with ORAN integration enabled
   - The ORAN module looks for the installed e2sim library

### Trace Files

- Simulations generate `.txt` trace files in the working directory
- PCAP files are also generated with names like `example-name-xx-xx.pcap`
- Use `delete_traces.sh` to clean up before new simulation runs
- The `default` file in ns-3-dev root contains flow monitor output (not a configuration file)

### Python vs C++

- **Primary development**: C++ for ns-3 models and simulations
- **Python bindings**: Available via cppyy, but not typically used for this codebase
- Some NR examples include Python scripts for visualization or ML-based scheduling (e.g., `gsoc-nr-rl-based-sched/`)

### Configuration Parameters

NR examples use extensive command-line parameters. View help with:
```bash
./ns3 run "cttc-nr-demo --PrintHelp"
```

Common parameters include:
- `--gNbNum`: Number of gNodeBs
- `--ueNumPergNb`: UEs per gNodeB
- `--operationMode`: FDD or TDD
- `--bandwidth`: Channel bandwidth (Hz)
- `--simTime`: Simulation duration (seconds)

### File Locations

- **User scratch code**: `ns-3-dev/scratch/`
- **Build artifacts**: `ns-3-dev/build/` (auto-generated)
- **Contributed modules**: `ns-3-dev/contrib/`
- **Official modules**: `ns-3-dev/src/`
- **Examples**: `ns-3-dev/examples/` and `ns-3-dev/contrib/*/examples/`

### Dependencies Between Modules

- NR module depends on LTE module (some LTE components are reused)
- ORAN module depends on custom e2sim library
- Both depend on core ns-3 modules (core, network, internet, etc.)

## Common Tasks

### Creating a New Simulation

1. Create `.cc` file in `ns-3-dev/scratch/` or `scratch/subdir/`
2. Add necessary module includes:
   ```cpp
   #include "ns3/core-module.h"
   #include "ns3/network-module.h"
   #include "ns3/nr-module.h"           // For NR features
   #include "ns3/oran-interface.h"      // For ORAN features
   ```
3. Use `NS_LOG_COMPONENT_DEFINE("MySimulation")`
4. Run with `./ns3 run scratch/sim-name` (omit .cc extension)

### Debugging

Enable logging for specific components:
```cpp
ns3::LogComponentEnable("MyComponent", ns3::LOG_LEVEL_ALL);
```

Or from command line:
```bash
./ns3 run "example --ns3::MyComponent::LogComponentEnable=true"
```

### Adding a New Module

1. Create directory in `contrib/my-module/`
2. Follow standard structure: doc/, examples/, helper/, model/, test/
3. Add CMakeLists.txt following existing module patterns
4. Reconfigure and build ns-3

## References

### Documentation

- **ns-3 Tutorial**: https://www.nsnam.org/docs/tutorial/html/index.html
- **ns-3 Manual**: https://www.nsnam.org/docs/manual/html/index.html
- **NR Module Manual**: `contrib/nr/doc/nrmodule.pdf` (or build from doc/)
- **NR Tutorial**: `contrib/nr/doc/cttc-nr-demo-tutorial.pdf`
- **NR Doxygen**: Available in `contrib/nr/doc/doc/html/` (build with `doxygen doxygen.conf`)

### Build Documentation

```bash
cd contrib/nr/doc
make latexpdf  # Build PDF manual
doxygen doxygen.conf  # Build Doxygen
```

### Quick Start Guides

- **ORAN Integration**: https://openrangym.com/tutorials/ns-o-ran
- **ns3-gym for RL**: https://github.com/tkn-tub/ns3-gym

## Version Information

- **ns-3 version**: 3.46 (see `ns-3-dev/VERSION`)
- **NR module version**: 5g-lena-v4.1.1
- **Build system**: CMake 3.20+
- **Python**: 3.8+
- **C++ Standard**: C++17 or later

## License

- **ns-3**: GPL-2.0-only
- **NR module**: GPL-2.0-only
- **ORAN module**: GPL-2.0
- **e2sim**: GPL-2.0 (with modifications)

All modifications must include appropriate SPDX-License-Identifier headers.
