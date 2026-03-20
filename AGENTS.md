# AGENTS.md

## Project Overview

Research codebase combining network simulation with O-RAN (Open Radio Access Network) and 5G NR (New Radio) technologies. Contains ns-3.46 simulator with NR and ORAN interface modules.

## Build Commands

All commands run from `/home/elioth/Documentos/artigo_jussi/ns-3-dev/`:

```bash
# Configure and build
./ns3 configure --enable-examples --enable-tests
./ns3 build

# Clean trace files
./delete_traces.sh
```

e2sim (from `/home/elioth/Documentos/artigo_jussi/oran-e2sim/e2sim/`):
```bash
./build_e2sim.sh [log_level]  # 0=UNCOND, 1=ERROR, 2=INFO, 3=DEBUG
```

## Test Commands

```bash
# Run all tests
./test.py

# Run single test suite
./test.py -s <test-suite-name>  # e.g., ./test.py -s lte-phy-error-model

# Run single example (as test)
./test.py -e <example-name>     # e.g., ./test.py -e cttc-nr-demo

# Run with verbose output
./test.py -v

# List all available tests
./test.py -l

# Run specific example directly
./ns3 run cttc-nr-demo
./ns3 run "example-name --param=value"
```

## Lint/Style Commands

```bash
# Format code with clang-format
./utils/check-style-clang-format.py --fix <file>

# Run clang-tidy checks (uses .clang-tidy config)
clang-tidy <file> -- -I/path/to/includes
```

## Code Style Guidelines

### File Headers

All C++ source files must include:
```cpp
// Copyright (c) 2023 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
```

ORAN module uses Northeastern University copyright header.

### Naming Conventions

- **Classes**: `CamelCase` (e.g., `NrMacScheduler`)
- **Methods/Functions**: `CamelCase` (e.g., `SendPacket`)
- **Member variables**: `m_camelCase` (e.g., `m_spectrum`)
- **Constants**: `UPPER_SNAKE_CASE` or `kCamelCase`
- **Filenames**: `kebab-case.cc` and `kebab-case.h`

### Imports and Headers

- Order: ns3 headers → system headers
- Use forward declarations in .h files to reduce compile times:
```cpp
#include "MyBossClass.h"  // Include only what's needed

class Spectrum;            // Forward-declare when possible
```

### Formatting (clang-format)

- Column limit: 100 characters
- Based on Microsoft style
- Line endings: LF
- Pointer alignment: Left (`Ptr<Spectrum>`, not `Spectrum *Ptr`)
- Qualifier alignment: Left (`const Type`, not `Type const`)
- Brace insertion: Always (InsertBraces: true)

### Types and Pointers

- Use `Ptr<T>` smart pointers for ns3 objects (ref-counted)
- Use `std::shared_ptr<T>` or `std::unique_ptr<T>` for non-ns3 objects
- Prefer `auto` when type is obvious (modernize-use-auto check)
- Use `nullptr` instead of `NULL` or `0`

### Namespace

- All code uses `ns3` namespace
- Prefer `using namespace ns3;` in .cc files
- Use explicit `ns3::` prefix in .h files

### Error Handling

- Use NS_ASSERT for invariants that should never fail
- Use NS_LOG_ERROR for runtime errors
- Return error codes or NS_LOG and return early for non-critical issues

### Class Structure

```cpp
NS_LOG_COMPONENT_DEFINE("ClassName");
NS_OBJECT_ENSURE_REGISTERED(ClassName);

class ClassName : public BaseClass
{
public:
  static TypeId GetTypeId();
  ClassName();
  ~ClassName() override;

protected:
  virtual void DoSomething() override;

private:
  Ptr<Spectrum> m_spectrum;
  uint32_t m_count;
};
```

### Modern C++ (clang-tidy checks)

- Use `override` keyword when overriding virtual methods
- Use `= delete` for deleted constructors
- Use `= default` for default constructors/destructors
- Prefer `std::make_shared`/`std::make_unique` over `new`
- Use `const` member functions when possible
- Avoid unnecessary copies (performance checks)

## Module Structure

- User code: `ns-3-dev/scratch/` (auto-compiled)
- Contrib modules: `ns-3-dev/contrib/nr/`, `contrib/oran-interface/`
- Each module: `doc/`, `examples/`, `helper/`, `model/`, `test/`

## Key Examples

- NR: `cttc-nr-demo`, `cttc-nr-mimo-demo`, `cttc-nr-cc-bwp-demo`
- ORAN: `oran-interface-example`, `e2sim-integration-example`
- List all: `./ns3 show targets | grep -E "(cttc|oran|ric)"`

## Prerequisites

- g++-11.1+ or clang++-17+
- CMake 3.20+, Python 3.8+
- NR: `libc6-dev`, `sqlite3`, `libeigen3-dev`
- ORAN: custom e2sim library in `oran-e2sim/`
