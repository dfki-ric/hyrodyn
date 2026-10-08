# Contributing to HyRoDyn

Thank you for your interest in contributing to Hybrid Robot Dynamics (HyRoDyn)! This document covers everything you need to know — from reporting bugs to submitting code.

---

## Table of Contents

- [Reporting Issues](#reporting-issues)
- [How to Contribute Code](#how-to-contribute-code)
- [Branch Naming](#branch-naming)
- [Coding Style](#coding-style)
- [Testing](#testing)
- [Documentation](#documentation)
- [Licensing](#licensing)

---

## Reporting Issues

Please open an issue on the [HyRoDyn GitHub repository](https://github.com/dfki-ric/hyrodyn/issues) and include:

- A short, descriptive title.
- Steps to reproduce the problem.
- The URDF / submechanisms YAML file (or a minimal reproducer) if applicable.
- Expected behaviour vs. actual behaviour.
- HyRoDyn version or commit hash.

Bug reports that come with a failing test case are especially welcome.

---

## How to Contribute Code

1. **Fork / clone** the repository:
   ```bash
   git clone --recursive git@github.com:dfki-ric/hyrodyn.git
   ```

2. **Create a feature branch** from `main` (see [Branch Naming](#branch-naming)).

3. **Implement the feature or fix**, following the [Coding Style](#coding-style) below.

4. **Write or update unit tests** covering the new functionality.  
   Bugfixes must include a test that reproduces the bug.

5. **Run the full test suite** and make sure everything passes (see [Testing](#testing)).

6. **Open a Pull Request** (PR) against `main`:
   - Describe *what* the PR changes and *why*.
   - Reference any related issues (e.g. `Closes #42`).
   - Keep the PR focused — one logical change per PR.

By submitting a Pull Request you confirm that you have the right to contribute the code and that you accept it being published under the [BSD 3-Clause License](LICENSE).

---

## Branch Naming

| Purpose | Convention | Example |
|---------|-----------|---------|
| New feature | `feature/<short-description>` | `feature/new-submechanism` |
| Bug fix | `fix/<short-description>` | `fix/fk-quaternion-sign` |
| Documentation | `docs/<short-description>` | `docs/tutorial-python` |
| Refactoring | `refactor/<short-description>` | `refactor/elcs-cleanup` |

---

## Coding Style

This section gives an overview of the coding conventions used in HyRoDyn.

Just like its parent library (RBDL), the algorithmic parts of HyRoDyn try to follow mathematical or algorithmic notation instead of wrapping algorithms in elaborate programming patterns.

### Aims and Non-Aims

HyRoDyn aims to be:

* **Lean** — think before adding an unnecessary dependency.
* **Easily integrated** — no framework dependence on RoCK or ROS.
* **Suitable as a foundation** for sophisticated control architectures.
* **Transparent** — only a thin abstraction layer over the actual computation.

HyRoDyn is **not**:

* A fully fledged simulator with collision detection or fancy graphics.
* A safety net — it does not keep you from screwing up things.

Multibody dynamics is a complicated subject and in this codebase the preference is mathematical and algorithmic clarity over elegant software architecture.

### Data Storage

HyRoDyn avoids dynamic allocations and prefers contiguous memory (`std::vector`) over fragmented structures (`std::list`, heap-allocated trees).

Use the **Structure-of-Arrays (SOA)** pattern where possible — e.g. the velocities `v` of all bodies are stored as a `std::vector<SpatialVector>` in the `Model` struct.

### Naming Conventions

1. Structs and classes: `CamelCase` — e.g. `ConstraintSet`
2. Struct/class members: `lowerCamelCase` — e.g. `Model::dofCount`
   - Exception: mathematical symbols from algorithm references (e.g. `S` for joint motion subspace, with subscripts via `_`).
3. Only the first letter of an acronym is capitalised — e.g. DOF → `jointDofCount`.
4. Local variables: `snake_case`.

**Examples:**

```cpp
struct Model {
  std::vector<SpatialVector> v;          // ok — v is a symbol
  std::vector<SpatialVector> S;          // ok — S is used in the reference algorithm
  std::vector<double> u;                 // ok
  std::vector<Vector3d> multdof3_u;      // ok — 3-dof specialisation of u

  std::vector<unsigned int> mJointIndex; // NOT OK: invalid prefix
  unsigned int DOFCount;                 // NOT OK: only first letter of abbreviation should be upper case
  double error_tol;                      // NOT OK: use lowerCamelCase for members
  void CalcPositions();                  // NOT OK: member functions must start with a lower-case letter
};
```

### Error Handling

HyRoDyn fails loudly and aborts on error — this helps you spot mistakes early.  
Code must compile **without warnings** with all compiler warnings enabled.

### Const Correctness

Parameters that are not expected to change must be `const`. Use const references whenever possible.

### Eigen

Use dynamic `VectorXd`/`MatrixXd` only when absolutely necessary. Always initialise them with zeros and a size before use to avoid garbage computations.

### Comments

The doxygen comments belong in the **header files**, not in `.cpp` files.  
Within the code itself, comments should clarify non-obvious ideas or sections — write readable code first.

---

## Testing

All code contributions must provide unit tests. HyRoDyn uses [GoogleTest](https://github.com/google/googletest).

Prefer many small tests that check single features over large tests that check multiple things simultaneously.

### Running the Tests

```bash
cd build
ctest --output-on-failure
```

Or run the test binary directly:

```bash
./hyrodyn_tests
```

The tests expect to be run from the **repository root** (the `CMakeLists.txt` in `test/` sets `WORKING_DIRECTORY` to `${CMAKE_SOURCE_DIR}`).

### Adding a New Test

1. Open `test/test_hyrodyn.cpp`.
2. Add a `TEST(Hyrodyn, YourTestName)` block.
3. Use `ASSERT_*` / `EXPECT_*` macros from GoogleTest.
4. Re-run `make -j && ctest` to verify.

---

## Documentation

API documentation is generated with Doxygen from the header files in `src/`.

```bash
# From the repository root
doxygen Doxyfile
```

HTML output is written to `docs/html/`. Open `docs/html/index.html` in your browser.

When adding new public functions, add a Doxygen comment in the corresponding `.hpp` file following the existing style (see `src/HyRoDyn.hpp`).

---

## Licensing

HyRoDyn is distributed under the [BSD 3-Clause License](LICENSE). There is no formal Contributor License Agreement. By submitting patches or opening a Pull Request you confirm that you have the rights to contribute the corresponding code and that you agree it will be published under this license as part of HyRoDyn.
