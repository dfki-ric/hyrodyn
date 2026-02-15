# HyRoDyn

Hybrid Robot Dynamics (HyRoDyn) is a kinematics and dynamics solver for series-parallel hybrid robots. 
It exploits the modularity in robot design and can help you solve kinematics and dynamics of such systems analytically. 

![HyRoDyn](doc/HyRoDyn.png)
## Requirements

### C++ Build

- CMake ≥ 3.10
- C++17 compatible compiler
- Eigen3
- yaml-cpp

### Python Bindings (Optional)

- Python ≥ 3.8
- pybind11
- Python development headers

## Build

### C++ Only
```
mkdir build
cd build
cmake ..
make -j
```

### With Python Bindings
```
cmake .. -DBUILD_PYTHON_BINDINGS=ON
make -j
```
You can then import it in Python:
```
import hyrodyn
```

## Installation

To install the library and headers:
```
cmake .. -DCMAKE_INSTALL_PREFIX=<install-path>
make
make install
```
This installs:

- The shared library
- The executable
- Public headers

## Architecture

HyRoDyn follows a layered architecture to ensure clear dependency separation and reproducible builds.

Core Components:

- hyrodyn  
  Main C++ dynamics library implementing hybrid rigid-body dynamics algorithms.

- urdfreader  
  Internal URDF parser used to construct robot models.

- rbdl  
  Integrated as a shallow git submodule and built internally. Provides rigid-body dynamics primitives.

Dependency Structure:

hyrodyn
urdfreader
rbdl

The Python bindings are built as a separate module that links against the hyrodyn C++ library:

hyrodyn_python (pybind11 module)
hyrodyn




