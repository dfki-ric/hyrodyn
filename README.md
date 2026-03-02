# HyRoDyn
[![pipeline status](https://git.hb.dfki.de/hyrodyn/hyrodyn/badges/main/pipeline.svg)](https://git.hb.dfki.de/hyrodyn/hyrodyn/-/pipelines)

Hybrid Robot Dynamics (HyRoDyn) is a kinematics and dynamics solver for series-parallel hybrid robots. 
It exploits the modularity in robot design and can help you solve kinematics and dynamics of such systems analytically. 

![HyRoDyn](docs/HyRoDyn.png)

# Requirements

## C++ Build

- CMake ≥ 3.10  
- C++17 compatible compiler  
- Eigen3  
- yaml-cpp  
- Boost (all components)  
- TinyXML  
- GoogleTest (for unit tests)

## Python Bindings (Optional)

- Python ≥ 3.8  
- pybind11  
- Python development headers  


# Building and Installation

## Linux: HyRoDyn

### 1. Update System

```bash
sudo apt update
sudo apt upgrade
```

### 2. Install Required System Dependencies

```bash
sudo apt install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    ca-certificates \
    libeigen3-dev \
    libyaml-cpp-dev \
    libboost-all-dev \
    libtinyxml-dev \
    libgtest-dev
```

Optional (recommended for advanced CMake configuration):

```bash
sudo apt install cmake-curses-gui
```

Optional: install clang

```bash
sudo apt install clang
```

### 3. Install Python Dependencies (Optional – for Python bindings)

```bash
sudo apt install python3-dev python3-pip
pip3 install pybind11
```

# Build HyRoDyn

## Clone the Repository

```bash
git clone --recursive git@git.hb.dfki.de:hyrodyn/hyrodyn.git
cd hyrodyn
```

If already cloned without `--recursive`:

```bash
git submodule update --init --recursive
```

## Compile

```bash
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j
```

## Build with Python Bindings

```bash
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON_BINDINGS=ON ..
make -j
```

You can then use it in Python:

```python
import hyrodyn
```

## Architecture

HyRoDyn follows a layered architecture to ensure clear dependency separation and reproducible builds.

Core Components:

- hyrodyn  
  Main C++ dynamics library implementing hybrid rigid-body dynamics algorithms.

- urdfreader  
  Internal URDF parser used to construct robot models.

- rbdl  
  Integrated as a shallow git submodule and built internally. Provides rigid-body dynamics primitives.

## License
HyRoDyn is distributed under the [3-clause BSD license](https://opensource.org/licenses/BSD-3-Clause). See the [LICENSE](LICENSE) file for more details.

## Contact Information:
For further questions or collaboration inquiries, please contact the developers at:
- [Rohit Kumar](mailto:r.kumar@dfki.de)


