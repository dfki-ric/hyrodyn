# HyRoDyn

Hybrid Robot Dynamics (HyRoDyn) is a kinematics and dynamics solver for series-parallel hybrid robots. 
It exploits the modularity in robot design and can help you solve kinematics and dynamics of such systems analytically. 

![HyRoDyn](doc/HyRoDyn.png)

============================================================

## Requirements

### C++ Build

- CMake >= 3.10
- C++17 compatible compiler
- Eigen3
- yaml-cpp

### Python Bindings (Optional)

- Python >= 3.8
- pybind11
- Python development headers

Building and Installation
=========================

## Linux: HyRoDyn

1. Update the system
```
sudo apt update
sudo apt upgrade
```
2. Install Git
```
sudo apt install git-core
```
3. Install CMake (>= 3.10)
```
sudo apt install cmake
```
Optional (recommended for advanced configuration):
```
sudo apt install cmake-curses-gui
```
4. Install Eigen3
```
sudo apt install libeigen3-dev
```
5. Install yaml-cpp
```
sudo apt install libyaml-cpp-dev
```
6. Install a C++ Compiler (C++17 required)
```
sudo apt install build-essential
```
Optional: install clang
```
sudo apt install clang
```
7. Install Python Dependencies (Optional – for Python bindings)
```
sudo apt install python3-dev python3-pip
pip3 install pybind11
```

Build
=====

Clone the repository:
```
git clone --recursive git@git.hb.dfki.de:hyrodyn/hyrodyn.git
cd hyrodyn
```
If you already cloned without --recursive, run:
```
git submodule update --init --recursive
```
Create a build directory and compile:
```
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=Release ..
make -j
```
To build with Python bindings:
```
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON_BINDINGS=ON ..
make -j
```
You can then import it in Python:
```
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

### Dependency Structure:

hyrodyn ->urdfreader->rbdl



## License
The code is licensed under the MIT license. See the [LICENSE](LICENSE) file for more details.

## Contact Information:
For further questions or collaboration inquiries, please contact the developers at:
- [Rohit Kumar](mailto:r.kumar@dfki.de)


