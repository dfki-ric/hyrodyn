# HyRoDyn
[![pipeline status](https://git.hb.dfki.de/hyrodyn/hyrodyn/badges/main/pipeline.svg)](https://git.hb.dfki.de/hyrodyn/hyrodyn/-/pipelines)
![C++17](https://img.shields.io/badge/C++-17-blue)
![Python](https://img.shields.io/badge/python-3.8+-green)
![License](https://img.shields.io/badge/license-BSD--3--Clause-orange)

## ✳️ Overview
Hybrid Robot Dynamics (HyRoDyn) is a kinematics and dynamics solver for series-parallel hybrid robots. 
It exploits the modularity in robot design and can help you solve kinematics and dynamics of such systems analytically. 

![HyRoDyn](docs/HyRoDyn.png)

<p align="center">
  <img src="docs/hyrodyn_abstraction.png" width="45.7%">
  <img src="docs/rh5_leg_swinging_animation.gif" width="49%">
</p>

## ✨ Features

- Analytical kinematics and dynamics for hybrid robots
- Support for series–parallel mechanisms
- Modular robot architecture representation
- Efficient rigid-body dynamics computation
- C++ core with optional Python bindings
- Compatible with URDF robot descriptions

## 📦 Requirements

### C++ Build

- CMake ≥ 3.10  
- C++17 compatible compiler  
- Eigen3  
- yaml-cpp  
- Boost (all components)  
- TinyXML  
- GoogleTest (for unit tests)

### Python Bindings (Optional)

- Python ≥ 3.8  
- pybind11  
- Python development headers  


## 🔧 Build Instructions

### Linux: HyRoDyn

#### 1. Update System

```bash
sudo apt update
sudo apt upgrade
```

#### 2. Install Required System Dependencies

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

#### 3. Install Python Dependencies (Optional – for Python bindings)

```bash
sudo apt install python3-dev python3-pip
pip3 install pybind11
```

## Build HyRoDyn

### Clone the Repository

```bash
git clone --recursive git@git.hb.dfki.de:hyrodyn/hyrodyn.git
cd hyrodyn
```

If already cloned without `--recursive`:

```bash
git submodule update --init --recursive
```

### Compile

```bash
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j
```

### Build with Python Bindings

```bash
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON_BINDINGS=ON ..
make -j
```

You can then use it in Python:

```python
import hyrodyn
```

## 📁 Folder Structure

1. **addons/**: Contains an internal URDF parser used to construct modular robot models.

2. **bin/**: Utility scripts used to generate or manage custom submechanism libraries.

3. **external/**: Third-party libraries used by HyRoDyn. Currently includes RBDL, which provides the rigid-body dynamics primitives.

4. **python/**: Optional Python bindings implemented using pybind11.

5. **robot/**: Example robots including:
   - URDF robot descriptions
   - YAML files defining submechanisms

6. **src/**: Main source code implementing HyRoDyn algorithms for solving the kinematics and dynamics of hybrid robots, including handling closed-loop mechanisms analytically or numerically.


7. **docs/**: Documentation generated using Doxygen. The configuration file (`Doxyfile`) is provided to generate the API documentation for the project.

## License
HyRoDyn is distributed under the [3-clause BSD license](https://opensource.org/licenses/BSD-3-Clause). See the [LICENSE](LICENSE) file for more details.

## 📚 Citation

If you use **analytical** approach from HyRoDyn in research, please cite:
```
@inproceedings{2019_Kumar_HyRoDynApproach_IDETC,
author={Kumar, Shivesh
and Mueller, Andreas},
title={An Analytical and Modular Software Workbench for Solving Kinematics and Dynamics of Series Parallel Hybrid Robots},
Booktitle = {ASME 2019 International Design Engineering Technical Conferences and Computers and Information in Engineering Conference},
Series = {43rd Mechanisms and Robotics Conference},
}
```
If you use **numerical** approach from hyrodyn, please cite:
```
@INPROCEEDINGS{hyrodyn_GCP,
  author={Kumar, Rohit and Kumar, Shivesh and Müller, Andreas and Kirchner, Frank},
  booktitle={2022 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)}, 
  title={Modular and Hybrid Numerical-Analytical Approach - A Case Study on Improving Computational Efficiency for Series-Parallel Hybrid Robots}, 
  year={2022},
  doi={10.1109/IROS47612.2022.9981474}}
```

## 🎉  Acknowledgements

This project would not be possible without the contributions of the following repositories:

- [urdfreader](https://github.com/ORB-HD/URDF_Parser): Internal URDF parser used to construct robot models.
- [rbdl](https://github.com/rbdl/rbdl): provides rigid-body dynamics primitives.


## Contact Information:
For further questions or collaboration inquiries, please contact the developers at:
- [Rohit Kumar](mailto:r.kumar@dfki.de)


