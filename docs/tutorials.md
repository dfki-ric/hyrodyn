# HyRoDyn Tutorials

This document provides step-by-step examples of using HyRoDyn from both C++ and Python. All examples assume you have already built HyRoDyn successfully (see the [Build Instructions](../README.md#-build-instructions)).

---

## Table of Contents

- [Key Concepts](#key-concepts)
- [Tutorial 1 — C++: Load a Robot and Compute Forward Kinematics](#tutorial-1--c-load-a-robot-and-compute-forward-kinematics)
- [Tutorial 2 — C++: Inverse Dynamics](#tutorial-2--c-inverse-dynamics)
- [Tutorial 3 — Python: Basic Usage](#tutorial-3--python-basic-usage)
- [Tutorial 4 — Python: Kinematics and Dynamics](#tutorial-4--python-kinematics-and-dynamics)
- [Example Robots](#example-robots)

---

## Key Concepts

HyRoDyn operates with three joint spaces:

| Space | Variable | Description |
|-------|----------|-------------|
| **Independent joint space** | `y`, `yd`, `ydd` | Minimal coordinates that fully describe the robot configuration (input). |
| **Spanning tree joint space** | `Q`, `QDot`, `QDDot` | All joints in the spanning tree, including passive joints (computed internally). |
| **Actuation space** | `u`, `ud`, `udd` | Joints driven by actuators (a subset of the spanning tree). |

Every HyRoDyn computation takes `y` (independent joint positions) as the primary input and returns quantities in one of the three spaces above.

Both a **URDF** file (robot geometry/inertia) and a **submechanisms YAML** file (closed-loop topology) are required to load a robot model.

---

## Tutorial 1 — C++: Load a Robot and Compute Forward Kinematics

```cpp
#include <iostream>
#include "robot_model_hyrodyn.hpp"

int main() {
    hyrodyn::RobotModel_HyRoDyn robot;

    // Load robot model (URDF + submechanisms YAML)
    robot.load_robotmodel(
        "robot/leg/urdf/leg.urdf",
        "robot/leg/urdf/submechanisms.yml"
    );

    // Set independent joint positions (zero configuration)
    robot.y = Eigen::VectorXd::Zero(robot.jointnames_independent.size());

    // Compute the full spanning-tree state from y
    robot.calculate_system_state();
    std::cout << "Spanning-tree positions Q:\n" << robot.Q.transpose() << "\n";

    // Forward kinematics for a specific body
    robot.calculate_forward_kinematics("LLAnklePitch_Link");
    // robot.pose is a 7D vector: [x, y, z, qx, qy, qz, qw]
    std::cout << "Pose [x y z qx qy qz qw]:\n"
              << robot.pose.transpose() << "\n";

    return 0;
}
```

### Compile and run

```bash
# From build/
g++ -std=c++17 my_example.cpp -o my_example \
    -I../src -L. -lhyrodyn $(pkg-config --cflags --libs eigen3)
./my_example
```

Or add the example to `CMakeLists.txt` as a new executable target and use `make`.

---

## Tutorial 2 — C++: Inverse Dynamics

This example shows how to compute the actuator torques required to produce a desired motion trajectory.

```cpp
#include <iostream>
#include "robot_model_hyrodyn.hpp"

int main() {
    hyrodyn::RobotModel_HyRoDyn robot;
    robot.load_robotmodel(
        "robot/leg/urdf/leg.urdf",
        "robot/leg/urdf/submechanisms.yml"
    );

    int n = robot.jointnames_independent.size();

    // Define a state: position, velocity, acceleration
    robot.y   = Eigen::VectorXd::Zero(n);
    robot.yd  = Eigen::VectorXd::Zero(n);
    robot.ydd = Eigen::VectorXd::Zero(n);

    // Move the first independent joint
    robot.y(0)   = 0.2;   // rad
    robot.yd(0)  = 0.1;   // rad/s
    robot.ydd(0) = 0.05;  // rad/s^2

    // Inverse dynamics: returns required actuator torques
    robot.calculate_inverse_dynamics();
    std::cout << "Actuator torques (Tau_actuated):\n"
              << robot.Tau_actuated.transpose() << "\n";

    return 0;
}
```

---

## Tutorial 3 — Python: Basic Usage

Make sure the Python bindings are built and the virtual environment is active (see [python/README.md](../python/README.md)).

```python
import hyrodyn
import numpy as np

# Load the robot model
robot = hyrodyn.RobotModel(
    "../robot/leg/urdf/leg.urdf",
    "../robot/leg/urdf/submechanisms.yml"
)

# Inspect joint spaces
print("Independent joints :", robot.jointnames_independent)
print("Spanning-tree joints:", robot.jointnames_spanningtree)
print("Active (actuated) joints:", robot.jointnames_active)

# DOF in each space
print("Independent DOF:", robot.independent_dof)
print("Spanning-tree DOF:", robot.spanningtree_dof)
print("Active DOF:", robot.active_dof)
```

---

## Tutorial 4 — Python: Kinematics and Dynamics

```python
import hyrodyn
import numpy as np

robot = hyrodyn.RobotModel(
    "../robot/leg/urdf/leg.urdf",
    "../robot/leg/urdf/submechanisms.yml"
)

n = robot.independent_dof

# ── System state ──────────────────────────────────────────────────────────────
robot.y = np.zeros(n)
robot.y[0] = 0.2          # move first independent joint 0.2 rad

robot.calculate_system_state()
print("Spanning-tree positions Q:\n", robot.Q)
print("Actuator positions u:\n", robot.u)

# ── Forward kinematics ────────────────────────────────────────────────────────
robot.calculate_forward_kinematics("LLAnklePitch_Link")
# pose = [x, y, z, qx, qy, qz, qw]
print("End-effector pose:", robot.pose)

# ── Inverse kinematics ────────────────────────────────────────────────────────
# Store target pose and solve IK
robot.pose_input = [robot.pose]
robot.y = np.zeros(n)          # reset configuration for initial guess
robot.calculate_inverse_kinematics(["LLAnklePitch_Link"])
print("IK solution y:", robot.y)

# ── Dynamics ──────────────────────────────────────────────────────────────────
robot.y   = np.zeros(n)
robot.yd  = np.zeros(n)
robot.ydd = np.zeros(n)

# Inverse dynamics (compute required torques for given motion)
robot.calculate_inverse_dynamics()
print("Inverse dynamics torques:", robot.Tau_actuated)

# Forward dynamics (compute resulting acceleration for given torques)
robot.calculate_forward_dynamics()
print("Forward dynamics accelerations ydd:", robot.ydd)

# ── Mass-inertia matrix and nonlinear effects ─────────────────────────────────
robot.calculate_mass_interia_matrix_actuation_space()
print("Mass-inertia matrix Hu:\n", robot.Hu)

robot.calculate_nle_actuation_space()
print("Nonlinear effects Cu:", robot.Cu)

# ── Center of mass ────────────────────────────────────────────────────────────
robot.calculate_com_properties()
print("Total mass:", robot.mass)
print("COM position:", robot.com)
```

---

## Example Robots

The `robot/` directory contains several example robots:

| Directory | Description |
|-----------|-------------|
| `robot/leg/` | Simple hybrid leg mechanism — used in most tests and tutorials. |
| `robot/lower_body/` | Lower body with floating-base support. |
| `robot/rh5v2/` | Full RH5v2 humanoid robot (analytical and numerical submechanisms). |

Each robot directory contains:
- `urdf/<robot>.urdf` — URDF kinematic/inertia description.
- `urdf/submechanisms.yml` — YAML file describing the closed-loop submechanism topology.

### Submechanisms YAML format

A minimal submechanisms file for a robot with one parallelogram chain looks like:

```yaml
submechanisms:
  - type: "1-RRPR"
    jointnames_spanningtree: [Joint1, Joint2, Joint3, Joint4]
    jointnames_independent: [Joint1]
    jointnames_active: [Joint1]
```

See `robot/leg/urdf/submechanisms.yml` for a real-world example.

---

For more details on the full API, see the [API documentation](../README.md#-api-documentation) or browse `src/HyRoDyn.hpp` and `src/robot_model_hyrodyn.hpp`.
