import hyrodyn_py as hyrodyn
import time
import numpy as np
import os

# from urdfpy import URDF
import robomeshcat as rmc
from scipy import integrate
import matplotlib.pyplot as plt

ydd_values = []


def rhs(t, state):

    dim = robot.independent_dof
    res = np.zeros(2 * dim)
    # print("Inside RHS: ", state)
    robot.y = state[
        :dim
    ].flatten()  # first m elements encodes the generalized position state
    robot.yd = state[
        dim:
    ].flatten()  # next m elements encodes the generalized velocity state

    # === APPLY SAME CONTROL AS MUJOCO ===
    baseline_torque = np.array([-11.98, -11.9768])  # Shape: (2,)
    amplitude = 25.0  # Nm
    frequency = 0.05   # Hz
    oscillation = - amplitude * np.sin(2 * np.pi * frequency * t)
    torque_input = baseline_torque + np.array([oscillation] * robot.active_dof)

    # Apply to actuated joints
    robot.Tau_actuated = torque_input

    # Forward dynamics
    robot.calculate_forward_dynamics()
    # print("Acceleration ind state: " , robot.ydd)
    """	
    # apply some basic control
    controller.y = robot.y.flatten()
    #controller.yd = robot.yd.flatten()
    #controller.ydd = robot.ydd.flatten()
    controller.calculate_inverse_dynamics()
    robot.Tau_actuated = controller.Tau_actuated.flatten()
    """
    # Next state
    res[:dim] = robot.yd
    res[dim:] = robot.ydd
    # print("Acceleration: ", robot.ydd)
    # print("Velocity: ", robot.yd)
    return res


def euler_integrator(t, y, h):

    return rhs(t, y)


def runge_integrator(t, y, h):
    dim = robot.independent_dof
    k1 = rhs(t, y)
    ydd_values.append(k1[dim:])
    k2 = rhs(t + 0.5 * h, y + 0.5 * h * k1)
    k3 = rhs(t + 0.5 * h, y + 0.5 * h * k2)
    k4 = rhs(t + h, y + h * k3)
    return (k1 + 2 * (k2 + k3) + k4) / 6.0


def simulate(t0, y0, tf, h, integrator="runge_kutta"):

    t = 0.0
    y = y0

    t_values = []
    y_values = []
    y_values.append(y)
    t_values.append(t)
    ydd_values.append(rhs(t0, y0)[robot.independent_dof :])
    Tau_values = []

    while t <= tf:
        if integrator == "runge_kutta":
            y = y + h * runge_integrator(t, y, h)
            # y = y + h * runge_integrator (t, y, h)
        elif integrator == "euler":
            y = y + h * euler_integrator(t, y, h)
        t = t + h
        # print("State : ",  y)
        # Store the time series output
        t_values.append(t)
        y_values.append(y)
        # Tau_values.append(robot.Tau_actuated.flatten())

    return t_values, y_values, Tau_values


# Path to URDF and Submechanism.yml files

# RH5 Leg model
# path_to_urdf =  os.environ["AUTOPROJ_CURRENT_ROOT"] + "/control/hyrodyn/data/hybrid/rh5/leg/urdf/leg.urdf"
# path_to_submechamisms = os.environ["AUTOPROJ_CURRENT_ROOT"] + "/control/hyrodyn/data/hybrid/rh5/leg/urdf/submechanisms.yml"

# Double pendulum model
# path_to_urdf =  os.environ["AUTOPROJ_CURRENT_ROOT"] + "/control/hyrodyn/data/serial/double_pendulum/urdf/double_pendulum.urdf"
# path_to_submechamisms = os.environ["AUTOPROJ_CURRENT_ROOT"] + "/control/hyrodyn/data/serial/double_pendulum/urdf/submechanisms.yml"

# Simple pendulum model
# path_to_urdf =  os.environ["AUTOPROJ_CURRENT_ROOT"] + "/control/hyrodyn/data/serial/pendulum/urdf/pendulum.urdf"
# path_to_submechamisms = os.environ["AUTOPROJ_CURRENT_ROOT"] + "/control/hyrodyn/data/serial/pendulum/urdf/submechanisms.yml"

# Path to URDF and Submechanism.yml files
# path_to_urdf =  "/home/dfki.uni-bremen.de/rkumar/hyrodyn_dev/hyrodyn/data/hybrid/rh5/leg/urdf/leg.urdf"
# path_to_submechamisms = "/home/dfki.uni-bremen.de/rkumar/hyrodyn_dev/hyrodyn/data/hybrid/rh5/leg/urdf/submechanisms.yml"

# path_to_urdf =  "data/mj_hyrodyn/submechanisms/singular_parallelogram_3.urdf"
# path_to_submechamisms =  "data/mj_hyrodyn/submechanisms/submechanisms_3.yml"

# path_to_urdf = "data/parallel/DOUBLE_PARALLELOGRAM/urdf/right_shoulder.urdf"
# path_to_submechamisms = "data/parallel/DOUBLE_PARALLELOGRAM/urdf/submechanisms.yml"


# path_to_urdf = "data/mj_hyrodyn/submechanisms/LLAnkle.urdf"
# path_to_submechamisms = "data/mj_hyrodyn/submechanisms/LLAnkle.yml"


path_to_urdf = "data/mj_hyrodyn/submechanisms/Body.urdf"
path_to_submechamisms = "data/mj_hyrodyn/submechanisms/Body.yml"


# Load the robot model in HyRoDyn
robot = hyrodyn.RobotModel(path_to_urdf, path_to_submechamisms)
controller = hyrodyn.RobotModel(path_to_urdf, path_to_submechamisms)

# Information of different joints in the robot
active_joints = robot.jointnames_active
independent_joints = robot.jointnames_independent
spanning_tree_joints = robot.jointnames_spanningtree
print("Names of active joints: ", active_joints)
print("Names of independent joints: ", independent_joints)
print("Names of spanning tree joints: ", spanning_tree_joints)

# DOF in different spaces
print("DOF of active joints: ", robot.active_dof)
print("DOF of independent joints: ", robot.independent_dof)
print("DOF of spanning tree joints: ", robot.spanningtree_dof)

# Compute the COM properties of the robot in zero configuration
# robot.y = np.array([3.1416])
robot.calculate_com_properties()
print("Total moving mass in zero config = ", robot.mass)
print("COM in upright config = ", robot.com)

# initial time
t0 = 0.0
# initial state
print("independent_dof", robot.independent_dof)
# y0p = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # Initial joint positions
y0 = np.hstack([np.zeros(robot.independent_dof), np.zeros(robot.independent_dof)])
# y0 = np.zeros(2 * robot.independent_dof)  # Initial joint positions and velocities

print("state_initial", y0)
# total simulation time
tf = 20.0
# Fixed step size
h = 0.001

# Simulate the robot starting at time = t0 and with initial condition y0 to time = tf with step size h
t_values, y_values, Tau_values = simulate(t0, y0, tf, h, integrator="runge_kutta")
# print(y_values)

# Store joint positions for animation
# counter = 5
# for i in range(counter):
#     robot.y = y_values[i][: robot.independent_dof]
#     print("Independent y : ", robot.y)
#     robot.calculate_system_state()
#     print("Full Q: ", robot.Q)

joint_trajectory = []
for i in range(len(y_values)):
    # Update independent joints
    robot.y = y_values[i][: robot.independent_dof]
    robot.yd = y_values[i][robot.independent_dof :]
    robot.ydd = ydd_values[i]
    robot.calculate_system_state()
    # Compute full system state
    # Store joint positions for animation
    joint_trajectory.append(np.array(robot.Q).flatten())
# Initialize robomeshcat
rob = rmc.Robot(urdf_path=path_to_urdf)
scene = rmc.Scene()
scene.add_robot(rob)
import time

# --- Constants ---
FPS = 60  # Target frame rate
FRAME_TIME = 1.0 / FPS  # Time per frame in seconds

# # --- Animation Loop ---
# start_time = time.time()  # Real-world start time
# traj_index = 0  # Index to track position in joint_trajectory

# while traj_index < len(joint_trajectory):
#     # Get current real-world elapsed time
#     current_real_time = time.time() - start_time

#     # Find the correct q for the current time
#     while traj_index < len(t_values) and t_values[traj_index] <= current_real_time:
#         traj_index += 1

#     # If we haven't reached the end, update the robot pose
#     if traj_index > 0:
#         q = joint_trajectory[traj_index - 1]
#         for j, joint_name in enumerate(robot.jointnames_spanningtree):
#             rob[joint_name] = q[j]

#     # Render at 60 FPS (regardless of trajectory update rate)
#     scene.render()

#     # Sleep to maintain 60 FPS
#     next_frame_time = start_time + (time.time() - start_time + FRAME_TIME)
#     while time.time() < next_frame_time:
#         time.sleep(0.001)  # Prevents busy-waiting

# --- Animation Loop ---
# for q in joint_trajectory:
# 	# Update all joints in robomeshcat
# 	for j, joint_name in enumerate(robot.jointnames_spanningtree):
# 		rob[joint_name] = q[j]  # Directly access the j-th joint value
# 	# Render the scene
# 	scene.render()
# 	time.sleep(1/60)  # Adjust for playback speed


# EXPORT DATA TO CSV FILE

import pandas as pd

q_values = []
qd_values = []
qdd_values = []

# Prepare column names
joint_names = robot.jointnames_spanningtree  # or whichever joints you want to track
columns = []
for name in joint_names:
    columns.extend([f"{name}_q", f"{name}_qd", f"{name}_qdd"])
    # columns.extend([f"{name}_q", f"{name}_qd", f"{name}_qdd", f"{name}_Tau"])

# Prepare data
data = []
for i in range(len(y_values)):
    # Update independent joints
    robot.y = y_values[i][: robot.independent_dof]
    robot.yd = y_values[i][robot.independent_dof :]
    robot.ydd = ydd_values[i]
    # Compute full system state
    robot.calculate_system_state()
    # Get all quantities
    row = []
    # print(q_values[0][1])
    for j in range(len(joint_names)):
        row.extend(
            [
                # q_values[i][j],
                # qd_values[i][j],
                # qdd_values[i][j]
                robot.Q[j],  # Position
                robot.QDot[j],  # Velocity
                robot.QDDot[j],  # Acceleration
                # robot.Tau_spanningtree[j]      # Torque
            ]
        )
    # print(row)
    data.append(row)

# Create DataFrame
df = pd.DataFrame(data, columns=columns)
print(len(t_values))
print(df.head())
# Add time as first column
df.insert(0, "time", t_values)

# Save to CSV
csv_filename = "data/mj_hyrodyn/submechanisms/body_ctrl/hyrodyn_simulation_analytical_data_actL_load_1kg.csv"
df.to_csv(csv_filename, index=False)  # index=False avoids adding an extra index column

print(f"Data saved to {csv_filename}")

# with scene.video_recording(
#     filename="data/mj_hyrodyn/submechanisms/csv_data/numerical_hyrodyn_no_load_video.mp4", fps=FPS
# ):
#     for counter in range(len(y_values)):
#         ind = min(counter, len(y_values) - 1)  # Ensure valid index range

#         # Update robot joint positions based on CSV data
#         for j, joint_name in enumerate(robot.jointnames_spanningtree):
#             rob[joint_name] = df[f"{joint_name}_q"][ind]

#         print(f"Rendering frame {counter}/{len(y_values)}, Index: {ind}")
#         scene.render()
